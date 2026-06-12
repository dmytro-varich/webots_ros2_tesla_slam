# SPDX-FileCopyrightText: 1996-2023 Cyberbotics Ltd.
# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""ROS2 Tesla Webots driver with odometry and IMU publishing.

This module is the core driver interface between the Tesla robot in Webots
simulation and ROS 2. It:
    - Reads robot pose and orientation from Webots
    - Publishes odometry messages (pose, twist in odom frame)
    - Publishes IMU data (orientation, angular velocity)
    - Broadcasts TF transforms (odom -> base_link)
    - Subscribes to Ackermann steering commands

The driver initializes odometry relative to the starting position, enabling
ROS 2 navigation stacks to work seamlessly. Timestamps are synchronized with
Webots simulation time.
"""

import math
import os

from builtin_interfaces.msg import Time
import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

MPS_TO_KMH = 3.6
BASE_LINK_X_OFFSET = -2.15


class TeslaDriver:
    """Webots Tesla robot ROS 2 interface driver.

    Integrates Tesla robot from Webots simulator with ROS 2 ecosystem.
    Publishes odometry, IMU, and TF transforms. Receives Ackermann steering
    commands to control the robot. Runs at Webots simulation step rate.
    """

    def init(self, webots_node, properties):
        """Initialize Webots Tesla driver node and ROS interface."""
        self.__robot = webots_node.robot
        self.__robot_node = self.__robot.getSelf()
        self.__initial_position = None
        self.__initial_yaw = None
        self.__use_sim_time = bool(properties.get('use_sim_time', True))
        self.__odom_topic = os.environ.get(
            'WEBOTS_ROS2_TESLA_ODOM_TOPIC',
            'odom'
        )
        self.__last_stamp_nanoseconds = None
        self.__last_odom_pose = None
        self.__last_odom_stamp_nanoseconds = None

        # ROS interface initialization
        rclpy.init(args=None)
        self.__node = rclpy.create_node('tesla_node')
        self.__odom_publisher = self.__node.create_publisher(
            Odometry, self.__odom_topic, 10
        )
        self.__imu_publisher = self.__node.create_publisher(Imu, 'imu', 10)
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        self.__cmd_ackermann_subscription = self.__node.create_subscription(
            AckermannDrive,
            'cmd_ackermann',
            self.__cmd_ackermann_callback,
            1
        )

    def __cmd_ackermann_callback(self, message):
        """Apply Ackermann steering command to robot.

        Args:
            message: AckermannDrive command with speed and steering_angle
        """
        self.__robot.setCruisingSpeed(message.speed * MPS_TO_KMH)
        self.__robot.setSteeringAngle(message.steering_angle)

    def step(self):
        """Execute driver step called each Webots simulation step."""
        self.__publish_odometry()
        rclpy.spin_once(self.__node, timeout_sec=0)

    def __publish_odometry(self):
        """Publish odometry, TF transform, and IMU messages."""
        # Get current pose from Webots
        position = self.__robot_node.getPosition()
        orientation = self.__robot_node.getOrientation()
        yaw = math.atan2(orientation[3], orientation[0])
        # Adjust position to base_link frame
        position = self.__base_link_position(position, yaw)
        stamp = self.__stamp_from_webots_time()
        if not all(
            math.isfinite(value)
            for value in [*position, *orientation, yaw]
        ):
            return
        # Initialize on first measurement
        if self.__initial_position is None:
            self.__initial_position = position
            self.__initial_yaw = yaw

        # Compute odometry relative to start position
        x, y, yaw = self.__relative_pose(position, yaw)
        linear_x, angular_z = self.__odometry_twist(x, y, yaw, stamp)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z
        # Covariance: position (x, y) and rotation (z) with some uncertainty
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        self.__odom_publisher.publish(odom)
        self.__publish_imu(stamp, yaw, angular_z)

        # Publish TF transform odom -> base_link
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = odom.pose.pose.orientation.z
        transform.transform.rotation.w = odom.pose.pose.orientation.w
        self.__tf_broadcaster.sendTransform(transform)

    def __stamp_from_webots_time(self):
        """Convert Webots simulation time to ROS 2 Time message."""
        time_nanoseconds = int(round(self.__robot.getTime() * 1e9))
        # Ensure monotonic increasing timestamps
        if (
            self.__last_stamp_nanoseconds is not None and
            time_nanoseconds <= self.__last_stamp_nanoseconds
        ):
            time_nanoseconds = self.__last_stamp_nanoseconds + 1
        self.__last_stamp_nanoseconds = time_nanoseconds

        stamp = Time()
        stamp.sec = time_nanoseconds // 1_000_000_000
        stamp.nanosec = time_nanoseconds % 1_000_000_000
        return stamp

    def __relative_pose(self, position, yaw):
        """Compute pose relative to initial position.

        Args:
            position: [x, y, z] in world frame
            yaw: rotation angle in world frame

        Returns:
            (x, y, yaw) in odometry frame (relative to initial pose)
        """
        dx = position[0] - self.__initial_position[0]
        dy = position[1] - self.__initial_position[1]
        cos_yaw = math.cos(self.__initial_yaw)
        sin_yaw = math.sin(self.__initial_yaw)
        # Rotate to initial frame
        x = cos_yaw * dx + sin_yaw * dy
        y = -sin_yaw * dx + cos_yaw * dy
        relative_yaw = yaw - self.__initial_yaw
        # Normalize angle to [-pi, pi]
        relative_yaw = math.atan2(
            math.sin(relative_yaw),
            math.cos(relative_yaw)
        )
        return x, y, relative_yaw

    def __odometry_twist(self, x, y, yaw, stamp):
        """Compute linear and angular velocity from pose derivatives.

        Args:
            x, y, yaw: current odometry pose
            stamp: current timestamp

        Returns:
            (linear_x, angular_z) velocity components
        """
        stamp_nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.__last_odom_pose is None:
            self.__last_odom_pose = (x, y, yaw)
            self.__last_odom_stamp_nanoseconds = stamp_nanoseconds
            return 0.0, 0.0

        dt = (stamp_nanoseconds - self.__last_odom_stamp_nanoseconds) / 1e9
        last_x, last_y, last_yaw = self.__last_odom_pose
        self.__last_odom_pose = (x, y, yaw)
        self.__last_odom_stamp_nanoseconds = stamp_nanoseconds
        if dt <= 0.0:
            return 0.0, 0.0

        # Compute pose delta
        dx = x - last_x
        dy = y - last_y
        dyaw = yaw - last_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        # Divide by time to get velocities
        linear_x = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        ) / dt
        angular_z = dyaw / dt
        return linear_x, angular_z

    def __publish_imu(self, stamp, yaw, angular_z):
        """Publish IMU message with orientation and angular velocity.

        Args:
            stamp: message timestamp
            yaw: robot orientation angle
            angular_z: rotation rate around Z axis
        """
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation.z = math.sin(yaw / 2.0)
        imu.orientation.w = math.cos(yaw / 2.0)
        imu.angular_velocity.z = angular_z
        # Covariance: yaw is trusted; roll/pitch and acceleration are not measured.
        imu.orientation_covariance[0] = 999.0
        imu.orientation_covariance[4] = 999.0
        imu.orientation_covariance[8] = 0.05
        imu.angular_velocity_covariance[0] = 999.0
        imu.angular_velocity_covariance[4] = 999.0
        imu.angular_velocity_covariance[8] = 0.05
        imu.linear_acceleration_covariance[0] = -1.0
        self.__imu_publisher.publish(imu)

    def __base_link_position(self, position, yaw):
        """Adjust world position to base_link frame.

        Args:
            position: [x, y, z] world coordinates
            yaw: robot heading angle

        Returns:
            [x, y, z] adjusted for base_link offset
        """
        return [
            position[0] + math.cos(yaw) * BASE_LINK_X_OFFSET,
            position[1] + math.sin(yaw) * BASE_LINK_X_OFFSET,
            position[2],
        ]
