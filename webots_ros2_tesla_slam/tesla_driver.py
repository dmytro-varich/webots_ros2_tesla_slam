# SPDX-FileCopyrightText: 1996-2023 Cyberbotics Ltd.
# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""ROS2 Tesla driver with new functions to initialize odometry."""

import math

from builtin_interfaces.msg import Time
import rclpy
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

MPS_TO_KMH = 3.6
BASE_LINK_X_OFFSET = -2.15


class TeslaDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_node = self.__robot.getSelf()
        self.__initial_position = None
        self.__initial_yaw = None
        self.__use_sim_time = bool(properties.get('use_sim_time', True))
        self.__last_stamp_nanoseconds = None

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('tesla_node')
        self.__node.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, self.__use_sim_time)]
        )
        self.__node.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)
        self.__odom_publisher = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

    def __cmd_ackermann_callback(self, message):
        self.__robot.setCruisingSpeed(message.speed * MPS_TO_KMH)
        self.__robot.setSteeringAngle(message.steering_angle)

    def step(self):
        self.__publish_odometry()
        rclpy.spin_once(self.__node, timeout_sec=0)

    def __publish_odometry(self):
        position = self.__robot_node.getPosition()
        orientation = self.__robot_node.getOrientation()
        velocity = self.__robot_node.getVelocity()
        yaw = math.atan2(orientation[3], orientation[0])
        position = self.__base_link_position(position, yaw)
        stamp = self.__stamp_from_webots_time()
        if not all(math.isfinite(value) for value in [*position, *orientation, *velocity, yaw]):
            return
        if self.__initial_position is None:
            self.__initial_position = position
            self.__initial_yaw = yaw

        x, y, yaw = self.__relative_pose(position, yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = (
            math.cos(yaw) * velocity[0] + math.sin(yaw) * velocity[1]
        )
        odom.twist.twist.angular.z = velocity[5]
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        self.__odom_publisher.publish(odom)

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
        time_nanoseconds = int(round(self.__robot.getTime() * 1e9))
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
        dx = position[0] - self.__initial_position[0]
        dy = position[1] - self.__initial_position[1]
        cos_yaw = math.cos(self.__initial_yaw)
        sin_yaw = math.sin(self.__initial_yaw)
        x = cos_yaw * dx + sin_yaw * dy
        y = -sin_yaw * dx + cos_yaw * dy
        relative_yaw = yaw - self.__initial_yaw
        relative_yaw = math.atan2(math.sin(relative_yaw), math.cos(relative_yaw))
        return x, y, relative_yaw

    def __base_link_position(self, position, yaw):
        return [
            position[0] + math.cos(yaw) * BASE_LINK_X_OFFSET,
            position[1] + math.sin(yaw) * BASE_LINK_X_OFFSET,
            position[2],
        ]
