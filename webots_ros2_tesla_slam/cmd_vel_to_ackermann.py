# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Convert Nav2 Twist commands to AckermannDrive commands.

This module provides a ROS 2 node that converts standard Twist velocity commands
(from Nav2 navigation stack) to Ackermann steering model commands suitable for
the Tesla robot with front-wheel steering.

Conversion includes:
    - Linear velocity pass-through
    - Angular velocity to steering angle using bicycle model
    - Steering angle saturation and optional inversion
    - Optional in-place turning support
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToAckermann(Node):
    """Convert Twist commands to Ackermann steering commands.

    Subscribes to Twist messages (linear and angular velocity) and publishes
    equivalent Ackermann drive commands. Uses the bicycle model for kinematic
    transformation with configurable parameters.

    Parameters:
        wheelbase: Distance between front and rear axles (default: 2.875 m)
        max_steering_angle: Maximum steering angle in radians (default: 0.6 rad)
        min_turning_speed: Minimum speed for in-place turns (default: 0.25 m/s)
        allow_in_place_turn: Enable rotation without forward motion
        invert_steering: Negate steering angle output
        debug_output: Enable verbose command logging
        input_topic: Twist subscription topic (default: /cmd_vel)
        output_topic: AckermannDrive publication topic (default: /cmd_ackermann)
    """

    def __init__(self):
        """Initialize the converter node with parameters and subscriptions."""
        super().__init__('cmd_vel_to_ackermann')

        self.declare_parameter('wheelbase', 2.875)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('min_turning_speed', 0.25)
        self.declare_parameter('allow_in_place_turn', False)
        self.declare_parameter('invert_steering', True)
        self.declare_parameter('debug_output', False)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_ackermann')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_turning_speed = self.get_parameter('min_turning_speed').value
        self.allow_in_place_turn = self.get_parameter('allow_in_place_turn').value
        self.invert_steering = self.get_parameter('invert_steering').value
        self.debug_output = self.get_parameter('debug_output').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self._cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            AckermannDrive,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Converter started! Listening: {input_topic} -> Publishing: {output_topic}'
        )

    def _cmd_vel_callback(self, msg):
        """Convert Twist command to Ackermann steering command.

        Args:
            msg: Twist message with linear.x (forward velocity) and
                 angular.z (rotation rate).
        """
        v = msg.linear.x
        w = msg.angular.z

        ack_msg = AckermannDrive()
        ack_msg.speed = v

        if abs(v) < 0.01:
            # Stationary case: handle in-place rotation if enabled
            if self.allow_in_place_turn and abs(w) > 0.01:
                ack_msg.speed = self.min_turning_speed
                ack_msg.steering_angle = math.copysign(self.max_steering_angle, w)
            else:
                ack_msg.speed = 0.0
                ack_msg.steering_angle = 0.0
        else:
            # Moving case: use bicycle model to compute steering angle
            # steering_angle = arctan(wheelbase * angular_velocity / linear_velocity)
            steering_angle = math.atan(self.wheelbase * w / v)
            ack_msg.steering_angle = max(
                -self.max_steering_angle,
                min(steering_angle, self.max_steering_angle)
            )

        if self.invert_steering:
            ack_msg.steering_angle = -ack_msg.steering_angle

        if self.debug_output:
            self.get_logger().info(
                'cmd_vel linear.x=%.3f angular.z=%.3f -> '
                'ackermann speed=%.3f steering=%.3f' %
                (v, w, ack_msg.speed, ack_msg.steering_angle),
                throttle_duration_sec=0.5,
            )

        self.publisher.publish(ack_msg)


def main(args=None):
    """Initialize and run the Cmd2Ackermann converter node."""
    rclpy.init(args=args)
    converter = CmdVelToAckermann()
    rclpy.spin(converter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
