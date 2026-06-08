# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Convert Nav2 Twist commands to AckermannDrive commands."""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        self.declare_parameter('wheelbase', 2.875)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('min_turning_speed', 0.25)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_ackermann')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_turning_speed = self.get_parameter('min_turning_speed').value
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
        v = msg.linear.x
        w = msg.angular.z

        ack_msg = AckermannDrive()
        ack_msg.speed = v

        if abs(v) < 0.01:
            if abs(w) > 0.01:
                ack_msg.speed = self.min_turning_speed
                ack_msg.steering_angle = math.copysign(self.max_steering_angle, w)
            else:
                ack_msg.steering_angle = 0.0
        else:
            steering_angle = math.atan2(self.wheelbase * w, abs(v))
            
            ack_msg.steering_angle = max(-self.max_steering_angle, min(steering_angle, self.max_steering_angle))

        self.publisher.publish(ack_msg)


def main(args=None):
    rclpy.init(args=args)
    converter = CmdVelToAckermann()
    rclpy.spin(converter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
