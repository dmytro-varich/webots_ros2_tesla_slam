# SPDX-FileCopyrightText: 1996-2023 Cyberbotics Ltd.
# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Vision-based lane follower for the Webots Tesla.

This module implements lane detection and following for the Tesla robot using
computer vision. It processes camera images, detects lane markings via HSV color
thresholding, computes lateral error, and commands steering to keep the robot
centered on the detected lane.

The control loop:
    1. Subscribe to camera image stream
    2. Segment image by HSV color range (green lanes)
    3. Find largest contour (main lane)
    4. Compute center of mass (lane position)
    5. Calculate steering angle proportional to lateral error
    6. Publish Ackermann steering command
"""

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node

CONTROL_COEFFICIENT = 0.0005
TARGET_SPEED_KMH = 20.0
KMH_TO_MPS = 1.0 / 3.6


class LaneFollower(Node):
    """Vision-based lane following driver for Tesla robot.

    Subscribes to camera images and publishes steering commands to follow
    detected lane markings. Uses HSV color segmentation to identify lane,
    computes centroid, and applies proportional steering control.

    The image processing region is cropped to bottom half to focus on lane area.
    Lane color target: green (HSV range [50-120, 110-255, 150-255]).
    """

    def __init__(self):
        """Initialize lane follower node and camera subscription."""
        super().__init__('lane_follower')

        # ROS publishers and subscribers
        self.__ackermann_publisher = self.create_publisher(
            AckermannDrive,
            'cmd_ackermann',
            1
        )

        qos_camera_data = qos_profile_sensor_data
        qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            Image,
            'vehicle/camera/image_color',
            self.__on_camera_image,
            qos_camera_data
        )

    def __on_camera_image(self, message):
        """Process camera image for lane detection and steering command.

        Args:
            message: ROS 2 Image message from camera
        """
        # Convert ROS image message to numpy array
        img = message.data
        img = np.frombuffer(img, dtype=np.uint8).reshape(
            (message.height, message.width, 4)
        )
        # Crop to bottom half of image to focus on lane area
        img = img[160:190, :]

        # Convert RGBA -> RGB -> HSV for color-based segmentation
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # Segment image: extract green lane markings (HSV range)
        mask = cv2.inRange(
            img,
            np.array([50, 110, 150]),
            np.array([120, 255, 255])
        )

        # Find contours and identify largest one (main lane area)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE
        )

        command_message = AckermannDrive()
        command_message.speed = TARGET_SPEED_KMH * KMH_TO_MPS
        command_message.steering_angle = 0.0

        if contours:
            # Get largest contour (most likely lane)
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour_center = cv2.moments(largest_contour)

            if largest_contour_center['m00'] != 0:
                # Compute lane centroid x-coordinate
                center_x = int(
                    largest_contour_center['m10'] /
                    largest_contour_center['m00']
                )
                # Calculate lateral error: distance from the image center
                # Positive error = lane to right, negative = lane to left
                image_center_x = message.width // 2
                error = center_x - image_center_x
                # Proportional steering control
                command_message.steering_angle = error * CONTROL_COEFFICIENT

        self.__ackermann_publisher.publish(command_message)


def main(args=None):
    """Initialize and run the lane follower node."""
    rclpy.init(args=args)
    follower = LaneFollower()
    rclpy.spin(follower)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
