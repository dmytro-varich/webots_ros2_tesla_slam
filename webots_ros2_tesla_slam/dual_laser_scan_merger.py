# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Merge front and rear laser scans into a single scan in the base_link frame.

This module combines laser scan data from two lidar sensors (front and rear)
into a single unified laser scan for AMCL localization. The merger:
    - Transforms sensor coordinates from their respective frames to base_link
    - Handles temporal synchronization (scans must be recent within max_scan_age)
    - Merges range data into a unified angular grid (-pi to pi)
    - Applies proper coordinate transformations based on sensor positions/orientations

The Tesla robot has two lidars positioned at:
    - Front: forward-facing at approximate [2.05, 0.0] with 0 degrees yaw
    - Rear: backward-facing at approximate [-2.58, 0.0] with 180 degrees yaw
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DualLaserScanMerger(Node):
    """Merge dual lidar scans into unified base_link coordinate frame.

    Subscribes to front and rear LaserScan topics, transforms scan points from
    sensor frames to base_link frame, and publishes merged scan. Uses known
    sensor mounting positions and orientations for accurate coordinate transformation.

    Parameters:
        front_scan_topic: Front lidar topic (default: /scan_front)
        rear_scan_topic: Rear lidar topic (default: /scan_rear)
        output_topic: Merged scan topic (default: /scan)
        target_frame: Output frame ID (default: base_link)
        front_x, front_y, front_yaw: Front sensor pose relative to base_link
        rear_x, rear_y, rear_yaw: Rear sensor pose relative to base_link
        max_scan_age: Maximum time difference between scans (default: 0.25s)
    """

    def __init__(self):
        """Initialize dual laser scan merger node."""
        super().__init__('dual_laser_scan_merger')

        self.declare_parameter('front_scan_topic', '/scan_front')
        self.declare_parameter('rear_scan_topic', '/scan_rear')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('front_x', 2.05)
        self.declare_parameter('front_y', 0.0)
        self.declare_parameter('front_yaw', 0.0)
        self.declare_parameter('rear_x', -2.58)
        self.declare_parameter('rear_y', 0.0)
        self.declare_parameter('rear_yaw', math.pi)
        self.declare_parameter('max_scan_age', 0.25)

        self.target_frame = self.get_parameter('target_frame').value
        self.front_transform = (
            self.get_parameter('front_x').value,
            self.get_parameter('front_y').value,
            self.get_parameter('front_yaw').value,
        )
        self.rear_transform = (
            self.get_parameter('rear_x').value,
            self.get_parameter('rear_y').value,
            self.get_parameter('rear_yaw').value,
        )
        self.max_scan_age = self.get_parameter('max_scan_age').value
        front_scan_topic = self.get_parameter('front_scan_topic').value
        rear_scan_topic = self.get_parameter('rear_scan_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.front_scan = None
        self.rear_scan = None
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.create_subscription(
            LaserScan,
            front_scan_topic,
            self._front_scan_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            rear_scan_topic,
            self._rear_scan_callback,
            10,
        )

        self.get_logger().info(
            f'Merging laser scans: {front_scan_topic} + {rear_scan_topic} -> {output_topic}'
        )

    def _front_scan_callback(self, message):
        """Handle incoming front lidar scan."""
        self.front_scan = message
        self._try_publish()

    def _rear_scan_callback(self, message):
        """Handle incoming rear lidar scan."""
        self.rear_scan = message
        self._try_publish()

    def _try_publish(self):
        """Merge and publish scans if both are available and synchronized."""
        if self.front_scan is None or self.rear_scan is None:
            return
        # Check temporal synchronization: scans must be close in time
        front_stamp = self._stamp_to_seconds(self.front_scan.header.stamp)
        rear_stamp = self._stamp_to_seconds(self.rear_scan.header.stamp)
        if abs(front_stamp - rear_stamp) > self.max_scan_age:
            return

        output = LaserScan()
        output.header.stamp = (
            self.front_scan.header.stamp
            if front_stamp >= rear_stamp
            else self.rear_scan.header.stamp
        )
        output.header.frame_id = self.target_frame
        output.angle_min = -math.pi
        output.angle_max = math.pi
        output.angle_increment = min(
            self.front_scan.angle_increment,
            self.rear_scan.angle_increment,
        )
        output.time_increment = 0.0
        output.scan_time = max(self.front_scan.scan_time, self.rear_scan.scan_time)
        output.range_min = min(self.front_scan.range_min, self.rear_scan.range_min)
        output.range_max = max(self.front_scan.range_max, self.rear_scan.range_max)

        # Initialize output scan grid from -pi to pi
        bin_count = int(
            math.floor((output.angle_max - output.angle_min) / output.angle_increment)
        ) + 1
        output.ranges = [float('inf')] * bin_count
        output.intensities = []

        # Transform and merge scan points from both sensors into base_link frame
        self._add_scan(output, self.front_scan, self.front_transform)
        self._add_scan(output, self.rear_scan, self.rear_transform)
        self.publisher.publish(output)

    def _add_scan(self, output, scan, transform):
        """Transform and merge a single scan into output grid.

        Args:
            output: Output LaserScan to accumulate ranges
            scan: Input LaserScan from a sensor
            transform: (x, y, yaw) pose of sensor relative to base_link
        """
        transform_x, transform_y, transform_yaw = transform
        # Pre-compute rotation matrix elements
        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)

        for index, scan_range in enumerate(scan.ranges):
            # Skip invalid ranges
            if not math.isfinite(scan_range):
                continue
            if scan_range < scan.range_min or scan_range > scan.range_max:
                continue

            # Convert polar (range, angle) to Cartesian in sensor frame
            scan_angle = scan.angle_min + index * scan.angle_increment
            sensor_x = scan_range * math.cos(scan_angle)
            sensor_y = scan_range * math.sin(scan_angle)
            # Transform point from sensor frame to base_link frame
            base_x = transform_x + cos_yaw * sensor_x - sin_yaw * sensor_y
            base_y = transform_y + sin_yaw * sensor_x + cos_yaw * sensor_y
            base_range = math.hypot(base_x, base_y)
            if base_range < output.range_min or base_range > output.range_max:
                continue

            # Convert back to polar and insert into output grid (minimum wins)
            base_angle = math.atan2(base_y, base_x)
            output_index = int(round(
                (base_angle - output.angle_min) / output.angle_increment
            ))
            if 0 <= output_index < len(output.ranges):
                output.ranges[output_index] = min(
                    output.ranges[output_index],
                    base_range,
                )

    def _stamp_to_seconds(self, stamp):
        """Convert ROS 2 Time message to float seconds."""
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    """Initialize and run the dual laser scan merger node."""
    rclpy.init(args=args)
    merger = DualLaserScanMerger()
    rclpy.spin(merger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
