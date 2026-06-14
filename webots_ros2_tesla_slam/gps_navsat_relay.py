# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

"""Relay Webots NavSatFix GPS data with covariance and simulated fix status.

This module converts raw GPS data from Webots into a properly formatted
NavSatFix message suitable for robot_localization. It adds:
    - Position covariance (uncertainty) matrices
    - Simulated FIX status when Webots reports NO_FIX
    - Configurable variance defaults for horizontal and vertical position

The relay ensures GPS data can be used by EKF fusion for global localization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

DEFAULT_HORIZONTAL_VARIANCE = 25.0
DEFAULT_VERTICAL_VARIANCE = 100.0


class GpsNavSatRelay(Node):
    """Relay and enhance GPS NavSatFix messages with covariance.

    Subscribes to raw GPS data, adds position covariance matrix (3x3 diagonal),
    coerces simulated NO_FIX status to FIX, and publishes for use in localization.

    Parameters:
        input_topic: Raw GPS NavSatFix topic (default: /gps/fix)
        output_topic: Enhanced NavSatFix output topic (default: /gps/navsat)
        horizontal_position_variance: Horizontal uncertainty (default: 25.0 m^2)
        vertical_position_variance: Vertical uncertainty (default: 100.0 m^2)
    """
    def __init__(self):
        """Initialize GPS NavSatFix relay node."""
        super().__init__('gps_navsat_relay')

        self.declare_parameter('input_topic', '/gps/fix')
        self.declare_parameter('output_topic', '/gps/navsat')
        self.declare_parameter(
            'horizontal_position_variance',
            DEFAULT_HORIZONTAL_VARIANCE,
        )
        self.declare_parameter(
            'vertical_position_variance',
            DEFAULT_VERTICAL_VARIANCE,
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.horizontal_position_variance = self.get_parameter(
            'horizontal_position_variance'
        ).value
        self.vertical_position_variance = self.get_parameter(
            'vertical_position_variance'
        ).value

        self.publisher = self.create_publisher(
            NavSatFix,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            NavSatFix,
            input_topic,
            self._gps_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Relaying NavSatFix GPS data: {input_topic} -> {output_topic}'
        )

    def _gps_callback(self, message):
        """Process and relay GPS message with covariance.

        Args:
            message: NavSatFix GPS message from Webots
        """
        # Add position covariance (3x3 diagonal matrix in row-major order)
        message.position_covariance = self._position_covariance(message)
        message.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )
        # In simulation, Webots may report NO_FIX even though coordinates are valid.
        if message.status.status == NavSatStatus.STATUS_NO_FIX:
            message.status.status = NavSatStatus.STATUS_FIX
        self.publisher.publish(message)

    def _position_covariance(self, message):
        """Build covariance matrix with horizontal/vertical variance.

        3x3 covariance matrix layout (row-major):
        [0] = X variance    [1] = XY         [2] = XZ
        [3] = YX            [4] = Y variance [5] = YZ
        [6] = ZX            [7] = ZY         [8] = Z variance
        """
        covariance = list(message.position_covariance)
        # Ensure minimum variance on diagonal elements
        covariance[0] = max(covariance[0], self.horizontal_position_variance)
        covariance[4] = max(covariance[4], self.horizontal_position_variance)
        covariance[8] = max(covariance[8], self.vertical_position_variance)
        return covariance


def main(args=None):
    """Initialize and run the GPS NavSatFix relay node."""
    rclpy.init(args=args)
    relay = GpsNavSatRelay()
    rclpy.spin(relay)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
