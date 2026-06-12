#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from collections import Counter


class MapValueChecker(Node):
    def __init__(self):
        super().__init__("map_value_checker")
        self.sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.callback,
            10,
        )

    def callback(self, msg):
        counter = Counter(msg.data)
        total = len(msg.data)

        unknown = sum(c for v, c in counter.items() if v == -1)
        likely_free = sum(c for v, c in counter.items() if 0 <= v < 50)
        uncertain = sum(c for v, c in counter.items() if v == 50)
        likely_occupied = sum(c for v, c in counter.items() if 50 < v <= 100)

        print(f"Map size: {msg.info.width} x {msg.info.height}")
        print(f"Resolution: {msg.info.resolution}")
        print(f"Origin: {msg.info.origin.position.x}, {msg.info.origin.position.y}")
        print()

        print("Grouped interpretation:")
        print(f"Unknown (-1):          {unknown:10d} cells ({unknown / total * 100:6.2f}%)")
        print(f"Likely free (0-49):    {likely_free:10d} cells ({likely_free / total * 100:6.2f}%)")
        print(f"Uncertain (50):        {uncertain:10d} cells ({uncertain / total * 100:6.2f}%)")
        print(f"Likely occupied 51-100:{likely_occupied:10d} cells ({likely_occupied / total * 100:6.2f}%)")

        print()
        print("Raw unique values:")
        for value, count in sorted(counter.items()):
            print(f"{value:4d}: {count:10d} cells ({count / total * 100:6.2f}%)")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = MapValueChecker()
    rclpy.spin(node)


if __name__ == "__main__":
    main()