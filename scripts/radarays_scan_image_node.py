#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class RadarScanImageNode(Node):
    def __init__(self) -> None:
        super().__init__("radarays_scan_image")

        self.declare_parameter("input_scan_topic", "/radar/scan")
        self.declare_parameter("output_image_topic", "/radar/image")
        self.declare_parameter("image_height", 1024)
        self.declare_parameter("hit_value", 255)
        self.declare_parameter("trail_value", 96)
        self.declare_parameter("trail_length", 2)

        input_scan_topic = self.get_parameter("input_scan_topic").get_parameter_value().string_value
        output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value

        self._image_height = max(1, self.get_parameter("image_height").get_parameter_value().integer_value)
        self._hit_value = max(0, min(255, self.get_parameter("hit_value").get_parameter_value().integer_value))
        self._trail_value = max(0, min(255, self.get_parameter("trail_value").get_parameter_value().integer_value))
        self._trail_length = max(0, self.get_parameter("trail_length").get_parameter_value().integer_value)

        self._publisher = self.create_publisher(Image, output_image_topic, 10)
        self.create_subscription(LaserScan, input_scan_topic, self._on_scan, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        width = len(msg.ranges)
        height = int(self._image_height)
        image = bytearray(width * height)

        denom = max(msg.range_max - msg.range_min, 1e-6)
        for angle_index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            normalized = (distance - msg.range_min) / denom
            normalized = min(max(normalized, 0.0), 1.0)
            row = height - 1 - int(round(normalized * (height - 1)))

            base = row * width + angle_index
            image[base] = self._hit_value

            for offset in range(1, self._trail_length + 1):
                trail_row = min(height - 1, row + offset)
                trail_index = trail_row * width + angle_index
                image[trail_index] = max(image[trail_index], self._trail_value)

        out = Image()
        out.header = msg.header
        out.height = height
        out.width = width
        out.encoding = "mono8"
        out.is_bigendian = False
        out.step = width
        out.data = list(image)
        self._publisher.publish(out)


def main() -> int:
    rclpy.init()
    node = RadarScanImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
