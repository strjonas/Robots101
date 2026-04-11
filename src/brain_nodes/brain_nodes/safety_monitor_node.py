from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from brain_nodes.constants import TOPIC_SAFETY_STOP
from brain_nodes.math_utils import finite_min


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.declare_parameter("stop_distance_m", 0.22)
        self.declare_parameter("front_half_angle_deg", 20.0)

        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.front_half_angle_deg = float(self.get_parameter("front_half_angle_deg").value)

        self.publisher = self.create_publisher(Bool, TOPIC_SAFETY_STOP, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.last_state = False

    def _scan_cb(self, scan: LaserScan) -> None:
        if not scan.ranges:
            return

        center_index = len(scan.ranges) // 2
        half_width = max(1, int(math.radians(self.front_half_angle_deg) / scan.angle_increment))
        window = scan.ranges[max(0, center_index - half_width): min(len(scan.ranges), center_index + half_width)]
        min_range = finite_min(window)
        stop = math.isfinite(min_range) and min_range < self.stop_distance

        if stop != self.last_state:
            self.last_state = stop
            self.get_logger().warning(f"Safety stop -> {stop} (front min {min_range:.3f} m)")

        self.publisher.publish(Bool(data=stop))


def main() -> None:
    rclpy.init()
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
