"""Safety monitor: watches the lidar and raises a flag when something is right in front.

Subscribes:  /scan               (sensor_msgs/LaserScan)
Publishes:   /brain/safety_stop  (std_msgs/Bool, True = obstacle ahead)

This node only reports. The command arbiter is the one that acts on the flag
(it blocks forward motion), so there is a single place where velocities change.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from brain_nodes.constants import TOPIC_SAFETY_STOP
from brain_nodes.scan_utils import FRONT, sector_min


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        # Distances are measured from the lidar, which sits about 0.13 m behind the front edge.
        self.declare_parameter("stop_distance_m", 0.22)
        self.declare_parameter("front_half_angle_deg", 20.0)

        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.front_half_angle = math.radians(float(self.get_parameter("front_half_angle_deg").value))

        self.publisher = self.create_publisher(Bool, TOPIC_SAFETY_STOP, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.last_state = False

    def _scan_cb(self, scan: LaserScan) -> None:
        if not scan.ranges:
            return

        front_min = sector_min(scan.ranges, scan.angle_min, scan.angle_increment, FRONT, self.front_half_angle)
        stop = front_min < self.stop_distance

        if stop != self.last_state:
            self.last_state = stop
            self.get_logger().warning(f"Safety stop -> {stop} (front min {front_min:.3f} m)")

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
