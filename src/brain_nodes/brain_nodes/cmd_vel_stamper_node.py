from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

from brain_nodes.constants import TOPIC_CMD_VEL, TOPIC_CMD_VEL_STAMPED


class CmdVelStamperNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_stamper")
        self.declare_parameter("frame_id", "base_link")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publisher = self.create_publisher(TwistStamped, TOPIC_CMD_VEL_STAMPED, 10)
        self.create_subscription(Twist, TOPIC_CMD_VEL, self._cmd_cb, 10)

    def _cmd_cb(self, message: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = message
        self.publisher.publish(stamped)


def main() -> None:
    rclpy.init()
    node = CmdVelStamperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
