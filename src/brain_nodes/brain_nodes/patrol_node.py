from __future__ import annotations

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
import rclpy
from brain_interfaces.msg import ControlMode, ObservationSummary
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from rclpy.node import Node

from brain_nodes.constants import DEFAULT_SEMANTIC_MAP_PATH, TOPIC_CONTROL_MODE, TOPIC_OBSERVATION_SUMMARY
from brain_nodes.semantic_map import SemanticMap


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_node")
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.declare_parameter("retry_delay_sec", 2.0)
        self.semantic_map = SemanticMap.load(Path(str(self.get_parameter("semantic_map_path").value)))
        self.retry_delay = Duration(seconds=float(self.get_parameter("retry_delay_sec").value))
        self.targets = list(self.semantic_map.patrol_targets())
        self.index = 0
        self.control_mode = ControlMode()
        self.localization_ok = False
        self.next_attempt_time = self.get_clock().now()
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.nav_goal_handle = None
        self.create_subscription(ControlMode, TOPIC_CONTROL_MODE, self._mode_cb, 10)
        self.create_subscription(ObservationSummary, TOPIC_OBSERVATION_SUMMARY, self._observation_cb, 10)
        self.create_timer(0.5, self._tick)

    def _mode_cb(self, message: ControlMode) -> None:
        self.control_mode = message
        if message.mode != ControlMode.PATROL:
            self._cancel_goal()
            self.next_attempt_time = self.get_clock().now()

    def _observation_cb(self, message: ObservationSummary) -> None:
        self.localization_ok = bool(message.localization_ok)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.control_mode.mode != ControlMode.PATROL:
            return
        if not self.targets:
            return
        if not self.localization_ok:
            return
        if now < self.next_attempt_time:
            return
        if self.nav_goal_handle is not None:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.next_attempt_time = now + self.retry_delay
            return

        target = self.targets[self.index % len(self.targets)]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.semantic_map.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target.x
        goal.pose.pose.position.y = target.y
        goal.pose.pose.orientation.z = math.sin(target.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target.yaw / 2.0)

        self.get_logger().info(f"Patrol goal -> {target.name}")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self.nav_goal_handle = future.result()
        if self.nav_goal_handle is None or not self.nav_goal_handle.accepted:
            self.get_logger().warning("Patrol goal rejected")
            self.nav_goal_handle = None
            self.next_attempt_time = self.get_clock().now() + self.retry_delay
            return
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.index += 1
        else:
            self.get_logger().warning("Patrol goal did not succeed")
            self.next_attempt_time = self.get_clock().now() + self.retry_delay
        self.nav_goal_handle = None

    def _cancel_goal(self) -> None:
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self.nav_goal_handle = None


def main() -> None:
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
