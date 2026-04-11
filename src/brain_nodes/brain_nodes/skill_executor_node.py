from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from brain_interfaces.msg import BrainAction, ControlMode, ExecutorStatus, TrackedObjectArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from brain_nodes.constants import (
    ACTION_TYPE_TO_NAME,
    DEFAULT_SEMANTIC_MAP_PATH,
    TOPIC_CMD_VEL_EXECUTOR,
    TOPIC_CONTROL_MODE,
    TOPIC_EXECUTOR_STATUS,
    TOPIC_PLANNER_ACTION,
    TOPIC_TRACKED_OBJECTS,
)
from brain_nodes.math_utils import clamp, finite_min, normalize_angle, quaternion_to_yaw
from brain_nodes.semantic_map import SemanticMap


@dataclass
class DirectActionState:
    start_x: float = 0.0
    start_y: float = 0.0
    start_yaw: float = 0.0
    target_yaw: float = 0.0
    target_distance: float = 0.0
    accumulated_angle: float = 0.0
    previous_yaw: float = 0.0
    lost_target_since_ns: int = 0


class SkillExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("skill_executor")
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.declare_parameter("linear_speed_mps", 0.16)
        self.declare_parameter("angular_speed_rps", 0.9)
        self.declare_parameter("angle_tolerance_rad", 0.05)
        self.declare_parameter("distance_tolerance_m", 0.05)
        self.declare_parameter("follow_timeout_sec", 2.0)
        self.declare_parameter("safe_stop_distance_m", 0.28)

        self.semantic_map = SemanticMap.load(Path(str(self.get_parameter("semantic_map_path").value)))
        self.linear_speed = float(self.get_parameter("linear_speed_mps").value)
        self.angular_speed = float(self.get_parameter("angular_speed_rps").value)
        self.angle_tolerance = float(self.get_parameter("angle_tolerance_rad").value)
        self.distance_tolerance = float(self.get_parameter("distance_tolerance_m").value)
        self.follow_timeout_sec = float(self.get_parameter("follow_timeout_sec").value)
        self.safe_stop_distance = float(self.get_parameter("safe_stop_distance_m").value)

        self.cmd_publisher = self.create_publisher(Twist, TOPIC_CMD_VEL_EXECUTOR, 10)
        self.status_publisher = self.create_publisher(ExecutorStatus, TOPIC_EXECUTOR_STATUS, 10)

        self.create_subscription(BrainAction, TOPIC_PLANNER_ACTION, self._action_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(TrackedObjectArray, TOPIC_TRACKED_OBJECTS, self._tracked_cb, 10)
        self.create_subscription(ControlMode, TOPIC_CONTROL_MODE, self._mode_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_timer(0.05, self._tick)

        self.current_action = None
        self.direct_state = DirectActionState()
        self.current_odom = None
        self.front_range = float("inf")
        self.tracked_objects = TrackedObjectArray()
        self.control_mode = ControlMode()
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.last_running_status_ns = 0
        self._publish_status(ExecutorStatus.IDLE, "Idle", "IDLE", 0.0)

    def _odom_cb(self, message: Odometry) -> None:
        self.current_odom = message

    def _scan_cb(self, message: LaserScan) -> None:
        if message.ranges:
            quarter = max(1, len(message.ranges) // 8)
            self.front_range = finite_min(message.ranges[:quarter] + message.ranges[-quarter:])

    def _tracked_cb(self, message: TrackedObjectArray) -> None:
        self.tracked_objects = message

    def _mode_cb(self, message: ControlMode) -> None:
        self.control_mode = message
        if self.current_action is not None and message.mode != ControlMode.BRAIN_TASK:
            self._finish_action(ExecutorStatus.PREEMPTED, f"Preempted by control mode {message.mode}")

    def _action_cb(self, message: BrainAction) -> None:
        if not message.valid:
            self._publish_status(ExecutorStatus.REJECTED, "Rejected invalid planner action", "INVALID", 0.0)
            return

        self._cancel_nav_goal()
        self._stop_motion()
        self.current_action = message

        if message.action_type == BrainAction.STOP:
            self._finish_action(ExecutorStatus.SUCCEEDED, message.rationale or "Planner issued STOP")
            return

        if self.current_odom is None:
            self._finish_action(ExecutorStatus.FAILED, "Cannot execute without odometry")
            return

        pose = self.current_odom.pose.pose
        yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        self.direct_state = DirectActionState(
            start_x=float(pose.position.x),
            start_y=float(pose.position.y),
            start_yaw=yaw,
            target_yaw=normalize_angle(yaw + message.angle_rad),
            target_distance=abs(message.distance_m),
            accumulated_angle=0.0,
            previous_yaw=yaw,
            lost_target_since_ns=0,
        )

        if message.action_type == BrainAction.GOTO_SEMANTIC:
            self._start_nav_goal(message)
            return

        if message.action_type == BrainAction.LOOK_AROUND:
            self.direct_state.target_yaw = yaw

        self._publish_status(ExecutorStatus.RUNNING, "Started direct action", ACTION_TYPE_TO_NAME[message.action_type], 0.0)

    def _start_nav_goal(self, action: BrainAction) -> None:
        target = self.semantic_map.get(action.target_id)
        if target is None:
            self._finish_action(ExecutorStatus.FAILED, f"Unknown semantic target '{action.target_id}'")
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._finish_action(ExecutorStatus.FAILED, "NavigateToPose action server unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.semantic_map.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target.x
        goal.pose.pose.position.y = target.y
        goal.pose.pose.orientation.z = math.sin(target.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target.yaw / 2.0)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)
        self._publish_status(ExecutorStatus.RUNNING, f"Navigating to {target.name}", "GOTO_SEMANTIC", 0.0)

    def _on_nav_goal_response(self, future) -> None:
        self.nav_goal_handle = future.result()
        if self.nav_goal_handle is None or not self.nav_goal_handle.accepted:
            self._finish_action(ExecutorStatus.FAILED, "NavigateToPose goal rejected")
            return
        self.nav_result_future = self.nav_goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        if result is None:
            self._finish_action(ExecutorStatus.FAILED, "NavigateToPose returned no result")
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_action(ExecutorStatus.SUCCEEDED, "Reached semantic target")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self._finish_action(ExecutorStatus.PREEMPTED, "Navigation goal canceled")
        else:
            self._finish_action(ExecutorStatus.FAILED, f"Navigation failed with status {result.status}")

    def _tick(self) -> None:
        if self.current_action is None or self.current_odom is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_running_status_ns > int(0.5 * 1e9):
            self.last_running_status_ns = now_ns
            self._publish_status(
                ExecutorStatus.RUNNING,
                "Executing direct action",
                ACTION_TYPE_TO_NAME[self.current_action.action_type],
                0.0,
            )

        if self.current_action.action_type == BrainAction.GOTO_SEMANTIC:
            return

        pose = self.current_odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        if self.current_action.action_type == BrainAction.TURN:
            self._tick_turn(yaw)
        elif self.current_action.action_type == BrainAction.DRIVE:
            self._tick_drive(x, y, yaw)
        elif self.current_action.action_type == BrainAction.LOOK_AROUND:
            self._tick_lookaround(yaw)
        elif self.current_action.action_type == BrainAction.FOLLOW_OBJECT:
            self._tick_follow()

    def _tick_turn(self, yaw: float) -> None:
        error = normalize_angle(self.direct_state.target_yaw - yaw)
        if abs(error) < self.angle_tolerance:
            self._finish_action(ExecutorStatus.SUCCEEDED, "Turn complete")
            return
        twist = Twist()
        twist.angular.z = clamp(1.8 * error, -self.angular_speed, self.angular_speed)
        self.cmd_publisher.publish(twist)

    def _tick_drive(self, x: float, y: float, yaw: float) -> None:
        if self.current_action.distance_m >= 0.0 and self.front_range < self.safe_stop_distance:
            self._finish_action(ExecutorStatus.FAILED, "Obstacle too close for drive action")
            return

        dx = x - self.direct_state.start_x
        dy = y - self.direct_state.start_y
        traveled = math.sqrt(dx * dx + dy * dy)
        if traveled >= self.direct_state.target_distance - self.distance_tolerance:
            self._finish_action(ExecutorStatus.SUCCEEDED, "Drive complete")
            return

        heading_error = normalize_angle(self.direct_state.start_yaw - yaw)
        twist = Twist()
        direction = 1.0 if self.current_action.distance_m >= 0.0 else -1.0
        twist.linear.x = direction * min(self.linear_speed, max(0.05, self.current_action.speed_limit))
        twist.angular.z = clamp(1.2 * heading_error, -0.4, 0.4)
        self.cmd_publisher.publish(twist)

    def _tick_lookaround(self, yaw: float) -> None:
        delta = abs(normalize_angle(yaw - self.direct_state.previous_yaw))
        self.direct_state.accumulated_angle += delta
        self.direct_state.previous_yaw = yaw
        if self.direct_state.accumulated_angle >= (2.0 * math.pi - self.angle_tolerance):
            self._finish_action(ExecutorStatus.SUCCEEDED, "Look-around complete")
            return
        twist = Twist()
        twist.angular.z = self.angular_speed * 0.7
        self.cmd_publisher.publish(twist)

    def _tick_follow(self) -> None:
        target = None
        if self.current_action.follow_track_id:
            for candidate in self.tracked_objects.objects:
                if candidate.track_id == self.current_action.follow_track_id:
                    target = candidate
                    break
        if target is None and self.current_action.follow_label:
            matches = [candidate for candidate in self.tracked_objects.objects if candidate.label == self.current_action.follow_label]
            if matches:
                target = max(matches, key=lambda item: item.confidence)

        if target is None:
            if self.direct_state.lost_target_since_ns == 0:
                self.direct_state.lost_target_since_ns = self.get_clock().now().nanoseconds
            elapsed = (self.get_clock().now().nanoseconds - self.direct_state.lost_target_since_ns) / 1e9
            if elapsed > self.follow_timeout_sec:
                self._finish_action(ExecutorStatus.FAILED, "Follow target lost")
            return

        self.direct_state.lost_target_since_ns = 0
        center_error = target.center_x - 0.5
        distance_error = target.estimated_distance_m - max(0.3, self.current_action.preferred_distance_m)

        twist = Twist()
        twist.angular.z = clamp(-2.4 * center_error, -self.angular_speed, self.angular_speed)
        if self.front_range < self.safe_stop_distance and distance_error > 0.0:
            twist.linear.x = 0.0
        else:
            twist.linear.x = clamp(0.5 * distance_error, -0.08, self.linear_speed)
        self.cmd_publisher.publish(twist)

    def _cancel_nav_goal(self) -> None:
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self.nav_goal_handle = None
        self.nav_result_future = None

    def _finish_action(self, state: int, detail: str) -> None:
        action_name = ACTION_TYPE_TO_NAME[self.current_action.action_type] if self.current_action is not None else "IDLE"
        task_id = self.current_action.task_id if self.current_action is not None else ""
        self._stop_motion()
        self._cancel_nav_goal()
        self._publish_status(state, detail, action_name, 1.0 if state == ExecutorStatus.SUCCEEDED else 0.0, task_id=task_id)
        self.current_action = None

    def _publish_status(
        self,
        state: int,
        detail: str,
        action_name: str,
        progress: float,
        task_id: str = "",
    ) -> None:
        message = ExecutorStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.task_id = task_id or (self.current_action.task_id if self.current_action is not None else "")
        message.executor_name = "skill_executor"
        message.state = state
        message.detail = detail
        message.active_action_type = action_name
        message.progress = progress
        self.status_publisher.publish(message)

    def _stop_motion(self) -> None:
        self.cmd_publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = SkillExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
