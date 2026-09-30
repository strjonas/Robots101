"""Skill executor: turns one planner action into actual robot motion.

Subscribes:    /brain/planner_action, /odom, /scan, /brain/tracked_objects, /brain/control_mode
Publishes:     /brain/cmd_vel_executor (velocity, goes through the arbiter)
               /brain/executor_status  (RUNNING heartbeat, then one final state)
Action client: /navigate_to_pose (Nav2), used for GOTO_SEMANTIC

Two styles of skill live here:
  - TURN, DRIVE, LOOK_AROUND and FOLLOW_OBJECT are small feedback loops that run
    at 20 Hz: measure (odometry, lidar, detections), compare with the target,
    publish a velocity proportional to the error. FOLLOW_OBJECT runs until the
    task is cleared, or until the target has been out of sight for a while
    (it spins in place to look for it first).
  - GOTO_SEMANTIC looks the name up in the semantic map and hands the pose to
    Nav2, which does path planning and obstacle avoidance on its own.

Every action ends with exactly one final status (SUCCEEDED, FAILED, PREEMPTED or
REJECTED). The planner and the task server wait for it before moving on.
"""

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
from brain_nodes.follow_logic import follow_command, search_direction
from brain_nodes.math_utils import clamp, normalize_angle, quaternion_to_yaw
from brain_nodes.scan_utils import FRONT, REAR, sector_min
from brain_nodes.semantic_map import SemanticMap


@dataclass
class DirectActionState:
    start_x: float = 0.0
    start_y: float = 0.0
    start_yaw: float = 0.0
    target_angle: float = 0.0
    target_distance: float = 0.0
    turned_angle: float = 0.0
    previous_yaw: float = 0.0
    lost_target_since_ns: int = 0
    followed_track_id: str = ""
    last_target_bearing: float | None = None


class SkillExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("skill_executor")
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.declare_parameter("linear_speed_mps", 0.16)
        self.declare_parameter("angular_speed_rps", 0.9)
        self.declare_parameter("angle_tolerance_rad", 0.05)
        self.declare_parameter("distance_tolerance_m", 0.05)
        self.declare_parameter("follow_speed_mps", 0.22)
        self.declare_parameter("follow_search_sec", 15.0)
        self.declare_parameter("safe_stop_distance_m", 0.28)

        self.semantic_map = SemanticMap.load(Path(str(self.get_parameter("semantic_map_path").value)))
        self.linear_speed = float(self.get_parameter("linear_speed_mps").value)
        self.angular_speed = float(self.get_parameter("angular_speed_rps").value)
        self.angle_tolerance = float(self.get_parameter("angle_tolerance_rad").value)
        self.distance_tolerance = float(self.get_parameter("distance_tolerance_m").value)
        self.follow_speed = float(self.get_parameter("follow_speed_mps").value)
        self.follow_search_sec = float(self.get_parameter("follow_search_sec").value)
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
        self.rear_range = float("inf")
        self.tracked_objects = TrackedObjectArray()
        self.control_mode = ControlMode()
        self.control_mode.mode = ControlMode.IDLE
        self.nav_goal_handle = None
        self.last_running_status_ns = 0
        self._publish_status(ExecutorStatus.IDLE, "Idle", "IDLE", 0.0)

    def _odom_cb(self, message: Odometry) -> None:
        self.current_odom = message

    def _scan_cb(self, message: LaserScan) -> None:
        if not message.ranges:
            return
        half_width = math.radians(30.0)
        self.front_range = sector_min(message.ranges, message.angle_min, message.angle_increment, FRONT, half_width)
        self.rear_range = sector_min(message.ranges, message.angle_min, message.angle_increment, REAR, half_width)

    def _tracked_cb(self, message: TrackedObjectArray) -> None:
        self.tracked_objects = message

    def _mode_cb(self, message: ControlMode) -> None:
        self.control_mode = message
        if self.current_action is not None and message.mode != ControlMode.BRAIN_TASK:
            self._finish_action(ExecutorStatus.PREEMPTED, f"Preempted by control mode {message.mode}")

    def _action_cb(self, message: BrainAction) -> None:
        action_name = ACTION_TYPE_TO_NAME.get(message.action_type, "INVALID")
        if not message.valid or action_name == "INVALID":
            self._publish_status(
                ExecutorStatus.REJECTED,
                f"Rejected invalid planner action: {message.rationale}",
                "INVALID",
                0.0,
                task_id=message.task_id,
            )
            return
        if self.control_mode.mode != ControlMode.BRAIN_TASK:
            self._publish_status(
                ExecutorStatus.PREEMPTED,
                "Ignored action: robot is not in BRAIN_TASK mode",
                action_name,
                0.0,
                task_id=message.task_id,
            )
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
            target_angle=2.0 * math.pi if message.action_type == BrainAction.LOOK_AROUND else float(message.angle_rad),
            target_distance=abs(message.distance_m),
            previous_yaw=yaw,
        )

        if message.action_type == BrainAction.GOTO_SEMANTIC:
            self._start_nav_goal(message)
            return

        self._publish_status(ExecutorStatus.RUNNING, f"Started {action_name}", action_name, 0.0)

    # ---- GOTO_SEMANTIC: delegate to Nav2 -------------------------------------------------

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
        goal_handle = future.result()
        accepted = goal_handle is not None and goal_handle.accepted
        if self.current_action is None or self.current_action.action_type != BrainAction.GOTO_SEMANTIC:
            # The action was preempted while Nav2 was still answering.
            if accepted:
                goal_handle.cancel_goal_async()
            return
        if not accepted:
            self._finish_action(ExecutorStatus.FAILED, "NavigateToPose goal rejected")
            return
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._on_nav_result(goal_handle, result_future)
        )

    def _on_nav_result(self, goal_handle, future) -> None:
        if goal_handle is not self.nav_goal_handle:
            return  # result of a goal we already moved on from
        result = future.result()
        if result is None:
            self._finish_action(ExecutorStatus.FAILED, "NavigateToPose returned no result")
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_action(ExecutorStatus.SUCCEEDED, "Reached semantic target")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self._finish_action(ExecutorStatus.PREEMPTED, "Navigation goal canceled")
        else:
            self._finish_action(ExecutorStatus.FAILED, f"Navigation failed with status {result.status}")

    def _cancel_nav_goal(self) -> None:
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        self.nav_goal_handle = None

    # ---- Direct skills: small feedback loops -----------------------------------------------

    def _tick(self) -> None:
        if self.current_action is None or self.current_odom is None:
            return

        action_type = self.current_action.action_type
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_running_status_ns > int(0.5 * 1e9):
            self.last_running_status_ns = now_ns
            name = ACTION_TYPE_TO_NAME[action_type]
            self._publish_status(ExecutorStatus.RUNNING, f"Executing {name}", name, 0.0)

        if action_type == BrainAction.GOTO_SEMANTIC:
            return  # Nav2 is driving; we only wait for its result.

        pose = self.current_odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        if action_type in (BrainAction.TURN, BrainAction.LOOK_AROUND):
            self._tick_turn(yaw)
        elif action_type == BrainAction.DRIVE:
            self._tick_drive(x, y, yaw)
        elif action_type == BrainAction.FOLLOW_OBJECT:
            self._tick_follow()

    def _tick_turn(self, yaw: float) -> None:
        # Add up the small yaw changes between ticks. Comparing against a single
        # target heading would break for turns of 180 degrees or more.
        state = self.direct_state
        state.turned_angle += normalize_angle(yaw - state.previous_yaw)
        state.previous_yaw = yaw

        error = state.target_angle - state.turned_angle
        if abs(error) < self.angle_tolerance:
            self._finish_action(ExecutorStatus.SUCCEEDED, "Turn complete")
            return
        twist = Twist()
        twist.angular.z = clamp(1.8 * error, -self.angular_speed, self.angular_speed)
        self.cmd_publisher.publish(twist)

    def _tick_drive(self, x: float, y: float, yaw: float) -> None:
        forward = self.current_action.distance_m >= 0.0
        clearance = self.front_range if forward else self.rear_range
        if clearance < self.safe_stop_distance:
            self._finish_action(ExecutorStatus.FAILED, "Obstacle too close for drive action")
            return

        traveled = math.hypot(x - self.direct_state.start_x, y - self.direct_state.start_y)
        if traveled >= self.direct_state.target_distance - self.distance_tolerance:
            self._finish_action(ExecutorStatus.SUCCEEDED, "Drive complete")
            return

        heading_error = normalize_angle(self.direct_state.start_yaw - yaw)
        twist = Twist()
        speed = min(self.linear_speed, max(0.05, self.current_action.speed_limit))
        twist.linear.x = speed if forward else -speed
        twist.angular.z = clamp(1.2 * heading_error, -0.4, 0.4)
        self.cmd_publisher.publish(twist)

    def _tick_follow(self) -> None:
        state = self.direct_state
        target = self._find_follow_target()
        now_ns = self.get_clock().now().nanoseconds

        if target is None:
            # Out of sight: spin towards where it was last seen, give up after a while.
            if state.lost_target_since_ns == 0:
                state.lost_target_since_ns = now_ns
            if (now_ns - state.lost_target_since_ns) / 1e9 > self.follow_search_sec:
                self._finish_action(ExecutorStatus.FAILED, "Follow target lost")
                return
            twist = Twist()
            twist.angular.z = 0.5 * search_direction(state.last_target_bearing)
            self.cmd_publisher.publish(twist)
            return

        state.lost_target_since_ns = 0
        state.followed_track_id = target.track_id
        state.last_target_bearing = target.bearing_rad
        linear, angular = follow_command(
            bearing_rad=target.bearing_rad,
            distance_m=target.estimated_distance_m,
            preferred_distance_m=max(0.5, self.current_action.preferred_distance_m),
            obstacle_ahead=self.front_range < self.safe_stop_distance,
            max_linear_mps=self.follow_speed,
            max_angular_rps=self.angular_speed,
        )
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_publisher.publish(twist)

    def _find_follow_target(self):
        """The object to follow: the one we were already following, else the closest with the right label."""
        wanted_id = self.direct_state.followed_track_id or self.current_action.follow_track_id
        for candidate in self.tracked_objects.objects:
            if wanted_id and candidate.track_id == wanted_id:
                return candidate
        label = self.current_action.follow_label
        matches = [obj for obj in self.tracked_objects.objects if label and obj.label == label]
        return min(matches, key=lambda obj: obj.estimated_distance_m) if matches else None

    # ---- Bookkeeping ------------------------------------------------------------------------

    def _finish_action(self, state: int, detail: str) -> None:
        action_name = ACTION_TYPE_TO_NAME[self.current_action.action_type] if self.current_action is not None else "IDLE"
        task_id = self.current_action.task_id if self.current_action is not None else ""
        self._stop_motion()
        self._cancel_nav_goal()
        self.current_action = None
        self._publish_status(state, detail, action_name, 1.0 if state == ExecutorStatus.SUCCEEDED else 0.0, task_id=task_id)

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
