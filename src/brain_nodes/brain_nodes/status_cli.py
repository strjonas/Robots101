"""Live status panel for the terminal: `pixi run status`.

Subscribes to the /brain/* topics and redraws a small summary once a second, so
you can see at a glance which mode the robot is in, who is driving, what the
planner decided and how the executor is doing. It only listens.
"""

from __future__ import annotations

import math

import rclpy
from brain_interfaces.msg import BrainAction, BrainTask, ControlMode, ExecutorStatus, ObservationSummary
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from brain_nodes.constants import (
    ACTION_TYPE_TO_NAME,
    TOPIC_ACTIVE_CMD_SOURCE,
    TOPIC_CMD_VEL,
    TOPIC_CONTROL_MODE,
    TOPIC_CURRENT_TASK,
    TOPIC_EXECUTOR_STATUS,
    TOPIC_OBSERVATION_SUMMARY,
    TOPIC_PLANNER_ACTION,
    TOPIC_SAFETY_STOP,
)

MODE_NAMES = {0: "MANUAL", 1: "PATROL", 2: "BRAIN_TASK", 3: "IDLE", 4: "EMERGENCY_STOP"}
EXECUTOR_STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "SUCCEEDED", 3: "FAILED", 4: "PREEMPTED", 5: "REJECTED"}
WAITING = "(no message yet)"


class StatusPanel(Node):
    def __init__(self) -> None:
        super().__init__("status_cli")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latest: dict[str, object] = {}
        for key, message_type, topic, qos in [
            ("mode", ControlMode, TOPIC_CONTROL_MODE, latched),
            ("task", BrainTask, TOPIC_CURRENT_TASK, latched),
            ("source", String, TOPIC_ACTIVE_CMD_SOURCE, 10),
            ("safety", Bool, TOPIC_SAFETY_STOP, 10),
            ("action", BrainAction, TOPIC_PLANNER_ACTION, 10),
            ("executor", ExecutorStatus, TOPIC_EXECUTOR_STATUS, 10),
            ("observation", ObservationSummary, TOPIC_OBSERVATION_SUMMARY, 10),
            ("cmd", Twist, TOPIC_CMD_VEL, 10),
        ]:
            self.create_subscription(message_type, topic, lambda message, key=key: self.latest.__setitem__(key, message), qos)
        self.create_timer(1.0, self._draw)

    def _draw(self) -> None:
        lines = ["robots101 status   (Ctrl-C to quit)", ""]

        mode = self.latest.get("mode")
        lines.append(f"mode        {MODE_NAMES.get(mode.mode, mode.mode)}  ({mode.detail})" if mode else f"mode        {WAITING}")

        source = self.latest.get("source")
        cmd = self.latest.get("cmd")
        velocity = f"linear {cmd.linear.x:+.2f} m/s  angular {cmd.angular.z:+.2f} rad/s" if cmd else WAITING
        lines.append(f"driving     {source.data if source else WAITING}  ->  {velocity}")

        safety = self.latest.get("safety")
        lines.append(f"safety      {'OBSTACLE AHEAD, forward blocked' if safety and safety.data else 'clear' if safety else WAITING}")

        task = self.latest.get("task")
        lines.append(f"task        {task.instruction if task and task.active else '(none)'}")

        action = self.latest.get("action")
        if action:
            name = ACTION_TYPE_TO_NAME.get(action.action_type, "?") if action.valid else "INVALID"
            lines.append(f"last plan   {name}  {self._action_details(action)}  \"{action.rationale}\"")
        else:
            lines.append("last plan   (none yet)")

        executor = self.latest.get("executor")
        if executor:
            state = EXECUTOR_STATE_NAMES.get(executor.state, executor.state)
            lines.append(f"executor    {state}  {executor.active_action_type}  {executor.detail}")
        else:
            lines.append(f"executor    {WAITING}")

        observation = self.latest.get("observation")
        if observation:
            frame = "map" if observation.localization_ok else "odom, not localized"
            lines.append(
                f"pose        x {observation.pose_x:+.2f}  y {observation.pose_y:+.2f}  "
                f"yaw {math.degrees(observation.pose_yaw):+.0f} deg  ({frame})  near: {observation.semantic_location}"
            )
            lines.append(
                f"lidar       front {observation.front_min_range:.2f}  left {observation.left_min_range:.2f}  "
                f"right {observation.right_min_range:.2f}  rear {observation.rear_min_range:.2f}  (m)"
            )
            seen = [
                f"{obj.label} {obj.estimated_distance_m:.1f} m at {math.degrees(obj.bearing_rad):+.0f} deg"
                for obj in observation.tracked_objects
            ]
            lines.append(f"sees        {', '.join(seen) or '(nothing detected)'}")
        else:
            lines.append(f"pose        {WAITING}")

        # Clear the screen and move the cursor home, then print the panel.
        print("\033[2J\033[H" + "\n".join(lines), flush=True)

    @staticmethod
    def _action_details(action: BrainAction) -> str:
        if action.action_type == BrainAction.TURN:
            return f"{math.degrees(action.angle_rad):+.0f} deg"
        if action.action_type == BrainAction.DRIVE:
            return f"{action.distance_m:+.2f} m"
        if action.action_type == BrainAction.GOTO_SEMANTIC:
            return action.target_id
        if action.action_type == BrainAction.FOLLOW_OBJECT:
            return action.follow_label or action.follow_track_id
        return ""


def main() -> None:
    rclpy.init()
    node = StatusPanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
