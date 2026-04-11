from __future__ import annotations

from dataclasses import dataclass

import rclpy
from brain_interfaces.msg import BrainTask, ControlMode, ObservationSummary
from brain_interfaces.srv import SetEmergencyStop
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from brain_nodes.constants import (
    SERVICE_SET_EMERGENCY_STOP,
    TOPIC_CMD_VEL_MANUAL,
    TOPIC_CONTROL_MODE,
    TOPIC_CURRENT_TASK,
    TOPIC_OBSERVATION_SUMMARY,
)


@dataclass
class EmergencyState:
    enabled: bool = False
    reason: str = ""


class ControlSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("control_supervisor")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_publisher = self.create_publisher(ControlMode, TOPIC_CONTROL_MODE, qos)
        self.create_subscription(Twist, TOPIC_CMD_VEL_MANUAL, self._on_manual_cmd, 10)
        self.create_subscription(BrainTask, TOPIC_CURRENT_TASK, self._on_task, qos)
        self.create_subscription(ObservationSummary, TOPIC_OBSERVATION_SUMMARY, self._on_observation, 10)
        self.create_service(SetEmergencyStop, SERVICE_SET_EMERGENCY_STOP, self._set_emergency_stop)

        self.declare_parameter("manual_timeout_sec", 0.6)
        self.declare_parameter("patrol_enabled", True)
        self.declare_parameter("require_localization_for_patrol", True)

        self.current_task = BrainTask()
        self.manual_timeout_sec = float(self.get_parameter("manual_timeout_sec").value)
        self.patrol_enabled = bool(self.get_parameter("patrol_enabled").value)
        self.require_localization_for_patrol = bool(self.get_parameter("require_localization_for_patrol").value)
        self.last_manual_time = self.get_clock().now()
        self.emergency_state = EmergencyState()
        self.localization_ok = False
        self.last_mode = None

        self.create_timer(0.1, self._publish_mode)

    def _on_manual_cmd(self, _: Twist) -> None:
        self.last_manual_time = self.get_clock().now()

    def _on_task(self, message: BrainTask) -> None:
        self.current_task = message

    def _on_observation(self, message: ObservationSummary) -> None:
        self.localization_ok = bool(message.localization_ok)

    def _set_emergency_stop(
        self,
        request: SetEmergencyStop.Request,
        response: SetEmergencyStop.Response,
    ) -> SetEmergencyStop.Response:
        self.emergency_state.enabled = request.enabled
        self.emergency_state.reason = request.reason.strip()
        response.success = True
        response.message = "Emergency stop enabled." if request.enabled else "Emergency stop released."
        self.get_logger().warning(response.message)
        return response

    def _publish_mode(self) -> None:
        now = self.get_clock().now()
        manual_active = (now - self.last_manual_time).nanoseconds / 1e9 < self.manual_timeout_sec

        message = ControlMode()
        message.header.stamp = now.to_msg()

        if self.emergency_state.enabled:
            message.mode = ControlMode.EMERGENCY_STOP
            message.source = "supervisor"
            message.detail = self.emergency_state.reason or "Emergency stop active"
        elif manual_active:
            message.mode = ControlMode.MANUAL
            message.source = "teleop"
            message.detail = "Recent manual input detected"
        elif self.current_task.active:
            message.mode = ControlMode.BRAIN_TASK
            message.source = "task_server"
            message.detail = self.current_task.instruction
        elif self.patrol_enabled and (self.localization_ok or not self.require_localization_for_patrol):
            message.mode = ControlMode.PATROL
            message.source = "supervisor"
            message.detail = "Default patrol mode"
        else:
            message.mode = ControlMode.IDLE
            message.source = "supervisor"
            if self.patrol_enabled and self.require_localization_for_patrol and not self.localization_ok:
                message.detail = "Waiting for localization"
            else:
                message.detail = "No active task"

        if message.mode != self.last_mode:
            self.get_logger().info(f"Control mode -> {int(message.mode)} ({message.detail})")
            self.last_mode = message.mode

        self.mode_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = ControlSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
