from __future__ import annotations

from datetime import datetime, timedelta, timezone
from uuid import uuid4

import rclpy
from builtin_interfaces.msg import Time
from brain_interfaces.msg import BrainTask, ExecutorStatus
from brain_interfaces.srv import ClearTask, SubmitTask
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from brain_nodes.constants import (
    SERVICE_CLEAR_TASK,
    SERVICE_SUBMIT_TASK,
    TOPIC_CURRENT_TASK,
    TOPIC_EXECUTOR_STATUS,
)


class TaskServerNode(Node):
    def __init__(self) -> None:
        super().__init__("task_server")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_publisher = self.create_publisher(BrainTask, TOPIC_CURRENT_TASK, qos)
        self.create_subscription(ExecutorStatus, TOPIC_EXECUTOR_STATUS, self._on_executor_status, 10)
        self.create_service(SubmitTask, SERVICE_SUBMIT_TASK, self._submit_task)
        self.create_service(ClearTask, SERVICE_CLEAR_TASK, self._clear_task)
        self.timer = self.create_timer(0.5, self._on_timer)

        self.current_task = BrainTask()
        self.current_task.active = False
        self.auto_complete_actions = {"TURN", "DRIVE", "GOTO_SEMANTIC", "LOOK_AROUND"}

    def _submit_task(self, request: SubmitTask.Request, response: SubmitTask.Response) -> SubmitTask.Response:
        now = datetime.now(timezone.utc)
        expires_at = now + timedelta(seconds=max(request.expires_after_sec, 600))
        task = BrainTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.task_id = str(uuid4())
        task.instruction = request.instruction.strip()
        task.target_hint = request.target_hint.strip()
        task.expires_at = Time(sec=int(expires_at.timestamp()))
        task.active = bool(task.instruction)

        if not task.active:
            response.accepted = False
            response.message = "Instruction cannot be empty."
            return response

        self.current_task = task
        self.task_publisher.publish(self.current_task)
        response.accepted = True
        response.task_id = task.task_id
        response.message = "Task accepted."
        self.get_logger().info(f"Accepted task {task.task_id}: {task.instruction}")
        return response

    def _clear_task(self, _: ClearTask.Request, response: ClearTask.Response) -> ClearTask.Response:
        had_task = self.current_task.active
        self.current_task = BrainTask()
        self.current_task.active = False
        self.task_publisher.publish(self.current_task)
        response.cleared = had_task
        response.message = "Task cleared." if had_task else "No task was active."
        return response

    def _on_executor_status(self, message: ExecutorStatus) -> None:
        if not self.current_task.active:
            return
        if message.task_id != self.current_task.task_id:
            return
        if message.state == ExecutorStatus.SUCCEEDED and message.active_action_type in self.auto_complete_actions:
            self.get_logger().info(
                f"Auto-completing task {message.task_id} after {message.active_action_type} success"
            )
            self.current_task = BrainTask()
            self.current_task.active = False
            self.task_publisher.publish(self.current_task)

    def _on_timer(self) -> None:
        if not self.current_task.active:
            self.task_publisher.publish(self.current_task)
            return

        expires_at = self.current_task.expires_at.sec
        if expires_at and datetime.now(timezone.utc).timestamp() > expires_at:
            self.get_logger().warning(f"Task {self.current_task.task_id} expired")
            self.current_task = BrainTask()
            self.current_task.active = False

        self.current_task.header.stamp = self.get_clock().now().to_msg()
        self.task_publisher.publish(self.current_task)


def main() -> None:
    rclpy.init()
    node = TaskServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
