"""Task server: owns the one task the robot is currently working on.

Services:    /brain/submit_task, /brain/clear_task
Subscribes:  /brain/executor_status
Publishes:   /brain/current_task (latched, so late joiners get it immediately)

A task stays active until the planner answers STOP ("done"), the action budget
is used up, too many actions failed, or it expires. The rules are in task_logic.py.
"""

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
from brain_nodes.task_logic import COMPLETE, FAIL, TaskProgress, update_task_progress


class TaskServerNode(Node):
    def __init__(self) -> None:
        super().__init__("task_server")
        self.declare_parameter("default_expiry_sec", 600)
        self.declare_parameter("max_actions_per_task", 6)
        self.declare_parameter("max_failures_per_task", 3)
        self.default_expiry_sec = int(self.get_parameter("default_expiry_sec").value)
        self.max_actions = int(self.get_parameter("max_actions_per_task").value)
        self.max_failures = int(self.get_parameter("max_failures_per_task").value)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_publisher = self.create_publisher(BrainTask, TOPIC_CURRENT_TASK, qos)
        self.create_subscription(ExecutorStatus, TOPIC_EXECUTOR_STATUS, self._on_executor_status, 10)
        self.create_service(SubmitTask, SERVICE_SUBMIT_TASK, self._submit_task)
        self.create_service(ClearTask, SERVICE_CLEAR_TASK, self._clear_task)
        self.timer = self.create_timer(0.5, self._on_timer)

        self.current_task = BrainTask()
        self.progress = TaskProgress()

    def _submit_task(self, request: SubmitTask.Request, response: SubmitTask.Response) -> SubmitTask.Response:
        instruction = request.instruction.strip()
        if not instruction:
            response.accepted = False
            response.message = "Instruction cannot be empty."
            return response

        expiry_sec = request.expires_after_sec if request.expires_after_sec > 0 else self.default_expiry_sec
        expires_at = datetime.now(timezone.utc) + timedelta(seconds=expiry_sec)

        task = BrainTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.task_id = str(uuid4())
        task.instruction = instruction
        task.target_hint = request.target_hint.strip()
        task.expires_at = Time(sec=int(expires_at.timestamp()))
        task.active = True

        self.current_task = task
        self.progress = TaskProgress()
        self.task_publisher.publish(self.current_task)
        response.accepted = True
        response.task_id = task.task_id
        response.message = "Task accepted."
        self.get_logger().info(f"Accepted task {task.task_id}: {task.instruction}")
        return response

    def _clear_task(self, _: ClearTask.Request, response: ClearTask.Response) -> ClearTask.Response:
        had_task = self.current_task.active
        self._end_task()
        response.cleared = had_task
        response.message = "Task cleared." if had_task else "No task was active."
        return response

    def _on_executor_status(self, message: ExecutorStatus) -> None:
        if not self.current_task.active or message.task_id != self.current_task.task_id:
            return

        outcome = update_task_progress(
            self.progress,
            executor_state=message.state,
            action_name=message.active_action_type,
            max_actions=self.max_actions,
            max_failures=self.max_failures,
        )
        if outcome == COMPLETE:
            self.get_logger().info(
                f"Task {message.task_id} complete after {self.progress.actions_done} action(s): "
                f"{message.active_action_type} - {message.detail}"
            )
            self._end_task()
        elif outcome == FAIL:
            self.get_logger().warning(
                f"Task {message.task_id} given up after {self.progress.failures} failed attempt(s): {message.detail}"
            )
            self._end_task()

    def _on_timer(self) -> None:
        if self.current_task.active:
            expires_at = self.current_task.expires_at.sec
            if expires_at and datetime.now(timezone.utc).timestamp() > expires_at:
                self.get_logger().warning(f"Task {self.current_task.task_id} expired")
                self._end_task()
                return
            self.current_task.header.stamp = self.get_clock().now().to_msg()
        self.task_publisher.publish(self.current_task)

    def _end_task(self) -> None:
        self.current_task = BrainTask()
        self.progress = TaskProgress()
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
