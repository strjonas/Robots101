"""Cancel the current task: `pixi run clear-task`. The robot goes back to patrolling."""

from __future__ import annotations

import rclpy
from brain_interfaces.srv import ClearTask
from rclpy.node import Node

from brain_nodes.cli_utils import call_service
from brain_nodes.constants import SERVICE_CLEAR_TASK


def main() -> None:
    rclpy.init()
    node = Node("clear_task_cli")
    response = call_service(node, ClearTask, SERVICE_CLEAR_TASK, ClearTask.Request())
    print(f"cleared={response.cleared} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
