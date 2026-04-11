from __future__ import annotations

import rclpy
from brain_interfaces.srv import ClearTask
from rclpy.node import Node

from brain_nodes.constants import SERVICE_CLEAR_TASK


def main() -> None:
    rclpy.init()
    node = Node("clear_task_cli")
    client = node.create_client(ClearTask, SERVICE_CLEAR_TASK)
    client.wait_for_service(timeout_sec=5.0)

    future = client.call_async(ClearTask.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is None:
        raise RuntimeError("ClearTask call failed.")
    print(f"cleared={response.cleared} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
