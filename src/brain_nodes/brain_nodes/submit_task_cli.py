from __future__ import annotations

import argparse

import rclpy
from brain_interfaces.srv import SubmitTask
from rclpy.node import Node

from brain_nodes.constants import SERVICE_SUBMIT_TASK


def main() -> None:
    parser = argparse.ArgumentParser(description="Submit a high-level brain task.")
    parser.add_argument("--instruction", required=True, help="Task instruction text.")
    parser.add_argument("--target-hint", default="", help="Optional semantic target hint.")
    parser.add_argument("--expires-after-sec", type=int, default=600, help="Expiry in seconds.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("submit_task_cli")
    client = node.create_client(SubmitTask, SERVICE_SUBMIT_TASK)
    client.wait_for_service(timeout_sec=5.0)

    request = SubmitTask.Request()
    request.instruction = args.instruction
    request.target_hint = args.target_hint
    request.expires_after_sec = args.expires_after_sec

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is None:
        raise RuntimeError("SubmitTask call failed.")
    print(f"accepted={response.accepted} task_id={response.task_id} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
