from __future__ import annotations

import argparse

import rclpy
from brain_interfaces.srv import RecordSemanticTarget
from rclpy.node import Node

from brain_nodes.constants import SERVICE_RECORD_SEMANTIC_TARGET


def main() -> None:
    parser = argparse.ArgumentParser(description="Save the robot's current pose into semantic_map.yaml.")
    parser.add_argument("--name", required=True, help="Semantic target name.")
    parser.add_argument("--tag", default="patrol", help="Tag to attach to the target.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("record_waypoint_cli")
    client = node.create_client(RecordSemanticTarget, SERVICE_RECORD_SEMANTIC_TARGET)
    client.wait_for_service(timeout_sec=5.0)

    request = RecordSemanticTarget.Request()
    request.name = args.name
    request.tag = args.tag

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is None:
        raise RuntimeError("RecordSemanticTarget call failed.")
    print(f"saved={response.saved} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
