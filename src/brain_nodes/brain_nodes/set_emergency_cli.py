from __future__ import annotations

import argparse

import rclpy
from brain_interfaces.srv import SetEmergencyStop
from rclpy.node import Node

from brain_nodes.constants import SERVICE_SET_EMERGENCY_STOP


def main() -> None:
    parser = argparse.ArgumentParser(description="Enable or release emergency stop.")
    parser.add_argument("--enabled", action="store_true", help="Enable emergency stop.")
    parser.add_argument("--reason", default="", help="Optional reason.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("set_emergency_cli")
    client = node.create_client(SetEmergencyStop, SERVICE_SET_EMERGENCY_STOP)
    client.wait_for_service(timeout_sec=5.0)

    request = SetEmergencyStop.Request()
    request.enabled = args.enabled
    request.reason = args.reason

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is None:
        raise RuntimeError("SetEmergencyStop call failed.")
    print(f"success={response.success} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
