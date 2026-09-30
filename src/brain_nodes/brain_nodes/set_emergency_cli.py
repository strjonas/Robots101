"""Emergency stop: `pixi run estop --enabled --reason "..."` to stop everything, `pixi run estop` to release."""

from __future__ import annotations

import argparse

import rclpy
from brain_interfaces.srv import SetEmergencyStop
from rclpy.node import Node

from brain_nodes.cli_utils import call_service
from brain_nodes.constants import SERVICE_SET_EMERGENCY_STOP


def main() -> None:
    parser = argparse.ArgumentParser(description="Enable or release emergency stop.")
    parser.add_argument("--enabled", action="store_true", help="Enable emergency stop.")
    parser.add_argument("--reason", default="", help="Optional reason.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("set_emergency_cli")
    request = SetEmergencyStop.Request()
    request.enabled = args.enabled
    request.reason = args.reason

    response = call_service(node, SetEmergencyStop, SERVICE_SET_EMERGENCY_STOP, request)
    print(f"success={response.success} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
