"""Save where the robot is now as a named place: `pixi run record-waypoint --name kitchen_table`.

The place is written to src/brain_bringup/config/semantic_map.yaml. With the default tag
"patrol" it is also added to the patrol route (restart the launch to use it)."""

from __future__ import annotations

import argparse

import rclpy
from brain_interfaces.srv import RecordSemanticTarget
from rclpy.node import Node

from brain_nodes.cli_utils import call_service
from brain_nodes.constants import SERVICE_RECORD_SEMANTIC_TARGET


def main() -> None:
    parser = argparse.ArgumentParser(description="Save the robot's current pose into semantic_map.yaml.")
    parser.add_argument("--name", required=True, help="Semantic target name.")
    parser.add_argument("--tag", default="patrol", help="Tag to attach to the target.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("record_waypoint_cli")
    request = RecordSemanticTarget.Request()
    request.name = args.name
    request.tag = args.tag

    response = call_service(node, RecordSemanticTarget, SERVICE_RECORD_SEMANTIC_TARGET, request)
    print(f"saved={response.saved} message={response.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
