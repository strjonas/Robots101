"""Shared helper for the small command-line tools: call one ROS service and return its answer."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

# DDS discovery can take several seconds on macOS, especially while a launch is still starting.
DISCOVERY_TIMEOUT_SEC = 15.0
CALL_TIMEOUT_SEC = 10.0


def call_service(node: Node, service_type, service_name: str, request):
    """Wait for the service, call it once, and return the response. Exits with a clear message on failure."""
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT_SEC):
        raise SystemExit(
            f"Service {service_name} is not available. Is the robot running? Start it with e.g. `pixi run sim`."
        )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_SEC)
    if future.result() is None:
        raise SystemExit(f"Service {service_name} did not answer within {CALL_TIMEOUT_SEC:.0f} s.")
    return future.result()
