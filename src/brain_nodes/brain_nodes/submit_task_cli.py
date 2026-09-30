"""Send the robot a task: `pixi run submit-task --instruction "turn left" --wait`.

Calls the /brain/submit_task service. With --wait it then stays subscribed and
prints each step as it happens (what the planner chose, how the executor did)
until the task ends, which is the easiest way to watch the brain work.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from brain_interfaces.msg import BrainAction, BrainTask, ExecutorStatus
from brain_interfaces.srv import SubmitTask
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from brain_nodes.cli_utils import call_service
from brain_nodes.constants import (
    ACTION_TYPE_TO_NAME,
    SERVICE_SUBMIT_TASK,
    TOPIC_CURRENT_TASK,
    TOPIC_EXECUTOR_STATUS,
    TOPIC_PLANNER_ACTION,
)

FINAL_STATES = {
    ExecutorStatus.SUCCEEDED: "SUCCEEDED",
    ExecutorStatus.FAILED: "FAILED",
    ExecutorStatus.PREEMPTED: "PREEMPTED",
    ExecutorStatus.REJECTED: "REJECTED",
}


def describe(action: BrainAction) -> str:
    if not action.valid:
        return "INVALID"
    name = ACTION_TYPE_TO_NAME.get(action.action_type, "?")
    detail = {
        BrainAction.TURN: f" {math.degrees(action.angle_rad):+.0f} deg",
        BrainAction.DRIVE: f" {action.distance_m:+.2f} m",
        BrainAction.GOTO_SEMANTIC: f" {action.target_id}",
        BrainAction.FOLLOW_OBJECT: f" {action.follow_label or action.follow_track_id}",
    }.get(action.action_type, "")
    return name + detail


def watch(node: Node, task_id: str, timeout_sec: float) -> None:
    start = time.monotonic()
    state = {"active": True}

    def say(text: str) -> None:
        print(f"[{time.monotonic() - start:6.1f}s] {text}", flush=True)

    def on_action(message: BrainAction) -> None:
        if message.task_id == task_id:
            say(f"plan      {describe(message)}   ({message.rationale})")

    def on_status(message: ExecutorStatus) -> None:
        if message.task_id == task_id and message.state in FINAL_STATES:
            say(f"executor  {FINAL_STATES[message.state]} {message.active_action_type}: {message.detail}")

    def on_task(message: BrainTask) -> None:
        if message.task_id != task_id:
            state["active"] = False

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(BrainAction, TOPIC_PLANNER_ACTION, on_action, 10)
    node.create_subscription(ExecutorStatus, TOPIC_EXECUTOR_STATUS, on_status, 10)
    node.create_subscription(BrainTask, TOPIC_CURRENT_TASK, on_task, latched)

    while state["active"] and time.monotonic() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
    say("task finished" if not state["active"] else f"stopped watching after {timeout_sec:.0f}s, task still running")


def main() -> None:
    parser = argparse.ArgumentParser(description="Submit a high-level brain task.")
    parser.add_argument("--instruction", required=True, help="Task instruction text.")
    parser.add_argument("--target-hint", default="", help="Optional semantic target hint.")
    parser.add_argument("--expires-after-sec", type=int, default=0, help="Expiry in seconds (0 = server default).")
    parser.add_argument("--wait", action="store_true", help="Print the task's progress until it ends.")
    parser.add_argument("--wait-timeout-sec", type=float, default=300.0, help="Give up watching after this long.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("submit_task_cli")
    request = SubmitTask.Request()
    request.instruction = args.instruction
    request.target_hint = args.target_hint
    request.expires_after_sec = args.expires_after_sec
    response = call_service(node, SubmitTask, SERVICE_SUBMIT_TASK, request)
    print(f"accepted={response.accepted} task_id={response.task_id} message={response.message}", flush=True)

    if args.wait and response.accepted:
        try:
            watch(node, response.task_id, args.wait_timeout_sec)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
