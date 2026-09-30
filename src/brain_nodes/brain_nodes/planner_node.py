"""Planner: decides the next action for the active task.

Subscribes:  /brain/current_task, /brain/control_mode, /brain/observation_summary,
             /brain/executor_status, /camera/image_raw
Publishes:   /brain/planner_action (one validated action at a time)

The loop is: ask the backend for one action -> publish it -> wait until the
executor reports a final status -> remember the outcome in `history` -> ask
again. The task server ends the task when the backend answers STOP.

The backend is swappable (`backend` parameter): "ollama" asks a local LLM,
"rules" uses keyword matching and needs no model. Inference can take seconds,
so it runs on a worker thread; the ROS callbacks keep running meanwhile and a
result that arrives after the task changed is thrown away.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import math
from pathlib import Path
from uuid import uuid4

import cv2
from cv_bridge import CvBridge
import rclpy
from brain_interfaces.msg import BrainAction, BrainTask, ControlMode, ExecutorStatus, ObservationSummary
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image

from brain_nodes.action_schema import ActionProposal, ParsedModelResult
from brain_nodes.constants import (
    ACTION_NAME_TO_TYPE,
    DEFAULT_PROMPT_PATH,
    DEFAULT_SEMANTIC_MAP_PATH,
    TOPIC_CONTROL_MODE,
    TOPIC_CURRENT_TASK,
    TOPIC_EXECUTOR_STATUS,
    TOPIC_OBSERVATION_SUMMARY,
    TOPIC_PLANNER_ACTION,
)
from brain_nodes.model_clients.base import ModelClient, ModelRequest
from brain_nodes.model_clients.ollama_client import OllamaClient
from brain_nodes.model_clients.rules_client import RulesClient
from brain_nodes.planner_logic import PlannerRequestContext, is_planner_result_current
from brain_nodes.semantic_map import SemanticMap

FINAL_STATE_NAMES = {
    ExecutorStatus.SUCCEEDED: "SUCCEEDED",
    ExecutorStatus.FAILED: "FAILED",
    ExecutorStatus.PREEMPTED: "PREEMPTED",
    ExecutorStatus.REJECTED: "REJECTED",
}
MAX_HISTORY_ENTRIES = 10


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")
        self.declare_parameter("backend", "ollama")
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter("model_name", "gemma4:e4b")
        self.declare_parameter("prompt_path", str(DEFAULT_PROMPT_PATH))
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.declare_parameter("planning_hz", 1.0)
        self.declare_parameter("min_plan_interval_sec", 2.0)
        self.declare_parameter("image_topic", "/camera/image_raw")

        self.bridge = CvBridge()
        self.semantic_map = SemanticMap.load(Path(str(self.get_parameter("semantic_map_path").value)))
        self.min_plan_interval_sec = float(self.get_parameter("min_plan_interval_sec").value)
        self.image_topic = str(self.get_parameter("image_topic").value)

        backend = str(self.get_parameter("backend").value)
        self.client: ModelClient
        if backend == "rules":
            self.client = RulesClient()
        elif backend == "ollama":
            self.client = OllamaClient(
                model_name=str(self.get_parameter("model_name").value),
                ollama_url=str(self.get_parameter("ollama_url").value),
                system_prompt_path=str(self.get_parameter("prompt_path").value),
            )
        else:
            raise ValueError(f"Unknown planner backend '{backend}' (use 'ollama' or 'rules')")
        self.needs_image = backend == "ollama"
        self.get_logger().info(f"Planner backend -> {backend}")
        if isinstance(self.client, OllamaClient):
            problem = self.client.check()
            if problem:
                self.get_logger().error(f"{problem} Tasks will fail until this is fixed (or use planner_backend:=rules).")

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(BrainAction, TOPIC_PLANNER_ACTION, 10)
        self.create_subscription(BrainTask, TOPIC_CURRENT_TASK, self._task_cb, transient_qos)
        self.create_subscription(ControlMode, TOPIC_CONTROL_MODE, self._mode_cb, transient_qos)
        self.create_subscription(ObservationSummary, TOPIC_OBSERVATION_SUMMARY, self._summary_cb, 10)
        self.create_subscription(ExecutorStatus, TOPIC_EXECUTOR_STATUS, self._executor_cb, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        self.current_task = BrainTask()
        self.current_mode = ControlMode()
        self.current_mode.mode = ControlMode.IDLE
        self.current_summary = None
        self.current_image = None
        self.executor_status = ExecutorStatus()
        self.last_plan_ns = 0

        # The action we published and are waiting on, and what happened so far in this task.
        self._awaiting_task_id = ""
        self._awaiting_action: dict = {}
        self._history_task_id = ""
        self._history: list[dict] = []

        self._planner_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="planner_infer")
        self._pending_future: Future[ParsedModelResult] | None = None
        self._pending_context: PlannerRequestContext | None = None

        period = 1.0 / max(0.1, float(self.get_parameter("planning_hz").value))
        self.create_timer(period, self._planning_tick)

    def _task_cb(self, message: BrainTask) -> None:
        self.current_task = message
        if message.task_id != self._history_task_id:
            self._history_task_id = message.task_id
            self._history = []
            self._awaiting_task_id = ""

    def _mode_cb(self, message: ControlMode) -> None:
        self.current_mode = message

    def _summary_cb(self, message: ObservationSummary) -> None:
        self.current_summary = message

    def _executor_cb(self, message: ExecutorStatus) -> None:
        self.executor_status = message
        if not message.task_id or message.task_id != self._awaiting_task_id:
            return
        if message.state not in FINAL_STATE_NAMES:
            return
        # The action we were waiting on is over: remember how it went, then plan again.
        self._history.append(
            {**self._awaiting_action, "result": FINAL_STATE_NAMES[message.state], "detail": message.detail}
        )
        self._history = self._history[-MAX_HISTORY_ENTRIES:]
        self._awaiting_task_id = ""

    def _image_cb(self, message: Image) -> None:
        self.current_image = message

    def _planning_tick(self) -> None:
        self._publish_pending_result()
        if self._pending_future is not None:
            return
        if not self.current_task.active:
            return
        if self.current_mode.mode != ControlMode.BRAIN_TASK:
            return
        if self.current_summary is None or (self.needs_image and self.current_image is None):
            return
        if self._awaiting_task_id == self.current_task.task_id:
            return
        if self.executor_status.state == ExecutorStatus.RUNNING:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.last_plan_ns) / 1e9 < self.min_plan_interval_sec:
            return

        image_bytes = None
        if self.needs_image:
            image_cv = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
            _, encoded = cv2.imencode(".jpg", image_cv)
            image_bytes = encoded.tobytes()

        context = PlannerRequestContext(
            task_id=self.current_task.task_id,
            instruction=self.current_task.instruction,
            target_hint=self.current_task.target_hint,
            dispatched_at_ns=now_ns,
        )
        request = ModelRequest(
            task_id=context.task_id,
            instruction=context.instruction,
            target_hint=context.target_hint,
            mode_name="BRAIN_TASK",
            observation=self._observation_dict(),
            semantic_targets=self.semantic_map.describe_targets(),
            image_bytes=image_bytes,
            history=list(self._history),
        )
        self.last_plan_ns = now_ns
        self._pending_context = context
        self._pending_future = self._planner_pool.submit(self.client.infer, request)

    def _observation_dict(self) -> dict:
        summary = self.current_summary
        return {
            "pose_x": summary.pose_x,
            "pose_y": summary.pose_y,
            "pose_yaw": summary.pose_yaw,
            "semantic_location": summary.semantic_location,
            "localization_ok": summary.localization_ok,
            "obstacle_close": summary.obstacle_close,
            "collision_imminent": summary.collision_imminent,
            "ranges": {
                "front": summary.front_min_range,
                "left": summary.left_min_range,
                "right": summary.right_min_range,
                "rear": summary.rear_min_range,
            },
            "tracked_objects": [
                {
                    "track_id": obj.track_id,
                    "label": obj.label,
                    "confidence": obj.confidence,
                    "center_x": obj.center_x,
                    "center_y": obj.center_y,
                    "width": obj.width,
                    "height": obj.height,
                    "bearing_deg": round(math.degrees(obj.bearing_rad)),
                    "estimated_distance_m": round(obj.estimated_distance_m, 2),
                }
                for obj in summary.tracked_objects
            ],
        }

    def _publish_pending_result(self) -> None:
        if self._pending_future is None or not self._pending_future.done():
            return

        future = self._pending_future
        context = self._pending_context
        self._pending_future = None
        self._pending_context = None
        if context is None:
            return

        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            result = ParsedModelResult(error=str(exc))

        if not is_planner_result_current(
            task_active=self.current_task.active,
            current_task_id=self.current_task.task_id,
            current_mode=self.current_mode.mode,
            brain_task_mode=ControlMode.BRAIN_TASK,
            context=context,
        ):
            self.get_logger().info(f"Dropping stale planner result for task {context.task_id}")
            return

        if result.proposal is None:
            # Publish it as an invalid action. The executor rejects it, the task server
            # counts the failure, and the task is given up after a few of them.
            error = result.error or "unknown"
            self.get_logger().warning(f"Planner backend gave no usable action: {error} {result.raw_content}")
            message = self._proposal_to_action(ActionProposal(action="STOP", rationale=error), context.task_id)
            message.valid = False
            summary = {"action": "INVALID_OUTPUT"}
        else:
            message = self._proposal_to_action(result.proposal, context.task_id)
            summary = self._summarize(result.proposal)

        self._awaiting_task_id = message.task_id
        self._awaiting_action = summary
        self.publisher.publish(message)
        self.get_logger().info(f"Planner action -> {summary} task={message.task_id} rationale={message.rationale}")

    @staticmethod
    def _summarize(proposal: ActionProposal) -> dict:
        """The part of an action worth showing in the history: its name and the fields it uses."""
        fields = {
            "TURN": ("angle_rad",),
            "DRIVE": ("distance_m",),
            "GOTO_SEMANTIC": ("target_id",),
            "FOLLOW_OBJECT": ("follow_label", "follow_track_id"),
        }.get(proposal.action, ())
        return {"action": proposal.action, **{name: getattr(proposal, name) for name in fields}}

    def _proposal_to_action(self, proposal: ActionProposal, task_id: str) -> BrainAction:
        message = BrainAction()
        message.header.stamp = self.get_clock().now().to_msg()
        message.task_id = task_id
        message.correlation_id = uuid4().hex
        message.action_type = ACTION_NAME_TO_TYPE[proposal.action]
        message.angle_rad = proposal.angle_rad
        message.distance_m = proposal.distance_m
        message.speed_limit = proposal.speed_limit
        message.target_id = proposal.target_id
        message.follow_label = proposal.follow_label
        message.follow_track_id = proposal.follow_track_id
        message.preferred_distance_m = proposal.preferred_distance_m
        message.confidence = proposal.confidence
        message.rationale = proposal.rationale
        message.valid = True
        return message

    def destroy_node(self) -> bool:
        if self._pending_future is not None:
            self._pending_future.cancel()
        self._planner_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
