"""What every planner backend looks like to the planner node.

A backend gets one ModelRequest (the task, what the robot currently senses, and
what has been tried so far) and returns one proposed action. Swap the backend
and the rest of the robot does not change.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

from brain_nodes.action_schema import ParsedModelResult


@dataclass
class ModelRequest:
    task_id: str
    instruction: str
    target_hint: str
    mode_name: str
    observation: dict[str, Any]
    semantic_targets: list[dict[str, Any]]
    image_bytes: bytes | None
    # Actions already carried out for this task, oldest first:
    # {"action": "TURN", "result": "SUCCEEDED", "detail": "Turn complete", ...}
    history: list[dict[str, Any]] = field(default_factory=list)


class ModelClient(Protocol):
    def infer(self, request: ModelRequest) -> ParsedModelResult:
        ...
