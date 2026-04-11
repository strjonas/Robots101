from __future__ import annotations

from dataclasses import dataclass
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


class ModelClient(Protocol):
    def infer(self, request: ModelRequest) -> ParsedModelResult:
        ...

