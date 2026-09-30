"""A planner backend with no language model: plain keyword rules.

Useful for learning and debugging because it is instant and always gives the
same answer, so you can exercise the whole task -> plan -> execute pipeline
without Ollama. Select it with `planner_backend:=rules`.

Understands, optionally chained with "then":
    turn left | turn right | turn around | turn left 45 degrees
    drive forward 1 m | back up 0.5 m
    go to <place>            (a name or tag from the semantic map)
    look around
    follow the <label>
    stop
"""

from __future__ import annotations

import math
import re
from typing import Any

from brain_nodes.action_schema import ActionProposal, ParsedModelResult
from brain_nodes.model_clients.base import ModelRequest

_NUMBER = r"(\d+(?:\.\d+)?)"


def split_steps(instruction: str) -> list[str]:
    parts = re.split(r"\s*(?:,|\band)?\s*\bthen\b\s*", instruction.strip().lower())
    return [part.strip(" .,") for part in parts if part.strip(" .,")]


def find_target(text: str, semantic_targets: list[dict[str, Any]]) -> str:
    """Match free text against target names first, then against their tags."""
    words = set(re.findall(r"[a-z0-9]+", text.replace("_", " ")))
    for target in semantic_targets:
        if target["name"] in text or set(target["name"].split("_")) <= words:
            return target["name"]
    for target in semantic_targets:
        if any(tag.replace("_", " ") in text.replace("_", " ") for tag in target.get("tags", [])):
            return target["name"]
    return ""


def parse_step(step: str, semantic_targets: list[dict[str, Any]]) -> ActionProposal:
    """Turn one phrase into an action. Raises ValueError if it is not understood."""
    rationale = f"rule matched: '{step}'"

    if step in ("stop", "halt", "wait"):
        return ActionProposal(action="STOP", confidence=1.0, rationale=rationale)

    if "look around" in step:
        return ActionProposal(action="LOOK_AROUND", confidence=1.0, rationale=rationale)

    if step.startswith("turn") or step.startswith("rotate"):
        if "around" in step:
            angle = math.pi
        else:
            degrees = re.search(_NUMBER + r"\s*(?:deg|degrees|°)", step)
            angle = math.radians(float(degrees.group(1))) if degrees else math.pi / 2.0
        if "right" in step:
            angle = -angle
        return ActionProposal(action="TURN", angle_rad=angle, confidence=1.0, rationale=rationale)

    follow = re.match(r"follow (?:the |a )?(.+)", step)
    if follow:
        return ActionProposal(action="FOLLOW_OBJECT", follow_label=follow.group(1), confidence=1.0, rationale=rationale)

    if re.match(r"(go|drive|navigate|move|head) (in)?to\b", step):
        target_id = find_target(step, semantic_targets)
        if not target_id:
            raise ValueError(f"no semantic target matches '{step}'")
        return ActionProposal(action="GOTO_SEMANTIC", target_id=target_id, confidence=1.0, rationale=rationale)

    if re.match(r"(drive|go|move|back|reverse)", step):
        meters = re.search(_NUMBER + r"\s*(?:m|meter|meters|metre|metres)\b", step)
        distance = float(meters.group(1)) if meters else 0.5
        if re.search(r"\b(back|backward|backwards|reverse)\b", step):
            distance = -distance
        return ActionProposal(action="DRIVE", distance_m=distance, confidence=1.0, rationale=rationale)

    raise ValueError(f"no rule understands '{step}'")


class RulesClient:
    def infer(self, request: ModelRequest) -> ParsedModelResult:
        steps = split_steps(request.instruction)
        # Each successful action ticks off one step; a failed one is simply tried again.
        done = sum(1 for entry in request.history if entry.get("result") == "SUCCEEDED")
        if done >= len(steps):
            return ParsedModelResult(proposal=ActionProposal(action="STOP", confidence=1.0, rationale="all steps done"))
        try:
            return ParsedModelResult(proposal=parse_step(steps[done], request.semantic_targets))
        except ValueError as exc:
            return ParsedModelResult(error=str(exc))
