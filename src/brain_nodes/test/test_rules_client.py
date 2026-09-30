import math

import pytest

from brain_nodes.model_clients.base import ModelRequest
from brain_nodes.model_clients.rules_client import RulesClient, parse_step, split_steps

TARGETS = [
    {"name": "hallway_mid", "tags": ["hallway", "patrol"]},
    {"name": "living_room_doorway", "tags": ["doorway", "living_room", "patrol"]},
    {"name": "kitchen_entry", "tags": ["doorway", "kitchen", "patrol"]},
]


def request(instruction: str, history: list[dict] | None = None) -> ModelRequest:
    return ModelRequest(
        task_id="t",
        instruction=instruction,
        target_hint="",
        mode_name="BRAIN_TASK",
        observation={},
        semantic_targets=TARGETS,
        image_bytes=None,
        history=history or [],
    )


def test_turns() -> None:
    assert parse_step("turn left", TARGETS).angle_rad == pytest.approx(math.pi / 2)
    assert parse_step("turn right 45 degrees", TARGETS).angle_rad == pytest.approx(-math.pi / 4)
    assert parse_step("turn around", TARGETS).angle_rad == pytest.approx(math.pi)


def test_drive_distances() -> None:
    assert parse_step("drive forward 1.5 m", TARGETS).distance_m == 1.5
    assert parse_step("back up 0.3 meters", TARGETS).distance_m == -0.3


def test_goto_matches_names_and_tags() -> None:
    assert parse_step("go to hallway_mid", TARGETS).target_id == "hallway_mid"
    assert parse_step("go to the kitchen", TARGETS).target_id == "kitchen_entry"
    assert parse_step("drive to doorway", TARGETS).target_id == "living_room_doorway"
    with pytest.raises(ValueError):
        parse_step("go to the moon", TARGETS)


def test_follow_and_look_around() -> None:
    assert parse_step("follow the person", TARGETS).follow_label == "person"
    assert parse_step("look around", TARGETS).action == "LOOK_AROUND"


def test_steps_are_split_on_then() -> None:
    assert split_steps("Go to the kitchen, then turn around and then stop.") == ["go to the kitchen", "turn around", "stop"]


def test_history_advances_through_the_steps() -> None:
    client = RulesClient()
    instruction = "go to the kitchen then turn left"
    assert client.infer(request(instruction)).proposal.action == "GOTO_SEMANTIC"

    one_done = [{"action": "GOTO_SEMANTIC", "result": "SUCCEEDED"}]
    assert client.infer(request(instruction, one_done)).proposal.action == "TURN"

    failed_turn = one_done + [{"action": "TURN", "result": "FAILED"}]
    assert client.infer(request(instruction, failed_turn)).proposal.action == "TURN"

    all_done = one_done + [{"action": "TURN", "result": "SUCCEEDED"}]
    assert client.infer(request(instruction, all_done)).proposal.action == "STOP"


def test_unknown_instruction_is_an_error_not_an_action() -> None:
    result = RulesClient().infer(request("make me a sandwich"))
    assert result.proposal is None and "sandwich" in result.error
