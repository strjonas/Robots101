from brain_nodes.action_schema import ActionProposal


def test_turn_requires_angle() -> None:
    try:
        ActionProposal(action="TURN", angle_rad=0.0, confidence=0.5)
    except Exception as exc:  # noqa: BLE001
        assert "angle_rad" in str(exc)
    else:
        raise AssertionError("TURN without angle should fail validation")


def test_follow_requires_label_or_track() -> None:
    try:
        ActionProposal(action="FOLLOW_OBJECT", confidence=0.5)
    except Exception as exc:  # noqa: BLE001
        assert "FOLLOW_OBJECT" in str(exc)
    else:
        raise AssertionError("FOLLOW_OBJECT without target should fail validation")


def test_goto_semantic_is_valid() -> None:
    proposal = ActionProposal(action="GOTO_SEMANTIC", target_id="doorway", confidence=0.8)
    assert proposal.target_id == "doorway"

