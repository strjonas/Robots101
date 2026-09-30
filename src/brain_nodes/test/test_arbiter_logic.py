from brain_nodes.arbiter_logic import (
    MODE_BRAIN_TASK,
    MODE_EMERGENCY_STOP,
    MODE_IDLE,
    MODE_MANUAL,
    MODE_PATROL,
    limit_forward_speed,
    select_source,
)


def test_emergency_stop_beats_everything() -> None:
    assert select_source(MODE_EMERGENCY_STOP, True, True, True) == "emergency_stop"


def test_manual_mode_only_listens_to_the_keyboard() -> None:
    assert select_source(MODE_MANUAL, True, True, True) == "manual"
    assert select_source(MODE_MANUAL, False, True, True) == "zero"


def test_brain_task_prefers_executor_then_nav() -> None:
    assert select_source(MODE_BRAIN_TASK, False, True, True) == "executor"
    assert select_source(MODE_BRAIN_TASK, False, False, True) == "nav"
    assert select_source(MODE_BRAIN_TASK, True, False, False) == "zero"


def test_patrol_only_listens_to_nav() -> None:
    assert select_source(MODE_PATROL, True, True, True) == "nav"
    assert select_source(MODE_PATROL, True, True, False) == "zero"


def test_idle_never_moves() -> None:
    assert select_source(MODE_IDLE, True, True, True) == "zero"


def test_obstacle_blocks_forward_but_not_reverse() -> None:
    assert limit_forward_speed(0.2, obstacle_ahead=True) == 0.0
    assert limit_forward_speed(-0.1, obstacle_ahead=True) == -0.1
    assert limit_forward_speed(0.2, obstacle_ahead=False) == 0.2
