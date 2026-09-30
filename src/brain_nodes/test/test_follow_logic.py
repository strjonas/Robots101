import math

import pytest

from brain_nodes.follow_logic import follow_command, search_direction


def command(bearing: float, distance: float, obstacle: bool = False) -> tuple[float, float]:
    return follow_command(bearing, distance, preferred_distance_m=1.0, obstacle_ahead=obstacle, max_linear_mps=0.2, max_angular_rps=0.9)


def test_target_straight_ahead_at_the_right_distance_means_stand_still() -> None:
    assert command(0.0, 1.0) == pytest.approx((0.0, 0.0))


def test_far_target_ahead_drives_forward_at_most_max_speed() -> None:
    linear, angular = command(0.0, 5.0)
    assert linear == pytest.approx(0.2) and angular == pytest.approx(0.0)


def test_target_on_the_left_turns_left_and_slows_down() -> None:
    linear, angular = command(math.radians(45), 3.0)
    assert angular > 0.0
    assert 0.0 < linear < 0.2


def test_target_beside_the_robot_turns_without_driving() -> None:
    linear, angular = command(math.radians(-100), 3.0)
    assert linear == 0.0 and angular < 0.0


def test_too_close_backs_off_slowly() -> None:
    linear, _ = command(0.0, 0.3)
    assert linear == pytest.approx(-0.1)


def test_obstacle_blocks_forward_but_not_turning() -> None:
    linear, angular = command(0.3, 3.0, obstacle=True)
    assert linear == 0.0 and angular > 0.0


def test_search_turns_towards_where_the_target_was_last_seen() -> None:
    assert search_direction(None) == 1.0
    assert search_direction(0.4) == 1.0
    assert search_direction(-0.4) == -1.0
