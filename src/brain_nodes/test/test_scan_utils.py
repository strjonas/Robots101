import math

from brain_nodes.scan_utils import FRONT, LEFT, REAR, RIGHT, sector_min

DEGREE = math.radians(1.0)


def turtlebot_scan(overrides: dict[int, float]) -> list[float]:
    """360 readings, one per degree, starting straight ahead like the TurtleBot3 lidar."""
    ranges = [3.0] * 360
    for index, value in overrides.items():
        ranges[index] = value
    return ranges


def test_front_is_index_zero_when_scan_starts_at_zero() -> None:
    ranges = turtlebot_scan({0: 0.2, 180: 0.5})
    assert sector_min(ranges, 0.0, DEGREE, FRONT, math.radians(20)) == 0.2
    assert sector_min(ranges, 0.0, DEGREE, REAR, math.radians(20)) == 0.5


def test_front_sector_wraps_around_the_end_of_the_array() -> None:
    ranges = turtlebot_scan({350: 0.3})  # 10 degrees to the right of straight ahead
    assert sector_min(ranges, 0.0, DEGREE, FRONT, math.radians(20)) == 0.3
    assert sector_min(ranges, 0.0, DEGREE, FRONT, math.radians(5)) == 3.0


def test_left_and_right() -> None:
    ranges = turtlebot_scan({90: 0.4, 270: 0.6})
    assert sector_min(ranges, 0.0, DEGREE, LEFT, math.radians(45)) == 0.4
    assert sector_min(ranges, 0.0, DEGREE, RIGHT, math.radians(45)) == 0.6


def test_front_is_the_middle_when_scan_starts_at_minus_pi() -> None:
    ranges = turtlebot_scan({180: 0.25})
    assert sector_min(ranges, -math.pi, DEGREE, FRONT, math.radians(20)) == 0.25


def test_invalid_readings_are_ignored() -> None:
    ranges = turtlebot_scan({0: float("inf"), 1: float("nan"), 2: 0.0})
    assert sector_min(ranges, 0.0, DEGREE, FRONT, math.radians(3)) == 3.0
    assert sector_min([], 0.0, DEGREE, FRONT, math.radians(20)) == float("inf")
