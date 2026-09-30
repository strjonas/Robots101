"""Helpers for reading a sensor_msgs/LaserScan by angle instead of by array index.

A LaserScan is a flat list of distances. Entry ``i`` was measured at the angle
``angle_min + i * angle_increment`` (radians, counter-clockwise, 0 = straight
ahead of the lidar). Which index is "front" therefore depends on the sensor:
the TurtleBot3 lidar starts at 0 rad, so index 0 looks forward and the middle
of the list looks backward. Always go through the angles, never assume.
"""

from __future__ import annotations

import math
from typing import Sequence

from brain_nodes.math_utils import finite_min, normalize_angle

FRONT = 0.0
LEFT = math.pi / 2.0
REAR = math.pi
RIGHT = -math.pi / 2.0


def sector_min(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    center_rad: float,
    half_width_rad: float,
    default: float = float("inf"),
) -> float:
    """Closest valid reading within ``center_rad`` +/- ``half_width_rad``."""
    in_sector = [
        distance
        for index, distance in enumerate(ranges)
        if abs(normalize_angle(angle_min + index * angle_increment - center_rad)) <= half_width_rad + 1e-9
    ]
    return finite_min(in_sector, default=default)
