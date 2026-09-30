"""The follow controller: from "where is the target" to a velocity command. Pure, so it can be tested.

It is two proportional controllers side by side:
  - turn toward the target, faster the further off-centre it is;
  - drive to keep a set distance, faster the further off that distance it is,
    and slower while the target is still far to the side (turn first, then go).
"""

from __future__ import annotations

import math

from brain_nodes.math_utils import clamp

TURN_GAIN = 1.5  # rad/s of turning per rad of bearing error
DISTANCE_GAIN = 0.6  # m/s of driving per metre of distance error
MAX_REVERSE_MPS = 0.1


def follow_command(
    bearing_rad: float,
    distance_m: float,
    preferred_distance_m: float,
    obstacle_ahead: bool,
    max_linear_mps: float,
    max_angular_rps: float,
) -> tuple[float, float]:
    """Return (linear m/s, angular rad/s) that keeps the target ahead at the preferred distance."""
    angular = clamp(TURN_GAIN * bearing_rad, -max_angular_rps, max_angular_rps)

    distance_error = distance_m - preferred_distance_m
    linear = clamp(DISTANCE_GAIN * distance_error, -MAX_REVERSE_MPS, max_linear_mps)
    # Only drive the component of speed that points at the target: full speed when
    # it is straight ahead, none when it is 90 degrees or more to the side.
    linear *= max(0.0, math.cos(bearing_rad))
    if obstacle_ahead and linear > 0.0:
        linear = 0.0
    return linear, angular


def search_direction(last_bearing_rad: float | None) -> float:
    """Which way to spin when the target is out of sight: towards where it was last seen, else left."""
    if last_bearing_rad is None or last_bearing_rad == 0.0:
        return 1.0
    return 1.0 if last_bearing_rad > 0.0 else -1.0
