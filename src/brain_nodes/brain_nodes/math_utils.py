from __future__ import annotations

import math
from typing import Iterable


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite_min(values: Iterable[float], default: float = float("inf")) -> float:
    finite = [value for value in values if math.isfinite(value) and value > 0.0]
    if not finite:
        return default
    return min(finite)

