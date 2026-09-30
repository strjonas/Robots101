"""The arbiter's decision rules, kept free of ROS so they are easy to read and test.

Several nodes want to drive the robot: the keyboard (manual), the skill
executor (brain tasks) and Nav2. Only one may win at a time. The control mode
says who is allowed; "fresh" means that source sent a command recently enough
to still be trusted.
"""

from __future__ import annotations

MODE_MANUAL = 0
MODE_PATROL = 1
MODE_BRAIN_TASK = 2
MODE_IDLE = 3
MODE_EMERGENCY_STOP = 4

SOURCE_ZERO = "zero"
SOURCE_MANUAL = "manual"
SOURCE_EXECUTOR = "executor"
SOURCE_NAV = "nav"
SOURCE_EMERGENCY = "emergency_stop"


def select_source(mode: int, manual_fresh: bool, executor_fresh: bool, nav_fresh: bool) -> str:
    """Pick whose velocity command is forwarded to the robot."""
    if mode == MODE_EMERGENCY_STOP:
        return SOURCE_EMERGENCY
    if mode == MODE_MANUAL:
        return SOURCE_MANUAL if manual_fresh else SOURCE_ZERO
    if mode == MODE_BRAIN_TASK:
        # GOTO_SEMANTIC is carried out by Nav2, the other skills by the executor itself.
        if executor_fresh:
            return SOURCE_EXECUTOR
        return SOURCE_NAV if nav_fresh else SOURCE_ZERO
    if mode == MODE_PATROL:
        return SOURCE_NAV if nav_fresh else SOURCE_ZERO
    return SOURCE_ZERO


def limit_forward_speed(linear_x: float, obstacle_ahead: bool) -> float:
    """Safety stop: refuse to drive forward into an obstacle.

    Turning and reversing stay allowed, otherwise the robot could never get
    away from the thing it stopped for.
    """
    if obstacle_ahead and linear_x > 0.0:
        return 0.0
    return linear_x
