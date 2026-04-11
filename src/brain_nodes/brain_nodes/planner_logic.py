from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PlannerRequestContext:
    task_id: str
    instruction: str
    target_hint: str
    dispatched_at_ns: int


def is_planner_result_current(
    *,
    task_active: bool,
    current_task_id: str,
    current_mode: int,
    brain_task_mode: int,
    context: PlannerRequestContext,
) -> bool:
    return bool(task_active and current_task_id and current_task_id == context.task_id and current_mode == brain_task_mode)
