"""When is a task finished? Pure rules used by the task server.

A task ("go to the kitchen, then turn around") is carried out as a sequence of
actions. After each action the executor reports how it went and the task server
decides whether the task goes on, is complete, or has failed.
"""

from __future__ import annotations

from dataclasses import dataclass

EXECUTOR_SUCCEEDED = 2
EXECUTOR_FAILED = 3
EXECUTOR_REJECTED = 5

CONTINUE = "continue"
COMPLETE = "complete"
FAIL = "fail"


@dataclass
class TaskProgress:
    actions_done: int = 0
    failures: int = 0


def update_task_progress(
    progress: TaskProgress,
    executor_state: int,
    action_name: str,
    max_actions: int,
    max_failures: int,
) -> str:
    """Record one finished action and return CONTINUE, COMPLETE or FAIL."""
    if executor_state == EXECUTOR_SUCCEEDED:
        # STOP is how the planner says "nothing left to do".
        if action_name == "STOP":
            return COMPLETE
        progress.actions_done += 1
        return COMPLETE if progress.actions_done >= max_actions else CONTINUE

    if executor_state in (EXECUTOR_FAILED, EXECUTOR_REJECTED):
        progress.failures += 1
        return FAIL if progress.failures >= max_failures else CONTINUE

    # RUNNING, IDLE and PREEMPTED (e.g. a manual override) do not end a task.
    return CONTINUE
