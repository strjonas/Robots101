from brain_nodes.task_logic import (
    COMPLETE,
    CONTINUE,
    EXECUTOR_FAILED,
    EXECUTOR_REJECTED,
    EXECUTOR_SUCCEEDED,
    FAIL,
    TaskProgress,
    update_task_progress,
)

RUNNING = 1
PREEMPTED = 4


def step(progress: TaskProgress, state: int, action: str) -> str:
    return update_task_progress(progress, state, action, max_actions=3, max_failures=2)


def test_stop_completes_the_task() -> None:
    assert step(TaskProgress(), EXECUTOR_SUCCEEDED, "STOP") == COMPLETE


def test_a_successful_action_keeps_the_task_open_for_the_next_step() -> None:
    progress = TaskProgress()
    assert step(progress, EXECUTOR_SUCCEEDED, "GOTO_SEMANTIC") == CONTINUE
    assert step(progress, EXECUTOR_SUCCEEDED, "TURN") == CONTINUE
    assert progress.actions_done == 2


def test_action_budget_ends_a_task_that_never_says_stop() -> None:
    progress = TaskProgress()
    outcomes = [step(progress, EXECUTOR_SUCCEEDED, "TURN") for _ in range(3)]
    assert outcomes == [CONTINUE, CONTINUE, COMPLETE]


def test_repeated_failures_give_up() -> None:
    progress = TaskProgress()
    assert step(progress, EXECUTOR_FAILED, "DRIVE") == CONTINUE
    assert step(progress, EXECUTOR_REJECTED, "INVALID") == FAIL


def test_heartbeats_and_preemption_change_nothing() -> None:
    progress = TaskProgress()
    assert step(progress, RUNNING, "TURN") == CONTINUE
    assert step(progress, PREEMPTED, "TURN") == CONTINUE
    assert progress == TaskProgress()
