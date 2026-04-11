from brain_nodes.planner_logic import PlannerRequestContext, is_planner_result_current


def test_planner_result_is_current_only_for_matching_brain_task() -> None:
    brain_task_mode = 2
    patrol_mode = 1
    context = PlannerRequestContext(task_id="task-1", instruction="turn left", target_hint="", dispatched_at_ns=123)

    assert is_planner_result_current(
        task_active=True,
        current_task_id="task-1",
        current_mode=brain_task_mode,
        brain_task_mode=brain_task_mode,
        context=context,
    )

    assert not is_planner_result_current(
        task_active=False,
        current_task_id="task-1",
        current_mode=brain_task_mode,
        brain_task_mode=brain_task_mode,
        context=context,
    )

    assert not is_planner_result_current(
        task_active=True,
        current_task_id="task-2",
        current_mode=brain_task_mode,
        brain_task_mode=brain_task_mode,
        context=context,
    )

    assert not is_planner_result_current(
        task_active=True,
        current_task_id="task-1",
        current_mode=patrol_mode,
        brain_task_mode=brain_task_mode,
        context=context,
    )
