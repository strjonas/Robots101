# Next Developer Handoff

This document is for the next engineer who picks up the repository.

It is intentionally factual. If something is not known, it is marked as not known.

## Executive Summary

The project is no longer just a scaffold.

During the implementation session, the following were directly validated on the target Mac:

- native `Pixi` + `RoboStack` + ROS 2 Jazzy + Gazebo Harmonic setup
- TurtleBot3 house simulation with camera, lidar, odometry, TF, AMCL, and Nav2
- semantic patrol in the generated apartment map
- manual override through the command arbiter
- local Gemma planning through Ollama for:
  - `turn left`
  - `drive to doorway`

The biggest remaining functional gap is:

- reliable end-to-end `FOLLOW_OBJECT` validation in a scene with live detections

The biggest remaining platform risk is:

- ongoing macOS Gazebo friction, especially anything GUI- or rendering-adjacent

## What Was Verified

Verified by direct runs:

- `pixi run build`
- `pixi run test`
- `pixi run nav`
- `pixi run brain`
- semantic patrol across the configured waypoints in `semantic_map.yaml`
- task ingress through `SubmitTask`
- planner output from Gemma via Ollama
- `TURN` execution
- `GOTO_SEMANTIC` execution through Nav2
- auto-completion of successful tasks
- manual preemption of patrol

Observed behavior that I am confident about:

- the current architecture boundary is real:
  `sensors -> perception/state -> planner/policy -> skill executor/controller -> command arbiter -> /cmd_vel`
- the LLM is not writing raw `/cmd_vel`
- the semantic map layer is real and currently used
- the controller and arbitration layers are meaningful, not fake placeholders

## What Was Implemented But Not Fully Validated

- `FOLLOW_OBJECT` exists in the action schema, planner contract, and executor
- `object_tracker_node` publishes tracked objects if YOLO sees something recognizable
- the executor has a follow loop using tracked detections and lidar safety constraints

What is still missing:

- a reliable live scenario in the house world where the tracker actually sees an object and keeps seeing it long enough to validate follow end to end

## What Was Not Re-Verified In The Final Pass

- the Gazebo GUI path with `with_gui:=true`
- the mapping workflow after the latest planner/executor fixes
- emergency stop in a dedicated end-to-end live run
- RViz, because it is not wired into the current launch files

Do not assume those are broken.
Do not assume they are working either.

## Critical Bugs That Were Found And Fixed

### 1. Nav commands were not reaching Gazebo

Root cause:

- Nav2 was publishing stamped velocity commands
- the arbiter was originally consuming nav commands as plain `Twist`
- Gazebo ultimately needed the stamped form on `/cmd_vel`

Fix:

- `cmd_arbiter_node` now subscribes to `/brain/cmd_vel_nav` as `TwistStamped`
- `cmd_vel_stamper_node` republishes the selected final `Twist` as stamped `/cmd_vel`

Why it matters:

- before this, patrol and Nav2 looked alive in logs but did not reliably move the robot

### 2. Planner could publish stale actions after a task had already ended

Root cause:

- model inference was synchronous in the planner timer callback
- while the planner was blocked on Ollama, it was not processing task-clear or control-mode updates

Fix:

- planner inference now runs off-thread
- planner results are checked against current task/mode before publish
- stale results are dropped

Relevant files:

- [planner_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/planner_node.py)
- [planner_logic.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/planner_logic.py)

### 3. Long-running semantic navigation was being re-planned mid-flight

Root cause:

- the planner relied too much on a single executor status sample
- `GOTO_SEMANTIC` execution did not continuously heartbeat `RUNNING`

Fix:

- the planner tracks an in-flight task id and suppresses new plans for that task until a terminal executor state arrives
- the executor now continues to publish `RUNNING` during `GOTO_SEMANTIC`

Relevant file:

- [skill_executor_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/skill_executor_node.py)

### 4. Localization needed a deterministic initial pose

Root cause:

- patrol and Nav2 are much more stable once AMCL is seeded

Fix:

- `localization_seed_node` publishes an initial pose from the semantic map until localization comes online

## Key Files To Understand First

### Launch and runtime wiring

- [sim_only.launch.py](/Users/jonas/RLKitchen/robots101/src/brain_bringup/launch/sim_only.launch.py)
- [nav_only.launch.py](/Users/jonas/RLKitchen/robots101/src/brain_bringup/launch/nav_only.launch.py)
- [brain_only.launch.py](/Users/jonas/RLKitchen/robots101/src/brain_bringup/launch/brain_only.launch.py)
- [full_stack.launch.py](/Users/jonas/RLKitchen/robots101/src/brain_bringup/launch/full_stack.launch.py)
- [runtime.py](/Users/jonas/RLKitchen/robots101/src/brain_bringup/brain_bringup/runtime.py)

### Interface contract

- [BrainAction.msg](/Users/jonas/RLKitchen/robots101/src/brain_interfaces/msg/BrainAction.msg)
- [BrainTask.msg](/Users/jonas/RLKitchen/robots101/src/brain_interfaces/msg/BrainTask.msg)
- [ControlMode.msg](/Users/jonas/RLKitchen/robots101/src/brain_interfaces/msg/ControlMode.msg)
- [ExecutorStatus.msg](/Users/jonas/RLKitchen/robots101/src/brain_interfaces/msg/ExecutorStatus.msg)
- [ObservationSummary.msg](/Users/jonas/RLKitchen/robots101/src/brain_interfaces/msg/ObservationSummary.msg)

### Control path

- [control_supervisor_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/control_supervisor_node.py)
- [cmd_arbiter_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/cmd_arbiter_node.py)
- [cmd_vel_stamper_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/cmd_vel_stamper_node.py)
- [skill_executor_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/skill_executor_node.py)

### Brain path

- [task_server_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/task_server_node.py)
- [planner_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/planner_node.py)
- [action_schema.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/action_schema.py)
- [ollama_client.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/model_clients/ollama_client.py)
- [gemma_system_prompt.md](/Users/jonas/RLKitchen/robots101/src/brain_bringup/config/gemma_system_prompt.md)

### Perception and state

- [observation_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/observation_node.py)
- [object_tracker_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/object_tracker_node.py)
- [semantic_map.yaml](/Users/jonas/RLKitchen/robots101/src/brain_bringup/config/semantic_map.yaml)
- [generate_house_map.py](/Users/jonas/RLKitchen/robots101/scripts/generate_house_map.py)

## What Is Ours Versus What Comes From Dependencies

Ours:

- everything in `src/brain_nodes`
- everything in `src/brain_bringup`
- the custom message/service definitions in `src/brain_interfaces`
- the helper scripts in `scripts`
- the semantic map config
- the generated house-map script and resulting map files
- the documentation in `docs`

Dependency-owned behavior:

- ROS 2 core runtime
- Gazebo Harmonic simulator
- TurtleBot3 robot/world packages
- Nav2, AMCL, and SLAM Toolbox
- `teleop_twist_keyboard`
- `ros_gz` simulation bridge pieces
- Ollama server/runtime
- Gemma model weights and inference behavior
- Ultralytics YOLO detection model behavior

Generated artifacts:

- `build/`
- `install/`
- `.ros/`
- generated Python/C/C++ interface code from the message and service definitions

## Realistic Remaining Work

If the goal is “solid simulated demo on this Mac”, I think that is realistic.

Why I think it is realistic:

- the core motion/control loop is already real
- patrol is already real
- Gemma tasking is already real for at least two representative task types
- the project no longer depends on pretending that planning is working

Main risks:

- Gazebo on macOS is still a best-effort path, especially with GUI rendering
- follow-object needs more scene engineering and probably better detection ergonomics
- there is still limited automated integration testing

If the goal is “smooth, low-friction simulator development platform for a team”, Linux is still the lower-risk choice.

## Recommended Next Steps

1. Validate the GUI path again and decide whether Gazebo-on-macOS is good enough for the team.
2. Set up a repeatable follow-object test scene with a visible YOLO-recognizable object.
3. Add more narrow unit tests for planner/executor/control invariants.
4. Decide whether to keep using Ollama as the main backend or add an MLX backend for local Apple Silicon inference.
5. Consider adding an RViz launch and a small debug dashboard if observability becomes painful.

## Things I Would Not Change First

- do not remove the controller/arbiter boundary just to make the LLM “more end to end”
- do not wire the model directly to `/cmd_vel`
- do not delete the semantic map layer before a stronger scene-grounding replacement exists

## One-Sentence Assessment

This is already a real robotics prototype, not a mockup, but the remaining work is still mostly in simulator/platform polish and richer task validation, not in inventing the architecture from scratch.
