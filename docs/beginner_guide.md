# Beginner Guide

This document is for someone who is new to ROS 2 and this repository.

The goal is not to explain every ROS detail.
The goal is to explain what this project is, what each part does, what is ours versus external, and where to start changing things without getting lost.

## What This Project Is

This repository builds a simulated mobile robot system with four important properties:

1. The robot lives in a real simulator, not a toy grid world.
2. The robot has sensors: camera, lidar, odometry, TF, localization.
3. A local LLM can issue high-level actions.
4. The LLM is not allowed to directly drive the motors.

That fourth point is the most important design decision in the repo.

The architecture is intentionally:

`sensors -> perception/state -> planner/policy -> skill executor/controller -> command arbiter -> /cmd_vel`

This means:

- the model decides what to do
- deterministic controllers decide how to do it
- a final arbiter decides which control source is actually allowed to move the robot

## What You Can Do Today

Directly verified in the implementation session:

- run the house simulation
- let the robot patrol by itself
- manually override motion
- submit the task `turn left`
- submit the task `drive to doorway`

Implemented but not yet reliably demonstrated in the default house scene:

- `follow the object`

## The Big Picture

### External pieces

These come from ROS, Gazebo, or other dependencies:

- Gazebo Harmonic: the simulator
- TurtleBot3 packages: the robot model and house world
- ROS 2 Jazzy: the middleware and node graph
- Nav2 + AMCL: navigation and localization
- SLAM Toolbox: map-building flow
- Ollama: local model server
- Gemma: the model itself
- Ultralytics YOLO: object detection

### Our pieces

These are the parts written for this project:

- message and service definitions under `src/brain_interfaces`
- all runtime nodes under `src/brain_nodes`
- launch files and config under `src/brain_bringup`
- helper scripts under `scripts`
- semantic waypoint config
- docs

### Generated pieces

These are produced by the build system:

- `build/`
- `install/`
- generated interface code from `.msg` and `.srv` files
- ROS log/cache directories such as `.ros/`

Do not hand-edit generated output.

## Repo Tour

### `src/brain_interfaces`

This package defines the internal contract between nodes.

Important files:

- `BrainTask.msg`: a human or client asks the robot to do something
- `BrainAction.msg`: a validated structured action chosen by the planner
- `ControlMode.msg`: the current top-level mode, such as `PATROL` or `BRAIN_TASK`
- `ExecutorStatus.msg`: what the active controller/executor is doing
- `ObservationSummary.msg`: the compact robot/world summary sent toward the planner

Why this package matters:

- it makes the system explicit
- it keeps nodes decoupled
- it is the main place where you can see the project’s “language”

### `src/brain_nodes`

This is the actual runtime logic.

Important nodes:

- `task_server_node.py`
  - stores the current task
  - exposes services for submit/clear task
- `control_supervisor_node.py`
  - decides high-level mode
  - modes include `MANUAL`, `PATROL`, `BRAIN_TASK`, `IDLE`, `EMERGENCY_STOP`
- `observation_node.py`
  - builds a structured summary from odometry, AMCL, lidar, and tracked objects
- `object_tracker_node.py`
  - runs YOLO on the camera image and publishes tracked detections
- `planner_node.py`
  - calls the model backend
  - turns observation + task + semantic context into one structured `BrainAction`
- `skill_executor_node.py`
  - turns `TURN`, `DRIVE`, `GOTO_SEMANTIC`, `FOLLOW_OBJECT`, `LOOK_AROUND` into actual execution
- `cmd_arbiter_node.py`
  - chooses which motion source currently wins
- `cmd_vel_stamper_node.py`
  - converts the final `Twist` into the stamped message expected by the current simulation wiring
- `patrol_node.py`
  - submits semantic patrol goals when the robot is in patrol mode
- `safety_monitor_node.py`
  - raises a safety stop from lidar conditions
- `localization_seed_node.py`
  - seeds AMCL from the semantic spawn point

### `src/brain_bringup`

This package wires everything together.

Important files:

- `sim_only.launch.py`
  - starts simulator-side infrastructure and core support nodes
- `nav_only.launch.py`
  - adds localization, Nav2, and patrol
- `brain_only.launch.py`
  - adds object tracking, LLM planner, and executor
- `full_stack.launch.py`
  - combines nav and brain launches
- `config/semantic_map.yaml`
  - named locations and patrol order
- `config/gemma_system_prompt.md`
  - system prompt for the planner backend

### `scripts`

Important scripts:

- `ros_env.sh`
  - activates the workspace environment
  - sets up macOS-specific Gazebo environment details
- `build_workspace.sh`
  - builds the workspace
- `generate_house_map.py`
  - generates a Nav2 occupancy map from the TurtleBot3 house SDF

## How The System Works

### Step 1: the simulator publishes robot state

Gazebo and the TurtleBot3 packages provide:

- camera images
- lidar scans
- odometry
- TF
- simulated robot motion

Nav2 and AMCL provide:

- localization
- path planning
- semantic target navigation once we send them a goal

### Step 2: we summarize the world

`observation_node.py` converts raw data into a simpler summary:

- current pose
- current heading
- whether localization is healthy
- obstacle sectors from lidar
- semantic location such as `hallway_mid`
- visible tracked objects
- recent executor state

Why this exists:

- large models should not have to rediscover everything from raw topics
- debugging is easier when the robot state is visible in a compact message

### Step 3: we decide the robot’s high-level mode

`control_supervisor_node.py` decides what kind of control is allowed right now.

Examples:

- if manual input is active, mode becomes `MANUAL`
- if there is an active task, mode becomes `BRAIN_TASK`
- otherwise, if localization is good, mode becomes `PATROL`

Why this exists:

- behavior priority must be explicit
- otherwise the LLM, patrol, and teleop would fight each other

### Step 4: the planner chooses one structured action

`planner_node.py` sends a request to the model backend.

The planner request includes:

- the active task
- the observation summary
- the semantic targets
- the current image

The model returns one action such as:

- `TURN`
- `DRIVE`
- `GOTO_SEMANTIC`
- `FOLLOW_OBJECT`
- `LOOK_AROUND`
- `STOP`

Why it is structured like this:

- it gives the model room to reason
- but it prevents the model from becoming the motor controller

### Step 5: the executor does the real work

`skill_executor_node.py` maps the action into one of two styles of control:

- direct local controller
  - `TURN`
  - `DRIVE`
  - `LOOK_AROUND`
  - `FOLLOW_OBJECT`
- Nav2 action client
  - `GOTO_SEMANTIC`

Why this is good:

- the planner stays simple
- the controller logic stays testable
- future model backends can change without rewriting robot control

### Step 6: the arbiter decides whose motion wins

`cmd_arbiter_node.py` is the last gate before motion reaches the robot.

Priority is effectively:

- emergency stop / safety stop
- manual input
- brain executor
- Nav2
- zero command

This is what makes the system safe enough to experiment with.

## Why It Was Built This Way

### Why not let Gemma write `/cmd_vel` directly?

Because that would mix reasoning and low-level control into one fragile loop.

If the model is slow, confused, or inconsistent:

- direct motor control becomes jittery
- debugging becomes hard
- safety becomes weaker

The current architecture lets the model be “the brain” without making it “the whole robot”.

### Why keep a semantic map?

Because phrases like “doorway” or “hallway” need a grounded representation somewhere.

Right now that grounding is a simple YAML config with named targets.

That is not the final form of semantic grounding.
It is an honest intermediate layer.

### Why keep a manual override?

Because it is useful for:

- smoke testing
- recovery
- comparing autonomous behavior to direct control

### Why does patrol exist?

Because a robot that can only move when prompted is harder to debug.

Patrol proves:

- Nav2 works
- localization works
- the arbiter works
- the robot can keep doing something useful while idle

## Where The Complexity Really Is

The hardest parts are not the obvious ones.

### 1. ROS graph coordination

The challenge is not just “write a node”.
The challenge is getting many nodes, topics, modes, and action servers to agree.

### 2. Message types and wiring

One real bug in this repo was exactly this:

- some motion commands were plain `Twist`
- some were `TwistStamped`
- the wrong subscription type silently broke motion

This is typical ROS complexity.

### 3. Async model inference

The planner cannot block the rest of the node forever while the model thinks.

That is why the planner now uses off-thread inference and drops stale results.

### 4. Nav2 preemption

When a new task comes in, patrol has to stop cleanly.

That interaction is real and slightly messy, especially when mode switches happen mid-navigation.

### 5. macOS Gazebo specifics

The repo has macOS-specific environment handling because Gazebo and OGRE resource lookup were not fully frictionless here.

That complexity is real platform work, not robot logic.

## What Is Verified Versus Not Verified

Verified:

- patrol
- `turn left`
- `drive to doorway`
- manual override
- local Gemma planning through Ollama

Implemented but not yet strongly validated:

- `FOLLOW_OBJECT`

Present in launch/config but not re-verified in the final pass:

- GUI mode
- mapping flow after the latest changes

## Can This Work With A Physical Robot?

In principle: yes.

Why I say yes:

- the brain and control architecture is not Gazebo-specific in concept
- the planner uses topics and services, not simulator-only hacks
- the LLM is already separated from the low-level controller boundary

Why I do not say “it is ready for a physical robot”:

- real robots need hardware drivers
- sensor topics may differ
- real localization and calibration are harder
- safety needs to be taken more seriously
- controller tuning will change

So the correct statement is:

- the architecture is compatible with a real robot path
- the repository is not yet a drop-in real robot deployment

## What Was Written Versus What Was Generated

Written by project code:

- node logic
- launch files
- interface definitions
- prompts
- semantic map
- map-generation script
- docs

Generated by build tools:

- interface bindings
- install tree
- build tree
- logs

Imported from dependencies:

- world files
- robot model
- navigation stack
- simulator
- model runtime

## Good Places To Experiment

### Easy experiments

- change the patrol order in `semantic_map.yaml`
- add or rename semantic targets
- change the system prompt
- submit different tasks
- tweak controller gains and tolerances in the executor

### Medium-difficulty experiments

- add new planner actions
- add a second model backend
- change the observation summary format
- improve the object tracker

### Hard experiments

- replace semantic-map grounding with learned scene understanding
- move toward a real robot
- make the model more end-to-end without breaking safety boundaries

## If You Only Read Three Files First

Read these first:

- [planner_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/planner_node.py)
- [skill_executor_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/skill_executor_node.py)
- [cmd_arbiter_node.py](/Users/jonas/RLKitchen/robots101/src/brain_nodes/brain_nodes/cmd_arbiter_node.py)

Those three files explain the most important design idea in the repo:

- the model proposes
- the executor controls
- the arbiter decides who is allowed to move the robot
