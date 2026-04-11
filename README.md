# robots101

Native macOS ROS 2 project for a simulated mobile robot with:

- TurtleBot3 Waffle Pi in Gazebo Harmonic
- camera + lidar sensing
- manual teleop through a command arbiter
- autonomous waypoint patrol via Nav2
- a backend-agnostic LLM/VLM planner
- a skill executor that turns planner actions into ROS-native motion
- a hybrid object-following loop that uses tracked detections for control

## Current Status

Verified on this machine:

- native `Pixi` + `RoboStack` + ROS 2 Jazzy + Gazebo Harmonic bringup on `osx-arm64`
- TurtleBot3 house simulation with camera, lidar, TF, odometry, AMCL, and Nav2
- command arbitration across patrol, brain-issued actions, and manual teleop
- end-to-end brain task execution for `turn left` and `drive to doorway` with local `gemma4:e4b`
- semantic patrol loop through the generated house map

Implemented but not yet live-validated in the default house view:

- `FOLLOW_OBJECT` behavior through the tracker/executor path
- current default viewpoints often produce no YOLO detections, so a reliable follow demo still needs a deliberately visible target in frame

## Repository Layout

- `docs/roadmap.md`: tracked execution plan and phase checklist
- `docs/operations.md`: operator runbook, start/stop commands, process inspection
- `docs/next_developer_handoff.md`: factual handoff for the next engineer
- `docs/beginner_guide.md`: beginner-facing explanation of how the system fits together
- `pixi.toml`: pinned environment and developer tasks
- `src/brain_interfaces`: ROS messages and services
- `src/brain_nodes`: runtime nodes, planner backend, controllers, CLIs
- `src/brain_bringup`: launch files and runtime configuration

## Root Path Handling

The repo no longer assumes a fixed absolute workspace path.

If you move the project and want one explicit root variable, use:

```bash
export ROBOTS101_ROOT=/path/to/the/repo
```

The helper scripts and launch defaults now honor that variable.

## Intended Flow

1. Bring up simulation and teleop through the arbiter.
2. Build or import a map for `turtlebot3_house`.
3. Run Nav2 and patrol named semantic targets.
4. Submit high-level tasks to the brain service.
5. Let the planner select structured actions while controllers and safety nodes retain low-level authority.

## First Commands

After installing `pixi`:

```bash
pixi install
pixi run build
pixi run validate-env
pixi run generate-house-map
pixi run nav
pixi run brain
```

On macOS, the launch stack now prepares a short OGRE plugin path under `/tmp` automatically before starting Gazebo. Keep `ROBOTS101_WITH_GUI=false` for the first smoke tests unless you specifically want to debug the GUI path.

For teleop in a separate terminal:

```bash
pixi run teleop
```

After the workspace is built and Nav2 is up, submit a task:

```bash
pixi run submit-task -- --instruction "drive to doorway"
```

Or launch everything together:

```bash
pixi run full-stack
```

## Map Bootstrap

- `maps/turtlebot3_house.yaml` and `maps/turtlebot3_house.pgm` are generated from the packaged `turtlebot3_house` SDF with `pixi run generate-house-map`.
- `pixi run mapping` is still available if you want to replace the generated occupancy map with a SLAM-produced map later.
