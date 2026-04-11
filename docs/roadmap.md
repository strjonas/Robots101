# Project Roadmap

This file is the tracked execution plan for the repository. It is intended to stay aligned with the actual code and launch scripts.

## Phase 1: Environment Bootstrap

- [x] Install `pixi`
- [x] Install and start `ollama`
- [x] Pull `gemma4:e4b`
- [x] Create the `osx-arm64` Pixi workspace
- [x] Solve RoboStack / Gazebo / Nav2 dependencies
- [x] Build the ROS workspace with `colcon`

## Phase 2: Simulation Foundation

- [x] Launch `turtlebot3_house`
- [x] Verify camera, lidar, TF, odometry
- [x] Route manual teleop through the command arbiter
- [x] Add safety monitor and emergency-stop service

## Phase 3: Navigation And Grounding

- [ ] Launch mapping flow for the house world
- [x] Save or import a usable map for Nav2
- [x] Bring up AMCL + Nav2 with `cmd_vel` remapped into the arbiter
- [x] Configure semantic targets and patrol order

## Phase 4: Perception And State

- [x] Publish structured observation summaries
- [x] Add detector/tracker-backed object state
- [x] Keep raw RGB available for the model backend

## Phase 5: Brain And Skills

- [x] Add swappable `ModelClient` interface
- [x] Implement Ollama backend for `gemma4:e4b`
- [x] Validate structured planner output
- [x] Execute `TURN`, `DRIVE`, `GOTO_SEMANTIC`, `FOLLOW_OBJECT`, `LOOK_AROUND`
- [x] Add task ingress CLI and ROS services

## Phase 6: Integrated Validation

- [x] Manual teleop works through the arbiter
- [x] Patrol mode cycles semantic waypoints
- [x] Task: `turn left`
- [x] Task: `drive to doorway`
- [ ] Task: `follow the object`
- [x] Capture rosbag profiles for debugging

## Notes

- The architecture boundary is fixed:
  `sensors -> perception/state -> planner/policy -> skill executor/controller -> command arbiter -> /cmd_vel`
- The planner is replaceable. Safety, arbitration, and controller ownership are not delegated to the model.
- If native Gazebo on macOS fails early smoke tests, pause feature work and resolve platform issues before adding more behaviors.
- Native macOS needed two concrete fixes:
  `gz sim` must run outside the sandbox, and Gazebo's OGRE plugin lookup on this machine needs a short real directory under `/tmp` to avoid a macOS path/plugin loading bug.
- The current house-world follow demo still needs a deliberately visible YOLO-recognizable object in frame; the tracker/executor path is implemented, but the default patrol viewpoints often produce no live detections.
- The planner now runs model inference off-thread and drops stale results so completed or preempted tasks do not publish delayed actions back into the executor path.
