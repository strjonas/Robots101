# Roadmap

## Works (verified on an M4 Mac, macOS 26.6 and 27.0)

- [x] Native Pixi + RoboStack install of ROS 2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox
- [x] TurtleBot3 house simulation: camera, lidar, odometry, TF; the Gazebo window renders (screenshot checked on 26.6)
- [x] Keyboard driving through the arbiter, manual override of patrol and tasks
- [x] Safety stop that blocks forward motion near obstacles but allows backing away
- [x] Emergency stop
- [x] SLAM mapping by driving, saving the map
- [x] AMCL localization + Nav2 patrol through named waypoints
- [x] Multi-step tasks: planner loop with history, STOP ends the task, failures are bounded
- [x] Rule-based planner (no LLM) and Gemma planner through Ollama, both tested with multi-step tasks
- [x] RViz layout (map, lidar, costmaps, plans, particles, camera, detections, waypoints)
- [x] Status panel, `submit-task --wait`, YOLO object detection
- [x] Walking person in the world; `FOLLOW_OBJECT` follows them (camera bearing + lidar distance, search when lost), with the rules planner and with Gemma
- [x] RViz draws the map, costmaps, lidar and robot (checked by eye on macOS 27)

## Not yet shown working

- [ ] Integration tests: everything above was checked by running the stack, not by an automated test

## Next steps

1. Keep following through doorways: when the person disappears, navigate to where they were last seen.
2. Add a new skill end to end (message, schema, executor, prompt), e.g. `DOCK` or `APPROACH_OBJECT`.
3. Semantic places from perception instead of hand-written coordinates ("where did I last see the cup?").
4. A launch-level smoke test that starts the headless stack and runs one task.
5. Manipulation: an arm, MoveIt 2 and pick-and-place, the step from "drives around" to "household robot". Likely easier on Linux.
