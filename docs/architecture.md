# Architecture

How the pieces fit, why they are split the way they are, and the real bugs this repo has had.

## The one rule

```
sensors → observation → planner → skill executor → arbiter → wheels
```

The planner (an LLM or keyword rules) proposes **one structured action** at a time. The skill executor turns it into motion with small, predictable feedback loops. The arbiter has the final say over which velocity command reaches the wheels. The model is never allowed to publish velocities itself, so a slow, confused or wrong model can waste time but cannot drive into things.

## Nodes

Every node is one file in `src/brain_nodes/brain_nodes/`, and each file starts with a docstring listing what it subscribes to and publishes.

| Node | Job | Key topics |
|---|---|---|
| `control_supervisor_node` | Decides the mode: EMERGENCY_STOP > MANUAL > BRAIN_TASK > PATROL > IDLE | → `/brain/control_mode` |
| `cmd_arbiter_node` | Forwards the one allowed velocity, blocks forward motion when an obstacle is ahead | manual / executor / Nav2 → `/brain/cmd_vel_final` |
| `cmd_vel_stamper_node` | Twist → TwistStamped for the Gazebo bridge | → `/cmd_vel` |
| `safety_monitor_node` | Flags an obstacle within 0.22 m in front (±20°) | `/scan` → `/brain/safety_stop` |
| `observation_node` | One compact summary: pose, nearest named place, distances per side, visible objects | → `/brain/observation_summary` |
| `object_tracker_node` | YOLO detections with stable ids, bearing, and distance measured by the lidar at that bearing | `/camera/image_raw` → `/brain/tracked_objects`, `/brain/detection_image` |
| `task_server_node` | Holds the current task, decides when it is done or failed | services `/brain/submit_task`, `/brain/clear_task` |
| `planner_node` | Asks the backend for the next action, with history | → `/brain/planner_action` |
| `skill_executor_node` | Runs TURN, DRIVE, LOOK_AROUND, FOLLOW_OBJECT (with search) itself; GOTO_SEMANTIC via Nav2 | → `/brain/cmd_vel_executor`, `/brain/executor_status` |
| `patrol_node` | Sends Nav2 one patrol waypoint at a time while in PATROL mode | action `/navigate_to_pose` |
| `localization_seed_node` | Gives AMCL the start pose | → `/initialpose` |
| `semantic_marker_node` | Draws the named places and patrol route in RViz | → `/brain/semantic_markers` |
| `status_cli` | The `pixi run status` panel; only listens | |

Decision rules that don't need ROS live in pure modules with unit tests: `arbiter_logic.py`, `task_logic.py`, `planner_logic.py`, `scan_utils.py`, `tracking_logic.py`, `follow_logic.py`, `action_schema.py`, `model_clients/rules_client.py`. The command-line tools share `cli_utils.py`.

The world is [worlds/robots101_house.world](../src/brain_bringup/worlds/robots101_house.world): the TurtleBot3 house plus an animated person walking a loop, so there is something to detect, follow and avoid. The person is an *actor*: the camera and lidar see it, but it has no physics.

Why the tracker measures distance with the lidar: the camera sits about 10 cm above the floor, so a person nearby is cut off by the image edge and their box looks too small, which makes a size-based guess say "far away" exactly when they are close. The lidar at the camera's bearing gives the real distance; the size guess is only the fallback beyond the lidar's 3.5 m range.

From outside the repo: Gazebo (simulator), the TurtleBot3 packages (robot model, house world, bridge), Nav2 and AMCL (planning, control, localization), SLAM Toolbox (mapping), Ollama and Gemma (the LLM), Ultralytics YOLO (detection).

## Who drives when

The supervisor publishes a mode 10 times a second. The arbiter reads it and forwards exactly one source:

| Mode | Forwarded | Typical cause |
|---|---|---|
| EMERGENCY_STOP | nothing (zero) | `pixi run estop --enabled` |
| MANUAL | keyboard | a key was pressed in the last 0.6 s |
| BRAIN_TASK | executor, or Nav2 during GOTO_SEMANTIC | a task is active |
| PATROL | Nav2 | localized and nothing else to do |
| IDLE | nothing | waiting for localization |

A source also has to be *fresh* (sent something in the last 0.3–0.5 s); otherwise the arbiter sends zero. A crashed node therefore stops the robot instead of leaving it driving. On top of that, when the safety monitor sees an obstacle ahead, any forward speed is set to zero while turning and reversing stay allowed.

## A task, step by step

```
submit_task ──▶ task_server: task active ──▶ supervisor: mode BRAIN_TASK
                                                   │
planner: ask backend(instruction, observation, image, history) ──▶ action
                                                   │
executor: run it ──▶ status RUNNING … then SUCCEEDED / FAILED / REJECTED / PREEMPTED
                                                   │
planner: append to history, ask again       task_server: STOP → done,
                                              3 failures → give up,
                                              6 actions → done, 10 min → expired
```

A result that arrives after the task changed (because the user cleared it, or took over with the keyboard) is dropped instead of acted on. LLM calls take seconds, so they run on a worker thread and the node keeps processing messages meanwhile.

## Bugs worth learning from

Each of these really happened in this repo. They are typical of robotics software: every node looked fine on its own, and the bug was in how they fit together.

**Nav2 looked busy but the robot didn't move.** Nav2 publishes `TwistStamped`; the arbiter subscribed as `Twist`. ROS topics with the same name but different types just never connect, with no error. *Lesson:* `pixi run ros topic info -v <topic>` shows every publisher and subscriber with its type.

**The safety monitor looked backwards.** It treated the middle of the lidar array as "front", but the TurtleBot3 scan starts at 0 rad, straight ahead, so the middle is behind the robot. It never fired in any of the logged runs. *Lesson:* convert indices to angles with `angle_min + i * angle_increment` (see `scan_utils.py`), and test safety features by actually driving into a wall.

**A stopped robot could never get unstuck.** With the safety stop fixed, the old arbiter zeroed *all* motion while an obstacle was ahead, including reversing away from it. It now blocks only forward motion.

**Startup always began in MANUAL mode.** The supervisor initialised "last key press" to the startup time, so the first 0.6 s looked like manual driving. Related trap: a freshly constructed `ControlMode()` message has `mode = 0`, which *is* MANUAL. Nodes now start from IDLE explicitly.

**The LLM got asked the same question forever.** If the model answered STOP, or gave invalid JSON, the task never ended, and the planner re-asked every 2 s for the full 10-minute expiry. Now STOP ends the task, failures are counted, and a task is given up after three.

**Patrol sent every goal several times.** Sending a Nav2 goal is asynchronous: the "accepted" answer arrives later. Until it did, the patrol timer saw "no goal yet" and sent another. A `goal_pending` flag fixed it.

**Stale plans after a task ended.** Model inference used to block the planner's only thread, so "task cleared" messages were processed only after the answer arrived, which was then published anyway. Inference now runs off-thread and results are checked against the current task before publishing.

## Configuration

`src/brain_bringup/config/` is read directly from the source tree (via `ROBOTS101_ROOT`, set by `scripts/ros_env.sh`), so edits apply on the next launch:

- `semantic_map.yaml`: named places in map coordinates, tags, patrol order
- `gemma_system_prompt.md`: the LLM's instructions
- `robots101.rviz`: the RViz layout (save over it from RViz with File → Save Config)

Node parameters (speeds, tolerances, distances) are declared at the top of each node and can be overridden in the launch files.

## Towards a real robot

Nothing in the brain is Gazebo-specific: it only uses standard topics (`/scan`, `/odom`, `/camera/image_raw`, `/cmd_vel`) and Nav2. A real TurtleBot3 provides the same topics. What changes is everything the simulator makes easy: sensor noise, real localization, calibration, and treating safety as more than a lidar threshold.
