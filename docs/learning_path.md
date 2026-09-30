# Learning path

Nine lessons, each 30–60 minutes, using this repo as the playground. Every lesson has something to run, something to look at, and a small change to make. Do them in order; each one uses the layer before it.

Keep three terminals open: one for the launch, one for RViz or the status panel, one for commands. All commands run from the repo folder.

---

## 1. Nodes and topics: drive the robot yourself

**Idea.** A ROS system is a set of small programs (*nodes*) that talk by publishing messages on named channels (*topics*). Nobody calls anybody directly; they just publish and subscribe.

**Run**
```bash
pixi run sim          # terminal 1: simulator with the robot
pixi run teleop       # terminal 2: keyboard driving (i = forward, j/l = turn, k = stop)
```

**Look**
```bash
pixi run ros node list
pixi run ros topic list
pixi run ros topic echo /brain/cmd_vel_manual     # what your keyboard sends
pixi run ros topic echo /cmd_vel                  # what actually reaches the robot
pixi run ros topic hz /scan                       # lidar rate (~10 Hz)
pixi run graph                                    # the whole graph as a picture
```

**Understand.** Your keys do not go to the robot. They go to `/brain/cmd_vel_manual`, the [arbiter](../src/brain_nodes/brain_nodes/cmd_arbiter_node.py) picks a winner and publishes `/brain/cmd_vel_final`, and the [stamper](../src/brain_nodes/brain_nodes/cmd_vel_stamper_node.py) forwards it to `/cmd_vel`, which the Gazebo bridge feeds to the simulated wheels. Follow that chain in `pixi run graph`.

**Change.** In `cmd_arbiter_node.py`, multiply `output.linear.x` by 0.5, run `pixi run build`, relaunch, and drive. Then undo it.

---

## 2. Sensors and the safety stop

**Idea.** A lidar reports distances all around the robot as one array. Which array index means "straight ahead" depends on the sensor, and getting it wrong is a classic bug.

**Run** `pixi run sim`, `pixi run teleop`, and `pixi run status`.

**Look.** `pixi run ros topic echo /scan --once` and find `angle_min`, `angle_increment` and `ranges`. Drive slowly at a wall and watch `safety` in the status panel switch to *OBSTACLE AHEAD* at about 0.22 m. Forward keys stop working; reverse and turning still work.

**Read** [scan_utils.py](../src/brain_nodes/brain_nodes/scan_utils.py), [safety_monitor_node.py](../src/brain_nodes/brain_nodes/safety_monitor_node.py), and `limit_forward_speed` in [arbiter_logic.py](../src/brain_nodes/brain_nodes/arbiter_logic.py). Then read "the safety monitor looked backwards" in [architecture.md](architecture.md#bugs-worth-learning-from).

**Change.** Raise `stop_distance_m` to 0.5 at the top of `safety_monitor_node.py`, rebuild and feel the difference. Add a test case to `src/brain_nodes/test/test_scan_utils.py` and run `pixi run test`.

---

## 3. Coordinate frames (TF) and RViz

**Idea.** Every sensor and body part has its own coordinate frame (`base_link`, `base_scan`, `odom`, `map`). TF is the live tree of how they relate, so a lidar point can be turned into a point on the map.

**Run** `pixi run sim` and `pixi run rviz`.

**Look.** In RViz, set *Fixed Frame* (top of the Displays panel) to `odom`; `map` doesn't exist yet in this layer. Tick the **TF** display to see the frame axes. Drive and watch the lidar points stay stuck to the walls. Then `pixi run ros run tf2_tools view_frames` records the tree for a few seconds and writes it as a PDF into the current folder.

**Understand.** `odom` is where the wheels *think* the robot went. It drifts over time. Lesson 5 adds `map`, which corrects the drift.

---

## 4. Build a map with SLAM

**Idea.** SLAM (Simultaneous Localization And Mapping) builds a map and tracks the robot's place in it at the same time, from lidar and odometry.

**Run**
```bash
pixi run mapping      # sim + SLAM Toolbox
pixi run rviz         # Fixed Frame: map
pixi run teleop       # drive slowly through every room
pixi run save-map     # writes maps/my_house.yaml + .pgm
```

**Look.** Watch the grey map grow in RViz as you drive. Spin in place in each room; turning slowly gives cleaner maps. Open `maps/my_house.pgm` in Preview: white is free, black is a wall, grey is unknown.

**Compare.** `pixi run generate-house-map` made `maps/turtlebot3_house.yaml` by reading the world file directly, which is only possible in simulation. Now navigate with your own map:

```bash
ROBOTS101_MAP_FILE=$PWD/maps/my_house.yaml pixi run nav
```

One catch, and a good lesson about frames: a SLAM map's origin (0, 0) is wherever the robot was when mapping started, while the generated map uses the world's origin. The named places in `semantic_map.yaml` and the automatic start pose are in world coordinates, so on your own map they are off by the spawn offset (-4, -2). Give AMCL the right start pose with **2D Pose Estimate** in RViz, and re-record the places you care about with `pixi run record-waypoint --name <place>`.

---

## 5. Localization and navigation (AMCL + Nav2)

**Idea.** With a known map, AMCL estimates where the robot is by matching lidar to the map (the cloud of arrows in RViz is its set of guesses). Nav2 plans a path on a *costmap* (the map plus a safety margin around walls) and follows it while avoiding new obstacles.

**Run** `pixi run nav` and `pixi run rviz`.

**Look.** In RViz: the particle cloud shrinks as AMCL becomes sure. The global plan (path) and local costmap update as the robot patrols. Use the **Nav2 Goal** tool in the toolbar to click a goal on the map. Use **2D Pose Estimate** to deliberately put AMCL in the wrong place, and watch it recover (or not).

**Read** [patrol_node.py](../src/brain_nodes/brain_nodes/patrol_node.py): it is a Nav2 *action* client. Actions are for long jobs: send a goal, get feedback, get one result, and you can cancel.

**Change.** Add a place to [semantic_map.yaml](../src/brain_bringup/config/semantic_map.yaml) (or drive there and run `pixi run record-waypoint --name bedroom`) and put it in `patrol_order`. Relaunch; it shows up as a blue dot in RViz and the patrol route changes. No rebuild needed.

---

## 6. Modes and arbitration: who is allowed to drive?

**Idea.** Several things want to move the robot. A clear priority order prevents them from fighting.

**Run** `pixi run nav`, `pixi run status`, `pixi run teleop`.

**Try.** While it patrols, press a key: mode becomes MANUAL, then returns to PATROL half a second after you stop. Run `pixi run estop --enabled --reason test`: everything stops. `pixi run estop` releases it.

**Read** [control_supervisor_node.py](../src/brain_nodes/brain_nodes/control_supervisor_node.py) (who is in charge) and [arbiter_logic.py](../src/brain_nodes/brain_nodes/arbiter_logic.py) (whose command is forwarded), with its tests in `test_arbiter_logic.py`. Keeping decisions in small pure functions is what makes them testable without a simulator.

---

## 7. The brain: tasks, planning, skills

**Idea.** A task is carried out one action at a time. After each action the planner is asked again, with the history of what already happened, until it answers STOP.

**Run**
```bash
ROBOTS101_PLANNER_BACKEND=rules pixi run full-stack
pixi run status
pixi run submit-task --instruction "turn left, then go to the hallway and then turn around" --wait
```

**Read in order**
1. [task_server_node.py](../src/brain_nodes/brain_nodes/task_server_node.py) and [task_logic.py](../src/brain_nodes/brain_nodes/task_logic.py): when a task ends.
2. [planner_node.py](../src/brain_nodes/brain_nodes/planner_node.py): the plan → wait → remember → plan loop.
3. [rules_client.py](../src/brain_nodes/brain_nodes/model_clients/rules_client.py): the simplest possible "brain".
4. [skill_executor_node.py](../src/brain_nodes/brain_nodes/skill_executor_node.py): each skill is a small feedback loop, except GOTO_SEMANTIC, which hands the job to Nav2.

**Change.** Teach the rules backend "spin twice" (a TURN of 4π). Add a test in `test_rules_client.py` first, make it pass, then try it live.

---

## 8. Perception: follow a person

**Idea.** A camera tells you *what* and *in which direction*; a lidar tells you *how far*. Combining them (sensor fusion) gives you where an object is. Following is then two small controllers: turn towards it, drive to keep a distance.

**Run** (a person walks a slow loop south of the house, right in front of where the robot starts)
```bash
ROBOTS101_PLANNER_BACKEND=rules pixi run full-stack
pixi run rviz          # tick "Detections" to see YOLO's boxes with distances
pixi run status        # "sees" shows label, distance and bearing
pixi run submit-task --instruction "follow the person"
pixi run clear-task    # following runs until you stop it
```

**Look.** In the detections image each box says `lidar` or `size`: whether its distance was measured by the lidar or guessed from the box height. Beyond 3.5 m (the lidar's range) it falls back to guessing, and the guess is poor because the low camera cuts the person off. When the person walks out of view the robot spins towards where it last saw them.

**Read** [object_tracker_node.py](../src/brain_nodes/brain_nodes/object_tracker_node.py) (bearing and fusion), [tracking_logic.py](../src/brain_nodes/brain_nodes/tracking_logic.py) (pinhole camera geometry), and [follow_logic.py](../src/brain_nodes/brain_nodes/follow_logic.py) (the controller) with their tests.

**Change.** In `rules_client.py`, give the FOLLOW_OBJECT proposal `preferred_distance_m=2.0` and watch the robot keep a bigger gap. Then change `TURN_GAIN` in `follow_logic.py` to 5.0 and watch the robot wobble: that is what "gain too high" looks like.

---

## 9. Swap in the LLM

**Run** (after `ollama pull gemma4:e4b`)
```bash
pixi run full-stack
pixi run submit-task --instruction "drive forward half a meter and then turn right" --wait
pixi run submit-task --instruction "make me a sandwich" --wait
pixi run submit-task --instruction "find the person and follow them" --wait
```

**Look.** Each `plan` line shows the model's rationale. Compare with lesson 7: same executor, same safety, different decision maker. The model sees the camera image too; in RViz, tick **Detections** to see what YOLO found.

**Change.** Edit [gemma_system_prompt.md](../src/brain_bringup/config/gemma_system_prompt.md), for example to make it always LOOK_AROUND before navigating, and relaunch (no rebuild). Watch how much a prompt changes behaviour, and how the executor still refuses anything outside the action schema.

---

## Where to go next

- **Add a skill.** For example `DOCK` (drive to a named place, then face a wall at 0.3 m). You touch `BrainAction.msg`, `action_schema.py`, the executor and the prompt: a full vertical slice.
- **Smarter following.** Keep following through doorways by handing the last known position to Nav2 when the person disappears around a corner.
- **Towards a household robot.** Anything that picks things up needs an arm: MoveIt 2 and a manipulator model are the next big topic. That work is smoother on Linux than on macOS.
