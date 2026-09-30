# Operations

Every command, setting and recovery step in one place. All commands run from the repo folder.

## Setup

```bash
pixi install                 # once, ~6 GB into .pixi/
pixi run build               # after changing Python, .msg/.srv, setup.py or package.xml
pixi run test                # unit tests
pixi run validate-env        # prints Python, ROS distro and package locations
pixi run generate-house-map  # once, writes maps/turtlebot3_house.{yaml,pgm}
```

## Running

Pick one launch per terminal:

| Command | Starts |
|---|---|
| `pixi run sim` | Gazebo + robot + control nodes |
| `pixi run mapping` | sim + SLAM Toolbox |
| `pixi run nav` | sim + AMCL + Nav2 + patrol |
| `pixi run brain` | tracker + planner + executor (on top of a running `nav`) |
| `pixi run full-stack` | nav + brain |

Tools, each in another terminal while something above runs:

| Command | Does |
|---|---|
| `pixi run rviz` | RViz with the project layout |
| `pixi run status` | live text panel |
| `pixi run teleop` | keyboard driving (i/j/k/l/…), preempts everything else |
| `pixi run graph` | rqt_graph node/topic picture |
| `pixi run ros <args>` | any `ros2` command, e.g. `pixi run ros topic echo /brain/control_mode` |
| `pixi run submit-task --instruction "…" [--wait]` | give it a task; `--wait` prints each step |
| `pixi run clear-task` | cancel the current task |
| `pixi run estop --enabled [--reason "…"]` / `pixi run estop` | emergency stop on / off |
| `pixi run record-waypoint --name kitchen_table [--tag patrol]` | save the current pose as a named place |
| `pixi run save-map` | save the SLAM map to `maps/my_house.{yaml,pgm}` |
| `pixi run rosbag-full` | record the important topics to `bags/` for replay |

Always start ROS programs through `pixi run`. Running a binary from `.pixi/envs/default/...` directly (for example double-clicking `rviz2`) skips the environment setup and crashes with `AMENT_PREFIX_PATH is not set`.

## Settings

| Variable | Default | Effect |
|---|---|---|
| `ROBOTS101_WITH_GUI` | `true` | `false` runs Gazebo without its window (faster, fine with RViz) |
| `ROBOTS101_PLANNER_BACKEND` | `ollama` | `rules` uses the keyword planner, no LLM needed |
| `ROBOTS101_MODEL` | `gemma4:e4b` | Ollama model name |
| `ROBOTS101_OLLAMA_URL` | `http://127.0.0.1:11434` | Ollama server |
| `ROBOTS101_MAP_FILE` | `maps/turtlebot3_house.yaml` | map for `nav` / `full-stack`, e.g. your SLAM map |
| `ROBOTS101_ROOT` | repo folder | set automatically; config files are read from here |
| `ROS_HOME` | `.ros/` in the repo | where ROS writes logs |

Example: `ROBOTS101_WITH_GUI=false ROBOTS101_PLANNER_BACKEND=rules pixi run full-stack`.

## What healthy looks like

In the launch terminal, within a minute or two of `pixi run full-stack`:

```
[control_supervisor]: Control mode -> 3 (Waiting for localization)
[localization_seed_node]: AMCL pose received. Localization seeded successfully.
[control_supervisor]: Control mode -> 1 (Default patrol mode)
[patrol_node]: Patrol goal -> foyer_spawn
[bt_navigator]: Goal succeeded
[patrol_node]: Patrol goal -> kitchen_entry
```

Mode numbers: 0 MANUAL, 1 PATROL, 2 BRAIN_TASK, 3 IDLE, 4 EMERGENCY_STOP.

Harmless noise you can ignore on macOS: RViz's `Validation Failed: Sampler error` (the map still draws), `Unable to load Ogre Plugin` (rendering works anyway), `Could not resolve file [Maple.jpg]`, Qt shader warnings, `Error_code parameters were not set`, and Nav2 controller-rate warnings.

## When something is wrong

**Robot doesn't appear / "Requesting list of world names" repeats.** The first start after installing or rebooting can take 1–2 minutes, and the very first start downloads the walking person's model from Gazebo Fuel. Wait. To run without the person: `pixi run ros launch brain_bringup sim_only.launch.py world:=$PWD/.pixi/envs/default/share/turtlebot3_gazebo/worlds/turtlebot3_house.world`.

**Nothing moves.** `pixi run status`: check `mode` and `driving`. IDLE means AMCL has no pose yet; in RViz use **2D Pose Estimate**. `safety: OBSTACLE AHEAD` blocks forward motion; drive back with teleop.

**A task does nothing.** Look at the `planner_node` lines in the launch terminal. With the Ollama backend: `curl http://127.0.0.1:11434/api/version` checks the server; `unknown model architecture` means Ollama is too old for the model. The task server gives up after 3 failed attempts, so a broken backend ends the task within about 10 s.

**Where are the logs?** `.ros/log/` (one folder per launch). `pixi run ros topic echo /brain/executor_status` shows what the executor is doing right now.

**Left-over processes after a crash.** Ctrl-C in the launch terminal is cleanest. If things are stuck:

```bash
pkill -f 'ros2 launch brain_bringup'; pkill -f 'gz sim'; pkill -f component_container; pkill -f brain_nodes
```

Then check nothing is left with `pgrep -fl 'gz sim|ros2|brain_nodes'`.

## Ollama

```bash
ollama pull gemma4:e4b   # 6.6 GB, once
ollama ps                # is a model loaded right now?
ollama stop gemma4:e4b   # free its memory, keep it on disk
```

`gemma4:e4b` needs a recent Ollama (0.35.0 works; 0.30.7 fails with "unknown model architecture"). The Ollama menu-bar app updates itself; the Homebrew CLI updates with `brew upgrade ollama`. Only one of them can serve port 11434 at a time. At startup the planner checks that the server is reachable and has the model, and logs an error saying what to do if not.
