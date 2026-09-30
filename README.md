# robots101

A simulated household robot you can drive, map with, send around the apartment, and give plain-English tasks to — built to learn ROS 2 from the ground up.

- **Robot:** TurtleBot3 Waffle Pi (wheels, lidar, camera) in the Gazebo TurtleBot3 house world, with a person walking around
- **Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox, YOLO, a local LLM through Ollama
- **Platform:** native macOS on Apple Silicon via [Pixi](https://pixi.sh) + RoboStack (no Docker, no VM)

```
you ──task──▶ planner (LLM or rules) ──one action──▶ skill executor ──velocity──▶ arbiter ──▶ robot
                  ▲                                         │                        ▲
                  └──────── observation summary ◀── lidar, camera, odometry          │
                                                                    keyboard, Nav2, safety stop
```

The LLM decides *what* to do next, deterministic controllers decide *how*, and an arbiter decides *who* may move the wheels. The model never drives the motors directly.

## Quick start

```bash
pixi install                 # once: ~6 GB of ROS, Gazebo and Python packages
pixi run build               # compile the three packages in src/
pixi run test                # unit tests, no simulator needed
pixi run generate-house-map  # once: maps/turtlebot3_house.yaml for navigation
```

Then, each in its own terminal:

```bash
pixi run full-stack          # simulator + navigation + brain; the Gazebo window opens, the robot starts patrolling
pixi run rviz                # what the robot believes: map, lidar, plan, camera, waypoints
pixi run status              # live text panel: mode, who is driving, task, planner, executor
pixi run submit-task --instruction "go to the kitchen, then turn around" --wait
```

More to try: `"turn left"`, `"drive to doorway"`, `"find the person and follow them"` (stop following with `pixi run clear-task`).

No LLM set up yet? Use the keyword planner, which needs no model:

```bash
ROBOTS101_PLANNER_BACKEND=rules pixi run full-stack
```

First start takes a minute or two before the robot appears. Stop everything with Ctrl-C in the launch terminal.

## Learning with it

Start with **[docs/learning_path.md](docs/learning_path.md)**: nine short hands-on lessons, from "what is a topic" to building your own map, following a person, and swapping in the LLM. It uses this repo as the playground, one layer at a time.

| Doc | What it is for |
|---|---|
| [docs/learning_path.md](docs/learning_path.md) | Hands-on lessons, in order |
| [docs/architecture.md](docs/architecture.md) | How the nodes fit together, and why; real bugs from this repo and what they teach |
| [docs/operations.md](docs/operations.md) | Every command, environment variable, and what to do when something hangs |
| [docs/roadmap.md](docs/roadmap.md) | What works, what doesn't yet, what's next |

## The layers

Each launch builds on the previous one, so you can run only as much as you are studying:

| Command | Adds | You can |
|---|---|---|
| `pixi run sim` | Gazebo, robot, bridge, arbiter, supervisor, safety monitor, observation | drive with `pixi run teleop` |
| `pixi run mapping` | sim + SLAM Toolbox | build a map by driving, `pixi run save-map` |
| `pixi run nav` | sim + AMCL localization + Nav2 + patrol | watch it patrol, click goals in RViz |
| `pixi run full-stack` | nav + object tracker, planner, skill executor | give it tasks in English |

Useful extras: `pixi run teleop` (keyboard driving, overrides everything for half a second per key), `pixi run estop --enabled` / `pixi run estop` (emergency stop on / off), `pixi run graph` (node graph), `pixi run ros topic list` (any `ros2` command).

`ROBOTS101_WITH_GUI=false` skips the Gazebo window.

## Code map

```
src/brain_interfaces/   messages and services: the vocabulary the nodes speak
src/brain_nodes/        all runtime nodes (one file each, each starts with what it subscribes/publishes)
  brain_nodes/*_logic.py, scan_utils.py   pure decision rules, unit-tested without ROS
  brain_nodes/model_clients/              planner backends: ollama_client.py, rules_client.py
  test/                                   unit tests
src/brain_bringup/      launch files (the layers above), config, and the world
  config/semantic_map.yaml        named places and the patrol route
  config/gemma_system_prompt.md   what the LLM is told
  config/robots101.rviz           the RViz layout
  worlds/robots101_house.world    the house plus a walking person
scripts/                environment setup, build, map generator
```

Config files are read straight from `src/`, so edits apply on the next launch. After changing Python, run `pixi run build` again.

## Using the LLM planner

The default backend asks `gemma4:e4b` through Ollama at `http://127.0.0.1:11434`:

```bash
ollama pull gemma4:e4b
```

`gemma4:e4b` needs a recent Ollama (0.35.0 works; 0.30.7 fails with "unknown model architecture"); the planner says so at startup if the server or model is missing. Each planning step takes 10–25 s on an M4 while the simulator runs. `ROBOTS101_MODEL` and `ROBOTS101_OLLAMA_URL` override the model and server.
