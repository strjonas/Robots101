# Operations Runbook

This file is the operator-facing runbook for this repository.

It deliberately separates:

- verified behavior: things that were directly exercised during the implementation session
- standard-but-not-re-verified commands: commands that are normal for ROS, Gazebo, or Ollama, but were not re-run in this documentation-only pass

## Verified State

The following were directly exercised during the implementation session:

- `pixi run build`
- `pixi run test`
- `pixi run nav`
- `pixi run brain`
- `pixi run submit-task -- --instruction "turn left"`
- `pixi run submit-task -- --instruction "drive to doorway"`
- manual velocity override by publishing on `/brain/cmd_vel_manual`

Observed working behavior:

- the robot patrols semantic waypoints in the TurtleBot3 house world
- the local Gemma planner can produce `TURN` and `GOTO_SEMANTIC` actions
- the executor can perform direct actions and Nav2-backed semantic navigation
- the task server auto-completes successful `TURN` and `GOTO_SEMANTIC` tasks
- manual input preempts patrol and then returns to patrol after timeout

Not directly verified in the final documentation pass:

- `FOLLOW_OBJECT` end-to-end in a reliable live scene
- the Gazebo GUI path with `with_gui:=true`
- a dedicated RViz workflow, because the repo currently does not launch RViz for you
- the mapping workflow after the latest planner/executor fixes

## Important Environment Variables

### `ROBOTS101_ROOT`

Optional override for the repository root.

Why it exists:

- the repo should keep working even if you move it somewhere else
- helper scripts now use this variable if it is set
- launch defaults for the house map also use this variable

Example:

```bash
export ROBOTS101_ROOT=/path/to/wherever/you/moved/the/repo
cd "$ROBOTS101_ROOT"
```

If you do not set it, the scripts infer the root from their own location.

### `ROBOTS101_WITH_GUI`

Controls whether the Gazebo client is requested by the Pixi tasks.

Examples:

```bash
export ROBOTS101_WITH_GUI=false
pixi run nav
```

```bash
export ROBOTS101_WITH_GUI=true
pixi run full-stack
```

### `ROBOTS101_MAP_FILE`

Optional override for the Nav2 map file.

Example:

```bash
export ROBOTS101_MAP_FILE="$ROBOTS101_ROOT/maps/turtlebot3_house.yaml"
```

### `ROS_HOME`

Optional override for ROS logs and cache state.

If unset, `scripts/ros_env.sh` uses `$ROBOTS101_ROOT/.ros`.

## Start Commands

### One-time setup

```bash
pixi install
pixi run build
pixi run test
pixi run generate-house-map
```

### Split start, headless

Terminal 1:

```bash
pixi run nav
```

Terminal 2:

```bash
pixi run brain
```

Terminal 3, optional manual teleop:

```bash
pixi run teleop
```

Terminal 4, submit tasks:

```bash
pixi run submit-task -- --instruction "turn left"
pixi run submit-task -- --instruction "drive to doorway"
```

### Combined start

```bash
pixi run full-stack
```

### Mapping

```bash
pixi run mapping
```

This launch exists. It was not the focus of the final validation pass.

## What You Can See

### Without GUI

Headless mode is the most trustworthy mode right now.

What to watch:

- terminal logs from `patrol_node`: semantic patrol targets changing
- terminal logs from `control_supervisor_node`: mode changes such as `PATROL`, `BRAIN_TASK`, `MANUAL`
- terminal logs from `planner_node`: structured actions chosen by Gemma
- terminal logs from `cmd_arbiter_node`: active command source switching between `nav`, `executor`, `manual`, `zero`

Useful ROS checks:

```bash
ros2 topic echo --once /brain/observation_summary
ros2 topic echo --once /brain/current_task
ros2 topic echo --once /brain/planner_action
ros2 topic echo --once /brain/executor_status
ros2 topic echo --once /brain/active_cmd_source
```

What a healthy headless smoke test looks like:

- patrol starts automatically
- `/brain/observation_summary` shows pose and semantic location
- a `turn left` task causes control mode to switch to `BRAIN_TASK`
- `planner_node` emits a `TURN`
- the executor runs the turn
- the task clears
- patrol resumes

### With GUI

The repo has a Gazebo GUI path through the `with_gui` launch argument.

What this means today:

- it asks Gazebo to start the client window
- it does not automatically start RViz

What I am sure about:

- the GUI launch path exists in the launch files

What I am not claiming here:

- that the GUI path was re-validated in this final documentation-only pass

If the GUI path works on your machine, you should be able to:

- see the apartment-like world
- see the robot move during patrol
- observe task-driven motion such as `turn left` or `drive to doorway`

## Ollama / Gemma Commands

These are standard Ollama commands. I am confident they are the intended commands, but I did not re-run them in this documentation-only pass.

Start the Ollama daemon:

```bash
ollama serve
```

List downloaded models:

```bash
ollama list
```

Show currently loaded / running models:

```bash
ollama ps
```

Download Gemma locally:

```bash
ollama pull gemma4:e4b
```

Unload the model from memory but keep it on disk:

```bash
ollama stop gemma4:e4b
```

Remove the model from disk entirely:

```bash
ollama rm gemma4:e4b
```

Notes:

- `ollama serve` starts the daemon, not necessarily the model itself
- the model is normally loaded on demand by a request
- `ollama stop gemma4:e4b` is the thing that should free model RAM if your local version supports it

## Process Inspection

These commands are useful when you think old processes are still alive.

### General process listing

```bash
pgrep -af 'ollama|ros2|gz|rviz|planner_node|skill_executor|nav2|slam_toolbox|component_container'
```

```bash
ps aux | rg 'ollama|ros2|gz|rviz|planner_node|skill_executor|nav2|slam_toolbox|component_container'
```

### ROS graph inspection

```bash
ros2 node list
ros2 topic list
ros2 action list
```

### Check whether Ollama is listening

```bash
lsof -i :11434
```

## Stop Commands

### Best option

Stop things from the terminals that launched them with `Ctrl-C`.

That is the cleanest route.

### If things are stuck

These are broad force-stop commands. Read them before using them.

Stop launch files and ROS runtime processes:

```bash
pkill -f 'ros2 launch brain_bringup'
pkill -f 'ros2 run brain_nodes'
pkill -f 'component_container_isolated'
pkill -f 'gz sim'
```

Stop teleop:

```bash
pkill -f 'teleop_twist_keyboard'
```

Stop Ollama if you launched it manually:

```bash
pkill -f 'ollama serve'
```

If you ever configure Ollama as a Homebrew background service, use the Homebrew service commands instead of `pkill`.

## Known Operational Caveats

- macOS Gazebo is viable here, but it is still a higher-friction platform than Linux for simulator work
- headless bringup is the most trustworthy path at the moment
- the Nav2 controller frequently logs rate warnings on this machine; those warnings did not prevent successful patrol or task execution in the verified runs
- `FOLLOW_OBJECT` needs a scene where the tracker can actually see a YOLO-recognizable object
- there is no packaged “show me everything” dashboard yet; the main observability is terminal logs plus ROS topics
