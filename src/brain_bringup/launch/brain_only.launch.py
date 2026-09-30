"""Layer 3: the "brain" - object tracker, planner and skill executor.

Run it on top of layer 2 (`pixi run nav`), or use full_stack.launch.py for both.

planner_backend:=ollama  asks a local LLM (needs `ollama serve` and the model)
planner_backend:=rules   keyword rules, no model needed - good for debugging
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from brain_bringup.runtime import config_path


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    semantic_map_path = config_path("semantic_map.yaml")
    prompt_path = config_path("gemma_system_prompt.md")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("planner_backend", default_value="ollama"),
            DeclareLaunchArgument("model_name", default_value="gemma4:e4b"),
            DeclareLaunchArgument("ollama_url", default_value="http://127.0.0.1:11434"),
            Node(package="brain_nodes", executable="object_tracker_node", output="screen", parameters=[{"use_sim_time": use_sim_time}]),
            Node(
                package="brain_nodes",
                executable="planner_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "semantic_map_path": semantic_map_path,
                        "prompt_path": prompt_path,
                        "backend": LaunchConfiguration("planner_backend"),
                        "model_name": LaunchConfiguration("model_name"),
                        "ollama_url": LaunchConfiguration("ollama_url"),
                    }
                ],
            ),
            Node(
                package="brain_nodes",
                executable="skill_executor_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "semantic_map_path": semantic_map_path}],
            ),
        ]
    )
