from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    semantic_map_path = PathJoinSubstitution([FindPackageShare("brain_bringup"), "config", "semantic_map.yaml"])
    prompt_path = PathJoinSubstitution([FindPackageShare("brain_bringup"), "config", "gemma_system_prompt.md"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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

