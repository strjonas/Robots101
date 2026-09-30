"""Everything at once: simulation, navigation and the brain (layers 1 to 3)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_gui = LaunchConfiguration("with_gui")
    map_file = LaunchConfiguration("map_file")
    repo_root = EnvironmentVariable("ROBOTS101_ROOT", default_value=EnvironmentVariable("PWD"))

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("brain_bringup"), "launch", "nav_only.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "with_gui": with_gui, "map_file": map_file}.items(),
    )
    brain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("brain_bringup"), "launch", "brain_only.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "planner_backend": LaunchConfiguration("planner_backend"),
            "model_name": LaunchConfiguration("model_name"),
            "ollama_url": LaunchConfiguration("ollama_url"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("with_gui", default_value="true"),
            DeclareLaunchArgument("planner_backend", default_value="ollama"),
            DeclareLaunchArgument("model_name", default_value="gemma4:e4b"),
            DeclareLaunchArgument("ollama_url", default_value="http://127.0.0.1:11434"),
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution([repo_root, "maps", "turtlebot3_house.yaml"]),
            ),
            nav_launch,
            brain_launch,
        ]
    )
