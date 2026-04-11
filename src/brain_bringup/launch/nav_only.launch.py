from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_gui = LaunchConfiguration("with_gui")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    repo_root = EnvironmentVariable("ROBOTS101_ROOT", default_value=EnvironmentVariable("PWD"))
    semantic_map_path = PathJoinSubstitution([FindPackageShare("brain_bringup"), "config", "semantic_map.yaml"])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("brain_bringup"), "launch", "sim_only.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "with_gui": with_gui}.items(),
    )

    nav_launch = GroupAction(
        [
            SetRemap(src="/cmd_vel", dst="/brain/cmd_vel_nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
                ),
                launch_arguments={
                    "slam": "False",
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": "True",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("with_gui", default_value="true"),
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution([repo_root, "maps", "turtlebot3_house.yaml"]),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("turtlebot3_navigation2"), "param", "waffle_pi.yaml"]
                ),
            ),
            sim_launch,
            nav_launch,
            Node(
                package="brain_nodes",
                executable="localization_seed_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "semantic_map_path": semantic_map_path}],
            ),
            Node(
                package="brain_nodes",
                executable="patrol_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "semantic_map_path": semantic_map_path}],
            ),
        ]
    )
