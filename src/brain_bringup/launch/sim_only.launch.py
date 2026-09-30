"""Layer 1: the simulated robot plus the always-on control nodes.

Starts Gazebo with the TurtleBot3 house world and a walking person
(worlds/robots101_house.world), spawns the robot (which also starts the
ROS<->Gazebo bridge for /scan, /odom, /tf, /cmd_vel and the camera), and the
nodes every other layer relies on: task server, control supervisor, command
arbiter, safety monitor, observation summary and the RViz place markers.

With only this running you can already drive with `pixi run teleop`.
"""

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from brain_bringup.runtime import config_path, prepare_macos_gazebo_env, world_path


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    semantic_map_path = config_path("semantic_map.yaml")
    with_gui = LaunchConfiguration("with_gui")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    turtlebot3_gazebo_share = FindPackageShare("turtlebot3_gazebo")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")
    world = LaunchConfiguration("world")

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items(),
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(with_gui),
    )
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([turtlebot3_gazebo_share, "launch", "robot_state_publisher.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([turtlebot3_gazebo_share, "launch", "spawn_turtlebot3.launch.py"])
        ),
        launch_arguments={"x_pose": spawn_x, "y_pose": spawn_y}.items(),
    )
    resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        PathJoinSubstitution([turtlebot3_gazebo_share, "models"]),
    )

    common_params = [{"use_sim_time": use_sim_time}]
    runtime_env_actions = prepare_macos_gazebo_env()

    return LaunchDescription(
        [
            *runtime_env_actions,
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("with_gui", default_value="true"),
            # The house plus a walking person. Pass the original turtlebot3_house.world for an empty house.
            DeclareLaunchArgument("world", default_value=world_path("robots101_house.world")),
            DeclareLaunchArgument("spawn_x", default_value="-4.0"),
            DeclareLaunchArgument("spawn_y", default_value="-2.0"),
            SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle_pi"),
            resource_path,
            gzserver_launch,
            gzclient_launch,
            robot_state_publisher_launch,
            spawn_launch,
            Node(package="brain_nodes", executable="task_server_node", output="screen", parameters=common_params),
            Node(package="brain_nodes", executable="control_supervisor_node", output="screen", parameters=common_params),
            Node(package="brain_nodes", executable="cmd_arbiter_node", output="screen", parameters=common_params),
            Node(package="brain_nodes", executable="cmd_vel_stamper_node", output="screen", parameters=common_params),
            Node(package="brain_nodes", executable="safety_monitor_node", output="screen", parameters=common_params),
            Node(
                package="brain_nodes",
                executable="observation_node",
                output="screen",
                parameters=common_params + [{"semantic_map_path": semantic_map_path}],
            ),
            Node(
                package="brain_nodes",
                executable="semantic_marker_node",
                output="screen",
                parameters=common_params + [{"semantic_map_path": semantic_map_path}],
            ),
        ]
    )
