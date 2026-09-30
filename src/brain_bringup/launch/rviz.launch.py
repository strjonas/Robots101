"""RViz with the robots101 view: map, lidar, costmaps, planned path, camera, waypoints.

RViz shows what the robot believes (its map, its pose estimate, its plan), while
Gazebo shows what is actually there. Comparing the two is how you debug.
Run it in its own terminal next to any of the other launches.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from brain_bringup.runtime import config_path


def generate_launch_description() -> LaunchDescription:
    default_config = config_path("robots101.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_config),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            ),
        ]
    )
