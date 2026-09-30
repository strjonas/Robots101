"""Build a map yourself: layer 1 plus SLAM Toolbox.

Drive around with `pixi run teleop` and watch the map grow in RViz
(`pixi run rviz`). SLAM estimates the map and the robot's position in it at the
same time from lidar and odometry. Save the result with `pixi run save-map`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_gui = LaunchConfiguration("with_gui")
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("brain_bringup"), "launch", "sim_only.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "with_gui": with_gui}.items(),
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("with_gui", default_value="true"),
            sim_launch,
            slam_launch,
        ]
    )
