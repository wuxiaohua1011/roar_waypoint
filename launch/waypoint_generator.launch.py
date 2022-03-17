from asyncio import base_subprocess
from email.mime import base

from sqlalchemy import true
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
import launch
import launch_ros


def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory("roar_waypoint"))
    rviz_path = base_path + "/configs/waypoint_generator.rviz"
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="ios_ip_address"),
            launch.actions.DeclareLaunchArgument(
                name="target_frame", default_value="base_link"
            ),
            launch.actions.DeclareLaunchArgument(name="rate", default_value="0.2"),
            launch.actions.DeclareLaunchArgument(
                name="dir", default_value="./data/waypoints/"
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_roar_streamer"),
                        "roar_sensor_stream_only_without_rviz.launch.py",
                    )
                ),
                launch_arguments={
                    "ios_ip_address": launch.substitutions.LaunchConfiguration(
                        "ios_ip_address"
                    )
                }.items(),
            ),
            Node(
                package="roar_waypoint",
                executable="waypoint_generator",
                name="waypoint_generator",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "target_frame": launch.substitutions.LaunchConfiguration(
                            "target_frame"
                        ),
                        "rate": launch.substitutions.LaunchConfiguration("rate"),
                        "dir": launch.substitutions.LaunchConfiguration("dir"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_path)],
            ),
        ]
    )
