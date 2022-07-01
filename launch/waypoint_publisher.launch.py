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
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="next_waypoint_topic", default_value="/next_waypoint"
            ),
            launch.actions.DeclareLaunchArgument(
                name="target_frame", default_value="ego_vehicle"
            ),
            launch.actions.DeclareLaunchArgument(
                name="source_frame", default_value="map"
            ),
            launch.actions.DeclareLaunchArgument(
                name="marker_size", default_value="1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="closeness_threshold", default_value="20.0"
            ),
            launch.actions.DeclareLaunchArgument(name="waypoint_file_path"),
            Node(
                package="roar_waypoint",
                executable="waypoint_publisher_node",
                name="waypoint_publisher_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "marker_size": launch.substitutions.LaunchConfiguration(
                            "marker_size"
                        ),
                        "closeness_threshold": launch.substitutions.LaunchConfiguration(
                            "closeness_threshold"
                        ),
                        "waypoint_file_path": launch.substitutions.LaunchConfiguration(
                            "waypoint_file_path"
                        ),
                        "target_frame": launch.substitutions.LaunchConfiguration(
                            "target_frame"
                        ),
                        "source_frame": launch.substitutions.LaunchConfiguration(
                            "source_frame"
                        ),
                    }
                ],
            ),
        ]
    )
