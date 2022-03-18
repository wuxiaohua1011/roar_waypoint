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
    rviz_path = base_path + "/configs/waypoint_follower.rviz"
    pid_config_path = base_path + "/configs/pid_config.json"
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="path_topic", default_value="/path"
            ),
            launch.actions.DeclareLaunchArgument(
                name="next_waypoint_topic", default_value="/next_waypoint"
            ),
            launch.actions.DeclareLaunchArgument(name="ios_ip_address"),
            launch.actions.DeclareLaunchArgument(
                name="max_reverse_throttle", default_value="-1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_forward_throttle", default_value="1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_steering", default_value="1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="steering_offset", default_value="0.0"
            ),
            launch.actions.DeclareLaunchArgument(name="rate", default_value="0.05"),
            launch.actions.DeclareLaunchArgument(name="waypoint_file_path"),
            launch.actions.DeclareLaunchArgument(
                name="pid_config_path", default_value=pid_config_path
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
                        "ios_ip_address",
                    )
                }.items(),
            ),
            Node(
                package="roar_waypoint",
                executable="local_planner_node",
                name="local_planner_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "waypoint_file_path": launch.substitutions.LaunchConfiguration(
                            "waypoint_file_path"
                        ),
                        "rate": launch.substitutions.LaunchConfiguration("rate"),
                        "pid_config_path": launch.substitutions.LaunchConfiguration(
                            "pid_config_path"
                        ),
                    }
                ],
            ),
            Node(
                package="roar_waypoint",
                executable="pid_controller_node",
                name="pid_controller_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "next_waypoint_topic": launch.substitutions.LaunchConfiguration(
                            "next_waypoint_topic"
                        ),
                        "path_topic": launch.substitutions.LaunchConfiguration(
                            "path_topic"
                        ),
                        "max_reverse_throttle": launch.substitutions.LaunchConfiguration(
                            "max_reverse_throttle"
                        ),
                        "max_forward_throttle": launch.substitutions.LaunchConfiguration(
                            "max_forward_throttle"
                        ),
                        "max_steering": launch.substitutions.LaunchConfiguration(
                            "max_steering"
                        ),
                        "steering_offset": launch.substitutions.LaunchConfiguration(
                            "steering_offset"
                        ),
                        "pid_config_path": launch.substitutions.LaunchConfiguration(
                            "pid_config_path"
                        ),
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
