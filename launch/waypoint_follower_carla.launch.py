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
    rviz_path = base_path + "/configs/waypoint_follower_carla.rviz"
    pid_config_path = base_path + "/configs/pid_config_carla.json"
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="path_topic", default_value="/path"
            ),
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
            launch.actions.DeclareLaunchArgument(
                name="pid_config_path", default_value=pid_config_path
            ),
            launch.actions.DeclareLaunchArgument(
                name="odom_topic", default_value="/carla/ego_vehicle/odometry"
            ),
            launch.actions.DeclareLaunchArgument(
                name="speed_topic", default_value="/carla/ego_vehicle/speedometer"
            ),
            launch.actions.DeclareLaunchArgument(
                name="next_waypoint_topic", default_value="/next_waypoint"
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("roar_carla_ros2"),
                        "roar_carla_no_rviz.launch.py",
                    )
                )
            ),
            launch.actions.DeclareLaunchArgument(
                name="target_speed", default_value="30.0"
            ),
            Node(
                package="roar_waypoint",
                executable="local_planner_node",
                name="local_planner_node",
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
                        "pid_config_path": launch.substitutions.LaunchConfiguration(
                            "pid_config_path"
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
            Node(
                package="roar_waypoint",
                executable="carla_pid_controller_node",
                name="carla_pid_controller_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "pid_config_path": launch.substitutions.LaunchConfiguration(
                            "pid_config_path"
                        ),
                        "odom_topic": launch.substitutions.LaunchConfiguration(
                            "odom_topic"
                        ),
                        "speed_topic": launch.substitutions.LaunchConfiguration(
                            "speed_topic"
                        ),
                        "next_waypoint_topic": launch.substitutions.LaunchConfiguration(
                            "next_waypoint_topic"
                        ),
                        "target_frame": launch.substitutions.LaunchConfiguration(
                            "target_frame"
                        ),
                        "source_frame": launch.substitutions.LaunchConfiguration(
                            "source_frame"
                        ),
                        "target_speed": launch.substitutions.LaunchConfiguration(
                            "target_speed"
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
