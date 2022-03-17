#!/usr/bin/env python3

from sympy import im
import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import PointCloud2
import socket
import numpy as np
from typing import Optional, Tuple
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from typing import List
from geometry_msgs.msg import Point
from pathlib import Path
from datetime import datetime


class WaypointGeneratorNode(Node):
    def __init__(self):
        super().__init__("waypoint_generator_node")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("rate", 0.2)
        self.declare_parameter("dir", "./data/waypoints/")
        self.declare_parameter("file_name", "")

        self.directory: Path = Path(
            self.get_parameter("dir").get_parameter_value().string_value
        )
        if self.directory.exists() == False:
            self.directory.mkdir(exist_ok=True, parents=True)
        self.file_name = (
            self.get_parameter("file_name").get_parameter_value().string_value
        )
        now = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        self.file_path: Path = (
            self.directory / f"{now}.txt"
            if self.file_name == ""
            else Path(self.file_name)
        )
        self.file = self.file_path.open("+w")
        self.path_publisher = self.create_publisher(
            msg_type=NavPath,
            topic="path",
            qos_profile=1,
        )

        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.rate = self.get_parameter("rate").get_parameter_value().double_value
        assert self.rate > 0, "Rate has to be a Float greater than 0"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(self.rate, self.on_timer)
        self.path: NavPath = NavPath()

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = "odom"

        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, now
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return
        current_pose = PoseStamped()
        current_pose.header.frame_id = "base_link"
        current_pose.pose.position = Point(
            x=trans.transform.translation.x,
            y=trans.transform.translation.y,
            z=trans.transform.translation.z,
        )
        current_pose.pose.orientation = trans.transform.rotation
        self.path.poses.append(current_pose)
        new_path = NavPath()
        new_path.poses = self.path.poses
        new_path.header.frame_id = "odom"
        self.path = new_path
        self.path_publisher.publish(self.path)

        self.write_transform_stamped_to_file(trans)

    def write_transform_stamped_to_file(self, t: TransformStamped):

        content = f"{t.transform.translation.x},{t.transform.translation.y},{t.transform.translation.z},{t.transform.rotation.x},{t.transform.rotation.y},{t.transform.rotation.z},{t.transform.rotation.w} \n"
        self.file.write(content)

    def destroy_node(self):
        self.file.close()
        self.get_logger().info(
            f"[{len(self.path.poses)}] points written to {self.file_path}"
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WaypointGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()