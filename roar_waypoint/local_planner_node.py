#!/usr/bin/env python3

from dis import dis
from importlib.resources import path
from cv2 import transform
from matplotlib.transforms import Transform
from pydantic import FilePath
from sympy import im, true
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
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
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped, Pose, Point
from typing import List
from geometry_msgs.msg import Point
from pathlib import Path
from datetime import datetime
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner_node")
        # parameter verification
        self.declare_parameter("waypoint_file_path", "")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("next_waypoint_topic", "/next_waypoint")
        self.declare_parameter("source_frame", "odom")
        self.declare_parameter("marker_size", 1.0)
        self.declare_parameter("closeness_threshold", 1.0)
        self.declare_parameter("odom_topic", "/carla/ego_vehicle/odometry")

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Listenng to odom topic {self.odom_topic}")
        self.closeness_threshold = (
            self.get_parameter("closeness_threshold").get_parameter_value().double_value
        )
        self.get_logger().info(f"Closeness_threshold = {self.closeness_threshold}")
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size = {self.marker_size}")
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"target frame = {self.target_frame}")
        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"source frame = {self.source_frame}")
        self.waypoint_file_path: Path = Path(
            self.get_parameter("waypoint_file_path").get_parameter_value().string_value
        )
        self.path_topic = (
            self.get_parameter("path_topic").get_parameter_value().string_value
        )

        self.get_logger().info(f"Path topic = {self.path_topic}")
        self.next_waypoint_topic = (
            self.get_parameter("next_waypoint_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"next waypoint topic = {self.next_waypoint_topic}")
        assert (
            self.waypoint_file_path.exists()
        ), f"{self.waypoint_file_path} does not exist"
        self.path: NavPath = self.read_waypoint_file(file_path=self.waypoint_file_path)
        # subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # publisher
        self.has_path_published: bool = False
        self.path_publisher = self.create_publisher(
            msg_type=NavPath,
            topic=self.path_topic,
            qos_profile=1,
        )
        self.waypoint_marker_publisher = self.create_publisher(
            msg_type=Marker, topic=self.next_waypoint_topic, qos_profile=1
        )

        self.get_logger().info(f"[{len(self.path.poses)}] points were read")
        # timer
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.timer_callback,
            10,
        )
        self.subscription
        self.create_timer(2, self.global_path_callback)

        # State variables
        self.is_init_position_in_path_determined = False
        self.closest_waypoint_index = 0

    def global_path_callback(self):
        self.path_publisher.publish(self.path)  # publish path globally

    def timer_callback(self, odom_msg: Odometry):
        self.find_next_waypoint(odom_msg)

    def find_next_waypoint(self, odom_msg: Odometry):
        # first get the vehicle's current transform
        target_frame = self.target_frame
        source_frame = self.source_frame

        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                source_frame, target_frame, now
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {source_frame} to {target_frame}: {ex}"
            )
            return

        # find the best starting point if first started
        if not self.is_init_position_in_path_determined:
            self.closest_waypoint_index = self.find_closest(self.path, trans)
            self.is_init_position_in_path_determined = true

        # find the first point outside of search radius
        self.closest_waypoint_index = self.find_index_of_first_point_outside_of_radius(
            path=self.path,
            curr_index=self.closest_waypoint_index,
            transform=trans,
            radius=self.closeness_threshold,
        )
        m = Marker(
            pose=self.path.poses[self.closest_waypoint_index].pose,
            color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            lifetime=Duration(sec=1),
            scale=Vector3(x=self.marker_size, y=self.marker_size, z=self.marker_size),
            type=2,
        )
        m.header.frame_id = self.source_frame
        m.header.stamp = odom_msg.header.stamp
        self.waypoint_marker_publisher.publish(m)

    @staticmethod
    def distance(p1: Pose, p2: Pose):
        return np.sqrt(
            (p1.position.x - p2.position.x) ** 2 + (p1.position.y - p2.position.y) ** 2
        )

    @staticmethod
    def find_index_of_first_point_outside_of_radius(
        path: NavPath,
        curr_index: int,
        transform: TransformStamped,
        radius: float = 0.1,
    ):
        assert curr_index < len(
            path.poses
        ), f"Something is wrong. curr_index = {curr_index}, len(path) = {len(path.poses)}"
        vehicle_loc = Pose(
            position=Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z,
            )
        )
        while True:

            curr_dist = LocalPlannerNode.distance(
                p1=vehicle_loc, p2=path.poses[curr_index].pose
            )
            if curr_dist > radius:
                break
            else:
                curr_index = (curr_index + 1) % len(path.poses)
        return curr_index

    @staticmethod
    def find_closest(path: NavPath, transform: TransformStamped):
        closest_dist = 100000
        closest_index = -1
        vehicle_loc = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ]
        )
        for i, pose_stamped in enumerate(path.poses):
            waypoint_loc = np.array(
                [pose_stamped.pose.position.x, pose_stamped.pose.position.y]
            )
            dist = np.linalg.norm(vehicle_loc - waypoint_loc)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
        return closest_index

    def read_waypoint_file(self, file_path: Path) -> NavPath:
        assert file_path.exists(), f"{file_path} does not exist"
        file = file_path.open("r")
        path = NavPath()
        path.header.frame_id = self.source_frame
        for line in file.readlines():
            data = line.split(",")
            data = [float(d) for d in data]
            current_pose = PoseStamped()
            current_pose.header.frame_id = self.source_frame
            current_pose.pose.position = Point(
                x=data[0],
                y=data[1],
                z=data[2],
            )
            current_pose.pose.orientation = Quaternion(
                x=data[3], y=data[4], z=data[5], w=data[6]
            )
            path.poses.append(current_pose)
        return path

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
