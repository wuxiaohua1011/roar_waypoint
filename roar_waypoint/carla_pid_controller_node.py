#!/usr/bin/env python3

from anyio import current_default_worker_thread_limiter
import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import socket
import numpy as np
from typing import Optional, Tuple
from sensor_msgs.msg import CameraInfo
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Transform
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from typing import List
from geometry_msgs.msg import Point, Pose
from pathlib import Path
from datetime import datetime
from visualization_msgs.msg import Marker
import message_filters
from nav_msgs.msg import Odometry
import tf_transformations
from collections import deque
import json
import math


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__("pid_controller_node")
        # parameter verification
        self.declare_parameter("target_frame", "ego_vehicle")
        self.declare_parameter("pid_config_path", "")
        self.declare_parameter("source_frame", "map")
        self.declare_parameter("odom_topic", "/carla/ego_vehicle/odometry")
        self.declare_parameter("speed_topic", "/carla/ego_vehicle/speedometer")
        self.declare_parameter("next_waypoint_topic", "/next_waypoint")
        self.declare_parameter("target_speed", 25.0)

        self.target_speed = (
            self.get_parameter("target_speed").get_parameter_value().double_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )

        self.pid_config_path: Path = Path(
            self.get_parameter("pid_config_path").get_parameter_value().string_value
        )
        assert self.pid_config_path.exists(), f"{self.pid_config_path} does not exist"
        self.config = json.load(self.pid_config_path.open("r"))
        self.lat_config = self.config["latitudinal_controller"]
        self.long_config: dict = self.config["longitudinal_controller"]

        # subscribe
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [
                message_filters.Subscriber(
                    self,
                    Odometry,
                    self.get_parameter("odom_topic").get_parameter_value().string_value,
                ),
                message_filters.Subscriber(
                    self,
                    Marker,
                    self.get_parameter("next_waypoint_topic")
                    .get_parameter_value()
                    .string_value,
                ),
            ],
            queue_size=10,
            slop=0.5,
            allow_headerless=True,
        )
        self.ats.registerCallback(self.on_msgs_received)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        print(
            f"Subscribing to odom topic: [{self.get_parameter('odom_topic').get_parameter_value().string_value}]"
        )
        print(
            f"Subscribing to next waypoint topic: [{self.get_parameter('next_waypoint_topic').get_parameter_value().string_value}]"
        )

        # publisher
        self.carla_msg_publisher = self.create_publisher(
            msg_type=CarlaEgoVehicleControl,
            topic="/carla/ego_vehicle/vehicle_control_cmd",
            qos_profile=10,
        )

        # state variables
        self.long_error_deque_length = 50
        self.lat_error_deque_length = 10
        self.lat_error_queue = deque(
            maxlen=self.lat_error_deque_length
        )  # how much error you want to accumulate
        self.long_error_queue = deque(
            maxlen=self.long_error_deque_length
        )  # how much error you want to accumulate

    def on_msgs_received(self, odom_msg: Odometry, waypoint_msg: Marker):
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

        steering = self.compute_steering(
            waypoint_pose=waypoint_msg.pose,
            odom_msg=odom_msg,
            transform=trans.transform,
        )
        throttle = self.compute_throttle(odom_msg)

        control = CarlaEgoVehicleControl(
            throttle=float(np.clip(throttle, -1, 1)),
            steer=float(np.clip(steering, -1, 1)),
        )
        control.header.frame_id = self.target_frame
        control.header.stamp = odom_msg.header.stamp

        self.carla_msg_publisher.publish(control)

    def compute_steering(
        self, waypoint_pose: Pose, odom_msg: Odometry, transform: Transform
    ):
        quat = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ]
        r, p, y = tf_transformations.euler_from_quaternion(quat)
        # VERY HACKY HERE
        current_heading = y
        desired_heading = math.atan2(
            waypoint_pose.position.y - transform.translation.y,
            waypoint_pose.position.x - transform.translation.x,
        )
        error = -1 * (desired_heading - current_heading)
        self.lat_error_queue.append(error)
        if len(self.lat_error_queue) >= 2:
            _de = self.lat_error_queue[-1] - self.lat_error_queue[-2]
            _ie = sum(self.lat_error_queue)
        else:
            _de = 0.0
            _ie = 0.0
        k_p, k_d, k_i = self.find_k_values(config=self.lat_config, odom=odom_msg)
        control = float(np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), -1, 1))
        return control

    def compute_throttle(self, odom_msg: Odometry):
        spd = self.get_speed(odom_msg)
        error = self.target_speed - spd
        self.long_error_queue.append(error)
        if len(self.long_error_queue) >= 2:
            _de = self.long_error_queue[-1] - self.long_error_queue[-2]
            _ie = sum(self.long_error_queue)
        else:
            _de = 0.0
            _ie = 0.0
        k_p, k_d, k_i = self.find_k_values(config=self.long_config, odom=odom_msg)
        control = float(np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), -1, 1))
        return control

    def destroy_node(self):
        super().destroy_node()

    @staticmethod
    def get_speed(odom: Odometry):
        return np.sqrt(
            odom.twist.twist.linear.x**2
            + odom.twist.twist.linear.y**2
            + odom.twist.twist.linear.z**2
        )

    @staticmethod
    def find_k_values(odom: Odometry, config: dict) -> np.array:
        current_speed = PIDControllerNode.get_speed(odom)
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])


def main(args=None):
    rclpy.init(args=args)

    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
