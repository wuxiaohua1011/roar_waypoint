#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaEgoVehicleControl
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


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__("pid_controller_node")
        # parameter verification
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("pid_config_path", "")
        self.declare_parameter("max_reverse_throttle", -1.0)
        self.declare_parameter("max_forward_throttle", 1.0)
        self.declare_parameter("max_steering", 1.0)
        self.declare_parameter("steering_offset", 0.0)
        self.declare_parameter("source_frame", "odom")

        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self.max_reverse_throttle = (
            self.get_parameter("max_reverse_throttle")
            .get_parameter_value()
            .double_value
        )
        self.max_forward_throttle = (
            self.get_parameter("max_forward_throttle")
            .get_parameter_value()
            .double_value
        )
        self.max_steering = (
            self.get_parameter("max_steering").get_parameter_value().double_value
        )
        self.steering_offset = (
            self.get_parameter("steering_offset").get_parameter_value().double_value
        )
        self.get_logger().info(
            f"NOTE: max_forward_throttle = {self.max_forward_throttle}"
        )
        self.get_logger().info(
            f"NOTE: max_reverse_throttle = {self.max_reverse_throttle}"
        )
        self.get_logger().info(f"NOTE: max_steering = {self.max_steering}")
        self.get_logger().info(f"NOTE: steering_offset = {self.steering_offset}")

        self.pid_config_path: Path = Path(
            self.get_parameter("pid_config_path").get_parameter_value().string_value
        )
        assert self.pid_config_path.exists(), f"{self.pid_config_path} does not exist"
        self.config = json.load(self.pid_config_path.open("r"))
        self.lat_config = self.config["latitudinal_controller"]
        self.long_config: dict = self.config["longitudinal_controller"]
        # subscribe
        self.odom_msg = message_filters.Subscriber(self, Odometry, "/iPhone_odom")
        self.waypoint_msg = message_filters.Subscriber(self, Marker, "/next_waypoint")
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [
                self.odom_msg,
                self.waypoint_msg,
            ],
            queue_size=10,
            slop=0.2,
        )
        self.ats.registerCallback(self.on_msgs_received)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # publisher
        self.carla_msg_publisher = self.create_publisher(
            msg_type=CarlaEgoVehicleControl,
            topic="/carla/ego_vehicle/vehicle_control_cmd_manual",
            qos_profile=1,
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

        self.downhill_throttle = self.long_config.get("downhill_throttle", -0.01)
        self.uphill_throttle = self.long_config.get("uphill_throttle", 0.3)
        self.flat_ground_throttle = self.long_config.get("flat_ground_throttle", 0.2)
        self.up_ramp_grade = self.long_config.get("up_ramp_grade", 10)
        self.down_ramp_grade = self.long_config.get("down_ramp_grade", -10)

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
        throttle = self.compute_throttle(
            waypoint_pose=waypoint_msg.pose, odom_msg=odom_msg
        )

        control = CarlaEgoVehicleControl(
            throttle=np.clip(
                throttle, self.max_reverse_throttle, self.max_forward_throttle
            ),
            steer=np.clip(steering, -self.max_steering, self.max_steering),
        )
        self.carla_msg_publisher.publish(control)

    def compute_steering(
        self, waypoint_pose: Pose, odom_msg: Odometry, transform: Transform
    ):

        v_begin = np.array(
            [
                transform.translation.x,
                0,
                transform.translation.y,
            ]
        )

        quat = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ]
        r, p, y = tf_transformations.euler_from_quaternion(quat)
        direction_vector = np.array([np.cos(y), 0, np.sin(y)])

        v_end = v_begin + direction_vector

        v_vec = np.array([v_end[0] - v_begin[0], 0, v_end[2] - v_begin[2]])
        w_vec = np.array(
            [
                waypoint_pose.position.x - v_begin[0],
                0,
                -(waypoint_pose.position.y - v_begin[2]),  # this is probably wrong lol
            ]
        )
        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        error = np.arccos(v_vec_normed @ w_vec_normed.T)
        _cross = np.cross(v_vec_normed, w_vec_normed)
        if _cross[1] > 0:
            error *= -1
        self.lat_error_queue.append(error)

        if len(self.lat_error_queue) >= 2:
            _de = self.lat_error_queue[-1] - self.lat_error_queue[-2]
            _ie = sum(self.lat_error_queue)
        else:
            _de = 0.0
            _ie = 0.0
        k_p, k_d, k_i = self.find_k_values(config=self.lat_config, odom=odom_msg)
        lat_control = float(np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), -1, 1))
        # print(
        #     f"v_vec_normed: {v_vec_normed} | w_vec_normed: {w_vec_normed} | error = {error} | lat_control = {lat_control}"
        # )

        return lat_control

    def compute_throttle(self, waypoint_pose: Pose, odom_msg: Odometry):
        quat = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ]
        r, p, y = tf_transformations.euler_from_quaternion(quat)
        neutral = -90
        incline = np.degrees(r) - neutral
        if incline < self.down_ramp_grade:
            # down hill
            return self.downhill_throttle
        elif incline > self.up_ramp_grade:
            return self.uphill_throttle
        else:
            return self.flat_ground_throttle

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
