#!/usr/bin/env python3
"""ROS 2 node for the phone controller."""

import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import Reset as SlamReset

from phone_controller_config import (
    LIVE_SLAM_COMPARISON_ENABLED,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
)
from phone_controller_state import PHONE_DIAGNOSTICS

class PhoneMapNode(Node):
    def __init__(self, map_state, odom_session):
        super().__init__("phone_4button_controller")
        self.map_state = map_state
        self.odom_session = odom_session
        self.fast_callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos)
        if LIVE_SLAM_COMPARISON_ENABLED:
            self.create_subscription(
                OccupancyGrid,
                "/map_off",
                self.map_off_callback,
                qos,
            )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            odom_qos,
            callback_group=self.fast_callback_group,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            scan_qos,
            callback_group=self.fast_callback_group,
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.reset_client = self.create_client(SlamReset, "/slam_toolbox/reset")
        self.reset_off_client = (
            self.create_client(SlamReset, "/slam_toolbox_off/reset")
            if LIVE_SLAM_COMPARISON_ENABLED
            else None
        )
        self.create_timer(
            0.05,
            self.update_robot_pose,
            callback_group=self.fast_callback_group,
        )

    def reset_slam(self):
        """Reset slam_toolbox in place and return (success, message)."""
        clients = [(self.reset_client, "slam_toolbox")]
        if self.reset_off_client is not None:
            clients.append((self.reset_off_client, "slam_toolbox_off"))
        for client, name in clients:
            if not client.wait_for_service(timeout_sec=2.0):
                return False, f"{name} reset service is unavailable"
            request = SlamReset.Request()
            request.pause_new_measurements = False
            future = client.call_async(request)
            deadline = time.monotonic() + 4.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                return False, f"{name} reset timed out"
            try:
                response = future.result()
            except Exception as exc:
                return False, f"{name} reset failed: {exc}"
            if response.result != SlamReset.Response.RESULT_SUCCESS:
                return False, f"{name} reset returned code {response.result}"
        return True, "Map reset" + (" (ON + OFF)" if self.reset_off_client else "")

    def map_callback(self, msg):
        detail = f"width={msg.info.width},height={msg.info.height}"
        with PHONE_DIAGNOSTICS.measure("map_callback", detail):
            self.map_state.update_map(msg)

    def map_off_callback(self, msg):
        detail = f"width={msg.info.width},height={msg.info.height}"
        with PHONE_DIAGNOSTICS.measure("map_off_callback", detail):
            self.map_state.update_map_off(msg)

    def scan_callback(self, _msg):
        with PHONE_DIAGNOSTICS.measure("scan_callback"):
            self.map_state.update_scan()

    def odom_callback(self, msg):
        with PHONE_DIAGNOSTICS.measure("odom_callback"):
            self.map_state.update_odom(msg)
            if LIVE_SLAM_COMPARISON_ENABLED:
                # The OFF branch is deliberately raw odometry.  Track its
                # pose from /odom directly instead of depending on the
                # diagnostic mapper's map_off -> odom TF publication.
                self.map_state.update_robot_off_from_odom(msg)
            self.odom_session.record(msg)

    def update_robot_pose(self):
        with PHONE_DIAGNOSTICS.measure("pose_timer"):
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                self.map_state.update_robot(transform)
            except Exception:
                # The map can arrive before map -> base_link TF is available.
                pass
