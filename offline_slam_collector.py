#!/usr/bin/env python3
"""Collect replayed Slam Toolbox map, pose, and odometry artifacts."""

import argparse
import csv
import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Collector(Node):
    def __init__(self):
        super().__init__("offline_slam_result_collector")
        map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.map = None
        self.poses = []
        self.odoms = []
        self.scan_count = 0
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.pose_callback, 20)
        self.create_subscription(PoseWithCovarianceStamped, "/slam_toolbox/pose", self.pose_callback, 20)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, scan_qos)

    @staticmethod
    def stamp(msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def map_callback(self, msg):
        self.map = {
            "frame_id": msg.header.frame_id,
            "stamp": self.stamp(msg),
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "data": [int(value) for value in msg.data],
        }

    def pose_callback(self, msg):
        self.poses.append({
            "timestamp": self.stamp(msg),
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
        })

    def odom_callback(self, msg):
        self.odoms.append({
            "timestamp": self.stamp(msg),
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
        })

    def scan_callback(self, _msg):
        self.scan_count += 1


def write_artifacts(node, output_dir):
    with open(output_dir + "/slam_map.json", "w", encoding="utf-8") as stream:
        json.dump(node.map or {}, stream)
    with open(output_dir + "/slam_trajectory.csv", "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["timestamp", "frame_id", "x", "y", "yaw"])
        writer.writeheader()
        writer.writerows(node.poses)
    with open(output_dir + "/replay_odom.csv", "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["timestamp", "x", "y", "yaw"])
        writer.writeheader()
        writer.writerows(node.odoms)
    with open(output_dir + "/replay_counts.json", "w", encoding="utf-8") as stream:
        json.dump({"scan_count": node.scan_count, "odom_count": len(node.odoms)}, stream, indent=2)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Collector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        write_artifacts(node, args.output_dir)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
