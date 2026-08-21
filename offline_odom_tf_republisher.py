#!/usr/bin/env python3
"""Republish only odom->base_link from recorded Odometry timestamps."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__("offline_odom_tf_republisher")
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, "/odom", self.callback, 20)

    def callback(self, msg):
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = msg.header.frame_id or "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTfRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
