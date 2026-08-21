#!/usr/bin/env python3
"""ROS smoke test for the C++ D500 node using a pseudo-terminal only."""

import os
import pty
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x4D) & 0xFF if (value & 0x80) else (value << 1) & 0xFF
    return value


def packet(start, end, distance=1000):
    raw = bytearray(47)
    raw[0:2] = b"\x54\x2c"
    struct.pack_into("<HH", raw, 2, 1200, start)
    for i in range(12):
        off = 6 + 3 * i
        struct.pack_into("<H", raw, off, distance + i)
        raw[off + 2] = 20
    struct.pack_into("<HH", raw, 42, end, 1)
    raw[-1] = crc8(raw[:-1])
    return bytes(raw)


class ScanWatcher(Node):
    def __init__(self):
        super().__init__("d500_cpp_ros_smoke_watcher")
        self.scan = None
        self.create_subscription(LaserScan, "/scan", self.receive, 10)

    def receive(self, message):
        self.scan = message


def main():
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "177"
    proc = None
    os.environ["ROS_DOMAIN_ID"] = "177"
    rclpy.init(args=None)
    watcher = ScanWatcher()
    try:
        proc = subprocess.Popen([
            "ros2", "run", "my_epuck_project_cpp", "d500_ros2_scan_cpp",
            "--ros-args", "-p", f"port:={slave_path}",
            "-p", "frame_id:=d500_lidar", "-p", "topic:=/scan",
        ], env=env, stdout=None, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.5)
        stream = b"garbage" + packet(35000, 35900) + packet(100, 1000)
        # The node flushes the serial input immediately after opening it.  A
        # few repeated writes make the fixture independent of process-start
        # timing while preserving the parser test itself.
        for _ in range(3):
            os.write(master, stream)
            time.sleep(0.4)
        deadline = time.monotonic() + 8.0
        while watcher.scan is None and time.monotonic() < deadline:
            rclpy.spin_once(watcher, timeout_sec=0.1)
        if watcher.scan is None:
            raise AssertionError("C++ D500 node published no scan")
        assert watcher.scan.header.frame_id == "d500_lidar"
        assert len(watcher.scan.ranges) == 720
        print("D500_CPP_ROS_SMOKE PASS frame=d500_lidar bins=720")
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5.0)
        watcher.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
