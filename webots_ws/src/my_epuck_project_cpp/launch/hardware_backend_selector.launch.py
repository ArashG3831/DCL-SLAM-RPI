#!/usr/bin/env python3
"""Opt-in backend selector; defaults to starting neither hardware node."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def select(context):
    motor = context.launch_configurations["motor_backend"]
    lidar = context.launch_configurations["lidar_backend"]
    allowed = {"none", "python", "cpp"}
    if motor not in allowed or lidar not in allowed:
        raise RuntimeError("motor_backend and lidar_backend must be none, python, or cpp")
    actions = []
    if motor == "cpp":
        actions.append(Node(package="my_epuck_project_cpp", executable="real_diffdrive_node_cpp",
                             name="real_diffdrive_node", output="screen"))
    elif motor == "python":
        actions.append(Node(package="my_epuck_project", executable="real_diffdrive_node",
                             name="real_diffdrive_node", output="screen"))
    if lidar == "cpp":
        actions.append(Node(package="my_epuck_project_cpp", executable="d500_ros2_scan_cpp",
                            name="d500_ros2_scan", output="screen",
                            parameters=[{"topic": "/scan", "frame_id": "d500_lidar"}]))
    elif lidar == "python":
        actions.append(ExecuteProcess(
            cmd=["python3", str(Path.home() / "d500_ros2_scan.py"),
                 "--topic", "/scan", "--frame-id", "d500_lidar"],
            output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motor_backend", default_value="none",
                              description="none|python|cpp; select exactly one motor backend"),
        DeclareLaunchArgument("lidar_backend", default_value="none",
                              description="none|python|cpp; select exactly one lidar backend"),
        OpaqueFunction(function=select),
    ])
