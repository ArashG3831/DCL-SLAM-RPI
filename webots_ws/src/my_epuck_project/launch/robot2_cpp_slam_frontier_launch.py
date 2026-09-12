#!/usr/bin/env python3
"""Single authoritative physical Robot 2 exploration stack.

This composes the already validated native hardware, live SLAM, direct Nav2,
and existing frontier-explorer launch paths.  It deliberately does not start
Webots, AMCL, a static map server, or any legacy Python hardware node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _python_launch(path):
    return PythonLaunchDescriptionSource(path)


def generate_launch_description():
    project_share = get_package_share_directory("my_epuck_project")
    cpp_share = get_package_share_directory("my_epuck_project_cpp")
    rviz_enabled = LaunchConfiguration("rviz")

    hardware = IncludeLaunchDescription(
        _python_launch(os.path.join(
            cpp_share, "launch", "hardware_backend_selector.launch.py")),
        launch_arguments={
            "motor_backend": "cpp",
            "lidar_backend": "cpp",
        }.items(),
    )

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d500_lidar_static_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.07",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "d500_lidar",
        ],
    )

    slam = IncludeLaunchDescription(
        _python_launch(os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": os.path.join(
                project_share, "resource", "slam_toolbox_real_d500.yaml"),
            "use_sim_time": "false",
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        _python_launch(os.path.join(
            project_share, "launch", "real_nav2_live_slam_launch.py")),
    )

    # Instantiate directly so the SLAM launch's global ``autostart`` argument
    # cannot collide with the frontier launch's same-named string argument.
    # The YAML remains authoritative for the typed boolean parameters.
    frontier = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            os.path.join(
                project_share, "resource", "frontier_explorer_real_balanced.yaml"),
            {
                "use_sim_time": False,
                "map_qos_durability": "transient_local",
                "map_qos_autodetect_on_startup": False,
                "costmap_qos_reliability": "reliable",
            },
        ],
    )

    # Start the established all-in-one RViz preset with the physical stack.
    # It already contains the map, TF, lidar, odometry, costmaps, and Nav2
    # path displays, with map as the fixed frame.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz_enabled),
        arguments=["-d", os.path.join(project_share, "resource", "all.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz locally when a display is available.",
        ),
        hardware,
        lidar_tf,
        slam,
        nav2,
        frontier,
        rviz,
    ])
