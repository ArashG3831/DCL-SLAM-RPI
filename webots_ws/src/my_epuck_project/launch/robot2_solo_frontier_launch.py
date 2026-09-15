#!/usr/bin/env python3
"""Robot 2 namespaced solo exploration mode.

This launch composes the validated local hardware/SLAM/Nav2 foundation with
the existing frontier_explorer as the sole high-level NavigateToPose owner.
It intentionally starts no fusion, allocator, unknown-pose, or cooperative
nodes.  Set ``frontier_autostart:=false`` for stationary graph validation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOT_NAMESPACE = "robot2"


def generate_launch_description():
    project_share = get_package_share_directory("my_epuck_project")
    local_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            project_share,
            "launch",
            "robot2_cooperative_local_slam_nav2_launch.py",
        )),
    )

    frontier = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        namespace=ROBOT_NAMESPACE,
        name="frontier_explorer",
        output="screen",
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        parameters=[
            os.path.join(
                project_share,
                "resource",
                "frontier_explorer_robot2_solo.yaml",
            ),
            {
                "use_sim_time": False,
                "map_topic": "/robot2/map",
                "costmap_topic": "/robot2/global_costmap/costmap",
                "local_costmap_topic": "/robot2/local_costmap/costmap",
                "navigate_to_pose_action_name": "navigate_to_pose",
                "global_frame": "robot2/map",
                "robot_base_frame": "robot2/base_link",
                "frontier_marker_topic": "/robot2/frontier_explorer/frontiers",
                "selected_frontier_topic": "/robot2/frontier_explorer/selected_frontier",
                "optimized_map_topic": "/robot2/frontier_explorer/optimized_map",
                "completion_event_topic": "/robot2/frontier_explorer/exploration_complete",
                "autostart": LaunchConfiguration("frontier_autostart"),
                "control_service_enabled": True,
                "completion_event_enabled": True,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "frontier_autostart",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Automatically begin solo frontier goal selection. Set false "
                "for stationary graph validation."
            ),
        ),
        local_stack,
        frontier,
    ])
