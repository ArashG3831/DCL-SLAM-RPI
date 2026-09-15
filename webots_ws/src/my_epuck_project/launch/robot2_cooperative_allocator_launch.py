#!/usr/bin/env python3
"""Robot 2 cooperative allocator mode with navigation dispatch disabled by inputs.

This launch is deliberately separate from the validated single-robot launcher.
It starts the namespaced local hardware/SLAM/Nav2 foundation, the reusable
candidate generator, the proposal adapter, and the canonical allocator.  It
does not start frontier_explorer, a peer fixture, a release publisher, fusion,
or any other goal owner.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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

    candidate_generator = Node(
        package="my_epuck_frontier_candidates",
        executable="frontier_candidate_generator",
        namespace=ROBOT_NAMESPACE,
        name="frontier_candidate_generator",
        output="screen",
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        parameters=[{
            "robot_id": ROBOT_NAMESPACE,
            "map_topic": "/robot2/map",
            "global_costmap_topic": "/robot2/global_costmap/costmap",
            "global_frame": "robot2/map",
            "robot_base_frame": "robot2/base_link",
            "compute_path_action": "/robot2/compute_path_to_pose",
            "candidate_topic": "/robot2/frontier_candidates",
            "marker_topic": "/robot2/frontier_candidate_markers",
            "processing_rate_hz": 0.5,
            "grid_subscription_reliability": "reliable",
            "grid_subscription_durability": "transient_local",
            "frontier_map_optimization_enabled": True,
            "sigma_s": 2.0,
            "sigma_r": 30.0,
            "dilation_kernel_radius_cells": 2,
            "minimum_frontier_cells": 5,
            "minimum_frontier_length_m": 0.05,
            "stable_id_quantization_m": 0.05,
            "approach_clearance_m": 0.15,
            "minimum_robot_distance_m": 0.08,
            "planner_id": "GridBased",
            "occupied_threshold": 50,
            "visible_gain_range_m": 12.0,
            "maximum_candidates_before_path_check": 10000,
            "maximum_path_queries_per_cycle": 10000,
            "path_query_timeout_s": 1.0,
            "selection_policy": "frontier_cost_only",
            "upstream_route_ordering_enabled": False,
            "handoff_gated": False,
            "stop_after_handoff": False,
            "event_driven_costing": False,
            "use_sim_time": False,
        }],
    )

    proposal_adapter = Node(
        package="my_epuck_project",
        executable="frontier_proposal_adapter",
        namespace=ROBOT_NAMESPACE,
        name="frontier_proposal_adapter",
        output="screen",
        parameters=[{
            "robot_id": ROBOT_NAMESPACE,
            "candidate_topic": "frontier_candidates",
            "task_snapshot_topic": "task_snapshot",
            "stop_after_handoff": False,
        }],
    )

    allocator = Node(
        package="my_epuck_project",
        executable="minimal_frontier_allocator",
        namespace=ROBOT_NAMESPACE,
        name="minimal_frontier_allocator",
        output="screen",
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        parameters=[{
            "robot_id": ROBOT_NAMESPACE,
            "global_frame": "robot2/map",
            "robot_base_frame": "robot2/base_link",
            "map_topic": "/robot2/map",
            "candidate_topic": "frontier_candidates",
            "nav2_node_prefix": "",
            # The authoritative coordinator has no dispatch_enabled
            # parameter.  The physical no-motion smoke remains fail-closed by
            # requiring the existing START_RELEASE while Robot 1 is absent.
            "common_start_release_required": True,
            "publish_cooperative_start_ready": False,
        }],
    )

    delayed_pipeline = TimerAction(
        period=LaunchConfiguration("costmap_readiness_delay_s"),
        actions=[candidate_generator, proposal_adapter, allocator],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "costmap_readiness_delay_s",
            default_value="5.0",
            description=(
                "Delay candidate/coordinator startup until the physical "
                "Nav2 costmap has had time to publish warmed data."
            ),
        ),
        local_stack,
        delayed_pipeline,
    ])
