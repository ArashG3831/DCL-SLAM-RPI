#!/usr/bin/env python3
"""Robot 2 physical custom frontier pipeline with minimal allocator solo mode.

This launch reuses the validated namespaced hardware/SLAM/Nav2 foundation and
adds only the custom candidate, proposal, and minimal allocator nodes.
The legacy autonomous owner and distributed fallback are excluded.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
            "path_valid_service": "/robot2/is_path_valid",
            "path_valid_preflight_enabled": False,
            "candidate_topic": "/robot2/frontier_candidates",
            "marker_topic": "/robot2/frontier_candidate_markers",
            "processing_rate_hz": 1.0,
            "grid_subscription_reliability": "reliable",
            "grid_subscription_durability": "transient_local",
            "frontier_map_optimization_enabled": True,
            "sigma_s": 2.0,
            "sigma_r": 30.0,
            "dilation_kernel_radius_cells": 2,
            "minimum_frontier_cells": 20,
            "minimum_frontier_length_m": 0.05,
            "stable_id_quantization_m": 0.05,
            "approach_clearance_m": 0.06,
            "frontier_goal_stepback_m": 0.20,
            "planner_tolerance_m": 0.0,
            "minimum_robot_distance_m": 0.08,
            "planner_id": "GridBased",
            "occupied_threshold": 50,
            "visible_gain_range_m": 12.0,
            "maximum_candidates_before_path_check": 10000,
            "maximum_path_queries_per_cycle": 10000,
            "path_query_timeout_s": 5.5,
            "cost_only_reference_linear_speed_mps": 0.13,
            "cost_only_reference_angular_speed_radps": 0.35,
            "selection_policy": "frontier_cost_only",
            "upstream_route_ordering_enabled": False,
            "handoff_gated": False,
            "stop_after_handoff": False,
            "event_driven_costing": False,
            "pause_planner_queries_while_navigating": True,
            "require_known_approach": True,
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
            "maximum_tasks": 10000,
            "validity_s": 8.0,
            "stop_after_handoff": False,
            "use_sim_time": False,
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
            "allow_solo_without_peer": True,
            "solo_startup_spin_enabled": True,
            "solo_startup_spin_angle_rad": 0.1745,
            "pause_planner_queries_while_navigating": True,
            "minimum_solo_visible_gain_m": 0.0,
            "dispatch_enabled": LaunchConfiguration("dispatch_enabled"),
            "max_navigation_goals": LaunchConfiguration(
                "max_navigation_goals",
            ),
            "peer_candidate_timeout_s": LaunchConfiguration(
                "peer_candidate_timeout_s",
            ),
            "global_frame": "robot2/map",
            "robot_base_frame": "robot2/base_link",
            "map_topic": "/robot2/map",
            "local_path_gate_mode": "MODE_B",
            "candidate_topic": "frontier_candidates",
            "common_start_release_required": False,
            "publish_cooperative_start_ready": False,
            "use_sim_time": False,
        }],
    )

    # Passive structured evidence only.  This is the verified legacy observer
    # from cooperative_migration_source; it has no publishers, action clients,
    # or motion-control path.  The runner supplies the current run directory
    # through environment variables so the observer artifact is nested inside
    # the same physical run artifact.
    observer = Node(
        package="my_epuck_project",
        executable="cooperative_experiment_logger",
        namespace=ROBOT_NAMESPACE,
        name="cooperative_experiment_logger",
        output="screen",
        sigterm_timeout="120.0",
        sigkill_timeout="30.0",
        parameters=[{
            "run_id": EnvironmentVariable(
                "ROBOT2_OBSERVER_RUN_ID", default_value="observer"),
            "output_root": EnvironmentVariable(
                "ROBOT2_OBSERVER_OUTPUT_ROOT",
                default_value="/home/robot1/robot2_frontier_exploration_results/observer_runs"),
            "launch_file": "robot2_custom_frontier_solo_launch.py",
            "experiment_condition": "robot2_stationary_observer",
            "robot_ids": [ROBOT_NAMESPACE],
            "robot_base_frame": "robot2/base_link",
            "global_frame": "robot2/map",
            "coverage_source": "local_map",
            "enable_local_map_capture": True,
            "enable_forensic_capture": False,
            "enable_contact_capture": False,
            "enable_rosout_collection": True,
            "diagnostic_frontier_capture": True,
            "enable_passive_rosbag": False,
            "enable_scientific_raw_capture": False,
            "enable_coverage_attribution": True,
            "enable_trajectory_overlap": True,
            "enable_console_status": True,
            "known_relative_transform": [0.0, 0.0, 0.0],
            "transform_source": "PHYSICAL_LOCAL_ONLY",
            "world_profile": "physical_robot2",
            "source_world_path": "",
            "installed_world_path": "",
            "world_dimensions": [0.0, 0.0],
            "robot_start_poses_json": "{}",
            "slam_resolution": 0.02,
            "fusion_resolution": 0.02,
            "global_costmap_resolution": 0.02,
            "local_costmap_resolution": 0.005,
            "lidar_maximum_range": 12.0,
            "coverage_attribution_resolution": 0.02,
            "initial_configuration_json": (
                '{"observer_mode":"stationary_physical_robot2",'
                '"dispatch_enabled":false,'
                '"allocator":"minimal_frontier_allocator",'
                '"frontier_explorer":false,'
                '"distributed_frontier_assignment":false}'
            ),
        }],
    )

    delayed_pipeline = TimerAction(
        period=LaunchConfiguration("costmap_readiness_delay_s"),
        actions=[candidate_generator, proposal_adapter, allocator],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "dispatch_enabled",
            default_value="false",
            choices=["true", "false"],
            description="Enable the local-only allocator's NavigateToPose dispatch.",
        ),
        DeclareLaunchArgument(
            "max_navigation_goals",
            default_value="0",
            description=(
                "Bound NavigateToPose dispatches for a controlled test; "
                "zero means unlimited."
            ),
        ),
        DeclareLaunchArgument(
            "peer_candidate_timeout_s",
            default_value="8.0",
            description="Freshness timeout for a real Robot 1 candidate stream.",
        ),
        DeclareLaunchArgument(
            "costmap_readiness_delay_s",
            default_value="5.0",
            description=(
                "Delay candidate generation until the physical Nav2 costmap "
                "has had time to publish warmed data."
            ),
        ),
        local_stack,
        observer,
        delayed_pipeline,
    ])
