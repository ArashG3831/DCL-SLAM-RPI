#!/usr/bin/env python3
"""Passive two-frontend unknown-pose validation harness.

Local Robot 1 and Robot 2 SLAM/Nav2/frontier stacks are intentionally external
inputs to this harness.  This workspace contains only the Robot 2 hardware
launch, so including it twice would duplicate Robot 2 hardware.  The harness
starts the two estimator instances and one bounded acceptance recorder; it
does not start navigation, allocation, phase switching, shared fusion, or
ground-truth machinery.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _frontend(robot_id, peer_id):
    return Node(
        package='my_epuck_project',
        executable='unknown_pose_frontend',
        namespace=robot_id,
        name='unknown_pose_frontend',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_id': robot_id,
            'peer_robot_id': peer_id,
            'map_topic': f'/{robot_id}/map',
            'descriptor_topic': '/cslam/relative_pose/descriptors',
            'crop_request_topic': '/cslam/relative_pose/crop_requests',
            'crop_topic': '/cslam/relative_pose/crops',
            'hypothesis_topic': '/cslam/relative_pose/hypotheses',
            'peer_map_topic': f'/cslam/unknown_pose/{robot_id}/local_map',
            'full_map_registration': True,
            'full_map_registration_period_s': 1.0,
            'full_map_max_snapshots': 64,
            'peer_map_publish_period_s': 5.0,
            'shared_frame': 'shared_map',
            'diagnostic_output': LaunchConfiguration('diagnostic_output'),
            'registration_backend': 'legacy',
            'max_verification_batches': 4,
            'verification_lifetime_s': 600.0,
        }],
    )


def generate_launch_description():
    observer = Node(
        package='my_epuck_project',
        executable='unknown_pose_validation_observer',
        name='unknown_pose_validation_observer',
        output='screen',
        parameters=[{
            'output_path': LaunchConfiguration('output_path'),
            'source_robot_id': 'robot1',
            'target_robot_id': 'robot2',
            'source_map_topic': '/robot1/map',
            'target_map_topic': '/robot2/map',
            'source_map_frame': 'robot1/map',
            'target_map_frame': 'robot2/map',
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('diagnostic_output', default_value=''),
        DeclareLaunchArgument('output_path', default_value=''),
        _frontend('robot1', 'robot2'),
        _frontend('robot2', 'robot1'),
        observer,
    ])
