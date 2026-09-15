#!/usr/bin/env python3
"""Robot 2 resilient supervisor launch.

The supervisor starts the existing namespaced solo launch immediately. It does
not launch cooperative algorithms or a second Nav2 stack in the current
physical profile; late handoff is fail-closed until its contracts are closed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'frontier_autostart', default_value='true',
            choices=['true', 'false'],
            description='Enable solo frontier dispatch; false is no-motion smoke only.'),
        Node(
            package='my_epuck_project',
            executable='robot2_resilient_mode_manager',
            namespace='robot2',
            name='robot2_resilient_mode_manager',
            output='screen',
            parameters=[{
                'robot_id': 'robot2',
                'peer_robot_id': 'robot1',
                'peer_descriptor_topic': '/cslam/relative_pose/descriptors',
                'peer_timeout_s': 5.0,
                'handoff_timeout_s': 15.0,
                'zero_velocity_settle_s': 0.5,
                'late_handoff_supported': False,
                'solo_launch_package': 'my_epuck_project',
                'solo_launch_file': 'robot2_solo_frontier_launch.py',
                'solo_frontier_autostart': LaunchConfiguration('frontier_autostart'),
            }],
        ),
    ])
