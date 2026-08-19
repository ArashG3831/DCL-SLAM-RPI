#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('my_epuck_project')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description_path = os.path.join(package_dir, 'resource', 'epuck_webots.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
    )

    # Webots launcher + supervisor (for /clock)
    webots = WebotsLauncher(
        world=PathJoinSubstitution([
            TextSubstitution(text=package_dir),
            'worlds',
            world,
        ]),
        ros2_supervisor=True,
    )

    # ros2_control controller spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    # ros2_control config
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')

    # Map diffdrive topics to generic cmd_vel / odom
    mappings = [
        ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
        ('/diffdrive_controller/odom', '/odom'),
    ]

    # WebotsController driver for e-puck
    epuck_driver = WebotsController(
        robot_name='e-puck',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # Epuck node (sensors, scan, etc.)
    epuck_process = Node(
        package='webots_ros2_epuck',
        executable='epuck_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    twist_stamper_node = Node(
        package='my_epuck_project',
        executable='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Start controllers only after driver connected
    waiting_nodes = WaitForControllerConnection(
        target_driver=epuck_driver,
        nodes_to_start=ros_control_spawners,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_world.wbt',
            description='World file name under my_epuck_project/worlds',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock',
        ),

        webots,
        webots._supervisor,   # this is what made /clock work for you earlier
        robot_state_publisher,
        epuck_driver,
        epuck_process,
	twist_stamper_node,
        waiting_nodes,
    ])
