#!/usr/bin/env python3
"""Namespaced Robot 2 cooperative hardware, local SLAM, and Nav2 mode.

This is deliberately separate from the validated single-robot launcher.  It
starts no frontier explorer, allocator, Webots process, AMCL, or static map
server.  The existing physical hardware, SLAM settings, Nav2 controller, and
motor safety behavior are reused with only topic/frame namespace bindings.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


ROBOT_NAMESPACE = "robot2"
ROBOT_MAP = "robot2/map"
ROBOT_ODOM = "robot2/odom"
ROBOT_BASE = "robot2/base_link"
ROBOT_LIDAR = "robot2/d500_lidar"
ROBOT_SCAN_TOPIC = "/robot2/scan"
ROBOT_ODOM_TOPIC = "/robot2/odom"
ROBOT_CMD_VEL_TOPIC = "/robot2/cmd_vel"
ROBOT_CMD_VEL_UNSTAMPED_TOPIC = "/robot2/cmd_vel_unstamped"


def generate_launch_description():
    project_share = get_package_share_directory("my_epuck_project")
    slam_params = os.path.join(
        project_share, "resource", "slam_toolbox_robot2_cooperative.yaml")
    nav2_params = os.path.join(
        project_share, "resource", "nav2_robot2_cooperative.yaml")
    bt_xml = os.path.join(
        project_share, "resource", "navigate_to_pose_no_backup.xml")

    motor = Node(
        package="my_epuck_project_cpp",
        executable="real_diffdrive_node_cpp",
        namespace=ROBOT_NAMESPACE,
        name="real_diffdrive_node",
        output="screen",
        parameters=[{
            "odom_topic": ROBOT_ODOM_TOPIC,
            "cmd_vel_topic": ROBOT_CMD_VEL_TOPIC,
            "cmd_vel_unstamped_topic": ROBOT_CMD_VEL_UNSTAMPED_TOPIC,
            "odom_frame": ROBOT_ODOM,
            "base_frame": ROBOT_BASE,
        }],
        remappings=[
            ("/motor_safety/fault", "/robot2/motor_safety/fault"),
        ],
    )

    lidar = Node(
        package="my_epuck_project_cpp",
        executable="d500_ros2_scan_cpp",
        namespace=ROBOT_NAMESPACE,
        name="d500_ros2_scan",
        output="screen",
        parameters=[{
            "topic": ROBOT_SCAN_TOPIC,
            "frame_id": ROBOT_LIDAR,
            "bins": 720,
            "min_mm": 30,
            "max_mm": 12000,
        }],
    )

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot2_d500_lidar_static_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.07",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", ROBOT_BASE,
            "--child-frame-id", ROBOT_LIDAR,
        ],
    )

    slam_autostart = LaunchConfiguration("slam_autostart")
    slam_use_lifecycle_manager = LaunchConfiguration("slam_use_lifecycle_manager")
    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace=ROBOT_NAMESPACE,
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_lifecycle_manager": slam_use_lifecycle_manager,
                "use_sim_time": False,
            },
        ],
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(AndSubstitution(
            slam_autostart, NotSubstitution(slam_use_lifecycle_manager))),
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(AndSubstitution(
            slam_autostart, NotSubstitution(slam_use_lifecycle_manager))),
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        namespace=ROBOT_NAMESPACE,
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        namespace=ROBOT_NAMESPACE,
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )
    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        namespace=ROBOT_NAMESPACE,
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        namespace=ROBOT_NAMESPACE,
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params, {"default_nav_to_pose_bt_xml": bt_xml}],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=ROBOT_NAMESPACE,
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav2_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "slam_autostart",
            default_value="true",
            description="Configure and activate the cooperative local SLAM node.",
        ),
        DeclareLaunchArgument(
            "slam_use_lifecycle_manager",
            default_value="false",
            description="Keep false to use the explicit local SLAM lifecycle events.",
        ),
        motor,
        lidar,
        lidar_tf,
        slam,
        slam_configure,
        slam_activate,
        controller,
        planner,
        behavior,
        bt_navigator,
        lifecycle_manager,
    ])
