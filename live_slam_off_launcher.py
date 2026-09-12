#!/usr/bin/env python3
"""Launch the uniquely named live scan-matching-OFF mapper.

This mirrors Slam Toolbox's online_async_launch.py lifecycle sequence while
allowing a second mapper in the same ROS domain without a node/TF collision.
It is started only by the opt-in phone-controller comparison mode.
"""

import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from live_slam_comparison import OFF_NODE_NAME


def make_description(params_path):
    node = LifecycleNode(
        parameters=[
            params_path,
            {"use_lifecycle_manager": False, "use_sim_time": False},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name=OFF_NODE_NAME,
        output="screen",
        namespace="",
        # Slam Toolbox 2.8.4 creates its map publisher during configuration,
        # before it reads the later map_name parameter.  Remap the absolute
        # publisher names explicitly so the two live mappers cannot share /map.
        remappings=[
            ("/map", "/map_off"),
            ("/map_metadata", "/map_off_metadata"),
        ],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LiveComparison] OFF mapper is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    return LaunchDescription([node, configure, activate])


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} PARAMS_YAML", file=sys.stderr)
        return 2
    params_path = os.path.abspath(os.path.expanduser(sys.argv[1]))
    launch_service = LaunchService()
    launch_service.include_launch_description(make_description(params_path))
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
