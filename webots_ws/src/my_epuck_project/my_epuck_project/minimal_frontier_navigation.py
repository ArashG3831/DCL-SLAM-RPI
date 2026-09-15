"""Adapt current frontier candidates to the ordinary local Nav2 boundary.

This module creates the existing immutable ``PhysicalTask`` shape and forwards
dispatch checks, navigation, and cancellation to ``LocalNav2`` unchanged.
It owns no navigation state machine, distributed commitment, restart handling,
or UUID recovery.
"""

from __future__ import annotations

import math
from typing import Callable

from my_epuck_interfaces.msg import FrontierCandidate

from .distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    LocalNav2,
    NavigationOutcome,
)
from .distributed_assignment.models import Bounds, PhysicalTask, Point


def to_physical_task(
        candidate: FrontierCandidate, source_robot_id: str) -> PhysicalTask:
    """Convert one current frontier candidate to the existing task model."""
    orientation = candidate.approach_pose.pose.orientation
    approach = candidate.approach_pose.pose.position
    approach_yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )
    return PhysicalTask(
        source_robot_id=source_robot_id,
        source_session_id='',
        source_snapshot_epoch=0,
        source_map_revision=0,
        physical_signature=str(candidate.frontier_id),
        local_frontier_id=int(candidate.frontier_id),
        centroid=(candidate.centroid.x, candidate.centroid.y),
        bounds=Bounds(
            (candidate.bounding_box_min.x, candidate.bounding_box_min.y),
            (candidate.bounding_box_max.x, candidate.bounding_box_max.y),
        ),
        approach=(approach.x, approach.y),
        approach_yaw=approach_yaw,
        visible_reveal_gain=float(candidate.information_gain),
        local_ordering_score=float(candidate.score),
        mrtsp_route_rank=int(candidate.mrtsp_route_rank),
        mrtsp_route_generation=int(candidate.mrtsp_route_generation),
        mrtsp_solver=str(candidate.mrtsp_solver),
        local_path_valid=(candidate.reachability_state == candidate.REACHABLE),
        local_path_length_m=(candidate.local_path_length_m or candidate.path_length_m),
        local_path=tuple((point.x, point.y) for point in candidate.local_path_samples),
        path_heading_cost_rad=float(candidate.heading_change_rad),
    )


def check_preconditions(
        nav: LocalNav2, task: PhysicalTask, final_path_valid: bool,
        callback: Callable[[DispatchPreconditions], None],
        path_samples: tuple[Point, ...] = (), path_frame_id: str = '') -> None:
    """Forward the ordinary asynchronous local dispatch-precondition check."""
    nav.check_dispatch_preconditions(
        task, final_path_valid, callback, path_samples, path_frame_id,
    )


def send(
        nav: LocalNav2, task: PhysicalTask,
        callback: Callable[[NavigationOutcome], None],
        diagnostic_path: tuple[Point, ...] = ()) -> bool:
    """Forward one ordinary NavigateToPose goal and preserve its callback."""
    return nav.send_navigation(task, callback, diagnostic_path)


def cancel(nav: LocalNav2) -> bool:
    """Request cancellation through the existing local navigation primitive."""
    return nav.cancel_navigation()
