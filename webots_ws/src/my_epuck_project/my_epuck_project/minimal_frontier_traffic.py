"""Adapt the existing traffic scheduler for the minimal coordinator.

This module preserves the scheduler's path geometry, priority, and evidence.
It only exposes the coordinator's wait/proceed query.
It does not add traffic geometry, lifecycle, history, replay, or reservations.
It does not alter messages, configuration, navigation, or scheduler behavior.
"""

from typing import Sequence

from .distributed_assignment import traffic_scheduler
from .distributed_assignment.models import Point
from .distributed_assignment.traffic_scheduler import TrafficDecision


def decide(
        r1_path: Sequence[Point], r2_path: Sequence[Point], *,
        robot1_safe_radius_m: float, robot2_safe_radius_m: float,
        reference_speed_mps: float, eta_tie_s: float = 0.05,
        active_robots: frozenset[str] = frozenset()) -> TrafficDecision:
    """Return the unchanged scheduler decision for the two route samples."""
    return traffic_scheduler.schedule_traffic(
        r1_path, r2_path,
        robot1_safe_radius_m=robot1_safe_radius_m,
        robot2_safe_radius_m=robot2_safe_radius_m,
        reference_speed_mps=reference_speed_mps,
        eta_tie_s=eta_tie_s,
        active_robots=active_robots,
    )


def should_wait(robot_id: str, decision: TrafficDecision) -> bool:
    """Return whether the scheduler designated ``robot_id`` to wait."""
    return decision.waiting_robot_id == robot_id
