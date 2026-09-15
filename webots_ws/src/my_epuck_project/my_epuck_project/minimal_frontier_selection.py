"""Adapt the existing pure pair scorer for the minimal coordinator.

This module binds the scorer to the canonical union hash and cost-only mode.
It normalizes the scorer's selected task IDs for the coordinator boundary.
It preserves both-idle pair selection and one-busy solo selection.
It excludes a busy robot's active goal from the other robot's pair.
It does not implement rounds, exchange, traffic, navigation, or lifecycle.
"""

from .distributed_assignment.models import BidBatch, CanonicalUnion
from .distributed_assignment.scoring import choose_pair_assignment


def choose_assignment(
        union: CanonicalUnion, r1: BidBatch, r2: BidBatch, *,
        active_robot1_id: str = '', active_robot2_id: str = '',
        ) -> tuple[str, str] | None:
    """Return the normalized pair assignment, or no-op when both are busy."""
    active1 = str(active_robot1_id) if active_robot1_id else ''
    active2 = str(active_robot2_id) if active_robot2_id else ''
    if active1 and active2:
        return None
    decision = choose_pair_assignment(
        round_id=union.union_hash,
        union=union,
        robot1_bids=r1,
        robot2_bids=r2,
        hard_failed_tasks=frozenset(),
        scoring_mode='frontier_cost_only',
        fixed_robot1_task_id=active1,
        fixed_robot2_task_id=active2,
    )
    selected1 = '' if active1 else str(decision.robot1_task_id or '')
    selected2 = '' if active2 else str(decision.robot2_task_id or '')
    return selected1, selected2
