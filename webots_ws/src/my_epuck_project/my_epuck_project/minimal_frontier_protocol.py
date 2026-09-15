"""Convert minimal coordinator bid batches at the ROS boundary.

This module binds batches to the deterministic union token and reuses the
existing TaskBidArray conversion.  It checks only complete matching vectors;
freshness, session safety, lifecycle, replay, and allocator policy remain
outside this Stage B boundary.
"""

from builtin_interfaces.msg import Time
from my_epuck_interfaces.msg import TaskBidArray

from .distributed_assignment.models import Bid, BidBatch, CanonicalUnion
from .distributed_assignment.ros_conversion import (
    bid_batch_from_msg,
    bid_batch_to_msg,
)


def make_batch(
        source_robot_id: str, source_session_id: str,
        source_snapshot_epoch: int, union: CanonicalUnion,
        bids: tuple[Bid, ...], validity_s: float) -> BidBatch:
    """Create a union-bound bid batch."""
    return BidBatch(
        round_id=union.union_hash,
        union_hash=union.union_hash,
        source_robot_id=source_robot_id,
        source_session_id=source_session_id,
        source_snapshot_epoch=source_snapshot_epoch,
        validity_s=validity_s,
        bids=bids,
    )


def to_msg(batch: BidBatch, stamp: Time) -> TaskBidArray:
    """Serialize a bid batch using the existing ROS conversion."""
    return bid_batch_to_msg(batch, stamp)


def from_msg(message: TaskBidArray) -> BidBatch:
    """Deserialize a TaskBidArray using the existing ROS conversion."""
    return bid_batch_from_msg(message)


def complete_pair(union: CanonicalUnion, r1: BidBatch, r2: BidBatch) -> bool:
    """Return whether both batches completely match the canonical union."""
    expected = {task.canonical_id for task in union.tasks}
    for batch in (r1, r2):
        ids = {bid.canonical_task_id for bid in batch.bids}
        if (batch.round_id != union.union_hash or
                batch.union_hash != union.union_hash or
                len(batch.bids) != len(expected) or ids != expected):
            return False
    return True
