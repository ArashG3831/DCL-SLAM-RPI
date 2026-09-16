"""Conversions between assignment-core models and project ROS interfaces."""

from dataclasses import asdict
import json
import math
from typing import Iterable

from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Point

from my_epuck_interfaces.msg import (
    PairDecision as PairDecisionMsg,
    PhysicalTask as PhysicalTaskMsg,
    TaskBid as TaskBidMsg,
    TaskBidArray as TaskBidArrayMsg,
    TaskSnapshot as TaskSnapshotMsg,
)

from unique_identifier_msgs.msg import UUID

from .models import (
    Bid,
    BidBatch,
    Bounds,
    PairDecision,
    PhysicalTask,
    TaskSnapshot,
)


def uuid_to_text(value: UUID) -> str:
    """Convert a ROS UUID byte array to lowercase hexadecimal text."""
    return bytes(value.uuid).hex()


def text_to_uuid(value: str) -> UUID:
    """Convert 32 hexadecimal characters to a ROS UUID."""
    message = UUID()
    raw = bytes.fromhex(value.replace('-', ''))
    if len(raw) != 16:
        raise ValueError('session UUID must contain exactly 16 bytes')
    message.uuid = list(raw)
    return message


def duration_to_seconds(value: Duration) -> float:
    """Convert a ROS duration without mixing it with a clock epoch."""
    return float(value.sec) + float(value.nanosec) / 1_000_000_000.0


def seconds_to_duration(value: float) -> Duration:
    """Convert bounded positive seconds to a ROS duration."""
    value = max(0.0, value)
    seconds = int(math.floor(value))
    return Duration(sec=seconds, nanosec=round((value - seconds) * 1_000_000_000))


def _point(value: tuple[float, float]) -> Point:
    return Point(x=value[0], y=value[1], z=0.0)


def _points(values: Iterable[tuple[float, float]]) -> list[Point]:
    return [_point(value) for value in values]


def task_from_msg(message: PhysicalTaskMsg) -> PhysicalTask:
    """Build one immutable task from its wire representation."""
    visible_bounds = None
    if message.has_visible_bounds:
        visible_bounds = Bounds(
            (message.visible_bounds_min.x, message.visible_bounds_min.y),
            (message.visible_bounds_max.x, message.visible_bounds_max.y),
        )
    orientation = message.approach_pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )
    stamp = message.generation_stamp
    return PhysicalTask(
        source_robot_id=message.source_robot_id,
        source_session_id=uuid_to_text(message.source_session_id),
        source_snapshot_epoch=message.source_snapshot_epoch,
        source_map_revision=message.source_map_revision,
        physical_signature=message.physical_signature,
        local_frontier_id=message.local_frontier_id,
        centroid=(message.centroid.x, message.centroid.y),
        bounds=Bounds(
            (message.bounding_box_min.x, message.bounding_box_min.y),
            (message.bounding_box_max.x, message.bounding_box_max.y),
        ),
        approach=(message.approach_pose.pose.position.x,
                  message.approach_pose.pose.position.y),
        approach_yaw=yaw,
        frontier_geometry=tuple((point.x, point.y)
                                for point in message.frontier_geometry),
        visible_cells=tuple((point.x, point.y) for point in message.visible_cells),
        visible_bounds=visible_bounds,
        visible_reveal_gain=message.visible_reveal_gain,
        local_ordering_score=message.local_ordering_score,
        mrtsp_route_rank=int(message.mrtsp_route_rank),
        mrtsp_route_generation=int(message.mrtsp_route_generation),
        mrtsp_solver=str(message.mrtsp_solver),
        local_path_valid=message.local_path_valid,
        local_path_length_m=message.local_path_length_m,
        local_path=tuple((point.x, point.y)
                         for point in message.local_path_samples),
        planned_path=getattr(message, 'planned_path', None),
        path_heading_cost_rad=message.path_heading_cost_rad,
        generation_ros_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
    )


def snapshot_from_msg(message: TaskSnapshotMsg) -> TaskSnapshot:
    """Build a snapshot while retaining source-local ROS provenance."""
    stamp = message.task_generation_stamp
    return TaskSnapshot(
        source_robot_id=message.source_robot_id,
        source_session_id=uuid_to_text(message.source_session_id),
        epoch=message.source_snapshot_epoch,
        map_revision=message.source_map_revision,
        costmap_revision=int(getattr(message, 'source_costmap_revision', 0)),
        map_fingerprint=message.source_map_fingerprint,
        generation_ros_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
        validity_s=duration_to_seconds(message.validity),
        tasks=tuple(task_from_msg(item) for item in message.tasks),
        lower_bound_context_fingerprint=str(
            getattr(message, 'lower_bound_context_fingerprint', '') or ''
        ),
        candidate_generation_id=int(
            getattr(message, 'candidate_generation_id', 0) or 0
        ),
    )


def bid_from_msg(message: TaskBidMsg) -> Bid:
    """Convert one bounded path bid."""
    task_stamp = message.task_generation_stamp
    query_stamp = message.path_query_stamp
    return Bid(
        canonical_task_id=message.canonical_task_id,
        path_valid=message.path_valid,
        path_length_m=message.path_length_m,
        estimated_travel_cost=message.estimated_travel_cost,
        heading_cost=message.heading_cost,
        own_utility_contribution=message.own_utility_contribution,
        task_generation_ros_ns=task_stamp.sec * 1_000_000_000 + task_stamp.nanosec,
        path_query_ros_ns=query_stamp.sec * 1_000_000_000 + query_stamp.nanosec,
        path=tuple((point.x, point.y) for point in message.path_samples),
    )


def bid_batch_from_msg(message: TaskBidArrayMsg) -> BidBatch:
    """Convert one round-bound bid array."""
    return BidBatch(
        round_id=message.round_id,
        union_hash=message.union_hash,
        source_robot_id=message.source_robot_id,
        source_session_id=uuid_to_text(message.source_session_id),
        source_snapshot_epoch=message.source_snapshot_epoch,
        validity_s=duration_to_seconds(message.validity),
        bids=tuple(bid_from_msg(item) for item in message.bids),
    )


def bid_batch_to_msg(batch: BidBatch, stamp) -> TaskBidArrayMsg:
    """Serialize deterministic bids with bounded path samples."""
    message = TaskBidArrayMsg()
    message.header.stamp = stamp
    message.header.frame_id = 'shared_map'
    message.round_id = batch.round_id
    message.union_hash = batch.union_hash
    message.source_robot_id = batch.source_robot_id
    message.source_session_id = text_to_uuid(batch.source_session_id)
    message.source_snapshot_epoch = batch.source_snapshot_epoch
    message.validity = seconds_to_duration(batch.validity_s)
    for bid in batch.bids:
        item = TaskBidMsg()
        item.canonical_task_id = bid.canonical_task_id
        item.path_valid = bid.path_valid
        item.path_length_m = float(bid.path_length_m)
        item.estimated_travel_cost = float(bid.estimated_travel_cost)
        item.heading_cost = float(bid.heading_cost)
        item.own_utility_contribution = float(bid.own_utility_contribution)
        item.task_generation_stamp.sec = bid.task_generation_ros_ns // 1_000_000_000
        item.task_generation_stamp.nanosec = bid.task_generation_ros_ns % 1_000_000_000
        item.path_query_stamp.sec = bid.path_query_ros_ns // 1_000_000_000
        item.path_query_stamp.nanosec = bid.path_query_ros_ns % 1_000_000_000
        item.path_samples = _points(bid.path)
        message.bids.append(item)
    return message


def decision_to_msg(
        decision: PairDecision, source_robot_id: str, source_session_id: str,
        robot1_epoch: int, robot2_epoch: int, state: str, validity_s: float,
        stamp) -> PairDecisionMsg:
    """Serialize all fields required for positive replicated agreement."""
    message = PairDecisionMsg()
    message.header.stamp = stamp
    message.header.frame_id = 'shared_map'
    message.source_robot_id = source_robot_id
    message.source_session_id = text_to_uuid(source_session_id)
    message.round_id = decision.round_id
    message.union_hash = decision.union_hash
    message.robot1_snapshot_epoch = robot1_epoch
    message.robot2_snapshot_epoch = robot2_epoch
    message.robot1_bid_fingerprint = decision.robot1_bid_fingerprint
    message.robot2_bid_fingerprint = decision.robot2_bid_fingerprint
    message.robot1_canonical_task_id = decision.robot1_task_id
    message.robot2_canonical_task_id = decision.robot2_task_id
    message.total_team_score = float(decision.score.total)
    message.team_visible_gain = float(decision.score.team_visible_gain)
    message.combined_path_cost = float(decision.score.combined_path_cost)
    message.nearby_goal_penalty = float(decision.score.nearby_goal_penalty)
    message.route_overlap_penalty = float(decision.score.route_overlap_penalty)
    message.hard_failure_penalty = float(decision.score.hard_failure_penalty)
    message.sensing_overlap_penalty = float(decision.score.sensing_overlap_penalty)
    message.workload_imbalance_penalty = float(
        decision.score.workload_imbalance_penalty,
    )
    message.decision_hash = decision.decision_hash
    message.coordinator_state = state
    message.validity = seconds_to_duration(validity_s)
    message.diagnostics_json = json.dumps(
        asdict(decision.diagnostics), sort_keys=True, separators=(',', ':'),
    )
    return message
