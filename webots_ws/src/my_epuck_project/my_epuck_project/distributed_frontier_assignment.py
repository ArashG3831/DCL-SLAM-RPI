"""Replicated peer-to-peer two-robot frontier assignment ROS node."""

from collections import deque
from dataclasses import dataclass, field, replace
import hashlib
import json
import math
import os
import time
from typing import Optional

from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    ExplorationFailure,
    FrontierCandidateArray,
    PairDecision as PairDecisionMsg,
    TaskBidArray as TaskBidArrayMsg,
    TaskSnapshot as TaskSnapshotMsg,
    RelativePoseHypothesis,
)
from std_msgs.msg import Bool, String

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .distributed_assignment.canonical import (
    build_canonical_union,
    canonical_round_id,
    equivalent_tasks,
    TaskIdentity,
)
from .distributed_assignment.failures import (
    HARD_FAILURES, bounded_suppression_duration,
)
from .distributed_assignment.local_nav2 import (
    classify_dispatch_precondition_failure,
    DispatchPreconditions,
    LocalNav2,
    NavigationOutcome,
    PathEvaluation,
    path_is_valid_finite,
)
from .distributed_assignment.models import (
    Bid,
    BidBatch,
    CanonicalTask,
    CanonicalUnion,
    CoordinatorState,
    FailureClass,
    PairDecision,
    PhysicalTask,
    TaskSnapshot,
)
from .distributed_assignment.protocol import (
    bid_batch_valid,
    CommittedRound,
    PeerLiveness,
    receive,
    Received,
    SnapshotLedger,
)
from .distributed_assignment.ros_conversion import (
    bid_batch_from_msg,
    bid_batch_to_msg,
    decision_to_msg,
    duration_to_seconds,
    seconds_to_duration,
    snapshot_from_msg,
    text_to_uuid,
    uuid_to_text,
)
from .distributed_assignment.scoring import (
    AssignmentWeights,
    choose_pair_assignment,
    choose_mrtsp_route_assignment,
    cost_only_dispatch_certificate,
    nominal_motion_cost_s,
    route_overlap,
    rank_solo_tasks,
)
from .distributed_assignment.traffic_scheduler import (
    TrafficDecision,
    project_path_progress,
    schedule_traffic,
)


DISTRIBUTED_EVENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


from .round_lifecycle import RoundGeneration
from .mission_termination import (
    CandidateEvidence,
    FrontierRegionEvidence,
    TerminalReason,
    classify_empty_frontiers,
    summarize_frontier_regions,
    credible_planner_infrastructure_failure,
    terminal_reason_is_success,
)


def dispatch_delay_elapsed(start_wall_s: float, now_wall_s: float,
                           delay_s: float) -> bool:
    """Return whether the optional pre-handoff dispatch hold has elapsed.

    The hold is a bounded evidence-acquisition aid for unknown-pose runs.  It
    is evaluated against monotonic wall time so a zero simulation clock or a
    paused Webots startup cannot accidentally release navigation early.
    """
    return delay_s <= 0.0 or now_wall_s - start_wall_s >= delay_s


def evidence_hold_active(lease_until_wall_s: float,
                         now_wall_s: float) -> bool:
    """Return whether a live evidence-opportunity lease blocks new goals."""
    return (math.isfinite(float(lease_until_wall_s)) and
            float(lease_until_wall_s) > float(now_wall_s))


def solo_retry_delay_s(retry_count: int) -> float:
    """Return bounded exponential delay for retryable local failures."""
    count = max(1, int(retry_count))
    return min(30.0, 2.0 ** min(count - 1, 5))


def lower_bound_context_matches(
        provenance: Optional[tuple[int, str, int]], snapshot: TaskSnapshot) -> bool:
    """Return whether compact bounds belong to this exact task snapshot."""
    return bool(
        provenance is not None and
        len(provenance) == 3 and
        provenance[0] == snapshot.map_revision and
        bool(provenance[1]) and
        provenance[1] == snapshot.lower_bound_context_fingerprint and
        int(provenance[2]) > 0 and
        int(getattr(snapshot, 'candidate_generation_id', 0) or 0) > 0 and
        int(provenance[2]) == int(snapshot.candidate_generation_id)
    )


def completion_evidence_matches_snapshots(
        candidate_metadata: Optional[dict[str, dict]],
        snapshots: tuple[TaskSnapshot, ...],
) -> bool:
    """Require candidate evidence to belong to the fresh snapshot pair.

    Candidate evidence is a terminal-proof input, not an indefinitely retained
    diagnostic.  The candidate generation, map, and costmap provenance must
    match the corresponding fresh task snapshots before it can support an
    irreversible completion decision.
    """
    metadata = candidate_metadata or {}
    if len(snapshots) != 2:
        return False
    for snapshot in snapshots:
        source = str(snapshot.source_robot_id)
        item = metadata.get(source)
        if not item:
            return False
        candidate_generation_id = int(
            getattr(snapshot, 'candidate_generation_id', 0) or 0)
        if candidate_generation_id <= 0:
            return False
        if int(item.get('candidate_generation_id', 0) or 0) != candidate_generation_id:
            return False
        if int(item.get('map_revision', 0) or 0) != int(snapshot.map_revision):
            return False
        if int(item.get('costmap_revision', 0) or 0) != int(
                getattr(snapshot, 'costmap_revision', 0) or 0):
            return False
    return True


CERTIFICATE_EVIDENCE_REASONS = frozenset({
    'OK', 'NO_CANDIDATE_BOUND_SUMMARY', 'EMPTY_BOUND_SUMMARY',
    'NONFINITE_BOUND', 'INCOMPLETE_BOUND_SET', 'MAP_REVISION_MISMATCH',
    'COSTMAP_REVISION_MISMATCH', 'FINGERPRINT_MISMATCH',
    'SESSION_EPOCH_MISMATCH', 'TASK_CANDIDATE_GENERATION_MISMATCH',
    'SOURCE_UNHEALTHY', 'OTHER',
})


LOCAL_PATH_EVALUATION_CACHE_MAX_ENTRIES = 32


# Candidate arrays and task snapshots arrive on independent subscriptions.
# Keep a small exact-context history so a certificate can join a retained
# snapshot to the matching candidate evidence without ever mixing contexts.
# This is not a retry/cache policy: entries are usable only for the exact
# map/fingerprint/costmap/generation tuple requested by the snapshot.
CANDIDATE_BOUND_CONTEXT_HISTORY_MAX_ENTRIES = 32


def classify_lower_bound_evidence(
        candidate_meta: Optional[dict], snapshot: Optional[TaskSnapshot],
        raw_bounds: Optional[tuple[float, ...]],
        expected_count: int) -> tuple[str, dict]:
    """Classify certificate evidence without changing certificate behavior.

    This is deliberately diagnostic-only.  The production certificate still
    uses its existing ``lower_bound_context_matches`` gate; this helper merely
    records which existing input made that gate conservative.
    """
    has_candidate_summary = candidate_meta is not None
    meta = candidate_meta or {}
    candidate_fp = str(meta.get('fingerprint', '') or '')
    candidate_map = meta.get('map_revision')
    candidate_costmap = meta.get('costmap_revision')
    candidate_generation = meta.get('generation_ros_ns')
    candidate_generation_id = int(meta.get('candidate_generation_id', 0) or 0)
    candidate_session = meta.get('source_session_id')
    candidate_epoch = meta.get('source_epoch')
    task_fp = ('' if snapshot is None else
               str(getattr(snapshot, 'lower_bound_context_fingerprint', '') or ''))
    task_map = None if snapshot is None else int(getattr(snapshot, 'map_revision', 0))
    task_costmap = (None if snapshot is None else
                    int(getattr(snapshot, 'costmap_revision', 0)))
    task_generation = (None if snapshot is None else
                       int(getattr(snapshot, 'generation_ros_ns', 0)))
    task_generation_id = (0 if snapshot is None else int(
        getattr(snapshot, 'candidate_generation_id', 0) or 0))
    task_session = (None if snapshot is None else snapshot.source_session_id)
    task_epoch = None if snapshot is None else int(snapshot.epoch)
    fingerprints_equal = candidate_fp == task_fp
    map_equal = (candidate_map is not None and task_map is not None and
                 int(candidate_map) == int(task_map))
    costmap_comparable = candidate_costmap is not None and task_costmap is not None
    costmaps_equal = (costmap_comparable and
                      int(candidate_costmap) == int(task_costmap))
    bound_count = int(meta.get('bound_entry_count', 0) or 0)
    finite = bool(meta.get('all_bounds_finite', False))
    state = str(meta.get('bound_state', '') or '')

    if not has_candidate_summary:
        reason = 'NO_CANDIDATE_BOUND_SUMMARY'
    elif snapshot is None:
        reason = 'OTHER'
    elif state == 'NO_CANDIDATE_BOUND_SUMMARY':
        reason = state
    elif state == 'EMPTY_BOUND_SUMMARY':
        reason = 'EMPTY_BOUND_SUMMARY'
    elif state in ('NONFINITE_BOUND', 'INCOMPLETE_BOUND_SET', 'OTHER'):
        reason = state
    elif ((candidate_session is not None and candidate_session != task_session) or
          (candidate_epoch is not None and int(candidate_epoch) != task_epoch)):
        reason = 'SESSION_EPOCH_MISMATCH'
    elif (candidate_generation_id <= 0 or task_generation_id <= 0 or
          candidate_generation_id != task_generation_id):
        reason = 'TASK_CANDIDATE_GENERATION_MISMATCH'
    elif expected_count > bound_count:
        reason = 'INCOMPLETE_BOUND_SET'
    elif raw_bounds is not None and not all(
            math.isfinite(float(value)) and float(value) >= 0.0
            for value in raw_bounds):
        reason = 'NONFINITE_BOUND'
    elif not map_equal:
        reason = 'MAP_REVISION_MISMATCH'
    elif costmap_comparable and not costmaps_equal:
        reason = 'COSTMAP_REVISION_MISMATCH'
    elif not candidate_fp or not task_fp or not fingerprints_equal:
        reason = 'FINGERPRINT_MISMATCH'
    elif (candidate_generation and task_generation and
          int(candidate_generation) != int(task_generation)):
        reason = 'TASK_CANDIDATE_GENERATION_MISMATCH'
    else:
        reason = 'OK'
    if reason not in CERTIFICATE_EVIDENCE_REASONS:
        reason = 'OTHER'
    comparison = {
        'candidate': {
            'lower_bound_context_fingerprint': candidate_fp,
            'map_revision': candidate_map,
            'costmap_revision': candidate_costmap,
            'source_session_id': meta.get('source_session_id'),
            'source_epoch': meta.get('source_epoch'),
            'generation_ros_ns': candidate_generation,
            'candidate_generation_id': candidate_generation_id,
            'bound_entry_count': bound_count,
            'expected_bound_entry_count': int(expected_count),
            'detected_not_queried_count': int(
                meta.get('detected_not_queried_count', 0) or 0),
        },
        'task_snapshot': {
            'lower_bound_context_fingerprint': task_fp,
            'map_revision': task_map,
            'costmap_revision': task_costmap,
            'source_session_id': (None if snapshot is None else
                                  snapshot.source_session_id),
            'source_epoch': (None if snapshot is None else snapshot.epoch),
            'generation_ros_ns': task_generation,
            'candidate_generation_id': task_generation_id,
        },
        'fingerprints_equal': fingerprints_equal,
        'map_revisions_equal': map_equal,
        'costmap_revisions_equal': (costmaps_equal
                                    if costmap_comparable else None),
        'revisions_equal': bool(map_equal and (
            costmaps_equal if costmap_comparable else True)),
        'bound_set_complete': bool(
            bound_count == int(expected_count) and finite),
        'reason': reason,
    }
    return reason, comparison


@dataclass
class InitialExplorationBarrier:
    """Replicated mutual-readiness gate for the two-robot local phase.

    This is deliberately a small state machine, independent of ROS transport.
    Each local allocator owns one instance and observes the same two readiness
    facts from its own local candidate pipeline and the peer's readiness
    announcement.  Handoff supersedes the gate while it is still closed.
    """

    enabled: bool
    local_ready: bool = False
    peer_ready: bool = False
    handoff_complete: bool = False
    released: bool = False

    def __post_init__(self) -> None:
        # A single-robot/known-pose instance bypasses the two-peer gate
        # explicitly; it must not wait for a nonexistent peer.
        self.released = not bool(self.enabled)

    def observe_local_ready(self) -> None:
        """Record local readiness idempotently."""
        self.local_ready = True

    def observe_peer_ready(self) -> None:
        """Record peer readiness idempotently."""
        self.peer_ready = True

    def observe_handoff(self) -> None:
        """Make an accepted canonical handoff supersede local exploration."""
        self.handoff_complete = True

    def maybe_release(self) -> bool:
        """Release exactly once when both ready and handoff has not won."""
        if (self.released or not self.enabled or self.handoff_complete or
                not (self.local_ready and self.peer_ready)):
            return False
        self.released = True
        return True

    @property
    def dispatch_allowed(self) -> bool:
        """Return whether local pre-handoff goal dispatch is permitted."""
        return self.released and not self.handoff_complete


@dataclass
class RoundWork:
    """Mutable bounded work for one uncommitted canonical round."""

    round_id: str
    union: CanonicalUnion
    snapshots: tuple[TaskSnapshot, TaskSnapshot]
    query_tasks: tuple[CanonicalTask, ...]
    content_fingerprint: str = ''
    query_index: int = 0
    bids: tuple[Bid, ...] = ()
    local_path_evaluations: dict[str, PathEvaluation] = field(default_factory=dict)
    local_batch: Optional[BidBatch] = None
    decision: Optional[PairDecision] = None
    traffic: Optional[TrafficDecision] = None
    decision_published: bool = False
    mode: str = 'normal'
    continuation_free_robot_id: str = ''
    continuation_busy_robot_id: str = ''
    continuation_commitment_id: str = ''


@dataclass(frozen=True)
class ActiveCommitment:
    """Replicated immutable description of one dispatched cooperative goal."""

    robot_id: str
    source_session_id: str
    source_snapshot_epoch: int
    canonical_id: str
    task: CanonicalTask
    decision_round_id: str
    decision_hash: str
    path: tuple[tuple[float, float], ...] = ()
    path_length_m: float = 0.0
    heading_cost_rad: float = 0.0
    commitment_id: str = ''


@dataclass
class ActiveNavigationAction:
    """Action-bound ownership that outlives an invalidated auction round."""

    action_id: str
    task: CanonicalTask
    canonical_task_id: str
    physical_signature: str
    round_id: str
    decision_hash: str
    generation: int
    path: tuple[tuple[float, float], ...] = ()
    state: str = 'PENDING_SEND'
    commitment_id: str = ''


@dataclass(frozen=True)
class ContinuationContext:
    """One free robot plus one still-active immutable peer commitment."""

    free_robot_id: str
    busy_robot_id: str
    free_snapshot: TaskSnapshot
    commitment: ActiveCommitment
    free_tasks: tuple[CanonicalTask, ...]


def round_pass_is_current(current_round, expected_round) -> bool:
    """Return whether a threaded tick still owns its round snapshot."""
    return current_round is expected_round


def normal_round_requires_replacement(
        current_round, first: TaskSnapshot, second: TaskSnapshot,
        allocation_fingerprint: str) -> bool:
    """Replace normal work only when its semantic allocation problem changed.

    Snapshot epochs remain protocol freshness/version evidence, but an epoch-only
    update does not change the immutable task problem represented by an active
    round.  Session changes and semantic content changes do.
    """
    if current_round is None:
        return True
    if getattr(current_round, 'mode', 'normal') != 'normal':
        return True
    current_snapshots = getattr(current_round, 'snapshots', ())
    if len(current_snapshots) != 2:
        return True
    current_sessions = tuple(
        snapshot.source_session_id for snapshot in current_snapshots
    )
    incoming_sessions = (first.source_session_id, second.source_session_id)
    return (
        current_sessions != incoming_sessions or
        current_round.content_fingerprint != allocation_fingerprint
    )


def normal_round_provenance_rebase_required(
        current_round, first: TaskSnapshot, second: TaskSnapshot,
        canonical_round: str, allocation_fingerprint: str) -> bool:
    """Detect an unreconciled epoch split in an uncommitted normal round.

    Epoch-only updates normally do not replace a semantic round.  A round can
    nevertheless become unreconcilable when one replica forms it before an
    epoch update and the peer forms it afterward.  This predicate identifies
    only that pre-decision, same-session, forward-epoch case; strict bid
    provenance validation remains the authority for accepting evidence.
    """
    if (current_round is None or
            getattr(current_round, 'mode', 'normal') != 'normal' or
            getattr(current_round, 'decision', None) is not None or
            current_round.content_fingerprint != allocation_fingerprint or
            current_round.round_id == canonical_round):
        return False
    current_snapshots = getattr(current_round, 'snapshots', ())
    if len(current_snapshots) != 2:
        return False
    incoming = (first, second)
    current_identity = tuple(
        (snapshot.source_robot_id, snapshot.source_session_id,
         int(snapshot.epoch))
        for snapshot in current_snapshots
    )
    incoming_identity = tuple(
        (snapshot.source_robot_id, snapshot.source_session_id,
         int(snapshot.epoch))
        for snapshot in incoming
    )
    if any(
            current[0] != latest[0] or current[1] != latest[1]
            for current, latest in zip(current_identity, incoming_identity)
    ):
        # Session changes and non-canonical ordering are handled by the
        # ordinary replacement path, not this provenance-only repair.
        return False
    return (
        all(latest[2] >= current[2]
            for current, latest in zip(current_identity, incoming_identity)) and
        any(latest[2] > current[2]
            for current, latest in zip(current_identity, incoming_identity))
    )


def selector_feasibility_identity(
        union_task_ids, hard_failed_task_ids, completed_task_ids,
        peer_reservation_task_ids) -> tuple[str, dict[str, tuple[str, ...]]]:
    """Return deterministic identity for the selector's suppression inputs.

    Bid/union fingerprints do not include local completion, hard-failure, or
    temporary peer-reservation suppression.  Those sets are part of the pure
    selector input and therefore must be bound before replicas can accept a
    pair decision as comparable.
    """
    union_ids = {str(item) for item in union_task_ids if str(item)}

    def selected(values) -> tuple[str, ...]:
        return tuple(sorted(union_ids.intersection(str(item) for item in values)))

    payload = {
        'completed': selected(completed_task_ids),
        'hard_failed': selected(hard_failed_task_ids),
        'peer_reservations': selected(peer_reservation_task_ids),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest(), {
        key: tuple(value) for key, value in payload.items()
    }


@dataclass
class TrafficHold:
    """A local deferred dispatch bound to one agreed traffic reservation."""

    round_id: str
    decision_hash: str
    winner_robot_id: str
    snapshot_epochs: tuple[int, int]
    created_steady_s: float
    winner_path: tuple[tuple[float, float], ...] = ()
    winner_base_frame: str = ''
    last_conflict_distance_m: float = 0.0
    clearance_m: float = 0.05
    winner_observed_active: bool = False


STATE_TO_MESSAGE = {
    CoordinatorState.WAITING_FOR_INPUTS: DistributedExplorationStatus.WAITING_FOR_INPUTS,
    CoordinatorState.BIDDING: DistributedExplorationStatus.BIDDING,
    CoordinatorState.WAITING_FOR_MATCHING_DECISION:
        DistributedExplorationStatus.WAITING_FOR_MATCHING_DECISION,
    CoordinatorState.WAITING_FOR_TRAFFIC:
        DistributedExplorationStatus.WAITING_FOR_TRAFFIC,
    CoordinatorState.NAVIGATING: DistributedExplorationStatus.NAVIGATING,
    CoordinatorState.DEGRADED_SOLO: DistributedExplorationStatus.DEGRADED_SOLO,
    CoordinatorState.COMPLETE: DistributedExplorationStatus.COMPLETE,
    CoordinatorState.BLOCKED: DistributedExplorationStatus.BLOCKED,
}


FAILURE_TO_MESSAGE = {
    FailureClass.HARD_UNREACHABLE: ExplorationFailure.HARD_UNREACHABLE,
    FailureClass.PLANNER_FAILURE: ExplorationFailure.PLANNER_FAILURE,
    FailureClass.CONTROLLER_NO_PROGRESS: ExplorationFailure.CONTROLLER_NO_PROGRESS,
    FailureClass.DYNAMIC_BLOCKAGE: ExplorationFailure.DYNAMIC_BLOCKAGE,
    FailureClass.TF_OR_LIFECYCLE: ExplorationFailure.TF_OR_LIFECYCLE,
    FailureClass.ACTION_REJECTION: ExplorationFailure.ACTION_REJECTION,
    FailureClass.TIMEOUT: ExplorationFailure.TIMEOUT,
    FailureClass.EXPLICIT_CANCELLATION: ExplorationFailure.EXPLICIT_CANCELLATION,
    FailureClass.UNKNOWN: ExplorationFailure.UNKNOWN,
}


def eligible_solo_tasks(
        tasks: tuple[PhysicalTask, ...],
        hard_failure_signatures: set[str],
        completed_signatures: set[str],
        minimum_visible_gain_m: float,
        minimum_ordering_score: float,
        selection_policy: str = 'legacy_weighted') -> tuple[PhysicalTask, ...]:
    """Return locally dispatchable tasks after bounded physical suppression.

    A successful finite Nav2 path is feasible at any distance.  Path length
    remains available to the local/distributed preference logic and is not a
    hard eligibility condition.
    """
    return tuple(
        task for task in tasks
        if task.physical_signature not in hard_failure_signatures
        and task.physical_signature not in completed_signatures
        and (selection_policy == 'frontier_cost_only' or
             task.visible_reveal_gain >= minimum_visible_gain_m)
        and (selection_policy == 'frontier_cost_only' or
             task.local_ordering_score >= minimum_ordering_score)
        and task.local_path_valid
        and math.isfinite(task.local_path_length_m)
        and task.local_path_length_m >= 0.0
        and math.isfinite(task.path_heading_cost_rad)
        and task.path_heading_cost_rad >= 0.0
        and (selection_policy == 'frontier_cost_only' or
             math.isfinite(task.local_ordering_score))
        and (selection_policy == 'frontier_cost_only' or
             math.isfinite(task.visible_reveal_gain))
    )


def classify_solo_dispatch_failure(
        checks: DispatchPreconditions, *, local_only: bool) -> FailureClass:
    """Keep unknown-map fallback rejections retryable without accepting them.

    A local-only physical run can observe a valid planner/costmap path while
    the corresponding SLAM occupancy cell is still unknown.  That remains a
    hard dispatch rejection, but it is not permanent unreachable evidence:
    the map/costmap/TF context may converge on a later snapshot.  Cooperative
    dispatch keeps the authoritative hard-failure classification unchanged.
    """
    failure = classify_dispatch_precondition_failure(checks)
    if not local_only or failure not in HARD_FAILURES:
        return failure
    if (
            checks.reason == 'goal occupancy-map cell is unknown or occupied' and
            checks.goal_costmap_value is not None and
            0 <= checks.goal_costmap_value < 253 and
            checks.local_path_clear and checks.final_path_valid
    ):
        return FailureClass.TF_OR_LIFECYCLE
    return failure


class DistributedFrontierAssignment(Node):
    """Compute a complete pair decision independently and dispatch only locally."""

    def __init__(self):
        """Configure one equal peer with identity-bound topic and Nav2 interfaces."""
        super().__init__('distributed_frontier_assignment')
        self._robot_id = self.declare_parameter('robot_id', '').value
        if self._robot_id not in ('robot1', 'robot2'):
            raise ValueError('robot_id must be exactly robot1 or robot2')
        self._peer_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        self._local_only = bool(self.declare_parameter('local_only', False).value)
        self._handoff_gated = bool(self.declare_parameter(
            'handoff_gated', False).value)
        self._stop_after_handoff = bool(self.declare_parameter(
            'stop_after_handoff', False).value)
        self._phase_gated = bool(self.declare_parameter(
            'phase_gated', False).value)
        self._shared_nav2_ready_topic = str(self.declare_parameter(
            'shared_nav2_ready_topic', '').value)
        self._common_start_release_required = bool(self.declare_parameter(
            'common_start_release_required', False).value)
        self._publish_cooperative_start_ready = bool(self.declare_parameter(
            'publish_cooperative_start_ready', False).value)
        self._start_release_received = not self._common_start_release_required
        self._start_release_sim_time_s: Optional[float] = None
        self._start_release_subscription = None
        self._cooperative_start_ready_publisher = None
        self._handoff_complete = False
        self._dispatch_enabled = bool(
            self.declare_parameter('dispatch_enabled', False).value,
        )
        self._preflight_warmup_cycles_required = max(
            0,
            int(self.declare_parameter('preflight_warmup_cycles', 0).value),
        )
        self._preflight_warmup_cycles_observed = 0
        self._preflight_only = bool(
            self.declare_parameter('preflight_only', False).value,
        )
        self._initial_peer_readiness_barrier = bool(
            self.declare_parameter(
                'initial_peer_readiness_barrier', False,
            ).value,
        )
        self._initial_exploration_barrier = InitialExplorationBarrier(
            self._initial_peer_readiness_barrier,
        )
        self._initial_local_ready_sim_time_s: Optional[float] = None
        self._initial_local_ready_wall_time_s: Optional[float] = None
        self._initial_peer_ready_sim_time_s: Optional[float] = None
        self._initial_peer_ready_wall_time_s: Optional[float] = None
        self._initial_local_ready_publisher = None
        self._initial_peer_ready_subscription = None
        self._prehandoff_dispatch_delay_s = max(0.0, float(
            self.declare_parameter('prehandoff_dispatch_delay_s', 0.0).value))
        self._evidence_hold_timeout_s = max(0.5, float(
            self.declare_parameter('evidence_hold_timeout_s', 2.0).value))
        self._evidence_hold_until_wall_s = {
            'robot1': 0.0,
            'robot2': 0.0,
        }
        self._mission_timeout_enabled = bool(
            self.declare_parameter('enable_mission_timeout', False).value,
        )
        self._mission_timeout_s = float(
            self.declare_parameter('mission_timeout_s', 0.0).value,
        )
        self._planner_failure_confirmation_s = float(
            self.declare_parameter('planner_failure_confirmation_s', 30.0).value,
        )
        self._mission_started_steady_s = time.monotonic()
        self._dispatch_hold_started_steady_s = self._mission_started_steady_s
        self._synthetic_bids = bool(
            self.declare_parameter('synthetic_bids', False).value,
        )
        self._synthetic_origin = (
            float(self.declare_parameter('synthetic_origin_x', 0.0).value),
            float(self.declare_parameter('synthetic_origin_y', 0.0).value),
        )
        self._maximum_union_tasks = int(
            # Each peer advertises at most K tasks.  The canonical union must
            # therefore admit the full two-peer bound 2K before equivalence
            # clustering (K=5 in the final launch).
            self.declare_parameter('maximum_union_tasks', 10).value,
        )
        self._maximum_path_queries = int(
            self.declare_parameter('maximum_path_queries', 8).value,
        )
        self._snapshot_maximum_tasks = int(
            self.declare_parameter('maximum_tasks_per_source', 5).value,
        )
        self._bid_validity_s = float(self.declare_parameter('bid_validity_s', 3.0).value)
        self._decision_validity_s = float(
            self.declare_parameter('decision_validity_s', 3.0).value,
        )
        peer_timeout_s = float(self.declare_parameter('peer_timeout_s', 6.0).value)
        self._minimum_solo_visible_gain_m = float(
            self.declare_parameter('minimum_solo_visible_gain_m', 0.05).value,
        )
        self._minimum_solo_ordering_score = float(
            self.declare_parameter('minimum_solo_ordering_score', 0.0).value,
        )
        self._post_goal_settle_s = float(
            self.declare_parameter('post_goal_settle_s', 1.0).value,
        )
        self._map_stability_grace_s = float(
            self.declare_parameter('map_stability_grace_s', 5.0).value,
        )
        self._completion_confirmation_s = float(
            self.declare_parameter('completion_confirmation_s', 4.0).value,
        )
        self._terminal_small_frontier_length_m = float(
            self.declare_parameter(
                'terminal_small_frontier_length_m', 0.20,
            ).value,
        )
        if self._terminal_small_frontier_length_m <= 0.0:
            raise ValueError('terminal_small_frontier_length_m must be positive')
        self._assignment_strategy = str(
            self.declare_parameter(
                'assignment_strategy', 'frontier_mrtsp').value,
        )
        if self._assignment_strategy not in (
                'frontier_cost_only', 'frontier_mrtsp'):
            raise ValueError(
                'assignment_strategy must be frontier_cost_only or '
                'frontier_mrtsp')
        self._burgard_beta = float(self.declare_parameter('burgard_beta', 1.0).value)
        self._burgard_sensor_max_range_m = float(self.declare_parameter(
            'burgard_sensor_max_range_m', 11.98,
        ).value)
        self._burgard_occupied_threshold = int(self.declare_parameter(
            'burgard_occupied_threshold', 50,
        ).value)
        self._burgard_allow_missing_map_for_test = bool(self.declare_parameter(
            'burgard_allow_missing_map_for_test', False,
        ).value)
        self._traffic_scheduler_enabled = bool(self.declare_parameter(
            'traffic_scheduler_enabled', False,
        ).value)
        self._synchronized_traffic_test = bool(self.declare_parameter(
            'synchronized_traffic_test', False,
        ).value)
        # Test-only fixture control.  When enabled, the selector below still
        # uses only real canonical tasks and real Nav2 bid paths; it merely
        # chooses a conflicting pair so OFF/ON traffic runs exercise the same
        # adversarial geometry.  It is never enabled by production defaults.
        self._traffic_test_force_conflict_pair = bool(
            self.declare_parameter('traffic_test_force_conflict_pair', False).value,
        )
        self._traffic_robot1_safe_radius_m = float(self.declare_parameter(
            'traffic_robot1_safe_radius_m', 0.08,
        ).value)
        self._traffic_robot2_safe_radius_m = float(self.declare_parameter(
            'traffic_robot2_safe_radius_m', 0.08,
        ).value)
        self._traffic_reference_speed_mps = float(self.declare_parameter(
            'traffic_reference_speed_mps', 0.13,
        ).value)
        self._traffic_eta_tie_s = float(self.declare_parameter(
            'traffic_eta_tie_s', 0.05,
        ).value)
        self._traffic_dispatch_grace_s = float(self.declare_parameter(
            'traffic_dispatch_grace_s', 8.0,
        ).value)
        self._traffic_conflict_clearance_m = float(self.declare_parameter(
            'traffic_conflict_clearance_m', 0.05,
        ).value)
        self._executor_threads = max(1, int(self.declare_parameter(
            'executor_threads', 4).value))
        if (self._burgard_beta < 0.0 or self._burgard_sensor_max_range_m <= 0.0 or
                self._traffic_robot1_safe_radius_m <= 0.0 or
                self._traffic_robot2_safe_radius_m <= 0.0 or
                self._traffic_reference_speed_mps <= 0.0 or
                self._traffic_conflict_clearance_m < 0.0):
            raise ValueError('Burgard and traffic parameters must be positive')
        self._weights = AssignmentWeights(
            gain=float(self.declare_parameter('weight_gain', 3.0).value),
            path=float(self.declare_parameter('weight_path', 1.0).value),
            nearby_goal=float(self.declare_parameter('weight_nearby_goal', 1.5).value),
            route_overlap=float(self.declare_parameter('weight_route_overlap', 3.0).value),
            hard_failure=float(self.declare_parameter('weight_hard_failure', 5.0).value),
            sensing_overlap=float(
                self.declare_parameter('weight_sensing_overlap', 2.0).value,
            ),
            workload_imbalance=float(
                self.declare_parameter('weight_workload_imbalance', 0.35).value,
            ),
            visible_gain_scale=float(
                self.declare_parameter('visible_gain_scale', 5.0).value,
            ),
            path_cost_scale_m=float(
                self.declare_parameter('path_cost_scale_m', 12.0).value,
            ),
            cost_only_reference_linear_speed_mps=float(
                self.declare_parameter(
                    'cost_only_reference_linear_speed_mps', 0.13,
                ).value,
            ),
            cost_only_reference_angular_speed_radps=float(
                self.declare_parameter(
                    'cost_only_reference_angular_speed_radps', 0.35,
                ).value,
            ),
            nearby_goal_distance_m=float(
                self.declare_parameter('nearby_goal_distance_m', 0.6).value,
            ),
            route_corridor_radius_m=float(
                self.declare_parameter('route_corridor_radius_m', 0.16).value,
            ),
            minimum_visible_gain_m=self._minimum_solo_visible_gain_m,
        )
        self.get_logger().info(
            'COST_ONLY_MOTION_REFERENCES linear_mps=%.6f angular_radps=%.6f' % (
                self._weights.cost_only_reference_linear_speed_mps,
                self._weights.cost_only_reference_angular_speed_radps,
            )
        )
        self._ledger = SnapshotLedger(self._snapshot_maximum_tasks)
        self._snapshots: dict[str, Received[TaskSnapshot]] = {}
        self._bid_batches: dict[str, Received[BidBatch]] = {}
        self._peer_decision: Optional[Received[PairDecisionMsg]] = None
        self._peer_status: Optional[Received[DistributedExplorationStatus]] = None
        self._hard_failure_signatures: dict[str, float] = {}
        # Keep bounded, evidence-based local/peer failure history so a hard
        # failure cannot immediately re-enter the next auction under the
        # same physical signature.  The duration escalates for repeated hard
        # evidence, while transient classes never enter this table.
        self._hard_failure_counts: dict[str, int] = {}
        # A successfully completed local-only frontier remains suppressed
        # while the generator continues to publish the same physical region.
        # Without this bounded set, resetting the semantic snapshot key after
        # every success can immediately redispatch a tiny residual frontier.
        self._completed_solo_physical_signatures: set[str] = set()
        # Replicated success state for shared dispatch. The normal success
        # lifecycle already emits DistributedExplorationEvent, so no new
        # protocol or permanent spatial ownership is needed. Exact canonical
        # tasks are suppressed only while they remain in current proposals;
        # a materially evolved task naturally receives a new identity.
        self._completed_shared_canonical_ids: set[str] = set()
        # Successful local routes are bounded history, not a corridor ban.
        # The solo scorer uses this only when a less-overlapping frontier is
        # available; necessary transit therefore remains selectable.
        self._solo_route_history = deque(maxlen=8)
        self._solo_retry_not_before: dict[str, float] = {}
        self._solo_retry_counts: dict[str, int] = {}
        self._active_dispatch_path: tuple[tuple[float, float], ...] = ()
        # Diagnostics-only counters.  These do not alter eligibility or
        # suppression; they separate structural controller failures from
        # transient TF/infrastructure failures for forensic replay.
        self._failure_task_diagnostics: dict[str, dict[str, object]] = {}
        self._peer_liveness = PeerLiveness(peer_timeout_s)
        self._round: Optional[RoundWork] = None
        self._round_lifecycle = RoundGeneration()
        self._round_created_count = 0
        self._round_completed_count = 0
        self._round_replaced_count = 0
        self._stale_tick_discard_count = 0
        self._dispatch_count = 0
        # Successful local planner results may outlive an ephemeral allocator
        # round, but only as an exact task/provenance/context-keyed cache.  A
        # new round still creates a new bid and must obtain a new peer batch.
        self._local_path_evaluation_cache: dict[tuple, PathEvaluation] = {}
        self._local_path_cache_hits = 0
        self._local_path_cache_misses = 0
        self._local_path_cache_invalidations = 0
        self._local_path_cache_evictions = 0
        self._last_tick_steady_s = time.monotonic()
        self._last_round_completion_steady_s = 0.0
        self._last_tick_log_key = None
        # Opt-in allocator attribution for bounded performance experiments.
        # This is deliberately disabled by default and records aggregate
        # section timings only; it does not participate in allocation.
        self._allocator_timing_enabled = os.environ.get(
            'MY_EPUCK_ALLOCATOR_TIMING', '',
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self._allocator_timing_stats = {
            section: {
                bucket: {'calls': 0, 'total_wall_s': 0.0,
                         'max_wall_s': 0.0}
                for bucket in ('early', 'late')
            }
            for section in (
                'tick_total', 'continue_bidding', 'pair_selection',
                'traffic_checks', 'consensus_continuation',
            )
        }
        self._allocator_timing_inputs = {
            bucket: {
                'candidate_count_total': 0,
                'candidate_count_calls': 0,
                'candidate_pairs_input_total': 0,
                'candidate_pairs_input_calls': 0,
                'traffic_checks_total': 0,
            }
            for bucket in ('early', 'late')
        }
        self._allocator_timing_last_log_wall_s = time.monotonic()
        self._coordinator_alive = True
        self._committed = CommittedRound()
        # Agreed cooperative goals outlive the ephemeral pair round that
        # selected them.  Each replica retains only the active task/path
        # needed to protect ownership and evaluate continuation traffic; it
        # never retains a stale frontier bid vector.
        self._active_commitments: dict[str, ActiveCommitment] = {}
        # Bounded diagnostics for selector-state divergence and the stronger
        # equal-input/different-output invariant violation.  These are
        # diagnostics only; neither path creates a retry protocol.
        self._last_selector_divergence_key = None
        self._last_decision_invariant_violation_key = None
        self._state = CoordinatorState.WAITING_FOR_INPUTS
        self._state_reason = 'startup'
        self._dispatch_in_progress = False
        self._settle_until_steady_s = 0.0
        self._active_task: Optional[CanonicalTask] = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        self._navigation_action_sequence = 0
        self._active_navigation_action: Optional[ActiveNavigationAction] = None
        self._traffic_hold: Optional[TrafficHold] = None
        # A conflict-clear event is replicated over the existing event topics.
        # It permits one fresh pair round while the previous winner is still
        # travelling, but the winner remains an active reservation if the new
        # paths conflict again.
        self._traffic_reallocation_after_clear = False
        self._released_traffic_winner_robot_id = ''
        self._last_peer_traffic_clear_key = None
        self._traffic_test_release_key: Optional[tuple[str, str]] = None
        self._traffic_test_release_at_sim_s: Optional[float] = None
        self._traffic_test_release_logged_key: Optional[tuple[str, str]] = None
        self._traffic_test_ready_key: Optional[tuple[str, str]] = None
        # Startup evidence is emitted once per allocator process.  These
        # markers are diagnostics only; the TF gate below is the authority
        # that prevents an actionable first pair decision from racing Nav2.
        self._shared_tf_ready_logged = False
        self._first_nonempty_task_snapshot_logged: set[str] = set()
        self._first_valid_task_snapshots_logged = False
        self._first_valid_pair_decision_logged = False
        self._first_cooperative_goal_logged = False
        self._last_shared_tf_wait_log_wall_s = 0.0
        # Local-only work is keyed by semantic task content, not heartbeat
        # epoch.  The key is cleared on a terminal result or failure so a
        # fresh path/action attempt still occurs when the previous attempt
        # actually ended.
        self._last_solo_snapshot_key: Optional[tuple[str, str]] = None
        self._map_versions: dict[str, tuple[str, int, str]] = {}
        self._maps_stable_since_steady_s = time.monotonic()
        self._completion_candidate_since_steady_s: Optional[float] = None
        self._completion_candidate_reason = ''
        self._planner_failure_candidate_since_steady_s: Optional[float] = None
        self._terminal = False
        self._terminal_success = False
        self._terminal_reason = ''
        self._terminal_epoch = 0
        self._terminal_finalization_attempt_count = 0
        self._terminal_commit_count = 0
        self._last_semantic_fingerprint = ''
        self._candidate_evidence = {
            robot: CandidateEvidence() for robot in ('robot1', 'robot2')
        }
        # Keep source-local evidence separate from the merged physical-region
        # view used by terminal/status reporting.  Certificate completeness
        # must never compare one source's bounds with a cross-robot union.
        self._candidate_source_local_evidence = {
            robot: CandidateEvidence() for robot in ('robot1', 'robot2')
        }
        # The task-snapshot TTL is deliberately short for peer/cooperative
        # evidence.  Keep the latest source-local physical signatures
        # separately so a temporarily expired local snapshot can only be
        # reused as a seed when the same current candidate is still being
        # advertised.  ``None`` means no candidate batch has been observed;
        # an empty set is authoritative evidence that no reachable candidates
        # were present in the latest batch.
        self._current_local_frontier_ids: dict[
            str, Optional[frozenset[str]]] = {
                robot: None for robot in ('robot1', 'robot2')}
        # Candidate/snapshot arrival is an immediate opportunity for the
        # existing degraded-solo path. The allocator tick consumes this
        # scheduling hint before beginning another cooperative wait; it does
        # not bypass any dispatch or safety validation.
        self._local_fallback_trigger_pending = False
        self._local_fallback_trigger_reason = ''
        self._candidate_region_snapshots: dict[
            str, tuple[FrontierRegionEvidence, ...]] = {}
        # Latest pre-query lower-bound evidence, retained separately from
        # terminal classification.  ``None`` means the source did not provide
        # enough geometry to certify a cost-only decision.
        self._unqueried_cost_bounds: dict[
            str, Optional[tuple[float, ...]]] = {
                robot: None for robot in ('robot1', 'robot2')}
        # (source-local map revision, exact generator context token,
        # immutable candidate-generation ID). Receipt time below is retained
        # for diagnostics only, never as a TTL.
        self._unqueried_cost_bound_provenance: dict[
            str, tuple[int, str, int]] = {}
        self._unqueried_cost_bounds_received: dict[str, float] = {}
        # Compact candidate-side provenance is retained for certificate
        # diagnostics and to bind terminal completion evidence to the same
        # source generation/map/costmap as the fresh task snapshot.
        self._candidate_lower_bound_metadata: dict[str, dict] = {}
        self._candidate_lower_bound_history: dict[str, dict[tuple, dict]] = {
            robot: {} for robot in ('robot1', 'robot2')}
        self._candidate_evidence_seen = set()
        self._last_cost_only_certificate_key = None
        # Certificate blocker history is diagnostic-only.  It is deliberately
        # kept separate from round state so it cannot affect allocation,
        # query scheduling, or certificate results.
        self._certificate_blocker_history: dict[tuple[str, str], dict] = {}
        self._certificate_blocker_last_round: dict[str, str] = {}
        self._nav2 = LocalNav2(self, phase_gated=self._phase_gated)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # The C++ candidate generator publishes volatile data.  Keep the
        # evidence subscription compatible with that live stream; task
        # snapshots/bids/decisions remain transient-local protocol data.
        candidate_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        hypothesis_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            # Accepted hypotheses are one-shot volatile announcements from
            # the frontend; do not request transient-local durability.
            durability=DurabilityPolicy.VOLATILE,
        )
        task_snapshot_topic = str(self.declare_parameter(
            'task_snapshot_topic', 'task_snapshot').value)
        candidate_topic = str(self.declare_parameter(
            'candidate_topic', 'frontier_candidates').value)
        robot_ids = (self._robot_id,) if self._local_only else ('robot1', 'robot2')
        self._phase_subscriptions = []
        self._phase_protocol = {
            'qos': qos,
            'candidate_qos': candidate_qos,
            'task_snapshot_topic': task_snapshot_topic,
            'candidate_topic': candidate_topic,
            'robot_ids': robot_ids,
        }
        if not self._phase_gated:
            self._activate_protocol_inputs()
        elif self._shared_nav2_ready_topic:
            self._shared_nav2_ready_subscription = self.create_subscription(
                Bool, self._shared_nav2_ready_topic,
                self._shared_nav2_ready_callback,
                QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        if self._common_start_release_required:
            start_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._start_release_subscription = self.create_subscription(
                String, '/cslam/unknown_pose/start_release',
                self._start_release_callback, start_qos,
            )
            if self._publish_cooperative_start_ready:
                self._cooperative_start_ready_publisher = self.create_publisher(
                    String,
                    f'/cslam/unknown_pose/cooperative_start_ready/{self._robot_id}',
                    start_qos,
                )
        if self._handoff_gated or self._stop_after_handoff:
            self.create_subscription(
                RelativePoseHypothesis, '/cslam/relative_pose/hypotheses',
                self._handoff_callback, hypothesis_qos,
            )
        if self._local_only and self._initial_peer_readiness_barrier:
            readiness_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._initial_local_ready_publisher = self.create_publisher(
                String,
                f'/cslam/unknown_pose/{self._robot_id}/initial_local_ready',
                readiness_qos,
            )
            self._initial_peer_ready_subscription = self.create_subscription(
                String,
                f'/cslam/unknown_pose/{self._peer_id}/initial_local_ready',
                self._initial_peer_ready_callback,
                readiness_qos,
            )
            self.get_logger().info(
                'INITIAL_LOCAL_EXPLORATION_BARRIER robot=%s enabled=true '
                'peer=%s' % (self._robot_id, self._peer_id))
        evidence_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for evidence_robot_id in ('robot1', 'robot2'):
            self.create_subscription(
                Bool,
                f'/cslam/relative_pose/{evidence_robot_id}/'
                'evidence_acquisition_active',
                lambda message, robot_id=evidence_robot_id:
                    self._evidence_status_callback(message, robot_id),
                evidence_qos,
            )
        self._bid_publisher = self.create_publisher(TaskBidArrayMsg, 'task_bids', qos)
        self._decision_publisher = self.create_publisher(
            PairDecisionMsg, 'pair_decision', qos,
        )
        self._status_publisher = self.create_publisher(
            DistributedExplorationStatus, 'distributed_status', qos,
        )
        self._failure_publisher = self.create_publisher(
            ExplorationFailure, 'exploration_failure', qos,
        )
        self._event_publisher = self.create_publisher(
            DistributedExplorationEvent, 'distributed_event',
            DISTRIBUTED_EVENT_QOS,
        )
        if self._synchronized_traffic_test and not self._local_only:
            traffic_test_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._traffic_test_ready_publisher = self.create_publisher(
                String,
                f'/cslam/traffic_test/dispatch_ready/{self._robot_id}',
                traffic_test_qos,
            )
            self._traffic_test_release_subscription = self.create_subscription(
                String,
                '/cslam/traffic_test/dispatch_release',
                self._traffic_test_release_callback,
                traffic_test_qos,
            )
        else:
            self._traffic_test_ready_publisher = None
            self._traffic_test_release_subscription = None
        self._tick_timer = None
        self._status_timer = None
        if not self._phase_gated:
            self._activate_assignment_timers()
        interfaces = self._nav2.interface_names()
        self.get_logger().info(
            'DISTRIBUTED_ASSIGNMENT robot=%s peer=%s dispatch=%s strategy=%s '
            'beta=%.3f path_cost_scale=%.3f sensor_range=%.3f traffic=%s '
            'terminal_small_frontier_length_m=%.3f '
            'traffic_radii=(%.3f,%.3f) traffic_speed=%.3f compute=%s navigate=%s '
            'local_only=%s preflight_only=%s no_peer_clients=%s no_cmd_vel=true' % (
                self._robot_id, self._peer_id, self._dispatch_enabled,
                self._assignment_strategy, self._burgard_beta,
                self._weights.path_cost_scale_m, self._burgard_sensor_max_range_m,
                self._traffic_scheduler_enabled,
                self._terminal_small_frontier_length_m,
                self._traffic_robot1_safe_radius_m,
                self._traffic_robot2_safe_radius_m, self._traffic_reference_speed_mps,
                interfaces['compute_path'], interfaces['navigate'],
                self._local_only, self._preflight_only, self._local_only,
            )
        )

    def _sim_time_s(self) -> float:
        """Return current simulation/ROS time for authoritative startup logs."""
        # Some focused certificate tests construct the node with __new__ so
        # they can exercise pure decision logic without starting ROS. Keep
        # diagnostic timestamps optional for those fixtures; a real Node
        # always has _clock initialized by rclpy.
        clock = getattr(self, '_clock', None)
        if clock is None:
            return 0.0
        return clock.now().nanoseconds / 1e9

    def _startup_event(self, event_type: str, **fields) -> None:
        """Emit one compact startup milestone to ROS logs and the observer."""
        payload = {
            'robot_id': self._robot_id,
            'sim_time_s': self._sim_time_s(),
            'wall_time_s': time.time(),
        }
        payload.update(fields)
        text = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        self.get_logger().info('%s %s' % (event_type, text))
        self._emit_event(event_type, text)

    def _publish_initial_local_ready(self, now: float) -> None:
        """Publish one peer-readable local exploration readiness fact."""
        barrier = self._initial_exploration_barrier
        if (not self._initial_peer_readiness_barrier or
                self._initial_local_ready_publisher is None or
                barrier.local_ready or barrier.handoff_complete):
            return
        sim_time = self._sim_time_s()
        wall_time = time.time()
        barrier.observe_local_ready()
        self._initial_local_ready_sim_time_s = sim_time
        self._initial_local_ready_wall_time_s = wall_time
        message = String()
        message.data = json.dumps({
            'event': 'INITIAL_LOCAL_READY',
            'robot_id': self._robot_id,
            'sim_time_s': sim_time,
            'wall_time_s': wall_time,
        }, sort_keys=True, separators=(',', ':'))
        self._initial_local_ready_publisher.publish(message)
        self.get_logger().info(
            'INITIAL_LOCAL_READY robot=%s sim_time_s=%.6f wall_time_s=%.6f' %
            (self._robot_id, sim_time, wall_time))
        self._emit_event(
            'INITIAL_LOCAL_READY', message.data,
        )
        self._maybe_release_initial_exploration_barrier()

    def _initial_peer_ready_callback(self, message: String) -> None:
        """Observe the peer's one-shot readiness announcement idempotently."""
        if not self._initial_peer_readiness_barrier:
            return
        try:
            payload = json.loads(str(message.data))
            peer_id = str(payload['robot_id'])
            if str(payload.get('event', '')) != 'INITIAL_LOCAL_READY':
                return
            sim_time = float(payload['sim_time_s'])
            wall_time = float(payload['wall_time_s'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(
                'INITIAL_PEER_READY_REJECTED robot=%s reason=malformed_payload' %
                self._robot_id)
            return
        if peer_id != self._peer_id:
            self.get_logger().warning(
                'INITIAL_PEER_READY_REJECTED robot=%s peer=%s expected=%s' %
                (self._robot_id, peer_id, self._peer_id))
            return
        if self._initial_exploration_barrier.peer_ready:
            return
        self._initial_exploration_barrier.observe_peer_ready()
        self._initial_peer_ready_sim_time_s = sim_time
        self._initial_peer_ready_wall_time_s = wall_time
        self.get_logger().info(
            'INITIAL_PEER_READY_OBSERVED robot=%s peer_robot_id=%s '
            'peer_sim_time_s=%.6f peer_wall_time_s=%.6f observed_sim_time_s=%.6f' %
            (self._robot_id, peer_id, sim_time, wall_time, self._sim_time_s()))
        self._maybe_release_initial_exploration_barrier()

    def _maybe_release_initial_exploration_barrier(self) -> None:
        """Release local dispatch only after both replicas are ready."""
        barrier = self._initial_exploration_barrier
        if not barrier.maybe_release():
            return
        sim_time = self._sim_time_s()
        wall_time = time.time()
        self.get_logger().info(
            'INITIAL_EXPLORATION_BARRIER_RELEASED robot=%s sim_time_s=%.6f '
            'wall_time_s=%.6f local_ready=%s peer_ready=%s '
            'local_ready_sim_time_s=%.6f peer_ready_sim_time_s=%.6f' % (
                self._robot_id, sim_time, wall_time, barrier.local_ready,
                barrier.peer_ready,
                self._initial_local_ready_sim_time_s or 0.0,
                self._initial_peer_ready_sim_time_s or 0.0,
            ))
        self._emit_event(
            'INITIAL_EXPLORATION_BARRIER_RELEASED', json.dumps({
                'local_ready_sim_time_s': self._initial_local_ready_sim_time_s,
                'local_ready_wall_time_s': self._initial_local_ready_wall_time_s,
                'peer_ready_sim_time_s': self._initial_peer_ready_sim_time_s,
                'peer_ready_wall_time_s': self._initial_peer_ready_wall_time_s,
                'release_sim_time_s': sim_time,
                'release_wall_time_s': wall_time,
            }, sort_keys=True, separators=(',', ':')),
        )

    def _traffic_test_release_callback(self, message: String) -> None:
        """Record a per-round simulated dispatch boundary from the test barrier."""
        try:
            payload = json.loads(str(message.data))
            key = (str(payload['round_id']), str(payload['decision_hash']))
            release_at = float(payload['release_at_sim_time_s'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not key[0] or not key[1]:
            return
        self._traffic_test_release_key = key
        self._traffic_test_release_at_sim_s = release_at
        sim_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            'TRAFFIC_TEST_DISPATCH_RELEASE_RECEIVED robot=%s round=%s '
            'decision_hash=%s release_at_sim_time_s=%.6f received_sim_time_s=%.6f' %
            (self._robot_id, key[0], key[1], release_at, sim_time),
        )

    def _publish_traffic_test_ready(self, round_work: RoundWork) -> None:
        """Advertise one agreed round to the test-only common barrier."""
        if self._traffic_test_ready_publisher is None:
            return
        if round_work.decision is None:
            return
        key = (round_work.round_id, round_work.decision.decision_hash)
        if key == self._traffic_test_ready_key:
            return
        payload = String()
        payload.data = json.dumps({
            'robot_id': self._robot_id,
            'round_id': key[0],
            'decision_hash': key[1],
            'sim_time_s': self.get_clock().now().nanoseconds / 1e9,
        }, sort_keys=True, separators=(',', ':'))
        self._traffic_test_ready_publisher.publish(payload)
        self._traffic_test_ready_key = key
        self.get_logger().info(
            'TRAFFIC_TEST_DISPATCH_READY robot=%s round=%s decision_hash=%s' %
            (self._robot_id, key[0], key[1]),
        )

    def _activate_protocol_inputs(self) -> None:
        """Subscribe to task/bid traffic only in the active shared phase."""
        if self._phase_subscriptions:
            return
        qos = self._phase_protocol['qos']
        candidate_qos = self._phase_protocol['candidate_qos']
        task_snapshot_topic = self._phase_protocol['task_snapshot_topic']
        candidate_topic = self._phase_protocol['candidate_topic']
        for robot_id in self._phase_protocol['robot_ids']:
            self._phase_subscriptions.extend([
                self.create_subscription(
                    FrontierCandidateArray,
                    candidate_topic if self._local_only else
                    f'/{robot_id}/frontier_candidates',
                    self._candidate_callback, candidate_qos),
                self.create_subscription(
                    TaskSnapshotMsg,
                    task_snapshot_topic if self._local_only else
                    f'/{robot_id}/task_snapshot',
                    self._snapshot_callback, qos),
            ])
        if not self._local_only:
            for robot_id in ('robot1', 'robot2'):
                self._phase_subscriptions.extend([
                    self.create_subscription(
                        TaskBidArrayMsg, f'/{robot_id}/task_bids',
                        self._bid_callback, qos),
                    self.create_subscription(
                        PairDecisionMsg, f'/{robot_id}/pair_decision',
                        self._decision_callback, qos),
                    self.create_subscription(
                        DistributedExplorationStatus,
                        f'/{robot_id}/distributed_status',
                        self._status_callback, qos),
                    self.create_subscription(
                        ExplorationFailure,
                        f'/{robot_id}/exploration_failure',
                        self._failure_callback, qos),
                ])
            # Traffic-clearance is a replicated lifecycle event, not a
            # central command.  It tells the other allocator to rebuild the
            # same fresh round while the prior winner may still be navigating.
            self._phase_subscriptions.append(
                self.create_subscription(
                    DistributedExplorationEvent,
                    f'/{self._peer_id}/distributed_event',
                    self._peer_event_callback, DISTRIBUTED_EVENT_QOS),
            )

    def _activate_assignment_timers(self) -> None:
        """Start assignment work only after the canonical handoff."""
        if self._tick_timer is None:
            tick_period_s = max(0.05, float(os.environ.get(
                'MY_EPUCK_ASSIGNMENT_TICK_PERIOD_S', '0.1')))
            self._tick_timer = self.create_timer(tick_period_s, self._tick)
        if self._status_timer is None:
            self._status_timer = self.create_timer(1.0, self._publish_status)

    def _activate_shared_phase(self) -> None:
        if not self._phase_gated:
            return
        self._phase_gated = False
        self._activate_protocol_inputs()
        self._nav2._activate_phase_inputs()
        self._activate_assignment_timers()
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE shared_assignment_active=true '
            'protocol_inputs=true timers=true')
        if self._cooperative_start_ready_publisher is not None:
            message = String()
            message.data = json.dumps({
                'event': 'COOPERATIVE_START_STATE_READY',
                'robot_id': self._robot_id,
                'sim_time_s': self._sim_time_s(),
                # C currently runs with the traffic gate disabled; that is an
                # explicit ready/no-gate state.  Enabled traffic is still
                # evaluated by the normal dispatch path after release.
                'traffic_scheduler_ready': True,
            }, sort_keys=True, separators=(',', ':'))
            self._cooperative_start_ready_publisher.publish(message)
            self._emit_event(
                'COOPERATIVE_START_STATE_READY', message.data,
            )

    def _start_release_callback(self, message: String) -> None:
        if self._start_release_received:
            return
        try:
            payload = json.loads(str(message.data))
            if str(payload.get('event', '')) != 'START_RELEASE':
                return
            release_time = float(payload['release_sim_time_s'])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            self.get_logger().warning(
                'START_RELEASE_REJECTED robot=%s reason=malformed_payload' %
                self._robot_id)
            return
        self._start_release_received = True
        self._start_release_sim_time_s = release_time
        self.get_logger().info(
            'START_RELEASE_RECEIVED robot=%s release_sim_time_s=%.6f' %
            (self._robot_id, release_time))
        self._emit_event('START_RELEASE_RECEIVED', message.data)

    def _exploration_dispatch_allowed(self) -> bool:
        """Keep every exploration send behind the common C release barrier."""
        return (not getattr(self, '_common_start_release_required', False) or
                getattr(self, '_start_release_received', False))

    def _shared_nav2_ready_callback(self, message: Bool) -> None:
        if not bool(message.data):
            return
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s shared_nav2_barrier_received=true' %
            self._robot_id)
        self._activate_shared_phase()

    def _handoff_callback(self, message: RelativePoseHypothesis) -> None:
        """Switch local-only dispatch off only after canonical acceptance."""
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        if self._handoff_complete:
            return
        self._handoff_complete = True
        was_waiting_for_initial_barrier = bool(
            self._local_only and self._initial_peer_readiness_barrier and
            not self._initial_exploration_barrier.released)
        self._initial_exploration_barrier.observe_handoff()
        self._activate_shared_phase()
        self._dispatch_enabled = False if self._local_only else True
        if self._local_only and self._nav2.local_goal_active:
            self._request_navigation_cancel()
        if was_waiting_for_initial_barrier:
            reason = json.dumps({
                'handoff_sim_time_s': self._sim_time_s(),
                'handoff_wall_time_s': time.time(),
                'local_ready': self._initial_exploration_barrier.local_ready,
                'peer_ready': self._initial_exploration_barrier.peer_ready,
            }, sort_keys=True, separators=(',', ':'))
            self.get_logger().info(
                'INITIAL_LOCAL_EXPLORATION_SKIPPED_DUE_TO_HANDOFF '
                'robot=%s reason=%s' % (self._robot_id, reason))
            self._emit_event(
                'INITIAL_LOCAL_EXPLORATION_SKIPPED_DUE_TO_HANDOFF', reason,
            )
        if self._local_only and self._stop_after_handoff:
            self._stop_local_phase()
        self.get_logger().info(
            'UNKNOWN_POSE_PHASE robot=%s phase=POST_HANDOFF dispatch=%s' %
            (self._robot_id, self._dispatch_enabled))

    def _stop_local_phase(self) -> None:
        """Stop pre-handoff allocation work after canonical handoff."""
        for subscription in self._phase_subscriptions:
            self.destroy_subscription(subscription)
        self._phase_subscriptions = []
        if self._tick_timer is not None:
            self._tick_timer.cancel()
            self._tick_timer = None
        if self._status_timer is not None:
            self._status_timer.cancel()
            self._status_timer = None
        self._round = None
        self._dispatch_in_progress = False
        self._state = CoordinatorState.WAITING_FOR_INPUTS
        self._state_reason = 'pre-handoff local phase stopped'
        self.get_logger().info(
            'FRONTIER_PHASE pre_handoff_stopped=true reason=ACCEPTED_HANDOFF')

    def _candidate_callback(self, message: FrontierCandidateArray) -> None:
        """Keep bounded generator evidence separate from reachable task bids."""
        if message.source_robot_id not in self._candidate_evidence:
            return
        self._candidate_evidence_seen.add(message.source_robot_id)
        # This is source-local current evidence, not a peer/evidence TTL.  The
        # fallback path uses it only to validate that a retained task still
        # represents the same physical candidate before asking Nav2 to build a
        # new path.
        if not hasattr(self, '_current_local_frontier_ids'):
            self._current_local_frontier_ids = {
                robot: None for robot in ('robot1', 'robot2')}
        self._current_local_frontier_ids[message.source_robot_id] = frozenset(
            str(getattr(candidate, 'physical_frontier_id',
                       getattr(candidate, 'frontier_id', '')))
            for candidate in message.candidates
            if str(getattr(candidate, 'physical_frontier_id',
                           getattr(candidate, 'frontier_id', '')))
        )
        if message.source_robot_id == getattr(self, '_robot_id', None):
            if message.candidates:
                self._local_fallback_trigger_pending = True
                self._local_fallback_trigger_reason = (
                    'local candidate batch became current while cooperative '
                    'evidence was pending')
            else:
                self._local_fallback_trigger_pending = False
                self._local_fallback_trigger_reason = ''
        fallback = CandidateEvidence(
            detected=int(message.detected_frontier_count),
            small=int(message.small_frontier_count),
            reachable=sum(
                1 for candidate in message.candidates
                if candidate.reachability_state == candidate.REACHABLE
            ),
            out_of_range=int(message.out_of_range_frontier_count),
            unreachable=int(message.unreachable_frontier_count),
            planner_failures=int(message.planner_failure_count),
            unclassified=int(message.unclassified_frontier_count),
            detected_not_queried=int(
                getattr(message, 'detected_not_queried_count',
                        message.unclassified_frontier_count)),
            below_minimum_gain=sum(
                1 for candidate in message.candidates
                if candidate.reachability_state == candidate.REACHABLE and
                candidate.information_gain < self._minimum_solo_visible_gain_m
            ),
            actionable_reachable=sum(
                1 for candidate in message.candidates
                if candidate.reachability_state == candidate.REACHABLE and
                candidate.information_gain >= self._minimum_solo_visible_gain_m
            ),
        )
        if not hasattr(self, '_candidate_source_local_evidence'):
            self._candidate_source_local_evidence = {
                robot: CandidateEvidence() for robot in ('robot1', 'robot2')
            }
        self._candidate_source_local_evidence[
            message.source_robot_id] = fallback
        raw_regions = str(getattr(
            message, 'terminal_frontier_regions_json', '') or '')
        if not raw_regions:
            raw_regions = str(getattr(
                message, 'diagnostic_regions_json', '') or '')
        regions = self._decode_frontier_regions(
            raw_regions,
            candidate_generation_id=int(getattr(
                message, 'candidate_generation_id', 0) or 0),
            map_revision=int(getattr(message, 'map_revision', 0) or 0),
            costmap_revision=int(getattr(
                message, 'costmap_revision', 0) or 0),
        )
        unqueried = tuple(
            region.optimistic_cost_lower_bound_s
            for region in regions
            if str(region.status) == 'DETECTED_NOT_QUERIED'
        )
        detected_not_queried = int(
            getattr(message, 'detected_not_queried_count',
                    message.unclassified_frontier_count) or 0)
        bound_values = tuple(
            region.optimistic_cost_lower_bound_s for region in regions
            if str(region.status) == 'DETECTED_NOT_QUERIED')
        if not raw_regions:
            bound_state = ('EMPTY_BOUND_SUMMARY' if detected_not_queried == 0
                           else 'NO_CANDIDATE_BOUND_SUMMARY')
        else:
            try:
                parsed_regions = json.loads(raw_regions)
                if isinstance(parsed_regions, dict):
                    parsed_regions = parsed_regions.get('regions', [])
                if not isinstance(parsed_regions, list):
                    bound_state = 'OTHER'
                elif detected_not_queried > len(bound_values):
                    bound_state = 'INCOMPLETE_BOUND_SET'
                elif any(value is None for value in bound_values):
                    bound_state = 'INCOMPLETE_BOUND_SET'
                elif any(not math.isfinite(float(value)) or float(value) < 0.0
                         for value in bound_values):
                    bound_state = 'NONFINITE_BOUND'
                else:
                    bound_state = 'OK'
            except (TypeError, ValueError, json.JSONDecodeError):
                bound_state = 'OTHER'
        header_stamp = getattr(message, 'header', None)
        generation_ros_ns = None
        if header_stamp is not None:
            generation_ros_ns = (int(getattr(header_stamp.stamp, 'sec', 0)) *
                                 1_000_000_000 +
                                 int(getattr(header_stamp.stamp, 'nanosec', 0)))
        if not hasattr(self, '_candidate_lower_bound_metadata'):
            self._candidate_lower_bound_metadata = {}
        candidate_metadata = {
            'fingerprint': str(getattr(
                message, 'lower_bound_context_fingerprint', '') or ''),
            'map_revision': int(getattr(message, 'map_revision', 0)),
            'costmap_revision': int(getattr(
                message, 'costmap_revision', 0)),
            # Candidate arrays predate session/epoch provenance; record that
            # absence explicitly rather than inventing a comparison.
            'source_session_id': None,
            'source_epoch': None,
            'generation_ros_ns': generation_ros_ns,
            'candidate_generation_id': int(getattr(
                message, 'candidate_generation_id', 0) or 0),
            'bound_entry_count': len(bound_values),
            'detected_not_queried_count': detected_not_queried,
            'all_bounds_finite': bool(
                all(value is not None and math.isfinite(float(value)) and
                    float(value) >= 0.0 for value in bound_values)),
            'bound_state': bound_state,
        }
        self._candidate_lower_bound_metadata[message.source_robot_id] = (
            candidate_metadata)
        # A generator may omit diagnostic region JSON.  In that case an empty
        # tuple must not be mistaken for proof that no unqueried options
        # exist; the certificate is conservative until their bounds arrive.
        if detected_not_queried > 0 and not unqueried:
            self._unqueried_cost_bounds[message.source_robot_id] = None
        else:
            self._unqueried_cost_bounds[message.source_robot_id] = (
                tuple(float(value) for value in unqueried)
                if all(value is not None and math.isfinite(float(value)) and
                       float(value) >= 0.0 for value in unqueried)
                else None
            )
        self._unqueried_cost_bound_provenance[message.source_robot_id] = (
            int(getattr(message, 'map_revision', 0)),
            str(getattr(message, 'lower_bound_context_fingerprint', '') or ''),
            int(getattr(message, 'candidate_generation_id', 0) or 0),
        )
        candidate_generation_id = int(getattr(
            message, 'candidate_generation_id', 0) or 0)
        candidate_fingerprint = str(getattr(
            message, 'lower_bound_context_fingerprint', '') or '')
        candidate_map_revision = int(getattr(message, 'map_revision', 0) or 0)
        candidate_costmap_revision = int(getattr(
            message, 'costmap_revision', 0) or 0)
        if candidate_generation_id > 0 and candidate_fingerprint:
            if not hasattr(self, '_candidate_lower_bound_history'):
                self._candidate_lower_bound_history = {}
            history = self._candidate_lower_bound_history.setdefault(
                message.source_robot_id, {})
            history[(candidate_map_revision, candidate_fingerprint,
                     candidate_costmap_revision,
                     candidate_generation_id)] = {
                'bounds': self._unqueried_cost_bounds[
                    message.source_robot_id],
                'provenance': self._unqueried_cost_bound_provenance[
                    message.source_robot_id],
                'metadata': dict(candidate_metadata),
            }
            while (len(history) >
                   CANDIDATE_BOUND_CONTEXT_HISTORY_MAX_ENTRIES):
                history.pop(next(iter(history)))
        self._unqueried_cost_bounds_received[
            message.source_robot_id] = time.monotonic()
        if regions:
            self._candidate_region_snapshots[message.source_robot_id] = regions
            if set(self._candidate_region_snapshots) == {'robot1', 'robot2'}:
                combined = summarize_frontier_regions(
                    self._candidate_region_snapshots['robot1'] +
                    self._candidate_region_snapshots['robot2'],
                    self._terminal_small_frontier_length_m,
                    self._minimum_solo_visible_gain_m,
                )
                # Store the unique physical union once.  This keeps status
                # reporting and terminal classification from summing peer
                # replicas as if they were separate frontiers.
                self._candidate_evidence['robot1'] = combined
                self._candidate_evidence['robot2'] = CandidateEvidence()
                return
        self._candidate_evidence[message.source_robot_id] = fallback

    @staticmethod
    def _candidate_bound_context_key(snapshot: TaskSnapshot) -> tuple:
        """Return the complete source context used by certificate evidence."""
        return (
            int(getattr(snapshot, 'map_revision', 0) or 0),
            str(getattr(snapshot, 'lower_bound_context_fingerprint', '') or ''),
            int(getattr(snapshot, 'costmap_revision', 0) or 0),
            int(getattr(snapshot, 'candidate_generation_id', 0) or 0),
        )

    def _candidate_bound_record_for_snapshot(
            self, robot_id: str, snapshot: Optional[TaskSnapshot]) -> Optional[dict]:
        """Return only candidate bounds proven coherent with ``snapshot``.

        Candidate and snapshot subscriptions are asynchronous.  The latest
        candidate record can be one generation ahead of a retained round, but
        an exact older record may still be valid for that round.  Selecting
        that record is a provenance-preserving join; it never authorizes a
        cross-generation bound.
        """
        if snapshot is None:
            return None
        metadata = getattr(self, '_candidate_lower_bound_metadata', {}).get(
            robot_id)
        bounds = getattr(self, '_unqueried_cost_bounds', {}).get(robot_id)
        provenance = getattr(
            self, '_unqueried_cost_bound_provenance', {}).get(robot_id)
        current_evidence = getattr(
            self, '_candidate_source_local_evidence',
            getattr(self, '_candidate_evidence', {}),
        ).get(robot_id, CandidateEvidence())
        expected_count = int(getattr(
            current_evidence, 'detected_not_queried', 0) or 0)

        def usable(item_metadata, item_bounds, item_provenance, source):
            if not lower_bound_context_matches(item_provenance, snapshot):
                return None
            item_costmap = item_metadata.get('costmap_revision')
            if (item_costmap is None or
                    int(item_costmap) != int(getattr(
                        snapshot, 'costmap_revision', 0) or 0)):
                return None
            item_expected = int(item_metadata.get(
                'detected_not_queried_count', expected_count) or 0)
            reason, _comparison = classify_lower_bound_evidence(
                item_metadata, snapshot, item_bounds, item_expected,
            )
            # No bound values are required when this exact source context
            # reports zero unqueried options.  Preserve the existing
            # diagnostic distinction for empty/omitted summaries.
            if reason not in (
                    'OK', 'EMPTY_BOUND_SUMMARY',
                    'NO_CANDIDATE_BOUND_SUMMARY'):
                return None
            if item_expected > 0 and item_bounds is None:
                return None
            return {
                'bounds': item_bounds,
                'provenance': item_provenance,
                'metadata': item_metadata,
                'source': source,
                'expected_count': item_expected,
            }

        if metadata is not None:
            current = usable(metadata, bounds, provenance, 'latest')
            if current is not None:
                return current
        history = getattr(self, '_candidate_lower_bound_history', {}).get(
            robot_id, {})
        item = history.get(self._candidate_bound_context_key(snapshot))
        if item is None:
            return None
        return usable(
            item.get('metadata', {}), item.get('bounds'),
            item.get('provenance'), 'matching_history',
        )

    @staticmethod
    def _decode_frontier_regions(
            value: str, candidate_generation_id: int = 0,
            map_revision: int = 0, costmap_revision: int = 0,
    ) -> tuple[FrontierRegionEvidence, ...]:
        """Decode compact diagnostic region geometry; malformed data is ignored."""
        if not value:
            return ()
        try:
            payload = json.loads(value)
        except (TypeError, ValueError):
            return ()
        if isinstance(payload, dict):
            payload = payload.get('regions', [])
        if not isinstance(payload, list):
            return ()
        output = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            physical_id = str(item.get('physical_id', item.get('id', '')))
            if not physical_id:
                continue
            try:
                size_m = float(item.get('size_m', 0.0))
                if size_m <= 0.0:
                    # Backward-compatible decoding for older diagnostic
                    # captures, whose schema used the production 0.03 m map.
                    size_m = float(item.get('cell_count', 0.0)) * 0.03
                gain_value = item.get('visible_reveal_gain')
                visible_reveal_gain = (
                    float(gain_value) if gain_value is not None else None)
                lower_bound_value = item.get('optimistic_cost_lower_bound_s')
                lower_bound = (
                    float(lower_bound_value)
                    if lower_bound_value is not None else None)
                item_query_count = int(item.get('query_count', 0) or 0)
                item_cycles_seen = int(item.get('cycles_seen', 0) or 0)
                item_cycles_not_queried = int(
                    item.get('cycles_not_queried', 0) or 0)
                item_last_query_ns = int(item.get('last_query_ns', 0) or 0)
            except (TypeError, ValueError):
                continue
            output.append(FrontierRegionEvidence(
                physical_id=physical_id,
                size_m=max(0.0, size_m),
                status=str(item.get('status', 'UNCLASSIFIED')),
                visible_reveal_gain=visible_reveal_gain,
                optimistic_cost_lower_bound_s=lower_bound,
                candidate_generation_id=int(item.get(
                    'candidate_generation_id', candidate_generation_id) or 0),
                map_revision=int(item.get('map_revision', map_revision) or 0),
                costmap_revision=int(item.get(
                    'costmap_revision', costmap_revision) or 0),
                query_count=item_query_count,
                cycles_seen=item_cycles_seen,
                cycles_not_queried=item_cycles_not_queried,
                last_query_ns=item_last_query_ns,
                last_query_result=str(item.get('last_query_result', '') or ''),
            ))
        return tuple(output)
    def _snapshot_callback(self, message: TaskSnapshotMsg) -> None:
        """Accept only bounded monotonic source-local snapshot provenance."""
        try:
            snapshot = snapshot_from_msg(message)
        except (ValueError, TypeError) as error:
            self.get_logger().error('TASK_SNAPSHOT_REJECTED decode=%s' % error)
            return
        now = time.monotonic()
        previous = self._snapshots.get(snapshot.source_robot_id)
        if not self._ledger.accept(snapshot):
            # An exact immutable retransmission is a heartbeat, not a new epoch.
            if previous is not None and previous.value == snapshot:
                self._snapshots[snapshot.source_robot_id] = receive(
                    snapshot, snapshot.validity_s, now,
                )
                if snapshot.source_robot_id == self._peer_id:
                    self._peer_liveness.observe(snapshot.source_session_id, now)
            return
        map_version = (
            snapshot.source_session_id, snapshot.map_revision,
            snapshot.map_fingerprint,
        )
        if self._map_versions.get(snapshot.source_robot_id) != map_version:
            self._map_versions[snapshot.source_robot_id] = map_version
            self._maps_stable_since_steady_s = now
            self._completion_candidate_since_steady_s = None
            self._completion_candidate_reason = ''
            self._planner_failure_candidate_since_steady_s = None
        self._snapshots[snapshot.source_robot_id] = receive(
            snapshot, snapshot.validity_s, now,
        )
        if (snapshot.source_robot_id == self._robot_id and snapshot.tasks and
                getattr(self, '_current_local_frontier_ids', {}).get(
                    self._robot_id) is not None):
            self._local_fallback_trigger_pending = True
            self._local_fallback_trigger_reason = (
                'local task snapshot became current while cooperative '
                'evidence was pending')
        if (not self._local_only and snapshot.tasks and
                snapshot.source_robot_id not in
                self._first_nonempty_task_snapshot_logged):
            self._first_nonempty_task_snapshot_logged.add(
                snapshot.source_robot_id)
            self._startup_event(
                'FIRST_NONEMPTY_TASK_SNAPSHOT_%s' %
                snapshot.source_robot_id.upper(),
                snapshot_epoch=snapshot.epoch,
                task_count=len(snapshot.tasks),
            )
        if snapshot.source_robot_id == self._peer_id:
            previous_session = (
                '' if previous is None else previous.value.source_session_id
            )
            if previous_session and previous_session != snapshot.source_session_id:
                self._emit_event(
                    'PEER_SESSION_RESTART',
                    'fresh peer session superseded prior session',
                )
                self._clear_active_commitment(
                    self._peer_id, 'peer session restarted; commitment invalidated',
                )
                if self._nav2.local_goal_active:
                    self._request_navigation_cancel()
                elif self._dispatch_in_progress:
                    self._invalidate_round(
                        FailureClass.EXPLICIT_CANCELLATION,
                        'peer session restarted before local dispatch',
                    )
            previous_state = self._peer_liveness.state
            self._peer_liveness.observe(snapshot.source_session_id, now)
            if (previous_state == CoordinatorState.DEGRADED_SOLO and
                    self._peer_liveness.state != CoordinatorState.DEGRADED_SOLO):
                self._reset_round('fresh peer session handshake')

    def _bid_callback(self, message: TaskBidArrayMsg) -> None:
        try:
            batch = bid_batch_from_msg(message)
        except (ValueError, TypeError) as error:
            self.get_logger().error('BID_REJECTED decode=%s' % error)
            return
        if batch.source_robot_id not in ('robot1', 'robot2'):
            return
        self._bid_batches[batch.source_robot_id] = receive(
            batch, batch.validity_s, time.monotonic(),
        )

    def _decision_callback(self, message: PairDecisionMsg) -> None:
        if message.source_robot_id != self._peer_id:
            return
        self._peer_decision = receive(
            message, duration_to_seconds(message.validity), time.monotonic(),
        )

    def _status_callback(self, message: DistributedExplorationStatus) -> None:
        if message.source_robot_id != self._peer_id:
            return
        peer_session = uuid_to_text(message.source_session_id)
        prior_commitment = self._active_commitments.get(self._peer_id)
        if peer_session:
            # Status is the peer liveness heartbeat.  Candidate snapshots are
            # proposals and may legitimately stop while the peer navigates.
            self._peer_liveness.observe(peer_session, time.monotonic())
        self._peer_status = receive(
            message, duration_to_seconds(message.validity), time.monotonic(),
        )
        if prior_commitment is not None:
            peer_active = bool(
                message.local_nav_goal_active or
                message.state == DistributedExplorationStatus.NAVIGATING
            )
            advertised_task_id = str(message.active_canonical_task_id).strip()
            # Some status heartbeats prove active liveness and session but do
            # not carry the optional active task identity.  That omission
            # must not discard the immutable commitment needed by the free
            # robot's continuation round.  A non-empty mismatch remains a
            # hard invalidation guard.
            # A status heartbeat is not the terminal event for an already
            # committed peer action.  LocalNav2 clears its action handle
            # before the allocator publishes NAVIGATION_*; DDS may therefore
            # deliver the inactive heartbeat before the terminal event.  Do
            # not erase the peer commitment in that drain window.  Session
            # changes remain authoritative, and an actively advertised
            # different task still proves supersession.
            if (peer_session != prior_commitment.source_session_id or
                    (peer_active and advertised_task_id and
                     advertised_task_id != prior_commitment.canonical_id)):
                self._clear_active_commitment(
                    self._peer_id, 'peer active commitment ended or changed',
                )

    def _failure_callback(self, message: ExplorationFailure) -> None:
        if message.source_robot_id == self._robot_id:
            return
        hard_values = {FAILURE_TO_MESSAGE[item] for item in HARD_FAILURES}
        if (message.failure_class in hard_values and
                message.physical_task_signature):
            self._record_hard_failure(
                message.physical_task_signature,
                duration_to_seconds(message.validity),
                canonical_task_id=str(message.canonical_task_id),
                failure_class=str(message.failure_class),
                reason='peer failure message',
            )

    def _navigation_event_matches_commitment(
            self, message: DistributedExplorationEvent,
            commitment: ActiveCommitment) -> bool:
        """Require complete available identity before consuming peer terminal evidence."""
        expected_signatures = {
            str(member.physical_signature)
            for member in getattr(commitment.task, 'members', ())
            if str(member.physical_signature)
        }
        return bool(
            str(getattr(message, 'canonical_task_id', '')) ==
            str(commitment.canonical_id) and
            str(getattr(message, 'physical_task_signature', '')) in
            expected_signatures and
            str(getattr(message, 'round_id', '')) ==
            str(commitment.decision_round_id) and
            str(getattr(message, 'decision_hash', '')) ==
            str(commitment.decision_hash) and
            uuid_to_text(getattr(message, 'source_session_id', None)) ==
            str(commitment.source_session_id)
        )

    def _peer_event_callback(self, message: DistributedExplorationEvent) -> None:
        """Replicate event-driven traffic release without a coordinator."""
        if message.source_robot_id != self._peer_id:
            return
        if message.event_type.startswith('NAVIGATION_'):
            commitment = self._active_commitments.get(self._peer_id)
            if (commitment is None or
                    not self._navigation_event_matches_commitment(
                        message, commitment)):
                self.get_logger().warning(
                    'STALE_OR_FOREIGN_NAVIGATION_EVENT_IGNORED robot=%s '
                    'event=%s task=%s round=%s decision=%s' % (
                        self._robot_id, message.event_type,
                        message.canonical_task_id, message.round_id,
                        message.decision_hash),
                )
                return
            self._clear_active_commitment(
                self._peer_id, 'peer navigation commitment terminated',
            )
            if message.event_type == 'NAVIGATION_SUCCEEDED':
                self._completed_shared_canonical_ids.add(
                    str(message.canonical_task_id))
                self.get_logger().info(
                    'COMPLETED_FRONTIER_REPLICATED robot=%s peer=%s task=%s' % (
                        self._robot_id, self._peer_id,
                        message.canonical_task_id))
            return
        if message.event_type != 'TRAFFIC_CONFLICT_CLEARED':
            return
        key = (str(message.round_id), str(message.decision_hash))
        if key == self._last_peer_traffic_clear_key:
            return
        if not self._retain_traffic_cleared_round(
                message.round_id, message.decision_hash, self._robot_id):
            return
        self._last_peer_traffic_clear_key = key
        self._transition(
            CoordinatorState.WAITING_FOR_MATCHING_DECISION,
            'peer traffic conflict cleared; retaining agreed deferred task',
        )

    def _record_hard_failure(
            self, signature: str, requested_ttl_s: float, *,
            canonical_task_id: str = '', failure_class: str = '',
            reason: str = '') -> None:
        """Record bounded suppression for directly observed hard evidence."""
        if not signature:
            return
        now = time.monotonic()
        count = self._hard_failure_counts.get(signature, 0) + 1
        self._hard_failure_counts[signature] = count
        ttl = bounded_suppression_duration(count, requested_ttl_s)
        self._hard_failure_signatures[signature] = max(
            self._hard_failure_signatures.get(signature, 0.0), now + ttl,
        )
        if len(self._hard_failure_counts) > 128:
            expired = [
                key for key in self._hard_failure_counts
                if key not in self._hard_failure_signatures
            ]
            if expired:
                self._hard_failure_counts.pop(expired[0], None)
        # Keep this diagnostic bounded and auditable without making failure
        # suppression dependent on logger timing.
        self.get_logger().info(
            'HARD_FAILURE_SUPPRESSION signature=%s count=%d ttl_s=%.3f '
            'canonical_task_id=%s failure_class=%s reason=%s active=%s' %
            (signature, count, ttl, canonical_task_id, failure_class,
             reason, bool(ttl > 0.0)),
        )

    def _fresh_snapshot(self, robot_id: str, now: float) -> Optional[TaskSnapshot]:
        received = self._snapshots.get(robot_id)
        return received.value if received is not None and received.fresh(now) else None

    def _local_fallback_snapshot(self, now: float) -> Optional[TaskSnapshot]:
        """Return a current or safely revalidated local task seed.

        ``Received.fresh`` remains the authority for cooperative/peer
        evidence.  Local degraded-solo work has a different lifetime: an
        expired local snapshot may be retained only when the latest local
        candidate batch still advertises every task selected from it.  The
        final dispatch path then performs the existing current TF, map,
        costmap, Nav2 path, traffic, reservation, and failure checks.  An
        absent/empty current candidate set never authorizes a stale task.
        """
        received = self._snapshots.get(self._robot_id)
        if received is None:
            return None
        snapshot = received.value
        current_frontier_ids = getattr(
            self, '_current_local_frontier_ids', {}).get(self._robot_id)
        if current_frontier_ids is None:
            return snapshot if received.fresh(now) else None
        tasks = tuple(
            task for task in snapshot.tasks
            if str(task.local_frontier_id) in current_frontier_ids
        )
        if not tasks:
            return None
        if tasks == snapshot.tasks:
            return snapshot
        return replace(snapshot, tasks=tasks)

    def _consume_local_fallback_trigger(self) -> bool:
        """Start existing fallback before another cooperative wait.

        This is a scheduler hint only. The fallback still performs current
        source-local identity, peer-liveness, TF, path, reservation, failure,
        and final dispatch checks; no retained path is sent.
        """
        if not getattr(self, '_local_fallback_trigger_pending', False):
            return False
        reason = getattr(
            self, '_local_fallback_trigger_reason',
            'current local candidate became available',
        )
        self._local_fallback_trigger_pending = False
        self._local_fallback_trigger_reason = ''
        if (self._local_only or self._terminal or
                self._nav2.local_goal_active or self._dispatch_in_progress):
            getattr(self._nav2, 'clear_path_query_priority', lambda: None)()
            return False
        local = self._local_fallback_snapshot(time.monotonic())
        if local is None:
            getattr(self._nav2, 'clear_path_query_priority', lambda: None)()
            return False
        return self._continue_local_work_while_waiting(local, reason)

    def _temporary_peer_reservation_ids(self, now: float) -> frozenset[str]:
        """Return task IDs advertised by the peer's degraded-solo fallback.

        A degraded-solo goal is a temporary local commitment, not a normal
        cooperative agreement.  It is nevertheless advertised through the
        existing distributed status heartbeat so the peer cannot select the
        same canonical task while cooperative evidence is catching up.
        Normal cooperative commitments use their ordinary decision hash and
        are deliberately not included here.
        """
        peer = self._peer_status
        if peer is None or not peer.fresh(now):
            return frozenset()
        task_id = str(peer.value.active_canonical_task_id).strip()
        if not task_id:
            return frozenset()
        if (str(peer.value.decision_hash) != 'DEGRADED_SOLO' and
                peer.value.state != DistributedExplorationStatus.DEGRADED_SOLO):
            return frozenset()
        return frozenset((task_id,))

    def _peer_unavailable_for_degraded_solo(self, now: float) -> bool:
        """Require bounded peer-loss evidence before cooperative solo dispatch."""
        peer_status = self._peer_status
        if peer_status is not None and peer_status.fresh(now):
            return False
        peer_snapshot = getattr(self, '_snapshots', {}).get(self._peer_id)
        if peer_snapshot is not None and peer_snapshot.fresh(now):
            return False
        return self._peer_liveness.evaluate(now) == CoordinatorState.DEGRADED_SOLO

    def _continue_local_work_while_waiting(
            self, snapshot: Optional[TaskSnapshot], reason: str) -> bool:
        """Use local fallback only after bounded peer unavailability.

        This does not manufacture a pair decision or relax the certificate.
        A responsive peer keeps selection on the paired path, where the
        authoritative traffic scheduler has both routes.  After the existing
        peer timeout, the temporary task is published through the normal status
        heartbeat and excluded by ``_temporary_peer_reservation_ids`` on the
        peer.
        """
        if (self._local_only or self._nav2.local_goal_active or
                self._dispatch_in_progress):
            return False
        now = time.monotonic()
        if not self._peer_unavailable_for_degraded_solo(now):
            return False
        # In the cooperative path, ignore the round's possibly expired
        # snapshot and select from the latest local seed that passed the
        # source-local candidate check.  Keep the argument for call-site and
        # test compatibility; it is only a fallback for ROS-free fixtures.
        if hasattr(self, '_snapshots'):
            snapshot = self._local_fallback_snapshot(now)
        if snapshot is None:
            return False
        self._continue_degraded_solo(snapshot)
        selected = (
            self._active_task is not None and
            self._active_decision_hash == 'DEGRADED_SOLO')
        if selected and not self._nav2.local_goal_active:
            self._transition(CoordinatorState.DEGRADED_SOLO, reason)
            self._publish_status()
            self._emit_event(
                'DEGRADED_SOLO_COMMITMENT',
                'temporary local work advertised while cooperative evidence waits',
            )
        return selected

    def _start_immediate_fallback_after_terminal(self) -> None:
        """Start peer-loss continuation without waiting for the next tick.

        A terminal navigation callback already establishes that this robot is
        free.  A local candidate that arrived while this robot was busy is
        already a scheduler trigger; consuming it here avoids waiting for
        another cooperative tick only when bounded liveness evidence has
        established peer unavailability.  A responsive peer resumes through
        the normal paired traffic path instead.
        """
        if self._local_only or self._terminal or self._nav2.local_goal_active:
            return
        if self._consume_local_fallback_trigger():
            return
        now = time.monotonic()
        local = self._local_fallback_snapshot(now)
        if local is not None:
            self._continue_local_work_while_waiting(
                local, 'immediate local work after navigation terminal result')

    @staticmethod
    def _finite_path_samples(path: tuple[tuple[float, float], ...]) -> bool:
        """Return whether a retained commitment has usable traffic geometry."""
        return bool(path) and all(
            math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
            for point in path
        )

    @staticmethod
    def _continuation_free_action_lineage(
            busy_commitment_id: str, free_robot_id: str,
            source_snapshot: TaskSnapshot, task: CanonicalTask,
    ) -> tuple[str, str, str]:
        """Derive one immutable lineage for a newly assigned free robot.

        Continuation round/decision IDs are local coordination wrappers and
        may differ between replicas.  The physical action lineage instead
        binds the retained busy action, free source provenance, and the exact
        canonical task contents.  Source epochs/session changes therefore
        produce a distinct action while equivalent replicas derive the same
        identity.
        """
        payload = {
            'kind': 'continuation-free-action',
            'busy_commitment_id': str(busy_commitment_id),
            'free_robot_id': str(free_robot_id),
            'source_session_id': str(source_snapshot.source_session_id),
            'source_snapshot_epoch': int(source_snapshot.epoch),
            'canonical_task_id': str(task.canonical_id),
            'physical_signatures': tuple(
                str(member.physical_signature) for member in task.members
            ),
            'canonical_task_repr': repr(task),
        }
        seed = hashlib.sha256(json.dumps(
            payload, sort_keys=True, separators=(',', ':'),
        ).encode('utf-8')).hexdigest()
        commitment_id = hashlib.sha256(
            ('continuation-free-commitment:' + seed).encode('utf-8'),
        ).hexdigest()
        decision_round_id = 'continuation-action-round:' + seed
        decision_hash = hashlib.sha256(
            ('continuation-free-decision:' + seed).encode('utf-8'),
        ).hexdigest()
        return commitment_id, decision_round_id, decision_hash

    def _remember_active_commitments(self, round_work: RoundWork) -> None:
        """Retain only the agreed task/path needed by future continuation rounds."""
        if round_work.decision is None:
            return
        batches = {
            'robot1': self._bid_batches.get('robot1'),
            'robot2': self._bid_batches.get('robot2'),
        }
        tasks = {task.canonical_id: task for task in round_work.union.tasks}
        for robot_id in ('robot1', 'robot2'):
            task_id = (
                round_work.decision.robot1_task_id if robot_id == 'robot1'
                else round_work.decision.robot2_task_id
            )
            if not task_id:
                continue
            task = tasks.get(task_id)
            received = batches[robot_id]
            if task is None or received is None:
                continue
            bid = next(
                (item for item in received.value.bids
                 if item.canonical_task_id == task_id), None,
            )
            if (bid is None or not bid.path_valid or
                    not math.isfinite(float(bid.path_length_m)) or
                    not self._finite_path_samples(tuple(bid.path))):
                continue
            # Normal rounds store snapshots in robot order.  A continuation
            # round deliberately stores the free snapshot beside a synthetic
            # busy snapshot, so always bind provenance by source identity.
            source_snapshot = next(
                (snapshot for snapshot in round_work.snapshots
                 if snapshot.source_robot_id == robot_id),
                None,
            )
            if source_snapshot is None:
                continue
            existing = self._active_commitments.get(robot_id)
            if (
                    existing is not None and
                    existing.canonical_id == task_id and
                    existing.source_session_id == source_snapshot.source_session_id and
                    existing.task == task
            ):
                # A continuation round may wrap the coordination context, but
                # it does not create a new physical action for the already
                # busy robot.  Preserve the original decision lineage so its
                # eventual terminal remains matchable by the peer.
                continue
            commitment_id = hashlib.sha256(repr((
                robot_id, source_snapshot.source_session_id,
                source_snapshot.epoch, task_id,
                round_work.round_id, round_work.decision.decision_hash,
            )).encode('utf-8')).hexdigest()
            decision_round_id = round_work.round_id
            decision_hash = round_work.decision.decision_hash
            if (
                    round_work.mode == 'continuation' and
                    robot_id == round_work.continuation_free_robot_id and
                    existing is None
            ):
                busy_commitment = self._active_commitments.get(
                    round_work.continuation_busy_robot_id,
                )
                busy_commitment_id = (
                    busy_commitment.commitment_id
                    if busy_commitment is not None
                    else round_work.continuation_commitment_id
                )
                if busy_commitment_id:
                    (
                        commitment_id,
                        decision_round_id,
                        decision_hash,
                    ) = self._continuation_free_action_lineage(
                        busy_commitment_id, robot_id, source_snapshot, task,
                    )
            self._active_commitments[robot_id] = ActiveCommitment(
                robot_id=robot_id,
                source_session_id=source_snapshot.source_session_id,
                source_snapshot_epoch=source_snapshot.epoch,
                canonical_id=task_id,
                task=task,
                decision_round_id=decision_round_id,
                decision_hash=decision_hash,
                path=tuple(bid.path),
                path_length_m=float(bid.path_length_m),
                heading_cost_rad=float(bid.heading_cost),
                commitment_id=commitment_id,
            )

    def _selector_feasibility_identity(
            self, union: CanonicalUnion, hard_failed_task_ids,
            completed_task_ids, peer_reservation_task_ids):
        """Return the exact suppression identity used by pair selection."""
        task_ids = tuple(task.canonical_id for task in union.tasks)
        fingerprint, selected = selector_feasibility_identity(
            task_ids, hard_failed_task_ids, completed_task_ids,
            peer_reservation_task_ids,
        )
        return fingerprint, selected

    def _selector_decision_requires_recompute(
            self, round_work: RoundWork, selector_fingerprint: str,
            selector_inputs: dict[str, tuple[str, ...]]) -> bool:
        """Invalidate only an uncommitted decision whose selector inputs changed."""
        if round_work.decision is None or self._committed.decision is not None:
            return False
        prior = str(
            getattr(
                round_work.decision.diagnostics,
                'selector_feasibility_fingerprint', '',
            ) or ''
        )
        if prior == selector_fingerprint:
            return False
        self._emit_event(
            'SELECTOR_FEASIBILITY_CHANGED_RECOMPUTE',
            json.dumps({
                'old_fingerprint': prior,
                'new_fingerprint': selector_fingerprint,
                'completed': selector_inputs['completed'],
                'hard_failed': selector_inputs['hard_failed'],
                'peer_reservations': selector_inputs['peer_reservations'],
            }, sort_keys=True, separators=(',', ':')),
        )
        round_work.decision = None
        round_work.decision_published = False
        round_work.traffic = None
        self._peer_decision = None
        return True

    @staticmethod
    def _peer_selector_feasibility_fingerprint(message) -> str:
        """Read selector provenance from the existing decision diagnostics."""
        try:
            payload = json.loads(str(getattr(message, 'diagnostics_json', '') or '{}'))
        except (TypeError, ValueError, json.JSONDecodeError):
            return ''
        return str(payload.get('selector_feasibility_fingerprint', '') or '')

    def _decision_identity_diagnostic(
            self, event_type: str, round_work: RoundWork, payload: dict) -> None:
        """Emit one bounded diagnostic without creating a retry protocol."""
        key = (
            round_work.round_id, event_type,
            json.dumps(payload, sort_keys=True, separators=(',', ':')),
        )
        marker = (
            '_last_selector_divergence_key'
            if event_type == 'SELECTOR_FEASIBILITY_DIVERGENCE'
            else '_last_decision_invariant_violation_key'
        )
        if key == getattr(self, marker, None):
            return
        setattr(self, marker, key)
        self._emit_event(
            event_type,
            json.dumps(payload, sort_keys=True, separators=(',', ':')),
        )
        self.get_logger().warning(
            '%s robot=%s round=%s details=%s' % (
                event_type, self._robot_id, round_work.round_id,
                json.dumps(payload, sort_keys=True, separators=(',', ':')),
            ),
        )

    def _request_navigation_cancel(self) -> bool:
        """Request Nav2 cancellation while retaining action ownership."""
        requested = self._nav2.cancel_navigation()
        action = getattr(self, '_active_navigation_action', None)
        if requested and action is not None:
            action.state = 'CANCELLING'
        return requested

    def _clear_active_commitment(self, robot_id: str, reason: str) -> None:
        """Drop one task commitment and invalidate any continuation using it."""
        if robot_id not in self._active_commitments:
            return
        self._active_commitments.pop(robot_id, None)
        if (self._round is not None and
                self._round.mode == 'continuation' and
                (self._round.continuation_busy_robot_id == robot_id or
                 self._round.continuation_free_robot_id == robot_id)):
            self._reset_round(reason)

    def _continuation_context(self, now: float) -> Optional[ContinuationContext]:
        """Return a safe one-free/one-busy context, without using busy proposals."""
        peer = self._peer_status
        if peer is None or not peer.fresh(now):
            return None
        local_active = bool(
            self._nav2.local_goal_active or self._state == CoordinatorState.NAVIGATING
        )
        peer_active = bool(
            peer.value.local_nav_goal_active or
            peer.value.state == DistributedExplorationStatus.NAVIGATING
        )
        if local_active == peer_active:
            return None
        busy_robot_id = self._robot_id if local_active else self._peer_id
        free_robot_id = self._peer_id if local_active else self._robot_id
        commitment = self._active_commitments.get(busy_robot_id)
        if commitment is None or not self._finite_path_samples(commitment.path):
            return None
        if busy_robot_id == self._peer_id:
            advertised_task_id = str(
                peer.value.active_canonical_task_id,
            ).strip()
            if (uuid_to_text(peer.value.source_session_id) !=
                    commitment.source_session_id or
                    (advertised_task_id and
                     advertised_task_id != commitment.canonical_id)):
                return None
        elif (self._active_task is not None and
              self._active_task.canonical_id != commitment.canonical_id):
            return None
        free_snapshot = self._fresh_snapshot(free_robot_id, now)
        if free_snapshot is None:
            return None
        if free_robot_id == 'robot1':
            free_union = build_canonical_union(
                free_snapshot.tasks, (), self._maximum_union_tasks,
            )
        else:
            free_union = build_canonical_union(
                (), free_snapshot.tasks, self._maximum_union_tasks,
            )
        free_tasks = tuple(
            task for task in free_union.tasks
            if task.canonical_id != commitment.canonical_id and not any(
                equivalent_tasks(member, committed_member)
                for member in task.members
                for committed_member in commitment.task.members
            )
        )
        return ContinuationContext(
            free_robot_id=free_robot_id,
            busy_robot_id=busy_robot_id,
            free_snapshot=free_snapshot,
            commitment=commitment,
            free_tasks=free_tasks,
        )

    def _continuation_round_requires_invalidation(self, now: float) -> bool:
        """Return whether missing continuation context is a hard invalidation.

        Continuation context can disappear temporarily while peer status or the
        free robot's proposal is between fresh receipts.  That is not an
        allocation change, so clearing the round would discard valid bids and
        create a canonical/continuation oscillation.  Retain the round in that
        case, but still invalidate it when a commitment, session, active-goal
        identity, or known safety state has changed.
        """
        round_work = self._round
        if (round_work is None or round_work.mode != 'continuation'):
            return False
        busy_robot_id = round_work.continuation_busy_robot_id
        free_robot_id = round_work.continuation_free_robot_id
        commitment = self._active_commitments.get(busy_robot_id)
        if (
                not busy_robot_id or not free_robot_id or
                commitment is None or
                commitment.commitment_id != round_work.continuation_commitment_id or
                not self._finite_path_samples(commitment.path)):
            return True

        # Once the formerly free robot has its own commitment, this is no
        # longer the one-free/one-busy allocation represented by the round.
        if self._active_commitments.get(free_robot_id) is not None:
            return True

        local_active = bool(
            self._nav2.local_goal_active or
            self._state == CoordinatorState.NAVIGATING
        )
        peer = self._peer_status
        peer_fresh = peer is not None and peer.fresh(now)
        peer_active = bool(
            peer_fresh and (
                peer.value.local_nav_goal_active or
                peer.value.state == DistributedExplorationStatus.NAVIGATING
            )
        )

        if busy_robot_id == self._robot_id:
            if not local_active:
                return True
            if (self._active_task is not None and
                    self._active_task.canonical_id != commitment.canonical_id):
                return True
        elif peer_fresh:
            advertised_task_id = str(
                peer.value.active_canonical_task_id,
            ).strip()
            if (
                    not peer_active or
                    uuid_to_text(peer.value.source_session_id) !=
                    commitment.source_session_id or
                    (advertised_task_id and
                     advertised_task_id != commitment.canonical_id)):
                return True

        # A fresh indication that the formerly free side is active is a
        # commitment/safety change.  An absent or stale peer indication is
        # deliberately treated as temporary below, and blocks progress until
        # fresh evidence returns rather than authorizing a dispatch.
        if free_robot_id == self._robot_id:
            if local_active:
                return True
        elif peer_fresh and peer_active:
            return True

        # A fresh free snapshot with no independent task is a semantic change;
        # an absent/stale snapshot is only temporary evidence loss.
        free_snapshot = self._fresh_snapshot(free_robot_id, now)
        if free_snapshot is not None:
            if free_robot_id == 'robot1':
                free_union = build_canonical_union(
                    free_snapshot.tasks, (), self._maximum_union_tasks,
                )
            else:
                free_union = build_canonical_union(
                    (), free_snapshot.tasks, self._maximum_union_tasks,
                )
            free_tasks = tuple(
                task for task in free_union.tasks
                if task.canonical_id != commitment.canonical_id and not any(
                    equivalent_tasks(member, committed_member)
                    for member in task.members
                    for committed_member in commitment.task.members
                )
            )
            if not free_tasks:
                return True
        return False

    def _continuation_round_id(
            self, context: ContinuationContext,
            allocation_fingerprint: str) -> str:
        """Hash allocation semantics, not the free snapshot heartbeat epoch."""
        return hashlib.sha256(repr((
            'continuation', context.free_robot_id,
            context.free_snapshot.source_session_id,
            context.commitment.robot_id,
            context.commitment.commitment_id,
            allocation_fingerprint,
        )).encode('utf-8')).hexdigest()

    def _activate_continuation_round(
            self, context: ContinuationContext) -> bool:
        """Install a continuation round or retain its unchanged generation."""
        if not context.free_tasks:
            if (self._round is not None and
                    self._round.mode == 'continuation'):
                self._reset_round('no independent continuation task remains')
            self._transition(
                CoordinatorState.WAITING_FOR_INPUTS,
                'no independent continuation task remains',
            )
            return False
        commitment = context.commitment
        busy_snapshot = TaskSnapshot(
            source_robot_id=commitment.robot_id,
            source_session_id=commitment.source_session_id,
            epoch=commitment.source_snapshot_epoch,
            map_revision=max((member.source_map_revision
                               for member in commitment.task.members), default=0),
            map_fingerprint='active-commitment:' + commitment.commitment_id,
            generation_ros_ns=max((member.generation_ros_ns
                                   for member in commitment.task.members), default=0),
            validity_s=self._bid_validity_s,
            tasks=commitment.task.members,
        )
        if context.free_robot_id == 'robot1':
            snapshots = (context.free_snapshot, busy_snapshot)
        else:
            snapshots = (busy_snapshot, context.free_snapshot)
        union_tasks = tuple(sorted(
            (commitment.task,) + context.free_tasks,
            key=lambda task: task.canonical_id,
        ))
        union_hash = hashlib.sha256(repr(tuple(
            task.canonical_id for task in union_tasks
        )).encode('utf-8')).hexdigest()
        union = CanonicalUnion(tasks=union_tasks, union_hash=union_hash)
        content_fingerprint = self._allocation_semantic_fingerprint(*snapshots)
        round_id = self._continuation_round_id(context, content_fingerprint)
        current = self._round
        if (current is not None and current.mode == 'continuation' and
                current.round_id == round_id and
                current.continuation_commitment_id == commitment.commitment_id):
            # A semantic continuation round may span source heartbeat epochs,
            # but its certificate context must still be a coherent snapshot /
            # bound pair.  Refresh the retained pre-decision context only when
            # the newer free snapshot already has matching candidate evidence.
            # Replacing the tuple as one value and advancing the lifecycle
            # generation makes callbacks from the previous context stale.
            committed = getattr(self, '_committed', None)
            if (current.decision is None and
                    getattr(committed, 'decision', None) is None):
                old_snapshot = next(
                    (snapshot for snapshot in current.snapshots
                     if snapshot.source_robot_id == context.free_robot_id),
                    None,
                )
                matching = self._candidate_bound_record_for_snapshot(
                    context.free_robot_id, context.free_snapshot,
                )
                if (old_snapshot != context.free_snapshot and
                        matching is not None):
                    current.snapshots = snapshots
                    current.query_tasks = context.free_tasks
                    current.bids = ()
                    current.local_path_evaluations.clear()
                    current.local_batch = None
                    current.query_index = 0
                    current.decision = None
                    current.traffic = None
                    current.decision_published = False
                    self._round_lifecycle.activate(current.round_id)
                    self._bid_batches.clear()
                    self._peer_decision = None
                    self._committed = CommittedRound()
                    self._transition(
                        CoordinatorState.BIDDING,
                        'refresh continuation certificate context',
                    )
                    self._log_round_lifecycle(
                        'CONTINUATION_CONTEXT_REFRESHED', current,
                        self._round_lifecycle.generation,
                        'new coherent snapshot and candidate-bound provenance',
                    )
                    self._emit_event(
                        'CONTINUATION_CONTEXT_REFRESHED',
                        json.dumps({
                            'round_id': current.round_id,
                            'robot_id': context.free_robot_id,
                            'old_candidate_generation_id': int(getattr(
                                old_snapshot, 'candidate_generation_id', 0) or 0),
                            'new_candidate_generation_id': int(getattr(
                                context.free_snapshot,
                                'candidate_generation_id', 0) or 0),
                            'bound_context_source': matching['source'],
                        }, sort_keys=True, separators=(',', ':')),
                    )
            return True
        new_round = RoundWork(
            round_id=round_id,
            union=union,
            snapshots=snapshots,
            query_tasks=context.free_tasks,
            content_fingerprint=content_fingerprint,
            mode='continuation',
            continuation_free_robot_id=context.free_robot_id,
            continuation_busy_robot_id=context.busy_robot_id,
            continuation_commitment_id=commitment.commitment_id,
        )
        generation = self._activate_round(new_round, 'new continuation round')
        self._bid_batches.clear()
        self._peer_decision = None
        self._committed = CommittedRound()
        self._transition(CoordinatorState.BIDDING, 'new continuation round')
        self._log_union(new_round)
        self._emit_event(
            'CONTINUATION_ROUND_STARTED',
            json.dumps({
                'mode': 'continuation',
                'free_robot': context.free_robot_id,
                'busy_robot': context.busy_robot_id,
                'commitment_id': commitment.commitment_id,
            }, sort_keys=True, separators=(',', ':')),
        )
        return bool(generation >= 0)

    def _continuation_busy_batch(
            self, round_work: RoundWork) -> Optional[BidBatch]:
        """Create a deterministic one-task bid for the fixed active commitment."""
        commitment = self._active_commitments.get(
            round_work.continuation_busy_robot_id,
        )
        if commitment is None or commitment.commitment_id != \
                round_work.continuation_commitment_id:
            return None
        source_stamp = max((member.generation_ros_ns
                            for member in commitment.task.members), default=0)
        return BidBatch(
            round_id=round_work.round_id,
            union_hash=round_work.union.union_hash,
            source_robot_id=commitment.robot_id,
            source_session_id=commitment.source_session_id,
            source_snapshot_epoch=commitment.source_snapshot_epoch,
            validity_s=self._bid_validity_s,
            bids=(Bid(
                canonical_task_id=commitment.canonical_id,
                path_valid=True,
                path_length_m=commitment.path_length_m,
                estimated_travel_cost=commitment.path_length_m,
                heading_cost=commitment.heading_cost_rad,
                own_utility_contribution=0.0,
                task_generation_ros_ns=source_stamp,
                path_query_ros_ns=source_stamp,
                path=commitment.path,
            ),),
        )

    def _continuation_bid_batches(
            self, now: float, round_work: RoundWork) -> Optional[
                tuple[BidBatch, BidBatch]]:
        """Validate only the free robot's exchanged batch plus fixed peer state."""
        busy_batch = self._continuation_busy_batch(round_work)
        if busy_batch is None:
            return None
        free_received = self._bid_batches.get(
            round_work.continuation_free_robot_id,
        )
        if free_received is None:
            return None
        batches = {
            busy_batch.source_robot_id: receive(
                busy_batch, busy_batch.validity_s, now,
            ),
            round_work.continuation_free_robot_id: free_received,
        }
        output = []
        for index, robot_id in enumerate(('robot1', 'robot2')):
            snapshot = round_work.snapshots[index]
            received = batches.get(robot_id)
            if received is None or not bid_batch_valid(
                    received, now, robot_id, snapshot.source_session_id,
                    snapshot.epoch, round_work.round_id,
                    round_work.union.union_hash, True):
                return None
            output.append(received.value)
        return output[0], output[1]

    @staticmethod
    def _snapshot_content_fingerprint(
            first: TaskSnapshot, second: TaskSnapshot,
            include_provenance: bool = True) -> str:
        """Fingerprint tasks plus optional freshness/provenance evidence.

        The default form is retained for local replay/diagnostic consumers
        that intentionally care about source evidence provenance.  Allocation
        round identity uses ``_allocation_semantic_fingerprint`` below so
        epoch, map, costmap, and lower-bound heartbeat changes cannot churn an
        otherwise unchanged problem.
        """
        def rounded_points(points):
            return tuple(
                tuple(round(float(value), 3) for value in point)
                for point in points
            )

        def rounded_bounds(bounds):
            if bounds is None:
                return None
            return (
                rounded_points((bounds.minimum,))[0],
                rounded_points((bounds.maximum,))[0],
            )

        payload = []
        for snapshot in (first, second):
            tasks = []
            for task in sorted(snapshot.tasks, key=lambda item: item.physical_signature):
                tasks.append((
                    task.physical_signature,
                    int(getattr(task, 'local_frontier_id', 0)),
                    tuple(round(value, 3) for value in task.approach),
                    round(task.approach_yaw, 3),
                    tuple(round(value, 3) for value in task.bounds.minimum),
                    tuple(round(value, 3) for value in task.bounds.maximum),
                    rounded_points(getattr(task, 'frontier_geometry', ())),
                    rounded_points(getattr(task, 'visible_cells', ())),
                    rounded_bounds(getattr(task, 'visible_bounds', None)),
                    round(task.visible_reveal_gain, 4),
                    round(task.local_ordering_score, 4),
                    int(getattr(task, 'mrtsp_route_rank', 2 ** 32 - 1)),
                    int(getattr(task, 'mrtsp_route_generation', 0)),
                    str(getattr(task, 'mrtsp_solver', '')),
                    bool(getattr(task, 'local_path_valid', False)),
                    round(getattr(task, 'local_path_length_m', 0.0), 3),
                    rounded_points(getattr(task, 'local_path', ())),
                    round(getattr(task, 'path_heading_cost_rad', 0.0), 3),
                ))
            identity = [
                snapshot.source_robot_id,
                snapshot.source_session_id,
            ]
            if include_provenance:
                identity.extend((
                    getattr(snapshot, 'map_revision', 0),
                    getattr(snapshot, 'map_fingerprint', ''),
                    getattr(snapshot, 'lower_bound_context_fingerprint', ''),
                ))
            payload.append((*identity, tuple(tasks)))
        return hashlib.sha256(repr(tuple(payload)).encode('utf-8')).hexdigest()

    @staticmethod
    def _allocation_semantic_fingerprint(
            first: TaskSnapshot, second: TaskSnapshot) -> str:
        """Fingerprint only allocation-relevant task/path semantics."""
        return DistributedFrontierAssignment._snapshot_content_fingerprint(
            first, second, include_provenance=False,
        )

    def _round_is_current(self, round_work: RoundWork, generation: int) -> bool:
        """Check both object identity and generation before committing work."""
        return (
            self._round is round_work and
            self._round_lifecycle.generation == generation and
            self._round_lifecycle.active_round_id == round_work.round_id
        )

    def _discard_stale_tick(
            self, round_work: Optional[RoundWork], captured_generation: int,
            reason: str) -> None:
        """Account for asynchronous work that no longer owns the round."""
        self._stale_tick_discard_count += 1
        current_round = self._round
        self.get_logger().warning(
            'ALLOCATOR_TICK_STALE_DISCARDED robot=%s round_id=%s '
            'captured_generation=%d current_round_id=%s current_generation=%d '
            'reason=%s' % (
                self._robot_id,
                '' if round_work is None else round_work.round_id,
                captured_generation,
                '' if current_round is None else current_round.round_id,
                self._round_lifecycle.generation,
                reason,
            ),
        )

    def _log_round_lifecycle(
            self, event: str, round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None, reason: str = '') -> None:
        """Emit compact lifecycle telemetry for post-run race auditing."""
        current = self._round if round_work is None else round_work
        active_generation = self._round_lifecycle.generation
        logged_generation = active_generation if generation is None else generation
        self.get_logger().info(
            '%s robot=%s round_id=%s generation=%d current_generation=%d '
            'timestamp=%.6f reason=%s' % (
                event, self._robot_id,
                '' if current is None else current.round_id,
                logged_generation, active_generation, time.monotonic(), reason,
            ),
        )

    def _activate_round(self, round_work: RoundWork, reason: str) -> int:
        """Install a round and return its generation token."""
        previous = self._round
        lease = self._round_lifecycle.activate(round_work.round_id)
        self._round = round_work
        self._round_created_count += 1
        if previous is None:
            self._log_round_lifecycle(
                'ALLOCATOR_ROUND_CREATED', round_work, lease.generation, reason,
            )
        else:
            self._round_replaced_count += 1
            self._log_round_lifecycle(
                'ALLOCATOR_ROUND_REPLACED', round_work, lease.generation, reason,
            )
        return lease.generation

    def _publish_health_diagnostic(self) -> None:
        """Publish liveness counters without changing allocator behavior."""
        now = time.monotonic()
        current = self._round
        last_tick_age = max(0.0, now - self._last_tick_steady_s)
        completion_age = (
            -1.0 if self._last_round_completion_steady_s <= 0.0 else
            max(0.0, now - self._last_round_completion_steady_s)
        )
        self.get_logger().info(
            'ALLOCATOR_HEALTH robot=%s coordinator_alive=%s current_round_id=%s '
            'current_generation=%d last_tick_age_s=%.3f '
            'last_round_completion_age_s=%.3f stale_tick_discard_count=%d '
            'rounds_created=%d rounds_completed=%d rounds_replaced=%d '
            'dispatch_count=%d path_cache_hits=%d path_cache_misses=%d '
            'path_cache_invalidations=%d path_cache_evictions=%d' % (
                self._robot_id, self._coordinator_alive,
                '' if current is None else current.round_id,
                self._round_lifecycle.generation, last_tick_age, completion_age,
                self._stale_tick_discard_count, self._round_created_count,
                self._round_completed_count, self._round_replaced_count,
                self._dispatch_count, self._local_path_cache_hits,
                self._local_path_cache_misses,
                self._local_path_cache_invalidations,
                self._local_path_cache_evictions,
            ),
        )

    def _evidence_status_callback(self, message: Bool,
                                  evidence_robot_id: str) -> None:
        """Refresh or clear the bounded local evidence-opportunity lease."""
        now = time.monotonic()
        if bool(message.data):
            was_active = evidence_hold_active(
                self._evidence_hold_until_wall_s[evidence_robot_id], now)
            self._evidence_hold_until_wall_s[evidence_robot_id] = (
                now + self._evidence_hold_timeout_s)
            if not was_active:
                self.get_logger().info(
                    'EVIDENCE_ACQUISITION_HOLD robot=%s source=%s '
                    'active=true lease_s=%.2f' %
                    (self._robot_id, evidence_robot_id,
                     self._evidence_hold_timeout_s))
        else:
            was_active = evidence_hold_active(
                self._evidence_hold_until_wall_s[evidence_robot_id], now)
            self._evidence_hold_until_wall_s[evidence_robot_id] = 0.0
            if was_active:
                self.get_logger().info(
                    'EVIDENCE_ACQUISITION_HOLD robot=%s source=%s '
                    'active=false' % (self._robot_id, evidence_robot_id))
            # Lease state is refreshed independently for each publisher;
            # expiry is evaluated by the dispatch tick.

    def _allocator_timing_bucket(self) -> Optional[str]:
        """Return the requested simulation-time attribution window."""
        if not self._allocator_timing_enabled:
            return None
        sim_time_s = self.get_clock().now().nanoseconds / 1e9
        if 50.0 <= sim_time_s < 100.0:
            return 'early'
        if 280.0 <= sim_time_s <= 330.0:
            return 'late'
        return None

    def _allocator_timing_begin(self):
        """Begin one aggregate diagnostic section measurement."""
        bucket = self._allocator_timing_bucket()
        if bucket is None:
            return None
        return time.perf_counter(), bucket

    def _allocator_timing_record(self, section: str, token) -> None:
        """Accumulate one section measurement without per-call logging."""
        if token is None:
            return
        started, bucket = token
        elapsed = max(0.0, time.perf_counter() - started)
        entry = self._allocator_timing_stats[section][bucket]
        entry['calls'] += 1
        entry['total_wall_s'] += elapsed
        entry['max_wall_s'] = max(entry['max_wall_s'], elapsed)

    def _allocator_timing_add_inputs(
            self, bucket: Optional[str], candidate_count: int = 0,
            candidate_pairs_input: int = 0,
            traffic_checks: int = 0) -> None:
        """Record call-site input cardinalities for the same two windows."""
        if bucket is None:
            return
        entry = self._allocator_timing_inputs[bucket]
        if candidate_count:
            entry['candidate_count_total'] += int(candidate_count)
            entry['candidate_count_calls'] += 1
        if candidate_pairs_input:
            entry['candidate_pairs_input_total'] += int(candidate_pairs_input)
            entry['candidate_pairs_input_calls'] += 1
        if traffic_checks:
            entry['traffic_checks_total'] += int(traffic_checks)

    def _allocator_timing_timed_call(self, section: str, callback, *args,
                                     **kwargs):
        """Time an existing allocator call while preserving its return value."""
        token = self._allocator_timing_begin()
        try:
            return callback(*args, **kwargs)
        finally:
            self._allocator_timing_record(section, token)

    def _allocator_timing_maybe_log(self) -> None:
        """Emit one compact cumulative snapshot periodically when enabled."""
        if not self._allocator_timing_enabled:
            return
        now = time.monotonic()
        if now - self._allocator_timing_last_log_wall_s < 5.0:
            return
        self._allocator_timing_last_log_wall_s = now
        payload = {
            'robot': self._robot_id,
            'sim_time_s': self.get_clock().now().nanoseconds / 1e9,
            'stats': self._allocator_timing_stats,
            'inputs': self._allocator_timing_inputs,
        }
        self.get_logger().info(
            'ALLOCATOR_TIMING_SUMMARY %s' % json.dumps(
                payload, sort_keys=True, separators=(',', ':')),
        )

    def _tick(self) -> None:
        """Run one allocator tick and optionally attribute its wall time."""
        if not self._allocator_timing_enabled:
            return self._tick_impl()
        token = self._allocator_timing_begin()
        try:
            return self._tick_impl()
        finally:
            self._allocator_timing_record('tick_total', token)
            self._allocator_timing_maybe_log()

    def _tick_impl(self) -> None:
        now = time.monotonic()
        self._last_tick_steady_s = now
        tick_round = self._round
        tick_generation = self._round_lifecycle.generation
        tick_key = (
            None if tick_round is None else tick_round.round_id,
            tick_generation,
        )
        if tick_key != self._last_tick_log_key:
            self._last_tick_log_key = tick_key
            self._log_round_lifecycle(
                'ALLOCATOR_TICK_STARTED', tick_round, tick_generation,
                'timer callback',
            )
        self._expire_failures(now)
        if self._local_only:
            if self._handoff_complete:
                return
            if self._nav2.local_goal_active or self._dispatch_in_progress:
                return
            if not self._preflight_warmup_ready():
                return
            if any(evidence_hold_active(lease, now)
                   for lease in self._evidence_hold_until_wall_s.values()):
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'unknown-pose evidence acquisition opportunity active',
                )
                return
            local = self._fresh_snapshot(self._robot_id, now)
            if local is not None:
                if (not self._handoff_complete and
                        not dispatch_delay_elapsed(
                            self._dispatch_hold_started_steady_s, now,
                            self._prehandoff_dispatch_delay_s)):
                    return
                if self._initial_peer_readiness_barrier:
                    self._publish_initial_local_ready(now)
                    if not self._initial_exploration_barrier.dispatch_allowed:
                        self._transition(
                            CoordinatorState.WAITING_FOR_INPUTS,
                            'waiting for both robots initial local readiness',
                        )
                        return
                self._continue_degraded_solo(local)
            return
        if self._terminal or self._state == CoordinatorState.COMPLETE:
            return

        peer_status = self._peer_status
        if (peer_status is not None and peer_status.fresh(now) and
                getattr(peer_status.value, 'terminal', False) and
                not self._nav2.local_goal_active and not self._dispatch_in_progress):
            peer_terminal_reason = str(
                getattr(peer_status.value, 'terminal_reason', ''),
            )
            if peer_terminal_reason.startswith('MISSION_ABORT_'):
                self._set_terminal(peer_terminal_reason, success=False)
                return
            if (terminal_reason_is_success(peer_terminal_reason) and
                    self._candidate_evidence_seen == {'robot1', 'robot2'}):
                first = self._fresh_snapshot('robot1', now)
                second = self._fresh_snapshot('robot2', now)
                if (first is not None and second is not None and
                        completion_evidence_matches_snapshots(
                            getattr(self, '_candidate_lower_bound_metadata', {}),
                            (first, second),
                        )):
                    local_reason = classify_empty_frontiers(
                        self._candidate_evidence['robot1'],
                        self._candidate_evidence['robot2'],
                    )
                    if (local_reason is not None and
                            local_reason.value == peer_terminal_reason):
                        self._set_terminal(peer_terminal_reason, success=True)
                        return
        if (self._mission_timeout_enabled and self._mission_timeout_s > 0.0 and
                now - self._mission_started_steady_s >= self._mission_timeout_s):
            if self._nav2.local_goal_active:
                self._request_navigation_cancel()
            self._set_terminal(TerminalReason.TIMEOUT.value, success=False)
            return
        if self._traffic_hold is not None:
            self._continue_traffic_hold(now)
            return
        # An active peer goal is not a global assignment barrier.  The idle
        # robot may receive an independently agreed task while its peer keeps
        # its existing commitment.  Any actual path conflict is still handled
        # by the finalized traffic scheduler after the pair is selected.
        peer_status = self._peer_status
        peer_active = bool(
            peer_status is not None and peer_status.fresh(now) and
            (peer_status.value.local_nav_goal_active or
             peer_status.value.state == DistributedExplorationStatus.NAVIGATING)
        )
        if (self._traffic_reallocation_after_clear and
                self._released_traffic_winner_robot_id == self._peer_id and
                not peer_active):
            # The old winner has already become terminal.  The temporary
            # replicated reservation is no longer needed for a new round.
            self._traffic_reallocation_after_clear = False
            self._released_traffic_winner_robot_id = ''
        if now < self._settle_until_steady_s:
            return
        if self._consume_local_fallback_trigger():
            return
        continuation = self._allocator_timing_timed_call(
            'consensus_continuation', self._continuation_context, now,
        )
        if (continuation is None and self._round is not None and
                self._round.mode == 'continuation'):
            if self._continuation_round_requires_invalidation(now):
                self._reset_round('continuation commitment no longer valid')
            else:
                # Preserve the in-flight continuation round while the peer or
                # free-robot context is temporarily unavailable.  Returning
                # here is fail-closed: no decision/dispatch may use incomplete
                # safety context, but the round's bids are not discarded and
                # it can resume when fresh context returns.
                self._transition(
                    CoordinatorState.BIDDING,
                    'continuation context temporarily unavailable',
                )
            return
        continuation_active = continuation is not None
        if continuation_active:
            if not self._allocator_timing_timed_call(
                    'consensus_continuation',
                    self._activate_continuation_round, continuation):
                return
        if ((self._nav2.local_goal_active and
             not self._traffic_reallocation_after_clear) or
                self._dispatch_in_progress) and not continuation_active:
            return
        if continuation_active:
            if self._round is None:
                return
            first, second = self._round.snapshots
        else:
            first = self._fresh_snapshot('robot1', now)
            second = self._fresh_snapshot('robot2', now)
            if first is None or second is None:
                local = first if self._robot_id == 'robot1' else second
                if not self._continue_local_work_while_waiting(
                        local,
                        'temporary local work while peer snapshot is unavailable'):
                    if self._peer_liveness.evaluate(now) == CoordinatorState.DEGRADED_SOLO:
                        self._transition(CoordinatorState.DEGRADED_SOLO, 'peer snapshot timeout')
                    else:
                        self._transition(
                            CoordinatorState.WAITING_FOR_INPUTS,
                            'fresh task snapshots from both source sessions required',
                        )
                return
        # Terminal significance is evaluated from the unique physical
        # frontier evidence, not from whether tiny regions happened to become
        # allocator tasks.  This prevents a 0.06--0.18 m residual fragment
        # from being dispatched merely because it has a valid approach pose.
        if self._candidate_evidence_seen == {'robot1', 'robot2'}:
            terminal_candidate = classify_empty_frontiers(
                self._candidate_evidence['robot1'],
                self._candidate_evidence['robot2'],
            )
            if terminal_candidate is not None:
                if self._consider_completion(now):
                    return
                return
        if continuation_active:
            if self._round is None:
                return
            round_id = self._round.round_id
            content_fingerprint = self._round.content_fingerprint
        else:
            round_id = canonical_round_id(
                TaskIdentity('robot1', first.source_session_id, first.epoch),
                TaskIdentity('robot2', second.source_session_id, second.epoch),
            )
            content_fingerprint = self._allocation_semantic_fingerprint(
                first, second,
            )
        current_round = self._round
        if (
                current_round is not None and current_round.decision is not None and
                not current_round.decision.robot1_task_id and
                not current_round.decision.robot2_task_id and
                current_round.content_fingerprint == content_fingerprint):
            # Heartbeat epochs can advance while the semantic task set stays
            # empty.  Revisit the persistent completion gate on every tick;
            # otherwise the first empty round could never reach confirmation.
            local = first if self._robot_id == 'robot1' else second
            if self._continue_local_work_while_waiting(
                    local, 'temporary local work while pair is idle'):
                return
            if self._consider_completion(now):
                return
            return
        if (self._round is None and self._last_semantic_fingerprint ==
                content_fingerprint):
            local = first if self._robot_id == 'robot1' else second
            if not self._continue_local_work_while_waiting(
                    local, 'temporary local work while semantic round is unchanged'):
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'unchanged semantic task content; no new planner round',
                )
            return
        if (
                current_round is not None and current_round.decision is not None and
                current_round.round_id != round_id and
                not current_round.decision.robot1_task_id and
                not current_round.decision.robot2_task_id and
                current_round.content_fingerprint == content_fingerprint):
            local = first if self._robot_id == 'robot1' else second
            if not self._continue_local_work_while_waiting(
                    local, 'temporary local work while cooperative IDLE repeats'):
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'unchanged IDLE task content; waiting for meaningful proposal change',
                )
            return
        provenance_rebase = False
        if not continuation_active:
            provenance_rebase = (
                normal_round_provenance_rebase_required(
                    current_round, first, second, round_id,
                    content_fingerprint,
                ) and
                self._peer_bid_matches_latest_normal_round(
                    now, first, second, round_id,
                    current_round.union.union_hash,
                )
            )
        if (not continuation_active and (
                provenance_rebase or normal_round_requires_replacement(
                    current_round, first, second, content_fingerprint))):
            preserved_peer_batch = (
                self._bid_batches.get(self._peer_id)
                if provenance_rebase else None
            )
            union = build_canonical_union(
                first.tasks, second.tasks, self._maximum_union_tasks,
            )
            live_ids = {task.canonical_id for task in union.tasks}
            # Completion is semantic rather than timed: retaining only IDs
            # still proposed blocks residual redispatch; disappearance lets
            # a later materially different frontier become actionable.
            self._completed_shared_canonical_ids.intersection_update(live_ids)
            new_round = RoundWork(
                round_id=round_id,
                union=union,
                snapshots=(first, second),
                query_tasks=union.tasks[:self._maximum_path_queries],
                content_fingerprint=content_fingerprint,
            )
            reason = (
                'rebase pre-decision round to peer-confirmed snapshot provenance'
                if provenance_rebase else 'new canonical round'
            )
            generation = self._activate_round(new_round, reason)
            self._bid_batches.clear()
            if preserved_peer_batch is not None:
                # This peer batch is already valid for the newer canonical
                # context. Keep it while the local side recomputes its bid;
                # no old-round bid crosses the provenance boundary.
                self._bid_batches[self._peer_id] = preserved_peer_batch
            self._peer_decision = None
            self._committed = CommittedRound()
            self._transition(CoordinatorState.BIDDING, reason)
            self._log_union(new_round)
        # This node uses a MultiThreadedExecutor.  A navigation-terminal
        # callback may reset self._round while this timer callback is still
        # unwinding.  Keep the round object local for this pass and abandon
        # the stale pass if another callback replaced it.
        round_work = self._round
        generation = self._round_lifecycle.generation
        if round_work is None:
            return
        if round_work.union.tasks:
            self._completion_candidate_since_steady_s = None
        if (round_work.local_batch is None and
                not (continuation_active and
                     self._robot_id == continuation.busy_robot_id)):
            self._continue_bidding(round_work, generation)
            return
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before bid validation')
            return
        if continuation_active:
            continuation_batches = self._allocator_timing_timed_call(
                'consensus_continuation', self._continuation_bid_batches,
                now, round_work,
            )
            if continuation_batches is None:
                local = next(
                    (snapshot for snapshot in round_work.snapshots
                     if snapshot.source_robot_id == self._robot_id), None)
                if not self._continue_local_work_while_waiting(
                        local,
                        'temporary local work while continuation bid is pending'):
                    self._transition(
                        CoordinatorState.BIDDING,
                        'waiting for valid free-robot continuation bid',
                    )
                return
            first_batch, second_batch = continuation_batches
        else:
            if not self._both_bid_batches_valid(now, round_work):
                local = next(
                    (snapshot for snapshot in round_work.snapshots
                     if snapshot.source_robot_id == self._robot_id), None)
                if not self._continue_local_work_while_waiting(
                        local, 'temporary local work while peer bid is pending'):
                    self._transition(CoordinatorState.BIDDING, 'waiting for valid peer bids')
                return
        # Do not publish the first actionable pair while the local shared
        # frame is still extrapolating.  The final dispatch gate remains in
        # place as a safety recheck, but making TF readiness a prerequisite
        # here avoids creating a decision that is immediately invalidated by
        # the same condition.  This is a local replicated readiness fact, not
        # a coordinator or a pose-estimation fallback.
        if round_work.union.tasks:
            tf_ready, tf_age_s, tf_reason = self._nav2.shared_tf_status()
            if not tf_ready:
                wait_now = time.monotonic()
                if wait_now - self._last_shared_tf_wait_log_wall_s >= 2.0:
                    self._last_shared_tf_wait_log_wall_s = wait_now
                    self.get_logger().info(
                        'SHARED_TF_NOT_READY robot=%s round=%s age_s=%s reason=%s' %
                        (self._robot_id, round_work.round_id, tf_age_s, tf_reason))
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'required shared-frame TF not yet usable for pair decision',
                )
                local = next(
                    (snapshot for snapshot in round_work.snapshots
                     if snapshot.source_robot_id == self._robot_id), None)
                self._continue_local_work_while_waiting(
                    local, 'temporary local work while shared TF is pending')
                return
            if not self._shared_tf_ready_logged:
                self._shared_tf_ready_logged = True
                self._startup_event(
                    'SHARED_TF_READY', round_id=round_work.round_id,
                    transform_age_s=tf_age_s,
                )
            if not self._first_valid_task_snapshots_logged:
                self._first_valid_task_snapshots_logged = True
                self._startup_event(
                    'FIRST_VALID_TASK_SNAPSHOTS', round_id=round_work.round_id,
                    robot1_epoch=round_work.snapshots[0].epoch,
                    robot2_epoch=round_work.snapshots[1].epoch,
                    robot1_task_count=len(round_work.snapshots[0].tasks),
                    robot2_task_count=len(round_work.snapshots[1].tasks),
                )
        if not continuation_active:
            first_batch = self._bid_batches['robot1'].value
            second_batch = self._bid_batches['robot2'].value
        hard_failed_task_ids = self._hard_failed_task_ids(round_work.union)
        completed_task_ids = frozenset(
            self._completed_shared_canonical_ids)
        peer_reservation_task_ids = frozenset(
            self._temporary_peer_reservation_ids(now))
        hard_ids = (
            hard_failed_task_ids |
            completed_task_ids |
            peer_reservation_task_ids
        )
        selector_fingerprint, selector_inputs = (
            self._selector_feasibility_identity(
                round_work.union, hard_failed_task_ids,
                completed_task_ids, peer_reservation_task_ids,
            )
        )

        # Selector suppression is independent of task/bid semantics.  Check
        # it before the decision branch as well as while creating a decision:
        # completion or reservation evidence may arrive after a pre-dispatch
        # decision was published.  In that case the old decision must not be
        # left in WAITING_FOR_MATCHING_DECISION indefinitely.  The helper is
        # still restricted to an uncommitted round, so committed/navigation-
        # active semantics remain strict and unchanged.
        self._selector_decision_requires_recompute(
            round_work, selector_fingerprint, selector_inputs,
        )

        if round_work.decision is None:
            fixed_kwargs = {}
            if continuation_active:
                if continuation.busy_robot_id == 'robot1':
                    fixed_kwargs['fixed_robot1_task_id'] = (
                        continuation.commitment.canonical_id)
                else:
                    fixed_kwargs['fixed_robot2_task_id'] = (
                        continuation.commitment.canonical_id)
            traffic_selection_checks = []

            selection_bucket = self._allocator_timing_bucket()
            self._allocator_timing_add_inputs(
                selection_bucket,
                candidate_count=len(first_batch.bids) + len(second_batch.bids),
                candidate_pairs_input=len(first_batch.bids) * len(second_batch.bids),
            )

            def traffic_compatible(first_id, first_bid, second_id, second_bid):
                """Use the exact dispatch scheduler as a selection gate."""
                if not first_id or not second_id:
                    return True
                traffic_token = self._allocator_timing_begin()
                try:
                    traffic_result = self._traffic_for_bid_pair(
                        first_bid, second_bid, round_work,
                    )
                finally:
                    self._allocator_timing_record(
                        'traffic_checks', traffic_token,
                    )
                self._allocator_timing_add_inputs(
                    selection_bucket, traffic_checks=1,
                )
                traffic_selection_checks.append(
                    (first_id, second_id, traffic_result),
                )
                return not traffic_result.conflict

            selection_kwargs = {
                'traffic_compatibility': traffic_compatible,
            }
            if self._assignment_strategy == 'frontier_mrtsp':
                decision = self._allocator_timing_timed_call(
                    'pair_selection', choose_mrtsp_route_assignment,
                    round_work.round_id, round_work.union,
                    first_batch, second_batch, hard_ids, self._weights,
                    **fixed_kwargs, **selection_kwargs,
                )
            else:
                decision = self._allocator_timing_timed_call(
                    'pair_selection', choose_pair_assignment,
                    round_work.round_id, round_work.union,
                    first_batch, second_batch, hard_ids, self._weights,
                    scoring_mode='frontier_cost_only',
                    **fixed_kwargs, **selection_kwargs,
                )
            self._log_traffic_aware_selection(
                decision, round_work, traffic_selection_checks,
            )
            decision = self._select_traffic_test_conflict_pair(
                decision, round_work.union, first_batch, second_batch,
            )
            decision = replace(
                decision,
                decision_hash=hashlib.sha256(json.dumps({
                    'base_decision_hash': decision.decision_hash,
                    'selector_feasibility_fingerprint': selector_fingerprint,
                }, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest(),
                diagnostics=replace(
                    decision.diagnostics,
                    selector_feasibility_fingerprint=selector_fingerprint,
                    selector_completed_task_ids=selector_inputs['completed'],
                    selector_hard_failed_task_ids=selector_inputs['hard_failed'],
                    selector_peer_reservation_task_ids=(
                        selector_inputs['peer_reservations']),
                ),
            )
            if self._assignment_strategy == 'frontier_cost_only':
                certified, blocking, optimistic_score, certificate_reason = (
                    self._cost_only_dispatch_certificate(
                        round_work, decision, first_batch, second_batch,
                    )
                )
                if not certified:
                    self._transition(
                        CoordinatorState.WAITING_FOR_INPUTS,
                        'cost-only dispatch certificate deferred: %s' %
                        certificate_reason,
                    )
                    local = next(
                        (snapshot for snapshot in round_work.snapshots
                         if snapshot.source_robot_id == self._robot_id), None)
                    self._continue_local_work_while_waiting(
                        local,
                        'temporary local work while cost-only certificate is pending')
                    return
            traffic = self._traffic_for_decision(decision, first_batch, second_batch)
            # Snapshot cardinalities are transport/provenance facts rather
            # than solver inputs.  Attach them here so the decision telemetry
            # can distinguish an empty source snapshot from a source whose
            # tasks were all rejected by local path validation.
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(round_work, generation, 'before decision commit')
                return
            round_work.decision = replace(
                decision,
                diagnostics=replace(
                    decision.diagnostics,
                    robot1_snapshot_task_count=len(
                        round_work.snapshots[0].tasks,
                    ),
                    robot2_snapshot_task_count=len(round_work.snapshots[1].tasks),
                    traffic=traffic.as_dict(),
                ),
            )
            round_work.traffic = traffic
            self._allocator_timing_timed_call(
                'consensus_continuation', self._publish_decision,
                round_work, generation,
            )
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(round_work, generation, 'after decision publication')
                return
            self._log_round_lifecycle(
                'ALLOCATOR_TICK_COMMITTED', round_work, generation,
                'decision published',
            )
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'local complete pair decision published',
            )
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before peer match')
            return
        if not self._allocator_timing_timed_call(
                'consensus_continuation', self._matching_peer_decision,
                round_work, generation, now):
            local = next(
                (snapshot for snapshot in round_work.snapshots
                 if snapshot.source_robot_id == self._robot_id), None)
            self._continue_local_work_while_waiting(
                local, 'temporary local work while peer agreement is pending')
            return
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'after peer match')
            return
        if self._committed.decision is None:
            self._committed.commit(round_work.decision)
            self._allocator_timing_timed_call(
                'consensus_continuation', self._remember_active_commitments,
                round_work,
            )
            self._last_semantic_fingerprint = round_work.content_fingerprint
            self._emit_event('DECISION_AGREED', 'positive replicated decision match')
            if round_work.mode == 'continuation':
                self._emit_event(
                    'CONTINUATION_DECISION_AGREED',
                    json.dumps({
                        'mode': 'continuation',
                        'free_robot': round_work.continuation_free_robot_id,
                        'busy_robot': round_work.continuation_busy_robot_id,
                        'commitment_id': round_work.continuation_commitment_id,
                        'free_task': (
                            round_work.decision.robot1_task_id
                            if round_work.continuation_free_robot_id == 'robot1'
                            else round_work.decision.robot2_task_id),
                    }, sort_keys=True, separators=(',', ':')),
                )
            if not self._first_valid_pair_decision_logged:
                self._first_valid_pair_decision_logged = True
                self._startup_event(
                    'FIRST_VALID_PAIR_DECISION', round_id=round_work.round_id,
                    decision_hash=round_work.decision.decision_hash,
                    robot1_task=round_work.decision.robot1_task_id or 'IDLE',
                    robot2_task=round_work.decision.robot2_task_id or 'IDLE',
                )
        if (not round_work.decision.robot1_task_id and
                not round_work.decision.robot2_task_id and
                self._consider_completion(now)):
            return
        if not self._dispatch_enabled:
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'decision agreed; dispatch disabled',
            )
            return
        if self._synchronized_traffic_test:
            current_key = (round_work.round_id, round_work.decision.decision_hash)
            sim_time = self.get_clock().now().nanoseconds / 1e9
            release_ready = (
                self._traffic_test_release_key == current_key and
                self._traffic_test_release_at_sim_s is not None and
                sim_time >= self._traffic_test_release_at_sim_s
            )
            if not release_ready:
                # Do not advertise a round until this replica's shared Nav2
                # action/lifecycle/map/TF inputs are genuinely usable.  This
                # keeps the test barrier from releasing a pair into the
                # startup readiness race that it is intended to measure.
                if not self._nav2.synchronized_test_inputs_ready():
                    self._transition(
                        CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                        'synchronized traffic-test waiting for local Nav2/map readiness',
                    )
                    return
            self._publish_traffic_test_ready(round_work)
            if not release_ready:
                self._transition(
                    CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                    'synchronized traffic-test dispatch barrier pending',
                )
                return
            if self._traffic_test_release_logged_key != current_key:
                self._traffic_test_release_logged_key = current_key
                self.get_logger().info(
                    'TRAFFIC_TEST_DISPATCH_ELIGIBILITY_RELEASED robot=%s '
                    'round=%s sim_time_s=%.6f' %
                    (self._robot_id, current_key[0], sim_time),
                )
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before local dispatch')
            return
        self._start_local_dispatch(round_work, generation)

    def _preflight_warmup_ready(self) -> bool:
        """Require consecutive local preflight input samples before selection.

        This is an integration readiness gate only.  It does not alter any
        candidate, path, footprint, unknown-cell, or lifecycle rejection
        rule.  A missing costmap/footprint resets the consecutive count and
        remains retryable on a later allocator tick.
        """
        required = int(getattr(self, '_preflight_warmup_cycles_required', 0))
        if required <= 0:
            return True
        nav2 = getattr(self, '_nav2', None)
        available_fn = getattr(nav2, 'preflight_inputs_available', None)
        available = bool(available_fn()) if callable(available_fn) else False
        if available:
            observed = int(getattr(
                self, '_preflight_warmup_cycles_observed', 0)) + 1
            self._preflight_warmup_cycles_observed = min(observed, required)
        else:
            self._preflight_warmup_cycles_observed = 0
        if self._preflight_warmup_cycles_observed < required:
            self._transition(
                CoordinatorState.WAITING_FOR_INPUTS,
                'preflight warmup %d/%d: local costmap and effective footprint '
                'required' % (
                    self._preflight_warmup_cycles_observed, required,
                ),
            )
            return False
        return True

    def _cost_only_certificate_blocker_diagnostics(
            self, round_work: RoundWork, decision: PairDecision,
            first_batch: BidBatch, second_batch: BidBatch,
            required_robots: tuple[str, ...],
            bounds: dict[str, Optional[tuple[float, ...]]],
            bound_diagnostics: dict[str, dict], certified: bool,
    ) -> tuple[list[dict], list[dict]]:
        """Describe certificate blockers without changing certificate behavior."""
        # Focused certificate tests may bypass __init__ because this method is
        # diagnostic-only. Real ROS nodes initialize these containers there.
        if not hasattr(self, '_candidate_region_snapshots'):
            self._candidate_region_snapshots = {}
        if not hasattr(self, '_certificate_blocker_history'):
            self._certificate_blocker_history = {}
        if not hasattr(self, '_certificate_blocker_last_round'):
            self._certificate_blocker_last_round = {}
        batches = {
            'robot1': first_batch,
            'robot2': second_batch,
        }

        def bid_costs(batch: BidBatch) -> list[float]:
            values = [0.0]
            for bid in batch.bids:
                if not bid.path_valid:
                    continue
                try:
                    value = nominal_motion_cost_s(
                        float(bid.path_length_m), float(bid.heading_cost),
                        self._weights.cost_only_reference_linear_speed_mps,
                        self._weights.cost_only_reference_angular_speed_radps,
                    )
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values.append(value)
            return values

        evaluated_costs = {
            robot: bid_costs(batch) for robot, batch in batches.items()
        }
        sim_time = self._sim_time_s()
        current_by_robot: dict[str, list[dict]] = {}

        for robot_id in required_robots:
            regions = tuple(
                region for region in self._candidate_region_snapshots.get(
                    robot_id, ())
                if str(region.status) == 'DETECTED_NOT_QUERIED'
            )
            current_ids = {str(region.physical_id) for region in regions}
            round_id = round_work.round_id
            is_new_round = self._certificate_blocker_last_round.get(robot_id) != round_id
            if is_new_round:
                for (history_robot, history_id), history in (
                        self._certificate_blocker_history.items()):
                    if history_robot != robot_id or not history.get('last_present'):
                        continue
                    if history_id not in current_ids:
                        history['last_present'] = False
                        history['disappeared_count'] += 1

            raw_bounds = bounds.get(robot_id)
            diagnostic = bound_diagnostics.get(robot_id, {})
            evidence_reason = str(diagnostic.get('reason', 'OTHER'))
            other_robot = 'robot2' if robot_id == 'robot1' else 'robot1'
            other_options = list(evaluated_costs[other_robot])
            other_bounds = bounds.get(other_robot)
            if other_bounds is not None:
                other_options.extend(float(value) for value in other_bounds)

            output = []
            for index, region in enumerate(regions):
                physical_id = str(region.physical_id)
                bound_value = None
                if raw_bounds is not None and index < len(raw_bounds):
                    bound_value = float(raw_bounds[index])
                evidence_available = bool(
                    evidence_reason == 'OK' and
                    bound_value is not None and
                    math.isfinite(bound_value) and bound_value >= 0.0)
                if not evidence_available:
                    if evidence_reason in {
                            'TASK_CANDIDATE_GENERATION_MISMATCH',
                            'MAP_REVISION_MISMATCH',
                            'COSTMAP_REVISION_MISMATCH',
                            'FINGERPRINT_MISMATCH',
                            'SESSION_EPOCH_MISMATCH'}:
                        evidence_state = 'STALE_EVIDENCE'
                    else:
                        evidence_state = 'UNAVAILABLE_EVIDENCE'
                elif region.query_count == 0:
                    evidence_state = 'PENDING_FIRST_EVALUATION'
                elif region.cycles_not_queried > 0:
                    evidence_state = 'STALE_EVIDENCE_REQUIRES_REQUERY'
                elif region.last_query_result:
                    evidence_state = 'FINISHED_QUERY_NOT_REFLECTED'
                else:
                    evidence_state = 'UNAVAILABLE_EVIDENCE'

                best_competing_score = float('-inf')
                if evidence_available:
                    for partner_cost in other_options:
                        if bound_value == 0.0 and partner_cost == 0.0:
                            continue
                        best_competing_score = max(
                            best_competing_score,
                            -(bound_value + partner_cost),
                        )
                is_blocking = bool(
                    evidence_available and
                    best_competing_score >= decision.score.total - 1e-9)
                key = (robot_id, physical_id)
                history = self._certificate_blocker_history.setdefault(key, {
                    'robot_id': robot_id,
                    'physical_id': physical_id,
                    'first_candidate_generation_id': int(
                        region.candidate_generation_id),
                    'candidate_generations_seen': [],
                    'first_seen_sim_time_s': sim_time,
                    'last_seen_sim_time_s': sim_time,
                    'certificate_rounds_waiting': 0,
                    'observed_count': 0,
                    'ever_queried': False,
                    'max_query_count': 0,
                    'last_query_result': '',
                    'disappeared_count': 0,
                    'reappeared_count': 0,
                    'prevented_certification_rounds': 0,
                    'last_present': False,
                    'last_round_id': '',
                })
                generation_id = int(region.candidate_generation_id)
                if generation_id and generation_id not in history[
                        'candidate_generations_seen']:
                    history['candidate_generations_seen'].append(generation_id)
                if (not history['last_present'] and history['observed_count'] and
                        is_new_round):
                    history['reappeared_count'] += 1
                history['last_present'] = True
                history['last_seen_sim_time_s'] = sim_time
                history['observed_count'] += 1
                history['ever_queried'] = bool(
                    history['ever_queried'] or region.query_count > 0)
                history['max_query_count'] = max(
                    history['max_query_count'], int(region.query_count))
                if region.last_query_result:
                    history['last_query_result'] = region.last_query_result
                if is_new_round:
                    history['certificate_rounds_waiting'] += 1
                    history['prevented_certification_rounds'] += int(
                        is_blocking and not certified)
                    history['last_round_id'] = round_id
                item = {
                    'robot_id': robot_id,
                    'physical_id': physical_id,
                    'candidate_generation_id': generation_id,
                    'map_revision': int(region.map_revision),
                    'costmap_revision': int(region.costmap_revision),
                    'certificate_evidence_reason': evidence_reason,
                    'evidence_state': evidence_state,
                    'bound_s': bound_value,
                    'bound_entry_count': (0 if raw_bounds is None
                                          else len(raw_bounds)),
                    'detected_not_queried_count': int(
                        getattr(self, '_candidate_source_local_evidence',
                                self._candidate_evidence).get(
                                    robot_id, CandidateEvidence()).detected_not_queried),
                    'query_count': int(region.query_count),
                    'cycles_seen': int(region.cycles_seen),
                    'cycles_not_queried': int(region.cycles_not_queried),
                    'last_query_ns': int(region.last_query_ns),
                    'last_query_result': region.last_query_result,
                    'ever_queried': bool(history['ever_queried']),
                    'best_competing_score': best_competing_score,
                    'is_blocking': is_blocking,
                    'certificate_rounds_waiting': history[
                        'certificate_rounds_waiting'],
                    'prevented_certification_rounds': history[
                        'prevented_certification_rounds'],
                    'disappeared_count': history['disappeared_count'],
                    'reappeared_count': history['reappeared_count'],
                }
                output.append(item)
            current_by_robot[robot_id] = output
            if is_new_round:
                self._certificate_blocker_last_round[robot_id] = round_id

        history_output = []
        for history in sorted(
                self._certificate_blocker_history.values(),
                key=lambda item: (item['robot_id'], item['physical_id'])):
            item = dict(history)
            item['candidate_generations_seen'] = list(
                item['candidate_generations_seen'])
            history_output.append(item)
        current_output = [
            item for robot_id in sorted(current_by_robot)
            for item in sorted(current_by_robot[robot_id],
                               key=lambda value: value['physical_id'])
        ]
        return current_output, history_output

    def _cost_only_dispatch_certificate(
            self, round_work: RoundWork, decision: PairDecision,
            first_batch: BidBatch, second_batch: BidBatch) -> tuple[
                bool, int, float, str]:
        """Prevent dispatch before an unqueried cost-only option is dominated."""
        if self._assignment_strategy != 'frontier_cost_only':
            return True, 0, float('-inf'), 'POLICY_NOT_COST_ONLY'

        bounds: dict[str, Optional[tuple[float, ...]]] = {}
        bound_diagnostics: dict[str, dict] = {}
        if round_work.mode == 'continuation':
            # The busy side is represented by its immutable commitment.  Its
            # stale proposal is deliberately irrelevant to continuation.
            bounds[round_work.continuation_busy_robot_id] = ()
        required_robots = (
            ('robot1', 'robot2') if round_work.mode != 'continuation' else
            (round_work.continuation_free_robot_id,)
        )
        snapshots_by_robot = {
            snapshot.source_robot_id: snapshot
            for snapshot in round_work.snapshots
        }
        for robot_id in required_robots:
            # Certificate completeness is source-local.  The merged evidence
            # view intentionally used for terminal/status reporting must not
            # inflate (or erase) this participant's DNU count.
            evidence = getattr(
                self, '_candidate_source_local_evidence',
                self._candidate_evidence,
            ).get(robot_id)
            snapshot = snapshots_by_robot.get(robot_id)
            candidate_record = self._candidate_bound_record_for_snapshot(
                robot_id, snapshot,
            )
            if candidate_record is not None:
                raw_bounds = candidate_record['bounds']
                provenance = candidate_record['provenance']
                candidate_metadata = candidate_record['metadata']
                matching_context = True
                expected_count = int(candidate_record['expected_count'])
                context_source = candidate_record['source']
            else:
                raw_bounds = getattr(
                    self, '_unqueried_cost_bounds', {},
                ).get(robot_id)
                provenance = getattr(
                    self, '_unqueried_cost_bound_provenance', {},
                ).get(robot_id)
                candidate_metadata = getattr(
                    self, '_candidate_lower_bound_metadata', {},
                ).get(robot_id)
                matching_context = bool(
                    snapshot is not None and
                    lower_bound_context_matches(provenance, snapshot)
                )
                expected_count = int(getattr(
                    evidence, 'detected_not_queried', 0) or 0)
                context_source = 'latest_unmatched'
            bound_diagnostics[robot_id] = {
                'required_for_certificate': robot_id in required_robots,
            }
            _evidence_reason, comparison = classify_lower_bound_evidence(
                candidate_metadata, snapshot,
                raw_bounds,
                expected_count,
            )
            bound_diagnostics[robot_id].update(comparison)
            bound_diagnostics[robot_id]['context_source'] = context_source
            if (expected_count == 0 and
                    raw_bounds is None):
                bounds[robot_id] = ()
            elif raw_bounds is not None and matching_context:
                bounds[robot_id] = raw_bounds
            else:
                # Compact bounds are revision/context evidence, not an
                # 8-second heartbeat. Any provenance mismatch blocks.
                bounds[robot_id] = None
        for robot_id in ('robot1', 'robot2'):
            bounds.setdefault(robot_id, ())

        evidence_reason = next((bound_diagnostics[robot]['reason'] for robot in
                                required_robots
                                if bound_diagnostics.get(robot, {}).get(
                                    'reason') != 'OK'), 'OK')
        provenance_key = tuple(
            (robot, bound_diagnostics.get(robot, {}).get('reason'),
             bound_diagnostics.get(robot, {}).get('candidate', {}).get(
                 'lower_bound_context_fingerprint'),
             bound_diagnostics.get(robot, {}).get('task_snapshot', {}).get(
                 'lower_bound_context_fingerprint'),
             bound_diagnostics.get(robot, {}).get('candidate', {}).get(
                 'generation_ros_ns'),
             bound_diagnostics.get(robot, {}).get('task_snapshot', {}).get(
                 'generation_ros_ns'),
             bound_diagnostics.get(robot, {}).get('candidate', {}).get(
                 'candidate_generation_id'),
             bound_diagnostics.get(robot, {}).get('task_snapshot', {}).get(
                 'candidate_generation_id'))
            for robot in required_robots)

        certified, blocking, optimistic_score, reason = (
            cost_only_dispatch_certificate(
                decision, first_batch, second_batch,
                bounds.get('robot1'), bounds.get('robot2'), self._weights,
            )
        )
        blocker_diagnostics, blocker_history = (
            self._cost_only_certificate_blocker_diagnostics(
                round_work, decision, first_batch, second_batch,
                required_robots, bounds, bound_diagnostics, certified,
            )
        )
        dnu = 0
        for robot_id in (
                ('robot1', 'robot2') if round_work.mode != 'continuation'
                else (round_work.continuation_free_robot_id,)):
            source_bounds = bounds.get(robot_id)
            if source_bounds is not None:
                dnu += len(source_bounds)
            else:
                comparison = bound_diagnostics.get(robot_id, {})
                candidate = comparison.get('candidate', {})
                dnu += int(candidate.get(
                    'expected_bound_entry_count', 0) or 0)
        key = (round_work.round_id, len(first_batch.bids), len(second_batch.bids),
               dnu, blocking, round(float(decision.score.total), 9),
               round(float(optimistic_score), 9), certified, reason,
               evidence_reason, provenance_key)
        if key != self._last_cost_only_certificate_key:
            self._last_cost_only_certificate_key = key
            self._emit_event(
                'COST_ONLY_DISPATCH_CERTIFICATE',
                json.dumps({
                    'evaluated_candidate_count': len(first_batch.bids) +
                    len(second_batch.bids),
                    'detected_not_queried_count': dnu,
                    'blocking_unqueried_candidates': blocking,
                    'current_evaluated_assignment_score': decision.score.total,
                    'best_optimistic_unqueried_score': optimistic_score,
                    'dispatch_certified': certified,
                    'reason': reason,
                    # ``evidence_reason`` is a diagnostic taxonomy for the
                    # exact conservative branch.  The legacy ``reason`` above
                    # remains unchanged for downstream consumers.
                    'evidence_reason': evidence_reason,
                    'evidence_branch': evidence_reason,
                    'provenance_comparison': bound_diagnostics,
                    'blocker_diagnostics': blocker_diagnostics,
                    'blocker_history': blocker_history,
                }, sort_keys=True, separators=(',', ':')),
            )
        return certified, blocking, optimistic_score, reason

    def _traffic_for_bid_pair(
            self, robot1_bid: Optional[Bid], robot2_bid: Optional[Bid],
            round_work: Optional[RoundWork] = None) -> TrafficDecision:
        """Run the one authoritative traffic model for candidate pair paths."""
        required = self._traffic_robot1_safe_radius_m + self._traffic_robot2_safe_radius_m
        if not self._traffic_scheduler_enabled:
            return TrafficDecision(required_separation_m=required, reason='DISABLED')
        if robot1_bid is None or robot2_bid is None:
            return TrafficDecision(required_separation_m=required, reason='SINGLE_ACTIVE_OR_IDLE')
        active_round = self._round if round_work is None else round_work
        active_robots = frozenset()
        if (active_round is not None and active_round.mode == 'continuation' and
                active_round.continuation_busy_robot_id):
            # The busy side is an already committed active reservation in a
            # continuation round.  Preserve that commitment's priority in
            # the existing scheduler; the scheduler itself is unchanged.
            active_robots = frozenset({active_round.continuation_busy_robot_id})
        elif (self._traffic_reallocation_after_clear and
              self._released_traffic_winner_robot_id):
            active_robots = frozenset({self._released_traffic_winner_robot_id})
        return schedule_traffic(
            robot1_bid.path, robot2_bid.path,
            robot1_safe_radius_m=self._traffic_robot1_safe_radius_m,
            robot2_safe_radius_m=self._traffic_robot2_safe_radius_m,
            reference_speed_mps=self._traffic_reference_speed_mps,
            eta_tie_s=self._traffic_eta_tie_s,
            # A matched round contains only new undispatched goals.  Normally
            # status is excluded so both replicas derive identical evidence.
            # After a replicated conflict-clear event, retain the old winner
            # as the deterministic active reservation for the one fresh round
            # that follows; this prevents a later conflict from stealing its
            # right of way while it is still in transit.
            active_robots=active_robots,
        )

    def _traffic_for_decision(
            self, decision: PairDecision, robot1_bids: BidBatch,
            robot2_bids: BidBatch) -> TrafficDecision:
        """Derive the same bounded traffic result from agreed bid geometry."""
        required = self._traffic_robot1_safe_radius_m + self._traffic_robot2_safe_radius_m
        if not decision.robot1_task_id or not decision.robot2_task_id:
            return TrafficDecision(
                required_separation_m=required,
                reason='SINGLE_ACTIVE_OR_IDLE',
            )
        first = {bid.canonical_task_id: bid for bid in robot1_bids.bids}.get(
            decision.robot1_task_id,
        )
        second = {bid.canonical_task_id: bid for bid in robot2_bids.bids}.get(
            decision.robot2_task_id,
        )
        if first is None or second is None:
            return TrafficDecision(
                required_separation_m=required,
                reason='SELECTED_BID_MISSING',
            )
        if not self._traffic_scheduler_enabled:
            return TrafficDecision(required_separation_m=required, reason='DISABLED')
        return self._traffic_for_bid_pair(first, second, self._round)

    def _log_traffic_aware_selection(
            self, decision: PairDecision, round_work: RoundWork,
            checks: list[tuple[str, str, TrafficDecision]]) -> None:
        """Log only policy choices skipped by the exact traffic scheduler."""
        conflicts = [item for item in checks if item[2].conflict]
        if not conflicts:
            return
        first_id, second_id, _ = checks[0]
        selected = (decision.robot1_task_id, decision.robot2_task_id)
        for index, (candidate1, candidate2, traffic) in enumerate(conflicts):
            if round_work.mode == 'continuation':
                busy_robot = round_work.continuation_busy_robot_id
                free_robot = round_work.continuation_free_robot_id
                task_id = candidate1 if free_robot == 'robot1' else candidate2
                candidate_rank = index
                event_name = 'TRAFFIC_AWARE_SELECTION_SKIP'
                reason = 'CONFLICT_WITH_ACTIVE_COMMITMENT'
            else:
                busy_robot = traffic.winner_robot_id or 'pair'
                free_robot = ''
                task_id = '%s|%s' % (candidate1, candidate2)
                candidate_rank = index
                event_name = 'TRAFFIC_AWARE_PAIR_SKIP'
                reason = 'CONFLICTING_PAIR'
            commitment = self._active_commitments.get(busy_robot)
            self.get_logger().info(
                '%s robot=%s candidate_rank=%d task=%s reason=%s '
                'busy_robot=%s busy_commitment=%s round=%s' % (
                    event_name, free_robot or 'pair', candidate_rank, task_id,
                    reason, busy_robot,
                    commitment.commitment_id if commitment else '',
                    round_work.round_id,
                ),
            )
        event_name = ('TRAFFIC_AWARE_SELECTION_FALLBACK' if
                      round_work.mode == 'continuation' else
                      'TRAFFIC_AWARE_PAIR_FALLBACK')
        if selected != (first_id, second_id):
            self.get_logger().info(
                '%s original_rank=0 selected_rank=%d round=%s' % (
                    event_name,
                    next(
                        (index for index, item in enumerate(checks)
                         if (item[0], item[1]) == selected),
                        -1,
                    ), round_work.round_id,
                ),
            )
        else:
            self.get_logger().info(
                '%s original_rank=0 selected_rank=0 '
                'reason=ALL_POLICY_OPTIONS_CONFLICT round=%s' % (
                    event_name, round_work.round_id,
                ),
            )

    def _select_traffic_test_conflict_pair(
            self, decision: PairDecision, union: CanonicalUnion,
            robot1_bids: BidBatch, robot2_bids: BidBatch) -> PairDecision:
        """Select a deterministic conflicting pair from real Nav2 bid paths.

        This is strictly a synchronized-test fixture operation.  It does not
        invent coordinates, use physical truth, or alter production Burgard
        allocation.  Every candidate is an actually valid bid for the owning
        robot, and conflict is evaluated by the same continuous scheduler used
        at dispatch time.  Sorting by strongest measured conflict and then
        canonical IDs makes both replicas choose the same pair.
        """
        if not (self._synchronized_traffic_test and
                self._traffic_test_force_conflict_pair):
            return decision
        tasks = {task.canonical_id for task in union.tasks}
        first = {
            bid.canonical_task_id: bid for bid in robot1_bids.bids
            if bid.canonical_task_id in tasks and bid.path_valid and len(bid.path) >= 1
        }
        second = {
            bid.canonical_task_id: bid for bid in robot2_bids.bids
            if bid.canonical_task_id in tasks and bid.path_valid and len(bid.path) >= 1
        }
        conflicts = []
        for first_id in sorted(first):
            for second_id in sorted(second):
                if first_id == second_id:
                    continue
                traffic = schedule_traffic(
                    first[first_id].path, second[second_id].path,
                    robot1_safe_radius_m=self._traffic_robot1_safe_radius_m,
                    robot2_safe_radius_m=self._traffic_robot2_safe_radius_m,
                    reference_speed_mps=self._traffic_reference_speed_mps,
                    eta_tie_s=self._traffic_eta_tie_s,
                    active_robots=frozenset(),
                )
                # Reject degenerate overlaps at a route origin and strongly
                # unbalanced cases where one robot is already in the shared
                # region.  The stress fixture must exercise two approaches
                # to the same region, not a post-hoc active-robot case.
                balanced = (
                    traffic.robot1_first_conflict_distance_m >= 0.50 and
                    traffic.robot2_first_conflict_distance_m >= 0.50 and
                    abs(
                        traffic.robot1_first_conflict_distance_m -
                        traffic.robot2_first_conflict_distance_m
                    ) <= 2.00
                )
                if traffic.conflict and balanced:
                    conflicts.append((
                        round(traffic.minimum_separation_m, 12),
                        first_id, second_id, traffic,
                    ))
        if not conflicts:
            self.get_logger().warning(
                'TRAFFIC_TEST_NO_REAL_CONFLICTING_BID_PAIR round=%s tasks=%d'
                % (decision.round_id, len(tasks)),
            )
            return decision
        _, first_id, second_id, traffic = min(conflicts)
        if (decision.robot1_task_id, decision.robot2_task_id) != (first_id, second_id):
            payload = {
                'round_id': decision.round_id,
                'union_hash': decision.union_hash,
                'robot1_bid_fingerprint': decision.robot1_bid_fingerprint,
                'robot2_bid_fingerprint': decision.robot2_bid_fingerprint,
                'robot1_task': first_id,
                'robot2_task': second_id,
                'fixture': 'real_bid_path_conflict_pair',
            }
            decision_hash = hashlib.sha256(json.dumps(
                payload, sort_keys=True, separators=(',', ':'),
            ).encode('utf-8')).hexdigest()
            decision = replace(
                decision,
                robot1_task_id=first_id,
                robot2_task_id=second_id,
                decision_hash=decision_hash,
            )
            self.get_logger().info(
                'TRAFFIC_TEST_CONFLICT_PAIR round=%s r1=%s r2=%s '
                'minimum_separation_m=%.6f first_conflict=(%.6f,%.6f) '
                'last_conflict=(%.6f,%.6f) '
                'eta=(%.6f,%.6f)' % (
                    decision.round_id, first_id, second_id,
                    traffic.minimum_separation_m,
                    traffic.robot1_first_conflict_distance_m,
                    traffic.robot2_first_conflict_distance_m,
                    traffic.robot1_last_conflict_distance_m,
                    traffic.robot2_last_conflict_distance_m,
                    traffic.robot1_eta_s, traffic.robot2_eta_s,
                ),
            )
        return decision

    def _traffic_reason(self, traffic: TrafficDecision) -> str:
        """Keep event/status evidence compact, structured, and deterministic."""
        return json.dumps(traffic.as_dict(), sort_keys=True, separators=(',', ':'))

    def _retain_traffic_cleared_round(
            self, round_id: str, decision_hash: str,
            winner_robot_id: str) -> bool:
        """Retain an agreed deferred assignment after geometric clearance."""
        round_work = self._round
        if (round_work is None or round_work.round_id != str(round_id) or
                round_work.decision is None or
                round_work.decision.decision_hash != str(decision_hash)):
            return False
        traffic = round_work.traffic
        if traffic is not None:
            round_work.traffic = replace(
                traffic,
                conflict=False,
                winner_robot_id='',
                waiting_robot_id='',
                reason='CONFLICT_CLEARED',
            )
        self._traffic_reallocation_after_clear = True
        self._released_traffic_winner_robot_id = str(winner_robot_id)
        return True

    def _continue_traffic_hold(self, now: float) -> None:
        """Release on conflict clearance, or terminal evidence as fallback."""
        hold = self._traffic_hold
        if hold is None:
            return
        peer = self._peer_status
        peer_active = bool(
            peer is not None and peer.fresh(now) and
            (peer.value.local_nav_goal_active or
             peer.value.state == DistributedExplorationStatus.NAVIGATING)
        )
        if peer_active:
            hold.winner_observed_active = True
            if hold.winner_path:
                winner_pose = self._nav2.lookup_pose_in_global(
                    hold.winner_base_frame,
                )
                if winner_pose is not None:
                    point, stamp_ns, age_s = winner_pose
                    progress = project_path_progress(hold.winner_path, point)
                    if (progress is not None and
                            progress[0] >= hold.last_conflict_distance_m +
                            hold.clearance_m):
                        self._emit_event(
                            'TRAFFIC_CONFLICT_CLEARED',
                            'winner=%s progress_m=%.6f last_conflict_m=%.6f '
                            'clearance_m=%.6f tf_stamp_ns=%d tf_age_s=%.6f' % (
                                hold.winner_robot_id, progress[0],
                                hold.last_conflict_distance_m, hold.clearance_m,
                                stamp_ns, age_s,
                            ),
                        )
                        self._traffic_hold = None
                        self._emit_event(
                            'TRAFFIC_RELEASED_FRESH_REALLOCATION',
                            'winner cleared committed path-conflict interval; '
                            'retaining agreed deferred task',
                            duration=now - hold.created_steady_s,
                        )
                        if self._retain_traffic_cleared_round(
                                hold.round_id, hold.decision_hash,
                                hold.winner_robot_id):
                            self._transition(
                                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                                'traffic conflict cleared; dispatching retained deferred task',
                            )
                        else:
                            self._reset_round(
                                'traffic conflict cleared; agreed round unavailable',
                            )
                        return
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'traffic reservation held by active %s' % hold.winner_robot_id,
            )
            return
        if not hold.winner_observed_active and (
                now - hold.created_steady_s < self._traffic_dispatch_grace_s):
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'waiting for traffic winner %s dispatch heartbeat' % hold.winner_robot_id,
            )
            return
        first = self._fresh_snapshot('robot1', now)
        second = self._fresh_snapshot('robot2', now)
        if first is None or second is None:
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'traffic winner released; waiting for fresh task snapshots',
            )
            return
        if (first.epoch <= hold.snapshot_epochs[0] and
                second.epoch <= hold.snapshot_epochs[1]):
            self._transition(
                CoordinatorState.WAITING_FOR_TRAFFIC,
                'traffic winner released; stale deferred goal discarded pending newer proposals',
            )
            return
        waited = now - hold.created_steady_s
        self._emit_event(
            'TRAFFIC_RELEASED_FRESH_REALLOCATION',
            'winner terminal/released; stale deferred task will not dispatch',
            duration=waited,
        )
        self._traffic_hold = None
        self._reset_round('traffic reservation released; rebuilding from fresh proposals')

    def _begin_traffic_wait(self, traffic: TrafficDecision) -> None:
        """Defer only this new local goal; never inject a Nav2 velocity hold."""
        round_work = self._round
        if round_work is None or round_work.decision is None:
            return
        if self._traffic_hold is None:
            winner_batch = self._bid_batches.get(traffic.winner_robot_id)
            winner_task_id = (
                round_work.decision.robot1_task_id
                if traffic.winner_robot_id == 'robot1' else
                round_work.decision.robot2_task_id
            )
            winner_path = ()
            if winner_batch is not None:
                winner_path = next(
                    (tuple(bid.path) for bid in winner_batch.value.bids
                     if bid.canonical_task_id == winner_task_id),
                    (),
                )
            # A continuation round deliberately does not retain a full bid
            # vector for the busy peer.  Its immutable active commitment is
            # nevertheless the authoritative route reservation for traffic.
            # Reuse that path here so a conflict can clear from geometric
            # progress instead of being held until the busy goal terminates.
            if not winner_path:
                commitment = self._active_commitments.get(traffic.winner_robot_id)
                if (commitment is not None and
                        commitment.canonical_id == winner_task_id and
                        commitment.path):
                    winner_path = commitment.path
            last_conflict = (
                traffic.robot1_last_conflict_distance_m
                if traffic.winner_robot_id == 'robot1' else
                traffic.robot2_last_conflict_distance_m
            )
            self._traffic_hold = TrafficHold(
                round_id=round_work.round_id,
                decision_hash=round_work.decision.decision_hash,
                winner_robot_id=traffic.winner_robot_id,
                snapshot_epochs=(round_work.snapshots[0].epoch,
                                 round_work.snapshots[1].epoch),
                created_steady_s=time.monotonic(),
                winner_path=winner_path,
                winner_base_frame=f'{traffic.winner_robot_id}/base_footprint',
                last_conflict_distance_m=last_conflict,
                clearance_m=self._traffic_conflict_clearance_m,
            )
            # This round has become a held reservation.  A later conflict
            # clear event will explicitly re-enable one fresh round.
            self._traffic_reallocation_after_clear = False
            self._released_traffic_winner_robot_id = ''
            self._emit_event('TRAFFIC_WAITING', self._traffic_reason(traffic))
        self._transition(
            CoordinatorState.WAITING_FOR_TRAFFIC,
            'traffic deferred local dispatch behind %s' % traffic.winner_robot_id,
        )

    def _consider_completion(self, now: float) -> bool:
        """Require matching healthy empty-round persistence before COMPLETE."""
        nav2_healthy, tf_healthy = self._nav2.health_flags()
        peer = self._peer_status
        peer_fresh = peer is not None and peer.fresh(now)
        peer_healthy = bool(
            peer_fresh and peer.value.nav2_healthy and peer.value.tf_healthy and
            peer.value.candidate_source_healthy and
            peer.value.peer_communication_healthy and
            not peer.value.local_nav_goal_active
        )
        local_snapshot_fresh = self._fresh_snapshot(self._robot_id, now) is not None
        first = self._fresh_snapshot('robot1', now)
        second = self._fresh_snapshot('robot2', now)
        evidence_current = bool(
            first is not None and second is not None and
            completion_evidence_matches_snapshots(
                getattr(self, '_candidate_lower_bound_metadata', {}),
                (first, second),
            )
        )
        candidate_reason = None
        if (self._candidate_evidence_seen == {'robot1', 'robot2'} and
                evidence_current):
            candidate_reason = classify_empty_frontiers(
                self._candidate_evidence['robot1'],
                self._candidate_evidence['robot2'],
            )
        peer_reason = '' if not peer_fresh else str(peer.value.reason)
        if peer_reason.startswith('COMPLETION_CANDIDATE:'):
            peer_reason = peer_reason.split(':', 1)[1]
        peer_candidate = bool(
            peer_fresh and candidate_reason is not None and
            peer_reason == candidate_reason.value
        )
        local_planner_failures = sum(
            item.planner_failures for item in self._candidate_evidence.values()
        )
        peer_planner_failures = int(
            getattr(peer.value, 'planner_failure_count', 0)
        ) if peer_fresh else 0
        planner_failure_candidate = bool(
            self._candidate_evidence_seen == {'robot1', 'robot2'} and
            all(item.unclassified == 0 for item in self._candidate_evidence.values()) and
            credible_planner_infrastructure_failure(
                nav2_healthy, peer_healthy,
                local_planner_failures, peer_planner_failures,
            )
        )
        if planner_failure_candidate:
            if self._planner_failure_candidate_since_steady_s is None:
                self._planner_failure_candidate_since_steady_s = now
            if (peer_fresh and peer_reason == 'PLANNER_FAILURE_CANDIDATE' and
                    now - self._planner_failure_candidate_since_steady_s >=
                    self._planner_failure_confirmation_s):
                self._set_terminal(
                    TerminalReason.PLANNER_INFRASTRUCTURE.value, success=False,
                )
                return True
            self._transition(
                CoordinatorState.BLOCKED,
                'PLANNER_FAILURE_CANDIDATE',
            )
            return False
        self._planner_failure_candidate_since_steady_s = None
        if not evidence_current:
            self._completion_candidate_since_steady_s = None
            self._transition(
                CoordinatorState.BLOCKED,
                'candidate completion evidence is stale or provenance-incomplete',
            )
            return False
        healthy = (
            nav2_healthy and tf_healthy and peer_healthy and local_snapshot_fresh and
            not self._nav2.local_goal_active and not self._dispatch_in_progress
        )
        if not healthy or candidate_reason is None:
            self._completion_candidate_since_steady_s = None
            self._transition(
                CoordinatorState.BLOCKED,
                'empty task union but completion evidence is incomplete',
            )
            return False
        if self._completion_candidate_reason != candidate_reason.value:
            self._completion_candidate_reason = candidate_reason.value
            self._completion_candidate_since_steady_s = now
        if self._completion_candidate_since_steady_s is None:
            self._completion_candidate_since_steady_s = now
        stable_duration = now - self._maps_stable_since_steady_s
        candidate_duration = now - self._completion_candidate_since_steady_s
        if (stable_duration >= self._map_stability_grace_s and peer_candidate and
                candidate_duration >= self._completion_confirmation_s):
            self._set_terminal(candidate_reason.value, success=True)
            return True
        self._transition(
            CoordinatorState.WAITING_FOR_INPUTS,
            'COMPLETION_CANDIDATE:' + candidate_reason.value,
        )
        return False

    def _set_terminal(self, reason: str, success: bool) -> None:
        """Freeze local dispatch after a replicated terminal semantic result."""
        self._terminal_finalization_attempt_count += 1
        self.get_logger().info(
            'TERMINAL_FINALIZATION_ATTEMPT robot=%s reason=%s success=%s '
            'attempt=%d' % (
                self._robot_id, reason, bool(success),
                self._terminal_finalization_attempt_count,
            ),
        )
        if self._terminal:
            # Multiple callbacks may observe the same peer terminal state.  A
            # committed terminal result is immutable; duplicate observations
            # must be harmless and must not publish a second result.
            if (self._terminal_success != bool(success) or
                    self._terminal_reason != str(reason)):
                self.get_logger().warning(
                    'TERMINAL_FINALIZATION_CONFLICT robot=%s existing=%s '
                    'requested=%s' % (
                        self._robot_id, self._terminal_reason, str(reason),
                    ),
                )
            else:
                self.get_logger().info(
                    'TERMINAL_FINALIZATION_DUPLICATE robot=%s reason=%s' % (
                        self._robot_id, self._terminal_reason,
                    ),
                )
            return
        self._terminal = True
        self._terminal_success = bool(success)
        self._terminal_reason = str(reason)
        # _snapshots stores Received[TaskSnapshot].  The receipt wrapper owns
        # freshness metadata; epoch belongs to its immutable TaskSnapshot
        # payload and must be read through .value.
        self._terminal_epoch = max(
            (received.value.epoch for received in self._snapshots.values()),
            default=0,
        )
        self._terminal_commit_count += 1
        self.get_logger().info(
            'TERMINAL_FINALIZATION_COMMITTED robot=%s reason=%s success=%s '
            'epoch=%d commit_count=%d' % (
                self._robot_id, self._terminal_reason, self._terminal_success,
                self._terminal_epoch, self._terminal_commit_count,
            ),
        )
        state = CoordinatorState.COMPLETE if success else CoordinatorState.BLOCKED
        self._transition(state, self._terminal_reason)
        self._emit_event(
            'MISSION_COMPLETE' if success else 'MISSION_ABORTED',
            self._terminal_reason,
        )

    def _continue_degraded_solo(self, snapshot: TaskSnapshot) -> None:
        """Dispatch at most one locally proposed task per epoch without team claims."""
        if (not self._dispatch_enabled or self._dispatch_in_progress or
                not self._exploration_dispatch_allowed() or
                (self._initial_peer_readiness_barrier and
                 not self._initial_exploration_barrier.dispatch_allowed)):
            return
        # Proposal epochs may advance for heartbeats or unchanged reachability
        # metadata.  Reusing the existing semantic fingerprint avoids
        # repeating an identical local path/action decision while preserving
        # all behavior when task content changes.
        key = (
            snapshot.source_session_id,
            self._snapshot_content_fingerprint(snapshot, snapshot),
        )
        if self._last_solo_snapshot_key == key:
            return
        live_signatures = {
            item.physical_signature for item in snapshot.tasks
            if item.physical_signature
        }
        self._completed_solo_physical_signatures.intersection_update(
            live_signatures
        )
        candidates = eligible_solo_tasks(
            snapshot.tasks,
            self._hard_failure_signatures,
            self._completed_solo_physical_signatures,
            self._minimum_solo_visible_gain_m,
            self._minimum_solo_ordering_score,
            self._assignment_strategy,
        )
        if not candidates:
            self._last_solo_snapshot_key = key
            return
        now = time.monotonic()
        ready = tuple(
            task for task in candidates
            if self._solo_retry_not_before.get(
                task.physical_signature, 0.0) <= now
        )
        if not ready:
            self.get_logger().info(
                'DEGRADED_SOLO_RETRY_BACKOFF robot=%s candidates=%d '
                'next_s=%s' % (
                    self._robot_id, len(candidates),
                    min(self._solo_retry_not_before.get(
                        task.physical_signature, now) for task in candidates),
                )
            )
            return
        self._last_solo_snapshot_key = key
        ranked = rank_solo_tasks(
            ready, tuple(self._solo_route_history), self._weights,
            scoring_mode=self._assignment_strategy)
        selected_member = ranked[0]
        union = build_canonical_union(
            ready, (), min(self._maximum_union_tasks, len(ready)),
        )
        task = next(
            (candidate for candidate in union.tasks
             if any(
                 member.physical_signature == selected_member.physical_signature
                 for member in candidate.members)),
            None,
        )
        if task is None:
            self.get_logger().warning(
                'DEGRADED_SOLO_SELECTION_LOST_AFTER_CANONICALIZATION '
                'signature=%s' % selected_member.physical_signature,
            )
            return
        candidate_audit = ';'.join(
            '%s:score=%.6f,gain=%.6f,stored_path_m=%.3f,overlap=%.3f' % (
                candidate.physical_signature,
                candidate.local_ordering_score,
                candidate.visible_reveal_gain,
                candidate.local_path_length_m,
                max((
                    route_overlap(candidate.local_path, prior,
                                  self._weights.route_corridor_radius_m)
                    for prior in self._solo_route_history
                ), default=0.0),
            )
            for candidate in ranked[:self._maximum_path_queries]
        )
        self.get_logger().info(
            'DEGRADED_SOLO_SELECTED robot=%s signature=%s score=%.6f gain=%.6f '
            'stored_path_m=%.3f route_overlap=%.3f selection_reason=%s '
            'candidates=%s' % (
                self._robot_id, selected_member.physical_signature,
                selected_member.local_ordering_score,
                selected_member.visible_reveal_gain,
                selected_member.local_path_length_m,
                max((
                    route_overlap(selected_member.local_path, prior,
                                  self._weights.route_corridor_radius_m)
                    for prior in self._solo_route_history
                ), default=0.0),
                'GENERATOR_SCORE_PRIMARY_ROUTE_NOVELTY_SECONDARY',
                candidate_audit,
            ),
        )
        self._dispatch_in_progress = True
        self._active_task = task
        self._active_round_id = 'degraded:%s:%s:%d' % (
            self._robot_id, snapshot.source_session_id, snapshot.epoch,
        )
        self._active_decision_hash = 'DEGRADED_SOLO'
        round_id = self._active_round_id

        def final_path(result: PathEvaluation):
            if self._active_round_id != round_id:
                return
            if not result.valid:
                self._invalidate_round(
                    result.failure_class,
                    'degraded solo ComputePathToPose failed: %s' % result.error_message,
                    result,
                )
                return
            self._nav2.check_dispatch_preconditions(
                task.members[0], True,
                lambda checks: self._dispatch_after_checks(task, result, checks),
                path_samples=result.samples,
                path_frame_id=result.path_frame_id,
                path=result.path,
            )

        if not self._nav2.evaluate_path(
                task.members[0], final_path, caller='DEGRADED_SOLO_DISPATCH'):
            failure_reason = getattr(
                self._nav2, 'path_start_failure_reason', lambda: '',
            )()
            if failure_reason != 'PATH_QUERY_LEASE_BUSY':
                self._invalidate_round(
                    FailureClass.TF_OR_LIFECYCLE,
                    'degraded solo local path action unavailable')
                return
            # Candidate generation may briefly own the per-robot planner
            # lease.  This is not a task/path failure: preserve the current
            # local seed and retry it on the next allocator tick rather than
            # waiting for a new snapshot to re-arm degraded-solo work.
            self._dispatch_in_progress = False
            self._active_task = None
            self._active_round_id = ''
            self._active_decision_hash = ''
            self._last_solo_snapshot_key = None
            self._local_fallback_trigger_pending = True
            self._local_fallback_trigger_reason = (
                'local fallback waiting for ComputePathToPose query lease')
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'waiting for local ComputePathToPose query lease',
            )

    def _continue_bidding(self, round_work: RoundWork, generation: int) -> None:
        """Continue bounded bidding and attribute the whole call if enabled."""
        return self._allocator_timing_timed_call(
            'continue_bidding', self._continue_bidding_impl,
            round_work, generation,
        )

    @staticmethod
    def _local_path_execution_key(
            round_work: RoundWork, task: CanonicalTask) -> tuple:
        """Return the exact task/provenance key for a reusable path result.

        The frozen ``CanonicalTask`` contains every source member and every
        path-relevant task field, rather than only its quantized canonical ID.
        The source snapshot adds candidate-generation and lower-bound
        provenance that is not carried by ``PhysicalTask`` itself.
        """
        source_robot_id = task.members[0].source_robot_id if task.members else ''
        source_snapshot = next(
            (snapshot for snapshot in round_work.snapshots
             if snapshot.source_robot_id == source_robot_id),
            None,
        )
        snapshot_provenance = None if source_snapshot is None else (
            source_snapshot.source_robot_id,
            source_snapshot.source_session_id,
            source_snapshot.epoch,
            source_snapshot.map_revision,
            source_snapshot.map_fingerprint,
            source_snapshot.generation_ros_ns,
            source_snapshot.lower_bound_context_fingerprint,
            source_snapshot.costmap_revision,
            source_snapshot.candidate_generation_id,
        )
        return task, snapshot_provenance

    @staticmethod
    def _local_path_result_cacheable(
            task: CanonicalTask, result: PathEvaluation) -> bool:
        """Accept only a successful finite allocator path result."""
        if not task.members or result.caller != 'ALLOCATOR_BID':
            return False
        if result.task_signature != task.members[0].physical_signature:
            return False
        if (result.error_code != 0 or
                result.failure_class != FailureClass.UNKNOWN or
                not path_is_valid_finite(result)):
            return False
        if (not math.isfinite(float(result.heading_cost)) or
                result.heading_cost < 0.0):
            return False
        return result.map_stamp_ns > 0 and result.costmap_stamp_ns > 0

    def _remember_local_path_evaluation(
            self, round_work: RoundWork, task: CanonicalTask,
            result: PathEvaluation) -> None:
        """Store one successful result with deterministic bounded eviction."""
        if not self._local_path_result_cacheable(task, result):
            return
        key = self._local_path_execution_key(round_work, task)
        cache = self._local_path_evaluation_cache
        if key not in cache and len(cache) >= LOCAL_PATH_EVALUATION_CACHE_MAX_ENTRIES:
            oldest = next(iter(cache))
            cache.pop(oldest)
            self._local_path_cache_evictions += 1
        cache[key] = result

    def _cached_local_path_evaluation(
            self, round_work: RoundWork,
            task: CanonicalTask) -> Optional[PathEvaluation]:
        """Return a current-context exact cached path, if one exists."""
        key = self._local_path_execution_key(round_work, task)
        cached = self._local_path_evaluation_cache.get(key)
        if cached is None:
            self._local_path_cache_misses += 1
            return None
        if not self._local_path_result_cacheable(task, cached):
            self._local_path_evaluation_cache.pop(key, None)
            self._local_path_cache_invalidations += 1
            self._local_path_cache_misses += 1
            return None
        if not self._nav2.path_context_matches(cached):
            self._local_path_evaluation_cache.pop(key, None)
            self._local_path_cache_invalidations += 1
            self._local_path_cache_misses += 1
            return None
        self._local_path_cache_hits += 1
        return cached

    def _continue_bidding_impl(self, round_work: RoundWork,
                               generation: int) -> None:
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'bid continuation entry')
            return
        if round_work.query_index >= len(round_work.query_tasks):
            self._finish_bids(round_work, generation)
            return
        task = round_work.query_tasks[round_work.query_index]
        if self._synthetic_bids:
            distance = math.dist(self._synthetic_origin, task.approach)
            samples = tuple((
                self._synthetic_origin[0] + step / 6.0 * (
                    task.approach[0] - self._synthetic_origin[0]
                ),
                self._synthetic_origin[1] + step / 6.0 * (
                    task.approach[1] - self._synthetic_origin[1]
                ),
            ) for step in range(7))
            self._append_bid(round_work, generation, task, PathEvaluation(
                True, distance, samples, self.get_clock().now().nanoseconds,
                0, '', FailureClass.UNKNOWN,
            ))
            return
        round_id = round_work.round_id

        local_member = next(
            (member for member in task.members
             if member.source_robot_id == self._robot_id),
            None,
        )
        if (local_member is not None and local_member.local_path_valid and
                math.isfinite(local_member.local_path_length_m) and
                local_member.local_path_length_m >= 0.0):
            self._append_bid(round_work, generation, task, PathEvaluation(
                True, local_member.local_path_length_m,
                tuple(local_member.local_path),
                self.get_clock().now().nanoseconds, 0, 'reused local candidate path',
                FailureClass.UNKNOWN,
                heading_cost=local_member.path_heading_cost_rad,
            ))
            return

        cached = self._cached_local_path_evaluation(round_work, task)
        if cached is not None:
            # _append_bid constructs a new bid for this exact current round;
            # the cached object supplies only the still-valid path result.
            self._append_bid(round_work, generation, task, cached)
            return

        def completed(result: PathEvaluation):
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(
                    round_work, generation, 'bid completion callback',
                )
                return
            self._append_bid(round_work, generation, task, result)

        if not self._nav2.evaluate_path(
                task.members[0], completed, caller='ALLOCATOR_BID'):
            # The per-robot planner lease may be held briefly by the
            # candidate generator.  Keep the round in bidding and retry on
            # the next coordinator tick instead of manufacturing a blocked
            # round or discarding the current task set.
            self._transition(
                CoordinatorState.BIDDING,
                'waiting for local ComputePathToPose query lease',
            )

    def _append_bid(
            self, round_work: RoundWork, generation: int,
            task: CanonicalTask, result: PathEvaluation) -> None:
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'append bid')
            return
        source_stamp = max(member.generation_ros_ns for member in task.members)
        heading_cost = float(result.heading_cost)
        if not math.isfinite(heading_cost) or heading_cost < 0.0:
            heading_cost = 0.0
        own_utility = (
            -nominal_motion_cost_s(
                result.length_m, heading_cost,
                self._weights.cost_only_reference_linear_speed_mps,
                self._weights.cost_only_reference_angular_speed_radps,
            )
            if self._assignment_strategy == 'frontier_cost_only' else
            task.visible_reveal_gain - result.length_m
        )
        bid = Bid(
            canonical_task_id=task.canonical_id,
            path_valid=result.valid,
            path_length_m=result.length_m,
            estimated_travel_cost=result.length_m,
            heading_cost=heading_cost,
            own_utility_contribution=own_utility,
            task_generation_ros_ns=source_stamp,
            path_query_ros_ns=result.query_ros_ns,
            path=result.samples,
        )
        round_work.bids = round_work.bids + (bid,)
        if (result.valid and result.caller != 'UNKNOWN' and
                result.map_stamp_ns and result.costmap_stamp_ns):
            round_work.local_path_evaluations[task.canonical_id] = result
        self._remember_local_path_evaluation(round_work, task, result)
        round_work.query_index += 1
        self._continue_bidding(round_work, generation)

    def _finish_bids(self, round_work: RoundWork, generation: int) -> None:
        if (not self._round_is_current(round_work, generation) or
                round_work.local_batch is not None):
            if not self._round_is_current(round_work, generation):
                self._discard_stale_tick(round_work, generation, 'finish bids')
            return
        local_snapshot = next(
            (snapshot for snapshot in round_work.snapshots
             if snapshot.source_robot_id == self._robot_id),
            None,
        )
        if local_snapshot is None:
            self._discard_stale_tick(
                round_work, generation, 'local source snapshot missing',
            )
            return
        batch = BidBatch(
            round_id=round_work.round_id,
            union_hash=round_work.union.union_hash,
            source_robot_id=self._robot_id,
            source_session_id=local_snapshot.source_session_id,
            source_snapshot_epoch=local_snapshot.epoch,
            validity_s=self._bid_validity_s,
            bids=round_work.bids,
        )
        round_work.local_batch = batch
        self._publish_local_bid_batch(round_work, generation, log_batch=True)

    def _publish_local_bid_batch(
            self, round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None, log_batch: bool = False) -> None:
        """Refresh the local bid heartbeat without changing round semantics."""
        round_work = self._round if round_work is None else round_work
        if round_work is None or round_work.local_batch is None:
            return
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'publish bid batch')
            return
        batch = round_work.local_batch
        message = bid_batch_to_msg(batch, self.get_clock().now().to_msg())
        self._bid_publisher.publish(message)
        self._bid_batches[self._robot_id] = receive(
            batch, batch.validity_s, time.monotonic(),
        )
        if log_batch:
            self.get_logger().info(
                'BID_ARRAY robot=%s round=%s union=%s bids=%d' % (
                    self._robot_id, batch.round_id, batch.union_hash, len(batch.bids),
                )
            )

    def _both_bid_batches_valid(
            self, now: float, round_work: Optional[RoundWork] = None) -> bool:
        round_work = self._round if round_work is None else round_work
        if round_work is None:
            return False
        for index, robot_id in enumerate(('robot1', 'robot2')):
            snapshot = round_work.snapshots[index]
            received = self._bid_batches.get(robot_id)
            if received is None or not bid_batch_valid(
                    received, now, robot_id, snapshot.source_session_id,
                    snapshot.epoch, round_work.round_id,
                    round_work.union.union_hash, True):
                return False
        return True

    def _peer_bid_matches_latest_normal_round(
            self, now: float, first: TaskSnapshot, second: TaskSnapshot,
            round_id: str, union_hash: str) -> bool:
        """Return whether the peer confirms the newer normal-round context."""
        received = self._bid_batches.get(self._peer_id)
        if received is None:
            return False
        peer_snapshot = second if self._peer_id == 'robot2' else first
        return bid_batch_valid(
            received, now, self._peer_id, peer_snapshot.source_session_id,
            peer_snapshot.epoch, round_id, union_hash, True,
        )

    def _publish_decision(
            self, round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None,
            log_decision: bool = True) -> None:
        round_work = self._round if round_work is None else round_work
        if round_work is None or round_work.decision is None:
            return
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'publish decision')
            return
        # Continuation rounds intentionally order snapshots by canonical
        # robot identity while the local robot may be the free peer.  Bind
        # the message provenance by source identity, never by tuple position.
        local_snapshot = next(
            (snapshot for snapshot in round_work.snapshots
             if snapshot.source_robot_id == self._robot_id),
            None,
        )
        if local_snapshot is None:
            self._discard_stale_tick(
                round_work, generation, 'local source snapshot missing',
            )
            return
        message = decision_to_msg(
            round_work.decision, self._robot_id, local_snapshot.source_session_id,
            round_work.snapshots[0].epoch, round_work.snapshots[1].epoch,
            self._state.value, self._decision_validity_s,
            self.get_clock().now().to_msg(),
        )
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'before decision publish')
            return
        self._decision_publisher.publish(message)
        if generation is not None and not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'after decision publish')
            return
        round_work.decision_published = True
        if not log_decision:
            return
        score = round_work.decision.score
        diagnostics = round_work.decision.diagnostics
        if diagnostics.strategy == 'burgard':
            self.get_logger().info(
                'BURGARD_DECISION robot=%s round=%s union=%s hash=%s '
                'r1=%s r2=%s beta=%.6f path_limit=%.6f sensor_range=%.6f '
                'selection_total=%.6f trace=%s traffic=%s' % (
                    self._robot_id, round_work.round_id,
                    round_work.union.union_hash,
                    round_work.decision.decision_hash,
                    round_work.decision.robot1_task_id or 'IDLE',
                    round_work.decision.robot2_task_id or 'IDLE',
                    diagnostics.beta, diagnostics.feasible_path_limit_m,
                    diagnostics.sensor_max_range_m, score.total,
                    json.dumps(diagnostics.burgard_trace, sort_keys=True,
                               separators=(',', ':')),
                    json.dumps(diagnostics.traffic, sort_keys=True,
                               separators=(',', ':')),
                )
            )
            return
        if diagnostics.strategy == 'frontier_cost_only':
            motion_cost_s = (
                score.combined_path_cost /
                self._weights.cost_only_reference_linear_speed_mps +
                score.combined_heading_cost /
                self._weights.cost_only_reference_angular_speed_radps
            )
            self.get_logger().info(
                'FRONTIER_COST_ONLY_DECISION robot=%s round=%s union=%s hash=%s '
                'r1=%s r2=%s motion_cost_s=%.6f path_m=%.6f heading_rad=%.6f '
                'nearby=%.6f route=%.6f '
                'hard=%.6f sensing=%.6f imbalance=%.6f gain_ignored=true '
                'traffic=%s' % (
                    self._robot_id, round_work.round_id,
                    round_work.union.union_hash,
                    round_work.decision.decision_hash,
                    round_work.decision.robot1_task_id or 'IDLE',
                    round_work.decision.robot2_task_id or 'IDLE',
                    motion_cost_s, score.combined_path_cost,
                    score.combined_heading_cost,
                    score.nearby_goal_penalty, score.route_overlap_penalty,
                    score.hard_failure_penalty, score.sensing_overlap_penalty,
                    score.workload_imbalance_penalty,
                    json.dumps(diagnostics.traffic, sort_keys=True,
                               separators=(',', ':')),
                )
            )
            return
        if diagnostics.strategy == 'frontier_gain':
            self.get_logger().info(
                'FRONTIER_GAIN_DECISION robot=%s round=%s union=%s hash=%s '
                'r1=%s r2=%s generator_score=%.6f gain=%.6f path=%.6f '
                'nearby=%.6f route=%.6f hard=%.6f sensing=%.6f '
                'imbalance=%.6f traffic=%s' % (
                    self._robot_id, round_work.round_id,
                    round_work.union.union_hash,
                    round_work.decision.decision_hash,
                    round_work.decision.robot1_task_id or 'IDLE',
                    round_work.decision.robot2_task_id or 'IDLE',
                    score.team_local_ordering_score,
                    score.team_visible_gain, score.combined_path_cost,
                    score.nearby_goal_penalty, score.route_overlap_penalty,
                    score.hard_failure_penalty, score.sensing_overlap_penalty,
                    score.workload_imbalance_penalty,
                    json.dumps(diagnostics.traffic, sort_keys=True,
                               separators=(',', ':')),
                )
            )
            return
        if diagnostics.strategy == 'frontier_mrtsp':
            self.get_logger().info(
                'FRONTIER_MRTSP_DECISION robot=%s round=%s union=%s hash=%s '
                'r1=%s r2=%s gain=%.6f path=%.6f route=%s traffic=%s' % (
                    self._robot_id, round_work.round_id,
                    round_work.union.union_hash,
                    round_work.decision.decision_hash,
                    round_work.decision.robot1_task_id or 'IDLE',
                    round_work.decision.robot2_task_id or 'IDLE',
                    score.team_visible_gain, score.combined_path_cost,
                    json.dumps(diagnostics.burgard_trace, sort_keys=True,
                               separators=(',', ':')),
                    json.dumps(diagnostics.traffic, sort_keys=True,
                               separators=(',', ':')),
                )
            )
            return
        self.get_logger().info(
            'PAIR_DECISION robot=%s round=%s union=%s hash=%s r1=%s r2=%s '
            'total=%.6f gain=%.6f path=%.6f nearby=%.6f route=%.6f hard=%.6f '
            'sensing=%.6f imbalance=%.6f' % (
                self._robot_id, round_work.round_id, round_work.union.union_hash,
                round_work.decision.decision_hash,
                round_work.decision.robot1_task_id or 'IDLE',
                round_work.decision.robot2_task_id or 'IDLE', score.total,
                score.team_visible_gain, score.combined_path_cost,
                score.nearby_goal_penalty, score.route_overlap_penalty,
                score.hard_failure_penalty, score.sensing_overlap_penalty,
                score.workload_imbalance_penalty,
            )
        )

    def _matching_peer_decision(
            self, round_work: RoundWork, generation: int, now: float) -> bool:
        if (not self._round_is_current(round_work, generation) or
                round_work.decision is None or
                self._peer_decision is None or not self._peer_decision.fresh(now)):
            return False
        peer = self._peer_decision.value
        local = round_work.decision
        peer_snapshot = next(
            (snapshot for snapshot in round_work.snapshots
             if snapshot.source_robot_id == self._peer_id),
            None,
        )
        if peer_snapshot is None:
            return False
        local_selector_fingerprint = str(
            getattr(local.diagnostics, 'selector_feasibility_fingerprint', '') or '')
        peer_selector_fingerprint = self._peer_selector_feasibility_fingerprint(peer)
        expected_session = peer_snapshot.source_session_id
        same_context = (
            uuid_to_text(peer.source_session_id) == expected_session and
            peer.round_id == local.round_id and
            peer.union_hash == local.union_hash and
            peer.robot1_snapshot_epoch == round_work.snapshots[0].epoch and
            peer.robot2_snapshot_epoch == round_work.snapshots[1].epoch and
            peer.robot1_bid_fingerprint == local.robot1_bid_fingerprint and
            peer.robot2_bid_fingerprint == local.robot2_bid_fingerprint
        )
        if not same_context:
            return False
        peer_diagnostics = {}
        try:
            peer_diagnostics = json.loads(
                str(getattr(peer, 'diagnostics_json', '') or '{}'),
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            pass
        if (not local_selector_fingerprint or
                peer_selector_fingerprint != local_selector_fingerprint):
            self._decision_identity_diagnostic(
                'SELECTOR_FEASIBILITY_DIVERGENCE', round_work, {
                    'local_fingerprint': local_selector_fingerprint,
                    'peer_fingerprint': peer_selector_fingerprint,
                    'local_completed': list(getattr(
                        local.diagnostics, 'selector_completed_task_ids', ())),
                    'peer_completed': peer_diagnostics.get(
                        'selector_completed_task_ids', ()),
                    'local_hard_failed': list(getattr(
                        local.diagnostics, 'selector_hard_failed_task_ids', ())),
                    'peer_hard_failed': peer_diagnostics.get(
                        'selector_hard_failed_task_ids', ()),
                    'local_peer_reservations': list(getattr(
                        local.diagnostics,
                        'selector_peer_reservation_task_ids', ())),
                    'peer_peer_reservations': peer_diagnostics.get(
                        'selector_peer_reservation_task_ids', ()),
                },
            )
            return False
        if (
                peer.robot1_canonical_task_id != local.robot1_task_id or
                peer.robot2_canonical_task_id != local.robot2_task_id or
                peer.decision_hash != local.decision_hash
        ):
            self._decision_identity_diagnostic(
                'PAIR_DECISION_INVARIANT_VIOLATION', round_work, {
                    'union_hash': round_work.union.union_hash,
                    'robot1_bid_fingerprint': local.robot1_bid_fingerprint,
                    'robot2_bid_fingerprint': local.robot2_bid_fingerprint,
                    'selector_feasibility_fingerprint': (
                        local_selector_fingerprint),
                    'local_tasks': (
                        local.robot1_task_id, local.robot2_task_id),
                    'peer_tasks': (
                        str(peer.robot1_canonical_task_id),
                        str(peer.robot2_canonical_task_id)),
                    'local_decision_hash': local.decision_hash,
                    'peer_decision_hash': str(peer.decision_hash),
                    'source_sessions': (
                        round_work.snapshots[0].source_session_id,
                        round_work.snapshots[1].source_session_id),
                    'snapshot_epochs': (
                        round_work.snapshots[0].epoch,
                        round_work.snapshots[1].epoch),
                },
            )
            return False
        return True

    def _start_local_dispatch(self, round_work: RoundWork, generation: int) -> None:
        if not self._round_is_current(round_work, generation):
            self._discard_stale_tick(round_work, generation, 'start local dispatch')
            return
        if round_work.decision is None:
            return
        if not self._exploration_dispatch_allowed():
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'common START_RELEASE barrier pending',
            )
            return
        if (round_work.mode == 'continuation' and
                self._robot_id == round_work.continuation_busy_robot_id):
            commitment = self._active_commitments.get(self._robot_id)
            busy_task_id = (
                round_work.decision.robot1_task_id
                if self._robot_id == 'robot1' else
                round_work.decision.robot2_task_id
            )
            if (commitment is not None and
                    busy_task_id == commitment.canonical_id and
                    self._nav2.local_goal_active):
                self._transition(
                    CoordinatorState.NAVIGATING,
                    'retaining active goal during continuation agreement',
                )
                return
            self._invalidate_round(
                FailureClass.EXPLICIT_CANCELLATION,
                'continuation busy commitment no longer active',
            )
            return
        if (self._traffic_reallocation_after_clear and
                self._nav2.local_goal_active):
            # The previous winner participates in the fresh replicated round
            # so the waiter can obtain matching evidence, but it must retain
            # its already-issued NavigateToPose goal.  Only the idle waiter
            # may dispatch from this round.
            self._traffic_reallocation_after_clear = False
            self._released_traffic_winner_robot_id = ''
            self._transition(
                CoordinatorState.NAVIGATING,
                'retaining active winner goal during fresh traffic round',
            )
            return
        traffic = round_work.traffic
        if (self._traffic_scheduler_enabled and traffic is not None and
                traffic.conflict):
            if traffic.waiting_robot_id == self._robot_id:
                self._begin_traffic_wait(traffic)
                return
            if traffic.winner_robot_id == self._robot_id:
                self._emit_event('TRAFFIC_PRIORITY_GRANTED', self._traffic_reason(traffic))
            elif traffic.reason == 'BOTH_ALREADY_ACTIVE_MONITOR_ONLY':
                self._emit_event('TRAFFIC_MONITOR_ONLY', self._traffic_reason(traffic))
        task_id = (
            round_work.decision.robot1_task_id if self._robot_id == 'robot1'
            else round_work.decision.robot2_task_id
        )
        if not task_id:
            local = next(
                (snapshot for snapshot in round_work.snapshots
                 if snapshot.source_robot_id == self._robot_id), None)
            if not self._continue_local_work_while_waiting(
                    local, 'temporary local work while agreed assignment is IDLE'):
                self._transition(
                    CoordinatorState.WAITING_FOR_INPUTS,
                    'local assignment is IDLE',
                )
            return
        tasks = {task.canonical_id: task for task in round_work.union.tasks}
        task = tasks.get(task_id)
        if task is None:
            self._invalidate_round(FailureClass.UNKNOWN, 'agreed task missing from union')
            return
        self._dispatch_in_progress = True
        self._active_task = task
        self._active_round_id = round_work.round_id
        self._active_decision_hash = round_work.decision.decision_hash
        round_id = self._active_round_id

        def final_path(result: PathEvaluation):
            if (self._active_round_id != round_id or
                    not self._round_is_current(round_work, generation)):
                self._discard_stale_tick(round_work, generation, 'final path callback')
                return
            if not result.valid:
                self._invalidate_round(
                    result.failure_class,
                    'final ComputePathToPose failed: %s' % result.error_message,
                    result,
                )
                return
            self._nav2.check_dispatch_preconditions(
                task.members[0], True,
                lambda checks: self._dispatch_after_checks(
                    task, result, checks, round_work, generation,
                ),
                path_samples=result.samples,
                path_frame_id=result.path_frame_id,
                path=result.path,
            )

        local_evaluation = round_work.local_path_evaluations.get(task_id)
        if (local_evaluation is not None and
                local_evaluation.path is not None and
                self._nav2.path_context_matches(local_evaluation)):
            # The local bid was computed for this exact immutable round.  The
            # dispatch gate still rechecks TF, map, costmap, lifecycle, and
            # active-goal state, so reusing the path does not make stale
            # planner output authoritative.
            self.get_logger().info(
                'COMPUTE_PATH_REUSED source=ALLOCATOR_BID task=%s round=%s '
                'path_length_m=%.3f' % (
                    task_id, round_work.round_id, local_evaluation.length_m,
                )
            )
            final_path(local_evaluation)
            return
        if not self._nav2.evaluate_path(
                task.members[0], final_path,
                caller='FINAL_DISPATCH_VALIDATION'):
            # A local path query can be serialized behind the candidate
            # generator.  Preserve the agreed round and retry dispatch; this
            # is not evidence that the task is unreachable.
            self._dispatch_in_progress = False
            self._active_task = None
            self._active_round_id = ''
            self._active_decision_hash = ''
            self._transition(
                CoordinatorState.WAITING_FOR_MATCHING_DECISION,
                'waiting for local ComputePathToPose query lease',
            )

    def _dispatch_after_checks(
            self, task: CanonicalTask, final_path: PathEvaluation,
            checks: DispatchPreconditions,
            round_work: Optional[RoundWork] = None,
            generation: Optional[int] = None) -> None:
        if (round_work is not None and generation is not None and
                not self._round_is_current(round_work, generation)):
            self._discard_stale_tick(round_work, generation, 'dispatch checks callback')
            self._dispatch_in_progress = False
            return
        failure = None
        if not checks.ready:
            failure = classify_solo_dispatch_failure(
                checks, local_only=self._local_only)
        failure_memory_interaction = (
            'NOT_APPLICABLE' if checks.ready else
            'HARD_FAILURE_SUPPRESSION_NEXT' if failure in HARD_FAILURES else
            'NO_HARD_FAILURE_SUPPRESSION')
        member = task.members[0] if task.members else None

        def point_record(point):
            if point is None:
                return None
            return {'x': float(point[0]), 'y': float(point[1])}

        def pose_record(pose_stamped):
            if pose_stamped is None:
                return None
            position = pose_stamped.pose.position
            rotation = pose_stamped.pose.orientation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
            )
            return {
                'x': float(position.x), 'y': float(position.y),
                'yaw': float(yaw),
                'frame_id': str(pose_stamped.header.frame_id or ''),
            }

        exact_path = final_path.path
        path_poses = () if exact_path is None else tuple(exact_path.poses)
        self.get_logger().info(
            'DISPATCH_GATE_DECISION %s' % json.dumps({
                'mode': checks.local_path_gate_mode,
                'robot': self._robot_id,
                'task_signature': (
                    task.members[0].physical_signature if task.members else ''),
                'maximum_local_cost': checks.local_path_maximum_cost,
                'first_blocked_point_index': (
                    checks.local_path_first_blocked_point_index),
                'first_blocked_point': checks.local_path_first_blocked_point,
                'first_blocked_local_point': (
                    checks.local_path_first_blocked_local_point),
                'nav2_path_valid': checks.final_path_valid,
                'candidate_goal': None if task is None else {
                    'x': float(task.approach[0]),
                    'y': float(task.approach[1]),
                    'yaw': float(task.approach_yaw),
                },
                'candidate_approach': None if member is None else {
                    'x': float(member.approach[0]),
                    'y': float(member.approach[1]),
                    'yaw': float(member.approach_yaw),
                },
                'path_frame': str(final_path.path_frame_id or ''),
                'path_pose_count': len(path_poses),
                'path_length_m': float(final_path.length_m),
                'path_first_pose': pose_record(path_poses[0]) if path_poses else None,
                'path_last_pose': pose_record(path_poses[-1]) if path_poses else None,
                'is_path_valid_service': checks.path_valid_service_name,
                'is_path_valid_checked': checks.path_valid_checked,
                'is_path_valid': checks.path_valid,
                'invalid_pose_indices': checks.path_invalid_pose_indices,
                'local_footprint_checked': checks.local_footprint_checked,
                'local_footprint_clear': checks.local_footprint_clear,
                'local_footprint_reason': checks.local_footprint_reason,
                'local_footprint_inspected_poses': (
                    checks.local_footprint_inspected_poses),
                'local_footprint_first_blocked_pose_index': (
                    checks.local_footprint_first_blocked_pose_index),
                'local_footprint_maximum_cost': (
                    checks.local_footprint_maximum_cost),
                'local_footprint_first_blocked_pose_yaw': (
                    checks.local_footprint_first_blocked_pose_yaw),
                'local_footprint_first_blocked_cell': (
                    checks.local_footprint_first_blocked_cell),
                'local_footprint_first_blocked_cell_world': (
                    point_record(checks.local_footprint_first_blocked_cell_world)),
                'local_footprint_effective_footprint': (
                    checks.local_footprint_effective_footprint),
                'local_costmap_frame': checks.local_footprint_costmap_frame,
                'local_costmap_origin': point_record(
                    checks.local_footprint_costmap_origin),
                'local_costmap_origin_yaw': checks.local_footprint_costmap_origin_yaw,
                'local_costmap_width': checks.local_footprint_costmap_width,
                'local_costmap_height': checks.local_footprint_costmap_height,
                'local_costmap_resolution': checks.local_footprint_costmap_resolution,
                'local_costmap_stamp_ns': checks.local_footprint_costmap_stamp_ns,
                'map_to_odom_translation': point_record(
                    checks.local_footprint_path_transform_translation),
                'map_to_odom_yaw': checks.local_footprint_path_transform_yaw,
                'map_to_odom_stamp_ns': checks.local_footprint_path_transform_stamp_ns,
                'map_to_odom_age_s': checks.local_footprint_path_transform_age_s,
                'preflight_only': getattr(self, '_preflight_only', False),
                'final_decision': (
                    'PREFLIGHT_ONLY_ACCEPT'
                    if checks.ready and getattr(self, '_preflight_only', False) else
                    'ACCEPT_FOR_DISPATCH' if checks.ready else 'REJECT'),
                'failure_memory_interaction': failure_memory_interaction,
                'failure_class': None if failure is None else failure.value,
                'reason': checks.reason,
            }, sort_keys=True, separators=(',', ':')),
        )
        self.get_logger().info(
            'DISPATCH_PRECONDITIONS robot=%s round=%s task=%s ready=%s '
            'action=%s lifecycle=%s tf=%s tf_age=%s map_inside=%s map_value=%s '
            'costmap_inside=%s costmap_value=%s local_path_clear=%s '
            'local_path_reason=%s local_path_inspected=%s '
            'local_path_outside=%s local_goal_clear=%s path=%s '
            'is_path_valid=%s local_footprint_clear=%s reason=%s' % (
                self._robot_id, self._active_round_id, task.canonical_id, checks.ready,
                checks.action_server_ready, checks.lifecycle_active,
                checks.transform_available, checks.transform_age_s,
                checks.goal_inside_map, checks.goal_map_value,
                checks.goal_inside_costmap, checks.goal_costmap_value,
                checks.local_path_clear, checks.local_path_reason,
                checks.local_path_inspected_points,
                checks.local_path_outside_points,
                checks.no_local_goal_active,
                checks.final_path_valid, checks.path_valid,
                checks.local_footprint_clear, checks.reason,
            )
        )
        if not checks.ready:
            self._invalidate_round(failure, checks.reason, final_path)
            return
        if getattr(self, '_preflight_only', False):
            self.get_logger().info(
                'PREFLIGHT_ONLY_NO_NAV_GOAL task=%s path_length_m=%.6f' % (
                    task.canonical_id, final_path.length_m,
                )
            )
            self._dispatch_in_progress = False
            self._active_task = None
            self._active_round_id = ''
            self._active_decision_hash = ''
            self._last_solo_snapshot_key = None
            self._reset_round('preflight-only diagnostic result')
            return
        if self._local_only and self._active_task is not None:
            selected = self._active_task.members[0]
            overlap = max((
                route_overlap(selected.local_path, prior,
                              self._weights.route_corridor_radius_m)
                for prior in self._solo_route_history
            ), default=0.0)
            self.get_logger().info(
                'DEGRADED_SOLO_SELECTION_VALIDATED robot=%s signature=%s '
                'score=%.6f gain=%.6f stored_path_m=%.3f fresh_path_m=%.6f '
                'route_overlap=%.3f selection_reason=%s' % (
                    self._robot_id, selected.physical_signature,
                    selected.local_ordering_score,
                    selected.visible_reveal_gain,
                    selected.local_path_length_m, final_path.length_m, overlap,
                    'GENERATOR_SCORE_PRIMARY_ROUTE_NOVELTY_SECONDARY',
                ),
            )
        if not path_is_valid_finite(final_path):
            self.get_logger().warning(
                'DISPATCH_REJECTED_INVALID_PATH robot=%s round=%s task=%s '
                'path_length_m=%.6f valid=%s' % (
                    self._robot_id, self._active_round_id, task.canonical_id,
                    final_path.length_m,
                    final_path.valid,
                ),
            )
            self._invalidate_round(
                FailureClass.HARD_UNREACHABLE,
                'fresh dispatch path is invalid or non-finite',
                final_path,
            )
            return
        if (getattr(self, '_active_navigation_action', None) is not None or
                getattr(self._nav2, 'local_goal_active', False)):
            self._invalidate_round(
                FailureClass.ACTION_REJECTION,
                'local navigation action already active', final_path,
            )
            return
        self._navigation_action_sequence = getattr(
            self, '_navigation_action_sequence', 0,
        ) + 1
        member = task.members[0]
        action_commitment = getattr(
            self, '_active_commitments', {},
        ).get(self._robot_id)
        action_round_id = self._active_round_id
        action_decision_hash = getattr(self, '_active_decision_hash', '')
        if (
                action_commitment is not None and
                action_commitment.canonical_id == task.canonical_id
        ):
            # A continuation wrapper is coordination-local.  Once the
            # commitment exists, terminal events must carry the immutable
            # action lineage independently derived by both replicas.
            action_round_id = action_commitment.decision_round_id
            action_decision_hash = action_commitment.decision_hash
        action = ActiveNavigationAction(
            action_id='%s:%d:%s' % (
                self._robot_id, self._navigation_action_sequence,
                task.canonical_id,
            ),
            task=task,
            canonical_task_id=task.canonical_id,
            physical_signature=member.physical_signature,
            round_id=action_round_id,
            decision_hash=action_decision_hash,
            generation=(generation if generation is not None else getattr(
                getattr(self, '_round_lifecycle', None), 'generation', 0)),
            path=tuple(final_path.samples),
            commitment_id=getattr(
                action_commitment,
                'commitment_id', '',
            ),
        )
        self._active_navigation_action = action

        def navigation_finished(outcome: NavigationOutcome) -> None:
            self._navigation_finished(outcome, action)

        if not self._nav2.send_navigation(
                member, navigation_finished,
                diagnostic_path=final_path.samples):
            if self._active_navigation_action is action:
                self._active_navigation_action = None
                self._active_task = None
                self._active_round_id = ''
                self._active_decision_hash = ''
            self._invalidate_round(
                FailureClass.ACTION_REJECTION,
                'local NavigateToPose send precondition changed', final_path,
            )
            return
        action.state = 'ACTIVE'
        self._active_dispatch_path = tuple(final_path.samples)
        self._dispatch_count += 1
        self._dispatch_in_progress = False
        self._traffic_reallocation_after_clear = False
        self._released_traffic_winner_robot_id = ''
        self._transition(CoordinatorState.NAVIGATING, 'local agreed goal accepted for send')
        self._emit_event(
            'NAV_GOAL_SENT', 'local-only NavigateToPose dispatch',
            path_length=final_path.length_m,
        )
        if (round_work is not None and
                round_work.mode == 'continuation'):
            self._emit_event(
                'CONTINUATION_GOAL_DISPATCHED',
                json.dumps({
                    'mode': 'continuation',
                    'busy_robot': round_work.continuation_busy_robot_id,
                    'commitment_id': round_work.continuation_commitment_id,
                    'canonical_task_id': task.canonical_id,
                }, sort_keys=True, separators=(',', ':')),
            )
        if (not self._local_only and
                not self._first_cooperative_goal_logged):
            self._first_cooperative_goal_logged = True
            self._startup_event(
                'FIRST_COOPERATIVE_GOAL', round_id=self._active_round_id,
                canonical_task_id=task.canonical_id,
                path_length_m=final_path.length_m,
            )

    def _navigation_finished(
            self, outcome: NavigationOutcome,
            action: Optional[ActiveNavigationAction] = None) -> None:
        action = action or getattr(self, '_active_navigation_action', None)
        if action is None:
            self.get_logger().error(
                'NAVIGATION_TERMINAL_WITHOUT_OWNERSHIP_RECORD',
            )
            return
        current_action = getattr(self, '_active_navigation_action', None)
        owns_current_action = current_action is action
        if not owns_current_action:
            action.state = 'TERMINAL'
            self.get_logger().warning(
                'NAVIGATION_TERMINAL_STALE_ACTION action_id=%s current=%s' % (
                    action.action_id,
                    '' if current_action is None else current_action.action_id,
                ),
            )
            return
        action.state = 'TERMINAL'
        result = 'SUCCEEDED' if (
            outcome.status == 4 and outcome.error_code == 0
        ) else 'FAILED'
        if result != 'SUCCEEDED':
            self.get_logger().warning(
                'NAV2_TERMINAL_RESULT robot=%s status=%s accepted=%s '
                'error_code=%s error_message=%r failure_class=%s recoveries=%s '
                'duration_s=%.3f travelled_m=%.3f follow_path_code=%s '
                'follow_path_name=%s failure_family=%s deepest_failure=%s' % (
                    self._robot_id, outcome.status, outcome.accepted,
                    outcome.error_code, outcome.error_message,
                    outcome.failure_class.value, outcome.recoveries,
                    outcome.duration_s, outcome.travelled_distance_m,
                    outcome.follow_path_error_code,
                    outcome.follow_path_error_name,
                    outcome.controller_failure_family,
                    outcome.deepest_failure_classification))
            physical_signature = action.physical_signature
            task_diag = self._failure_task_diagnostics.setdefault(
                physical_signature or '__unknown__', {
                    'controller_failure_count': 0,
                    'tf_failure_count': 0,
                    'last_failure_type': '',
                    'last_failure_time_s': 0.0,
                    'last_failure_geometry_signature': '',
                    'consecutive_structural_failure_count': 0,
                },
            )
            if outcome.controller_failure_family.startswith('CONTROLLER'):
                task_diag['controller_failure_count'] = int(
                    task_diag['controller_failure_count']) + 1
                task_diag['consecutive_structural_failure_count'] = int(
                    task_diag['consecutive_structural_failure_count']) + 1
            elif outcome.controller_failure_family == 'NAV_INFRASTRUCTURE_TF':
                task_diag['tf_failure_count'] = int(task_diag['tf_failure_count']) + 1
                task_diag['consecutive_structural_failure_count'] = 0
            else:
                task_diag['consecutive_structural_failure_count'] = 0
            task_diag['last_failure_type'] = outcome.deepest_failure_classification
            task_diag['last_failure_time_s'] = self.get_clock().now().nanoseconds / 1e9
            if outcome.diagnostic_snapshot_json:
                try:
                    task_diag['last_failure_geometry_signature'] = json.loads(
                        outcome.diagnostic_snapshot_json,
                    ).get('execution_geometry_signature', '')
                except (TypeError, ValueError):
                    pass
            self.get_logger().warning(
                'NAVIGATION_TASK_FAILURE_COUNTS physical_signature=%s data=%s' %
                (physical_signature, json.dumps(task_diag, sort_keys=True)),
            )
            if outcome.diagnostic_snapshot_json:
                self.get_logger().warning(
                    'NAVIGATION_FAILURE_SNAPSHOT %s' % outcome.diagnostic_snapshot_json,
                )
            self.get_logger().warning(
                'NAVIGATION_FAILURE_PROPAGATION robot=%s physical_signature=%s '
                'navigate_to_pose_error_code=%s navigate_to_pose_error_text=%r '
                'follow_path_error_code=%s follow_path_error_name=%s '
                'failure_family=%s deepest_failure=%s '
                'deepest_failure_timestamp_ros_ns=%s' % (
                    self._robot_id, physical_signature, outcome.error_code,
                    outcome.error_message, outcome.follow_path_error_code,
                    outcome.follow_path_error_name, outcome.controller_failure_family,
                    outcome.deepest_failure_classification,
                    outcome.deepest_failure_timestamp_ros_ns,
                )
            )
        self._emit_event(
            'NAVIGATION_' + result, outcome.error_message or result,
            travelled=outcome.travelled_distance_m,
            duration=outcome.duration_s, recoveries=outcome.recoveries,
            failure=outcome.failure_class,
            nav2_error_code=outcome.error_code,
            nav2_error_message=outcome.error_message,
            nav2_error_name=outcome.nav2_error_name,
            action=action,
        )
        if result != 'SUCCEEDED':
            self._publish_failure(
                outcome.failure_class, outcome.error_message,
                nav2_error_code=outcome.error_code,
                nav2_error_message=outcome.error_message,
                nav2_error_name=outcome.nav2_error_name,
                action=action,
            )
        elif action.task is not None:
            # Suppress only the exact physical region that just succeeded.
            # A later disappearance from the snapshot permits it to be
            # reconsidered; unchanged residual fragments cannot churn goals.
            self._completed_solo_physical_signatures.update(
                member.physical_signature
                for member in action.task.members
                if member.physical_signature
            )
            if not self._local_only:
                self._completed_shared_canonical_ids.add(action.canonical_task_id)
                self.get_logger().info(
                    'COMPLETED_FRONTIER_LOCAL robot=%s task=%s' % (
                        self._robot_id, action.canonical_task_id))
            if self._local_only and action.path:
                self._solo_route_history.append(action.path)
            if action.physical_signature:
                signature = action.physical_signature
                self._solo_retry_not_before.pop(signature, None)
                self._solo_retry_counts.pop(signature, None)
        self._active_navigation_action = None
        self._active_task = None
        self._active_round_id = ''
        self._active_decision_hash = ''
        self._dispatch_in_progress = False
        self._active_dispatch_path = ()
        self._clear_active_commitment(
            self._robot_id, 'local navigation commitment terminated',
        )
        self._settle_until_steady_s = time.monotonic() + self._post_goal_settle_s
        # Robot availability is part of the semantic trigger.  A completed
        # goal must permit a fresh auction even when the task-set fingerprint
        # is unchanged.
        self._last_semantic_fingerprint = ''
        self._last_solo_snapshot_key = None
        self._reset_round('navigation terminal result')
        self._start_immediate_fallback_after_terminal()

    def _invalidate_round(
            self, failure: FailureClass, reason: str,
            path: Optional[PathEvaluation] = None) -> None:
        self._publish_failure(failure, reason, path)
        self._emit_event('ROUND_INVALIDATED', reason, failure=failure)
        self._dispatch_in_progress = False
        action = getattr(self, '_active_navigation_action', None)
        if action is None and not getattr(self._nav2, 'local_goal_active', False):
            self._active_task = None
            self._active_round_id = ''
            self._active_decision_hash = ''
        elif action is not None:
            # The auction lease may be invalidated, but the action lease must
            # survive until LocalNav2 invokes the terminal callback.  Keeping
            # these fields aligned also preserves status/event observability;
            # _navigation_finished() uses the immutable action record itself.
            self._active_task = action.task
            self._active_round_id = action.round_id
            self._active_decision_hash = action.decision_hash
        else:
            # Fail closed if an externally active goal is ever observed without
            # an allocator record.  Do not erase the last known ownership while
            # Nav2 may still be executing it.
            self.get_logger().error(
                'NAVIGATION_OWNERSHIP_RECORD_MISSING_WHILE_ACTIVE '
                'reason=%s' % reason,
            )
        # A failure changes the effective feasible-pair set even when the
        # published task geometry is unchanged.  Permit exactly one fresh
        # semantic round so failure suppression can take effect.
        self._last_semantic_fingerprint = ''
        self._last_solo_snapshot_key = None
        self._settle_until_steady_s = time.monotonic() + self._post_goal_settle_s
        self._reset_round('round invalidated')

    def _publish_failure(
            self, failure: FailureClass, reason: str,
            path: Optional[PathEvaluation] = None,
            nav2_error_code: int = 0, nav2_error_message: str = '',
            nav2_error_name: str = '',
            action: Optional[ActiveNavigationAction] = None) -> None:
        owner = action or getattr(self, '_active_navigation_action', None)
        task = owner.task if owner is not None else self._active_task
        if task is None:
            return
        member = task.members[0]
        if failure in HARD_FAILURES:
            # Record local suppression before consulting snapshot provenance;
            # a stale snapshot must not make the failing robot immediately
            # reselect the same physical task.
            self._record_hard_failure(
                member.physical_signature, 15.0,
                canonical_task_id=str(task.canonical_id),
                failure_class=failure.value,
                reason=reason,
            )
        elif self._local_only and member.physical_signature:
            # Infrastructure/TF failures are retryable, but never in a tight
            # loop while the same stale condition persists.
            count = self._solo_retry_counts.get(member.physical_signature, 0) + 1
            self._solo_retry_counts[member.physical_signature] = count
            delay_s = solo_retry_delay_s(count)
            self._solo_retry_not_before[member.physical_signature] = (
                time.monotonic() + delay_s)
            self.get_logger().info(
                'SOLO_RETRY_BACKOFF signature=%s count=%d delay_s=%.3f' % (
                    member.physical_signature, count, delay_s))
        local_snapshot = self._fresh_snapshot(self._robot_id, time.monotonic())
        if local_snapshot is None:
            return
        message = ExplorationFailure()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        message.source_session_id = text_to_uuid(local_snapshot.source_session_id)
        message.round_id = (
            owner.round_id if owner is not None else self._active_round_id)
        message.canonical_task_id = task.canonical_id
        message.physical_task_signature = member.physical_signature
        message.approach_pose.header = message.header
        message.approach_pose.pose.position.x, message.approach_pose.pose.position.y = (
            member.approach
        )
        message.approach_pose.pose.orientation.z = math.sin(member.approach_yaw / 2.0)
        message.approach_pose.pose.orientation.w = math.cos(member.approach_yaw / 2.0)
        message.failure_class = FAILURE_TO_MESSAGE[failure]
        if path is not None:
            message.path_length_m = path.length_m
            for x, y in path.samples:
                from geometry_msgs.msg import Point

                message.path_samples.append(Point(x=x, y=y))
        message.retry_count = 1
        message.nav2_error_code = int(max(0, nav2_error_code))
        message.nav2_error_message = nav2_error_message
        message.nav2_error_name = nav2_error_name or {
            102: 'TF_ERROR',
            104: 'PATIENCE_EXCEEDED',
            105: 'FAILED_TO_MAKE_PROGRESS',
            106: 'NO_VALID_CONTROL',
            107: 'CONTROLLER_TIMED_OUT',
        }.get(int(nav2_error_code), '')
        message.validity = seconds_to_duration(15.0 if failure in HARD_FAILURES else 2.0)
        message.evidence = reason
        self._failure_publisher.publish(message)

    def _hard_failed_task_ids(self, union: CanonicalUnion) -> frozenset[str]:
        signatures = set(self._hard_failure_signatures)
        return frozenset(
            task.canonical_id for task in union.tasks
            if any(member.physical_signature in signatures for member in task.members)
        )

    def _expire_failures(self, now: float) -> None:
        self._hard_failure_signatures = {
            signature: expiry for signature, expiry in self._hard_failure_signatures.items()
            if expiry > now
        }

    def _reset_round(self, reason: str) -> None:
        previous = self._round
        new_generation = self._round_lifecycle.invalidate()
        if previous is not None:
            completed = reason == 'navigation terminal result'
            if completed:
                self._round_completed_count += 1
                self._last_round_completion_steady_s = time.monotonic()
                self._log_round_lifecycle(
                    'ALLOCATOR_ROUND_COMPLETED', previous, new_generation, reason,
                )
            else:
                self._log_round_lifecycle(
                    'ALLOCATOR_ROUND_CLEARED', previous, new_generation, reason,
                )
        self._round = None
        # A reset discards the decision context that made this semantic
        # fingerprint suppress a new round.  Leaving it armed can strand the
        # allocator after a continuation is invalidated before its replacement
        # decision is agreed: the next ordinary snapshots match the previous
        # fingerprint and _tick_impl() returns through its unchanged-content
        # gate instead of creating a canonical round.
        self._last_semantic_fingerprint = ''
        self._bid_batches.clear()
        self._peer_decision = None
        self._committed = CommittedRound()
        self._transition(CoordinatorState.WAITING_FOR_INPUTS, reason)

    def _transition(self, state: CoordinatorState, reason: str) -> None:
        if state == self._state and reason == self._state_reason:
            return
        previous = self._state
        self._state, self._state_reason = state, reason
        now = time.monotonic()
        ages = {}
        for robot_id, received_snapshot in self._snapshots.items():
            ages[robot_id] = max(0.0, now - received_snapshot.receipt_steady_s)
        peer_bid = self._bid_batches.get(self._peer_id)
        self.get_logger().info(
            'STATE_TRANSITION robot=%s session=%s round=%s previous=%s next=%s '
            'reason=%s task=%s peer_state=%s local_snapshot_age=%s peer_snapshot_age=%s '
            'local_bid_age=%s peer_bid_age=%s decision=%s nav_active=%s' % (
                self._robot_id, self._local_session_text(), self._current_round_id(),
                previous.value, state.value, reason,
                '' if self._active_task is None else self._active_task.canonical_id,
                self._peer_state_text(), ages.get(self._robot_id), ages.get(self._peer_id),
                self._receipt_age(self._bid_batches.get(self._robot_id), now),
                self._receipt_age(peer_bid, now), self._active_decision_hash,
                self._nav2.local_goal_active,
            )
        )
        self._emit_event(
            'STATE_TRANSITION', reason,
            previous=previous.value, next_state=state.value,
        )

    @staticmethod
    def _receipt_age(received_value, now: float):
        return None if received_value is None else now - received_value.receipt_steady_s

    def _local_session_text(self) -> str:
        item = self._snapshots.get(self._robot_id)
        return '' if item is None else item.value.source_session_id

    def _current_round_id(self) -> str:
        round_work = self._round
        return '' if round_work is None else round_work.round_id

    def _peer_state_text(self) -> str:
        if self._peer_status is None:
            return 'UNKNOWN'
        return str(self._peer_status.value.state)

    def _publish_status(self) -> None:
        self._nav2.refresh_health()
        round_work = self._round
        generation = self._round_lifecycle.generation
        self._publish_local_bid_batch(round_work, generation)
        if round_work is not None and round_work.decision is not None:
            self._publish_decision(round_work, generation, log_decision=False)
        self._publish_health_diagnostic()
        message = DistributedExplorationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        session = self._local_session_text()
        if session:
            message.source_session_id = text_to_uuid(session)
        message.state = STATE_TO_MESSAGE[self._state]
        message.round_id = '' if round_work is None else round_work.round_id
        message.union_hash = '' if round_work is None else round_work.union.union_hash
        decision = None if round_work is None else round_work.decision
        message.decision_hash = '' if decision is None else decision.decision_hash
        message.active_canonical_task_id = (
            '' if self._active_task is None else self._active_task.canonical_id
        )
        message.local_nav_goal_active = self._nav2.local_goal_active
        message.nav2_healthy, message.tf_healthy = self._nav2.health_flags()
        message.candidate_source_healthy = self._fresh_snapshot(
            self._robot_id, time.monotonic(),
        ) is not None
        message.peer_communication_healthy = self._peer_liveness.state != (
            CoordinatorState.DEGRADED_SOLO
        )
        message.terminal = self._terminal
        message.terminal_reason = self._terminal_reason
        message.terminal_epoch = self._terminal_epoch
        message.terminal_map_revision = max(
            (received.value.map_revision
             for received in self._snapshots.values()),
            default=0,
        )
        evidence = tuple(self._candidate_evidence.values())
        message.remaining_frontier_count = sum(item.detected for item in evidence)
        message.remaining_small_frontier_count = sum(item.small for item in evidence)
        message.remaining_out_of_range_count = sum(item.out_of_range for item in evidence)
        message.remaining_unreachable_count = sum(item.unreachable for item in evidence)
        message.planner_failure_count = sum(item.planner_failures for item in evidence)
        message.detected_not_queried_count = sum(
            item.detected_not_queried for item in evidence)
        message.below_minimum_gain_count = sum(
            item.below_minimum_gain for item in evidence)
        message.actionable_reachable_count = sum(
            (item.actionable_reachable if item.actionable_reachable is not None
             else item.reachable) for item in evidence)
        # These fields are passive evidence only.  They expose the existing
        # source-local candidate state explicitly so avoidable idle can be
        # reconstructed offline without treating absence of a dispatch as
        # proof that work was available.
        local_evidence = getattr(
            self, '_candidate_source_local_evidence', {}).get(
                self._robot_id, CandidateEvidence())
        local_feasible = int(local_evidence.reachable) > 0
        local_actionable = int(
            local_evidence.actionable_reachable or 0) > 0
        message.feasible_work_available = local_feasible
        message.actionable_work_available = local_actionable
        if local_actionable:
            message.work_availability_reason = 'ACTIONABLE_REACHABLE'
        elif local_feasible:
            message.work_availability_reason = 'REACHABLE_BELOW_GAIN'
        elif local_evidence.detected_not_queried:
            message.work_availability_reason = 'UNQUERIED_EVIDENCE_PENDING'
        elif local_evidence.detected:
            message.work_availability_reason = 'NO_FEASIBLE_REACHABLE_TASK'
        else:
            message.work_availability_reason = 'NO_DETECTED_FRONTIER'
        message.validity = seconds_to_duration(2.5)
        message.reason = self._state_reason
        self._status_publisher.publish(message)

    def _emit_event(
            self, event_type: str, reason: str, previous: str = '',
            next_state: str = '', path_length: float = 0.0,
            travelled: float = 0.0, duration: float = 0.0,
            recoveries: int = 0, failure: FailureClass = FailureClass.UNKNOWN,
            nav2_error_code: int = 0, nav2_error_message: str = '',
            nav2_error_name: str = '',
            action: Optional[ActiveNavigationAction] = None) -> None:
        message = DistributedExplorationEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        session = self._local_session_text()
        if session:
            message.source_session_id = text_to_uuid(session)
        message.event_type = event_type
        round_work = self._round
        message.round_id = (
            action.round_id if action is not None else
            self._active_round_id or (
                '' if round_work is None else round_work.round_id))
        message.union_hash = '' if round_work is None else round_work.union.union_hash
        # Agreement and state-transition events can be emitted before a local
        # task is promoted to ``_active_task``.  Preserve the round's decision
        # fingerprint in that interval instead of emitting an empty hash.
        message.decision_hash = (
            action.decision_hash if action is not None else
            self._active_decision_hash)
        if (not message.decision_hash and round_work is not None and
                round_work.decision is not None):
            message.decision_hash = round_work.decision.decision_hash
        if action is not None:
            message.canonical_task_id = action.canonical_task_id
            message.physical_task_signature = action.physical_signature
        elif self._active_task is not None:
            message.canonical_task_id = self._active_task.canonical_id
            message.physical_task_signature = self._active_task.members[0].physical_signature
        message.previous_state = previous
        message.next_state = next_state
        message.reason = reason
        message.path_length_m = path_length
        message.travelled_distance_m = travelled
        message.navigation_duration_s = duration
        message.result = event_type
        message.failure_class = FAILURE_TO_MESSAGE[failure]
        message.recoveries = recoveries
        message.nav2_error_code = int(max(0, nav2_error_code))
        message.nav2_error_message = nav2_error_message
        message.nav2_error_name = nav2_error_name or {
            102: 'TF_ERROR',
            104: 'PATIENCE_EXCEEDED',
            105: 'FAILED_TO_MAKE_PROGRESS',
            106: 'NO_VALID_CONTROL',
            107: 'CONTROLLER_TIMED_OUT',
        }.get(int(nav2_error_code), '')
        if round_work is not None and round_work.decision is not None:
            message.route_overlap_score = round_work.decision.score.route_overlap_penalty
            message.sensing_overlap_estimate = (
                round_work.decision.score.sensing_overlap_penalty
            )
        self._event_publisher.publish(message)

    def _log_union(self, round_work: RoundWork) -> None:
        self.get_logger().info(
            'CANONICAL_UNION robot=%s round=%s union=%s tasks=%s' % (
                self._robot_id, round_work.round_id, round_work.union.union_hash,
                ','.join(task.canonical_id for task in round_work.union.tasks),
            )
        )


def main(args=None):
    """Run one namespaced replicated assignment peer."""
    rclpy.init(args=args)
    node = DistributedFrontierAssignment()
    executor_override = os.environ.get('MY_EPUCK_ASSIGNMENT_EXECUTOR_THREADS')
    executor_threads = max(1, int(executor_override or node._executor_threads))
    if executor_threads == 1:
        executor = SingleThreadedExecutor()
    else:
        executor = MultiThreadedExecutor(num_threads=executor_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
