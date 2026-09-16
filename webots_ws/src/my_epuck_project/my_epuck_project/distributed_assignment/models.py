"""Transport-independent models for exactly two assignment peers."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional, Tuple


Point = Tuple[float, float]


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned world bounds for compact frontier geometry."""

    minimum: Point
    maximum: Point


@dataclass(frozen=True)
class PhysicalTask:
    """One source robot's physical observation-task proposal."""

    source_robot_id: str
    source_session_id: str
    source_snapshot_epoch: int
    source_map_revision: int
    physical_signature: str
    local_frontier_id: int
    centroid: Point
    bounds: Bounds
    approach: Point
    approach_yaw: float = 0.0
    frontier_geometry: Tuple[Point, ...] = ()
    visible_cells: Tuple[Point, ...] = ()
    visible_bounds: Optional[Bounds] = None
    visible_reveal_gain: float = 0.0
    local_ordering_score: float = 0.0
    # Upstream MRTSP route context is ordinal and source-local.  It replaces
    # the old batch-normalized scalar as the route-aware mode's preference
    # evidence; UINT32_MAX denotes an item outside the bounded route horizon.
    mrtsp_route_rank: int = (2 ** 32 - 1)
    mrtsp_route_generation: int = 0
    mrtsp_solver: str = ''
    local_path_valid: bool = False
    local_path_length_m: float = 0.0
    local_path: Tuple[Point, ...] = ()
    planned_path: Any = None
    path_heading_cost_rad: float = 0.0
    generation_ros_ns: int = 0


@dataclass(frozen=True)
class TaskSnapshot:
    """Bounded task snapshot with ROS and lower-bound provenance."""

    source_robot_id: str
    source_session_id: str
    epoch: int
    map_revision: int
    map_fingerprint: str
    generation_ros_ns: int
    validity_s: float
    tasks: Tuple[PhysicalTask, ...]
    # Separate from sender TTL: unchanged snapshot evidence is not a
    # heartbeat, while a context change invalidates old lower bounds.
    lower_bound_context_fingerprint: str = ''
    # Diagnostic provenance only; does not participate in allocation validity.
    costmap_revision: int = 0
    # Immutable source-local candidate-generation identity.  Zero means that
    # the producer did not provide the required provenance.
    candidate_generation_id: int = 0


@dataclass(frozen=True)
class CanonicalTask:
    """Deterministic union task produced from equivalent source proposals."""

    canonical_id: str
    members: Tuple[PhysicalTask, ...]
    centroid: Point
    bounds: Bounds
    approach: Point
    approach_yaw: float
    frontier_geometry: Tuple[Point, ...]
    visible_cells: Tuple[Point, ...]
    visible_bounds: Optional[Bounds]
    visible_reveal_gain: float


@dataclass(frozen=True)
class CanonicalUnion:
    """Canonical ordered physical task set for one coordination round."""

    tasks: Tuple[CanonicalTask, ...]
    union_hash: str


@dataclass(frozen=True)
class Bid:
    """One robot's bounded local Nav2 evaluation of a canonical task."""

    canonical_task_id: str
    path_valid: bool
    path_length_m: float
    estimated_travel_cost: float
    heading_cost: float = 0.0
    own_utility_contribution: float = 0.0
    task_generation_ros_ns: int = 0
    path_query_ros_ns: int = 0
    path: Tuple[Point, ...] = ()


@dataclass(frozen=True)
class BidBatch:
    """Round-bound bid array from one robot."""

    round_id: str
    union_hash: str
    source_robot_id: str
    source_session_id: str
    source_snapshot_epoch: int
    validity_s: float
    bids: Tuple[Bid, ...]


@dataclass(frozen=True)
class AssignmentScore:
    """Bounded, individually auditable pair-score terms."""

    team_visible_gain: float = 0.0
    # Sum of the source frontier generator's already-computed local scores.
    # This is populated only by the explicit frontier_gain strategy; keeping
    # it in the transport-independent score makes the mode auditable without
    # changing the existing PairDecision wire fields.
    team_local_ordering_score: float = 0.0
    combined_path_cost: float = 0.0
    combined_heading_cost: float = 0.0
    nearby_goal_penalty: float = 0.0
    route_overlap_penalty: float = 0.0
    hard_failure_penalty: float = 0.0
    sensing_overlap_penalty: float = 0.0
    workload_imbalance_penalty: float = 0.0
    total: float = 0.0


@dataclass(frozen=True)
class AssignmentDiagnostics:
    """Deterministic explanation of a pair decision, including IDLE cases."""

    idle_reason: str = 'OTHER'
    availability_reason: str = 'OTHER'
    union_task_count: int = 0
    robot1_snapshot_task_count: int = 0
    robot2_snapshot_task_count: int = 0
    robot1_bid_count: int = 0
    robot2_bid_count: int = 0
    robot1_valid_bid_count: int = 0
    robot2_valid_bid_count: int = 0
    reachable_by_both_count: int = 0
    reachable_only_robot1_count: int = 0
    reachable_only_robot2_count: int = 0
    rejected_equivalence_count: int = 0
    rejected_freshness_count: int = 0
    rejected_failure_suppression_count: int = 0
    rejected_active_reservation_count: int = 0
    rejected_gain_threshold_count: int = 0
    rejected_local_score_threshold_count: int = 0
    rejected_path_threshold_count: int = 0
    feasible_useful_robot1_count: int = 0
    feasible_useful_robot2_count: int = 0
    valid_one_active_assignment_count: int = 0
    valid_two_active_pair_count: int = 0
    idle_idle_permitted: bool = False
    robot1_idle_reason: str = 'NOT_IDLE'
    robot2_idle_reason: str = 'NOT_IDLE'
    best_non_idle_robot1_task_id: str = ''
    best_non_idle_robot2_task_id: str = ''
    best_non_idle_score: AssignmentScore = AssignmentScore()
    idle_score: float = 0.0
    # ``legacy_weighted`` retains the historical seven-term diagnostic score.
    # ``burgard`` serializes the bounded, deterministic Algorithm-1-style
    # selection trace instead of relabelling old heuristic terms.
    strategy: str = 'legacy_weighted'
    beta: float = 0.0
    feasible_path_limit_m: float = 0.0
    sensor_max_range_m: float = 0.0
    burgard_trace: Tuple[dict[str, Any], ...] = ()
    traffic: dict[str, Any] = field(default_factory=dict)
    # Deterministic selector-feasibility provenance.  These fields bind the
    # local suppression inputs that are not represented by the bid arrays.
    selector_feasibility_fingerprint: str = ''
    selector_completed_task_ids: Tuple[str, ...] = ()
    selector_hard_failed_task_ids: Tuple[str, ...] = ()
    selector_peer_reservation_task_ids: Tuple[str, ...] = ()


@dataclass(frozen=True)
class PairDecision:
    """Complete replicated assignment for Robot 1 and Robot 2."""

    round_id: str
    union_hash: str
    robot1_task_id: str
    robot2_task_id: str
    robot1_bid_fingerprint: str
    robot2_bid_fingerprint: str
    score: AssignmentScore
    decision_hash: str
    combined_path_length_m: float
    maximum_path_length_m: float
    diagnostics: AssignmentDiagnostics = AssignmentDiagnostics()


class FailureClass(str, Enum):
    """Evidence-conservative navigation failure classes."""

    HARD_UNREACHABLE = 'HARD_UNREACHABLE'
    PLANNER_FAILURE = 'PLANNER_FAILURE'
    CONTROLLER_NO_PROGRESS = 'CONTROLLER_NO_PROGRESS'
    DYNAMIC_BLOCKAGE = 'DYNAMIC_BLOCKAGE'
    TF_OR_LIFECYCLE = 'TF_OR_LIFECYCLE'
    ACTION_REJECTION = 'ACTION_REJECTION'
    TIMEOUT = 'TIMEOUT'
    EXPLICIT_CANCELLATION = 'EXPLICIT_CANCELLATION'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class FailureEvidence:
    """Only directly observed evidence used for classification."""

    compute_path_error: Optional[str] = None
    nav2_error_code: Optional[int] = None
    nav2_error_message: str = ''
    controller_no_progress: bool = False
    dynamic_obstacle_confirmed: bool = False
    tf_unavailable: bool = False
    lifecycle_inactive: bool = False
    action_rejected: bool = False
    timed_out: bool = False
    explicitly_cancelled: bool = False


@dataclass(frozen=True)
class FailureRecord:
    """Bounded team-visible failure evidence for one physical task."""

    source_robot_id: str
    source_session_id: str
    round_id: str
    canonical_task_id: str
    physical_signature: str
    approach: Point
    failure_class: FailureClass
    retry_count: int
    validity_s: float
    alternative_approach: bool = False


class CoordinatorState(str, Enum):
    """Small public coordinator lifecycle."""

    WAITING_FOR_INPUTS = 'WAITING_FOR_INPUTS'
    BIDDING = 'BIDDING'
    WAITING_FOR_MATCHING_DECISION = 'WAITING_FOR_MATCHING_DECISION'
    WAITING_FOR_TRAFFIC = 'WAITING_FOR_TRAFFIC'
    NAVIGATING = 'NAVIGATING'
    DEGRADED_SOLO = 'DEGRADED_SOLO'
    COMPLETE = 'COMPLETE'
    BLOCKED = 'BLOCKED'


@dataclass(frozen=True)
class CompletionInputs:
    """Health and stability evidence required for operational completion."""

    both_snapshots_fresh: bool
    both_statuses_fresh: bool
    robot1_has_valid_task: bool
    robot2_has_valid_task: bool
    valid_pair_exists: bool
    shared_maps_stable: bool
    active_assignment: bool
    local_nav_goal_active: bool
    peer_nav_goal_active: bool
    tf_healthy: bool
    nav2_healthy: bool
    candidates_healthy: bool
    communication_healthy: bool
    peer_completion_matches: bool
    condition_duration_s: float
    confirmation_interval_s: float


@dataclass
class TravelDistance:
    """Accumulate odometric distance while rejecting discontinuous jumps."""

    maximum_step_m: float = 1.0
    distance_m: float = 0.0
    _previous: Optional[Point] = field(default=None, repr=False)

    def observe(self, point: Point) -> float:
        """Add one valid odometry displacement and return the total."""
        import math

        if self._previous is not None:
            step = math.dist(self._previous, point)
            if 0.0 <= step <= self.maximum_step_m:
                self.distance_m += step
        self._previous = point
        return self.distance_m
