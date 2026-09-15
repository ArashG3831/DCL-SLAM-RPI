"""Round freshness, source provenance, agreement, and completion rules."""

from dataclasses import dataclass
from typing import Generic, Optional, TypeVar

from .models import (
    BidBatch,
    CompletionInputs,
    CoordinatorState,
    PairDecision,
    TaskSnapshot,
)


T = TypeVar('T')


def clamp_ttl(ttl_s: float, minimum_s: float = 0.1, maximum_s: float = 10.0) -> float:
    """Clamp sender-advertised validity before local steady-clock use."""
    if ttl_s != ttl_s:
        return minimum_s
    return max(minimum_s, min(maximum_s, ttl_s))


@dataclass(frozen=True)
class Received(Generic[T]):
    """Message receipt whose age uses only the receiver's steady clock."""

    value: T
    receipt_steady_s: float
    accepted_ttl_s: float

    def fresh(self, now_steady_s: float) -> bool:
        """Return whether the locally measured receipt age is within TTL."""
        age = now_steady_s - self.receipt_steady_s
        return 0.0 <= age <= self.accepted_ttl_s


def receive(value: T, sender_ttl_s: float, now_steady_s: float) -> Received[T]:
    """Record a value with receiver-local expiry state."""
    return Received(value, now_steady_s, clamp_ttl(sender_ttl_s))


@dataclass(frozen=True)
class SourceVersion:
    """Last accepted source-local session, epoch, and map revision."""

    session_id: str
    snapshot_epoch: int
    map_revision: int


class SnapshotLedger:
    """Reject stale snapshots without comparing revisions across sources."""

    def __init__(self, maximum_tasks_per_source: int = 5):
        """Configure the per-source snapshot task bound."""
        self._maximum_tasks_per_source = maximum_tasks_per_source
        self._versions: dict[str, SourceVersion] = {}
        self._retired_sessions: dict[str, set[str]] = {}

    def accept(self, snapshot: TaskSnapshot) -> bool:
        """Validate identity, bounds, session, epoch, and source-local revision."""
        if snapshot.source_robot_id not in ('robot1', 'robot2'):
            return False
        if not snapshot.source_session_id or snapshot.epoch <= 0:
            return False
        if len(snapshot.tasks) > self._maximum_tasks_per_source:
            return False
        if any(
            task.source_robot_id != snapshot.source_robot_id or
            task.source_session_id != snapshot.source_session_id or
            task.source_snapshot_epoch != snapshot.epoch or
            task.source_map_revision != snapshot.map_revision
            for task in snapshot.tasks
        ):
            return False
        previous = self._versions.get(snapshot.source_robot_id)
        if previous is not None:
            if snapshot.source_session_id in self._retired_sessions.get(
                    snapshot.source_robot_id, set()):
                return False
            if previous.session_id == snapshot.source_session_id:
                if snapshot.epoch <= previous.snapshot_epoch:
                    return False
                if snapshot.map_revision < previous.map_revision:
                    return False
            else:
                self._retired_sessions.setdefault(snapshot.source_robot_id, set()).add(
                    previous.session_id,
                )
        self._versions[snapshot.source_robot_id] = SourceVersion(
            snapshot.source_session_id, snapshot.epoch, snapshot.map_revision,
        )
        return True

    def version(self, robot_id: str) -> Optional[SourceVersion]:
        """Return one source's accepted provenance state."""
        return self._versions.get(robot_id)

    def compare_revisions(self, first_robot: str, second_robot: str) -> int:
        """Compare only versions from the same source and same session."""
        if first_robot != second_robot:
            raise ValueError('cross-source map revision comparison is invalid')
        return 0


def bid_batch_valid(
        received: Received[BidBatch], now_steady_s: float,
        expected_robot_id: str, expected_session_id: str,
        expected_snapshot_epoch: int, round_id: str,
        union_hash: str, peer_snapshot_fresh: bool) -> bool:
    """Validate all protocol bindings required before a bid can participate."""
    batch = received.value
    return (
        received.fresh(now_steady_s) and peer_snapshot_fresh and
        batch.source_robot_id == expected_robot_id and
        batch.source_session_id == expected_session_id and
        batch.source_snapshot_epoch == expected_snapshot_epoch and
        batch.round_id == round_id and batch.union_hash == union_hash and
        len(batch.bids) <= 10000 and
        len({bid.canonical_task_id for bid in batch.bids}) == len(batch.bids)
    )


@dataclass
class CommittedRound:
    """Protect a committed navigation goal from delayed protocol messages."""

    decision: Optional[PairDecision] = None
    navigation_started: bool = False

    def commit(self, decision: PairDecision) -> None:
        """Record positive peer agreement before dispatch."""
        self.decision = decision
        self.navigation_started = True

    def delayed_message_can_cancel(self, round_id: str) -> bool:
        """Old or unrelated rounds never cancel a committed local goal."""
        if self.decision is None or not self.navigation_started:
            return False
        return False if round_id != self.decision.round_id else False

    def explicit_reauction_allowed(self, reason: str) -> bool:
        """Whitelist meaningful events that may invalidate committed navigation."""
        return reason in {
            'GOAL_SUCCEEDED',
            'ACCEPTED_GOAL_FAILURE',
            'VERIFIED_CANCELLATION',
            'TASK_MATERIALLY_INVALIDATED',
            'GOAL_OCCUPIED',
            'GOAL_UNREACHABLE',
            'PEER_SESSION_RESTART',
            'COMMUNICATION_TIMEOUT',
            'MAP_MATERIAL_INVALIDATION',
            'NAVIGATION_TIMEOUT',
            'OPERATOR_STOP',
        }


@dataclass
class PeerLiveness:
    """Steady-clock peer timeout and fresh-session recovery policy."""

    timeout_s: float
    last_peer_receipt_steady_s: Optional[float] = None
    timed_out_session_id: str = ''
    state: CoordinatorState = CoordinatorState.WAITING_FOR_INPUTS

    def observe(self, session_id: str, now_steady_s: float) -> CoordinatorState:
        """Accept a peer heartbeat, requiring a new session after degraded mode."""
        if (self.state == CoordinatorState.DEGRADED_SOLO and
                session_id == self.timed_out_session_id):
            return self.state
        self.last_peer_receipt_steady_s = now_steady_s
        self.timed_out_session_id = session_id
        self.state = CoordinatorState.WAITING_FOR_INPUTS
        return self.state

    def evaluate(self, now_steady_s: float) -> CoordinatorState:
        """Enter degraded solo after a bounded receiver-local timeout."""
        if self.last_peer_receipt_steady_s is None:
            return self.state
        if now_steady_s - self.last_peer_receipt_steady_s > self.timeout_s:
            self.state = CoordinatorState.DEGRADED_SOLO
        return self.state


def completion_state(inputs: CompletionInputs) -> tuple[CoordinatorState, str]:
    """Return operational completion only with fresh healthy matching evidence."""
    health = (
        inputs.tf_healthy and inputs.nav2_healthy and
        inputs.candidates_healthy and inputs.communication_healthy
    )
    if not health:
        return CoordinatorState.BLOCKED, 'required subsystem is degraded'
    fresh = inputs.both_snapshots_fresh and inputs.both_statuses_fresh
    if not fresh:
        return CoordinatorState.WAITING_FOR_INPUTS, 'fresh peer evidence missing'
    no_work = (
        not inputs.robot1_has_valid_task and
        not inputs.robot2_has_valid_task and
        not inputs.valid_pair_exists
    )
    inactive = (
        not inputs.active_assignment and
        not inputs.local_nav_goal_active and
        not inputs.peer_nav_goal_active
    )
    confirmed = (
        inputs.shared_maps_stable and inputs.peer_completion_matches and
        inputs.condition_duration_s >= inputs.confirmation_interval_s
    )
    if no_work and inactive and confirmed:
        return CoordinatorState.COMPLETE, 'matching stable operational exhaustion'
    return CoordinatorState.WAITING_FOR_INPUTS, 'completion conditions not persistent'
