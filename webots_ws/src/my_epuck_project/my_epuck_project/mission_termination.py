"""Small, deterministic mission-terminal taxonomy shared by both peers."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Mapping

from .frontier_actionability import (
    is_actionable_reachable,
)


class TerminalReason(str, Enum):
    """Stable semantic reasons written to replicated status messages."""

    NO_FRONTIERS = 'MISSION_COMPLETE_NO_FRONTIERS'
    ONLY_SMALL_FRONTIERS = 'MISSION_COMPLETE_ONLY_SMALL_FRONTIERS'
    ONLY_OUT_OF_RANGE_FRONTIERS = 'MISSION_COMPLETE_ONLY_OUT_OF_RANGE_FRONTIERS'
    NO_REACHABLE_FRONTIERS = 'MISSION_COMPLETE_NO_REACHABLE_FRONTIERS'
    NO_ACTIONABLE_FRONTIERS = 'MISSION_COMPLETE_NO_ACTIONABLE_FRONTIERS'
    NAVIGATION_INFRASTRUCTURE = 'MISSION_ABORT_NAVIGATION_INFRASTRUCTURE_FAILURE'
    PLANNER_INFRASTRUCTURE = 'MISSION_ABORT_PLANNER_INFRASTRUCTURE_FAILURE'
    TF_OR_SENSOR_STALE = 'MISSION_ABORT_TF_OR_SENSOR_STALE'
    STALLED = 'MISSION_ABORT_STALLED'
    TIMEOUT = 'MISSION_ABORT_TIMEOUT'
    INTERNAL_ERROR = 'MISSION_ABORT_INTERNAL_ERROR'


@dataclass(frozen=True)
class CandidateEvidence:
    """Bounded generator evidence needed to classify an empty task union."""

    detected: int = 0
    small: int = 0
    reachable: int = 0
    out_of_range: int = 0
    unreachable: int = 0
    planner_failures: int = 0
    unclassified: int = 0
    detected_not_queried: int = 0
    # ``terminal_small`` is deliberately separate from ``small``.  The
    # generator's normal minimum is a detection/candidate policy; this field
    # is the conservative end-of-mission significance policy and may include
    # a detected-but-not-queried region when its geometry is already tiny.
    terminal_small: int = 0
    meaningful_detected_not_queried: int = 0
    largest_relevant_frontier_m: float = 0.0
    below_minimum_gain: int = 0
    actionable_reachable: int | None = None

    @property
    def classified(self) -> int:
        """Return the number of regions with a defensible terminal class."""
        return (self.small + self.out_of_range + self.unreachable +
                self.below_minimum_gain)


@dataclass(frozen=True)
class FrontierRegionEvidence:
    """Compact geometry/status evidence for one physical frontier region."""

    physical_id: str
    size_m: float
    status: str
    visible_reveal_gain: float | None = None
    # Conservative pre-Nav2 motion-cost bound, when supplied by the
    # candidate generator for an unevaluated but policy-valid region.
    optimistic_cost_lower_bound_s: float | None = None
    # The remaining fields are diagnostic provenance from the candidate
    # generator.  They are intentionally not used by terminal classification
    # or allocation decisions.
    candidate_generation_id: int = 0
    map_revision: int = 0
    costmap_revision: int = 0
    query_count: int = 0
    cycles_seen: int = 0
    cycles_not_queried: int = 0
    last_query_ns: int = 0
    last_query_result: str = ''


def credible_planner_infrastructure_failure(
        local_nav2_healthy: bool,
        peer_nav2_healthy: bool,
        local_planner_failures: int,
        peer_planner_failures: int,
) -> bool:
    """Require planner-query evidence plus an observed unhealthy stack.

    Candidate-specific NO_VALID_PATH and similar responses are useful
    evidence that a particular frontier is not executable, but they do not
    prove that the planner infrastructure is dead.  The replicated abort
    path therefore requires both peers to have observed query failures and
    at least one peer to report unhealthy Nav2 infrastructure.
    """
    return bool(
        int(local_planner_failures) > 0 and
        int(peer_planner_failures) > 0 and
        (not bool(local_nav2_healthy) or not bool(peer_nav2_healthy))
    )


_BLOCKING_REGION_STATUSES = frozenset({
    'REACHABLE', 'PLANNER_FAILED', 'UNCLASSIFIED',
    'DETECTED', 'DETECTED_NOT_QUERIED',
})


def summarize_frontier_regions(
        regions: Iterable[FrontierRegionEvidence],
        terminal_small_frontier_length_m: float,
        minimum_visible_gain_m: float = 0.05,
) -> CandidateEvidence:
    """Summarize unique physical regions without counting peer replicas twice.

    A physical ID can be present in both replicas with different local
    classifications.  The summary is conservative: geometry uses the larger
    observed size and a blocking status wins over a terminal status.
    """
    merged: dict[str, tuple[float, set[str], list[float | None]]] = {}
    for region in regions:
        if not region.physical_id:
            continue
        size_m = max(0.0, float(region.size_m))
        current = merged.setdefault(region.physical_id, (size_m, set(), []))
        current[1].add(str(region.status or 'UNCLASSIFIED'))
        current[2].append(region.visible_reveal_gain)
        merged[region.physical_id] = (max(current[0], size_m), current[1], current[2])

    detected = small = reachable = out_of_range = unreachable = planner_failed = 0
    unclassified = detected_not_queried = meaningful_dnu = 0
    largest_relevant = 0.0
    below_minimum_gain = actionable_reachable = 0
    for size_m, statuses, gains in merged.values():
        is_terminal_small = size_m < float(terminal_small_frontier_length_m)
        if is_terminal_small:
            small += 1
        else:
            largest_relevant = max(largest_relevant, size_m)
        detected += 1
        has_planner_failure = 'PLANNER_FAILED' in statuses
        has_unclassified = 'UNCLASSIFIED' in statuses
        has_not_queried = (
            'DETECTED_NOT_QUERIED' in statuses or 'DETECTED' in statuses)
        if has_planner_failure:
            planner_failed += 1
        if has_unclassified:
            unclassified += 1
        if has_not_queried:
            detected_not_queried += 1
            if not is_terminal_small:
                meaningful_dnu += 1
        if is_terminal_small:
            # Terminal-small is a geometry-first endgame class.  A local
            # planner status for a microscopic region must not turn it back
            # into meaningful work.
            continue
        # Once a physical region carries unresolved evidence, it is not
        # counted as out-of-range/unreachable merely because the peer had a
        # different local classification for the same ID.
        if has_planner_failure or has_unclassified or has_not_queried:
            continue
        if 'REACHABLE' in statuses:
            # A reachable region is meaningful only when it is not terminal
            # small.  Tiny fragments remain visible in diagnostics but do not
            # block terminal completion.
            if not is_terminal_small:
                reachable += 1
                known_gains = [gain for gain in gains if gain is not None and
                               math.isfinite(float(gain))]
                if known_gains and not is_actionable_reachable(
                        'REACHABLE', max(known_gains), minimum_visible_gain_m):
                    below_minimum_gain += 1
                else:
                    actionable_reachable += 1
        elif 'OUT_OF_RANGE' in statuses:
            out_of_range += 1
        elif ('UNREACHABLE' in statuses or
              'UNREACHABLE_SAFE_APPROACH' in statuses):
            unreachable += 1

    # ``detected_not_queried`` remains useful for reporting all such regions;
    # terminal classification uses the meaningful subset above.
    return CandidateEvidence(
        detected=detected,
        small=small,
        reachable=reachable,
        terminal_small=small,
        out_of_range=out_of_range,
        unreachable=unreachable,
        planner_failures=planner_failed,
        unclassified=unclassified,
        detected_not_queried=detected_not_queried,
        meaningful_detected_not_queried=meaningful_dnu,
        largest_relevant_frontier_m=largest_relevant,
        below_minimum_gain=below_minimum_gain,
        actionable_reachable=actionable_reachable,
    )


def evidence_from_mapping(value: Mapping[str, object] | None) -> CandidateEvidence:
    """Decode message-like evidence without trusting missing fields."""
    value = value or {}
    return CandidateEvidence(
        detected=max(0, int(value.get('detected', 0) or 0)),
        small=max(0, int(value.get('small', 0) or 0)),
        reachable=max(0, int(value.get('reachable', 0) or 0)),
        out_of_range=max(0, int(value.get('out_of_range', 0) or 0)),
        unreachable=max(0, int(value.get('unreachable', 0) or 0)),
        planner_failures=max(0, int(value.get('planner_failures', 0) or 0)),
        unclassified=max(0, int(value.get('unclassified', 0) or 0)),
        detected_not_queried=max(0, int(value.get('detected_not_queried', 0) or 0)),
        terminal_small=max(0, int(value.get('terminal_small', 0) or 0)),
        meaningful_detected_not_queried=max(
            0, int(value.get('meaningful_detected_not_queried', 0) or 0)),
        largest_relevant_frontier_m=max(
            0.0, float(value.get('largest_relevant_frontier_m', 0.0) or 0.0)),
        below_minimum_gain=max(0, int(value.get('below_minimum_gain', 0) or 0)),
        actionable_reachable=(
            max(0, int(value['actionable_reachable']))
            if value.get('actionable_reachable') is not None else None
        ),
    )


def classify_empty_frontiers(
        first: CandidateEvidence,
        second: CandidateEvidence,
) -> TerminalReason | None:
    """Return success only for authoritative exhaustion evidence.

    ``unreachable`` is deliberately not a terminal class.  It describes the
    current planner/path result, which can change when map, TF, costmap, or
    planner evidence catches up.  Treating it as mission exhaustion made a
    transient all-unreachable view irreversible.
    """
    evidence = (first, second)
    def actionable_count(item: CandidateEvidence) -> int:
        return (item.actionable_reachable if item.actionable_reachable is not None
                else item.reachable)

    if any(item.planner_failures or item.unclassified or actionable_count(item)
           for item in evidence):
        return None
    # New region-aware evidence distinguishes harmless tiny unqueried
    # fragments from a meaningful frontier which has not been evaluated.
    if any(item.meaningful_detected_not_queried for item in evidence):
        return None
    # Preserve the old conservative behavior for callers that have no region
    # sizes at all (legacy messages/tests/diagnostic-disabled launches).
    if any(item.detected_not_queried and not item.terminal_small for item in evidence):
        return None
    detected = sum(item.detected for item in evidence)
    small = sum(item.terminal_small or item.small for item in evidence)
    out_of_range = sum(item.out_of_range for item in evidence)
    unreachable = sum(item.unreachable for item in evidence)
    # A frontier that is currently unreachable is not proof that exploration
    # is exhausted.  It may become reachable after fresh map/TF/planner
    # evidence arrives.  Keep the legacy enum for wire/diagnostic compatibility
    # but never produce it as a successful completion classification.
    if unreachable > 0:
        return None
    if detected == 0:
        return TerminalReason.NO_FRONTIERS
    below_gain = sum(item.below_minimum_gain for item in evidence)
    terminal_classified = small + out_of_range + unreachable + below_gain
    if terminal_classified == detected and below_gain > 0:
        return TerminalReason.NO_ACTIONABLE_FRONTIERS
    if terminal_classified == detected and small == detected:
        return TerminalReason.ONLY_SMALL_FRONTIERS
    if terminal_classified == detected and out_of_range > 0 and unreachable == 0:
        return TerminalReason.ONLY_OUT_OF_RANGE_FRONTIERS
    return None


def all_physical_tasks_suppressed(
        task_signatures: Iterable[Iterable[str]],
        suppressed_signatures: set[str],
) -> bool:
    """Return true only when every current task has hard evidence.

    A canonical task may contain equivalent source observations from both
    robots.  Requiring every member signature to be suppressed prevents one
    robot's failed local approach from incorrectly declaring a task globally
    unreachable when the peer still has an executable view.
    """
    groups = [
        frozenset(signature for signature in group if signature)
        for group in task_signatures
    ]
    return bool(groups) and all(group and group <= suppressed_signatures for group in groups)


def terminal_reason_is_success(reason: str) -> bool:
    """Return whether a reason represents successful map exhaustion."""
    return reason.startswith('MISSION_COMPLETE_')


def terminal_reason_is_abort(reason: str) -> bool:
    """Return whether a reason represents abnormal mission termination."""
    return reason.startswith('MISSION_ABORT_')


def matching_terminal_reason(
        local_terminal: bool, local_reason: str,
        peer_terminal: bool, peer_reason: str) -> str | None:
    """Return a terminal result only when both replicas carry the same reason."""
    if local_terminal and peer_terminal and local_reason and local_reason == peer_reason:
        return local_reason
    return None


def recommended_exit_code(reason: str) -> int:
    """Map terminal semantics to the compact runner result convention."""
    if terminal_reason_is_success(reason):
        return 0
    if terminal_reason_is_abort(reason):
        return 1
    return 2
