"""Pure exhaustive pair scoring for exactly Robot 1 and Robot 2."""

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Callable, Iterable, Mapping, Optional, Sequence

from .canonical import bounds_iou
from .models import (
    AssignmentScore,
    AssignmentDiagnostics,
    Bid,
    BidBatch,
    CanonicalTask,
    CanonicalUnion,
    PairDecision,
    Point,
)


IDLE_TASK_ID = ''
_UNRANKED_ROUTE = (2 ** 32 - 1)

# The scoring module remains transport-independent.  The allocator supplies
# this callback with the authoritative traffic scheduler; scoring never
# reconstructs traffic geometry or priority itself.
TrafficCompatibility = Callable[[str, Optional[Bid], str, Optional[Bid]], bool]


@dataclass(frozen=True)
class AssignmentWeights:
    """Pair-utility weights, geometry scales, and physical motion references.

    ``path`` and ``heading`` remain available for the historical scoring
    modes.  ``frontier_cost_only`` deliberately does not use either weight:
    its path/heading tradeoff is expressed in seconds by the explicit motion
    references below.
    """

    gain: float = 3.0
    path: float = 1.0
    heading: float = 0.2
    nearby_goal: float = 1.5
    route_overlap: float = 3.0
    hard_failure: float = 5.0
    sensing_overlap: float = 2.0
    workload_imbalance: float = 0.35
    visible_gain_scale: float = 5.0
    path_cost_scale_m: float = 12.0
    cost_only_reference_linear_speed_mps: float = 0.13
    cost_only_reference_angular_speed_radps: float = 0.35
    nearby_goal_distance_m: float = 0.60
    route_corridor_radius_m: float = 0.16
    sensing_approach_scale_m: float = 1.5
    minimum_useful_score: float = 1e-6
    minimum_visible_gain_m: float = 0.05


def _bounded(value: float) -> float:
    return max(0.0, min(1.0, value))


def nominal_motion_cost_s(
        path_length_m: float,
        heading_cost_rad: float,
        reference_linear_speed_mps: float = 0.13,
        reference_angular_speed_radps: float = 0.35,
        ) -> float:
    """Return nominal translation plus initial reorientation time.

    This is a physically scaled frontier preference, not a Nav2 ETA and not
    an RPP trajectory simulation.  Invalid values are rejected instead of
    silently becoming an attractive candidate.
    """
    values = (
        path_length_m, heading_cost_rad, reference_linear_speed_mps,
        reference_angular_speed_radps,
    )
    if (not all(math.isfinite(float(value)) for value in values) or
            path_length_m < 0.0 or heading_cost_rad < 0.0 or
            reference_linear_speed_mps <= 0.0 or
            reference_angular_speed_radps <= 0.0):
        raise ValueError('nominal motion cost requires finite valid inputs')
    return (path_length_m / reference_linear_speed_mps +
            heading_cost_rad / reference_angular_speed_radps)


def cost_only_dispatch_certificate(
        decision: PairDecision,
        robot1_bids: BidBatch,
        robot2_bids: BidBatch,
        robot1_unqueried_bounds: Optional[Sequence[float]],
        robot2_unqueried_bounds: Optional[Sequence[float]],
        weights: AssignmentWeights,
        ) -> tuple[bool, int, float, str]:
    """Certify that unevaluated cost-only options cannot improve a decision.

    The unknown option bounds are optimistic motion costs.  Pair penalties are
    deliberately set to zero here, making every unknown combination at least
    as attractive as it could be under the real ``_score_assignment``.  A
    missing bound set is not certified.  MRTSP never calls this helper.
    """
    if robot1_unqueried_bounds is None or robot2_unqueried_bounds is None:
        return False, 0, float('-inf'), 'MISSING_UNQUERIED_LOWER_BOUNDS'

    def bid_cost(bid: Bid) -> Optional[float]:
        if not bid.path_valid:
            return None
        try:
            value = nominal_motion_cost_s(
                float(bid.path_length_m), float(bid.heading_cost),
                weights.cost_only_reference_linear_speed_mps,
                weights.cost_only_reference_angular_speed_radps,
            )
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def bounds(values: Sequence[float]) -> Optional[list[float]]:
        output = []
        for value in values:
            try:
                value = float(value)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(value) or value < 0.0:
                return None
            output.append(value)
        return output

    unknown1 = bounds(robot1_unqueried_bounds)
    unknown2 = bounds(robot2_unqueried_bounds)
    if unknown1 is None or unknown2 is None:
        return False, 0, float('-inf'), 'INVALID_UNQUERIED_LOWER_BOUNDS'

    # ``True`` marks an option supplied only by an unevaluated frontier.
    options1 = [(0.0, False)]
    options2 = [(0.0, False)]
    options1.extend((cost, False) for bid in robot1_bids.bids
                    for cost in [bid_cost(bid)] if cost is not None)
    options2.extend((cost, False) for bid in robot2_bids.bids
                    for cost in [bid_cost(bid)] if cost is not None)
    options1.extend((value, True) for value in unknown1)
    options2.extend((value, True) for value in unknown2)

    best_unknown_score = float('-inf')
    blocking_count = 0
    for cost1, unknown_flag1 in options1:
        for cost2, unknown_flag2 in options2:
            if not (unknown_flag1 or unknown_flag2):
                continue
            if cost1 == 0.0 and cost2 == 0.0:
                continue
            optimistic_score = -(cost1 + cost2)
            best_unknown_score = max(best_unknown_score, optimistic_score)
            if optimistic_score >= decision.score.total - 1e-9:
                if unknown_flag1:
                    blocking_count += 1
                if unknown_flag2:
                    blocking_count += 1

    if best_unknown_score == float('-inf'):
        return True, 0, best_unknown_score, 'NO_UNQUERIED_OPTIONS'
    if best_unknown_score >= decision.score.total - 1e-9:
        return (False, blocking_count, best_unknown_score,
                'UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT')
    return True, 0, best_unknown_score, 'ALL_UNQUERIED_OPTIONS_DOMINATED'


def _hash(payload: object) -> str:
    data = json.dumps(
        payload, sort_keys=True, separators=(',', ':'), allow_nan=False,
    ).encode('utf-8')
    return hashlib.sha256(data).hexdigest()


def bid_fingerprint(batch: BidBatch) -> str:
    """Fingerprint semantic bid data independent of message arrival order."""
    def safe_milli(value: float):
        return round(value * 1000.0) if math.isfinite(value) else None

    def safe_path(path):
        if not all(
                math.isfinite(float(x)) and math.isfinite(float(y))
                for x, y in path):
            return None
        return [(round(x * 100.0), round(y * 100.0)) for x, y in path]

    payload = []
    for bid in sorted(batch.bids, key=lambda item: item.canonical_task_id):
        payload.append({
            'id': bid.canonical_task_id,
            'valid': bid.path_valid,
            'length_mm': safe_milli(bid.path_length_m),
            'travel_milli': safe_milli(bid.estimated_travel_cost),
            'heading_milli': safe_milli(bid.heading_cost),
            'path_cm': safe_path(bid.path),
        })
    return _hash({
        'round': batch.round_id,
        'union': batch.union_hash,
        'robot': batch.source_robot_id,
        'session': batch.source_session_id,
        'epoch': batch.source_snapshot_epoch,
        'bids': payload,
    })


def _path_overlap_one_way(
        first: Sequence[Point], second: Sequence[Point], radius: float) -> float:
    if not first or not second:
        return 0.0
    return sum(
        any(math.dist(point, other) <= 2.0 * radius for other in second)
        for point in first
    ) / len(first)


def route_overlap(
        first: Sequence[Point], second: Sequence[Point], corridor_radius_m: float) -> float:
    """Return bounded geometric corridor overlap, without traffic timing claims."""
    if not first or not second:
        return 0.0
    return _bounded(0.5 * (
        _path_overlap_one_way(first, second, corridor_radius_m) +
        _path_overlap_one_way(second, first, corridor_radius_m)
    ))


def rank_solo_tasks(
        tasks: Sequence[object],
        route_history: Sequence[Sequence[Point]] = (),
        weights: AssignmentWeights = AssignmentWeights(),
        scoring_mode: str = 'legacy_weighted',
        ) -> tuple[object, ...]:
    """Order local tasks by policy, with bounded route reuse.

    The historical modes retain their generator-score ordering and bounded
    route reuse.  ``frontier_cost_only`` uses only candidate-local actual path
    length and initial path-heading mismatch using the physical nominal motion
    cost; gain is absent from both the score and all tie-breaks.
    """
    if not tasks:
        return ()
    if scoring_mode == 'frontier_cost_only':
        def value(task, name, default=0.0):
            try:
                result = float(getattr(task, name, default))
            except (TypeError, ValueError):
                return 0.0
            return result if math.isfinite(result) else 0.0

        def key(task):
            path_length = value(task, 'local_path_length_m')
            heading = value(task, 'path_heading_cost_rad')
            motion_cost = nominal_motion_cost_s(
                path_length, heading,
                weights.cost_only_reference_linear_speed_mps,
                weights.cost_only_reference_angular_speed_radps,
            )
            return (
                motion_cost,
                path_length,
                heading,
                int(getattr(task, 'generation_ros_ns', 0)),
                str(getattr(task, 'physical_signature', '')),
            )

        return tuple(sorted(tasks, key=key))

    overlaps = {}
    for task in tasks:
        path = tuple(getattr(task, 'local_path', ()) or ())
        overlaps[id(task)] = max(
            (route_overlap(path, prior, weights.route_corridor_radius_m)
             for prior in route_history),
            default=0.0,
        )
    # If every candidate substantially reuses the travelled route, suppressing
    # all of them would incorrectly ban necessary transit through a corridor.
    novel_available = any(value < 0.95 for value in overlaps.values())
    route_scale = weights.route_overlap / max(
        weights.route_overlap + weights.gain + weights.path, 1e-9)

    def key(task: object) -> tuple:
        score = float(getattr(task, 'local_ordering_score', 0.0))
        gain = float(getattr(task, 'visible_reveal_gain', 0.0))
        path_length = float(getattr(task, 'local_path_length_m', 0.0))
        overlap = overlaps[id(task)]
        effective = score - (route_scale * overlap if novel_available else 0.0)
        return (
            -effective,
            -score,
            -gain,
            path_length,
            int(getattr(task, 'generation_ros_ns', 0)),
            str(getattr(task, 'physical_signature', '')),
        )

    return tuple(sorted(tasks, key=key))


def nearby_goal_penalty(
        first: Point, second: Point, distance_scale_m: float) -> float:
    """Penalize distinct goals that occupy the same local work region."""
    if distance_scale_m <= 0.0:
        return 0.0
    return _bounded(1.0 - math.dist(first, second) / distance_scale_m)


def _visible_cell_overlap(first: Iterable[Point], second: Iterable[Point]) -> Optional[float]:
    quantum = 0.05
    first_set = {(round(x / quantum), round(y / quantum)) for x, y in first}
    second_set = {(round(x / quantum), round(y / quantum)) for x, y in second}
    if not first_set or not second_set:
        return None
    union = first_set | second_set
    return len(first_set & second_set) / len(union)


def sensing_overlap_estimate(
        first: CanonicalTask, second: CanonicalTask,
        approach_scale_m: float) -> float:
    """Use visible cells when present, otherwise a bounded geometry approximation."""
    cell_overlap = _visible_cell_overlap(first.visible_cells, second.visible_cells)
    if cell_overlap is not None:
        return _bounded(cell_overlap)
    first_bounds = first.visible_bounds or first.bounds
    second_bounds = second.visible_bounds or second.bounds
    viewpoint_overlap = _bounded(
        1.0 - math.dist(first.approach, second.approach) /
        max(approach_scale_m, 1e-9),
    )
    return _bounded(0.65 * bounds_iou(first_bounds, second_bounds) +
                    0.35 * viewpoint_overlap)


def _score_assignment(
        first_task: Optional[CanonicalTask], second_task: Optional[CanonicalTask],
        first_bid: Optional[Bid], second_bid: Optional[Bid],
        hard_failed_tasks: frozenset[str],
        weights: AssignmentWeights,
        scoring_mode: str = 'legacy_weighted',
        ) -> AssignmentScore:
    tasks = tuple(task for task in (first_task, second_task) if task is not None)
    bids = tuple(bid for bid in (first_bid, second_bid) if bid is not None)
    gain = 0.0 if scoring_mode == 'frontier_cost_only' else sum(
        _bounded(task.visible_reveal_gain / weights.visible_gain_scale)
        for task in tasks
        if math.isfinite(task.visible_reveal_gain)
    )
    generator_score = 0.0
    if scoring_mode == 'frontier_gain':
        # ``local_ordering_score`` is the exact final score produced by the
        # existing frontier candidate generator.  Prefer the member generated
        # by the robot receiving the task; peer-only tasks have one fallback
        # member and remain deterministic.
        for task, robot_id in ((first_task, 'robot1'), (second_task, 'robot2')):
            if task is None:
                continue
            source_members = tuple(
                member for member in task.members
                if member.source_robot_id == robot_id and
                math.isfinite(member.local_ordering_score)
            )
            members = source_members or tuple(
                member for member in task.members
                if math.isfinite(member.local_ordering_score)
            )
            generator_score += max(
                (member.local_ordering_score for member in members),
                default=0.0,
            )
    if scoring_mode == 'frontier_cost_only':
        # The bid carries the actual finite Nav2 path length and the initial
        # path-heading cost.  Keep the wire-compatible path/heading diagnostics
        # in their native units and use one canonical physical quantity for
        # the policy objective.
        path = sum(bid.path_length_m for bid in bids)
        heading = sum(bid.heading_cost for bid in bids)
        motion_cost = sum(
            nominal_motion_cost_s(
                bid.path_length_m, bid.heading_cost,
                weights.cost_only_reference_linear_speed_mps,
                weights.cost_only_reference_angular_speed_radps,
            )
            for bid in bids
        )
    else:
        path = sum(_bounded(bid.estimated_travel_cost / weights.path_cost_scale_m)
                   for bid in bids)
        heading = sum(_bounded(bid.heading_cost / math.pi) for bid in bids)
        motion_cost = 0.0
    proximity = overlap = sensing = imbalance = 0.0
    if first_task is not None and second_task is not None:
        proximity = nearby_goal_penalty(
            first_task.approach, second_task.approach,
            weights.nearby_goal_distance_m,
        )
        overlap = route_overlap(
            first_bid.path, second_bid.path, weights.route_corridor_radius_m,
        )
        sensing = sensing_overlap_estimate(
            first_task, second_task, weights.sensing_approach_scale_m,
        )
        imbalance = _bounded(
            abs(first_bid.path_length_m - second_bid.path_length_m) /
            weights.path_cost_scale_m,
        )
    failures = sum(task.canonical_id in hard_failed_tasks for task in tasks)
    failure_penalty = _bounded(float(failures))
    if scoring_mode == 'frontier_gain':
        # The source score already includes its normalized gain, path and
        # heading terms.  Apply only the existing pair-level separation,
        # failure, sensing, and balance terms here.
        total = (
            generator_score - weights.nearby_goal * proximity -
            weights.route_overlap * overlap -
            weights.hard_failure * failure_penalty -
            weights.sensing_overlap * sensing -
            weights.workload_imbalance * imbalance
        )
    elif scoring_mode == 'frontier_cost_only':
        # Physical nominal motion cost is the complete frontier preference.
        # Pair-level safety/separation terms remain shared cooperation
        # semantics and do not use information gain.
        total = (
            -motion_cost -
            weights.nearby_goal * proximity -
            weights.route_overlap * overlap -
            weights.hard_failure * failure_penalty -
            weights.sensing_overlap * sensing -
            weights.workload_imbalance * imbalance
        )
    else:
        total = (
            weights.gain * gain - weights.path * path -
            weights.nearby_goal * proximity -
            weights.route_overlap * overlap -
            weights.hard_failure * failure_penalty -
            weights.sensing_overlap * sensing -
            weights.workload_imbalance * imbalance
        )
    return AssignmentScore(
        team_visible_gain=gain,
        team_local_ordering_score=generator_score,
        combined_path_cost=path,
        combined_heading_cost=heading,
        nearby_goal_penalty=proximity,
        route_overlap_penalty=overlap,
        hard_failure_penalty=failure_penalty,
        sensing_overlap_penalty=sensing,
        workload_imbalance_penalty=imbalance,
        total=total,
    )


def _bid_map(batch: BidBatch) -> Mapping[str, Bid]:
    result = {}
    for bid in batch.bids:
        if bid.canonical_task_id in result:
            raise ValueError('duplicate canonical task in bid batch')
        if not math.isfinite(bid.path_length_m) or bid.path_length_m < 0.0:
            continue
        if not math.isfinite(bid.estimated_travel_cost) or bid.estimated_travel_cost < 0.0:
            continue
        if (not math.isfinite(bid.heading_cost) or bid.heading_cost < 0.0):
            continue
        if bid.path_valid and (
                not bid.path or any(
                    not (math.isfinite(float(x)) and math.isfinite(float(y)))
                    for x, y in bid.path
                )):
            continue
        result[bid.canonical_task_id] = bid
    return result


def _task_feasible(
        task: CanonicalTask, bid: Optional[Bid],
        hard_failed_tasks: frozenset[str],
        weights: AssignmentWeights,
        require_visible_gain: bool = True) -> bool:
    """Apply explicit assignment safety/utility feasibility gates.

    Soft pair utility is deliberately not part of this predicate.  A useful
    frontier may have negative absolute score after travel and overlap terms,
    but it must still be assignable when it is the best feasible work.
    """
    if bid is None or not bid.path_valid:
        return False
    if task.canonical_id in hard_failed_tasks:
        return False
    if require_visible_gain:
        if not math.isfinite(task.visible_reveal_gain):
            return False
        if task.visible_reveal_gain < weights.minimum_visible_gain_m:
            return False
    if not math.isfinite(bid.path_length_m) or bid.path_length_m < 0.0:
        return False
    if (not math.isfinite(bid.estimated_travel_cost) or
            bid.estimated_travel_cost < 0.0):
        return False
    if not math.isfinite(bid.heading_cost) or bid.heading_cost < 0.0:
        return False
    return True


def _assignment_rank(item: tuple) -> tuple:
    """Keep the deterministic pair ranking independent of selection policy."""
    return (
        -round(item[2].total, 12),
        round(item[3], 12),
        round(item[4], 12),
        item[0], item[1],
    )


def _is_conflict_free_two_active_assignment(item: tuple) -> bool:
    """Return whether a feasible pair has no cooperative-conflict signal.

    Feasibility is established before this predicate is evaluated.  Exact zero
    is intentional: the geometry helpers clamp genuinely separated work to
    0.0, while any nonzero value represents an existing soft conflict signal.
    """
    first_id, second_id, score, _, _ = item
    return bool(first_id and second_id) and (
        score.nearby_goal_penalty == 0.0 and
        score.route_overlap_penalty == 0.0 and
        score.sensing_overlap_penalty == 0.0 and
        score.hard_failure_penalty == 0.0
    )


def choose_pair_assignment(
        round_id: str, union: CanonicalUnion,
        robot1_bids: BidBatch, robot2_bids: BidBatch,
        hard_failed_tasks: frozenset[str] = frozenset(),
        weights: AssignmentWeights = AssignmentWeights(),
        scoring_mode: str = 'legacy_weighted',
        fixed_robot1_task_id: str = '',
        fixed_robot2_task_id: str = '',
        traffic_compatibility: Optional[TrafficCompatibility] = None,
        ) -> PairDecision:
    """Exhaustively evaluate bounded ordered task pairs and idle cases.

    ``legacy_weighted`` and ``frontier_gain`` remain callable for historical
    fixture compatibility.  Runtime policy selection uses only
    ``frontier_cost_only`` and ``frontier_mrtsp``; the latter is implemented
    by ``choose_mrtsp_route_assignment`` below.  Cost-only uses physically
    scaled actual path and heading terms and never information gain.
    """
    if scoring_mode not in ('legacy_weighted', 'frontier_gain',
                            'frontier_cost_only'):
        raise ValueError('unknown scoring mode: %s' % scoring_mode)
    for batch, robot_id in ((robot1_bids, 'robot1'), (robot2_bids, 'robot2')):
        if batch.round_id != round_id or batch.union_hash != union.union_hash:
            raise ValueError('bid batch does not reference the canonical round')
        if batch.source_robot_id != robot_id:
            raise ValueError('unexpected bidder identity')
    tasks = {task.canonical_id: task for task in union.tasks}
    first_map, second_map = _bid_map(robot1_bids), _bid_map(robot2_bids)
    require_gain = scoring_mode != 'frontier_cost_only'
    valid_first = {
        task_id for task_id, bid in first_map.items()
        if task_id in tasks and _task_feasible(
            tasks[task_id], bid, hard_failed_tasks, weights,
            require_visible_gain=require_gain,
        )
    }
    valid_second = {
        task_id for task_id, bid in second_map.items()
        if task_id in tasks and _task_feasible(
            tasks[task_id], bid, hard_failed_tasks, weights,
            require_visible_gain=require_gain,
        )
    }
    for fixed_id, feasible, robot_id in (
            (fixed_robot1_task_id, valid_first, 'robot1'),
            (fixed_robot2_task_id, valid_second, 'robot2')):
        if fixed_id and fixed_id not in feasible:
            raise ValueError(
                '%s fixed task is not a valid bid in this round: %s' %
                (robot_id, fixed_id))
    # A fixed active commitment is not an ordinary bid choice.  In
    # continuation mode the other robot is free specifically to obtain work;
    # keep IDLE only when it has no valid task at all.  This prevents the
    # unconstrained pair utility from parking the free robot while retaining
    # the busy task.
    choices1 = ([fixed_robot1_task_id] if fixed_robot1_task_id else
                ([IDLE_TASK_ID] if fixed_robot2_task_id and not valid_first
                 else sorted(valid_first) if fixed_robot2_task_id
                 else [IDLE_TASK_ID] + sorted(valid_first)))
    choices2 = ([fixed_robot2_task_id] if fixed_robot2_task_id else
                ([IDLE_TASK_ID] if fixed_robot1_task_id and not valid_second
                 else sorted(valid_second) if fixed_robot1_task_id
                 else [IDLE_TASK_ID] + sorted(valid_second)))
    candidates = []
    for first_id in choices1:
        for second_id in choices2:
            if first_id == second_id:
                continue
            first_task = tasks.get(first_id)
            second_task = tasks.get(second_id)
            first_bid = first_map.get(first_id)
            second_bid = second_map.get(second_id)
            score = _score_assignment(
                first_task, second_task, first_bid, second_bid,
                hard_failed_tasks, weights, scoring_mode,
            )
            combined = sum(
                bid.path_length_m for bid in (first_bid, second_bid)
                if bid is not None
            )
            maximum = max(
                (bid.path_length_m for bid in (first_bid, second_bid)
                 if bid is not None), default=0.0,
            )
            candidates.append((first_id, second_id, score, combined, maximum))
    non_idle = [item for item in candidates
                if item[0] or item[1]]
    two_active = [item for item in non_idle
                  if item[0] and item[1]]
    one_active = [item for item in non_idle
                  if bool(item[0]) ^ bool(item[1])]
    conflict_free_two_active = [
        item for item in two_active
        if _is_conflict_free_two_active_assignment(item)
    ]
    if two_active:
        # Active-assignment cardinality is the first policy tier: when both
        # robots have a valid canonical-distinct pair, IDLE cannot make a
        # one-active assignment cheaper.  Preserve the existing conflict-free
        # preference and score/tie-break ordering within the two-active tier.
        selection_pool = conflict_free_two_active or two_active
    elif one_active:
        # A one-active assignment is valid only when no valid two-active pair
        # exists.  Existing score/rank semantics remain unchanged here.
        selection_pool = one_active
    else:
        selection_pool = []
    if selection_pool:
        ordered_pool = sorted(selection_pool, key=_assignment_rank)
        selected = ordered_pool[0]
        if traffic_compatibility is not None:
            # Traffic is a feasibility filter around the existing pair score;
            # it is not a new score term.  If every policy-valid option needs
            # serialization, retain the old winner for the unchanged
            # scheduler/hold path.
            selected = next(
                (
                    item for item in ordered_pool
                    if traffic_compatibility(
                        item[0], first_map.get(item[0]),
                        item[1], second_map.get(item[1]),
                    )
                ),
                selected,
            )
    else:
        selected = (IDLE_TASK_ID, IDLE_TASK_ID, AssignmentScore(), 0.0, 0.0)
    first_id, second_id, score, combined, maximum = selected
    best_non_idle = selected if non_idle else None
    bid_task_ids = set(first_map) | set(second_map)
    hard_rejected = bid_task_ids & hard_failed_tasks
    gain_rejected = {
        task_id for task_id in bid_task_ids if task_id in tasks and
        require_gain and
        tasks[task_id].visible_reveal_gain < weights.minimum_visible_gain_m
    }
    # There is intentionally no ordinary path-length eligibility cutoff.
    # Invalid/non-finite bids are excluded by _bid_map; a valid long path is
    # still a reachable task whose distance contributes to its score.
    path_rejected = set()
    if not union.tasks:
        idle_reason = 'NO_TASKS'
    elif not bid_task_ids:
        idle_reason = 'NO_VALID_BIDS'
    elif not valid_first and not valid_second:
        if hard_rejected and hard_rejected >= bid_task_ids:
            idle_reason = 'FAILURE_SUPPRESSED'
        elif gain_rejected and gain_rejected >= bid_task_ids:
            idle_reason = 'BELOW_GAIN_THRESHOLD'
        else:
            idle_reason = 'NO_FEASIBLE_TASK'
    else:
        idle_reason = 'OTHER'
    if valid_first and valid_second:
        availability_reason = 'BOTH_ROBOTS_REACHABLE'
    elif valid_first:
        availability_reason = 'ONLY_ROBOT1_REACHABLE'
    elif valid_second:
        availability_reason = 'ONLY_ROBOT2_REACHABLE'
    else:
        availability_reason = 'NO_VALID_BIDS'
    rejected_equivalence = sum(
        max(0, len(task.members) - 1) for task in union.tasks
    )
    best_score = best_non_idle[2] if best_non_idle else AssignmentScore()
    diagnostics = AssignmentDiagnostics(
        idle_reason=idle_reason,
        availability_reason=availability_reason,
        union_task_count=len(union.tasks),
        robot1_bid_count=len(robot1_bids.bids),
        robot2_bid_count=len(robot2_bids.bids),
        robot1_valid_bid_count=len(valid_first),
        robot2_valid_bid_count=len(valid_second),
        reachable_by_both_count=len(valid_first & valid_second),
        reachable_only_robot1_count=len(valid_first - valid_second),
        reachable_only_robot2_count=len(valid_second - valid_first),
        rejected_equivalence_count=rejected_equivalence,
        rejected_failure_suppression_count=sum(
            task.canonical_id in hard_failed_tasks for task in union.tasks
        ),
        rejected_gain_threshold_count=len(gain_rejected),
        rejected_path_threshold_count=len(path_rejected),
        feasible_useful_robot1_count=len(valid_first),
        feasible_useful_robot2_count=len(valid_second),
        valid_one_active_assignment_count=sum(
            1 for item in non_idle if bool(item[0]) ^ bool(item[1])
        ),
        valid_two_active_pair_count=sum(
            1 for item in non_idle if item[0] and item[1]
        ),
        idle_idle_permitted=not bool(non_idle),
        robot1_idle_reason=(
            'NOT_IDLE' if first_id else (
                idle_reason if not second_id else
                'BEST_FEASIBLE_ASSIGNMENT_TO_ROBOT2'
            )
        ),
        robot2_idle_reason=(
            'NOT_IDLE' if second_id else (
                idle_reason if not first_id else
                'BEST_FEASIBLE_ASSIGNMENT_TO_ROBOT1'
            )
        ),
        best_non_idle_robot1_task_id=best_non_idle[0] if best_non_idle else '',
        best_non_idle_robot2_task_id=best_non_idle[1] if best_non_idle else '',
        best_non_idle_score=best_score,
        strategy=scoring_mode,
    )
    first_fingerprint = bid_fingerprint(robot1_bids)
    second_fingerprint = bid_fingerprint(robot2_bids)
    decision_payload = {
        'round_id': round_id,
        'union_hash': union.union_hash,
        'robot1_bid_fingerprint': first_fingerprint,
        'robot2_bid_fingerprint': second_fingerprint,
        'robot1_task': first_id,
        'robot2_task': second_id,
        'score_millionths': {
            name: round(getattr(score, name) * 1_000_000)
            for name in score.__dataclass_fields__
        },
    }
    return PairDecision(
        round_id=round_id,
        union_hash=union.union_hash,
        robot1_task_id=first_id,
        robot2_task_id=second_id,
        robot1_bid_fingerprint=first_fingerprint,
        robot2_bid_fingerprint=second_fingerprint,
        score=score,
        decision_hash=_hash(decision_payload),
        combined_path_length_m=combined,
        maximum_path_length_m=maximum,
        diagnostics=diagnostics,
    )


def _source_route_rank(task: CanonicalTask, robot_id: str) -> int:
    """Return the source-local upstream route rank for one robot/task pair.

    A bounded upstream DP route intentionally only ranks its selected horizon.
    Tasks outside that horizon are not silently promoted by the old local
    min-max scalar; they remain unavailable to the route-aware resolver until
    a fresh upstream route is published.
    """
    ranks = [
        int(member.mrtsp_route_rank) for member in task.members
        if member.source_robot_id == robot_id and
        str(member.mrtsp_solver).lower() == 'dp' and
        int(member.mrtsp_route_rank) < _UNRANKED_ROUTE
    ]
    return min(ranks, default=_UNRANKED_ROUTE)


def choose_mrtsp_route_assignment(
        round_id: str, union: CanonicalUnion,
        robot1_bids: BidBatch, robot2_bids: BidBatch,
        hard_failed_tasks: frozenset[str] = frozenset(),
        weights: AssignmentWeights = AssignmentWeights(),
        fixed_robot1_task_id: str = '',
        fixed_robot2_task_id: str = '',
        traffic_compatibility: Optional[TrafficCompatibility] = None,
        ) -> PairDecision:
    """Resolve two distinct tasks from two upstream DP-ordered preferences.

    This intentionally uses ordinal route evidence, not the project
    generator's batch-normalized ``local_ordering_score``.  Each peer's
    candidate generator has already run the vendored upstream MRTSP/DP solver
    over its local candidate set.  If their first tasks collide physically,
    the lower *raw Nav2 path cost* owns that shared first preference and the
    other peer advances through its existing route order.  A robot ID tie
    keeps replicas deterministic.  No central state and no new scalar team
    objective are introduced here.
    """
    for batch, robot_id in ((robot1_bids, 'robot1'), (robot2_bids, 'robot2')):
        if batch.round_id != round_id or batch.union_hash != union.union_hash:
            raise ValueError('bid batch does not reference the canonical round')
        if batch.source_robot_id != robot_id:
            raise ValueError('unexpected bidder identity')
    tasks = {task.canonical_id: task for task in union.tasks}
    first_map, second_map = _bid_map(robot1_bids), _bid_map(robot2_bids)

    def preferences(robot_id: str, bids: Mapping[str, Bid], fixed_task_id: str = ''):
        result = []
        for task_id, task in tasks.items():
            bid = bids.get(task_id)
            if (not _task_feasible(task, bid, hard_failed_tasks, weights)):
                continue
            rank = _source_route_rank(task, robot_id)
            if rank >= _UNRANKED_ROUTE and task_id != fixed_task_id:
                continue
            result.append((rank, bid.path_length_m, task_id))
        output = sorted(result, key=lambda item: (item[0], item[1], item[2]))
        if fixed_task_id:
            if not any(item[2] == fixed_task_id for item in output):
                raise ValueError(
                    'fixed %s task is not a valid bid in this round' % robot_id)
            return [item for item in output if item[2] == fixed_task_id]
        return output

    first_preferences = preferences('robot1', first_map, fixed_robot1_task_id)
    second_preferences = preferences('robot2', second_map, fixed_robot2_task_id)
    def resolve_default() -> tuple[str, str, str]:
        """Preserve the pre-existing MRTSP ordinal/duplicate resolver."""
        first_index = second_index = 0
        first_id = second_id = IDLE_TASK_ID
        duplicate_resolution = 'DISTINCT_TOP_PREFERENCES'
        while (first_index < len(first_preferences) or
               second_index < len(second_preferences)):
            candidate1 = (first_preferences[first_index]
                          if first_index < len(first_preferences) else None)
            candidate2 = (second_preferences[second_index]
                          if second_index < len(second_preferences) else None)
            if candidate1 is None:
                second_id = candidate2[2]
                duplicate_resolution = 'ROBOT1_IDLE_NO_ROUTE_CANDIDATE'
                break
            if candidate2 is None:
                first_id = candidate1[2]
                duplicate_resolution = 'ROBOT2_IDLE_NO_ROUTE_CANDIDATE'
                break
            if candidate1[2] != candidate2[2]:
                first_id, second_id = candidate1[2], candidate2[2]
                break
            # Both route plans lead with the same physical task.  Compare the
            # directly comparable raw path quantities requested by the design;
            # never compare the unrelated normalized generator scalars.
            first_cost = first_map[candidate1[2]].estimated_travel_cost
            second_cost = second_map[candidate2[2]].estimated_travel_cost
            if first_cost < second_cost - 1e-9:
                first_id = candidate1[2]
                second_index += 1
                duplicate_resolution = 'SHARED_TOP_LOWER_RAW_PATH_ROBOT1'
            elif second_cost < first_cost - 1e-9:
                second_id = candidate2[2]
                first_index += 1
                duplicate_resolution = 'SHARED_TOP_LOWER_RAW_PATH_ROBOT2'
            else:
                # Match the explicit deterministic project convention: the
                # higher numeric robot ID wins a genuine numeric tie.
                second_id = candidate2[2]
                first_index += 1
                duplicate_resolution = 'SHARED_TOP_RAW_PATH_TIE_ROBOT2'
            if first_id or second_id:
                # Preserve the winner while the other peer advances.  A later
                # candidate may still equal it, in which case the loop resolves
                # that duplicate before constructing the final pair.
                while (second_index < len(second_preferences) and second_id and
                       second_preferences[second_index][2] == second_id):
                    second_index += 1
                while (first_index < len(first_preferences) and first_id and
                       first_preferences[first_index][2] == first_id):
                    first_index += 1
                if first_id and second_id:
                    break
                if first_id and second_index < len(second_preferences):
                    second_id = second_preferences[second_index][2]
                    if second_id != first_id:
                        break
                    continue
                if second_id and first_index < len(first_preferences):
                    first_id = first_preferences[first_index][2]
                    if first_id != second_id:
                        break
                    continue
                break
        return first_id, second_id, duplicate_resolution

    first_id, second_id, duplicate_resolution = resolve_default()
    default_pair = (first_id, second_id)
    traffic_fallback = False
    if traffic_compatibility is not None:
        # Keep the current resolver's result first.  The remaining candidates
        # are ordered by the same source-local route ordinals, then by the
        # existing deterministic path/id ties.  This is a feasibility filter,
        # not a new MRTSP utility or route re-ranking.
        options = []
        first_options = list(first_preferences) or [None]
        second_options = list(second_preferences) or [None]
        for candidate1 in first_options:
            for candidate2 in second_options:
                first_candidate_id = candidate1[2] if candidate1 else IDLE_TASK_ID
                second_candidate_id = candidate2[2] if candidate2 else IDLE_TASK_ID
                if (not first_candidate_id and not second_candidate_id) or (
                        first_candidate_id and
                        first_candidate_id == second_candidate_id):
                    continue
                options.append((candidate1, candidate2))

        def option_key(option):
            candidate1, candidate2 = option
            rank1 = candidate1[0] if candidate1 else _UNRANKED_ROUTE
            rank2 = candidate2[0] if candidate2 else _UNRANKED_ROUTE
            path1 = candidate1[1] if candidate1 else math.inf
            path2 = candidate2[1] if candidate2 else math.inf
            task1 = candidate1[2] if candidate1 else IDLE_TASK_ID
            task2 = candidate2[2] if candidate2 else IDLE_TASK_ID
            return (max(rank1, rank2), rank1 + rank2,
                    rank1, rank2, path1, path2, task1, task2)

        base_option = next(
            (
                option for option in options
                if ((option[0][2] if option[0] else IDLE_TASK_ID),
                    (option[1][2] if option[1] else IDLE_TASK_ID)) ==
                default_pair
            ),
            (None, None),
        )
        ordered_options = [base_option] + [
            option for option in sorted(options, key=option_key)
            if option != base_option
        ]
        selected_option = base_option
        for option in ordered_options:
            candidate1, candidate2 = option
            candidate1_id = candidate1[2] if candidate1 else IDLE_TASK_ID
            candidate2_id = candidate2[2] if candidate2 else IDLE_TASK_ID
            if traffic_compatibility(
                    candidate1_id, first_map.get(candidate1_id),
                    candidate2_id, second_map.get(candidate2_id)):
                selected_option = option
                break
        first_id = selected_option[0][2] if selected_option[0] else IDLE_TASK_ID
        second_id = selected_option[1][2] if selected_option[1] else IDLE_TASK_ID
        traffic_fallback = (first_id, second_id) != default_pair
        if traffic_fallback:
            duplicate_resolution = 'TRAFFIC_COMPATIBLE_ROUTE_FALLBACK'

    first_task, second_task = tasks.get(first_id), tasks.get(second_id)
    first_bid, second_bid = first_map.get(first_id), second_map.get(second_id)
    selected_bids = tuple(item for item in (first_bid, second_bid) if item is not None)
    selected_tasks = tuple(item for item in (first_task, second_task) if item is not None)
    combined = sum(item.path_length_m for item in selected_bids)
    maximum = max((item.path_length_m for item in selected_bids), default=0.0)
    selected_rank_1 = _source_route_rank(first_task, 'robot1') if first_task else -1
    selected_rank_2 = _source_route_rank(second_task, 'robot2') if second_task else -1
    score = AssignmentScore(
        team_visible_gain=sum(item.visible_reveal_gain for item in selected_tasks),
        # Explicitly zero: this field documents that route mode did not use
        # legacy local_ordering_score as a distributed utility.
        team_local_ordering_score=0.0,
        combined_path_cost=combined,
        total=-float(max(selected_rank_1, selected_rank_2, 0)),
    )
    route_trace = ({
        'policy': 'UPSTREAM_DP_ORDINAL_ROUTE',
        'robot1_preferences': [
            {'rank': rank, 'path_m': path, 'task': task}
            for rank, path, task in first_preferences],
        'robot2_preferences': [
            {'rank': rank, 'path_m': path, 'task': task}
            for rank, path, task in second_preferences],
        'robot1_selected_rank': selected_rank_1,
        'robot2_selected_rank': selected_rank_2,
        'duplicate_resolution': duplicate_resolution,
        'traffic_aware_fallback': traffic_fallback,
    },)
    diagnostics = AssignmentDiagnostics(
        idle_reason=(
            'NO_UPSTREAM_ROUTE_CANDIDATES' if not selected_tasks else 'OTHER'),
        availability_reason=(
            'BOTH_ROBOTS_REACHABLE' if first_preferences and second_preferences
            else ('ONLY_ROBOT1_REACHABLE' if first_preferences
                  else ('ONLY_ROBOT2_REACHABLE' if second_preferences
                        else 'NO_VALID_BIDS'))),
        union_task_count=len(union.tasks),
        robot1_bid_count=len(robot1_bids.bids),
        robot2_bid_count=len(robot2_bids.bids),
        robot1_valid_bid_count=len(first_preferences),
        robot2_valid_bid_count=len(second_preferences),
        reachable_by_both_count=len({item[2] for item in first_preferences} &
                                    {item[2] for item in second_preferences}),
        reachable_only_robot1_count=len({item[2] for item in first_preferences} -
                                         {item[2] for item in second_preferences}),
        reachable_only_robot2_count=len({item[2] for item in second_preferences} -
                                         {item[2] for item in first_preferences}),
        rejected_equivalence_count=sum(max(0, len(task.members) - 1)
                                       for task in union.tasks),
        rejected_failure_suppression_count=sum(
            task.canonical_id in hard_failed_tasks for task in union.tasks),
        feasible_useful_robot1_count=len(first_preferences),
        feasible_useful_robot2_count=len(second_preferences),
        valid_one_active_assignment_count=int(bool(first_id)) + int(bool(second_id)),
        valid_two_active_pair_count=int(bool(first_id and second_id)),
        idle_idle_permitted=not bool(first_id or second_id),
        robot1_idle_reason='NOT_IDLE' if first_id else 'NO_ROUTE_CANDIDATE',
        robot2_idle_reason='NOT_IDLE' if second_id else 'NO_ROUTE_CANDIDATE',
        best_non_idle_robot1_task_id=first_id,
        best_non_idle_robot2_task_id=second_id,
        best_non_idle_score=score,
        strategy='frontier_mrtsp',
        burgard_trace=route_trace,
    )
    first_fingerprint, second_fingerprint = (
        bid_fingerprint(robot1_bids), bid_fingerprint(robot2_bids))
    payload = {
        'round_id': round_id, 'union_hash': union.union_hash,
        'robot1_bid_fingerprint': first_fingerprint,
        'robot2_bid_fingerprint': second_fingerprint,
        'robot1_task': first_id, 'robot2_task': second_id,
        'strategy': 'frontier_mrtsp',
        'route_ranks': [selected_rank_1, selected_rank_2],
        'duplicate_resolution': duplicate_resolution,
    }
    return PairDecision(
        round_id=round_id, union_hash=union.union_hash,
        robot1_task_id=first_id, robot2_task_id=second_id,
        robot1_bid_fingerprint=first_fingerprint,
        robot2_bid_fingerprint=second_fingerprint,
        score=score, decision_hash=_hash(payload),
        combined_path_length_m=combined, maximum_path_length_m=maximum,
        diagnostics=diagnostics,
    )


def decisions_match(first: PairDecision, second: PairDecision) -> bool:
    """Require positive agreement on every decision-binding fingerprint."""
    return (
        first.round_id == second.round_id and
        first.union_hash == second.union_hash and
        first.robot1_bid_fingerprint == second.robot1_bid_fingerprint and
        first.robot2_bid_fingerprint == second.robot2_bid_fingerprint and
        first.robot1_task_id == second.robot1_task_id and
        first.robot2_task_id == second.robot2_task_id and
        first.decision_hash == second.decision_hash
    )
