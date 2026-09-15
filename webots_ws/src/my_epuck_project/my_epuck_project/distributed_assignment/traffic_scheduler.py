"""Bounded deterministic pre-dispatch route-conflict scheduling.

This is deliberately a small ordering gate, not a replacement Nav2 controller
and not a claim of universal collision freedom in dynamic environments.
"""

from dataclasses import asdict, dataclass
import math
from typing import Sequence

from .models import Point


@dataclass(frozen=True)
class TrafficDecision:
    """Replicated geometry and priority evidence for one agreed task pair."""

    conflict: bool = False
    minimum_separation_m: float = math.inf
    required_separation_m: float = 0.0
    robot1_first_conflict_distance_m: float = 0.0
    robot2_first_conflict_distance_m: float = 0.0
    # The complete conservative interval spanned by all conflicting segment
    # pairs in each route.  The scheduler does not attempt to identify a
    # semantic doorway; this is purely path geometry.
    robot1_last_conflict_distance_m: float = 0.0
    robot2_last_conflict_distance_m: float = 0.0
    robot1_eta_s: float = 0.0
    robot2_eta_s: float = 0.0
    winner_robot_id: str = ''
    waiting_robot_id: str = ''
    reason: str = 'NO_CONFLICT'

    def as_dict(self) -> dict[str, object]:
        """Return JSON-friendly bounded telemetry."""
        result = asdict(self)
        result['minimum_separation_m'] = (
            None if not math.isfinite(self.minimum_separation_m)
            else round(self.minimum_separation_m, 12)
        )
        for key, value in tuple(result.items()):
            if isinstance(value, float):
                result[key] = round(value, 12)
        return result


def _sub(first: Point, second: Point) -> Point:
    return first[0] - second[0], first[1] - second[1]


def _dot(first: Point, second: Point) -> float:
    return first[0] * second[0] + first[1] * second[1]


def _add(first: Point, second: Point) -> Point:
    return first[0] + second[0], first[1] + second[1]


def _scale(value: Point, factor: float) -> Point:
    return value[0] * factor, value[1] * factor


def _segment_distance(
        first_start: Point, first_end: Point,
        second_start: Point, second_end: Point) -> tuple[float, float, float]:
    """Closest distance and normalized positions on two finite 2-D segments."""
    first = _sub(first_end, first_start)
    second = _sub(second_end, second_start)
    offset = _sub(first_start, second_start)
    first_sq, second_sq = _dot(first, first), _dot(second, second)
    cross = _dot(first, second)
    first_offset, second_offset = _dot(first, offset), _dot(second, offset)
    epsilon = 1e-12
    if first_sq <= epsilon and second_sq <= epsilon:
        return math.dist(first_start, second_start), 0.0, 0.0
    if first_sq <= epsilon:
        second_t = max(0.0, min(1.0, second_offset / second_sq))
        return math.dist(first_start, _add(second_start, _scale(second, second_t))), 0.0, second_t
    if second_sq <= epsilon:
        first_t = max(0.0, min(1.0, -first_offset / first_sq))
        return math.dist(_add(first_start, _scale(first, first_t)), second_start), first_t, 0.0
    denominator = first_sq * second_sq - cross * cross
    if denominator > epsilon:
        first_t = max(0.0, min(1.0, (cross * second_offset - second_sq * first_offset) / denominator))
    else:
        first_t = 0.0
    second_t = max(0.0, min(1.0, (cross * first_t + second_offset) / second_sq))
    first_t = max(0.0, min(1.0, (cross * second_t - first_offset) / first_sq))
    return math.dist(
        _add(first_start, _scale(first, first_t)),
        _add(second_start, _scale(second, second_t)),
    ), first_t, second_t


def _segments(path: Sequence[Point]) -> tuple[tuple[Point, Point, float], ...]:
    if not path:
        return ()
    if len(path) == 1:
        return ((path[0], path[0], 0.0),)
    cumulative = 0.0
    result = []
    for first, second in zip(path, path[1:]):
        result.append((first, second, cumulative))
        cumulative += math.dist(first, second)
    return tuple(result)


def detect_path_conflict(
        robot1_path: Sequence[Point], robot2_path: Sequence[Point],
        required_separation_m: float) -> tuple[bool, float, float, float]:
    """Compatibility wrapper returning the first joint conflict evidence."""
    conflict, minimum, first1, first2, _last1, _last2 = (
        detect_path_conflict_interval(
            robot1_path, robot2_path, required_separation_m,
        )
    )
    return conflict, minimum, first1, first2


def detect_path_conflict_interval(
        robot1_path: Sequence[Point], robot2_path: Sequence[Point],
        required_separation_m: float,
        ) -> tuple[bool, float, float, float, float, float]:
    """Return conservative first/last distances for a continuous path conflict.

    Every segment pair whose minimum separation is within the required center
    separation contributes its closest-point distance along each route.  The
    returned interval is the deterministic min/max envelope of those points.
    If separate conflict regions exist, the envelope intentionally covers the
    gap between them; that is conservative and avoids releasing a waiter while
    a later conflict on the same committed pair remains ahead.
    """
    if required_separation_m <= 0.0:
        raise ValueError('required traffic separation must be positive')
    first_segments, second_segments = _segments(robot1_path), _segments(robot2_path)
    if not first_segments or not second_segments:
        return False, math.inf, 0.0, 0.0, 0.0, 0.0
    minimum = math.inf
    conflicts = []
    for first_index, (first_start, first_end, first_prefix) in enumerate(first_segments):
        first_length = math.dist(first_start, first_end)
        for second_index, (second_start, second_end, second_prefix) in enumerate(second_segments):
            second_length = math.dist(second_start, second_end)
            separation, first_t, second_t = _segment_distance(
                first_start, first_end, second_start, second_end,
            )
            minimum = min(minimum, separation)
            if separation <= required_separation_m + 1e-9:
                first_distance = first_prefix + first_t * first_length
                second_distance = second_prefix + second_t * second_length
                conflicts.append((
                    round(max(first_distance, second_distance), 12),
                    round(first_distance + second_distance, 12),
                    round(first_distance, 12), round(second_distance, 12),
                    first_index, second_index,
                ))
    if not conflicts:
        return False, minimum, 0.0, 0.0, 0.0, 0.0
    first_distance = min(item[2] for item in conflicts)
    second_distance = min(item[3] for item in conflicts)
    last_distance = max(item[2] for item in conflicts)
    second_last_distance = max(item[3] for item in conflicts)
    return (
        True, minimum, first_distance, second_distance,
        last_distance, second_last_distance,
    )


def project_path_progress(
        path: Sequence[Point], point: Point) -> tuple[float, float] | None:
    """Project a point onto a sampled path.

    Returns ``(distance_along_path_m, lateral_distance_m)`` for the nearest
    point on any finite segment.  It is deliberately small and stateless: the
    traffic hold owns the committed path and calls this on each timer tick.
    """
    segments = _segments(path)
    if not segments:
        return None
    best = None
    for start, end, prefix in segments:
        delta = _sub(end, start)
        length_sq = _dot(delta, delta)
        if length_sq <= 1e-12:
            fraction = 0.0
        else:
            fraction = max(0.0, min(1.0, _dot(_sub(point, start), delta) / length_sq))
        projected = _add(start, _scale(delta, fraction))
        distance = math.dist(point, projected)
        progress = prefix + fraction * math.sqrt(length_sq)
        candidate = (round(distance, 12), round(progress, 12))
        if best is None or candidate < best:
            best = candidate
    return best[1], best[0]


def schedule_traffic(
        robot1_path: Sequence[Point], robot2_path: Sequence[Point], *,
        robot1_safe_radius_m: float, robot2_safe_radius_m: float,
        reference_speed_mps: float, eta_tie_s: float = 0.05,
        active_robots: frozenset[str] = frozenset()) -> TrafficDecision:
    """Order one new conflicting dispatch by active state, ETA, then robot ID."""
    if reference_speed_mps <= 0.0:
        raise ValueError('traffic reference speed must be positive')
    required = robot1_safe_radius_m + robot2_safe_radius_m
    (conflict, minimum, first_distance, second_distance, last_distance,
     second_last_distance) = detect_path_conflict_interval(
        robot1_path, robot2_path, required,
    )
    if not conflict:
        return TrafficDecision(
            minimum_separation_m=minimum, required_separation_m=required,
        )
    eta1, eta2 = first_distance / reference_speed_mps, second_distance / reference_speed_mps
    active = sorted(active_robots & {'robot1', 'robot2'})
    if len(active) == 1:
        winner, reason = active[0], 'ALREADY_ACTIVE'
    elif len(active) > 1:
        return TrafficDecision(
            conflict=True, minimum_separation_m=minimum,
            required_separation_m=required,
            robot1_first_conflict_distance_m=first_distance,
            robot2_first_conflict_distance_m=second_distance,
            robot1_last_conflict_distance_m=last_distance,
            robot2_last_conflict_distance_m=second_last_distance,
            robot1_eta_s=eta1, robot2_eta_s=eta2,
            reason='BOTH_ALREADY_ACTIVE_MONITOR_ONLY',
        )
    elif abs(eta1 - eta2) <= eta_tie_s:
        # Keep the replicated tie policy independent of callback arrival
        # order.  The project policy gives the higher numeric robot ID the
        # deterministic right of way, so robot2 wins a genuine ETA tie.
        winner, reason = 'robot2', 'ETA_TIE_ROBOT_ID'
    elif eta1 < eta2:
        winner, reason = 'robot1', 'LOWER_ETA'
    else:
        winner, reason = 'robot2', 'LOWER_ETA'
    loser = 'robot2' if winner == 'robot1' else 'robot1'
    return TrafficDecision(
        conflict=True, minimum_separation_m=minimum,
        required_separation_m=required,
        robot1_first_conflict_distance_m=first_distance,
        robot2_first_conflict_distance_m=second_distance,
        robot1_last_conflict_distance_m=last_distance,
        robot2_last_conflict_distance_m=second_last_distance,
        robot1_eta_s=eta1, robot2_eta_s=eta2,
        winner_robot_id=winner, waiting_robot_id=loser, reason=reason,
    )
