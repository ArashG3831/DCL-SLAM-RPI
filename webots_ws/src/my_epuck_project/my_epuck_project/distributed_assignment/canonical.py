"""Physical task equivalence and canonical two-source union construction."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Iterable, List, Sequence, Tuple

from .models import Bounds, CanonicalTask, CanonicalUnion, PhysicalTask, Point


@dataclass(frozen=True)
class EquivalenceTolerances:
    """Documented geometry tolerances for e-puck frontier matching."""

    approach_distance_m: float = 0.25
    practical_approach_distance_m: float = 0.40
    centroid_distance_m: float = 0.60
    bounds_iou: float = 0.20
    strong_bounds_iou: float = 0.45
    geometry_sample_distance_m: float = 0.12
    geometry_overlap: float = 0.35
    bounds_gap_m: float = 0.12
    quantization_m: float = 0.05


def _stable_hash(payload: object) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(',', ':'), allow_nan=False,
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def canonical_round_id(robot1: TaskIdentity, robot2: TaskIdentity) -> str:
    """Hash Robot 1 then Robot 2 source sessions and snapshot epochs."""
    if robot1.robot_id != 'robot1' or robot2.robot_id != 'robot2':
        raise ValueError('round identities must be ordered robot1, robot2')
    return _stable_hash([
        robot1.session_id, robot1.snapshot_epoch,
        robot2.session_id, robot2.snapshot_epoch,
    ])


@dataclass(frozen=True)
class TaskIdentity:
    """Source identity fields used by canonical round hashing."""

    robot_id: str
    session_id: str
    snapshot_epoch: int


def bounds_iou(first: Bounds, second: Bounds) -> float:
    """Return bounded intersection-over-union of two task rectangles."""
    ix = max(0.0, min(first.maximum[0], second.maximum[0]) -
             max(first.minimum[0], second.minimum[0]))
    iy = max(0.0, min(first.maximum[1], second.maximum[1]) -
             max(first.minimum[1], second.minimum[1]))
    intersection = ix * iy
    area_a = max(0.0, first.maximum[0] - first.minimum[0]) * max(
        0.0, first.maximum[1] - first.minimum[1],
    )
    area_b = max(0.0, second.maximum[0] - second.minimum[0]) * max(
        0.0, second.maximum[1] - second.minimum[1],
    )
    union = area_a + area_b - intersection
    return 0.0 if union <= 0.0 else min(1.0, intersection / union)


def bounds_gap(first: Bounds, second: Bounds) -> float:
    """Return Euclidean separation between rectangles, zero on overlap."""
    dx = max(
        first.minimum[0] - second.maximum[0],
        second.minimum[0] - first.maximum[0], 0.0,
    )
    dy = max(
        first.minimum[1] - second.maximum[1],
        second.minimum[1] - first.maximum[1], 0.0,
    )
    return math.hypot(dx, dy)


def geometry_overlap(
        first: Sequence[Point], second: Sequence[Point],
        sample_distance_m: float) -> float:
    """Estimate symmetric sample overlap without depending on sample order."""
    if not first or not second:
        return 0.0

    def fraction(source: Sequence[Point], target: Sequence[Point]) -> float:
        matched = sum(
            any(math.dist(point, other) <= sample_distance_m for other in target)
            for point in source
        )
        return matched / len(source)

    return min(fraction(first, second), fraction(second, first))


def equivalent_tasks(
        first: PhysicalTask, second: PhysicalTask,
        tolerances: EquivalenceTolerances = EquivalenceTolerances()) -> bool:
    """Match practical observation tasks using several geometry signals."""
    approach_distance = math.dist(first.approach, second.approach)
    centroid_distance = math.dist(first.centroid, second.centroid)
    overlap = bounds_iou(first.bounds, second.bounds)
    samples = geometry_overlap(
        first.frontier_geometry, second.frontier_geometry,
        tolerances.geometry_sample_distance_m,
    )
    gap = bounds_gap(first.bounds, second.bounds)

    near_approach_with_shared_geometry = (
        approach_distance <= tolerances.approach_distance_m and
        (overlap >= tolerances.bounds_iou or
         samples >= tolerances.geometry_overlap or
         (centroid_distance <= tolerances.centroid_distance_m and
          gap <= tolerances.bounds_gap_m))
    )
    split_or_merged_opening = (
        overlap >= tolerances.strong_bounds_iou and
        (centroid_distance <= tolerances.centroid_distance_m or
         samples >= tolerances.geometry_overlap)
    )
    same_practical_viewpoint = (
        approach_distance <= tolerances.practical_approach_distance_m and
        samples >= tolerances.geometry_overlap and
        gap <= tolerances.bounds_gap_m
    )
    return (near_approach_with_shared_geometry or split_or_merged_opening or
            same_practical_viewpoint)


def _quantize(value: float, quantum: float) -> int:
    return int(math.floor(value / quantum + 0.5))


def _quantized_points(points: Iterable[Point], quantum: float) -> Tuple[Tuple[int, int], ...]:
    return tuple(sorted({
        (_quantize(point[0], quantum), _quantize(point[1], quantum))
        for point in points
    }))


def _make_canonical_task(
        members: Sequence[PhysicalTask],
        tolerances: EquivalenceTolerances) -> CanonicalTask:
    ordered = tuple(sorted(members, key=lambda task: (
        task.source_robot_id, task.physical_signature,
        task.local_frontier_id, task.approach, task.centroid,
    )))
    count = float(len(ordered))
    centroid = (
        sum(task.centroid[0] for task in ordered) / count,
        sum(task.centroid[1] for task in ordered) / count,
    )
    approach = (
        sum(task.approach[0] for task in ordered) / count,
        sum(task.approach[1] for task in ordered) / count,
    )
    yaw = math.atan2(
        sum(math.sin(task.approach_yaw) for task in ordered),
        sum(math.cos(task.approach_yaw) for task in ordered),
    )
    bounds = Bounds(
        minimum=(
            min(task.bounds.minimum[0] for task in ordered),
            min(task.bounds.minimum[1] for task in ordered),
        ),
        maximum=(
            max(task.bounds.maximum[0] for task in ordered),
            max(task.bounds.maximum[1] for task in ordered),
        ),
    )
    geometry = tuple(sorted({point for task in ordered
                             for point in task.frontier_geometry}))
    visible_cells = tuple(sorted({point for task in ordered
                                  for point in task.visible_cells}))
    visible_bounds_values = [task.visible_bounds for task in ordered
                             if task.visible_bounds is not None]
    visible_bounds = None
    if visible_bounds_values:
        visible_bounds = Bounds(
            minimum=(
                min(item.minimum[0] for item in visible_bounds_values),
                min(item.minimum[1] for item in visible_bounds_values),
            ),
            maximum=(
                max(item.maximum[0] for item in visible_bounds_values),
                max(item.maximum[1] for item in visible_bounds_values),
            ),
        )
    quantum = tolerances.quantization_m
    identity_payload = {
        'approaches': _quantized_points(
            (task.approach for task in ordered), quantum,
        ),
        'bounds': (
            _quantize(bounds.minimum[0], quantum),
            _quantize(bounds.minimum[1], quantum),
            _quantize(bounds.maximum[0], quantum),
            _quantize(bounds.maximum[1], quantum),
        ),
        'centroids': _quantized_points(
            (task.centroid for task in ordered), quantum,
        ),
        'geometry': _quantized_points(geometry, quantum),
    }
    return CanonicalTask(
        canonical_id=_stable_hash(identity_payload)[:24],
        members=ordered,
        centroid=centroid,
        bounds=bounds,
        approach=approach,
        approach_yaw=yaw,
        frontier_geometry=geometry,
        visible_cells=visible_cells,
        visible_bounds=visible_bounds,
        visible_reveal_gain=max(task.visible_reveal_gain for task in ordered),
    )


def build_canonical_union(
        robot1_tasks: Sequence[PhysicalTask],
        robot2_tasks: Sequence[PhysicalTask], max_union_tasks: int = 10,
        tolerances: EquivalenceTolerances = EquivalenceTolerances()) -> CanonicalUnion:
    """Cluster equivalent proposals and return deterministic bounded ordering."""
    all_tasks = list(robot1_tasks) + list(robot2_tasks)
    if len(all_tasks) > max_union_tasks:
        raise ValueError('canonical union exceeds configured resource bound')
    parent = list(range(len(all_tasks)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def join(first: int, second: int) -> None:
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parent[max(first_root, second_root)] = min(first_root, second_root)

    for first_index in range(len(all_tasks)):
        for second_index in range(first_index + 1, len(all_tasks)):
            if equivalent_tasks(
                    all_tasks[first_index], all_tasks[second_index], tolerances):
                join(first_index, second_index)

    clusters: dict[int, List[PhysicalTask]] = {}
    for index, task in enumerate(all_tasks):
        clusters.setdefault(find(index), []).append(task)
    canonical = tuple(sorted(
        (_make_canonical_task(cluster, tolerances)
         for cluster in clusters.values()),
        key=lambda task: task.canonical_id,
    ))
    union_hash = _stable_hash([task.canonical_id for task in canonical])
    return CanonicalUnion(tasks=canonical, union_hash=union_hash)


def world_from_rotated_grid_cell(
        origin: Point, origin_yaw: float, resolution: float,
        cell_x: int, cell_y: int) -> Point:
    """Convert a cell center through an occupancy-grid origin rotation."""
    local_x = (cell_x + 0.5) * resolution
    local_y = (cell_y + 0.5) * resolution
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    return (
        origin[0] + cosine * local_x - sine * local_y,
        origin[1] + sine * local_x + cosine * local_y,
    )
