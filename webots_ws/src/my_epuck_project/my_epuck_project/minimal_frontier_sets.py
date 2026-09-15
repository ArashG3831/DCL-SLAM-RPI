"""Build the minimal coordinator's current frontier union and local bids.

This module uses frontier IDs as the current task keys.
It preserves the specified context and digest semantics.
It converts only into existing assignment models.
It does not perform physical-equivalence clustering.
It does not own history, lifecycle, navigation, traffic, or selection.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Mapping

from my_epuck_interfaces.msg import FrontierCandidateArray
from .distributed_assignment.models import Bid, Bounds, CanonicalTask, CanonicalUnion, PhysicalTask
def compatible_context(r1: FrontierCandidateArray, r2: FrontierCandidateArray) -> bool:
    """Return whether the ordered arrays can form one current ID union."""
    return (
        (r1.source_robot_id, r2.source_robot_id) == ('robot1', 'robot2')
        and r1.header.frame_id == r2.header.frame_id
    )
def build_union(r1: FrontierCandidateArray, r2: FrontierCandidateArray) -> CanonicalUnion | None:
    """Build the ID union and its exact specification-defined digest."""
    if not compatible_context(r1, r2):
        return None
    groups: dict[int, list[PhysicalTask]] = {}
    for array in (r1, r2):
        ids = [int(candidate.frontier_id) for candidate in array.candidates]
        if len(ids) != len(set(ids)):
            raise ValueError('duplicate frontier_id in candidate array')
        for candidate in array.candidates:
            frontier_id = int(candidate.frontier_id)
            centroid = (float(candidate.centroid.x), float(candidate.centroid.y))
            bounds = Bounds((float(candidate.bounding_box_min.x), float(candidate.bounding_box_min.y)), (float(candidate.bounding_box_max.x), float(candidate.bounding_box_max.y)))
            approach = (float(candidate.approach_pose.pose.position.x), float(candidate.approach_pose.pose.position.y))
            path = tuple((float(point.x), float(point.y)) for point in candidate.local_path_samples)
            groups.setdefault(frontier_id, []).append(PhysicalTask(
                str(array.source_robot_id), '', int(array.candidate_generation_id), int(array.map_revision), str(frontier_id), frontier_id, centroid, bounds, approach,
                visible_reveal_gain=float(candidate.information_gain), local_ordering_score=float(candidate.score),
                local_path_valid=candidate.reachability_state == candidate.REACHABLE,
                local_path_length_m=float(candidate.path_length_m), local_path=path,
                path_heading_cost_rad=float(candidate.heading_change_rad)))
    union_ids = sorted(groups)
    payload = {
        'frame_id': r1.header.frame_id,
        'map_revision': int(r1.map_revision),
        'union_ids': union_ids,
    }
    union_hash = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()
    tasks = []
    for frontier_id in union_ids:
        members = tuple(groups[frontier_id])
        first = members[0]
        tasks.append(CanonicalTask(str(frontier_id), members, first.centroid, first.bounds, first.approach,
                                   first.approach_yaw, first.frontier_geometry, first.visible_cells,
                                   first.visible_bounds, first.visible_reveal_gain))
    return CanonicalUnion(tasks=tuple(tasks), union_hash=union_hash)
def materialize_costs(array: FrontierCandidateArray, union: CanonicalUnion) -> Mapping[str, float]:
    """Return every union cost, using infinity for unavailable local paths."""
    candidates = {}
    for candidate in array.candidates:
        frontier_id = int(candidate.frontier_id)
        if frontier_id in candidates:
            raise ValueError('duplicate frontier_id in candidate array')
        candidates[frontier_id] = candidate
    costs: dict[str, float] = {}
    for task in union.tasks:
        key = str(task.canonical_id)
        if key in costs or str(int(key)) != key:
            raise ValueError('duplicate or ambiguous canonical task ID')
        candidate = candidates.get(int(key))
        length = getattr(candidate, 'path_length_m', math.inf)
        reachable = candidate is not None and getattr(candidate, 'reachability_state', 0) == getattr(candidate, 'REACHABLE', 1)
        try:
            length = float(length)
        except (TypeError, ValueError):
            length = math.inf
        costs[key] = length if reachable and math.isfinite(length) and length >= 0.0 else math.inf
    return costs
def materialize_bids(array: FrontierCandidateArray, union: CanonicalUnion) -> tuple[Bid, ...]:
    """Return a deterministic full bid vector with finite wire placeholders."""
    costs = materialize_costs(array, union)
    candidates = {int(candidate.frontier_id): candidate for candidate in array.candidates}
    result = []
    for task in union.tasks:
        key = str(task.canonical_id)
        candidate = candidates.get(int(key))
        cost = costs[key]
        valid = math.isfinite(cost)
        heading = float(getattr(candidate, 'heading_change_rad', 0.0)) if candidate else 0.0
        path = tuple((float(point.x), float(point.y)) for point in candidate.local_path_samples) if valid and candidate else ()
        valid = valid and math.isfinite(heading) and heading >= 0.0
        if not valid:
            cost, heading, path = 0.0, 0.0, ()
        result.append(Bid(key, valid, cost, cost, heading_cost=heading, path=path))
    return tuple(result)
