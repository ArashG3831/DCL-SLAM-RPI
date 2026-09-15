"""Deferred coverage replay using the legacy map and attribution primitives."""

from __future__ import annotations

import csv
import hashlib
import json
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from .experiment_metrics import CoverageAttribution, Grid, known_counts, known_world_cells


def _yaw(quaternion):
    import math
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 +
                     float(quaternion.z) ** 2))


def _digest(message):
    return hashlib.sha256(
        np.asarray(message.data, dtype=np.int8).tobytes()).hexdigest()


def _grid(message):
    return Grid(
        int(message.info.width), int(message.info.height),
        float(message.info.resolution),
        float(message.info.origin.position.x),
        float(message.info.origin.position.y),
        _yaw(message.info.origin.orientation), message.data)


def replay_coverage_from_bag(
        bag_directory: Path, receipt_path: Path, coverage_request_path: Path,
        robots, coverage_source: str, coverage_resolution: float,
        known_relative_transform, simultaneous_window_s: float):
    """Reconstruct coverage samples at the legacy timer request times.

    The native bag supplies the map payloads.  ``map_receipts.jsonl`` is only
    a compact causal identity ledger, allowing a bag payload to be joined to
    the exact latest message visible to the legacy coverage timer.  All cell,
    ownership, duplicate, and simultaneous semantics are delegated to the
    existing ``known_counts``/``known_world_cells``/``CoverageAttribution``
    implementations.
    """
    import rosbag2_py

    bag_directory = Path(bag_directory)
    receipt_path = Path(receipt_path)
    coverage_request_path = Path(coverage_request_path)
    for path in (bag_directory, receipt_path, coverage_request_path):
        if not path.exists():
            raise FileNotFoundError(path)
    robots = tuple(str(robot) for robot in robots)
    grid_type = get_message('nav_msgs/msg/OccupancyGrid')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    payloads = defaultdict(deque)
    topics = {f'/{robot}/map': (robot, 'map') for robot in robots}
    topics.update({f'/{robot}/shared_map': (robot, 'shared_map')
                   for robot in robots})
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        identity = topics.get(topic)
        if identity is None:
            continue
        message = deserialize_message(serialized, grid_type)
        payloads[identity].append((_digest(message), message))

    receipts = []
    with receipt_path.open(newline='', encoding='utf-8') as stream:
        for row_number, row in enumerate(stream, start=1):
            try:
                row = json.loads(row)
                identity = (str(row['robot_id']), str(row['map_key']))
                receipts.append({
                    'sequence': int(row['sequence']),
                    'identity': identity,
                    'received_ros_time_s': float(row['received_ros_time_s']),
                    'received_wall_elapsed_s': float(
                        row['received_wall_elapsed_s']),
                    'data_sha256': str(row['data_sha256']),
                })
            except (TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
                raise ValueError(
                    f'invalid map receipt row {row_number}') from exc
    receipts.sort(key=lambda row: row['sequence'])
    visible = defaultdict(list)
    for receipt in receipts:
        candidates = payloads[receipt['identity']]
        match = None
        for index, (digest, message) in enumerate(candidates):
            if digest == receipt['data_sha256']:
                match = index
                break
        if match is None:
            raise ValueError(
                f'map payload missing for receipt {receipt["sequence"]}: '
                f'{receipt["identity"]}')
        digest, message = candidates[match]
        del candidates[match]
        visible[receipt['identity']].append((
            receipt['received_ros_time_s'],
            receipt['received_wall_elapsed_s'], digest, message))
    for identity in visible:
        visible[identity].sort(key=lambda row: row[0])

    if coverage_request_path.suffix == '.jsonl':
        with coverage_request_path.open(encoding='utf-8') as stream:
            coverage_rows = [json.loads(line) for line in stream if line.strip()]
    else:
        with coverage_request_path.open(newline='', encoding='utf-8') as stream:
            coverage_rows = list(csv.DictReader(stream))
    if not coverage_rows:
        raise ValueError('coverage evidence is empty')

    cursors = defaultdict(int)
    latest = {}
    attribution = CoverageAttribution(float(simultaneous_window_s))
    last_attributed = {}
    transformed_cache = {}
    initial_known = None
    previous_known = None
    output_rows = []
    for row_number, row in enumerate(coverage_rows, start=2):
        try:
            query_ros = float(row['ros_time_sec']) + float(
                row['ros_time_nanosec']) * 1.0e-9
            query_wall = float(row['wall_elapsed_s'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'invalid coverage request row {row_number}') from exc
        for identity, records in visible.items():
            cursor = cursors[identity]
            while (cursor < len(records) and
                   records[cursor][0] < query_ros):
                latest[identity] = records[cursor]
                cursor += 1
            cursors[identity] = cursor

        shared = [latest.get((robot, 'shared_map')) for robot in robots]
        local = [latest.get((robot, 'map')) for robot in robots]
        chosen = local if coverage_source in ('local_map', 'local_map_union') else shared
        if not all(chosen):
            raise ValueError(
                f'missing {coverage_source} map at coverage row {row_number}')
        chosen_grids = [_grid(record[3]) for record in chosen]
        counts = [known_counts(grid) for grid in chosen_grids]
        known = [free + occupied for free, occupied, _unknown in counts]
        local_grids = [_grid(record[3]) if record else None for record in local]
        local_counts = [known_counts(grid) for grid in local_grids]
        local_known = [free + occupied for free, occupied, _unknown in
                       local_counts]
        transforms = {
            'robot1': (0.0, 0.0, 0.0),
            'robot2': tuple(float(value) for value in known_relative_transform),
        }
        if any(record is None for record in local_grids):
            raise ValueError(f'missing local map at coverage row {row_number}')
        for robot, record, grid in zip(robots, local, local_grids):
            if last_attributed.get(robot) == record[2]:
                continue
            transformed = transformed_cache.get(robot)
            if transformed is None or transformed[0] != record[2]:
                transformed = (
                    record[2], known_world_cells(
                        grid, float(coverage_resolution), transforms.get(
                            robot, (0.0, 0.0, 0.0))))
                transformed_cache[robot] = transformed
            attribution.observe(robot, transformed[1], query_wall)
            last_attributed[robot] = record[2]
        attributed = attribution.summary()
        if coverage_source == 'local_map_union':
            current = attributed['total_known_union_cells']
        else:
            current = max(known)
        if initial_known is None:
            initial_known = current
        gain = current - (previous_known if previous_known is not None else current)
        previous_known = current
        shared_equivalent = (
            all(shared) and
            (chosen_grids[0].width, chosen_grids[0].height,
             chosen_grids[0].resolution, chosen_grids[0].origin_x,
             chosen_grids[0].origin_y) ==
            (chosen_grids[1].width, chosen_grids[1].height,
             chosen_grids[1].resolution, chosen_grids[1].origin_x,
             chosen_grids[1].origin_y) and shared[0][2] == shared[1][2]
        ) if len(robots) > 1 else None
        semantic = {
            'robot1_local_known': local_known[0] if len(robots) > 0 else None,
            'robot2_local_known': local_known[1] if len(robots) > 1 else None,
            'robot1_shared_known': known[0] if len(robots) > 0 else None,
            'robot2_shared_known': known[1] if len(robots) > 1 else None,
            'shared_free_cells': counts[0][0],
            'shared_occupied_cells': counts[0][1],
            'shared_unknown_cells': counts[0][2],
            'known_area_m2': (current * float(coverage_resolution) ** 2
                              if coverage_source == 'local_map_union'
                              else known[0] * chosen_grids[0].resolution ** 2),
            'coverage_gain_cells': gain,
            'coverage_gain_since_start_cells': current - initial_known,
            'unique_first_seen_robot1_cells': attributed[
                'unique_first_seen_cells'].get('robot1', 0),
            'unique_first_seen_robot2_cells': attributed[
                'unique_first_seen_cells'].get('robot2', 0),
            'later_duplicated_by_robot1_cells': attributed[
                'later_duplicated_cells'].get('robot1', 0),
            'later_duplicated_by_robot2_cells': attributed[
                'later_duplicated_cells'].get('robot2', 0),
            'simultaneously_observed_cells': attributed[
                'simultaneously_observed_cells'],
            'total_known_union_cells': attributed['total_known_union_cells'],
            'duplicated_known_fraction': attributed['duplicated_known_fraction'],
            'shared_maps_equivalent': shared_equivalent,
        }
        output_rows.append({
            'row_number': row_number,
            'query_ros_time_s': query_ros,
            'semantic': semantic,
            'request': dict(row),
        })
    return {
        'rows': output_rows,
        'sample_count': len(output_rows),
        'final_state': {
            'initial_known': initial_known,
            'previous_known': previous_known,
            'attribution': attribution.summary(),
        },
    }
