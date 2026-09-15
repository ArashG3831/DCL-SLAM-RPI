"""Deferred replay for the legacy pair-decision accounting family.

The live logger and this replay deliberately share ``pair_decision_outcome``;
the replay does not define a second classification rule.
"""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
import re

from .experiment_metrics import (
    WarningDeduplicator, rosout_diagnostic_category, warning_category)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def pair_decision_outcome(message):
    """Return the legacy deterministic outcome and parsed diagnostics."""
    try:
        diagnostics = json.loads(message.diagnostics_json or '{}')
    except (TypeError, ValueError):
        diagnostics = {}
    if not message.robot1_canonical_task_id and not message.robot2_canonical_task_id:
        if not diagnostics.get('union_task_count', 0):
            outcome = 'NO_CANONICAL_TASKS'
        elif diagnostics.get('rejected_failure_suppression_count', 0):
            outcome = 'TASK_SUPPRESSED_BY_FAILURE_MEMORY'
        elif diagnostics.get('rejected_path_threshold_count', 0):
            outcome = 'TASKS_OUT_OF_RANGE'
        elif diagnostics.get('robot1_valid_bid_count', 0) == 0 and \
                diagnostics.get('robot2_valid_bid_count', 0) == 0:
            outcome = 'NO_REACHABLE_TASK'
        else:
            outcome = 'IDLE_BY_DETERMINISTIC_ASSIGNMENT'
    else:
        outcome = 'DISPATCHABLE_ASSIGNMENT'
    return outcome, diagnostics


def select_pair_decision_outcomes(live, replay):
    """Select deferred outcomes only after the live parity gate passes."""
    if (replay.get('status') in ('PARITY_PASS', 'DEFERRED_AUTHORITATIVE') and
            replay.get('deferred') is not None):
        return dict(replay['deferred']), 'deferred'
    return dict(live), 'live'


def replay_pair_decisions_from_bag(bag_directory: Path, robots):
    """Replay pair-decision callbacks with legacy duplicate suppression."""
    import rosbag2_py

    bag_directory = Path(bag_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(bag_directory)
    robots = tuple(str(robot) for robot in robots)
    topics = {f'/{robot}/pair_decision': robot for robot in robots}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics
               if topic in type_map and
               type_map[topic] != 'my_epuck_interfaces/msg/PairDecision']
    if missing:
        raise ValueError(f'pair-decision topics missing or mistyped: {missing}')
    message_type = get_message('my_epuck_interfaces/msg/PairDecision')
    previous = {}
    outcomes = Counter()
    records = []
    deserialized_messages = 0
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        robot = topics.get(topic)
        if robot is None:
            continue
        message = deserialize_message(serialized, message_type)
        deserialized_messages += 1
        fingerprint = (
            message.round_id, message.union_hash, message.decision_hash)
        if previous.get(robot) == fingerprint:
            continue
        previous[robot] = fingerprint
        outcome, diagnostics = pair_decision_outcome(message)
        outcomes[outcome] += 1
        records.append({
            'robot_id': robot,
            'round_id': message.round_id,
            'union_hash': message.union_hash,
            'decision_hash': message.decision_hash,
            'decision_outcome': outcome,
            'diagnostics': diagnostics,
        })
    return {
        'round_outcomes': dict(outcomes),
        'records': records,
        'deserialized_messages': deserialized_messages,
        'robots': list(robots),
    }


def replay_agreement_counters_from_bag(bag_directory: Path, robots):
    """Replay the legacy DECISION_AGREED counter semantics from raw events."""
    import rosbag2_py

    bag_directory = Path(bag_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(bag_directory)
    robots = tuple(str(robot) for robot in robots)
    topics = {f'/{robot}/distributed_event': robot for robot in robots}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics
               if topic in type_map and
               type_map[topic] != (
                   'my_epuck_interfaces/msg/DistributedExplorationEvent')]
    if missing:
        raise ValueError(f'distributed-event topics mistyped: {missing}')
    message_type = get_message(
        'my_epuck_interfaces/msg/DistributedExplorationEvent')
    publications = 0
    rounds = set()
    decisions = set()
    deserialized_messages = 0
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        if topic not in topics:
            continue
        message = deserialize_message(serialized, message_type)
        deserialized_messages += 1
        if message.event_type != 'DECISION_AGREED':
            continue
        publications += 1
        rounds.add(message.round_id)
        if message.decision_hash:
            decisions.add(message.decision_hash)
    return {
        'agreement_publications': publications,
        'unique_agreed_rounds': len(rounds),
        'unique_agreed_decisions': len(decisions),
        'round_ids': sorted(rounds),
        'decision_hashes': sorted(decisions),
        'deserialized_messages': deserialized_messages,
        'robots': list(robots),
    }


def warning_record_semantics(records):
    """Return warning fields whose equality is independent of wall-clock time."""
    result = []
    for record in records:
        value = record if isinstance(record, dict) else record.__dict__
        result.append({key: value[key] for key in (
            'node_name', 'severity', 'representative_message',
            'normalized_message', 'occurrence_count', 'category')})
    return sorted(result, key=lambda item: (
        item['node_name'], item['severity'], item['normalized_message']))


def diagnostic_record_semantics(records):
    """Return Nav2 diagnostic fields independent of callback timing."""
    result = []
    for record in records:
        value = record if isinstance(record, dict) else record.__dict__
        result.append({key: value.get(key) for key in (
            'node', 'severity', 'category', 'message') } | {
            'source_stamp_sec': value.get(
                'source_stamp_sec', value.get('ros_time_sec')),
            'source_stamp_nanosec': value.get(
                'source_stamp_nanosec', value.get('ros_time_nanosec')),
        })
    return result


def replay_warning_records_from_receipts(receipt_path: Path):
    """Replay the legacy WarningDeduplicator from observer receipt rows."""
    receipt_path = Path(receipt_path)
    if not receipt_path.is_file():
        raise FileNotFoundError(receipt_path)
    deduplicator = WarningDeduplicator()
    receipt_count = 0
    warning_receipt_count = 0
    with receipt_path.open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                level = int(row['level'])
                name = str(row['name'])
                message = str(row['message'])
                wall_time = str(row.get(
                    'warning_wall_time_utc', row['wall_time_utc']))
            except (TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
                raise ValueError(
                    f'invalid rosout receipt at line {line_number}: {exc}') from exc
            receipt_count += 1
            if level < 30:
                continue
            warning_receipt_count += 1
            severity = 'ERROR' if level >= 40 else 'WARN'
            category = warning_category(f'{name} {message}')
            deduplicator.add(name, severity, message, wall_time, category)
    records = [record.__dict__.copy()
               for record in deduplicator.records.values()]
    return {
        'receipt_count': receipt_count,
        'warning_receipt_count': warning_receipt_count,
        'records': records,
        'authority_ready': True,
    }


def replay_nav2_diagnostics_from_receipts(receipt_path: Path):
    """Replay the legacy rate-limited Nav2 diagnostic stream."""
    receipt_path = Path(receipt_path)
    if not receipt_path.is_file():
        raise FileNotFoundError(receipt_path)
    records = []
    previous = {}
    receipt_count = 0
    with receipt_path.open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                level = int(row['level'])
                name = str(row['name'])
                message = str(row['message'])
                now = float(row['elapsed_s'])
                source_sec = int(row['source_stamp_sec'])
                source_nanosec = int(row['source_stamp_nanosec'])
            except (TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
                raise ValueError(
                    f'invalid rosout receipt at line {line_number}: {exc}') from exc
            receipt_count += 1
            category = rosout_diagnostic_category(f'{name} {message}')
            if category is None:
                continue
            key = (name, category, message)
            last = previous.get(key)
            if (last is not None and now - last < 0.25) or len(records) >= 10000:
                continue
            previous[key] = now
            records.append({
                'schema_version': row.get('schema_version', '1.1.0'),
                'run_id': row.get('run_id', ''),
                'event_sequence': row.get('diagnostic_event_sequence'),
                'wall_time_utc': row.get(
                    'diagnostic_wall_time_utc', row.get('wall_time_utc')),
                'ros_time_sec': row.get(
                    'diagnostic_ros_time_sec', source_sec),
                'ros_time_nanosec': row.get(
                    'diagnostic_ros_time_nanosec', source_nanosec),
                'elapsed_s': row.get('diagnostic_elapsed_s', now),
                'wall_elapsed_s': row.get(
                    'diagnostic_wall_elapsed_s', row.get('wall_elapsed_s')),
                'robot_id': None,
                'source': f'/rosout:{name}',
                'severity': 'ERROR' if level >= 40 else (
                    'WARN' if level >= 30 else 'INFO'),
                'category': category,
                'message': message,
                'node': name,
                'source_stamp_sec': source_sec,
                'source_stamp_nanosec': source_nanosec,
            })
    return {
        'receipt_count': receipt_count,
        'records': records,
        'authority_ready': all(
            record.get('event_sequence') is not None
            and record.get('wall_elapsed_s') is not None
            for record in records),
    }
