"""Lossless rosbag2 capture/export for observer-only navigation evidence."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path
from bisect import bisect_right


def passive_topics(robots):
    topics = []
    for robot in robots:
        topics.extend([
            f'/{robot}/navigate_to_pose/_action/feedback',
            f'/{robot}/navigate_to_pose/_action/status',
            f'/{robot}/follow_path/_action/status',
            f'/{robot}/compute_path_to_pose/_action/status',
            f'/{robot}/plan',
        ])
    return tuple(topics)


def offloaded_sensor_topics(robots):
    """Return the passive high-rate sensor topics owned by the bag path.

    These topics are deliberately limited to data that the logger uses only
    for health/cadence evidence.  The raw messages remain lossless in the
    bag; the finalizer reconstructs the existing cadence/age fields.
    """
    topics = ['/clock']
    for robot in robots:
        topics.extend([
            f'/{robot}/joint_states',
            f'/{robot}/scan_d500_fixed',
            f'/{robot}/scan_d500_slam',
            f'/{robot}/scan_d500_nav',
        ])
    return tuple(topics)


def scientific_raw_topics(robots):
    """Return the thin-recorder raw topics without deserializing them live."""
    # Keep one authoritative raw source per scientific semantic. The legacy
    # offloaded sensor set is intentionally not copied into thin mode: the
    # three scan variants and joint states are duplicate diagnostic streams,
    # while thesis replay uses odom/commands/maps/TF plus GT/contact.
    topics = ['/clock', '/tf', '/tf_static', '/rosout',
              '/cslam/unknown_pose/start_release']
    for robot in robots:
        prefix = f'/{robot}'
        topics.extend([
            f'{prefix}/odom', f'{prefix}/cmd_vel_nav', f'{prefix}/cmd_vel',
            f'{prefix}/map', f'{prefix}/shared_map',
            f'/cslam/unknown_pose/{robot}/local_map',
            f'{prefix}/frontier_candidates',
            f'{prefix}/task_snapshot', f'{prefix}/task_bids',
            f'{prefix}/pair_decision', f'{prefix}/distributed_status',
            f'{prefix}/distributed_event', f'{prefix}/exploration_failure',
            f'{prefix}/navigate_to_pose/_action/status',
            f'{prefix}/navigate_to_pose/_action/feedback',
            f'{prefix}/follow_path/_action/status',
            f'{prefix}/compute_path_to_pose/_action/status',
            f'{prefix}/plan',
            f'/cslam/unknown_pose/{robot}/exploration_status',
            f'/cslam/unknown_pose/{robot}/exploration_event',
        ])
    return tuple(dict.fromkeys(topics))


def recorded_topics(robots, include_offloaded=False,
                    include_scientific_raw=False):
    # The legacy passive set remains the default.  Thin mode supplies its own
    # complete raw contract and must not silently add the legacy diagnostic
    # action/plan streams back in as duplicate capture.
    topics = ([] if include_scientific_raw else list(passive_topics(robots)))
    if include_offloaded:
        topics.extend(offloaded_sensor_topics(robots))
    if include_scientific_raw:
        topics.extend(scientific_raw_topics(robots))
    return tuple(dict.fromkeys(topics))


def export_required_topics(robots, include_scientific_raw=False,
                           required_topics=None):
    """Resolve the strict export set without promoting optional streams."""
    if required_topics is None and include_scientific_raw:
        from .raw_evidence_contract import required_nonempty_topics
        return required_nonempty_topics(robots)
    return tuple(required_topics or ())


def write_qos_profile_overrides(output_directory: Path) -> Path:
    """Write the native rosbag2 QoS override required by Webots `/clock`."""
    output_directory = Path(output_directory)
    path = output_directory.parent / 'passive_rosbag_qos_overrides.yaml'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '/clock:\n'
        '  history: keep_last\n'
        '  depth: 1\n'
        '  reliability: best_effort\n'
        '  durability: volatile\n',
        encoding='utf-8')
    return path


def recorder_command(output_directory: Path, robots, include_offloaded=False,
                     include_scientific_raw=False):
    """Build the standard rosbag2 command and its run-local QoS file."""
    qos_path = write_qos_profile_overrides(output_directory)
    return [
        'ros2', 'bag', 'record', '--storage', 'sqlite3',
        '-o', str(output_directory),
        '--qos-profile-overrides-path', str(qos_path),
        '--include-hidden-topics',
        *recorded_topics(
            robots, include_offloaded=include_offloaded,
            include_scientific_raw=include_scientific_raw),
    ], qos_path


def start_recorder(output_directory: Path, robots, environment=None,
                   include_offloaded=False, include_scientific_raw=False):
    """Start the standard rosbag2 recorder with explicit topic coverage."""
    output_directory = Path(output_directory)
    output_directory.parent.mkdir(parents=True, exist_ok=True)
    command, _ = recorder_command(
        output_directory, robots, include_offloaded=include_offloaded,
        include_scientific_raw=include_scientific_raw)
    log_path = output_directory.parent / 'passive_rosbag_recorder.log'
    log = log_path.open('w', encoding='utf-8')
    try:
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT,
            env=environment or os.environ.copy(), start_new_session=True,
        )
    except OSError:
        log.close()
        raise
    return process, log, command, log_path


def validate_raw_bag(bag_directory: Path, robots,
                     include_offloaded=False, include_scientific_raw=False,
                     required_topics=None, condition=None,
                     unknown_initial_pose=False, semantic_artifacts=None,
                     assignment_strategy=None):
    """Validate raw bag presence without deserializing or reserializing it.

    The thin recorder keeps rosbag2 as the authoritative ROS evidence.  This
    bounded post-run check counts selected topics and verifies their types,
    without creating a second JSONL copy of every message.
    """
    import rosbag2_py

    bag_directory = Path(bag_directory)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    selected_topics = recorded_topics(
        robots, include_offloaded=include_offloaded,
        include_scientific_raw=include_scientific_raw)
    counts = {topic: 0 for topic in selected_topics}
    while reader.has_next():
        topic, _data, _timestamp = reader.read_next()
        if topic in counts:
            counts[topic] += 1
    status = raw_bag_contract_status(
        type_map, counts, selected_topics, required_topics,
        condition=condition, robots=robots,
        unknown_initial_pose=unknown_initial_pose,
        semantic_artifacts=semantic_artifacts,
        assignment_strategy=assignment_strategy)
    return {
        'schema_version': 'passive_rosbag_validation_1.0',
        'raw_bag_directory': str(bag_directory),
        'selected_topics': list(selected_topics),
        'message_counts': counts,
        'topic_types': type_map,
        'complete': status['complete'],
        # Topics marked ``if_published`` in the raw contract are requested by
        # rosbag2 but are legitimately absent when that behavior did not occur
        # in a run.  Their absence is recorded for provenance, not treated as
        # a failure of the required raw-evidence contract.
        'missing_required_types': status['missing_required_types'],
        'missing_optional_topics': status['missing_optional_topics'],
        'offloaded_topics': list(
            offloaded_sensor_topics(robots) if include_offloaded else ()),
        'scientific_raw_topics': list(
            scientific_raw_topics(robots) if include_scientific_raw else ()),
        'required_topics': list(required_topics),
        'condition': condition,
        'semantic_contract': status['semantic_contract'],
    }


def raw_bag_contract_status(type_map, counts, selected_topics,
                            required_topics, condition=None, robots=(),
                            unknown_initial_pose=False,
                            semantic_artifacts=None, assignment_strategy=None):
    """Evaluate required raw evidence without requiring optional streams.

    Contract entries marked ``if_published`` are requested when available, but
    a run is complete when every required stream is present, typed, and
    non-empty.  This keeps optional event/status streams informative without
    making a valid run fail merely because an event never occurred.
    """
    selected_topics = tuple(selected_topics or ())
    required_topics = tuple(required_topics or ())
    missing_required_types = [
        topic for topic in required_topics if topic not in type_map]
    missing_optional_topics = [
        topic for topic in selected_topics
        if topic not in required_topics and topic not in type_map]
    topic_complete = bool(
        not missing_required_types and
        all(int(counts.get(topic, 0)) > 0 for topic in required_topics))
    from .raw_evidence_contract import semantic_contract_status
    semantic = semantic_contract_status(
        counts, condition, robots=robots,
        unknown_initial_pose=unknown_initial_pose,
        semantic_artifacts=semantic_artifacts,
        assignment_strategy=assignment_strategy)
    complete = bool(topic_complete and semantic['complete'])
    return {
        'complete': complete,
        'topic_complete': topic_complete,
        'missing_required_types': missing_required_types,
        'missing_optional_topics': missing_optional_topics,
        'semantic_contract': semantic,
    }


def stop_recorder(process, log, timeout_s=30.0):
    """Stop rosbag2 and its private process group before exporting the bag.

    The recorder is started in its own session.  Signalling only the ros2 CLI
    wrapper can leave a recorder/storage child running while finalization tries
    to open the SQLite database.  Signal the private group so the recorder
    receives the shutdown request, then allow its cache flush to complete.
    This changes shutdown robustness only; it does not add mission-time work.
    """
    if process is not None and process.poll() is None:
        try:
            process_group = os.getpgid(process.pid)
            os.killpg(process_group, signal.SIGINT)
            process.wait(timeout=float(timeout_s))
        except (OSError, subprocess.TimeoutExpired):
            try:
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    if process.poll() is None:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait(timeout=3.0)
                except (OSError, subprocess.TimeoutExpired):
                    pass
    if log is not None:
        try:
            log.flush()
            log.close()
        except OSError:
            pass
    return None if process is None else process.returncode


def export_index(bag_directory: Path, output_path: Path, robots,
                 include_offloaded=False, include_scientific_raw=False,
                 required_topics=None):
    """Index every selected bag message without dropping the raw bag record."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected_topics = recorded_topics(
        robots, include_offloaded=include_offloaded,
        include_scientific_raw=include_scientific_raw)
    selected = set(selected_topics)
    counts = {topic: 0 for topic in selected}
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as stream:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic not in selected:
                continue
            type_name = type_map.get(topic)
            if not type_name:
                continue
            message = deserialize_message(data, get_message(type_name))
            row = {
                'topic': topic,
                'type': type_name,
                'bag_time_ns': int(timestamp),
            }
            header = getattr(message, 'header', None)
            if header is not None:
                row['header_stamp_s'] = (
                    int(header.stamp.sec) +
                    int(header.stamp.nanosec) * 1.0e-9)
                row['frame_id'] = str(getattr(header, 'frame_id', ''))
            if hasattr(message, 'clock'):
                row['clock_s'] = (
                    int(message.clock.sec) +
                    int(message.clock.nanosec) * 1.0e-9)
            if hasattr(message, 'ranges') and hasattr(message, 'angle_min'):
                row['message_kind'] = 'laser_scan'
                row['range_count'] = len(message.ranges)
            elif hasattr(message, 'name') and hasattr(message, 'position'):
                row['message_kind'] = 'joint_state'
                row['name_count'] = len(message.name)
                row['position_count'] = len(message.position)
                row['velocity_count'] = len(message.velocity)
                row['effort_count'] = len(message.effort)
            if hasattr(message, 'feedback'):
                feedback = message.feedback
                pose = getattr(feedback, 'current_pose', None)
                row['feedback'] = {
                    'distance_remaining': float(
                        getattr(feedback, 'distance_remaining', 0.0)),
                    'number_of_recoveries': int(
                        getattr(feedback, 'number_of_recoveries', 0)),
                    'frame_id': str(getattr(getattr(pose, 'header', None),
                                            'frame_id', '')),
                }
            elif hasattr(message, 'status_list'):
                row['statuses'] = [
                    {'status': int(item.status),
                     'goal_id': bytes(item.goal_info.goal_id.uuid).hex()}
                    for item in message.status_list
                ]
            elif hasattr(message, 'poses'):
                row['path'] = {
                    'frame_id': str(message.header.frame_id),
                    'pose_count': len(message.poses),
                    'points': [[float(item.pose.position.x),
                                float(item.pose.position.y)]
                               for item in message.poses],
                }
            stream.write(json.dumps(row, sort_keys=True,
                                    separators=(',', ':')) + '\n')
            counts[topic] += 1
    # ``scientific_raw_topics()`` intentionally includes streams marked
    # ``if_published`` by the raw contract.  rosbag2 may omit the type entry
    # entirely when such a stream never published in this run.  Export
    # completeness must therefore use the contract's required non-empty set,
    # not every requested topic.  Otherwise a valid zero-event run is rejected
    # before the semantic/finalization checks can distinguish zero from
    # missing evidence.
    required_topics = export_required_topics(
        robots, include_scientific_raw=include_scientific_raw,
        required_topics=required_topics)
    type_presence_ok = all(topic in type_map for topic in required_topics)
    metadata = {
        'schema_version': 'passive_rosbag_index_1.0',
        'raw_bag_directory': str(bag_directory),
        'selected_topics': list(selected_topics),
        'message_counts': counts,
        'complete': bool(counts) and type_presence_ok and all(
            int(counts.get(topic, 0)) > 0
                  for topic in required_topics),
        'offloaded_topics': list(
            offloaded_sensor_topics(robots) if include_offloaded else ()),
        'scientific_raw_topics': list(
            scientific_raw_topics(robots) if include_scientific_raw else ()),
        'required_topics': list(required_topics),
    }
    output_path.with_suffix('.json').write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    return metadata


def export_index_with_bounded_retry(bag_directory: Path, output_path: Path,
                                    robots, include_offloaded=False,
                                    include_scientific_raw=False,
                                    required_topics=None):
    """Export with bounded backoff for transient rosbag2 conversion failures.

    rosbag2 may return from the recorder process while its final storage flush
    is still becoming readable.  Four bounded attempts with short exponential
    backoff let that flush settle without hiding any other exporter error or
    dropping a raw bag message.  This is post-run finalization only and does
    not change the observer's mission-time workload.
    """
    for attempt in range(4):
        try:
            return export_index(
                bag_directory, output_path, robots,
                include_offloaded=include_offloaded,
                include_scientific_raw=include_scientific_raw,
                required_topics=required_topics)
        except TypeError as exc:
            transient = (
                'Unable to convert function return value to a Python type'
                in str(exc))
            if attempt >= 3 or not transient:
                raise
            time.sleep(0.5 * (2 ** attempt))
    raise AssertionError('bounded rosbag export retry did not return')


def load_offloaded_timing(export_path: Path):
    """Load deterministic receive-time indices from the exported bag rows.

    The bag timestamp is mapped to the nearest preceding recorded `/clock`
    sample.  This preserves the observer's existing simulation-time age/rate
    semantics without replaying messages through rclpy.
    """
    rows = []
    with Path(export_path).open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    clocks = sorted(
        (int(row['bag_time_ns']), float(row['clock_s']))
        for row in rows if 'clock_s' in row)
    clock_times = [item[0] for item in clocks]
    by_topic = {}
    for row in rows:
        topic = str(row.get('topic', ''))
        if topic == '/clock':
            continue
        index = bisect_right(clock_times, int(row['bag_time_ns'])) - 1
        received_sim_s = (
            clocks[index][1] if index >= 0 else None)
        if received_sim_s is None:
            continue
        by_topic.setdefault(topic, []).append({
            'bag_time_ns': int(row['bag_time_ns']),
            # Headerless streams use their mapped receipt time for health
            # reconstruction. Headered streams retain their source stamp.
            'header_stamp_s': float(
                row.get('header_stamp_s', received_sim_s)),
            'received_sim_s': received_sim_s,
        })
    for values in by_topic.values():
        values.sort(key=lambda item: (item['bag_time_ns'],
                                      item['header_stamp_s']))
    return by_topic


def semantic_export_complete(metadata, include_offloaded=False,
                             required_topics=None):
    """Return true only when raw export and derived replay both completed."""
    if not isinstance(metadata, dict):
        return False
    if metadata.get('complete') is not True:
        return False
    if metadata.get('recorder_return_code') != 0:
        return False
    if include_offloaded or required_topics:
        counts = metadata.get('message_counts')
        required_topics = tuple(required_topics or
                                metadata.get('required_topics') or
                                metadata.get('offloaded_topics') or ())
        if not isinstance(counts, dict) or any(
                int(counts.get(topic, 0)) <= 0
                for topic in required_topics):
            return False
        reconstruction = metadata.get('offloaded_reconstruction')
        if not isinstance(reconstruction, dict) or reconstruction.get(
                'complete') is not True:
            return False
    return True
