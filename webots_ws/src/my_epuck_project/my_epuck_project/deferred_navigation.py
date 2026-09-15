"""Deferred replay for the legacy observer's action/path evidence.

The legacy observer only needs a small normalized view of these streams:
action-status transitions, planner-path lengths, and recovery transitions from
NavigateToPose feedback.  The native rosbag remains the authoritative raw
source; this module deliberately does not retain complete deserialized ROS
messages.
"""

from __future__ import annotations

import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


_STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}


def _stamp(value):
    stamp = getattr(getattr(value, 'header', None), 'stamp', None)
    if stamp is None:
        return 0, 0
    return int(stamp.sec), int(stamp.nanosec)


def _path_length(path):
    points = getattr(path, 'poses', ())
    if not points:
        return 0.0, True
    values = []
    for pose in points:
        position = pose.pose.position
        x, y = float(position.x), float(position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return None, False
        values.append((x, y))
    return sum(
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(values, values[1:])), True


def replay_navigation_evidence_from_bag(bag_directory: Path, robots):
    """Replay action/path streams in native rosbag storage order."""
    import rosbag2_py

    bag_directory = Path(bag_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(bag_directory)
    robots = tuple(str(robot) for robot in robots)
    topics = {}
    expected_types = {}
    for robot in robots:
        prefix = f'/{robot}'
        topics[f'{prefix}/navigate_to_pose/_action/status'] = (
            robot, 'NAVIGATE_TO_POSE', 'action_msgs/msg/GoalStatusArray')
        topics[f'{prefix}/navigate_to_pose/_action/feedback'] = (
            robot, 'NAVIGATE_TO_POSE_FEEDBACK',
            'nav2_msgs/action/NavigateToPose_FeedbackMessage')
        topics[f'{prefix}/follow_path/_action/status'] = (
            robot, 'FOLLOW_PATH', 'action_msgs/msg/GoalStatusArray')
        topics[f'{prefix}/compute_path_to_pose/_action/status'] = (
            robot, 'COMPUTE_PATH_TO_POSE', 'action_msgs/msg/GoalStatusArray')
        topics[f'{prefix}/plan'] = (
            robot, 'PLAN', 'nav_msgs/msg/Path')
    expected_types = {topic: value[2] for topic, value in topics.items()}

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    present = set(topics) & set(type_map)
    mistyped = [topic for topic in sorted(present)
                if type_map[topic] != expected_types[topic]]
    if mistyped:
        raise ValueError(f'action/path topics have wrong types: {mistyped}')
    missing = sorted(set(topics) - present)
    message_types = {
        type_name: get_message(type_name)
        for type_name in set(expected_types.values())
        if any(expected_types[topic] == type_name for topic in present)
    }
    previous_status = {}
    previous_recoveries = {}
    status_transitions = []
    plan_records = []
    recovery_transitions = []
    message_counts = {topic: 0 for topic in topics}
    deserialized_messages = 0

    while reader.has_next():
        topic, serialized, bag_timestamp = reader.read_next()
        descriptor = topics.get(topic)
        if descriptor is None:
            continue
        robot, kind, type_name = descriptor
        message = deserialize_message(serialized, message_types[type_name])
        message_counts[topic] += 1
        deserialized_messages += 1
        stamp_sec, stamp_nanosec = _stamp(message)
        if kind in ('NAVIGATE_TO_POSE', 'FOLLOW_PATH', 'COMPUTE_PATH_TO_POSE'):
            for status in message.status_list:
                goal_id = bytes(status.goal_info.goal_id.uuid).hex()
                key = (robot, kind, goal_id)
                value = int(status.status)
                if previous_status.get(key) == value:
                    continue
                previous_status[key] = value
                status_transitions.append({
                    'robot_id': robot,
                    'action': kind,
                    'goal_uuid': goal_id,
                    'status_value': value,
                    'status_name': _STATUS_NAMES.get(value, str(value)),
                    'stamp_sec': stamp_sec,
                    'stamp_nanosec': stamp_nanosec,
                    'bag_timestamp_ns': int(bag_timestamp),
                })
        elif kind == 'PLAN':
            length, valid = _path_length(message)
            plan_records.append({
                'robot_id': robot,
                'frame_id': str(getattr(
                    getattr(message, 'header', None), 'frame_id', '')),
                'stamp_sec': stamp_sec,
                'stamp_nanosec': stamp_nanosec,
                'bag_timestamp_ns': int(bag_timestamp),
                'pose_count': len(getattr(message, 'poses', ())),
                'path_length_m': length,
                'valid': valid,
            })
        elif kind == 'NAVIGATE_TO_POSE_FEEDBACK':
            feedback = message.feedback
            recoveries = int(feedback.number_of_recoveries)
            previous = previous_recoveries.get(robot)
            if previous != recoveries:
                previous_recoveries[robot] = recoveries
                pose_stamp_sec, pose_stamp_nanosec = _stamp(
                    feedback.current_pose)
                recovery_transitions.append({
                    'robot_id': robot,
                    'recoveries': recoveries,
                    'distance_remaining_m': float(feedback.distance_remaining),
                    'stamp_sec': pose_stamp_sec,
                    'stamp_nanosec': pose_stamp_nanosec,
                    'bag_timestamp_ns': int(bag_timestamp),
                })

    status_counts = {}
    for item in status_transitions:
        key = f"{item['action']}.{item['status_name']}"
        status_counts[key] = status_counts.get(key, 0) + 1
    required_status_topics = [
        f'/{robot}/navigate_to_pose/_action/status' for robot in robots]
    # A present action status channel with no messages is valid evidence that
    # NavigateToPose was not invoked during this run.  Completeness is about
    # the channel being present and correctly typed; event occurrence is a
    # separate fact and must not be collapsed into missing evidence.
    required_status_topics_present = [
        topic for topic in required_status_topics if topic in present]
    missing_required_status_topics = [
        topic for topic in required_status_topics if topic not in present]
    complete = not missing_required_status_topics
    return {
        'schema_version': 'navigation_action_replay_1.0',
        'robots': list(robots),
        'complete': complete,
        'status': 'COMPLETE' if complete else 'MISSING_NAVIGATION_STATUS',
        'missing_optional_topics': missing,
        'required_status_topics': required_status_topics,
        'required_status_topics_present': required_status_topics_present,
        'missing_required_status_topics': missing_required_status_topics,
        'navigate_to_pose_invoked': {
            robot: message_counts[
                f'/{robot}/navigate_to_pose/_action/status'] > 0
            for robot in robots
        },
        'message_counts': message_counts,
        'deserialized_messages': deserialized_messages,
        'status_counts': status_counts,
        'status_transitions': status_transitions,
        'plan_records': plan_records,
        'recovery_transitions': recovery_transitions,
    }
