"""Deferred shared-frame trajectory replay for the legacy observer.

This module replays the native rosbag2 stream through the same tf2 Buffer
semantics used by the live observer.  It is intentionally limited to the
shared trajectory/overlap metric family; it does not define a second metric.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TransformStamped

from .experiment_metrics import TrajectoryOverlap


def _yaw(quaternion):
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 +
                     float(quaternion.z) ** 2),
    )


def _finite_pose(x, y, yaw):
    return all(math.isfinite(float(value)) for value in (x, y, yaw))


def replay_shared_trajectory_from_bag(
        bag_directory: Path, robots, global_frame: str,
        bin_size: float = .05, exclusion_radius: float = .15):
    """Replay shared trajectory points in rosbag arrival order.

    The bag is the authoritative raw source.  TF and TF-static messages are
    inserted as they occur in the bag, then each odometry message performs the
    same zero-timeout tf2 lookup used by the live callback.  Missing transforms
    are recorded as skipped evidence, never converted into a valid point.
    """
    import rosbag2_py

    bag_directory = Path(bag_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(bag_directory)
    robots = tuple(str(robot) for robot in robots)
    topics = {f'/{robot}/odom': robot for robot in robots}
    tf_topic = '/tf'
    tf_static_topic = '/tf_static'
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    required_types = {
        tf_topic: 'tf2_msgs/msg/TFMessage',
        tf_static_topic: 'tf2_msgs/msg/TFMessage',
    }
    required_types.update({
        topic: 'nav_msgs/msg/Odometry' for topic in topics})
    missing = [topic for topic, message_type in required_types.items()
               if type_map.get(topic) != message_type]
    if missing:
        raise ValueError(f'raw trajectory topics missing or mistyped: {missing}')

    tf_message_type = get_message('tf2_msgs/msg/TFMessage')
    odometry_type = get_message('nav_msgs/msg/Odometry')
    buffer = Buffer()
    trajectory = TrajectoryOverlap(
        bin_size=float(bin_size), exclusion_radius=float(exclusion_radius))
    accepted = 0
    skipped = 0
    deserialized = 0
    errors = []
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        if topic == tf_topic or topic == tf_static_topic:
            message = deserialize_message(serialized, tf_message_type)
            deserialized += 1
            for transform in message.transforms:
                try:
                    if topic == tf_static_topic:
                        buffer.set_transform_static(transform, 'deferred')
                    else:
                        buffer.set_transform(transform, 'deferred')
                except Exception as exc:  # fail closed at the evidence boundary
                    errors.append(f'{topic}:{type(exc).__name__}:{exc}')
            continue
        robot = topics.get(topic)
        if robot is None:
            continue
        message = deserialize_message(serialized, odometry_type)
        deserialized += 1
        pose = message.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        local_yaw = _yaw(pose.orientation)
        if not _finite_pose(x, y, local_yaw):
            raise ValueError(f'non-finite odometry pose on {topic}')
        try:
            transform = buffer.lookup_transform(
                global_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            skipped += 1
            continue
        translation = transform.transform.translation
        heading = _yaw(transform.transform.rotation)
        cosine, sine = math.cos(heading), math.sin(heading)
        shared_x = float(translation.x) + cosine * x - sine * y
        shared_y = float(translation.y) + sine * x + cosine * y
        shared_yaw = (heading + local_yaw + math.pi) % (2.0 * math.pi) - math.pi
        if not _finite_pose(shared_x, shared_y, shared_yaw):
            raise ValueError(f'non-finite transformed pose on {topic}')
        trajectory.add(robot, shared_x, shared_y)
        accepted += 1
    return {
        'summary': trajectory.summary(),
        'trajectory': trajectory,
        'accepted_samples': accepted,
        'skipped_transform_samples': skipped,
        'deserialized_messages': deserialized,
        'errors': errors,
        'global_frame': str(global_frame),
        'robots': list(robots),
    }


def replay_shared_trajectory_from_forensic_capture(
        forensic_directory: Path, robots, global_frame: str,
        bin_size: float = .05, exclusion_radius: float = .15):
    """Replay the legacy logger's causal TF/odom callback stream.

    Native rosbag is the authoritative payload capture, but rosbag2 does not
    promise to preserve the cross-topic callback order observed by the legacy
    logger.  That order is observable here because the legacy metric performs
    a zero-timeout tf2 lookup from each odometry callback.  The existing raw
    forensic rows retain that causal receipt order without changing the
    message payload or metric semantics.  Replaying those rows through the
    same tf2 Buffer therefore reproduces the old live lookup boundary exactly.
    """
    forensic_directory = Path(forensic_directory)
    robots = tuple(str(robot) for robot in robots)
    raw_tf_path = forensic_directory / 'raw_tf.csv'
    if not raw_tf_path.is_file():
        raise FileNotFoundError(raw_tf_path)

    events = []
    with raw_tf_path.open(newline='', encoding='utf-8') as stream:
        for sequence, row in enumerate(csv.DictReader(stream)):
            try:
                received_wall = float(row['received_wall_elapsed_s'])
                received_ros = float(row['received_ros_time_s'])
                transform_stamp = float(row['transform_stamp'])
                values = [float(row[field]) for field in (
                    'translation_x', 'translation_y', 'translation_z',
                    'rotation_x', 'rotation_y', 'rotation_z', 'rotation_w')]
                if not all(math.isfinite(value) for value in (
                        received_wall, received_ros, transform_stamp, *values)):
                    raise ValueError('non-finite TF evidence')
                event = ('tf', received_wall, received_ros, sequence, row)
                events.append(event)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f'invalid raw TF evidence row {sequence + 2}') from exc

    for robot in robots:
        odom_path = forensic_directory / f'{robot}_odom.csv'
        if not odom_path.is_file():
            raise FileNotFoundError(odom_path)
        with odom_path.open(newline='', encoding='utf-8') as stream:
            for sequence, row in enumerate(csv.DictReader(stream)):
                try:
                    received_wall = float(row['received_wall_elapsed_s'])
                    received_ros = float(row['received_ros_time_s'])
                    header_stamp = float(row['header_stamp'])
                    x = float(row['pose_x'])
                    y = float(row['pose_y'])
                    z = float(row['orientation_z'])
                    w = float(row['orientation_w'])
                    frame_id = str(row['frame_id'])
                    if not all(math.isfinite(value) for value in (
                            received_wall, received_ros, header_stamp,
                            x, y, z, w)):
                        raise ValueError('non-finite odometry evidence')
                    events.append((
                        'odom', received_wall, received_ros, sequence,
                        (robot, row)))
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f'invalid {robot} odometry evidence row '
                        f'{sequence + 2}') from exc

    # Receipt clocks are the legacy causal ordering fields.  The source file
    # sequence is the deterministic tie-breaker available in the preserved
    # evidence; no wall-time proximity is used for transform lookup itself.
    events.sort(key=lambda event: (event[1], event[2], event[3], event[0]))
    buffer = Buffer()
    trajectory = TrajectoryOverlap(
        bin_size=float(bin_size), exclusion_radius=float(exclusion_radius))
    live_dynamic_edges = {
        (f'{robot}/map', f'{robot}/odom') for robot in robots}
    live_dynamic_edges.update(
        (f'{robot}/odom', f'{robot}/base_footprint') for robot in robots)
    accepted = 0
    skipped = 0
    for kind, _received_wall, _received_ros, _sequence, payload in events:
        if kind == 'tf':
            row = payload
            message = TransformStamped()
            message.header.frame_id = str(row['parent_frame'])
            message.child_frame_id = str(row['child_frame'])
            stamp = float(row['transform_stamp'])
            seconds = int(stamp)
            message.header.stamp.sec = seconds
            message.header.stamp.nanosec = int(round(
                (stamp - seconds) * 1.0e9))
            message.transform.translation.x = float(row['translation_x'])
            message.transform.translation.y = float(row['translation_y'])
            message.transform.translation.z = float(row['translation_z'])
            message.transform.rotation.x = float(row['rotation_x'])
            message.transform.rotation.y = float(row['rotation_y'])
            message.transform.rotation.z = float(row['rotation_z'])
            message.transform.rotation.w = float(row['rotation_w'])
            try:
                if str(row.get('static', '')).lower() == 'true':
                    buffer.set_transform_static(message, 'forensic')
                elif (message.header.frame_id, message.child_frame_id) \
                        in live_dynamic_edges:
                    buffer.set_transform(message, 'forensic')
            except Exception as exc:
                raise ValueError('invalid TF transform evidence') from exc
            continue

        robot, row = payload
        seconds = int(float(row['header_stamp']))
        nanoseconds = int(round(
            (float(row['header_stamp']) - seconds) * 1.0e9))
        odometry_time = Time(seconds=seconds, nanoseconds=nanoseconds)
        try:
            transform = buffer.lookup_transform(
                global_frame, str(row['frame_id']), odometry_time,
                timeout=Duration(seconds=0.0))
        except TransformException:
            skipped += 1
            continue
        translation = transform.transform.translation
        heading = _yaw(transform.transform.rotation)
        x = float(row['pose_x'])
        y = float(row['pose_y'])
        shared_x = float(translation.x) + math.cos(heading) * x \
            - math.sin(heading) * y
        shared_y = float(translation.y) + math.sin(heading) * x \
            + math.cos(heading) * y
        if not _finite_pose(shared_x, shared_y, heading):
            raise ValueError(f'non-finite transformed pose on {robot}')
        trajectory.add(robot, shared_x, shared_y)
        accepted += 1
    return {
        'summary': trajectory.summary(),
        'trajectory': trajectory,
        'accepted_samples': accepted,
        'skipped_transform_samples': skipped,
        'event_count': len(events),
        'source': 'forensic_callback_ordered_tf_odom',
        'global_frame': str(global_frame),
        'robots': list(robots),
    }
