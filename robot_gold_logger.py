#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped, PolygonStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException


# Physical Nav2 footprint, base_link coordinates.
FOOTPRINT = [
    (0.20, 0.125),
    (0.20, -0.125),
    (-0.07, -0.125),
    (-0.07, 0.125),
]

# Diagnostic thresholds only. This node never controls the robot.
TIGHT_CLEARANCE_M = 0.08
COLLISION_MARGIN_M = 0.02
GOAL_CHANGE_DISTANCE_M = 0.15
GOAL_SPAM_CHANGES = 3
GOAL_SPAM_WINDOW_S = 10.0
NO_PROGRESS_WINDOW_S = 8.0
NO_PROGRESS_REQUIRED_M = 0.03
STUCK_REQUIRED_S = 3.0
OSCILLATION_WINDOW_S = 5.0
OSCILLATION_FLIPS = 4


STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def quaternion_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def point_inside_polygon(x: float, y: float, polygon) -> bool:
    inside = False
    count = len(polygon)
    j = count - 1

    for i in range(count):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        intersects = (
            ((yi > y) != (yj > y))
            and
            (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi)
        )

        if intersects:
            inside = not inside

        j = i

    return inside


def point_segment_distance(px, py, ax, ay, bx, by) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay

    denominator = vx * vx + vy * vy
    if denominator <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denominator))
    qx = ax + t * vx
    qy = ay + t * vy
    return math.hypot(px - qx, py - qy)


def point_polygon_distance(x: float, y: float, polygon) -> float:
    if point_inside_polygon(x, y, polygon):
        return 0.0

    best = math.inf
    for index in range(len(polygon)):
        ax, ay = polygon[index]
        bx, by = polygon[(index + 1) % len(polygon)]
        best = min(best, point_segment_distance(x, y, ax, ay, bx, by))

    return best


def polygon_metrics(points):
    if len(points) < 3:
        return math.nan, math.nan, math.nan

    area_twice = 0.0
    edges = []

    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]

        area_twice += x1 * y2 - x2 * y1
        edges.append(math.hypot(x2 - x1, y2 - y1))

    return abs(area_twice) / 2.0, min(edges), max(edges)


def finite_or_blank(value, digits=4):
    if value is None:
        return ""
    try:
        if not math.isfinite(float(value)):
            return ""
    except (TypeError, ValueError):
        return ""
    return f"{float(value):.{digits}f}"


class RobotGoldLogger(Node):
    def __init__(self, output_dir: Path):
        super().__init__("robot_gold_logger")

        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.telemetry_path = self.output_dir / "telemetry.csv"
        self.events_path = self.output_dir / "events.log"
        self.metadata_path = self.output_dir / "metadata.txt"

        self.telemetry_file = self.telemetry_path.open("w", newline="", buffering=1)
        self.events_file = self.events_path.open("w", buffering=1)

        self.csv_writer = csv.writer(self.telemetry_file)
        self.csv_writer.writerow([
            "utc_time",
            "ros_time_s",

            "map_x_m",
            "map_y_m",
            "map_yaw_deg",

            "odom_x_m",
            "odom_y_m",
            "odom_yaw_deg",

            "measured_v_mps",
            "measured_w_radps",
            "command_v_mps",
            "command_w_radps",

            "goal_x_m",
            "goal_y_m",
            "goal_frame",
            "goal_source",
            "goal_distance_m",
            "goal_heading_error_deg",

            "nav_status",
            "frontier_points",

            "plan_pose_count",
            "plan_length_m",

            "lethal_clearance_m",
            "cost50_clearance_m",
            "unknown_near_ratio",
            "tight_space",
            "collision_margin",

            "goal_changes_10s",
            "angular_sign_flips_5s",
            "stuck_seconds",
            "no_progress",
            "angular_oscillation",

            "scan_age_s",
            "odom_age_s",
            "cmd_age_s",
            "map_age_s",
            "costmap_age_s",
            "plan_age_s",

            "footprint_area_m2",
            "footprint_min_edge_m",
            "footprint_max_edge_m",
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_msg = None
        self.cmd_msg = None
        self.costmap_msg = None

        self.goal = None
        self.goal_source = ""
        self.goal_frame = ""

        self.path_pose_count = 0
        self.path_length = math.nan
        self.frontier_points = 0
        self.nav_status = "IDLE"

        self.footprint_area = math.nan
        self.footprint_min_edge = math.nan
        self.footprint_max_edge = math.nan

        self.received_at = {}

        self.goal_change_times = deque()
        self.angular_sign_history = deque()
        self.distance_history = deque()

        self.last_angular_sign = 0
        self.stuck_started_at = None
        self.event_flags = {}

        self.status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            20,
        )

        self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.cmd_callback,
            20,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            5,
        )

        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.costmap_callback,
            5,
        )

        self.create_subscription(
            NavPath,
            "/plan",
            self.path_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/frontier_explorer/selected_frontier",
            self.frontier_goal_callback,
            10,
        )

        self.create_subscription(
            MarkerArray,
            "/frontier_explorer/frontiers",
            self.frontiers_callback,
            10,
        )

        self.create_subscription(
            PolygonStamped,
            "/local_costmap/published_footprint",
            self.footprint_callback,
            10,
        )

        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.status_callback,
            self.status_qos,
        )

        self.create_timer(1.0, self.sample_once_per_second)

        self.write_metadata()
        self.event("LOGGER_START", f"output_dir={self.output_dir}")
        self.get_logger().info(
            f"Gold logger started: {self.telemetry_path}"
        )

    def monotonic_now(self):
        return time.monotonic()

    def mark_received(self, name):
        self.received_at[name] = self.monotonic_now()

    def age(self, name):
        stamp = self.received_at.get(name)
        if stamp is None:
            return math.inf
        return self.monotonic_now() - stamp

    def utc_now(self):
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

    def event(self, name, details=""):
        clean = str(details).replace("\n", " ").replace("\r", " ")
        self.events_file.write(f"{self.utc_now()},{name},{clean}\n")
        self.events_file.flush()

    def transition_event(self, name, active, active_details="", clear_details=""):
        previous = self.event_flags.get(name, False)

        if active and not previous:
            self.event(name + "_START", active_details)
        elif previous and not active:
            self.event(name + "_CLEAR", clear_details)

        self.event_flags[name] = bool(active)

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.mark_received("odom")

    def cmd_callback(self, msg):
        self.cmd_msg = msg
        self.mark_received("cmd")

        w = float(msg.twist.angular.z)
        sign = 1 if w > 0.05 else -1 if w < -0.05 else 0

        if sign != 0 and sign != self.last_angular_sign:
            now = self.monotonic_now()
            self.angular_sign_history.append((now, sign))
            self.last_angular_sign = sign

    def scan_callback(self, _msg):
        self.mark_received("scan")

    def map_callback(self, _msg):
        self.mark_received("map")

    def costmap_callback(self, msg):
        self.costmap_msg = msg
        self.mark_received("costmap")

    def set_goal(self, x, y, frame, source):
        now = self.monotonic_now()
        new_goal = (float(x), float(y))

        changed = False
        if self.goal is None:
            changed = True
        else:
            changed = math.hypot(
                new_goal[0] - self.goal[0],
                new_goal[1] - self.goal[1],
            ) >= GOAL_CHANGE_DISTANCE_M

        if changed:
            previous = self.goal
            self.goal = new_goal
            self.goal_frame = frame.lstrip("/") if frame else "map"
            self.goal_source = source
            self.distance_history.clear()
            self.goal_change_times.append(now)

            if previous is None:
                self.event(
                    "GOAL_SET",
                    f"source={source} frame={self.goal_frame} "
                    f"x={new_goal[0]:.3f} y={new_goal[1]:.3f}",
                )
            else:
                self.event(
                    "GOAL_CHANGED",
                    f"source={source} frame={self.goal_frame} "
                    f"old=({previous[0]:.3f},{previous[1]:.3f}) "
                    f"new=({new_goal[0]:.3f},{new_goal[1]:.3f})",
                )
        else:
            self.goal = new_goal
            self.goal_frame = frame.lstrip("/") if frame else self.goal_frame
            self.goal_source = source

    def path_callback(self, msg):
        self.mark_received("plan")
        self.path_pose_count = len(msg.poses)

        length = 0.0
        for index in range(1, len(msg.poses)):
            first = msg.poses[index - 1].pose.position
            second = msg.poses[index].pose.position
            length += math.hypot(second.x - first.x, second.y - first.y)

        self.path_length = length

        if msg.poses:
            endpoint = msg.poses[-1]
            frame = endpoint.header.frame_id or msg.header.frame_id or "map"
            self.set_goal(
                endpoint.pose.position.x,
                endpoint.pose.position.y,
                frame,
                "global_plan",
            )

    def frontier_goal_callback(self, msg):
        self.mark_received("selected_frontier")
        self.set_goal(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.header.frame_id or "map",
            "frontier_explorer",
        )

    def frontiers_callback(self, msg):
        self.mark_received("frontiers")
        self.frontier_points = sum(len(marker.points) for marker in msg.markers)

    def footprint_callback(self, msg):
        self.mark_received("footprint")

        points = [(point.x, point.y) for point in msg.polygon.points]
        area, edge_min, edge_max = polygon_metrics(points)

        first_valid = not math.isfinite(self.footprint_area)

        self.footprint_area = area
        self.footprint_min_edge = edge_min
        self.footprint_max_edge = edge_max

        if first_valid:
            self.event(
                "FOOTPRINT_RECEIVED",
                f"area={area:.4f}m2 min_edge={edge_min:.3f}m "
                f"max_edge={edge_max:.3f}m points={len(points)}",
            )

    def status_callback(self, msg):
        self.mark_received("nav_status")

        if not msg.status_list:
            status_name = "IDLE"
        else:
            latest = max(
                msg.status_list,
                key=lambda item: (
                    item.goal_info.stamp.sec,
                    item.goal_info.stamp.nanosec,
                ),
            )
            status_name = STATUS_NAMES.get(int(latest.status), str(latest.status))

        if status_name != self.nav_status:
            old = self.nav_status
            self.nav_status = status_name
            self.event("NAV_STATUS", f"{old}->{status_name}")

    def robot_pose_in_frame(self, frame):
        frame = (frame or "").lstrip("/")
        if not frame:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                "base_link",
                Time(),
            )

            translation = transform.transform.translation
            yaw = quaternion_yaw(transform.transform.rotation)

            return float(translation.x), float(translation.y), yaw
        except TransformException:
            return None

    def odom_pose(self):
        if self.odom_msg is None:
            return None

        pose = self.odom_msg.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_yaw(pose.orientation),
        )

    def current_goal_metrics(self):
        if self.goal is None:
            return math.nan, math.nan, None

        robot_pose = self.robot_pose_in_frame(self.goal_frame)

        if robot_pose is None and self.goal_frame == "odom":
            robot_pose = self.odom_pose()

        if robot_pose is None:
            return math.nan, math.nan, None

        robot_x, robot_y, robot_yaw = robot_pose
        dx = self.goal[0] - robot_x
        dy = self.goal[1] - robot_y

        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_error = normalize_angle(bearing - robot_yaw)

        return distance, heading_error, robot_pose

    def transform_costmap_point_to_base(
        self,
        source_x,
        source_y,
        transform,
    ):
        tf_yaw = quaternion_yaw(transform.transform.rotation)
        cos_tf = math.cos(tf_yaw)
        sin_tf = math.sin(tf_yaw)

        translation = transform.transform.translation

        base_x = (
            translation.x
            + cos_tf * source_x
            - sin_tf * source_y
        )

        base_y = (
            translation.y
            + sin_tf * source_x
            + cos_tf * source_y
        )

        return base_x, base_y

    def compute_clearance(self):
        msg = self.costmap_msg

        if msg is None:
            return math.nan, math.nan, math.nan

        frame = msg.header.frame_id.lstrip("/")
        if not frame:
            return math.nan, math.nan, math.nan

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                frame,
                Time(),
            )
        except TransformException:
            return math.nan, math.nan, math.nan

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        origin = msg.info.origin
        origin_yaw = quaternion_yaw(origin.orientation)
        cos_origin = math.cos(origin_yaw)
        sin_origin = math.sin(origin_yaw)

        lethal_clearance = math.inf
        cost50_clearance = math.inf

        near_cells = 0
        unknown_near = 0

        cell_half_diagonal = resolution * math.sqrt(2.0) / 2.0

        for row in range(height):
            row_offset = row * width

            for column in range(width):
                value = int(msg.data[row_offset + column])

                local_x = (column + 0.5) * resolution
                local_y = (row + 0.5) * resolution

                source_x = (
                    origin.position.x
                    + cos_origin * local_x
                    - sin_origin * local_y
                )

                source_y = (
                    origin.position.y
                    + sin_origin * local_x
                    + cos_origin * local_y
                )

                base_x, base_y = self.transform_costmap_point_to_base(
                    source_x,
                    source_y,
                    transform,
                )

                if abs(base_x) > 0.80 or abs(base_y) > 0.80:
                    continue

                if math.hypot(base_x, base_y) <= 0.50:
                    near_cells += 1
                    if value < 0:
                        unknown_near += 1

                if value < 50:
                    continue

                clearance = max(
                    0.0,
                    point_polygon_distance(base_x, base_y, FOOTPRINT)
                    - cell_half_diagonal,
                )

                if value >= 50:
                    cost50_clearance = min(cost50_clearance, clearance)

                if value >= 90:
                    lethal_clearance = min(lethal_clearance, clearance)

        unknown_ratio = (
            unknown_near / near_cells
            if near_cells > 0
            else math.nan
        )

        if not math.isfinite(lethal_clearance):
            lethal_clearance = math.nan

        if not math.isfinite(cost50_clearance):
            cost50_clearance = math.nan

        return lethal_clearance, cost50_clearance, unknown_ratio

    def count_angular_flips(self):
        now = self.monotonic_now()

        while (
            self.angular_sign_history
            and now - self.angular_sign_history[0][0] > OSCILLATION_WINDOW_S
        ):
            self.angular_sign_history.popleft()

        signs = [entry[1] for entry in self.angular_sign_history]
        return sum(
            1
            for index in range(1, len(signs))
            if signs[index] != signs[index - 1]
        )

    def update_goal_change_window(self):
        now = self.monotonic_now()

        while (
            self.goal_change_times
            and now - self.goal_change_times[0] > GOAL_SPAM_WINDOW_S
        ):
            self.goal_change_times.popleft()

        return len(self.goal_change_times)

    def write_metadata(self):
        self.metadata_path.write_text(
            "\n".join([
                "Robot gold diagnostic logger",
                "",
                "Rate: 1 telemetry row per second",
                "",
                f"TIGHT_CLEARANCE_M={TIGHT_CLEARANCE_M}",
                f"COLLISION_MARGIN_M={COLLISION_MARGIN_M}",
                f"GOAL_CHANGE_DISTANCE_M={GOAL_CHANGE_DISTANCE_M}",
                f"GOAL_SPAM_CHANGES={GOAL_SPAM_CHANGES}",
                f"GOAL_SPAM_WINDOW_S={GOAL_SPAM_WINDOW_S}",
                f"NO_PROGRESS_WINDOW_S={NO_PROGRESS_WINDOW_S}",
                f"NO_PROGRESS_REQUIRED_M={NO_PROGRESS_REQUIRED_M}",
                f"STUCK_REQUIRED_S={STUCK_REQUIRED_S}",
                f"OSCILLATION_WINDOW_S={OSCILLATION_WINDOW_S}",
                f"OSCILLATION_FLIPS={OSCILLATION_FLIPS}",
                "",
                "Subscribed topics:",
                "/odom",
                "/cmd_vel",
                "/scan",
                "/map",
                "/local_costmap/costmap",
                "/plan",
                "/frontier_explorer/selected_frontier",
                "/frontier_explorer/frontiers",
                "/local_costmap/published_footprint",
                "/navigate_to_pose/_action/status",
                "",
                "Clearance is measured from the configured rectangular",
                "robot footprint to costmap cells, not from robot center.",
                "",
            ]) + "\n"
        )

    def sample_once_per_second(self):
        now = self.monotonic_now()

        map_pose = self.robot_pose_in_frame("map")
        odom_pose = self.odom_pose()

        map_x = map_y = map_yaw = math.nan
        if map_pose is not None:
            map_x, map_y, map_yaw = map_pose

        odom_x = odom_y = odom_yaw = math.nan
        measured_v = measured_w = math.nan

        if odom_pose is not None:
            odom_x, odom_y, odom_yaw = odom_pose

        if self.odom_msg is not None:
            measured_v = float(self.odom_msg.twist.twist.linear.x)
            measured_w = float(self.odom_msg.twist.twist.angular.z)

        command_v = command_w = 0.0
        if self.cmd_msg is not None:
            command_v = float(self.cmd_msg.twist.linear.x)
            command_w = float(self.cmd_msg.twist.angular.z)

        goal_distance, goal_heading_error, _goal_robot_pose = (
            self.current_goal_metrics()
        )

        goal_x = self.goal[0] if self.goal is not None else math.nan
        goal_y = self.goal[1] if self.goal is not None else math.nan

        lethal_clearance, cost50_clearance, unknown_ratio = (
            self.compute_clearance()
        )

        tight_space = (
            math.isfinite(lethal_clearance)
            and lethal_clearance < TIGHT_CLEARANCE_M
        )

        collision_margin = (
            math.isfinite(lethal_clearance)
            and lethal_clearance < COLLISION_MARGIN_M
        )

        command_motion = (
            abs(command_v) > 0.015
            or abs(command_w) > 0.05
        )

        measured_motion = (
            math.isfinite(measured_v)
            and math.isfinite(measured_w)
            and (
                abs(measured_v) > 0.004
                or abs(measured_w) > 0.02
            )
        )

        if command_motion and not measured_motion:
            if self.stuck_started_at is None:
                self.stuck_started_at = now
            stuck_seconds = now - self.stuck_started_at
        else:
            self.stuck_started_at = None
            stuck_seconds = 0.0

        stuck = stuck_seconds >= STUCK_REQUIRED_S

        active_navigation = self.nav_status in {
            "ACCEPTED",
            "EXECUTING",
            "CANCELING",
        }

        if active_navigation and math.isfinite(goal_distance):
            self.distance_history.append((now, goal_distance))

            while (
                self.distance_history
                and now - self.distance_history[0][0] > NO_PROGRESS_WINDOW_S
            ):
                self.distance_history.popleft()
        else:
            self.distance_history.clear()

        no_progress = False
        progress_m = math.nan

        if len(self.distance_history) >= 2:
            elapsed = (
                self.distance_history[-1][0]
                - self.distance_history[0][0]
            )

            progress_m = (
                self.distance_history[0][1]
                - self.distance_history[-1][1]
            )

            no_progress = (
                elapsed >= NO_PROGRESS_WINDOW_S - 1.0
                and self.distance_history[-1][1] > 0.15
                and progress_m < NO_PROGRESS_REQUIRED_M
            )

        angular_flips = self.count_angular_flips()

        oscillating = (
            angular_flips >= OSCILLATION_FLIPS
            and abs(command_v) < 0.03
        )

        goal_changes = self.update_goal_change_window()
        goal_spam = goal_changes >= GOAL_SPAM_CHANGES

        sensor_stale = (
            self.age("scan") > 2.0
            or self.age("odom") > 1.5
            or self.age("costmap") > 3.0
        )

        self.transition_event(
            "TIGHT_SPACE",
            tight_space,
            (
                f"lethal_clearance={finite_or_blank(lethal_clearance)} "
                f"cost50_clearance={finite_or_blank(cost50_clearance)}"
            ),
            f"lethal_clearance={finite_or_blank(lethal_clearance)}",
        )

        self.transition_event(
            "COLLISION_MARGIN",
            collision_margin,
            f"lethal_clearance={finite_or_blank(lethal_clearance)}",
            f"lethal_clearance={finite_or_blank(lethal_clearance)}",
        )

        self.transition_event(
            "ROBOT_STUCK",
            stuck,
            (
                f"cmd_v={command_v:.3f} cmd_w={command_w:.3f} "
                f"measured_v={finite_or_blank(measured_v)} "
                f"measured_w={finite_or_blank(measured_w)}"
            ),
            f"stuck_duration={stuck_seconds:.1f}s",
        )

        self.transition_event(
            "NO_GOAL_PROGRESS",
            no_progress,
            (
                f"goal_distance={finite_or_blank(goal_distance)} "
                f"progress={finite_or_blank(progress_m)} "
                f"window={NO_PROGRESS_WINDOW_S}s"
            ),
            f"goal_distance={finite_or_blank(goal_distance)}",
        )

        self.transition_event(
            "ANGULAR_OSCILLATION",
            oscillating,
            (
                f"sign_flips={angular_flips} "
                f"window={OSCILLATION_WINDOW_S}s "
                f"cmd_w={command_w:.3f}"
            ),
            f"sign_flips={angular_flips}",
        )

        self.transition_event(
            "GOAL_SPAM",
            goal_spam,
            (
                f"goal_changes={goal_changes} "
                f"window={GOAL_SPAM_WINDOW_S}s"
            ),
            f"goal_changes={goal_changes}",
        )

        self.transition_event(
            "SENSOR_STALE",
            sensor_stale,
            (
                f"scan_age={finite_or_blank(self.age('scan'))} "
                f"odom_age={finite_or_blank(self.age('odom'))} "
                f"costmap_age={finite_or_blank(self.age('costmap'))}"
            ),
            "sensor streams recovered",
        )

        self.csv_writer.writerow([
            self.utc_now(),
            f"{self.get_clock().now().nanoseconds / 1e9:.3f}",

            finite_or_blank(map_x),
            finite_or_blank(map_y),
            finite_or_blank(math.degrees(map_yaw), 2),

            finite_or_blank(odom_x),
            finite_or_blank(odom_y),
            finite_or_blank(math.degrees(odom_yaw), 2),

            finite_or_blank(measured_v),
            finite_or_blank(measured_w),
            finite_or_blank(command_v),
            finite_or_blank(command_w),

            finite_or_blank(goal_x),
            finite_or_blank(goal_y),
            self.goal_frame,
            self.goal_source,
            finite_or_blank(goal_distance),
            finite_or_blank(
                math.degrees(goal_heading_error)
                if math.isfinite(goal_heading_error)
                else math.nan,
                2,
            ),

            self.nav_status,
            self.frontier_points,

            self.path_pose_count,
            finite_or_blank(self.path_length),

            finite_or_blank(lethal_clearance),
            finite_or_blank(cost50_clearance),
            finite_or_blank(unknown_ratio),
            int(tight_space),
            int(collision_margin),

            goal_changes,
            angular_flips,
            finite_or_blank(stuck_seconds, 1),
            int(no_progress),
            int(oscillating),

            finite_or_blank(self.age("scan"), 2),
            finite_or_blank(self.age("odom"), 2),
            finite_or_blank(self.age("cmd"), 2),
            finite_or_blank(self.age("map"), 2),
            finite_or_blank(self.age("costmap"), 2),
            finite_or_blank(self.age("plan"), 2),

            finite_or_blank(self.footprint_area),
            finite_or_blank(self.footprint_min_edge),
            finite_or_blank(self.footprint_max_edge),
        ])

        self.telemetry_file.flush()

    def close_files(self):
        self.event("LOGGER_STOP", "logger shutting down")

        try:
            self.telemetry_file.flush()
            self.telemetry_file.close()
        except Exception:
            pass

        try:
            self.events_file.flush()
            self.events_file.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory for telemetry.csv, events.log and metadata.txt",
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RobotGoldLogger(Path(args.output_dir).expanduser())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
