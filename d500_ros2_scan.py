#!/usr/bin/env python3
import math
import struct
import time
import argparse
import threading
from dataclasses import dataclass
from pathlib import Path

import serial

from builtin_interfaces.msg import Time as RosTime
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# =========================
# D500 / STL-19P packet format
# =========================
HEADER0 = 0x54
VERLEN = 0x2C
POINTS = 12
PKT_LEN = 47


def crc8_table_poly_0x4D():
    poly = 0x4D
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            if c & 0x80:
                c = ((c << 1) ^ poly) & 0xFF
            else:
                c = (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = crc8_table_poly_0x4D()


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c


def parse_packet(pkt: bytes):
    """Return (rpm, angles_deg[12], pts[(dist_mm, intensity)], timestamp)."""
    speed_dps, start_angle = struct.unpack_from("<HH", pkt, 2)

    pts = []
    off = 6
    for _ in range(POINTS):
        dist_mm = struct.unpack_from("<H", pkt, off)[0]
        intensity = pkt[off + 2]
        pts.append((dist_mm, intensity))
        off += 3

    end_angle, timestamp = struct.unpack_from("<HH", pkt, 42)

    start_deg = (start_angle % 36000) / 100.0
    end_deg = (end_angle % 36000) / 100.0

    diff = (end_deg - start_deg + 360.0) % 360.0
    step = diff / (POINTS - 1)
    angles_deg = [(start_deg + i * step) % 360.0 for i in range(POINTS)]

    rpm = speed_dps / 6.0  # per your earlier code
    return rpm, angles_deg, pts, timestamp


def find_device_by_id():
    by_id = Path("/dev/serial/by-id")
    if by_id.exists():
        entries = sorted(by_id.iterdir())
        if entries:
            return str(entries[0])
    return None


def is_wrap(prev_deg, cur_deg):
    """Detect 360->0 wrap as a large backward jump."""
    return prev_deg is not None and (prev_deg - cur_deg) > 180.0



def _mirror_laserscan_left_right(msg):
    """
    Fix D500 left/right mirror.

    ROS LaserScan convention:
      angle 0      = +X/front
      positive yaw = CCW = +Y/left

    Our D500 data had front correct, but left/right mirrored.
    This remaps each output angle theta to raw angle -theta.
    """
    n = len(msg.ranges)
    if n == 0:
        return msg

    out = LaserScan()
    out.header = msg.header

    out.angle_min = -math.pi
    out.angle_increment = 2.0 * math.pi / float(n)
    out.angle_max = out.angle_min + out.angle_increment * float(n - 1)

    out.time_increment = msg.time_increment
    out.scan_time = msg.scan_time
    out.range_min = msg.range_min
    out.range_max = msg.range_max

    out.ranges = [float("inf")] * n
    out.intensities = [0.0] * n if len(msg.intensities) == n else []

    raw_inc = msg.angle_increment
    if abs(raw_inc) < 1e-12:
        return msg

    for i in range(n):
        theta_out = out.angle_min + i * out.angle_increment
        theta_raw = (-theta_out) % (2.0 * math.pi)
        j = int(round((theta_raw - msg.angle_min) / raw_inc)) % n

        out.ranges[i] = msg.ranges[j]
        if out.intensities:
            out.intensities[i] = msg.intensities[j]

    return out


@dataclass(frozen=True)
class CompletedScan:
    """One immutable scan handed from the serial worker to the ROS thread."""

    ranges: tuple
    intensities: tuple
    acquisition_start_wall: float
    acquisition_end_wall: float
    acquisition_midpoint_wall: float
    scan_time: float
    rpm: float
    valid_bins: int


class SerialScanWorker(threading.Thread):
    """Own the blocking serial port and assemble complete revolutions."""

    def __init__(self, args):
        super().__init__(name="d500_serial_reader", daemon=True)
        self.port = args.port or find_device_by_id() or "/dev/ttyUSB0"
        self.baud = args.baud
        self.bins = args.bins
        self.min_mm = args.min_mm
        self.max_mm = args.max_mm
        self.min_intensity = args.min_intensity
        self.invert = args.invert
        self.angle_offset_deg = args.angle_offset_deg

        self.stop_event = threading.Event()
        self.handoff_lock = threading.Lock()
        self.stats_lock = threading.Lock()
        self.latest_scan = None

        self.serial_error_count = 0
        self.parser_error_count = 0
        self.bad_crc_count = 0
        self.packet_count = 0
        self.completed_scan_count = 0
        self.handoff_drop_count = 0
        self.last_rpm = 0.0
        self.last_error = ""
        self.last_acquisition_duration = 0.0
        self.max_acquisition_duration = 0.0
        self.acquisition_durations = []
        self.ser = None

        self.buf = bytearray()
        self.last_angle = None
        self.started = False
        self.current_ranges = [math.inf] * self.bins
        self.current_intensities = [0.0] * self.bins
        self.rev_start_wall = None

    def stop(self):
        self.stop_event.set()

    def take_latest_scan(self):
        """Return the newest complete scan and discard no newer scan."""
        with self.handoff_lock:
            scan = self.latest_scan
            self.latest_scan = None
            return scan

    def snapshot(self):
        with self.stats_lock:
            durations = tuple(self.acquisition_durations)
            return {
                "serial_error_count": self.serial_error_count,
                "parser_error_count": self.parser_error_count,
                "bad_crc_count": self.bad_crc_count,
                "packet_count": self.packet_count,
                "completed_scan_count": self.completed_scan_count,
                "handoff_drop_count": self.handoff_drop_count,
                "last_rpm": self.last_rpm,
                "last_error": self.last_error,
                "last_acquisition_duration": self.last_acquisition_duration,
                "max_acquisition_duration": self.max_acquisition_duration,
                "acquisition_mean_duration": (
                    sum(durations) / len(durations) if durations else 0.0
                ),
            }

    def _record_error(self, kind, exc):
        with self.stats_lock:
            if kind == "serial":
                self.serial_error_count += 1
            else:
                self.parser_error_count += 1
            self.last_error = f"{kind}: {exc}"

    def _reset_current_scan(self):
        self.current_ranges = [math.inf] * self.bins
        self.current_intensities = [0.0] * self.bins

    def _angle_to_bin(self, ang_deg):
        a = (ang_deg + self.angle_offset_deg) % 360.0
        if self.invert:
            a = (360.0 - a) % 360.0
        return int((a / 360.0) * self.bins) % self.bins

    def _update_scan_with_point(self, ang_deg, dist_mm, intensity):
        if dist_mm == 0:
            return
        if dist_mm < self.min_mm or dist_mm > self.max_mm:
            return
        if intensity < self.min_intensity:
            return

        idx = self._angle_to_bin(ang_deg)
        r_m = dist_mm / 1000.0
        if r_m < self.current_ranges[idx]:
            self.current_ranges[idx] = r_m
            self.current_intensities[idx] = float(intensity)

    def _handoff_completed_scan(self, scan):
        with self.handoff_lock:
            if self.latest_scan is not None:
                # Keep only the newest complete scan. A blocked ROS executor
                # must not create an unbounded backlog of stale scans.
                with self.stats_lock:
                    self.handoff_drop_count += 1
            self.latest_scan = scan

        duration = scan.scan_time
        with self.stats_lock:
            self.completed_scan_count += 1
            self.last_acquisition_duration = duration
            self.max_acquisition_duration = max(
                self.max_acquisition_duration, duration
            )
            self.acquisition_durations.append(duration)
            if len(self.acquisition_durations) > 120:
                del self.acquisition_durations[:-120]

    def _process_packet(self, pkt):
        if crc8(pkt[:-1]) != pkt[-1]:
            with self.stats_lock:
                self.bad_crc_count += 1
            return

        with self.stats_lock:
            self.packet_count += 1

        try:
            rpm, angles_deg, pts, _sensor_timestamp = parse_packet(pkt)
        except Exception as exc:
            self._record_error("parser", exc)
            return

        with self.stats_lock:
            self.last_rpm = rpm

        for ang_deg, (dist_mm, intensity) in zip(angles_deg, pts):
            if not self.started:
                if dist_mm > 0:
                    self.started = True
                    self.last_angle = ang_deg
                    self.rev_start_wall = time.time()
                self._update_scan_with_point(ang_deg, dist_mm, intensity)
                continue

            if is_wrap(self.last_angle, ang_deg):
                end_wall = time.time()
                start_wall = self.rev_start_wall or end_wall
                scan_time = max(0.0, end_wall - start_wall)
                midpoint_wall = start_wall + scan_time / 2.0
                scan = CompletedScan(
                    ranges=tuple(self.current_ranges),
                    intensities=tuple(self.current_intensities),
                    acquisition_start_wall=start_wall,
                    acquisition_end_wall=end_wall,
                    acquisition_midpoint_wall=midpoint_wall,
                    scan_time=scan_time,
                    rpm=rpm,
                    valid_bins=sum(
                        1 for value in self.current_ranges
                        if math.isfinite(value)
                    ),
                )
                self._handoff_completed_scan(scan)
                self._reset_current_scan()
                self.rev_start_wall = end_wall

            self.last_angle = ang_deg
            self._update_scan_with_point(ang_deg, dist_mm, intensity)

    def _parse_available(self):
        while True:
            i = self.buf.find(bytes([HEADER0, VERLEN]))
            if i < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                return

            if len(self.buf) - i < PKT_LEN:
                if i > 0:
                    del self.buf[:i]
                return

            pkt = bytes(self.buf[i:i + PKT_LEN])
            del self.buf[:i + PKT_LEN]
            self._process_packet(pkt)

    def run(self):
        try:
            self.ser = serial.Serial(
                self.port,
                baudrate=self.baud,
                timeout=0.05,
            )
            self.ser.reset_input_buffer()
            while not self.stop_event.is_set():
                chunk = self.ser.read(4096)
                if chunk:
                    self.buf.extend(chunk)
                self._parse_available()
        except serial.SerialException as exc:
            self._record_error("serial", exc)
        except Exception as exc:
            self._record_error("serial", exc)
        finally:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass


def wall_time_to_ros_time(wall_time):
    """Convert system wall-clock acquisition time to a ROS system-time stamp."""
    seconds = int(wall_time)
    nanoseconds = int(round((wall_time - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return RosTime(sec=seconds, nanosec=nanoseconds)


class D500Ros2ScanNode(Node):
    def __init__(self, args):
        super().__init__("d500_ros2_scan")

        self.frame_id = args.frame_id
        self.bins = args.bins
        self.min_mm = args.min_mm
        self.max_mm = args.max_mm
        self.topic = args.topic

        self.scan_pub = self.create_publisher(LaserScan, self.topic, 10)

        self.worker = SerialScanWorker(args)
        self.worker.start()
        self.published_count = 0
        self.last_publish_wall = None
        self.max_publish_gap = 0.0
        self.last_status_log_wall = time.time()

        # This timer only takes an already-complete scan from a one-slot
        # handoff. It never waits for serial data or performs packet parsing.
        self.publish_timer = self.create_timer(0.005, self._publish_pending_scan)
        self.status_timer = self.create_timer(1.0, self._log_worker_status)

        self.get_logger().info(f"Opening {self.worker.port} @ {self.worker.baud}")
        self.get_logger().info(
            f"Publishing LaserScan on {self.topic} | frame_id={self.frame_id} | bins={self.bins}"
        )
        self.get_logger().info(
            "If RViz scan looks mirrored/rotated, try --invert and/or --angle-offset-deg."
        )
        self.get_logger().info(
            "D500 acquisition: dedicated serial/parser thread, latest-scan handoff, "
            "acquisition-midpoint timestamps"
        )

    def destroy_node(self):
        try:
            if hasattr(self, "worker"):
                self.worker.stop()
                self.worker.join(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()

    def _publish_current_scan(self, scan):
        msg = LaserScan()
        # Real-robot runs use system time (use_sim_time=false). The timestamp
        # is the midpoint of the host-observed acquisition interval, rather
        # than the later ROS publication time.
        msg.header.stamp = wall_time_to_ros_time(
            scan.acquisition_midpoint_wall
        )
        msg.header.frame_id = self.frame_id

        # Standard ROS CCW convention around +Z, 0 angle along +X
        # We'll publish [0, 2pi) discretized.
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / float(self.bins)

        # Timing
        scan_time = scan.scan_time if scan.scan_time > 0.0 else 0.1
        msg.scan_time = float(scan_time)
        msg.time_increment = float(scan_time / self.bins)

        msg.range_min = self.min_mm / 1000.0
        msg.range_max = self.max_mm / 1000.0

        # LaserScan expects NaN/inf for no return; RViz handles inf fine.
        msg.ranges = list(scan.ranges)
        msg.intensities = list(scan.intensities)

        msg = _mirror_laserscan_left_right(msg)
        self.scan_pub.publish(msg)
        self.published_count += 1

    def _publish_pending_scan(self):
        scan = self.worker.take_latest_scan()
        if scan is None:
            return

        now_wall = time.time()
        if self.last_publish_wall is not None:
            publish_gap = max(0.0, now_wall - self.last_publish_wall)
            self.max_publish_gap = max(self.max_publish_gap, publish_gap)
        self.last_publish_wall = now_wall
        scan_age = max(0.0, now_wall - scan.acquisition_midpoint_wall)

        self._publish_current_scan(scan)

        if self.published_count % 20 == 0:
            stats = self.worker.snapshot()
            self.get_logger().info(
                "D500_TIMING "
                f"scans={self.published_count} "
                f"acq_last={stats['last_acquisition_duration'] * 1000.0:.1f}ms "
                f"acq_mean={stats['acquisition_mean_duration'] * 1000.0:.1f}ms "
                f"acq_max={stats['max_acquisition_duration'] * 1000.0:.1f}ms "
                f"pub_gap_max={self.max_publish_gap * 1000.0:.1f}ms "
                f"age={scan_age * 1000.0:.1f}ms "
                f"handoff_drops={stats['handoff_drop_count']} "
                f"bad_crc={stats['bad_crc_count']} "
                f"serial_errors={stats['serial_error_count']} "
                f"parser_errors={stats['parser_error_count']} "
                f"valid_bins={scan.valid_bins}/{self.bins} "
                f"rpm~{scan.rpm:.1f}"
            )

    def _log_worker_status(self):
        if self.worker.is_alive():
            return
        stats = self.worker.snapshot()
        if stats["last_error"]:
            self.get_logger().error(
                f"D500 acquisition worker stopped: {stats['last_error']}"
            )


def parse_args():
    ap = argparse.ArgumentParser(
        description="ROS2 LaserScan publisher for D500/STL-19P-style LiDAR over USB serial."
    )
    ap.add_argument("--port", default=None,
                    help="Serial port (e.g. /dev/ttyUSB0 or /dev/serial/by-id/...).")
    ap.add_argument("--baud", type=int, default=230400,
                    help="Serial baudrate (D500 default is 230400).")
    ap.add_argument("--topic", default="/scan", help="ROS2 topic name for LaserScan.")
    ap.add_argument("--frame-id", default="laser", help="LaserScan frame_id.")
    ap.add_argument("--bins", type=int, default=720,
                    help="Angle bins per revolution (720 = 0.5 deg/bin).")
    ap.add_argument("--min-mm", type=int, default=30, help="Minimum valid range (mm).")
    ap.add_argument("--max-mm", type=int, default=12000, help="Maximum valid range (mm).")
    ap.add_argument("--min-intensity", type=int, default=0,
                    help="Drop returns below this intensity.")
    ap.add_argument("--invert", action="store_true",
                    help="Invert angle direction (use if RViz looks mirrored/spins wrong direction).")
    ap.add_argument("--angle-offset-deg", type=float, default=0.0,
                    help="Rotate scan by this offset in degrees (for RViz alignment).")
    return ap.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = None
    try:
        node = D500Ros2ScanNode(args)
        rclpy.spin(node)
    except (
        KeyboardInterrupt,
        ExternalShutdownException,
        rclpy_implementation.RCLError,
    ):
        pass
    except serial.SerialException as e:
        print(f"Serial error opening/using port: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
