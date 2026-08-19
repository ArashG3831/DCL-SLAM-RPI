#!/usr/bin/env python3
import math
import struct
import time
import argparse
from pathlib import Path

import serial

import rclpy
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

class D500Ros2ScanNode(Node):
    def __init__(self, args):
        super().__init__("d500_ros2_scan")

        self.port = args.port or find_device_by_id() or "/dev/ttyUSB0"
        self.baud = args.baud
        self.frame_id = args.frame_id
        self.bins = args.bins
        self.min_mm = args.min_mm
        self.max_mm = args.max_mm
        self.min_intensity = args.min_intensity
        self.invert = args.invert  # reverse angle direction for RViz if needed
        self.angle_offset_deg = args.angle_offset_deg  # rotate scan if needed
        self.topic = args.topic

        self.scan_pub = self.create_publisher(LaserScan, self.topic, 10)

        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.05)
        self.ser.reset_input_buffer()

        self.buf = bytearray()

        self.last_angle = None
        self.started = False
        self.current_ranges = [math.inf] * self.bins
        self.current_intensities = [0.0] * self.bins

        self.rev_start_time = time.time()
        self.last_rpm = 0.0
        self.packet_count = 0
        self.bad_crc_count = 0
        self.published_count = 0

        # Timer to poll serial and parse packets
        self.timer = self.create_timer(0.005, self._poll_serial)

        self.get_logger().info(f"Opening {self.port} @ {self.baud}")
        self.get_logger().info(
            f"Publishing LaserScan on {self.topic} | frame_id={self.frame_id} | bins={self.bins}"
        )
        self.get_logger().info(
            "If RViz scan looks mirrored/rotated, try --invert and/or --angle-offset-deg."
        )

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def _reset_current_scan(self):
        self.current_ranges = [math.inf] * self.bins
        self.current_intensities = [0.0] * self.bins

    def _angle_to_bin(self, ang_deg):
        # Apply optional rotation offset
        a = (ang_deg + self.angle_offset_deg) % 360.0

        # Optional invert direction (clockwise<->CCW)
        if self.invert:
            a = (360.0 - a) % 360.0

        idx = int((a / 360.0) * self.bins) % self.bins
        return idx

    def _update_scan_with_point(self, ang_deg, dist_mm, intensity):
        # Filter invalid / out-of-range / low intensity
        if dist_mm == 0:
            return
        if dist_mm < self.min_mm or dist_mm > self.max_mm:
            return
        if intensity < self.min_intensity:
            return

        idx = self._angle_to_bin(ang_deg)
        r_m = dist_mm / 1000.0

        # Keep nearest return in each bin (works well for obstacles)
        if r_m < self.current_ranges[idx]:
            self.current_ranges[idx] = r_m
            self.current_intensities[idx] = float(intensity)

    def _publish_current_scan(self, scan_time):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Standard ROS CCW convention around +Z, 0 angle along +X
        # We'll publish [0, 2pi) discretized.
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / float(self.bins)

        # Timing
        if scan_time <= 0.0:
            scan_time = 0.1
        msg.scan_time = float(scan_time)
        msg.time_increment = float(scan_time / self.bins)

        msg.range_min = self.min_mm / 1000.0
        msg.range_max = self.max_mm / 1000.0

        # LaserScan expects NaN/inf for no return; RViz handles inf fine.
        msg.ranges = self.current_ranges
        msg.intensities = self.current_intensities

        msg = _mirror_laserscan_left_right(msg)
        self.scan_pub.publish(msg)
        self.published_count += 1

        if self.published_count % 20 == 0:
            valid = sum(1 for r in self.current_ranges if math.isfinite(r))
            self.get_logger().info(
                f"Published scans={self.published_count}, valid_bins={valid}/{self.bins}, "
                f"rpm~{self.last_rpm:.1f}, bad_crc={self.bad_crc_count}"
            )

    def _poll_serial(self):
        try:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf.extend(chunk)
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            return

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

            if crc8(pkt[:-1]) != pkt[-1]:
                self.bad_crc_count += 1
                continue

            self.packet_count += 1

            try:
                rpm, angles_deg, pts, _ts = parse_packet(pkt)
            except Exception as e:
                self.get_logger().warn(f"Packet parse error: {e}")
                continue

            self.last_rpm = rpm

            for ang_deg, (dist_mm, intensity) in zip(angles_deg, pts):
                if not self.started:
                    if dist_mm > 0:
                        self.started = True
                        self.last_angle = ang_deg
                        self.rev_start_time = time.time()
                    # Even before started, still allow point update
                    self._update_scan_with_point(ang_deg, dist_mm, intensity)
                    continue

                # Detect revolution wrap -> publish one full scan
                if is_wrap(self.last_angle, ang_deg):
                    now = time.time()
                    scan_time = now - self.rev_start_time
                    self._publish_current_scan(scan_time)
                    self._reset_current_scan()
                    self.rev_start_time = now

                self.last_angle = ang_deg
                self._update_scan_with_point(ang_deg, dist_mm, intensity)


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
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"Serial error opening/using port: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
