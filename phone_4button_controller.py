#!/usr/bin/env python3

"""Safe phone teleoperation page with a responsive live SLAM map."""

import csv
import json
import math
import os
import shlex
import socket
import signal
import subprocess
import threading
import time
from contextlib import contextmanager
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import Reset as SlamReset


# Robot geometry. The motor node remains the authority for wheel conversion;
# these values are used only to generate the fixed manual-drive speed.
WHEEL_RADIUS = 0.0350
# Must match real_diffdrive_node's effective odometry separation.
WHEEL_SEPARATION = 0.2216

# Forward/backward full speed and moderate mapping spin.
FORWARD_RPM = 50.0
SPIN_MAX_RADPS = 0.75

# Safety and transport.
PUB_RATE_HZ = 20.0
COMMAND_TIMEOUT_S = 0.35
TRANSITION_REST_S = 0.50
HTTP_PORT = 8080
MAP_POLL_MS = 250
POSE_POLL_MS = 50
MAX_MAP_DISPLAY_DIM = 1200
# The lidar publishes at roughly 10 Hz, so a 0.4 s gap means several missed
# scans.  This is just above the largest observed callback stall (0.353 s)
# caused by copying the map, while still detecting a 0.9 s interruption early.
LIDAR_GAP_THRESHOLD_S = 0.40

# The phone program supervises the minimum real-robot stack needed for the
# live map and teleoperation page.  Existing processes are reused; only
# processes started by this program are stopped on exit.
SLAM_PARAMS = os.path.expanduser(
    os.environ.get(
        "ROBOT1_SLAM_PARAMS",
        "~/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml",
    )
)
START_LIDAR_SCRIPT = os.path.expanduser(
    os.environ.get("ROBOT1_LIDAR_START_SCRIPT", "~/start_d500_with_recovery.sh")
)
START_MOTOR = os.environ.get("ROBOT1_PHONE_START_MOTOR", "1").strip().lower() not in {
    "0",
    "false",
    "no",
    "off",
}
START_LIDAR = os.environ.get("ROBOT1_PHONE_START_LIDAR", "1").strip().lower() not in {
    "0",
    "false",
    "no",
    "off",
}
STACK_LOG_DIR = os.path.expanduser(
    os.environ.get("ROBOT1_PHONE_STACK_LOG_DIR", "/tmp/robot1_phone_stack")
)
MAP_SAVE_DIR = os.path.expanduser(
    os.environ.get("ROBOT1_MAP_SAVE_DIR", "~/robot1_maps")
)
MAP_CHECKPOINT_INTERVAL_S = 10.0
MAP_SAVE_TIMEOUT_S = 15.0


class PhoneTimingDiagnostics:
    """Optional, asynchronous timing trace for the phone ROS/HTTP workload."""

    def __init__(self):
        self.enabled = os.environ.get("R1_PHONE_DIAGNOSTICS", "0") == "1"
        self.path = os.environ.get(
            "R1_PHONE_DIAGNOSTICS_PATH", "/tmp/r1_phone_timing.csv"
        )
        self.events = []
        self.lock = threading.Lock()
        self.wakeup = threading.Event()
        self.stop_event = threading.Event()
        self.thread = None
        if self.enabled:
            self.thread = threading.Thread(
                target=self._writer,
                name="phone_timing_writer",
                daemon=True,
            )
            self.thread.start()

    def record(self, name, phase, duration_ms=None, detail=""):
        if not self.enabled:
            return
        event = (
            time.time(),
            time.monotonic(),
            threading.current_thread().name,
            name,
            phase,
            "" if duration_ms is None else f"{duration_ms:.3f}",
            detail,
        )
        with self.lock:
            if len(self.events) < 50000:
                self.events.append(event)
        self.wakeup.set()

    @contextmanager
    def measure(self, name, detail=""):
        if not self.enabled:
            yield
            return
        started = time.monotonic()
        self.record(name, "start", detail=detail)
        try:
            yield
        finally:
            duration_ms = (time.monotonic() - started) * 1000.0
            self.record(name, "end", duration_ms=duration_ms, detail=detail)

    def _writer(self):
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.path, "w", newline="", buffering=1) as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "wall_time_iso", "wall_time_s", "monotonic_s", "thread",
                "event", "phase", "duration_ms", "detail",
            ])
            while not self.stop_event.is_set() or self.events:
                self.wakeup.wait(0.25)
                self.wakeup.clear()
                with self.lock:
                    batch = self.events[:]
                    del self.events[:]
                for wall_s, monotonic_s, thread_name, name, phase, duration, detail in batch:
                    writer.writerow([
                        datetime.fromtimestamp(wall_s).isoformat(timespec="milliseconds"),
                        f"{wall_s:.6f}",
                        f"{monotonic_s:.6f}",
                        thread_name,
                        name,
                        phase,
                        duration,
                        detail,
                    ])

    def close(self):
        if not self.enabled:
            return
        self.stop_event.set()
        self.wakeup.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            self.thread = None


PHONE_DIAGNOSTICS = PhoneTimingDiagnostics()


def ros_shell_command(command):
    """Run a ROS CLI command with the robot workspace overlays sourced."""
    setup = [
        "source /opt/ros/jazzy/setup.bash",
        '[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"',
        '[ -f "$HOME/nav2_ws/install/setup.bash" ] && source "$HOME/nav2_ws/install/setup.bash"',
        '[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"',
    ]
    return ["bash", "-lc", "\n".join(setup + [f"exec {shlex.join(command)}"])]


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
  <meta name="theme-color" content="#101217">
  <title>Robot 1</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101217;
      --panel: #181b22;
      --button: #2a2f39;
      --button-active: #1268d9;
      --stop: #b51f35;
      --text: #f3f5f7;
      --muted: #aeb6c2;
    }

    * { box-sizing: border-box; }

    html, body {
      width: 100%;
      height: 100%;
      height: 100dvh;
      margin: 0;
      overflow: hidden;
      background: var(--bg);
      color: var(--text);
      font-family: Arial, sans-serif;
      overscroll-behavior: none;
      touch-action: none;
      user-select: none;
    }

    #app {
      width: 100%;
      height: 100%;
      height: 100dvh;
      min-height: 0;
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }

    #header {
      flex: 0 0 auto;
      min-height: 40px;
      padding: 8px 12px;
      padding-top: max(8px, env(safe-area-inset-top));
      background: var(--panel);
      color: var(--muted);
      font-size: 13px;
      line-height: 1.25;
      text-align: center;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    #mapPanel {
      position: relative;
      flex: 1 1 auto;
      min-height: 0;
      overflow: hidden;
      background: #292d34;
    }

    #mapCanvas {
      position: absolute;
      inset: 0;
      display: block;
      width: 100%;
      height: 100%;
    }

    #controls {
      flex: 0 0 auto;
      display: flex;
      justify-content: center;
      padding: 8px 10px;
      padding-bottom: max(8px, env(safe-area-inset-bottom));
      background: var(--panel);
      overflow: hidden;
    }

    #controlStack {
      width: min(100%, 320px);
    }

    #speedControl {
      margin-bottom: 6px;
      padding: 0 3px;
    }

    #speedHeader {
      display: flex;
      justify-content: space-between;
      align-items: center;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.2;
    }

    #speedValue {
      color: var(--text);
      font-weight: 700;
    }

    #speedSlider {
      display: block;
      width: 100%;
      height: 24px;
      margin: 0;
      accent-color: var(--button-active);
      touch-action: auto;
    }

    #grid {
      width: 100%;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      grid-template-rows: repeat(3, clamp(46px, 8.5vh, 72px));
      gap: clamp(6px, 1.6vw, 10px);
    }

    button {
      min-width: 0;
      min-height: 0;
      border: 0;
      border-radius: 16px;
      background: var(--button);
      color: var(--text);
      font-size: clamp(27px, 8vw, 40px);
      font-weight: 700;
      box-shadow: 0 3px 0 #090a0d;
      touch-action: none;
    }

    button:active, button.active {
      background: var(--button-active);
      transform: translateY(2px);
      box-shadow: 0 1px 0 #090a0d;
    }

    button:disabled {
      cursor: not-allowed;
      background: #464c55;
      color: #8d96a2;
      opacity: 0.78;
      box-shadow: 0 3px 0 #292e35;
      transform: none;
    }

    button:disabled:active {
      background: #464c55;
      box-shadow: 0 3px 0 #292e35;
      transform: none;
    }

    #speedSlider:disabled {
      opacity: 0.42;
      cursor: not-allowed;
    }

    #reset {
      background: var(--stop);
      font-size: clamp(16px, 4.5vw, 25px);
      width: 70%;
      height: 70%;
      justify-self: center;
      align-self: center;
      border-radius: 12px;
    }

    #lidarHealth {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      min-width: 0;
      padding: 3px 4px;
      border-radius: 8px;
      border: 1px solid rgba(255, 255, 255, 0.18);
      background: transparent;
      color: var(--muted);
      font-size: clamp(9px, 2.4vw, 13px);
      line-height: 1.05;
      text-align: center;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    #lidarHealthLabel,
    #lidarHealthValue {
      display: block;
      max-width: 100%;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    #lidarHealth.fault {
      background: var(--stop);
      color: var(--text);
    }

    #motorSpeed {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      min-width: 0;
      padding: 3px 4px;
      border-radius: 8px;
      border: 1px solid rgba(255, 255, 255, 0.18);
      background: transparent;
      color: var(--muted);
      font-size: clamp(9px, 2.4vw, 13px);
      line-height: 1.05;
      text-align: center;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    #motorSpeedNow,
    #motorSpeedAvg {
      display: block;
      max-width: 100%;
      overflow: hidden;
      text-overflow: ellipsis;
    }

    @media (orientation: landscape) and (max-height: 520px) {
      #header { min-height: 30px; padding: 4px 10px; }
      #controls { padding: 5px 10px; }
      #grid { grid-template-rows: repeat(3, 42px); max-width: 280px; }
    }
  </style>
</head>
<body>
<main id="app">
  <header id="header">Please wait — initializing robot stack…</header>

  <section id="mapPanel" aria-label="Live SLAM map">
    <canvas id="mapCanvas"></canvas>
  </section>

  <section id="controls" aria-label="Robot controls">
    <div id="controlStack">
      <div id="speedControl">
        <div id="speedHeader">
          <span>Speed</span>
          <span id="speedValue">50 RPM</span>
        </div>
        <input id="speedSlider" type="range" min="0" max="50" step="1" value="50" aria-label="Speed in RPM" disabled>
      </div>

      <div id="grid">
        <div id="lidarHealth" aria-live="polite">
          <span id="lidarHealthLabel">Lidar gap</span>
          <span id="lidarHealthValue">0.00 s</span>
        </div>
        <button id="w" aria-label="Forward" disabled>▲</button>
        <div id="motorSpeed" aria-live="polite">
          <span id="motorSpeedNow">0.0 RPM</span>
          <span id="motorSpeedAvg">Avg 0.0 RPM</span>
        </div>

        <button id="a" aria-label="Spin left" disabled>◀</button>
        <button id="reset" aria-label="Reset map" disabled>RESET</button>
        <button id="d" aria-label="Spin right" disabled>▶</button>

        <div></div>
        <button id="s" aria-label="Reverse" disabled>▼</button>
        <div></div>
      </div>
    </div>
  </section>
</main>

<script>
const mapPanel = document.getElementById("mapPanel");
const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const header = document.getElementById("header");
const speedSlider = document.getElementById("speedSlider");
const driveButtons = ["w", "a", "s", "d", "reset"]
  .map((key) => document.getElementById(key));
const speedValue = document.getElementById("speedValue");
const lidarHealth = document.getElementById("lidarHealth");
const lidarHealthValue = document.getElementById("lidarHealthValue");
const motorSpeedNow = document.getElementById("motorSpeedNow");
const motorSpeedAvg = document.getElementById("motorSpeedAvg");
const mapPixels = document.createElement("canvas");
const mapPixelsCtx = mapPixels.getContext("2d");
const mapLayer = document.createElement("canvas");
const mapLayerCtx = mapLayer.getContext("2d");
const MAP_POLL_MS = 250;
const POSE_POLL_MS = 50;
const MOTOR_AVG_WINDOW_MS = 5000;
const STARTUP_POLL_MS = 500;

let latestMap = null;
let trail = [];
let trailVersion = 0;
let pressedButtons = new Map();
let safetyLock = false;
let commandInterval = null;
let mapRequestInFlight = false;
let poseRequestInFlight = false;
let mapLayerKey = null;
let motorSpeedSamples = [];
let controlsReady = false;
let startupRequestInFlight = false;

function selectedSpeedRpm() {
  return Number(speedSlider.value);
}

function updateSpeedLabel() {
  speedValue.textContent = selectedSpeedRpm() + " RPM";
}

function setStatus(text) {
  if (!controlsReady && !text.startsWith("Stopping")) return;
  header.textContent = text;
}

function setControlsReady(ready) {
  controlsReady = Boolean(ready);
  for (const button of driveButtons) button.disabled = !controlsReady;
  speedSlider.disabled = !controlsReady;
}

function startupText(pending) {
  if (!pending.length) return "Please wait — initializing robot stack…";
  return "Please wait — initializing " + pending.join(", ") + "…";
}

async function updateStartup() {
  if (startupRequestInFlight) return;
  startupRequestInFlight = true;
  try {
    const response = await fetch("/status?ts=" + Date.now(), {cache: "no-store"});
    const status = await response.json();
    const startup = status.startup || {};
    const pending = Array.isArray(startup.pending) ? startup.pending : [];
    if (startup.ready === true) {
      if (!controlsReady) {
        setControlsReady(true);
        header.textContent = "Robot ready — controls enabled";
      }
    } else {
      setControlsReady(false);
      header.textContent = startupText(pending);
    }
  } catch (error) {
    setControlsReady(false);
    header.textContent = "Please wait — connecting to robot stack…";
  } finally {
    startupRequestInFlight = false;
    window.setTimeout(updateStartup, STARTUP_POLL_MS);
  }
}

async function send(key, speedRpm = selectedSpeedRpm()) {
  if (!controlsReady && key !== "x") return;
  try {
    await fetch(
      "/cmd?key=" + encodeURIComponent(key) + "&speed=" + encodeURIComponent(speedRpm),
      {
      cache: "no-store",
      credentials: "same-origin"
      }
    );
  } catch (error) {
    // The controller's command timeout stops the robot if the phone loses
    // connectivity; avoid putting noisy errors on the control screen.
  }
}

function currentMotionKey() {
  return ["w", "s", "a", "d"]
    .filter((key) => pressedButtons.has(key))
    .join("") || "x";
}

function conflictingButtons(keys) {
  return (
    (keys.includes("w") && keys.includes("s")) ||
    (keys.includes("a") && keys.includes("d"))
  );
}

function refreshMotionCommand() {
  if (commandInterval !== null) {
    clearInterval(commandInterval);
    commandInterval = null;
  }

  const key = safetyLock ? "x" : currentMotionKey();
  const speed = key === "x" ? 0 : selectedSpeedRpm();
  send(key, speed);
  if (key !== "x") {
    commandInterval = setInterval(() => {
      const liveKey = safetyLock ? "x" : currentMotionKey();
      send(liveKey, liveKey === "x" ? 0 : selectedSpeedRpm());
    }, 100);
  }
}

function pressMotionButton(key, pointerId) {
  if (!controlsReady || pressedButtons.has(key)) return;
  pressedButtons.set(key, pointerId ?? null);
  const keys = Array.from(pressedButtons.keys());
  if (keys.length > 2 || conflictingButtons(keys)) {
    // A conflicting or over-capacity combination is a latched UI safety stop.
    // The user must release all motion buttons before motion can resume.
    safetyLock = true;
    setStatus("Safety stop — release the conflicting buttons");
  }
  document.getElementById(key).classList.add("active");
  refreshMotionCommand();
}

function releaseMotionButton(key, pointerId = null) {
  if (!pressedButtons.has(key)) return;
  const owner = pressedButtons.get(key);
  if (pointerId !== null && owner !== null && owner !== pointerId) return;
  pressedButtons.delete(key);
  document.getElementById(key).classList.remove("active");

  // Do not resume a remaining button after an invalid combination.  Require
  // all buttons to be released so the next command is an intentional one.
  if (safetyLock && pressedButtons.size === 0) safetyLock = false;
  refreshMotionCommand();
}

function stop(sendStop = true) {
  for (const key of ["w", "s", "a", "d"]) {
    document.getElementById(key).classList.remove("active");
  }
  pressedButtons.clear();
  safetyLock = false;
  if (commandInterval !== null) {
    clearInterval(commandInterval);
    commandInterval = null;
  }
  if (sendStop) send("x", 0);
}

speedSlider.addEventListener("input", () => {
  updateSpeedLabel();
  if (pressedButtons.size && !safetyLock) refreshMotionCommand();
});
updateSpeedLabel();
setControlsReady(false);

for (const key of ["w", "a", "s", "d"]) {
  const button = document.getElementById(key);

  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    pressMotionButton(key, event.pointerId);
  });

  button.addEventListener("pointerup", (event) => {
    event.preventDefault();
    releaseMotionButton(key, event.pointerId);
  });

  button.addEventListener("pointercancel", (event) => {
    event.preventDefault();
    releaseMotionButton(key, event.pointerId);
  });

  // Pointer capture makes this reliable when the finger leaves the button.
  button.addEventListener("lostpointercapture", () => {
    releaseMotionButton(key);
  });
}

document.getElementById("reset").addEventListener("pointerdown", async (event) => {
  event.preventDefault();
  stop(true);
  setStatus("Resetting map…");
  try {
    const response = await fetch("/reset?source=phone", {
      cache: "no-store",
      credentials: "same-origin"
    });
    const result = await response.json();
    if (!response.ok || result.ok !== true) {
      setStatus(result.message || "Map reset failed");
      return;
    }
    // Reinitialize the lightweight phone UI without restarting ROS nodes.
    window.location.reload();
  } catch (error) {
    setStatus("Map reset unavailable");
  }
});

window.addEventListener("blur", () => stop(true));
window.addEventListener("beforeunload", () => stop(true));

function mapToCanvas(point, originX, originY, resolution, height, scale, dx, dy) {
  const mapX = (point.x - originX) / resolution;
  const mapY = (point.y - originY) / resolution;
  return {
    x: dx + mapX * scale,
    y: dy + (height - mapY) * scale
  };
}

function drawPath(path, originX, originY, resolution, height, scale, dx, dy) {
  if (!Array.isArray(path) || path.length < 2) return;

  ctx.save();
  ctx.beginPath();
  const stride = Math.max(1, Math.ceil(path.length / 1500));
  for (let i = 0; i < path.length; i += stride) {
    const point = mapToCanvas(path[i], originX, originY, resolution, height, scale, dx, dy);
    if (i === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  }
  const last = path[path.length - 1];
  const lastPoint = mapToCanvas(last, originX, originY, resolution, height, scale, dx, dy);
  ctx.lineTo(lastPoint.x, lastPoint.y);
  ctx.strokeStyle = "#168cff";
  ctx.lineWidth = 2.5;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.stroke();
  ctx.restore();
}

function drawRobot(robot, originX, originY, resolution, width, height, scale, dx, dy) {
  if (!robot || !Number.isFinite(robot.x) || !Number.isFinite(robot.y)) return;

  const point = mapToCanvas(robot, originX, originY, resolution, height, scale, dx, dy);
  if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) return;

  const x = point.x;
  const y = point.y;
  const size = Math.max(7, Math.min(18, 10 + scale * 0.12));

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-robot.yaw);
  ctx.fillStyle = "#e53935";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.8, -size * 0.65);
  ctx.lineTo(-size * 0.45, 0);
  ctx.lineTo(-size * 0.8, size * 0.65);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function buildMapPixels(map, width, height) {
  mapPixels.width = width;
  mapPixels.height = height;
  const image = mapPixelsCtx.createImageData(width, height);
  const data = map.data;
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const value = Number(data[y * width + x]);
      let color;
      if (value < 0) {
        color = 105;              // unknown
      } else if (value === 0) {
        color = 245;              // free
      } else {
        color = Math.max(20, 245 - Math.round(225 * Math.min(100, value) / 100));
      }
      const dstY = height - 1 - y;
      const index = (dstY * width + x) * 4;
      image.data[index] = color;
      image.data[index + 1] = color;
      image.data[index + 2] = color;
      image.data[index + 3] = 255;
    }
  }
  mapPixelsCtx.putImageData(image, 0, 0);
  mapPixels.dataset.version = String(map.version);
}

function renderMap(map) {
  if (!map || !map.ok) return;

  const width = Number(map.width);
  const height = Number(map.height);
  const resolution = Number(map.resolution);
  const data = map.data;
  if (!Number.isInteger(width) || !Number.isInteger(height) ||
      width <= 0 || height <= 0 || !Number.isFinite(resolution) ||
      resolution <= 0 || !Array.isArray(data) || data.length !== width * height) {
    setStatus("Received an invalid /map message");
    return;
  }

  const panelRect = mapPanel.getBoundingClientRect();
  const cssWidth = Math.max(1, Math.floor(panelRect.width));
  const cssHeight = Math.max(1, Math.floor(panelRect.height));
  const dpr = Math.min(2, Math.max(1, window.devicePixelRatio || 1));
  const pixelWidth = Math.max(1, Math.floor(cssWidth * dpr));
  const pixelHeight = Math.max(1, Math.floor(cssHeight * dpr));
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  canvas.style.width = cssWidth + "px";
  canvas.style.height = cssHeight + "px";

  // Rebuild the native map image only when the SLAM map version changes.
  if (mapPixels.width !== width || mapPixels.height !== height ||
      String(map.version) !== mapPixels.dataset.version) {
    buildMapPixels(map, width, height);
  }

  const padding = 12;
  const scale = Math.min(
    (cssWidth - 2 * padding) / width,
    (cssHeight - 2 * padding) / height
  );
  const drawWidth = Math.max(1, width * scale);
  const drawHeight = Math.max(1, height * scale);
  const dx = (cssWidth - drawWidth) / 2;
  const dy = (cssHeight - drawHeight) / 2;

  // Cache the fitted map layer. Pose/path updates redraw only this cached
  // layer plus the small vector overlays, rather than rescaling the whole
  // occupancy image on every pose update.
  const nextMapLayerKey = `${map.version}|${cssWidth}|${cssHeight}`;
  if (mapLayerKey !== nextMapLayerKey) {
    mapLayer.width = cssWidth;
    mapLayer.height = cssHeight;
    mapLayerCtx.setTransform(1, 0, 0, 1, 0, 0);
    mapLayerCtx.clearRect(0, 0, cssWidth, cssHeight);
    mapLayerCtx.fillStyle = "#292d34";
    mapLayerCtx.fillRect(0, 0, cssWidth, cssHeight);
    mapLayerCtx.imageSmoothingEnabled = false;
    mapLayerCtx.drawImage(mapPixels, dx, dy, drawWidth, drawHeight);
    mapLayerCtx.strokeStyle = "#6d7480";
    mapLayerCtx.lineWidth = 1;
    mapLayerCtx.strokeRect(dx, dy, drawWidth, drawHeight);
    mapLayerKey = nextMapLayerKey;
  }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssWidth, cssHeight);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapLayer, 0, 0, cssWidth, cssHeight);

  drawPath(
    trail,
    Number(map.origin_x),
    Number(map.origin_y),
    resolution,
    height,
    scale,
    dx,
    dy
  );

  drawRobot(
    map.robot,
    Number(map.origin_x),
    Number(map.origin_y),
    resolution,
    width,
    height,
    scale,
    dx,
    dy
  );

  const mapWidthM = width * resolution;
  const mapHeightM = height * resolution;
  const pose = map.robot
    ? ` | pose ${map.robot.x.toFixed(2)}, ${map.robot.y.toFixed(2)} | trail ${trail.length} pts`
    : "";
  setStatus(`${width}×${height} cells | ${mapWidthM.toFixed(2)}×${mapHeightM.toFixed(2)} m | ${resolution.toFixed(3)} m/cell${pose}`);
}

async function updateMap() {
  if (mapRequestInFlight) return;
  mapRequestInFlight = true;
  try {
    const knownVersion = latestMap ? latestMap.version : 0;
    const response = await fetch(
      "/map.json?version=" + encodeURIComponent(knownVersion) + "&ts=" + Date.now(),
      {cache: "no-store"}
    );
    const map = await response.json();
    if (map.unchanged) {
      // The cached fitted layer remains valid; only pose/path overlays need
      // updating through the lightweight /pose.json endpoint.
    } else if (map.ok) {
      // A new OccupancyGrid does not carry the robot pose.  Keep the last
      // valid pose attached while the separate pose request catches up.
      const lastRobot = latestMap && latestMap.robot ? latestMap.robot : null;
      latestMap = map;
      if (!latestMap.robot && lastRobot) latestMap.robot = lastRobot;
      renderMap(map);
    } else {
      setStatus(map.status || "Waiting for /map from slam_toolbox...");
    }
  } catch (error) {
    setStatus("Map connection unavailable");
  } finally {
    mapRequestInFlight = false;
    window.setTimeout(updateMap, MAP_POLL_MS);
  }
}

async function updatePose() {
  if (poseRequestInFlight) return;
  poseRequestInFlight = true;
  try {
    const response = await fetch(
      "/pose.json?path_from=" + encodeURIComponent(trailVersion) + "&ts=" + Date.now(),
      {cache: "no-store"}
    );
    const pose = await response.json();
    updateLidarHealth(pose.lidar);
    updateMotorSpeed(pose.motor_rpm);
    const points = Array.isArray(pose.path_points) ? pose.path_points : [];
    if (pose.path_reset) trail = points;
    else trail = trail.concat(points);
    if (Number.isInteger(pose.path_version)) trailVersion = pose.path_version;
    if (latestMap && latestMap.ok && pose.robot) {
      // Never erase a good pose because a transient TF lookup returned no
      // transform.  Keep displaying the last known pose until a newer one
      // arrives.
      latestMap.robot = pose.robot;
      renderMap(latestMap);
    }
  } catch (error) {
    // The map remains visible while a transient pose request fails.
  } finally {
    poseRequestInFlight = false;
    window.setTimeout(updatePose, POSE_POLL_MS);
  }
}

function updateLidarHealth(lidar) {
  // Show the currently active outage, not historical recovered gaps.  The
  // cumulative total remains available in the /status diagnostics.
  const gapSeconds = Number(lidar && lidar.current_gap_seconds);
  const safeGapSeconds = Number.isFinite(gapSeconds) ? Math.max(0, gapSeconds) : 0;
  lidarHealthValue.textContent = safeGapSeconds.toFixed(2) + " s";
  lidarHealth.classList.toggle("fault", safeGapSeconds > 0);
}

function updateMotorSpeed(rpm) {
  const left = Number(rpm && rpm.left);
  const right = Number(rpm && rpm.right);
  const leftAbs = Number.isFinite(left) ? Math.abs(left) : 0;
  const rightAbs = Number.isFinite(right) ? Math.abs(right) : 0;
  const actual = (leftAbs + rightAbs) / 2.0;
  const now = performance.now();
  motorSpeedSamples.push({now, rpm: actual});
  const cutoff = now - MOTOR_AVG_WINDOW_MS;
  while (motorSpeedSamples.length && motorSpeedSamples[0].now < cutoff) {
    motorSpeedSamples.shift();
  }
  let total = 0;
  for (const sample of motorSpeedSamples) total += sample.rpm;
  const average = motorSpeedSamples.length
    ? total / motorSpeedSamples.length
    : 0;
  motorSpeedNow.textContent = actual.toFixed(1) + " RPM";
  motorSpeedAvg.textContent = "Avg " + average.toFixed(1) + " RPM";
}

window.addEventListener("resize", () => {
  if (latestMap) renderMap(latestMap);
});

updateMap();
updatePose();
updateStartup();
</script>
</body>
</html>
"""


def rpm_to_mps(rpm):
    return (2.0 * math.pi * WHEEL_RADIUS) * (rpm / 60.0)


def make_twist(v=0.0, w=0.0):
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    return msg


class SharedCommand:
    MOTION_KEYS = {"w", "s", "a", "d"}

    def __init__(self):
        self.lock = threading.Lock()
        self.key = "x"
        self.speed_rpm = FORWARD_RPM
        self.last_update = time.monotonic()
        self.rest_until = 0.0

    @classmethod
    def normalize_key(cls, key):
        """Allow at most one key per axis and canonicalize valid pairs."""
        raw = str(key or "")
        if raw == "x":
            return "x"
        if any(char not in cls.MOTION_KEYS for char in raw):
            return "x"
        if len(raw) == 0 or len(raw) > 2 or len(set(raw)) != len(raw):
            return "x"
        if ("w" in raw and "s" in raw) or ("a" in raw and "d" in raw):
            return "x"
        return "".join(key_name for key_name in ("w", "s", "a", "d") if key_name in raw)

    @staticmethod
    def motion_axes(key):
        linear = 1 if "w" in key else -1 if "s" in key else 0
        turn = 1 if "a" in key else -1 if "d" in key else 0
        return linear, turn

    def set_key(self, key, speed_rpm=None):
        now = time.monotonic()
        key = self.normalize_key(key)

        parsed_speed = None
        if speed_rpm is not None:
            try:
                parsed_speed = float(speed_rpm)
            except (TypeError, ValueError):
                parsed_speed = None
            if parsed_speed is not None and not math.isfinite(parsed_speed):
                parsed_speed = None

        with self.lock:
            old_key = self.key
            old_linear, old_turn = self.motion_axes(old_key)
            new_linear, new_turn = self.motion_axes(key)
            changing_motion = (
                old_key != "x"
                and key != "x"
                and (
                    (old_linear and new_linear and old_linear != new_linear)
                    or (old_turn and new_turn and old_turn != new_turn)
                )
            )
            if changing_motion:
                self.rest_until = now + TRANSITION_REST_S
                print(f"Transition {old_key} -> {key}: rest {TRANSITION_REST_S:.2f}s")
            self.key = key
            if parsed_speed is not None:
                self.speed_rpm = max(0.0, min(FORWARD_RPM, parsed_speed))
            self.last_update = now

    def status(self):
        with self.lock:
            return self.key, self.speed_rpm, self.last_update

    def get_twist(self):
        now = time.monotonic()
        with self.lock:
            key = self.key
            speed_rpm = self.speed_rpm
            last_update = self.last_update
            rest_until = self.rest_until

        if now < rest_until or now - last_update > COMMAND_TIMEOUT_S:
            return make_twist()

        speed_fraction = speed_rpm / FORWARD_RPM if FORWARD_RPM else 0.0
        v = rpm_to_mps(speed_rpm)
        linear_sign, turn_sign = self.motion_axes(key)
        return make_twist(
            linear_sign * v,
            turn_sign * SPIN_MAX_RADPS * speed_fraction,
        )


class LiveMapState:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_json_lock = threading.Lock()
        self.map = None
        self.map_json_body = None
        self.map_json_version = None
        self.robot = None
        self.path = []
        self.path_base_version = 0
        self.path_version = 0
        self.version = 0
        self.motor_rpm = {"left": 0.0, "right": 0.0}
        # Do not count lidar startup or pre-map delays. Monitoring begins only
        # when the first valid OccupancyGrid has arrived.
        self.lidar_monitor_started_at = None
        self.lidar_last_seen_at = None
        self.lidar_last_scan_at = None
        self.lidar_completed_gap_s = 0.0
        self.lidar_max_gap_s = 0.0
        self.lidar_gap_events = 0

    def update_map(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return

        # Copy the ROS message data before taking the shared-state lock.  The
        # copy can be large; pose, odometry, and scan callbacks must not wait
        # behind it while the map is being replaced.
        map_data = list(msg.data)
        with self.lock:
            if self.lidar_monitor_started_at is None:
                self.lidar_monitor_started_at = time.monotonic()
            self.version += 1
            self.map = {
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": map_data,
                "version": self.version,
            }
            self.map_json_body = None
            self.map_json_version = None

    def update_robot(self, transform):
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": float(yaw),
        }
        with self.lock:
            if self.robot is None:
                self.path = [pose]
                self.path_base_version = 0
                self.path_version = 1
            else:
                dx = pose["x"] - self.robot["x"]
                dy = pose["y"] - self.robot["y"]
                # Keep a useful trail without storing hundreds of identical
                # points while the robot is stationary.
                if dx * dx + dy * dy >= 0.001 ** 2:
                    self.path.append(pose)
                    self.path_version += 1
                    if len(self.path) > 5000:
                        dropped = len(self.path) - 5000
                        self.path = self.path[-5000:]
                        self.path_base_version += dropped
            self.robot = pose

    def update_scan(self):
        now = time.monotonic()
        with self.lock:
            # Keep a separate startup/health timestamp even before the first
            # map exists.  Pre-map scans must not count as outage time.
            self.lidar_last_seen_at = now
            if self.lidar_monitor_started_at is None:
                return
            previous = (
                self.lidar_last_scan_at
                if self.lidar_last_scan_at is not None
                else self.lidar_monitor_started_at
            )
            gap = max(0.0, now - previous)
            if gap > LIDAR_GAP_THRESHOLD_S:
                self.lidar_completed_gap_s += gap
                self.lidar_max_gap_s = max(self.lidar_max_gap_s, gap)
                self.lidar_gap_events += 1
            self.lidar_last_scan_at = now

    def lidar_is_ready(self, timeout_s=1.0):
        with self.lock:
            return (
                self.lidar_last_seen_at is not None
                and time.monotonic() - self.lidar_last_seen_at <= timeout_s
            )

    def reset_for_new_map(self):
        """Clear phone-side map/pose state after slam_toolbox resets."""
        with self.lock:
            self.map = None
            self.map_json_body = None
            self.map_json_version = None
            self.robot = None
            self.path = []
            self.path_base_version = 0
            self.path_version = 0
            self.version = 0
            self.motor_rpm = {"left": 0.0, "right": 0.0}
            self.lidar_monitor_started_at = None
            self.lidar_last_seen_at = None
            self.lidar_last_scan_at = None
            self.lidar_completed_gap_s = 0.0
            self.lidar_max_gap_s = 0.0
            self.lidar_gap_events = 0

    def update_odom(self, msg):
        v = float(msg.twist.twist.linear.x)
        omega = float(msg.twist.twist.angular.z)
        v_left = v - omega * WHEEL_SEPARATION / 2.0
        v_right = v + omega * WHEEL_SEPARATION / 2.0
        meters_per_revolution = 2.0 * math.pi * WHEEL_RADIUS
        left_rpm = v_left / meters_per_revolution * 60.0
        right_rpm = v_right / meters_per_revolution * 60.0
        with self.lock:
            self.motor_rpm = {"left": left_rpm, "right": right_rpm}

    def _lidar_snapshot_locked(self, now):
        if self.lidar_monitor_started_at is None:
            return {
                "gap_seconds": 0.0,
                "current_gap_seconds": 0.0,
                "events": 0,
                "max_gap_seconds": 0.0,
                "healthy": True,
            }
        previous = (
            self.lidar_last_scan_at
            if self.lidar_last_scan_at is not None
            else self.lidar_monitor_started_at
        )
        current_gap = max(0.0, now - previous)
        active_gap = current_gap > LIDAR_GAP_THRESHOLD_S
        total_gap = self.lidar_completed_gap_s + (current_gap if active_gap else 0.0)
        return {
            "gap_seconds": total_gap,
            "current_gap_seconds": current_gap if active_gap else 0.0,
            "events": self.lidar_gap_events,
            "max_gap_seconds": self.lidar_max_gap_s,
            "healthy": not active_gap,
        }

    def lidar_snapshot(self):
        with self.lock:
            return self._lidar_snapshot_locked(time.monotonic())

    def map_body(self):
        with self.map_json_lock:
            with self.lock:
                if self.map is None:
                    return None
                if self.map_json_body is not None and self.map_json_version == self.version:
                    return self.map_json_body
            payload = self.display_map_payload(self.map)

            # Serialize only once per new SLAM map, in the HTTP worker rather
            # than inside the ROS callback/control loop.
            body = json.dumps(
                {"ok": True, **payload}, separators=(",", ":")
            ).encode("utf-8")
            with self.lock:
                if self.version == payload["version"]:
                    self.map_json_body = body
                    self.map_json_version = self.version
            return body

    @staticmethod
    def display_map_payload(map_record):
        width = map_record["width"]
        height = map_record["height"]
        source = map_record["data"]
        scale = max(
            1,
            math.ceil(max(width, height) / MAX_MAP_DISPLAY_DIM),
        )

        if scale == 1:
            return dict(map_record)

        display_width = math.ceil(width / scale)
        display_height = math.ceil(height / scale)
        display_data = []
        for display_y in range(display_height):
            source_y0 = display_y * scale
            source_y1 = min(height, source_y0 + scale)
            for display_x in range(display_width):
                source_x0 = display_x * scale
                source_x1 = min(width, source_x0 + scale)
                maximum_occupied = 0
                has_unknown = False
                for source_y in range(source_y0, source_y1):
                    row_start = source_y * width
                    for source_x in range(source_x0, source_x1):
                        value = int(source[row_start + source_x])
                        if value < 0:
                            has_unknown = True
                        elif value > maximum_occupied:
                            maximum_occupied = value

                if maximum_occupied >= 50:
                    display_data.append(maximum_occupied)
                elif has_unknown:
                    display_data.append(-1)
                else:
                    display_data.append(maximum_occupied)

        return {
            "width": display_width,
            "height": display_height,
            "resolution": map_record["resolution"] * scale,
            "origin_x": map_record["origin_x"],
            "origin_y": map_record["origin_y"],
            "data": display_data,
            "version": map_record["version"],
            "display_scale": scale,
        }

    def pose_snapshot(self, requested_path_version):
        with self.lock:
            requested_path_version = max(0, int(requested_path_version))
            reset = requested_path_version < self.path_base_version
            if reset:
                points = list(self.path)
            else:
                start = requested_path_version - self.path_base_version
                points = list(self.path[max(0, start):])
            return {
                "ok": self.map is not None,
                "robot": self.robot,
                "path_reset": reset,
                "path_base_version": self.path_base_version,
                "path_version": self.path_version,
                "path_points": points,
                "map_version": self.version,
                "lidar": self._lidar_snapshot_locked(time.monotonic()),
                "motor_rpm": dict(self.motor_rpm),
            }

    def has_map(self):
        with self.lock:
            return self.map is not None

    def version_number(self):
        with self.lock:
            return self.version

    def map_snapshot_for_save(self):
        """Return a detached map snapshot for the background file writer."""
        with self.lock:
            if self.map is None:
                return None
            record = self.map
            data = list(record["data"])
            return {
                "width": record["width"],
                "height": record["height"],
                "resolution": record["resolution"],
                "origin_x": record["origin_x"],
                "origin_y": record["origin_y"],
                "data": data,
                "version": record["version"],
            }


class MapCheckpointSaver:
    """Persist the current OccupancyGrid without blocking ROS callbacks."""

    def __init__(self, map_state):
        self.map_state = map_state
        self.save_dir = MAP_SAVE_DIR
        self.stage_dir = os.path.join(self.save_dir, ".checkpoint_staging")
        os.makedirs(self.stage_dir, exist_ok=True)
        self.stop_event = threading.Event()
        self.save_lock = threading.Lock()
        self.thread = None
        self.last_saved_at = None
        self.last_error = None

    def start(self):
        self.thread = threading.Thread(
            target=self._run,
            name="map_checkpoint_saver",
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        next_save = time.monotonic()
        while not self.stop_event.is_set():
            if self.map_state.has_map():
                self.save_now()
            next_save += MAP_CHECKPOINT_INTERVAL_S
            wait_s = max(0.0, next_save - time.monotonic())
            if self.stop_event.wait(wait_s):
                break

    def save_now(self):
        with PHONE_DIAGNOSTICS.measure("map_checkpoint_save"):
            return self._save_now()

    def _save_now(self):
        """Save one complete checkpoint; return true only on success."""
        snapshot = self.map_state.map_snapshot_for_save()
        if snapshot is None:
            return False

        with self.save_lock:
            snapshot = self.map_state.map_snapshot_for_save()
            if snapshot is None:
                return False

            stage_base = os.path.join(self.stage_dir, "recovery_map")
            stage_yaml = f"{stage_base}.yaml"
            stage_pgm = f"{stage_base}.pgm"
            final_base = os.path.join(self.save_dir, "recovery_map")
            final_yaml = f"{final_base}.yaml"
            final_pgm = f"{final_base}.pgm"

            try:
                width = int(snapshot["width"])
                height = int(snapshot["height"])
                data = snapshot["data"]
                if width <= 0 or height <= 0 or len(data) != width * height:
                    raise RuntimeError("cached map dimensions are invalid")

                with open(stage_pgm, "wb") as image_file:
                    image_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
                    # OccupancyGrid row zero is the bottom row; PGM row zero
                    # is the top row.  Write rows in reverse Y order.
                    for y in range(height - 1, -1, -1):
                        row = data[y * width : (y + 1) * width]
                        pixels = bytes(
                            205 if int(value) < 0
                            else 0 if int(value) >= 65
                            else 254 if int(value) <= 25
                            else 205
                            for value in row
                        )
                        image_file.write(pixels)

                with open(stage_yaml, "w", encoding="ascii") as metadata_file:
                    metadata_file.write(
                        "image: recovery_map.pgm\n"
                        "mode: trinary\n"
                        f"resolution: {float(snapshot['resolution']):.3f}\n"
                        f"origin: [{float(snapshot['origin_x']):.6f}, "
                        f"{float(snapshot['origin_y']):.6f}, 0]\n"
                        "negate: 0\n"
                        "occupied_thresh: 0.65\n"
                        "free_thresh: 0.196\n"
                    )

                # The staged YAML refers to recovery_map.pgm, so the pair is
                # self-consistent after both files are moved into place.
                os.replace(stage_pgm, final_pgm)
                os.replace(stage_yaml, final_yaml)
                self.last_saved_at = time.time()
                self.last_error = None
                print(
                    "Map checkpoint saved: ~/robot1_maps/recovery_map.yaml "
                    f"(version {snapshot['version']})"
                )
                return True
            except (OSError, RuntimeError, ValueError) as exc:
                self.last_error = str(exc)
                print(f"Map checkpoint warning: {exc}")
                return False

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=MAP_SAVE_TIMEOUT_S + 2.0)
            self.thread = None


class PhoneMapNode(Node):
    def __init__(self, map_state):
        super().__init__("phone_4button_controller")
        self.map_state = map_state
        self.fast_callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos)
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            odom_qos,
            callback_group=self.fast_callback_group,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            scan_qos,
            callback_group=self.fast_callback_group,
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.reset_client = self.create_client(SlamReset, "/slam_toolbox/reset")
        self.create_timer(
            0.05,
            self.update_robot_pose,
            callback_group=self.fast_callback_group,
        )

    def reset_slam(self):
        """Reset slam_toolbox in place and return (success, message)."""
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            return False, "slam_toolbox reset service is unavailable"

        request = SlamReset.Request()
        request.pause_new_measurements = False
        future = self.reset_client.call_async(request)
        deadline = time.monotonic() + 4.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return False, "slam_toolbox reset timed out"

        try:
            response = future.result()
        except Exception as exc:
            return False, f"slam_toolbox reset failed: {exc}"
        if response.result != SlamReset.Response.RESULT_SUCCESS:
            return False, f"slam_toolbox reset returned code {response.result}"
        return True, "Map reset"

    def map_callback(self, msg):
        detail = f"width={msg.info.width},height={msg.info.height}"
        with PHONE_DIAGNOSTICS.measure("map_callback", detail):
            self.map_state.update_map(msg)

    def scan_callback(self, _msg):
        with PHONE_DIAGNOSTICS.measure("scan_callback"):
            self.map_state.update_scan()

    def odom_callback(self, msg):
        with PHONE_DIAGNOSTICS.measure("odom_callback"):
            self.map_state.update_odom(msg)

    def update_robot_pose(self):
        with PHONE_DIAGNOSTICS.measure("pose_timer"):
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                self.map_state.update_robot(transform)
            except Exception:
                # The map can arrive before map -> base_link TF is available.
                pass


def make_handler(shared, map_state, shutdown_event, supervisor, node):
    class Handler(BaseHTTPRequestHandler):
        server_version = "Robot1Phone/1.0"

        def log_message(self, fmt, *args):
            return

        def send_body(self, body, content_type, status=200):
            started = getattr(self, "_phone_request_started", None)
            path = getattr(self, "_phone_request_path", "unknown")
            if started is not None:
                PHONE_DIAGNOSTICS.record(
                    "http_request",
                    "end",
                    duration_ms=(time.monotonic() - started) * 1000.0,
                    detail=f"path={path},status={status},bytes={len(body)}",
                )
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            self._phone_request_started = time.monotonic()
            self._phone_request_path = parsed.path
            PHONE_DIAGNOSTICS.record(
                "http_request", "start", detail=f"path={parsed.path}"
            )

            if parsed.path == "/":
                self.send_body(HTML.encode("utf-8"), "text/html; charset=utf-8")
                return

            if parsed.path == "/cmd":
                query = parse_qs(parsed.query)
                key = query.get("key", ["x"])[0]
                speed = query.get("speed", [None])[0]
                shared.set_key(key, speed)
                self.send_body(b"OK\n", "text/plain; charset=utf-8")
                return

            if parsed.path == "/shutdown":
                shared.set_key("x", 0)
                shutdown_event.set()
                self.send_body(b"SHUTTING DOWN\n", "text/plain; charset=utf-8")
                return

            if parsed.path == "/reset":
                shared.set_key("x", 0)
                success, message = node.reset_slam()
                if success:
                    map_state.reset_for_new_map()
                    body = json.dumps({"ok": True, "message": message}).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8")
                else:
                    body = json.dumps({"ok": False, "message": message}).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8", 503)
                return

            if parsed.path == "/map.json":
                try:
                    known_version = int(parse_qs(parsed.query).get("version", ["0"])[0])
                except (TypeError, ValueError):
                    known_version = 0
                current_version = map_state.version_number()
                if current_version > 0 and known_version == current_version:
                    body = json.dumps({
                        "ok": True,
                        "unchanged": True,
                        "version": current_version,
                    }).encode("utf-8")
                    self.send_body(body, "application/json; charset=utf-8")
                    return
                body = map_state.map_body()
                if body is None:
                    body = json.dumps({
                        "ok": False,
                        "status": "Waiting for /map from slam_toolbox...",
                    }).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            if parsed.path == "/pose.json":
                try:
                    requested = int(parse_qs(parsed.query).get("path_from", ["0"])[0])
                except (TypeError, ValueError):
                    requested = 0
                body = json.dumps(
                    map_state.pose_snapshot(requested),
                    separators=(",", ":"),
                ).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            if parsed.path == "/status":
                key, speed_rpm, last_update = shared.status()
                lidar = map_state.lidar_snapshot()
                body = json.dumps({
                    "key": key,
                    "speed_rpm": speed_rpm,
                    "seconds_since_command": max(0.0, time.monotonic() - last_update),
                    "map": map_state.has_map(),
                    "lidar": lidar,
                    "startup": supervisor.startup_status(map_state),
                }).encode("utf-8")
                self.send_body(body, "application/json; charset=utf-8")
                return

            self.send_body(b"Not found\n", "text/plain; charset=utf-8", 404)

    return Handler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class RobotStackSupervisor:
    """Start missing real-robot support nodes without duplicating existing ones."""

    def __init__(self):
        self.children = []
        os.makedirs(STACK_LOG_DIR, exist_ok=True)

    @staticmethod
    def process_running(pattern):
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return result.returncode == 0
        except OSError:
            return False

    @staticmethod
    def scan_publisher_exists():
        """Return true only when ROS reports a publisher on /scan."""
        try:
            result = subprocess.run(
                ros_shell_command(["ros2", "topic", "info", "/scan"]),
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False

        for line in result.stdout.splitlines():
            if line.strip().lower().startswith("publisher count:"):
                try:
                    return int(line.split(":", 1)[1].strip()) > 0
                except (IndexError, ValueError):
                    return False
        return False

    def spawn(self, name, command):
        log_path = os.path.join(STACK_LOG_DIR, f"{name}.log")
        log_file = open(log_path, "ab", buffering=0)
        try:
            child = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            log_file.close()
            raise

        self.children.append((name, child, log_file))
        print(f"Started {name} (pid {child.pid}); log: {log_path}")
        return child

    def start(self):
        if START_LIDAR and not self.scan_publisher_exists() and not self.process_running("d500_ros2_scan.py"):
            self.spawn("lidar", ["bash", START_LIDAR_SCRIPT])
        elif self.scan_publisher_exists():
            print("Reusing existing /scan publisher.")
        else:
            print("Reusing existing lidar process; waiting for /scan.")

        if START_MOTOR and not self.process_running("real_diffdrive_node"):
            self.spawn(
                "motor",
                ros_shell_command(
                    ["ros2", "run", "my_epuck_project", "real_diffdrive_node"]
                ),
            )
        elif self.process_running("real_diffdrive_node"):
            print("Reusing existing real_diffdrive_node.")

        if not self.process_running("static_transform_publisher.*d500_lidar"):
            self.spawn(
                "lidar_tf",
                ros_shell_command(
                    [
                        "ros2",
                        "run",
                        "tf2_ros",
                        "static_transform_publisher",
                        "--x",
                        "0.0",
                        "--y",
                        "0.0",
                        "--z",
                        "0.07",
                        "--roll",
                        "0.0",
                        "--pitch",
                        "0.0",
                        "--yaw",
                        "0.0",
                        "--frame-id",
                        "base_link",
                        "--child-frame-id",
                        "d500_lidar",
                    ]
                ),
            )

        if not self.process_running("slam_toolbox"):
            self.spawn(
                "slam",
                ros_shell_command(
                    [
                        "ros2",
                        "launch",
                        "slam_toolbox",
                        "online_async_launch.py",
                        f"slam_params_file:={SLAM_PARAMS}",
                        "use_sim_time:=false",
                    ]
                ),
            )
        else:
            print("Reusing existing slam_toolbox.")

    def startup_status(self, map_state):
        """Return the components that must be ready before teleoperation."""
        pending = []
        if not self.process_running("real_diffdrive_node"):
            pending.append("motor node")
        if not map_state.lidar_is_ready():
            pending.append("lidar")
        if not self.process_running("static_transform_publisher.*d500_lidar"):
            pending.append("lidar TF")
        if not self.process_running("slam_toolbox"):
            pending.append("SLAM")
        if not map_state.has_map():
            pending.append("first map")
        return {
            "ready": not pending,
            "pending": pending,
        }

    def stop_owned_children(self):
        for name, child, log_file in reversed(self.children):
            if child.poll() is not None:
                log_file.close()
                continue
            print(f"Stopping {name} (pid {child.pid})...")
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

        deadline = time.monotonic() + 5.0
        for _, child, _ in self.children:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                child.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

        for _, child, log_file in self.children:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            log_file.close()

    def stop_all_stack_processes(self):
        """Stop this program's complete real-robot stack, including reused nodes."""
        self.stop_owned_children()

        # A previous run may have exited before its Popen handles reached the
        # cleanup code.  Kill the exact stack components so stale SLAM/map,
        # motor, TF, and lidar processes cannot survive into the next run.
        patterns = (
            "[s]tart_d500_with_recovery.sh",
            "[d]500_ros2_scan.py",
            "[r]eal_diffdrive_node",
            "[s]tatic_transform_publisher.*d500_lidar",
            "[s]lam_toolbox",
        )
        for sig in ("-INT", "-TERM", "-KILL"):
            for pattern in patterns:
                subprocess.run(
                    ["pkill", sig, "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            if sig != "-KILL":
                time.sleep(1.0)


def advertised_host():
    configured = os.environ.get("PHONE_CONTROLLER_HOST", "").strip()
    if configured:
        return configured

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # No packet is sent; this selects the address used for the LAN route.
        sock.connect(("192.0.2.1", 9))
        return sock.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())
    finally:
        sock.close()


def main():
    rclpy.init()
    shared = SharedCommand()
    map_state = LiveMapState()
    node = PhoneMapNode(map_state)
    supervisor = RobotStackSupervisor()
    map_saver = MapCheckpointSaver(map_state)
    shutdown_event = threading.Event()

    try:
        supervisor.start()
        map_saver.start()
        server = ReusableThreadingHTTPServer(
            ("0.0.0.0", HTTP_PORT),
            make_handler(shared, map_state, shutdown_event, supervisor, node),
        )
    except Exception as exc:
        map_saver.stop()
        supervisor.stop_owned_children()
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"Could not open phone-controller port {HTTP_PORT}: {exc}")

    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    # Keep ROS callbacks running continuously.  The old loop dispatched only
    # one callback every 50 ms, even though /tf and the sensor topics together
    # can deliver many more callbacks per second.  That made the pose timer
    # read stale map -> base_link transforms.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="phone_ros_executor",
        daemon=True,
    )
    executor_thread.start()

    print()
    print("Robot 1 phone controller with live map running.")
    print(f"Open on the phone: http://{advertised_host()}:{HTTP_PORT}/")
    print(f"Forward/back RPM: {FORWARD_RPM:.0f}")
    print(f"Forward speed:    {rpm_to_mps(FORWARD_RPM):.3f} m/s")
    print(f"Spin command:     {SPIN_MAX_RADPS:.3f} rad/s")
    print("Live map source:   /map and map -> base_link TF")
    print("Stack startup:     lidar, motor, lidar TF, and SLAM are supervised")
    print("Disable motor auto-start with: ROBOT1_PHONE_START_MOTOR=0")
    print("Press Ctrl+C to quit.")
    print()

    dt = 1.0 / PUB_RATE_HZ
    pub = node.create_publisher(Twist, "/cmd_vel_unstamped", 10)
    try:
        while rclpy.ok() and not shutdown_event.is_set():
            pub.publish(shared.get_twist())
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping robot...")
        shared.set_key("x")
        try:
            stop = make_twist()
            for _ in range(20):
                if not rclpy.ok():
                    break
                try:
                    pub.publish(stop)
                except Exception as exc:
                    print(f"Stop publish skipped: {exc}")
                    break
                time.sleep(0.02)
        finally:
            try:
                server.shutdown()
                server.server_close()
                http_thread.join(timeout=1.0)
            except Exception as exc:
                print(f"HTTP shutdown warning: {exc}")

            # This is deliberately before any final ROS teardown can prevent
            # cleanup from running. It also removes reused/orphaned nodes.
            map_saver.stop()
            print("Saving final map checkpoint...")
            map_saver.save_now()
            supervisor.stop_all_stack_processes()
            PHONE_DIAGNOSTICS.close()

            try:
                executor.shutdown(timeout_sec=1.0)
                executor_thread.join(timeout=1.0)
            except Exception as exc:
                print(f"ROS executor shutdown warning: {exc}")

            try:
                node.destroy_node()
            except Exception as exc:
                print(f"ROS node shutdown warning: {exc}")
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as exc:
                print(f"ROS context shutdown warning: {exc}")
        print("Stopped.")


if __name__ == "__main__":
    main()
