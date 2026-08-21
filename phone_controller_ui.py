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

    #odomModeControl,
    #spinSettings {
      display: flex;
      align-items: center;
      gap: 6px;
      margin: 0 3px 5px;
      color: var(--muted);
      font-size: 11px;
    }

    #odomModeControl select,
    #spinSettings select,
    #spinSettings input {
      min-width: 0;
      flex: 1 1 auto;
      border: 1px solid rgba(255, 255, 255, 0.18);
      border-radius: 7px;
      padding: 4px 5px;
      background: var(--button);
      color: var(--text);
      font-size: 11px;
    }

    #spinSettings input {
      flex: 0 0 48px;
      width: 48px;
    }

    #odomModeControl select:disabled,
    #spinSettings select:disabled,
    #spinSettings input:disabled {
      opacity: 0.5;
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

    #finished,
    #reset {
      background: var(--stop);
      font-size: clamp(16px, 4.5vw, 25px);
      width: 70%;
      height: 70%;
      justify-self: center;
      align-self: center;
      border-radius: 12px;
    }

    #finished {
      font-size: clamp(11px, 3.2vw, 18px);
    }

    #odomResult {
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
      font-size: clamp(8px, 2.15vw, 12px);
      line-height: 1.08;
      text-align: center;
      white-space: pre-line;
      overflow: hidden;
    }

    #odomResult.complete {
      color: var(--text);
      border-color: var(--button-active);
    }

    #odomResult.error {
      background: var(--stop);
      color: var(--text);
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
        <div id="odomModeControl">
          <span>Odom test</span>
          <select id="odomMode" aria-label="Odometry test mode" disabled>
            <option value="" selected disabled>SELECT MODE</option>
            <option value="CLOSED_LOOP">CLOSED LOOP</option>
            <option value="STRAIGHT_STOP">STRAIGHT / STOP</option>
            <option value="SPIN">SPIN</option>
          </select>
        </div>
        <div id="spinSettings" hidden>
          <span>Spin</span>
          <select id="spinDirection" aria-label="Spin direction">
            <option value="CW">CW</option>
            <option value="CCW">CCW</option>
          </select>
          <span>turns</span>
          <input id="spinTurns" type="number" min="1" max="10" step="1" value="5" aria-label="Expected full rotations">
        </div>
        <div id="speedHeader">
          <span>Cmd RPM</span>
          <span id="speedValue">50 RPM</span>
        </div>
        <input id="speedSlider" type="range" min="12" max="50" step="1" value="50" aria-label="Command wheel speed in RPM" disabled>
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
        <button id="finished" aria-label="Finish odometry test" disabled>FINISHED</button>
        <button id="d" aria-label="Spin right" disabled>▶</button>

        <button id="reset" aria-label="Reset map and odometry test" disabled>RESET</button>
        <button id="s" aria-label="Reverse" disabled>▼</button>
        <div id="odomResult" aria-live="polite">ODOM TEST
Waiting for odometry</div>
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
const motionButtons = ["w", "a", "s", "d"]
  .map((key) => document.getElementById(key));
const finishedButton = document.getElementById("finished");
const resetButton = document.getElementById("reset");
const odomResult = document.getElementById("odomResult");
const odomMode = document.getElementById("odomMode");
const spinSettings = document.getElementById("spinSettings");
const spinDirection = document.getElementById("spinDirection");
const spinTurns = document.getElementById("spinTurns");
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
let odomTestState = "WAITING_FOR_ODOM";
let odomTestMode = "CLOSED_LOOP";

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
  const motionAllowed = controlsReady && odomTestState === "RECORDING";
  for (const button of motionButtons) button.disabled = !motionAllowed;
  finishedButton.disabled = !motionAllowed;
  resetButton.disabled = !controlsReady || odomTestState === "FINALIZING";
  speedSlider.disabled = !motionAllowed;
  // Mode selection is safe before teleoperation is ready and must be
  // available during stack startup.  The odometry session still rejects
  // configuration after recording begins, so this does not mix modes.
  const configurationAllowed = odomTestState === "WAITING_FOR_ODOM";
  odomMode.disabled = !configurationAllowed;
  spinDirection.disabled = !configurationAllowed || odomTestMode !== "SPIN";
  spinTurns.disabled = !configurationAllowed || odomTestMode !== "SPIN";
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
    updateOdomTestStatus(status.odom_test);
    const pending = Array.isArray(startup.pending) ? startup.pending : [];
    if (startup.ready === true) {
      if (!controlsReady) {
        setControlsReady(true);
        header.textContent = "Robot ready — waiting for odometry";
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

function formatSigned(value, digits = 1) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  return (number >= 0 ? "+" : "") + number.toFixed(digits);
}

function modeLabel(mode) {
  if (mode === "") return "SELECT MODE";
  if (mode === "STRAIGHT_STOP") return "STRAIGHT / STOP";
  if (mode === "SPIN") return "SPIN";
  return "CLOSED LOOP";
}

function formatNumber(value, digits = 1, fallback = "—") {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : fallback;
}

async function configureOdomTest() {
  if (odomTestState !== "WAITING_FOR_ODOM") return;
  const turns = Math.max(1, Math.min(10, Math.round(Number(spinTurns.value) || 5)));
  spinTurns.value = String(turns);
  const query = new URLSearchParams({
    mode: odomMode.value,
    direction: spinDirection.value,
    turns: String(turns),
  });
  try {
    const response = await fetch("/odom_test/config?" + query.toString(), {
      cache: "no-store",
      credentials: "same-origin",
    });
    const status = await response.json();
    if (!response.ok) {
      setStatus(status.message || "Reset is required before changing mode");
      return;
    }
    updateOdomTestStatus(status);
  } catch (error) {
    setStatus("Odometry test mode unavailable");
  }
}

function updateOdomTestStatus(status) {
  if (!status || typeof status !== "object") return;
  odomTestState = String(status.state || "WAITING_FOR_ODOM");
  const modeConfigured = status.mode_configured === true;
  odomTestMode = modeConfigured ? String(status.mode || odomTestMode) : "";
  if (modeConfigured && ["CLOSED_LOOP", "STRAIGHT_STOP", "SPIN"].includes(odomTestMode)) {
    odomMode.value = odomTestMode;
  } else if (!modeConfigured) {
    odomMode.value = "";
  }
  if (status.spin_direction) spinDirection.value = status.spin_direction;
  if (Number.isFinite(Number(status.expected_turns))) {
    spinTurns.value = String(status.expected_turns);
  }
  spinSettings.hidden = !modeConfigured || odomTestMode !== "SPIN";
  const result = status.result || null;
  odomResult.classList.toggle("complete", odomTestState === "COMPLETE");
  odomResult.classList.toggle("error", odomTestState === "ERROR");

  if (odomTestState === "WAITING_FOR_ODOM") {
    const referenceReady = status.reference_ready === true;
    odomResult.textContent = !modeConfigured
      ? "ODOM TEST\nSelect a mode to begin"
      : referenceReady
      ? "ODOM TEST\nSelect mode\nStarting shortly"
      : "ODOM TEST\n" + modeLabel(odomTestMode) + "\nWaiting for odometry";
  } else if (odomTestState === "RECORDING") {
    odomResult.textContent = "ODOM TEST\n" + modeLabel(odomTestMode) + "\nRecording";
  } else if (odomTestState === "FINALIZING") {
    odomResult.textContent = "ODOM TEST\n" + modeLabel(odomTestMode) + "\nFinalizing";
  } else if (odomTestState === "COMPLETE" && result && result.status === "ok") {
    if (odomTestMode === "STRAIGHT_STOP") {
      odomResult.textContent =
        "STRAIGHT / STOP\n" +
        "Runs " + (result.valid_segment_count ?? 0) + "\n" +
        "Cruise " + formatSigned(result.mean_cruise_yaw_deg) + "°\n" +
        "Stop " + formatSigned(result.mean_stop_yaw_deg) + "°\n" +
        "Worst " + formatNumber(result.max_abs_stop_yaw_deg) + "°";
    } else if (odomTestMode === "SPIN") {
      odomResult.textContent =
        "SPIN\n" +
        "Err " + formatSigned(result.rotation_error_deg) + "°\n" +
        "Scale " + formatSigned(result.rotation_scale_error_percent, 2) + "%\n" +
        "B " + formatNumber(result.inferred_wheel_separation_mm, 1) + " mm";
    } else {
      const drift = result.endpoint_position_drift_percent === null
        ? "—"
        : Number(result.endpoint_position_drift_percent).toFixed(2) + "%";
      odomResult.textContent =
        "ODOM DRIFT\n" +
        "Pos " + Number(result.position_error_cm).toFixed(1) + " cm\n" +
        "Yaw " + formatSigned(result.yaw_error_deg) + "°\n" +
        "L " + Number(result.odom_estimated_path_length_m).toFixed(1) + " m\n" +
        "Drift " + drift;
    }
  } else if (odomTestState === "ERROR") {
    odomResult.textContent = "ODOM TEST\n" + String(status.message || "Error");
  }

  if (controlsReady) {
    if (odomTestState === "RECORDING") {
      header.textContent = "ODOM TEST — " + modeLabel(odomTestMode) + " — recording";
    } else if (odomTestState === "FINALIZING") {
      header.textContent = "ODOM TEST — stopping and analyzing…";
    } else if (odomTestState === "COMPLETE") {
      header.textContent = "ODOM TEST complete — press RESET for another run";
    } else if (odomTestState === "ERROR") {
      header.textContent = "ODOM TEST error — press RESET to start again";
    }
    const comparison = status.comparison || null;
    if (comparison && comparison.state === "PROCESSING") {
      header.textContent = comparison.message || "Processing SLAM scan-matching comparison…";
    } else if (comparison && comparison.state === "COMPLETE") {
      header.textContent = "ODOM TEST + SLAM comparison complete — press RESET for another run";
    }
  }
  setControlsReady(controlsReady);
}

odomMode.addEventListener("change", configureOdomTest);
spinDirection.addEventListener("change", configureOdomTest);
spinTurns.addEventListener("change", configureOdomTest);

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

finishedButton.addEventListener("pointerdown", async (event) => {
  event.preventDefault();
  if (!controlsReady || odomTestState !== "RECORDING") return;
  stop(true);
  finishedButton.disabled = true;
  setStatus("ODOM TEST — finalizing…");
  try {
    const response = await fetch("/odom_test/finish", {
      cache: "no-store",
      credentials: "same-origin"
    });
    const result = await response.json();
    if (!response.ok && result.state !== "FINALIZING" && result.state !== "COMPLETE") {
      setStatus(result.message || "Odometry test could not finish");
    }
  } catch (error) {
    setStatus("Odometry test finalization unavailable");
  }
});

resetButton.addEventListener("pointerdown", async (event) => {
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
