#!/usr/bin/env bash

set -o pipefail

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
export ROS_STATIC_PEERS=192.168.1.108
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ATTEMPTS="${D500_START_ATTEMPTS:-4}"
SCAN_TIMEOUT="${D500_SCAN_TIMEOUT:-20}"

DRIVER_PID=""

stop_driver()
{
  if [ -n "$DRIVER_PID" ]; then
    kill -INT "$DRIVER_PID" 2>/dev/null || true
    sleep 1
    kill -TERM "$DRIVER_PID" 2>/dev/null || true
    wait "$DRIVER_PID" 2>/dev/null || true
    DRIVER_PID=""
  fi
}

reset_usb_device()
{
  local port="$1"
  local device
  local sys_path

  device="$(readlink -f "$port" 2>/dev/null || true)"
  [ -n "$device" ] || return 1

  sys_path="/sys$(udevadm info --query=path --name="$device" 2>/dev/null)"

  # Walk upward until the actual USB device directory is found.
  while [ "$sys_path" != "/sys" ] && [ ! -f "$sys_path/idVendor" ]; do
    sys_path="$(dirname "$sys_path")"
  done

  if [ ! -f "$sys_path/idVendor" ]; then
    echo "[D500 startup] could not locate USB sysfs device for $port"
    return 1
  fi

  echo "[D500 startup] resetting USB device at $sys_path"

  echo 0 | sudo tee "$sys_path/authorized" >/dev/null || return 1
  sleep 2
  echo 1 | sudo tee "$sys_path/authorized" >/dev/null || return 1
  sleep 4
}

cleanup()
{
  stop_driver
}

trap cleanup INT TERM EXIT

for attempt in $(seq 1 "$ATTEMPTS"); do
  echo
  echo "[D500 startup] attempt $attempt/$ATTEMPTS"

  # A USB reconnect can change the visible serial path. Permit an explicit
  # current device (for example /dev/ttyUSB0), while retaining automatic
  # CP210x/by-id detection when no override is supplied.
  if [ -n "${D500_PORT:-}" ] && [ -e "$D500_PORT" ]; then
    PORT="$D500_PORT"
    echo "[D500 startup] using explicit port: $PORT"
  else
    if [ -n "${D500_PORT:-}" ]; then
      echo "[D500 startup] requested D500_PORT is unavailable; re-detecting CP210x"
    fi
    PORT="$("$HOME/find_d500_port.sh")" || true
  fi

  if [ -z "$PORT" ]; then
    echo "[D500 startup] lidar serial adapter not found"
    sleep 3
    continue
  fi

  echo "[D500 startup] selected port: $PORT"
  sudo chmod a+rw "$PORT" || true

  python3 -u "$HOME/d500_ros2_scan.py" \
    --port "$PORT" \
    --topic /scan \
    --frame-id d500_lidar &

  DRIVER_PID=$!

  echo "[D500 startup] waiting for an actual LaserScan..."

  if timeout "$SCAN_TIMEOUT" ros2 topic echo \
      /scan sensor_msgs/msg/LaserScan \
      --once \
      --qos-reliability best_effort \
      >/dev/null 2>&1
  then
    echo "[D500 startup] SUCCESS: live LaserScan confirmed on /scan"

    if [ -n "${D500_SELECTED_PORT_FILE:-}" ]; then
      printf '%s\n' "$PORT" > "$D500_SELECTED_PORT_FILE"
    fi

    # Keep this wrapper alive for as long as the real driver is alive.
    wait "$DRIVER_PID"
    RESULT=$?
    DRIVER_PID=""
    exit "$RESULT"
  fi

  echo "[D500 startup] no LaserScan received after ${SCAN_TIMEOUT}s"
  stop_driver

  if [ "$attempt" -lt "$ATTEMPTS" ]; then
    reset_usb_device "$PORT" || true
  fi
done

echo
echo "[D500 startup] ERROR: correct CP2102 adapter was found, but the lidar"
echo "[D500 startup] produced no LaserScan after $ATTEMPTS attempts."
exit 1
