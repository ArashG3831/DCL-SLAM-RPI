#!/usr/bin/env bash

set -o pipefail

declare -a matches=()
declare -A seen=()

stable_path_for()
{
  local device="$1"
  local real_device
  local link

  real_device="$(readlink -f "$device" 2>/dev/null || true)"

  for link in /dev/serial/by-id/*; do
    [ -e "$link" ] || continue

    if [ "$(readlink -f "$link")" = "$real_device" ]; then
      printf '%s\n' "$link"
      return 0
    fi
  done

  printf '%s\n' "$device"
}

inspect_device()
{
  local device="$1"
  local real_device
  local properties
  local stable

  [ -e "$device" ] || return 0

  real_device="$(readlink -f "$device" 2>/dev/null || true)"
  [ -n "$real_device" ] || return 0

  [ -z "${seen[$real_device]+x}" ] || return 0
  seen["$real_device"]=1

  properties="$(udevadm info --query=property --name="$real_device" 2>/dev/null || true)"

  # Silicon Labs CP210x: vendor 10c4, product ea60.
  if grep -Eq '^ID_VENDOR_ID=10c4$' <<< "$properties" &&
     grep -Eq '^ID_MODEL_ID=ea60$' <<< "$properties"
  then
    stable="$(stable_path_for "$device")"
    matches+=("$stable")
  fi
}

echo "[D500 detect] scanning serial USB devices..." >&2

for device in /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM*; do
  inspect_device "$device"
done

if [ "${#matches[@]}" -eq 0 ]; then
  echo "[D500 detect] ERROR: no Silicon Labs CP210x lidar adapter found." >&2
  echo "[D500 detect] USB inventory:" >&2
  lsusb >&2 || true
  exit 1
fi

if [ "${#matches[@]}" -gt 1 ]; then
  echo "[D500 detect] WARNING: multiple CP210x devices found:" >&2
  printf '  %s\n' "${matches[@]}" >&2
  echo "[D500 detect] selecting the first one." >&2
fi

selected="${matches[0]}"

echo "[D500 detect] selected: $selected" >&2

# stdout contains only the selected path for command substitution.
printf '%s\n' "$selected"
