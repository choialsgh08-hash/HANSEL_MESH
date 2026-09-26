#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
echo "[CHECK] ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "[CHECK] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[CHECK] ros2=$(command -v ros2)"
[ -z "${CYCLONEDDS_URI:-}" ] || echo "[CHECK] CYCLONEDDS_URI=$CYCLONEDDS_URI"
for pkg in rmw_cyclonedds_cpp hansel_bringup hansel_unit_control hansel_interfaces; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || { echo "[ERROR] Missing ROS package: $pkg" >&2; exit 1; }
done
python3 - <<'PYTHON_HEAD'
import rclpy
import serial
print('[CHECK] Python rclpy import: OK')
print('[CHECK] Python pyserial import: OK')
PYTHON_HEAD
if ip -4 addr show dev bat0 2>/dev/null | grep -qE '^[[:space:]]*inet[[:space:]]'; then
  echo "[CHECK] bat0 IPv4: $(ip -4 -brief addr show dev bat0)"
else
  echo "[WARN] bat0 has no IPv4 address. Local ROS can run, but mesh discovery will not use the repository bat0 CycloneDDS config." >&2
fi
found=0
for dev in /dev/hansel_nano /dev/ttyACM* /dev/ttyUSB*; do
  if [ -e "$dev" ]; then
    found=1
    if [ -r "$dev" ] && [ -w "$dev" ]; then echo "[CHECK] serial: $dev (read/write OK)"; else echo "[WARN] serial: $dev exists but current user lacks read/write permission" >&2; fi
  fi
done
[ "$found" -eq 1 ] || echo "[WARN] No Nano serial device detected yet." >&2
echo "[OK] Head software preflight completed."
