#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
UNIT_ID="${1:-head}"
SERIAL_PORT="${2:-}"
case "$UNIT_ID" in
  head) ROLE=head ;;
  node1|node2) ROLE=rear ;;
  *) echo "Usage: $0 head|node1|node2 [serial-port]" >&2; exit 2 ;;
esac

# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
ros2 pkg prefix hansel_bringup >/dev/null 2>&1 || { echo "[ERROR] hansel_bringup is not built. Run ./scripts/build_ros2.sh" >&2; exit 1; }
ros2 pkg prefix hansel_unit_control >/dev/null 2>&1 || { echo "[ERROR] hansel_unit_control is not built." >&2; exit 1; }
python3 - <<'PYTHON_SERIAL'
try:
    import serial  # noqa: F401
except Exception as exc:
    raise SystemExit(f"[ERROR] pyserial is required for Nano control: {exc}\n        Install: sudo apt install python3-serial")
PYTHON_SERIAL

if [ -z "$SERIAL_PORT" ]; then
  candidates=()
  [ -e /dev/hansel_nano ] && candidates+=(/dev/hansel_nano)
  for dev in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$dev" ] && candidates+=("$dev"); done
  if [ "${#candidates[@]}" -eq 0 ]; then
    echo "[ERROR] Arduino Nano serial device not found." >&2
    echo "        Checked: /dev/hansel_nano, /dev/ttyACM*, /dev/ttyUSB*" >&2
    echo "        Inspect with: lsusb ; dmesg | tail -n 40" >&2
    exit 1
  fi
  SERIAL_PORT="${candidates[0]}"
  if [ "${#candidates[@]}" -gt 1 ]; then
    echo "[WARN] Multiple serial devices found: ${candidates[*]}" >&2
    echo "       Auto-selecting $SERIAL_PORT. Pass an explicit port as argument 2 if needed." >&2
  fi
fi

[ -e "$SERIAL_PORT" ] || { echo "[ERROR] Serial device does not exist: $SERIAL_PORT" >&2; exit 1; }
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
  echo "[ERROR] No read/write permission for $SERIAL_PORT" >&2
  echo "        Current groups: $(id -nG)" >&2
  echo "        Usually fix with: sudo usermod -aG dialout $USER" >&2
  echo "        Then log out of SSH and reconnect." >&2
  exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser "$SERIAL_PORT" >/dev/null 2>&1; then
  echo "[ERROR] Serial device is already in use: $SERIAL_PORT" >&2
  fuser -v "$SERIAL_PORT" >&2 || true
  exit 1
fi

echo "[INFO] ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[INFO] unit=$UNIT_ID role=$ROLE serial=$SERIAL_PORT"
[ -z "${CYCLONEDDS_URI:-}" ] || echo "[INFO] CYCLONEDDS_URI=$CYCLONEDDS_URI"
PWM_ARGS=()
if [ -n "${HEAD_MAX_PWM:-}" ]; then
  PWM_ARGS+=(maximum_pwm_percent:="$HEAD_MAX_PWM")
fi
exec ros2 launch hansel_bringup unit.launch.py \
  "${PWM_ARGS[@]}" \
  unit_id:="$UNIT_ID" role:="$ROLE" \
  hardware_backend:=nano_serial nano_serial_port:="$SERIAL_PORT"
