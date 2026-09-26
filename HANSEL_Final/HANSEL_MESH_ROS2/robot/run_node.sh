#!/bin/bash
# Rear Raspberry Pi GPIO launcher. Head continues to use the unchanged run_unit.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
UNIT_ID="${1:-}"
case "$UNIT_ID" in
  node1|node2) shift ;;
  *) echo "Usage: $0 node1|node2 [gpio-params.yaml] [--check]" >&2; exit 2 ;;
esac
GPIO_PARAMS="${HANSEL_GPIO_PARAMS:-$ROOT/ros2_ws/src/hansel_bringup/config/node_gpio.yaml}"
CHECK_ONLY=0
PARAMS_SEEN=0
for arg in "$@"; do
  case "$arg" in
    --check) CHECK_ONLY=1 ;;
    --*) echo "[ERROR] Unknown option: $arg" >&2; exit 2 ;;
    *)
      if [ "$PARAMS_SEEN" -eq 1 ]; then echo "[ERROR] Only one parameter file is accepted" >&2; exit 2; fi
      GPIO_PARAMS="$arg"; PARAMS_SEEN=1 ;;
  esac
done
[ -r "$GPIO_PARAMS" ] || { echo "[ERROR] Cannot read GPIO parameters: $GPIO_PARAMS" >&2; exit 1; }
GPIO_PARAMS="$(cd "$(dirname "$GPIO_PARAMS")" && pwd)/$(basename "$GPIO_PARAMS")"
# Reuse the project's source-built ROS Lyrical / CycloneDDS setup, including nounset handling.
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
ros2 pkg prefix hansel_bringup >/dev/null 2>&1 || {
  echo "[ERROR] hansel_bringup is not built. Run bash ./scripts/build_ros2.sh" >&2; exit 1;
}
CONTROL_PREFIX="$(ros2 pkg prefix hansel_unit_control)" || {
  echo "[ERROR] hansel_unit_control is not built." >&2; exit 1;
}
UNIT_EXEC="$CONTROL_PREFIX/lib/hansel_unit_control/unit_controller"
[ -r "$UNIT_EXEC" ] || { echo "[ERROR] Controller entry point is missing: $UNIT_EXEC" >&2; exit 1; }
IFS= read -r SHEBANG < "$UNIT_EXEC"
case "$SHEBANG" in
  '#!/usr/bin/env python3') PYTHON="$(command -v python3)" ;;
  '#!'/*python*) PYTHON="${SHEBANG#\#!}" ;;
  *) echo "[ERROR] Cannot determine controller Python from $UNIT_EXEC; rebuild using system Python." >&2; exit 1 ;;
esac
[ -x "$PYTHON" ] || { echo "[ERROR] Controller Python is unavailable: $PYTHON" >&2; exit 1; }

# Preflight reads configuration and queries the local daemon without writing GPIO.
"$PYTHON" -m hansel_unit_control.gpio_preflight --root "$ROOT" --params "$GPIO_PARAMS"
if [ "$CHECK_ONLY" -eq 1 ]; then exit 0; fi

echo "[INFO] unit=$UNIT_ID role=rear backend=rpi_gpio library=pigpio"
echo "[INFO] ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
# Replacing the wrapper lets ROS launch receive termination signals directly.
exec ros2 launch hansel_bringup unit.launch.py \
  unit_id:="$UNIT_ID" role:=rear hardware_backend:=rpi_gpio \
  maximum_pwm_percent:="${UNIT_MAX_PWM:-75.0}" \
  gpio_params_file:="$GPIO_PARAMS"
