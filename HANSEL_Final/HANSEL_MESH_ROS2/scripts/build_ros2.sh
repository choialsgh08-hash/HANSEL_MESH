#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
export HANSEL_SOURCE_WORKSPACE=0
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
unset HANSEL_SOURCE_WORKSPACE

for cmd in python3 colcon ros2; do
  command -v "$cmd" >/dev/null 2>&1 || { echo "[ERROR] Required command not found: $cmd" >&2; exit 1; }
done

python3 - <<'PYTHON_PREFLIGHT'
import importlib
required = ['rclpy', 'yaml', 'ament_index_python']
missing=[]
for name in required:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append(f"{name}: {exc}")
if missing:
    raise SystemExit('[ERROR] ROS Python preflight failed:\n  ' + '\n  '.join(missing))
PYTHON_PREFLIGHT

required_ros_packages=(
  rmw_cyclonedds_cpp rclpy rcl_interfaces launch launch_ros
  rosidl_default_generators diagnostic_msgs sensor_msgs nav_msgs tf2_msgs
)
for pkg in "${required_ros_packages[@]}"; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || {
    echo "[ERROR] Required ROS package is missing from ~/ros2_lyrical: $pkg" >&2
    exit 1
  }
done

cd "$ROOT/ros2_ws"
if [ "${HANSEL_RUN_ROSDEP:-0}" = "1" ]; then
  command -v rosdep >/dev/null 2>&1 || { echo "[ERROR] rosdep not found" >&2; exit 1; }
  rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical
else
  echo "[INFO] Skipping rosdep for the source-built ROS underlay. Set HANSEL_RUN_ROSDEP=1 only when OS dependencies changed."
fi

colcon build --symlink-install

export HANSEL_SOURCE_WORKSPACE=1
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
unset HANSEL_SOURCE_WORKSPACE
for pkg in hansel_interfaces hansel_bringup hansel_unit_control hansel_operator; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || {
    echo "[ERROR] Build completed but package is missing from the overlay: $pkg" >&2
    exit 1
  }
done

echo "[OK] HANSEL ROS 2 workspace build and overlay validation passed."
