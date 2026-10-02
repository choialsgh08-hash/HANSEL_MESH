#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
launch_args=(
  "head_only:=${HEAD_ONLY:-false}"
  "use_rqt:=${USE_RQT:-true}"
  "use_camera_receiver:=${USE_CAMERA_RECEIVER:-false}"
  "use_network_adapter:=${USE_NETWORK_ADAPTER:-true}"
  "use_radar_adapter:=${USE_RADAR_ADAPTER:-false}"
  "network_udp_port:=${NETWORK_UDP_PORT:-7100}"
)
# ROS launch rejects an empty CLI value; use the launch file's default instead.
if [ -n "${RADAR_MISSION_LOG:-}" ]; then
  launch_args+=("radar_mission_log:=$RADAR_MISSION_LOG")
fi
exec ros2 launch hansel_bringup operator.launch.py "${launch_args[@]}"
