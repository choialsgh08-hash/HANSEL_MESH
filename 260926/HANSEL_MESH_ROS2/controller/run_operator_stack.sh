#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
exec ros2 launch hansel_bringup operator.launch.py \
  use_rqt:="${USE_RQT:-true}" \
  use_camera_receiver:="${USE_CAMERA_RECEIVER:-false}" \
  use_network_adapter:="${USE_NETWORK_ADAPTER:-true}" \
  use_radar_adapter:="${USE_RADAR_ADAPTER:-false}" \
  network_udp_port:="${NETWORK_UDP_PORT:-7100}" \
  radar_mission_log:="${RADAR_MISSION_LOG:-}"
