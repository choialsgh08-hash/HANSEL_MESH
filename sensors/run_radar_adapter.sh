#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MISSION_LOG="${1:-${RADAR_MISSION_LOG:-}}"
[ -n "$MISSION_LOG" ] || { echo "Usage: $0 /path/to/mission.jsonl" >&2; exit 2; }
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1090
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 run hansel_radar_adapter radar_adapter --ros-args \
  -p provider_mode:=mission_log \
  -p mission_log_path:="$MISSION_LOG" \
  -p mission_log_start_at_end:=true
