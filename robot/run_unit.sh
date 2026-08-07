#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_ID="${1:-head}"
SERIAL_PORT="${2:-/dev/hansel_nano}"
case "$UNIT_ID" in
  head) ROLE=head ;;
  node1|node2|node3) ROLE=rear ;;
  *) echo "Usage: $0 head|node1|node2|node3 [serial-port]" >&2; exit 2 ;;
esac
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1090
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 launch hansel_bringup unit.launch.py \
  unit_id:="$UNIT_ID" role:="$ROLE" \
  hardware_backend:=nano_serial nano_serial_port:="$SERIAL_PORT"
