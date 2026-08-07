#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1090
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 run hansel_operator operator_input --ros-args \
  -p active_targets:="${ACTIVE_TARGETS:-['head','node1','node2','node3']}"
