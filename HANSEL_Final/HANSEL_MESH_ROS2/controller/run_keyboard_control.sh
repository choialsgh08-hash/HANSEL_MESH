#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HANSEL_ROOT="$ROOT"
# shellcheck disable=SC1091
source "$ROOT/scripts/ros_env.sh"
exec ros2 run hansel_operator operator_input --ros-args \
  -p active_targets:="${ACTIVE_TARGETS:-['head','node1','node2']}" \
  -p speed_scale:="${SPEED_SCALE:-0.75}"
