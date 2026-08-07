#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1090
source "$ROOT/ros2_ws/install/setup.bash"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch hansel_bringup dummy_system.launch.py \
  use_rviz:=false \
  use_rqt:=true \
  use_camera_receiver:=false \
  use_dummy_camera:=true \
  use_adapter_stubs:=false &
LAUNCH_PID=$!

printf 'Waiting for the dummy ROS graph and RQT...\n'
sleep "${HANSEL_STARTUP_WAIT_S:-4}"
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo 'dummy_system.launch.py exited before keyboard control started.' >&2
  wait "$LAUNCH_PID"
fi

cat <<'EOF'
Keyboard control is active in THIS terminal (no Enter required).
W/S forward/backward | A/D spin | Q/E forward curve | Z/C backward curve
X or Space stop | U/J head +/-2 deg | K center | P quit
RQT edits only: Straight RPM, Turn RPM, Up limit, Down limit.
The RQT camera panel should show the HANSEL single-PC test pattern.
EOF

ros2 run hansel_operator operator_input --ros-args \
  -p active_targets:="['head','node1','node2','node3']"
