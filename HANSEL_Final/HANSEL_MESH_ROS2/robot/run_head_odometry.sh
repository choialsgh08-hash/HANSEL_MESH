#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/home/hansel/ros2_lyrical/install/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$ROOT/ros2_ws/install/setup.bash}"
VENDOR_DIR="${HANSEL_PYTHON_VENDOR:-$ROOT/vendor/python}"

for setup_file in "$ROS_SETUP" "$WORKSPACE_SETUP"; do
    if [ ! -f "$setup_file" ]; then
        echo "[ERROR] ROS setup not found: $setup_file" >&2
        exit 1
    fi
    # Generated ROS/colcon setup files may read optional variables before
    # assigning them, so nounset must be disabled while they are sourced.
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
done

if [ -d "$VENDOR_DIR" ]; then
    export PYTHONPATH="$VENDOR_DIR${PYTHONPATH:+:$PYTHONPATH}"
fi

exec ros2 launch hansel_bringup head_odometry.launch.py "$@"
