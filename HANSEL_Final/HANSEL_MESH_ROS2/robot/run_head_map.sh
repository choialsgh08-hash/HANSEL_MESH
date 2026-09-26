#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/home/hansel/ros2_lyrical/install/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$ROOT/ros2_ws/install/setup.bash}"

for setup_file in "$ROS_SETUP" "$WORKSPACE_SETUP"; do
    if [ ! -f "$setup_file" ]; then
        echo "[ERROR] ROS setup not found: $setup_file" >&2
        exit 1
    fi
    # Generated colcon scripts read optional variables under nounset.
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
done

exec python3 "$ROOT/scripts/run_head_map.py" --host "${HANSEL_MAP_HOST:-192.168.50.10}" "$@"
