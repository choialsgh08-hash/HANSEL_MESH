#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m compileall -q "$ROOT/ros2_ws/src" "$ROOT/monitor" "$ROOT/listen" "$ROOT/tools" "$ROOT/tests"
python3 "$ROOT/tools/single_pc_control_smoke.py"
cd "$ROOT"
python3 -m pytest -q
