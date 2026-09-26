#!/usr/bin/env python3
"""Source-tree launcher: no colcon rebuild or additional packages required."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/hansel_localization"))

from hansel_localization.map_viewer import main

if __name__ == "__main__":
    main(ROOT / "monitor/odometry_map")

