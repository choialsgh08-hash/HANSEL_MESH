#!/usr/bin/env python3
"""Check fresh BNO085 samples without ROS or starting any motors."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor/python"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/hansel_localization"))

from hansel_localization.bno085 import Bno085I2cSource, Bno085RvcSource
from hansel_localization.core import quaternion_yaw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=("i2c", "uart_rvc"), default="uart_rvc")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive")
    source = None
    try:
        source = (
            Bno085RvcSource(args.port)
            if args.transport == "uart_rvc"
            else Bno085I2cSource()
        )
        start = last_sample = last_print = time.monotonic()
        max_gap = 0.0
        samples = errors = 0
        yaw = None
        print("Sensor-only test. Keep still first, then gently turn left/right.", flush=True)
        while time.monotonic() - start < args.seconds:
            try:
                sample = source.read()
            except Exception as exc:
                errors += 1
                sample = None
                if errors <= 3:
                    print(f"read error: {exc!r}", flush=True)
            now = time.monotonic()
            max_gap = max(max_gap, now - last_sample)
            if sample is not None:
                samples += 1
                last_sample = now
                yaw = math.degrees(quaternion_yaw(*sample.quaternion_xyzw))
            if now - last_print >= 1.0:
                print(
                    f"fresh_samples={samples} yaw_deg={yaw} "
                    f"age_s={now - last_sample:.3f} max_gap_s={max_gap:.3f} errors={errors}",
                    flush=True,
                )
                last_print = now
            time.sleep(0.005)
        invalid = source.frames.invalid_frames if isinstance(source, Bno085RvcSource) else 0
        good = samples > 0 and errors == 0 and invalid == 0 and max_gap <= 0.35
        print(
            f"{'PASS' if good else 'FAIL'} link test: fresh_samples={samples} "
            f"max_gap_s={max_gap:.3f} errors={errors} invalid_frames={invalid}"
        )
        print("This does not validate heading sign, drift or position accuracy.")
        return 0 if good else 1
    except KeyboardInterrupt:
        print("Test stopped; no pass result.")
        return 130
    except Exception as exc:
        print(f"[ERROR] Cannot open {args.transport} source: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
