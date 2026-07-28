#!/usr/bin/env python3
"""Apply a TI mmWave CLI profile across its dynamic UART baud switch.

The xWRL6432 Presence_Demo closes and reopens its only CLI/data UART when the
``baudRate`` command is handled.  The command echo is observed at the old baud,
while ``Done`` and the next prompt are emitted at the new baud.  Sending
``sensorStart`` before that new-baud prompt can silently lose the command.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from sensors.ti_radar_control import (
    ProfileCommands,
    apply_profile,
    load_commands,
    partition_at_baud,
    validate_profile_result,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Apply an xWRL6432 CLI profile and correctly cross its "
            "dynamic baud-rate switch."
        )
    )
    parser.add_argument("--port", required=True, help="COM3 or /dev/ttyACM0")
    parser.add_argument("--cfg", required=True, type=Path)
    parser.add_argument("--initial-baud", type=int, default=115200)
    parser.add_argument("--command-timeout", type=float, default=3.0)
    parser.add_argument("--reopen-delay", type=float, default=0.5)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print the profile without opening the UART",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.initial_baud < 1:
        raise SystemExit("--initial-baud must be positive")
    if args.command_timeout <= 0:
        raise SystemExit("--command-timeout must be positive")
    if args.reopen_delay < 0:
        raise SystemExit("--reopen-delay must be non-negative")

    commands = load_commands(args.cfg)
    profile = partition_at_baud(commands)
    if args.dry_run:
        result = {
            "cfg": str(args.cfg.resolve()),
            "commands": len(commands),
            "before_baud": len(profile.before_baud),
            "baud_command": profile.baud_command,
            "target_baud": profile.target_baud,
            "after_baud": list(profile.after_baud),
        }
    else:
        result = apply_profile(
            port=args.port,
            profile=profile,
            initial_baud=args.initial_baud,
            command_timeout_s=args.command_timeout,
            reopen_delay_s=args.reopen_delay,
        )
        validate_profile_result(result, expected_commands=len(commands))
        result["cfg"] = str(args.cfg.resolve())
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
