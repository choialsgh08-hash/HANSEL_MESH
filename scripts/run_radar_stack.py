#!/usr/bin/env python3
"""Run the production radar capture, watchdog, and viewer supervisor."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Iterator, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from serial.tools.list_ports import comports

from sensors.radar_stack_processes import RadarStackProcesses
from sensors.radar_supervisor import (
    EpochPaths,
    RadarSupervisor,
    RadarSupervisorConfig,
    RadarSupervisorDependencies,
)
from sensors.radar_watchdog import ExpectedRadarEvidence, RadarEpochWatchdog
from sensors.ti_radar_control import (
    RadarPortIdentity,
    apply_profile,
    find_xds110_reset,
    load_commands,
    partition_at_baud,
    reset_xds110_target,
)


DEFAULT_PROFILE = (
    REPOSITORY_ROOT
    / "configs"
    / "radar"
    / "iwrl6432_3d_operator_near_10hz.cfg"
)
RESET_SEARCH_ROOTS = (Path("C:/ti"),)


def _positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return parsed


def _positive_int(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least one")
    return parsed


def _http_port(value: str) -> int:
    parsed = _positive_int(value)
    if parsed > 65_535:
        raise argparse.ArgumentTypeError("must be between 1 and 65535")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the supervised IWRL6432 radar stack. The operator URL is "
            "http://127.0.0.1:8081/ by default."
        )
    )
    parser.add_argument("--port", help="explicit Application/User UART")
    parser.add_argument("--xds-serial", help="selected XDS110 serial number")
    parser.add_argument(
        "--reset-executable",
        "--xds110reset",
        dest="reset_executable",
        type=Path,
        help="exact xds110reset executable (otherwise auto-discovered)",
    )
    parser.add_argument("--run-id", help="artifact identifier (UTC when omitted)")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=REPOSITORY_ROOT,
        help="root containing missions, captures, and runtime",
    )
    parser.add_argument("--cfg", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        "--clutter-calibration",
        dest="calibration",
        type=Path,
        help="existing persistent clutter calibration JSON",
    )
    parser.add_argument(
        "--frame-timeout",
        type=_positive_float,
        default=2.5,
    )
    parser.add_argument(
        "--first-frame-timeout",
        type=_positive_float,
        default=3.0,
    )
    parser.add_argument(
        "--verification-timeout",
        type=_positive_float,
        default=5.0,
    )
    parser.add_argument("--verify-frames", type=_positive_int, default=30)
    parser.add_argument(
        "--retry-initial",
        type=_positive_float,
        default=0.5,
    )
    parser.add_argument("--retry-max", type=_positive_float, default=5.0)
    parser.add_argument("--http-port", type=_http_port, default=8081)
    return parser


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _configure_profile(
    port: RadarPortIdentity,
    config: RadarSupervisorConfig,
) -> dict[str, object]:
    commands = load_commands(config.profile_path)
    profile = partition_at_baud(commands)
    if profile.target_baud != config.data_baud:
        raise RuntimeError(
            "profile baud does not match configured radar data baud: "
            f"{profile.target_baud} != {config.data_baud}"
        )
    return apply_profile(
        port=port.device,
        profile=profile,
        initial_baud=config.initial_baud,
        command_timeout_s=config.verification_timeout_s,
        reopen_delay_s=config.retry_initial_s,
    )


def _reset_target(
    port: RadarPortIdentity,
    config: RadarSupervisorConfig,
) -> bool:
    if config.reset_executable is None:
        return False
    reset_xds110_target(
        config.reset_executable,
        port.serial_number,
        subprocess.run,
    )
    return True


def _watchdog(
    paths: EpochPaths,
    config: RadarSupervisorConfig,
    started_at_s: float,
) -> RadarEpochWatchdog:
    return RadarEpochWatchdog(
        mission_path=paths.mission,
        raw_path=paths.raw,
        expected=ExpectedRadarEvidence(
            profile_id=config.profile_id,
            heatmap_azimuth_bins=config.heatmap_azimuth_bins,
            heatmap_range_bins=config.heatmap_range_bins,
            heatmap_range_step_m=config.heatmap_range_step_m,
        ),
        started_at_s=started_at_s,
        first_frame_timeout_s=config.first_frame_timeout_s,
        frame_timeout_s=config.frame_timeout_s,
        required_consecutive_frames=config.verification_frames,
        verification_timeout_s=config.verification_timeout_s,
    )


@contextmanager
def _shutdown_requested() -> Iterator[object]:
    requested = False
    installed: list[tuple[int, object]] = []
    primary_error: BaseException | None = None
    primary_traceback = None
    restoration_failures: list[tuple[int, BaseException]] = []

    def request_shutdown(signum: int, frame: object) -> None:
        del signum, frame
        nonlocal requested
        requested = True

    try:
        for name in ("SIGINT", "SIGTERM"):
            signum = getattr(signal, name, None)
            if signum is not None:
                previous = signal.getsignal(signum)
                signal.signal(signum, request_shutdown)
                installed.append((signum, previous))
        yield lambda: requested
    except BaseException as error:
        primary_error = error
        primary_traceback = error.__traceback__

    for signum, handler in reversed(installed):
        try:
            signal.signal(signum, handler)
        except BaseException as error:
            restoration_failures.append((signum, error))

    if primary_error is not None:
        for signum, error in restoration_failures:
            primary_error.add_note(
                f"signal handler restoration failed for {signum!s}: "
                f"{error!r}"
            )
        raise primary_error.with_traceback(primary_traceback)

    if restoration_failures:
        _, first_error = restoration_failures[0]
        for signum, error in restoration_failures:
            first_error.add_note(
                f"signal handler restoration failed for {signum!s}: "
                f"{error!r}"
            )
        raise first_error


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.retry_initial > args.retry_max:
        raise SystemExit("--retry-initial must not exceed --retry-max")
    output_root = args.output_root.resolve()
    profile_path = args.cfg.resolve()
    calibration_path = (
        args.calibration.resolve() if args.calibration is not None else None
    )
    reset_path = (
        args.reset_executable.resolve()
        if args.reset_executable is not None
        else None
    )
    if not profile_path.is_file():
        raise SystemExit("--cfg must name an existing radar profile file")
    if calibration_path is None or not calibration_path.is_file():
        raise SystemExit(
            "--clutter-calibration must name an existing calibration file"
        )

    reset_executable: Path | None
    reset_unavailable_reason: str | None
    try:
        reset_executable = find_xds110_reset(
            reset_path,
            RESET_SEARCH_ROOTS,
        )
    except RuntimeError as exc:
        reset_executable = None
        reset_unavailable_reason = str(exc)
    else:
        reset_unavailable_reason = None

    run_id = args.run_id
    if run_id is None:
        run_id = _utc_now().strftime("%Y%m%d-%H%M%S")

    config = RadarSupervisorConfig(
        repository_root=REPOSITORY_ROOT,
        output_root=output_root,
        profile_path=profile_path,
        calibration_path=calibration_path,
        run_id=run_id,
        explicit_port=args.port,
        xds_serial=args.xds_serial,
        reset_executable=reset_executable,
        reset_unavailable_reason=reset_unavailable_reason,
        first_frame_timeout_s=args.first_frame_timeout,
        frame_timeout_s=args.frame_timeout,
        verification_timeout_s=args.verification_timeout,
        verification_frames=args.verify_frames,
        retry_initial_s=args.retry_initial,
        retry_max_s=args.retry_max,
        http_port=args.http_port,
    )
    dependencies = RadarSupervisorDependencies(
        port_provider=comports,
        reset_target=_reset_target,
        configure=_configure_profile,
        processes=RadarStackProcesses(),
        watchdog_factory=_watchdog,
        monotonic=time.monotonic,
        sleep=time.sleep,
        utc_now=_utc_now,
    )
    with _shutdown_requested() as stop_requested:
        RadarSupervisor(config, dependencies).run(stop_requested)
    return 0


if __name__ == "__main__":
    sys.exit(main())
