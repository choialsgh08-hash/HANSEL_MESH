#!/usr/bin/env python3
"""Cross-platform command line tools for HANSEL sensor logs and radar data."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import uuid
from typing import Iterator

from common.radar_geometry import RadarAxes
from common.sensor_contract import (
    ImuSample,
    RadarFrame,
    RadarPoint,
    SensorHeader,
    WheelState,
    record_to_dict,
)
from common.sensor_json import canonical_json_bytes
from sensors.mission_log import (
    MissionLogWriter,
    inspect_mission_log,
    iter_replay,
)
from sensors.radar_capture import (
    capture_radar_uart,
    capture_stats_dict,
    classify_frame_transition,
)
from sensors.radar_owner_lock import (
    RADAR_UART_LOCK_ROOT,
    acquire_radar_owner_lock,
)
from sensors.radar_parent_lease import start_parent_death_watcher
from sensors.raw_capture_index import inspect_uart_chunk_index
from sensors.radar_calibration import build_clutter_model
from sensors.ti_mmwave import TiMmwavePacketParser, TiMmwaveStreamDecoder


def _headers(
    mission_id: str,
    producer_id: str,
    stream_id: str,
    count: int,
    start_ns: int,
    period_ns: int,
    frame_id: str,
) -> Iterator[SensorHeader]:
    for index in range(count):
        yield SensorHeader(
            mission_id=mission_id,
            unit_id="head",
            boot_id="demo-boot",
            producer_id=producer_id,
            stream_id=stream_id,
            seq=index + 1,
            monotonic_ns=start_ns + index * period_ns,
            frame_id=frame_id,
        )


def command_demo(args: argparse.Namespace) -> int:
    mission_id = args.mission_id or f"demo-{uuid.uuid4()}"
    producer_id = f"demo-{uuid.uuid4()}"
    start_ns = time.monotonic_ns()
    output = Path(args.output)

    radar_headers = _headers(
        mission_id,
        producer_id,
        "radar/front",
        args.frames,
        start_ns,
        100_000_000,
        "radar_native",
    )
    imu_headers = _headers(
        mission_id,
        producer_id,
        "imu/body",
        args.frames,
        start_ns + 10_000_000,
        100_000_000,
        "imu_link",
    )
    wheel_headers = _headers(
        mission_id,
        producer_id,
        "wheel/drive",
        args.frames,
        start_ns + 20_000_000,
        100_000_000,
        "base_link",
    )

    with MissionLogWriter(output, overwrite=args.overwrite) as writer:
        for index, (radar_header, imu_header, wheel_header) in enumerate(
            zip(radar_headers, imu_headers, wheel_headers)
        ):
            radar = RadarFrame(
                header=radar_header,
                frame_number=index,
                subframe_number=0,
                complete=True,
                dropped_frames_since_previous=0,
                points=(
                    RadarPoint(
                        x_m=1.0 + index * 0.05,
                        y_m=0.2,
                        z_m=0.0,
                        radial_velocity_mps=-0.1,
                        snr_db=18.0,
                        noise_db=6.0,
                    ),
                ),
                source_format="synthetic",
                sdk_version="demo-1",
            )
            imu = ImuSample(
                header=imu_header,
                specific_force_mps2=(0.0, 0.0, 9.80665),
                angular_velocity_radps=(0.0, 0.0, 0.01),
                temperature_c=25.0,
            )
            wheel = WheelState(
                header=wheel_header,
                left_ticks=index * 10,
                right_ticks=index * 10,
                sample_period_ns=100_000_000,
            )
            for record in (radar, imu, wheel):
                if not writer.submit(record):
                    raise RuntimeError("demo writer queue unexpectedly filled")

    print(
        json.dumps(
            inspect_mission_log(output),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def command_inspect(args: argparse.Namespace) -> int:
    report = inspect_mission_log(Path(args.path))
    print(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if report["healthy"] else 2


def command_replay(args: argparse.Namespace) -> int:
    counts = {}
    for entry in iter_replay(Path(args.path), speed=args.speed):
        record_type = record_to_dict(entry.record)["record_type"]
        counts[record_type] = counts.get(record_type, 0) + 1
        if args.emit_json:
            sys.stdout.buffer.write(
                canonical_json_bytes(
                    {
                        "log_seq": entry.log_seq,
                        "record": record_to_dict(entry.record),
                    }
                )
                + b"\n"
            )
    if not args.emit_json:
        print(
            json.dumps(
                {
                    "records": sum(counts.values()),
                    "record_counts": dict(sorted(counts.items())),
                    "speed": args.speed,
                },
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
        )
    return 0


def command_radar_bin(args: argparse.Namespace) -> int:
    requested_header = (
        "auto" if args.header_size == "auto" else int(args.header_size)
    )
    parser = TiMmwavePacketParser(
        header_size=requested_header,
        float_point_tlv=args.float_point_tlv,
        side_info_tlv=args.side_info_tlv,
        compressed_point_tlv=args.compressed_point_tlv,
        heatmap_azimuth_bins=getattr(
            args,
            "heatmap_azimuth_bins",
            None,
        ),
        heatmap_range_bins=getattr(
            args,
            "heatmap_range_bins",
            None,
        ),
        heatmap_range_step_m=getattr(
            args,
            "heatmap_range_step_m",
            None,
        ),
        tlv_length_includes_header=args.tlv_length_includes_header,
        allow_elided_empty_point_tlv=getattr(
            args,
            "allow_elided_empty_point_tlv",
            False,
        ),
        allow_nonzero_padding=getattr(
            args,
            "allow_nonzero_padding",
            False,
        ),
    )
    decoder = TiMmwaveStreamDecoder(parser=parser)
    frame_count = 0
    point_cloud_frames = 0
    empty_point_frames = 0
    nonzero_padding_frames = 0
    point_count = 0
    heatmap_frames = 0
    major_heatmap_frames = 0
    minor_heatmap_frames = 0
    missing_heatmap_frames = 0
    heatmap_cells_decoded = 0
    incomplete_count = 0
    missing_point_tlv_frames = 0
    radar_frame_gaps = 0
    device_discontinuities = 0
    previous_frame = None
    header_sizes = {}
    with Path(args.path).open("rb") as handle:
        while True:
            chunk = handle.read(args.chunk_bytes)
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                gap, transition = classify_frame_transition(
                    previous_frame,
                    frame.header.frame_number,
                )
                previous_frame = frame.header.frame_number
                radar_frame_gaps += gap
                if transition in {"duplicate", "reset_or_out_of_order"}:
                    device_discontinuities += 1
                frame_count += 1
                if any(
                    warning.startswith("nonzero_padding:")
                    for warning in frame.warnings
                ):
                    nonzero_padding_frames += 1
                if frame.point_format in {"float", "compressed", "empty"}:
                    point_cloud_frames += 1
                    if frame.point_format == "empty":
                        empty_point_frames += 1
                else:
                    missing_point_tlv_frames += 1
                point_count += len(frame.points)
                if frame.heatmap is None:
                    if getattr(args, "heatmap_azimuth_bins", None) is not None:
                        missing_heatmap_frames += 1
                else:
                    heatmap_frames += 1
                    heatmap_cells_decoded += len(frame.heatmap.data)
                    if frame.heatmap.motion_mode == "major":
                        major_heatmap_frames += 1
                    else:
                        minor_heatmap_frames += 1
                incomplete_count += 0 if frame.complete else 1
                header_sizes[str(frame.header_size)] = (
                    header_sizes.get(str(frame.header_size), 0) + 1
                )
                if args.frames:
                    print(
                        json.dumps(
                            {
                                "frame_number": frame.header.frame_number,
                                "header_size": frame.header_size,
                                "point_format": frame.point_format,
                                "points": len(frame.points),
                                "complete": frame.complete,
                                "frame_transition": transition,
                                "dropped_frames_since_previous": gap,
                                "heatmap": (
                                    None
                                    if frame.heatmap is None
                                    else {
                                        "azimuth_bins": (
                                            frame.heatmap.azimuth_bins
                                        ),
                                        "range_bins": (
                                            frame.heatmap.range_bins
                                        ),
                                        "cells": len(frame.heatmap.data),
                                        "motion_mode": (
                                            frame.heatmap.motion_mode
                                        ),
                                        "tlv_type": frame.heatmap.tlv_type,
                                        "floor_db": frame.heatmap.floor_db,
                                        "ceiling_db": (
                                            frame.heatmap.ceiling_db
                                        ),
                                    }
                                ),
                                "warnings": list(frame.warnings),
                            },
                            ensure_ascii=False,
                            sort_keys=True,
                        )
                    )
    print(
        json.dumps(
            {
                "frames": frame_count,
                "point_cloud_frames": point_cloud_frames,
                "empty_point_frames": empty_point_frames,
                "nonzero_padding_frames": nonzero_padding_frames,
                "points": point_count,
                "heatmap_frames": heatmap_frames,
                "major_heatmap_frames": major_heatmap_frames,
                "minor_heatmap_frames": minor_heatmap_frames,
                "missing_heatmap_frames": missing_heatmap_frames,
                "heatmap_cells_decoded": heatmap_cells_decoded,
                "heatmap_expected": (
                    parser.heatmap_azimuth_bins is not None
                ),
                "heatmap_azimuth_bins": parser.heatmap_azimuth_bins,
                "heatmap_range_bins": parser.heatmap_range_bins,
                "heatmap_range_step_m": parser.heatmap_range_step_m,
                "incomplete_frames": incomplete_count,
                "missing_point_tlv_frames": missing_point_tlv_frames,
                "radar_frame_gaps": radar_frame_gaps,
                "device_discontinuities": device_discontinuities,
                "header_sizes": header_sizes,
                "discarded_bytes": decoder.discarded_bytes,
                "parse_errors": decoder.parse_errors,
                "startup_sync_discarded_bytes": (
                    decoder.startup_sync_discarded_bytes
                ),
                "startup_sync_parse_errors": (
                    decoder.startup_sync_parse_errors
                ),
                "post_sync_discarded_bytes": (
                    decoder.post_sync_discarded_bytes
                ),
                "post_sync_parse_errors": (
                    decoder.post_sync_parse_errors
                ),
                "buffered_tail_bytes": decoder.buffered_bytes,
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    usable = (
        frame_count > 0
        and point_cloud_frames > 0
        and missing_point_tlv_frames == 0
        and missing_heatmap_frames == 0
        and incomplete_count == 0
        and radar_frame_gaps == 0
        and device_discontinuities == 0
        and (
            getattr(args, "allow_startup_resync", False)
            or (
                decoder.startup_sync_discarded_bytes == 0
                and decoder.startup_sync_parse_errors == 0
            )
        )
        and decoder.post_sync_discarded_bytes == 0
        and decoder.post_sync_parse_errors == 0
        and decoder.buffered_bytes == 0
    )
    return 0 if usable else 2


@contextmanager
def _radar_shutdown_signals():
    previous: dict[int, object] = {}

    def interrupt_capture(signum, frame):
        raise KeyboardInterrupt

    for name in ("SIGTERM", "SIGBREAK"):
        signum = getattr(signal, name, None)
        if signum is not None:
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt_capture)
    try:
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def command_radar_live(args: argparse.Namespace) -> int:
    parent_lease_path = getattr(args, "supervisor_parent_lease", None)
    xds_owner_serial = getattr(args, "xds_owner_serial", None)
    xds_owner_run_id = getattr(args, "xds_owner_run_id", None)
    managed_values = (
        parent_lease_path,
        xds_owner_serial,
        xds_owner_run_id,
    )
    if any(value is not None for value in managed_values) and not all(
        value is not None for value in managed_values
    ):
        raise ValueError(
            "managed radar capture ownership arguments must be supplied "
            "together"
        )

    parent_watcher = None
    uart_lock = None
    if parent_lease_path is not None:
        parent_watcher = start_parent_death_watcher(
            Path(parent_lease_path)
        )
        if not parent_watcher.ready.wait(5.0):
            raise RuntimeError("parent-death watcher did not become ready")
        if parent_watcher.stop_requested.is_set():
            raise RuntimeError("radar supervisor exited before capture startup")
        uart_lock = acquire_radar_owner_lock(
            RADAR_UART_LOCK_ROOT,
            str(xds_owner_serial),
            str(xds_owner_run_id),
        )
        if parent_watcher.stop_requested.is_set():
            uart_lock.release()
            raise RuntimeError("radar supervisor exited before UART open")

    try:
        with _radar_shutdown_signals():
            stats = capture_radar_uart(
                port=args.port,
                baudrate=args.baud,
                mission_log=Path(args.output),
                mission_id=args.mission_id,
                profile_id=args.profile_id,
                calibration_id=args.calibration_id,
                unit_id=args.unit_id,
                boot_id=args.boot_id,
                raw_capture=(
                    None
                    if args.raw_output is None
                    else Path(args.raw_output)
                ),
                raw_index=(
                    None
                    if args.raw_index is None
                    else Path(args.raw_index)
                ),
                duration_s=args.duration,
                read_size=args.read_bytes,
                serial_timeout_s=args.serial_timeout,
                health_interval_s=getattr(args, "health_interval", 0.5),
                overwrite=args.overwrite,
                header_size=int(args.header_size),
                allow_elided_empty_point_tlv=getattr(
                    args,
                    "allow_elided_empty_point_tlv",
                    False,
                ),
                allow_nonzero_padding=getattr(
                    args,
                    "allow_nonzero_padding",
                    False,
                ),
                heatmap_azimuth_bins=getattr(
                    args,
                    "heatmap_azimuth_bins",
                    None,
                ),
                heatmap_range_bins=getattr(
                    args,
                    "heatmap_range_bins",
                    None,
                ),
                heatmap_range_step_m=getattr(
                    args,
                    "heatmap_range_step_m",
                    None,
                ),
                stop_requested=(
                    None
                    if parent_watcher is None
                    else parent_watcher.stop_requested.is_set
                ),
            )
    finally:
        if uart_lock is not None:
            uart_lock.release()
    print(
        json.dumps(
            capture_stats_dict(stats),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    usable = (
        stats.frames_decoded > 0
        and stats.point_cloud_frames > 0
        and stats.missing_point_tlv_frames == 0
        and stats.missing_heatmap_frames == 0
        and stats.incomplete_frames == 0
        and stats.radar_frame_gaps == 0
        and stats.writer_drops == 0
        and stats.post_sync_discarded_bytes == 0
        and stats.post_sync_parse_errors == 0
        and stats.buffered_tail_bytes == 0
        and stats.device_discontinuities == 0
    )
    return 0 if usable else 2


def command_radar_index(args: argparse.Namespace) -> int:
    raw_path = Path(args.path)
    index_path = (
        Path(args.index)
        if args.index is not None
        else Path(f"{raw_path}.chunks.jsonl")
    )
    report = inspect_uart_chunk_index(raw_path, index_path)
    print(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if report["healthy"] else 2


def command_radar_calibrate(args: argparse.Namespace) -> int:
    output = Path(args.output)
    if output.exists() and not args.overwrite:
        raise ValueError(
            f"output already exists; pass --overwrite to replace {output.name}"
        )

    frames = tuple(
        entry.record
        for entry in iter_replay(Path(args.path))
        if (
            isinstance(entry.record, RadarFrame)
            and entry.record.complete
            and entry.record.heatmap is not None
        )
    )
    axes = RadarAxes(
        forward_axis=args.forward_axis,
        forward_sign=args.forward_sign,
        lateral_axis=args.lateral_axis,
        lateral_sign=args.lateral_sign,
    )
    model = build_clutter_model(
        frames,
        axes,
        min_frames=args.min_frames,
    )

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=f".{output.name}.",
            suffix=".tmp",
            dir=output.parent,
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write(model.canonical_bytes() + b"\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, output)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass

    print(
        json.dumps(
            {
                "azimuth_bins": model.azimuth_bins,
                "calibration_id": model.calibration_id,
                "frames_used": len(frames),
                "motion_mode": model.motion_mode,
                "point_clusters": len(model.point_clusters),
                "profile_id": model.profile_id,
                "range_bins": model.range_bins,
                "range_step_m": model.range_step_m,
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    )
    return 0


def _calibration_min_frames(value: str) -> int:
    result = int(value)
    if result < 5:
        raise argparse.ArgumentTypeError("must be at least 5")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "HANSEL sensor data contract, JSONL mission log, replay, and "
            "TI mmWave binary inspection tools"
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    demo = subparsers.add_parser(
        "demo",
        help="write a small hardware-free synthetic mission log",
    )
    demo.add_argument("output", help="new JSONL output path")
    demo.add_argument("--frames", type=int, default=5)
    demo.add_argument("--mission-id")
    demo.add_argument("--overwrite", action="store_true")
    demo.set_defaults(func=command_demo)

    inspect_parser = subparsers.add_parser(
        "inspect",
        help="strictly validate and summarize a mission log",
    )
    inspect_parser.add_argument("path")
    inspect_parser.set_defaults(func=command_inspect)

    replay = subparsers.add_parser(
        "replay",
        help="replay a mission log in file order",
    )
    replay.add_argument("path")
    replay.add_argument(
        "--speed",
        type=float,
        default=0.0,
        help="0=no sleeps, 1=realtime, 2=twice realtime",
    )
    replay.add_argument(
        "--emit-json",
        action="store_true",
        help="write each replayed record to stdout as canonical JSON",
    )
    replay.set_defaults(func=command_replay)

    radar = subparsers.add_parser(
        "radar-bin",
        help="inspect a captured IWRL6432 UART binary stream",
    )
    radar.add_argument("path")
    radar.add_argument(
        "--header-size",
        choices=("auto", "40", "52"),
        default="40",
    )
    radar.add_argument("--float-point-tlv", type=int, default=1)
    radar.add_argument("--side-info-tlv", type=int, default=7)
    radar.add_argument("--compressed-point-tlv", type=int, default=301)
    radar.add_argument(
        "--heatmap-azimuth-bins",
        type=int,
        help=(
            "decode TI TLV 304/305 as a range-azimuth heatmap with this "
            "azimuth dimension; omitted leaves those TLVs unknown"
        ),
    )
    radar.add_argument(
        "--heatmap-range-bins",
        type=int,
        help=(
            "configured number of heatmap range bins; required with "
            "--heatmap-azimuth-bins and --heatmap-range-step-m"
        ),
    )
    radar.add_argument(
        "--heatmap-range-step-m",
        type=float,
        help=(
            "metres per heatmap range bin; required with "
            "--heatmap-azimuth-bins and --heatmap-range-bins"
        ),
    )
    radar.add_argument("--tlv-length-includes-header", action="store_true")
    radar.add_argument(
        "--allow-elided-empty-point-tlv",
        action="store_true",
        help=(
            "accept a missing point-cloud TLV only when the packet header "
            "reports zero detected objects"
        ),
    )
    radar.add_argument(
        "--allow-nonzero-padding",
        action="store_true",
        help=(
            "accept non-zero trailing bytes only as <=31-byte, "
            "32-byte-aligned packet padding"
        ),
    )
    radar.add_argument(
        "--allow-startup-resync",
        action="store_true",
        help=(
            "allow bytes discarded before the first valid frame; intended "
            "for captures that began mid-stream"
        ),
    )
    radar.add_argument("--chunk-bytes", type=int, default=4096)
    radar.add_argument(
        "--frames",
        action="store_true",
        help="print one summary line for every decoded frame",
    )
    radar.set_defaults(func=command_radar_bin)

    radar_index = subparsers.add_parser(
        "radar-index",
        help="validate a raw UART capture against its chunk timing index",
    )
    radar_index.add_argument("path", help="raw UART .bin path")
    radar_index.add_argument(
        "--index",
        help="timing JSONL path (default: PATH.chunks.jsonl)",
    )
    radar_index.set_defaults(func=command_radar_index)

    calibrate = subparsers.add_parser(
        "radar-calibrate",
        help="build a deterministic profile-bound radar self-clutter model",
    )
    calibrate.add_argument("path")
    calibrate.add_argument("--output", required=True)
    calibrate.add_argument(
        "--min-frames",
        type=_calibration_min_frames,
        default=50,
    )
    calibrate.add_argument(
        "--forward-axis",
        choices=("x", "y"),
        default="y",
    )
    calibrate.add_argument(
        "--forward-sign",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    calibrate.add_argument(
        "--lateral-axis",
        choices=("x", "y"),
        default="x",
    )
    calibrate.add_argument(
        "--lateral-sign",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    calibrate.add_argument("--overwrite", action="store_true")
    calibrate.set_defaults(func=command_radar_calibrate)

    live = subparsers.add_parser(
        "radar-live",
        help="capture an already-running IWRL6432 UART stream",
    )
    live.add_argument("--port", required=True, help="COM5 or /dev/ttyACM0")
    live.add_argument("--baud", type=int, default=115200)
    live.add_argument("--output", required=True, help="new mission JSONL path")
    live.add_argument("--raw-output", help="optional raw UART .bin path")
    live.add_argument(
        "--raw-index",
        help=(
            "optional UART chunk timing JSONL path; requires --raw-output "
            "(default: RAW_OUTPUT.chunks.jsonl)"
        ),
    )
    live.add_argument("--mission-id", required=True)
    live.add_argument(
        "--profile-id",
        required=True,
        help="pinned SDK/appimage/cfg identifier or compact hash",
    )
    live.add_argument(
        "--calibration-id",
        default="uncalibrated",
        help="radar-to-base calibration identifier",
    )
    live.add_argument("--unit-id", default="head")
    live.add_argument("--boot-id")
    live.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="seconds; 0 runs until Ctrl+C",
    )
    live.add_argument("--read-bytes", type=int, default=1024)
    live.add_argument("--serial-timeout", type=float, default=0.01)
    live.add_argument(
        "--health-interval",
        type=float,
        default=0.5,
        help="seconds between live parser/writer health records",
    )
    live.add_argument("--header-size", choices=("40", "52"), default="40")
    live.add_argument(
        "--heatmap-azimuth-bins",
        type=int,
        help=(
            "decode TI TLV 304/305 using this configured azimuth FFT size"
        ),
    )
    live.add_argument(
        "--heatmap-range-bins",
        type=int,
        help=(
            "configured number of heatmap range bins; required with "
            "--heatmap-azimuth-bins and --heatmap-range-step-m"
        ),
    )
    live.add_argument(
        "--heatmap-range-step-m",
        type=float,
        help=(
            "metres per heatmap range bin; required with "
            "--heatmap-azimuth-bins and --heatmap-range-bins"
        ),
    )
    live.add_argument(
        "--allow-elided-empty-point-tlv",
        action="store_true",
        help=(
            "accept the official demo's omitted point TLV when zero "
            "objects are detected"
        ),
    )
    live.add_argument(
        "--allow-nonzero-padding",
        action="store_true",
        help=(
            "accept the official demo's uninitialized packet padding "
            "under strict alignment checks"
        ),
    )
    live.add_argument(
        "--supervisor-parent-lease",
        type=Path,
        help=argparse.SUPPRESS,
    )
    live.add_argument(
        "--xds-owner-serial",
        help=argparse.SUPPRESS,
    )
    live.add_argument(
        "--xds-owner-run-id",
        help=argparse.SUPPRESS,
    )
    live.add_argument("--overwrite", action="store_true")
    live.set_defaults(func=command_radar_live)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "demo" and args.frames < 1:
        parser.error("--frames must be positive")
    if getattr(args, "chunk_bytes", 1) < 1:
        parser.error("--chunk-bytes must be positive")
    if getattr(args, "read_bytes", 1) < 1:
        parser.error("--read-bytes must be positive")
    heatmap_azimuth_bins = getattr(args, "heatmap_azimuth_bins", None)
    heatmap_range_bins = getattr(args, "heatmap_range_bins", None)
    heatmap_range_step_m = getattr(args, "heatmap_range_step_m", None)
    heatmap_configuration = (
        heatmap_azimuth_bins,
        heatmap_range_bins,
        heatmap_range_step_m,
    )
    if any(value is not None for value in heatmap_configuration) and not all(
        value is not None for value in heatmap_configuration
    ):
        parser.error(
            "--heatmap-azimuth-bins, --heatmap-range-bins, and "
            "--heatmap-range-step-m "
            "must be supplied together"
        )
    if heatmap_azimuth_bins is not None and heatmap_azimuth_bins < 1:
        parser.error("--heatmap-azimuth-bins must be positive")
    if heatmap_range_bins is not None and heatmap_range_bins < 1:
        parser.error("--heatmap-range-bins must be positive")
    if heatmap_range_step_m is not None and (
        not math.isfinite(heatmap_range_step_m)
        or heatmap_range_step_m <= 0
    ):
        parser.error("--heatmap-range-step-m must be positive")
    try:
        return int(args.func(args))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
