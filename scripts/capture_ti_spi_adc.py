#!/usr/bin/env python3
"""Configure IWRL6432, capture compact SPI RAW ADC, and emit near-field peaks."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Sequence

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from sensors.ti_radar_control import load_commands
from sensors.ti_spi_adc import (
    FtdiFccspCapture,
    NearFieldAdcProfile,
    process_near_field_adc,
    unpack_fccsp_spi_bytes,
)


DEFAULT_PROFILE = (
    REPOSITORY_ROOT / "configs" / "radar" / "iwrl6432_spi_near_30cm.cfg"
)


def _read_response(port: object, *, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    response = bytearray()
    while time.monotonic() < deadline:
        waiting = getattr(port, "in_waiting")
        chunk = port.read(waiting or 1)
        if chunk:
            response.extend(chunk)
            if b"mmwDemo:/>" in response:
                break
    return bytes(response)


def _send(port: object, command: str, *, timeout_s: float) -> bytes:
    port.write((command + "\n").encode("ascii"))
    port.flush()
    response = _read_response(port, timeout_s=timeout_s)
    lower = response.lower()
    if b"error" in lower or b"not recognized" in lower:
        raise RuntimeError(
            f"radar command failed: {command!r}: "
            f"{response.decode('ascii', errors='replace')!r}"
        )
    if b"Done" not in response:
        raise TimeoutError(f"radar command timed out: {command!r}")
    return response


def _commands_for_capture(path: Path, frames: int) -> tuple[str, ...]:
    commands = []
    saw_frame = False
    for command in load_commands(path):
        name = command.split(maxsplit=1)[0]
        if name in {"sensorStart", "baudRate"}:
            raise ValueError(f"SPI profile must not contain {name}")
        if name == "frameCfg":
            fields = command.split()
            if len(fields) != 7:
                raise ValueError("frameCfg must contain six values")
            fields[-1] = str(frames)
            command = " ".join(fields)
            saw_frame = True
        commands.append(command)
    if not saw_frame:
        raise ValueError("SPI profile is missing frameCfg")
    return tuple(commands)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture IWRL6432 FCCSP RAW ADC through C232HM-DDHSL-0. "
            "The board must run the SPI-enabled appimage and be freshly reset."
        )
    )
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--cfg", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--ftdi-index", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--command-timeout", type=float, default=5.0)
    parser.add_argument("--first-frame-timeout", type=float, default=45.0)
    parser.add_argument("--frame-timeout", type=float, default=5.0)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print the effective profile without touching hardware",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.frames < 1 or args.frames > 65_535:
        raise SystemExit("--frames must be between 1 and 65535")
    if args.ftdi_index < 0:
        raise SystemExit("--ftdi-index must be non-negative")
    for name in ("command_timeout", "first_frame_timeout", "frame_timeout"):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    commands = _commands_for_capture(args.cfg.resolve(), args.frames)
    profile = NearFieldAdcProfile()
    if args.dry_run:
        print(
            json.dumps(
                {
                    "cfg": str(args.cfg.resolve()),
                    "commands": list(commands),
                    "frames": args.frames,
                    "frame_bytes": profile.frame_bytes,
                    "binary_capture_bytes": profile.frame_bytes * args.frames,
                    "hpf_corner_range_m": profile.hpf_corner_range_m,
                    "trusted_minimum_range_m": profile.minimum_trusted_range_m,
                    "physical_range_resolution_m": (
                        profile.physical_range_resolution_m
                    ),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    output = args.output.resolve()
    metadata_path = output.with_suffix(output.suffix + ".json")
    existing = [str(path) for path in (output, metadata_path) if path.exists()]
    if existing:
        raise SystemExit("refusing to overwrite: " + ", ".join(existing))
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required") from exc

    with serial.Serial(
        args.port,
        115_200,
        timeout=0.05,
        write_timeout=1.0,
    ) as cli:
        cli.reset_input_buffer()
        for command in commands:
            _send(cli, command, timeout_s=args.command_timeout)

        with FtdiFccspCapture(device_index=args.ftdi_index) as capture:
            _send(cli, "sensorStart 0 0 0 0", timeout_s=args.command_timeout)
            frames = capture.capture_frames(
                args.frames,
                profile.frame_bytes,
                first_ready_timeout_s=args.first_frame_timeout,
                ready_timeout_s=args.frame_timeout,
            )

    payload = b"".join(frames)
    expected = args.frames * profile.frame_bytes
    if len(payload) != expected:
        raise RuntimeError(f"expected {expected} capture bytes, got {len(payload)}")
    values = unpack_fccsp_spi_bytes(payload)
    scene = process_near_field_adc(values, frames=args.frames, profile=profile)
    digest = hashlib.sha256(payload).hexdigest().upper()
    metadata = {
        "format": "hansel-iwrl6432-fccsp-raw-v1",
        "cfg": str(args.cfg.resolve()),
        "frames": args.frames,
        "frame_bytes": profile.frame_bytes,
        "capture_bytes": len(payload),
        "sha256": digest,
        "adc_samples": profile.adc_samples,
        "chirps_per_frame": profile.chirps_per_frame,
        "rx_antennas": profile.rx_antennas,
        "mimo": "BPM_2TX",
        "sample_rate_hz": profile.sample_rate_hz,
        "slope_hz_per_s": profile.slope_hz_per_s,
        "physical_range_resolution_m": profile.physical_range_resolution_m,
        "hpf_corner_range_m": profile.hpf_corner_range_m,
        "trusted_minimum_range_m": scene.trusted_minimum_range_m,
        "maximum_range_m": scene.maximum_range_m,
        "blind_zone_confidence": scene.blind_zone_confidence,
        "blind_zone_blocked": scene.blind_zone_blocked,
        "blind_zone_proxy_range_m": scene.blind_zone_proxy_range_m,
        "blind_zone_evidence_peak_m": scene.blind_zone_evidence_peak_m,
        "peaks": [
            {
                "range_m": peak.range_m,
                "angle_deg": peak.angle_deg,
                "x_m": peak.x_m,
                "y_m": peak.y_m,
                "relative_db": peak.relative_db,
                "spatial_coherence": peak.spatial_coherence,
                "angular_prominence_db": peak.angular_prominence_db,
            }
            for peak in scene.peaks
        ],
    }
    output.write_bytes(payload)
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(metadata, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
