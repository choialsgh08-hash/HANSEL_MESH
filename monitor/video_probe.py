#!/usr/bin/env python3
"""Receiver-side video quality probe (runs on the operator laptop).

Replaces the plain ffplay viewer with a single ffmpeg process that BOTH shows
the video AND measures its quality. It parses two things from ffmpeg:

  * stdout  : `-progress` key=value blocks -> frame, fps, drop_frames, bitrate
  * stderr  : decoder complaints -> count of corrupted / concealed frames
              (these are the direct fingerprint of UDP packet loss)

Each interval it writes one JSON line to a log file and (optionally) sends the
same stats to the mesh collector (dashboard.py) over UDP :7100, so video
quality can be charted on the same time axis as RSSI / TQ / RTT.

Run on the laptop (where you currently run receive_camera_stream.sh):
    python3 monitor/video_probe.py --send 192.168.50.1:7100

No display (headless measure only):
    python3 monitor/video_probe.py --no-display --send 192.168.50.1:7100
"""

from __future__ import annotations  # 3.9 compatibility

import argparse
import json
import re
import socket
import subprocess
import threading
import time


# Decoder lines that signal packet-loss damage to the H.264 stream.
ERROR_RE = re.compile(
    r"error while decoding|concealing|corrupt|Invalid NAL|missing picture|"
    r"decode_slice_header|non-existing|mmco|out of range|Marker bit|"
    r"reference picture",
    re.IGNORECASE,
)


# --------------------------------------------------------------------------- #
# Pure parsers (unit-testable without ffmpeg)                                  #
# --------------------------------------------------------------------------- #

def is_error_line(line: str) -> bool:
    """True if an ffmpeg stderr line indicates a decode/loss problem."""
    return bool(ERROR_RE.search(line))


def parse_progress_value(key: str, raw: str):
    """Convert one ffmpeg -progress value to a number where it makes sense."""
    raw = raw.strip()
    if key == "bitrate":
        m = re.search(r"[\d.]+", raw)
        return float(m.group()) if m else None  # kbits/s
    if key in ("frame", "drop_frames", "dup_frames"):
        return int(raw) if raw.isdigit() else None
    if key == "fps":
        try:
            return float(raw)
        except ValueError:
            return None
    if key == "out_time_us":
        return int(raw) if raw.isdigit() else None
    return raw


def feed_progress_line(acc: dict, line: str):
    """Accumulate one progress line into acc.

    Returns a finished block dict when the line closes a block
    (`progress=continue|end`), else None.
    """
    line = line.strip()
    if not line or "=" not in line:
        return None
    key, _, val = line.partition("=")
    key = key.strip()
    if key == "progress":
        block = dict(acc)
        block["_progress"] = val.strip()
        acc.clear()
        return block
    acc[key] = parse_progress_value(key, val)
    return None


# --------------------------------------------------------------------------- #
# ffmpeg runner                                                               #
# --------------------------------------------------------------------------- #

def build_ffmpeg_cmd(port: int, display: bool) -> list:
    cmd = [
        "ffmpeg", "-hide_banner",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-i", f"udp://0.0.0.0:{port}",
        "-progress", "pipe:1", "-nostats",
    ]
    if display:
        cmd += ["-f", "sdl", "HANSEL_MESH video"]
    else:
        cmd += ["-f", "null", "-"]
    return cmd


class ErrorCounter:
    """Thread-safe count of decode-error lines seen on ffmpeg stderr."""

    def __init__(self):
        self._n = 0
        self._lock = threading.Lock()

    def bump(self):
        with self._lock:
            self._n += 1

    def value(self) -> int:
        with self._lock:
            return self._n


def _drain_stderr(pipe, counter: ErrorCounter, echo: bool):
    for line in pipe:
        if is_error_line(line):
            counter.bump()
        if echo:
            # ffmpeg is chatty; only surface the damage lines.
            if is_error_line(line):
                print("[ffmpeg]", line.rstrip())


# --------------------------------------------------------------------------- #
# Sample assembly                                                             #
# --------------------------------------------------------------------------- #

def make_sample(block: dict, errors_total: int, prev: dict, now: float,
                target_fps: float) -> dict:
    """Turn a raw progress block + error count into a quality sample."""
    frame = block.get("frame")
    drop = block.get("drop_frames")
    bitrate = block.get("bitrate")
    fps = block.get("fps")

    dt = now - prev.get("ts", now)
    dt = dt if dt > 0 else None

    err_delta = errors_total - prev.get("errors_total", errors_total)
    drop_delta = (drop - prev.get("drop", drop)) if (
        drop is not None and prev.get("drop") is not None) else None

    sample = {
        "ts": round(now, 2),
        "fps": fps,
        "target_fps": target_fps,
        "fps_ratio": round(fps / target_fps, 3) if (fps and target_fps) else None,
        "frame": frame,
        "drop_total": drop,
        "drop_rate": round(drop_delta / dt, 2) if (drop_delta is not None and dt) else None,
        "errors_total": errors_total,
        "err_rate": round(err_delta / dt, 2) if dt else None,
        "bitrate_kbps": bitrate,
    }
    return sample


# --------------------------------------------------------------------------- #
# Output                                                                      #
# --------------------------------------------------------------------------- #

def emit(sample: dict, log_fh, sender, collector):
    line = json.dumps(sample)
    if log_fh:
        log_fh.write(line + "\n")
        log_fh.flush()
    if sender and collector:
        host, _, port = collector.partition(":")
        sender.sendto(json.dumps({"video": sample}).encode("utf-8"),
                      (host, int(port)))
    print(f"fps={sample.get('fps')} err/s={sample.get('err_rate')} "
          f"drop/s={sample.get('drop_rate')} br={sample.get('bitrate_kbps')}kbps")


def run(args) -> int:
    cmd = build_ffmpeg_cmd(args.port, display=not args.no_display)
    print("[video_probe] launching:", " ".join(cmd))
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1,
    )

    counter = ErrorCounter()
    threading.Thread(target=_drain_stderr,
                     args=(proc.stderr, counter, args.verbose),
                     daemon=True).start()

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.send else None
    log_fh = open(args.log, "a", encoding="utf-8") if args.log else None
    if log_fh:
        print(f"[video_probe] logging to {args.log}")

    acc: dict = {}
    prev: dict = {}
    try:
        for line in proc.stdout:
            block = feed_progress_line(acc, line)
            if block is None:
                continue
            now = time.time()
            sample = make_sample(block, counter.value(), prev, now, args.target_fps)
            emit(sample, log_fh, sender, args.send)
            prev = {"ts": now, "errors_total": sample["errors_total"],
                    "drop": sample["drop_total"]}
            if block.get("_progress") == "end":
                break
    except KeyboardInterrupt:
        print("\n[video_probe] stopping")
    finally:
        proc.terminate()
        if log_fh:
            log_fh.close()
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Measure received video quality.")
    p.add_argument("--port", type=int, default=5600, help="UDP port of the stream")
    p.add_argument("--target-fps", type=float, default=15.0, help="expected fps")
    p.add_argument("--send", default=None, help="collector host:port (e.g. 192.168.50.1:7100)")
    p.add_argument("--log", default="video_quality.jsonl", help="JSONL log file")
    p.add_argument("--no-display", action="store_true", help="measure only, no window")
    p.add_argument("--verbose", action="store_true", help="print decode-error lines")
    return p.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
