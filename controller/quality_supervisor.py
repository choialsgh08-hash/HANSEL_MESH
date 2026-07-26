#!/usr/bin/env python3
"""Video-first communication quality supervisor for HANSEL_MESH.

The robot is operator-driven, so the camera stream is the hard requirement:
small UDP control packets can still work after the H.264 stream is already too
damaged to drive safely. This module scores video quality first, then uses
RTT/loss and optional BATMAN TQ as secondary predictors.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from dataclasses import replace
import json
import math
import os
import re
import subprocess
import threading
import time
from typing import Optional


HEAD_IP = "192.168.50.10"


@dataclass
class QualityConfig:
    video_log: Optional[str] = None
    target_fps: float = 15.0
    interval: float = 0.5
    ping_ip: Optional[str] = HEAD_IP
    ping_interval: float = 2.0
    base_ssh: Optional[str] = None
    batman_interval: float = 3.0

    fps_warn_ratio: float = 0.85
    fps_danger_ratio: float = 0.60
    err_warn_rate: float = 0.20
    err_danger_rate: float = 1.00
    drop_warn_rate: float = 1.00
    drop_danger_rate: float = 3.00
    video_stale_warn_s: float = 1.00
    video_stale_danger_s: float = 2.00

    rtt_warn_ms: float = 120.0
    rtt_danger_ms: float = 180.0
    loss_warn_pct: float = 5.0
    loss_danger_pct: float = 10.0
    tq_warn: int = 200
    tq_danger: int = 180

    warn_hold_s: float = 3.0
    danger_hold_s: float = 1.5
    warn_speed_cap: float = 0.35
    danger_speed_cap: float = 0.0
    async_stale_s: float = 8.0


@dataclass
class QualityDecision:
    status: str
    raw_status: str
    speed_cap: Optional[float]
    camera_profile: int
    reasons: list[str]
    video: dict
    network: dict


class QualitySupervisor:
    def __init__(self, config: QualityConfig) -> None:
        self.config = config
        self.warn_since: Optional[float] = None
        self.danger_since: Optional[float] = None
        self.last_ping_at = 0.0
        self.last_batman_at = 0.0
        self.last_ping: dict = {}
        self.last_batman: dict = {}
        self.last_decision = QualityDecision(
            status="UNKNOWN",
            raw_status="UNKNOWN",
            speed_cap=config.danger_speed_cap,
            camera_profile=2,
            reasons=["no samples yet"],
            video={},
            network={},
        )

    def update(self, now: Optional[float] = None) -> QualityDecision:
        now = time.time() if now is None else now
        video = read_latest_video_sample(self.config.video_log) if self.config.video_log else {}

        if self.config.ping_ip and now - self.last_ping_at >= self.config.ping_interval:
            self.last_ping = ping_stats(self.config.ping_ip)
            self.last_ping_at = now

        if self.config.base_ssh and now - self.last_batman_at >= self.config.batman_interval:
            self.last_batman = batman_stats(self.config.base_ssh)
            self.last_batman_at = now

        raw_status, reasons = score_quality(video, self.last_ping, self.last_batman, self.config, now)
        status = self._apply_hysteresis(raw_status, now)
        speed_cap = None
        camera_profile = 0

        control_status = raw_status if status == "TRANSIENT" else status
        if control_status == "WARN":
            speed_cap = self.config.warn_speed_cap
            camera_profile = 1
        elif control_status in {"DANGER", "UNKNOWN"}:
            speed_cap = self.config.danger_speed_cap
            camera_profile = 2

        self.last_decision = QualityDecision(
            status=status,
            raw_status=raw_status,
            speed_cap=speed_cap,
            camera_profile=camera_profile,
            reasons=reasons,
            video=video,
            network={
                "ping": self.last_ping,
                "batman": self.last_batman,
            },
        )
        return self.last_decision

    def _apply_hysteresis(self, raw_status: str, now: float) -> str:
        if raw_status == "DANGER":
            if self.danger_since is None:
                self.danger_since = now
            self.warn_since = self.warn_since or now
        elif raw_status == "WARN":
            self.danger_since = None
            if self.warn_since is None:
                self.warn_since = now
        else:
            self.warn_since = None
            self.danger_since = None
            return raw_status

        if self.danger_since is not None and now - self.danger_since >= self.config.danger_hold_s:
            return "DANGER"
        if self.warn_since is not None and now - self.warn_since >= self.config.warn_hold_s:
            return "WARN"
        return "TRANSIENT"


class AsyncQualitySupervisor:
    """Run a :class:`QualitySupervisor` without blocking the control loop.

    ``QualitySupervisor.update()`` intentionally remains synchronous because it
    is also useful for one-shot diagnostics.  Network probes performed by that
    method may take several seconds, however, so callers that refresh motor
    commands should use this wrapper and only read :meth:`latest`.

    Until the first successful sample, after an update error, or when the last
    success becomes stale, the published decision has a zero speed cap.
    """

    def __init__(
        self,
        supervisor: QualitySupervisor,
        interval: Optional[float] = None,
    ) -> None:
        self.supervisor = supervisor
        configured_interval = supervisor.config.interval if interval is None else interval
        if not math.isfinite(configured_interval) or configured_interval <= 0:
            raise ValueError("interval must be finite and greater than zero")
        self.interval = configured_interval

        initial = copy.deepcopy(supervisor.last_decision)
        self._last_successful_decision = initial
        self._latest_decision = replace(
            initial,
            status="NOT_READY",
            speed_cap=supervisor.config.danger_speed_cap,
            camera_profile=2,
            reasons=["quality supervisor has not completed its first update"],
        )
        self._last_error: Optional[str] = None
        self._has_success = False
        self._last_success_monotonic: Optional[float] = None
        self._decision_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "AsyncQualitySupervisor":
        """Start the daemon worker; repeated calls while running are harmless."""

        with self._state_lock:
            if self._thread is not None and self._thread.is_alive():
                return self

            stop_event = threading.Event()
            thread = threading.Thread(
                target=self._run,
                args=(stop_event,),
                name="hansel-quality-supervisor",
                daemon=True,
            )
            self._stop_event = stop_event
            self._thread = thread
            thread.start()
        return self

    def stop(self, timeout: float = 1.0) -> bool:
        """Request shutdown and wait at most ``timeout`` seconds.

        The worker is a daemon thread, so a probe stuck in an operating-system
        call cannot prevent process exit.  The return value is ``True`` when
        the worker stopped within the requested bound.
        """

        timeout = max(0.0, timeout)
        with self._state_lock:
            thread = self._thread
            self._stop_event.set()

        if thread is None:
            return True

        thread.join(timeout)
        stopped = not thread.is_alive()
        if stopped:
            with self._state_lock:
                if self._thread is thread:
                    self._thread = None
        return stopped

    def latest(self) -> QualityDecision:
        """Return a thread-safe snapshot without performing network I/O."""

        with self._decision_lock:
            snapshot = copy.deepcopy(self._latest_decision)
            last_success = self._last_success_monotonic
            has_success = self._has_success
        if (
            has_success
            and last_success is not None
            and time.monotonic() - last_success
            > self.supervisor.config.async_stale_s
        ):
            age = time.monotonic() - last_success
            return replace(
                snapshot,
                status="STALE",
                speed_cap=self.supervisor.config.danger_speed_cap,
                camera_profile=2,
                reasons=list(snapshot.reasons)
                + [f"quality update stale for {age:.1f}s"],
            )
        return snapshot

    @property
    def last_error(self) -> Optional[str]:
        with self._decision_lock:
            return self._last_error

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            return self._thread is not None and self._thread.is_alive()

    def __enter__(self) -> "AsyncQualitySupervisor":
        return self.start()

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.stop()

    def _run(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            try:
                decision = self.supervisor.update()
            except Exception as exc:  # Keep motor command refresh alive on probe failures.
                error = f"quality update error: {type(exc).__name__}: {exc}"
                with self._decision_lock:
                    previous = self._last_successful_decision
                    self._latest_decision = replace(
                        previous,
                        status="ERROR",
                        speed_cap=self.supervisor.config.danger_speed_cap,
                        camera_profile=2,
                        reasons=list(previous.reasons) + [error],
                    )
                    self._last_error = error
            else:
                snapshot = copy.deepcopy(decision)
                with self._decision_lock:
                    self._last_successful_decision = snapshot
                    self._latest_decision = snapshot
                    self._last_error = None
                    self._has_success = True
                    self._last_success_monotonic = time.monotonic()

            if stop_event.wait(self.interval):
                break


def read_latest_video_sample(path: Optional[str]) -> dict:
    if not path or not os.path.exists(path):
        return {}
    try:
        with open(path, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - 16384))
            lines = fh.read().decode("utf-8", errors="ignore").splitlines()
    except OSError:
        return {}

    for line in reversed(lines):
        line = line.strip()
        if not line:
            continue
        try:
            sample = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "ts" not in sample:
            try:
                sample["ts"] = os.path.getmtime(path)
            except OSError:
                pass
        return sample
    return {}


def ping_stats(ip: str) -> dict:
    try:
        out = subprocess.run(
            ["ping", "-c", "3", "-W", "1", ip],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        return {"error": str(exc), "loss_pct": 100.0}
    return parse_ping(out.stdout + out.stderr)


def parse_ping(text: str) -> dict:
    result: dict = {}
    loss = re.search(r"(\d+(?:\.\d+)?)%\s*packet loss", text)
    if loss:
        result["loss_pct"] = float(loss.group(1))
    rtt = re.search(r"=\s*[\d.]+/([\d.]+)/[\d.]+/[\d.]+\s*ms", text)
    if rtt:
        result["rtt_avg_ms"] = float(rtt.group(1))
    if "Destination Host Unreachable" in text or "100% packet loss" in text:
        result.setdefault("loss_pct", 100.0)
    return result


def batman_stats(base_ssh: str) -> dict:
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=2", base_ssh, "batctl o"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        return {"error": str(exc)}
    tqs = parse_selected_tqs(out.stdout + out.stderr)
    if not tqs:
        return {"selected_tq_min": None}
    return {
        "selected_tq_min": min(tqs),
        "selected_tq_values": tqs,
    }


def parse_selected_tqs(text: str) -> list[int]:
    values: list[int] = []
    for line in text.splitlines():
        if "*" not in line:
            continue
        m = re.search(r"\((\d+)\)", line)
        if m:
            values.append(int(m.group(1)))
    return values


def score_quality(video: dict, ping: dict, batman: dict, config: QualityConfig, now: float) -> tuple[str, list[str]]:
    status = "GOOD"
    reasons: list[str] = []

    def raise_to(next_status: str, reason: str) -> None:
        nonlocal status
        priorities = {"GOOD": 0, "WARN": 1, "DANGER": 2}
        if priorities[next_status] > priorities[status]:
            status = next_status
        reasons.append(reason)

    def finite_number(value: object) -> Optional[float]:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    if not video:
        raise_to("DANGER", "video sample missing")
    else:
        sample_ts = finite_number(video.get("ts"))
        if sample_ts is None or sample_ts <= 0:
            raise_to("DANGER", "video timestamp missing or invalid")
        else:
            age = now - sample_ts
            if age < -1.0:
                raise_to("DANGER", f"video timestamp is {-age:.1f}s in the future")
            elif age >= config.video_stale_danger_s:
                raise_to("DANGER", f"video stale {age:.1f}s")
            elif age >= config.video_stale_warn_s:
                raise_to("WARN", f"video stale {age:.1f}s")

        target_fps = finite_number(
            video.get("target_fps", config.target_fps)
        )
        fps = finite_number(video.get("fps"))
        fps_ratio = finite_number(video.get("fps_ratio"))
        if fps_ratio is None and fps is not None and target_fps and target_fps > 0:
            fps_ratio = fps / target_fps
        if fps_ratio is None:
            raise_to("DANGER", "video FPS unavailable or invalid")
        else:
            if fps_ratio < config.fps_danger_ratio:
                raise_to("DANGER", f"fps ratio {fps_ratio:.2f}")
            elif fps_ratio < config.fps_warn_ratio:
                raise_to("WARN", f"fps ratio {fps_ratio:.2f}")

        err_rate = video.get("err_rate")
        if err_rate is not None:
            parsed_err_rate = finite_number(err_rate)
            if parsed_err_rate is None:
                raise_to("DANGER", "decode error rate invalid")
            elif parsed_err_rate > config.err_danger_rate:
                raise_to("DANGER", f"decode err/s {err_rate}")
            elif parsed_err_rate > config.err_warn_rate:
                raise_to("WARN", f"decode err/s {err_rate}")

        drop_rate = video.get("drop_rate")
        if drop_rate is not None:
            parsed_drop_rate = finite_number(drop_rate)
            if parsed_drop_rate is None:
                raise_to("DANGER", "video drop rate invalid")
            elif parsed_drop_rate > config.drop_danger_rate:
                raise_to("DANGER", f"drop/s {drop_rate}")
            elif parsed_drop_rate > config.drop_warn_rate:
                raise_to("WARN", f"drop/s {drop_rate}")

    if ping:
        loss = ping.get("loss_pct")
        if loss is not None:
            parsed_loss = finite_number(loss)
            if parsed_loss is None:
                raise_to("DANGER", "ping loss value invalid")
            elif parsed_loss > config.loss_danger_pct:
                raise_to("DANGER", f"ping loss {loss}%")
            elif parsed_loss > config.loss_warn_pct:
                raise_to("WARN", f"ping loss {loss}%")
        rtt = ping.get("rtt_avg_ms")
        if rtt is not None:
            parsed_rtt = finite_number(rtt)
            if parsed_rtt is None:
                raise_to("DANGER", "RTT value invalid")
            elif parsed_rtt > config.rtt_danger_ms:
                raise_to("DANGER", f"rtt {rtt}ms")
            elif parsed_rtt > config.rtt_warn_ms:
                raise_to("WARN", f"rtt {rtt}ms")

    tq = batman.get("selected_tq_min") if batman else None
    if tq is not None:
        parsed_tq = finite_number(tq)
        if parsed_tq is None:
            raise_to("DANGER", "BATMAN TQ value invalid")
        elif parsed_tq < config.tq_danger:
            raise_to("DANGER", f"BATMAN TQ {tq}")
        elif parsed_tq < config.tq_warn:
            raise_to("WARN", f"BATMAN TQ {tq}")

    if not reasons:
        reasons.append("healthy")
    return status, reasons


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate video-first link quality.")
    parser.add_argument("--video-log", default="video_quality.jsonl")
    parser.add_argument("--target-fps", type=float, default=15.0)
    parser.add_argument("--ping-ip", default=HEAD_IP)
    parser.add_argument("--base-ssh", default=None, help="optional base SSH target, e.g. hansel@192.168.60.1")
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args(argv)
    if not math.isfinite(args.target_fps) or args.target_fps <= 0:
        parser.error("--target-fps must be finite and greater than zero")
    if not math.isfinite(args.interval) or args.interval <= 0:
        parser.error("--interval must be finite and greater than zero")
    return args


def main() -> int:
    args = parse_args()
    config = QualityConfig(
        video_log=args.video_log,
        target_fps=args.target_fps,
        interval=args.interval,
        ping_ip=args.ping_ip,
        base_ssh=args.base_ssh,
    )
    supervisor = QualitySupervisor(config)
    while True:
        decision = supervisor.update()
        print(
            f"[quality] status={decision.status} raw={decision.raw_status} "
            f"speed_cap={decision.speed_cap} profile={decision.camera_profile} "
            f"reasons={'; '.join(decision.reasons)}"
        )
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
