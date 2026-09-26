#!/usr/bin/env python3
"""Operator client for versioned, acknowledged HANSEL UDP control."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple
import uuid

try:
    import termios
    import tty
except ImportError:  # Windows supports line mode and protocol tests, not live TTY.
    termios = None  # type: ignore[assignment]
    tty = None  # type: ignore[assignment]


REPO_ROOT = str(Path(__file__).resolve().parent.parent)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from common.control_protocol import (  # noqa: E402
    DEFAULT_TTL_MS,
    MAX_TTL_MS,
    PROTOCOL_VERSION,
    build_command,
)

try:  # noqa: E402
    from controller import quality_supervisor as quality_module
except ImportError:
    import quality_supervisor as quality_module

QualityConfig = quality_module.QualityConfig
QualitySupervisor = quality_module.QualitySupervisor
AsyncQualitySupervisor = getattr(
    quality_module,
    "AsyncQualitySupervisor",
    None,
)


COMMANDS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "x": "stop",
    " ": "stop",
    "fl": "forward_left",
    "fr": "forward_right",
    "bl": "backward_left",
    "br": "backward_right",
    "mfl": "mild_forward_left",
    "mfr": "mild_forward_right",
    "mbl": "mild_backward_left",
    "mbr": "mild_backward_right",
    "sf": "slow_forward",
    "sb": "slow_backward",
    "hu": "head_servo_up",
    "hd": "head_servo_down",
    "hc": "head_servo_center",
    "hmin": "head_servo_min",
    "hmax": "head_servo_max",
    "front": "front_motor_forward",
    "front_back": "front_motor_backward",
    "front_stop": "front_motor_stop",
    "detach": "detach_press",
    "detach_rest": "detach_rest",
    "enable": "drive_enable",
    "1": "detach_node1",
    "2": "detach_node2",
    "3": "detach_node3",
    "detach_node1": "detach_node1",
    "detach_node2": "detach_node2",
    "detach_node3": "detach_node3",
}

TARGETS = {
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
    "node3": "192.168.50.13",
}

DEFAULT_ACTIVE_TARGETS = ("head", "node1", "node2")

LIVE_KEYS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "x": "stop",
    " ": "stop",
    "e": "forward_right",
    "q": "forward_left",
    "c": "backward_right",
    "z": "backward_left",
}

ONE_SHOT_LIVE_KEYS = {
    "u": "head_servo_up",
    "j": "head_servo_down",
    "k": "head_servo_center",
    "f": "front_motor_forward",
    "v": "front_motor_stop",
}

QUALITY_SENSITIVE_MOTION_COMMANDS = {
    "forward",
    "backward",
    "left",
    "right",
    "forward_left",
    "forward_right",
    "backward_left",
    "backward_right",
    "front_motor_forward",
    "front_motor_backward",
}

DETACH_RELEASE_ACTUATORS = {
    "node1": "head",
    "node2": "node1",
    "node3": "node2",
}

MANUAL_DETACH_KEYS = {
    "1": "node1",
    "2": "node2",
    "3": "node3",
}

HEAD_ONLY_COMMANDS = {
    "head_servo_up",
    "head_servo_down",
    "head_servo_center",
    "head_servo_min",
    "head_servo_max",
    "servo_up",
    "servo_down",
    "servo_center",
    "servo_min",
    "servo_max",
    "front_motor_forward",
    "front_motor_backward",
    "front_motor_stop",
    "front_forward",
    "front_backward",
    "front_stop",
}

TARGET_SPECIFIC_COMMANDS = {
    "detach_press",
    "detach_rest",
    "relay_hold",
    "drive_disable",
    "drive_enable",
}

FORWARD_STEERING_COMMANDS = {
    "forward_left",
    "forward_right",
    "mild_forward_left",
    "mild_forward_right",
}

BACKWARD_STEERING_COMMANDS = {
    "backward_left",
    "backward_right",
    "mild_backward_left",
    "mild_backward_right",
}

HEAD_SPIN_COMMANDS = {"left", "right"}

STRAIGHT_ALL_COMMANDS = {
    "forward",
    "backward",
    "stop",
    "slow_forward",
    "slow_backward",
}


def target_items(
    target: str,
    command: str,
    active_targets: Optional[List[str]] = None,
) -> List[Tuple[str, str, str]]:
    active_names = (
        active_targets if active_targets is not None else list(TARGETS.keys())
    )
    active_names = [name for name in active_names if name in TARGETS]

    if target == "all" and command in TARGET_SPECIFIC_COMMANDS:
        print(f"[SAFE SKIP] command={command} requires a single target, not all")
        return []
    if target == "all" and command in HEAD_ONLY_COMMANDS:
        return (
            [("head", TARGETS["head"], command)]
            if "head" in active_names
            else []
        )
    if target == "all" and command in FORWARD_STEERING_COMMANDS:
        items: List[Tuple[str, str, str]] = []
        if "head" in active_names:
            items.append(("head", TARGETS["head"], command))
        items.extend(
            (name, TARGETS[name], "slow_forward")
            for name in active_names
            if name != "head"
        )
        return items
    if target == "all" and command in BACKWARD_STEERING_COMMANDS:
        items = []
        if "head" in active_names:
            items.append(("head", TARGETS["head"], command))
        items.extend(
            (name, TARGETS[name], "slow_backward")
            for name in active_names
            if name != "head"
        )
        return items
    if target == "all" and command in HEAD_SPIN_COMMANDS:
        items = []
        if "head" in active_names:
            items.append(("head", TARGETS["head"], command))
        items.extend(
            (name, TARGETS[name], "stop")
            for name in active_names
            if name != "head"
        )
        return items
    if target == "all":
        if command not in STRAIGHT_ALL_COMMANDS:
            print(f"[SAFE SKIP] command={command} is not allowed for all")
            return []
        return [
            (name, TARGETS[name], command)
            for name in active_names
        ]
    if target not in active_names:
        print(
            f"[SAFE SKIP] target={target} is not in the active moving "
            "target list"
        )
        return []
    return [(target, TARGETS[target], command)]


def parse_active_targets(raw: str) -> List[str]:
    active: List[str] = []
    for item in raw.split(","):
        name = item.strip().lower()
        if not name:
            continue
        if name not in TARGETS:
            raise argparse.ArgumentTypeError(
                f"unknown active target {name!r}; choose from "
                + ", ".join(TARGETS)
            )
        if name not in active:
            active.append(name)
    if not active:
        raise argparse.ArgumentTypeError(
            "--active-targets must contain at least one role"
        )
    return active


def active_targets_for_args(args: argparse.Namespace) -> List[str]:
    active = list(args.active_targets)
    if args.target != "all" and args.target not in active:
        active.append(args.target)
    return active


@dataclass
class CommandRef:
    session_id: str
    seq: int
    target: str
    command: str
    expected_ip: str
    expected_port: int
    event: threading.Event = field(default_factory=threading.Event)
    ack: Optional[Dict[str, object]] = None


@dataclass(frozen=True)
class DetachResult:
    command_completed: bool
    faulted: bool
    reason: str


class AckReceiver:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.running = threading.Event()
        self.lock = threading.Lock()
        self.waiters: Dict[Tuple[str, int, str], CommandRef] = {}
        self.thread = threading.Thread(
            target=self._run,
            name="hansel-ack-receiver",
            daemon=True,
        )

    def start(self) -> "AckReceiver":
        self.running.set()
        self.thread.start()
        return self

    def expect(self, ref: CommandRef) -> None:
        key = (ref.session_id, ref.seq, ref.target)
        with self.lock:
            self.waiters[key] = ref

    def cancel(self, ref: CommandRef) -> None:
        key = (ref.session_id, ref.seq, ref.target)
        with self.lock:
            self.waiters.pop(key, None)

    def wait(self, ref: CommandRef, timeout: float) -> Optional[Dict[str, object]]:
        ref.event.wait(max(0.0, timeout))
        self.cancel(ref)
        return ref.ack

    def stop(self, timeout: float = 1.0) -> bool:
        self.running.clear()
        self.thread.join(timeout=max(0.0, timeout))
        return not self.thread.is_alive()

    def _run(self) -> None:
        while self.running.is_set():
            try:
                data, peer = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return

            try:
                ack = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(ack, dict):
                continue
            if (
                ack.get("protocol_version") != PROTOCOL_VERSION
                or ack.get("type") != "ack"
            ):
                continue
            session_id = ack.get("session_id")
            seq = ack.get("seq")
            target = ack.get("target")
            if (
                not isinstance(session_id, str)
                or isinstance(seq, bool)
                or not isinstance(seq, int)
                or not isinstance(target, str)
            ):
                continue

            key = (session_id, seq, target)
            with self.lock:
                ref = self.waiters.get(key)
                if (
                    ref is not None
                    and peer[0] == ref.expected_ip
                    and peer[1] == ref.expected_port
                ):
                    ref.ack = ack
                    ref.event.set()

            if ref is None and ack.get("status") == "rejected":
                print(
                    f"[ACK] rejected target={target} seq={seq} "
                    f"reason={ack.get('reason')}"
                )


class ControlTransport:
    def __init__(
        self,
        port: int,
        ttl_ms: int = DEFAULT_TTL_MS,
        session_id: Optional[str] = None,
        sock: Optional[socket.socket] = None,
    ) -> None:
        self.port = port
        self.ttl_ms = ttl_ms
        self.session_id = session_id or uuid.uuid4().hex
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))
        self.sock.settimeout(0.1)
        self.seq_lock = threading.Lock()
        self.seq = 0
        self.acks = AckReceiver(self.sock).start()

    def next_seq(self) -> int:
        with self.seq_lock:
            self.seq += 1
            return self.seq

    def send_one(
        self,
        target: str,
        command: str,
        speed: Optional[float] = None,
        source: str = "operator",
        extra: Optional[Dict[str, object]] = None,
        expect_ack: bool = False,
    ) -> CommandRef:
        ip = TARGETS[target]
        seq = self.next_seq()
        fields: Dict[str, object] = dict(extra or {})
        if speed is not None:
            fields["speed"] = speed
        message = build_command(
            session_id=self.session_id,
            seq=seq,
            target=target,
            command=command,
            source=source,
            ttl_ms=self.ttl_ms,
            extra=fields,
        )
        ref = CommandRef(
            session_id=self.session_id,
            seq=seq,
            target=target,
            command=command,
            expected_ip=ip,
            expected_port=self.port,
        )
        if expect_ack:
            self.acks.expect(ref)
        try:
            self.sock.sendto(
                json.dumps(message, separators=(",", ":")).encode("utf-8"),
                (ip, self.port),
            )
        except OSError:
            if expect_ack:
                self.acks.cancel(ref)
            raise
        print(
            f"sent seq={seq} target={target} ip={ip} command={command}"
        )
        return ref

    def send_routed(
        self,
        target: str,
        command: str,
        speed: Optional[float],
        active_targets: Optional[List[str]] = None,
        expect_ack: bool = False,
    ) -> List[CommandRef]:
        refs = []
        for name, _ip, routed_command in target_items(
            target,
            command,
            active_targets,
        ):
            refs.append(
                self.send_one(
                    name,
                    routed_command,
                    speed,
                    expect_ack=expect_ack,
                )
            )
        return refs

    def wait_applied(
        self,
        ref: CommandRef,
        timeout: float,
    ) -> Tuple[bool, str]:
        ack = self.acks.wait(ref, timeout)
        if ack is None:
            return False, "ack_timeout"
        if ack.get("command") != ref.command:
            return False, "ack_command_mismatch"
        status = str(ack.get("status", "rejected"))
        reason = str(ack.get("reason", "missing_reason"))
        return status == "applied", reason

    def cancel_wait(self, ref: CommandRef) -> None:
        self.acks.cancel(ref)

    def close(self) -> None:
        self.acks.stop()
        self.sock.close()


def wait_for_all_applied(
    transport: ControlTransport,
    refs: Sequence[CommandRef],
    timeout: float,
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout)
    all_applied = True
    for ref in refs:
        remaining = max(0.0, deadline - time.monotonic())
        applied, reason = transport.wait_applied(ref, remaining)
        if not applied:
            print(
                f"[ACK] target={ref.target} command={ref.command} "
                f"failed: {reason}"
            )
            all_applied = False
    return all_applied


def stop_active_targets(
    transport: ControlTransport,
    active_targets: Sequence[str],
    ack_timeout: float,
) -> bool:
    applied, _ = stop_active_targets_with_refs(
        transport,
        active_targets,
        ack_timeout,
    )
    return applied


def stop_active_targets_with_refs(
    transport: ControlTransport,
    active_targets: Sequence[str],
    ack_timeout: float,
) -> Tuple[bool, Dict[str, CommandRef]]:
    refs = [
        transport.send_one(
            target,
            "stop",
            expect_ack=True,
        )
        for target in active_targets
        if target in TARGETS
    ]
    if not refs:
        return True, {}
    if not wait_for_all_applied(transport, refs, ack_timeout):
        print("[safety] not every requested target confirmed stop")
        return False, {}
    return True, {ref.target: ref for ref in refs}


def session_targets(
    target: str,
    active_targets: Sequence[str],
) -> List[str]:
    if target == "all":
        return [name for name in active_targets if name in TARGETS]
    if target in TARGETS and target in active_targets:
        return [target]
    return []


def initialize_control_session(
    transport: ControlTransport,
    target: str,
    active_targets: Sequence[str],
    ack_timeout: float,
) -> bool:
    """Establish each receiver's monotonic baseline with an acknowledged stop."""

    requested = session_targets(target, active_targets)
    print(
        "[safety] establishing control session with stop ACK: "
        + ", ".join(requested)
    )
    if not requested:
        print("[safety] abort: no active control target is available")
        return False
    if not stop_active_targets(transport, requested, ack_timeout):
        print(
            "[safety] abort: control input remains disabled until every "
            "requested target acknowledges stop"
        )
        return False
    return True


def send_detach_release(
    transport: ControlTransport,
    released_node: str,
    active_targets: Sequence[str],
    ack_timeout: float,
) -> DetachResult:
    actuator = DETACH_RELEASE_ACTUATORS.get(released_node)
    if actuator is None:
        print(
            f"[SAFE SKIP] no detach actuator is mapped for released node: "
            f"{released_node}"
        )
        return DetachResult(False, False, "missing_actuator_mapping")
    if released_node not in active_targets:
        print(
            f"[SAFE SKIP] {released_node} is not in the configured active "
            "target list"
        )
        return DetachResult(False, False, "released_node_not_active")
    if actuator not in TARGETS or released_node not in TARGETS:
        print("[SAFE SKIP] detach mapping contains an unknown target")
        return DetachResult(False, False, "invalid_actuator_mapping")
    if actuator not in active_targets:
        print(
            f"[SAFE SKIP] detach actuator {actuator} is not in the active "
            "target list"
        )
        return DetachResult(False, False, "actuator_not_active")

    print("[detach] stopping every active moving target")
    stopped, stop_refs = stop_active_targets_with_refs(
        transport,
        active_targets,
        ack_timeout,
    )
    if not stopped:
        return DetachResult(False, True, "stop_ack_unconfirmed")
    actuator_stop_ref = stop_refs.get(actuator)
    if actuator_stop_ref is None:
        print("[detach] abort: actuator stop receipt is missing")
        return DetachResult(False, True, "actuator_stop_receipt_missing")

    print(
        f"[detach] latching released node {released_node} in relay hold"
    )
    hold_ref = transport.send_one(
        released_node,
        "relay_hold",
        expect_ack=True,
    )
    hold_applied, hold_reason = transport.wait_applied(
        hold_ref,
        ack_timeout,
    )
    if not hold_applied:
        print(
            f"[detach] abort: {released_node} relay_hold was not confirmed "
            f"({hold_reason})"
        )
        return DetachResult(False, True, f"relay_hold_{hold_reason}")

    print(
        f"[detach] relay hold confirmed; actuating {actuator} detach servo"
    )
    detach_ref = transport.send_one(
        actuator,
        "detach_press",
        extra={
            "detach_context": {
                "released_node": released_node,
                "stop_and_hold_acknowledged": True,
                "actuator_stop_seq": actuator_stop_ref.seq,
            },
        },
        expect_ack=True,
    )
    detach_applied, detach_reason = transport.wait_applied(
        detach_ref,
        ack_timeout,
    )
    if not detach_applied:
        print(
            f"[detach] failed: {actuator} detach_press was not confirmed "
            f"({detach_reason}); active target list is unchanged"
        )
        stop_active_targets(transport, active_targets, ack_timeout)
        return DetachResult(False, True, f"detach_press_{detach_reason}")

    print(
        f"[detach] command sequence completed for {released_node}; "
        "physical release still requires operator/sensor confirmation"
    )
    return DetachResult(True, False, "actuator_acknowledged")


def resolve_detach_result(
    result: DetachResult,
    released_node: str,
    assume_on_ack: bool,
    live_terminal: bool,
) -> Tuple[bool, bool]:
    """Return (remove_from_active_targets, terminate_control_session)."""
    if result.faulted:
        print(
            f"[detach] SAFETY FAULT: {result.reason}. "
            "The robot remains stopped; restart control only after inspection."
        )
        return False, True
    if not result.command_completed:
        return False, False
    if assume_on_ack:
        print(
            "[detach] WARNING: treating actuator ACK as physical release "
            "because --assume-detach-on-actuator-ack was specified"
        )
        return True, False

    if live_terminal:
        print(
            f"[detach] visually confirm that {released_node} is physically "
            "separated, then press 'y'. Any other key ends the stopped session."
        )
        answer = sys.stdin.read(1).strip().lower()
        print()
        confirmed = answer == "y"
    else:
        try:
            answer = input(
                f"[detach] type CONFIRM {released_node} after visually "
                "confirming physical separation: "
            ).strip()
        except EOFError:
            answer = ""
        confirmed = answer == f"CONFIRM {released_node}"

    if confirmed:
        return True, False
    print(
        "[detach] physical release not confirmed. The node remains in the "
        "active list and the stopped control session will end."
    )
    return False, True


class _FallbackAsyncQualitySupervisor:
    """Nonblocking adapter used with an older quality_supervisor module."""

    def __init__(self, supervisor: Any, interval: Optional[float] = None) -> None:
        self.supervisor = supervisor
        self.interval = (
            supervisor.config.interval if interval is None else interval
        )
        self._latest = supervisor.last_decision
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="hansel-quality-supervisor",
            daemon=True,
        )

    def start(self) -> "_FallbackAsyncQualitySupervisor":
        self._stop_event.clear()
        self._thread.start()
        return self

    def latest(self) -> Any:
        with self._lock:
            return self._latest

    def stop(self, timeout: float = 1.0) -> bool:
        self._stop_event.set()
        self._thread.join(timeout=max(0.0, timeout))
        return not self._thread.is_alive()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            decision = self.supervisor.update()
            with self._lock:
                self._latest = decision
            self._stop_event.wait(self.interval)


def build_quality_supervisor(args: argparse.Namespace) -> Optional[Any]:
    if not args.quality_log:
        return None
    config = QualityConfig(
        video_log=args.quality_log,
        target_fps=args.quality_target_fps,
        interval=args.quality_interval,
        ping_ip=args.quality_ping_ip,
        base_ssh=args.quality_base_ssh,
        warn_speed_cap=args.quality_warn_speed,
        danger_speed_cap=0.0,
    )
    base = QualitySupervisor(config)
    async_class = AsyncQualitySupervisor or _FallbackAsyncQualitySupervisor
    return async_class(base).start()


def parse_detach_order(raw: str) -> List[str]:
    order = []
    for item in raw.split(","):
        name = item.strip()
        if name and name in TARGETS and name != "head":
            order.append(name)
    return order


def effective_speed(
    requested: Optional[float],
    speed_cap: Optional[float],
) -> Optional[float]:
    if requested is not None and not math.isfinite(requested):
        return 0.0
    if speed_cap is None:
        return requested
    if not math.isfinite(speed_cap):
        return 0.0
    base = 1.0 if requested is None else requested
    return min(base, speed_cap)


def quality_blocks_motion(decision: Any) -> bool:
    if decision is None:
        return False
    if decision.status in {"DANGER", "NOT_READY", "STALE", "ERROR", "UNKNOWN"}:
        return True
    if decision.speed_cap is None:
        return False
    return (
        not math.isfinite(decision.speed_cap)
        or decision.speed_cap <= 0.0
    )


def quality_blocks_command(decision: Any, command: str) -> bool:
    return (
        command in QUALITY_SENSITIVE_MOTION_COMMANDS
        and quality_blocks_motion(decision)
    )


def quality_sample_arms_auto_detach(decision: Any) -> bool:
    return bool(
        decision
        and decision.video
        and decision.raw_status in {"GOOD", "WARN"}
    )


AUTO_DETACH_ARM_PHRASE = "ARM AUTO DETACH"


def confirm_auto_detach_actuation() -> bool:
    """Require a deliberate preflight confirmation before servo actuation."""
    print(
        "[safety] automatic detach can physically actuate a release servo. "
        "Keep the robot supported and the detach area clear."
    )
    try:
        answer = input(
            f"[safety] type {AUTO_DETACH_ARM_PHRASE} to permit automatic "
            "detach actuation: "
        ).strip()
    except EOFError:
        answer = ""
    if answer == AUTO_DETACH_ARM_PHRASE:
        return True
    print("[safety] automatic detach was not armed; live control aborted")
    return False


def run_camera_profile_command(
    command_template: Optional[str],
    profile: int,
) -> None:
    if not command_template:
        return
    command = command_template.format(profile=profile)
    print(f"[quality] camera profile command: {command}")
    subprocess.Popen(command, shell=True)


def send_camera_profile(
    transport: ControlTransport,
    profile: int,
    dest_ip: str,
    dest_port: int,
    camera_transport: str,
) -> CommandRef:
    ref = transport.send_one(
        "head",
        "camera_profile",
        source="quality-supervisor",
        extra={
            "profile": profile,
            "dest_ip": dest_ip,
            "dest_port": dest_port,
            "transport": camera_transport,
        },
        expect_ack=True,
    )
    print(f"[quality] requested camera profile={profile} from head")
    return ref


@dataclass
class PendingCameraProfile:
    profile: int
    ref: CommandRef
    sent_at: float


class CameraProfileSync:
    """Non-blocking ACK/retry state for quality-driven camera profiles.

    Waiting synchronously in the live drive loop would stop motion refreshes
    long enough to trip the motor watchdog.  This state machine checks the
    background ACK receiver on each loop iteration instead.
    """

    def __init__(
        self,
        initial_profile: Optional[int] = None,
        retry_delay_s: float = 1.0,
    ) -> None:
        self.applied_profile = initial_profile
        self.retry_delay_s = retry_delay_s
        self.pending: Optional[PendingCameraProfile] = None
        self.retry_after = 0.0

    def update(
        self,
        transport: ControlTransport,
        desired_profile: int,
        dest_ip: str,
        dest_port: int,
        camera_transport: str,
        ack_timeout: float,
        now: Optional[float] = None,
    ) -> None:
        current = time.monotonic() if now is None else now
        pending = self.pending

        if pending is not None and pending.profile != desired_profile:
            transport.cancel_wait(pending.ref)
            self.pending = None
            self.retry_after = current
            pending = None

        if pending is not None:
            ack_ready = pending.ref.event.is_set()
            timed_out = current - pending.sent_at >= ack_timeout
            if not ack_ready and not timed_out:
                return
            applied, reason = transport.wait_applied(pending.ref, 0.0)
            self.pending = None
            if applied:
                self.applied_profile = pending.profile
                self.retry_after = 0.0
                print(
                    f"[quality] camera profile={pending.profile} "
                    f"confirmed by head ({reason})"
                )
            else:
                self.retry_after = current + self.retry_delay_s
                print(
                    f"[quality] camera profile={pending.profile} "
                    f"not applied ({reason}); retry scheduled"
                )

        if desired_profile == self.applied_profile:
            return
        if self.pending is not None or current < self.retry_after:
            return

        try:
            ref = send_camera_profile(
                transport,
                desired_profile,
                dest_ip,
                dest_port,
                camera_transport,
            )
        except OSError as exc:
            self.retry_after = current + self.retry_delay_s
            print(
                f"[quality] camera profile send failed: {exc}; "
                "retry scheduled"
            )
            return
        self.pending = PendingCameraProfile(
            profile=desired_profile,
            ref=ref,
            sent_at=current,
        )

    def close(self, transport: ControlTransport) -> None:
        if self.pending is not None:
            transport.cancel_wait(self.pending.ref)
            self.pending = None


def run_line_mode(args: argparse.Namespace) -> int:
    transport = ControlTransport(
        port=args.port,
        ttl_ms=args.command_ttl_ms,
        session_id=args.session_id,
    )
    target = args.target
    active_targets = active_targets_for_args(args)
    detached = set()

    print("End-to-end mesh control client")
    print(f"protocol={PROTOCOL_VERSION} session={transport.session_id}")
    print("commands: w=forward s=backward a=left d=right x=stop quit=quit")
    print(
        "extra: fl/fr/bl/br, hu/hd/hc/hmin/hmax, front/front_stop, "
        "detach/detach_rest"
    )
    print(
        "manual detach: 1=head releases node1, 2=node1 releases node2, "
        "3=node2 releases node3"
    )
    print("change target: t head | t node1 | t node2 | t node3 | t all")

    if not initialize_control_session(
        transport,
        target,
        active_targets,
        args.ack_timeout,
    ):
        transport.close()
        return 3

    try:
        while True:
            try:
                raw = input(f"[{target}]> ").strip()
            except EOFError:
                break
            if not raw:
                continue
            if raw in {"q", "quit"}:
                break
            if raw.startswith("t "):
                next_target = raw.split(maxsplit=1)[1]
                if next_target != "all" and next_target not in TARGETS:
                    print(f"unknown target: {next_target}")
                    continue
                if not initialize_control_session(
                    transport,
                    next_target,
                    active_targets,
                    args.ack_timeout,
                ):
                    print(f"[safety] target remains {target}")
                    continue
                target = next_target
                continue

            command = COMMANDS.get(raw, raw)
            if command.startswith("detach_node"):
                released_node = command.replace("detach_", "", 1)
                if released_node in detached:
                    print(f"[SAFE SKIP] {released_node} is already detached")
                    continue
                detach_result = send_detach_release(
                    transport,
                    released_node,
                    active_targets,
                    args.ack_timeout,
                )
                remove_node, terminate_session = resolve_detach_result(
                    detach_result,
                    released_node,
                    args.assume_detach_on_actuator_ack,
                    live_terminal=False,
                )
                if remove_node:
                    detached.add(released_node)
                    active_targets = [
                        name
                        for name in active_targets
                        if name != released_node
                    ]
                    print(
                        f"[detach] active moving targets now: {active_targets}"
                    )
                if terminate_session:
                    return 3
                continue

            if command == "detach_press" and not args.allow_unsafe_raw_detach:
                print(
                    "[SAFE SKIP] raw detach_press bypasses stop/relay_hold. "
                    "Use 1/2/3 for a safe release sequence, or explicitly pass "
                    "--allow-unsafe-raw-detach for bench testing."
                )
                continue

            for _ in range(args.repeat):
                transport.send_routed(
                    target,
                    command,
                    args.speed,
                    active_targets,
                )
                time.sleep(args.repeat_delay)
    finally:
        stop_active_targets(
            transport,
            active_targets,
            min(args.ack_timeout, 0.5),
        )
        transport.close()

    return 0


def run_live_mode(args: argparse.Namespace) -> int:
    if termios is None or tty is None:
        print(
            "error: --live requires a Linux/POSIX terminal. "
            "Windows can use line mode and protocol/server dry-run tests.",
            file=sys.stderr,
        )
        return 2
    if not sys.stdin.isatty():
        print(
            "error: --live requires an interactive terminal; "
            "use line mode for redirected input.",
            file=sys.stderr,
        )
        return 2

    if args.auto_detach and not confirm_auto_detach_actuation():
        return 2

    transport = ControlTransport(
        port=args.port,
        ttl_ms=args.command_ttl_ms,
        session_id=args.session_id,
    )
    target = args.target
    active_command = "stop"
    last_send = 0.0
    active_targets = active_targets_for_args(args)
    detach_order = parse_detach_order(args.detach_order)
    detached = set()
    last_detach_at = 0.0
    last_quality_status = None
    camera_profile_sync = CameraProfileSync()
    last_local_camera_profile: Optional[int] = None
    quality_stop_sent = False
    auto_detach_armed = False

    if not initialize_control_session(
        transport,
        target,
        active_targets,
        args.ack_timeout,
    ):
        transport.close()
        return 3

    supervisor = build_quality_supervisor(args)
    old_settings = termios.tcgetattr(sys.stdin)

    print("Live mesh control")
    print(f"protocol={PROTOCOL_VERSION} session={transport.session_id}")
    print(
        "drive: w/s, steer head only: a/d/q/e/z/c, stop: x or space, "
        "quit: Ctrl-C"
    )
    print(
        "one-shot: u=head up, j=head down, k=head center, "
        "f=front motor, v=front stop"
    )
    print(
        "manual detach: 1=head releases node1, 2=node1 releases node2, "
        "3=node2 releases node3"
    )
    print(
        f"target={target} "
        f"speed={args.speed if args.speed is not None else 'role default'}"
    )
    if supervisor:
        print(
            f"quality supervision: log={args.quality_log} "
            f"warn_speed={args.quality_warn_speed} "
            f"auto_detach={args.auto_detach}"
        )

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            decision = supervisor.latest() if supervisor else None
            motion_blocked = quality_blocks_motion(decision)
            send_speed = effective_speed(
                args.speed,
                decision.speed_cap if decision else None,
            )

            if decision and decision.status != last_quality_status:
                print()
                print(
                    f"[quality] status={decision.status} "
                    f"raw={decision.raw_status} "
                    f"speed_cap={decision.speed_cap} "
                    f"profile={decision.camera_profile} "
                    f"reasons={'; '.join(decision.reasons)}"
                )
                last_quality_status = decision.status

            if (
                args.auto_detach
                and not auto_detach_armed
                and quality_sample_arms_auto_detach(decision)
            ):
                auto_detach_armed = True
                print(
                    "[quality] auto-detach armed after a usable video "
                    "sample was observed"
                )

            if decision:
                if args.auto_camera_profile:
                    camera_profile_sync.update(
                        transport,
                        decision.camera_profile,
                        args.camera_dest_ip,
                        args.camera_dest_port,
                        args.camera_transport,
                        args.ack_timeout,
                    )
                if decision.camera_profile != last_local_camera_profile:
                    run_camera_profile_command(
                        args.camera_profile_cmd,
                        decision.camera_profile,
                    )
                    last_local_camera_profile = decision.camera_profile

            if motion_blocked:
                if active_command != "stop" or not quality_stop_sent:
                    transport.send_routed(
                        target,
                        "stop",
                        send_speed,
                        active_targets,
                    )
                    active_command = "stop"
                    quality_stop_sent = True
                if (
                    decision
                    and decision.status == "DANGER"
                    and args.auto_detach
                    and auto_detach_armed
                    and time.monotonic() - last_detach_at
                    >= args.detach_cooldown
                ):
                    next_detach = next(
                        (
                            name
                            for name in detach_order
                            if name in active_targets
                            and name not in detached
                        ),
                        None,
                    )
                    if next_detach:
                        print(
                            f"[quality] DANGER persists. "
                            f"Detaching {next_detach}."
                        )
                        detach_result = send_detach_release(
                            transport,
                            next_detach,
                            active_targets,
                            args.ack_timeout,
                        )
                        remove_node, terminate_session = resolve_detach_result(
                            detach_result,
                            next_detach,
                            args.assume_detach_on_actuator_ack,
                            live_terminal=True,
                        )
                        if remove_node:
                            detached.add(next_detach)
                            active_targets = [
                                name
                                for name in active_targets
                                if name != next_detach
                            ]
                            last_detach_at = time.monotonic()
                            print(
                                "[quality] active moving targets now: "
                                f"{active_targets}"
                            )
                        if terminate_session:
                            return 3
            else:
                quality_stop_sent = False

            ready, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                args.send_interval,
            )
            if ready:
                key = sys.stdin.read(1)
                if key == "\x03":
                    raise KeyboardInterrupt
                if key in LIVE_KEYS:
                    active_command = LIVE_KEYS[key]
                    if quality_blocks_command(decision, active_command):
                        print(
                            f"[quality] {decision.status}: drive command "
                            "suppressed until quality recovers"
                        )
                        active_command = "stop"
                    transport.send_routed(
                        target,
                        active_command,
                        send_speed,
                        active_targets,
                    )
                    last_send = time.monotonic()
                elif key in ONE_SHOT_LIVE_KEYS:
                    one_shot_command = ONE_SHOT_LIVE_KEYS[key]
                    if quality_blocks_command(decision, one_shot_command):
                        print(
                            f"[quality] {decision.status}: "
                            f"{one_shot_command} suppressed until quality "
                            "recovers"
                        )
                        continue
                    transport.send_routed(
                        target,
                        one_shot_command,
                        send_speed,
                        active_targets,
                    )
                elif key in MANUAL_DETACH_KEYS:
                    released_node = MANUAL_DETACH_KEYS[key]
                    if released_node in detached:
                        print(
                            f"[SAFE SKIP] {released_node} is already detached"
                        )
                        continue
                    active_command = "stop"
                    detach_result = send_detach_release(
                        transport,
                        released_node,
                        active_targets,
                        args.ack_timeout,
                    )
                    remove_node, terminate_session = resolve_detach_result(
                        detach_result,
                        released_node,
                        args.assume_detach_on_actuator_ack,
                        live_terminal=True,
                    )
                    if remove_node:
                        detached.add(released_node)
                        active_targets = [
                            name
                            for name in active_targets
                            if name != released_node
                        ]
                        print(
                            f"[detach] active moving targets now: "
                            f"{active_targets}"
                        )
                    if terminate_session:
                        return 3

            now = time.monotonic()
            if (
                active_command != "stop"
                and now - last_send >= args.send_interval
            ):
                transport.send_routed(
                    target,
                    active_command,
                    send_speed,
                    active_targets,
                )
                last_send = now

    except KeyboardInterrupt:
        print()
        return 0
    finally:
        camera_profile_sync.close(transport)
        stop_active_targets(
            transport,
            active_targets,
            min(args.ack_timeout, 0.5),
        )
        if supervisor:
            supervisor.stop()
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_settings,
        )
        transport.close()


def run(args: argparse.Namespace) -> int:
    if args.live:
        return run_live_mode(args)
    return run_line_mode(args)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send versioned UDP commands directly to mesh node IPs."
    )
    parser.add_argument(
        "--target",
        default="all",
        choices=sorted(TARGETS) + ["all"],
        help="initial target",
    )
    parser.add_argument(
        "--active-targets",
        type=parse_active_targets,
        default=list(DEFAULT_ACTIVE_TARGETS),
        metavar="ROLE[,ROLE...]",
        help=(
            "roles physically present in the moving chain "
            "(default: head,node1,node2; add node3 explicitly when installed)"
        ),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=7000,
        help="node UDP control port",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help="0.0-1.0 speed scale sent with drive commands",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="line mode send count; each repeat has a new sequence number",
    )
    parser.add_argument(
        "--repeat-delay",
        type=float,
        default=0.01,
        help="seconds between line mode repeats",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="send current key command repeatedly for driving (Linux terminal)",
    )
    parser.add_argument(
        "--send-interval",
        type=float,
        default=0.1,
        help="live mode send interval",
    )
    parser.add_argument(
        "--command-ttl-ms",
        type=int,
        default=DEFAULT_TTL_MS,
        help="maximum excess network delay for non-stop commands",
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=1.5,
        help="bounded wait for each safety-critical detach ACK sequence",
    )
    parser.add_argument(
        "--session-id",
        default=None,
        help="optional stable test session ID; random by default",
    )
    parser.add_argument(
        "--quality-log",
        default=None,
        help="enable video-first supervision using this video_probe JSONL log",
    )
    parser.add_argument(
        "--quality-target-fps",
        type=float,
        default=15.0,
        help="expected camera fps for quality supervision",
    )
    parser.add_argument(
        "--quality-interval",
        type=float,
        default=0.5,
        help="quality evaluation interval",
    )
    parser.add_argument(
        "--quality-ping-ip",
        default="192.168.50.10",
        help="IP to ping for RTT/loss while supervising",
    )
    parser.add_argument(
        "--quality-base-ssh",
        default=None,
        help="optional base SSH target for BATMAN TQ",
    )
    parser.add_argument(
        "--quality-warn-speed",
        type=float,
        default=0.35,
        help="speed cap while video/network is in WARN",
    )
    parser.add_argument(
        "--auto-detach",
        action="store_true",
        help="on sustained DANGER, stop and detach nodes in --detach-order",
    )
    parser.add_argument(
        "--detach-order",
        default="node2,node1",
        help="comma-separated released-node order",
    )
    parser.add_argument(
        "--detach-cooldown",
        type=float,
        default=6.0,
        help="minimum seconds between automatic detach actions",
    )
    parser.add_argument(
        "--assume-detach-on-actuator-ack",
        action="store_true",
        help=(
            "UNSAFE: remove a node from the active list after software ACK "
            "without visual/physical confirmation"
        ),
    )
    parser.add_argument(
        "--allow-unsafe-raw-detach",
        action="store_true",
        help=(
            "UNSAFE bench mode: allow the line-mode 'detach' command; the "
            "server must also explicitly allow unsafe raw detach"
        ),
    )
    parser.add_argument(
        "--auto-camera-profile",
        action="store_true",
        help=(
            "request profile-only restarts through the head's managed camera "
            "service; destination and transport remain administrator-controlled"
        ),
    )
    parser.add_argument(
        "--camera-dest-ip",
        default="192.168.60.2",
        help="laptop IP receiving the head camera stream",
    )
    parser.add_argument(
        "--camera-dest-port",
        type=int,
        default=5600,
        help="laptop UDP port receiving the head camera stream",
    )
    parser.add_argument(
        "--camera-transport",
        choices=("rtp", "raw"),
        default="rtp",
        help="head camera transport",
    )
    parser.add_argument(
        "--camera-profile-cmd",
        default=None,
        help=(
            "optional local shell command run for a new camera profile; "
            "use {profile}"
        ),
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.speed is not None and (
        not math.isfinite(args.speed) or not 0.0 <= args.speed <= 1.0
    ):
        parser.error("--speed must be between 0.0 and 1.0")
    if args.repeat < 1:
        parser.error("--repeat must be at least 1")
    if not math.isfinite(args.repeat_delay) or args.repeat_delay < 0:
        parser.error("--repeat-delay cannot be negative")
    if not math.isfinite(args.send_interval) or args.send_interval <= 0:
        parser.error("--send-interval must be greater than zero")
    if not 1 <= args.command_ttl_ms <= MAX_TTL_MS:
        parser.error(
            f"--command-ttl-ms must be between 1 and {MAX_TTL_MS}"
        )
    if not math.isfinite(args.ack_timeout) or args.ack_timeout <= 0:
        parser.error("--ack-timeout must be greater than zero")
    if (
        not math.isfinite(args.quality_target_fps)
        or args.quality_target_fps <= 0
    ):
        parser.error("--quality-target-fps must be greater than zero")
    if (
        not math.isfinite(args.quality_interval)
        or args.quality_interval <= 0
    ):
        parser.error("--quality-interval must be greater than zero")
    if (
        not math.isfinite(args.quality_warn_speed)
        or not 0.0 <= args.quality_warn_speed <= 1.0
    ):
        parser.error("--quality-warn-speed must be between 0.0 and 1.0")
    if (
        not math.isfinite(args.detach_cooldown)
        or args.detach_cooldown < 0
    ):
        parser.error("--detach-cooldown cannot be negative")
    if not 1 <= args.camera_dest_port <= 65535:
        parser.error("--camera-dest-port must be between 1 and 65535")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
