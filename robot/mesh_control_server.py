#!/usr/bin/env python3
"""Versioned, acknowledged UDP control server for a HANSEL mesh node."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from typing import Dict, List, Optional, Sequence, Tuple, Union


REPO_ROOT = str(Path(__file__).resolve().parent.parent)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from common.control_protocol import (  # noqa: E402
    MAX_TTL_MS,
    PROTOCOL_VERSION,
    ProtocolValidator,
    build_ack,
    build_command,
    normalize_command,
)

try:  # noqa: E402
    from robot.motor_driver import build_robot_controller
except ImportError:  # Direct execution from the robot directory.
    from motor_driver import build_robot_controller


Network = Union[ipaddress.IPv4Network, ipaddress.IPv6Network]

MOTION_COMMANDS = {
    "forward",
    "backward",
    "left",
    "right",
    "forward_left",
    "forward_right",
    "backward_left",
    "backward_right",
    "mild_forward_left",
    "mild_forward_right",
    "mild_backward_left",
    "mild_backward_right",
    "slow_forward",
    "slow_backward",
    "front_motor_forward",
    "front_motor_backward",
    "front_forward",
    "front_backward",
}

FULL_STOP_COMMANDS = {
    "stop",
    "relay_hold",
    "drive_disable",
}

DETACH_ACTUATOR_BY_RELEASED_NODE = {
    "node1": "head",
    "node2": "node1",
    "node3": "node2",
}

DRIVE_DISABLE_COMMANDS = {
    "relay_hold",
    "drive_disable",
}

DRIVE_ENABLE_COMMAND = "drive_enable"

MANAGED_CAMERA_SERVICE = "hansel-camera.service"
MANAGED_CAMERA_PROFILE_FILE = "/run/hansel-camera-profile"
ALLOWED_CAMERA_PROFILES = {
    "custom",
    "0",
    "1",
    "2",
    "3",
    "high",
    "medium",
    "low",
    "survival",
}

DRIVE_STATE_SCHEMA_VERSION = 1
DEFAULT_DRIVE_STATE_DIR = "/var/lib/hansel-mesh"
DETACH_STOP_WINDOW_NS = 10_000_000_000


def reject_nonstandard_json_constant(value: str) -> object:
    raise ValueError(f"non-standard JSON constant: {value}")


class DriveLatchStore:
    """Atomic, per-physical-Pi persistence for the propulsion safety latch."""

    def __init__(self, state_dir: Union[str, Path], role: str) -> None:
        self.state_dir = Path(state_dir)
        self.role = role
        # The latch belongs to the physical robot, not its configurable role.
        # A role change must never make a detached unit appear attached.
        self.path = self.state_dir / "drive-latch.json"
        self.enable_pending_path = (
            self.state_dir / "drive-enable.pending"
        )

    def load_drive_enabled(self) -> bool:
        """Return the saved state; anything invalid fails closed."""
        try:
            self.enable_pending_path.stat()
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(
                f"[{self.role}] drive enable transaction marker unreadable; "
                f"failing closed: {exc}"
            )
            return False
        else:
            print(
                f"[{self.role}] unfinished drive enable transaction found; "
                "failing closed"
            )
            return False

        try:
            raw = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            # Migrate conservatively from the early per-role Phase 0 format.
            # Any disabled, corrupt, or unreadable legacy record keeps this
            # physical Pi disabled after a role change.
            legacy_paths = list(
                self.state_dir.glob("drive-latch-*.json")
            )
            if not legacy_paths:
                # A Pi which has never been detached is still in the chain.
                return True
            return all(
                self._load_state_file(path)
                for path in legacy_paths
            )
        except (OSError, UnicodeError) as exc:
            print(
                f"[{self.role}] drive latch state unreadable; "
                f"failing closed: {exc}"
            )
            return False

        return self._decode_state(raw, self.path)

    def _load_state_file(self, path: Path) -> bool:
        try:
            raw = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            print(
                f"[{self.role}] legacy drive latch {path.name} unreadable; "
                f"failing closed: {exc}"
            )
            return False
        return self._decode_state(raw, path)

    def _decode_state(self, raw: str, path: Path) -> bool:
        try:
            state = json.loads(raw)
        except json.JSONDecodeError as exc:
            print(
                f"[{self.role}] drive latch {path.name} corrupt; "
                f"failing closed: {exc}"
            )
            return False

        if (
            not isinstance(state, dict)
            or state.get("schema_version") != DRIVE_STATE_SCHEMA_VERSION
            or not isinstance(state.get("role"), str)
            or not isinstance(state.get("drive_enabled"), bool)
        ):
            print(
                f"[{self.role}] drive latch {path.name} invalid; "
                "failing closed"
            )
            return False
        return bool(state["drive_enabled"])

    def save_drive_enabled(self, drive_enabled: bool) -> None:
        """Durably replace physical-Pi state without exposing partial JSON."""
        state = {
            "schema_version": DRIVE_STATE_SCHEMA_VERSION,
            "role": self.role,
            "drive_enabled": bool(drive_enabled),
        }
        encoded = (
            json.dumps(state, sort_keys=True, separators=(",", ":")) + "\n"
        )
        self._atomic_replace(self.path, encoded)

    def begin_drive_enable(self) -> None:
        """Persist a fail-closed marker before writing an enabled state."""
        marker = {
            "schema_version": DRIVE_STATE_SCHEMA_VERSION,
            "role": self.role,
            "operation": "drive_enable",
        }
        encoded = (
            json.dumps(marker, sort_keys=True, separators=(",", ":"))
            + "\n"
        )
        self._atomic_replace(self.enable_pending_path, encoded)

    def commit_drive_enable(self) -> None:
        """Remove the pending marker only after hardware enable succeeds."""
        self.enable_pending_path.unlink()
        self._fsync_directory()

    def _prepare_state_dir(self) -> None:
        self.state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            os.chmod(self.state_dir, 0o700)
        except OSError:
            # systemd StateDirectory owns the production directory. chmod may
            # be unsupported on a non-POSIX development host.
            pass

    def _atomic_replace(self, path: Path, encoded: str) -> None:
        self._prepare_state_dir()
        temporary_path = self.state_dir / (
            f".{path.name}.{os.getpid()}.{time.monotonic_ns()}.tmp"
        )

        try:
            with temporary_path.open("x", encoding="utf-8", newline="\n") as handle:
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())
            try:
                os.chmod(temporary_path, 0o600)
            except OSError:
                pass
            os.replace(temporary_path, path)
            self._fsync_directory()
        finally:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass

    def _fsync_directory(self) -> None:
        if os.name != "posix":
            return
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory_fd = os.open(str(self.state_dir), flags)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)


def command_starts_motion(role: str, command: str) -> bool:
    normalized = normalize_command(command)
    if normalized not in MOTION_COMMANDS:
        return False
    if role != "head" and normalized in {
        "left",
        "right",
        "front_motor_forward",
        "front_motor_backward",
        "front_forward",
        "front_backward",
    }:
        return False
    return True


def command_stops_all_motion(command: str) -> bool:
    return normalize_command(command) in FULL_STOP_COMMANDS


def parse_source_networks(values: Sequence[str]) -> List[Network]:
    networks: List[Network] = []
    for raw_value in values:
        for item in raw_value.split(","):
            item = item.strip()
            if item:
                networks.append(ipaddress.ip_network(item, strict=False))
    return networks


def source_is_allowed(source_ip: str, networks: Sequence[Network]) -> bool:
    if not networks:
        return True
    try:
        address = ipaddress.ip_address(source_ip)
    except ValueError:
        return False
    return any(address in network for network in networks)


def _write_camera_profile_override(path: Path, profile: str) -> None:
    """Atomically publish a root-owned, data-only camera profile override."""

    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=str(path.parent),
    )
    try:
        os.chmod(temporary_name, 0o644)
        with os.fdopen(fd, "w", encoding="ascii", newline="\n") as handle:
            fd = -1
            handle.write(profile)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
        if os.name != "nt":
            directory_fd = os.open(str(path.parent), os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    finally:
        if fd >= 0:
            os.close(fd)
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def _restore_camera_profile_override(
    path: Path,
    previous_profile: Optional[str],
) -> bool:
    """Best-effort rollback after systemd rejects a queued restart."""

    try:
        if previous_profile is None:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            if os.name != "nt":
                directory_fd = os.open(str(path.parent), os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
        else:
            _write_camera_profile_override(path, previous_profile)
    except OSError as exc:
        print(f"[head] cannot roll back camera profile override: {exc}")
        return False
    return True


def request_managed_camera_profile(profile: str) -> Tuple[bool, str]:
    """Queue a profile-only restart through the production camera service."""

    if profile not in ALLOWED_CAMERA_PROFILES:
        return False, "invalid_camera_profile"

    service = os.environ.get(
        "HANSEL_CAMERA_SYSTEMD_UNIT",
        MANAGED_CAMERA_SERVICE,
    )
    profile_path = Path(
        os.environ.get(
            "HANSEL_CAMERA_PROFILE_FILE",
            MANAGED_CAMERA_PROFILE_FILE,
        )
    )
    quiet = {
        "stdin": subprocess.DEVNULL,
        "stdout": subprocess.DEVNULL,
        "stderr": subprocess.DEVNULL,
        "check": False,
    }

    try:
        active = subprocess.run(
            ["systemctl", "is-active", "--quiet", service],
            **quiet,
        )
    except OSError as exc:
        print(f"[head] cannot query managed camera service: {exc}")
        return False, "camera_service_unavailable"
    if active.returncode != 0:
        print(
            "[head] camera profile rejected; managed camera service is "
            "not active"
        )
        return False, "camera_service_inactive"

    previous_profile: Optional[str] = None
    try:
        if profile_path.exists():
            previous_profile = profile_path.read_text(
                encoding="ascii"
            ).strip()
            if previous_profile not in ALLOWED_CAMERA_PROFILES:
                print("[head] existing camera profile override is invalid")
                return False, "camera_profile_state_invalid"
    except (OSError, UnicodeError) as exc:
        print(f"[head] cannot read camera profile override: {exc}")
        return False, "camera_profile_state_unreadable"

    try:
        _write_camera_profile_override(profile_path, profile)
    except OSError as exc:
        print(f"[head] cannot write camera profile override: {exc}")
        return False, "camera_profile_write_failed"

    try:
        queued = subprocess.run(
            [
                "systemctl",
                "--no-block",
                "try-restart",
                service,
            ],
            **quiet,
        )
    except OSError as exc:
        print(f"[head] cannot queue managed camera restart: {exc}")
        if not _restore_camera_profile_override(
            profile_path,
            previous_profile,
        ):
            return False, "camera_profile_rollback_failed"
        return False, "camera_service_unavailable"
    if queued.returncode != 0:
        print("[head] managed camera profile restart could not be queued")
        if not _restore_camera_profile_override(
            profile_path,
            previous_profile,
        ):
            return False, "camera_profile_rollback_failed"
        return False, "camera_service_restart_failed"

    print(
        f"[head] queued managed camera profile={profile}; destination and "
        "transport remain administrator-controlled"
    )
    return True, "camera_profile_restart_queued"


def handle_camera_command(
    role: str,
    command: str,
    message: Dict[str, object],
    allow_profile_restart: bool = False,
) -> Optional[Tuple[bool, str]]:
    normalized = normalize_command(command)
    if not (normalized == "camera_profile" or normalized.startswith("camera_profile_")):
        return None

    if role != "head":
        print(f"[{role}] camera profile command rejected; camera is controlled by head")
        return False, "camera_requires_head"

    profile = message.get("profile")
    if profile is None and normalized.startswith("camera_profile_"):
        profile = normalized.rsplit("_", 1)[1]
    if profile is None:
        profile = 0

    profile_text = str(profile)
    if profile_text not in ALLOWED_CAMERA_PROFILES:
        print(f"[{role}] invalid camera profile: {profile_text}")
        return False, "invalid_camera_profile"

    if not allow_profile_restart:
        return request_managed_camera_profile(profile_text)

    dest_ip = str(message.get("dest_ip", "192.168.60.2"))
    try:
        ipaddress.ip_address(dest_ip)
    except ValueError:
        print(f"[{role}] invalid camera destination IP: {dest_ip}")
        return False, "invalid_camera_destination"

    try:
        dest_port = int(message.get("dest_port", 5600))
    except (TypeError, ValueError):
        return False, "invalid_camera_destination_port"
    if not 1 <= dest_port <= 65535:
        return False, "invalid_camera_destination_port"

    transport = str(
        message.get("transport", os.environ.get("CAMERA_TRANSPORT", "rtp"))
    )
    if transport not in {"rtp", "raw"}:
        print(f"[{role}] invalid camera transport: {transport}")
        return False, "invalid_camera_transport"

    script = os.environ.get(
        "HANSEL_CAMERA_RESTART_SCRIPT",
        "/home/hansel/HANSEL_MESH/scripts/restart_camera_profile.sh",
    )
    if not os.path.exists(script):
        print(f"[{role}] camera restart script not found: {script}")
        return False, "camera_restart_script_not_found"

    print(
        f"[{role}] restarting camera profile={profile_text} "
        f"transport={transport} dest={dest_ip}:{dest_port}"
    )
    env = os.environ.copy()
    env["CAMERA_TRANSPORT"] = transport
    subprocess.Popen(
        ["bash", script, profile_text, dest_ip, str(dest_port)],
        env=env,
    )
    return True, "applied"


def apply_command(
    controller: object,
    role: str,
    command: str,
    message: Dict[str, object],
    allow_camera_profile_restart: bool = False,
) -> Tuple[bool, str]:
    print(
        f"[{role}] apply command={command} seq={message.get('seq')} "
        f"source={message.get('source', 'unknown')}"
    )
    camera_result = handle_camera_command(
        role,
        command,
        message,
        allow_profile_restart=allow_camera_profile_restart,
    )
    if camera_result is not None:
        return camera_result

    result = controller.handle_command(command, message)  # type: ignore[attr-defined]
    if isinstance(result, tuple) and len(result) == 2:
        return bool(result[0]), str(result[1])
    # Compatibility for external/custom controllers using the old None return.
    return True, "applied"


@dataclass(frozen=True)
class ProcessResult:
    ack: Dict[str, object]
    message: Optional[Dict[str, object]]
    applied: bool
    command: str


class ControlServerCore:
    """Pure packet-processing core, separated from the blocking UDP loop."""

    def __init__(
        self,
        controller: object,
        role: str,
        allowed_sources: Optional[Sequence[Network]] = None,
        allow_legacy_plaintext: bool = False,
        allow_unsafe_raw_detach: bool = False,
        allow_camera_profile_restart: bool = False,
        max_ttl_ms: int = MAX_TTL_MS,
        drive_latch_store: Optional[DriveLatchStore] = None,
    ) -> None:
        self.controller = controller
        self.role = role
        self.allowed_sources = list(allowed_sources or [])
        self.allow_legacy_plaintext = allow_legacy_plaintext
        self.allow_unsafe_raw_detach = allow_unsafe_raw_detach
        self.allow_camera_profile_restart = allow_camera_profile_restart
        self.validator = ProtocolValidator(max_ttl_ms=max_ttl_ms)
        self._legacy_seq: Dict[Tuple[str, int], int] = {}
        self.drive_latch_store = drive_latch_store
        self._recent_stops: Dict[Tuple[str, str], Tuple[int, int]] = {}

    def process_datagram(
        self,
        data: bytes,
        peer: Tuple[str, int],
        now_monotonic_ns: Optional[int] = None,
    ) -> ProcessResult:
        recv_monotonic_ns = (
            time.monotonic_ns()
            if now_monotonic_ns is None
            else now_monotonic_ns
        )

        if not source_is_allowed(peer[0], self.allowed_sources):
            request_identity: Optional[Dict[str, object]] = None
            try:
                decoded_identity = json.loads(data.decode("utf-8"))
                if isinstance(decoded_identity, dict):
                    request_identity = decoded_identity
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
            ack = build_ack(
                request_identity,
                self.role,
                "rejected",
                "source_not_allowed",
                server_monotonic_ns=recv_monotonic_ns,
            )
            return ProcessResult(ack, None, False, "")

        message, decode_reason = self._decode_datagram(
            data,
            peer,
            recv_monotonic_ns,
        )
        if message is None:
            ack = build_ack(
                None,
                self.role,
                "rejected",
                decode_reason,
                server_monotonic_ns=recv_monotonic_ns,
            )
            return ProcessResult(ack, None, False, "")

        validation = self.validator.validate(
            message,
            expected_target=self.role,
            source_id=peer[0],
            now_monotonic_ns=recv_monotonic_ns,
        )
        if not validation.accepted:
            ack = build_ack(
                message,
                self.role,
                "rejected",
                validation.reason,
                server_monotonic_ns=recv_monotonic_ns,
            )
            return ProcessResult(
                ack,
                message,
                False,
                validation.command,
            )

        if (
            validation.command == "detach_press"
            and not self._detach_request_is_armed(
                message,
                peer[0],
                recv_monotonic_ns,
            )
        ):
            ack = build_ack(
                message,
                self.role,
                "rejected",
                "detach_safety_precondition_missing",
                server_monotonic_ns=recv_monotonic_ns,
            )
            return ProcessResult(
                ack,
                message,
                False,
                validation.command,
            )

        try:
            applied, apply_reason = self._apply_validated_command(
                validation.command,
                message,
            )
        except Exception as exc:
            print(f"[{self.role}] command failed: {type(exc).__name__}: {exc}")
            ack = build_ack(
                message,
                self.role,
                "rejected",
                "command_apply_failed",
                server_monotonic_ns=recv_monotonic_ns,
            )
            return ProcessResult(ack, message, False, validation.command)

        status = "applied" if applied else "rejected"
        reason = apply_reason
        if applied and validation.command == "stop":
            self._remember_stop(message, peer[0], recv_monotonic_ns)
        elif applied and validation.command == "detach_press":
            self._consume_stop(message, peer[0])
        if applied and validation.reason != "ok":
            reason = validation.reason
        ack = build_ack(
            message,
            self.role,
            status,
            reason,
            server_monotonic_ns=recv_monotonic_ns,
        )
        return ProcessResult(ack, message, applied, validation.command)

    def _apply_validated_command(
        self,
        command: str,
        message: Dict[str, object],
    ) -> Tuple[bool, str]:
        normalized = normalize_command(command)
        store = self.drive_latch_store
        if store is None:
            return self._apply_command(command, message)

        if normalized in DRIVE_DISABLE_COMMANDS:
            applied, reason = self._apply_command(command, message)
            if not applied:
                return applied, reason
            try:
                store.save_drive_enabled(False)
            except Exception as exc:
                # Hardware has already stopped and latched off. Report failure
                # so the operator does not mistake an unrecorded detach for a
                # durable state transition.
                print(
                    f"[{self.role}] disabled drive but failed to persist "
                    f"latch state: {type(exc).__name__}: {exc}"
                )
                return False, "drive_state_persist_failed"
            return applied, reason

        if normalized == DRIVE_ENABLE_COMMAND:
            try:
                store.begin_drive_enable()
                store.save_drive_enabled(True)
            except Exception as exc:
                # save_drive_enabled() may fail after os.replace(), for
                # example while fsyncing the directory. Treat the on-disk
                # state as possibly enabled and force both hardware and disk
                # back to the safe state before rejecting the command.
                self._rollback_failed_drive_enable(message)
                print(
                    f"[{self.role}] drive enable rejected; failed to persist "
                    f"state: {type(exc).__name__}: {exc}"
                )
                return False, "drive_state_persist_failed"

            try:
                applied, reason = self._apply_command(command, message)
            except Exception:
                self._rollback_failed_drive_enable(message)
                raise
            if not applied:
                self._rollback_failed_drive_enable(message)
                return False, reason
            try:
                store.commit_drive_enable()
            except Exception as exc:
                self._rollback_failed_drive_enable(message)
                print(
                    f"[{self.role}] drive enable rolled back; failed to "
                    f"commit state: {type(exc).__name__}: {exc}"
                )
                return False, "drive_state_persist_failed"
            return applied, reason

        return self._apply_command(command, message)

    def _apply_command(
        self,
        command: str,
        message: Dict[str, object],
    ) -> Tuple[bool, str]:
        return apply_command(
            self.controller,
            self.role,
            command,
            message,
            allow_camera_profile_restart=self.allow_camera_profile_restart,
        )

    def _rollback_failed_drive_enable(
        self,
        message: Dict[str, object],
    ) -> None:
        """Return both hardware and persisted latch to the safe state."""
        try:
            apply_command(
                self.controller,
                self.role,
                "relay_hold",
                {
                    "seq": message.get("seq", "rollback"),
                    "source": "drive-enable-rollback",
                },
            )
        except Exception as exc:
            print(
                f"[{self.role}] CRITICAL: drive-enable rollback hold failed: "
                f"{type(exc).__name__}: {exc}"
            )
        try:
            if self.drive_latch_store is not None:
                # Recreate/refresh this first. Even if writing the disabled
                # state then fails before replace, startup sees the marker
                # and keeps propulsion latched off.
                self.drive_latch_store.begin_drive_enable()
        except Exception as exc:
            print(
                f"[{self.role}] drive-enable rollback marker failed: "
                f"{type(exc).__name__}: {exc}"
            )
        try:
            if self.drive_latch_store is not None:
                self.drive_latch_store.save_drive_enabled(False)
        except Exception as exc:
            print(
                f"[{self.role}] drive-enable rollback persistence failed; "
                f"future load will fail closed only if state is invalid: "
                f"{type(exc).__name__}: {exc}"
            )

    def _remember_stop(
        self,
        message: Dict[str, object],
        source_id: str,
        now_monotonic_ns: int,
    ) -> None:
        session_id = message.get("session_id")
        seq = message.get("seq")
        if (
            not isinstance(session_id, str)
            or isinstance(seq, bool)
            or not isinstance(seq, int)
        ):
            return
        self._recent_stops[(source_id, session_id)] = (
            seq,
            now_monotonic_ns,
        )
        if len(self._recent_stops) > 256:
            oldest = min(
                self._recent_stops,
                key=lambda key: self._recent_stops[key][1],
            )
            self._recent_stops.pop(oldest, None)

    def _consume_stop(
        self,
        message: Dict[str, object],
        source_id: str,
    ) -> None:
        session_id = message.get("session_id")
        if isinstance(session_id, str):
            self._recent_stops.pop((source_id, session_id), None)

    def _detach_request_is_armed(
        self,
        message: Dict[str, object],
        source_id: str,
        now_monotonic_ns: int,
    ) -> bool:
        if self.allow_unsafe_raw_detach:
            return True
        context = message.get("detach_context")
        if not isinstance(context, dict):
            return False
        released_node = context.get("released_node")
        if not isinstance(released_node, str):
            return False
        if context.get("stop_and_hold_acknowledged") is not True:
            return False
        if DETACH_ACTUATOR_BY_RELEASED_NODE.get(released_node) != self.role:
            return False

        actuator_stop_seq = context.get("actuator_stop_seq")
        session_id = message.get("session_id")
        if (
            isinstance(actuator_stop_seq, bool)
            or not isinstance(actuator_stop_seq, int)
            or not isinstance(session_id, str)
        ):
            return False
        recent = self._recent_stops.get((source_id, session_id))
        if recent is None:
            return False
        stop_seq, stop_at = recent
        age_ns = now_monotonic_ns - stop_at
        return (
            actuator_stop_seq == stop_seq
            and 0 <= age_ns <= DETACH_STOP_WINDOW_NS
        )

    def _decode_datagram(
        self,
        data: bytes,
        peer: Tuple[str, int],
        recv_monotonic_ns: int,
    ) -> Tuple[Optional[Dict[str, object]], str]:
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            return None, "invalid_utf8"

        try:
            decoded = json.loads(
                text,
                parse_constant=reject_nonstandard_json_constant,
            )
        except (json.JSONDecodeError, ValueError):
            command = text.strip()
            if not self.allow_legacy_plaintext:
                if command.startswith(("{", "[")):
                    return None, "invalid_json"
                return None, "legacy_plaintext_disabled"
            if not command:
                return None, "empty_legacy_command"

            next_seq = self._legacy_seq.get(peer, 0) + 1
            self._legacy_seq[peer] = next_seq
            message = build_command(
                session_id=f"legacy:{peer[0]}:{peer[1]}",
                seq=next_seq,
                target=self.role,
                command=command,
                source="legacy-plaintext",
                ttl_ms=self.validator.max_ttl_ms,
                sent_monotonic_ns=recv_monotonic_ns,
            )
            return message, "legacy_plaintext"

        if not isinstance(decoded, dict):
            return None, "message_must_be_object"
        return decoded, "ok"


def restore_persisted_drive_latch(
    controller: object,
    role: str,
    store: DriveLatchStore,
) -> bool:
    """Restore a saved disabled latch before the control socket is opened."""
    if store.load_drive_enabled():
        return True

    try:
        applied, reason = apply_command(
            controller,
            role,
            "relay_hold",
            {"seq": "startup", "source": "persistent-drive-latch"},
        )
    except Exception as exc:
        print(
            f"[{role}] failed to restore disabled drive latch: "
            f"{type(exc).__name__}: {exc}"
        )
        return False
    if not applied:
        print(
            f"[{role}] failed to restore disabled drive latch: {reason}"
        )
        return False

    try:
        # Heal a corrupt state file after the hardware has been made safe.
        store.save_drive_enabled(False)
    except Exception as exc:
        # Continue only in the physically disabled state. A subsequent
        # drive_enable will still be rejected until persistence succeeds.
        print(
            f"[{role}] warning: drive is disabled, but safe latch state "
            f"could not be rewritten: {type(exc).__name__}: {exc}"
        )
    return True


def run(args: argparse.Namespace) -> int:
    raw_allowed_sources = list(args.allow_source)
    env_allow_sources = os.environ.get("HANSEL_CONTROL_ALLOW_SOURCES", "")
    if env_allow_sources:
        raw_allowed_sources.append(env_allow_sources)
    try:
        allowed_sources = parse_source_networks(raw_allowed_sources)
    except ValueError as exc:
        print(f"[{args.role}] invalid --allow-source value: {exc}")
        return 2
    allowlist_required = (
        args.require_source_allowlist or not args.dry_run
    )
    if allowlist_required and not allowed_sources:
        print(
            f"[{args.role}] refusing to start: a non-empty control source "
            "allowlist is required for a production motor server"
        )
        return 2

    controller: Optional[object] = None
    try:
        controller = build_robot_controller(args.role, dry_run=args.dry_run)
        controller.start()
    except Exception as exc:
        if controller is not None:
            try:
                controller.stop()  # type: ignore[attr-defined]
            except Exception as cleanup_exc:
                print(
                    f"[{args.role}] controller cleanup after start failure "
                    f"also failed: {cleanup_exc}"
                )
        print(f"[{args.role}] failed to start motor controller: {exc}")
        print(
            f"[{args.role}] check: sudo, RPi.GPIO, encoder wiring, "
            "and duplicate GPIO pins"
        )
        return 2

    drive_latch_store: Optional[DriveLatchStore] = None
    persistence_enabled = (
        not args.dry_run
        and not args.unsafe_no_drive_state_persistence
    )
    if persistence_enabled:
        drive_latch_store = DriveLatchStore(
            args.drive_state_dir,
            args.role,
        )
        if not restore_persisted_drive_latch(
            controller,
            args.role,
            drive_latch_store,
        ):
            controller.stop()
            return 2

    core = ControlServerCore(
        controller=controller,
        role=args.role,
        allowed_sources=allowed_sources,
        allow_legacy_plaintext=args.allow_legacy_plaintext,
        allow_unsafe_raw_detach=args.allow_unsafe_raw_detach,
        allow_camera_profile_restart=args.allow_camera_profile_restart,
        max_ttl_ms=args.max_ttl_ms,
        drive_latch_store=drive_latch_store,
    )

    sock: Optional[socket.socket] = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.host, args.port))
        sock.settimeout(0.1)
    except OSError as exc:
        print(
            f"[{args.role}] failed to open control socket "
            f"{args.host}:{args.port}: {exc}"
        )
        if sock is not None:
            sock.close()
        try:
            controller.stop()
        except Exception as cleanup_exc:
            print(
                f"[{args.role}] controller cleanup after socket failure "
                f"also failed: {cleanup_exc}"
            )
        return 2
    assert sock is not None

    last_motion_seen = 0.0
    stopped = True

    print(f"[{args.role}] listening on {args.host}:{args.port}")
    print(f"[{args.role}] timeout: {args.timeout}s")
    print(f"[{args.role}] protocol_version: {PROTOCOL_VERSION}")
    print(f"[{args.role}] dry_run: {args.dry_run}")
    print(
        f"[{args.role}] drive_state_persistence: "
        f"{'enabled' if persistence_enabled else 'disabled'}"
    )
    if persistence_enabled:
        print(
            f"[{args.role}] drive_state_file: "
            f"{drive_latch_store.path}"
        )
    elif args.unsafe_no_drive_state_persistence:
        print(
            f"[{args.role}] WARNING: drive latch persistence was disabled "
            "by an unsafe command-line option"
        )
    print(f"[{args.role}] legacy_plaintext: {args.allow_legacy_plaintext}")
    print(f"[{args.role}] unsafe_raw_detach: {args.allow_unsafe_raw_detach}")
    print(
        f"[{args.role}] unmanaged_camera_profile_restart: "
        f"{args.allow_camera_profile_restart}"
    )
    if allowed_sources:
        print(
            f"[{args.role}] allowed control sources: "
            + ", ".join(str(network) for network in allowed_sources)
        )
    else:
        print(
            f"[{args.role}] WARNING: no source allowlist configured; "
            "all source IPs are accepted"
        )

    try:
        while True:
            now = time.monotonic()
            if (
                last_motion_seen
                and not stopped
                and now - last_motion_seen > args.timeout
            ):
                apply_command(
                    controller,
                    args.role,
                    "stop",
                    {"seq": "timeout", "source": "watchdog"},
                )
                stopped = True

            try:
                data, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue

            result = core.process_datagram(data, peer)
            try:
                sock.sendto(
                    json.dumps(result.ack, separators=(",", ":")).encode("utf-8"),
                    peer,
                )
            except OSError as exc:
                print(f"[{args.role}] ACK send failed to {peer}: {exc}")

            print(
                f"[{args.role}] packet from={peer} command={result.command or '-'} "
                f"status={result.ack['status']} reason={result.ack['reason']}"
            )
            if not result.applied:
                continue
            if command_stops_all_motion(result.command):
                stopped = True
            elif command_starts_motion(args.role, result.command):
                last_motion_seen = time.monotonic()
                stopped = False

    except KeyboardInterrupt:
        print()
        print(f"[{args.role}] KeyboardInterrupt detected.")
    finally:
        controller.stop()
        sock.close()

    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive acknowledged UDP control over BATMAN mesh."
    )
    parser.add_argument(
        "--role",
        required=True,
        choices=("head", "node1", "node2", "node3"),
        help="local node role",
    )
    parser.add_argument("--host", default="0.0.0.0", help="bind address")
    parser.add_argument("--port", type=int, default=7000, help="UDP control port")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="seconds before automatic stop",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="log commands without touching GPIO",
    )
    parser.add_argument(
        "--allow-legacy-plaintext",
        action="store_true",
        help="UNSAFE compatibility mode for unversioned plaintext commands",
    )
    parser.add_argument(
        "--allow-unsafe-raw-detach",
        action="store_true",
        help="UNSAFE bench mode: allow detach_press without stop/hold context",
    )
    parser.add_argument(
        "--allow-camera-profile-restart",
        action="store_true",
        help=(
            "UNSAFE legacy mode: allow control packets to kill/restart an "
            "unmanaged camera process; keep disabled with systemd camera"
        ),
    )
    parser.add_argument(
        "--drive-state-dir",
        default=DEFAULT_DRIVE_STATE_DIR,
        help=(
            "root-owned directory for persistent physical-Pi drive latch "
            "state "
            f"(default: {DEFAULT_DRIVE_STATE_DIR})"
        ),
    )
    parser.add_argument(
        "--unsafe-no-drive-state-persistence",
        action="store_true",
        help=(
            "UNSAFE: allow a production controller to forget relay_hold/"
            "drive_disable state after restart"
        ),
    )
    parser.add_argument(
        "--allow-source",
        action="append",
        default=[],
        metavar="IP_OR_CIDR",
        help=(
            "allow a control source IP/CIDR; repeat or comma-separate values. "
            "When omitted, all source IPs are accepted"
        ),
    )
    parser.add_argument(
        "--require-source-allowlist",
        action="store_true",
        help=(
            "require an allowlist even in dry-run; production mode always "
            "requires one"
        ),
    )
    parser.add_argument(
        "--max-ttl-ms",
        type=int,
        default=MAX_TTL_MS,
        help="maximum client-specified command TTL",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    if not 1 <= args.max_ttl_ms <= MAX_TTL_MS:
        parser.error(
            f"--max-ttl-ms must be between 1 and {MAX_TTL_MS}"
        )
    try:
        parse_source_networks(args.allow_source)
    except ValueError as exc:
        parser.error(f"invalid --allow-source value: {exc}")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
