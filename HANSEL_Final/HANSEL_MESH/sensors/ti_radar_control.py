"""Reusable TI mmWave profile application across its UART baud switch."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import os
import re
import subprocess
import time
from typing import Callable, Iterable, List, Mapping, Sequence, Tuple


TI_MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
CLI_PROMPT = b"mmwDemo:/>"
# sensorStop can place its text immediately after the last binary frame.
_DONE_LINE = re.compile(rb"(?:^|[^\x20-\x7e])Done\r?\n")
_ERROR_LINE = re.compile(
    rb"(?:^|[\r\n])(?:Error\b[ -~]*|[ -~]*not recognized[ -~]*)[\r\n]",
    re.IGNORECASE,
)
_VERSION_LINE = re.compile(
    rb"(?:^|[\r\n]|mmwDemo:/>)\s*(?:version\r?\n|Platform\s*:|[ -~]*SDK Version\s*:)",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class ProfileCommands:
    before_baud: Tuple[str, ...]
    baud_command: str
    target_baud: int
    after_baud: Tuple[str, ...]


@dataclass(frozen=True)
class RadarPortIdentity:
    device: str
    vid: int
    pid: int
    serial_number: str
    description: str
    location: str


def _port_metadata_text(value: object) -> str:
    return "" if value is None else str(value).strip()


def is_xds110_application_port(
    device: str,
    description: str,
    location: str,
) -> bool:
    """Recognize the XDS110 Application UART on Windows and Linux."""
    description_folded = description.casefold()
    location_folded = location.casefold()
    if "auxiliary" in description_folded or location_folded.endswith(".3"):
        return False
    if "application/user uart" in description_folded:
        return True
    return (
        device.startswith("/dev/ttyACM")
        and "xds110" in description_folded
        and location_folded.endswith(".0")
    )


def select_application_port(
    ports: Iterable[object],
    explicit_port: str | None = None,
    xds_serial: str | None = None,
) -> RadarPortIdentity:
    """Select exactly one XDS110 Application/User UART from a port inventory."""
    candidates = []
    for port in ports:
        device = _port_metadata_text(getattr(port, "device", ""))
        if explicit_port is not None and os.path.realpath(device) != os.path.realpath(explicit_port):
            continue
        vid = getattr(port, "vid", None)
        pid = getattr(port, "pid", None)
        description = _port_metadata_text(getattr(port, "description", ""))
        serial_number = _port_metadata_text(getattr(port, "serial_number", ""))
        location = _port_metadata_text(getattr(port, "location", ""))
        if (vid, pid) != (0x0451, 0xBEF3):
            continue
        if not is_xds110_application_port(device, description, location):
            continue
        if not serial_number:
            continue
        if xds_serial is not None and serial_number != xds_serial:
            continue
        candidates.append(
            RadarPortIdentity(
                device=device,
                vid=vid,
                pid=pid,
                serial_number=serial_number,
                description=description,
                location=location,
            )
        )

    if len(candidates) != 1:
        filters = []
        if explicit_port is not None:
            filters.append(f"explicit port {explicit_port!r}")
        if xds_serial is not None:
            filters.append(f"serial {xds_serial!r}")
        filter_text = ", ".join(filters) or "available ports"
        raise RuntimeError(
            "expected exactly one XDS110 Application/User UART for "
            f"{filter_text}; found {len(candidates)}"
        )
    return candidates[0]


def _uniflash_version(path: Path) -> tuple[int, ...]:
    version = path.name.removeprefix("uniflash_")
    try:
        return tuple(int(part) for part in version.split("."))
    except ValueError:
        return ()


def find_xds110_reset(
    explicit: Path | None,
    search_roots: Iterable[Path],
) -> Path:
    """Locate xds110reset, preferring explicit and newest UniFlash installs."""
    if explicit is not None:
        if explicit.is_file():
            return explicit
        raise RuntimeError(f"explicit xds110reset executable was not found: {explicit}")

    for executable_name in ("xds110reset.exe", "xds110reset"):
        found = shutil.which(executable_name)
        if found:
            return Path(found)

    candidates = []
    for root in search_roots:
        for install in root.glob("uniflash_*"):
            for relative_path in (
                Path("deskdb/content/TICloudAgent/win/ccs_base/common/uscif/xds110/xds110reset.exe"),
                Path("simplelink/imagecreator/bin/xds110reset.exe"),
            ):
                executable = install / relative_path
                if executable.is_file():
                    candidates.append((install, executable))
    if candidates:
        return max(candidates, key=lambda candidate: _uniflash_version(candidate[0]))[1]
    raise RuntimeError(
        "xds110reset executable was not found; install TI UniFlash or provide its path"
    )


def reset_xds110_target(
    executable: Path,
    serial_number: str,
    runner: Callable[..., object],
) -> None:
    """Toggle reset only for the XDS110 identified by *serial_number*."""
    if not serial_number.strip():
        raise ValueError("XDS110 serial number must not be empty")
    command = [
        str(executable),
        "-a",
        "toggle",
        "-d",
        "100",
        "-s",
        serial_number,
    ]
    try:
        runner(command, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr or ""
        raise RuntimeError(f"XDS110 reset failed for serial {serial_number!r}: {stderr}") from exc


def load_commands(path: Path) -> Tuple[str, ...]:
    commands: List[str] = []
    for line_number, raw in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        1,
    ):
        command = raw.strip()
        if not command or command.startswith("%"):
            continue
        if "%" in command:
            raise ValueError(
                f"{path}:{line_number}: inline '%' comments are unsupported"
            )
        try:
            command.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError(
                f"{path}:{line_number}: CLI commands must be ASCII"
            ) from exc
        commands.append(command)
    if not commands:
        raise ValueError(f"{path}: profile contains no commands")
    return tuple(commands)


def partition_at_baud(commands: Sequence[str]) -> ProfileCommands:
    matches = [
        (index, command)
        for index, command in enumerate(commands)
        if command.split(maxsplit=1)[0] == "baudRate"
    ]
    if len(matches) != 1:
        raise ValueError("profile must contain exactly one baudRate command")
    index, command = matches[0]
    fields = command.split()
    if len(fields) != 2:
        raise ValueError("baudRate command must contain one value")
    try:
        target_baud = int(fields[1], 10)
    except ValueError as exc:
        raise ValueError("baudRate value must be an integer") from exc
    if target_baud < 1 or target_baud > 10_000_000:
        raise ValueError("baudRate value is out of range")
    if not commands[index + 1 :]:
        raise ValueError("profile has no command after baudRate")
    return ProfileCommands(
        before_baud=tuple(commands[:index]),
        baud_command=command,
        target_baud=target_baud,
        after_baud=tuple(commands[index + 1 :]),
    )


def _read_until(
    serial_port: object,
    needles: Iterable[bytes],
    timeout_s: float,
) -> bytes:
    expected = tuple(needles)
    data = bytearray()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        chunk = serial_port.read(4096)
        if chunk:
            data.extend(chunk)
            if any(needle in data for needle in expected):
                break
            if len(data) > 65536:
                del data[:-65536]
    return bytes(data)


def _command_failed(response: bytes) -> bool:
    return _ERROR_LINE.search(response) is not None


def _write_cli_line(serial_port: object, command: str) -> bytes:
    """Keep draining the streaming UART while delivering a stop command."""
    stopping = command.split()[:1] == ["sensorStop"]
    received = bytearray()
    for value in (command + "\n").encode("ascii"):
        serial_port.write(bytes((value,)))
        serial_port.flush()
        if stopping:
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline:
                received.extend(serial_port.read(4096))
                if len(received) > 131072:
                    del received[:-65536]
        else:
            time.sleep(0.002)
    return bytes(received)


def _read_cli_reply(
    serial_port: object,
    command: str,
    timeout_s: float,
    initial: bytes = b"",
) -> bytes:
    response = bytearray(initial)
    deadline = time.monotonic() + timeout_s
    while True:
        # A prompt preceding Done belongs to a previous command. Wait for the
        # prompt AFTER this completion, even when both arrive in separate reads.
        version = _VERSION_LINE.search(response) if command == "version" else None
        start = max(0, version.end() - 1) if version else 0
        done = _DONE_LINE.search(response, start)
        prompt = response.find(CLI_PROMPT, done.end()) if done else -1
        end = prompt + len(CLI_PROMPT) if prompt >= 0 else len(response)
        if _command_failed(response[:end]):
            raise RuntimeError(
                f"command failed: {command!r}: "
                f"{bytes(response[:end][-500:])!r}"
            )
        if prompt >= 0 and (command != "version" or version is not None):
            return bytes(response)
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"command timed out without complete Done/prompt: {command!r}; "
                f"baud={getattr(serial_port, 'baudrate', '?')}; "
                f"response_tail={bytes(response[-240:])!r}"
            )
        response.extend(serial_port.read(4096))
        if len(response) > 131072:
            del response[:-65536]


def _send_command(serial_port: object, command: str, timeout_s: float) -> bytes:
    response = _write_cli_line(serial_port, command)
    return _read_cli_reply(serial_port, command, timeout_s, response)


def _stop_stream(cli: object, timeout_s: float) -> None:
    try:
        _send_command(cli, "sensorStop 0", timeout_s)
    except TimeoutError as exc:
        # Some firmware loses the stop reply while the last frame is draining.
        # A version reply alone is insufficient: it can coexist with frames.
        print(f"[radar] stop acknowledgement incomplete: {exc}", flush=True)
        reply = _send_command(cli, "version", min(timeout_s, 1.5))
        reply += _read_until(cli, (TI_MAGIC_WORD,), 0.4)
        if TI_MAGIC_WORD in reply:
            raise TimeoutError("radar still streaming after sensorStop") from exc


def establish_cli(cli: object, initial_baud: int, target_baud: int,
                  timeout_s: float) -> int:
    """Find the current UART mode before applying configuration.

    A host restart does not reset the radar: it can still be streaming at
    target_baud. Stop that stream at its current baud, never treat it as an
    already validated profile. The complete requested profile is reapplied.
    """
    observations = []
    for baud in dict.fromkeys((target_baud, initial_baud)):
        cli.baudrate = baud
        cli.reset_input_buffer()
        response = _read_until(cli, (TI_MAGIC_WORD, CLI_PROMPT), min(timeout_s, 0.6))
        if response and TI_MAGIC_WORD not in response and CLI_PROMPT not in response:
            # Do not inject commands into an active stream at a mismatched baud.
            observations.append(f"{baud}: unrecognized traffic, bytes={len(response)}")
            continue
        # Terminate a partial command left by an interrupted previous process.
        # Keep the reply: a stream can start during this probe.
        cli.write(b"\n")
        cli.flush()
        response += _read_until(cli, (TI_MAGIC_WORD, CLI_PROMPT), min(timeout_s, 0.6))
        streaming = TI_MAGIC_WORD in response
        try:
            if streaming:
                print(f"[radar] stopping existing stream: baud={baud}", flush=True)
                _stop_stream(cli, timeout_s)
            reply = _send_command(cli, "version", min(timeout_s, 1.5))
            if TI_MAGIC_WORD in reply:
                # A prompt can arrive before a frame during the passive probe.
                streaming = True
                _stop_stream(cli, timeout_s)
                _send_command(cli, "version", min(timeout_s, 1.5))
            print(f"[radar] CLI ready: baud={baud} stopped_stream={streaming}", flush=True)
            return baud
        except (TimeoutError, RuntimeError) as exc:
            observations.append(f"{baud}: {exc}")
            if streaming:
                # Magic identified the baud. Retrying at 115200 cannot stop this
                # stream; leave recovery at this baud to the supervisor retry.
                break
    raise TimeoutError("Radar CLI unavailable; check power/mode or reset target. " + "; ".join(observations))


def stop_sensor(port: str, data_baud: int, timeout_s: float = 1.0) -> None:
    """Quiesce the owned radar on exit, without resetting USB or other devices."""
    import serial

    with serial.Serial(port, data_baud, timeout=0.05, write_timeout=1,
                       **({"exclusive": True} if os.name == "posix" else {})) as cli:
        # Only use the baud established by this run. A freshly power-cycled
        # board does not need shutdown commands at its cold-start baud.
        establish_cli(cli, data_baud, data_baud, timeout_s)


def apply_profile(
    port: str,
    profile: ProfileCommands,
    initial_baud: int,
    command_timeout_s: float,
    reopen_delay_s: float,
) -> dict[str, object]:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required; install requirements-sensors.txt"
        ) from exc

    completed = 0
    startup = b""
    started = False
    begin = time.monotonic()
    with serial.Serial(port, profile.target_baud, timeout=0.05, write_timeout=1,
                       **({"exclusive": True} if os.name == "posix" else {})) as cli:
        current_baud = establish_cli(cli, initial_baud, profile.target_baud, command_timeout_s)
        for command in profile.before_baud:
            print(f"[radar] configure: {command}", flush=True)
            _send_command(cli, command, command_timeout_s)
            completed += 1

        if current_baud != profile.target_baud:
            print(f"[radar] switching UART: {current_baud} -> {profile.target_baud}", flush=True)
            _write_cli_line(cli, profile.baud_command)
            # Keep exclusive ownership across the switch. The acknowledgement
            # can be lost while the host changes baud, so verify with version.
            cli.baudrate = profile.target_baud
            time.sleep(reopen_delay_s)
            startup = _read_until(cli, (CLI_PROMPT,), min(command_timeout_s, 0.6))
            startup += _send_command(cli, "version", command_timeout_s)
        # If already at target baud, establish_cli has verified it. There is no
        # unsolicited startup prompt to wait for and no reason to reopen UART.
        completed += 1

        for command in profile.after_baud:
            print(f"[radar] configure: {command}", flush=True)
            response = _send_command(cli, command, command_timeout_s)
            completed += 1
            if command.split(maxsplit=1)[0] == "sensorStart":
                if TI_MAGIC_WORD not in response:
                    response += _read_until(
                        cli, (TI_MAGIC_WORD,), min(command_timeout_s, 0.75),
                    )
                started = TI_MAGIC_WORD in response

    elapsed = time.monotonic() - begin
    print(f"[radar] profile applied: commands={completed} elapsed_s={elapsed:.3f}", flush=True)
    return {
        "port": port,
        "initial_baud": initial_baud,
        "detected_baud": current_baud,
        "baud_already_configured": current_baud == profile.target_baud,
        "target_baud": profile.target_baud,
        "commands_completed": completed,
        "new_baud_prompt_observed": True,
        "new_baud_verified_by_version": True,
        "first_magic_observed": started,
        "new_baud_startup_bytes": len(startup),
        "configuration_elapsed_s": elapsed,
    }


def validate_profile_result(
    result: Mapping[str, object],
    expected_commands: int,
    require_first_magic: bool = True,
) -> None:
    if result.get("commands_completed") != expected_commands:
        raise RuntimeError("not every radar profile command completed")
    if result.get("new_baud_prompt_observed") is not True:
        raise RuntimeError("new radar baud prompt was not observed")
    if require_first_magic and result.get("first_magic_observed") is not True:
        raise RuntimeError("first radar frame magic was not observed")
