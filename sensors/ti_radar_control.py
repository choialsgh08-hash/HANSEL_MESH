"""Reusable TI mmWave profile application across its UART baud switch."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import time
from typing import Callable, Iterable, List, Mapping, Sequence, Tuple


TI_MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


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


def select_application_port(
    ports: Iterable[object],
    explicit_port: str | None = None,
    xds_serial: str | None = None,
) -> RadarPortIdentity:
    """Select exactly one XDS110 Application/User UART from a port inventory."""
    candidates = []
    for port in ports:
        device = str(getattr(port, "device", ""))
        if explicit_port is not None and device != explicit_port:
            continue
        vid = getattr(port, "vid", None)
        pid = getattr(port, "pid", None)
        description = str(getattr(port, "description", ""))
        serial_number = str(getattr(port, "serial_number", ""))
        if (vid, pid) != (0x0451, 0xBEF3):
            continue
        if "Application/User UART" not in description:
            continue
        if "Auxiliary" in description:
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
                location=str(getattr(port, "location", "")),
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
    return bytes(data)


def _command_failed(response: bytes) -> bool:
    lower = response.lower()
    return b"error" in lower or b"not recognized" in lower


def _send_command(
    serial_port: object,
    command: str,
    timeout_s: float,
) -> bytes:
    serial_port.write((command + "\n").encode("ascii"))
    serial_port.flush()
    response = _read_until(
        serial_port,
        (b"Done", b"Error", b"not recognized"),
        timeout_s,
    )
    if _command_failed(response):
        raise RuntimeError(
            f"command failed: {command!r}: "
            f"{response.decode('ascii', errors='replace')!r}"
        )
    if b"Done" not in response:
        raise TimeoutError(f"command timed out without Done: {command!r}")
    return response


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
    with serial.Serial(port, initial_baud, timeout=0.05) as cli:
        cli.reset_input_buffer()
        for command in profile.before_baud:
            _send_command(cli, command, command_timeout_s)
            completed += 1

        cli.write((profile.baud_command + "\n").encode("ascii"))
        cli.flush()
        _read_until(cli, (b"\n",), min(command_timeout_s, 0.25))

    time.sleep(reopen_delay_s)
    startup = b""
    started = False
    verified_by_version = False
    with serial.Serial(port, profile.target_baud, timeout=0.05) as cli:
        startup = _read_until(
            cli,
            (b"mmwDemo:/>", b"Error", b"not recognized"),
            command_timeout_s,
        )
        if _command_failed(startup):
            raise RuntimeError(
                "baud switch failed: "
                + startup.decode("ascii", errors="replace")
            )
        if b"Done" not in startup or b"mmwDemo:/>" not in startup:
            cli.reset_input_buffer()
            cli.write(b"version\n")
            cli.flush()
            version_response = _read_until(
                cli,
                (b"Done", b"Error", b"not recognized"),
                command_timeout_s,
            )
            prompt_tail = _read_until(
                cli,
                (b"mmwDemo:/>", b"Error", b"not recognized"),
                min(command_timeout_s, 0.75),
            )
            startup += version_response + prompt_tail
            verified_by_version = True
            if _command_failed(startup):
                raise RuntimeError(
                    "new-baud version probe failed: "
                    + startup.decode("ascii", errors="replace")
                )
            probe_response = version_response + prompt_tail
            if b"Done" not in probe_response or b"mmwDemo:/>" not in probe_response:
                raise TimeoutError(
                    "new-baud prompt and version probe were not observed; "
                    "reset and retry"
                )
        completed += 1

        for command in profile.after_baud:
            response = _send_command(cli, command, command_timeout_s)
            completed += 1
            if command.split(maxsplit=1)[0] == "sensorStart":
                tail = _read_until(
                    cli,
                    (TI_MAGIC_WORD, b"Error"),
                    min(command_timeout_s, 0.75),
                )
                response += tail
                if _command_failed(response):
                    raise RuntimeError(
                        "sensorStart reported an error: "
                        + response.decode("ascii", errors="replace")
                    )
                started = TI_MAGIC_WORD in response

    return {
        "port": port,
        "initial_baud": initial_baud,
        "target_baud": profile.target_baud,
        "commands_completed": completed,
        "new_baud_prompt_observed": True,
        "new_baud_verified_by_version": verified_by_version,
        "first_magic_observed": started,
        "new_baud_startup_bytes": len(startup),
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
