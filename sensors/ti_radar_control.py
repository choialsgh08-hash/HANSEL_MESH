"""Reusable TI mmWave profile application across its UART baud switch."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Iterable, List, Mapping, Sequence, Tuple


TI_MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


@dataclass(frozen=True)
class ProfileCommands:
    before_baud: Tuple[str, ...]
    baud_command: str
    target_baud: int
    after_baud: Tuple[str, ...]


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
