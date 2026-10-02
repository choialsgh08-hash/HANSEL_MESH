import tempfile
import os
import select
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import sensors.ti_radar_control as radar_control

from sensors.ti_radar_control import (
    TI_MAGIC_WORD, apply_profile, establish_cli, partition_at_baud,
    select_application_port, _send_command,
    stop_sensor,
)


class RadarUart:
    def __init__(self, actual_baud=115200, streaming=False, **kwargs):
        self.actual_baud = actual_baud
        self.baudrate = 115200
        self.streaming = streaming
        self.buffer = b""
        self.commands = []
        self.pending = b""
        self.writes = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def reset_input_buffer(self):
        self.buffer = b""

    def flush(self):
        pass

    def read(self, size):
        if self.buffer:
            result, self.buffer = self.buffer[:size], self.buffer[size:]
            return result
        if self.streaming and self.baudrate == self.actual_baud:
            return TI_MAGIC_WORD + bytes(20)
        return b""

    def write(self, data):
        self.writes.append((self.baudrate, data))
        if self.baudrate != self.actual_baud:
            return len(data)
        self.pending += data
        while b"\n" in self.pending:
            line, self.pending = self.pending.split(b"\n", 1)
            command = line.decode()
            if not command:
                if not self.streaming:
                    self.buffer += b"\r\nmmwDemo:/>"
                continue
            self.commands.append(command)
            if command == "sensorStop 0":
                self.streaming = False
            elif command.startswith("baudRate "):
                self.actual_baud = int(command.split()[1])
            elif command.startswith("sensorStart "):
                self.streaming = True
            elif self.streaming:
                continue
            if command == "version":
                self.buffer += b"version\r\nPlatform: xWRL6432\r\nSDK Version: 05.05.04.02\r\n"
            self.buffer += b"Done\r\nmmwDemo:/>"
        return len(data)


@pytest.mark.parametrize("baud,streaming", [(115200, False), (1250000, False), (1250000, True)])
def test_cli_cold_and_warm_restart(baud, streaming):
    uart = RadarUart(baud, streaming)
    assert establish_cli(uart, 115200, 1250000, .001) == baud
    assert ("sensorStop 0" in uart.commands) == streaming


def test_warm_restart_reapplies_profile_without_changing_baud():
    uart = RadarUart(1250000, True)
    opened = []
    def open_uart(port, baud, **kwargs):
        opened.append(baud)
        uart.baudrate = baud
        assert kwargs["write_timeout"] == 1
        assert kwargs["exclusive"] is True
        return uart
    profile = partition_at_baud(["channelCfg 7 3 0", "baudRate 1250000", "sensorStart 0 0 0 0"])
    with patch("serial.Serial", open_uart):
        result = apply_profile("/dev/fake", profile, 115200, .001, 0)
    assert uart.commands.index("sensorStop 0") < uart.commands.index("channelCfg 7 3 0")
    assert "baudRate 1250000" not in uart.commands
    assert result["commands_completed"] == 3
    assert result["first_magic_observed"] is True
    assert opened == [1250000]
    assert {baud for baud, _ in uart.writes} == {1250000}


class ChunkedReplyUart(RadarUart):
    def __init__(self, chunks):
        super().__init__()
        self.chunks = list(chunks)

    def read(self, size):
        return self.chunks.pop(0) if self.chunks else b""


def test_command_waits_for_prompt_after_done_across_reads():
    uart = ChunkedReplyUart([b"channelCfg 7 3 0\r\nDo", b"ne\r\n", b"mmwDemo:/>"])
    result = _send_command(uart, "channelCfg 7 3 0", .01)
    assert result.endswith(b"mmwDemo:/>")
    assert uart.chunks == []


@pytest.mark.parametrize("chunks", [
    [b"mmwDemo:/>", b"version\r\nDone\r\n"],
    [b"notDone\r\nmmwDemo:/>"],
    [b"Done"],
])
def test_incomplete_or_stale_prompt_does_not_complete_command(chunks):
    with pytest.raises(TimeoutError):
        _send_command(ChunkedReplyUart(chunks), "version", .01)


def test_binary_error_text_after_start_reply_is_not_a_cli_error():
    response = b"sensorStart 0 0 0 0\r\nDone\r\nmmwDemo:/>" + TI_MAGIC_WORD + b"Error"
    assert _send_command(ChunkedReplyUart([response]), "sensorStart 0 0 0 0", .01) == response


def test_cli_error_is_not_ignored():
    with pytest.raises(RuntimeError, match="command failed"):
        _send_command(ChunkedReplyUart([b"Error: invalid argument\r\nmmwDemo:/>"]), "channelCfg", .01)


def test_version_cannot_use_previous_commands_done_and_prompt():
    uart = ChunkedReplyUart([
        b"Done\r\nmmwDemo:/>",
        b"version\r\nPlatform: xWRL6432\r\n",
        b"Done\r\n", b"mmwDemo:/>",
    ])
    result = _send_command(uart, "version", .01)
    assert b"Platform: xWRL6432" in result
    assert uart.chunks == []


def test_version_requires_own_reply_not_just_prompt():
    with pytest.raises(TimeoutError):
        _send_command(ChunkedReplyUart([b"Done\r\nmmwDemo:/>"]), "version", .01)


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        self.now += .001
        return self.now

    def sleep(self, seconds):
        self.now += seconds


@pytest.mark.parametrize("lose_baud_ack", [False, True])
def test_repeated_cold_then_warm_restarts_keep_one_uart_owner(monkeypatch, lose_baud_ack):
    class Uart(RadarUart):
        def write(self, data):
            previous_baud = self.actual_baud
            result = super().write(data)
            if lose_baud_ack and previous_baud != self.actual_baud:
                self.buffer = b""
            return result

    clock = Clock()
    monkeypatch.setattr(radar_control, "time", clock)
    uart = Uart()
    opened = []

    def open_uart(port, baud, **kwargs):
        opened.append(baud)
        uart.baudrate = baud
        return uart

    monkeypatch.setattr("serial.Serial", open_uart)
    profile = partition_at_baud(["channelCfg 7 3 0", "baudRate 1250000", "sensorStart 0 0 0 0"])
    for index in range(5):
        uart.writes.clear()
        result = apply_profile("/dev/fake", profile, 115200, 8, .5)
        assert result["commands_completed"] == 3
        assert result["first_magic_observed"] is True
        assert result["configuration_elapsed_s"] < 5
        if index:
            assert {baud for baud, _ in uart.writes} == {1250000}
    assert opened == [1250000] * 5
    assert uart.commands.count("baudRate 1250000") == 1
    assert uart.commands.count("sensorStop 0") == 4
    assert uart.commands.count("channelCfg 7 3 0") == 5


def test_wrong_baud_noise_is_not_treated_as_cli(monkeypatch):
    class NoisyWrongBaud(RadarUart):
        def read(self, size):
            if self.baudrate != self.actual_baud:
                return b"\xff\xdd\x00\xfe"
            return super().read(size)

    monkeypatch.setattr(radar_control, "time", Clock())
    uart = NoisyWrongBaud(115200, True)
    assert establish_cli(uart, 115200, 1250000, 2) == 115200
    assert {baud for baud, _ in uart.writes} == {115200}


def test_version_with_fragmented_prompt_and_leading_old_prompt():
    uart = ChunkedReplyUart([
        b"mmwDemo:/>", b"ver", b"sion\r\n", b"Do", b"ne\r\n",
        b"mmw", b"Demo:/", b">",
    ])
    assert _send_command(uart, "version", .01).endswith(b"mmwDemo:/>")
    assert uart.chunks == []


def test_usb_read_failure_is_preserved():
    class Disconnected(RadarUart):
        def read(self, size):
            raise OSError("USB disconnected")

    with pytest.raises(OSError, match="USB disconnected"):
        establish_cli(Disconnected(), 115200, 1250000, .01)


@pytest.mark.skipif(os.name != "posix", reason="requires a POSIX pseudo-terminal")
def test_real_serial_waits_between_split_responses():
    import serial

    master, slave = os.openpty()
    stopped = threading.Event()
    commands = []
    early_commands = []

    def radar():
        pending = b""
        prompt_at = None
        while not stopped.is_set():
            if prompt_at is not None and time.monotonic() >= prompt_at:
                os.write(master, b"mmwDemo:/>")
                prompt_at = None
            if not select.select([master], [], [], .005)[0]:
                continue
            pending += os.read(master, 4096)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                commands.append(line)
                if prompt_at is not None:
                    early_commands.append(line)
                os.write(master, b"\r\nDone\r\n")
                prompt_at = time.monotonic() + .12

    worker = threading.Thread(target=radar)
    try:
        with serial.Serial(os.ttyname(slave), 1250000, timeout=.02, exclusive=True) as uart:
            worker.start()
            _send_command(uart, "channelCfg 7 3 0", 1)
            _send_command(uart, "lowPowerCfg 0", 1)
        assert commands == [b"channelCfg 7 3 0", b"lowPowerCfg 0"]
        assert early_commands == []
    finally:
        stopped.set()
        if worker.ident is not None:
            worker.join(timeout=2)
        os.close(master)
        os.close(slave)


def test_cold_restart_switches_baud_and_starts_frames():
    uart = RadarUart()
    def open_uart(port, baud, **kwargs):
        uart.baudrate = baud
        return uart
    profile = partition_at_baud(["channelCfg 7 3 0", "baudRate 1250000", "sensorStart 0 0 0 0"])
    with patch("serial.Serial", open_uart):
        result = apply_profile("/dev/fake", profile, 115200, .001, 0)
    assert "baudRate 1250000" in uart.commands
    assert result["first_magic_observed"] is True


def test_stop_consumes_reply_received_during_transmit():
    from sensors.ti_radar_control import _send_command
    uart = RadarUart(115200, True)
    response = _send_command(uart, "sensorStop 0", .001)
    assert b"Done" in response
    assert uart.streaming is False
    assert uart.buffer == b""


def test_no_data_is_not_claimed_ready():
    with pytest.raises(TimeoutError, match="CLI unavailable"):
        establish_cli(RadarUart(999), 115200, 1250000, .001)


@pytest.mark.parametrize("streaming", [False, True])
def test_shutdown_leaves_sensor_idle_without_reconfiguring(monkeypatch, streaming):
    monkeypatch.setattr(radar_control, "time", Clock())
    uart = RadarUart(1250000, streaming)

    def open_uart(port, baud, **kwargs):
        assert port == "/dev/fake"
        assert kwargs["exclusive"] is True
        uart.baudrate = baud
        return uart

    monkeypatch.setattr("serial.Serial", open_uart)
    stop_sensor("/dev/fake", 1250000)
    assert not uart.streaming
    assert uart.actual_baud == 1250000
    assert set(uart.commands) <= {"sensorStop 0", "version"}
    assert uart.commands.count("sensorStop 0") == int(streaming)


def test_repeated_session_start_and_shutdown_preserve_working_baud(monkeypatch):
    monkeypatch.setattr(radar_control, "time", Clock())
    uart = RadarUart()

    def open_uart(port, baud, **kwargs):
        uart.baudrate = baud
        assert kwargs["exclusive"] is True
        return uart

    monkeypatch.setattr("serial.Serial", open_uart)
    profile = partition_at_baud(["channelCfg 7 3 0", "baudRate 1250000", "sensorStart 0 0 0 0"])
    for _ in range(10):
        result = apply_profile("/dev/fake", profile, 115200, 3, .25)
        assert result["first_magic_observed"] is True
        stop_sensor("/dev/fake", 1250000)
        assert not uart.streaming
    assert uart.commands.count("baudRate 1250000") == 1
    assert uart.commands.count("channelCfg 7 3 0") == 10
    assert uart.commands.count("sensorStop 0") == 10


def test_stop_without_ack_requires_quiet_stream_not_just_version(monkeypatch):
    class LostStopAck(RadarUart):
        def __init__(self, stop_works):
            super().__init__(1250000, True)
            self.stop_works = stop_works

        def write(self, data):
            command = self.pending + data
            result = super().write(data)
            if command == b"sensorStop 0\n":
                self.buffer = b""
                self.streaming = not self.stop_works
            if command == b"version\n" and self.streaming:
                self.buffer = b"version\r\nPlatform: xWRL6432\r\nDone\r\nmmwDemo:/>"
            return result

    monkeypatch.setattr(radar_control, "time", Clock())
    assert establish_cli(LostStopAck(True), 115200, 1250000, .1) == 1250000
    broken = LostStopAck(False)
    with pytest.raises(TimeoutError, match="still streaming"):
        establish_cli(broken, 115200, 1250000, .1)
    assert set(baud for baud, _ in broken.writes) == {1250000}


def test_stream_in_version_reply_is_stopped_even_when_probe_saw_prompt(monkeypatch):
    class PromptBeforeFrame(RadarUart):
        def reset_input_buffer(self):
            self.buffer = b"mmwDemo:/>"

        def read(self, size):
            if self.buffer:
                return super().read(size)
            return b""

        def write(self, data):
            command = self.pending + data
            result = super().write(data)
            if command == b"\n" and self.streaming:
                self.buffer = b"mmwDemo:/>"
            if command == b"version\n" and self.streaming:
                self.buffer = b"version\r\nDone\r\nmmwDemo:/>" + TI_MAGIC_WORD
            return result

    monkeypatch.setattr(radar_control, "time", Clock())
    uart = PromptBeforeFrame(1250000, True)
    assert establish_cli(uart, 115200, 1250000, 1) == 1250000
    assert not uart.streaming
    assert uart.commands.index("version") < uart.commands.index("sensorStop 0")


def test_by_id_follows_device_renumbering(tmp_path):
    device = tmp_path / "ttyACM9"
    device.touch()
    identity = tmp_path / "usb-TI-if00"
    identity.symlink_to(device)
    port = SimpleNamespace(device=str(device), vid=0x0451, pid=0xBEF3,
                           serial_number="RI32", description="XDS110 Class Application/User UART", location="1-1:1.0")
    assert select_application_port([port], explicit_port=str(identity)).device == str(device)


def test_no_reset_utility_recovers_without_usb_removal():
    from tests.test_radar_supervisor import RecoveryFixture, snapshot
    from sensors.radar_supervisor import RadarSupervisor
    with tempfile.TemporaryDirectory() as directory:
        fixture = RecoveryFixture(directory, resets=[False], watchdogs=[
            [snapshot(), snapshot(), snapshot(verified=False, frames=0, fault="radar_frame_timeout")],
            [snapshot()],
        ])
        def done():
            if not fixture.manifest.exists():
                return False
            data = fixture.payload()
            return data["state"] == "RUNNING" and len(data["epochs"]) == 2
        RadarSupervisor(fixture.config, fixture.dependencies).run(done)
        assert fixture.actions.count("configure:COM3") == 2
        assert fixture.payload()["recovery_count"] == 1
