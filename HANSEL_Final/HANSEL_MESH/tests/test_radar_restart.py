import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from sensors.ti_radar_control import (
    TI_MAGIC_WORD, apply_profile, establish_cli, partition_at_baud,
    select_application_port,
)


class RadarUart:
    def __init__(self, actual_baud=115200, streaming=False, **kwargs):
        self.actual_baud = actual_baud
        self.baudrate = 115200
        self.streaming = streaming
        self.buffer = b""
        self.commands = []

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
        if self.baudrate != self.actual_baud:
            return len(data)
        for command in data.decode().splitlines():
            if not command:
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
            self.buffer += b"Done\r\nmmwDemo:/>"
        return len(data)


@pytest.mark.parametrize("baud,streaming", [(115200, False), (1250000, False), (1250000, True)])
def test_cli_cold_and_warm_restart(baud, streaming):
    uart = RadarUart(baud, streaming)
    assert establish_cli(uart, 115200, 1250000, .001) == baud
    assert ("sensorStop 0" in uart.commands) == streaming


def test_warm_restart_reapplies_profile_without_changing_baud():
    uart = RadarUart(1250000, True)
    def open_uart(port, baud, **kwargs):
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


def test_no_data_is_not_claimed_ready():
    with pytest.raises(TimeoutError, match="CLI unavailable"):
        establish_cli(RadarUart(999), 115200, 1250000, .001)


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
