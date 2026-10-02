from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys

import pytest

from sensors.radar_supervisor import RadarSupervisor
from tests.test_radar_supervisor import (
    RecoveryFixture, SupervisorFixture, application_port, snapshot,
)
from tests.test_radar_stack_processes import load_launcher


@pytest.mark.parametrize("failure", [None, "radar_frame_timeout"])
def test_capture_exit_reuses_stream_once_then_falls_back_to_full_config(tmp_path, failure):
    watchdogs = [[snapshot()], [snapshot()]]
    target_epoch = 2
    if failure:
        watchdogs[1] = [snapshot(verified=False, frames=0, fault=failure)]
        watchdogs.append([snapshot()])
        target_epoch = 3
    fixture = RecoveryFixture(str(tmp_path), resets=[False], watchdogs=watchdogs)
    config = replace(fixture.config, resume_capture_on_exit=True)
    armed = False
    checks = 0

    def stop():
        nonlocal armed, checks
        checks += 1
        assert checks < 100, "recovery did not converge"
        if not fixture.manifest.exists():
            return False
        payload = fixture.payload()
        if payload["state"] != "RUNNING":
            return False
        if payload["epoch"] == 1 and not armed:
            fixture.processes.children[0].exit_code = 17
            armed = True
        return payload["epoch"] == target_epoch

    RadarSupervisor(config, fixture.dependencies).run(stop)
    assert fixture.actions.count("configure:COM3") == (2 if failure else 1)
    assert fixture.actions.count("reset:COM3") == (2 if failure else 1)
    assert fixture.payload()["epochs"][1]["configuration_reused"] is True
    if failure:
        assert fixture.payload()["epochs"][2]["configuration_reused"] is False
    else:
        assert fixture.sleeps == []  # No retry delay for the first capture-only attempt.


@pytest.mark.parametrize("reappeared_device", ["COM3", "COM9"])
def test_usb_absence_does_not_reuse_old_stream(tmp_path, reappeared_device):
    fixture = RecoveryFixture(
        str(tmp_path), resets=[False],
        ports=[[application_port("COM3")], [application_port("COM3")],
               [], [application_port(reappeared_device)]],
        watchdogs=[[snapshot()], [snapshot()]],
    )
    armed = False
    checks = 0

    def stop():
        nonlocal armed, checks
        checks += 1
        assert checks < 100
        if not fixture.manifest.exists():
            return False
        payload = fixture.payload()
        if payload["state"] == "RUNNING" and payload["epoch"] == 1 and not armed:
            fixture.processes.children[0].exit_code = 17
            armed = True
        return payload["state"] == "RUNNING" and payload["epoch"] == 2

    RadarSupervisor(replace(fixture.config, resume_capture_on_exit=True), fixture.dependencies).run(stop)
    assert fixture.payload()["epochs"][1]["configuration_reused"] is False
    assert sum(action.startswith("configure:") for action in fixture.actions) == 2


@pytest.mark.parametrize("raises", [False, True])
def test_shutdown_stops_sensor_only_after_receiver_closed_and_with_uart_lease(tmp_path, raises):
    fixture = SupervisorFixture(str(tmp_path))
    calls = []
    supervisor = None

    def stop_sensor(port, config):
        calls.append(port.serial_number)
        assert supervisor._active_capture is None
        assert supervisor._owned_children == {}
        assert supervisor._uart_lock is not None
        assert supervisor._owner_lock is not None
        if raises:
            raise OSError("USB disconnected during shutdown")

    supervisor = RadarSupervisor(fixture.config, replace(fixture.dependencies, stop_sensor=stop_sensor))
    supervisor.run(fixture.stop_when_running)
    assert calls == ["RI32"]
    payload = json.loads(fixture.manifest.read_text())
    assert payload["state"] == "STOPPED"
    assert payload["sensor_shutdown_attempted"] is True
    assert bool(payload["sensor_shutdown_error"]) is raises
    assert supervisor._owner_lock is None
    assert supervisor._uart_lock is None
    supervisor._quiesce_owned_sensor()
    assert calls == ["RI32"]


def test_shutdown_does_not_send_commands_without_ever_configuring_sensor(tmp_path):
    fixture = SupervisorFixture(str(tmp_path))
    calls = []
    dependencies = replace(fixture.dependencies, stop_sensor=lambda *args: calls.append(args))
    RadarSupervisor(fixture.config, dependencies).run(lambda: True)
    assert calls == []


def test_shutdown_does_not_send_commands_to_a_replacement_board(tmp_path):
    fixture = SupervisorFixture(str(tmp_path))
    calls = []

    def stop():
        if not fixture.stop_when_running():
            return False
        fixture.port_inventories = [[application_port("COM3", serial="OTHER")]]
        return True

    RadarSupervisor(fixture.config, replace(
        fixture.dependencies, stop_sensor=lambda *args: calls.append(args),
    )).run(stop)
    assert calls == []


def test_cli_timeout_is_independent_of_frame_verification(tmp_path, monkeypatch):
    launcher = load_launcher()
    fixture = RecoveryFixture(str(tmp_path))
    fixture.config.profile_path.write_text("channelCfg 7 3 0\nbaudRate 1250000\nsensorStart 0 0 0 0\n")
    observed = {}

    def apply_profile(**kwargs):
        observed.update(kwargs)
        return {}

    monkeypatch.setattr(launcher, "apply_profile", apply_profile)
    config = replace(fixture.config, command_timeout_s=3, verification_timeout_s=8)
    launcher._configure_profile(application_port("COM3"), config)
    assert observed["command_timeout_s"] == 3


@pytest.mark.parametrize("overrides,expected", [
    ({}, ["3", "3", "0.25", "1"]),
    ({"RADAR_VERIFY_FRAMES": "5", "RADAR_COMMAND_TIMEOUT": "4",
      "RADAR_RETRY_INITIAL": "0.5", "RADAR_RETRY_MAX": "2"}, ["5", "4", "0.5", "2"]),
])
def test_head_wrapper_uses_fast_recovery_without_changing_profile(tmp_path, overrides, expected):
    root = Path(__file__).resolve().parents[1]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    wrapper = scripts / "start_head_radar_mesh.sh"
    shutil.copyfile(root / "scripts/start_head_radar_mesh.sh", wrapper)
    profile = tmp_path / "existing.cfg"
    calibration = tmp_path / "existing.json"
    profile.write_text("unchanged profile")
    calibration.write_text("unchanged calibration")
    python = tmp_path / "python3"
    python.write_text(f"#!{sys.executable}\nimport json, sys\n"
                      "if sys.argv[1] != '-c': print(json.dumps(sys.argv[1:]))\n")
    python.chmod(0o755)
    ip = tmp_path / "ip"
    ip.write_text("#!/bin/sh\nprintf 'bat0 UNKNOWN 192.168.50.10/24\\n'\n")
    ip.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}",
               RADAR_PROFILE=str(profile), RADAR_CALIBRATION=str(calibration),
               RADAR_OUTPUT_ROOT=str(tmp_path / "output"))
    for key in ("RADAR_VERIFY_FRAMES", "RADAR_COMMAND_TIMEOUT", "RADAR_RETRY_INITIAL", "RADAR_RETRY_MAX"):
        env.pop(key, None)
    env.update(overrides)
    result = subprocess.run(["bash", str(wrapper)], env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    args = json.loads(result.stdout.splitlines()[-1])
    flags = ["--verify-frames", "--command-timeout", "--retry-initial", "--retry-max"]
    assert [args[args.index(flag) + 1] for flag in flags] == expected
    assert "--resume-capture" in args
    assert args[args.index("--cfg") + 1] == str(profile)
    assert args[args.index("--clutter-calibration") + 1] == str(calibration)


def test_dead_live_source_exits_viewer_for_supervisor_restart(tmp_path):
    root = Path(__file__).resolve().parents[1]
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    script = (
        "from monitor import radar_front as front\n"
        "def broken(self):\n"
        "    raise RuntimeError('synthetic processing crash')\n"
        "front.MissionLogFollower.run = broken\n"
        f"raise SystemExit(front.main(['--follow', {str(tmp_path / 'mission.jsonl')!r}, "
        f"'--http-port', {str(port)!r}]))\n"
    )
    result = subprocess.run([sys.executable, "-c", script], cwd=root,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 1, result.stderr
    assert "synthetic processing crash" in result.stderr
    assert "source worker exited; restarting viewer" in result.stderr
