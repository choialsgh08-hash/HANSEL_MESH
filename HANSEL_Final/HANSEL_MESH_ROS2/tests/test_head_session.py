import importlib.util
import gzip
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import threading
import urllib.request

import pytest

import _paths
from hansel_localization.local_inputs import LocalInputs
from hansel_localization.map_viewer import MapState, make_server

ROOT = _paths.REPOSITORY
spec = importlib.util.spec_from_file_location("head_session", ROOT / "controller/run_head_session.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def test_commands_keep_sensors_local_and_keyboard_on_base():
    config = json.loads((ROOT / "configs/head_session.json").read_text())
    launcher.validate(config)
    steps = {name: (host, command) for host, name, command in launcher.commands(config, "/tmp/fresh.sdp")}
    assert steps["keyboard"][0] == "base"
    assert "ACTIVE_TARGETS=\"['head']\"" in steps["keyboard"][1]
    assert "HEAD_ONLY=true" in steps["operator"][1]
    for name in ("map", "odometry"):
        assert "ros_local_env.sh" in steps[name][1]
        assert "ros_env.sh" not in steps[name][1]
    assert "local_input_socket:=" in steps["odometry"][1]
    assert "ros_env.sh" in steps["inputs"][1]
    assert "/dev/serial/by-id/" in steps["unit"][1]
    assert "/dev/serial/by-id/" in steps["radar"][1]
    assert steps["camera"][0] == "head"
    assert "RTP_IMPL=ffmpeg" in steps["camera"][1]
    assert "CAMERA_SDP_FILE=/tmp/fresh.sdp" in steps["camera"][1]
    assert "start_camera_stream.sh 192.168.60.2 5600" in steps["camera"][1]


def test_equal_dds_domains_rejected():
    config = json.loads((ROOT / "configs/head_session.json").read_text())
    config["local_domain"] = config["mesh_domain"]
    with pytest.raises(ValueError, match="differ"):
        launcher.validate(config)


def test_three_unit_mode_keeps_keyboard_on_base_and_75_percent_everywhere():
    config = json.loads((ROOT / "configs/head_session.json").read_text())
    config["with_nodes"] = True
    steps = launcher.commands(config, "/tmp/test.sdp")
    drives = [(host, command) for host, name, command in steps if name == "unit"]
    assert {host for host, _ in drives} == {"head", "node1", "node2"}
    assert all("MAX_PWM=75.0" in command for _, command in drives)
    assert "HEAD_ONLY=false" in next(command for _, name, command in steps if name == "operator")
    assert steps[-1][0] == "base" and "['head','node1','node2']" in steps[-1][2]


def test_local_socket_freshness_and_exclusive_owner(tmp_path):
    path = str(tmp_path / "inputs.sock")
    receiver = LocalInputs(path)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        with pytest.raises(RuntimeError, match="owned"):
            LocalInputs(path)
        now = time.monotonic()
        for stamp in (now - 1, now + 1, float("nan"), now):
            sender.sendto(b"W" + struct.pack("!d", stamp) + b"payload", path)
        assert receiver.receive() == [(b"W", b"payload")]
    finally:
        sender.close()
        receiver.close()
    assert not Path(path).exists()
    replacement = LocalInputs(path)
    replacement.close()


def test_managed_process_stops_on_input_eof_and_keeps_unrelated_process():
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    proc = subprocess.Popen([sys.executable, str(ROOT / "scripts/run_managed.py"), "inputs", "--",
                             sys.executable, "-c", "import time; time.sleep(30)"],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        line = proc.stdout.readline().decode()
        assert "[managed] inputs pid=" in line
        child = int(line.strip().split("pid=")[1])
        assert proc.poll() is None
        proc.stdin.close()
        assert proc.wait(timeout=8) == 0
        assert not Path(f"/proc/{child}").exists()
        assert unrelated.poll() is None
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        unrelated.terminate()
        unrelated.wait()


@pytest.mark.parametrize("mode", [[], ["--with-nodes"]])
def test_dry_run_never_needs_ssh(mode):
    result = subprocess.run([sys.executable, str(ROOT / "controller/run_head_session.py"), "--dry-run", *mode],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert "[base/keyboard]" in result.stdout
    assert "[head/unit]" in result.stdout
    assert "[head/radar]" in result.stdout
    assert "[head/odometry]" in result.stdout
    assert "[head/camera]" in result.stdout
    assert "RTP_IMPL=ffmpeg" in result.stdout


def test_map_gzip_preserves_data_and_no_gzip_remains_supported():
    state = MapState()
    state.path = [{"x": i / 100, "y": 0} for i in range(1000)]
    server = make_server(state, ROOT / "monitor/odometry_map", "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        url = f"http://127.0.0.1:{server.server_port}/api/state"
        request = urllib.request.Request(url, headers={"Accept-Encoding": "gzip"})
        with urllib.request.urlopen(request, timeout=2) as response:
            assert response.headers["Content-Encoding"] == "gzip"
            compressed = response.read()
            assert len(json.loads(gzip.decompress(compressed))["path"]) == 1000
        with urllib.request.urlopen(url, timeout=2) as response:
            assert response.headers["Content-Encoding"] is None
            plain = response.read()
            assert len(json.loads(plain)["path"]) == 1000
            assert len(compressed) < len(plain)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
