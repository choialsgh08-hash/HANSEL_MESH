import json
import os
from pathlib import Path
import resource
import shutil
import signal
import subprocess
import sys
import time

import pytest

import _paths


ROOT = _paths.REPOSITORY


@pytest.mark.parametrize("mission_log", [None, "", "/tmp/radar logs/mission.jsonl"])
def test_operator_wrapper_omits_empty_optional_launch_argument(tmp_path, mission_log):
    controller = tmp_path / "controller"
    controller.mkdir()
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    (scripts / "ros_env.sh").write_text("# Test environment: no ROS or hardware.\n")
    wrapper = controller / "run_operator_stack.sh"
    shutil.copyfile(ROOT / "controller/run_operator_stack.sh", wrapper)
    binary = tmp_path / "ros2"
    binary.write_text(
        "#!/usr/bin/env python3\n"
        "import json, sys\n"
        "if any(arg.endswith(':=') for arg in sys.argv[1:]):\n"
        "    sys.exit('malformed empty launch argument')\n"
        "print(json.dumps(sys.argv[1:]))\n"
    )
    binary.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}")
    env.pop("RADAR_MISSION_LOG", None)
    if mission_log is not None:
        env["RADAR_MISSION_LOG"] = mission_log
    result = subprocess.run(["bash", str(wrapper)], env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    args = json.loads(result.stdout)
    assert args[:3] == ["launch", "hansel_bringup", "operator.launch.py"]
    optional = [arg for arg in args if arg.startswith("radar_mission_log:=")]
    assert optional == ([f"radar_mission_log:={mission_log}"] if mission_log else [])


def _no_core_dump():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


@pytest.mark.parametrize("exit_code", [0, 7])
@pytest.mark.parametrize("name", ["inputs", "router"])
def test_managed_child_exit_keeps_return_code_with_stdin_open(exit_code, name):
    proc = subprocess.Popen(
        [sys.executable, str(ROOT / "scripts/run_managed.py"), name, "--",
         sys.executable, "-c", f"import sys; sys.exit({exit_code})"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        preexec_fn=_no_core_dump,
    )
    try:
        # The SSH parent can keep stdin open after the child fails to launch.
        assert proc.wait(timeout=6) == exit_code
        assert "Fatal Python error" not in proc.stderr.read().decode()
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        proc.stdin.close()
        proc.stdout.close()
        proc.stderr.close()


def test_managed_sigterm_stops_owned_child_with_stdin_open():
    proc = subprocess.Popen(
        [sys.executable, str(ROOT / "scripts/run_managed.py"), "inputs", "--",
         sys.executable, "-c", "import time; time.sleep(30)"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        preexec_fn=_no_core_dump,
    )
    try:
        line = proc.stdout.readline().decode()
        child_pid = int(line.strip().split("pid=")[1])
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(timeout=6) == 0
        assert not Path(f"/proc/{child_pid}").exists()
        assert "Fatal Python error" not in proc.stderr.read().decode()
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        proc.stdin.close()
        proc.stdout.close()
        proc.stderr.close()


def test_managed_radar_allows_uart_shutdown_to_finish(tmp_path):
    completed = tmp_path / "sensor-stopped"
    ready = tmp_path / "ready"
    script = tmp_path / "fake_radar.py"
    script.write_text(
        "import signal, time\nfrom pathlib import Path\n"
        "def stop(*_):\n"
        "    time.sleep(2.5)\n"
        f"    Path({str(completed)!r}).touch()\n"
        "    raise SystemExit(0)\n"
        "signal.signal(signal.SIGINT, stop)\n"
        f"Path({str(ready)!r}).touch()\n"
        "while True: time.sleep(0.05)\n"
    )
    proc = subprocess.Popen(
        [sys.executable, str(ROOT / "scripts/run_managed.py"), "radar", "--",
         sys.executable, str(script)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert ready.exists()
        proc.stdin.close()
        assert proc.wait(timeout=7) == 0
        assert completed.exists(), proc.stderr.read().decode()
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        proc.stdout.close()
        proc.stderr.close()
