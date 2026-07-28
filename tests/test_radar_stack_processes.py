from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from sensors.radar_owner_lock import acquire_radar_owner_lock
from sensors.radar_parent_lease import create_parent_death_lease
from sensors.radar_watchdog import ExpectedRadarEvidence, RadarEpochWatchdog
from sensors.mission_log import iter_mission_log
from sensors.radar_stack_processes import (
    ManagedChild,
    RadarStackProcesses,
    build_capture_command,
    build_viewer_command,
)
from sensors.radar_supervisor import EpochPaths, RadarSupervisorConfig
from sensors.ti_radar_control import RadarPortIdentity


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
LAUNCHER_PATH = REPOSITORY_ROOT / "scripts" / "run_radar_stack.py"


def load_launcher():
    spec = importlib.util.spec_from_file_location(
        "run_radar_stack_under_test",
        LAUNCHER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("could not create launcher module spec")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeProcess:
    def __init__(self, pid=321, poll_result=None, waits=()):
        self.pid = pid
        self._poll_result = poll_result
        self._waits = iter(waits)
        self.events = []

    def poll(self):
        return self._poll_result

    def wait(self, timeout):
        self.events.append(("wait", timeout))
        result = next(self._waits)
        if isinstance(result, BaseException):
            raise result
        return result

    def send_signal(self, value):
        self.events.append(("send_signal", value))

    def terminate(self):
        self.events.append(("terminate",))

    def kill(self):
        self.events.append(("kill",))


class RadarStackProcessesTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        output = Path(self.temporary_directory.name) / "output"
        self.port = RadarPortIdentity(
            device="COM3",
            vid=0x0451,
            pid=0xBEF3,
            serial_number="SERIAL",
            description="XDS110 Application/User UART",
            location="usb-1",
        )
        self.root = Path("repository")
        self.paths = EpochPaths(
            mission=output / "missions/epoch.jsonl",
            raw=output / "captures/epoch.bin",
            raw_index=output / "captures/epoch.bin.chunks.jsonl",
            runtime_dir=output / "runtime/run",
            capture_stdout=output / "runtime/run/capture.out",
            capture_stderr=output / "runtime/run/capture.err",
            viewer_stdout=output / "runtime/run/viewer.out",
            viewer_stderr=output / "runtime/run/viewer.err",
        )
        self.config = RadarSupervisorConfig(
            repository_root=self.root,
            output_root=output,
            profile_path=Path("profile.cfg"),
            calibration_path=Path("clutter.json"),
            run_id="run-1",
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_capture_command_passes_pinned_radar_settings_to_radar_live(self):
        self.assertEqual(
            build_capture_command(self.port, self.paths, self.config),
            [
                sys.executable, "-m", "sensors", "radar-live", "--port", "COM3",
                "--baud", "1250000", "--allow-elided-empty-point-tlv",
                "--allow-nonzero-padding", "--heatmap-azimuth-bins", "16",
                "--heatmap-range-bins", "128", "--heatmap-range-step-m",
                "0.09765625", "--output", str(self.paths.mission),
                "--raw-output", str(self.paths.raw), "--raw-index",
                str(self.paths.raw_index), "--mission-id",
                "radar-board-live", "--profile-id",
                "lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1",
                "--calibration-id", "uncalibrated",
            ],
        )

    def test_viewer_command_follows_epoch_with_supervisor_settings(self):
        self.assertEqual(
            build_viewer_command(self.paths, self.config),
            [
                sys.executable, "monitor/radar_front.py", "--follow",
                str(self.paths.mission), "--clutter-calibration",
                str(self.config.calibration_path), "--supervisor-manifest",
                str(
                    self.config.output_root
                    / "runtime"
                    / "radar-supervisor-run-1.json"
                ),
                "--bind", "127.0.0.1", "--http-port", "8081",
                "--max-range-m", "3", "--history-window", "0.3", "--quiet",
            ],
        )

    def test_owned_commands_receive_parent_death_and_uart_handoff(self):
        parent_lease = self.paths.runtime_dir / "capture-parent.lease"
        capture = build_capture_command(
            self.port,
            self.paths,
            self.config,
            parent_lease_path=parent_lease,
        )
        self.assertEqual(
            capture[-6:],
            [
                "--supervisor-parent-lease",
                str(parent_lease),
                "--xds-owner-serial",
                "SERIAL",
                "--xds-owner-run-id",
                "run-1",
            ],
        )

        viewer_lease = self.paths.runtime_dir / "viewer-parent.lease"
        viewer = build_viewer_command(
            self.paths,
            self.config,
            parent_lease_path=viewer_lease,
        )
        self.assertEqual(
            viewer[-2:],
            [
                "--supervisor-parent-lease",
                str(viewer_lease),
            ],
        )

    def test_switch_viewer_stops_registered_current_before_starting_replacement(self):
        events = []

        class OrderedStopProcess(FakeProcess):
            def wait(self, timeout):
                result = super().wait(timeout)
                events.append("stop:old-viewer")
                return result

        old = OrderedStopProcess(pid=10, waits=(0,))
        new = FakeProcess(pid=11, poll_result=0)

        def start(command, **kwargs):
            if not events:
                events.append("start:old-viewer")
                return old
            events.append("start:new-viewer")
            return new

        manager = RadarStackProcesses(popen_factory=start)
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"), mock.patch("sensors.radar_stack_processes.os.killpg", create=True) as killpg, mock.patch(
            "sensors.radar_stack_processes.os.getpgid", return_value=77, create=True
        ):
            current = manager.switch_viewer(None, self.paths, self.config)
            replacement = manager.switch_viewer(current, self.paths, self.config)

        self.assertEqual(events, ["start:old-viewer", "stop:old-viewer", "start:new-viewer"])
        killpg.assert_not_called()
        self.assertEqual(replacement.pid, 11)
        manager.stop_owned_children()

    def test_start_capture_owns_child_and_creates_runtime_log_parents(self):
        child_process = FakeProcess(poll_result=0)
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / "nested" / "runtime"
            paths = EpochPaths(
                self.paths.mission, self.paths.raw, self.paths.raw_index, runtime,
                runtime / "capture.out", runtime / "capture.err",
                runtime / "viewer.out", runtime / "viewer.err",
            )
            started = []
            manager = RadarStackProcesses(
                popen_factory=lambda command, **kwargs: started.append((command, kwargs)) or child_process
            )
            child = manager.start_capture(self.port, paths, self.config)

            self.assertTrue(paths.capture_stdout.parent.exists())
            self.assertEqual(child.role, "capture")
            self.assertEqual(started[0][1]["cwd"], self.root)
            self.assertEqual(
                Path(started[0][1]["stdout"].name), paths.capture_stdout
            )
            self.assertEqual(
                Path(started[0][1]["stderr"].name), paths.capture_stderr
            )
            self.assertTrue(started[0][1]["stdout"].closed)
            self.assertTrue(started[0][1]["stderr"].closed)
            if os.name == "nt":
                self.assertEqual(
                    started[0][1]["creationflags"],
                    subprocess.CREATE_NEW_PROCESS_GROUP,
                )
                self.assertEqual(
                    started[0][1]["startupinfo"].dwFlags
                    & subprocess.STARTF_USESHOWWINDOW,
                    subprocess.STARTF_USESHOWWINDOW,
                )
                self.assertEqual(
                    started[0][1]["startupinfo"].wShowWindow,
                    subprocess.SW_HIDE,
                )
            manager.stop_owned_children()

    def test_parent_death_lease_blocks_uart_reuse_until_child_closes(self):
        root = Path(self.temporary_directory.name)
        parent_lease_root = root / "parent-leases"
        uart_lock_root = root / "uart-locks"
        child_ready = root / "child-ready"
        stop_observed = root / "stop-observed"
        child_stopped = root / "child-stopped"
        child_pid_path = root / "child-pid"
        allow_uart_release = root / "allow-uart-release"
        child_code = textwrap.dedent(
            """
            import os
            from pathlib import Path
            import sys
            import time
            from sensors.radar_owner_lock import acquire_radar_owner_lock
            from sensors.radar_parent_lease import start_parent_death_watcher

            parent_lease_path = Path(sys.argv[1])
            uart_lock_root = Path(sys.argv[2])
            child_ready = Path(sys.argv[3])
            stop_observed = Path(sys.argv[4])
            child_stopped = Path(sys.argv[5])
            child_pid_path = Path(sys.argv[6])
            allow_uart_release = Path(sys.argv[7])
            watcher = start_parent_death_watcher(parent_lease_path)
            if not watcher.ready.wait(5):
                raise RuntimeError("parent-death watcher did not become ready")
            if watcher.stop_requested.is_set():
                raise RuntimeError("parent died before child startup")
            uart_lock = acquire_radar_owner_lock(
                uart_lock_root, "RI32", "capture-run"
            )
            child_pid_path.write_text(str(os.getpid()), encoding="utf-8")
            child_ready.write_text("ready", encoding="utf-8")
            if not watcher.stop_requested.wait(10):
                raise RuntimeError("parent death was not observed")
            stop_observed.write_text("stopping", encoding="utf-8")
            deadline = time.monotonic() + 5
            while (
                not allow_uart_release.exists()
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            uart_lock.release()
            child_stopped.write_text("stopped", encoding="utf-8")
            """
        )
        parent_code = textwrap.dedent(
            """
            import os
            from pathlib import Path
            import subprocess
            import sys
            import time
            from sensors.radar_parent_lease import create_parent_death_lease

            root = Path(sys.argv[1])
            child_code = sys.argv[2]
            parent_lease_root = Path(sys.argv[3])
            uart_lock_root = Path(sys.argv[4])
            child_ready = Path(sys.argv[5])
            stop_observed = Path(sys.argv[6])
            child_stopped = Path(sys.argv[7])
            child_pid_path = Path(sys.argv[8])
            allow_uart_release = Path(sys.argv[9])
            lease = create_parent_death_lease(
                parent_lease_root, "capture"
            )
            subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    child_code,
                    str(lease.path),
                    str(uart_lock_root),
                    str(child_ready),
                    str(stop_observed),
                    str(child_stopped),
                    str(child_pid_path),
                    str(allow_uart_release),
                ],
                cwd=root,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            deadline = time.monotonic() + 5
            while not child_ready.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not child_ready.exists():
                raise RuntimeError("child did not acquire UART lease")
            os._exit(0)
            """
        )
        parent = subprocess.Popen(
            [
                sys.executable,
                "-c",
                parent_code,
                str(REPOSITORY_ROOT),
                child_code,
                str(parent_lease_root),
                str(uart_lock_root),
                str(child_ready),
                str(stop_observed),
                str(child_stopped),
                str(child_pid_path),
                str(allow_uart_release),
            ],
            cwd=REPOSITORY_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        child_pid: int | None = None
        try:
            deadline = time.monotonic() + 10
            while (
                not child_ready.exists()
                and parent.poll() is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            if not child_ready.exists():
                _, parent_stderr = parent.communicate(timeout=5)
                self.fail(
                    "parent-death child was not ready: "
                    f"{parent_stderr.strip()}"
                )
            child_pid = int(child_pid_path.read_text(encoding="utf-8"))
            parent_stdout, parent_stderr = parent.communicate(timeout=5)
            self.assertEqual(
                parent.returncode,
                0,
                f"{parent_stdout}\n{parent_stderr}",
            )

            deadline = time.monotonic() + 5
            while (
                not stop_observed.exists()
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            self.assertTrue(stop_observed.exists())
            with self.assertRaisesRegex(RuntimeError, "capture-run"):
                acquire_radar_owner_lock(
                    uart_lock_root,
                    "RI32",
                    "replacement-supervisor",
                )
            allow_uart_release.write_text("release", encoding="utf-8")

            recovered = None
            deadline = time.monotonic() + 5
            while recovered is None and time.monotonic() < deadline:
                try:
                    recovered = acquire_radar_owner_lock(
                        uart_lock_root,
                        "RI32",
                        "replacement-supervisor",
                    )
                except RuntimeError:
                    time.sleep(0.02)
            self.assertIsNotNone(recovered)
            assert recovered is not None
            recovered.release()
            self.assertTrue(child_stopped.exists())

        finally:
            allow_uart_release.touch(exist_ok=True)
            if parent.poll() is None:
                parent.kill()
            if parent.stdout is not None and not parent.stdout.closed:
                parent.communicate(timeout=5)
            if child_pid is not None and not child_stopped.exists():
                try:
                    os.kill(child_pid, signal.SIGTERM)
                except OSError:
                    pass

    def test_parent_watcher_does_not_treat_repeated_contention_as_death(self):
        import sensors.radar_owner_lock as owner_lock_module
        from sensors.radar_parent_lease import start_parent_death_watcher

        path = Path(self.temporary_directory.name) / "parent.lease"
        attempts = 0
        repeated_contention = threading.Event()
        release_allowed = threading.Event()

        def fake_locking(_descriptor, mode, _length):
            nonlocal attempts
            if mode == 2:
                return
            if mode != 1:
                raise AssertionError("blocking LK_LOCK must not be used")
            attempts += 1
            if not release_allowed.is_set():
                if attempts >= 12:
                    repeated_contention.set()
                raise PermissionError(13, "lease is still locked")

        fake_msvcrt = SimpleNamespace(
            LK_NBLCK=1,
            LK_UNLCK=2,
            LK_LOCK=3,
            locking=fake_locking,
        )
        with mock.patch.object(
            owner_lock_module,
            "_IS_WINDOWS",
            True,
        ), mock.patch.dict(
            sys.modules,
            {"msvcrt": fake_msvcrt},
        ):
            watcher = start_parent_death_watcher(path)
            self.assertTrue(watcher.ready.wait(1.0))
            self.assertTrue(repeated_contention.wait(2.0))
            self.assertFalse(watcher.stop_requested.is_set())
            release_allowed.set()
            self.assertTrue(watcher.stop_requested.wait(1.0))
            watcher.thread.join(1.0)
            self.assertFalse(watcher.thread.is_alive())

    def test_managed_parent_stop_preserves_capture_footer_before_group_signal(self):
        root = Path(self.temporary_directory.name)
        lease = create_parent_death_lease(root / "leases", "capture")
        mission = root / "mission.jsonl"
        raw = root / "capture.bin"
        index = root / "capture.index.jsonl"
        capture_ready = root / "capture-ready"
        footer_started = root / "footer-started"
        helper_code = textwrap.dedent(
            """
            import sys
            import time
            import types
            from pathlib import Path

            from sensors.cli import _radar_shutdown_signals
            from sensors.radar_capture import capture_radar_uart
            import sensors.radar_capture as radar_capture
            from sensors.radar_parent_lease import start_parent_death_watcher

            lease_path = Path(sys.argv[1])
            mission = Path(sys.argv[2])
            raw = Path(sys.argv[3])
            index = Path(sys.argv[4])
            capture_ready = Path(sys.argv[5])
            footer_started = Path(sys.argv[6])

            class EmptySerial:
                def __enter__(self):
                    return self

                def __exit__(self, *unused):
                    return False

                def read(self, size):
                    del size
                    capture_ready.touch(exist_ok=True)
                    time.sleep(0.01)
                    return b""

            original_canonical_json_bytes = radar_capture.canonical_json_bytes

            def pause_during_capture_footer(value):
                if (
                    isinstance(value, dict)
                    and value.get("record_type") == "capture_end"
                ):
                    footer_started.touch(exist_ok=True)
                    time.sleep(0.4)
                return original_canonical_json_bytes(value)

            radar_capture.canonical_json_bytes = pause_during_capture_footer
            sys.modules["serial"] = types.SimpleNamespace(
                Serial=lambda **unused: EmptySerial()
            )
            watcher = start_parent_death_watcher(lease_path)
            if not watcher.ready.wait(5.0):
                raise RuntimeError("parent-death watcher did not become ready")
            with _radar_shutdown_signals():
                capture_radar_uart(
                    port="COM_TEST",
                    baudrate=115200,
                    mission_log=mission,
                    mission_id="mission-1",
                    profile_id="profile-1",
                    calibration_id="uncalibrated",
                    unit_id="head",
                    boot_id="boot-1",
                    raw_capture=raw,
                    raw_index=index,
                    duration_s=0.0,
                    serial_timeout_s=0.01,
                    stop_requested=watcher.stop_requested.is_set,
                )
            """
        )
        popen_kwargs = {
            "cwd": REPOSITORY_ROOT,
            "stdout": subprocess.DEVNULL,
            "stderr": subprocess.DEVNULL,
        }
        if os.name == "nt":
            popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            popen_kwargs["start_new_session"] = True
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                helper_code,
                str(lease.path),
                str(mission),
                str(raw),
                str(index),
                str(capture_ready),
                str(footer_started),
            ],
            **popen_kwargs,
        )

        class FooterBarrierProcess:
            def __init__(self, wrapped):
                self.wrapped = wrapped
                self.pid = wrapped.pid
                self.signal_sent = False

            def poll(self):
                return self.wrapped.poll()

            def wait(self, timeout):
                return self.wrapped.wait(timeout=timeout)

            def send_signal(self, value):
                if not self._wait_for_footer():
                    raise RuntimeError("capture footer was not reached")
                self.signal_sent = True
                return self.wrapped.send_signal(value)

            def terminate(self):
                return self.wrapped.terminate()

            def kill(self):
                return self.wrapped.kill()

            @staticmethod
            def _wait_for_footer():
                deadline = time.monotonic() + 5.0
                while (
                    not footer_started.exists()
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.01)
                return footer_started.exists()

        wrapped = FooterBarrierProcess(process)
        child = ManagedChild("capture", wrapped, parent_lease=lease)
        original_killpg = getattr(os, "killpg", None)

        def interrupt_group_after_footer(process_group, signum):
            if not wrapped._wait_for_footer():
                raise RuntimeError("capture footer was not reached")
            wrapped.signal_sent = True
            assert original_killpg is not None
            return original_killpg(process_group, signum)

        try:
            deadline = time.monotonic() + 5.0
            while (
                not capture_ready.exists()
                and process.poll() is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            self.assertTrue(capture_ready.exists())
            if os.name == "nt":
                result = child.stop(grace_s=1.0)
            else:
                with mock.patch(
                    "sensors.radar_stack_processes.os.killpg",
                    side_effect=interrupt_group_after_footer,
                ):
                    result = child.stop(grace_s=1.0)

            self.assertEqual(result.exit_code, 0)
            self.assertEqual(result.escalation, "graceful")
            self.assertFalse(wrapped.signal_sent)
            footer = json.loads(index.read_text("utf-8").splitlines()[-1])
            self.assertEqual(footer["record_type"], "capture_end")
            self.assertEqual(footer["stop_reason"], "stop_requested")
            health_records = [
                entry.record
                for entry in iter_mission_log(mission)
                if type(entry.record).__name__ == "SensorHealth"
            ]
            self.assertTrue(health_records)
            self.assertIn("health_kind=final", health_records[-1].detail)
        finally:
            lease.release()
            if process.poll() is None:
                process.kill()
            process.wait(timeout=5.0)

    def test_start_capture_uses_posix_session_when_selected(self):
        process = FakeProcess(pid=82, poll_result=0)
        started = []
        manager = RadarStackProcesses(
            popen_factory=lambda command, **kwargs: started.append(kwargs) or process
        )
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"):
            manager.start_capture(self.port, self.paths, self.config)

        self.assertTrue(started[0]["start_new_session"])
        manager.stop_owned_children()

    @unittest.skipUnless(os.name == "nt", "Windows launch semantics")
    def test_windows_child_is_hidden_without_disabling_ctrl_break_delivery(self):
        process = FakeProcess(pid=83, poll_result=0)
        started = []
        manager = RadarStackProcesses(
            popen_factory=lambda command, **kwargs: (
                started.append(kwargs) or process
            )
        )

        manager.start_capture(self.port, self.paths, self.config)

        self.assertEqual(
            started[0]["creationflags"],
            subprocess.CREATE_NEW_PROCESS_GROUP,
        )
        startupinfo = started[0]["startupinfo"]
        self.assertEqual(
            startupinfo.dwFlags & subprocess.STARTF_USESHOWWINDOW,
            subprocess.STARTF_USESHOWWINDOW,
        )
        self.assertEqual(startupinfo.wShowWindow, subprocess.SW_HIDE)
        self.assertFalse(
            started[0]["creationflags"] & subprocess.CREATE_NO_WINDOW
        )
        manager.stop_owned_children()

    def test_stop_uses_windows_ctrl_break_before_waiting(self):
        process = FakeProcess(waits=(0,))
        child = ManagedChild("capture", process)
        with mock.patch("sensors.radar_stack_processes.os.name", "nt"), mock.patch(
            "sensors.radar_stack_processes.signal.CTRL_BREAK_EVENT", 41, create=True
        ):
            result = child.stop(0.25)

        self.assertEqual(result.escalation, "graceful")
        self.assertEqual(process.events, [("send_signal", 41), ("wait", 0.25)])

    def test_stop_uses_posix_interrupt_before_terminate_or_kill(self):
        process = FakeProcess(waits=(0,))
        child = ManagedChild("capture", process)
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"), mock.patch("sensors.radar_stack_processes.os.killpg", create=True) as killpg, mock.patch(
            "sensors.radar_stack_processes.os.getpgid", return_value=77, create=True
        ):
            result = child.stop(0.25)

        self.assertEqual(result.escalation, "graceful")
        self.assertEqual(result.exit_code, 0)
        killpg.assert_called_once_with(77, signal.SIGINT)
        self.assertEqual(process.events, [("wait", 0.25)])

    def test_stop_escalates_to_terminate_after_grace_timeout(self):
        process = FakeProcess(waits=(subprocess.TimeoutExpired("x", 1), 3))
        child = ManagedChild("viewer", process)
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"), mock.patch("sensors.radar_stack_processes.os.killpg", create=True), mock.patch(
            "sensors.radar_stack_processes.os.getpgid", return_value=77, create=True
        ):
            result = child.stop(0.25)

        self.assertEqual(result.escalation, "terminate")
        self.assertEqual(result.exit_code, 3)
        self.assertEqual(process.events, [("wait", 0.25), ("terminate",), ("wait", 0.25)])

    def test_stop_escalates_to_kill_after_terminate_timeout(self):
        process = FakeProcess(waits=(subprocess.TimeoutExpired("x", 1), subprocess.TimeoutExpired("x", 1), 9))
        child = ManagedChild("capture", process)
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"), mock.patch("sensors.radar_stack_processes.os.killpg", create=True), mock.patch(
            "sensors.radar_stack_processes.os.getpgid", return_value=77, create=True
        ):
            result = child.stop(0.25)

        self.assertEqual(result.escalation, "kill")
        self.assertEqual(result.exit_code, 9)
        self.assertEqual(process.events, [("wait", 0.25), ("terminate",), ("wait", 0.25), ("kill",), ("wait", 0.25)])

    def test_stop_returns_already_exited_without_signaling(self):
        process = FakeProcess(poll_result=4)
        result = ManagedChild("viewer", process).stop()
        self.assertEqual((result.role, result.pid, result.exit_code, result.escalation), ("viewer", 321, 4, "already_exited"))
        self.assertEqual(process.events, [])

    def test_manager_rejects_unowned_unregistered_and_replaced_children(self):
        manager = RadarStackProcesses()
        for child in (ManagedChild("capture", FakeProcess(), owned=False), ManagedChild("capture", FakeProcess())):
            with self.assertRaisesRegex(RuntimeError, "unowned|unregistered"):
                manager.stop_child(child)

        registered_process = FakeProcess(pid=12, poll_result=0)
        manager = RadarStackProcesses(popen_factory=lambda *args, **kwargs: registered_process)
        registered = manager.start_capture(self.port, self.paths, self.config)
        replacement = ManagedChild("capture", FakeProcess(pid=12))
        with self.assertRaisesRegex(RuntimeError, "unregistered"):
            manager.stop_child(replacement)
        self.assertEqual(manager.stop_owned_children()[0].pid, 12)

    def test_manager_rejects_same_wrapper_with_replaced_process_before_signal(self):
        original = FakeProcess(pid=16, poll_result=0)
        replacement = FakeProcess(pid=16, poll_result=0)
        manager = RadarStackProcesses(popen_factory=lambda *args, **kwargs: original)
        child = manager.start_capture(self.port, self.paths, self.config)
        child.process = replacement

        with self.assertRaisesRegex(RuntimeError, "unregistered"):
            manager.stop_child(child)

        self.assertEqual(original.events, [])
        self.assertEqual(replacement.events, [])
        child.process = original
        self.assertEqual(manager.stop_owned_children()[0].pid, 16)

    def test_duplicate_pid_registration_stops_new_process_before_raising(self):
        existing = FakeProcess(pid=31, poll_result=0)
        duplicate = FakeProcess(
            pid=31,
            waits=(subprocess.TimeoutExpired("duplicate", 2.0), 0),
        )
        processes = iter((existing, duplicate))
        manager = RadarStackProcesses(
            popen_factory=lambda *args, **kwargs: next(processes)
        )
        manager.start_capture(self.port, self.paths, self.config)

        with mock.patch("sensors.radar_stack_processes.os.name", "nt"), mock.patch(
            "sensors.radar_stack_processes.signal.CTRL_BREAK_EVENT", 41, create=True
        ):
            with self.assertRaisesRegex(RuntimeError, "duplicate owned process pid"):
                manager.switch_viewer(None, self.paths, self.config)

        self.assertEqual(
            duplicate.events,
            [("wait", 2.0), ("send_signal", 41), ("wait", 2.0)],
        )
        self.assertEqual(manager.stop_owned_children()[0].pid, 31)

    def test_owned_shutdown_reports_failure_after_stopping_later_children(self):
        first = FakeProcess(pid=51, waits=(RuntimeError("wait failed"), 7))
        second = FakeProcess(pid=52, poll_result=0)
        processes = iter((first, second))
        manager = RadarStackProcesses(
            popen_factory=lambda *args, **kwargs: next(processes)
        )
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"), mock.patch(
            "sensors.radar_stack_processes.os.killpg", create=True
        ), mock.patch("sensors.radar_stack_processes.os.getpgid", return_value=9, create=True):
            manager.start_capture(self.port, self.paths, self.config)
            manager.switch_viewer(None, self.paths, self.config)
            with self.assertRaises(RuntimeError) as raised:
                manager.stop_owned_children()

            self.assertEqual(raised.exception.results[0].pid, 52)
            self.assertEqual(manager.stop_owned_children()[0].pid, 51)

    def test_owned_shutdown_returns_all_successful_stop_results(self):
        first = FakeProcess(pid=61, poll_result=0)
        second = FakeProcess(pid=62, poll_result=3)
        processes = iter((first, second))
        manager = RadarStackProcesses(
            popen_factory=lambda *args, **kwargs: next(processes)
        )
        manager.start_capture(self.port, self.paths, self.config)
        manager.switch_viewer(None, self.paths, self.config)

        self.assertEqual(
            [(result.role, result.pid, result.exit_code) for result in manager.stop_owned_children()],
            [("capture", 61, 0), ("viewer", 62, 3)],
        )

    def test_stopped_child_is_not_stopped_twice_by_owned_shutdown(self):
        process = FakeProcess(pid=98, poll_result=0)
        manager = RadarStackProcesses(popen_factory=lambda *args, **kwargs: process)
        child = manager.switch_viewer(None, self.paths, self.config)
        first = manager.stop_child(child)
        self.assertEqual(first.pid, 98)
        self.assertEqual(manager.stop_owned_children(), ())


class RadarStackLauncherTests(unittest.TestCase):
    def test_parser_defaults_and_help_work_outside_repository(self):
        launcher = load_launcher()
        args = launcher.build_parser().parse_args([])
        self.assertEqual(args.port, None)
        self.assertEqual(args.xds_serial, None)
        self.assertEqual(args.run_id, None)
        self.assertEqual(args.output_root, REPOSITORY_ROOT)
        self.assertEqual(args.frame_timeout, 2.5)
        self.assertEqual(args.first_frame_timeout, 3.0)
        self.assertEqual(args.verify_frames, 5)
        self.assertEqual(args.http_port, 8081)
        self.assertEqual(
            args.cfg.name,
            "iwrl6432_3d_operator_near_10hz.cfg",
        )

        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, str(LAUNCHER_PATH), "--help"],
                cwd=directory,
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--clutter-calibration", result.stdout)
        self.assertIn("--xds-serial", result.stdout)

    def test_parser_and_main_reject_unsafe_values_before_external_effects(self):
        launcher = load_launcher()
        parser = launcher.build_parser()
        invalid_parser_args = (
            ("--frame-timeout", "0"),
            ("--first-frame-timeout", "-1"),
            ("--verification-timeout", "0"),
            ("--verify-frames", "0"),
            ("--retry-initial", "-0.5"),
            ("--retry-max", "0"),
            ("--http-port", "0"),
            ("--http-port", "65536"),
        )
        for option, value in invalid_parser_args:
            with self.subTest(option=option, value=value):
                with self.assertRaises(SystemExit):
                    parser.parse_args([option, value])

        with self.assertRaisesRegex(SystemExit, "retry"):
            launcher.main(
                [
                    "--clutter-calibration",
                    str(__file__),
                    "--retry-initial",
                    "2",
                    "--retry-max",
                    "1",
                ]
            )
        missing = Path(tempfile.gettempdir()) / "missing-radar-calibration.json"
        with self.assertRaisesRegex(SystemExit, "calibration"):
            launcher.main(["--clutter-calibration", str(missing)])

    def test_main_generates_one_utc_run_id_and_composes_real_dependencies(self):
        launcher = load_launcher()
        port = SimpleNamespace(
            device="COM9",
            vid=0x0451,
            pid=0xBEF3,
            serial_number="RI32",
            description="XDS110 Application/User UART",
            location="usb-1",
        )
        selected = RadarPortIdentity(
            device="COM9",
            vid=0x0451,
            pid=0xBEF3,
            serial_number="RI32",
            description="XDS110 Application/User UART",
            location="usb-1",
        )
        generated_at = datetime(2026, 7, 29, 1, 2, 3, tzinfo=timezone.utc)
        reset_executable = Path("C:/ti/xds110reset.exe")
        captured: dict[str, object] = {}
        installed_handlers: dict[int, object] = {}

        class FakeSupervisor:
            def __init__(self, config, dependencies):
                captured["config"] = config
                captured["dependencies"] = dependencies

            def run(self, stop_requested):
                captured["stop_before_signal"] = stop_requested()
                handler = installed_handlers[signal.SIGTERM]
                handler(signal.SIGTERM, None)
                captured["stop_after_signal"] = stop_requested()

        def install_signal(signum, handler):
            if callable(handler):
                installed_handlers[signum] = handler

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = root / "clutter.json"
            calibration.write_text("{}", encoding="utf-8")
            output = root / "radar-output"
            with (
                mock.patch.object(
                    launcher,
                    "_utc_now",
                    mock.Mock(return_value=generated_at),
                ) as utc_now,
                mock.patch.object(
                    launcher,
                    "comports",
                    mock.Mock(return_value=[port]),
                ) as comports,
                mock.patch.object(
                    launcher,
                    "find_xds110_reset",
                    mock.Mock(return_value=reset_executable),
                ) as find_reset,
                mock.patch.object(launcher, "RadarSupervisor", FakeSupervisor),
                mock.patch.object(
                    launcher.signal,
                    "signal",
                    side_effect=install_signal,
                ),
                mock.patch.object(
                    launcher.signal,
                    "getsignal",
                    return_value=signal.SIG_DFL,
                ),
            ):
                result = launcher.main(
                    [
                        "--xds-serial",
                        "RI32",
                        "--clutter-calibration",
                        str(calibration),
                        "--output-root",
                        str(output),
                    ]
                )

        self.assertEqual(result, 0)
        self.assertEqual(utc_now.call_count, 1)
        config = captured["config"]
        dependencies = captured["dependencies"]
        self.assertEqual(config.run_id, "20260729-010203")
        self.assertEqual(config.output_root, output)
        self.assertEqual(config.calibration_path, calibration)
        self.assertEqual(config.reset_executable, reset_executable)
        self.assertIsNone(config.reset_unavailable_reason)
        self.assertEqual(config.data_baud, 1_250_000)
        self.assertEqual(config.http_bind, "127.0.0.1")
        self.assertIs(dependencies.port_provider, comports)
        self.assertIsInstance(dependencies.processes, RadarStackProcesses)
        self.assertIs(dependencies.monotonic, launcher.time.monotonic)
        self.assertIs(dependencies.sleep, launcher.time.sleep)
        self.assertFalse(captured["stop_before_signal"])
        self.assertTrue(captured["stop_after_signal"])
        self.assertIn(signal.SIGINT, installed_handlers)
        self.assertIn(signal.SIGTERM, installed_handlers)
        find_reset.assert_called_once_with(None, (Path("C:/ti"),))

        with (
            mock.patch.object(
                launcher,
                "reset_xds110_target",
            ) as reset_target,
            mock.patch.object(
                launcher,
                "apply_profile",
                return_value={
                    "commands_completed": 25,
                    "new_baud_prompt_observed": True,
                    "first_magic_observed": True,
                },
            ) as apply_profile,
        ):
            self.assertTrue(dependencies.reset_target(selected, config))
            profile_result = dependencies.configure(selected, config)

        reset_target.assert_called_once_with(
            reset_executable,
            "RI32",
            launcher.subprocess.run,
        )
        profile = apply_profile.call_args.kwargs["profile"]
        self.assertEqual(profile.target_baud, config.data_baud)
        self.assertEqual(apply_profile.call_args.kwargs["port"], "COM9")
        self.assertEqual(profile_result["commands_completed"], 25)
        paths = EpochPaths(
            mission=output / "missions" / "e.jsonl",
            raw=output / "captures" / "e.bin",
            raw_index=output / "captures" / "e.index.jsonl",
            runtime_dir=output / "runtime" / "run",
            capture_stdout=output / "runtime" / "capture.out",
            capture_stderr=output / "runtime" / "capture.err",
            viewer_stdout=output / "runtime" / "viewer.out",
            viewer_stderr=output / "runtime" / "viewer.err",
        )
        watchdog = dependencies.watchdog_factory(paths, config, 4.5)
        self.assertIsInstance(watchdog, RadarEpochWatchdog)
        self.assertEqual(
            watchdog._expected,
            ExpectedRadarEvidence(
                profile_id=config.profile_id,
                heatmap_azimuth_bins=16,
                heatmap_range_bins=128,
                heatmap_range_step_m=0.09765625,
            ),
        )

    def test_reset_discovery_failure_is_preserved_as_recoverable_capability(self):
        launcher = load_launcher()
        captured: dict[str, object] = {}
        reason = (
            "xds110reset executable was not found; install TI UniFlash "
            "or provide its path"
        )

        class FakeSupervisor:
            def __init__(self, config, dependencies):
                captured["config"] = config
                captured["dependencies"] = dependencies

            def run(self, stop_requested):
                del stop_requested

        with tempfile.TemporaryDirectory() as directory:
            calibration = Path(directory) / "clutter.json"
            calibration.write_text("{}", encoding="utf-8")
            with (
                mock.patch.object(
                    launcher,
                    "find_xds110_reset",
                    side_effect=RuntimeError(reason),
                ),
                mock.patch.object(launcher, "RadarSupervisor", FakeSupervisor),
                mock.patch.object(launcher.signal, "signal"),
                mock.patch.object(
                    launcher.signal,
                    "getsignal",
                    return_value=signal.SIG_DFL,
                ),
            ):
                self.assertEqual(
                    launcher.main(
                        [
                            "--run-id",
                            "known-run",
                            "--clutter-calibration",
                            str(calibration),
                        ]
                    ),
                    0,
                )

        config = captured["config"]
        dependencies = captured["dependencies"]
        self.assertIsNone(config.reset_executable)
        self.assertEqual(config.reset_unavailable_reason, reason)
        self.assertFalse(
            dependencies.reset_target(
                RadarPortIdentity(
                    "COM3",
                    0x0451,
                    0xBEF3,
                    "RI32",
                    "XDS110 Application/User UART",
                    "usb-1",
                ),
                config,
            )
        )

    def test_relative_paths_are_resolved_once_from_the_callers_working_directory(self):
        launcher = load_launcher()
        captured: dict[str, object] = {}

        class FakeSupervisor:
            def __init__(self, config, dependencies):
                del dependencies
                captured["config"] = config

            def run(self, stop_requested):
                del stop_requested

        with tempfile.TemporaryDirectory() as directory:
            caller_root = Path(directory)
            profile = caller_root / "profile.cfg"
            profile.write_text(
                "sensorStop\nbaudRate 1250000\nsensorStart\n",
                encoding="utf-8",
            )
            calibration = caller_root / "calibration.json"
            calibration.write_text("{}", encoding="utf-8")
            reset_executable = caller_root / "xds110reset.exe"
            reset_executable.touch()
            previous_cwd = Path.cwd()
            try:
                os.chdir(caller_root)
                with (
                    mock.patch.object(
                        launcher,
                        "find_xds110_reset",
                        side_effect=lambda explicit, roots: explicit,
                    ),
                    mock.patch.object(
                        launcher,
                        "RadarSupervisor",
                        FakeSupervisor,
                    ),
                    mock.patch.object(launcher.signal, "signal"),
                    mock.patch.object(
                        launcher.signal,
                        "getsignal",
                        return_value=signal.SIG_DFL,
                    ),
                ):
                    self.assertEqual(
                        launcher.main(
                            [
                                "--run-id",
                                "relative-path-run",
                                "--output-root",
                                "relative-out",
                                "--cfg",
                                "profile.cfg",
                                "--clutter-calibration",
                                "calibration.json",
                                "--reset-executable",
                                "xds110reset.exe",
                            ]
                        ),
                        0,
                    )
            finally:
                os.chdir(previous_cwd)

        config = captured["config"]
        self.assertEqual(
            config.output_root,
            caller_root / "relative-out",
        )
        self.assertEqual(config.profile_path, profile)
        self.assertEqual(config.calibration_path, calibration)
        self.assertEqual(config.reset_executable, reset_executable)
        for path in (
            config.output_root,
            config.profile_path,
            config.calibration_path,
            config.reset_executable,
        ):
            self.assertTrue(path.is_absolute())

        paths = EpochPaths(
            mission=config.output_root / "missions" / "epoch.jsonl",
            raw=config.output_root / "captures" / "epoch.bin",
            raw_index=config.output_root / "captures" / "epoch.index.jsonl",
            runtime_dir=config.output_root / "runtime" / "run",
            capture_stdout=config.output_root / "runtime" / "capture.out",
            capture_stderr=config.output_root / "runtime" / "capture.err",
            viewer_stdout=config.output_root / "runtime" / "viewer.out",
            viewer_stderr=config.output_root / "runtime" / "viewer.err",
        )
        capture_command = build_capture_command(
            RadarPortIdentity(
                "COM3",
                0x0451,
                0xBEF3,
                "RI32",
                "XDS110 Application/User UART",
                "usb-1",
            ),
            paths,
            config,
        )
        viewer_command = build_viewer_command(paths, config)
        self.assertEqual(
            Path(capture_command[capture_command.index("--output") + 1]),
            paths.mission,
        )
        self.assertEqual(
            Path(capture_command[capture_command.index("--raw-output") + 1]),
            paths.raw,
        )
        self.assertEqual(
            Path(
                viewer_command[
                    viewer_command.index("--clutter-calibration") + 1
                ]
            ),
            calibration,
        )

    def test_missing_profile_is_rejected_before_reset_or_hardware_dependencies(self):
        launcher = load_launcher()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = root / "calibration.json"
            calibration.write_text("{}", encoding="utf-8")
            missing_profile = root / "missing.cfg"
            with (
                mock.patch.object(
                    launcher,
                    "find_xds110_reset",
                    return_value=root / "xds110reset.exe",
                ) as find_reset,
                mock.patch.object(launcher, "RadarSupervisor") as supervisor,
                mock.patch.object(launcher, "comports") as comports,
                mock.patch.object(
                    launcher,
                    "reset_xds110_target",
                ) as reset_target,
            ):
                with self.assertRaisesRegex(SystemExit, "cfg|profile"):
                    launcher.main(
                        [
                            "--cfg",
                            str(missing_profile),
                            "--clutter-calibration",
                            str(calibration),
                        ]
                    )

        find_reset.assert_not_called()
        supervisor.assert_not_called()
        comports.assert_not_called()
        reset_target.assert_not_called()

    def test_signal_handlers_restore_in_reverse_after_supervisor_return_or_error(self):
        launcher = load_launcher()
        for supervisor_error in (None, RuntimeError("supervisor failed")):
            with self.subTest(supervisor_error=supervisor_error):
                originals = {
                    signal.SIGINT: object(),
                    signal.SIGTERM: object(),
                }
                signal_calls: list[tuple[int, object]] = []

                def set_signal(signum, handler):
                    signal_calls.append((signum, handler))

                class FakeSupervisor:
                    def __init__(self, config, dependencies):
                        del config, dependencies

                    def run(self, stop_requested):
                        del stop_requested
                        if supervisor_error is not None:
                            raise supervisor_error

                with tempfile.TemporaryDirectory() as directory:
                    calibration = Path(directory) / "calibration.json"
                    calibration.write_text("{}", encoding="utf-8")
                    with (
                        mock.patch.object(
                            launcher,
                            "find_xds110_reset",
                            side_effect=RuntimeError("unavailable"),
                        ),
                        mock.patch.object(
                            launcher,
                            "RadarSupervisor",
                            FakeSupervisor,
                        ),
                        mock.patch.object(
                            launcher.signal,
                            "getsignal",
                            side_effect=lambda signum: originals[signum],
                        ),
                        mock.patch.object(
                            launcher.signal,
                            "signal",
                            side_effect=set_signal,
                        ),
                    ):
                        if supervisor_error is None:
                            self.assertEqual(
                                launcher.main(
                                    [
                                        "--run-id",
                                        "signal-restore",
                                        "--clutter-calibration",
                                        str(calibration),
                                    ]
                                ),
                                0,
                            )
                        else:
                            with self.assertRaises(RuntimeError) as raised:
                                launcher.main(
                                    [
                                        "--run-id",
                                        "signal-restore",
                                        "--clutter-calibration",
                                        str(calibration),
                                    ]
                                )
                            self.assertIs(raised.exception, supervisor_error)

                self.assertEqual(
                    signal_calls[-2:],
                    [
                        (signal.SIGTERM, originals[signal.SIGTERM]),
                        (signal.SIGINT, originals[signal.SIGINT]),
                    ],
                )

    def test_partial_signal_installation_restores_first_and_preserves_error(self):
        launcher = load_launcher()
        original_sigint = object()
        installation_error = RuntimeError("SIGTERM installation failed")
        signal_calls: list[tuple[int, object]] = []

        def set_signal(signum, handler):
            signal_calls.append((signum, handler))
            if signum == signal.SIGTERM and callable(handler):
                raise installation_error

        with (
            mock.patch.object(
                launcher.signal,
                "getsignal",
                side_effect=lambda signum: {
                    signal.SIGINT: original_sigint,
                    signal.SIGTERM: object(),
                }[signum],
            ),
            mock.patch.object(
                launcher.signal,
                "signal",
                side_effect=set_signal,
            ),
        ):
            with self.assertRaises(RuntimeError) as raised:
                with launcher._shutdown_requested():
                    self.fail("context body must not run")

        self.assertIs(raised.exception, installation_error)
        self.assertEqual(
            signal_calls[-1],
            (signal.SIGINT, original_sigint),
        )

    def test_partial_install_and_restore_failures_preserve_install_error(self):
        launcher = load_launcher()
        original_sigint = object()
        installation_error = RuntimeError("INSTALL_ORIGINAL")
        restoration_error = RuntimeError("RESTORE_FAILURE")
        signal_calls: list[tuple[int, object]] = []

        def set_signal(signum, handler):
            signal_calls.append((signum, handler))
            if signum == signal.SIGTERM and callable(handler):
                raise installation_error
            if signum == signal.SIGINT and handler is original_sigint:
                raise restoration_error

        with (
            mock.patch.object(
                launcher.signal,
                "getsignal",
                side_effect=lambda signum: {
                    signal.SIGINT: original_sigint,
                    signal.SIGTERM: object(),
                }[signum],
            ),
            mock.patch.object(
                launcher.signal,
                "signal",
                side_effect=set_signal,
            ),
        ):
            with self.assertRaises(RuntimeError) as raised:
                with launcher._shutdown_requested():
                    self.fail("context body must not run")

        self.assertIs(raised.exception, installation_error)
        self.assertEqual(
            signal_calls[-1],
            (signal.SIGINT, original_sigint),
        )
        self.assertTrue(
            any(
                "RESTORE_FAILURE" in note
                for note in getattr(raised.exception, "__notes__", ())
            )
        )

    def test_body_failure_survives_restore_failure_and_all_restores_run(self):
        launcher = load_launcher()
        originals = {
            signal.SIGINT: object(),
            signal.SIGTERM: object(),
        }
        supervisor_error = RuntimeError("SUPERVISOR_ORIGINAL")
        restoration_error = RuntimeError("RESTORE_FAILURE")
        signal_calls: list[tuple[int, object]] = []

        def set_signal(signum, handler):
            signal_calls.append((signum, handler))
            if (
                signum == signal.SIGTERM
                and handler is originals[signal.SIGTERM]
            ):
                raise restoration_error

        with (
            mock.patch.object(
                launcher.signal,
                "getsignal",
                side_effect=lambda signum: originals[signum],
            ),
            mock.patch.object(
                launcher.signal,
                "signal",
                side_effect=set_signal,
            ),
        ):
            with self.assertRaises(RuntimeError) as raised:
                with launcher._shutdown_requested():
                    raise supervisor_error

        self.assertIs(raised.exception, supervisor_error)
        self.assertEqual(
            signal_calls[-2:],
            [
                (signal.SIGTERM, originals[signal.SIGTERM]),
                (signal.SIGINT, originals[signal.SIGINT]),
            ],
        )
        self.assertTrue(
            any(
                "RESTORE_FAILURE" in note
                for note in getattr(raised.exception, "__notes__", ())
            )
        )

    def test_normal_body_reports_restore_failure_after_all_restores_run(self):
        launcher = load_launcher()
        originals = {
            signal.SIGINT: object(),
            signal.SIGTERM: object(),
        }
        restoration_error = RuntimeError("RESTORE_FAILURE")
        signal_calls: list[tuple[int, object]] = []

        def set_signal(signum, handler):
            signal_calls.append((signum, handler))
            if (
                signum == signal.SIGTERM
                and handler is originals[signal.SIGTERM]
            ):
                raise restoration_error

        with (
            mock.patch.object(
                launcher.signal,
                "getsignal",
                side_effect=lambda signum: originals[signum],
            ),
            mock.patch.object(
                launcher.signal,
                "signal",
                side_effect=set_signal,
            ),
        ):
            with self.assertRaises(RuntimeError) as raised:
                with launcher._shutdown_requested():
                    pass

        self.assertIs(raised.exception, restoration_error)
        self.assertEqual(
            signal_calls[-2:],
            [
                (signal.SIGTERM, originals[signal.SIGTERM]),
                (signal.SIGINT, originals[signal.SIGINT]),
            ],
        )
        self.assertTrue(
            any(
                "signal handler restoration failed" in note
                for note in getattr(raised.exception, "__notes__", ())
            )
        )


if __name__ == "__main__":
    unittest.main()
