import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from sensors.radar_stack_processes import (
    ManagedChild,
    RadarStackProcesses,
    build_capture_command,
    build_viewer_command,
)
from sensors.radar_supervisor import EpochPaths, RadarSupervisorConfig
from sensors.ti_radar_control import RadarPortIdentity


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
                str(self.config.calibration_path), "--bind", "127.0.0.1", "--http-port", "8081",
                "--max-range-m", "3", "--history-window", "0.3", "--quiet",
            ],
        )

    def test_switch_viewer_stops_registered_current_before_starting_replacement(self):
        events = []
        old = FakeProcess(pid=10, waits=(0,))
        new = FakeProcess(pid=11)

        def start(command, **kwargs):
            if not events:
                events.append("start:old-viewer")
                return old
            events.append("start:new-viewer")
            return new

        manager = RadarStackProcesses(popen_factory=start)
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"), mock.patch("sensors.radar_stack_processes.os.killpg", side_effect=lambda *args: events.append("stop:old-viewer"), create=True), mock.patch(
            "sensors.radar_stack_processes.os.getpgid", return_value=77, create=True
        ):
            current = manager.switch_viewer(None, self.paths, self.config)
            replacement = manager.switch_viewer(current, self.paths, self.config)

        self.assertEqual(events, ["start:old-viewer", "stop:old-viewer", "start:new-viewer"])
        self.assertEqual(replacement.pid, 11)

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
                    subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW,
                )
            manager.stop_owned_children()

    def test_start_capture_uses_posix_session_when_selected(self):
        process = FakeProcess(pid=82, poll_result=0)
        started = []
        manager = RadarStackProcesses(
            popen_factory=lambda command, **kwargs: started.append(kwargs) or process
        )
        with mock.patch("sensors.radar_stack_processes.os.name", "posix"):
            manager.start_capture(self.port, self.paths, self.config)

        self.assertTrue(started[0]["start_new_session"])

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
        duplicate = FakeProcess(pid=31, waits=(0,))
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

        self.assertEqual(duplicate.events, [("send_signal", 41), ("wait", 2.0)])
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


if __name__ == "__main__":
    unittest.main()
