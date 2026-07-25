import ipaddress
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from common.control_protocol import build_command
from robot.mesh_control_server import (
    ControlServerCore,
    command_starts_motion,
    command_stops_all_motion,
    parse_args,
    request_managed_camera_profile,
    run,
)
from robot.motor_driver import DryRunRobotController


class ControlServerCoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = DryRunRobotController("head")
        self.controller.start()
        self.core = ControlServerCore(
            controller=self.controller,
            role="head",
        )
        self.peer = ("192.168.50.2", 42000)
        self.sender_clock = 1_000_000_000
        self.receiver_clock = 5_000_000_000
        self.process(1, "stop")

    def packet(
        self,
        seq: int,
        command: str,
        target: str = "head",
    ) -> bytes:
        message = build_command(
            session_id="server-test",
            seq=seq,
            target=target,
            command=command,
            ttl_ms=750,
            sent_monotonic_ns=self.sender_clock + seq * 1_000_000,
        )
        return json.dumps(message).encode("utf-8")

    def process(self, seq: int, command: str, target: str = "head"):
        return self.core.process_datagram(
            self.packet(seq, command, target),
            self.peer,
            now_monotonic_ns=self.receiver_clock + seq * 1_000_000,
        )

    def test_valid_command_is_applied_and_acknowledged(self) -> None:
        result = self.process(2, "forward")
        self.assertTrue(result.applied)
        self.assertEqual(result.ack["status"], "applied")
        self.assertEqual(result.ack["seq"], 2)
        self.assertEqual(self.controller.last_command, "forward")

    def test_duplicate_non_stop_is_rejected(self) -> None:
        self.process(2, "forward")
        result = self.process(2, "backward")
        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "seq_not_strictly_increasing",
        )

    def test_duplicate_stop_is_applied(self) -> None:
        self.process(2, "forward")
        result = self.process(2, "stop")
        self.assertTrue(result.applied)
        self.assertEqual(result.ack["status"], "applied")
        self.assertIn("safety_stop", result.ack["reason"])
        self.assertEqual(self.controller.last_command, "stop")

    def test_target_mismatch_is_rejected(self) -> None:
        result = self.process(1, "forward", target="node1")
        self.assertFalse(result.applied)
        self.assertEqual(result.ack["reason"], "target_mismatch")
        self.assertEqual(self.controller.last_command, "stop")

    def test_relay_hold_latches_drive_until_enable(self) -> None:
        hold = self.process(2, "relay_hold")
        blocked = self.process(3, "forward")
        enabled = self.process(4, "drive_enable")
        moving = self.process(5, "forward")

        self.assertTrue(hold.applied)
        self.assertFalse(blocked.applied)
        self.assertEqual(blocked.ack["reason"], "drive_disabled")
        self.assertTrue(enabled.applied)
        self.assertTrue(moving.applied)

    def test_raw_detach_is_rejected_without_safety_context(self) -> None:
        result = self.process(2, "detach_press")
        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "detach_safety_precondition_missing",
        )
        self.assertEqual(self.controller.last_command, "stop")

    def test_default_camera_profile_uses_managed_service_path(self) -> None:
        with mock.patch(
            "robot.mesh_control_server.request_managed_camera_profile",
            return_value=(True, "camera_profile_restart_queued"),
        ) as request:
            result = self.process(2, "camera_profile")

        self.assertTrue(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "camera_profile_restart_queued",
        )
        request.assert_called_once_with("0")

    def test_inactive_managed_camera_rejects_profile_change(self) -> None:
        with mock.patch(
            "robot.mesh_control_server.request_managed_camera_profile",
            return_value=(False, "camera_service_inactive"),
        ):
            result = self.process(2, "camera_profile")

        self.assertFalse(result.applied)
        self.assertEqual(result.ack["reason"], "camera_service_inactive")

    def test_detach_context_must_match_local_actuator_mapping(self) -> None:
        message = build_command(
            session_id="server-test",
            seq=2,
            target="head",
            command="detach_press",
            ttl_ms=750,
            sent_monotonic_ns=self.sender_clock + 2_000_000,
            extra={
                "detach_context": {
                    "released_node": "node2",
                    "stop_and_hold_acknowledged": True,
                    "actuator_stop_seq": 1,
                }
            },
        )
        result = self.core.process_datagram(
            json.dumps(message).encode("utf-8"),
            self.peer,
            now_monotonic_ns=self.receiver_clock + 2_000_000,
        )
        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "detach_safety_precondition_missing",
        )

    def test_safe_detach_context_is_accepted_by_mapped_actuator(self) -> None:
        message = build_command(
            session_id="server-test",
            seq=2,
            target="head",
            command="detach_press",
            ttl_ms=750,
            sent_monotonic_ns=self.sender_clock + 2_000_000,
            extra={
                "detach_context": {
                    "released_node": "node1",
                    "stop_and_hold_acknowledged": True,
                    "actuator_stop_seq": 1,
                }
            },
        )
        result = self.core.process_datagram(
            json.dumps(message).encode("utf-8"),
            self.peer,
            now_monotonic_ns=self.receiver_clock + 2_000_000,
        )
        self.assertTrue(result.applied)

        replay = build_command(
            session_id="server-test",
            seq=3,
            target="head",
            command="detach_press",
            ttl_ms=750,
            sent_monotonic_ns=self.sender_clock + 3_000_000,
            extra={
                "detach_context": {
                    "released_node": "node1",
                    "stop_and_hold_acknowledged": True,
                    "actuator_stop_seq": 1,
                }
            },
        )
        replay_result = self.core.process_datagram(
            json.dumps(replay).encode("utf-8"),
            self.peer,
            now_monotonic_ns=self.receiver_clock + 3_000_000,
        )
        self.assertFalse(replay_result.applied)
        self.assertEqual(
            replay_result.ack["reason"],
            "detach_safety_precondition_missing",
        )

    def test_plaintext_is_disabled_by_default(self) -> None:
        result = self.core.process_datagram(
            b"forward",
            self.peer,
            now_monotonic_ns=self.receiver_clock,
        )
        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "legacy_plaintext_disabled",
        )

    def test_nonstandard_json_nan_is_rejected_before_motor_apply(self) -> None:
        raw = self.packet(2, "forward").replace(
            b'"ttl_ms": 750',
            b'"speed": NaN, "ttl_ms": 750',
        )
        result = self.core.process_datagram(
            raw,
            self.peer,
            now_monotonic_ns=self.receiver_clock + 2_000_000,
        )
        self.assertFalse(result.applied)
        self.assertEqual(result.ack["reason"], "invalid_json")
        self.assertEqual(self.controller.last_command, "stop")

    def test_plaintext_requires_explicit_opt_in(self) -> None:
        core = ControlServerCore(
            controller=self.controller,
            role="head",
            allow_legacy_plaintext=True,
        )
        initial = core.process_datagram(
            b"stop",
            self.peer,
            now_monotonic_ns=self.receiver_clock,
        )
        result = core.process_datagram(
            b"forward",
            self.peer,
            now_monotonic_ns=self.receiver_clock + 1_000_000,
        )
        self.assertTrue(initial.applied)
        self.assertTrue(result.applied)

    def test_new_session_rejects_motion_before_initial_stop(self) -> None:
        fresh_core = ControlServerCore(
            controller=self.controller,
            role="head",
        )
        result = fresh_core.process_datagram(
            self.packet(2, "forward"),
            self.peer,
            now_monotonic_ns=self.receiver_clock,
        )
        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "session_requires_initial_stop",
        )

    def test_source_allowlist_rejects_other_network(self) -> None:
        core = ControlServerCore(
            controller=self.controller,
            role="head",
            allowed_sources=[ipaddress.ip_network("192.168.60.0/24")],
        )
        result = core.process_datagram(
            self.packet(1, "forward"),
            self.peer,
            now_monotonic_ns=self.receiver_clock,
        )
        self.assertFalse(result.applied)
        self.assertEqual(result.ack["reason"], "source_not_allowed")
        self.assertEqual(result.ack["session_id"], "server-test")

    def test_only_real_motion_commands_start_watchdog_state(self) -> None:
        self.assertTrue(command_starts_motion("head", "forward"))
        self.assertFalse(command_starts_motion("head", "camera_profile"))
        self.assertFalse(command_starts_motion("head", "detach_press"))
        self.assertFalse(command_starts_motion("node1", "left"))
        self.assertTrue(command_starts_motion("node1", "forward_left"))

    def test_front_stop_does_not_disarm_main_drive_watchdog(self) -> None:
        self.assertTrue(command_stops_all_motion("stop"))
        self.assertTrue(command_stops_all_motion("relay_hold"))
        self.assertFalse(command_stops_all_motion("front_motor_stop"))
        self.assertFalse(command_stops_all_motion("front_stop"))


class ServerArgumentTests(unittest.TestCase):
    def test_invalid_max_ttl_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            parse_args(["--role", "head", "--max-ttl-ms", "0"])

    def test_non_finite_watchdog_timeout_is_rejected(self) -> None:
        for value in ("nan", "inf", "-inf"):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                parse_args(["--role", "head", "--timeout", value])

    def test_invalid_allow_source_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            parse_args(
                ["--role", "head", "--allow-source", "not-a-network"]
            )

    def test_required_allowlist_fails_closed_before_controller_start(self) -> None:
        args = parse_args(
            ["--role", "head", "--dry-run", "--require-source-allowlist"]
        )
        with mock.patch.dict(
            os.environ,
            {"HANSEL_CONTROL_ALLOW_SOURCES": ""},
            clear=False,
        ):
            self.assertEqual(run(args), 2)

    def test_production_mode_always_requires_source_allowlist(self) -> None:
        args = parse_args(
            [
                "--role",
                "head",
                "--unsafe-no-drive-state-persistence",
            ]
        )
        with mock.patch.dict(
            os.environ,
            {"HANSEL_CONTROL_ALLOW_SOURCES": ""},
            clear=False,
        ), mock.patch(
            "robot.mesh_control_server.build_robot_controller"
        ) as build:
            self.assertEqual(run(args), 2)
        build.assert_not_called()

    def test_controller_is_cleaned_up_when_start_fails(self) -> None:
        controller = mock.Mock()
        controller.start.side_effect = RuntimeError("GPIO setup failed")
        args = parse_args(
            [
                "--role",
                "head",
                "--allow-source",
                "127.0.0.1/32",
                "--unsafe-no-drive-state-persistence",
            ]
        )
        with mock.patch(
            "robot.mesh_control_server.build_robot_controller",
            return_value=controller,
        ):
            self.assertEqual(run(args), 2)
        controller.stop.assert_called_once_with()

    def test_controller_is_cleaned_up_when_udp_bind_fails(self) -> None:
        controller = mock.Mock()
        fake_socket = mock.Mock()
        fake_socket.bind.side_effect = OSError("address in use")
        args = parse_args(["--role", "head", "--dry-run"])
        with mock.patch(
            "robot.mesh_control_server.build_robot_controller",
            return_value=controller,
        ):
            with mock.patch(
                "robot.mesh_control_server.socket.socket",
                return_value=fake_socket,
            ):
                self.assertEqual(run(args), 2)
        fake_socket.close.assert_called_once_with()
        controller.stop.assert_called_once_with()


class ManagedCameraProfileTests(unittest.TestCase):
    def test_invalid_profile_never_reaches_systemd_or_disk(self) -> None:
        with mock.patch(
            "robot.mesh_control_server.subprocess.run",
        ) as run_command:
            result = request_managed_camera_profile("../../bad")

        self.assertEqual(result, (False, "invalid_camera_profile"))
        run_command.assert_not_called()

    def test_profile_override_is_atomic_and_restart_is_nonblocking(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = Path(temp_dir) / "camera-profile"
            active = mock.Mock(returncode=0)
            queued = mock.Mock(returncode=0)
            with mock.patch.dict(
                os.environ,
                {
                    "HANSEL_CAMERA_PROFILE_FILE": str(profile_path),
                    "HANSEL_CAMERA_SYSTEMD_UNIT": "hansel-camera.service",
                },
                clear=False,
            ), mock.patch(
                "robot.mesh_control_server.subprocess.run",
                side_effect=[active, queued],
            ) as run_command:
                result = request_managed_camera_profile("low")

            self.assertEqual(
                result,
                (True, "camera_profile_restart_queued"),
            )
            self.assertEqual(profile_path.read_text(encoding="ascii"), "low\n")
            self.assertEqual(
                run_command.call_args_list[0].args[0],
                [
                    "systemctl",
                    "is-active",
                    "--quiet",
                    "hansel-camera.service",
                ],
            )
            self.assertEqual(
                run_command.call_args_list[1].args[0],
                [
                    "systemctl",
                    "--no-block",
                    "try-restart",
                    "hansel-camera.service",
                ],
            )

    def test_inactive_service_does_not_write_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = Path(temp_dir) / "camera-profile"
            with mock.patch.dict(
                os.environ,
                {"HANSEL_CAMERA_PROFILE_FILE": str(profile_path)},
                clear=False,
            ), mock.patch(
                "robot.mesh_control_server.subprocess.run",
                return_value=mock.Mock(returncode=3),
            ) as run_command:
                result = request_managed_camera_profile("medium")

            self.assertEqual(result, (False, "camera_service_inactive"))
            self.assertFalse(profile_path.exists())
            run_command.assert_called_once()

    def test_failed_restart_restores_previous_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = Path(temp_dir) / "camera-profile"
            profile_path.write_text("high\n", encoding="ascii")
            with mock.patch.dict(
                os.environ,
                {"HANSEL_CAMERA_PROFILE_FILE": str(profile_path)},
                clear=False,
            ), mock.patch(
                "robot.mesh_control_server.subprocess.run",
                side_effect=[
                    mock.Mock(returncode=0),
                    mock.Mock(returncode=1),
                ],
            ):
                result = request_managed_camera_profile("low")

            self.assertEqual(
                result,
                (False, "camera_service_restart_failed"),
            )
            self.assertEqual(
                profile_path.read_text(encoding="ascii"),
                "high\n",
            )

    def test_failed_restart_removes_new_override(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = Path(temp_dir) / "camera-profile"
            with mock.patch.dict(
                os.environ,
                {"HANSEL_CAMERA_PROFILE_FILE": str(profile_path)},
                clear=False,
            ), mock.patch(
                "robot.mesh_control_server.subprocess.run",
                side_effect=[
                    mock.Mock(returncode=0),
                    OSError("systemctl unavailable"),
                ],
            ):
                result = request_managed_camera_profile("medium")

            self.assertEqual(
                result,
                (False, "camera_service_unavailable"),
            )
            self.assertFalse(profile_path.exists())


if __name__ == "__main__":
    unittest.main()
