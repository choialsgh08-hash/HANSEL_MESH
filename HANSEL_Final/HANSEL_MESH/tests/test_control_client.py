import json
import socket
import threading
import unittest
from unittest import mock

from common.control_protocol import build_ack
from controller import mesh_control_client as client


class FakeTransport:
    def __init__(self, outcomes=None) -> None:
        self.actions = []
        self.messages = []
        self.outcomes = dict(outcomes or {})
        self.seq = 0
        self.cancelled = []

    def send_one(
        self,
        target,
        command,
        speed=None,
        source="operator",
        extra=None,
        expect_ack=False,
    ):
        self.seq += 1
        ref = client.CommandRef(
            session_id="fake",
            seq=self.seq,
            target=target,
            command=command,
            expected_ip="127.0.0.1",
            expected_port=7000,
        )
        self.actions.append((target, command, expect_ack))
        self.messages.append(
            {
                "target": target,
                "command": command,
                "speed": speed,
                "extra": extra,
            }
        )
        return ref

    def wait_applied(self, ref, timeout):
        return self.outcomes.get(
            (ref.target, ref.command),
            (True, "applied"),
        )

    def cancel_wait(self, ref):
        self.cancelled.append(ref)


class DetachSequenceTests(unittest.TestCase):
    def test_detach_orders_stop_hold_then_actuator(self) -> None:
        transport = FakeTransport()
        result = client.send_detach_release(
            transport,
            "node1",
            ["head", "node1"],
            0.2,
        )
        self.assertTrue(result.command_completed)
        self.assertFalse(result.faulted)
        self.assertEqual(
            transport.actions,
            [
                ("head", "stop", True),
                ("node1", "stop", True),
                ("node1", "relay_hold", True),
                ("head", "detach_press", True),
            ],
        )
        detach_message = transport.messages[-1]
        self.assertEqual(
            detach_message["extra"]["detach_context"][
                "actuator_stop_seq"
            ],
            1,
        )

    def test_relay_hold_failure_prevents_actuator_and_faults(self) -> None:
        transport = FakeTransport(
            {("node1", "relay_hold"): (False, "ack_timeout")}
        )
        result = client.send_detach_release(
            transport,
            "node1",
            ["head", "node1"],
            0.2,
        )
        self.assertFalse(result.command_completed)
        self.assertTrue(result.faulted)
        self.assertNotIn(
            ("head", "detach_press", True),
            transport.actions,
        )

    def test_actuator_failure_faults_and_does_not_confirm_removal(self) -> None:
        transport = FakeTransport(
            {("head", "detach_press"): (False, "ack_timeout")}
        )
        active = ["head", "node1"]
        result = client.send_detach_release(
            transport,
            "node1",
            active,
            0.2,
        )
        remove, terminate = client.resolve_detach_result(
            result,
            "node1",
            assume_on_ack=False,
            live_terminal=False,
        )
        self.assertFalse(remove)
        self.assertTrue(terminate)
        self.assertEqual(active, ["head", "node1"])

    def test_actuator_ack_needs_physical_confirmation_by_default(self) -> None:
        result = client.DetachResult(
            True,
            False,
            "actuator_acknowledged",
        )
        with mock.patch("builtins.input", return_value="not confirmed"):
            remove, terminate = client.resolve_detach_result(
                result,
                "node1",
                assume_on_ack=False,
                live_terminal=False,
            )
        self.assertFalse(remove)
        self.assertTrue(terminate)

    def test_explicit_assumption_allows_removal(self) -> None:
        result = client.DetachResult(
            True,
            False,
            "actuator_acknowledged",
        )
        remove, terminate = client.resolve_detach_result(
            result,
            "node1",
            assume_on_ack=True,
            live_terminal=False,
        )
        self.assertTrue(remove)
        self.assertFalse(terminate)

    def test_inactive_optional_node_cannot_be_detached(self) -> None:
        transport = FakeTransport()
        result = client.send_detach_release(
            transport,
            "node3",
            ["head", "node1", "node2"],
            0.2,
        )
        self.assertFalse(result.command_completed)
        self.assertFalse(result.faulted)
        self.assertEqual(result.reason, "released_node_not_active")
        self.assertEqual(transport.actions, [])


class SessionInitializationTests(unittest.TestCase):
    def test_all_target_session_starts_with_acknowledged_stop(self) -> None:
        transport = FakeTransport()
        initialized = client.initialize_control_session(
            transport,
            "all",
            ["head", "node1"],
            0.2,
        )
        self.assertTrue(initialized)
        self.assertEqual(
            transport.actions,
            [
                ("head", "stop", True),
                ("node1", "stop", True),
            ],
        )

    def test_session_does_not_start_when_stop_ack_is_missing(self) -> None:
        transport = FakeTransport(
            {("node1", "stop"): (False, "ack_timeout")}
        )
        initialized = client.initialize_control_session(
            transport,
            "all",
            ["head", "node1"],
            0.2,
        )
        self.assertFalse(initialized)

    def test_single_target_session_only_requires_selected_target(self) -> None:
        transport = FakeTransport()
        initialized = client.initialize_control_session(
            transport,
            "head",
            ["head", "node1"],
            0.2,
        )
        self.assertTrue(initialized)
        self.assertEqual(
            transport.actions,
            [("head", "stop", True)],
        )

    def test_routing_never_reactivates_removed_single_target(self) -> None:
        items = client.target_items(
            "node1",
            "drive_enable",
            ["head", "node2"],
        )
        self.assertEqual(items, [])


class AckReceiverTests(unittest.TestCase):
    def test_background_receiver_matches_applied_ack(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.bind(("127.0.0.1", 0))
        server.settimeout(1.0)
        port = server.getsockname()[1]
        old_ip = client.TARGETS["head"]
        client.TARGETS["head"] = "127.0.0.1"

        def serve_once():
            data, peer = server.recvfrom(4096)
            request = json.loads(data.decode("utf-8"))
            ack = build_ack(
                request,
                "head",
                "applied",
                "applied",
            )
            server.sendto(json.dumps(ack).encode("utf-8"), peer)

        thread = threading.Thread(target=serve_once, daemon=True)
        thread.start()
        transport = client.ControlTransport(
            port=port,
            session_id="ack-test",
        )
        try:
            ref = transport.send_one(
                "head",
                "stop",
                expect_ack=True,
            )
            applied, reason = transport.wait_applied(ref, 1.0)
            self.assertTrue(applied)
            self.assertEqual(reason, "applied")
        finally:
            transport.close()
            client.TARGETS["head"] = old_ip
            server.close()
            thread.join(timeout=1.0)


class CameraProfileSyncTests(unittest.TestCase):
    def test_unknown_initial_profile_sends_first_decision(self) -> None:
        transport = FakeTransport()
        sync = client.CameraProfileSync()

        sync.update(
            transport,
            desired_profile=0,
            dest_ip="192.168.60.2",
            dest_port=5600,
            camera_transport="rtp",
            ack_timeout=0.5,
            now=10.0,
        )

        self.assertEqual(
            transport.actions,
            [("head", "camera_profile", True)],
        )
        self.assertIsNone(sync.applied_profile)

    def test_profile_request_requires_ack_before_marking_applied(self) -> None:
        transport = FakeTransport()
        sync = client.CameraProfileSync(initial_profile=0)

        sync.update(
            transport,
            desired_profile=2,
            dest_ip="192.168.60.2",
            dest_port=5600,
            camera_transport="rtp",
            ack_timeout=0.5,
            now=10.0,
        )
        self.assertEqual(
            transport.actions,
            [("head", "camera_profile", True)],
        )
        self.assertEqual(sync.applied_profile, 0)
        self.assertIsNotNone(sync.pending)

        sync.pending.ref.event.set()
        sync.update(
            transport,
            desired_profile=2,
            dest_ip="192.168.60.2",
            dest_port=5600,
            camera_transport="rtp",
            ack_timeout=0.5,
            now=10.1,
        )
        self.assertEqual(sync.applied_profile, 2)
        self.assertIsNone(sync.pending)

    def test_rejected_profile_is_retried_without_blocking_drive_loop(self) -> None:
        transport = FakeTransport(
            {("head", "camera_profile"): (False, "camera_service_inactive")}
        )
        sync = client.CameraProfileSync(
            initial_profile=0,
            retry_delay_s=1.0,
        )
        update_args = {
            "transport": transport,
            "desired_profile": 1,
            "dest_ip": "192.168.60.2",
            "dest_port": 5600,
            "camera_transport": "rtp",
            "ack_timeout": 0.5,
        }

        sync.update(now=20.0, **update_args)
        sync.pending.ref.event.set()
        sync.update(now=20.1, **update_args)
        self.assertEqual(len(transport.actions), 1)
        self.assertEqual(sync.applied_profile, 0)

        sync.update(now=21.2, **update_args)
        self.assertEqual(len(transport.actions), 2)
        self.assertIsNotNone(sync.pending)

    def test_changed_desired_profile_cancels_obsolete_waiter(self) -> None:
        transport = FakeTransport()
        sync = client.CameraProfileSync(initial_profile=0)
        common = {
            "transport": transport,
            "dest_ip": "192.168.60.2",
            "dest_port": 5600,
            "camera_transport": "rtp",
            "ack_timeout": 0.5,
        }
        sync.update(desired_profile=1, now=30.0, **common)
        first_ref = sync.pending.ref
        sync.update(desired_profile=2, now=30.1, **common)

        self.assertIn(first_ref, transport.cancelled)
        self.assertEqual(len(transport.actions), 2)
        self.assertEqual(sync.pending.profile, 2)


class ClientArgumentTests(unittest.TestCase):
    def test_default_active_chain_excludes_optional_node3(self) -> None:
        args = client.parse_args([])
        self.assertEqual(
            args.active_targets,
            ["head", "node1", "node2"],
        )

    def test_optional_node3_can_be_added_explicitly(self) -> None:
        args = client.parse_args(
            [
                "--active-targets",
                "head,node1,node2,node3",
            ]
        )
        self.assertEqual(
            args.active_targets,
            ["head", "node1", "node2", "node3"],
        )

    def test_unknown_active_target_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            client.parse_args(["--active-targets", "head,node9"])

    def test_invalid_speed_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            client.parse_args(["--speed", "1.1"])

    def test_invalid_ttl_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            client.parse_args(["--command-ttl-ms", "0"])

    def test_non_finite_safety_floats_are_rejected(self) -> None:
        options = (
            "--speed",
            "--repeat-delay",
            "--send-interval",
            "--ack-timeout",
            "--quality-target-fps",
            "--quality-interval",
            "--quality-warn-speed",
            "--detach-cooldown",
        )
        for option in options:
            with self.subTest(option=option), self.assertRaises(SystemExit):
                client.parse_args([option, "nan"])

    def test_invalid_ack_timeout_is_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            client.parse_args(["--ack-timeout", "0"])

    def test_windows_live_mode_fails_cleanly(self) -> None:
        if client.termios is not None:
            self.skipTest("This check is only applicable on Windows")
        args = client.parse_args(["--live"])
        self.assertEqual(client.run_live_mode(args), 2)


class QualityFailClosedTests(unittest.TestCase):
    def test_non_ready_error_and_stale_states_block_motion(self) -> None:
        for status in ("UNKNOWN", "NOT_READY", "STALE", "ERROR", "DANGER"):
            with self.subTest(status=status):
                decision = mock.Mock(status=status, speed_cap=None)
                self.assertTrue(client.quality_blocks_motion(decision))

    def test_good_state_without_zero_cap_allows_motion(self) -> None:
        decision = mock.Mock(status="GOOD", speed_cap=None)
        self.assertFalse(client.quality_blocks_motion(decision))

    def test_zero_speed_cap_blocks_motion_for_any_status(self) -> None:
        decision = mock.Mock(status="WARN", speed_cap=0.0)
        self.assertTrue(client.quality_blocks_motion(decision))

    def test_non_finite_quality_cap_fails_closed(self) -> None:
        decision = mock.Mock(status="WARN", speed_cap=float("nan"))
        self.assertTrue(client.quality_blocks_motion(decision))
        self.assertEqual(
            client.effective_speed(1.0, float("nan")),
            0.0,
        )

    def test_front_motor_one_shot_is_blocked_when_quality_is_not_ready(self) -> None:
        decision = mock.Mock(status="NOT_READY", speed_cap=0.0)
        self.assertTrue(
            client.quality_blocks_command(
                decision,
                "front_motor_forward",
            )
        )
        self.assertFalse(
            client.quality_blocks_command(
                decision,
                "front_motor_stop",
            )
        )

    def test_auto_detach_needs_prior_usable_video_sample(self) -> None:
        missing = mock.Mock(
            raw_status="DANGER",
            video={},
        )
        degraded_at_start = mock.Mock(
            raw_status="DANGER",
            video={"fps_ratio": 0.1},
        )
        usable = mock.Mock(
            raw_status="WARN",
            video={"fps_ratio": 0.8},
        )
        self.assertFalse(
            client.quality_sample_arms_auto_detach(missing)
        )
        self.assertFalse(
            client.quality_sample_arms_auto_detach(degraded_at_start)
        )
        self.assertTrue(
            client.quality_sample_arms_auto_detach(usable)
        )

    def test_auto_detach_actuation_requires_exact_preflight_phrase(self) -> None:
        with mock.patch(
            "builtins.input",
            return_value=client.AUTO_DETACH_ARM_PHRASE,
        ):
            self.assertTrue(client.confirm_auto_detach_actuation())
        with mock.patch("builtins.input", return_value="yes"):
            self.assertFalse(client.confirm_auto_detach_actuation())


if __name__ == "__main__":
    unittest.main()
