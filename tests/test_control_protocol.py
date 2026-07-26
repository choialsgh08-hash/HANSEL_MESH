import unittest

from common.control_protocol import (
    PROTOCOL_VERSION,
    ProtocolValidator,
    build_command,
)


class ControlProtocolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.validator = ProtocolValidator(max_ttl_ms=5000)

    def message(
        self,
        seq: int,
        command: str = "forward",
        sent_monotonic_ns: int = 1_000_000_000,
        ttl_ms: int = 500,
    ):
        return build_command(
            session_id="test-session",
            seq=seq,
            target="head",
            command=command,
            ttl_ms=ttl_ms,
            sent_at=123.0,
            sent_monotonic_ns=sent_monotonic_ns,
        )

    def test_first_stop_establishes_monotonic_offset_baseline(self) -> None:
        result = self.validator.validate(
            self.message(1, "stop"),
            expected_target="head",
            source_id="192.168.50.2",
            now_monotonic_ns=9_000_000_000,
        )
        self.assertTrue(result.accepted)

    def test_first_motion_packet_is_rejected_until_stop_baseline(self) -> None:
        motion = self.validator.validate(
            self.message(1, "forward"),
            expected_target="head",
            source_id="192.168.50.2",
            now_monotonic_ns=9_000_000_000,
        )
        self.assertFalse(motion.accepted)
        self.assertEqual(motion.reason, "session_requires_initial_stop")

        stop = self.validator.validate(
            self.message(2, "stop"),
            expected_target="head",
            source_id="192.168.50.2",
            now_monotonic_ns=9_010_000_000,
        )
        self.assertTrue(stop.accepted)

    def test_excess_delay_expires_later_non_stop_packet(self) -> None:
        first = self.validator.validate(
            self.message(1, "stop", sent_monotonic_ns=1_000_000_000),
            expected_target="head",
            source_id="192.168.50.2",
            now_monotonic_ns=5_000_000_000,
        )
        delayed = self.validator.validate(
            self.message(2, sent_monotonic_ns=1_100_000_000),
            expected_target="head",
            source_id="192.168.50.2",
            now_monotonic_ns=5_900_000_000,
        )
        self.assertTrue(first.accepted)
        self.assertFalse(delayed.accepted)
        self.assertEqual(delayed.reason, "command_expired")

    def test_sequence_must_strictly_increase(self) -> None:
        self.validator.validate(
            self.message(1, "stop"),
            "head",
            "192.168.50.2",
            5_000_000_000,
        )
        self.validator.validate(
            self.message(2),
            "head",
            "192.168.50.2",
            5_001_000_000,
        )
        duplicate = self.validator.validate(
            self.message(2),
            "head",
            "192.168.50.2",
            5_010_000_000,
        )
        self.assertFalse(duplicate.accepted)
        self.assertEqual(
            duplicate.reason,
            "seq_not_strictly_increasing",
        )

    def test_duplicate_stop_is_always_applied_for_safety(self) -> None:
        self.validator.validate(
            self.message(1, "stop"),
            "head",
            "192.168.50.2",
            5_000_000_000,
        )
        self.validator.validate(
            self.message(3, "forward"),
            "head",
            "192.168.50.2",
            5_010_000_000,
        )
        duplicate_stop = self.validator.validate(
            self.message(3, "stop"),
            "head",
            "192.168.50.2",
            6_000_000_000,
        )
        self.assertTrue(duplicate_stop.accepted)
        self.assertIn("safety_stop", duplicate_stop.reason)

    def test_target_and_version_are_validated_even_for_stop(self) -> None:
        wrong_target = self.message(1, "stop")
        wrong_target["target"] = "node1"
        result = self.validator.validate(
            wrong_target,
            "head",
            "192.168.50.2",
            5_000_000_000,
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "target_mismatch")

        wrong_version = self.message(1, "stop")
        wrong_version["protocol_version"] = PROTOCOL_VERSION + 1
        result = self.validator.validate(
            wrong_version,
            "head",
            "192.168.50.2",
            5_000_000_000,
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "unsupported_protocol_version")

    def test_extra_cannot_override_protocol_identity(self) -> None:
        with self.assertRaises(ValueError):
            build_command(
                session_id="session",
                seq=1,
                target="head",
                command="stop",
                extra={"target": "node1"},
            )

    def test_speed_must_be_finite_and_within_unit_interval(self) -> None:
        self.validator.validate(
            self.message(1, "stop"),
            "head",
            "192.168.50.2",
            5_000_000_000,
        )
        invalid_values = [
            -0.01,
            1.01,
            float("nan"),
            float("inf"),
            float("-inf"),
            True,
            "0.5",
        ]
        for index, speed in enumerate(invalid_values, start=2):
            with self.subTest(speed=speed):
                message = self.message(index, "forward")
                message["speed"] = speed
                result = self.validator.validate(
                    message,
                    "head",
                    "192.168.50.2",
                    5_000_000_000 + index * 1_000_000,
                )
                self.assertFalse(result.accepted)
                self.assertEqual(result.reason, "invalid_speed")

        valid = self.message(20, "forward")
        valid["speed"] = 0.5
        accepted = self.validator.validate(
            valid,
            "head",
            "192.168.50.2",
            5_020_000_000,
        )
        self.assertTrue(accepted.accepted)


if __name__ == "__main__":
    unittest.main()
