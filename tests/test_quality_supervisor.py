import unittest
from unittest import mock

from controller.quality_supervisor import (
    QualityConfig,
    QualitySupervisor,
    parse_args,
    score_quality,
)


class QualityScoringSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = QualityConfig(
            ping_ip=None,
            warn_hold_s=3.0,
            danger_hold_s=1.5,
        )
        self.now = 100.0

    def test_missing_video_is_danger_not_drivable_warn(self) -> None:
        status, reasons = score_quality(
            {},
            {},
            {},
            self.config,
            self.now,
        )
        self.assertEqual(status, "DANGER")
        self.assertIn("video sample missing", reasons)

    def test_incomplete_or_non_finite_video_evidence_is_danger(self) -> None:
        samples = [
            {"ts": self.now},
            {
                "ts": self.now,
                "fps_ratio": float("nan"),
            },
            {
                "ts": self.now,
                "fps_ratio": 1.0,
                "err_rate": float("inf"),
            },
        ]
        for sample in samples:
            with self.subTest(sample=sample):
                status, _ = score_quality(
                    sample,
                    {},
                    {},
                    self.config,
                    self.now,
                )
                self.assertEqual(status, "DANGER")

    def test_fresh_complete_video_sample_can_be_good(self) -> None:
        status, reasons = score_quality(
            {
                "ts": self.now,
                "target_fps": 15.0,
                "fps": 15.0,
                "fps_ratio": 1.0,
                "err_rate": 0.0,
                "drop_rate": 0.0,
            },
            {},
            {},
            self.config,
            self.now,
        )
        self.assertEqual(status, "GOOD")
        self.assertEqual(reasons, ["healthy"])

    def test_first_missing_sample_applies_zero_cap_during_hysteresis(self) -> None:
        supervisor = QualitySupervisor(self.config)
        with mock.patch(
            "controller.quality_supervisor.read_latest_video_sample",
            return_value={},
        ):
            decision = supervisor.update(now=self.now)
        self.assertEqual(decision.raw_status, "DANGER")
        self.assertEqual(decision.status, "TRANSIENT")
        self.assertEqual(decision.speed_cap, 0.0)

    def test_non_finite_cli_timing_values_are_rejected(self) -> None:
        for option in ("--target-fps", "--interval"):
            with self.subTest(option=option), self.assertRaises(SystemExit):
                parse_args([option, "nan"])


if __name__ == "__main__":
    unittest.main()
