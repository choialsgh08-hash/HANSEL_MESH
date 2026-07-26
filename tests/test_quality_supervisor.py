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


class LinkHealthScoringTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = QualityConfig(ping_ip=None)
        self.now = 100.0
        # A healthy video sample so the base status is GOOD before link scoring.
        self.good_video = {
            "ts": self.now,
            "fps_ratio": 1.0,
            "err_rate": 0.0,
            "drop_rate": 0.0,
        }

    def _score(self, link):
        return score_quality(
            self.good_video, {}, {}, self.config, self.now, link=link
        )

    def test_link_absent_keeps_good(self) -> None:
        status, _ = self._score(None)
        self.assertEqual(status, "GOOD")

    def test_healthy_link_keeps_good(self) -> None:
        status, _ = self._score(
            {"signal_worst_dbm": -55, "inactive_worst_ms": 100}
        )
        self.assertEqual(status, "GOOD")

    def test_weak_signal_warns(self) -> None:
        status, reasons = self._score({"signal_worst_dbm": -78})
        self.assertEqual(status, "WARN")
        self.assertTrue(any("radio signal" in r for r in reasons))

    def test_very_weak_signal_is_danger(self) -> None:
        status, _ = self._score({"signal_worst_dbm": -85})
        self.assertEqual(status, "DANGER")

    def test_long_peer_silence_warns_then_dangers(self) -> None:
        warn_status, _ = self._score({"inactive_worst_ms": 2000})
        danger_status, _ = self._score({"inactive_worst_ms": 3500})
        self.assertEqual(warn_status, "WARN")
        self.assertEqual(danger_status, "DANGER")


if __name__ == "__main__":
    unittest.main()
