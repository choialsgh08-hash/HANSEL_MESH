import unittest
from unittest import mock

import os
import tempfile

from controller.quality_supervisor import (
    QualityConfig,
    QualitySupervisor,
    config_with_overrides,
    load_quality_overrides,
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


class QualityOverrideTests(unittest.TestCase):
    def test_environ_overrides_are_typed(self) -> None:
        overrides = load_quality_overrides(
            None, {"SIGNAL_WARN_DBM": "-70", "TQ_WARN": "210"}
        )
        self.assertEqual(overrides["signal_warn_dbm"], -70.0)
        self.assertEqual(overrides["tq_warn"], 210)
        self.assertIsInstance(overrides["tq_warn"], int)

    def test_unknown_and_unparseable_keys_ignored(self) -> None:
        overrides = load_quality_overrides(
            None, {"NOT_A_KEY": "5", "SIGNAL_WARN_DBM": "abc"}
        )
        self.assertEqual(overrides, {})

    def test_environ_wins_over_file(self) -> None:
        with tempfile.NamedTemporaryFile(
            "w", suffix=".env", delete=False, encoding="utf-8"
        ) as fh:
            fh.write("# comment\nSIGNAL_WARN_DBM=-75\nRTT_WARN_MS=100\n")
            path = fh.name
        try:
            overrides = load_quality_overrides(path, {"SIGNAL_WARN_DBM": "-70"})
        finally:
            os.unlink(path)
        self.assertEqual(overrides["signal_warn_dbm"], -70.0)  # env wins
        self.assertEqual(overrides["rtt_warn_ms"], 100.0)      # from file

    def test_config_with_overrides_applies_fields(self) -> None:
        cfg = config_with_overrides(
            QualityConfig(), None, {"SIGNAL_DANGER_DBM": "-80"}
        )
        self.assertEqual(cfg.signal_danger_dbm, -80.0)

    def test_no_sources_keeps_defaults(self) -> None:
        base = QualityConfig()
        cfg = config_with_overrides(base, None, {})
        self.assertEqual(cfg.signal_warn_dbm, base.signal_warn_dbm)


if __name__ == "__main__":
    unittest.main()
