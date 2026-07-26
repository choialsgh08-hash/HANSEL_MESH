import unittest

from monitor import calibrate_thresholds as calib


class PercentileTests(unittest.TestCase):
    def test_empty_is_none(self) -> None:
        self.assertIsNone(calib.percentile([], 50))

    def test_median_and_p95(self) -> None:
        values = [1, 2, 3, 4, 5]
        self.assertEqual(calib.percentile(values, 50), 3)
        self.assertGreaterEqual(calib.percentile(values, 95), 4)


class VideoLevelTests(unittest.TestCase):
    def test_clean_video_is_good(self) -> None:
        self.assertEqual(
            calib.video_level({"vid_err": 0.0, "vid_fps_ratio": 1.0}), "GOOD"
        )

    def test_high_error_is_danger(self) -> None:
        self.assertEqual(calib.video_level({"vid_err": 5.0}), "DANGER")

    def test_mild_fps_dip_is_warn(self) -> None:
        self.assertEqual(
            calib.video_level({"vid_err": 0.0, "vid_fps_ratio": 0.7}), "WARN"
        )


class SignalThresholdTests(unittest.TestCase):
    def _records(self):
        # Strong signal -> clean video; as RSSI weakens, video degrades.
        recs = []
        for rssi in range(-72, -49):          # healthy band (-72..-50)
            recs.append({"net_rssi": rssi, "vid_err": 0.0, "vid_fps_ratio": 1.0})
        for rssi in range(-81, -73):          # WARN band (-81..-74)
            recs.append({"net_rssi": rssi, "vid_err": 0.5, "vid_fps_ratio": 0.8})
        for rssi in range(-89, -81):          # DANGER band (-89..-82)
            recs.append({"net_rssi": rssi, "vid_err": 3.0, "vid_fps_ratio": 0.4})
        return recs

    def test_thresholds_land_in_expected_bands(self) -> None:
        out = calib.derive_signal_thresholds(self._records())
        self.assertIsNotNone(out["signal_warn_dbm"])
        self.assertIsNotNone(out["signal_danger_dbm"])
        # warn triggers at a stronger (less negative) signal than danger
        self.assertGreater(out["signal_warn_dbm"], out["signal_danger_dbm"])
        # warn near where video first degrades (~-74), danger near ~-82
        self.assertLess(out["signal_warn_dbm"], -60)
        self.assertLess(out["signal_danger_dbm"], -75)

    def test_insufficient_samples_reports_note(self) -> None:
        out = calib.derive_signal_thresholds(
            [{"net_rssi": -55, "vid_err": 0.0, "vid_fps_ratio": 1.0}]
        )
        self.assertIsNone(out["signal_warn_dbm"])
        self.assertTrue(out["notes"])


class ReconvergeTests(unittest.TestCase):
    def test_natural_reconnect_without_link_down(self) -> None:
        # Two organically observed reconnects, anchored on neighbor_lost.
        events = [
            {"type": "neighbor_lost", "ts": 100.0},
            {"type": "route_changed", "ts": 100.9},
            {"type": "video_recovered", "ts": 102.5},
            {"type": "neighbor_lost", "ts": 200.0},
            {"type": "route_changed", "ts": 201.2},
            {"type": "video_recovered", "ts": 203.0},
        ]
        out = calib.derive_reconverge_times(events)
        self.assertEqual(out["cycles"], 2)
        self.assertEqual(out["reconverge_s"]["count"], 2)
        self.assertEqual(out["video_recovery_s"]["count"], 2)
        # detect_delay needs a forced link_down; none here
        self.assertEqual(out["detect_delay_s"]["count"], 0)
        # total outage measured from neighbor_lost
        self.assertAlmostEqual(out["total_outage_s"]["max"], 3.0, places=3)

    def test_forced_break_adds_detect_delay(self) -> None:
        events = [
            {"type": "link_down", "ts": 10.0},
            {"type": "neighbor_lost", "ts": 10.4},
            {"type": "route_changed", "ts": 11.3},
            {"type": "video_recovered", "ts": 13.0},
        ]
        out = calib.derive_reconverge_times(events)
        self.assertEqual(out["cycles"], 1)
        self.assertAlmostEqual(out["detect_delay_s"]["p50"], 0.4, places=3)
        self.assertAlmostEqual(out["total_outage_s"]["p50"], 3.0, places=3)


if __name__ == "__main__":
    unittest.main()
