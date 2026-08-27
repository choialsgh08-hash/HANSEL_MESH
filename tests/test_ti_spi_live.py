import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.radar_geometry import RadarAxes
from monitor.radar_scene import RadarSceneEstimator
from sensors.ti_spi_adc import NearFieldMap, NearFieldPeak
from sensors.ti_spi_live import (
    SPI_HEATMAP_AZIMUTH_BINS,
    SPI_HEATMAP_RANGE_BINS,
    SPI_SOURCE_FORMAT,
    commands_for_spi_stream,
    near_field_map_to_heatmap,
    near_field_map_to_radar_frame,
)


def sample_scene() -> NearFieldMap:
    ranges = np.linspace(0.0, 0.40, 81)
    angles = np.linspace(-60.0, 60.0, 121)
    power = np.full((ranges.size, angles.size), -36.0)
    left = (
        (abs(ranges[:, None] - 0.24) <= 0.012)
        & (abs(angles[None, :] + 25.0) <= 8.0)
    )
    right = (
        (abs(ranges[:, None] - 0.22) <= 0.012)
        & (abs(angles[None, :] - 45.0) <= 8.0)
    )
    power[left] = 0.0
    power[right] = -1.0
    return NearFieldMap(
        ranges_m=ranges,
        angles_deg=angles,
        power_db=power,
        spatial_coherence=np.full(ranges.size, 0.9),
        peaks=(
            NearFieldPeak(0.24, -25.0, -0.1014, 0.2175, 0.0, 0.9),
            NearFieldPeak(0.22, 45.0, 0.1556, 0.1556, -1.0, 0.88),
        ),
        trusted_minimum_range_m=0.10,
        maximum_range_m=0.30,
    )


class TiSpiLiveTests(unittest.TestCase):
    def test_stream_profile_uses_infinite_frame_count(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "spi.cfg"
            path.write_text(
                "sensorStop 0\nframeCfg 8 0 403 1 125 40\nadcLogging 2\n",
                encoding="ascii",
            )
            commands = commands_for_spi_stream(path)
        self.assertIn("frameCfg 8 0 403 1 125 0", commands)

    def test_full_map_is_resampled_without_claiming_blind_near_field(self) -> None:
        heatmap = near_field_map_to_heatmap(sample_scene())
        self.assertEqual(heatmap.range_bins, SPI_HEATMAP_RANGE_BINS)
        self.assertEqual(heatmap.azimuth_bins, SPI_HEATMAP_AZIMUTH_BINS)
        self.assertEqual(
            len(heatmap.data),
            SPI_HEATMAP_RANGE_BINS * SPI_HEATMAP_AZIMUTH_BINS,
        )
        blind_cells = 5 * SPI_HEATMAP_AZIMUTH_BINS
        self.assertEqual(set(heatmap.data[:blind_cells]), {0})
        self.assertGreater(max(heatmap.data[blind_cells:]), 0)

    def test_peak_coordinates_keep_left_and_right_semantics(self) -> None:
        frame = near_field_map_to_radar_frame(
            sample_scene(),
            seq=1,
            frame_number=4,
            monotonic_ns=1,
            wall_time_ns=2,
            producer_id="spi-test",
        )
        self.assertEqual(frame.source_format, SPI_SOURCE_FORMAT)
        self.assertLess(frame.points[0].x_m, 0.0)
        self.assertGreater(frame.points[1].x_m, 0.0)
        self.assertTrue(all(point.y_m > 0.0 for point in frame.points))

    def test_blind_zone_blocker_adds_conservative_stop_proxy(self) -> None:
        scene = sample_scene()
        blocked = NearFieldMap(
            ranges_m=scene.ranges_m,
            angles_deg=scene.angles_deg,
            power_db=scene.power_db,
            spatial_coherence=scene.spatial_coherence,
            peaks=scene.peaks,
            trusted_minimum_range_m=scene.trusted_minimum_range_m,
            maximum_range_m=scene.maximum_range_m,
            blind_zone_confidence=0.95,
            blind_zone_blocked=True,
            blind_zone_proxy_range_m=0.035,
            blind_zone_evidence_peak_m=0.085,
        )
        frame = near_field_map_to_radar_frame(
            blocked,
            seq=1,
            frame_number=4,
            monotonic_ns=1,
            wall_time_ns=2,
            producer_id="spi-test",
        )
        self.assertEqual(frame.points[0].x_m, 0.0)
        self.assertEqual(frame.points[0].y_m, 0.035)
        self.assertGreater(frame.points[0].snr_db, 28.0)

    def test_spi_raw_frame_is_trusted_without_room_calibration(self) -> None:
        estimator = RadarSceneEstimator(
            axes=RadarAxes(), require_calibration=False
        )
        frame = near_field_map_to_radar_frame(
            sample_scene(),
            seq=1,
            frame_number=4,
            monotonic_ns=1,
            wall_time_ns=2,
            producer_id="spi-test",
        )
        estimator.ingest(frame, received_at=1.0)
        scene = estimator.snapshot(source_status="live", now=1.0)
        self.assertEqual(scene["calibration_status"], "spi_raw")
        self.assertGreater(
            scene["diagnostics"]["reflectivity_surface_cell_count"], 0
        )


if __name__ == "__main__":
    unittest.main()
