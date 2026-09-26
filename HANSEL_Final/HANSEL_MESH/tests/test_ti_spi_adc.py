import unittest

import numpy as np

from sensors.ti_spi_adc import (
    BLIND_ZONE_BLOCK_CONFIDENCE,
    FtdiFccspCapture,
    NearFieldAdcProfile,
    _blind_zone_guard,
    _azimuth_snapshots,
    _capon_map,
    _read_command,
    decode_bpm,
    reshape_spi_adc,
    unpack_fccsp_spi_bytes,
)


class TiSpiAdcTests(unittest.TestCase):
    def test_unpack_matches_ti_low_word_then_high_word_order(self) -> None:
        payload = bytes.fromhex("1234fffe80007fff")
        actual = unpack_fccsp_spi_bytes(payload)
        np.testing.assert_array_equal(actual, [-2, 0x1234, 0x7FFF, -32768])

    def test_reshape_matches_matlab_column_major_contract(self) -> None:
        profile = NearFieldAdcProfile(
            adc_samples=2,
            rx_antennas=1,
            chirps_per_frame=4,
        )
        values = np.arange(8, dtype=np.int16)
        actual = reshape_spi_adc(values, profile, frames=1)
        self.assertEqual(actual.shape, (1, 2, 1, 2, 2))
        np.testing.assert_array_equal(actual[0, 0, 0, 0], [0, 1])
        np.testing.assert_array_equal(actual[0, 1, 0, 0], [2, 3])
        np.testing.assert_array_equal(actual[0, 0, 0, 1], [4, 5])
        np.testing.assert_array_equal(actual[0, 1, 0, 1], [6, 7])

    def test_bpm_sum_difference_recovers_both_transmitters(self) -> None:
        tx0 = np.array([[[[1.0, 4.0]]]])
        tx1 = np.array([[[[3.0, -2.0]]]])
        coded = np.stack((tx0 + tx1, tx0 - tx1), axis=1).reshape(1, 2, 1, 1, 2)
        decoded = decode_bpm(coded)
        np.testing.assert_allclose(decoded[:, 0], tx0)
        np.testing.assert_allclose(decoded[:, 1], tx1)

    def test_read_command_uses_mpsse_length_minus_one(self) -> None:
        self.assertEqual(
            _read_command(0x1234),
            bytes.fromhex("80000b20331280080b"),
        )

    def test_profile_reports_measured_transport_and_physical_limits(self) -> None:
        profile = NearFieldAdcProfile()
        self.assertEqual(profile.frame_bytes, 6144)
        self.assertAlmostEqual(profile.hpf_corner_range_m, 0.0690315, places=5)
        self.assertAlmostEqual(profile.physical_range_resolution_m, 0.03852, places=4)

    def test_blind_zone_guard_distinguishes_coupling_from_close_blocker(self) -> None:
        profile = NearFieldAdcProfile()
        ranges = np.linspace(0.0, 0.40, 81)
        shape = (4, 2, 3, 4, ranges.size)
        coupling_power = np.full(ranges.size, 0.02)
        coupling_power[(ranges >= 0.01) & (ranges < 0.07)] = 1.0
        coupling_power[(ranges >= 0.07) & (ranges < 0.12)] = 0.45
        blocker_power = np.full(ranges.size, 0.38)
        blocker_power[(ranges >= 0.01) & (ranges < 0.07)] = 1.0
        blocker_power[(ranges >= 0.07) & (ranges < 0.12)] = 2.0

        def spectrum(power: np.ndarray) -> np.ndarray:
            return np.broadcast_to(np.sqrt(power), shape).copy()

        coupling, _ = _blind_zone_guard(
            spectrum(coupling_power), ranges, profile,
            trusted_minimum_range_m=0.12,
        )
        blocker, peak_m = _blind_zone_guard(
            spectrum(blocker_power), ranges, profile,
            trusted_minimum_range_m=0.12,
        )
        self.assertLess(coupling, 0.1)
        self.assertGreaterEqual(blocker, BLIND_ZONE_BLOCK_CONFIDENCE)
        self.assertGreaterEqual(peak_m, profile.hpf_corner_range_m)
        self.assertLess(peak_m, 0.12)

    def test_capon_preserves_right_positive_azimuth(self) -> None:
        angles_deg = np.arange(-60.0, 61.0)
        antenna_columns = np.arange(4, dtype=np.float64)

        for expected_angle_deg in (-28.0, 28.0):
            manifold = np.exp(
                1j
                * np.pi
                * antenna_columns
                * np.sin(np.deg2rad(expected_angle_deg))
            )
            snapshots = np.tile(manifold, (16, 1))[:, :, None]
            power, _ = _capon_map(
                snapshots,
                np.array([0.15]),
                angles_deg,
            )
            measured_angle_deg = angles_deg[int(np.argmax(power[0]))]
            self.assertAlmostEqual(
                measured_angle_deg,
                expected_angle_deg,
                delta=1.0,
            )

    def test_capon_map_is_stable_for_coherent_wall_returns(self) -> None:
        angles_deg = np.arange(-60.0, 61.0)
        antenna_columns = np.arange(4, dtype=np.float64)

        def manifold(angle_deg: float) -> np.ndarray:
            return np.exp(
                1j
                * np.pi
                * antenna_columns
                * np.sin(np.deg2rad(angle_deg))
            )

        dominant = manifold(-26.0)
        secondary = manifold(31.0)
        measured = []
        for relative_phase_deg in (-12.0, -6.0, 0.0, 6.0, 12.0):
            phase = np.exp(1j * np.deg2rad(relative_phase_deg))
            snapshots = np.tile(
                dominant + 0.72 * phase * secondary,
                (24, 1),
            )[:, :, None]
            power, _ = _capon_map(
                snapshots,
                np.array([0.15]),
                angles_deg,
            )
            measured.append(angles_deg[int(np.argmax(power[0]))])

        self.assertTrue(all(angle < 0.0 for angle in measured))
        self.assertLessEqual(max(measured) - min(measured), 4.0)

    def test_azimuth_snapshots_use_static_slow_time_component(self) -> None:
        spectrum = np.zeros((2, 2, 3, 4, 1), dtype=np.complex128)
        spectrum[:, :, :, :, 0] = np.array([1.0, 2.0, 3.0, 4.0])

        snapshots = _azimuth_snapshots(spectrum)

        self.assertEqual(snapshots.shape, (2, 4, 1))
        # Mean slow-time value is 2.5.  The configured EVM compensation makes
        # the two TX1 row-zero elements negative.
        np.testing.assert_allclose(
            snapshots[:, :, 0],
            np.array([[2.5, -2.5, 2.5, -2.5]] * 2),
        )

    def test_capture_requires_context_manager(self) -> None:
        capture = FtdiFccspCapture(handle_factory=lambda _index: object())
        with self.assertRaisesRegex(RuntimeError, "not open"):
            capture.read_gpio()


if __name__ == "__main__":
    unittest.main()
