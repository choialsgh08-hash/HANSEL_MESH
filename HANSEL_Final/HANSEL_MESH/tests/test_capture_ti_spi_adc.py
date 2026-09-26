import tempfile
import unittest
from pathlib import Path

from scripts.capture_ti_spi_adc import _commands_for_capture


class CaptureTiSpiAdcTests(unittest.TestCase):
    def test_frame_count_is_replaced_without_changing_chirp_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / "near.cfg"
            profile.write_text(
                "sensorStop 0\n"
                "chirpComnCfg 8 0 0 128 4 11.6 0\n"
                "frameCfg 8 0 403 1 250 40\n",
                encoding="ascii",
            )
            commands = _commands_for_capture(profile, 7)
        self.assertIn("chirpComnCfg 8 0 0 128 4 11.6 0", commands)
        self.assertIn("frameCfg 8 0 403 1 250 7", commands)

    def test_profile_must_leave_sensor_start_to_capture_orchestrator(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / "bad.cfg"
            profile.write_text(
                "frameCfg 8 0 403 1 250 1\nsensorStart 0 0 0 0\n",
                encoding="ascii",
            )
            with self.assertRaisesRegex(ValueError, "sensorStart"):
                _commands_for_capture(profile, 1)


if __name__ == "__main__":
    unittest.main()
