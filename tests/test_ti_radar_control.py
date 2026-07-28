from pathlib import Path
import unittest

from sensors.ti_radar_control import (
    load_commands,
    partition_at_baud,
    validate_profile_result,
)


class TiRadarControlTest(unittest.TestCase):
    def test_active_profile_partitions_at_1250000_baud(self):
        path = (
            Path(__file__).resolve().parent.parent
            / "configs"
            / "radar"
            / "iwrl6432_3d_operator_near_10hz.cfg"
        )
        profile = partition_at_baud(load_commands(path))
        self.assertEqual(profile.target_baud, 1_250_000)
        self.assertEqual(profile.after_baud, ("sensorStart 0 0 0 0",))

    def test_profile_result_requires_every_command_and_start_evidence(self):
        with self.assertRaisesRegex(RuntimeError, "first radar frame"):
            validate_profile_result(
                {
                    "commands_completed": 25,
                    "new_baud_prompt_observed": True,
                    "first_magic_observed": False,
                },
                expected_commands=25,
            )


if __name__ == "__main__":
    unittest.main()
