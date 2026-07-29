import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from sensors import ti_radar_control


SCRIPT = (
    Path(__file__).resolve().parent.parent
    / "scripts"
    / "configure_ti_radar.py"
)
SPEC = importlib.util.spec_from_file_location("configure_ti_radar", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ConfigureTiRadarTest(unittest.TestCase):
    def test_wrapper_reexports_profile_control_objects(self):
        self.assertIs(MODULE.ProfileCommands, ti_radar_control.ProfileCommands)
        self.assertIs(MODULE.load_commands, ti_radar_control.load_commands)
        self.assertIs(MODULE.partition_at_baud, ti_radar_control.partition_at_baud)
        self.assertIs(MODULE.apply_profile, ti_radar_control.apply_profile)

    def test_help_runs_outside_repository(self):
        with tempfile.TemporaryDirectory() as directory:
            completed = subprocess.run(
                [sys.executable, str(SCRIPT), "--help"],
                cwd=directory,
                check=False,
                capture_output=True,
                text=True,
            )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("Apply an xWRL6432 CLI profile", completed.stdout)

    def test_low_latency_profile_is_10hz_with_16_azimuth_bins(self):
        cfg = (
            Path(__file__).resolve().parent.parent
            / "configs"
            / "radar"
            / "iwrl6432_heatmap_10hz.cfg"
        )
        commands = MODULE.load_commands(cfg)
        profile = MODULE.partition_at_baud(commands)
        self.assertEqual(profile.target_baud, 1_250_000)
        self.assertIn(
            "frameCfg 2 8 600 16 100 0",
            profile.before_baud,
        )
        self.assertIn(
            "guiMonitor 2 0 0 1 0 1 0 0 0 0 0",
            profile.before_baud,
        )
        self.assertIn(
            "sigProcChainCfg 16 4 1 2 8 4 0 0.3 0",
            profile.before_baud,
        )
        self.assertIn("rangeSelCfg 0.25 7.5", profile.before_baud)
        self.assertIn("clutterRemoval 0", profile.before_baud)
        self.assertEqual(
            profile.after_baud,
            ("sensorStart 0 0 0 0",),
        )

    def test_3d_profile_uses_conservative_elevation_fft(self):
        cfg = (
            Path(__file__).resolve().parent.parent
            / "configs"
            / "radar"
            / "iwrl6432_3d_operator_10hz.cfg"
        )
        profile = MODULE.partition_at_baud(MODULE.load_commands(cfg))
        self.assertIn(
            "sigProcChainCfg 16 8 1 2 8 4 0 0.3 0",
            profile.before_baud,
        )
        self.assertIn(
            "cfarCfg 2 8 4 3 0 15 0 0.5 0 1 1 1",
            profile.before_baud,
        )

    def test_near_3d_profile_starts_at_first_practical_range_bin(self):
        cfg = (
            Path(__file__).resolve().parent.parent
            / "configs"
            / "radar"
            / "iwrl6432_3d_operator_near_10hz.cfg"
        )
        profile = MODULE.partition_at_baud(MODULE.load_commands(cfg))
        self.assertEqual(profile.target_baud, 1_250_000)
        self.assertIn("rangeSelCfg 0.07 7.5", profile.before_baud)
        self.assertIn("lowPowerCfg 0", profile.before_baud)
        self.assertNotIn("lowPowerCfg 1", profile.before_baud)
        self.assertIn(
            "sigProcChainCfg 16 8 1 2 8 4 0 0.3 0",
            profile.before_baud,
        )
        self.assertIn(
            "cfarCfg 2 8 4 3 0 15 0 0.5 0 1 1 1",
            profile.before_baud,
        )
        self.assertIn(
            "frameCfg 2 8 600 16 100 0",
            profile.before_baud,
        )
        self.assertEqual(profile.after_baud, ("sensorStart 0 0 0 0",))

    def test_stable_near_3d_profile_is_8hz_and_only_changes_frame_period(self):
        root = Path(__file__).resolve().parent.parent / "configs" / "radar"
        stable = MODULE.partition_at_baud(
            MODULE.load_commands(
                root / "iwrl6432_3d_operator_near_8hz.cfg"
            )
        )
        experimental = MODULE.partition_at_baud(
            MODULE.load_commands(
                root / "iwrl6432_3d_operator_near_10hz.cfg"
            )
        )

        expected_stable_commands = tuple(
            (
                "frameCfg 2 8 600 16 125 0"
                if command == "frameCfg 2 8 600 16 100 0"
                else command
            )
            for command in experimental.before_baud
        )

        self.assertEqual(stable.target_baud, 1_250_000)
        self.assertEqual(stable.before_baud, expected_stable_commands)
        self.assertEqual(stable.after_baud, experimental.after_baud)
        self.assertIn("lowPowerCfg 0", stable.before_baud)
        self.assertIn(
            "sigProcChainCfg 16 8 1 2 8 4 0 0.3 0",
            stable.before_baud,
        )

    def test_load_and_partition_heatmap_profile(self):
        cfg = (
            Path(__file__).resolve().parent.parent
            / "configs"
            / "radar"
            / "iwrl6432_heatmap_5hz.cfg"
        )
        commands = MODULE.load_commands(cfg)
        profile = MODULE.partition_at_baud(commands)
        self.assertEqual(profile.target_baud, 1_250_000)
        self.assertTrue(
            any(
                command.startswith("guiMonitor 2 0 0 1 ")
                for command in profile.before_baud
            )
        )
        self.assertTrue(
            any(
                command.startswith("sigProcChainCfg 32 ")
                for command in profile.before_baud
            )
        )
        self.assertNotIn("sensorStop 0", profile.before_baud)
        self.assertEqual(
            profile.after_baud,
            ("sensorStart 0 0 0 0",),
        )

    def test_rejects_inline_comment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.cfg"
            path.write_text("channelCfg 7 3 0 % bad\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "inline"):
                MODULE.load_commands(path)

    def test_requires_exactly_one_baud_command(self):
        with self.assertRaisesRegex(ValueError, "exactly one"):
            MODULE.partition_at_baud(("channelCfg 7 3 0",))
        with self.assertRaisesRegex(ValueError, "exactly one"):
            MODULE.partition_at_baud(
                (
                    "baudRate 1250000",
                    "baudRate 921600",
                    "sensorStart 0 0 0 0",
                )
            )

    def test_requires_command_after_baud(self):
        with self.assertRaisesRegex(ValueError, "no command after"):
            MODULE.partition_at_baud(("baudRate 1250000",))


if __name__ == "__main__":
    unittest.main()
