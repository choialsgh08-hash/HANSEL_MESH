from pathlib import Path
import subprocess
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from sensors.ti_radar_control import (
    find_xds110_reset,
    load_commands,
    partition_at_baud,
    reset_xds110_target,
    select_application_port,
    validate_profile_result,
)


class TiRadarControlTest(unittest.TestCase):
    def setUp(self):
        self.application = SimpleNamespace(
            device="COM3",
            vid=0x0451,
            pid=0xBEF3,
            serial_number="RI32",
            description="XDS110 Class Application/User UART(COM3)",
            location="1-3:x.0",
        )
        self.auxiliary = SimpleNamespace(
            device="COM4",
            vid=0x0451,
            pid=0xBEF3,
            serial_number="RI32",
            description="XDS110 Class Auxiliary Data Port(COM4)",
            location="1-3:x.3",
        )

    def test_select_application_port_uses_identity_not_port_order(self):
        selected = select_application_port(
            [self.auxiliary, self.application],
            xds_serial="RI32",
        )
        self.assertEqual(selected.device, "COM3")

    def test_select_application_port_accepts_renumbered_application_port(self):
        renumbered = SimpleNamespace(
            **{**self.application.__dict__, "device": "COM9"}
        )
        self.assertEqual(
            select_application_port([renumbered], xds_serial="RI32").device,
            "COM9",
        )

    def test_select_application_port_rejects_auxiliary_only(self):
        with self.assertRaisesRegex(RuntimeError, "Application/User"):
            select_application_port([self.auxiliary], xds_serial="RI32")

    def test_select_application_port_rejects_ambiguous_applications(self):
        other = SimpleNamespace(
            **{**self.application.__dict__, "device": "COM9", "location": "1-4:x.0"}
        )
        with self.assertRaisesRegex(RuntimeError, "exactly one"):
            select_application_port([self.application, other])

    def test_select_application_port_rejects_nonexistent_explicit_port(self):
        with self.assertRaisesRegex(RuntimeError, "COM9"):
            select_application_port([self.application], explicit_port="COM9")

    def test_select_application_port_rejects_mismatched_serial(self):
        with self.assertRaisesRegex(RuntimeError, "RI99"):
            select_application_port([self.application], xds_serial="RI99")

    def test_find_xds110_reset_prefers_existing_explicit_path(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            explicit = root / "chosen-reset.exe"
            explicit.touch()
            self.assertEqual(
                find_xds110_reset(explicit, [root]),
                explicit,
            )

    def test_find_xds110_reset_chooses_newest_uniflash_version(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            older = (
                root
                / "uniflash_8.4.0"
                / "deskdb/content/TICloudAgent/win/ccs_base/common/uscif/xds110"
            )
            newer = root / "uniflash_8.7.1" / "simplelink/imagecreator/bin"
            older.mkdir(parents=True)
            newer.mkdir(parents=True)
            (older / "xds110reset.exe").touch()
            expected = newer / "xds110reset.exe"
            expected.touch()
            with patch("sensors.ti_radar_control.shutil.which", return_value=None):
                self.assertEqual(find_xds110_reset(None, [root]), expected)

    def test_find_xds110_reset_reports_missing_tool(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            with patch("sensors.ti_radar_control.shutil.which", return_value=None):
                with self.assertRaisesRegex(RuntimeError, "xds110reset"):
                    find_xds110_reset(None, [Path(temporary_directory)])

    def test_reset_xds110_target_scopes_toggle_to_serial_number(self):
        reset_executable = Path("C:/tools/xds110reset.exe")
        calls = []

        def runner(*args, **kwargs):
            calls.append((args, kwargs))

        reset_xds110_target(reset_executable, "RI32", runner)
        self.assertEqual(len(calls), 1)
        command, = calls[0][0]
        self.assertEqual(
            command,
            [
                str(reset_executable),
                "-a",
                "toggle",
                "-d",
                "100",
                "-s",
                "RI32",
            ],
        )
        self.assertEqual(
            calls[0][1],
            {"check": True, "capture_output": True, "text": True},
        )

    def test_reset_xds110_target_rejects_empty_serial_without_running(self):
        called = False

        def runner(*args, **kwargs):
            nonlocal called
            called = True

        with self.assertRaisesRegex(ValueError, "serial"):
            reset_xds110_target(Path("xds110reset.exe"), "", runner)
        self.assertFalse(called)

    def test_reset_xds110_target_reports_tool_stderr(self):
        def runner(*args, **kwargs):
            raise subprocess.CalledProcessError(
                1,
                args[0],
                stderr="target RI32 was not found",
            )

        with self.assertRaisesRegex(RuntimeError, "target RI32 was not found"):
            reset_xds110_target(Path("xds110reset.exe"), "RI32", runner)

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
