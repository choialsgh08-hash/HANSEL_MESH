from __future__ import annotations

import re
import threading
import time
import unittest
from pathlib import Path

import _paths  # noqa: F401
from hansel_unit_control.core import GitHubMotionMapper, PidController
from hansel_unit_control.hardware import NanoSerialHardwareAdapter


ROOT = Path(__file__).resolve().parents[1]
RUNTIME_CONFIG = (
    ROOT / "ros2_ws/src/hansel_bringup/config/unit_runtime.yaml"
)
FIRMWARE = ROOT / "robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino"


class MotorSafetyTests(unittest.TestCase):
    def test_runtime_config_uses_75_percent_limit(self) -> None:
        parameters = {}
        for line in RUNTIME_CONFIG.read_text(encoding="utf-8").splitlines():
            match = re.match(r"\s+([a-z0-9_]+):\s+(-?[0-9.]+)\s*$", line)
            if match:
                parameters[match.group(1)] = float(match.group(2))

        self.assertEqual(parameters["straight_rpm"], 50.0)
        self.assertEqual(parameters["turn_rpm"], 30.0)
        self.assertEqual(parameters["maximum_pwm_percent"], 75.0)
        self.assertEqual(parameters["pwm_ramp_percent_per_second"], 100.0)

        # These calibration/controller values are intentionally unchanged.
        self.assertEqual(parameters["minimum_pwm_percent"], 25.0)
        self.assertEqual(parameters["left_kp"], 0.035)
        self.assertEqual(parameters["left_ki"], 0.015)
        self.assertEqual(parameters["left_kd"], 0.0)
        self.assertEqual(parameters["right_kp"], 0.035)
        self.assertEqual(parameters["right_ki"], 0.015)
        self.assertEqual(parameters["right_kd"], 0.0)
        self.assertEqual(parameters["encoder_counts_per_revolution"], 400.0)

    def test_ros_pid_output_is_clamped_to_40_percent(self) -> None:
        pid = PidController(
            0.035,
            0.015,
            0.0,
            output_limit=40.0,
            integral_limit=500.0,
            minimum_output=25.0,
            maximum_target=50.0 * 400.0 / 60.0,
        )

        self.assertEqual(pid.step(10_000.0, 0.0, 0.05), 40.0)

    def test_turn_target_is_separate_from_straight_target(self) -> None:
        straight_cps = 50.0 * 400.0 / 60.0
        turn_cps = 30.0 * 400.0 / 60.0
        mapper = GitHubMotionMapper(
            "head",
            full_speed_cps_left=straight_cps,
            full_speed_cps_right=straight_cps,
            turn_speed_cps_left=turn_cps,
            turn_speed_cps_right=turn_cps,
        )

        spin = mapper.map("left", 1.0)
        self.assertAlmostEqual(spin.left_cps, -turn_cps)
        self.assertAlmostEqual(spin.right_cps, turn_cps)

        commands = (
            "forward",
            "backward",
            "left",
            "right",
            "forward_left",
            "forward_right",
            "backward_left",
            "backward_right",
            "mild_forward_left",
            "mild_forward_right",
            "mild_backward_left",
            "mild_backward_right",
        )
        for command in commands:
            targets = mapper.map(command, 1.0)
            self.assertLessEqual(abs(targets.left_cps), straight_cps)
            self.assertLessEqual(abs(targets.right_cps), straight_cps)

    @staticmethod
    def _adapter_with_send_capture():
        sent: list[str] = []
        adapter = NanoSerialHardwareAdapter.__new__(NanoSerialHardwareAdapter)
        adapter.front_follow_drive = True
        adapter.motor_pwm_limit_percent = 40.0
        adapter._left_cps = 0.0
        adapter._right_cps = 0.0
        adapter._state_lock = threading.Lock()
        adapter._read_error = ""
        adapter._last_state_monotonic = time.monotonic()
        adapter._front_override = None
        adapter._configuration_pending = False
        adapter._send = sent.append
        return adapter, sent

    def test_ros_serial_drive_command_is_clamped_to_configured_limit(self) -> None:
        adapter, sent = self._adapter_with_send_capture()

        adapter.update(100.0, -100.0, True, 0.05)

        self.assertEqual(sent, ["M,40.000,-40.000,1"])

    def test_ros_serial_front_only_command_is_clamped_to_configured_limit(self) -> None:
        adapter, sent = self._adapter_with_send_capture()

        adapter.set_front_motor("front_motor_forward", 1.0)

        self.assertEqual(sent, ["F,1,40.000"])

    def test_nano_firmware_declares_and_uses_75_percent_hard_limit(self) -> None:
        source = FIRMWARE.read_text(encoding="utf-8")
        match = re.search(
            r"MOTOR_PWM_LIMIT_PERCENT\s*=\s*([0-9.]+)f",
            source,
        )
        self.assertIsNotNone(match)
        limit = float(match.group(1))
        self.assertEqual(limit, 75.0)
        self.assertIn(
            "if (value > MOTOR_PWM_LIMIT_PERCENT) "
            "value = MOTOR_PWM_LIMIT_PERCENT;",
            source,
        )
        self.assertEqual(int(limit * 2.55 + 0.5), 191)


if __name__ == "__main__":
    unittest.main()
