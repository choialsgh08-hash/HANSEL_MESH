import threading
import unittest
from unittest import mock

from robot import motor_driver
from robot.motor_driver import (
    DryRunRobotController,
    GpioRobotController,
    clamp,
    role_config,
)


class MotorLatchTests(unittest.TestCase):
    def test_encoder_snapshot_is_read_only_atomic_copy(self) -> None:
        controller = object.__new__(GpioRobotController)
        controller.encoder_lock = threading.Lock()
        controller.left_count = -12
        controller.right_count = 34

        with mock.patch(
            "robot.motor_driver.time.monotonic_ns",
            return_value=987654,
        ):
            snapshot = controller.encoder_snapshot()

        self.assertEqual(snapshot.monotonic_ns, 987654)
        self.assertEqual(snapshot.left_count, -12)
        self.assertEqual(snapshot.right_count, 34)
        self.assertEqual(controller.left_count, -12)
        self.assertEqual(controller.right_count, 34)

    def test_dry_run_encoder_snapshot_is_zero(self) -> None:
        controller = DryRunRobotController("head")
        snapshot = controller.encoder_snapshot()
        self.assertEqual(snapshot.left_count, 0)
        self.assertEqual(snapshot.right_count, 0)

    def test_non_finite_numeric_value_clamps_to_safe_minimum(self) -> None:
        self.assertEqual(clamp(float("nan"), 0.0, 1.0), 0.0)
        self.assertEqual(clamp(float("inf"), 0.0, 1.0), 0.0)

    def test_relay_hold_stops_and_blocks_drive_until_enable(self) -> None:
        controller = DryRunRobotController("node1")
        controller.start()

        applied, _ = controller.handle_command("forward")
        self.assertTrue(applied)
        self.assertEqual(controller.last_command, "forward")

        applied, reason = controller.handle_command("relay_hold")
        self.assertTrue(applied)
        self.assertEqual(reason, "drive_disabled")
        self.assertEqual(controller.last_command, "stop")

        applied, reason = controller.handle_command("forward")
        self.assertFalse(applied)
        self.assertEqual(reason, "drive_disabled")
        self.assertEqual(controller.last_command, "stop")

        applied, reason = controller.handle_command("drive_enable")
        self.assertTrue(applied)
        self.assertEqual(reason, "drive_enabled")

        applied, _ = controller.handle_command("forward")
        self.assertTrue(applied)
        self.assertEqual(controller.last_command, "forward")

    def test_missing_gpio_requires_explicit_dry_run(self) -> None:
        with mock.patch.object(motor_driver, "GPIO", None):
            with self.assertRaisesRegex(RuntimeError, "RPi.GPIO is not installed"):
                motor_driver.build_robot_controller("head")

            controller = motor_driver.build_robot_controller("head", dry_run=True)
            self.assertIsInstance(controller, DryRunRobotController)

    def test_front_motor_key_command_respects_speed_cap(self) -> None:
        controller = object.__new__(GpioRobotController)
        controller.config = role_config("head")
        controller.FRONT_MOTOR_KEY_PWM = 100.0

        self.assertEqual(controller._front_command_pwm({"speed": 0.35}), 35.0)
        self.assertEqual(controller._front_command_pwm({"speed": 0.0}), 0.0)
        self.assertEqual(
            controller._front_command_pwm({"speed": float("nan")}),
            0.0,
        )

    def test_control_loop_fault_blocks_motion_and_reenable(self) -> None:
        controller = object.__new__(GpioRobotController)
        controller.role = "head"
        controller.drive_enabled = False
        controller.control_fault = "encoder: RuntimeError: read failed"

        applied, reason = controller.handle_command("forward")
        self.assertFalse(applied)
        self.assertEqual(reason, "controller_fault")

        applied, reason = controller.handle_command("drive_enable")
        self.assertFalse(applied)
        self.assertEqual(reason, "controller_fault")

    def test_control_loop_failure_latches_stop_state(self) -> None:
        controller = object.__new__(GpioRobotController)
        controller.role = "head"
        controller.drive_enabled = True
        controller.running = mock.Mock()
        controller.stop_all = mock.Mock()

        controller._fail_closed_control_loop(
            "pid",
            RuntimeError("PWM write failed"),
        )

        self.assertFalse(controller.drive_enabled)
        self.assertIn("pid: RuntimeError", controller.control_fault)
        controller.running.clear.assert_called_once_with()
        controller.stop_all.assert_called_once_with()

    def test_detach_servo_returns_to_rest_when_press_timing_fails(self) -> None:
        controller = object.__new__(GpioRobotController)
        controller.DETACH_PRESS_ANGLE = 75
        controller.DETACH_REST_ANGLE = 20
        controller.DETACH_PRESS_TIME = 0.35
        controller.set_detach_servo_angle = mock.Mock()

        with mock.patch(
            "robot.motor_driver.time.sleep",
            side_effect=RuntimeError("simulated timer failure"),
        ), self.assertRaises(RuntimeError):
            controller.detach_servo_press()

        self.assertEqual(
            controller.set_detach_servo_angle.call_args_list,
            [
                mock.call(75, hold=True),
                mock.call(20, hold=False),
            ],
        )


if __name__ == "__main__":
    unittest.main()
