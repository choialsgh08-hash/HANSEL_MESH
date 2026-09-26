from __future__ import annotations

import unittest

import _paths  # noqa: F401
from hansel_unit_control.core import (
    GitHubMotionMapper,
    HeadServoStepper,
    MotionInput,
    PidController,
    RampLimiter,
    SafetyController,
)


class ControlCoreTests(unittest.TestCase):
    def test_github_style_head_servo_steps_and_clamps(self) -> None:
        head = HeadServoStepper(-180.0, 180.0, center_angle_deg=0.0, step_angle_deg=2.0)
        self.assertEqual(head.up(), 2.0)
        self.assertEqual(head.down(), 0.0)
        self.assertEqual(head.minimum(), -180.0)
        self.assertEqual(head.down(), -180.0)
        self.assertEqual(head.maximum(), 180.0)
        self.assertEqual(head.up(), 180.0)
        self.assertEqual(head.center(), 0.0)

    def test_head_semantic_steering_matches_github(self) -> None:
        mapper = GitHubMotionMapper("head")
        self.assertEqual(mapper.map("forward", 0.5).left_cps, 400.0)
        self.assertEqual(mapper.map("forward", 0.5).right_cps, 400.0)
        left = mapper.map("left", 0.5)
        self.assertEqual(left.left_cps, -340.0)
        self.assertEqual(left.right_cps, 340.0)
        curve = mapper.map("forward_left", 0.5)
        self.assertEqual(curve.left_cps, 180.0)
        self.assertEqual(curve.right_cps, 400.0)
        backward = mapper.map("backward_left", 0.5)
        self.assertEqual(backward.left_cps, -400.0)
        self.assertEqual(backward.right_cps, -180.0)

    def test_rear_normalizes_steering_like_github_router(self) -> None:
        mapper = GitHubMotionMapper("rear", node_slow_ratio=0.45)
        curve = mapper.map("forward_left", 0.5)
        self.assertEqual(curve.command, "slow_forward")
        self.assertEqual(curve.left_cps, 180.0)
        self.assertEqual(curve.right_cps, 180.0)
        spin = mapper.map("left", 0.5)
        self.assertEqual((spin.left_cps, spin.right_cps), (0.0, 0.0))
        self.assertEqual(spin.command, "stop")

    def test_left_right_target_scale_is_parameterized(self) -> None:
        mapper = GitHubMotionMapper("head", left_target_scale=0.9, right_target_scale=1.1)
        targets = mapper.map("forward", 0.5)
        self.assertEqual(targets.left_cps, 360.0)
        self.assertAlmostEqual(targets.right_cps, 440.0)

    def test_stale_and_non_increasing_commands_are_rejected(self) -> None:
        safety = SafetyController(command_timeout_s=0.5, max_command_age_s=0.5)
        stale = MotionInput(9.0, 1, "forward", 0.5, "test")
        self.assertFalse(safety.accept_motion(stale, 10.0, 1.0)[0])
        fresh = MotionInput(10.0, 2, "forward", 0.5, "test")
        self.assertTrue(safety.accept_motion(fresh, 10.1, 1.1)[0])
        accepted, reason = safety.accept_motion(fresh, 10.2, 1.2)
        self.assertFalse(accepted)
        self.assertIn("sequence", reason)

    def test_timeout_requires_explicit_reenable(self) -> None:
        safety = SafetyController(command_timeout_s=0.5, max_command_age_s=0.5)
        command = MotionInput(10.0, 1, "forward", 0.5, "test")
        self.assertTrue(safety.accept_motion(command, 10.0, 1.0)[0])
        self.assertTrue(safety.tick(1.51))
        self.assertFalse(safety.drive_enabled)
        self.assertFalse(safety.accept_motion(command, 10.1, 1.6)[0])
        self.assertTrue(safety.request_drive_enable()[0])

    def test_estop_blocks_reenable_until_cleared(self) -> None:
        safety = SafetyController()
        safety.set_estop(True)
        self.assertFalse(safety.request_drive_enable()[0])
        safety.set_estop(False)
        self.assertTrue(safety.request_drive_enable()[0])

    def test_hansel_mesh_feedforward_pid_matches_reference_shape(self) -> None:
        pid = PidController(
            0.035, 0.015, 0.0,
            output_limit=100.0,
            integral_limit=500.0,
            minimum_output=25.0,
            maximum_target=800.0,
        )
        # At 400 CPS, feed-forward=62.5, P=14.0, I=0.3.
        self.assertAlmostEqual(pid.step(400.0, 0.0, 0.05), 76.8, places=4)

    def test_hansel_mesh_pwm_ramp_is_percent_per_second(self) -> None:
        ramp = RampLimiter(220.0)
        self.assertAlmostEqual(ramp.step(100.0, 0.05), 11.0)
        self.assertAlmostEqual(ramp.step(100.0, 0.05), 22.0)


if __name__ == "__main__":
    unittest.main()
