#!/usr/bin/env python3
"""ROS 없이 semantic steering, PID/ramp, U/J/K stepping을 검증한다."""

from __future__ import annotations
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "hansel_unit_control"))

from hansel_unit_control.core import GitHubMotionMapper, HeadServoStepper, PidController, RampLimiter


def main() -> None:
    head = GitHubMotionMapper("head")
    rear = GitHubMotionMapper("rear")
    head_curve = head.map("forward_left", 0.5)
    rear_curve = rear.map("forward_left", 0.5)
    assert (head_curve.left_cps, head_curve.right_cps) == (180.0, 400.0)
    assert (rear_curve.left_cps, rear_curve.right_cps) == (180.0, 180.0)
    spin = head.map("left", 0.5)
    assert (spin.left_cps, spin.right_cps) == (-340.0, 340.0)

    pid = PidController(0.035, 0.015, 0.0, output_limit=100.0,
                        integral_limit=500.0, minimum_output=25.0,
                        maximum_target=800.0)
    ramp = RampLimiter(220.0)
    first_pwm = ramp.step(pid.step(400.0, 0.0, 0.05), 0.05)
    assert first_pwm == 11.0

    servo = HeadServoStepper(-180.0, 180.0, center_angle_deg=0.0, step_angle_deg=2.0)
    assert [servo.up(), servo.up(), servo.down(), servo.center()] == [2.0, 4.0, 2.0, 0.0]

    print("PASS: Q/E/Z/C steering mapping follows HANSEL_MESH ratios")
    print(f"  head forward-left = {head_curve.left_cps:.0f}/{head_curve.right_cps:.0f} CPS")
    print(f"  rear follow        = {rear_curve.left_cps:.0f}/{rear_curve.right_cps:.0f} CPS")
    print(f"PASS: A/D spin       = {spin.left_cps:.0f}/{spin.right_cps:.0f} CPS")
    print(f"PASS: first ramp PWM = {first_pwm:.1f}%")
    print("PASS: U/J/K           = +2/-2/center in -180..+180 logical range")


if __name__ == "__main__":
    main()
