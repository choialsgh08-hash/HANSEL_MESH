from __future__ import annotations
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
ROS_SOURCE = ROOT / "ros2_ws" / "src"
INTERFACES = ROS_SOURCE / "hansel_interfaces"


class InterfaceContractTests(unittest.TestCase):
    def test_expected_package_boundaries_exist(self) -> None:
        expected = {
            "hansel_interfaces", "hansel_operator", "hansel_unit_control",
            "hansel_camera_bridge", "hansel_network_adapter",
            "hansel_radar_adapter", "hansel_survivor_adapter",
            "hansel_description", "hansel_bringup",
        }
        self.assertEqual(expected, {p.name for p in ROS_SOURCE.iterdir() if p.is_dir()})

    def test_motion_contract_uses_github_semantic_commands(self) -> None:
        msg = (INTERFACES / "msg" / "MotionCommand.msg").read_text()
        operator = (ROS_SOURCE / "hansel_operator/hansel_operator/operator_input.py").read_text()
        router = (ROS_SOURCE / "hansel_operator/hansel_operator/command_router.py").read_text()
        self.assertIn("string command", msg)
        for token in ('"w": "forward"', '"a": "left"', '"q": "forward_left"', '"z": "backward_left"'):
            self.assertIn(token, operator)
        self.assertIn('return "slow_forward"', router)
        self.assertIn('return "slow_backward"', router)

    def test_rqt_is_parameter_only(self) -> None:
        panel = (ROS_SOURCE / "hansel_operator/hansel_operator/rqt_plugin.py").read_text()
        for token in ("Straight RPM", "Turn RPM", "Up limit", "Down limit"):
            self.assertIn(token, panel)
        for forbidden in (
            "Speed scale", "left_kp", "minimum_pwm_percent",
            "encoder_counts_per_revolution", "head_min_pulse_us",
            "head_step_angle_deg", "Head UP",
        ):
            self.assertNotIn(forbidden, panel)
        self.assertNotIn("create_publisher(MotionCommand", panel)

    def test_nano_wiring_and_firmware_match_uploaded_sheet(self) -> None:
        firmware = (ROOT / "robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino").read_text()
        for token in (
            "PWM_LEFT = 5", "PWM_RIGHT = 6", "FRONT_L_IN1 = 7",
            "FRONT_L_IN2 = 8", "FRONT_R_IN1 = 11", "FRONT_R_IN2 = 12",
            "REAR_L_IN1 = 13", "REAR_L_IN2 = A0", "REAR_R_IN1 = A1",
            "REAR_R_IN2 = A2", "ENC_L_A = 2", "ENC_L_B = 4",
            "ENC_R_A = 3", "ENC_R_B = A3", "HEAD_SERVO_PIN = 9",
            "DETACH_SERVO_PIN = 10",
        ):
            self.assertIn(token, firmware)
        self.assertIn("COMMAND_WATCHDOG_MS = 500", firmware)
        self.assertIn("rearRightReverse = true", firmware)

    def test_nano_serial_adapter_uses_compact_protocol(self) -> None:
        hardware = (ROS_SOURCE / "hansel_unit_control/hansel_unit_control/hardware.py").read_text()
        self.assertIn('self._send(f"M,', hardware)
        self.assertIn('self._send(f"F,', hardware)
        self.assertIn('self._send(f"H,', hardware)
        self.assertIn('parts[0] == "T"', hardware)
        self.assertNotIn("pigpio", hardware)

    def test_network_and_radar_match_github_contracts(self) -> None:
        network = (ROS_SOURCE / "hansel_network_adapter/hansel_network_adapter/hansel_mesh_provider.py").read_text()
        radar = (ROS_SOURCE / "hansel_radar_adapter/hansel_radar_adapter/hansel_mesh_provider.py").read_text()
        for key in ('snapshot.get("links")', 'snapshot.get("end_to_end")', 'snapshot.get("bat0")'):
            self.assertIn(key, network)
        self.assertIn('record.get("record_type") != "radar_frame"', radar)
        self.assertIn('item["x_m"]', radar)
        self.assertIn('item["radial_velocity_mps"]', radar)
        self.assertIn('(raw_y, -raw_x, raw_z', radar)

    def test_urdf_exposes_full_logical_head_range(self) -> None:
        urdf = (ROS_SOURCE / "hansel_description/urdf/hansel_head.urdf.xacro").read_text()
        self.assertIn('lower="-3.141593"', urdf)
        self.assertIn('upper="3.141593"', urdf)

    def test_detach_contract_never_claims_physical_success(self) -> None:
        service = (INTERFACES / "srv/DetachUnit.srv").read_text()
        combined = "\n".join(path.read_text() for path in (INTERFACES / "msg").glob("*.msg"))
        self.assertIn("command_completed", service)
        self.assertNotIn("physical_detached", service + combined)


if __name__ == "__main__":
    unittest.main()
