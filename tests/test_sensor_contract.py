import json
import unittest

from common.sensor_contract import (
    DropEvent,
    ImuSample,
    RadarFrame,
    RadarHeatmap,
    RadarPoint,
    SensorHeader,
    SensorHealth,
    WheelState,
    record_to_dict,
)
from common.sensor_json import decode_sensor_record, encode_sensor_record


def header(stream_id="radar/front", seq=1):
    return SensorHeader(
        mission_id="mission-1",
        unit_id="head",
        boot_id="boot-1",
        producer_id="producer-1",
        stream_id=stream_id,
        seq=seq,
        monotonic_ns=1_000_000_000 + seq,
        frame_id="base_link",
    )


class SensorContractTests(unittest.TestCase):
    def records(self):
        return [
            RadarFrame(
                header=header(),
                frame_number=7,
                subframe_number=0,
                complete=True,
                dropped_frames_since_previous=0,
                points=(
                    RadarPoint(1.0, -0.2, 0.1, -0.5, 14.2, 3.1),
                    RadarPoint(2.0, 0.4, 0.0, 0.0),
                ),
                source_format="ti-mmwave-compressed",
                sdk_version="5.5.0.2",
                device_time_cycles=123456,
                frame_transition="consecutive",
                profile_id="sdk5502-test",
                capture_baudrate=1_250_000,
                heatmap=RadarHeatmap(
                    data=b"\x00\x40\x80\xff",
                    range_bins=2,
                    azimuth_bins=2,
                    range_step_m=0.05,
                    tlv_type=304,
                    motion_mode="major",
                    floor_db=10.0,
                    ceiling_db=50.0,
                ),
            ),
            ImuSample(
                header=header("imu/body"),
                specific_force_mps2=(0.0, 0.0, 9.80665),
                angular_velocity_radps=(0.1, 0.2, 0.3),
                temperature_c=24.5,
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                accel_covariance=(0.1,) * 9,
            ),
            WheelState(
                header=header("wheel/drive"),
                left_ticks=-4,
                right_ticks=5,
                sample_period_ns=50_000_000,
                left_angular_velocity_radps=-1.0,
                right_angular_velocity_radps=1.1,
            ),
            DropEvent(
                header=header("drop/events"),
                event_id="drop-1",
                released_unit_id="node1",
                actuator_unit_id="head",
                phase="physically_confirmed",
                anchor_keyframe_id="keyframe-8",
                keyframe_to_unit_xyyaw=(0.1, 0.2, 0.3),
                covariance_3x3=(0.01,) * 9,
                estimation_method="head_pose",
                confirmation_method="operator",
            ),
            SensorHealth(
                header=header("health/radar"),
                subject_stream_id="radar/front",
                status="ok",
                observed_rate_hz=10.0,
                last_sample_monotonic_ns=999_000_000,
                device_discontinuities_total=1,
            ),
        ]

    def test_all_record_types_round_trip_canonical_json(self):
        for record in self.records():
            with self.subTest(record=type(record).__name__):
                encoded = encode_sensor_record(record)
                decoded = decode_sensor_record(encoded)
                self.assertEqual(decoded, record)
                self.assertEqual(encode_sensor_record(decoded), encoded)
                self.assertNotIn(b"NaN", encoded)

    def test_zero_point_radar_frame_is_valid(self):
        record = RadarFrame(
            header=header(),
            frame_number=1,
            subframe_number=0,
            complete=True,
            dropped_frames_since_previous=0,
            points=(),
        )
        self.assertEqual(decode_sensor_record(encode_sensor_record(record)), record)

    def test_heatmap_bytes_use_base64_and_round_trip(self):
        record = self.records()[0]
        encoded = encode_sensor_record(record)
        wire = json.loads(encoded)

        self.assertEqual(
            wire["payload"]["heatmap"]["data_base64"],
            "AECA/w==",
        )
        self.assertEqual(wire["payload"]["heatmap"]["floor_db"], 10.0)
        self.assertEqual(wire["payload"]["heatmap"]["ceiling_db"], 50.0)
        self.assertNotIn("[0,64,128,255]", encoded.decode("ascii"))
        self.assertEqual(decode_sensor_record(encoded), record)

    def test_heatmap_shape_mode_and_base64_are_strict(self):
        with self.assertRaisesRegex(ValueError, "data length"):
            RadarHeatmap(
                data=b"\x00",
                range_bins=2,
                azimuth_bins=2,
                range_step_m=0.05,
                tlv_type=304,
                motion_mode="major",
                floor_db=0.0,
                ceiling_db=1.0,
            )
        with self.assertRaisesRegex(ValueError, "does not match"):
            RadarHeatmap(
                data=b"\x00",
                range_bins=1,
                azimuth_bins=1,
                range_step_m=0.05,
                tlv_type=305,
                motion_mode="major",
                floor_db=0.0,
                ceiling_db=1.0,
            )

        wire = record_to_dict(self.records()[0])
        wire["payload"]["heatmap"]["data_base64"] = "***"
        with self.assertRaisesRegex(ValueError, "valid base64"):
            decode_sensor_record(json.dumps(wire).encode("utf-8"))

    def test_anchor_updated_requires_keyframe_pose_and_covariance(self):
        with self.assertRaisesRegex(ValueError, "anchor_updated requires"):
            DropEvent(
                header=header("drop/events"),
                event_id="drop-1",
                released_unit_id="node1",
                actuator_unit_id="head",
                phase="anchor_updated",
            )
        event = DropEvent(
            header=header("drop/events"),
            event_id="drop-1",
            released_unit_id="node1",
            actuator_unit_id="head",
            phase="anchor_updated",
            anchor_keyframe_id="keyframe-8",
            keyframe_to_unit_xyyaw=(0.1, 0.2, 0.3),
            covariance_3x3=(0.01,) * 9,
        )
        self.assertEqual(event.anchor_keyframe_id, "keyframe-8")

    def test_numeric_values_are_canonical_after_direct_construction(self):
        point = RadarPoint(1, 2, 3, 4, 5, 6)
        self.assertIsInstance(point.x_m, float)
        record = RadarFrame(
            header=header(),
            frame_number=1,
            subframe_number=0,
            complete=True,
            dropped_frames_since_previous=0,
            points=(point,),
        )
        first = encode_sensor_record(record)
        second = encode_sensor_record(decode_sensor_record(first))
        self.assertEqual(first, second)

    def test_non_finite_numbers_are_rejected_on_create_and_decode(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    RadarPoint(value, 0.0, 0.0, 0.0)

        valid = record_to_dict(self.records()[0])
        valid["payload"]["points"][0]["x_m"] = float("nan")
        with self.assertRaises(ValueError):
            json.dumps(valid, allow_nan=False)
        raw = json.dumps(valid).encode("utf-8")
        with self.assertRaises(ValueError):
            decode_sensor_record(raw)

    def test_bool_is_not_accepted_as_integer_or_number(self):
        with self.assertRaises(ValueError):
            SensorHeader(
                mission_id="mission",
                unit_id="head",
                boot_id="boot",
                producer_id="producer",
                stream_id="imu/body",
                seq=True,
                monotonic_ns=1,
            )
        with self.assertRaises(ValueError):
            RadarPoint(True, 0.0, 0.0, 0.0)

    def test_imu_vector_and_covariance_lengths_are_strict(self):
        with self.assertRaises(ValueError):
            ImuSample(
                header=header("imu/body"),
                specific_force_mps2=(0.0, 0.0),
                angular_velocity_radps=(0.0, 0.0, 0.0),
            )
        with self.assertRaises(ValueError):
            ImuSample(
                header=header("imu/body"),
                specific_force_mps2=(0.0, 0.0, 1.0),
                angular_velocity_radps=(0.0, 0.0, 0.0),
                accel_covariance=(0.0,) * 8,
            )
        with self.assertRaises(ValueError):
            ImuSample(
                header=header("imu/body"),
                specific_force_mps2=(0.0, 0.0, 1.0),
                angular_velocity_radps=(0.0, 0.0, 0.0),
                orientation_xyzw=(0.0, 0.0, 0.0, 0.0),
            )

    def test_duplicate_json_keys_are_rejected(self):
        raw = (
            b'{"schema_version":1,"schema_version":1,'
            b'"record_type":"wheel_state","header":{},"payload":{}}'
        )
        with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
            decode_sensor_record(raw)

    def test_huge_integer_is_rejected_as_validation_error(self):
        raw = (
            b'{"schema_version":1,"record_type":"wheel_state","header":'
            b'{"mission_id":"m","unit_id":"head","boot_id":"b",'
            b'"producer_id":"p","stream_id":"wheel/drive","seq":1,'
            b'"monotonic_ns":'
            + b"9" * 256
            + b'},"payload":{"left_ticks":0,"right_ticks":0,'
            b'"sample_period_ns":1}}'
        )
        with self.assertRaisesRegex(ValueError, "integer exceeds"):
            decode_sensor_record(raw)

    def test_unknown_version_is_rejected(self):
        data = record_to_dict(self.records()[0])
        data["schema_version"] = 2
        with self.assertRaisesRegex(ValueError, "unsupported sensor schema"):
            decode_sensor_record(json.dumps(data).encode("utf-8"))

    def test_unknown_optional_field_is_ignored_within_version_one(self):
        original = self.records()[0]
        data = record_to_dict(original)
        data["payload"]["future_optional_field"] = {"value": 1}
        decoded = decode_sensor_record(json.dumps(data).encode("utf-8"))
        self.assertEqual(decoded, original)


if __name__ == "__main__":
    unittest.main()
