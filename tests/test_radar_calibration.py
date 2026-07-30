import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from common.radar_geometry import RadarAxes
from common.sensor_contract import (
    RadarFrame,
    RadarHeatmap,
    RadarPoint,
    SensorHeader,
)
from sensors.radar_calibration import (
    ClutterPointCluster,
    RadarClutterModel,
    build_clutter_model,
    load_clutter_model,
)
from sensors.mission_log import iter_replay


PROFILE_ID = "synthetic-profile"
REAL_PROFILE = (
    "lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1"
)
REAL_FIXTURE = (
    Path(__file__).parent
    / "fixtures"
    / "radar_clutter_20260728_f2286_f2291.jsonl"
)


def synthetic_calibration_frames(count: int = 50) -> tuple[RadarFrame, ...]:
    frames = []
    for index in range(count):
        jitter = (-0.004, 0.0, 0.004)[index % 3]
        heatmap_data = bytes(
            (
                20 + index % 3,
                40 + index % 5,
                60 + index % 7,
                80 + index % 2,
            )
        )
        frames.append(
            RadarFrame(
                header=SensorHeader(
                    mission_id="calibration-test",
                    unit_id="head",
                    boot_id="boot-1",
                    producer_id="radar-1",
                    stream_id="radar/front",
                    seq=index + 1,
                    monotonic_ns=index * 100_000_000,
                    frame_id="radar_native",
                ),
                frame_number=index,
                subframe_number=0,
                complete=True,
                dropped_frames_since_previous=0,
                points=(
                    RadarPoint(
                        x_m=-0.020 + jitter,
                        y_m=0.092 - jitter,
                        z_m=0.020,
                        radial_velocity_mps=0.0,
                    ),
                    RadarPoint(
                        x_m=0.041 - jitter,
                        y_m=0.131 + jitter,
                        z_m=-0.010,
                        radial_velocity_mps=0.0,
                    ),
                    RadarPoint(
                        x_m=0.120,
                        y_m=0.360,
                        z_m=0.0,
                        radial_velocity_mps=0.0,
                    ),
                ),
                source_format="synthetic",
                profile_id=PROFILE_ID,
                heatmap=RadarHeatmap(
                    data=heatmap_data,
                    range_bins=2,
                    azimuth_bins=2,
                    range_step_m=0.05,
                    tlv_type=304,
                    motion_mode="major",
                    floor_db=10.0,
                    ceiling_db=110.0,
                ),
            )
        )
    return tuple(frames)


def make_test_model() -> RadarClutterModel:
    model = RadarClutterModel(
        schema_version=1,
        calibration_id="",
        profile_id=PROFILE_ID,
        axes=RadarAxes(),
        range_bins=2,
        azimuth_bins=2,
        range_step_m=0.05,
        motion_mode="major",
        point_clusters=(
            ClutterPointCluster(0.092, -0.020, 0.020, 0.025, 1.0),
        ),
        heatmap_median_db=(20.0, 30.0, 40.0, 50.0),
        heatmap_mad_db=(1.0, 1.0, 1.0, 1.0),
    )
    return RadarClutterModel(
        **{
            **model.__dict__,
            "calibration_id": "radar-clutter-test",
        }
    )


def mismatched_bindings(
    model: RadarClutterModel,
) -> tuple[tuple[str, RadarHeatmap, RadarAxes], ...]:
    matching = RadarHeatmap(
        data=b"\x00" * 4,
        range_bins=2,
        azimuth_bins=2,
        range_step_m=0.05,
        tlv_type=304,
        motion_mode="major",
        floor_db=10.0,
        ceiling_db=110.0,
    )
    wrong_shape = RadarHeatmap(
        data=b"\x00" * 6,
        range_bins=3,
        azimuth_bins=2,
        range_step_m=0.05,
        tlv_type=304,
        motion_mode="major",
        floor_db=10.0,
        ceiling_db=110.0,
    )
    wrong_step = RadarHeatmap(
        data=b"\x00" * 4,
        range_bins=2,
        azimuth_bins=2,
        range_step_m=0.06,
        tlv_type=304,
        motion_mode="major",
        floor_db=10.0,
        ceiling_db=110.0,
    )
    wrong_mode = RadarHeatmap(
        data=b"\x00" * 4,
        range_bins=2,
        azimuth_bins=2,
        range_step_m=0.05,
        tlv_type=305,
        motion_mode="minor",
        floor_db=10.0,
        ceiling_db=110.0,
    )
    return (
        ("other-profile", matching, RadarAxes()),
        (model.profile_id, wrong_shape, RadarAxes()),
        (model.profile_id, wrong_step, RadarAxes()),
        (model.profile_id, wrong_mode, RadarAxes()),
        (
            model.profile_id,
            matching,
            RadarAxes(
                forward_axis="x",
                lateral_axis="y",
            ),
        ),
    )


def load_real_fixture() -> tuple[RadarFrame, ...]:
    return tuple(
        entry.record
        for entry in iter_replay(REAL_FIXTURE)
        if isinstance(entry.record, RadarFrame)
    )


def with_recomputed_calibration_id(
    payload: dict[str, object],
) -> dict[str, object]:
    without_id = dict(payload)
    del without_id["calibration_id"]
    canonical = json.dumps(
        without_id,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    payload["calibration_id"] = (
        "radar-clutter-" + hashlib.sha256(canonical).hexdigest()[:16]
    )
    return payload


class RadarClutterModelTests(unittest.TestCase):
    def assert_cluster_override_is_rejected(
        self,
        **override: float,
    ) -> None:
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        payload = model.to_dict()
        payload["point_clusters"][0].update(override)
        with_recomputed_calibration_id(payload)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_clutter_model(path)

    def test_build_is_deterministic_and_finds_persistent_near_clusters(self):
        frames = synthetic_calibration_frames(count=50)
        first = build_clutter_model(frames, RadarAxes(), min_frames=50)
        second = build_clutter_model(frames, RadarAxes(), min_frames=50)
        self.assertEqual(first.canonical_bytes(), second.canonical_bytes())
        self.assertEqual(first.calibration_id, second.calibration_id)
        self.assertEqual(len(first.point_clusters), 2)
        self.assertTrue(first.matches_point(0.092, -0.020, 0.020))
        self.assertFalse(first.matches_point(0.360, 0.120, 0.000))

    def test_binding_rejects_profile_shape_step_and_axes_mismatch(self):
        model = make_test_model()
        for profile, heatmap, axes in mismatched_bindings(model):
            with self.subTest(profile=profile, axes=axes.to_dict()):
                self.assertEqual(
                    model.binding_status(profile, heatmap, axes),
                    "profile_mismatch",
                )

    def test_requires_minimum_complete_heatmap_frames(self):
        with self.assertRaisesRegex(ValueError, "at least 50"):
            build_clutter_model(
                synthetic_calibration_frames(count=49),
                RadarAxes(),
                min_frames=50,
            )

    def test_real_fixture_preserves_profile_heatmap_and_close_clusters(self):
        frames = load_real_fixture()
        self.assertEqual(len(frames), 6)
        self.assertTrue(
            all(frame.profile_id == REAL_PROFILE for frame in frames)
        )
        self.assertTrue(
            all(frame.heatmap.range_bins == 128 for frame in frames)
        )
        self.assertTrue(
            all(frame.heatmap.azimuth_bins == 16 for frame in frames)
        )
        self.assertTrue(
            all(
                frame.heatmap.range_step_m == 0.09765625
                for frame in frames
            )
        )
        self.assertLess(REAL_FIXTURE.stat().st_size, 40 * 1024)

    def test_load_round_trips_a_canonical_model(self):
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_bytes(model.canonical_bytes() + b"\n")
            loaded = load_clutter_model(path)
        self.assertEqual(loaded, model)
        self.assertEqual(loaded.canonical_bytes(), model.canonical_bytes())

    def test_load_rejects_malformed_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "invalid"):
                load_clutter_model(path)

    def test_load_rejects_duplicate_keys(self):
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        duplicate = model.canonical_bytes().replace(
            b'{"axes":',
            b'{"schema_version":1,"axes":',
            1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_bytes(duplicate)
            with self.assertRaisesRegex(ValueError, "duplicate"):
                load_clutter_model(path)

    def test_load_rejects_nonfinite_numbers(self):
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        payload = model.to_dict()
        payload["range_step_m"] = float("nan")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "nonfinite"):
                load_clutter_model(path)

    def test_load_rejects_wrong_schema(self):
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        payload = model.to_dict()
        payload["schema_version"] = 2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "schema"):
                load_clutter_model(path)

    def test_load_rejects_wrong_heatmap_vector_length(self):
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        payload = model.to_dict()
        payload["heatmap_mad_db"] = payload["heatmap_mad_db"][:-1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "length"):
                load_clutter_model(path)

    def test_load_rejects_inconsistent_calibration_id(self):
        model = build_clutter_model(
            synthetic_calibration_frames(),
            RadarAxes(),
        )
        payload = model.to_dict()
        payload["profile_id"] = "tampered-profile"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "calibration_id"):
                load_clutter_model(path)

    def test_load_rejects_cluster_radius_below_builder_minimum(self):
        self.assert_cluster_override_is_rejected(radius_m=0.024999)

    def test_load_rejects_cluster_radius_above_builder_maximum(self):
        self.assert_cluster_override_is_rejected(radius_m=0.080001)

    def test_load_rejects_cluster_below_persistence_minimum(self):
        self.assert_cluster_override_is_rejected(
            observation_fraction=0.599999
        )

    def test_load_rejects_cluster_center_outside_near_range(self):
        self.assert_cluster_override_is_rejected(
            forward_m=0.150001,
            lateral_m=0.0,
            height_m=0.0,
        )


if __name__ == "__main__":
    unittest.main()
