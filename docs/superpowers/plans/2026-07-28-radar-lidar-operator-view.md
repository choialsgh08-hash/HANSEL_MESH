# HANSEL Radar LiDAR Operator View Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the misleading camera-like IWRL6432 view with a calibrated, robot-relative 3 m LiDAR-style occupancy view, a 50 cm collision inset, and a confirmed-point-only 10 cm danger warning.

**Architecture:** Keep the existing single-owner UART and canonical mission-log pipeline. Add a deterministic profile-bound clutter model in `sensors`, a testable scene/grid/track/hazard estimator in `monitor`, then make the browser a presentation-only consumer of the versioned `scene` API. The first release is robot-relative and never invents free space, wall height, or surfaces; encoder/IMU motion compensation remains a later contract-compatible phase.

**Tech Stack:** Python 3.9+ standard library, `unittest`, canonical JSONL sensor contracts, browser Canvas 2D, dependency-free JavaScript, Node built-in `node:test`, Windows PowerShell for bench operation.

## Global Constraints

- Main map extent is forward `0..3.0 m`, lateral `-1.5..+1.5 m`, at `0.05 m` grid resolution.
- Collision inset extent is `0..0.50 m`; distance-based red enters only at a confirmed non-clutter point track `<= 0.10 m`.
- Danger release hysteresis is `>= 0.13 m`; an unobserved danger track expires at `300 ms`.
- A point track is confirmed by at least two spatially associated point observations in the latest three frames.
- Heatmap bins 0/1 alone never create `DANGER`.
- Grid byte `0` means `UNKNOWN`; `1..255` means occupied/return evidence. No byte means `FREE` or `SAFE`.
- Grid layout is `forward-major_lateral-minor`, index `forward_index * lateral_cells + lateral_index`.
- Heatmap data never creates height. Only point-cloud `z` may populate `height_m`.
- `NORMAL` means only “no confirmed point inside 10 cm”; it never means the path is safe or free.
- Zero returns and heatmap-only returns remain `UNKNOWN`; they never become `NORMAL`.
- Missing/mismatched calibration is fail-closed as `UNKNOWN`; stale/fault/replay-end is a blocking stop overlay.
- The clutter model has its own `calibration_id`; it never reuses `SensorHeader.calibration_id`.
- Calibration binding includes profile ID, heatmap shape, range step, motion mode, and axis signature.
- Exact `3.0 m` evidence is included by clamping it to the final forward cell; evidence beyond `3.0 m` is excluded.
- Producer changes, device discontinuities, and replay loops reset scene tracks and hazard hysteresis.
- No external browser libraries, CDNs, Python dependencies, or motor-control side effects.
- Preserve unrelated dirty files. Every commit stages only the exact files listed by its task.
- Follow TDD for every behavior change: add a failing test, observe the intended failure, implement minimally, then rerun focused and broader tests.

---

### Task 1: Freeze the verified IWRL6432 R8 foundation as a baseline

**Files:**
- Stage only: `common/sensor_contract.py`
- Stage only: `sensors/cli.py`
- Stage only: `sensors/radar_capture.py`
- Stage only: `sensors/raw_capture_index.py`
- Stage only: `sensors/ti_mmwave.py`
- Stage only: `monitor/radar_front.py`
- Stage only: `monitor/web/radar_front.html`
- Stage only: `monitor/web/radar_panel.js`
- Stage only: `configs/radar/`
- Stage only: `scripts/configure_ti_radar.py`
- Stage only: `tests/test_configure_ti_radar.py`
- Stage only: `tests/test_radar_capture.py`
- Stage only: `tests/test_radar_front.py`
- Stage only: `tests/test_raw_capture_index.py`
- Stage only: `tests/test_sensor_cli.py`
- Stage only: `tests/test_sensor_contract.py`
- Stage only: `tests/test_ti_mmwave.py`
- Stage only: `docs/phase1_sensor_foundation.md`
- Stage only: `docs/radar_front_view.md`
- Stage only: `docs/radar_reconnect_windows.md`

**Interfaces:**
- Consumes: the already-tested R8 UART/heatmap/point-cloud implementation in the working tree.
- Produces: one baseline commit so later task commits contain only R9 behavior changes.

- [ ] **Step 1: Record the exact unrelated files that must stay uncommitted**

Run:

```powershell
git status --short
```

Expected: `LICENSE`, `cripts`, the presentation PDF, `monitor_session.jsonl`, and `video_quality.jsonl` remain dirty but are not in the task staging list.

- [ ] **Step 2: Run the full current sensor/radar baseline**

Run:

```powershell
python -m unittest `
  tests.test_sensor_contract `
  tests.test_ti_mmwave `
  tests.test_radar_capture `
  tests.test_raw_capture_index `
  tests.test_sensor_cli `
  tests.test_configure_ti_radar `
  tests.test_radar_front -v
```

Expected: all existing tests pass with exit code `0`.

- [ ] **Step 3: Stage only the verified radar foundation**

Run:

```powershell
git add -- `
  common/sensor_contract.py `
  sensors/cli.py sensors/radar_capture.py sensors/raw_capture_index.py sensors/ti_mmwave.py `
  monitor/radar_front.py monitor/web/radar_front.html monitor/web/radar_panel.js `
  configs/radar scripts/configure_ti_radar.py `
  tests/test_configure_ti_radar.py tests/test_radar_capture.py tests/test_radar_front.py `
  tests/test_raw_capture_index.py tests/test_sensor_cli.py tests/test_sensor_contract.py tests/test_ti_mmwave.py `
  docs/phase1_sensor_foundation.md docs/radar_front_view.md docs/radar_reconnect_windows.md
git diff --cached --check
git diff --cached --name-only
```

Expected: only paths named above are staged; no log, video-quality, PDF, `LICENSE`, or `cripts` path appears.

- [ ] **Step 4: Commit the foundation**

Run:

```powershell
git commit -m "feat: add IWRL6432 radar capture and R8 operator foundation"
```

Expected: commit succeeds while unrelated dirty files remain untouched.

---

### Task 2: Add deterministic profile-bound clutter calibration and CLI

**Files:**
- Create: `common/radar_geometry.py`
- Create: `sensors/radar_calibration.py`
- Create: `tests/test_radar_calibration.py`
- Create: `tests/fixtures/radar_clutter_20260728_f2286_f2291.jsonl`
- Modify: `sensors/cli.py`
- Modify: `tests/test_sensor_cli.py`

**Interfaces:**
- Consumes: canonical `RadarFrame`, `RadarHeatmap`, `RadarPoint`, `iter_replay`, and TI native `x/y/z`.
- Produces:
  - `RadarAxes(forward_axis="y", forward_sign=1, lateral_axis="x", lateral_sign=1)`
  - `RadarClutterModel.to_dict() -> dict`
  - `RadarClutterModel.canonical_bytes() -> bytes`
  - `RadarClutterModel.binding_status(profile_id, heatmap, axes) -> str`
  - `RadarClutterModel.matches_point(forward_m, lateral_m, height_m) -> bool`
  - `build_clutter_model(frames, axes, min_frames=50) -> RadarClutterModel`
  - `load_clutter_model(path) -> RadarClutterModel`
  - CLI: `python -m sensors radar-calibrate INPUT.jsonl --output MODEL.json`

- [ ] **Step 1: Write failing calibration-model tests**

Add tests with these public behaviors:

```python
class RadarClutterModelTests(unittest.TestCase):
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
```

- [ ] **Step 2: Run the new test and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_calibration -v
```

Expected: import failure for missing `sensors.radar_calibration`.

- [ ] **Step 3: Implement the calibration data model and builder**

Put the axis transform in `common/radar_geometry.py` so capture/calibration
code never imports the monitor package. `monitor.radar_front` re-exports this
same class for backward compatibility:

```python
@dataclass(frozen=True)
class RadarAxes:
    forward_axis: str = "y"
    forward_sign: int = 1
    lateral_axis: str = "x"
    lateral_sign: int = 1

    def __post_init__(self) -> None:
        if {self.forward_axis, self.lateral_axis} != {"x", "y"}:
            raise ValueError("forward_axis and lateral_axis must be x/y and differ")
        if self.forward_sign not in {-1, 1} or self.lateral_sign not in {-1, 1}:
            raise ValueError("axis signs must be -1 or 1")

    def map_point(self, point: RadarPoint) -> Tuple[float, float]:
        values = {"x": point.x_m, "y": point.y_m}
        return (
            values[self.forward_axis] * self.forward_sign,
            values[self.lateral_axis] * self.lateral_sign,
        )

    def to_dict(self) -> Dict[str, object]:
        return {
            "forward_axis": self.forward_axis,
            "forward_sign": self.forward_sign,
            "lateral_axis": self.lateral_axis,
            "lateral_sign": self.lateral_sign,
            "lateral_positive": "right",
            "frame": "robot_relative_uncalibrated",
        }

    def signature(self) -> str:
        return (
            f"{self.forward_sign:+d}{self.forward_axis}:forward,"
            f"{self.lateral_sign:+d}{self.lateral_axis}:right"
        )
```

Implement these exact immutable calibration fields:

```python


@dataclass(frozen=True)
class ClutterPointCluster:
    forward_m: float
    lateral_m: float
    height_m: float
    radius_m: float
    observation_fraction: float


@dataclass(frozen=True)
class RadarClutterModel:
    schema_version: int
    calibration_id: str
    profile_id: str
    axes: RadarAxes
    range_bins: int
    azimuth_bins: int
    range_step_m: float
    motion_mode: str
    point_clusters: Tuple[ClutterPointCluster, ...]
    heatmap_median_db: Tuple[float, ...]
    heatmap_mad_db: Tuple[float, ...]

    def to_dict(self) -> Dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "calibration_id": self.calibration_id,
            "profile_id": self.profile_id,
            "axes": self.axes.to_dict(),
            "range_bins": self.range_bins,
            "azimuth_bins": self.azimuth_bins,
            "range_step_m": self.range_step_m,
            "motion_mode": self.motion_mode,
            "point_clusters": [asdict(value) for value in self.point_clusters],
            "heatmap_median_db": list(self.heatmap_median_db),
            "heatmap_mad_db": list(self.heatmap_mad_db),
        }

    def canonical_bytes(self) -> bytes:
        return json.dumps(
            self.to_dict(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")

    def binding_status(
        self,
        profile_id: Optional[str],
        heatmap: Optional[RadarHeatmap],
        axes: RadarAxes,
    ) -> str:
        if heatmap is None:
            return "heatmap_missing"
        matches = (
            profile_id == self.profile_id
            and heatmap.range_bins == self.range_bins
            and heatmap.azimuth_bins == self.azimuth_bins
            and math.isclose(
                heatmap.range_step_m,
                self.range_step_m,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            and heatmap.motion_mode == self.motion_mode
            and axes.signature() == self.axes.signature()
        )
        return "ok" if matches else "profile_mismatch"

    def matches_point(
        self,
        forward_m: float,
        lateral_m: float,
        height_m: float,
    ) -> bool:
        return any(
            math.dist(
                (forward_m, lateral_m, height_m),
                (cluster.forward_m, cluster.lateral_m, cluster.height_m),
            )
            <= cluster.radius_m
            for cluster in self.point_clusters
        )
```

Builder rules:

```python
POINT_VOXEL_M = 0.02
POINT_CLUSTER_MIN_FRACTION = 0.60
POINT_CLUSTER_MIN_RADIUS_M = 0.025
POINT_CLUSTER_MAX_RADIUS_M = 0.08
POINT_CALIBRATION_MAX_RANGE_M = 0.15
```

Convert each heatmap byte back to the recorded relative dB scale with
`floor_db + value / 255 * (ceiling_db - floor_db)`. Store per-cell median
and median absolute deviation. Compute `calibration_id` as
`"radar-clutter-" + sha256(payload_without_id).hexdigest()[:16]`.

- [ ] **Step 4: Run calibration tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_radar_calibration -v
```

Expected: all calibration model tests pass.

- [ ] **Step 5: Add the real six-frame fixture and fixture integrity test**

Extract canonical `RadarFrame` records 2286 through 2291 from
`missions/radar-r8-20260728-165843.jsonl`, renumber only `log_seq` to `1..6`,
and keep the original point and `128 x 16` heatmap payloads.

Add:

```python
def test_real_fixture_preserves_profile_heatmap_and_close_clusters(self):
    frames = load_real_fixture()
    self.assertEqual(len(frames), 6)
    self.assertTrue(all(frame.profile_id == REAL_PROFILE for frame in frames))
    self.assertTrue(all(frame.heatmap.range_bins == 128 for frame in frames))
    self.assertTrue(all(frame.heatmap.azimuth_bins == 16 for frame in frames))
    self.assertTrue(
        all(frame.heatmap.range_step_m == 0.09765625 for frame in frames)
    )
    self.assertLess(REAL_FIXTURE.stat().st_size, 40 * 1024)
```

- [ ] **Step 6: Run the fixture test and verify RED, then create the fixture**

Run before extraction:

```powershell
python -m unittest `
  tests.test_radar_calibration.RadarClutterModelTests.test_real_fixture_preserves_profile_heatmap_and_close_clusters -v
```

Expected before file creation: failure because the fixture is missing.

Run after extraction: the test passes.

- [ ] **Step 7: Write failing CLI tests**

Add this helper and four concrete assertions:

```python
def invoke_calibrate(self, input_path, output_path, *extra):
    argv = [
        "sensors", "radar-calibrate", str(input_path),
        "--output", str(output_path), *extra,
    ]
    stdout = io.StringIO()
    stderr = io.StringIO()
    with mock.patch("sys.argv", argv), \
            contextlib.redirect_stdout(stdout), \
            contextlib.redirect_stderr(stderr):
        try:
            result = main()
        except SystemExit as error:
            result = error.code
    return result, stdout.getvalue(), stderr.getvalue()

def test_radar_calibrate_writes_deterministic_profile_bound_model(self):
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "model.json"
        result, stdout, _ = self.invoke_calibrate(
            REAL_FIXTURE, output, "--min-frames", "6",
        )
        self.assertEqual(result, 0)
        self.assertEqual(
            json.loads(output.read_text("utf-8"))["profile_id"],
            REAL_PROFILE,
        )
        self.assertEqual(json.loads(stdout)["frames_used"], 6)

def test_radar_calibrate_refuses_existing_output_without_overwrite(self):
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "model.json"
        output.write_text("owned", encoding="utf-8")
        result, _, stderr = self.invoke_calibrate(
            REAL_FIXTURE, output, "--min-frames", "6",
        )
        self.assertNotEqual(result, 0)
        self.assertEqual(output.read_text("utf-8"), "owned")
        self.assertIn("--overwrite", stderr)

def test_radar_calibrate_rejects_mixed_profiles(self):
    result, _, stderr = self.invoke_calibrate(
        MIXED_PROFILE_FIXTURE, self.output, "--min-frames", "6",
    )
    self.assertNotEqual(result, 0)
    self.assertIn("mixed profile", stderr)

def test_radar_calibrate_rejects_insufficient_frames(self):
    result, _, stderr = self.invoke_calibrate(
        REAL_FIXTURE, self.output, "--min-frames", "7",
    )
    self.assertNotEqual(result, 0)
    self.assertIn("at least 7", stderr)
```

Each test calls `sensors.cli.main()` with patched `sys.argv`,
`TemporaryDirectory`, and captured stdout/stderr. Use `--min-frames 6` for
the real fixture; production default remains `50`.

- [ ] **Step 8: Run CLI tests and verify RED**

Run:

```powershell
python -m unittest `
  tests.test_sensor_cli.SensorCliTests.test_radar_calibrate_writes_deterministic_profile_bound_model `
  tests.test_sensor_cli.SensorCliTests.test_radar_calibrate_refuses_existing_output_without_overwrite `
  tests.test_sensor_cli.SensorCliTests.test_radar_calibrate_rejects_mixed_profiles `
  tests.test_sensor_cli.SensorCliTests.test_radar_calibrate_rejects_insufficient_frames -v
```

Expected: parser rejects unknown `radar-calibrate`.

- [ ] **Step 9: Implement the `radar-calibrate` subcommand**

Parser contract:

```python
calibrate = subparsers.add_parser(
    "radar-calibrate",
    help="build a deterministic profile-bound radar self-clutter model",
)
calibrate.add_argument("path")
calibrate.add_argument("--output", required=True)
calibrate.add_argument("--min-frames", type=int, default=50)
calibrate.add_argument("--forward-axis", choices=("x", "y"), default="y")
calibrate.add_argument("--forward-sign", type=int, choices=(-1, 1), default=1)
calibrate.add_argument("--lateral-axis", choices=("x", "y"), default="x")
calibrate.add_argument("--lateral-sign", type=int, choices=(-1, 1), default=1)
calibrate.add_argument("--overwrite", action="store_true")
calibrate.set_defaults(func=command_radar_calibrate)
```

`command_radar_calibrate` must read only complete `RadarFrame` records with
heatmaps, reject mixed profile/shape/range-step input, refuse an existing
output unless `--overwrite`, write `canonical_bytes() + b"\n"` atomically,
and print a sorted JSON summary without absolute paths or wall-clock time.
Use a sibling temporary file, flush and `os.fsync`, then `os.replace`; delete
the temporary file if validation or replacement fails.

- [ ] **Step 10: Run focused and sensor CLI tests**

Run:

```powershell
python -m unittest tests.test_radar_calibration tests.test_sensor_cli -v
```

Expected: all tests pass.

- [ ] **Step 11: Commit the calibration slice**

Run:

```powershell
git add -- `
  common/radar_geometry.py sensors/radar_calibration.py sensors/cli.py `
  tests/test_radar_calibration.py tests/test_sensor_cli.py `
  tests/fixtures/radar_clutter_20260728_f2286_f2291.jsonl
git diff --cached --check
git diff --cached --name-only
git commit -m "feat: add deterministic radar clutter calibration"
```

Expected: only the six named paths are committed.

---

### Task 3: Build the robot-relative scene grid, tracks, and 10 cm hazard evaluator

**Files:**
- Create: `monitor/radar_scene.py`
- Create: `tests/test_radar_scene.py`

**Interfaces:**
- Consumes: `RadarAxes`, `RadarClutterModel`, canonical `RadarFrame`, and a monotonic clock.
- Produces:
  - `RadarSceneEstimator(axes, clutter_model=None, require_calibration=True, clock=time.monotonic)`
  - `RadarSceneEstimator.ingest(frame, received_at=None) -> None`
  - `RadarSceneEstimator.snapshot(source_status="live", now=None) -> dict`
  - `RadarHazardEvaluator.update(tracks, calibration_status, source_status, now) -> dict`

- [ ] **Step 1: Write failing scene/grid tests**

Add:

```python
class RadarSceneEstimatorTests(unittest.TestCase):
    def test_three_metre_grid_retains_049_and_299_but_rejects_301(self):
        estimator = calibrated_estimator()
        estimator.ingest(frame_with_forward_ranges(0.49, 2.99, 3.01))
        scene = estimator.snapshot()
        distances = [track["distance_m"] for track in scene["tracks"]]
        self.assertTrue(any(abs(value - 0.49) < 0.01 for value in distances))
        self.assertTrue(any(abs(value - 2.99) < 0.01 for value in distances))
        self.assertFalse(any(value > 3.0 for value in distances))
        grid = base64.b64decode(scene["grid"]["data_base64"])
        self.assertEqual(len(grid), 60 * 60)
        self.assertGreater(grid[59 * 60 + 30], 0)

    def test_grid_zero_is_unknown_and_layout_is_explicit(self):
        scene = calibrated_estimator().snapshot()
        self.assertEqual(scene["grid"]["unknown_value"], 0)
        self.assertEqual(
            scene["grid"]["layout"],
            "forward-major_lateral-minor",
        )
        self.assertEqual(set(base64.b64decode(scene["grid"]["data_base64"])), {0})

    def test_heatmap_never_invents_height(self):
        estimator = calibrated_estimator()
        estimator.ingest(heatmap_only_frame(range_index=4, azimuth_index=8))
        track = estimator.snapshot()["tracks"][0]
        self.assertEqual(track["source"], "heatmap")
        self.assertIsNone(track["height_m"])
        self.assertAlmostEqual(track["range_uncertainty_m"], 0.09765625)
```

- [ ] **Step 2: Run scene tests and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_scene -v
```

Expected: import failure for missing `monitor.radar_scene`.

- [ ] **Step 3: Implement scene types, point filtering, heatmap residuals, and grid**

Use exact constants:

```python
SCENE_SCHEMA_VERSION = 1
GRID_RESOLUTION_M = 0.05
GRID_FORWARD_CELLS = 60
GRID_LATERAL_CELLS = 60
GRID_ORIGIN_FORWARD_CELL = 0
GRID_ORIGIN_LATERAL_CELL = 30
SCENE_MAX_FORWARD_M = 3.0
SCENE_HALF_WIDTH_M = 1.5
TRACK_ASSOCIATION_M = 0.18
TRACK_TTL_S = 0.300
HEATMAP_MIN_RESIDUAL_DB = 6.0
HEATMAP_MAD_MULTIPLIER = 4.0
```

Return this grid metadata on every snapshot:

```python
{
    "resolution_m": 0.05,
    "forward_cells": 60,
    "lateral_cells": 60,
    "origin_forward_cell": 0,
    "origin_lateral_cell": 30,
    "encoding": "occupancy-u8-base64",
    "layout": "forward-major_lateral-minor",
    "unknown_value": 0,
    "data_base64": base64.b64encode(bytes(grid)).decode("ascii"),
}
```

For calibrated heatmaps, accept a cell only when:

```python
residual_db >= max(
    HEATMAP_MIN_RESIDUAL_DB,
    HEATMAP_MAD_MULTIPLIER * calibration_mad_db,
)
```

Never use range index `0`. Range index `1` may create occupancy/track evidence
but never point-confirmed danger. Map FFT-shifted azimuth with
`asin(2 * (index - bins / 2) / bins)` and clip to `-70..+70` degrees.

- [ ] **Step 4: Run scene/grid tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_radar_scene -v
```

Expected: scene/grid tests pass.

- [ ] **Step 5: Write failing tracking and hazard tests**

Add:

```python
def test_hazard_enters_at_10cm_after_two_of_three_and_releases_at_13cm(self):
    estimator = calibrated_estimator()
    estimator.ingest(point_frame(0.099), received_at=10.0)
    self.assertEqual(estimator.snapshot(now=10.0)["hazard"]["level"], "UNKNOWN")
    estimator.ingest(point_frame(0.098), received_at=10.1)
    self.assertEqual(estimator.snapshot(now=10.1)["hazard"]["level"], "DANGER")
    estimator.ingest(point_frame(0.129), received_at=10.2)
    self.assertEqual(estimator.snapshot(now=10.2)["hazard"]["level"], "DANGER")
    estimator.ingest(point_frame(0.131), received_at=10.3)
    self.assertEqual(estimator.snapshot(now=10.3)["hazard"]["level"], "NORMAL")

def test_track_is_present_before_but_not_at_300ms(self):
    estimator = calibrated_estimator()
    estimator.ingest(point_frame(0.40), received_at=20.0)
    self.assertTrue(estimator.snapshot(now=20.299999999)["tracks"])
    self.assertFalse(estimator.snapshot(now=20.300000000)["tracks"])

def test_saturated_heatmap_bins_zero_and_one_never_enter_danger(self):
    estimator = calibrated_estimator()
    for index in range(3):
        estimator.ingest(
            saturated_near_heatmap_frame(index),
            received_at=30.0 + index * 0.1,
        )
    hazard = estimator.snapshot(now=30.2)["hazard"]
    self.assertNotEqual(hazard["level"], "DANGER")
    self.assertIsNone(hazard["nearest_confirmed_m"])

def test_missing_and_mismatched_calibration_are_unknown(self):
    for estimator in (uncalibrated_estimator(), mismatched_estimator()):
        estimator.ingest(point_frame(0.30))
        scene = estimator.snapshot()
        self.assertIn(
            scene["calibration_status"],
            {"calibration_required", "profile_mismatch"},
        )
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")

def test_zero_returns_and_heatmap_only_evidence_remain_unknown(self):
    estimator = calibrated_estimator()
    estimator.ingest(empty_frame(), received_at=40.0)
    self.assertEqual(estimator.snapshot(now=40.0)["hazard"]["level"], "UNKNOWN")
    estimator.ingest(heatmap_only_frame(4, 8), received_at=40.1)
    self.assertEqual(estimator.snapshot(now=40.1)["hazard"]["level"], "UNKNOWN")
```

- [ ] **Step 6: Run hazard tests and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_scene.RadarSceneEstimatorTests -v
```

Expected: failures because tracks lack two-of-three confirmation, expiry, and
hazard hysteresis.

- [ ] **Step 7: Implement tracks and `RadarHazardEvaluator`**

Track dictionary contract:

```python
{
    "track_id": 1,
    "forward_m": 0.36,
    "lateral_m": -0.12,
    "height_m": None,
    "distance_m": 0.38,
    "range_uncertainty_m": 0.09765625,
    "confidence": 0.82,
    "source": "heatmap",
    "point_confirmed": False,
    "age_ms": 76,
}
```

Hazard evaluator contract:

```python
DANGER_ENTER_M = 0.10
DANGER_EXIT_M = 0.13

{
    "level": "DANGER" | "NORMAL" | "UNKNOWN" | "SENSOR_FAULT",
    "nearest_confirmed_m": 0.098 | None,
    "threshold_m": 0.10,
    "release_m": 0.13,
    "reason": "confirmed_point_inside_threshold",
}
```

Only a `point_confirmed` track with two hits in the current three-frame hit
window may enter danger. Keep the same danger `track_id` until it reaches
`0.13 m` or expires at exactly `300 ms`.

Treat exact `3.0 m` point evidence as the final forward cell and drop only
values greater than `3.0 m`. Add `reset(reason)` and invoke it on producer
change, `duplicate`/`reset_or_out_of_order`, and replay-loop restart.

- [ ] **Step 8: Run focused and real-clutter regression tests**

Add and run:

```python
def test_real_977cm_clutter_is_rejected_without_erasing_transient_points(self):
    model = build_clutter_model(load_real_fixture(), RadarAxes(), min_frames=6)
    estimator = RadarSceneEstimator(RadarAxes(), model)
    kept = rejected = danger_frames = 0
    for frame in load_real_fixture():
        estimator.ingest(frame)
        scene = estimator.snapshot()
        kept += scene["diagnostics"]["scene_point_count"]
        rejected += scene["diagnostics"]["clutter_points_rejected"]
        danger_frames += scene["hazard"]["level"] == "DANGER"
    self.assertGreater(rejected, 0)
    self.assertEqual(danger_frames, 0)
    self.assertGreater(kept, 0)
```

Run:

```powershell
python -m unittest tests.test_radar_scene tests.test_radar_calibration -v
```

Expected: all scene and calibration tests pass.

- [ ] **Step 9: Commit the scene estimator**

Run:

```powershell
git add -- monitor/radar_scene.py tests/test_radar_scene.py
git diff --cached --check
git diff --cached --name-only
git commit -m "feat: estimate calibrated radar occupancy and hazards"
```

---

### Task 4: Integrate the scene estimator into `RadarFrontState` and `/api/radar`

**Files:**
- Modify: `monitor/radar_front.py`
- Modify: `tests/test_radar_front.py`

**Interfaces:**
- Consumes: `RadarSceneEstimator`, optional `RadarClutterModel`, existing mission follower/replay/demo.
- Produces:
  - `/api/radar.scene` with schema version `1`
  - CLI option `--clutter-calibration PATH`
  - demo-only trusted status `synthetic`
  - UI build ID `20260728-lidar-operator-r9`

- [ ] **Step 1: Write failing state/API integration tests**

Add:

```python
def test_snapshot_exposes_scene_and_retains_three_metre_point(self):
    state = RadarFrontState(
        "test",
        max_range_m=3.0,
        scene_estimator=calibrated_scene_estimator(),
        clock=FakeClock(),
    )
    state.ingest(radar_frame([RadarPoint(0.0, 2.99, 0.0, 0.0)]))
    snapshot = state.snapshot()
    self.assertEqual(snapshot["scene"]["schema_version"], 1)
    self.assertTrue(
        any(track["distance_m"] > 2.9 for track in snapshot["scene"]["tracks"])
    )

def test_state_status_fault_overrides_scene_hazard(self):
    clock = FakeClock()
    state = calibrated_state(clock=clock)
    state.ingest(radar_frame([RadarPoint(0.0, 0.09, 0.0, 0.0)]))
    clock.value += 2.1
    scene = state.snapshot()["scene"]
    self.assertEqual(scene["hazard"]["level"], "SENSOR_FAULT")

def test_cli_accepts_clutter_calibration_path(self):
    args = parse_args(["--demo", "--clutter-calibration", "model.json"])
    self.assertEqual(args.clutter_calibration, "model.json")

def test_demo_scene_is_explicitly_synthetic_not_calibration_required(self):
    state = RadarFrontState("demo", clock=FakeClock())
    state.ingest(make_demo_frame(1, 1, seed=6432))
    self.assertEqual(
        state.snapshot()["scene"]["calibration_status"],
        "synthetic",
    )
```

- [ ] **Step 2: Run integration tests and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_front -v
```

Expected: failures for missing `scene`, constructor argument, and CLI option.

- [ ] **Step 3: Integrate scene ingestion and snapshot status**

Import `RadarAxes` from `common.radar_geometry` in both
`sensors.radar_calibration` and `monitor.radar_front`; the latter import
continues to re-export the same name so older imports keep working.

Change:

```python
UI_BUILD_ID = "20260728-lidar-operator-r9"

class RadarFrontState:
    def __init__(
        self,
        source_mode: str,
        axes: RadarAxes = RadarAxes(),
        max_points: int = 2048,
        max_range_m: float = 3.0,
        min_forward_m: float = 0.0,
        history_window_s: float = 0.3,
        stale_after_s: float = 0.75,
        fault_after_s: float = 2.0,
        clock: Callable[[], float] = time.monotonic,
        scene_estimator: Optional[RadarSceneEstimator] = None,
    ) -> None:
        self.scene_estimator = scene_estimator or RadarSceneEstimator(
            axes=axes,
            clutter_model=None,
            require_calibration=(source_mode != "demo"),
            synthetic=(source_mode == "demo"),
            clock=clock,
        )
```

For every complete frame, pass the original canonical frame to
`scene_estimator.ingest(record, received_at=now)` before display-only point
truncation. In `snapshot`, determine source status first, then call
`scene_estimator.snapshot(source_status=status, now=current)`.

Add parser option:

```python
parser.add_argument(
    "--clutter-calibration",
    help="profile-bound radar self-clutter calibration JSON",
)
```

`run()` loads it once with `load_clutter_model`. File or schema errors must
return through the existing CLI error path; profile mismatch is a live
fail-closed scene state rather than a process crash.

- [ ] **Step 4: Run focused integration and full Python radar tests**

Run:

```powershell
python -m unittest `
  tests.test_radar_front `
  tests.test_radar_scene `
  tests.test_radar_calibration -v
```

Expected: all tests pass.

- [ ] **Step 5: Commit API integration**

Run:

```powershell
git add -- monitor/radar_front.py tests/test_radar_front.py
git diff --cached --check
git diff --cached --name-only
git commit -m "feat: expose calibrated radar scene API"
```

---

### Task 5: Add a dependency-free browser scene contract module

**Files:**
- Create: `monitor/web/radar_scene.js`
- Create: `tests/web/radar_scene.test.js`

**Interfaces:**
- Consumes: `/api/radar` snapshot and `scene` schema version `1`.
- Produces:
  - `parseRadarScene(snapshot) -> presentation`
  - `decodeOccupancyGrid(grid) -> Uint8Array`
  - `filterFreshTracks(tracks, maxAgeMs=300) -> Array`
  - `makeMapTransform(width, height, forwardMaxM, halfWidthM) -> object`
  - `projectMapPoint(transform, forwardM, lateralM) -> {x, y}`

- [ ] **Step 1: Write failing Node contract tests**

Create:

```javascript
const test = require("node:test");
const assert = require("node:assert/strict");
const sceneApi = require("../../monitor/web/radar_scene.js");

test("decodes forward-major occupancy and preserves zero as UNKNOWN", () => {
  const grid = makeGrid(60, 60);
  grid.bytes[59 * 60 + 30] = 255;
  const decoded = sceneApi.decodeOccupancyGrid(grid.payload);
  assert.equal(decoded.length, 3600);
  assert.equal(decoded[0], 0);
  assert.equal(decoded[59 * 60 + 30], 255);
});

test("rejects a grid whose base64 length does not match dimensions", () => {
  assert.throws(() => sceneApi.decodeOccupancyGrid(badGrid()), /length/);
});

test("drops a track older than 300ms", () => {
  assert.deepEqual(
    sceneApi.filterFreshTracks([{ track_id: 1, age_ms: 301 }], 300),
    [],
  );
});

test("blocks inconsistent danger without a confirmed point inside 10cm", () => {
  assert.throws(
    () => sceneApi.parseRadarScene(inconsistentDangerSnapshot()),
    /DANGER contract/,
  );
});

test("normal never becomes safe or free", () => {
  const parsed = sceneApi.parseRadarScene(normalSnapshot());
  assert.equal(parsed.hazardCopy, "10cm 이내 확인 장애물 없음 · 미관측 영역 존재");
  assert.equal(parsed.safe, undefined);
});
```

- [ ] **Step 2: Run Node tests and verify RED**

Run:

```powershell
node --test tests/web/radar_scene.test.js
```

Expected: module-not-found failure.

- [ ] **Step 3: Implement the UMD/CommonJS scene contract module**

Use a dependency-free wrapper:

```javascript
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  if (root) {
    root.HanselRadarScene = api;
  }
})(typeof globalThis === "object" ? globalThis : this, function () {
  "use strict";
  const TRACK_MAX_AGE_MS = 300;
  function decodeBase64(text) {
    if (typeof Buffer === "function") {
      return Uint8Array.from(Buffer.from(text, "base64"));
    }
    return Uint8Array.from(atob(text), (character) => character.charCodeAt(0));
  }
  function decodeOccupancyGrid(grid) {
    if (!grid || grid.encoding !== "occupancy-u8-base64" ||
        grid.layout !== "forward-major_lateral-minor" ||
        grid.unknown_value !== 0) {
      throw new Error("invalid grid contract");
    }
    const bytes = decodeBase64(grid.data_base64);
    if (bytes.length !== grid.forward_cells * grid.lateral_cells) {
      throw new Error("grid length does not match dimensions");
    }
    return bytes;
  }
  function filterFreshTracks(tracks, maxAgeMs = TRACK_MAX_AGE_MS) {
    return (Array.isArray(tracks) ? tracks : []).filter(
      (track) => Number.isFinite(track.age_ms) &&
        track.age_ms >= 0 && track.age_ms < maxAgeMs,
    );
  }
  function parseRadarScene(snapshot) {
    const scene = snapshot && snapshot.scene;
    if (!scene || scene.schema_version !== 1) {
      throw new Error("unsupported scene schema");
    }
    const blocking = new Set(["waiting", "stale", "fault", "replay_end"]);
    if (blocking.has(snapshot.status) ||
        !["ok", "synthetic"].includes(scene.calibration_status)) {
      return { blocked: true, reason: snapshot.status, scene };
    }
    const tracks = filterFreshTracks(scene.tracks);
    const threshold = scene.hazard.threshold_m;
    if (scene.hazard.level === "DANGER" && !tracks.some(
      (track) => track.point_confirmed === true &&
        track.distance_m <= threshold,
    )) {
      throw new Error("DANGER contract is inconsistent");
    }
    return {
      blocked: false,
      grid: decodeOccupancyGrid(scene.grid),
      gridMeta: scene.grid,
      tracks,
      hazard: scene.hazard,
      hazardCopy: scene.hazard.level === "NORMAL"
        ? "10cm 이내 확인 장애물 없음 · 미관측 영역 존재"
        : scene.hazard.reason,
    };
  }
  function makeMapTransform(width, height, forwardMaxM, halfWidthM) {
    const scale = Math.min(width / (halfWidthM * 2), height / forwardMaxM);
    return { originX: width / 2, originY: height, scale };
  }
  function projectMapPoint(transform, forwardM, lateralM) {
    return {
      x: transform.originX + lateralM * transform.scale,
      y: transform.originY - forwardM * transform.scale,
    };
  }
  return {
    TRACK_MAX_AGE_MS,
    decodeOccupancyGrid,
    filterFreshTracks,
    parseRadarScene,
    makeMapTransform,
    projectMapPoint,
  };
});
```

`parseRadarScene` must block `waiting/stale/fault/replay_end`, invalid schema,
calibration required/mismatch, invalid grid, and inconsistent danger. It must
not emit a `safe`, `free`, or inferred-wall field.

- [ ] **Step 4: Run Node tests and verify GREEN**

Run:

```powershell
node --test tests/web/radar_scene.test.js
```

Expected: all Node contract tests pass.

- [ ] **Step 5: Commit the browser contract**

Run:

```powershell
git add -- monitor/web/radar_scene.js tests/web/radar_scene.test.js
git diff --cached --check
git diff --cached --name-only
git commit -m "feat: validate radar scene data in browser"
```

---

### Task 6: Replace the R8 camera façade with the R9 dual-map LiDAR UI

**Files:**
- Modify: `monitor/web/radar_front.html`
- Replace: `monitor/web/radar_panel.js`
- Modify: `monitor/radar_front.py`
- Modify: `tests/test_radar_front.py`

**Interfaces:**
- Consumes: `window.HanselRadarScene.parseRadarScene`, scene grid/tracks/hazard.
- Produces:
  - main canvas `#radar-main-canvas`
  - collision inset canvas `#collision-canvas`
  - `window.HanselRadarPanel`
  - no camera, hemisphere, wall-height, surface-mesh, or 900ms contour code

- [ ] **Step 1: Replace R8 HTTP/static assertions with failing R9 assertions**

Update the HTTP test to assert:

```python
self.assertEqual(
    payload["ui_build_id"],
    "20260728-lidar-operator-r9",
)
self.assertIn('id="radar-main-canvas"', html)
self.assertIn('id="collision-canvas"', html)
self.assertIn("0~3m LiDAR형", html)
self.assertIn("0~50cm 충돌 확대", html)
self.assertIn("UNKNOWN ≠ FREE", html)
self.assertIn("ROBOT RELATIVE", html)
self.assertIn("/radar_scene.js", html)
self.assertIn("drawLidarTopView", javascript)
self.assertIn("drawCollisionInset", javascript)
for forbidden in (
    "drawDepthCamera",
    "drawPerspectiveSurfaceMesh",
    "wallHeight",
    "depthContourCandidates",
    "WALL_TRACK_HOLD_MS",
    "CAUTION_RANGE_M",
):
    self.assertNotIn(forbidden, javascript)
for forbidden in ("view-select", "outline-toggle"):
    self.assertNotIn(forbidden, html)
```

Also fetch `/radar_scene.js` and assert it is served offline with no `https://`.

- [ ] **Step 2: Run the HTTP test and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_front.RadarFrontHttpTests.test_api_and_offline_assets_are_served -v
```

Expected: failures because R8 HTML/JS remains and `/radar_scene.js` is not
served.

- [ ] **Step 3: Replace the page structure**

Use this required stage structure:

```html
<div class="radar-stage" id="radar-panel">
  <canvas id="radar-main-canvas"
          aria-label="전방 0~3미터 LiDAR형 레이더 점유 지도"></canvas>
  <section id="collision-inset" data-hazard="UNKNOWN">
    <header>
      <strong>충돌 확대</strong>
      <span id="collision-distance">-- cm</span>
    </header>
    <canvas id="collision-canvas"
            aria-label="전방 0~50센티미터 충돌 확대 지도"></canvas>
  </section>
</div>
```

Keep fullscreen, status, FPS, age, profile, axes, frame/gap counters. Add
hazard, calibration, pose mode, raw point count, confirmed track count,
clutter rejected, heatmap cells rejected, and grid decode status. Remove all
view/range/persistence/outline controls.

Load scripts in order:

```html
<script src="/radar_scene.js?v=20260728-lidar-operator-r9"></script>
<script src="/radar_panel.js?v=20260728-lidar-operator-r9"></script>
```

- [ ] **Step 4: Replace `radar_panel.js` with presentation-only rendering**

Use exact constants:

```javascript
const UI_BUILD_ID = "20260728-lidar-operator-r9";
const MAIN_MAX_RANGE_M = 3.0;
const MAIN_HALF_WIDTH_M = 1.5;
const CLOSE_MAX_RANGE_M = 0.5;
const DANGER_RANGE_M = 0.1;
const TRACK_MAX_AGE_MS = 300;
```

Implement:

```javascript
acceptSnapshot(snapshot)
resizeCanvas(canvas)
draw()
drawLidarTopView(ctx, transform, scene)
drawCollisionInset(ctx, transform, scene)
drawMetricGrid(ctx, transform, options)
drawEvidenceGrid(ctx, transform, grid)
drawTracks(ctx, transform, tracks, options)
drawRawDebugPoints(ctx, transform, frame)
drawRobot(ctx, transform)
drawBlockingOverlay(ctx, width, height, presentation)
updateText(presentation)
updateDiagnostics(presentation)
updateSectors(presentation)
```

Rendering rules:

- Main: `0..3 m`, `-1.5..+1.5 m`, 0.5 m range guides.
- Inset: 10/20/30/40/50 cm guides using the same grid/tracks.
- `grid byte 0`: dark UNKNOWN background, never green.
- `1..255`: cyan/white occupied evidence scaled by confidence.
- Point track: crisp marker; heatmap track: range/angle uncertainty arc.
- Draw `z` only when `height_m !== null`.
- Label at most five closest fresh tracks.
- Red only when `scene.hazard.level === "DANGER"`,
  `track.point_confirmed === true`, and
  `track.distance_m <= scene.hazard.threshold_m`.
- `NORMAL` copy: `10cm 이내 확인 장애물 없음 · 미관측 영역 존재`.
- Blocking conditions replace distance graphics with a calibration/sensor
  overlay.

- [ ] **Step 5: Serve the scene module and synchronize build IDs**

In `build_handler` add:

```python
"/radar_scene.js": (
    "radar_scene.js",
    "text/javascript; charset=utf-8",
),
```

Keep Python and both JavaScript references on
`20260728-lidar-operator-r9`.

- [ ] **Step 6: Run focused UI contract tests**

Run:

```powershell
python -m unittest tests.test_radar_front -v
node --test tests/web/radar_scene.test.js
```

Expected: all Python HTTP/state and Node browser-contract tests pass.

- [ ] **Step 7: Commit the R9 UI**

Run:

```powershell
git add -- `
  monitor/web/radar_front.html monitor/web/radar_panel.js `
  monitor/radar_front.py tests/test_radar_front.py
git diff --cached --check
git diff --cached --name-only
git commit -m "feat: replace radar camera facade with LiDAR operator view"
```

---

### Task 7: Update operating documentation and complete replay/browser verification

**Files:**
- Modify: `docs/radar_front_view.md`
- Modify: `docs/radar_reconnect_windows.md`
- Modify: `docs/superpowers/plans/2026-07-28-radar-lidar-operator-view.md`

**Interfaces:**
- Consumes: completed calibration CLI, R9 API/UI, real fixture and full mission.
- Produces: reproducible Windows commands, fresh test evidence, desktop/browser
  verification, and checked plan boxes.

- [ ] **Step 1: Write failing documentation contract assertions**

Add to `tests/test_radar_front.py` HTTP/static checks or a focused docs test:

```python
docs = (REPO_ROOT / "docs" / "radar_front_view.md").read_text("utf-8")
self.assertIn("radar-calibrate", docs)
self.assertIn("--clutter-calibration", docs)
self.assertIn("0~3m", docs)
self.assertIn("0~50cm", docs)
self.assertIn("10cm", docs)
self.assertIn("UNKNOWN", docs)
self.assertNotIn("흑백 벽면", docs)
```

- [ ] **Step 2: Run the docs assertion and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_front -v
```

Expected: documentation assertion fails on the old R8 instructions.

- [ ] **Step 3: Replace R8 instructions with the calibrated R9 workflow**

Document these commands exactly:

```powershell
python -m sensors radar-calibrate missions\radar-empty-scene.jsonl `
  --output configs\radar\calibrations\head-near.json `
  --min-frames 50

python monitor\radar_front.py `
  --follow missions\radar-board-live.jsonl `
  --clutter-calibration configs\radar\calibrations\head-near.json `
  --max-range-m 3 `
  --history-window 0.3
```

Explain:

- calibration capture must be board-stationary with no object inside 3 m;
- profile, heatmap shape/range step, and axes are bound;
- main map is current robot-relative evidence, not SLAM;
- 50 cm is an inset, not the main visibility limit;
- only confirmed non-clutter points create 10 cm red;
- heatmap near bins and missing returns never mean safe;
- encoder/IMU motion compensation is the next phase.

- [ ] **Step 4: Run full automated verification**

Run:

```powershell
python -m unittest discover -s tests -v
node --test tests/web/radar_scene.test.js
git diff --check
```

Expected: all Python and Node tests pass; whitespace check returns exit `0`.

- [ ] **Step 5: Generate a temporary real-fixture calibration**

Run:

```powershell
python -m sensors radar-calibrate `
  tests\fixtures\radar_clutter_20260728_f2286_f2291.jsonl `
  --output tmp\radar-r9-fixture-calibration.json `
  --min-frames 6 `
  --overwrite
```

Expected: exit `0`, deterministic calibration summary, and no repository file
outside `tmp` changes.

- [ ] **Step 6: Start demo and real replay servers for browser verification**

Demo:

```powershell
python monitor\radar_front.py --demo --bind 127.0.0.1 `
  --http-port 8081 --max-range-m 3 --history-window 0.3 --quiet
```

Real fixture replay:

```powershell
python monitor\radar_front.py `
  --replay tests\fixtures\radar_clutter_20260728_f2286_f2291.jsonl `
  --clutter-calibration tmp\radar-r9-fixture-calibration.json `
  --bind 127.0.0.1 --http-port 8082 --max-range-m 3 `
  --history-window 0.3 --loop --quiet
```

- [ ] **Step 7: Verify desktop and mobile browser layouts**

At `1440 x 900` and `390 x 844`, verify:

- main 3 m map and 50 cm inset are both visible;
- no fake wall, vertical camera panel, hemisphere, or surface mesh exists;
- UNKNOWN background is not green;
- 0.70 m evidence never becomes collision red;
- a synthetic confirmed 0.08 m scene turns only the track/inset border red;
- calibration-required, profile-mismatch, stale, fault, and replay-end block
  both maps;
- fullscreen keeps the inset and hazard badge visible;
- no console errors, uncaught exceptions, horizontal scrolling, or clipped
  controls.

Capture one full-page screenshot for the implementation report.

- [ ] **Step 8: Commit docs and checked plan**

Mark completed checkboxes in this plan, then run:

```powershell
git add -- `
  docs/radar_front_view.md docs/radar_reconnect_windows.md `
  docs/superpowers/plans/2026-07-28-radar-lidar-operator-view.md `
  tests/test_radar_front.py
git diff --cached --check
git diff --cached --name-only
git commit -m "docs: document calibrated radar LiDAR operation"
```

- [ ] **Step 9: Final verification and branch audit**

Run fresh:

```powershell
python -m unittest discover -s tests -v
node --test tests/web/radar_scene.test.js
git diff --check
git status --short --branch
git log --oneline --decorate -8
```

Expected:

- all tests pass;
- only pre-existing unrelated user files remain dirty;
- the task branch contains the design, baseline, calibration, scene, API,
  browser contract, UI, and docs commits in order.
