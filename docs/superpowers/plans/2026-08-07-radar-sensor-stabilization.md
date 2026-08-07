# HANSEL Radar Sensor Stabilization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a stable 2D/2.5D probabilistic obstacle map, a robot-width-aware safe local path, and a filtered radar driving-assistance screen for dusty collapse sites.

**Architecture:** Keep the verified TI UART, 8 Hz profile, calibration gate, and 10/13 cm hazard behavior. Add three dependency-free radar components—robust point filtering, conservative radar-only motion estimation, and a decaying probabilistic local map—then make `RadarSceneEstimator` orchestrate them and expose one additive API consumed by a strictly validated LiDAR-style 2D/2.5D browser view.

**Tech Stack:** Python 3.9+ standard library, `unittest`, canonical JSONL radar records, dependency-free JavaScript, Canvas 2D, Node built-in `node:test`, Windows PowerShell.

## Global Constraints

- Execution prerequisite: the planning turn commits this approved plan before Task 1 begins. Start implementation only from a clean worktree containing that plan commit; do not leave the plan as an untracked handoff artifact.
- Change only radar-related files under `monitor/`, `scripts/`, `sensors/`, `tests/`, and `docs/`; the only permitted `sensors/` production change is `sensors/radar_stack_processes.py` for calibration-ID propagation. Do not modify controller, robot, rescue-network, listen, or general network code.
- Keep `configs/radar/iwrl6432_3d_operator_near_8hz.cfg` unchanged, including 8 Hz timing and `clutterRemoval 0`.
- Preserve `RadarSceneEstimator`'s positional constructor compatibility; do not insert parameters before `clock`.
- Preserve the existing `point_confirmed` 2-of-3 contract for `DANGER`; enter at `<= 0.10 m` and release at `>= 0.13 m`.
- Add a separate point-map confirmation requiring at least 3 hits in the latest 5 frame windows.
- Use a robot-centred 0.50 m probabilistic map with 0.05 m cells, prior probability 0.5 (`UNKNOWN`), occupied threshold 0.75, 1.0 s log-odds half-life, and 2.5 s TTL.
- Radar misses never create `FREE`; they only allow positive evidence to decay toward `UNKNOWN`.
- Use fixed hit probability 0.80, clamp stored probability to 0.97, and cap hit counts at 255.
- Radar-only motion requires at least 5 confirmed point correspondences, match ratio `>= 0.70`, and RMSE `<= 0.05 m`; rejected motion never transforms the map.
- Keep point-cloud z only as `sensor_relative_uncalibrated`; bands are `below_sensor` for z `< -0.10 m`, `sensor_level` for `-0.10..+0.10 m`, `above_sensor` for z `> +0.10 m`, and `unknown` for no measured z.
- Project every confirmed height band into the 2D path costmap until IMU pose and radar-to-base extrinsics exist.
- Use robot width 0.15 m plus 0.03 m safety margin on each side; minimum passage width remains 0.21 m.
- Replan a blocked selected path immediately; otherwise require the same new curvature for 3 consecutive ingested radar frames before switching.
- Preserve the 0.01 m path sampling contract as display/planning sampling, not sensor accuracy.
- Default browser rendering uses filtered scene tracks and probabilistic cells. Raw points are available only behind an unchecked `RAW DEBUG · 표시 전용` control and never affect map, hazard, or path.
- Keep `API_VERSION=1`, `SCENE_SCHEMA_VERSION=1`, legacy root fields, `scene.grid`, and `frame.points`; add new fields without deleting existing consumers' fields.
- Keep `rolling_local_map` for one release as an alias containing exactly the new occupied cells and `retention: "probability_decay_with_ttl"`.
- Do not add Python packages, npm packages, browser CDNs, camera fusion, Doppler odometry, IMU/encoder code, motor commands, or new radar firmware/configuration.
- Follow strict TDD for every production change: write one behavioral test, observe the expected failure, implement minimally, rerun focused tests, then commit only the task's listed files.
- In every GREEN/commit PowerShell block, check each native command's `$LASTEXITCODE` immediately and throw on non-zero; never continue to `git add` or `git commit` after a failed Python, Node, compile, or Git check. RED blocks are the explicit exception: capture the expected non-zero result and throw if the command unexpectedly returns zero.
- Treat the existing timing-sensitive `MissionLogFollowerTests.test_partial_line_is_not_published_until_newline_arrives` as a pre-existing flake; if it recurs, rerun it once and record both results without changing it in this plan.
- `calibration/profile mismatch` includes the actual `SensorHeader.calibration_id`: the supervised capture command must load the selected `RadarClutterModel.calibration_id`, pass it to `radar-live`, and the scene binding must compare it with the loaded model. Profile ID, axes, and heatmap geometry/range-step remain independently checked. Board serial, mount ID, cable state, temperature, and location provenance remain a documented recapture checklist because the capture contract does not provide trustworthy automatic values for them; do not claim the runtime detects those manual provenance fields.

---

### Task 1: Add the decaying probabilistic local-map component

**Files:**
- Create: `monitor/radar_local_map.py`
- Create: `tests/test_radar_local_map.py`

**Interfaces:**
- Consumes: confirmed obstacle hits as forward/lateral metres, confidence, source, range/lateral uncertainty, and monotonic seconds.
- Produces: `ProbabilisticRollingGrid.add_hit(forward_m, lateral_m, confidence, source, range_uncertainty_m, lateral_uncertainty_m, now)`, `transform(delta_forward_m, delta_lateral_m, yaw_rad, now)`, `clear()`, `payload(now)`, `occupied_cells(now)`, and `diagnostics()`.
- Payload cells use metres and contain `forward_m`, `lateral_m`, `probability`, `confidence`, `occupied`, `hits`, `age_ms`, `range_uncertainty_m`, `lateral_uncertainty_m`, and `source`.

- [ ] **Step 1: Write all failing local-map behavior tests**

Create `tests/test_radar_local_map.py` with these literal behaviors:

```python
import math
import unittest

from monitor.radar_local_map import ProbabilisticRollingGrid


class ProbabilisticRollingGridTests(unittest.TestCase):
    def make_grid(self):
        return ProbabilisticRollingGrid(
            radius_m=0.50,
            resolution_m=0.05,
            occupied_threshold=0.75,
            decay_half_life_s=1.0,
            ttl_s=2.5,
            hit_probability=0.80,
            maximum_probability=0.97,
        )

    def test_unknown_map_starts_without_invented_free_cells(self):
        self.assertEqual(self.make_grid().payload(1.0)["cells"], [])

    def test_confirmed_hit_is_occupied_and_preserves_uncertainty(self):
        grid = self.make_grid()
        grid.add_hit(0.31, -0.04, 0.8, "point", 0.049, 0.052, 1.0)
        cell = grid.payload(1.0)["cells"][0]
        self.assertEqual((cell["forward_m"], cell["lateral_m"]), (0.30, -0.05))
        self.assertAlmostEqual(cell["probability"], 0.80, places=3)
        self.assertTrue(cell["occupied"])
        self.assertEqual(cell["hits"], 1)
        self.assertEqual(cell["range_uncertainty_m"], 0.049)

    def test_one_half_life_decays_toward_unknown_without_becoming_free(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        cell = grid.payload(2.0)["cells"][0]
        self.assertAlmostEqual(cell["probability"], 2.0 / 3.0, places=3)
        self.assertFalse(cell["occupied"])
        self.assertGreater(cell["probability"], 0.5)
        self.assertEqual(grid.diagnostics()["decayed_cell_count"], 1)

    def test_cell_is_removed_at_exact_2500ms_ttl(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        self.assertEqual(grid.payload(3.5)["cells"], [])
        self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"], 1)

    def test_probability_and_hit_count_are_capped(self):
        grid = self.make_grid()
        for _ in range(300):
            grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        cell = grid.payload(1.0)["cells"][0]
        self.assertAlmostEqual(cell["probability"], 0.97, places=6)
        self.assertEqual(cell["hits"], 255)

    def test_repeated_key_conservatively_merges_metadata_and_source(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.08, 0.03, 1.0)
        grid.add_hit(0.31, 0.01, 0.6, "heatmap", 0.02, 0.09, 1.0)
        cell = grid.payload(1.0)["cells"][0]
        self.assertEqual(cell["confidence"], 0.9)
        self.assertEqual(cell["range_uncertainty_m"], 0.08)
        self.assertEqual(cell["lateral_uncertainty_m"], 0.09)
        self.assertEqual(cell["source"], "mixed")

    def test_authorized_transform_rebins_without_duplicating_probability(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        grid.transform(-0.05, 0.0, 0.0, 1.0)
        cell = grid.occupied_cells(1.0)[0]
        self.assertEqual(cell["forward_m"], 0.25)
        self.assertAlmostEqual(cell["probability"], 0.80, places=3)

    def test_transform_drops_rebinned_cell_outside_map_without_ttl_count(self):
        grid = self.make_grid()
        grid.add_hit(0.48, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        grid.transform(0.05, 0.0, 0.0, 1.0)
        self.assertEqual(grid.payload(1.0)["cells"], [])
        self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"], 0)

    def test_transform_uses_old_to_new_rotation_then_translation(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.10, 0.9, "point", 0.05, 0.05, 1.0)
        grid.transform(0.05, -0.02, math.radians(90.0), 1.0)
        cell = grid.payload(1.0)["cells"][0]
        self.assertEqual((cell["forward_m"], cell["lateral_m"]), (-0.05, 0.30))

    def test_transform_collision_conservatively_merges_without_summing(self):
        grid = self.make_grid()
        grid.add_hit(0.05, -0.40, 0.9, "point", 0.08, 0.03, 1.0)
        grid.add_hit(0.05, -0.35, 0.6, "heatmap", 0.02, 0.09, 1.0)
        grid.transform(0.0, 0.0, math.radians(15.0), 1.0)
        cells = grid.payload(1.0)["cells"]
        self.assertEqual(len(cells), 1)
        self.assertAlmostEqual(cells[0]["probability"], 0.80, places=3)
        self.assertEqual(cells[0]["hits"], 1)
        self.assertEqual(cells[0]["confidence"], 0.9)
        self.assertEqual(cells[0]["range_uncertainty_m"], 0.08)
        self.assertEqual(cells[0]["lateral_uncertainty_m"], 0.09)
        self.assertEqual(cells[0]["source"], "mixed")

    def test_decay_and_ttl_counters_are_poll_idempotent(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        grid.payload(2.0)
        grid.payload(2.0)
        grid.occupied_cells(2.0)
        self.assertEqual(grid.diagnostics()["decayed_cell_count"], 1)
        grid.payload(3.5)
        grid.payload(3.5)
        self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"], 1)

    def test_occupied_cells_poll_alone_advances_decay_and_ttl(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        self.assertEqual(grid.occupied_cells(2.0), [])
        self.assertEqual(grid.diagnostics()["decayed_cell_count"], 1)
        self.assertEqual(grid.occupied_cells(3.5), [])
        self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"], 1)

    def test_clear_removes_cells_and_resets_every_counter(self):
        grid = self.make_grid()
        grid.add_hit(0.30, 0.0, 0.9, "point", 0.05, 0.05, 1.0)
        grid.payload(2.0)
        grid.payload(3.5)
        self.assertEqual(grid.diagnostics()["decayed_cell_count"], 1)
        self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"], 1)
        grid.clear()
        self.assertEqual(grid.payload(3.5)["cells"], [])
        self.assertEqual(grid.diagnostics(), {
            "active_cell_count": 0,
            "occupied_cell_count": 0,
            "decayed_cell_count": 0,
            "ttl_expired_cell_count": 0,
        })
```

- [ ] **Step 2: Run the new test and verify the missing-module failure**

Run:

```powershell
python -B -m unittest tests.test_radar_local_map -v
if ($LASTEXITCODE -eq 0) { throw "expected local-map RED failure" }
```

Expected first run: FAIL because `monitor.radar_local_map` does not exist. Then create an importable `monitor/radar_local_map.py` skeleton containing the named class/public methods, with behavior methods raising `NotImplementedError`, and rerun. Require failures to reach the behavioral calls (not import/fixture errors) before Step 3; do not commit the skeleton separately.

- [ ] **Step 3: Implement the minimal log-odds grid**

Create `monitor/radar_local_map.py`, starting with `from __future__ import annotations`, `import math`, and the standard-library dataclass/typing imports required by the implementation. Add one `ProbabilisticRollingGrid` class. Its exact public methods are `add_hit(forward_m, lateral_m, confidence, source, range_uncertainty_m, lateral_uncertainty_m, now) -> None`, `transform(delta_forward_m, delta_lateral_m, yaw_rad, now) -> None`, `clear() -> None`, `payload(now) -> dict`, `occupied_cells(now) -> list[dict]`, and `diagnostics() -> dict`.

Use the following exact calculations:

```python
hit_log_odds = math.log(0.80 / 0.20)
max_log_odds = math.log(0.97 / 0.03)
decay = 2.0 ** (-elapsed_seconds / 1.0)
cell.log_odds *= decay
probability = 1.0 / (1.0 + math.exp(-cell.log_odds))
def quantized_index(value_m, resolution_m):
    magnitude = math.floor(abs(value_m) / resolution_m + 0.5)
    return int(math.copysign(magnitude, value_m))

key = (
    quantized_index(forward_m, 0.05),
    quantized_index(lateral_m, 0.05),
)
```

`transform(delta_forward_m, delta_lateral_m, yaw_rad, now)` uses the same old-to-new scene convention as Task 4: rotate first, then translate.

```python
new_forward = (
    math.cos(yaw_rad) * old_forward
    - math.sin(yaw_rad) * old_lateral
    + delta_forward_m
)
new_lateral = (
    math.sin(yaw_rad) * old_forward
    + math.cos(yaw_rad) * old_lateral
    + delta_lateral_m
)
```

Re-quantize each result with `quantized_index`; never use Python's banker-rounding for transformed cell keys.

Before every hit, transform, payload, or `occupied_cells(now)` query, advance decay from each cell's `_last_decay_ns`; delete a cell when `now_ns - _last_hit_ns >= 2_500_000_000`. `occupied_cells(now)` must perform this advance itself even when no payload call preceded it, because the planner polls it directly. Clamp hit count to 255 and log odds to the configured 0.97 ceiling. Increment `decayed_cell_count` exactly once when an occupied cell crosses below 0.75 during one occupancy episode; a later confirmed hit may begin a new episode. Increment `ttl_expired_cell_count` only on TTL deletion. Both are cumulative until `clear()`. `payload()` returns metadata `radius_m=0.50`, `resolution_m=0.05`, `occupied_threshold=0.75`, `decay_half_life_ms=1000`, `ttl_ms=2500`, `retention="probability_decay_with_ttl"`, `pose_mode="robot_relative_short_memory"`, plus every still-positive evidence cell so the UI can fade it. `occupied_cells()` returns only cells with probability `>= 0.75`. Ignore hits whose quantized centre lies outside the 0.50 m circle. After every transform, also discard any re-quantized cell whose new centre lies outside that same circle; this is a geometry-boundary discard, not a TTL expiry, so it must not increment `ttl_expired_cell_count`.

On every same-key merge—including an ordinary repeated `add_hit` and a transform rebin collision—retain maximum confidence and maximum range/lateral uncertainty. Preserve `source` when contributors agree and set it deterministically to `"mixed"` when point/heatmap/mixed contributors differ. An ordinary new hit still adds one hit log-odds increment and increments the capped hit count; a transform collision retains maximum log-odds, latest hit time, and maximum hit count rather than summing duplicated evidence.

- [ ] **Step 4: Run the local-map tests and commit**

Run:

```powershell
python -B -m unittest tests.test_radar_local_map -v
if ($LASTEXITCODE -ne 0) { throw "local-map tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- monitor/radar_local_map.py tests/test_radar_local_map.py
if ($LASTEXITCODE -ne 0) { throw "local-map staging failed" }
git commit -m "feat(radar): add probabilistic local map"
if ($LASTEXITCODE -ne 0) { throw "local-map commit failed" }
```

Expected: all new tests pass and only the two listed files are committed.

---

### Task 2: Add robust point filtering, association, and 2.5D metadata

**Files:**
- Create: `monitor/radar_tracking.py`
- Create: `tests/test_radar_tracking.py`
- Modify: `monitor/radar_scene.py:18-99, 308-492, 2381-2528`
- Modify: `tests/test_radar_scene.py:212-228, 448-476, 956-969, 1100-1121`

**Interfaces:**
- Consumes: calibrated point measurements and `RadarHeatmap.range_step_m`; falls back to a bound clutter model's range step, or to `0.09765625 m` only for the verified 8 Hz profile and explicit synthetic tests.
- Produces: `resolve_range_step_m`, `point_quality_score`, `point_association_gate`, `robust_point_estimate`, and `height_band` helpers used by the existing scene track dictionaries.
- Preserves: `point_confirmed` as hazard-only 2-of-3; adds `map_confirmed` as 3-of-5.

- [ ] **Step 1: Write failing pure-filter tests**

Create `tests/test_radar_tracking.py`:

```python
import unittest

from monitor.radar_tracking import (
    DEFAULT_RANGE_STEP_M,
    VERIFIED_NEAR_8HZ_PROFILE_ID,
    height_band,
    point_association_gate,
    point_quality_score,
    resolve_range_step_m,
    robust_point_estimate,
)


class RadarTrackingTests(unittest.TestCase):
    def test_range_step_precedence_never_borrows_from_unknown_profile(self):
        cases = (
            (0.08, 0.09, True, "unknown", False, 0.08),
            (None, 0.09, True, "unknown", False, 0.09),
            (0.0, 0.09, True, "unknown", False, 0.09),
            (None, None, False, VERIFIED_NEAR_8HZ_PROFILE_ID, False,
             DEFAULT_RANGE_STEP_M),
            (None, None, False, "unknown", True, DEFAULT_RANGE_STEP_M),
            (float("nan"), None, False, "unknown", False, None),
            (None, None, False, "unknown", False, None),
        )
        for heatmap, calibrated, bound, profile, synthetic, expected in cases:
            with self.subTest(profile=profile, heatmap=heatmap):
                self.assertEqual(resolve_range_step_m(
                    heatmap_range_step_m=heatmap,
                    calibrated_range_step_m=calibrated,
                    calibration_profile_matches=bound,
                    frame_profile_id=profile,
                    synthetic=synthetic,
                ), expected)

    def test_quality_uses_snr_without_subtracting_ti_noise_db(self):
        self.assertEqual(point_quality_score(None, 90.0), 0.50)
        self.assertEqual(point_quality_score(15.0, 90.0), 0.50)
        self.assertEqual(point_quality_score(24.0, 70.0), 0.80)

    def test_association_separates_range_and_range_scaled_lateral_gates(self):
        self.assertTrue(point_association_gate(0.40, 0.0, 0.50, 0.0, DEFAULT_RANGE_STEP_M))
        self.assertFalse(point_association_gate(0.40, 0.0, 0.40, 0.25, DEFAULT_RANGE_STEP_M))

    def test_median_rejects_one_position_spike(self):
        estimate = robust_point_estimate([
            (0.40, 0.00, 0.02), (0.41, 0.01, 0.02),
            (0.39, -0.01, 0.03), (0.40, 0.00, 0.02),
            (0.63, 0.22, 0.40),
        ], residual_limit_m=0.13)
        self.assertAlmostEqual(estimate.forward_m, 0.40, places=2)
        self.assertAlmostEqual(estimate.lateral_m, 0.00, places=2)
        self.assertFalse(estimate.latest_inlier)

    def test_height_bands_are_sensor_relative(self):
        self.assertEqual(height_band(-0.101), "below_sensor")
        self.assertEqual(height_band(-0.10), "sensor_level")
        self.assertEqual(height_band(0.10), "sensor_level")
        self.assertEqual(height_band(0.101), "above_sensor")
        self.assertEqual(height_band(None), "unknown")
```

- [ ] **Step 2: Run the pure-filter tests and verify RED**

Run:

```powershell
python -B -m unittest tests.test_radar_tracking -v
if ($LASTEXITCODE -eq 0) { throw "expected tracking RED failure" }
```

Expected first run: FAIL because `monitor.radar_tracking` does not exist. Then create an importable skeleton defining the imported constants/dataclass/function names, with behavior functions raising `NotImplementedError`, and rerun. Require failures to reach behavioral calls rather than import or signature errors before Step 3; do not commit the skeleton separately.

- [ ] **Step 3: Implement the pure helpers with fixed uncertainty rules**

Create `monitor/radar_tracking.py` with `from __future__ import annotations`, `import math`, `from dataclasses import dataclass`, and the standard-library typing imports required by the signatures. Add:

```python
DEFAULT_RANGE_STEP_M = 0.09765625
VERIFIED_NEAR_8HZ_PROFILE_ID = (
    "lsdk-05.05.04.02-presence-near-"
    "heatmap16-elev8-cfar15-8hz-v1"
)
POINT_HISTORY_FRAMES = 5
POINT_MAP_CONFIRM_HITS = 3
POINT_HAZARD_WINDOW_FRAMES = 3
POINT_HAZARD_CONFIRM_HITS = 2
POINT_MIN_QUALITY = 0.50
POINT_STRONG_QUALITY = 0.70
POINT_POSITION_RESIDUAL_LIMIT_M = 0.13
POINT_AZIMUTH_ASSOCIATION_DEG = 19.0
POINT_AZIMUTH_UNCERTAINTY_DEG = 9.5

def point_quality_score(snr_db, noise_db):
    del noise_db
    return 0.50 if snr_db is None else max(0.05, min(1.0, snr_db / 30.0))

def range_gate_m(range_step_m):
    return max(0.10, 1.25 * range_step_m)

def lateral_gate_m(distance_m):
    return max(0.05, distance_m * math.tan(math.radians(19.0)))

def lateral_uncertainty_m(distance_m):
    return max(0.05, distance_m * math.tan(math.radians(9.5)))
```

Implement `resolve_range_step_m(heatmap_range_step_m, calibrated_range_step_m, calibration_profile_matches, frame_profile_id, synthetic)` with the exact precedence in Step 7 and return `None` when no verified source exists. Return `RobustPointEstimate` as a frozen dataclass with fields `forward_m`, `lateral_m`, `height_m`, `position_jitter_m`, `latest_inlier`, `inlier_count`, and `rejected_count`. `noise_db` is retained in diagnostics but is not subtracted from TI's already-reported SNR. `point_association_gate` requires both absolute range difference within `range_gate_m()` and cross-range angular residual within `lateral_gate_m(max(old_range, new_range))`. `robust_point_estimate` computes coordinate-wise preliminary medians, removes samples whose 2D residual exceeds the supplied limit, recomputes medians over inliers, reports maximum inlier residual as jitter, and marks whether the newest sample survived. Scene integration always passes `POINT_POSITION_RESIDUAL_LIMIT_M`.

- [ ] **Step 4: Run the pure-filter suite and verify GREEN**

Run:

```powershell
python -B -m unittest tests.test_radar_tracking -v
if ($LASTEXITCODE -ne 0) { throw "tracking unit tests failed" }
```

Expected: all pure range-step, quality, association, robust-position, and height-band tests pass before scene integration begins.

- [ ] **Step 5: Write failing scene-integration tests**

First replace the existing `point_frame` helper in `tests/test_radar_scene.py` with this backward-compatible literal version so the RED failures exercise scene behavior rather than argument parsing:

```python
def point_frame(
    forward_m: float,
    *,
    frame_number: int = 1,
    snr_db: float = 20.0,
) -> RadarFrame:
    return radar_frame(
        (
            RadarPoint(
                x_m=0.0,
                y_m=forward_m,
                z_m=0.01,
                radial_velocity_mps=0.0,
                snr_db=snr_db,
            ),
        ),
        frame_number=frame_number,
    )
```

The default preserves every existing caller while map-confirmation tests request the required strong 24 dB quality explicitly. Then add these methods to `RadarSceneEstimatorTests`:

```python
def test_point_map_confirmation_is_three_of_five_while_hazard_remains_two_of_three(self):
    estimator = calibrated_estimator()
    estimator.ingest(point_frame(0.09, frame_number=1, snr_db=24.0), received_at=1.000)
    estimator.ingest(point_frame(0.09, frame_number=2, snr_db=24.0), received_at=1.125)
    track = estimator.snapshot(now=1.125)["tracks"][0]
    self.assertTrue(track["point_confirmed"])
    self.assertFalse(track["map_confirmed"])
    self.assertEqual(estimator.snapshot(now=1.125)["hazard"]["level"], "DANGER")
    estimator.ingest(point_frame(0.09, frame_number=3, snr_db=24.0), received_at=1.250)
    self.assertTrue(estimator.snapshot(now=1.250)["tracks"][0]["map_confirmed"])

def test_weak_hits_confirm_hazard_history_but_not_strong_map_history(self):
    estimator = calibrated_estimator()
    for frame_number, snr_db in ((1, 15.0), (2, 15.0), (3, 24.0)):
        estimator.ingest(
            point_frame(0.30, frame_number=frame_number, snr_db=snr_db),
            received_at=1.0 + frame_number * 0.125,
        )
    track = estimator.snapshot(now=1.375)["tracks"][0]
    self.assertTrue(track["point_confirmed"])
    self.assertFalse(track["map_confirmed"])
    for frame_number in (4, 5):
        estimator.ingest(
            point_frame(0.30, frame_number=frame_number, snr_db=24.0),
            received_at=1.0 + frame_number * 0.125,
        )
    self.assertTrue(estimator.snapshot(now=1.625)["tracks"][0]["map_confirmed"])

def test_filtered_point_preserves_measured_height_and_band(self):
    estimator = calibrated_estimator()
    point = RadarPoint(0.0, 0.40, 0.12, 0.0, 24.0, 81.0)
    estimator.ingest(radar_frame((point,)), received_at=3.0)
    track = estimator.snapshot(now=3.0)["tracks"][0]
    self.assertEqual(track["height_m"], 0.12)
    self.assertEqual(track["height_band"], "above_sensor")
    self.assertEqual(track["height_reference"], "sensor_relative_uncalibrated")
    self.assertEqual((track["snr_db"], track["noise_db"]), (24.0, 81.0))

def test_one_frame_position_spike_does_not_move_filtered_track(self):
    estimator = calibrated_estimator()
    for index, forward in enumerate((0.40, 0.41, 0.39, 0.40), 1):
        estimator.ingest(point_frame(forward, frame_number=index), received_at=1.0 + index * 0.1)
    received_before_outlier = estimator._tracks[0]["_received_at_ns"]
    estimator.ingest(radar_frame((
        RadarPoint(0.13, 0.45, 0.40, 0.0, 24.0),
    ), frame_number=5), received_at=1.5)
    track = estimator.snapshot(now=1.5)["tracks"][0]
    self.assertAlmostEqual(track["forward_m"], 0.40, delta=0.02)
    self.assertGreaterEqual(track["outlier_rejections"], 1)
    self.assertEqual(
        estimator._tracks[0]["_received_at_ns"], received_before_outlier,
    )
    self.assertFalse(estimator._tracks[0]["_updated_by_point"])
    self.assertGreater(track["age_ms"], 0)

def test_four_dropped_windows_clear_five_frame_map_confirmation(self):
    estimator = calibrated_estimator()
    for frame_number in range(1, 4):
        estimator.ingest(point_frame(
            0.30, frame_number=frame_number, snr_db=24.0,
        ), received_at=1.0 + frame_number * 0.125)
    estimator.ingest(radar_frame(
        (), frame_number=8, dropped_frames_since_previous=4,
    ), received_at=1.5)
    track = estimator.snapshot(now=1.5)["tracks"][0]
    self.assertFalse(track["map_confirmed"])
    self.assertFalse(track["point_confirmed"])

def test_trusted_scene_without_verified_range_step_rejects_point(self):
    class TrustedWithoutRangeStep(RadarSceneEstimator):
        def _binding_status(self, frame):
            return "ok"

    estimator = TrustedWithoutRangeStep(
        RadarAxes(), require_calibration=False, synthetic=False,
    )
    estimator.ingest(radar_frame(
        (RadarPoint(0.0, 0.30, 0.0, 0.0, 24.0),),
        frame_number=1, profile_id="unknown-profile",
    ), received_at=1.0)
    scene = estimator.snapshot(now=1.0)
    self.assertEqual(scene["tracks"], [])
    self.assertEqual(scene["diagnostics"]["quality_gate_rejected_count"], 1)
```

Replace the pre-existing `test_point_elevation_is_excluded_from_azimuth_only_scene` with `test_point_elevation_is_preserved_but_not_used_for_2d_projection`: retain the assertions `projection_mode == "range_azimuth_2d"` and `elevation_used is False`, but change the old `height_m is None` expectation to `height_m == 0.75`, `height_band == "above_sensor"`, and `height_reference == "sensor_relative_uncalibrated"`. This removes the contradictory legacy expectation before the full scene suite is used as the GREEN gate.

- [ ] **Step 6: Run the scene subset and verify the old behavior fails**

Run:

```powershell
python -B -m unittest -v `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_point_map_confirmation_is_three_of_five_while_hazard_remains_two_of_three `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_weak_hits_confirm_hazard_history_but_not_strong_map_history `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_filtered_point_preserves_measured_height_and_band `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_one_frame_position_spike_does_not_move_filtered_track `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_four_dropped_windows_clear_five_frame_map_confirmation `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_trusted_scene_without_verified_range_step_rejects_point
if ($LASTEXITCODE -eq 0) { throw "expected scene-tracking RED failure" }
```

Expected: FAIL because map confirmation and height metadata are absent and point positions are overwritten.

- [ ] **Step 7: Integrate helpers into point tracks without changing heatmap semantics**

In `monitor/radar_scene.py`:

- Resolve range step in this order: a positive frame heatmap value; the bound clutter model's positive `range_step_m` when its profile matches the frame; `DEFAULT_RANGE_STEP_M` when `self.synthetic` is true or the frame profile equals `VERIFIED_NEAR_8HZ_PROFILE_ID`; otherwise reject the point from new/updated tracks and increment `quality_gate_rejected_count`. Never silently assign the verified 8 Hz step to an unknown live profile.
- Reject point quality `< 0.50`; missing SNR remains accepted at exactly 0.50 but cannot become a strong map hit.
- Replace fixed point nearest-neighbour admission with `point_association_gate`; keep one-to-one `_updated_by_point` exclusion.
- Keep separate histories: `_point_hits` contains accepted latest-inlier observations for the 2-of-3 hazard contract, while `_point_strong_hits` contains only latest-inlier observations whose quality is `>= 0.70` for the 3-of-5 map contract. Extend both histories with `min(5, dropped_frames_since_previous + 1)` false windows, retain five entries, compute `point_confirmed = sum(_point_hits[-3:]) >= 2`, and compute `map_confirmed = sum(_point_strong_hits[-5:]) >= 3`. Weak accepted hits may confirm hazard presence but can never satisfy map history. Continue trimming heatmap histories to their own configured lengths rather than reusing a hard-coded point-window cap.
- Store the latest five `(forward, lateral, z)` samples and use `robust_point_estimate`. When `latest_inlier` is false, retain the prior filtered coordinates and prior `_received_at_ns`, leave `_updated_by_point=False`, and leave the already pre-appended current history slot false; do not append a second false element for the same frame. The rejected newest observation must serialize with non-zero age and must not count as a current detection.
- Serialize `height_m`, `height_band`, `height_reference`, `snr_db`, `noise_db`, `quality_score`, `range_uncertainty_m = range_step_m / 2`, `lateral_uncertainty_m`, `position_jitter_m`, and `outlier_rejections`.
- Leave heatmap `height_m=None`, `height_band="unknown"`, and `height_reference="unavailable"`, but assign conservative `lateral_uncertainty_m(distance_m)` to every heatmap track; never serialize zero lateral uncertainty merely because heatmap has no z. Extend an existing confirmed-heatmap regression to assert its lateral uncertainty is at least 0.05 m.

- [ ] **Step 8: Run focused regressions and commit**

Run:

```powershell
python -B -m unittest tests.test_radar_tracking tests.test_radar_scene -v
if ($LASTEXITCODE -ne 0) { throw "tracking and scene tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- monitor/radar_tracking.py monitor/radar_scene.py tests/test_radar_tracking.py tests/test_radar_scene.py
if ($LASTEXITCODE -ne 0) { throw "tracking staging failed" }
git commit -m "feat(radar): stabilize filtered point tracks"
if ($LASTEXITCODE -ne 0) { throw "tracking commit failed" }
```

Expected: point, hazard, and heatmap scene tests pass; the 10/13 cm hazard regression remains unchanged.

---

### Task 3: Integrate the probabilistic map, resets, diagnostics, and compatibility API

**Files:**
- Modify: `sensors/radar_stack_processes.py:12-23, 122-170`
- Modify: `monitor/radar_scene.py:223-307, 473-717, 1167-1280, 1478-1511`
- Modify: `monitor/radar_front.py:238-345, 569-701`
- Modify: `monitor/web/radar_scene.js:81-105`
- Modify: `tests/test_radar_scene.py:781-869, 956-969, 1220-1378, 1400-1435`
- Modify: `tests/test_radar_front.py:106-152, 298-325, 1197-1418`
- Modify: `tests/web/radar_scene.test.js:94-233`
- Modify: `tests/test_radar_stack_processes.py:74-150, 1310-1345`

**Interfaces:**
- Consumes: `ProbabilisticRollingGrid` and point `map_confirmed`; existing confirmed heatmap tracks remain valid strong evidence.
- Produces: `scene.pose_mode="robot_relative_short_memory"`, `scene.probabilistic_local_map`, compatibility `scene.rolling_local_map`, and additive diagnostics.
- Reset contract: producer change, duplicate/out-of-order transition, profile/calibration mismatch, source fault, replay restart, and explicit reset clear the map and counters.

- [ ] **Step 1: Replace permanent-map expectations with failing probability-map tests**

Add or replace tests in `tests/test_radar_scene.py`:

```python
from sensors.radar_calibration import ClutterPointCluster

CALIBRATION_ID = "radar-clutter-scene-test"

def confirmed_probability_scene_fixture():
    estimator = calibrated_estimator()
    for frame_number in range(1, 4):
        estimator.ingest(point_frame(0.30, frame_number=frame_number, snr_db=24.0),
                         received_at=1.0 + frame_number * 0.125)
    return estimator.snapshot(now=1.375)

def test_two_hit_point_ghost_never_enters_probability_map(self):
    estimator = calibrated_estimator()
    estimator.ingest(point_frame(0.30, frame_number=1, snr_db=24.0), received_at=1.000)
    estimator.ingest(point_frame(0.30, frame_number=2, snr_db=24.0), received_at=1.125)
    self.assertEqual(estimator.snapshot(now=1.125)["probabilistic_local_map"]["cells"], [])

def test_two_weak_hits_plus_one_strong_hit_never_enter_probability_map(self):
    estimator = calibrated_estimator()
    for frame_number, snr_db in ((1, 15.0), (2, 15.0), (3, 24.0)):
        estimator.ingest(
            point_frame(0.30, frame_number=frame_number, snr_db=snr_db),
            received_at=1.0 + frame_number * 0.125,
        )
    scene = estimator.snapshot(now=1.375)
    self.assertTrue(scene["tracks"][0]["point_confirmed"])
    self.assertFalse(scene["tracks"][0]["map_confirmed"])
    self.assertEqual(scene["probabilistic_local_map"]["cells"], [])

def test_three_of_five_point_enters_probability_map(self):
    local_map = confirmed_probability_scene_fixture()["probabilistic_local_map"]
    self.assertEqual(local_map["resolution_m"], 0.05)
    self.assertEqual(local_map["occupied_threshold"], 0.75)
    occupied = [cell for cell in local_map["cells"] if cell["occupied"]]
    self.assertEqual(len(occupied), 1)
    self.assertAlmostEqual(occupied[0]["probability"], 0.80, places=3)
    self.assertEqual(occupied[0]["hits"], 1)

def test_probability_map_decays_and_expires_without_new_ingest(self):
    estimator = calibrated_estimator()
    for frame_number in range(1, 4):
        estimator.ingest(point_frame(0.30, frame_number=frame_number, snr_db=24.0),
                         received_at=1.0 + frame_number * 0.125)
    self.assertFalse(estimator.snapshot(now=2.375)["probabilistic_local_map"]["cells"][0]["occupied"])
    self.assertEqual(estimator.snapshot(now=3.875)["probabilistic_local_map"]["cells"], [])

def test_rolling_map_is_exact_occupied_alias(self):
    scene = confirmed_probability_scene_fixture()
    occupied = [cell for cell in scene["probabilistic_local_map"]["cells"] if cell["occupied"]]
    self.assertEqual(scene["rolling_local_map"]["cells"], occupied)
    self.assertEqual(scene["rolling_local_map"]["retention"], "probability_decay_with_ttl")

def test_raw_and_filter_stage_counts_have_distinct_semantics(self):
    model = RadarClutterModel(**{
        **clutter_model().__dict__,
        "point_clusters": (ClutterPointCluster(
            forward_m=0.10, lateral_m=0.0, height_m=0.0,
            radius_m=0.03, observation_fraction=1.0,
        ),),
    })
    estimator = RadarSceneEstimator(RadarAxes(), model)
    estimator.ingest(radar_frame((
        RadarPoint(0.0, 0.10, 0.0, 0.0, 24.0),  # self clutter
        RadarPoint(0.0, 0.30, 0.0, 0.0, 3.0),   # quality reject
        RadarPoint(0.0, 0.40, 0.0, 0.0, 24.0),  # accepted
    )), received_at=1.0)
    diagnostics = estimator.snapshot(now=1.0)["diagnostics"]
    self.assertEqual(diagnostics["raw_point_count"], 3)
    self.assertEqual(diagnostics["self_clutter_rejected_count"], 1)
    self.assertEqual(diagnostics["quality_gate_rejected_count"], 1)
    self.assertEqual(diagnostics["scene_point_count"], 1)

def test_calibration_rejected_frame_still_reports_raw_input_count(self):
    estimator = mismatched_estimator()
    estimator.ingest(radar_frame((
        RadarPoint(0.0, 0.30, 0.0, 0.0, 24.0),
        RadarPoint(0.0, 0.40, 0.0, 0.0, 24.0),
    )), received_at=1.0)
    scene = estimator.snapshot(now=1.0)
    self.assertEqual(scene["calibration_status"], "profile_mismatch")
    self.assertEqual(scene["diagnostics"]["raw_point_count"], 2)
    self.assertEqual(scene["diagnostics"]["self_clutter_rejected_count"], 0)
    self.assertEqual(scene["diagnostics"]["quality_gate_rejected_count"], 0)
```

At the same RED stage, make `clutter_model()` use `CALIBRATION_ID`, add `calibration_id=CALIBRATION_ID` to the `radar_frame(...)` helper signature, and pass it into `SensorHeader`. Add a regression that seeds an occupied probability cell, ingests a frame with `calibration_id="different-calibration"`, and asserts `calibration_status == "calibration_mismatch"`, both map payloads are empty, and hazard/tracks are cleared. Existing profile-mismatch tests must continue to report `profile_mismatch`, proving the two gates are distinct.

Import `replace` from `dataclasses` in `tests/test_radar_scene.py` and add `bind_calibration(frames, calibration_id)` that returns cloned `RadarFrame` values with cloned headers carrying that calibration ID. In the existing `test_real_977cm_clutter_is_rejected_without_erasing_transient_points`, build the model from the original historical fixture, then ingest `bind_calibration(load_real_fixture(), model.calibration_id)`; never edit the captured fixture bytes. Task 8 reuses the same helper for its six-frame replay test. In `tests/test_radar_front.py`, add `calibration_id="radar-clutter-front-test"` to its `radar_frame(...)` helper and pass that argument into the header instead of hard-coding `"uncalibrated"`; use an explicit different value only in the calibration-mismatch regression.

Update the pre-existing `test_scene_v1_declares_robot_relative_pose_and_exact_grid_geometry` assertion from `robot_relative` to `robot_relative_short_memory`. Delete the obsolete `test_rolling_map_keeps_half_centimeter_transform`: the permanent 5 mm store is removed in this task, and Task 1's transform/re-quantization tests now own the 5 cm probability-map contract.

Also delete `test_rolling_map_retains_strong_obstacle_inside_fifty_centimeters` and `test_rolling_map_moves_and_rotates_around_robot_center`: the first asserts the removed permanent-retention contract and the second directly seeds the deleted `_rolling_map_cells` store. Task 1's direct decay/TTL/transform tests and this task's probability-map integration tests replace both. Update `test_only_strong_confirmed_point_is_retained_for_five_seconds` to ingest three strong frames at `1.0`, `1.1`, and `1.2` so the new 3-of-5 `map_confirmed` admission can still feed the compatibility display; move its retained/expired checks to `6.199`/`6.2` and keep the exact one-millisecond remaining assertion.

Add `test_scene_epoch_is_stable_until_a_reset_boundary`: assert a new estimator reports integer `diagnostics.scene_epoch == 0`, a normal accepted ingest and repeated snapshots keep it at zero, explicit `reset(...)` changes it to one, and the next normal ingest keeps it at one. Extend the existing table-driven producer/transition/profile/calibration/fault/replay reset tests to capture the prior epoch and assert every boundary produces a strictly larger value.

Update `test_near_obstacle_expires_from_path_after_one_second`: retain the compatibility-display assertions, but at `now=3.0` require the decoded `scene.grid` to contain only UNKNOWN zeros because the reactive cell expired and the probability cell has decayed below occupancy. Old five-second stable-display memory must no longer feed the safety grid. In `RadarFrontStateTests.test_default_ti_axes_map_y_forward_and_x_right`, change the exact point-field list from five to six entries by appending `"noise_db"`; give the fixture point a finite noise value and assert it appears at mapped index 5 while the original five indices remain unchanged.

In `tests/test_radar_stack_processes.py`, patch `sensors.radar_stack_processes.load_clutter_model` in every capture-command/start test that uses the placeholder `clutter.json`, return a model-like object with `calibration_id="radar-clutter-live-test"`, and replace the expected `--calibration-id uncalibrated` pair with that ID. Add an assertion that the loader receives `config.calibration_path` exactly once for each command construction; a missing, malformed, or inconsistent calibration file must therefore fail before the capture child starts.

Add module-scope `seed_probability_cell(estimator, producer_id="radar-scene-producer", start=1.0)` that ingests three literal 24 dB point frames 125 ms apart, plus `assert_probability_maps_empty(test_case, scene)` that checks both `probabilistic_local_map.cells` and `rolling_local_map.cells`. Add `RadarSceneEstimatorTests` methods for explicit reset, producer change, duplicate and out-of-order transitions, mismatched profile, and `snapshot(source_status="fault")`; each must first assert the helper created one occupied cell, trigger exactly one boundary using the existing `radar_frame` helper, then assert both maps empty. Extend `RadarFrontStateTests.test_replay_loop_restart_clears_scene_once_then_accepts_new_frame` to seed three strong frames before restart and assert both maps are empty on the restart snapshot.

Extend the existing `test_scene_extension_retains_legacy_snapshot_fields` setup in `tests/test_radar_front.py` with:

```python
    self.assertEqual(snapshot["version"], 1)
    self.assertIn("grid", snapshot["scene"])
    self.assertIn("frame", snapshot)
    self.assertEqual(snapshot["scene"]["pose_mode"], "robot_relative_short_memory")
    self.assertEqual(snapshot["scene"]["probabilistic_local_map"]["resolution_m"], 0.05)
    json.dumps(snapshot, allow_nan=False)
```

In the same RED edit, update the API-payload pose assertion in `RadarFrontHttpTests.test_api_and_offline_assets_are_served` from `robot_relative` to `robot_relative_short_memory`. Leave the HTML title/build/copy assertions unchanged until Task 7; this immediate assertion change is required for Task 3's full `tests.test_radar_front` GREEN gate.

Add this transitional consumer test to `tests/web/radar_scene.test.js` so producer and bundled parser change atomically before strict probability parsing in Task 6:

```js
test("accepts the short-memory pose enum during additive map rollout", () => {
  const snapshot = makeSnapshot();
  snapshot.scene.pose_mode = "robot_relative_short_memory";
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.poseMode, "robot_relative_short_memory");
});
```

- [ ] **Step 2: Run scene tests and verify RED**

Run:

```powershell
python -B -m unittest tests.test_radar_scene -v
$pythonRedExit = $LASTEXITCODE
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --test tests\web\radar_scene.test.js
$nodeRedExit = $LASTEXITCODE
if ($pythonRedExit -eq 0) { throw "expected probability-scene Python RED failure" }
if ($nodeRedExit -eq 0) { throw "expected short-memory parser RED failure" }
```

Expected: Python fails because the old 5 mm rolling map has no probability, decay, or TTL, and Node fails because the current parser rejects the new pose enum.

- [ ] **Step 3: Wire the grid into scene ingestion and reset**

Construct one `ProbabilisticRollingGrid` in `RadarSceneEstimator.__init__` with the Global Constraints and call `clear()` from `_clear_evidence`. Replace `_update_rolling_map()` with:

```python
def _update_probability_map(self, received_at: float) -> None:
    for track in self._strong_updated_tracks():
        self._probability_grid.add_hit(
            float(track["forward_m"]),
            float(track["lateral_m"]),
            float(track["confidence"]),
            str(track["source"]),
            float(track.get("range_uncertainty_m", 0.0)),
            float(track.get("lateral_uncertainty_m", 0.0)),
            received_at,
        )
```

For point tracks, `_strong_updated_tracks()` requires `map_confirmed is True` and quality `>= 0.70`; for heatmap tracks retain the existing confirmed/peak/strength rules. Remove `_rolling_map_cells`, `ROLLING_MAP_RESOLUTION_M`, `ROLLING_MAP_MIN_HITS`, and the permanent retention code after tests are green. In the same task, remove the old `_rolling_map_cells` loop/assignment from `_transform_tracks()` so Task 3's existing raw-motion path cannot reference deleted state; retain an independently named 0.50 m trail-radius constant where still needed. Task 4 will add the probability-grid transform only at its confirmed-motion gate.

Call `_update_probability_map(received_at)` exactly once per accepted frame after track updates. The exact `probability=0.80, hits=1` assertion above prevents an accidental duplicate hit in one frame.

In `sensors/radar_stack_processes.py`, import `load_clutter_model`, load `config.calibration_path` inside `build_capture_command`, and pass the returned immutable model's `calibration_id` to `radar-live` instead of the literal `"uncalibrated"`. Do not catch or downgrade model-load/validation errors. In `RadarSceneEstimator._binding_status`, retain the existing profile/axes check first, then return `"calibration_mismatch"` unless `frame.header.calibration_id == self.clutter_model.calibration_id`, before heatmap geometry/range-step validation. Synthetic mode remains the only bypass.

- [ ] **Step 4: Serialize the additive API and diagnostics**

In `snapshot()` obtain `probability_map = self._probability_grid.payload(observed_at)` and `occupied_cells = self._probability_grid.occupied_cells(observed_at)`. Emit:

```python
"pose_mode": "robot_relative_short_memory",
"probabilistic_local_map": probability_map,
"rolling_local_map": {
    "radius_m": 0.50,
    "resolution_m": 0.05,
    "minimum_hits": 1,
    "motion_relative": True,
    "retention": "probability_decay_with_ttl",
    "cells": occupied_cells,
},
```

Keep `scene.grid`, but populate it from confirmed tracks, confirmed reactive cells, and occupied probability cells only. Preserve existing diagnostic keys and add:

```text
raw_point_count
self_clutter_rejected_count
quality_gate_rejected_count
tentative_point_track_count
confirmed_point_track_count
heatmap_reflection_count
confirmed_heatmap_track_count
probability_active_cell_count
probability_occupied_cell_count
probability_decayed_cell_count
probability_ttl_expired_cell_count
path_switch_pending_count
path_switch_accepted_count
scene_epoch
```

The two probability removal counters are cumulative since the last reset; active/occupied and track counters describe the current snapshot. `scene_epoch` is a monotonic non-negative integer for the estimator lifetime: initialize `_scene_epoch=0`, increment it at the start of every `_clear_evidence` call, never assign it inside the clear body, and serialize it in diagnostics. Constructor initialization is not a reset boundary, normal ingests/snapshots do not change it, and multiple real clear calls are allowed to advance it by more than one. In `monitor/radar_front.py`, append `noise_db` to each raw debug array and `frame.point_fields`; retain the prior five elements in the same order.

Define `raw_point_count` as `len(frame.points)` captured before scene-range, calibration self-clutter, or quality filtering. Define `self_clutter_rejected_count` and `quality_gate_rejected_count` as current-frame stage counts, while preserving legacy `scene_point_count` as accepted in-scene point count. If a frame is rejected at the calibration/profile gate, clear evidence/cumulative state as required but still expose that frame's raw input count; downstream filter-stage counts remain zero because point filtering did not run.

- [ ] **Step 5: Implement reset, HTTP, and transitional browser compatibility**

Route all listed scene boundaries through `_clear_evidence`, which advances `scene_epoch` and clears the probability grid and later path state. In `monitor/web/radar_scene.js`, temporarily accept either `robot_relative` or `robot_relative_short_memory` while leaving the old rolling-map presentation unchanged; Task 6 will add strict parsing and true-legacy fallback. This is the only non-additive enum extension in schema v1, required by the approved design, so bundled consumer support lands in the same commit before the producer payload can be deployed.

- [ ] **Step 6: Run backend API regressions and commit**

Run:

```powershell
python -B -m unittest -v `
  tests.test_radar_local_map `
  tests.test_radar_tracking `
  tests.test_radar_scene `
  tests.test_radar_front `
  tests.test_radar_stack_processes
if ($LASTEXITCODE -ne 0) { throw "backend API Python tests failed" }
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --test tests\web\radar_scene.test.js
if ($LASTEXITCODE -ne 0) { throw "transitional parser tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- sensors/radar_stack_processes.py monitor/radar_scene.py monitor/radar_front.py monitor/web/radar_scene.js tests/test_radar_scene.py tests/test_radar_front.py tests/test_radar_stack_processes.py tests/web/radar_scene.test.js
if ($LASTEXITCODE -ne 0) { throw "probability API staging failed" }
git commit -m "feat(radar): expose decaying probability map"
if ($LASTEXITCODE -ne 0) { throw "probability API commit failed" }
```

Expected: all listed suites pass, bundled old/new pose parsing works, and legacy root fields remain present.

---

### Task 4: Restrict radar-only motion to confirmed point correspondences

**Files:**
- Create: `monitor/radar_motion.py`
- Create: `tests/test_radar_motion.py`
- Modify: `monitor/radar_scene.py:25-33, 258-260, 291-293, 332-368, 720-1075`
- Modify: `tests/test_radar_scene.py:477-588, 846-868`

**Interfaces:**
- Consumes: explicit one-to-one pairs `((old_forward, old_lateral), (new_forward, new_lateral))` whose old side is a strong `map_confirmed` point track and whose new side has quality `>= 0.70`.
- Produces: `RadarMotionEstimator.update(correspondences, eligible_anchor_count)` with the existing motion payload keys.
- Applies: transforms to tracks, short-memory cells, near/stable legacy memory, and trail only when `applied is True`. Rejected motion leaves existing coordinates fixed; normal decay still occurs, and current strong confirmed measurements may still add ordinary hits after track upsert.

- [ ] **Step 1: Write failing motion-estimator gate tests**

Create `tests/test_radar_motion.py`:

```python
import math
import unittest

from monitor.radar_motion import RadarMotionEstimator


def translated_pairs(count=5, delta_forward=-0.02):
    old = [(0.25 + index * 0.08, -0.16 + index * 0.07) for index in range(count)]
    return [(point, (point[0] + delta_forward, point[1])) for point in old]


def rotated_pairs(yaw_deg):
    angle = math.radians(yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        (old, (
            cosine * old[0] - sine * old[1],
            sine * old[0] + cosine * old[1],
        ))
        for old, _ in translated_pairs(5, 0.0)
    ]


class RadarMotionEstimatorTests(unittest.TestCase):
    def test_requires_five_confirmed_correspondences(self):
        result = RadarMotionEstimator().update(translated_pairs(4), 4)
        self.assertFalse(result["applied"])
        self.assertEqual(result["status"], "insufficient_points")

    def test_rejects_match_ratio_below_seventy_percent(self):
        result = RadarMotionEstimator().update(translated_pairs(5), 8)
        self.assertFalse(result["applied"])
        self.assertEqual(result["status"], "low_confidence")

    def test_rejects_rmse_over_five_centimeters(self):
        pairs = translated_pairs(5)
        pairs[2] = (pairs[2][0], (pairs[2][1][0], pairs[2][1][1] + 0.16))
        result = RadarMotionEstimator(stabilization_frames=1).update(pairs, 5)
        self.assertFalse(result["applied"])
        self.assertEqual(result["status"], "high_rmse")
        self.assertGreater(result["rmse_m"], 0.05)

    def test_rejects_translation_and_yaw_limits_independently(self):
        translation = RadarMotionEstimator(stabilization_frames=1).update(
            translated_pairs(5, -0.081), 5,
        )
        yaw = RadarMotionEstimator(stabilization_frames=1).update(
            rotated_pairs(8.01), 5,
        )
        self.assertEqual(translation["status"], "implausible_motion")
        self.assertEqual(yaw["status"], "implausible_motion")
        self.assertFalse(translation["applied"])
        self.assertFalse(yaw["applied"])

    def test_invalid_values_and_anchor_counts_are_guarded(self):
        pairs = translated_pairs(5)
        pairs[0] = (pairs[0][0], (float("nan"), pairs[0][1][1]))
        self.assertEqual(
            RadarMotionEstimator().update(pairs, 5)["status"],
            "invalid_input",
        )
        self.assertEqual(
            RadarMotionEstimator().update(translated_pairs(5), 4)["status"],
            "invalid_input",
        )
        self.assertEqual(
            RadarMotionEstimator().update([], 0)["status"],
            "insufficient_points",
        )

    def test_inconsistent_estimate_restarts_stabilization_and_reset_forgets_it(self):
        estimator = RadarMotionEstimator(stabilization_frames=2)
        estimator.update(translated_pairs(5, -0.02), 5)
        unstable = estimator.update(translated_pairs(5, 0.04), 5)
        self.assertEqual(unstable["status"], "unstable")
        self.assertFalse(unstable["applied"])
        self.assertTrue(estimator.update(
            translated_pairs(5, 0.04), 5,
        )["applied"])
        estimator.reset()
        after_reset = estimator.update(translated_pairs(5, 0.04), 5)
        self.assertEqual(after_reset["status"], "stabilizing")
        self.assertFalse(after_reset["applied"])

    def test_two_consistent_five_match_estimates_are_applied(self):
        estimator = RadarMotionEstimator(stabilization_frames=2)
        first = estimator.update(translated_pairs(5), 5)
        second = estimator.update(translated_pairs(5), 5)
        self.assertEqual(first["status"], "stabilizing")
        self.assertFalse(first["applied"])
        self.assertEqual(second["status"], "ok")
        self.assertTrue(second["applied"])
        self.assertAlmostEqual(second["transform_forward_m"], -0.02, delta=0.005)

    def test_ego_payload_is_inverse_transform_and_confidence_is_bounded(self):
        yaw = math.radians(5.0)
        translation = (0.02, -0.01)
        pairs = []
        for old, _ in translated_pairs(5, 0.0):
            rotated = (
                math.cos(yaw) * old[0] - math.sin(yaw) * old[1],
                math.sin(yaw) * old[0] + math.cos(yaw) * old[1],
            )
            pairs.append((old, (
                rotated[0] + translation[0],
                rotated[1] + translation[1],
            )))
        result = RadarMotionEstimator(stabilization_frames=1).update(pairs, 5)
        expected_ego = (
            -(math.cos(yaw) * translation[0] + math.sin(yaw) * translation[1]),
            -(-math.sin(yaw) * translation[0] + math.cos(yaw) * translation[1]),
        )
        self.assertAlmostEqual(result["transform_forward_m"], 0.02, places=4)
        self.assertAlmostEqual(result["transform_lateral_m"], -0.01, places=4)
        self.assertAlmostEqual(result["transform_yaw_deg"], 5.0, places=3)
        self.assertAlmostEqual(result["ego_forward_m"], expected_ego[0], places=4)
        self.assertAlmostEqual(result["ego_lateral_m"], expected_ego[1], places=4)
        self.assertAlmostEqual(result["ego_yaw_deg"], -5.0, places=3)
        self.assertGreaterEqual(result["confidence"], 0.0)
        self.assertLessEqual(result["confidence"], 1.0)
```

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```powershell
python -B -m unittest tests.test_radar_motion -v
if ($LASTEXITCODE -eq 0) { throw "expected motion-estimator RED failure" }
```

Expected first run: FAIL because `monitor.radar_motion` does not exist. Then create an importable skeleton with `RadarMotionEstimator` and its exact constructor/method signatures, raising `NotImplementedError` from `update`, and rerun. Require the tests to reach estimator behavior rather than import/fixture failure before Step 3; do not commit the skeleton separately.

- [ ] **Step 3: Implement dependency-free rigid 2D estimation and stabilization**

Create `monitor/radar_motion.py` with `from __future__ import annotations`, `import math`, and only standard-library typing imports, plus these exact defaults:

```python
MOTION_MIN_MATCHES = 5
MOTION_MIN_MATCH_RATIO = 0.70
MOTION_MAX_RMSE_M = 0.05
MOTION_MAX_TRANSLATION_M = 0.08
MOTION_MAX_YAW_DEG = 8.0
MOTION_STABILITY_TRANSLATION_M = 0.05
MOTION_STABILITY_YAW_DEG = 2.0
MOTION_CONFIDENCE_RMSE_SCALE_M = 0.16
```

Expose one `RadarMotionEstimator` class with exact public methods `__init__(stabilization_frames=2)`, `reset() -> None`, and `update(correspondences, eligible_anchor_count) -> dict`.

Compute old/new centroids, then the least-squares 2D rotation and translation:

```python
def rotate(point, yaw_rad):
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        cosine * point[0] - sine * point[1],
        sine * point[0] + cosine * point[1],
    )

cross = sum(ox * ny - oy * nx for (ox, oy), (nx, ny) in centered_pairs)
dot = sum(ox * nx + oy * ny for (ox, oy), (nx, ny) in centered_pairs)
yaw_rad = math.atan2(cross, dot)
rotated_centroid = rotate(old_centroid, yaw_rad)
translation = (
    new_centroid[0] - rotated_centroid[0],
    new_centroid[1] - rotated_centroid[1],
)
residual_squares = []
for old, new in correspondences:
    rotated = rotate(old, yaw_rad)
    residual_squares.append(
        (rotated[0] + translation[0] - new[0]) ** 2
        + (rotated[1] + translation[1] - new[1]) ** 2
    )
rmse = math.sqrt(sum(residual_squares) / len(residual_squares))
match_ratio = len(correspondences) / eligible_anchor_count
```

The solved `transform_*` fields map an old static-scene point into the current radar frame: `p_new = R(yaw) * p_old + translation`. Preserve the existing inverse ego-motion meaning with:

```python
ego_yaw_rad = -yaw_rad
ego_forward_m, ego_lateral_m = rotate(
    (-translation[0], -translation[1]), ego_yaw_rad,
)
ego_yaw_deg = -math.degrees(yaw_rad)
confidence = max(0.0, min(
    1.0,
    match_ratio * max(0.0, 1.0 - rmse / MOTION_CONFIDENCE_RMSE_SCALE_M),
))
```

Return confidence `0.0` when no finite rigid estimate exists. Once an estimate exists, retain the bounded formula for `low_confidence`, `high_rmse`, `implausible_motion`, `stabilizing`, `unstable`, and `ok`; changing `applied` or `status` must not silently change the transform/ego field semantics.

Validate `eligible_anchor_count` as a non-boolean, non-negative integer. Treat zero through four eligible anchors as the normal `insufficient_points` state; use `invalid_input` only for a negative/non-integer count, non-finite coordinates, or `len(correspondences) > eligible_anchor_count`. Check the five-match minimum before dividing, so startup and sparse scenes cannot divide by zero. Then reject ratio below 0.70 (`low_confidence`), RMSE above 0.05 m (`high_rmse`), translations above 0.08 m or yaw above 8 degrees (`implausible_motion`), and consecutive qualifying estimates differing by more than 0.05 m translation or 2 degrees yaw (`unstable`, restarting the streak at one). Preserve payload keys `status`, `applied`, `confidence`, `matched_points`, `match_ratio`, `rmse_m`, `ego_forward_m`, `ego_lateral_m`, `ego_yaw_deg`, `transform_forward_m`, `transform_lateral_m`, and `transform_yaw_deg`.

- [ ] **Step 4: Run the motion-estimator suite and verify GREEN**

Run:

```powershell
python -B -m unittest tests.test_radar_motion -v
if ($LASTEXITCODE -ne 0) { throw "motion-estimator tests failed" }
```

Expected: all minimum-count, ratio, RMSE, plausibility, inverse-payload, stabilization, and reset tests pass before scene orchestration begins.

- [ ] **Step 5: Write failing scene integration tests**

In `tests/test_radar_scene.py`, import `Mock` from `unittest.mock` and add these literal fixtures (TI axes are x=lateral, y=forward):

```python
def point_scan(count: int, offset: float, frame_number: int) -> RadarFrame:
    positions = (
        (0.28, -0.30, -0.12),
        (0.34, -0.15, -0.04),
        (0.42, 0.00, 0.02),
        (0.36, 0.16, 0.08),
        (0.25, 0.28, 0.14),
    )
    return radar_frame(tuple(
        RadarPoint(lateral, forward + offset, height, 0.0, 24.0)
        for forward, lateral, height in positions[:count]
    ), frame_number=frame_number)


def five_point_scan(offset: float, frame_number: int) -> RadarFrame:
    return point_scan(5, offset, frame_number)
```

Add:

```python
def test_tentative_raw_points_cannot_move_probability_map(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    estimator.ingest(five_point_scan(0.0, 1), received_at=1.000)
    estimator.ingest(five_point_scan(-0.02, 2), received_at=1.125)
    motion = estimator.snapshot(now=1.125)["motion_compensation"]
    self.assertFalse(motion["applied"])
    self.assertEqual(motion["matched_points"], 0)

def test_rejected_four_anchor_motion_never_calls_map_transform(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    for frame_number in range(1, 4):
        estimator.ingest(point_scan(4, 0.0, frame_number),
                         received_at=1.0 + frame_number * 0.125)
    before = sorted(
        (cell["forward_m"], cell["lateral_m"])
        for cell in estimator.snapshot(now=1.375)["rolling_local_map"]["cells"]
    )
    real_transform = estimator._probability_grid.transform
    estimator._probability_grid.transform = Mock(wraps=real_transform)
    estimator.ingest(point_scan(4, -0.02, 4), received_at=1.500)
    scene = estimator.snapshot(now=1.500)
    after = sorted(
        (cell["forward_m"], cell["lateral_m"])
        for cell in scene["rolling_local_map"]["cells"]
    )
    self.assertEqual(scene["motion_compensation"]["status"], "insufficient_points")
    estimator._probability_grid.transform.assert_not_called()
    self.assertEqual(after, before)

def test_only_stable_five_match_motion_transforms_short_memory_map(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    for frame_number in range(1, 4):
        estimator.ingest(five_point_scan(0.0, frame_number),
                         received_at=1.0 + frame_number * 0.125)
    before = sorted(cell["forward_m"] for cell in
                    estimator.snapshot(now=1.375)["rolling_local_map"]["cells"])
    estimator.ingest(five_point_scan(-0.02, 4), received_at=1.500)
    estimator.ingest(five_point_scan(-0.04, 5), received_at=1.625)
    scene = estimator.snapshot(now=1.625)
    after = sorted(cell["forward_m"] for cell in scene["rolling_local_map"]["cells"])
    self.assertTrue(scene["motion_compensation"]["applied"])
    self.assertEqual(scene["motion_compensation"]["matched_points"], 5)
    self.assertLess(min(after), min(before))
    self.assertEqual(
        sorted(round(track["height_m"], 2) for track in scene["tracks"]),
        [-0.12, -0.04, 0.02, 0.08, 0.14],
    )

def test_near_cell_transform_collision_merges_uncertainty_and_source(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    estimator._near_cells = {
        (1, -8): {
            "forward_m": 0.025, "lateral_m": -0.200,
            "range_uncertainty_m": 0.08, "lateral_uncertainty_m": 0.03,
            "confidence": 0.90, "source": "point", "hits": 2,
            "reactive": True, "_last_seen_ns": 1_000_000_000,
        },
        (1, -7): {
            "forward_m": 0.025, "lateral_m": -0.175,
            "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.09,
            "confidence": 0.95, "source": "heatmap", "hits": 5,
            "reactive": True, "_last_seen_ns": 1_100_000_000,
        },
    }
    estimator._transform_tracks(0.0, 0.0, math.radians(15.0))
    cell = estimator._near_obstacle_payload(1.1)[0]
    self.assertEqual(len(estimator._near_cells), 1)
    self.assertEqual(cell["range_uncertainty_m"], 0.08)
    self.assertEqual(cell["lateral_uncertainty_m"], 0.09)
    self.assertEqual(cell["confidence"], 0.95)
    self.assertEqual(cell["source"], "mixed")
    self.assertEqual(cell["hits"], 5)
```

Before the RED run, remove/replace the three legacy raw-motion tests `test_radar_scan_matching_compensates_forward_robot_motion`, `test_radar_scan_matching_preserves_sub_centimeter_motion`, and `test_radar_scan_matching_rejects_implausible_frame_jump`. Their four raw matches and deleted `_estimate_ego_motion` helper intentionally violate the new five-confirmed-anchor gate; the pure `RadarMotionEstimator` tests plus the scene tests above are their replacements.

- [ ] **Step 6: Run scene integration tests and verify RED**

Run:

```powershell
python -B -m unittest -v `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_tentative_raw_points_cannot_move_probability_map `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_rejected_four_anchor_motion_never_calls_map_transform `
  tests.test_radar_scene.RadarSceneEstimatorTests.test_only_stable_five_match_motion_transforms_short_memory_map
if ($LASTEXITCODE -eq 0) { throw "expected motion-scene RED failure" }
```

Expected: FAIL because scene ingestion still uses raw-point motion and does not own a `RadarMotionEstimator`.

- [ ] **Step 7: Reorder ingestion around confirmed anchors**

In `RadarSceneEstimator.ingest`:

1. Build `prior_anchors` only from point tracks with `map_confirmed`, quality `>= 0.70`, and age `< 300 ms`.
2. Greedily associate current accepted quality `>= 0.70` measurements to those anchors with `point_association_gate`, using each anchor and measurement at most once.
3. Call `self._motion_estimator.update(correspondences, len(prior_anchors))` before point-track upserts.
4. If and only if `motion["applied"]`, call the existing scene transform and `self._probability_grid.transform(float(motion["transform_forward_m"]), float(motion["transform_lateral_m"]), math.radians(float(motion["transform_yaw_deg"])), now)`.
5. In `_transform_tracks`, transform only forward/lateral values in point `_position_history` triples and preserve each z value unchanged; retain the existing two-value heatmap history shape.
   When distinct `_near_cells` re-bin onto one key, use the same conservative merge as Task 5 ingestion: maximum confidence, range uncertainty, lateral uncertainty, and capped maximum hit count; latest `_last_seen_ns`; and `source="mixed"` when sensor sources differ. Never let the highest-confidence contributor discard a different contributor's larger uncertainty.
6. Upsert current point measurements into the transformed tracks, then relocate the one existing `_update_probability_map(now)` call to this point. It must still run when motion was rejected so current confirmed evidence is not discarded, but it must run exactly once per accepted frame; remove the earlier Task 3 call rather than leaving two call sites. Preserve the Task 3 `probability=0.80, hits=1` scene assertion as the duplicate-hit regression.
7. On every scene reset, call `self._motion_estimator.reset()`.

Remove the old raw `_previous_point_positions`, vote-bin estimator, and duplicated motion constants after all motion tests pass.

- [ ] **Step 8: Run motion and scene regressions and commit**

Run:

```powershell
python -B -m unittest -v `
  tests.test_radar_motion `
  tests.test_radar_local_map `
  tests.test_radar_tracking `
  tests.test_radar_scene
if ($LASTEXITCODE -ne 0) { throw "motion and scene regression tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- monitor/radar_motion.py monitor/radar_scene.py tests/test_radar_motion.py tests/test_radar_scene.py
if ($LASTEXITCODE -ne 0) { throw "motion staging failed" }
git commit -m "feat(radar): gate radar motion by confirmed tracks"
if ($LASTEXITCODE -ne 0) { throw "motion commit failed" }
```

Expected: all tests pass, four-point/raw-only scans cannot move the map, and five stable confirmed correspondences can.

---

### Task 5: Add uncertainty-aware collision checks and path-switch hysteresis

**Files:**
- Create: `monitor/radar_path.py`
- Create: `tests/test_radar_path.py`
- Modify: `monitor/radar_scene.py:34-68, 223-717, 1291-1535, 1536-2157`
- Modify: `tests/test_radar_scene.py:590-955, 1220-1435`

**Interfaces:**
- Consumes: occupied probability cells plus short-lived confirmed reactive evidence; tentative/raw points never enter the planner.
- Produces: `PathSelectionHysteresis.select(candidate, current_blocked, blocked_segments, now_ns)` and additive `planned_path.blocked_previous_path`.
- Every normalized planning obstacle carries an `evidence_kind` (`probability`, `reactive`, `mixed`, or `direct`) and `support_radius_m`: `0.05 / sqrt(2)` for a 5 cm probability cell, `0.025 / sqrt(2)` for a 2.5 cm reactive cell, and the maximum contributing support for a mixed cell. A direct private caller/test defaults to `0.0` but may explicitly supply a finite non-negative physical support for geometry tests. Keep `source` separately for the sensor origin (`point`, `heatmap`, or `mixed`).
- Physical collision radius: `0.15 / 2 + 0.03 + max(range_uncertainty_m, lateral_uncertainty_m) + support_radius_m`. Because the existing planner rasterizes collision space at 2.5 cm, the radius actually blocked by the grid also adds `PATH_COLLISION_GRID_M / sqrt(2)`. Report this term separately as `collision_grid_padding_m`; do not conflate raster padding with obstacle support or silently give a direct track the support radius of a probability cell.

- [ ] **Step 1: Write failing path-selection state-machine tests**

Create `tests/test_radar_path.py`:

```python
import math
import unittest

from monitor.radar_path import PathSelectionHysteresis


def path(curvature, lateral=0.0):
    return {
        "status": "clear",
        "curvature_per_m": curvature,
        "resolution_m": 0.01,
        "length_m": 0.02,
        "points": [[0.0, 0.0], [0.01, lateral], [0.02, lateral]],
    }


class PathSelectionHysteresisTests(unittest.TestCase):
    def test_unblocked_route_switches_after_three_equal_candidates(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        initial = selector.select(path(0.0), False, [], 1_000_000_000)
        self.assertFalse(initial["switch_accepted"])
        self.assertEqual(selector.diagnostics(), {
            "path_switch_pending_count": 0,
            "path_switch_accepted_count": 0,
        })
        first = selector.select(path(1.0, 0.01), False, [], 1_125_000_000)
        second = selector.select(path(1.0, 0.01), False, [], 1_250_000_000)
        third = selector.select(path(1.0, 0.01), False, [], 1_375_000_000)
        self.assertEqual(first["path"]["curvature_per_m"], 0.0)
        self.assertEqual(second["pending_switch_frames"], 2)
        self.assertEqual(third["path"]["curvature_per_m"], 1.0)
        self.assertTrue(third["switch_accepted"])
        self.assertEqual(third["pending_switch_frames"], 0)
        self.assertEqual(selector.diagnostics(), {
            "path_switch_pending_count": 0,
            "path_switch_accepted_count": 1,
        })

    def test_pending_switch_resets_when_candidate_changes(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        selector.select(path(1.0), False, [], 1_125_000_000)
        result = selector.select(path(-1.0), False, [], 1_250_000_000)
        self.assertEqual(result["pending_switch_frames"], 1)
        self.assertEqual(result["path"]["curvature_per_m"], 0.0)

    def test_return_to_selected_curvature_breaks_alternate_consecutive_run(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        selector.select(path(1.0), False, [], 1_125_000_000)
        returned = selector.select(path(0.0), False, [], 1_250_000_000)
        self.assertEqual(returned["pending_switch_frames"], 0)
        selector.select(path(1.0), False, [], 1_375_000_000)
        result = selector.select(path(1.0), False, [], 1_500_000_000)
        self.assertEqual(result["pending_switch_frames"], 2)
        self.assertEqual(result["path"]["curvature_per_m"], 0.0)

    def test_blocked_selected_path_switches_immediately_and_keeps_blocked_overlay(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        result = selector.select(
            path(1.0), True,
            [{"start_index": 1, "end_index": 2, "reason": "confirmed_obstacle"}],
            1_125_000_000,
        )
        self.assertEqual(result["path"]["curvature_per_m"], 1.0)
        self.assertEqual(result["status"], "route_changed_blocked")
        self.assertEqual(result["blocked_previous_path"]["points"][1], [0.01, 0.0])

    def test_blocked_path_preserves_candidate_geometry_status(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        limited = dict(path(1.0), status="limited")
        result = selector.select(
            limited, True,
            [{"start_index": 1, "end_index": 2, "reason": "confirmed_obstacle"}],
            1_125_000_000,
        )
        self.assertEqual(result["status"], "route_changed_blocked")
        self.assertEqual(result["path"]["status"], "limited")
        self.assertIsNotNone(result["blocked_previous_path"])

    def test_repeated_near_field_stop_is_idempotent_and_resume_needs_three_frames(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        clear = path(0.0)
        selector.select(clear, False, [], 1_000_000_000)
        stopped = dict(
            path(0.0), status="blocked_near_field",
            points=[[0.0, 0.0]], length_m=0.0,
        )
        first = selector.select(
            stopped, True,
            [{"start_index": 0, "end_index": 1, "reason": "near_field_overlap"}],
            1_125_000_000,
        )
        held = selector.select(
            stopped, True,
            [{"start_index": 0, "end_index": 0, "reason": "near_field_overlap"}],
            1_250_000_000,
        )
        self.assertTrue(first["switch_accepted"])
        self.assertFalse(held["switch_accepted"])
        self.assertEqual(selector.diagnostics()["path_switch_accepted_count"], 1)
        self.assertEqual(held["path"]["status"], "blocked_near_field")
        self.assertEqual(held["blocked_previous_path"], first["blocked_previous_path"])
        one = selector.select(clear, False, [], 1_375_000_000)
        two = selector.select(clear, False, [], 1_500_000_000)
        three = selector.select(clear, False, [], 1_625_000_000)
        self.assertEqual(one["path"]["status"], "blocked_near_field")
        self.assertEqual(two["pending_switch_frames"], 2)
        self.assertEqual(three["path"]["status"], "clear")

    def test_reset_clears_selected_pending_overlay_and_epoch_counters(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        selector.select(path(1.0), False, [], 1_125_000_000)
        self.assertEqual(selector.diagnostics()["path_switch_pending_count"], 1)
        selector.reset()
        self.assertEqual(selector.diagnostics(), {
            "path_switch_pending_count": 0,
            "path_switch_accepted_count": 0,
        })
        fresh = selector.select(path(-1.0), False, [], 1_250_000_000)
        self.assertFalse(fresh["switch_accepted"])
        self.assertEqual(fresh["pending_switch_frames"], 0)
        self.assertIsNone(fresh.get("blocked_previous_path"))
```

- [ ] **Step 2: Run the path-state tests and verify RED**

Run:

```powershell
python -B -m unittest tests.test_radar_path -v
if ($LASTEXITCODE -eq 0) { throw "expected path-selector RED failure" }
```

Expected first run: FAIL because `monitor.radar_path` does not exist. Then create an importable skeleton with `PathSelectionHysteresis` and its exact constructor/method signatures, raising `NotImplementedError` from `select`, and rerun. Require the tests to reach selector behavior rather than import/fixture failure before Step 3; do not commit the skeleton separately.

- [ ] **Step 3: Implement the three-frame selector**

Create `monitor/radar_path.py` with one `PathSelectionHysteresis` class. Its exact public methods are `__init__(confirm_frames: int = 3)`, `reset() -> None`, `select(candidate, current_blocked, blocked_segments, now_ns) -> dict`, and `diagnostics() -> dict`.

Rules:

- Initial candidate is accepted immediately but is initialization, not a route switch, so it does not increment `path_switch_accepted_count`.
- Same selected curvature updates path geometry, preserves `selected_since_ns`, and clears any pending alternate path/count so confirmation is strictly consecutive.
- Treat `blocked_near_field` versus non-blocked geometry as part of route identity even when both curvatures are zero. Repeating an already-selected identical `blocked_near_field` candidate is idempotent: hold the first blocked overlay, emit no new accepted switch, and do not increment counters. After evidence clears, require three consecutive non-blocked candidates before resuming motion.
- A different, unblocked curvature increments pending only when equal within `1e-9`; a changed candidate resets pending to 1.
- Pending count 3 accepts the new route, increments the accepted-switch counter once, and clears pending path/count so the returned `pending_switch_frames` is 0.
- `current_blocked=True` accepts immediately and returns selector/stability `status="route_changed_blocked"`, while preserving the candidate's own geometric `path["status"]` (`clear`, `limited`, or `blocked_near_field`). Include the old path as `blocked_previous_path={"points": old_path["points"], "segments": blocked_segments}`. Each segment has inclusive integer `start_index`, `end_index`, and non-empty `reason`, indexed against the unfiltered old `points` array.
- `reset()` clears selected/pending paths, per-session pending/accepted counters, and any blocked overlay. Diagnostic switch counters are scoped to the current scene epoch and therefore restart at zero at every reset boundary.
- `diagnostics()` returns exactly `path_switch_pending_count == current pending_switch_frames` (an integer in `0..2`) and `path_switch_accepted_count == cumulative real route switches in the current epoch`. Initial route selection is `(0, 0)`, the third accepted alternate is `(0, 1)`, and `reset()` returns both to zero.

- [ ] **Step 4: Run the path-selector suite and verify GREEN**

Run:

```powershell
python -B -m unittest tests.test_radar_path -v
if ($LASTEXITCODE -ne 0) { throw "path-selector tests failed" }
```

Expected: all initial, three-frame, candidate-change, immediate-block, geometry-status, and reset behaviors pass before scene integration begins.

- [ ] **Step 5: Write failing planner integration tests**

Replace the single-frame reactive and immediate-switch expectations in `tests/test_radar_scene.py` with:

```python
def test_single_high_confidence_point_does_not_enter_path(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    estimator.ingest(radar_frame((RadarPoint(0.0, 0.35, 0.0, 0.0, 30.0),),
                                 frame_number=1), received_at=1.0)
    scene = estimator.snapshot(now=1.0)
    self.assertEqual(scene["near_occupancy"]["cells"], [])
    self.assertEqual(scene["planned_path"]["curvature_per_m"], 0.0)

def test_path_uncertainty_is_added_to_robot_width_margin(self):
    obstacle = {
        "forward_m": 0.30, "lateral_m": 0.18,
        "range_uncertainty_m": 0.04, "lateral_uncertainty_m": 0.08,
        "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
    }
    path = RadarSceneEstimator._plan_path([obstacle])
    self.assertEqual(path["robot_width_m"], 0.15)
    self.assertEqual(path["safety_margin_m"], 0.03)
    self.assertEqual(path["minimum_passage_width_m"], 0.21)
    self.assertEqual(path["range_uncertainty_lateral_inflation_m"], 0.08)
    self.assertEqual(path["obstacle_support_radius_m"], 0.0)
    self.assertAlmostEqual(
        path["collision_grid_padding_m"], 0.025 / math.sqrt(2),
    )
    self.assertAlmostEqual(
        path["maximum_collision_radius_m"],
        0.15 / 2 + 0.03 + 0.08 + 0.025 / math.sqrt(2),
    )

def test_probability_cell_adds_its_half_diagonal_support(self):
    obstacle = {
        "forward_m": 0.30, "lateral_m": 0.18,
        "range_uncertainty_m": 0.04, "lateral_uncertainty_m": 0.08,
        "support_radius_m": 0.05 / math.sqrt(2),
        "evidence_kind": "probability", "source": "point",
    }
    path = RadarSceneEstimator._plan_path([obstacle])
    self.assertAlmostEqual(
        path["maximum_collision_radius_m"],
        0.15 / 2 + 0.03 + 0.08
        + 0.05 / math.sqrt(2) + 0.025 / math.sqrt(2),
    )

def test_maximum_collision_radius_uses_one_obstacles_combined_terms(self):
    obstacles = [
        {"forward_m": 0.30, "lateral_m": -0.20,
         "range_uncertainty_m": 0.10, "lateral_uncertainty_m": 0.02,
         "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point"},
        {"forward_m": 0.30, "lateral_m": 0.20,
         "range_uncertainty_m": 0.01, "lateral_uncertainty_m": 0.01,
         "support_radius_m": 0.07, "evidence_kind": "direct", "source": "point"},
    ]
    path = RadarSceneEstimator._plan_path(obstacles)
    self.assertEqual(path["range_uncertainty_lateral_inflation_m"], 0.10)
    self.assertEqual(path["obstacle_support_radius_m"], 0.07)
    self.assertAlmostEqual(
        path["maximum_collision_radius_m"],
        0.15 / 2 + 0.03 + 0.10 + 0.025 / math.sqrt(2),
    )

def test_planner_rejects_invalid_uncertainty_and_support_values(self):
    base = {
        "forward_m": 0.30, "lateral_m": 0.18,
        "range_uncertainty_m": 0.01, "lateral_uncertainty_m": 0.01,
        "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
    }
    for field in (
        "range_uncertainty_m", "lateral_uncertainty_m", "support_radius_m",
    ):
        for invalid in (-0.01, float("nan"), float("inf"), float("-inf")):
            with self.subTest(field=field, invalid=invalid), self.assertRaises(ValueError):
                RadarSceneEstimator._plan_path([dict(base, **{field: invalid})])

def test_wall_interpolation_conservatively_carries_endpoint_inflation(self):
    endpoints = [
        {
            "forward_m": 0.20, "lateral_m": 0.15,
            "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.07,
            "support_radius_m": 0.01, "confidence": 0.8,
            "evidence_kind": "direct", "source": "point",
        },
        {
            "forward_m": 0.27, "lateral_m": 0.15,
            "range_uncertainty_m": 0.08, "lateral_uncertainty_m": 0.03,
            "support_radius_m": 0.04, "confidence": 0.9,
            "evidence_kind": "probability", "source": "heatmap",
        },
    ]
    interpolated, segment_count = RadarSceneEstimator._interpolate_wall_obstacles(
        endpoints,
    )
    middle = min(
        interpolated,
        key=lambda obstacle: abs(obstacle["forward_m"] - 0.25),
    )
    self.assertEqual(segment_count, 1)
    self.assertAlmostEqual(middle["forward_m"], 0.25, delta=0.013)
    self.assertEqual(middle["range_uncertainty_m"], 0.08)
    self.assertEqual(middle["lateral_uncertainty_m"], 0.07)
    self.assertEqual(middle["support_radius_m"], 0.04)
    self.assertEqual(middle["confidence"], 0.9)
    self.assertEqual(middle["evidence_kind"], "mixed")
    self.assertEqual(middle["source"], "mixed")

def test_per_obstacle_inflation_widens_goal_exclusion_and_clearance(self):
    base = {
        "forward_m": 0.35, "lateral_m": 0.0,
        "range_uncertainty_m": 0.0, "lateral_uncertainty_m": 0.0,
        "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
    }
    inflated = dict(
        base, range_uncertainty_m=0.08, support_radius_m=0.02,
    )
    base_path = RadarSceneEstimator._plan_path([base])
    inflated_path = RadarSceneEstimator._plan_path([inflated])
    self.assertLess(
        inflated_path["goal_corridor_width_deg"],
        base_path["goal_corridor_width_deg"],
    )
    self.assertGreater(
        abs(inflated_path["goal_angle_deg"]), abs(base_path["goal_angle_deg"]),
    )
    expected_clearance = min(
        math.hypot(point[0] - 0.35, point[1]) - (0.075 + 0.03 + 0.08 + 0.02)
        for point in inflated_path["points"]
    )
    self.assertAlmostEqual(
        inflated_path["minimum_clearance_m"], round(expected_clearance, 3),
    )

def test_current_path_collision_can_be_caused_only_by_uncertainty(self):
    selected = {
        "points": [[0.0, 0.0], [0.10, 0.0], [0.20, 0.0], [0.30, 0.0]],
    }
    base = {
        "forward_m": 0.20, "lateral_m": 0.14,
        "range_uncertainty_m": 0.0, "lateral_uncertainty_m": 0.0,
        "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
    }
    self.assertEqual(
        RadarSceneEstimator._path_collision_segments(selected, [base]), [],
    )
    inflated = dict(base, lateral_uncertainty_m=0.04)
    segments = RadarSceneEstimator._path_collision_segments(selected, [inflated])
    self.assertTrue(segments)
    self.assertEqual(segments[0]["reason"], "confirmed_obstacle")

def test_probability_and_reactive_evidence_merge_conservatively(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    estimator._probability_grid.add_hit(
        0.30, 0.10, now=1.0, confidence=1.0,
        source="point",
        range_uncertainty_m=0.08, lateral_uncertainty_m=0.03,
    )
    estimator._near_cells[(12, 4)] = {
        "forward_m": 0.30, "lateral_m": 0.10,
        "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.09,
        "confidence": 0.95, "source": "point", "hits": 2,
        "reactive": True, "_last_seen_ns": 1_000_000_000,
    }
    obstacles = estimator._planning_obstacle_payload(1.0)
    merged = next(item for item in obstacles if item["evidence_kind"] == "mixed")
    self.assertEqual(merged["range_uncertainty_m"], 0.08)
    self.assertEqual(merged["lateral_uncertainty_m"], 0.09)
    self.assertAlmostEqual(merged["support_radius_m"], 0.05 / math.sqrt(2))

def test_confirmed_point_propagates_lateral_uncertainty_to_reactive_cell(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    point = RadarPoint(0.10, 0.30, 0.0, 0.0, 30.0)
    estimator.ingest(radar_frame((point,), frame_number=1), received_at=1.0)
    estimator.ingest(radar_frame((point,), frame_number=2), received_at=1.125)
    cell = estimator.snapshot(now=1.125)["near_occupancy"]["cells"][0]
    self.assertGreater(cell["lateral_uncertainty_m"], 0.0)

def test_above_sensor_confirmed_point_still_blocks_the_2d_path(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    elevated = RadarPoint(0.0, 0.20, 0.50, 0.0, 30.0)
    estimator.ingest(radar_frame((elevated,), frame_number=1), received_at=1.000)
    estimator.ingest(radar_frame((elevated,), frame_number=2), received_at=1.125)
    scene = estimator.snapshot(now=1.125)
    self.assertEqual(scene["tracks"][0]["height_band"], "above_sensor")
    self.assertTrue(scene["near_occupancy"]["cells"])
    self.assertNotEqual(scene["planned_path"]["status"], "clear")

def test_same_key_point_and_heatmap_reactive_hits_merge_conservatively(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    estimator._tracks = [
        {
            "source": "point", "forward_m": 0.30, "lateral_m": 0.10,
            "confidence": 0.90, "point_confirmed": True,
            "range_uncertainty_m": 0.08, "lateral_uncertainty_m": 0.03,
            "_updated_by_point": True,
        },
        {
            "source": "heatmap", "forward_m": 0.30, "lateral_m": 0.10,
            "confidence": 0.95, "heatmap_confirmed": True,
            "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.09,
            "_updated_by_heatmap": True,
        },
    ]
    estimator._update_near_occupancy(1.0)
    cell = estimator._near_obstacle_payload(1.0)[0]
    self.assertEqual(cell["range_uncertainty_m"], 0.08)
    self.assertEqual(cell["lateral_uncertainty_m"], 0.09)
    self.assertEqual(cell["confidence"], 0.95)
    self.assertEqual(cell["source"], "mixed")
    self.assertEqual(cell["hits"], 2)

def test_unblocked_candidate_is_accepted_on_third_ingested_frame(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    straight = estimator._plan_path([])
    alternate = dict(straight, curvature_per_m=1.0)
    with patch.object(estimator, "_plan_path",
                      side_effect=[straight, alternate, alternate, alternate]):
        scenes = []
        for number, when in enumerate((1.0, 1.125, 1.250, 1.375), start=1):
            estimator.ingest(radar_frame((), frame_number=number), received_at=when)
            scenes.append(estimator.snapshot(now=when))
    self.assertEqual(scenes[1]["planned_path"]["curvature_per_m"], 0.0)
    self.assertEqual(scenes[2]["planned_path"]["pending_switch_frames"], 2)
    self.assertEqual(scenes[3]["planned_path"]["curvature_per_m"], 1.0)

def test_confirmed_obstacle_switches_blocked_path_immediately(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    estimator.ingest(radar_frame((), frame_number=1), received_at=1.0)
    original = estimator.snapshot(now=1.0)["planned_path"]["points"]
    point = RadarPoint(0.0, 0.35, 0.0, 0.0, 30.0)
    estimator.ingest(radar_frame((point,), frame_number=2), received_at=1.125)
    estimator.ingest(radar_frame((point,), frame_number=3), received_at=1.250)
    path = estimator.snapshot(now=1.250)["planned_path"]
    self.assertEqual(path["stability_status"], "route_changed_blocked")
    self.assertIn(path["status"], {"clear", "limited"})
    self.assertNotEqual(path["curvature_per_m"], 0.0)
    self.assertEqual(path["blocked_previous_path"]["points"], original)
    self.assertTrue(path["blocked_previous_path"]["segments"])
    self.assertTrue(path["blocked_previous_path"]["segments"][0]["reason"])

def test_confirmed_eleven_centimeter_obstacle_stops_path_before_danger(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    point = RadarPoint(0.0, 0.11, 0.0, 0.0, 30.0)
    estimator.ingest(radar_frame((point,), frame_number=1), received_at=1.000)
    estimator.ingest(radar_frame((point,), frame_number=2), received_at=1.125)
    scene = estimator.snapshot(now=1.125)
    self.assertEqual(scene["hazard"]["level"], "NORMAL")
    self.assertEqual(scene["planned_path"]["status"], "blocked_near_field")
    self.assertEqual(scene["planned_path"]["length_m"], 0.0)
    self.assertEqual(scene["planned_path"]["points"], [[0.0, 0.0]])

def test_repeated_eleven_centimeter_stop_does_not_count_switch_each_frame(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    point = RadarPoint(0.0, 0.11, 0.0, 0.0, 30.0)
    for frame_number in range(1, 6):
        estimator.ingest(
            radar_frame((point,), frame_number=frame_number),
            received_at=1.0 + frame_number * 0.125,
        )
    scene = estimator.snapshot(now=1.625)
    self.assertEqual(scene["planned_path"]["status"], "blocked_near_field")
    self.assertEqual(scene["diagnostics"]["path_switch_accepted_count"], 1)

def test_snapshot_after_init_or_reset_never_plans_without_ingest(self):
    estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
    with patch.object(estimator, "_plan_path",
                      side_effect=AssertionError("snapshot planned")):
        initial = estimator.snapshot(now=1.0)
        estimator.reset("explicit_test")
        reset = estimator.snapshot(now=1.1)
    for scene in (initial, reset):
        self.assertEqual(scene["planned_path"]["points"], [[0.0, 0.0]])
        self.assertEqual(scene["planned_path"]["robot_width_m"], 0.15)
        self.assertEqual(scene["planned_path"]["safety_margin_m"], 0.03)
        self.assertEqual(scene["planned_path"]["minimum_passage_width_m"], 0.21)
        self.assertEqual(scene["planned_path"]["minimum_hold_ms"], 0)
        self.assertEqual(scene["planned_path"]["switch_confirm_frames"], 3)
        self.assertFalse(scene["planned_path"]["switch_accepted"])
        self.assertTrue(scene["planned_path"]["replan_every_frame"])
        self.assertEqual(scene["planned_path"]["pending_switch_frames"], 0)
        self.assertIsNone(scene["planned_path"].get("blocked_previous_path"))
        self.assertEqual(scene["diagnostics"]["path_switch_pending_count"], 0)
        self.assertEqual(scene["diagnostics"]["path_switch_accepted_count"], 0)
```

Import `patch` from `unittest.mock` alongside the Task 4 `Mock` import. The patched candidate test intentionally exercises the once-per-`ingest()` orchestration without depending on a fragile geometric scene. The blocked-route test uses real two-frame point confirmation and real collision geometry.

Convert the pre-existing `test_three_centimetre_margin_blocks_fifteen_centimeter_corridor` into a margin-only fixture by changing both legacy `range_uncertainty_m=0.2` values to `0.0`; retain its expected inflation of `0.0`. The new uncertainty-specific tests above own the non-zero inflation behavior, so the old test no longer contradicts the conservative uncertainty contract or accidentally becomes an origin-overlap case.

Delete the pre-existing `test_path_replans_without_holding_the_whole_route`: it calls the removed `_update_stable_path` twice and requires a one-frame unblocked curvature change, which directly contradicts the new three-consecutive-frame selector. `test_unblocked_candidate_is_accepted_on_third_ingested_frame` is its replacement.

Replace `test_unstable_motion_freezes_selected_path` and its obsolete `replanned_frozen_map` expectation. The new regression must spy that rejected/unstable motion does not call the probability-map transform, while a confirmed obstacle that blocks the currently selected path still produces `stability_status="route_changed_blocked"` immediately. Selector safety no longer accepts motion status as an excuse to hold a blocked route.

At this RED stage, add a table-driven scene reset regression: seed one accepted path plus one pending alternate, then independently trigger explicit reset, producer change, duplicate transition, out-of-order transition, profile mismatch, calibration mismatch, source fault, and replay restart. For every boundary assert `pending_switch_frames == 0`, `path_switch_pending_count == 0`, `path_switch_accepted_count == 0`, and no stale `blocked_previous_path`. This test must fail before Step 7 and proves selector state cannot leak across scene epochs.

- [ ] **Step 6: Run the new scene tests and verify RED**

Run:

```powershell
python -B -m unittest -v tests.test_radar_path tests.test_radar_scene
if ($LASTEXITCODE -eq 0) { throw "expected planner-integration RED failure" }
```

Expected: FAIL because scene obstacles do not yet carry source support/lateral uncertainty, co-located sources are not conservatively merged, and scene ingestion does not yet use the selector.

- [ ] **Step 7: Use one collision geometry for planning and current-path checks**

In `monitor/radar_scene.py`:

- `_update_near_occupancy` accepts point tracks only when `point_confirmed is True`, and heatmap tracks only when `heatmap_confirmed is True`. Preserve both `range_uncertainty_m` and `lateral_uncertainty_m` in every reactive cell; do not collapse them into a single range value. Remove its `PATH_NEAR_FIELD_EXCLUSION_M <= distance` lower bound completely: accept every finite, in-FOV confirmed track from zero through `PATH_LOCAL_MAX_RANGE_M`, leaving self-clutter rejection to calibration. When point and heatmap updates land on the same 2.5 cm key in one or later frames, max-merge both uncertainties and confidence, increment hits for both contributions, and set `source="mixed"` instead of allowing the last source to overwrite the first.
- `_planning_obstacle_payload` combines `self._probability_grid.occupied_cells(now)` with those confirmed reactive cells. Normalize probability cells with `evidence_kind="probability"` and `support_radius_m=0.05 / sqrt(2)`; normalize reactive cells with `evidence_kind="reactive"` and `support_radius_m=0.025 / sqrt(2)`.
- Preserve existing private planner callers by defaulting missing uncertainties and `support_radius_m` to `0.0` and missing `evidence_kind` to `"direct"`; validate supplied values as finite and non-negative before collision use.
- Quantize both containers onto the existing 2.5 cm planning key before fusion. For a co-located key, max-merge `range_uncertainty_m`, `lateral_uncertainty_m`, `confidence`, and `support_radius_m`; set `evidence_kind="mixed"`, and set sensor `source="mixed"` only when the contributing sensor sources differ. This prevents the lower-support reactive copy from overwriting a probability cell without conflating sensor provenance with evidence-container provenance.
- Extract the wall-linking/re-sampling loop as `_interpolate_wall_obstacles(obstacles) -> (list[dict], int)`, use it from `_plan_path`, and change its internal `obstacle_positions` from a set of `(forward, lateral)` to a mapping from position to the complete normalized obstacle payload. Every interpolated sample max-merges both endpoint uncertainties, support radii, and confidence; it uses `evidence_kind="mixed"` or `source="mixed"` when those endpoint values differ. This private helper is directly locked by the RED test above so interpolation cannot silently drop inflation before collision planning.
- Build each collision disk with the exact physical per-source radius in this task's interface, then add `PATH_COLLISION_GRID_M / sqrt(2)` only while rasterizing it. Report the independent diagnostic maxima as `range_uncertainty_lateral_inflation_m` and `obstacle_support_radius_m`, and raster padding as `collision_grid_padding_m`. Compute `maximum_collision_radius_m` as `robot_half_width + margin + grid_padding + max_i(uncertainty_i + support_i)` over actual obstacles—not by summing independent maxima that may come from different obstacles.
- Replace the one global `collision_radius` throughout the planner with a per-obstacle physical-radius helper. `select_dynamic_goal()` must compute each angular exclusion with that obstacle's physical radius (without raster padding), collision-grid disks must use physical radius plus grid padding, and `minimum_clearance_m` must subtract that same obstacle's physical radius from point-to-obstacle distance. Thus corridor selection, lattice collision, current-path blocking, and clearance diagnostics cannot disagree about uncertainty/support.
- Remove the existing `< PATH_NEAR_FIELD_EXCLUSION_M` deletion for trusted planning obstacles and replace its old regression. Calibration is responsible for self-clutter; geometry must not discard a confirmed 0.11 m obstacle. If any normalized obstacle's effective collision radius (physical radius plus grid padding) overlaps the robot origin, return a fail-closed geometric path with `status="blocked_near_field"`, `points=[[0.0, 0.0]]`, `length_m=0.0`, `curvature_per_m=0.0`, and the standard diagnostics. Preserve legacy metadata fields with `near_field_exclusion_m=0.0` and `near_field_excluded=0` for one release.
- Add `_path_collision_segments(path, obstacles)` using the same collision-radius helper and 0.01 m path indices. Merge adjacent blocked indices into inclusive `{start_index, end_index, reason}` ranges.
- Call `PathSelectionHysteresis.select` once per ingested frame, never from `snapshot()`.
- Add `minimum_hold_ms=0`, `switch_confirm_frames=3`, `pending_switch_frames`, `switch_accepted`, `replan_every_frame=True`, and optional `blocked_previous_path` to `planned_path`.
- Initialize and reset `_planned_path` to a neutral serialized path with `status="initializing"` or `"reset"`, `stability_status` matching that state, `resolution_m=0.01`, `length_m=0.0`, `curvature_per_m=0.0`, `points=[[0.0, 0.0]]`, `robot_width_m=0.15`, `safety_margin_m=0.03`, `minimum_passage_width_m=0.21`, `minimum_hold_ms=0`, `switch_confirm_frames=3`, `pending_switch_frames=0`, `switch_accepted=False`, `replan_every_frame=True`, and no blocked overlay. Remove the current lazy `_plan_path(...)` call from `snapshot()`; a snapshot before the first accepted ingest or immediately after reset must serialize neutral state without running the planner or selector.
- Reset the selector for every existing scene reset/fault/calibration boundary.

- [ ] **Step 8: Run path, scene, and hazard regressions and commit**

Run:

```powershell
python -B -m unittest -v `
  tests.test_radar_path `
  tests.test_radar_local_map `
  tests.test_radar_scene `
  tests.test_radar_front
if ($LASTEXITCODE -ne 0) { throw "path and scene regression tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- monitor/radar_path.py monitor/radar_scene.py tests/test_radar_path.py tests/test_radar_scene.py
if ($LASTEXITCODE -ne 0) { throw "path staging failed" }
git commit -m "feat(radar): stabilize width-aware local path"
if ($LASTEXITCODE -ne 0) { throw "path commit failed" }
```

Expected: three-frame path hold, immediate blocked switching, uncertainty inflation, 0.21 m passage width, and 10/13 cm hazard tests all pass.

---

### Task 6: Validate and normalize the additive browser scene contract

**Files:**
- Modify: `monitor/web/radar_scene.js:11-22, 65-79, 81-244`
- Modify: `tests/web/radar_scene.test.js:11-49, 94-233, 265-469`

**Interfaces:**
- Consumes: scene schema v1 with the new probability map and filtered track metadata; accepts the legacy `robot_relative` plus `rolling_local_map` pair only as fallback.
- Produces: `presentation.probabilisticLocalMap`, `filteredTracks`, validated `diagnostics`, and existing confirmed/reference/path/hazard fields.
- Preference rule: when `probabilistic_local_map` exists, never use legacy `scene.grid` or `rolling_local_map` as the normal map source.
- Normalization rule: map metadata becomes camelCase (`radiusM`, `resolutionM`, `occupiedThreshold`, `decayHalfLifeMs`, `ttlMs`, `retention`, `poseMode`, `sourceField`); cells, tracks, diagnostics, and path payload keys remain snake_case.
- True legacy rule: a snapshot with no `probabilistic_local_map` returns `probabilisticLocalMap: null`, uses the existing decoded grid/rolling circles, derives `filteredTracks` from cloned legacy scene tracks, and represents unavailable additive diagnostic values as `null`, never as measured zero.

- [ ] **Step 1: Update the JS fixture and add failing probability-contract tests**

Change `makeSnapshot()` in `tests/web/radar_scene.test.js` to include:

```js
pose_mode: "robot_relative_short_memory",
probabilistic_local_map: {
  radius_m: 0.50,
  resolution_m: 0.05,
  occupied_threshold: 0.75,
  decay_half_life_ms: 1000,
  ttl_ms: 2500,
  retention: "probability_decay_with_ttl",
  pose_mode: "robot_relative_short_memory",
  cells: [],
},
planned_path: {
  status: "clear",
  stability_status: "replanned",
  resolution_m: 0.01,
  length_m: 0.02,
  curvature_per_m: 0.0,
  points: [[0.0, 0.0], [0.01, 0.0], [0.02, 0.0]],
  robot_width_m: 0.15,
  safety_margin_m: 0.03,
  minimum_passage_width_m: 0.21,
  minimum_hold_ms: 0,
  switch_confirm_frames: 3,
  pending_switch_frames: 0,
  switch_accepted: false,
  replan_every_frame: true,
  blocked_previous_path: null,
},
diagnostics: {
  raw_point_count: 0,
  self_clutter_rejected_count: 0,
  quality_gate_rejected_count: 0,
  tentative_point_track_count: 0,
  confirmed_point_track_count: 0,
  heatmap_reflection_count: 0,
  confirmed_heatmap_track_count: 0,
  probability_active_cell_count: 0,
  probability_occupied_cell_count: 0,
  probability_decayed_cell_count: 0,
  probability_ttl_expired_cell_count: 0,
  path_switch_pending_count: 0,
  path_switch_accepted_count: 0,
  scene_epoch: 0,
},
```

Add:

```js
test("prefers probability cells over the rolling compatibility alias", () => {
  const probabilityCell = {
    forward_m: 0.30, lateral_m: 0.05, probability: 0.8,
    confidence: 0.9, occupied: true, hits: 3, age_ms: 125,
    range_uncertainty_m: 0.049, lateral_uncertainty_m: 0.06,
    source: "point",
  };
  const parsed = sceneApi.parseRadarScene(makeSnapshot({ scene: {
    probabilistic_local_map: {
      ...makeSnapshot().scene.probabilistic_local_map,
      cells: [probabilityCell],
    },
    rolling_local_map: {
      radius_m: 0.5, resolution_m: 0.05, minimum_hits: 1,
      motion_relative: true, retention: "probability_decay_with_ttl",
      cells: [{ ...probabilityCell, forward_m: 0.45 }],
    },
  }}));
  assert.equal(parsed.probabilisticLocalMap.sourceField, "probabilistic_local_map");
  assert.deepEqual(parsed.probabilisticLocalMap.cells, [probabilityCell]);
  assert.equal(parsed.probabilisticLocalMap.radiusM, 0.50);
  assert.equal(parsed.probabilisticLocalMap.decayHalfLifeMs, 1000);
  assert.equal(parsed.probabilisticLocalMap.ttlMs, 2500);
  assert.equal(
    parsed.probabilisticLocalMap.poseMode,
    "robot_relative_short_memory",
  );
});

test("rejects a probability cell whose occupied flag disagrees with threshold", () => {
  const snapshot = makeSnapshot();
  snapshot.scene.probabilistic_local_map.cells = [{
    forward_m: 0.30, lateral_m: 0, probability: 0.70,
    confidence: 0.8, occupied: true, hits: 1, age_ms: 0,
    range_uncertainty_m: 0.05, lateral_uncertainty_m: 0.05,
    source: "point",
  }];
  assert.throws(() => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/);
});

test("invalid new probability map cannot fall back to a valid rolling alias", () => {
  const snapshot = makeSnapshot();
  snapshot.scene.probabilistic_local_map.radius_m = 0.49;
  assert.throws(() => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/);
});

test("present null or non-object probability map cannot activate legacy fallback", () => {
  for (const invalid of [null, "invalid"]) {
    const snapshot = makeSnapshot();
    snapshot.scene.probabilistic_local_map = invalid;
    assert.throws(
      () => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/,
    );
  }
});
```

Add table-driven RED cases for every strict map field, not just radius: mutate one at a time across `resolution_m`, `occupied_threshold`, `decay_half_life_ms`, `ttl_ms`, `retention`, nested `pose_mode`, non-finite/out-of-radius coordinates, probability at/below 0.5 or above 0.97, confidence outside `0..1`, non-positive/fractional hits, negative/`>=2500` age, negative/non-finite uncertainties, invalid source, and occupied/threshold disagreement. Each case must throw `/probabilistic map contract/`; this table is the executable specification for Step 4.

- [ ] **Step 2: Add failing filtered-height and diagnostics tests**

Add table cases for point heights `-0.11`, `0.0`, `0.11` with `height_reference="sensor_relative_uncalibrated"` and a heatmap `null` with `height_reference="unavailable"`; assert `filteredTracks` preserves them. Add inconsistency cases for `height_m: 0.11` with `height_band: "sensor_level"`, a finite point height with `height_reference="unavailable"`, and a heatmap/null height with `height_reference="sensor_relative_uncalibrated"`; expect a track-contract error for each. Set `diagnostics.raw_point_count=7` and assert the returned cloned diagnostics has 7. Mutate the parsed diagnostics, one parsed cell, one parsed track, and one nested path point and assert none changes the input snapshot.

Add table-driven new-mode filtered-track failures for: missing/zero/negative/fractional `track_id`; negative/fractional `age_ms`; invalid `source`/`classification`; non-finite forward/lateral/distance; `distance_m` differing from `hypot(forward_m, lateral_m)` by more than 1 mm; confidence outside `0..1`; missing/negative/non-finite range or lateral uncertainty; point `classification`, `point_confirmed`, and `map_confirmed` inconsistency; heatmap `classification` and `heatmap_confirmed` inconsistency; and missing source-specific confirmation flags. Keep the true-legacy branch permissive only where the old schema genuinely lacked these fields.

For every additive diagnostic key listed in Task 3, add a table that deletes it, sets it to `null`, `-1`, `0.5`, and `NaN` one case at a time and expects a diagnostics-contract error. This proves the renderer receives validated non-negative integers rather than merely cloned values.

Add table-driven new-mode path failures for each malformed/missing physical/selector field. Require exact finite `robot_width_m=0.15`, `safety_margin_m=0.03`, `minimum_passage_width_m=0.21`, `minimum_hold_ms=0`, `switch_confirm_frames=3`, `replan_every_frame=true`, boolean `switch_accepted`, and integer `pending_switch_frames` in `0..2`. Add a valid `status="blocked_near_field"`, `stability_status="route_changed_blocked"`, `length_m=0`, `points=[[0,0]]`, NORMAL-hazard case and assert both statuses survive parsing so the UI cannot present it as an ordinary zero-length stable route.

Add a parser consistency test that sets `planned_path.pending_switch_frames=1` while `diagnostics.path_switch_pending_count=0` and expects a contract error. Add a table over neutral `status/stability_status` pairs `("initializing", "initializing")` and `("reset", "reset")`, with the full physical/selector fields, `points=[[0,0]]`, `length_m=0`, both switch counters zero, and no blocked overlay; assert both pre-first-ingest and post-reset payloads parse without inventing a route.

Add this regression proving the driving view receives all fresh validated scene tracks rather than only confirmed `obstacle_tracks`:

```js
test("filtered tracks retain tentative point and heatmap reflection evidence", () => {
  const tracks = [
    { track_id: 11, age_ms: 0, source: "point",
      classification: "point_candidate", point_confirmed: false,
      map_confirmed: false,
      forward_m: 0.30, lateral_m: 0.02, distance_m: 0.301,
      height_m: 0.0, height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.6,
      range_uncertainty_m: 0.049, lateral_uncertainty_m: 0.05 },
    { track_id: 12, age_ms: 0, source: "heatmap",
      classification: "reflection_evidence", heatmap_confirmed: false,
      forward_m: 0.35, lateral_m: -0.03, distance_m: 0.351,
      height_m: null, height_band: "unknown",
      height_reference: "unavailable", confidence: 0.5,
      range_uncertainty_m: 0.098, lateral_uncertainty_m: 0.06 },
  ];
  const snapshot = makeSnapshot();
  snapshot.scene.tracks = tracks;
  snapshot.scene.obstacle_tracks = [];
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.deepEqual(parsed.filteredTracks.map((track) => track.track_id), [11, 12]);
  assert.deepEqual(parsed.tracks, []);
});
```

Create a separate `makeLegacySnapshot()` by starting with `makeSnapshot()`, deleting `probabilistic_local_map`, changing `pose_mode` to `robot_relative`, deleting the additive diagnostics, and restoring a pre-change rolling cell without probability metadata. Assert parsing returns `probabilisticLocalMap === null`, `filteredTracks` as cloned scene tracks, and each unavailable new diagnostic as `null`. Do not build the legacy case with a shallow override that leaves the new map attached.

Because `makeSnapshot()` now defaults to the new contract, update every pre-existing track literal used with it—not only the new table cases—to the full production 2.5D shape: common identity/age/coordinates/distance/confidence plus both uncertainties; point tracks include booleans `point_confirmed` and independent `map_confirmed`, finite `height_m`, matching `height_band`, and `height_reference="sensor_relative_uncalibrated"`; heatmap tracks include boolean `heatmap_confirmed`, `height_m=null`, `height_band="unknown"`, and `height_reference="unavailable"`. Point classification follows `point_confirmed`, not `map_confirmed`; 2-of-3 hazard and 3-of-5 map state may legitimately differ. Route only tests intentionally exercising the old schema through `makeLegacySnapshot()`. Do not weaken `validateFilteredTrack` merely to keep incomplete old test fixtures passing.

Likewise, every existing new-mode test that replaces `planned_path` must spread `makeSnapshot().scene.planned_path` before overriding its targeted fields. In particular, update the valid one-centimeter preservation case and invalid-resolution case this way so they test their intended behavior rather than failing because the newly required physical/selector fields were accidentally omitted.

Before running RED, update the existing pose and rolling-map test cases to cover both new and true-legacy pairs. Add `blocked_previous_path` contract cases for the exact shape `{points: [[forward_m, lateral_m], ...], segments: [{start_index, end_index, reason}]}`: reject non-finite or non-pair points, non-integer/out-of-range/reversed inclusive indices, and empty reasons. Retain the existing DANGER consistency case tied to a finite `point_confirmed === true` track within threshold, plus every blocked-status, calibration, grid, no-safe, and no-free regression.

- [ ] **Step 3: Run the parser suite and verify RED**

Run:

```powershell
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --test tests\web\radar_scene.test.js
if ($LASTEXITCODE -eq 0) { throw "expected strict-parser RED failure" }
```

Expected: FAIL because Task 3 only added transitional short-memory pose acceptance; strict probability-map, height/reference, diagnostic, blocked-path, clone, and true-legacy normalization are still absent.

- [ ] **Step 4: Implement strict normalization with one-release fallback**

In `monitor/web/radar_scene.js`, add:

```js
const HEIGHT_BANDS = new Set([
  "below_sensor", "sensor_level", "above_sensor", "unknown",
]);

function expectedHeightBand(heightM) {
  if (heightM === null) return "unknown";
  if (heightM < -0.10) return "below_sensor";
  if (heightM > 0.10) return "above_sensor";
  return "sensor_level";
}
```

Implement `parseProbabilisticLocalMap(scene)`, `validateFilteredTrack(track)`, and `parseDiagnostics(diagnostics, required)` as module-local functions, and call all three from `parseRadarScene` before constructing the returned presentation.

`validateFilteredTrack` enforces the complete Step 2 table: positive-integer identity and non-negative-integer age, finite metric coordinates with distance/hypot agreement within 1 mm, bounded confidence, finite non-negative uncertainties, allowed source/classification, and source-specific confirmation/classification consistency. It accepts only these height/reference pairings: a point with finite `height_m`, its exact derived non-unknown band, and `height_reference="sensor_relative_uncalibrated"`; or a heatmap/no-height track with `height_m=null`, `height_band="unknown"`, and `height_reference="unavailable"`. Reject cross-paired or invented references.

For the new branch, first validate and recursively clone every `scene.tracks` entry, including age, then apply the freshness filter to derive `filteredTracks`; invalid negative/fractional/non-finite age must throw rather than being silently dropped. Do not derive `filteredTracks` from confirmed-only `obstacle_tracks` or the presentation's existing `tracks` field. Continue deriving confirmed guidance `tracks` and `referenceTracks` with their existing semantics.

Validate the new-mode physical and selector path fields pinned in Step 2 before returning `plannedPath`. Require `planned_path.pending_switch_frames === diagnostics.path_switch_pending_count`; both represent the selector's current consecutive pending count and a mismatch is a fail-closed contract error. Keep legacy path parsing backward-compatible when the probability field is truly absent.

Validate exact map metadata `radius_m=0.50`, `resolution_m=0.05`, `occupied_threshold=0.75`, `decay_half_life_ms=1000`, `ttl_ms=2500`, retention string, and nested `pose_mode` equal to the scene's `robot_relative_short_memory`. Cell coordinates must be finite and within radius, probability must be `> 0.5 && <= 0.97`, confidence `0..1`, hits a positive integer, age `>=0 && <2500`, uncertainties finite/nonnegative, and `occupied === (probability >= 0.75)`.

Use `Object.prototype.hasOwnProperty.call(scene, "probabilistic_local_map")` to choose the new branch. For a present new probability payload, require a non-null plain object, `pose_mode="robot_relative_short_memory"`, and every additive diagnostic as a finite non-negative integer. If the field is present but invalid—including `null` or a primitive—throw immediately; never fall back to an alias. Only true property absence selects legacy `pose_mode="robot_relative"`, returns `probabilisticLocalMap=null`, retains existing `rollingMapObstacles`/decoded-grid fallback, and uses `null` for unavailable additive diagnostics. Copy every returned array/object recursively at the map-cell, track, diagnostic, path-point, and blocked-path levels.

- [ ] **Step 5: Implement safety and fallback preservation**

Implement the `blocked_previous_path` validation exercised in Step 2: finite two-number points, integer inclusive indices satisfying `0 <= start_index <= end_index < points.length`, and a non-empty reason. Preserve the existing DANGER consistency rule and all blocked status, calibration, grid, no-safe/no-free behavior while routing new and legacy payloads through their explicit branches.

- [ ] **Step 6: Run parser tests and commit**

Run:

```powershell
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --check monitor\web\radar_scene.js
if ($LASTEXITCODE -ne 0) { throw "radar_scene.js syntax failed" }
& $radarNode --test tests\web\radar_scene.test.js
if ($LASTEXITCODE -ne 0) { throw "parser tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- monitor/web/radar_scene.js tests/web/radar_scene.test.js
if ($LASTEXITCODE -ne 0) { throw "parser staging failed" }
git commit -m "feat(radar): validate probability scene contract"
if ($LASTEXITCODE -ne 0) { throw "parser commit failed" }
```

Expected: all parser tests pass with no external JS dependency.

---

### Task 7: Render the probability map and filtered front-facing 2.5D driving view

**Files:**
- Modify: `monitor/web/radar_panel.js:4-11, 73-169, 247-315, 364-599, 647-693, 863-1241, 1380-1521`
- Modify: `monitor/web/radar_front.html:178-339, 554-708, 721-891`
- Modify: `monitor/radar_front.py:54, 1050-1069`
- Modify: `tests/web/radar_panel.test.js:63-285, 287-358, 375-612`
- Modify: `tests/test_radar_front.py:1275-1418`

**Interfaces:**
- Consumes: normalized `presentation.probabilisticLocalMap`, `filteredTracks`, `diagnostics`, `plannedPath`, and legacy grid fallback.
- Produces: `drawProbabilisticLocalMap`, `drawPointCloud25d`, `drawRobotFootprint`, physical-width path envelope, and optional raw-debug overlays.
- Safety: blocked presentations clear and overlay all canvases; unchecked raw mode never reads raw points for normal rendering.

- [ ] **Step 1: Extend the fake canvas harness and write failing probability-render tests**

In `tests/web/radar_panel.test.js`, make `makeContext()` record `fillRectStyles` as `{fillStyle, globalAlpha, args}`, `strokeRectCalls`, `arcCalls`, `ellipseCalls`, and every numeric `globalAlpha` assignment through a getter/setter-backed `globalAlphaValues` array; expose a readable/writable numeric `shadowBlur` initialized to zero; add no-op `closePath`, `rect`, `clip`, `translate`, and `setLineDash`. Also make `beginPath()` start a recorded path, make `moveTo`/`lineTo` append numeric commands, make `stroke()` append `{strokeStyle, lineWidth, path}` to `strokeCalls`, and make `fill()` append `{fillStyle, path}` to `fillCalls`. Make `ellipse(...args)` append to `ellipseCalls`. Make `makeCanvas()` expose no-op `addEventListener()` and `setPointerCapture()`, then add `#pointcloud-3d-canvas` plus a third 2.5D context to `makeHarness()`. Add `#pointcloud-3d-status` and every new diagnostics ID from Step 6 to the fake element map. Reset all three contexts after panel construction. Update every blocked-overlay regression to expect three clears and three overlays rather than two.

Add:

```js
test("probability cells fade as probability approaches unknown", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[0];
  const transform = panel.makeCircularViewportTransform(
    500, 500, 0.5, { top: 0, right: 0, bottom: 0, left: 0 },
  );
  panel.drawProbabilisticLocalMap(context, transform, {
    resolutionM: 0.05,
    cells: [
      { forward_m: 0.20, lateral_m: 0, probability: 0.80, occupied: true },
      { forward_m: 0.30, lateral_m: 0, probability: 0.60, occupied: false },
    ],
  }, { clipShape: "circle" });
  const alphas = context.globalAlphaValues.filter(
    (value) => value > 0 && value < 1,
  ).slice(-2);
  assert.ok(alphas[0] > alphas[1]);
  assert.deepEqual(context.fillRectCalls[0], [237.5, 137.5, 25, 25]);
  assert.deepEqual(
    context.fillRectStyles.slice(-2).map((call) => call.fillStyle),
    ["rgb(255, 126, 54)", "rgb(92, 228, 208)"],
  );
  assert.equal(context.globalAlphaValues.at(-1), 1);
});

test("probability evidence is nearly invisible as it reaches unknown", () => {
  const { panel, contexts } = makeHarness();
  const transform = panel.makeCircularViewportTransform(
    500, 500, 0.5, { top: 0, right: 0, bottom: 0, left: 0 },
  );
  panel.drawProbabilisticLocalMap(contexts[0], transform, {
    resolutionM: 0.05,
    cells: [{
      forward_m: 0.20, lateral_m: 0, probability: 0.5001, occupied: false,
    }],
  }, { clipShape: "circle" });
  assert.ok(contexts[0].globalAlphaValues.some(
    (value) => value > 0 && value < 0.01,
  ));
});

test("new probability map suppresses both legacy layers in both map views", () => {
  const { panel } = makeHarness();
  let legacyGridCalls = 0;
  let legacyCircleCalls = 0;
  panel.drawEvidenceGrid = () => { legacyGridCalls += 1; };
  panel.drawAccumulatedObstacles = () => { legacyCircleCalls += 1; };
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const presentation = livePresentationWithProbabilityMap();
  panel.drawLidarTopView(makeContext(), transform, presentation);
  panel.drawCollisionInset(makeContext(), transform, presentation);
  assert.equal(legacyGridCalls, 0);
  assert.equal(legacyCircleCalls, 0);
});

test("null probability map preserves both legacy layers in both map views", () => {
  const { panel } = makeHarness();
  let legacyGridCalls = 0;
  let legacyCircleCalls = 0;
  panel.drawEvidenceGrid = () => { legacyGridCalls += 1; };
  panel.drawAccumulatedObstacles = () => { legacyCircleCalls += 1; };
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const presentation = livePresentationWithProbabilityMap();
  presentation.probabilisticLocalMap = null;
  panel.drawLidarTopView(makeContext(), transform, presentation);
  panel.drawCollisionInset(makeContext(), transform, presentation);
  assert.equal(legacyGridCalls, 2);
  assert.equal(legacyCircleCalls, 2);
});

test("both map views invoke probability and robot-footprint renderers", () => {
  const { panel, contexts } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  let probabilityCalls = 0;
  let footprintCalls = 0;
  panel.drawProbabilisticLocalMap = () => { probabilityCalls += 1; };
  panel.drawRobotFootprint = () => { footprintCalls += 1; };
  panel.drawLidarTopView(contexts[0], transform, presentation);
  panel.drawCollisionInset(contexts[1], transform, presentation);
  assert.equal(probabilityCalls, 2);
  assert.equal(footprintCalls, 2);
});

test("both map views draw probability evidence before every track layer", () => {
  const { panel, contexts } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  for (const [context, render] of [
    [contexts[0], () => panel.drawLidarTopView(contexts[0], transform, presentation)],
    [contexts[1], () => panel.drawCollisionInset(contexts[1], transform, presentation)],
  ]) {
    const order = [];
    panel.drawProbabilisticLocalMap = () => { order.push("probability"); };
    panel.drawTracks = () => { order.push("tracks"); };
    render();
    assert.equal(order[0], "probability");
    assert.ok(order.includes("tracks"));
    assert.ok(order.slice(1).every((entry) => entry === "tracks"));
    assert.ok(context);
  }
});
```

Define the test helper exactly as:

```js
function livePresentationWithProbabilityMap() {
  return {
    blocked: false,
    reason: null,
    hazard: { level: "NORMAL", threshold_m: 0.10 },
    probabilisticLocalMap: {
      sourceField: "probabilistic_local_map",
      radiusM: 0.50,
      resolutionM: 0.05,
      occupiedThreshold: 0.75,
      cells: [],
    },
    grid: { cells: [] },
    rollingMapObstacles: [
      { forward_m: 0.25, lateral_m: 0.05, confidence: 0.8, age_ms: 0 },
    ],
    travelTrail: { points: [] },
    plannedPath: {
      points: [[0.00, 0.00], [0.01, 0.00]],
      minimum_passage_width_m: 0.21,
      blocked_previous_path: null,
    },
    referenceTracks: [],
    tracks: [],
    filteredTracks: [],
    diagnostics: {},
  };
}
```

- [ ] **Step 2: Write failing 2.5D, raw-gate, and footprint tests**

Add:

```js
test("2.5d view renders filtered height bands and unknown heatmap height", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[2];
  panel.drawPointCloud25d(context, 420, 300, [
    { source: "point", classification: "confirmed_obstacle", forward_m: 0.30,
      lateral_m: 0.05, distance_m: 0.304, height_m: 0.12,
      height_band: "above_sensor",
      height_reference: "sensor_relative_uncalibrated",
      confidence: 0.8, age_ms: 0 },
    { source: "heatmap", classification: "reflection_evidence", forward_m: 0.35,
      lateral_m: -0.05, distance_m: 0.354, height_m: null,
      height_band: "unknown", height_reference: "unavailable",
      confidence: 0.5, age_ms: 0 },
  ], [], false);
  assert.ok(context.fillTextCalls.some(([text]) => String(text).includes("FILTERED 2")));
  assert.ok(context.fillTextCalls.some(([text]) => String(text).includes("Z UNKNOWN")));
  assert.ok(context.fillTextCalls.some(([text]) => String(text).includes("0.30m")));
});

test("full draw never evaluates frame points while raw debug is off", () => {
  const { panel } = makeHarness();
  panel.presentation = livePresentationWithProbabilityMap();
  panel.snapshot = { frame: {} };
  Object.defineProperty(panel.snapshot.frame, "points", {
    get() { throw new Error("raw accessed"); },
  });
  panel.rawToggle.checked = false;
  assert.doesNotThrow(() => panel.draw());
});

test("raw debug reads points only when checked without changing guidance", () => {
  const { panel } = makeHarness();
  panel.presentation = livePresentationWithProbabilityMap();
  const before = JSON.stringify(panel.presentation);
  let reads = 0;
  panel.snapshot = { frame: {} };
  Object.defineProperty(panel.snapshot.frame, "points", {
    get() { reads += 1; return []; },
  });
  panel.rawToggle.checked = true;
  panel.draw();
  assert.equal(reads, 1);
  assert.equal(JSON.stringify(panel.presentation), before);
});

test("robot footprint uses fifteen centimeters plus three centimeter margins", () => {
  const { panel, contexts } = makeHarness();
  panel.drawRobotFootprint(contexts[0], { originX: 250, originY: 250, scale: 400 });
  const radii = contexts[0].arcCalls.map((args) => args[2]);
  assert.ok(radii.some((radius) => Math.abs(radius - 30) < 0.001));
  assert.ok(radii.some((radius) => Math.abs(radius - 42) < 0.001));
});

test("range and lateral uncertainty render as a thin metric outline", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawUncertaintyOutline(contexts[0], transform, {
    forward_m: 0.30, lateral_m: 0.10,
    range_uncertainty_m: 0.05, lateral_uncertainty_m: 0.08,
  });
  const [x, y, lateralRadius, rangeRadius, rotation] =
    contexts[0].ellipseCalls[0];
  assert.deepEqual([x, y], [290, 130]);
  assert.equal(lateralRadius, 32);
  assert.equal(rangeRadius, 20);
  assert.ok(Math.abs(rotation - Math.atan2(0.10, 0.30)) < 1e-9);
  assert.equal(contexts[0].fillCalls.length, 0);
  assert.equal(contexts[0].shadowBlur, 0);
  assert.ok(contexts[0].strokeCalls.some(
    (call) => call.strokeStyle === "rgba(223, 252, 255, 0.35)" &&
      call.lineWidth === 1,
  ));
});

test("normal map rendering invokes uncertainty outlines for map and track", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const presentation = livePresentationWithProbabilityMap();
  presentation.probabilisticLocalMap.cells = [{
    forward_m: 0.20, lateral_m: 0.0, probability: 0.8, occupied: true,
    range_uncertainty_m: 0.05, lateral_uncertainty_m: 0.06,
    source: "point",
  }];
  presentation.filteredTracks = [{
    forward_m: 0.30, lateral_m: 0.02,
    range_uncertainty_m: 0.04, lateral_uncertainty_m: 0.07,
    source: "heatmap",
  }];
  const outlined = [];
  panel.drawUncertaintyOutline = (_ctx, _transform, evidence) => {
    outlined.push(evidence.source);
  };
  panel.drawLidarTopView(contexts[0], transform, presentation);
  assert.deepEqual(outlined.sort(), ["heatmap", "point"]);
});

test("planned path underlay uses the physical minimum passage width", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawPlannedPath(contexts[0], transform, {
    points: [[0.0, 0.0], [0.10, 0.0], [0.20, 0.0]],
    minimum_passage_width_m: 0.21,
    blocked_previous_path: null,
  });
  assert.ok(contexts[0].strokeCalls.some(
    (call) => Math.abs(call.lineWidth - 84) < 0.001,
  ));
});

test("blocked path segments use original inclusive point indices", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawPlannedPath(contexts[0], transform, {
    points: [[0.0, 0.0], [0.10, -0.02]],
    minimum_passage_width_m: 0.21,
    blocked_previous_path: {
      points: [[-0.01, 0.0], [0.10, 0.0], [0.20, 0.05], [0.30, 0.05]],
      segments: [{ start_index: 1, end_index: 2, reason: "confirmed_obstacle" }],
    },
  });
  const blocked = contexts[0].strokeCalls.find(
    (call) => call.strokeStyle === "rgba(255, 81, 81, 0.92)",
  );
  assert.deepEqual(blocked.path, [
    ["moveTo", 250, 210], ["lineTo", 270, 170],
  ]);
});

test("2.5d view excludes tracks beyond fifty centimeters or sixty degrees", () => {
  const { panel, contexts } = makeHarness();
  panel.drawPointCloud25d(contexts[2], 420, 300, [
    { source: "point", classification: "confirmed_obstacle",
      forward_m: 0.51, lateral_m: 0, distance_m: 0.51, height_m: 0,
      height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
    { source: "point", classification: "confirmed_obstacle",
      forward_m: 0.10, lateral_m: 0.30, distance_m: 0.316, height_m: 0,
      height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
  ], [], false);
  assert.ok(contexts[2].fillTextCalls.some(
    ([text]) => String(text).includes("FILTERED 0"),
  ));
});

test("2.5d markers distinguish confirmed tentative heatmap and raw geometry", () => {
  const { panel, contexts } = makeHarness();
  const tracks = [
    { source: "point", classification: "confirmed_obstacle",
      point_confirmed: true, forward_m: 0.20, lateral_m: -0.04,
      distance_m: 0.204, height_m: 0.0, height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.9, age_ms: 0 },
    { source: "point", classification: "point_candidate",
      point_confirmed: false, forward_m: 0.30, lateral_m: 0.04,
      distance_m: 0.303, height_m: 0.12, height_band: "above_sensor",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.6, age_ms: 0 },
    { source: "heatmap", classification: "reflection_evidence",
      heatmap_confirmed: false, forward_m: 0.40, lateral_m: 0.0,
      distance_m: 0.40, height_m: null, height_band: "unknown",
      height_reference: "unavailable", confidence: 0.5, age_ms: 0 },
    { source: "point", classification: "confirmed_obstacle",
      point_confirmed: true, forward_m: 0.25, lateral_m: 0.0,
      distance_m: 0.25, height_m: -0.12, height_band: "below_sensor",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
  ];
  panel.drawPointCloud25d(
    contexts[2], 420, 300, tracks,
    [[0.25, 0.0, 0.0, 0.0, 24.0, 70.0]], true,
  );
  assert.ok(contexts[2].fillCalls.some(
    (call) => call.fillStyle === "#dffcff",
  ));
  assert.ok(contexts[2].fillCalls.some(
    (call) => call.fillStyle === "#5bbcff",
  ));
  assert.ok(contexts[2].strokeCalls.some(
    (call) => call.strokeStyle === "#ffb84d",
  ));
  assert.ok(contexts[2].fillCalls.some(
    (call) => call.fillStyle === "#a98cff" &&
      call.path.filter(([command]) => command === "lineTo").length >= 3,
  ));
  assert.ok(contexts[2].strokeCalls.some(
    (call) => call.strokeStyle === "rgba(170, 180, 180, 0.55)",
  ));
  assert.ok(contexts[2].fillTextCalls.some(
    ([text]) => String(text).includes("RAW DEBUG 1"),
  ));
  assert.equal(contexts[2].shadowBlur, 0);
  for (const label of ["10cm", "30cm", "50cm"]) {
    assert.ok(contexts[2].fillTextCalls.some(
      ([text]) => String(text).includes(label),
    ));
  }
});

test("2.5d labels only the nearest five points with signed z", () => {
  const { panel, contexts } = makeHarness();
  const tracks = Array.from({ length: 6 }, (_, index) => ({
    source: "point", classification: "confirmed_obstacle",
    point_confirmed: true, forward_m: 0.10 + index * 0.05, lateral_m: 0.0,
    distance_m: 0.10 + index * 0.05, height_m: (index + 1) * 0.01,
    height_band: "sensor_level",
    height_reference: "sensor_relative_uncalibrated",
    confidence: 0.9, age_ms: 0,
  }));
  panel.drawPointCloud25d(contexts[2], 420, 300, tracks, [], false);
  const metricLabels = contexts[2].fillTextCalls
    .map(([value]) => String(value))
    .filter((value) => /^0\.\d{2}m · Z [+-]0\.\d{2}m$/.test(value));
  assert.equal(metricLabels.length, 5);
  assert.ok(metricLabels.includes("0.10m · Z +0.01m"));
  assert.equal(metricLabels.includes("0.35m · Z +0.06m"), false);
});

test("2.5d raw marker is absent when raw debug is off", () => {
  const { panel, contexts } = makeHarness();
  panel.drawPointCloud25d(
    contexts[2], 420, 300, [],
    [[0.25, 0.0, 0.0, 0.0, 24.0, 70.0]], false,
  );
  assert.equal(contexts[2].strokeCalls.some(
    (call) => call.strokeStyle === "rgba(170, 180, 180, 0.55)"), false,
  );
});

test("validated diagnostics and blocked-near-field status drive existing rows", () => {
  const { panel, elements } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  presentation.diagnostics = {
    raw_point_count: 7,
    self_clutter_rejected_count: 2,
    quality_gate_rejected_count: 1,
    tentative_point_track_count: 3,
    confirmed_point_track_count: 4,
    heatmap_reflection_count: 5,
    confirmed_heatmap_track_count: 6,
    probability_active_cell_count: 8,
    probability_occupied_cell_count: 2,
    probability_decayed_cell_count: 9,
    probability_ttl_expired_cell_count: 10,
    path_switch_pending_count: 1,
    path_switch_accepted_count: 12,
  };
  presentation.plannedPath = {
    ...presentation.plannedPath,
    status: "blocked_near_field",
    stability_status: "route_changed_blocked",
    length_m: 0,
    points: [[0, 0]],
  };
  panel.updateDiagnostics(presentation);
  assert.equal(elements.get("#raw-point-value").textContent, "7");
  assert.equal(elements.get("#clutter-rejected-value").textContent, "2");
  assert.equal(elements.get("#reflection-value").textContent, "5");
  assert.equal(elements.get("#confirmed-track-value").textContent, "10");
  assert.equal(elements.get("#quality-rejected-value").textContent, "1");
  assert.equal(elements.get("#probability-active-value").textContent, "8");
  assert.equal(elements.get("#probability-occupied-value").textContent, "2");
  assert.equal(elements.get("#probability-decayed-value").textContent, "9");
  assert.equal(elements.get("#probability-expired-value").textContent, "10");
  assert.equal(elements.get("#path-pending-value").textContent, "1");
  assert.equal(elements.get("#path-accepted-value").textContent, "12");
  assert.match(
    elements.get("#path-status-value").textContent,
    /BLOCKED_NEAR_FIELD.*ROUTE_CHANGED_BLOCKED/,
  );
});
```

Replace the pre-existing `drawPointCloud3d(...)` test at the old panel-test location with an equivalent `drawPointCloud25d(...)` test using filtered-track inputs and the new `FILTERED ... · Z SENSOR-RELATIVE` header. Remove every production and test reference to `drawPointCloud3d`; do not leave an obsolete compatibility alias that could keep raw rendering on the normal path.

Before Step 3's RED run, add two more harness regressions: (1) render live diagnostics once, then a blocked presentation, and assert all seven new rows plus `raw-point-value`, `clutter-rejected-value`, `reflection-value`, and `confirmed-track-value` become `--`; (2) render a live short-memory pose/2.5D status, then block, and assert the pose label does not retain the live pose while `#pointcloud-3d-status` becomes exactly `SENSOR FAULT`. These tests belong to the RED batch, not the implementation step.

- [ ] **Step 3: Run panel tests and verify RED**

Run:

```powershell
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --test tests\web\radar_panel.test.js
if ($LASTEXITCODE -eq 0) { throw "expected panel-renderer RED failure" }
```

Expected: FAIL because the new render methods and filtered 2.5D contract do not exist.

- [ ] **Step 4: Implement probability, physical path, and footprint rendering**

Add `drawProbabilisticLocalMap(ctx, transform, localMap, options)`. Treat each coordinate as a 5 cm cell centre. Use:

```js
const certainty = Math.max(0, Math.min(1,
  (cell.probability - 0.5) / (0.97 - 0.5)));
const alpha = certainty * 0.70;
const color = cell.occupied ? [255, 126, 54] : [92, 228, 208];
```

Set `ctx.globalAlpha=alpha` for each cell and restore it to 1 after the map layer so tests inspect numeric opacity rather than CSS formatting. Draw the new map before tracks. Invoke legacy `drawEvidenceGrid` and old rolling circles only when `probabilisticLocalMap` is `null`.

Add `drawUncertaintyOutline(ctx, transform, evidence)`. Project the evidence centre, orient an ellipse along its radial azimuth, use `lateral_uncertainty_m * scale` for the tangential semi-axis and `range_uncertainty_m * scale` for the radial semi-axis, and stroke it at 1 px with `rgba(223, 252, 255, 0.35)` and no fill/shadow. Draw these thin outlines for probability cells and filtered tracks when both uncertainties are finite/non-negative; they communicate measured uncertainty and never create filled obstacle area or alter planning.

Change the path underlay width to `plannedPath.minimum_passage_width_m * transform.scale`, draw the centreline on top, and draw `blocked_previous_path.segments` with exact color `rgba(255, 81, 81, 0.92)` by their inclusive indices against the original, unfiltered `blocked_previous_path.points`; rely on the canvas clip rather than reindexing a `visiblePoints` array. `drawRobotFootprint` draws circular radii `0.075 * scale` and `0.105 * scale`, matching the planner's circular collision footprint.

- [ ] **Step 5: Replace raw 3D rendering with a crisp filtered 2.5D view**

Rename `drawPointCloud3d` to `drawPointCloud25d(ctx, width, height, filteredTracks, rawPoints, showRaw)`. Keep the draggable projection, but replace the cubic floor grid with a front-facing ±60 degree azimuth/±40 degree elevation wireframe and labelled 10, 30, and 50 cm depth shells. Before projection, exclude tracks with distance outside `0..0.50 m`, non-positive forward distance, or absolute azimuth greater than 60 degrees.

Render:

- confirmed point: filled cross-circle with band colors—blue below (`#5bbcff`), white/cyan sensor-level (`#dffcff`), amber above (`#ffb84d`);
- tentative point: the same geometry as an unfilled thin outline;
- heatmap/no-height: violet (`#a98cff`) diamond on the reference plane with `Z UNKNOWN` label;
- nearest five tracks: exact point label `<distance to 2 decimals>m · Z <signed z to 2 decimals>m` (for example `0.30m · Z +0.12m`); heatmap labels retain `Z UNKNOWN`;
- raw debug: 2 px grey crosses with `rgba(170, 180, 180, 0.55)` only when `showRaw === true`.

Set all marker shadow blur to zero. Header text is `FILTERED <N> · RAW OFF · Z SENSOR-RELATIVE` or `FILTERED <N> · RAW DEBUG <M>`. In `draw()`, pass `presentation.filteredTracks`. Read `snapshot.frame.points` at most once per draw: `const rawPoints = showRaw ? snapshot.frame.points : []`, with the property access itself inside the checked raw-toggle conditional, then pass that cached array to both the old 2D raw-debug overlay and `drawPointCloud25d`. This makes the unchecked getter test stay at zero reads and the checked test stay at exactly one without letting either renderer re-read raw state.

- [ ] **Step 6: Use only validated diagnostics and update HTML copy**

Change `updateDiagnostics` to read `presentation.diagnostics`, not `snapshot.scene.diagnostics`. Map existing rows exactly: `raw-point-value ← raw_point_count`, `clutter-rejected-value ← self_clutter_rejected_count`, `reflection-value ← heatmap_reflection_count`, and `confirmed-track-value ← confirmed_point_track_count + confirmed_heatmap_track_count`. Add these exact HTML IDs, fake-harness elements, and update targets: `quality-rejected-value`, `probability-active-value`, `probability-occupied-value`, `probability-decayed-value`, `probability-expired-value`, `path-pending-value`, and `path-accepted-value`. Render a finite value as its decimal count and `null`/blocked as `--`, satisfying the prewritten live-then-blocked regression. Keep other existing counters as compatibility display rows, but for each optional legacy diagnostic perform a local finite non-negative integer check and show `--` when absent/invalid; never read unvalidated nested snapshot state.

Format `path-status-value` as `<length>cm · <GEOMETRIC STATUS> · <STABILITY STATUS>` so `0cm · BLOCKED_NEAR_FIELD · ROUTE_CHANGED_BLOCKED` is visibly distinct from an ordinary stable zero-length path.

Display `robot_relative_short_memory` as `ROBOT RELATIVE · SHORT MEMORY`. On every blocked presentation, set `#pointcloud-3d-status` to `SENSOR FAULT` before the early return and clear the pose label, satisfying the prewritten stale-live-state regression. Do not edit `monitor/web/radar_front.html` in this step; its production change follows the HTTP RED in Step 7.

- [ ] **Step 7: Write the HTTP asset RED, then synchronize build IDs**

First extend `RadarFrontHttpTests.test_api_and_offline_assets_are_served` to replace every stale R31/`3D POINT CLOUD`/`AZIMUTH ONLY`/`높이 제외`/plain `ROBOT RELATIVE` assertion with the new build, `2.5D FILTERED TRACKS · 0~50cm`, sensor-relative height legend, `ROBOT RELATIVE · SHORT MEMORY`, `5cm PROBABILITY`, `1s DECAY · 2.5s TTL`, robot-width/margin, raw copy, visible collision inset, and all seven diagnostics IDs. Extract the real `#raw-toggle` input tag and assert it has no `checked` attribute. Keep the existing stop-overlay, offline-asset, canvas, and parser-hook assertions.

Run the focused HTTP test before touching production assets:

```powershell
python -B -m unittest -v `
  tests.test_radar_front.RadarFrontHttpTests.test_api_and_offline_assets_are_served
if ($LASTEXITCODE -eq 0) { throw "expected HTTP asset RED failure" }
```

Expected: FAIL on stale build/copy/IDs. Only after observing that RED, update `monitor/web/radar_front.html` to:

- title the panel `2.5D FILTERED TRACKS · 0~50cm`;
- add band legend labels `BELOW`, `SENSOR LEVEL`, `ABOVE`, and `Z UNKNOWN`;
- change checkbox copy to `RAW DEBUG · 표시 전용`;
- add text stating raw debug cannot affect map/path/hazard;
- keep `#raw-toggle` without a `checked` attribute;
- remove the `hidden` attribute from `#collision-inset` so the already-rendered 0–50 cm collision view is actually visible;
- keep all fail-closed warning copy and responsive/fullscreen layout.

Then set all build references to the literal `20260807-probability-25d-r32` in `monitor/radar_front.py`, `monitor/web/radar_panel.js`, the HTML build tag (`UI R32`), and both asset query strings. Do not modify the just-written assertions while making production GREEN.

- [ ] **Step 8: Run browser and HTTP regressions and commit**

Run:

```powershell
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --check monitor\web\radar_scene.js
if ($LASTEXITCODE -ne 0) { throw "radar_scene.js syntax failed" }
& $radarNode --check monitor\web\radar_panel.js
if ($LASTEXITCODE -ne 0) { throw "radar_panel.js syntax failed" }
& $radarNode --test tests\web\radar_scene.test.js tests\web\radar_panel.test.js
if ($LASTEXITCODE -ne 0) { throw "Node radar suites failed" }
python -B -m unittest -v `
  tests.test_radar_front.RadarFrontHttpTests.test_api_and_offline_assets_are_served `
  tests.test_radar_front.RadarFrontStateTests
if ($LASTEXITCODE -ne 0) { throw "radar front HTTP/state tests failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- monitor/web/radar_panel.js monitor/web/radar_front.html monitor/radar_front.py tests/web/radar_panel.test.js tests/test_radar_front.py
if ($LASTEXITCODE -ne 0) { throw "panel staging failed" }
git commit -m "feat(radar): render filtered probability driving view"
if ($LASTEXITCODE -ne 0) { throw "panel commit failed" }
```

Expected: both Node suites and focused Python HTTP/state tests pass; default rendering contains no raw point-cloud dependency.

---

### Task 8: Add replay diagnostics, radar documentation, and final verification

**Files:**
- Create: `scripts/analyze_radar_scene.py`
- Create: `tests/test_radar_replay.py`
- Modify: `docs/radar_front_view.md`

**Interfaces:**
- Consumes: canonical mission JSONL via `sensors.mission_log.iter_replay`, an optional clutter model, `RadarSceneEstimator` snapshots, and an optional strict bench-annotation sidecar.
- Produces: one JSON report on stdout with stage counts, map/path boundedness, processing latency, and—when annotated—reproducible physical-bench metrics; no input or config files are modified.
- Long logs remain optional local inputs and are not committed.

- [ ] **Step 1: Write failing replay-report tests**

Create `tests/test_radar_replay.py`:

```python
from __future__ import annotations

import hashlib
import io
import json
import math
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from common.radar_geometry import RadarAxes
from monitor.radar_scene import RadarSceneEstimator
from scripts.analyze_radar_scene import analyze_records, main
from sensors.mission_log import encode_log_entry
from sensors.radar_calibration import build_clutter_model
from tests.test_radar_scene import (
    bind_calibration,
    five_point_scan,
    load_real_fixture,
    point_frame,
    radar_frame,
)


class RadarReplayReportTests(unittest.TestCase):
    def test_six_frame_fixture_reports_each_filter_stage(self):
        frames = load_real_fixture()
        model = build_clutter_model(frames, RadarAxes(), min_frames=6)
        frames = bind_calibration(frames, model.calibration_id)
        report = analyze_records(
            frames,
            RadarSceneEstimator(RadarAxes(), clutter_model=model),
            max_frames=6,
        )
        self.assertEqual(report["frames_processed"], 6)
        self.assertGreater(report["raw_points_total"], 0)
        self.assertGreater(report["self_clutter_rejected_total"], 0)
        self.assertIn("quality_gate_rejected_total", report)
        self.assertIn("tentative_point_tracks_max", report)
        self.assertIn("confirmed_point_tracks_max", report)
        self.assertIn("probability_cells_max", report)
        self.assertIn("occupied_cells_max", report)
        self.assertIn("path_switches", report)
        self.assertIn("calibration_status_counts", report)
        self.assertIn("motion_status_counts", report)
        self.assertGreaterEqual(report["producer_reported_drops_total"], 0)
        self.assertEqual(report["confirmed_tracks_in_self_clutter_zone_max"], 0)
        self.assertEqual(report["reactive_cells_in_self_clutter_zone_max"], 0)
        self.assertEqual(report["occupied_cells_in_self_clutter_zone_max"], 0)
        self.assertEqual(report["danger_frames"], 0)
        self.assertGreaterEqual(report["processing_ms_p95"], 0.0)
        self.assertIsNone(report["bench"])

    def test_empty_input_has_finite_zero_report(self):
        report = analyze_records(
            [], RadarSceneEstimator(RadarAxes(), synthetic=True), max_frames=10,
        )
        self.assertEqual(report["frames_processed"], 0)
        json.dumps(report, allow_nan=False)

    def test_counter_deltas_survive_scene_counter_reset(self):
        class ResettingEstimator:
            def __init__(self):
                self._counts = iter((0, 1, 2, 0, 1))
                self._epochs = iter((0, 0, 0, 1, 1))

            def ingest(self, _record, received_at):
                self.received_at = received_at

            def snapshot(self, now):
                accepted = next(self._counts)
                scene_epoch = next(self._epochs)
                return {
                    "calibration_status": "synthetic",
                    "motion_compensation": {"status": "insufficient_points"},
                    "probabilistic_local_map": {"cells": []},
                    "tracks": [],
                    "near_occupancy": {"cells": []},
                    "planned_path": {
                        "status": "clear", "stability_status": "replanned",
                        "curvature_per_m": 0.0, "switch_accepted": False,
                    },
                    "hazard": {"level": "NORMAL"},
                    "diagnostics": {
                        "raw_point_count": 0,
                        "self_clutter_rejected_count": 0,
                        "quality_gate_rejected_count": 0,
                        "tentative_point_track_count": 0,
                        "confirmed_point_track_count": 0,
                        "confirmed_heatmap_track_count": 0,
                        "heatmap_reflection_count": 0,
                        "probability_decayed_cell_count": accepted,
                        "probability_ttl_expired_cell_count": accepted,
                        "path_switch_accepted_count": accepted,
                        "scene_epoch": scene_epoch,
                    },
                }

        report = analyze_records(
            [five_point_scan(0.0, index) for index in range(1, 6)],
            ResettingEstimator(),
            max_frames=5,
            timer=iter((0.000, 0.001, 0.001, 0.002, 0.002,
                        0.003, 0.003, 0.004, 0.004, 0.005)).__next__,
        )
        self.assertEqual(report["probability_decayed_total"], 3)
        self.assertEqual(report["probability_ttl_expired_total"], 3)

    def test_latency_summary_uses_nearest_rank_p95_and_max(self):
        ticks = iter((0.000, 0.001, 0.010, 0.015, 0.020, 0.023))
        report = analyze_records(
            [five_point_scan(0.0, index) for index in range(1, 4)],
            RadarSceneEstimator(RadarAxes(), synthetic=True),
            max_frames=3,
            timer=ticks.__next__,
        )
        self.assertAlmostEqual(report["processing_ms_mean"], 3.0)
        self.assertAlmostEqual(report["processing_ms_p95"], 5.0)
        self.assertAlmostEqual(report["processing_ms_max"], 5.0)

    def test_max_frames_must_be_positive_when_supplied(self):
        for value in (0, -1, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                analyze_records([], RadarSceneEstimator(RadarAxes(), synthetic=True),
                                max_frames=value)

    def test_max_frames_truncates_a_longer_input_exactly(self):
        report = analyze_records(
            [five_point_scan(0.0, index) for index in range(1, 6)],
            RadarSceneEstimator(RadarAxes(), synthetic=True),
            max_frames=2,
        )
        self.assertEqual(report["records_seen"], 2)
        self.assertEqual(report["frames_processed"], 2)

    def test_repeated_scene_keeps_map_and_path_state_bounded(self):
        records = [five_point_scan(0.0, index) for index in range(1, 241)]
        report = analyze_records(
            records,
            RadarSceneEstimator(RadarAxes(), synthetic=True),
            max_frames=240,
        )
        self.assertEqual(report["frames_processed"], 240)
        self.assertLessEqual(report["probability_cells_max"], 25)
        self.assertLessEqual(report["occupied_cells_max"], 25)
        self.assertLessEqual(report["path_switches"], 3)
        json.dumps(report, allow_nan=False)

    def test_confirmed_ghost_expires_and_path_returns_straight(self):
        records = [
            point_frame(0.30, frame_number=index, snr_db=24.0)
            for index in range(1, 4)
        ] + [
            radar_frame((), frame_number=index)
            for index in range(4, 31)
        ]
        report = analyze_records(
            records,
            RadarSceneEstimator(RadarAxes(), synthetic=True),
            max_frames=30,
        )
        self.assertGreaterEqual(report["occupied_cells_max"], 1)
        self.assertEqual(report["probability_cells_final"], 0)
        self.assertEqual(report["occupied_cells_final"], 0)
        self.assertAlmostEqual(report["planned_path_curvature_final"], 0.0)

    def test_cli_prints_one_json_report_and_returns_zero(self):
        with TemporaryDirectory() as directory:
            log_path = Path(directory) / "three-frames.jsonl"
            payload = b"".join(
                encode_log_entry(index, frame) + b"\n"
                for index, frame in enumerate(load_real_fixture()[:3], start=1)
            )
            log_path.write_bytes(payload)
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                exit_code = main([
                    "--input", str(log_path), "--synthetic", "--max-frames", "3",
                ])
        self.assertEqual(exit_code, 0)
        self.assertEqual(json.loads(stdout.getvalue())["frames_processed"], 3)

    def test_cli_uses_calibration_axes_and_rejects_conflicting_modes(self):
        frames = load_real_fixture()
        nondefault_axes = RadarAxes(
            forward_axis="x", forward_sign=-1,
            lateral_axis="y", lateral_sign=1,
        )
        model = build_clutter_model(frames, nondefault_axes, min_frames=6)
        with patch("scripts.analyze_radar_scene.load_clutter_model",
                   return_value=model), \
             patch("scripts.analyze_radar_scene.iter_replay", return_value=()), \
             patch("scripts.analyze_radar_scene.RadarSceneEstimator") as constructor:
            constructor.return_value = RadarSceneEstimator(model.axes, synthetic=True)
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main([
                    "--input", "unused.jsonl",
                    "--clutter-calibration", "model.json",
                ]), 0)
            constructor.assert_called_once_with(model.axes, clutter_model=model)

        with self.assertRaises(SystemExit) as caught:
            main([
                "--input", "unused.jsonl", "--synthetic",
                "--clutter-calibration", "model.json",
            ])
        self.assertEqual(caught.exception.code, 2)
        with self.assertRaises(SystemExit) as caught:
            main(["--input", "unused.jsonl"])
        self.assertEqual(caught.exception.code, 2)

    def test_cli_rejects_invalid_limits_and_returns_two_on_threshold_failure(self):
        for option, value in (
            ("--max-frames", "0"),
            ("--max-p95-ms", "nan"),
            ("--max-p95-ms", "inf"),
            ("--max-frame-ms", "-1"),
        ):
            with self.subTest(option=option), self.assertRaises(SystemExit) as caught:
                main(["--input", "unused.jsonl", "--synthetic", option, value])
            self.assertEqual(caught.exception.code, 2)

        threshold_cases = (
            ({"frames_processed": 1, "processing_ms_mean": 6.0,
              "processing_ms_p95": 6.0, "processing_ms_max": 6.0}, "p95"),
            ({"frames_processed": 1, "processing_ms_mean": 4.0,
              "processing_ms_p95": 4.0, "processing_ms_max": 8.0}, "max"),
        )
        for failing_report, label in threshold_cases:
            with self.subTest(label=label), \
                 patch("scripts.analyze_radar_scene.iter_replay", return_value=()), \
                 patch("scripts.analyze_radar_scene.analyze_records",
                       return_value=failing_report), \
                 redirect_stdout(io.StringIO()):
                self.assertEqual(main([
                    "--input", "unused.jsonl", "--synthetic",
                    "--max-p95-ms", "5", "--max-frame-ms", "7",
                ]), 2)
```

Add `test_state_bounds_fail_on_too_many_probability_cells_or_path_points`: use scripted scenes to prove at most 317 distinct 5 cm cell centres can exist in the 0.50 m circle, occupied count cannot exceed active count, and a 1.20 m path sampled every 1 cm contains at most 121 points. Assert exact valid-bound equality passes, each one-over case sets `state_bounds_passed=False` with a named failure, and CLI prints the report then returns 2 for a bound failure even when no optional latency threshold was supplied.

Add `test_counter_delta_uses_scene_epoch_when_post_reset_count_equals_previous`: script probability cumulative values `(0, 1, 1)` with epochs `(0, 0, 1)` and assert both probability report totals equal `2`, proving equality after a reset is counted as new epoch evidence rather than mistaken for an unchanged counter. Add `test_controlled_mixed_records_report_exact_stage_counts_and_final_state`. Feed, in order, one non-radar object, `replace(radar_frame(...), complete=False)`, and two complete frames whose dropped counts are 2 and 1 into a scripted estimator. Script diagnostics so the two accepted frames have raw `(3, 2)`, self-clutter `(1, 0)`, quality rejects `(1, 1)`, heatmap reflections `(2, 1)`, tentative maxima `(1, 0)`, confirmed point maxima `(0, 2)`, confirmed heatmap maxima `(1, 1)`, one accepted blocked switch, stable non-decreasing `scene_epoch`, and distinct final calibration/motion/path values. Assert exact `records_seen=4`, skip counts `1/1`, `frames_processed=2`, drops `3`, every total/max, `blocked_path_replans=1`, final status/stability/curvature, and exact calibration/motion count dictionaries. This prevents the core stage report from passing on field presence alone.

Every scripted estimator diagnostics object in this task must include a non-negative integer `scene_epoch`; use zero for ordinary no-reset fixtures and explicit increasing values only at scripted reset boundaries. Do not make analyzer tests pass by defaulting a missing epoch.

Add a strict, dependency-free bench evaluator to the same test file before the RED run. Pin these constants in both tests and implementation:

```python
BENCH_ASSOCIATION_RADIUS_M = 0.15
BENCH_MIN_POSITION_SAMPLES = 30
BENCH_PERSISTENCE_LIMIT_MS = 2500.0

BENCH_FIXED_TARGETS = {
    "front_030_metal": (0.30, 0.0, "metal"),
    "front_050_metal": (0.50, 0.0, "metal"),
    "front_100_metal": (1.00, 0.0, "metal"),
    "azimuth_neg30_metal": (
        0.50 * math.cos(math.radians(30.0)),
        -0.50 * math.sin(math.radians(30.0)),
        "metal",
    ),
    "azimuth_pos30_metal": (
        0.50 * math.cos(math.radians(30.0)),
        0.50 * math.sin(math.radians(30.0)),
        "metal",
    ),
    "front_050_wood": (0.50, 0.0, "wood"),
}
```

Each hardware JSONL has an untracked sibling `*.radar-bench.json` with this exact top-level shape:

```json
{
  "schema_version": 1,
  "log_file": "front-targets.jsonl",
  "log_sha256": "64 lowercase hex characters",
  "profile_id": "lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-8hz-v1",
  "calibration_id": "radar-clutter-...",
  "provenance": {
    "board_serial": "manual board serial",
    "robot_mount_id": "head-final-mount-v1",
    "mount_orientation": "forward level, connector down",
    "usb_cable_adapter": "manual cable/adapter description",
    "power_state": "manual power-source description",
    "physical_location": "bench/location identifier",
    "ambient_temperature_c": 24.5,
    "created_utc": "2026-08-07T08:00:00Z",
    "operator": "operator name"
  },
  "stream": {
    "mission_id": "captured mission id",
    "unit_id": "head",
    "boot_id": "captured boot id",
    "producer_id": "captured producer id",
    "stream_id": "radar/front"
  },
  "recording": {
    "id": "front-targets-20260807",
    "kind": "frontal_targets",
    "start_seq": 100,
    "end_seq": 620
  },
  "windows": [
    {
      "id": "front-030",
      "kind": "fixed_target",
      "start_seq": 120,
      "end_seq": 180,
      "coverage_key": "front_030_metal"
    }
  ]
}
```

Allow recording kinds exactly `empty_mounted`, `frontal_targets`, `angled_targets`, `materials`, and `approach_retreat`. Every window has a unique non-empty `id`, a supported `kind`, and inclusive integer `start_seq <= end_seq` inside the recording. Window-specific fields are:

| Window kind | Required additional fields |
|---|---|
| `empty` | none |
| `fixed_target` | `coverage_key`, which must be a key in `BENCH_FIXED_TARGETS` |
| `ghost` | `target`, `evidence_seqs` of length exactly 1 or 2 |
| `removal` | `target`, `last_present_seq` |
| `transient_path` | `target`, `event_seq` |
| `confirmed_collision` | `target`, `evidence_seqs` of length exactly 2, `confirmation_seq` equal to the final evidence seq |
| `approach_retreat` | none; distances come from finite `hazard.nearest_confirmed_m` at state transitions |

`target` is exactly `{ "forward_m": finite > 0, "lateral_m": finite }` and event targets must lie within the 0.50 m local-map circle. Every provenance string must be trimmed and non-empty, `ambient_temperature_c` must be finite, and `created_utc` must be a timezone-aware RFC 3339 UTC timestamp ending in `Z`; these values are operator attestations, not automatically discovered hardware facts. Reject duplicate JSON keys, unknown/missing keys at every level, bool-as-int, non-finite values, duplicate window IDs, overlapping windows of the same kind, and any seq not belonging to the sidecar's single `(mission_id, unit_id, boot_id, producer_id, stream_id)` domain. Key annotated frames by that full five-field domain plus seq; a bare seq is never globally unique. The loader must stream SHA-256 over the input and require `log_file == input_path.name`, matching lowercase log hash, model profile/calibration IDs, complete start/end frames, strictly increasing seqs, strictly increasing `header.monotonic_ns`, and recording duration `>= 60.0 s`. Equal or regressing timestamps are invalid because retained tracks with `age_ms == 0` would otherwise masquerade as current detections. Require recording start/end to equal the first/last complete radar frame of that bound log/domain and reject complete radar frames from another domain, so bench latency/path/calibration/switch aggregates cannot be polluted by unannotated frames outside a subrange. Reject `--max-frames` with annotations so a gate cannot silently analyze a truncated window.

Reject vacuous or incompatible coverage by recording kind:

- `empty_mounted`: exactly one `empty` window spanning the recording;
- `frontal_targets`: fixed windows for `front_030_metal`, `front_050_metal`, and `front_100_metal`, plus at least one each of `ghost`, `removal`, `transient_path`, and `confirmed_collision`;
- `angled_targets`: fixed windows for both `azimuth_neg30_metal` and `azimuth_pos30_metal`;
- `materials`: fixed `front_050_metal` and `front_050_wood` windows in the same recording, therefore bound to the exact same sidecar mount/location provenance;
- `approach_retreat`: exactly one `approach_retreat` window.

Every required window must contain at least one complete frame. Event seqs must exist inside their windows. Ghost evidence and both collision evidence seqs must be unique, ordered, and consecutive complete processed frames: every later seq is the prior seq plus one and reports zero dropped frames since the prior record. Transient windows must include the event plus its next two likewise contiguous complete frames. Removal windows must include at least one complete observation whose timestamp is at or after `last_present_time + 2.5 s`, and collision windows must include the confirmation frame. Reject duplicate/reversed/gapped event seqs, missing required coverage, and window kinds incompatible with the recording kind before analysis. A point-backed `confirmed_collision` intentionally confirms on the second frame because path safety consumes the unchanged 2-of-3 `point_confirmed` reactive contract; 3-of-5 remains the separate probability-map admission test.

Event windows also require physical evidence and cannot pass by annotation alone. At every `ghost.evidence_seqs`, `transient_path.event_seq`, and `confirmed_collision.evidence_seqs` frame, require at least one mapped raw point within 0.15 m of `target`. At `removal.last_present_seq`, require a confirmed current track or active/occupied probability cell in the same ROI. Missing evidence is a sidecar/evaluation failure. Define route identity exactly as `(planned_path.status == "blocked_near_field", planned_path.curvature_per_m)` with curvature equality tolerance `1e-9`; ordinary point-array redraws do not count as route switches.

Add these named deterministic tests, using a tiny scripted estimator where exact scene transitions are easier to control:

- `test_annotations_reject_wrong_sha_domain_profile_calibration_provenance_unknown_and_duplicate_keys`
- `test_annotations_reject_empty_or_incomplete_recording_kind_coverage`
- `test_recording_duration_accepts_exact_sixty_seconds_and_rejects_one_ns_less`
- `test_annotations_reject_equal_or_regressing_monotonic_timestamps`
- `test_event_sequences_reject_duplicates_reversal_gaps_and_reported_drops`
- `test_fixed_target_pairs_same_frame_current_samples_for_population_2d_std`: 30 same-frame raw/filtered pairs with raw 0.28/0.32 m and filtered 0.295/0.305 m yield 0.02/0.005 m scatter; 30 pairs pass, 29 fail, equal scatter fails, and a retained `age_ms=125` track on a miss frame contributes no pair/sample/detection.
- `test_empty_metrics_report_exact_false_rates_switches_per_min_and_block_run`: raw counts `(2, 0, 1, 1)` report `1.0/frame`, confirmed counts `(0, 1, 1, 0)` report `0.5/frame`, and two accepted switches across exactly 60 s report `2.0/min`.
- `test_one_and_two_frame_ghosts_never_enter_map_but_three_frame_annotation_is_invalid`
- `test_event_windows_require_target_roi_evidence_and_removal_presence`
- `test_removal_is_clear_on_first_observation_at_or_after_deadline`: clear exactly at 2500 ms passes, a still-present cell at 2500 ms or any later first observation fails, and a first post-deadline observation at 2625 ms passes when already clear.
- `test_transient_path_holds_route_and_confirmed_collision_has_zero_frame_delay`
- `test_hazard_boundaries_accept_exact_010_and_013_and_reject_outside_values`
- `test_blocked_replan_counts_only_switch_accepted_and_reports_blocked_near_run`
- `test_cli_rejects_synthetic_or_max_frames_with_annotations_and_require_without_annotations`
- `test_cli_prints_failed_bench_report_before_returning_two`

Use this exact population 2D scatter for fixed targets, selecting at most one nearest raw point and one nearest `source="point", map_confirmed is True, age_ms == 0` filtered track per frame within 0.15 m of the fixed nominal target. Compute both raw and filtered scatter only from frames where both samples exist, report `paired_target_samples`, and require at least 30 pairs. A retained track and a latest robust outlier rejected without a current update are not detections and must not contribute repeated coordinates:

```python
mean_f = sum(forward for forward, _ in samples) / len(samples)
mean_l = sum(lateral for _, lateral in samples) / len(samples)
position_std_m = math.sqrt(sum(
    (forward - mean_f) ** 2 + (lateral - mean_l) ** 2
    for forward, lateral in samples
) / len(samples))
```

Target detection rate is annotated frames with such a filtered target divided by complete annotated frames. Empty false-obstacle rate is the total number of confirmed point/heatmap tracks per frame divided by complete empty frames. Convert raw points through the estimator/model `RadarAxes.map_point`; never assume TI x/y signs in the analyzer.

For an `empty` window, define a confirmed route-blocked frame exactly as: (a) the scene contains at least one serialized `point_confirmed is True` or `heatmap_confirmed is True` track, or an occupied probability cell; and (b) `planned_path.status == "blocked_near_field"`, `planned_path.stability_status == "route_changed_blocked"`, or `blocked_previous_path` is non-null. Start a contiguous run at the first such frame, close it at the first later non-matching frame using that closing frame's monotonic timestamp, and flush an open run at the window's final monotonic timestamp. `longest_confirmed_route_blocked_ms` is the maximum non-negative elapsed value over those runs; a single matching final frame therefore has zero observed duration rather than an invented frame period.

The annotated report adds:

```json
{
  "bench": {
    "schema_version": 1,
    "annotations_sha256": "...",
    "log_sha256": "...",
    "recording_id": "...",
    "recording_kind": "frontal_targets",
    "provenance": { "...": "strict sidecar provenance echoed verbatim" },
    "duration_s": 60.0,
    "path_switches": 0,
    "path_switches_per_min": 0.0,
    "passed": true,
    "failures": [],
    "windows": []
  }
}
```

Every window reports `id`, `kind`, `frames_evaluated`, `duration_s`, `passed`, and `failures`. Add per-kind fields: empty—`raw_false_points_per_frame`, `confirmed_false_obstacles_per_frame`, `longest_confirmed_route_blocked_ms`; fixed—`coverage_key`, raw/filtered/paired sample counts, `target_detection_rate`, and paired-frame raw/filtered position std; ghost—`evidence_seqs`, `probability_map_entries`; removal—`last_present_seq`, `first_clear_seq`, `ghost_persistence_ms`; transient—`event_seq`, `path_switches_during_event`, `route_identity_changed`; collision—`confirmation_seq`, `response_seq`, `response_delay_frames`, where response is the first frame with `switch_accepted is True` and `stability_status == "route_changed_blocked"`; approach/retreat—entry/release seqs and finite nearest-confirmed distances.

Bench failure semantics are exact after sidecar validation succeeds: any annotated `calibration_status != "ok"`; an empty confirmed route-blocked run `> 2500.0 ms`; fewer than 30 paired fixed samples; filtered paired-frame std greater than or equal to raw paired-frame std; a one/two-frame ghost ever entering the target ROI in the probability map; missing event/raw evidence or removal presence; target-ROI evidence still present on the first complete observation at or after `last_present_time + 2.5 s`; a transient accepted switch or route-identity change from its event through the following two frames; missing collision response or `response_seq != confirmation_seq` (zero-frame response delay is required); missing first `NORMAL -> DANGER` transition or an entry distance `> 0.10 m`; missing/non-finite first later `DANGER -> NORMAL` distance or one `< 0.13 m`; p95/max latency `> 125 ms`. `ghost_persistence_ms` remains the observed first-clear delay (or `null` if never clear), but a sparse first clear after the deadline is not itself a failure when the first available at/after-deadline observation is already clear. Recording duration below 60 s is a schema/binding rejection before analysis—not a reportable bench failure—matching `test_recording_duration_accepts_exact_sixty_seconds_and_rejects_one_ns_less`. Equality at 2.5 s, 10 cm, 13 cm, and 125 ms passes. Raw/confirmed false rates, target detection rate, path switches/min, and drops are report-only because the approved design did not assign arbitrary physical thresholds to them.

- [ ] **Step 2: Run replay tests and verify RED**

Run:

```powershell
python -B -m unittest tests.test_radar_replay -v
if ($LASTEXITCODE -eq 0) { throw "expected replay-analyzer RED failure" }
```

Expected first run: FAIL because `scripts.analyze_radar_scene` does not exist. Then create an importable module skeleton exposing `analyze_records` and `main` with the final signatures and raising `NotImplementedError`; rerun and require analyzer/CLI behavioral failures rather than import/fixture errors before Step 3. Do not commit the skeleton separately.

- [ ] **Step 3: Implement the read-only analyzer**

Create `scripts/analyze_radar_scene.py` with these imports and analysis function:

```python
from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from collections.abc import Sequence
from pathlib import Path

from common.radar_geometry import RadarAxes
from common.sensor_contract import RadarFrame
from monitor.radar_scene import RadarSceneEstimator
from sensors.mission_log import iter_replay
from sensors.radar_calibration import load_clutter_model


def analyze_records(
    records,
    estimator,
    max_frames=None,
    timer=time.perf_counter,
    bench_annotations=None,
):
    if (max_frames is not None and
            (isinstance(max_frames, bool) or not isinstance(max_frames, int)
             or max_frames <= 0)):
        raise ValueError("max_frames must be a positive integer or None")
    processing_ms = []
    report = {
        "records_seen": 0,
        "frames_processed": 0,
        "non_radar_records_skipped": 0,
        "incomplete_frames_skipped": 0,
        "producer_reported_drops_total": 0,
        "raw_points_total": 0,
        "self_clutter_rejected_total": 0,
        "quality_gate_rejected_total": 0,
        "heatmap_reflections_total": 0,
        "tentative_point_tracks_max": 0,
        "confirmed_point_tracks_max": 0,
        "confirmed_heatmap_tracks_max": 0,
        "probability_cells_max": 0,
        "occupied_cells_max": 0,
        "confirmed_tracks_in_self_clutter_zone_max": 0,
        "reactive_cells_in_self_clutter_zone_max": 0,
        "occupied_cells_in_self_clutter_zone_max": 0,
        "probability_decayed_total": 0,
        "probability_ttl_expired_total": 0,
        "path_switches": 0,
        "path_switches_per_min": 0.0,
        "danger_frames": 0,
        "blocked_path_replans": 0,
        "blocked_near_field_frames": 0,
        "blocked_near_field_longest_ms": 0.0,
        "duration_s": 0.0,
        "probability_cells_final": 0,
        "occupied_cells_final": 0,
        "planned_path_points_max": 0,
        "state_bounds_passed": True,
        "state_bound_failures": [],
        "planned_path_status_final": "no_frames",
        "planned_path_stability_status_final": "no_frames",
        "planned_path_curvature_final": 0.0,
        "calibration_status_counts": {},
        "motion_status_counts": {},
        "bench": None,
    }
    cumulative_keys = {
        "probability_decayed_cell_count": "probability_decayed_total",
        "probability_ttl_expired_cell_count": "probability_ttl_expired_total",
    }
    previous_cumulative = {key: 0 for key in cumulative_keys}
    previous_scene_epoch = None
    for record in records:
        if (max_frames is not None and
                report["frames_processed"] >= max_frames):
            break
        report["records_seen"] += 1
        if not isinstance(record, RadarFrame):
            report["non_radar_records_skipped"] += 1
            continue
        if not record.complete:
            report["incomplete_frames_skipped"] += 1
            continue
        received_at = record.header.monotonic_ns / 1_000_000_000.0
        started = timer()
        estimator.ingest(record, received_at=received_at)
        scene = estimator.snapshot(now=received_at)
        processing_ms.append(max(0.0, (timer() - started) * 1000.0))

        diagnostics = scene["diagnostics"]
        probability_cells = scene["probabilistic_local_map"]["cells"]
        report["frames_processed"] += 1
        report["producer_reported_drops_total"] += int(
            record.dropped_frames_since_previous
        )
        report["raw_points_total"] += int(diagnostics["raw_point_count"])
        report["self_clutter_rejected_total"] += int(
            diagnostics["self_clutter_rejected_count"]
        )
        report["quality_gate_rejected_total"] += int(
            diagnostics["quality_gate_rejected_count"]
        )
        report["heatmap_reflections_total"] += int(
            diagnostics["heatmap_reflection_count"]
        )
        report["tentative_point_tracks_max"] = max(
            report["tentative_point_tracks_max"],
            int(diagnostics["tentative_point_track_count"]),
        )
        report["confirmed_point_tracks_max"] = max(
            report["confirmed_point_tracks_max"],
            int(diagnostics["confirmed_point_track_count"]),
        )
        report["confirmed_heatmap_tracks_max"] = max(
            report["confirmed_heatmap_tracks_max"],
            int(diagnostics["confirmed_heatmap_track_count"]),
        )
        report["probability_cells_max"] = max(
            report["probability_cells_max"], len(probability_cells)
        )
        report["occupied_cells_max"] = max(
            report["occupied_cells_max"],
            sum(bool(cell["occupied"]) for cell in probability_cells),
        )
        occupied_count = sum(bool(cell["occupied"]) for cell in probability_cells)
        def inside_self_clutter_zone(item):
            return math.hypot(
                float(item["forward_m"]), float(item["lateral_m"])
            ) <= 0.15
        report["confirmed_tracks_in_self_clutter_zone_max"] = max(
            report["confirmed_tracks_in_self_clutter_zone_max"],
            sum(
                inside_self_clutter_zone(track)
                and bool(track.get("point_confirmed")
                         or track.get("heatmap_confirmed"))
                for track in scene["tracks"]
            ),
        )
        report["reactive_cells_in_self_clutter_zone_max"] = max(
            report["reactive_cells_in_self_clutter_zone_max"],
            sum(inside_self_clutter_zone(cell)
                for cell in scene["near_occupancy"]["cells"]),
        )
        report["occupied_cells_in_self_clutter_zone_max"] = max(
            report["occupied_cells_in_self_clutter_zone_max"],
            sum(bool(cell["occupied"]) and inside_self_clutter_zone(cell)
                for cell in probability_cells),
        )
        report["probability_cells_final"] = len(probability_cells)
        report["occupied_cells_final"] = occupied_count
        planned_path = scene["planned_path"]
        path_points = planned_path["points"]
        report["planned_path_points_max"] = max(
            report["planned_path_points_max"], len(path_points)
        )
        bound_failures = []
        if len(probability_cells) > 317:
            bound_failures.append("probability_cells_exceed_317")
        if occupied_count > len(probability_cells):
            bound_failures.append("occupied_cells_exceed_active_cells")
        if len(path_points) > 121:
            bound_failures.append("planned_path_points_exceed_121")
        if not all(
            len(point) == 2
            and all(math.isfinite(float(value)) for value in point)
            and math.hypot(float(point[0]), float(point[1])) <= 0.60 + 1e-9
            for point in path_points
        ):
            bound_failures.append("planned_path_point_out_of_bounds")
        if bound_failures:
            report["state_bounds_passed"] = False
            for failure in bound_failures:
                if failure not in report["state_bound_failures"]:
                    report["state_bound_failures"].append(failure)
        report["planned_path_status_final"] = str(planned_path["status"])
        report["planned_path_stability_status_final"] = str(
            planned_path["stability_status"]
        )
        report["planned_path_curvature_final"] = float(
            planned_path["curvature_per_m"]
        )
        if scene["hazard"]["level"] == "DANGER":
            report["danger_frames"] += 1
        if (planned_path["stability_status"] == "route_changed_blocked"
                and planned_path["switch_accepted"] is True):
            report["blocked_path_replans"] += 1
        if planned_path["switch_accepted"] is True:
            report["path_switches"] += 1
        if planned_path["status"] == "blocked_near_field":
            report["blocked_near_field_frames"] += 1
        scene_epoch = int(diagnostics["scene_epoch"])
        if previous_scene_epoch is not None and scene_epoch < previous_scene_epoch:
            raise ValueError("scene_epoch must be monotonic")
        for diagnostic_key, report_key in cumulative_keys.items():
            current = int(diagnostics[diagnostic_key])
            previous = previous_cumulative[diagnostic_key]
            if previous_scene_epoch is None or scene_epoch != previous_scene_epoch:
                report[report_key] += current
            else:
                report[report_key] += max(0, current - previous)
            previous_cumulative[diagnostic_key] = current
        previous_scene_epoch = scene_epoch

        calibration_status = str(scene.get("calibration_status", "missing"))
        report["calibration_status_counts"][calibration_status] = (
            report["calibration_status_counts"].get(calibration_status, 0) + 1
        )
        motion_status = str(
            scene.get("motion_compensation", {}).get("status", "missing")
        )
        report["motion_status_counts"][motion_status] = (
            report["motion_status_counts"].get(motion_status, 0) + 1
        )

    ordered = sorted(processing_ms)
    count = len(ordered)
    report["processing_ms_mean"] = sum(ordered) / count if count else 0.0
    report["processing_ms_p95"] = (
        ordered[max(0, math.ceil(0.95 * count) - 1)] if count else 0.0
    )
    report["processing_ms_max"] = ordered[-1] if count else 0.0
    return report
```

Probability-removal totals use non-negative within-epoch deltas and add the full current value whenever validated `scene_epoch` changes, including the ambiguous case where the post-reset value equals the prior value. Reject a decreasing epoch. Count `path_switches` directly from the per-frame `planned_path.switch_accepted is True` event instead of differencing its diagnostic counter, so a scene reset cannot hide or duplicate a switch.

Track the first and last processed monotonic timestamps. Set `duration_s=max(0, last-first)` and `path_switches_per_min=path_switches * 60 / duration_s` (zero when duration is zero). Track each contiguous `planned_path.status == "blocked_near_field"` run from its first blocked timestamp until the first later clear frame, including the closing interval; flush an open run at the last processed timestamp. `blocked_path_replans` is an event count and increments only on a frame with both `switch_accepted is True` and `stability_status == "route_changed_blocked"`; a held blocked route must not inflate it.

Implement `load_bench_annotations(path: Path, input_path: Path, clutter_model) -> dict` with the strict schema/hash/domain rules from Step 1, using `hashlib.sha256` streaming reads and a duplicate-key-rejecting `json.loads(..., object_pairs_hook=...)`. Add a private bench evaluator invoked by `analyze_records` when `bench_annotations` is not `None`. Index processed `(mission_id, unit_id, boot_id, producer_id, stream_id, seq, record, scene)` tuples only for annotated windows; every lookup uses the full five-field domain plus seq. Map raw points through `estimator.axes.map_point`, compute every window metric/formula and failure exactly as specified, and assign the complete bench object to `report["bench"]`. Do not allow annotations to weaken calibration, latency, map, path, or hazard gates; every output remains JSON-finite (`None`, never NaN, when a failed window lacks a measurable value).

CLI arguments are:

```text
--input PATH                  required canonical JSONL
--clutter-calibration PATH    optional model; required unless --synthetic
--synthetic                   explicit calibration bypass for fixtures only
--max-frames N                positive integer
--max-p95-ms FLOAT            optional failure threshold
--max-frame-ms FLOAT          optional worst-frame failure threshold
--annotations PATH            strict *.radar-bench.json sidecar
--require-bench-pass          fail after printing when bench.passed is false
```

Implement `main(argv: Sequence[str] | None = None) -> int`. Parse paths as `Path`; use dedicated argparse type functions to reject booleans/non-integers/non-positive `--max-frames` and negative or non-finite latency thresholds. When `--synthetic` is set, construct `RadarSceneEstimator(RadarAxes(), synthetic=True)` and reject simultaneous `--clutter-calibration` through `parser.error`. Otherwise require `--clutter-calibration`, load it with `load_clutter_model`, and construct `RadarSceneEstimator(model.axes, clutter_model=model)` so a valid non-default axes binding is not accidentally rejected. `--annotations` conflicts with `--synthetic` and `--max-frames`; `--require-bench-pass` requires `--annotations`. Load/validate the sidecar before analysis. Pass `(entry.record for entry in iter_replay(args.input))` to `analyze_records`, print exactly one sorted UTF-8 JSON object with `ensure_ascii=False` and `allow_nan=False`, and return exit code 2 when `state_bounds_passed` is false, either supplied threshold is exceeded (`processing_ms_p95 > --max-p95-ms` or `processing_ms_max > --max-frame-ms`), or `--require-bench-pass` is set and `bench.passed` is false; otherwise return 0. Schema/hash/binding errors call `parser.error` before any report, while a valid failed state/bench gate prints its report before returning 2. Do not write a report file. Compute p95 by sorting measured values and selecting index `ceil(0.95 * count) - 1`.

End the module with the literal entrypoint below, and invoke it with `python -m` so the repository root is on `sys.path` without a script-local path bootstrap:

```python
if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run all analyzer and CLI tests and verify GREEN**

Run:

```powershell
python -B -m unittest tests.test_radar_replay -v
if ($LASTEXITCODE -ne 0) { throw "replay analyzer tests failed" }
```

Expected: all stage-count, reset-delta, deterministic latency, bounds, calibration-axis, argparse, JSON-output, and threshold-exit tests pass. The 2,400-frame real-log check remains local because the source file is about 444 MB and is not a CI fixture.

- [ ] **Step 5: Update the radar-only operating and validation guide**

In `docs/radar_front_view.md`, document:

- default probability map versus `RAW DEBUG · 표시 전용`;
- 3-of-5 map confirmation versus unchanged 2-of-3 hazard confirmation;
- 5 cm cells, 0.75 occupancy, 1 s half-life, and 2.5 s TTL;
- 15 cm robot width, 3 cm margins, three-frame route switching, and immediate blocked-path switching;
- `sensor_relative_uncalibrated` height bands and why no clearance is inferred;
- `robot_relative_short_memory` and why this is not SLAM or Doppler odometry;
- the analyzer command and meanings of every report field;
- five 60-second bench scenes: empty mounted robot, frontal targets at 0.30/0.50/1.00 m, ±30-degree targets, wood versus metal, and slow wall approach/retreat;
- record raw false points/frame, confirmed false obstacles/frame, position standard deviation, target detection rate, ghost persistence, path switches/min, processing latency, and frame drops;
- the strict sidecar schema, SHA/domain binding, all bench report fields, the five recording-kind coverage rules, and `--annotations --require-bench-pass` command;
- a calibration provenance checklist containing profile ID, calibration ID, board serial, robot/mount ID and orientation, USB cable/adapter, power state, physical location, ambient temperature, UTC creation time, and operator. State explicitly that profile ID, calibration ID, axes, and heatmap geometry/range-step are runtime-validated; board/mount/cable/power/location/temperature/operator provenance remains manually recorded and must not be claimed as automatically verified.

- [ ] **Step 6: Run focused replay checks and commit**

Run:

```powershell
python -B -m unittest tests.test_radar_replay tests.test_radar_scene -v
if ($LASTEXITCODE -ne 0) { throw "focused replay and scene tests failed" }
python -B -m scripts.analyze_radar_scene `
  --input tests\fixtures\radar_clutter_20260728_f2286_f2291.jsonl `
  --synthetic `
  --max-frames 6
if ($LASTEXITCODE -ne 0) { throw "synthetic analyzer smoke failed" }
git diff --check
if ($LASTEXITCODE -ne 0) { throw "whitespace check failed" }
git add -- scripts/analyze_radar_scene.py tests/test_radar_replay.py docs/radar_front_view.md
if ($LASTEXITCODE -ne 0) { throw "replay report staging failed" }
git commit -m "test(radar): add stabilization replay report"
if ($LASTEXITCODE -ne 0) { throw "replay report commit failed" }
```

Expected: focused tests pass and the CLI prints one finite JSON object.

- [ ] **Step 7: Run the complete automated verification matrix**

Run:

```powershell
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
function Assert-NativeSuccess([string]$label) {
  if ($LASTEXITCODE -ne 0) {
    throw "$label failed with exit code $LASTEXITCODE"
  }
}

python -B -m unittest discover -s tests -v
Assert-NativeSuccess "full Python suite"
& $radarNode --check monitor\web\radar_scene.js
Assert-NativeSuccess "radar_scene.js syntax"
& $radarNode --check monitor\web\radar_panel.js
Assert-NativeSuccess "radar_panel.js syntax"
& $radarNode --test tests\web\radar_scene.test.js tests\web\radar_panel.test.js
Assert-NativeSuccess "Node radar suites"
python -m compileall -q monitor scripts sensors tests
Assert-NativeSuccess "Python compileall"
git diff --check
Assert-NativeSuccess "working-tree whitespace check"
git diff --exit-code e2ec90b -- configs\radar\iwrl6432_3d_operator_near_8hz.cfg
Assert-NativeSuccess "verified 8 Hz cfg immutability"

$allowedRadarPaths = @(
  '^monitor/radar_', '^monitor/web/radar_', '^tests/test_radar_',
  '^tests/web/radar_', '^scripts/analyze_radar_scene\.py$',
  '^sensors/radar_stack_processes\.py$',
  '^docs/radar_front_view\.md$',
  '^docs/superpowers/plans/2026-08-07-radar-sensor-stabilization\.md$'
)
$changedRadarPaths = @(git diff --name-only e2ec90b..HEAD)
Assert-NativeSuccess "committed path enumeration"
$unexpectedPaths = @()
foreach ($changedPath in $changedRadarPaths) {
  $allowed = $false
  foreach ($pattern in $allowedRadarPaths) {
    if ($changedPath -match $pattern) { $allowed = $true; break }
  }
  if (-not $allowed) { $unexpectedPaths += $changedPath }
}
if ($unexpectedPaths.Count -ne 0) {
  throw "non-radar paths changed: $($unexpectedPaths -join ', ')"
}

$unstagedPaths = @(git diff --name-only)
Assert-NativeSuccess "unstaged path enumeration"
$stagedPaths = @(git diff --cached --name-only)
Assert-NativeSuccess "staged path enumeration"
$untrackedPaths = @(git ls-files --others --exclude-standard)
Assert-NativeSuccess "untracked path enumeration"
$uncommittedPaths = @(
  $unstagedPaths
  $stagedPaths
  $untrackedPaths
) | Where-Object { $_ }
if ($uncommittedPaths.Count -ne 0) {
  throw "uncommitted/staged/untracked files remain: $($uncommittedPaths -join ', ')"
}
git status --short --branch
Assert-NativeSuccess "final Git status"
```

Expected: one uninterrupted full Python run and both Node suites pass, syntax checks succeed, the 8 Hz cfg diff is empty, and the path allowlist proves only intentional radar files differ from `e2ec90b`. If an apparent pre-existing flaky test fails, isolate it for diagnosis, but rerun this entire matrix from the beginning and require a clean pass before claiming completion.

- [ ] **Step 8: Run HTTP/demo smoke without a board**

Run from a dedicated PowerShell terminal:

```powershell
$portProbe = [System.Net.Sockets.TcpListener]::new(
  [System.Net.IPAddress]::Loopback, 0
)
$portProbe.Start()
$radarPort = ([System.Net.IPEndPoint]$portProbe.LocalEndpoint).Port
$portProbe.Stop()

$radarDemo = Start-Process python -ArgumentList @(
  "-B", "monitor\radar_front.py", "--demo", "--bind", "127.0.0.1",
  "--http-port", "$radarPort", "--quiet"
) -PassThru -WindowStyle Hidden
try {
  $deadline = [DateTime]::UtcNow.AddSeconds(10)
  $snapshot = $null
  do {
    if ($radarDemo.HasExited) { throw "radar demo exited before readiness" }
    try {
      $candidate = Invoke-RestMethod "http://127.0.0.1:$radarPort/api/radar"
      if (($candidate.status -eq "live") -and ($null -ne $candidate.frame)) {
        $snapshot = $candidate
      }
    } catch { }
    if ($null -eq $snapshot) { Start-Sleep -Milliseconds 250 }
  } while ($null -eq $snapshot -and [DateTime]::UtcNow -lt $deadline)
  if ($null -eq $snapshot) { throw "radar demo readiness timeout" }
  if ($snapshot.scene.pose_mode -ne "robot_relative_short_memory") {
    throw "unexpected pose mode"
  }
  if ($snapshot.scene.probabilistic_local_map.resolution_m -ne 0.05) {
    throw "unexpected probability-map resolution"
  }
} finally {
  Stop-Process -Id $radarDemo.Id -ErrorAction SilentlyContinue
}
```

Expected: the HTTP assertions pass and the hidden demo process is stopped.

- [ ] **Step 9: Run the optional 2,400-frame local real-log check when present**

Run only if the local log and its matching calibration exist:

```powershell
$radarLog = "C:\Users\minho\Documents\Codex\2026-07-25\f\HANSEL_MESH_RADAR_RECOVERY\missions\radar-board-live-20260729-174058226-e001.jsonl"
$radarCalibration = "configs\radar\calibrations\head-near-8hz.json"
if ((Test-Path -LiteralPath $radarLog) -and
    (Test-Path -LiteralPath $radarCalibration)) {
  $realLogGateStatus = "UNUSABLE"
  $radarReportJson = python -B -m scripts.analyze_radar_scene `
    --input $radarLog `
    --clutter-calibration $radarCalibration `
    --max-frames 2400 `
    --max-p95-ms 125 `
    --max-frame-ms 125
  if ($LASTEXITCODE -ne 0) {
    throw "2,400-frame replay gate failed with exit code $LASTEXITCODE"
  }
  $radarReport = $radarReportJson | ConvertFrom-Json
  $bindingStatuses = @(
    $radarReport.calibration_status_counts.PSObject.Properties.Name
  )
  $bindingFailures = @(
    $bindingStatuses | Where-Object {
      $_ -in @("calibration_mismatch", "profile_mismatch")
    }
  )
  if ($bindingFailures.Count -ne 0) {
    Write-Warning (
      "historical log is unusable for the performance gate: " +
      ($bindingFailures -join ", ")
    )
  } elseif (($radarReport.frames_processed -ne 2400) -or
            ($radarReport.calibration_status_counts.ok -ne 2400)) {
    throw "2,400-frame log was not fully accepted with calibration status ok"
  } elseif (($radarReport.state_bounds_passed -ne $true) -or
            ($radarReport.probability_cells_max -gt 317) -or
            ($radarReport.occupied_cells_max -gt
             $radarReport.probability_cells_max) -or
            ($radarReport.planned_path_points_max -gt 121)) {
    throw "2,400-frame state bounds failed"
  } else {
    $realLogGateStatus = "PASSED"
  }
  Write-Output "REAL_LOG_GATE=$realLogGateStatus"
}
```

Expected: an attested matching log prints `REAL_LOG_GATE=PASSED`, has 2,400 accepted `ok` frames, at most 317 probability cells and 121 path points, occupied count no greater than active count, and p95/worst-frame time no greater than 125 ms. A historical mismatch prints `REAL_LOG_GATE=UNUSABLE`—never `PASSED`—while recording the actual status (`calibration_mismatch` for an uncalibrated header, or `profile_mismatch` when applicable); never bypass it with `--synthetic`.

- [ ] **Step 10: Run the board-backed bench acceptance gate**

When the IWRL6432BOOST and final robot mount are available, follow the recapture/calibration procedure in `docs/radar_front_view.md`, record each of the five specified scenes for at least 60 seconds, and analyze each canonical JSONL with the matching calibration. Save the report output outside Git unless the user explicitly asks to commit measurements.

Run this exact gate from the repository root after placing each JSONL and strict sidecar in one external bench directory:

```powershell
$ErrorActionPreference = "Stop"
$benchRoot = "C:\radar-bench\20260807"
$calibration = Resolve-Path "configs\radar\calibrations\head-near-8hz.json"
$reportRoot = Join-Path $benchRoot "reports"
New-Item -ItemType Directory -Force -Path $reportRoot | Out-Null
$benchRootResolved = (Resolve-Path -LiteralPath $benchRoot).Path
$reportRootResolved = (Resolve-Path -LiteralPath $reportRoot).Path

$reports = @()
$seenRecordingIds = @{}
$sidecars = @(Get-ChildItem -LiteralPath $benchRoot -Filter "*.radar-bench.json")
if ($sidecars.Count -eq 0) { throw "no radar bench annotation sidecars" }

foreach ($sidecar in ($sidecars | Sort-Object FullName)) {
  $annotation = Get-Content -Raw -LiteralPath $sidecar.FullName |
    ConvertFrom-Json
  $recordingId = [string]$annotation.recording.id
  if (($recordingId -in @(".", "..")) -or
      ($recordingId -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$')) {
    throw "unsafe recording id: $recordingId"
  }
  if ($seenRecordingIds.ContainsKey($recordingId)) {
    throw "duplicate recording id: $recordingId"
  }
  $seenRecordingIds[$recordingId] = $true

  $log = [IO.Path]::GetFullPath(
    (Join-Path $benchRootResolved ([string]$annotation.log_file))
  )
  if (-not [StringComparer]::OrdinalIgnoreCase.Equals(
      [IO.Path]::GetDirectoryName($log), $benchRootResolved)) {
    throw "log_file must name a file directly inside benchRoot"
  }
  $json = python -B -m scripts.analyze_radar_scene `
    --input $log `
    --clutter-calibration $calibration `
    --annotations $sidecar.FullName `
    --require-bench-pass `
    --max-p95-ms 125 `
    --max-frame-ms 125
  $exitCode = $LASTEXITCODE
  if ($exitCode -ne 0) {
    throw "bench case failed before report write: $recordingId"
  }
  $report = (($json -join [Environment]::NewLine) | ConvertFrom-Json)
  if (($report.bench.passed -ne $true) -or
      ([string]$report.bench.recording_id -ne $recordingId)) {
    throw "bench report identity/pass verification failed: $recordingId"
  }
  $reportPath = [IO.Path]::GetFullPath(
    (Join-Path $reportRootResolved "$recordingId.json")
  )
  if (-not [StringComparer]::OrdinalIgnoreCase.Equals(
      [IO.Path]::GetDirectoryName($reportPath), $reportRootResolved)) {
    throw "unsafe report path: $reportPath"
  }
  $json | Set-Content -Encoding utf8 -LiteralPath $reportPath
  $reports += $report
}

$requiredRecordingKinds = @(
  "empty_mounted", "frontal_targets", "angled_targets",
  "materials", "approach_retreat"
)
$requiredCoverage = @(
  "front_030_metal", "front_050_metal", "front_100_metal",
  "azimuth_neg30_metal", "azimuth_pos30_metal", "front_050_wood"
)
$requiredWindowKinds = @(
  "empty", "fixed_target", "ghost", "removal",
  "transient_path", "confirmed_collision", "approach_retreat"
)

$recordingKinds = @($reports | ForEach-Object { $_.bench.recording_kind })
$mountBindings = @($reports | ForEach-Object {
  "$($_.bench.provenance.board_serial)|" +
  "$($_.bench.provenance.robot_mount_id)|" +
  "$($_.bench.provenance.mount_orientation)"
} | Sort-Object -Unique)
$windows = @($reports | ForEach-Object { $_.bench.windows })
$coverage = @($windows | Where-Object kind -eq "fixed_target" |
  ForEach-Object coverage_key)
$windowKinds = @($windows | ForEach-Object kind)

foreach ($required in $requiredRecordingKinds) {
  if ($required -notin $recordingKinds) { throw "missing recording: $required" }
}
if ($mountBindings.Count -ne 1) {
  throw "bench recordings do not share one board/mount/orientation binding"
}
foreach ($required in $requiredCoverage) {
  if ($required -notin $coverage) { throw "missing target coverage: $required" }
}
foreach ($required in $requiredWindowKinds) {
  if ($required -notin $windowKinds) { throw "missing bench event: $required" }
}
```

Accept the physical-noise claim only when all of these hold:

- every sidecar hash/domain/calibration binding and manual provenance schema passes, all recordings share one attested board/mount/orientation binding, and all five recording kinds, six target coverages, and seven window kinds are present;
- the empty mounted scene produces no confirmed route-blocking run longer than 2.5 seconds;
- a one- or two-frame ghost never enters the probability map, and removed confirmed evidence is absent on the first complete observation at or after the 2.5-second TTL deadline;
- every fixed target has at least 30 paired same-frame raw/current-filtered samples, and filtered position standard deviation over those pairs is strictly lower than raw at every distance/angle/material;
- no single-frame unblocked candidate changes the selected route, while a confirmed collision changes it immediately;
- 10 cm danger entry and 13 cm release remain correct on measured approach/retreat;
- reported p95 and worst-frame processing time are each at most 125 ms, with false rates, target detection rate, path switches/min, and frame drops explicitly recorded even though no unapproved arbitrary thresholds are assigned to those report-only metrics.

If hardware is unavailable during implementation, mark this gate `PENDING HARDWARE` in the handoff and claim only automated/code completion—not proven real-world noise reduction.

- [ ] **Step 11: Review the whole branch before integration**

Use `superpowers:requesting-code-review` over the range `e2ec90b..HEAD`. Verify every design acceptance criterion against task tests and the replay report. Fix Critical/Important findings through the subagent review loop, rerun Step 7, then use `superpowers:finishing-a-development-branch` to present merge/push options. Do not push without the user's explicit request in that finishing step.
