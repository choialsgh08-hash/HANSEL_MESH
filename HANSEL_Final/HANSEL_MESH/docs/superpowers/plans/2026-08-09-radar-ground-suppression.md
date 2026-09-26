# Radar Ground Suppression Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove ground returns from the radar operator view and every driving decision while preserving untouched raw frames for replay and diagnosis.

**Architecture:** A small pure-Python mount-geometry component converts sensor forward/height into ground-relative coordinates and classifies traversable ground. `RadarSceneEstimator` applies that gate before tracking or heatmap correlation, while the web contract exposes auditable filter settings/counts and suppresses unconfirmed heatmap geometry. The current hardware defaults are 0.065 m height, 0 degrees upward pitch, and a 0.035 m ground allowance.

**Tech Stack:** Python 3 standard library and `unittest`; browser JavaScript with Node's built-in test runner; TI MMWAVE-L-SDK CLI profile.

## Global Constraints

- Touch only radar-related code, tests, configuration, and documentation.
- Raw `RadarFrame.points` and JSONL recordings remain unchanged.
- Ground points never enter tracking, occupancy, probability maps, paths, hazards, or operator rendering.
- Existing danger thresholds remain 0.10 m enter and 0.13 m release.
- `clutterRemoval` remains disabled so static walls and rubble are retained.
- Heatmap-only evidence cannot become or render as a hard obstacle without a non-ground 3D point correlation.
- Live servo-angle transport, terrain cost, wheel-climb modeling, and body/bumper volumes remain out of scope.

---

### Task 1: Pure mount geometry and ground classifier

**Files:**
- Create: `monitor/radar_ground.py`
- Create: `tests/test_radar_ground.py`

**Interfaces:**
- Produces: immutable `RadarMountGeometry(sensor_height_m=0.065, sensor_pitch_deg=0.0, ground_rejection_height_m=0.035, filter_range_m=0.60)`.
- Produces: `RadarMountGeometry.robot_coordinates(forward_m, lateral_m, height_m) -> tuple[float, float, float]` using upward-positive pitch.
- Produces: `RadarMountGeometry.is_ground(forward_m, lateral_m, height_m) -> bool`.
- Produces: `RadarMountGeometry.payload() -> dict[str, object]` with `sensor_height_m`, `sensor_pitch_deg`, `ground_rejection_height_m`, `filter_range_m`, `pitch_convention`, and `status`.

- [ ] **Step 1: Write validation and transform tests**

Add tests proving the defaults, the zero-pitch result `(f, l, 0.065 + z)`, the +30 degree formulas below, and rejection of booleans, NaN, infinity, negative heights, a non-positive range, and `ground_rejection_height_m >= sensor_height_m`.

```python
robot_forward = math.cos(pitch) * forward - math.sin(pitch) * height
robot_height = sensor_height + math.sin(pitch) * forward + math.cos(pitch) * height
```

- [ ] **Step 2: Run the pure tests and capture RED**

Run: `python -B -m unittest tests.test_radar_ground -v`

Expected: import failure because `monitor.radar_ground` does not exist.

- [ ] **Step 3: Implement the frozen geometry value object**

Use `@dataclass(frozen=True)` and strict real/non-boolean finite validation in `__post_init__`. `is_ground` returns false for missing/non-finite point heights, points behind the sensor, or points outside `filter_range_m`; otherwise it compares transformed height with `ground_rejection_height_m` inclusively.

- [ ] **Step 4: Run pure tests to GREEN**

Run: `python -B -m unittest tests.test_radar_ground -v`

Expected: all ground-geometry tests pass.

- [ ] **Step 5: Commit the isolated component**

```powershell
git add -- monitor/radar_ground.py tests/test_radar_ground.py
git commit -m "feat(radar): classify ground-relative returns"
```

### Task 2: Apply ground suppression before scene evidence

**Files:**
- Modify: `monitor/radar_scene.py:229-394, 732-869`
- Modify: `tests/test_radar_scene.py`
- Modify: `monitor/radar_front.py:1134-1299`
- Modify: `tests/test_radar_front.py`
- Modify: `configs/radar/iwrl6432_3d_operator_near_8hz.cfg:36`

**Interfaces:**
- Consumes: `RadarMountGeometry` from Task 1.
- Extends: `RadarSceneEstimator(..., mount_geometry: RadarMountGeometry | None = None)` without changing existing positional parameter meaning.
- Produces: `scene.ground_filter` from `mount_geometry.payload()`.
- Produces: integer `scene.diagnostics.ground_rejected_count` for the current frame.
- Adds CLI flags `--sensor-height-m`, `--sensor-pitch-deg`, and `--ground-rejection-height-m` with the hardware defaults.

- [ ] **Step 1: Write scene-level failing regressions**

Add real-frame tests that ingest repeated strong points and assert:

```python
floor = RadarPoint(0.0, 0.08, -0.050, 0.0, 30.0)
raised = RadarPoint(0.0, 0.08, 0.000, 0.0, 30.0)
```

- The floor point remains in the input frame but yields no scene track, near cell, probability cell, accumulated obstacle, planned-path blockage, or `DANGER` after three frames.
- `ground_rejected_count == 1` and raw count remains one.
- The raised point at the same horizontal distance is retained and reaches `DANGER` after the existing two-of-three confirmation.
- A point outside 0.60 m is not silently classified as ground.
- A ground point cannot provide `point_correlated` confirmation to repeated heatmap evidence.
- Reset and calibration-mismatch paths reset/report the current-frame ground counter deterministically.

- [ ] **Step 2: Run focused scene tests and capture RED**

Run: `python -B -m unittest tests.test_radar_ground tests.test_radar_scene -v`

Expected: failures for missing estimator geometry, ground counter, and suppression behavior.

- [ ] **Step 3: Integrate the gate before clutter, quality, tracking, and heatmap correlation**

At ingest start reset `_ground_rejected_count`. After native axis mapping, call `mount_geometry.is_ground(...)`; increment and continue before self-clutter matching. Preserve `frame.points` and raw counts. Pass only non-ground points to accepted point positions so heatmap confirmation cannot reuse ground evidence. Add `ground_filter` and the count to scene snapshots.

- [ ] **Step 4: Add CLI binding tests and validation**

Test exact defaults, valid overrides, parser errors for non-finite/negative/inverted height values, and construction of `RadarSceneEstimator` with those values. Keep the existing constructor working for tests and replay callers.

- [ ] **Step 5: Bind the live TI profile**

Change only the default 8 Hz operator profile line to:

```text
sensorPosition 0 0 0.065 0 0
```

Keep `clutterRemoval 0`, `rangeSelCfg 0.07 7.5`, CFAR, and frame timing unchanged. Add a test or static assertion that the default profile contains this exact binding.

- [ ] **Step 6: Run backend GREEN and regression suites**

Run:

```powershell
python -B -m unittest tests.test_radar_ground tests.test_radar_scene tests.test_radar_front tests.test_radar_replay tests.test_radar_stack_processes -v
```

Expected: all selected backend tests pass with no threshold regression.

- [ ] **Step 7: Commit backend integration**

```powershell
git add -- monitor/radar_scene.py monitor/radar_front.py configs/radar/iwrl6432_3d_operator_near_8hz.cfg tests/test_radar_scene.py tests/test_radar_front.py
git commit -m "feat(radar): suppress ground from driving evidence"
```

### Task 3: Validate and display the filter contract without ground geometry

**Files:**
- Modify: `monitor/web/radar_scene.js`
- Modify: `monitor/web/radar_panel.js`
- Modify: `monitor/web/radar_front.html`
- Modify: `tests/web/radar_scene.test.js`
- Modify: `tests/web/radar_panel.test.js`
- Modify: `monitor/radar_front.py:54`
- Modify: `tests/test_radar_front.py:1355-1415`

**Interfaces:**
- Consumes: `scene.ground_filter` and `diagnostics.ground_rejected_count` from Task 2.
- Produces: parsed `presentation.groundFilter` with the four finite geometry values and status.
- Produces: `#ground-rejected-value` diagnostic row and a dynamic mount/filter tag.
- Changes: primary track rendering ignores heatmap tracks unless `heatmap_confirmed === true`.

- [ ] **Step 1: Write strict parser RED tests**

Require `ground_rejected_count` in the new probability scene diagnostics and require a ground-filter object with exact finite non-negative fields, positive filter range, `ground_rejection_height_m < sensor_height_m`, `status === "configured"`, and `pitch_convention === "positive_up"`. Add mutation-table tests for missing, NaN, infinity, string, boolean, negative, and inverted values.

- [ ] **Step 2: Write renderer RED tests**

Assert the diagnostic row receives the count, the tag reports `SENSOR 6.5cm / PITCH 0.0deg / GROUND HIDDEN`, and unconfirmed heatmap reflection tracks generate no uncertainty/track drawing calls in both the main and collision views. Confirmed heatmap and point tracks must still render.

- [ ] **Step 3: Run Node tests and capture RED**

Run:

```powershell
node --test tests/web/radar_scene.test.js tests/web/radar_panel.test.js
```

Expected: failures for the missing contract, diagnostic elements, and unconfirmed heatmap suppression.

- [ ] **Step 4: Implement parser, renderer, HTML, and build binding**

Add the strict parser and filtered drawing source without mutating parsed diagnostic data. Add the two HTML elements, update the panel text binding, and bump both Python and JavaScript build IDs plus the visible tag from R32 to R33 so browsers cannot mix assets.

- [ ] **Step 5: Run web and HTTP GREEN suites**

Run:

```powershell
node --check monitor/web/radar_scene.js
node --check monitor/web/radar_panel.js
node --test tests/web/radar_scene.test.js tests/web/radar_panel.test.js
python -B -m unittest tests.test_radar_front -v
```

Expected: JavaScript syntax succeeds and all web/HTTP tests pass.

- [ ] **Step 6: Commit the operator contract**

```powershell
git add -- monitor/web/radar_scene.js monitor/web/radar_panel.js monitor/web/radar_front.html monitor/radar_front.py tests/web/radar_scene.test.js tests/web/radar_panel.test.js tests/test_radar_front.py
git commit -m "feat(radar): expose hidden-ground diagnostics"
```

### Task 4: Replay metric, documentation, and live verification

**Files:**
- Modify: `scripts/analyze_radar_scene.py`
- Modify: `tests/test_radar_replay.py`
- Modify: `docs/radar_front_view.md`

**Interfaces:**
- Produces: replay report integer `ground_rejected_total`, accumulated from complete processed frames.
- Documents: fixed 6.5 cm/0 degree mount, hidden-ground semantics, pitch sign, raw-record preservation, and the later servo-angle integration point.

- [ ] **Step 1: Write replay report RED test**

Feed one ground-only and one raised-point frame through the real estimator. Assert two raw points are analyzed, exactly one ground rejection is accumulated, only the raised return contributes to detection/hazard evidence, and the JSON report remains finite and serializable.

- [ ] **Step 2: Run replay test and capture RED**

Run: `python -B -m unittest tests.test_radar_replay -v`

Expected: failure because `ground_rejected_total` is absent.

- [ ] **Step 3: Aggregate the metric and update the operator guide**

Initialize the report field to zero and accumulate the validated per-frame diagnostic counter alongside self-clutter and quality rejections. Document that raw logs retain floor points even though the operator scene hides them.

- [ ] **Step 4: Run full automated verification**

Run:

```powershell
python -B -m unittest tests.test_radar_ground tests.test_radar_tracking tests.test_radar_motion tests.test_radar_local_map tests.test_radar_path tests.test_radar_scene tests.test_radar_front tests.test_radar_replay tests.test_radar_stack_processes -v
node --check monitor/web/radar_scene.js
node --check monitor/web/radar_panel.js
node --test tests/web/radar_scene.test.js tests/web/radar_panel.test.js
python -m compileall -q monitor sensors scripts tests
git diff --check
```

Expected: every command exits zero.

- [ ] **Step 5: Restart the current radar stack and perform a read-only live smoke**

Use the existing supervisor workflow to restart with the modified profile. Poll `/api/radar` until it is live, then collect at least 80 unique frames. Verify frame gaps and parse errors remain zero, `ground_filter` reports 0.065/0/0.035, the repeated floor clusters increase `ground_rejected_count`, no filtered point track at those coordinates appears, and a raised object remains visible. Do not claim physical success if the raised-object observation is not performed.

- [ ] **Step 6: Commit replay/docs changes after fresh verification**

```powershell
git add -- scripts/analyze_radar_scene.py tests/test_radar_replay.py docs/radar_front_view.md
git commit -m "test(radar): report ground suppression evidence"
```

- [ ] **Step 7: Audit final scope**

Run `git status --short`, `git log --oneline 724ad03..HEAD`, and `git diff --stat 724ad03..HEAD`. Confirm every changed path is radar-related and report hardware observations separately from automated claims.
