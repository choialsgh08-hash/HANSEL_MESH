# Radar Corridor Surface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render stable heatmap wall surfaces separately from hard obstacles, use anisotropic collision geometry, suppress the 3.5--5 cm ground fringe, and prove a right-diagonal corridor can produce a rightward path.

**Architecture:** A pure five-frame reflectivity component converts calibrated residual cells into a display-only Cartesian surface. `RadarSceneEstimator` publishes that surface without feeding it to hazard or planning, classifies definite ground and the measurement fringe before all evidence, and replaces circular collision membership with radial/tangential ellipses. The strict browser parser validates the new contract and the canvas renderer draws crisp surface cells below confirmed driving evidence.

**Tech Stack:** Python 3 standard library and `unittest`; browser JavaScript and Node's built-in test runner; HTML canvas; existing TI heatmap/clutter calibration pipeline.

## Global Constraints

- Touch only radar-related source, tests, configuration, and documentation.
- Preserve raw `RadarFrame` data and recording schemas.
- `reflectivity_surface` is display-only and must never enter probability occupancy, path, hazard, or free-space claims.
- Existing robot width 0.15 m, safety margin 0.03 m, and danger latch 0.10/0.13 m remain unchanged.
- Existing confirmed point and point-correlated heatmap requirements remain unchanged.
- Ground and ground-fringe returns never enter tracking, heatmap correlation, maps, path, hazard, or operator track rendering.
- No body mask, servo pitch transport, Doppler odometry, IMU fusion, or global SLAM is added.

---

### Task 1: Five-frame display-only reflectivity surface

**Files:**
- Create: `monitor/radar_reflectivity.py`
- Create: `tests/test_radar_reflectivity.py`
- Modify: `monitor/radar_scene.py`
- Modify: `tests/test_radar_scene.py`

**Interfaces:**
- Produces: `RadarReflectivitySurface(radius_m=0.50, resolution_m=0.025, history_frames=5, required_hits=3)`.
- Produces: `ingest(observations, received_at, elapsed_frame_windows=1) -> None`, where each observation is `(forward_m, lateral_m, intensity)`.
- Produces: `payload(now) -> dict[str, object]` with strict metadata and ordered cells.
- Produces: `reset() -> None`.
- Extends: `_heatmap_clusters(frame)` to return cluster summaries, candidate count, and every accepted residual cell used by the display surface.

- [ ] **Step 1: Write pure failing tests**

Add tests that call the desired API and prove: two hits publish nothing; three of five publish one quantized cell; skipped frame windows insert misses; three misses remove a prior cell; reset empties the payload; invalid observations cannot emit NaN; and cells are ordered by forward then lateral coordinate.

```python
surface.ingest([(0.401, 0.061, 0.8)], 1.000)
surface.ingest([(0.399, 0.059, 0.9)], 1.125)
surface.ingest([(0.400, 0.060, 1.0)], 1.250)
self.assertEqual(surface.payload(1.250)["cells"][0]["hits"], 3)
```

- [ ] **Step 2: Run and verify RED**

Run: `python -B -m unittest tests.test_radar_reflectivity -v`

Expected: import failure because `monitor.radar_reflectivity` is absent.

- [ ] **Step 3: Implement the minimal pure component**

Use a fixed-length deque of per-frame dictionaries keyed by Cartesian grid coordinates. Quantize with `round(value / resolution_m)`, merge duplicate observations with maximum intensity, insert `elapsed_frame_windows - 1` empty dictionaries, and publish only keys with at least three hits. Clamp intensity to `[0, 1]`; reject booleans and non-finite coordinates or times without throwing from the scene loop.

- [ ] **Step 4: Run pure GREEN**

Run: `python -B -m unittest tests.test_radar_reflectivity -v`

Expected: all reflectivity tests pass.

- [ ] **Step 5: Write scene integration RED tests**

Feed repeated calibrated heatmap frames without point correlation and assert that after three frames the surface contains cells while `probabilistic_local_map.cells`, `obstacle_tracks`, and `hazard` remain unaffected. Add a dropped-frame case and a reset case. Assert the largest cluster's component cells remain spatially distinct rather than collapsing to one centroid.

- [ ] **Step 6: Integrate accepted residual cells and publish the payload**

Return each candidate's `forward_m`, `lateral_m`, and `min(1, max(0, residual_db / 24))` from the calibrated heatmap pass. Feed only those observations to the surface after sequence-gap calculation. Publish `reflectivity_surface` in every scene snapshot and reset it with the estimator. Do not reference the surface from any planning or hazard method.

- [ ] **Step 7: Run focused backend GREEN**

Run: `python -B -m unittest tests.test_radar_reflectivity tests.test_radar_scene -v`

Expected: both suites pass.

- [ ] **Step 8: Commit the backend surface**

Run `git diff --check`, then commit only the four Task 1 files with `feat(radar): retain calibrated reflectivity surfaces`.

### Task 2: Strict parser and crisp surface renderer

**Files:**
- Modify: `monitor/web/radar_scene.js`
- Modify: `monitor/web/radar_panel.js`
- Modify: `monitor/web/radar_front.html`
- Modify: `monitor/radar_front.py`
- Modify: `tests/web/radar_scene.test.js`
- Modify: `tests/web/radar_panel.test.js`
- Modify: `tests/test_radar_front.py`

**Interfaces:**
- Consumes: `scene.reflectivity_surface` from Task 1.
- Produces: `presentation.reflectivitySurface` with camel-case metadata and detached cells.
- Produces: `drawReflectivitySurface(ctx, transform, surface, options)`.
- Draw order: reflectivity surface, probability map, tracks, footprint/path/labels.

- [ ] **Step 1: Write strict parser RED tests**

Extend the complete scene fixture and add mutation cases for every metadata field, cell coordinate, intensity, persistence, hit count, age, radius boundary, and duplicate quantized coordinate. Require exact metadata values from the design and detached clones.

- [ ] **Step 2: Write renderer RED tests**

Spy on layer methods and assert the surface is invoked before probability and every track layer in both main and collision views. Assert adjacent cells create filled grayscale cells and only exposed four-neighbor edges; nonadjacent cells are not joined. Confirm no surface cell reaches `drawTracks`, `drawProbabilityCell`, hazard distance, or path inputs.

- [ ] **Step 3: Run and verify Node RED**

Run with the bundled Node executable:

```powershell
$node='C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe'
& $node --test tests/web/radar_scene.test.js tests/web/radar_panel.test.js
```

Expected: failures for the missing parser field and drawing method.

- [ ] **Step 4: Implement parser and renderer**

Validate exact metadata, finite bounded cell fields, grid alignment, uniqueness, and age. Draw one cell-sized rectangle per published cell with opacity derived from `0.55 * intensity + 0.45 * persistence`; draw a one-pixel boundary only where the four-neighbor cell is absent. Clip to 0.50 m and never use canvas blur or shadow.

- [ ] **Step 5: Update visible contract and build IDs**

Add a `WHITE = STABLE REFLECTIVITY` legend tag, a diagnostic cell count, and bump the shared HTML/JS asset build from R33 to R34 in producer, HTML, and HTTP tests.

- [ ] **Step 6: Run web GREEN**

Run JavaScript syntax checks, both Node suites, and `python -B -m unittest tests.test_radar_front -v`. Every command must exit zero.

- [ ] **Step 7: Commit the web surface**

Run `git diff --check`, then commit only Task 2 files with `feat(radar): draw stable reflectivity corridor`.

### Task 3: Ground fringe and anisotropic collision membership

**Files:**
- Modify: `monitor/radar_ground.py`
- Modify: `monitor/radar_scene.py`
- Modify: `monitor/radar_front.py`
- Modify: `monitor/web/radar_scene.js`
- Modify: `monitor/web/radar_panel.js`
- Modify: `monitor/web/radar_front.html`
- Modify: `tests/test_radar_ground.py`
- Modify: `tests/test_radar_scene.py`
- Modify: `tests/test_radar_front.py`
- Modify: `tests/web/radar_scene.test.js`
- Modify: `tests/web/radar_panel.test.js`

**Interfaces:**
- Extends: `RadarMountGeometry(..., ground_uncertainty_m=0.015)`.
- Produces: `ground_classification(...) -> "ground" | "ground_fringe" | None`.
- Adds: `ground_fringe_rejected_count` and effective rejection height to the strict scene/UI contract.
- Replaces: `_obstacle_collision_radii` with `_obstacle_collision_axes(obstacle) -> ((radial, tangential), (grid_radial, grid_tangential))`.
- Replaces: disk membership with an ellipse oriented along the obstacle's bearing from the radar origin.

- [ ] **Step 1: Write ground-fringe RED tests**

Prove the transformed 0.035 m boundary is `ground`, 0.035001--0.050 m is `ground_fringe`, and above 0.050 m is retained. Reject negative/non-finite uncertainty and configurations where 0.035 + uncertainty is at or above sensor height. At scene level prove fringe points remain raw but never create tracks, maps, path blockage, heatmap confirmation, or danger.

- [ ] **Step 2: Run ground RED**

Run: `python -B -m unittest tests.test_radar_ground tests.test_radar_scene -v`

Expected: missing constructor field, classifier, and diagnostic failures.

- [ ] **Step 3: Implement fringe classification and contracts**

Classify using robot coordinates once, increment exactly one of the two counters, and reject before clutter/quality/tracking. Extend CLI defaults, payload validation, HTML diagnostics, and parser mutation tables with `ground_uncertainty_m`, `effective_rejection_height_m`, and `ground_fringe_rejected_count`.

- [ ] **Step 4: Write anisotropic planner RED tests**

Construct obstacles whose radial uncertainty is 0.20 m and lateral uncertainty is 0.03 m. Prove a test point displaced tangentially outside the narrow axis is clear even though the old max-radius disk would block it, while radial and exact-boundary points remain blocked. Prove invalid uncertainty/support values fail closed. Add raster-versus-exact-path agreement tests.

- [ ] **Step 5: Implement elliptical collision geometry**

Project query delta onto the obstacle radial unit vector and its tangential perpendicular. A point is inside when `(dr / radial_axis)^2 + (dt / tangential_axis)^2 <= 1`. For a zero-range obstacle, use robot forward/right axes deterministically. Use heatmap range-bin resolution for radial localization rather than cluster spread and retain the old maximum collision radius diagnostic as `max(radial, tangential)`.

- [ ] **Step 6: Run backend and web GREEN**

Run focused ground/scene/front Python tests plus both Node suites and syntax checks. Every command must exit zero.

- [ ] **Step 7: Commit fringe and planner geometry**

Run `git diff --check`, then commit only Task 3 files with `feat(radar): use ground fringe and anisotropic clearance`.

### Task 4: Right-diagonal corridor regression and live verification

**Files:**
- Modify: `tests/test_radar_scene.py`
- Modify: `docs/radar_front_view.md`

**Interfaces:**
- Consumes: confirmed obstacle geometry from Task 3.
- Verifies: a corridor whose center shifts right produces a path with positive lateral endpoint and goal angle.

- [ ] **Step 1: Write the diagonal-corridor RED test**

Generate two confirmed wall rows at 0.05 m forward spacing with at least 0.21 m free passage. Shift both wall rows right as forward distance increases. Assert the planner expands beyond one state, reaches a positive-lateral goal, produces a positive goal angle, and every 0.01 m path sample remains outside all collision ellipses.

- [ ] **Step 2: Run the test and classify the result**

If it fails because the goal selector still prefers straight motion, adjust only corridor scoring/goal selection. If it passes once anisotropic geometry is active, retain the regression without changing selector thresholds.

- [ ] **Step 3: Make the minimal selector change if RED requires it**

Score complete corridor strips rather than endpoint clearance alone, retain the existing unknown-space penalty, and keep three-frame route switching. Do not allow unconfirmed reflectivity cells into the score.

- [ ] **Step 4: Run complete automated verification**

Run all `test_radar*.py` tests, every `tests/web/*.test.js` test, JavaScript syntax checks, `python -m compileall -q monitor sensors scripts tests`, and `git diff --check`.

- [ ] **Step 5: Restart the current stack and sample unchanged hardware**

Restart through the existing supervisor, wait for a live R34 snapshot, and collect at least 100 unique frames. Report surface cell count and persistence, confirmed hard obstacles, ground/fringe rejection counts, maximum radial/tangential axes, path expansions/status, goal lateral/angle, and frame continuity. Do not call a real point false merely because it blocks the path.

- [ ] **Step 6: Document and commit verification**

Update the operator guide with the visual-versus-hard-evidence distinction, ground fringe, ellipse semantics, and hardware observations. Run fresh focused and full checks, then commit the test/document files with `test(radar): verify diagonal corridor guidance`.

- [ ] **Step 7: Audit final scope**

Run `git status --short`, `git log --oneline e555339..HEAD`, `git diff --stat e555339..HEAD`, and `git diff --check e555339..HEAD`. Every changed path must be radar-related.
