# Radar Two-Wall Corridor Guidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect two stable reflectivity walls, use their axis as a soft local-path heading, and render both measured wall segments without weakening confirmed-obstacle collision safety.

**Architecture:** A new pure `ReflectivityCorridorEstimator` converts the existing three-of-five reflectivity cells into a temporally confirmed two-line geometry payload. `RadarSceneEstimator` passes only its confirmed guidance heading into `_plan_path`; the original probability/reactive obstacles remain the complete collision input. The browser strictly parses the new payload and renders measured walls below probability/path/track guidance.

**Tech Stack:** Python 3.12, `unittest`, deterministic geometry using the standard library, browser Canvas 2D, Node test runner.

## Global Constraints

- Reflectivity geometry may bias goal orientation but must never enter probability, hazard, or collision membership.
- A wall requires intensity >= 0.30, at least four cells, span >= 0.14 m, and point-to-line residual <= 0.026 m.
- A pair requires heading difference <= 10 degrees, normal separation 0.14--0.55 m, and at most one shared cell.
- Corridor confirmation is three compatible results in five complete frame slots; dropped frames insert empty slots.
- Guidance heading is clamped to +/-55 degrees; the planner trusted field remains +/-60 degrees.
- Candidate walls are dashed and never influence planning. Confirmed walls are solid.
- The 0.15 m robot width, 0.03 m per-side margin, and existing 0.10/0.13 m danger latch remain unchanged.
- A central confirmed near obstacle must still produce `blocked_near_field` even when the wall heading points right.

---

### Task 1: Deterministic two-wall estimator

**Files:**
- Create: `monitor/radar_corridor.py`
- Create: `tests/test_radar_corridor.py`

**Interfaces:**
- Consumes: `Sequence[Mapping[str, object]]` stable reflectivity cells with `forward_m`, `lateral_m`, and `intensity`.
- Produces: `ReflectivityCorridorEstimator.ingest(cells, elapsed_frame_windows=1)`, `.payload()`, and `.reset()`.
- Payload keys: `status`, `method`, `history_frames`, `required_hits`, `hits`, `measured_heading_deg`, `guidance_heading_deg`, `width_m`, `confidence`, and `walls`.

- [ ] **Step 1: Write failing estimator tests**

Create a recorded-layout fixture containing the 12-cell +63.4 degree wall and six-cell +67.4 degree wall. Assert that one ingest is `candidate`, three compatible ingests are `confirmed`, guidance is +55 degrees, width is approximately 0.193 m, endpoints are finite and ordered, and exactly two walls are returned. Add tests for shuffled deterministic input, one wall, random scatter, two misses, dropped-frame slots, and reset.

- [ ] **Step 2: Run the estimator tests and capture RED**

Run:

```powershell
python -B -m unittest tests.test_radar_corridor -v
```

Expected: import failure because `monitor.radar_corridor` does not exist.

- [ ] **Step 3: Implement the pure estimator**

Create:

```python
class ReflectivityCorridorEstimator:
    def __init__(self, history_frames: int = 5, required_hits: int = 3): ...
    def reset(self) -> None: ...
    def ingest(self, cells, elapsed_frame_windows: int = 1) -> None: ...
    def payload(self) -> dict: ...
```

Enumerate finite candidate lines from cell pairs, normalize headings to
`[-90, 90]`, deduplicate identical inlier sets, retain a bounded deterministic
candidate list, and select the best compatible non-overlapping pair. Average
compatible history fits with medians so one noisy frame cannot move a wall.

- [ ] **Step 4: Run estimator tests and commit**

Run the focused suite, `py_compile`, and `git diff --check`, then commit only the estimator and its tests:

```powershell
git commit -m "feat(radar): estimate stable two-wall corridors"
```

---

### Task 2: Scene and safe planner integration

**Files:**
- Modify: `monitor/radar_scene.py`
- Modify: `monitor/radar_path.py`
- Modify: `tests/test_radar_scene.py`
- Modify: `tests/test_radar_path.py`

**Interfaces:**
- Consumes: `ReflectivityCorridorEstimator.payload()`.
- Produces: required scene field `reflectivity_corridor` and `_plan_path(..., preferred_goal_angle_deg=None)` behavior.

- [ ] **Step 1: Write failing scene and selector tests**

Add tests proving:

```python
path = RadarSceneEstimator._plan_path(
    [], preferred_goal_angle_deg=55.0,
)
self.assertGreater(path["goal_angle_deg"], 40.0)
self.assertGreater(path["goal_lateral_m"], 0.0)
```

Add a central confirmed obstacle and assert the same hint preserves a positive
goal but still returns `blocked_near_field` with `points == [[0.0, 0.0]]`.
Add a full scene fixture that confirms the two-wall payload while probability
and hazard results remain identical to the no-corridor control. Add a selector
test where a repeated stopped candidate changes goal angle from 0 to +55
without incrementing `path_switch_accepted_count`.

- [ ] **Step 2: Run the focused tests and capture RED**

```powershell
python -B -m unittest tests.test_radar_scene tests.test_radar_path -v
```

Expected: missing corridor field, unsupported planner keyword, and frozen stopped metadata failures.

- [ ] **Step 3: Wire the estimator and goal hint**

Instantiate/reset the corridor estimator with the scene. After each
reflectivity ingest, feed its current cells with the frame-gap window count.
Publish its payload in every scene snapshot. Pass only a confirmed
`guidance_heading_deg` to `_plan_path`.

Update dynamic-goal selection so a preferred heading chooses the closest
heading inside a hard-obstacle free interval. When no free interval exists,
retain the preferred heading in metadata while collision search remains
blocked. Set `goal_selection_mode` to distinguish corridor-guided selection.

In `PathSelectionHysteresis`, refresh `_selected` from a repeated stopped
candidate while preserving selection time, switch count, and blocked overlay.

- [ ] **Step 4: Prove collision separation and commit**

Run both focused suites plus `tests.test_radar_local_map` and
`tests.test_radar_reflectivity`. Confirm no test observes added probability
cells or a weakened hazard. Commit:

```powershell
git commit -m "feat(radar): guide local path from wall geometry"
```

---

### Task 3: Strict browser scene contract

**Files:**
- Modify: `monitor/web/radar_scene.js`
- Modify: `tests/web/radar_scene.test.js`

**Interfaces:**
- Consumes: backend `scene.reflectivity_corridor`.
- Produces: detached presentation property `reflectivityCorridor` with camel-cased metadata and cloned wall endpoints.

- [ ] **Step 1: Write failing parser tables**

Extend complete new-scene fixtures with one valid confirmed corridor. Add table
tests for missing/null objects, invalid status/method/history/hits, non-finite
headings/width/confidence, wrong guidance clamp, wrong wall count or IDs,
non-pair/non-finite endpoints, short spans, invalid support counts, and duplicate
wall IDs. Mutate parsed endpoints and prove the source snapshot is detached.

- [ ] **Step 2: Run parser RED**

```powershell
node --test tests/web/radar_scene.test.js
```

Expected: valid fixtures lack the new presentation field and malformed payloads are accepted.

- [ ] **Step 3: Implement strict parsing**

Add `parseReflectivityCorridor` and require the field for probability scenes.
Allow `unavailable` only with null geometry and no walls; require two valid
walls for `candidate` and `confirmed`; require `confirmed` hits >= 3 and
candidate hits in 1--2.

- [ ] **Step 4: Run parser tests and commit**

Run Node syntax and parser tests, then commit:

```powershell
git commit -m "feat(radar): validate wall corridor scene data"
```

---

### Task 4: Two-wall operator rendering and R37 assets

**Files:**
- Modify: `monitor/web/radar_panel.js`
- Modify: `monitor/web/radar_front.html`
- Modify: `monitor/radar_front.py`
- Modify: `tests/web/radar_panel.test.js`
- Modify: `tests/test_radar_front.py`

**Interfaces:**
- Consumes: presentation `reflectivityCorridor`.
- Produces: `drawReflectivityCorridor(ctx, transform, corridor)` and UI build `20260810-wall-corridor-r37` / `UI R37`.

- [ ] **Step 1: Write failing renderer and HTTP tests**

Assert that confirmed geometry creates two pale-cyan line strokes, a faint
measured-corridor polygon, and a width label. Assert candidate geometry uses a
dash pattern and is still visual only. Assert unavailable geometry draws
nothing. Record renderer call order and require reflectivity surface, corridor,
probability map, path, then tracks. Add a positive-lateral guidance assertion
for the stopped path and update HTTP build/cache expectations to R37.

- [ ] **Step 2: Run renderer/HTTP RED**

```powershell
node --test tests/web/radar_panel.test.js
python -B -m unittest tests.test_radar_front.RadarFrontHttpTests -v
```

Expected: missing drawing method/order calls and stale R36 asset identifiers.

- [ ] **Step 3: Implement measured-wall rendering**

Draw only supplied endpoints. Fill the four-endpoint region with low alpha and
never use `FREE` or `SAFE` copy. Draw each wall with a dark underlay and crisp
pale-cyan core; apply dashes only to candidates. Label confirmed/candidate,
rounded width centimeters, and `NARROW` when width is below 0.21 m. Invoke the
layer in both main and collision views. Bump Python, JS, HTML, and tests to R37.

- [ ] **Step 4: Run web and HTTP suites and commit**

Run all Node radar tests, Python HTTP tests, syntax, and diff checks. Commit:

```powershell
git commit -m "feat(radar): draw measured two-wall corridors"
```

---

### Task 5: Full regression and live board gate

**Files:**
- Modify only if a test exposes a requirement defect; add a focused RED before any correction.

**Interfaces:**
- Consumes: the complete R37 stack.
- Produces: verified live service on `http://127.0.0.1:8081/`.

- [ ] **Step 1: Run complete automated verification**

Run all radar Python modules, all radar Node suites, JS syntax checks,
`compileall`, `git diff --check`, and `git status --short`. Every command must
exit zero and the worktree must contain no uncommitted source changes.

- [ ] **Step 2: Restart only the verified R36 supervisor as R37**

Verify the current supervisor command line and PID before stopping it. Start
the same COM8, XDS serial, 8 Hz config, profile, calibration, and port with run
ID `codex-wall-corridor-20260810-r37` in a hidden process.

- [ ] **Step 3: Validate consecutive live frames**

Collect at least 25 unique complete frames. Require:

```text
ui_build_id == 20260810-wall-corridor-r37
status in {live, degraded}
calibration_status == ok
reflectivity_corridor.status == confirmed
len(reflectivity_corridor.walls) == 2
measured_heading_deg > 40
guidance_heading_deg > 40
planned_path.goal_angle_deg > 40
planned_path.goal_lateral_m > 0
transport gap delta == 0
```

Report `blocked_near_field` as expected while the confirmed central near point
remains; do not suppress that point to make the gate pass.

- [ ] **Step 4: Record final evidence**

Run `git log -5 --oneline`, `git show --check`, and a final live API snapshot.
Report exact test counts, commits, measured headings/width, path status, FPS,
and any remaining hardware limitation.
