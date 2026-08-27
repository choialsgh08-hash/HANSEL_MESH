# Radar Evidence Guidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace synthetic wall reconstruction and unsafe empty-space routing with a continuous 0–30 cm reflection-evidence view, evidence-bounded steering advice, and point-only 10 cm collision warning.

**Architecture:** The calibrated full heatmap remains the complete operator-visible measurement. The probability grid consumes each heatmap observation once; the UART two-chirp ADC phase surface stays diagnostic and never acts as independent occupancy confirmation. The UI renders measured reflection bands without joining them into walls and presents a short candidate heading only when both sides of the robot-width envelope are observed; otherwise it explicitly reports insufficient coverage without blanking the evidence view.

**Tech Stack:** Python 3.12 `unittest`, browser JavaScript, Node `node:test`, HTML canvas, current TI UART heatmap/ADC stream.

## Global Constraints

- Main view is robot-relative and limited to 0–30 cm.
- Unknown is never classified as free or safe.
- Only confirmed point tracks can produce the 10 cm collision warning.
- Heatmap amplitude changes confidence continuously; no scene-shape threshold or layout-specific constant is allowed.
- The last-two-chirp UART ADC surface is display-only because it is correlated with the firmware heatmap.
- Do not modify code outside radar-related files.

---

### Task 1: Remove Correlated ADC Occupancy Fusion

**Files:**
- Modify: `monitor/radar_scene.py`
- Modify: `monitor/radar_adc.py`
- Test: `tests/test_radar_scene.py`
- Test: `tests/test_radar_adc.py`

**Interfaces:**
- Consumes: calibrated `reflectivity_surface.cells` and display-only `adc_range_angle_surface`
- Produces: probability cells sourced only from `point`, `heatmap`, or `mixed`

- [ ] Add a regression proving an ADC peak coincident with its heatmap lobe cannot create a second map observation or `adc_heatmap_fused` cell.
- [ ] Run the focused test and capture the expected failure.
- [ ] Remove `fused_adc_heatmap_measurements` from the driving-map update path and mark the ADC contract display-only/correlated.
- [ ] Run focused Python tests.

### Task 2: Replace Synthetic Walls and Unsafe Route with Evidence-Bounded Guidance

**Files:**
- Modify: `monitor/web/radar_panel.js`
- Modify: `monitor/web/radar_front.html`
- Modify: `monitor/radar_front.py`
- Test: `tests/web/radar_panel.test.js`
- Test: `tests/test_radar_front.py`

**Interfaces:**
- Consumes: `reflectivitySurface`, `probabilisticLocalMap`, `plannedPath`, `hazard`
- Produces: direct polar reflection rendering and `deriveOperatorGuide()` result with `advisory`, `evidenceState`, `direction`, `confidence`, and optional points

- [ ] Add renderer regressions proving sparse returns are never joined/labeled as walls and one-sided/unobserved evidence produces no directional route.
- [ ] Run focused Node tests and capture expected failures.
- [ ] Replace `drawOperatorWalls` with direct reflection-return bands and change guide fallback to withhold direction unless both sides bound a robot-width passage.
- [ ] Change visible copy from wall/path claims to reflection evidence/candidate heading and increment the build ID.
- [ ] Run focused Node and HTTP asset tests.

### Task 3: Live 11-Wall Verification

**Files:**
- Create: `runtime/radar-r65-before-after.json` (ignored runtime evidence)
- Modify: `docs/radar_front_view.md`

**Interfaces:**
- Consumes: live `/api/radar`, current fixed 11-wall placement
- Produces: measured before/after cell sources, coverage state, candidate direction, parser state, and screen capture

- [ ] Record the pre-change snapshot and summarize duplicate evidence and current route.
- [ ] Restart the live stack on the same COM8/profile/calibration with the new build.
- [ ] Wait for live frames, collect at least 100 complete frames, and record post-change source/coverage/direction stability.
- [ ] Verify the page never shows `SCENE VERIFYING .../undefined` and remains visually populated during insufficient geometry.
- [ ] Run the complete radar Python and Node suites, syntax/compile checks, and `git diff --check`.
