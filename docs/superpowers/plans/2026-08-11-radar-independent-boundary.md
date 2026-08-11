# Radar Independent Boundary Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Display a temporally confirmed single physical radar wall without inventing a corridor, while preventing brief parser transitions from blanking the operator view.

**Architecture:** Extend the existing deterministic reflectivity estimator with a separate display-only boundary history.  Serialize it beside the corridor, validate it in the browser scene parser, and render it below the confirmed corridor/path layers.  Keep every planner and hazard input unchanged.

**Tech Stack:** Python 3 unittest backend, browser JavaScript canvas renderer, Node test runner.

---

### Task 1: Lock the live wall and invalid-scene behavior with failing tests

**Files:**
- Modify: `tests/test_radar_corridor.py`
- Modify: `tests/test_radar_scene.py`
- Modify: `tests/web/radar_scene.test.js`
- Modify: `tests/web/radar_panel.test.js`

Add regressions for the measured 9 cm sampling gap, mirrored geometry, dense scatter rejection, scene serialization, parser validation, renderer calls, and the two-frame/500 ms invalid-scene hold. Run the focused tests and record their expected failures.

### Task 2: Implement the independent boundary estimator and scene contract

**Files:**
- Modify: `monitor/radar_corridor.py`
- Modify: `monitor/radar_scene.py`
- Modify: `monitor/web/radar_scene.js`

Add a three-of-five compatible boundary history, the strict display-only admission gate, and a `reflectivity_boundaries` payload. Keep corridor path-obstacle production unchanged. Make the focused Python and parser tests pass.

### Task 3: Render boundaries and retain brief invalid snapshots

**Files:**
- Modify: `monitor/web/radar_panel.js`
- Modify: `monitor/radar_front.py`
- Modify: `monitor/web/radar_front.html`
- Modify: `tests/test_radar_front.py`

Draw confirmed independent walls without duplicating confirmed corridor walls. Add an injected clock and bounded presentation hold in `acceptSnapshot`. Bump the UI build identifier and make focused renderer/HTTP tests pass.

### Task 4: Verify against software and the connected sensor

Run all radar Python and Node suites, syntax/compile checks, and diff checks. Restart the local radar stack, sample the API until the independent right boundary confirms, and visually inspect the delivered browser view. Commit only the scoped radar/docs/test changes.
