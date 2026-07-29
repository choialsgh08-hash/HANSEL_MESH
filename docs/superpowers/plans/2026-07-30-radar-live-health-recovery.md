# Radar Live Health Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Return the operator API to `live` after an isolated radar transport
fault has a quiet recovery interval, without erasing cumulative diagnostics or
weakening supervisor recovery gates.

**Architecture:** Capture periodic health uses deltas from the last accepted
periodic health record, while final health remains cumulative. The viewer
separates recoverable capture degradation from non-recoverable mission-log
integrity degradation and clears only the former on explicit `ok` health.

**Tech Stack:** Python 3 standard library, `unittest`, TI mmWave mission JSONL,
the existing `MissionLogWriter`, and the existing local HTTP radar API.

## Global Constraints

- Modify only radar-related source, tests, and radar documentation.
- Preserve the 30-frame supervisor probation, 2.5-second running frame
  timeout, retry backoff, port selection, and collision threshold. A later
  user-authorized live-evidence amendment changes only production cadence
  from the bundled 10 Hz cfg to a new 8 Hz cfg and requires a newly generated
  profile-bound calibration.
- Never reset or hide cumulative health counters.
- Final capture health and offline mission inspection remain unhealthy when
  any lifetime fault occurred.
- Do not open COM7/COM8 from diagnostic commands.
- Runtime artifacts remain untracked and must never be staged.

## File map

- `sensors/radar_capture.py`: owns periodic versus cumulative capture-health
  semantics.
- `monitor/radar_front.py`: owns current operator status and cumulative viewer
  diagnostics.
- `tests/test_radar_capture.py`: proves degraded interval, quiet recovery, and
  cumulative final health.
- `tests/test_radar_front.py`: proves selective viewer recovery and integrity
  latching.
- `docs/radar_auto_recovery.md`: explains current health versus mission
  history for operators.

---

### Task 1: Interval-based periodic capture health

**Files:**
- Modify: `sensors/radar_capture.py`
- Modify: `tests/test_radar_capture.py`

**Interfaces:**
- Consumes: cumulative capture counters already owned by
  `capture_radar_uart`.
- Produces: periodic `SensorHealth.status` based on new faults since the last
  successfully submitted periodic health record; final `SensorHealth` remains
  cumulative.

- [ ] **Step 1: Add a deterministic gap-then-quiet fake UART**

Add a fake that emits frames 1 and 3, waits long enough to emit periodic
health, emits frame 4, waits for another periodic record, and then stops:

```python
class GapThenQuietFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.read_count = 0

    def read(self, size):
        del size
        self.read_count += 1
        if self.read_count == 1:
            return one_point_packet(1) + one_point_packet(3)
        if self.read_count == 2:
            time.sleep(0.02)
            return b""
        if self.read_count == 3:
            return one_point_packet(4)
        if self.read_count == 4:
            time.sleep(0.02)
            return b""
        raise KeyboardInterrupt
```

- [ ] **Step 2: Write the failing interval-health test**

Run capture with `health_interval_s=0.01`, collect periodic health records by
the `health_kind=periodic` detail prefix, and assert:

```python
self.assertEqual(periodic[0].status, "degraded")
self.assertEqual(periodic[-1].status, "ok")
self.assertTrue(all(item.seq_gaps_total == 1 for item in periodic))
self.assertEqual(final.status, "degraded")
self.assertEqual(final.seq_gaps_total, 1)
```

- [ ] **Step 3: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_capture.RadarCaptureTests.test_periodic_health_recovers_after_quiet_interval_without_erasing_gap
```

Expected: FAIL because every health record after the gap is currently
`degraded`.

- [ ] **Step 4: Add an immutable health-fault snapshot**

In `sensors/radar_capture.py`, add a private frozen dataclass containing:

```python
post_sync_parse_errors: int
post_sync_discarded_bytes: int
incomplete_frames: int
missing_point_tlv_frames: int
missing_heatmap_frames: int
writer_drops: int
radar_frame_gaps: int
device_discontinuities: int
```

It exposes `increased_since(previous) -> bool` and `any_faults() -> bool`.
Construct the snapshot from the existing live counters immediately before
building each health record.

- [ ] **Step 5: Split periodic and final status calculations**

Initialize the periodic baseline to the all-zero snapshot. For periodic
health, use `current.increased_since(last_accepted_periodic)` as the
degradation input. For final health, use `current.any_faults()` plus the
existing unresolved startup/buffered-tail checks. Preserve existing
`frames_decoded == 0` and `point_cloud_frames == 0` branches.

In `emit_periodic_health_if_due`, update the baseline only in the successful
branch:

```python
if writer.submit(health):
    last_accepted_periodic_faults = current_faults
else:
    writer_drops += 1
```

- [ ] **Step 6: Run focused and capture regression tests**

Run:

```powershell
python -m unittest tests.test_radar_capture.RadarCaptureTests.test_periodic_health_recovers_after_quiet_interval_without_erasing_gap
python -m unittest tests.test_radar_capture
```

Expected: the focused test passes and the complete capture module has zero
failures/errors.

- [ ] **Step 7: Commit Task 1**

```powershell
git add -- sensors/radar_capture.py tests/test_radar_capture.py
git diff --cached --check
git commit -m "fix: recover periodic radar health"
```

---

### Task 2: Selective viewer recovery

**Files:**
- Modify: `monitor/radar_front.py`
- Modify: `tests/test_radar_front.py`

**Interfaces:**
- Consumes: periodic `SensorHealth(status="ok")` emitted by Task 1.
- Produces: top-level API status `live` after capture recovery while
  cumulative counters remain unchanged.

- [ ] **Step 1: Extend the sequence-gap regression test**

Keep the existing assertion that a clean frame alone does not clear
degradation. Then ingest:

```python
SensorHealth(
    header=SensorHeader(
        mission_id="test-mission",
        unit_id="head",
        boot_id="test-boot",
        producer_id="health-producer",
        stream_id="health/radar",
        seq=1,
        monotonic_ns=2_000_000_000,
    ),
    subject_stream_id="radar/front",
    status="ok",
    seq_gaps_total=1,
    detail="quiet interval after one gap",
)
```

Assert `snapshot["status"] == "live"` and
`snapshot["counters"]["frame_gaps_total"] == 1`.

- [ ] **Step 2: Add integrity-latch recovery tests**

For `note_parse_error` and `note_log_sequence_error`, use separate subtests:
ingest a valid frame, record the integrity error, ingest an `ok` health
record, and assert status remains degraded with the corresponding cumulative
counter still one.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```powershell
python -m unittest tests.test_radar_front.RadarFrontStateTests.test_sensor_sequence_gap_marks_live_view_degraded
python -m unittest tests.test_radar_front.RadarFrontStateTests.test_ok_health_does_not_clear_viewer_integrity_failures
```

Expected: the sequence-gap test fails because the viewer latch never clears;
the new integrity test establishes the required selective behavior.

- [ ] **Step 4: Separate degradation reasons**

Replace the single `_degraded_reason` field with:

```python
self._capture_degraded_reason: Optional[str] = None
self._integrity_degraded_reason: Optional[str] = None
```

Radar frame anomalies and non-`ok`/non-`starting` sensor health update the
capture reason. Invalid log records and log-sequence errors update the
integrity reason. `SensorHealth(status="ok")` clears only the capture reason.

Add a locked helper that returns integrity reason first, otherwise capture
reason. Use it in `snapshot()` and in `health.degraded_reason`.

- [ ] **Step 5: Preserve replay reset semantics**

Update the `reset_sensor_sequence_tracking` docstring: cumulative diagnostics
and integrity failures remain latched, while capture degradation can clear
only through explicit `ok` sensor health. Do not reset either counter set.

- [ ] **Step 6: Run focused and viewer regression tests**

Run:

```powershell
python -m unittest tests.test_radar_front.RadarFrontStateTests.test_sensor_sequence_gap_marks_live_view_degraded
python -m unittest tests.test_radar_front.RadarFrontStateTests.test_ok_health_does_not_clear_viewer_integrity_failures
python -m unittest tests.test_radar_front
```

Expected: zero failures/errors.

- [ ] **Step 7: Commit Task 2**

```powershell
git add -- monitor/radar_front.py tests/test_radar_front.py
git diff --cached --check
git commit -m "fix: recover live radar status"
```

---

### Task 3: Documentation, integrated verification, and live recovery

**Files:**
- Modify: `docs/radar_auto_recovery.md`
- Read only before live action:
  `runtime/radar-supervisor-20260729-153809258.json`

**Interfaces:**
- Consumes: Tasks 1 and 2.
- Produces: operator guidance, full regression evidence, and a running
  recovered hardware epoch.

- [ ] **Step 1: Document status semantics**

Add a short section stating that top-level `LIVE/DEGRADED` describes the
current capture interval, diagnostic counters are lifetime totals, and final
mission health remains cumulative. Explicitly state that a single frame gap
does not trigger supervisor recovery while fresh frames continue.

- [ ] **Step 2: Run radar-focused integration tests**

Run:

```powershell
python -m unittest tests.test_radar_capture tests.test_radar_front tests.test_radar_watchdog tests.test_radar_supervisor tests.test_radar_supervisor_integration
```

Expected: zero failures/errors.

- [ ] **Step 3: Run the full Python and browser suites**

Run:

```powershell
python -m unittest discover -s tests -v
node --test tests/web/radar_scene.test.js tests/web/radar_panel.test.js
```

Require zero failures, errors, and skipped tests from Python, and exit code
zero from Node.

- [ ] **Step 4: Review the exact live ownership boundary**

Read the active manifest and verify:

- state is `RUNNING`;
- supervisor PID is alive;
- capture PID and viewer PID are children owned by that manifest;
- selected XDS serial is `RI32RI32`;
- selected application port is COM8.

Abort the live action if any identity differs.

- [ ] **Step 5: Exercise automatic recovery once**

Stop only the manifest-owned capture PID. Do not stop the supervisor, open a
serial port, invoke bootloader commands, or reset unrelated processes. Poll
the manifest until it enters a later epoch and returns to `RUNNING`.

- [ ] **Step 6: Verify the recovered epoch**

For at least 120 seconds, poll the API every two seconds and require:

- frames increase on every usable sample;
- status is `live` after the first quiet periodic interval;
- age stays below 750 ms and reported frame rate remains near 10 Hz;
- supervisor remains `RUNNING`;
- no additional recovery occurs;
- incomplete, writer-drop, parse-error, and device-discontinuity counters
  remain zero;
- any historical counter shown belongs only to the current immutable epoch.

- [ ] **Step 7: Commit documentation**

```powershell
git add -- docs/radar_auto_recovery.md
git diff --cached --check
git commit -m "docs: explain radar health recovery"
```

- [ ] **Step 8: Final review and push**

Run fresh `git diff --check`, full tests, process/API checks, and verify that
only `runtime/` is untracked. Push without force:

```powershell
git push origin codex/radar-auto-recovery
git rev-parse HEAD
git ls-remote --heads origin codex/radar-auto-recovery
```

The local and remote hashes must match exactly.
