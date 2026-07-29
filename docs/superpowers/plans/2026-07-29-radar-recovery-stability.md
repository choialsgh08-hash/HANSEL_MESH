# Radar Recovery Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent short-lived IWRL6432 recovery epochs from appearing as a live driving view, slow repeated reset loops safely, update the selected RI32 XDS110 probe to firmware 3.0.0.43, and prove the same radar epoch remains healthy for 15 minutes.

**Architecture:** Keep the existing watchdog qualification and fail-closed viewer contract, but raise the recovery probation defaults to 30 consecutive frames within five seconds. The supervisor owns retry state across epochs: a faulted running epoch consumes the next bounded delay, a new `RUNNING` epoch does not clear that delay, and only 30 seconds of fresh healthy evidence resets it. After automated verification, stop only the manifest-owned radar processes, update the single preflight-identified XDS110, restart with a new immutable run ID, and observe the API, manifest, UART counters, COM inventory, and browser for 15 minutes.

**Tech Stack:** Python 3.12, `unittest`, immutable dataclasses, Windows PowerShell/CIM, TI `xdsdfu.exe`, IWRL6432BOOST/XDS110, local HTTP radar viewer.

## Global Constraints

- Scope is the radar stack only; do not change non-radar folders or unrelated files.
- Preserve the 10 Hz `configs/radar/iwrl6432_3d_operator_near_10hz.cfg` profile, its 16-by-128 heatmap, 1,250,000-baud data UART, current calibration file, and current map UI.
- A recovering epoch needs 30 consecutive complete, exact-profile, exact-heatmap frames within 5.0 seconds before viewer switch and `RUNNING`.
- Keep first-frame timeout at 3.0 seconds, running frame timeout at 2.5 seconds, viewer stale threshold at 0.75 seconds, and viewer fault threshold at 2.0 seconds.
- Fault backoff is exactly 0.5, 1.0, 2.0, 4.0, then 5.0 seconds for later consecutive failures.
- Promotion to `RUNNING` and same-epoch viewer restart do not reset retry delay; 30.0 continuous healthy seconds do.
- Never display stale evidence as current or free space.
- Do not change collision thresholds, scene estimation, clutter calibration, frame rate, or heatmap resolution.
- Preserve immutable epoch artifacts, manifest history, process ownership checks, and parent-death cleanup.
- Preserve the untracked `runtime/` evidence directory; never stage, remove, or rewrite historical evidence.
- Test changes must be observed failing before production changes and passing afterward.
- Firmware operations are permitted only when `xdsdfu -e` identifies exactly one runtime XDS110 with serial `RI32`; otherwise stop before `-m` or `-f`.
- The authorized firmware image is `C:\ti\uniflash_9.6.0\deskdb\content\TICloudAgent\win\ccs_base\common\uscif\xds110\firmware_3.0.0.43.bin` with SHA-256 `F00B865F0F179F5B0499E4A05894ECA6C5DC71A0A8C4D741B9F731DF47A65658`.
- If flashing, DFU re-enumeration, runtime re-enumeration, serial verification, or version verification fails, leave the radar stack stopped and report the exact state. Do not attempt bootloader recovery in this plan.
- The separately approved lower-bandwidth profile and new calibration are a later fallback only if this plan's 15-minute soak fails.

## File Structure

- `sensors/radar_supervisor.py`: owns probation defaults, cross-epoch retry delay, running-health timer, state transitions, and fault recovery ordering.
- `scripts/run_radar_stack.py`: exposes the safe default probation values and constructs `RadarSupervisorConfig`.
- `sensors/radar_watchdog.py`: remains behaviorally unchanged; its existing configurable consecutive-frame and deadline logic enforces the new defaults.
- `tests/test_radar_watchdog.py`: proves five frames are insufficient, 30 consecutive frames pass before five seconds, and a bad frame resets the count.
- `tests/test_radar_supervisor.py`: proves viewer gating, fault backoff progression, shutdown during recovery backoff, and 30-second healthy reset.
- `tests/test_radar_stack_processes.py`: proves launcher defaults and retains process ownership/cleanup regression coverage.
- `tests/test_radar_front.py`: retains stale/fault fail-closed regression coverage without production UI changes.
- `tests/test_radar_supervisor_integration.py`: retains immutable epoch and real child-process integration coverage.
- `docs/superpowers/specs/2026-07-29-radar-recovery-stability-design.md`: approved behavior contract; no behavioral edits.
- `runtime/`, `missions/`, and `captures/`: generated untracked hardware evidence only; never commit.

---

### Task 1: Enforce a 30-frame, five-second recovery probation

**Files:**

- Modify: `tests/test_radar_watchdog.py:128-343`
- Modify: `tests/test_radar_supervisor.py:51-160`
- Modify: `tests/test_radar_supervisor.py:452-590`
- Modify: `tests/test_radar_supervisor.py:1130-1290`
- Modify: `tests/test_radar_supervisor.py:1240-1300`
- Modify: `tests/test_radar_stack_processes.py:842-860`
- Modify: `sensors/radar_supervisor.py:83-165`
- Modify: `sensors/radar_supervisor.py:902-940`
- Modify: `scripts/run_radar_stack.py:85-125`

**Interfaces:**

- Consumes: `RadarEpochWatchdog(..., required_consecutive_frames: int, verification_timeout_s: float)`, `RadarWatchdogSnapshot.verified`, and `RadarWatchdogSnapshot.consecutive_good_frames`.
- Produces: `RadarSupervisorConfig.verification_frames == 30` and `RadarSupervisorConfig.verification_timeout_s == 5.0` by default; launcher arguments `--verify-frames` and `--verification-timeout` retain their existing names; the supervisor independently refuses a false `verified=True` snapshot carrying fewer than the configured frame count.

- [ ] **Step 1: Read the required TDD testing guidance**

Read `superpowers:test-driven-development` and its linked `writing-good-tests.md` completely before changing a test.

- [ ] **Step 2: Change watchdog tests to state the new probation contract**

Change the test helper and the old five-frame test to the following behavior:

```python
def make_watchdog(mission: Path, raw: Path) -> RadarEpochWatchdog:
    return RadarEpochWatchdog(
        mission_path=mission,
        raw_path=raw,
        expected=EXPECTED,
        started_at_s=0.0,
        first_frame_timeout_s=3.0,
        frame_timeout_s=2.5,
        required_consecutive_frames=30,
        verification_timeout_s=5.0,
    )


def test_five_frames_do_not_verify_but_thirty_consecutive_frames_do(self):
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        mission = root / "mission.jsonl"
        watchdog = make_watchdog(mission, root / "capture.bin")
        for log_seq in range(1, 6):
            append_radar_frame(
                mission,
                log_seq=log_seq,
                frame_number=100 + log_seq,
            )

        five_frames = watchdog.poll(0.5)
        for log_seq in range(6, 31):
            append_radar_frame(
                mission,
                log_seq=log_seq,
                frame_number=100 + log_seq,
            )
        thirty_frames = watchdog.poll(3.0)

        self.assertFalse(five_frames.verified)
        self.assertEqual(five_frames.consecutive_good_frames, 5)
        self.assertTrue(thirty_frames.verified)
        self.assertEqual(thirty_frames.consecutive_good_frames, 30)
```

Keep `test_each_nonqualifying_frame_resets_consecutive_evidence` intact. Update `test_verification_must_finish_before_deadline` to append only 29 qualifying frames and assert `radar_verification_timeout` at `5.0`.

- [ ] **Step 3: Make the startup fixture configurable without rewriting unrelated recovery tests**

Add `verification_frames: int = 5` to `SupervisorFixture.__init__`, build its default snapshot sequence from that value, and pass it into the fixture config:

```python
class SupervisorFixture:
    def __init__(
        self,
        directory: str,
        *,
        ports: list[list[object]] | None = None,
        reset_result: bool = True,
        profile_result: dict[str, object] | None = None,
        snapshots: list[RadarWatchdogSnapshot] | None = None,
        verification_frames: int = 5,
    ) -> None:
        self.watchdog = FakeWatchdog(
            snapshots
            or [
                RadarWatchdogSnapshot(False, count, 100.0, count, None)
                for count in range(1, verification_frames)
            ]
            + [
                RadarWatchdogSnapshot(
                    True,
                    verification_frames,
                    100.0,
                    verification_frames,
                    None,
                )
            ],
            self.actions,
        )
        self.config = RadarSupervisorConfig(
            repository_root=self.root,
            output_root=self.root / "output",
            profile_path=profile_path,
            calibration_path=self.root / "calibration.json",
            run_id="board-live",
            xds_serial="RI32",
            verification_frames=verification_frames,
)
```

Apply the signature change to the current fixture, replace only its current `self.watchdog = FakeWatchdog(...)` assignment with the shown assignment, and add only `verification_frames=verification_frames` to its current config constructor. Keep the shared `snapshot()` helper and `RecoveryFixture` at five frames so existing unit tests continue to exercise their own recovery behavior cheaply. Product defaults are verified separately by contract and launcher tests.

- [ ] **Step 4: Add supervisor-level tests for false and valid probation snapshots**

Add these tests to `RadarSupervisorStartupTests`:

```python
def test_five_frame_snapshot_does_not_pass_thirty_frame_probation(self) -> None:
    with tempfile.TemporaryDirectory() as directory:
        fixture = SupervisorFixture(
            directory,
            verification_frames=30,
            snapshots=[
                snapshot(verified=True, frames=5),
                snapshot(
                    verified=False,
                    frames=0,
                    fault="radar_verification_timeout",
                ),
            ],
        )
        stopped = False

        def stop_on_retry(delay_s: float) -> None:
            nonlocal stopped
            fixture.sleep(delay_s)
            stopped = True

        dependencies = replace(
            fixture.dependencies,
            sleep=stop_on_retry,
        )
        RadarSupervisor(fixture.config, dependencies).run(lambda: stopped)

        self.assertIsNone(fixture.processes.started_viewer)
        self.assertNotIn("viewer:e001", fixture.actions)


def test_viewer_switch_waits_for_thirty_consecutive_frames(self) -> None:
    with tempfile.TemporaryDirectory() as directory:
        fixture = SupervisorFixture(
            directory,
            verification_frames=30,
        )
        RadarSupervisor(fixture.config, fixture.dependencies).run(
            fixture.stop_when_running
        )

        self.assertEqual(fixture.processes.viewer_watchdog_poll_count, 30)
        self.assertLess(
            fixture.actions.index("verified:30"),
            fixture.actions.index("viewer:e001"),
        )
```

- [ ] **Step 5: Change default-contract assertions before production code**

In `RadarSupervisorContractTests.test_public_states_and_config_defaults_are_stable`, assert:

```python
self.assertEqual(config.verification_timeout_s, 5.0)
self.assertEqual(config.verification_frames, 30)
```

In `RadarStackLauncherTests.test_parser_defaults_and_help_work_outside_repository`, assert:

```python
self.assertEqual(args.verification_timeout, 5.0)
self.assertEqual(args.verify_frames, 30)
```

- [ ] **Step 6: Run the focused tests and observe RED**

Run:

```powershell
python -m unittest -v `
  tests.test_radar_watchdog.RadarEpochWatchdogTests.test_five_frames_do_not_verify_but_thirty_consecutive_frames_do `
  tests.test_radar_watchdog.RadarEpochWatchdogTests.test_each_nonqualifying_frame_resets_consecutive_evidence `
  tests.test_radar_watchdog.RadarEpochWatchdogTests.test_verification_must_finish_before_deadline `
  tests.test_radar_supervisor.RadarSupervisorContractTests.test_public_states_and_config_defaults_are_stable `
  tests.test_radar_supervisor.RadarSupervisorStartupTests.test_five_frame_snapshot_does_not_pass_thirty_frame_probation `
  tests.test_radar_supervisor.RadarSupervisorStartupTests.test_viewer_switch_waits_for_thirty_consecutive_frames `
  tests.test_radar_stack_processes.RadarStackLauncherTests.test_parser_defaults_and_help_work_outside_repository
```

Expected: default-contract tests fail because production still reports `3.0`/`5`, and the false-snapshot test fails because `_wait_until_verified` currently trusts `verified=True` without checking the count. The watchdog behavior tests pass because the watchdog already accepts explicit values.

- [ ] **Step 7: Implement the defaults and defensive supervisor gate**

In `RadarSupervisorConfig`, change:

```python
verification_timeout_s: float = 5.0
verification_frames: int = 30
```

In `build_parser()`, change:

```python
parser.add_argument(
    "--verification-timeout",
    type=_positive_float,
    default=5.0,
)
parser.add_argument("--verify-frames", type=_positive_int, default=30)
```

In `_wait_until_verified`, replace the bare verified check with:

```python
if (
    snapshot.verified is True
    and snapshot.consecutive_good_frames
    >= self._config.verification_frames
):
    return snapshot
```

Do not alter `_qualifies`, first-frame timeout, running timeout, or viewer thresholds.

- [ ] **Step 8: Run the probation tests and observe GREEN**

Run the command from Step 6.

Expected: `Ran 7 tests` and `OK`.

- [ ] **Step 9: Commit the independently working probation change**

```powershell
git add -- `
  sensors/radar_supervisor.py `
  scripts/run_radar_stack.py `
  tests/test_radar_watchdog.py `
  tests/test_radar_supervisor.py `
  tests/test_radar_stack_processes.py
git commit -m "fix: require stable radar recovery evidence"
```

Confirm `git status --short` shows only `?? runtime/`.

---

### Task 2: Apply bounded backoff after every faulted running epoch

**Files:**

- Modify: `tests/test_radar_supervisor.py:1636-2810`
- Modify: `sensors/radar_supervisor.py:419-475`
- Modify: `sensors/radar_supervisor.py:650-735`

**Interfaces:**

- Consumes: existing `_backoff(stop_requested: Callable[[], bool]) -> bool` and `_retry_delay_s`.
- Produces: every non-shutdown return from `_monitor_until_fault` is followed by `_backoff` before the next `_attempt_until_running`; successful promotion and same-epoch viewer restart preserve `_retry_delay_s`.

- [ ] **Step 1: Add a failing short-epoch backoff test**

Add to `RadarSupervisorRecoveryTests`:

```python
def test_short_running_epochs_increase_fault_backoff_to_cap(self) -> None:
    with tempfile.TemporaryDirectory() as directory:
        short_epoch = [
            snapshot(),
            snapshot(),
            snapshot(
                verified=False,
                frames=0,
                fault="radar_frame_timeout",
            ),
        ]
        fixture = RecoveryFixture(
            directory,
            watchdogs=[short_epoch for _ in range(5)] + [[snapshot()]],
        )

        RadarSupervisor(fixture.config, fixture.dependencies).run(
            lambda: self._stop_after_running_epoch(fixture, 6)
        )

        recovery_sleeps = [
            delay
            for delay in fixture.sleeps
            if delay != fixture.config.poll_interval_s
        ]
        self.assertEqual(recovery_sleeps[:5], [0.5, 1.0, 2.0, 4.0, 5.0])
        self.assertEqual(fixture.payload()["recovery_count"], 5)
```

- [ ] **Step 2: Add a failing shutdown-during-fault-backoff test**

Add:

```python
def test_shutdown_during_fault_backoff_does_not_reset_again(self) -> None:
    with tempfile.TemporaryDirectory() as directory:
        fixture = RecoveryFixture(
            directory,
            watchdogs=[[
                snapshot(),
                snapshot(),
                snapshot(
                    verified=False,
                    frames=0,
                    fault="radar_frame_timeout",
                ),
            ]],
        )
        stopped = False

        def stop_in_fault_backoff(delay_s: float) -> None:
            nonlocal stopped
            fixture.sleep(delay_s)
            if delay_s == fixture.config.retry_initial_s:
                stopped = True

        dependencies = replace(
            fixture.dependencies,
            sleep=stop_in_fault_backoff,
        )
        RadarSupervisor(fixture.config, dependencies).run(
            lambda: (
                stopped
                or fixture.actions.count("reset:COM3") >= 2
            )
        )

        self.assertEqual(fixture.actions.count("reset:COM3"), 1)
        self.assertEqual(fixture.payload()["recovery_count"], 1)
        self.assertEqual(fixture.payload()["state"], "STOPPED")
```

- [ ] **Step 3: Run the two tests and observe RED**

Run:

```powershell
python -m unittest -v `
  tests.test_radar_supervisor.RadarSupervisorRecoveryTests.test_short_running_epochs_increase_fault_backoff_to_cap `
  tests.test_radar_supervisor.RadarSupervisorRecoveryTests.test_shutdown_during_fault_backoff_does_not_reset_again
```

Expected: the first test reports no fault-recovery sleeps and the second reaches a second reset, because the current main run loop immediately begins the next attempt.

- [ ] **Step 4: Put the backoff between finalized fault and reset**

In `RadarSupervisor.run`, immediately after `_finalize_epoch(reason=reason)` and before `_attempt_until_running`, add:

```python
if not self._backoff(stop_requested):
    self._shutdown()
    return
```

Delete both immediate retry resets:

```python
self._retry_delay_s = self._config.retry_initial_s
```

The deleted lines are the one after `self._transition(SupervisorState.RUNNING, "verified_frames")` and the one after `self._transition(SupervisorState.RUNNING, "viewer_restarted")`.

- [ ] **Step 5: Update old recovery assertions for the new mandatory fault delay**

Make these three exact regression adjustments:

- In `test_recovery_attempt_failures_do_not_start_new_fault_episodes`, change the filtered sleep expectation from `[0.5]` to `[0.5, 1.0]`: the first value is the faulted-epoch delay and the second is the failed verification delay.
- In `test_capture_start_backoff_persists_latest_verified_flat_paths`, ignore the first non-poll sleep because it is now the faulted-running-epoch delay. Capture `backoff_payload` only on the second non-poll sleep, which is the `capture_start_failed` retry being asserted.
- In `test_same_epoch_viewer_retry_propagates_watchdog_fault`, let the first fault backoff complete and stop after the next manifest transition that the test is asserting; do not stop merely because `RECOVERING` appears before the reset.

Keep startup-only failure expectations unchanged.

- [ ] **Step 6: Run recovery tests and observe GREEN**

Run:

```powershell
python -m unittest -v tests.test_radar_supervisor.RadarSupervisorRecoveryTests
```

Expected: all `RadarSupervisorRecoveryTests` pass.

- [ ] **Step 7: Commit the independently working fault backoff**

```powershell
git add -- sensors/radar_supervisor.py tests/test_radar_supervisor.py
git commit -m "fix: back off faulted radar epochs"
```

Confirm `git status --short` shows only `?? runtime/`.

---

### Task 3: Reset retry delay only after 30 continuous healthy seconds

**Files:**

- Modify: `tests/test_radar_supervisor.py:51-160`
- Modify: `tests/test_radar_supervisor.py:1636-2810`
- Modify: `sensors/radar_supervisor.py:83-165`
- Modify: `sensors/radar_supervisor.py:390-735`

**Interfaces:**

- Consumes: injected `dependencies.monotonic()`, `RadarWatchdogSnapshot.fault_reason`, and retained `_retry_delay_s`.
- Produces: `RadarSupervisorConfig.stable_running_reset_s: float = 30.0` and a monitor-local `healthy_since_s: float`; retry delay resets to `retry_initial_s` only after a fault-free watchdog poll at or beyond the threshold.

- [ ] **Step 1: Add the configuration contract before production code**

Update the default assertion:

```python
self.assertEqual(config.stable_running_reset_s, 30.0)
```

Add `"stable_running_reset_s"` to `positive_float_fields`, so `0`, negatives, booleans, NaN, and infinities are rejected.

- [ ] **Step 2: Add a failing 30-second healthy-reset test**

Add to `RadarSupervisorRecoveryTests`:

```python
def test_thirty_healthy_seconds_reset_fault_backoff(self) -> None:
    with tempfile.TemporaryDirectory() as directory:
        fault = snapshot(
            verified=False,
            frames=0,
            fault="radar_frame_timeout",
        )
        fixture = RecoveryFixture(directory)
        healthy_polls = (
            int(30.0 / fixture.config.poll_interval_s)
            + 2
        )
        fixture.watchdogs = [
            [snapshot(), snapshot(), fault],
            [snapshot(), snapshot()]
            + [snapshot() for _ in range(healthy_polls)]
            + [fault],
            [snapshot(), snapshot()],
        ]

        RadarSupervisor(fixture.config, fixture.dependencies).run(
            lambda: self._stop_after_running_epoch(fixture, 3)
        )

        recovery_sleeps = [
            delay
            for delay in fixture.sleeps
            if delay != fixture.config.poll_interval_s
        ]
        self.assertEqual(recovery_sleeps, [0.5, 0.5])
        self.assertEqual(fixture.payload()["recovery_count"], 2)
```

The two extra polls avoid a binary floating-point boundary ambiguity while still proving the configured 30.0-second threshold.

- [ ] **Step 3: Run the focused tests and observe RED**

Run:

```powershell
python -m unittest -v `
  tests.test_radar_supervisor.RadarSupervisorContractTests.test_public_states_and_config_defaults_are_stable `
  tests.test_radar_supervisor.RadarSupervisorContractTests.test_config_rejects_invalid_numeric_and_identifier_values `
  tests.test_radar_supervisor.RadarSupervisorRecoveryTests.test_thirty_healthy_seconds_reset_fault_backoff
```

Expected: constructor/default tests fail because the field is absent, and the recovery test sees `[0.5, 1.0]`.

- [ ] **Step 4: Add the validated 30-second configuration**

In `RadarSupervisorConfig`, add:

```python
stable_running_reset_s: float = 30.0
```

Add `"stable_running_reset_s"` to the tuple validated by `_require_positive_float`.

- [ ] **Step 5: Start one local health interval per running epoch**

In `_monitor_until_fault`, immediately after the existing active-child/watchdog invariant check, add:

```python
healthy_since_s = self._dependencies.monotonic()
```

The local survives a same-epoch viewer restart because that restart stays inside the same `_monitor_until_fault` invocation. A radar fault returns from the method, so the next verified epoch automatically receives a new interval without mutable cross-epoch timer state.

- [ ] **Step 6: Reset retry delay only after fresh evidence proves the interval**

In `_monitor_until_fault`, obtain monotonic time once for the watchdog poll:

```python
now_s = self._dependencies.monotonic()
snapshot = self._active_watchdog.poll(now_s)
```

After checking `snapshot.fault_reason is None`, add:

```python
if (
    now_s - healthy_since_s
    >= self._config.stable_running_reset_s
):
    self._retry_delay_s = self._config.retry_initial_s
```

This check must remain after the fault check so a faulting poll at the boundary cannot count as healthy.

- [ ] **Step 7: Run the focused tests and observe GREEN**

Run the command from Step 3.

Expected: `Ran 3 tests` and `OK`.

- [ ] **Step 8: Run all supervisor tests**

Run:

```powershell
python -m unittest -v `
  tests.test_radar_watchdog `
  tests.test_radar_supervisor `
  tests.test_radar_supervisor_integration
```

Expected: all tests pass with no failure or error.

- [ ] **Step 9: Commit the stability-aware reset**

```powershell
git add -- sensors/radar_supervisor.py tests/test_radar_supervisor.py
git commit -m "fix: reset radar retries after stable running"
```

Confirm `git status --short` shows only `?? runtime/`.

---

### Task 4: Prove radar-only regression safety before touching firmware

**Files:**

- Verify only: `sensors/`
- Verify only: `monitor/radar_front.py`
- Verify only: `scripts/run_radar_stack.py`
- Verify only: `tests/test_radar_*.py`
- Preserve: `runtime/`

**Interfaces:**

- Consumes: all behavior produced by Tasks 1-3 and the existing process manager from commit `6daef7e`.
- Produces: a clean automated baseline showing probation, fail-closed UI, immutable epochs, and exact child ownership are intact before hardware mutation.

- [ ] **Step 1: Run ownership mutation tests**

Run:

```powershell
python -m unittest -v `
  tests.test_radar_stack_processes.RadarStackProcessesTests.test_manager_rejects_same_wrapper_with_replaced_process_before_signal `
  tests.test_radar_stack_processes.RadarStackProcessesTests.test_duplicate_pid_registration_stops_new_process_before_raising `
  tests.test_radar_stack_processes.RadarStackProcessesTests.test_owned_shutdown_reports_failure_after_stopping_later_children
```

Expected: `Ran 3 tests` and `OK`.

- [ ] **Step 2: Run fail-closed viewer tests**

Run:

```powershell
python -m unittest -v `
  tests.test_radar_front.RadarFrontStateTests.test_live_stale_and_fault_transitions_use_viewer_clock `
  tests.test_radar_front.RadarFrontHttpTests.test_non_running_supervisor_state_blocks_the_very_next_api_response
```

Expected: `Ran 2 tests` and `OK`.

- [ ] **Step 3: Run the complete repository suite before hardware mutation**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: exit code 0, with no failure or error.

- [ ] **Step 4: Inspect scope and immutable evidence**

Run:

```powershell
git status --short
git diff --check
git diff HEAD~3 --name-only
```

Expected:

- `git diff --check` prints nothing.
- Changed production paths are limited to `sensors/radar_supervisor.py` and `scripts/run_radar_stack.py`.
- Changed tests are limited to radar test modules.
- `runtime/` remains untracked and unstaged.
- No profile, calibration, collision, scene, or non-radar file changed.

---

### Task 5: Stop the exact radar stack and update RI32 XDS110 firmware

**Files:**

- Read: `runtime/radar-acceptance-current.json`
- Read: the manifest path named by that pointer
- Read: `C:\ti\uniflash_9.6.0\deskdb\content\TICloudAgent\win\ccs_base\common\uscif\xds110\firmware_3.0.0.43.bin`
- Generate by TI tool only: XDS110 device firmware
- Preserve: all existing manifest, mission, capture, and runtime files

**Interfaces:**

- Consumes: pointer fields `supervisor_pid`, `run_id`, `repository_root`, and `manifest_path`; XDS110 runtime identity `RI32`.
- Produces: exactly one runtime XDS110 reporting serial `RI32`, mode `Runtime`, and version `3.0.0.43`, with COM3 and COM4 present before stack restart.

- [ ] **Step 1: Re-read TI's official update procedure**

Use the installed tool help and the official TI XDS110 update page:

- `https://software-dl.ti.com/ccs/esd/documents/xdsdebugprobes/emu_xds110.html`
- `https://www.ti.com/lit/pdf/koku001`

Confirm the required order remains separate invocations: enumerate, `-m`, wait for DFU enumeration, `-f <image> -r`, then verify runtime enumeration.

- [ ] **Step 2: Resolve and verify the exact running supervisor**

Run:

```powershell
$repo = 'C:\Users\minho\Documents\Codex\2026-07-25\f\HANSEL_MESH_RADAR_RECOVERY'
$pointerPath = Join-Path $repo 'runtime\radar-acceptance-current.json'
$pointer = Get-Content -LiteralPath $pointerPath -Raw | ConvertFrom-Json
$supervisor = Get-CimInstance Win32_Process -Filter "ProcessId=$($pointer.supervisor_pid)"

if ($null -eq $supervisor) {
  throw "Recorded supervisor PID is not running"
}
if ($pointer.repository_root -ne $repo) {
  throw "Pointer repository does not match the approved radar worktree"
}
if (
  $supervisor.CommandLine -notlike '*scripts\run_radar_stack.py*' -or
  $supervisor.CommandLine -notlike "*--run-id $($pointer.run_id)*" -or
  $supervisor.CommandLine -notlike "*$repo*"
) {
  throw "PID does not match the recorded radar supervisor command"
}

$manifestBefore = Get-Content -LiteralPath $pointer.manifest_path -Raw |
  ConvertFrom-Json
if ($manifestBefore.run_id -ne $pointer.run_id) {
  throw "Manifest run ID does not match the pointer"
}
```

Any thrown condition ends the firmware task without stopping or flashing.

- [ ] **Step 3: Record the exact hardware preflight**

Run:

```powershell
$xdsDir = 'C:\ti\uniflash_9.6.0\deskdb\content\TICloudAgent\win\ccs_base\common\uscif\xds110'
$xdsDfu = Join-Path $xdsDir 'xdsdfu.exe'
$firmware = Join-Path $xdsDir 'firmware_3.0.0.43.bin'

$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $firmware).Hash
if ($hash -ne 'F00B865F0F179F5B0499E4A05894ECA6C5DC71A0A8C4D741B9F731DF47A65658') {
  throw "Firmware image hash mismatch"
}

$runtimeEnumeration = (& $xdsDfu -e 2>&1 | Out-String)
if (
  $runtimeEnumeration -notmatch 'Found 1 device' -or
  $runtimeEnumeration -notmatch 'Serial Num:\s+RI32' -or
  $runtimeEnumeration -notmatch 'Mode:\s+Runtime' -or
  $runtimeEnumeration -notmatch 'Version:\s+3\.0\.0\.13'
) {
  throw "Expected one RI32 runtime probe at firmware 3.0.0.13"
}

$comDevices = @(
  Get-CimInstance Win32_PnPEntity |
    Where-Object {
      $_.Name -match '^XDS110 Class (Application/User UART|Auxiliary Data Port)\(COM[34]\)$' -and
      $_.DeviceID -match '^USB\\VID_0451&PID_BEF3'
    }
)
if ($comDevices.Count -ne 2) {
  throw "Expected the RI32 XDS110 COM3/COM4 pair"
}
```

Do not continue if the version is already different; report the observed version and reassess instead of assuming an upgrade path.

- [ ] **Step 4: Stop only the verified supervisor and its manifest-owned children**

First snapshot the currently active child PIDs from the final `process_events` state. Stop the verified supervisor PID:

```powershell
$activeByPid = @{}
foreach ($event in $manifestBefore.process_events) {
  if ($event.action -eq 'started') {
    $activeByPid[[int]$event.pid] = [string]$event.role
  } elseif ($event.action -eq 'stopped') {
    $activeByPid.Remove([int]$event.pid)
  }
}

Stop-Process -Id ([int]$pointer.supervisor_pid) -ErrorAction Stop
$deadline = (Get-Date).AddSeconds(15)
do {
  $supervisorAlive = $null -ne (
    Get-Process -Id ([int]$pointer.supervisor_pid) -ErrorAction SilentlyContinue
  )
  $ownedAlive = @(
    foreach ($pidValue in $activeByPid.Keys) {
      Get-CimInstance Win32_Process -Filter "ProcessId=$pidValue"
    }
  )
  if (-not $supervisorAlive -and $ownedAlive.Count -eq 0) {
    break
  }
  Start-Sleep -Milliseconds 250
} while ((Get-Date) -lt $deadline)

if ($supervisorAlive) {
  throw "Verified radar supervisor did not stop"
}
if ($ownedAlive.Count -ne 0) {
  foreach ($process in $ownedAlive) {
    $role = $activeByPid[[int]$process.ProcessId]
    $isOwnedCommand = (
      ($role -eq 'capture' -and $process.CommandLine -like '*-m sensors radar-live*') -or
      ($role -eq 'viewer' -and $process.CommandLine -like '*monitor/radar_front.py*')
    )
    if (-not $isOwnedCommand) {
      throw "An active PID no longer matches its manifest-owned radar role"
    }
  }
  foreach ($process in $ownedAlive) {
    Stop-Process -Id ([int]$process.ProcessId) -ErrorAction Stop
  }
}
```

Re-run the final `Get-Process`/CIM checks and require the supervisor and all recorded active child PIDs to be absent. Do not stop other Python processes.

- [ ] **Step 5: Re-enumerate after process release**

Run `& $xdsDfu -e` again and require exactly one `RI32`, `Runtime`, `3.0.0.13` device. If enumeration is zero, ambiguous, or different, stop without issuing `-m`.

- [ ] **Step 6: Enter DFU mode as a separate operation**

Run:

```powershell
& $xdsDfu -m
if ($LASTEXITCODE -ne 0) {
  throw "xdsdfu -m failed with exit code $LASTEXITCODE"
}
```

Then poll for up to 30 seconds:

```powershell
$deadline = (Get-Date).AddSeconds(30)
$dfuEnumeration = ''
do {
  Start-Sleep -Milliseconds 500
  $dfuEnumeration = (& $xdsDfu -e 2>&1 | Out-String)
  if (
    $dfuEnumeration -match 'Found 1 device' -and
    $dfuEnumeration -match 'Mode:\s+DFU'
  ) {
    break
  }
} while ((Get-Date) -lt $deadline)
if (
  $dfuEnumeration -notmatch 'Found 1 device' -or
  $dfuEnumeration -notmatch 'Mode:\s+DFU'
) {
  throw "The single preflight-identified probe did not re-enumerate in DFU mode"
}
```

- [ ] **Step 7: Flash the exact image and reset**

Run:

```powershell
& $xdsDfu -f $firmware -r
if ($LASTEXITCODE -ne 0) {
  throw "XDS110 firmware download/reset failed with exit code $LASTEXITCODE"
}
```

- [ ] **Step 8: Verify runtime version, serial, and COM ports**

Poll `& $xdsDfu -e` for up to 30 seconds and require all four expressions:

```text
Found 1 device
Serial Num:    RI32
Mode:          Runtime
Version:       3.0.0.43
```

Then poll CIM for up to 30 seconds and require:

```text
XDS110 Class Application/User UART(COM3)
XDS110 Class Auxiliary Data Port(COM4)
USB\VID_0451&PID_BEF3\RI32
```

The update is not accepted from exit code alone.

---

### Task 6: Restart with a new run ID and complete a 15-minute hardware soak

**Files:**

- Execute: `scripts/run_radar_stack.py`
- Read: `configs/radar/iwrl6432_3d_operator_near_10hz.cfg`
- Read: `C:\Users\minho\AppData\Local\Temp\hansel-r9-fixture-calibration.json`
- Generate: new untracked manifest, epoch missions, captures, and process logs
- Preserve: all prior run IDs and artifacts

**Interfaces:**

- Consumes: XDS110 `RI32` at firmware 3.0.0.43 and the code defaults produced by Tasks 1-3.
- Produces: a new immutable run ID whose same epoch stays `RUNNING` for at least 900 seconds with near-10-Hz increasing frames, zero parser errors, zero writer drops, no recovery, COM3/COM4 present, and a stable browser view.

- [ ] **Step 1: Reconfirm profile and calibration inputs**

Run:

```powershell
$repo = 'C:\Users\minho\Documents\Codex\2026-07-25\f\HANSEL_MESH_RADAR_RECOVERY'
$profile = Join-Path $repo 'configs\radar\iwrl6432_3d_operator_near_10hz.cfg'
$calibration = 'C:\Users\minho\AppData\Local\Temp\hansel-r9-fixture-calibration.json'
if (-not (Test-Path -LiteralPath $profile -PathType Leaf)) {
  throw "Approved 10 Hz radar profile is missing"
}
if (-not (Test-Path -LiteralPath $calibration -PathType Leaf)) {
  throw "Existing clutter calibration is missing"
}
```

Confirm the calibration SHA-256 remains `A674FD5E58E3EF39C9020B8F68F1DB82C4A85BA458DF977BBC9C2487619EF8B7`.

- [ ] **Step 2: Start one hidden supervisor with a new immutable run ID**

Run:

```powershell
$python = 'C:\Users\minho\AppData\Local\Programs\Python\Python312\python.exe'
$runId = (Get-Date).ToUniversalTime().ToString('yyyyMMdd-HHmmssfff')
$supervisorOut = Join-Path $repo "runtime\radar-supervisor-$runId.stdout.log"
$supervisorErr = Join-Path $repo "runtime\radar-supervisor-$runId.stderr.log"
$arguments = @(
  'scripts\run_radar_stack.py',
  '--run-id', $runId,
  '--output-root', $repo,
  '--xds-serial', 'RI32',
  '--cfg', $profile,
  '--clutter-calibration', $calibration
)
$stackProcess = Start-Process `
  -FilePath $python `
  -ArgumentList $arguments `
  -WorkingDirectory $repo `
  -RedirectStandardOutput $supervisorOut `
  -RedirectStandardError $supervisorErr `
  -WindowStyle Hidden `
  -PassThru
$manifestPath = Join-Path $repo "runtime\radar-supervisor-$runId.json"
```

Do not reuse the old run ID.

- [ ] **Step 3: Require 30-frame probation before accepting `RUNNING`**

Poll the new manifest for up to 45 seconds. Accept startup only when:

```powershell
$manifest.state -eq 'RUNNING'
$manifest.epoch -eq 1
$manifest.recovery_count -eq 0
$manifest.verified_consecutive_frames -ge 30
$manifest.xds_serial -eq 'RI32'
```

Also require `http://127.0.0.1:8081/api/radar` to return:

```powershell
$api.status -eq 'live'
$api.supervisor_state -eq 'RUNNING'
$api.frame.complete -eq $true
$api.frame.heatmap.range_bins -eq 128
$api.frame.heatmap.azimuth_bins -eq 16
$api.counters.parse_errors_total -eq 0
$api.counters.writer_drops_total -eq 0
```

If epoch 1 faults before acceptance, the code is behaving fail-closed; record the failure and do not relabel a later recovery as an epoch-1 pass.

- [ ] **Step 4: Open or reload the operator browser**

Use the `browser:control-in-app-browser` skill, navigate the existing in-app browser tab to `http://127.0.0.1:8081/`, and verify visually that the page leaves `재연결 중` only after the manifest has passed the 30-frame probation. Do not use the browser state as a substitute for API/manifest evidence.

- [ ] **Step 5: Observe the same epoch for 900 seconds**

At five-second intervals, collect:

```powershell
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
$api = Invoke-RestMethod -Uri 'http://127.0.0.1:8081/api/radar' -TimeoutSec 3
$ports = @(
  Get-CimInstance Win32_PnPEntity |
    Where-Object {
      $_.Name -match '^XDS110 Class (Application/User UART|Auxiliary Data Port)\(COM[34]\)$'
    }
)
```

For all 180 samples require:

```text
manifest.state == RUNNING
manifest.epoch == 1
manifest.recovery_count == 0
api.status == live
api.supervisor_state == RUNNING
api.age_ms < 750
api.frame.complete == true
api.counters.parse_errors_total == 0
api.counters.writer_drops_total == 0
COM3 and COM4 both present
```

Record the first and last API frame numbers and elapsed monotonic time. Require strictly increasing frame numbers and an observed rate between 8.0 and 12.0 frames per second over the full interval. Keep each wait at or below 60 seconds so progress can be reported during the soak.

- [ ] **Step 6: Perform final acceptance checks**

After at least 900 seconds, require:

- `xdsdfu -e` reports exactly one `RI32`, `Runtime`, `3.0.0.43` device.
- The manifest still reports epoch 1, `RUNNING`, recovery count 0, and no epoch end reason.
- API counters report parser errors 0, writer drops 0, and no frame-gap increase.
- The supervisor PID and its manifest-owned capture/viewer PIDs are running.
- COM3 and COM4 are present.
- The browser has not returned to the reconnect loop.
- `git status --short` still shows generated `runtime/`, mission, and capture evidence as untracked rather than staged.

- [ ] **Step 7: Run verification-before-completion**

Read and apply `superpowers:verification-before-completion`. Re-run:

```powershell
python -m unittest discover -s tests -v
git diff --check
git status --short
git log -6 --oneline
git rev-parse HEAD
git rev-parse origin/codex/radar-auto-recovery
```

Do not claim success unless the fresh test output and 15-minute hardware evidence satisfy every acceptance condition.

- [ ] **Step 8: Push the radar-only branch**

The design plan commit and implementation commits must be pushed, while `runtime/`, missions, and captures remain untracked:

```powershell
git push origin codex/radar-auto-recovery
```

Verify:

```powershell
git rev-parse HEAD
git ls-remote --heads origin codex/radar-auto-recovery
git status --short
```

The local HEAD and remote branch hash must match. Report a failed 15-minute soak honestly and do not execute the lower-bandwidth fallback without a separate implementation cycle.
