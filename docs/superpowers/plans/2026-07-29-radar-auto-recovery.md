# IWRL6432 Radar Automatic Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the 10 Hz IWRL6432BOOST operator view fail-closed and automatically recover configuration, capture, and display after firmware stalls or XDS110 USB reconnects.

**Architecture:** Keep each `radar-live` process and its mission/raw files as one immutable device epoch. Add a supervisor that identifies the Application/User UART, resets the exact XDS110 target, applies the pinned profile, verifies five complete heatmap frames, then switches the viewer to that epoch. Recovery is triggered by real `RadarFrame` freshness or a known firmware error, never by process existence or mission-file growth alone.

**Tech Stack:** Python 3.12, `unittest`, pyserial 3.5, subprocess/Windows process groups, TI MMWAVE-L-SDK 05.05.04.02, TI `xds110reset.exe`, existing mission-log contracts, vanilla JavaScript/Node test runner.

## Global Constraints

- Active hardware is IWRL6432BOOST through XDS110 Application/User UART, currently VID:PID `0451:BEF3`, serial `RI32`, interface `.0`.
- Never select the XDS110 Auxiliary Data Port, currently interface `.3`/COM4.
- Only one process may own the radar CLI/data UART at any instant.
- Keep `frameCfg 2 8 600 16 100 0`, 16 azimuth bins, 128 range bins, `0.09765625 m` range step, elevation FFT 8, CFAR 15 dB, and 10 Hz.
- Change only `configs/radar/iwrl6432_3d_operator_near_10hz.cfg` from `lowPowerCfg 1` to `lowPowerCfg 0`.
- First-frame deadline is 3.0 seconds; running frame timeout is 2.5 seconds; recovery requires five consecutive complete frames with the configured heatmap within 3.0 seconds.
- Retry delay starts at 0.5 seconds and is capped at 5.0 seconds; retries continue until explicit shutdown.
- All non-running states are movement-blocking and must not expose old scene evidence as current.
- Every recovery creates new mission/raw/index artifacts; never pass `--overwrite`.
- The viewer remains on `127.0.0.1:8081`; collision distance, map geometry, calibration contract, and pose contract do not change.
- Preserve the user's unrelated dirty files: `LICENSE`, `cripts`, `docs/HANSEL_MESH_presentation_code_guide.pdf`, `monitor_session.jsonl`, and `video_quality.jsonl`.

---

## File Structure

### New files

- `sensors/ti_radar_control.py` — profile parsing/application, XDS110 UART identity selection, reset executable discovery, and target reset.
- `sensors/radar_watchdog.py` — incremental mission/raw evidence tailing, firmware fault detection, freshness timeout, and five-frame verification gate.
- `sensors/radar_supervisor.py` — recovery state machine, epoch allocation/manifest, retries, and owned-child lifecycle orchestration.
- `sensors/radar_stack_processes.py` — exact capture/viewer subprocess commands and cross-platform graceful shutdown.
- `scripts/run_radar_stack.py` — thin production command-line entry point.
- `tests/test_ti_radar_control.py` — port selection, reset scoping, and reusable profile-control tests.
- `tests/test_radar_watchdog.py` — health-only stall, partial line, firmware marker, timeout, and verification tests.
- `tests/test_radar_supervisor.py` — deterministic state-machine, retry, artifact, and recovery tests.
- `tests/test_radar_supervisor_integration.py` — real lightweight child-process and scripted disconnect/reconnect integration.
- `tests/test_radar_stack_processes.py` — exact command construction and owned-process termination tests.
- `tests/web/radar_panel.test.js` — browser-VM fail-closed rendering and
  operator-state tests.
- `docs/radar_auto_recovery.md` — operator runbook for one-command startup and recovery states.

### Modified files

- `configs/radar/iwrl6432_3d_operator_near_10hz.cfg` — disable low power at 10 Hz.
- `scripts/configure_ti_radar.py` — remain a compatible wrapper around `sensors.ti_radar_control`.
- `sensors/cli.py` — translate managed termination signals into a graceful radar capture stop.
- `tests/test_sensor_cli.py` — signal-handler and radar-live shutdown contract.
- `tests/test_configure_ti_radar.py` — profile regression and wrapper compatibility.
- `monitor/radar_front.py` — bump the served UI contract/build identifier.
- `monitor/web/radar_front.html` — keep cache-busting asset identifiers aligned.
- `monitor/web/radar_panel.js` — explicit reconnecting/stop copy for every
  movement-blocking live-input state.
- `tests/test_radar_front.py` — served asset contract for the reconnecting/stop copy.
- `docs/radar_front_view.md` — link the new supervised startup runbook.

---

### Task 1: Remove the Confirmed 10 Hz Low-Power Assert

**Files:**

- Modify: `configs/radar/iwrl6432_3d_operator_near_10hz.cfg:30`
- Modify: `tests/test_configure_ti_radar.py`

**Interfaces:**

- Consumes: existing `load_commands(Path)` and `partition_at_baud(Sequence[str])`.
- Produces: the same profile identifier and geometry with `lowPowerCfg 0`.

- [ ] **Step 1: Strengthen the profile regression test**

Add the following assertions to
`ConfigureTiRadarTest.test_near_3d_profile_starts_at_first_practical_range_bin`:

```python
self.assertIn("lowPowerCfg 0", profile.before_baud)
self.assertNotIn("lowPowerCfg 1", profile.before_baud)
self.assertIn(
    "frameCfg 2 8 600 16 100 0",
    profile.before_baud,
)
```

- [ ] **Step 2: Run the focused test and confirm the expected failure**

Run:

```powershell
python -m unittest `
  tests.test_configure_ti_radar.ConfigureTiRadarTest.test_near_3d_profile_starts_at_first_practical_range_bin `
  -v
```

Expected: FAIL because the active profile still contains `lowPowerCfg 1`.

- [ ] **Step 3: Make the one-line profile change**

Change:

```text
lowPowerCfg 1
```

to:

```text
lowPowerCfg 0
```

Update the adjacent profile comment to state that 10 Hz heatmap output disables
the low-power framework because the TI firmware otherwise asserts when frame
idle time is insufficient.

- [ ] **Step 4: Verify the focused profile and dry-run configuration**

Run:

```powershell
python -m unittest tests.test_configure_ti_radar -v
python scripts\configure_ti_radar.py `
  --port COM3 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --dry-run
```

Expected: all configuration tests PASS; dry-run reports 25 commands,
`target_baud` 1250000, and `sensorStart 0 0 0 0` after the baud switch.

- [ ] **Step 5: Commit the prevention change**

```powershell
git add -- `
  configs/radar/iwrl6432_3d_operator_near_10hz.cfg `
  tests/test_configure_ti_radar.py
git commit -m "fix: prevent radar low power timing assert"
```

---

### Task 2: Extract Reusable TI Profile Control Without Changing CLI Behavior

**Files:**

- Create: `sensors/ti_radar_control.py`
- Modify: `scripts/configure_ti_radar.py`
- Create: `tests/test_ti_radar_control.py`
- Modify: `tests/test_configure_ti_radar.py`

**Interfaces:**

- Consumes: pyserial and the current profile file format.
- Produces:
  - `ProfileCommands`
  - `load_commands(path: Path) -> tuple[str, ...]`
  - `partition_at_baud(commands: Sequence[str]) -> ProfileCommands`
  - `apply_profile(port: str, profile: ProfileCommands, initial_baud: int, command_timeout_s: float, reopen_delay_s: float) -> dict[str, object]`
  - `validate_profile_result(result: Mapping[str, object], expected_commands: int, require_first_magic: bool = True) -> None`

- [ ] **Step 1: Write import-compatibility and strict-result tests**

Create `tests/test_ti_radar_control.py` with tests shaped as follows:

```python
from pathlib import Path
import unittest

from sensors.ti_radar_control import (
    load_commands,
    partition_at_baud,
    validate_profile_result,
)


class TiRadarControlTest(unittest.TestCase):
    def test_active_profile_partitions_at_1250000_baud(self):
        path = (
            Path(__file__).resolve().parent.parent
            / "configs"
            / "radar"
            / "iwrl6432_3d_operator_near_10hz.cfg"
        )
        profile = partition_at_baud(load_commands(path))
        self.assertEqual(profile.target_baud, 1_250_000)
        self.assertEqual(profile.after_baud, ("sensorStart 0 0 0 0",))

    def test_profile_result_requires_every_command_and_start_evidence(self):
        with self.assertRaisesRegex(RuntimeError, "first radar frame"):
            validate_profile_result(
                {
                    "commands_completed": 25,
                    "new_baud_prompt_observed": True,
                    "first_magic_observed": False,
                },
                expected_commands=25,
            )
```

Update `tests/test_configure_ti_radar.py` to assert that its dynamically loaded
wrapper exposes the same `ProfileCommands`, `load_commands`,
`partition_at_baud`, and `apply_profile` objects as
`sensors.ti_radar_control`. Add a subprocess regression that executes the
script's `--help` from a working directory outside the repository so the
wrapper's import bootstrap is exercised.

- [ ] **Step 2: Run the tests and confirm the module is missing**

Run:

```powershell
python -m unittest `
  tests.test_ti_radar_control `
  tests.test_configure_ti_radar `
  -v
```

Expected: ERROR importing `sensors.ti_radar_control`.

- [ ] **Step 3: Move the existing implementation into the sensor module**

Move the existing constants, dataclass, and functions from
`scripts/configure_ti_radar.py` into `sensors/ti_radar_control.py` without
changing their serial protocol:

```python
TI_MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


@dataclass(frozen=True)
class ProfileCommands:
    before_baud: tuple[str, ...]
    baud_command: str
    target_baud: int
    after_baud: tuple[str, ...]
```

Keep `_read_until`, `_command_failed`, and `_send_command` private. Add strict
validation:

```python
def validate_profile_result(
    result: Mapping[str, object],
    expected_commands: int,
    require_first_magic: bool = True,
) -> None:
    if result.get("commands_completed") != expected_commands:
        raise RuntimeError("not every radar profile command completed")
    if result.get("new_baud_prompt_observed") is not True:
        raise RuntimeError("new radar baud prompt was not observed")
    if (
        require_first_magic
        and result.get("first_magic_observed") is not True
    ):
        raise RuntimeError("first radar frame magic was not observed")
```

The production supervisor calls this with `require_first_magic=False` and then
uses its own three-second complete-frame verification. The standalone
configurator keeps the default `True` and reports missing start evidence.

- [ ] **Step 4: Make the existing script a thin compatible wrapper**

At the top of `scripts/configure_ti_radar.py`, import and re-export:

```python
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from sensors.ti_radar_control import (
    ProfileCommands,
    apply_profile,
    load_commands,
    partition_at_baud,
    validate_profile_result,
)
```

Keep `build_parser()` and `main()` in the script. In non-dry-run mode, call
`validate_profile_result(result, expected_commands=len(commands))` before
printing success JSON.

- [ ] **Step 5: Run focused and full configuration tests**

Run:

```powershell
python -m unittest `
  tests.test_ti_radar_control `
  tests.test_configure_ti_radar `
  -v
python scripts\configure_ti_radar.py `
  --port COM3 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --dry-run
```

Expected: PASS; the dry-run JSON contract remains unchanged.

- [ ] **Step 6: Commit the reusable control module**

```powershell
git add -- `
  sensors/ti_radar_control.py `
  scripts/configure_ti_radar.py `
  tests/test_ti_radar_control.py `
  tests/test_configure_ti_radar.py
git commit -m "refactor: expose reusable TI radar profile control"
```

---

### Task 3: Select the Correct XDS110 Port and Reset Only the Intended Board

**Files:**

- Modify: `sensors/ti_radar_control.py`
- Modify: `tests/test_ti_radar_control.py`

**Interfaces:**

- Consumes: objects returned by `serial.tools.list_ports.comports()` and an
  injectable `subprocess.run` compatible callable.
- Produces:
  - `RadarPortIdentity`
  - `select_application_port(ports: Iterable[object], explicit_port: str | None = None, xds_serial: str | None = None) -> RadarPortIdentity`
  - `find_xds110_reset(explicit: Path | None, search_roots: Iterable[Path]) -> Path`
  - `reset_xds110_target(executable: Path, serial_number: str, runner: Callable[..., object]) -> None`

- [ ] **Step 1: Write port-selection tests**

Use `SimpleNamespace` objects to model COM3 and COM4:

```python
application = SimpleNamespace(
    device="COM3",
    vid=0x0451,
    pid=0xBEF3,
    serial_number="RI32",
    description="XDS110 Class Application/User UART(COM3)",
    location="1-3:x.0",
)
auxiliary = SimpleNamespace(
    device="COM4",
    vid=0x0451,
    pid=0xBEF3,
    serial_number="RI32",
    description="XDS110 Class Auxiliary Data Port(COM4)",
    location="1-3:x.3",
)
```

Required assertions:

```python
selected = select_application_port(
    [auxiliary, application],
    xds_serial="RI32",
)
self.assertEqual(selected.device, "COM3")

renumbered = SimpleNamespace(**{**application.__dict__, "device": "COM9"})
self.assertEqual(
    select_application_port([renumbered], xds_serial="RI32").device,
    "COM9",
)

with self.assertRaisesRegex(RuntimeError, "Application/User"):
    select_application_port([auxiliary], xds_serial="RI32")
```

Also test an ambiguous pair, an explicit nonexistent port, and a mismatched
serial number.

- [ ] **Step 2: Write reset discovery and exact-command tests**

Test that an explicit existing executable wins, that UniFlash wildcard search
chooses the newest semantic version, and that a missing tool raises a
descriptive error. Capture the reset runner arguments and assert:

```python
self.assertEqual(
    command,
    [
        str(reset_executable),
        "-a",
        "toggle",
        "-d",
        "100",
        "-s",
        "RI32",
    ],
)
```

Assert that an empty serial number is rejected before invoking the runner.

- [ ] **Step 3: Run the focused tests and confirm missing interfaces**

Run:

```powershell
python -m unittest tests.test_ti_radar_control -v
```

Expected: import or attribute failures for the new interfaces.

- [ ] **Step 4: Implement identity selection**

Add:

```python
@dataclass(frozen=True)
class RadarPortIdentity:
    device: str
    vid: int
    pid: int
    serial_number: str
    description: str
    location: str
```

Filter to VID/PID `0x0451/0xBEF3`, require description containing
`Application/User UART`, exclude descriptions containing `Auxiliary`, apply
the optional serial filter, then require exactly one match. The `.0` location
is a validation/tie-break signal, never a reason to select `.3`.

- [ ] **Step 5: Implement reset executable discovery and scoped reset**

Search in this order:

1. explicit path;
2. `xds110reset.exe`/`xds110reset` on `PATH`;
3. `C:\ti\uniflash_*\deskdb\content\TICloudAgent\win\ccs_base\common\uscif\xds110\xds110reset.exe`;
4. `C:\ti\uniflash_*\simplelink\imagecreator\bin\xds110reset.exe`.

Invoke the exact serial-scoped command with `check=True`,
`capture_output=True`, and `text=True`. Include stderr in the raised
`RuntimeError` while never logging unrelated environment variables.

- [ ] **Step 6: Verify against fakes and inspect the real port inventory**

Run:

```powershell
python -m unittest tests.test_ti_radar_control -v
python -m serial.tools.list_ports -v
```

Expected: tests PASS; real inventory shows Application/User COM3 and Auxiliary
COM4 with serial `RI32`.

- [ ] **Step 7: Commit identity and reset control**

```powershell
git add -- `
  sensors/ti_radar_control.py `
  tests/test_ti_radar_control.py
git commit -m "feat: identify and reset the selected XDS110 radar"
```

---

### Task 4: Build a Real-Radar-Frame Watchdog

**Files:**

- Create: `sensors/radar_watchdog.py`
- Create: `tests/test_radar_watchdog.py`

**Interfaces:**

- Consumes: one growing mission JSONL path, one growing raw binary path, the
  existing `decode_log_entry`, and an injected monotonic time.
- Produces:
  - `ExpectedRadarEvidence`
  - `RadarWatchdogSnapshot`
  - `RadarEpochWatchdog.poll(now_s: float) -> RadarWatchdogSnapshot`

- [ ] **Step 1: Write mission-tail and health-only-stall tests**

Create test helpers that append complete encoded mission entries and raw
bytes. The central regression must prove that health records do not refresh
the radar:

```python
watchdog = RadarEpochWatchdog(
    mission_path=mission,
    raw_path=raw,
    expected=expected,
    started_at_s=0.0,
    first_frame_timeout_s=3.0,
    frame_timeout_s=2.5,
    required_consecutive_frames=5,
    verification_timeout_s=3.0,
)

append_radar_frame(mission, log_seq=1, frame_number=10)
self.assertIsNone(watchdog.poll(0.1).fault_reason)
append_sensor_health(mission, log_seq=2, status="ok")
self.assertEqual(
    watchdog.poll(2.7).fault_reason,
    "radar_frame_timeout",
)
```

Add tests for a JSON line split across two polls and rejection of a truncated
or replaced epoch file.

- [ ] **Step 2: Write verification-gate tests**

Expected evidence is exact:

```python
expected = ExpectedRadarEvidence(
    profile_id=(
        "lsdk-05.05.04.02-presence-near-"
        "heatmap16-elev8-cfar15-10hz-v1"
    ),
    heatmap_azimuth_bins=16,
    heatmap_range_bins=128,
    heatmap_range_step_m=0.09765625,
)
```

Append four qualifying frames and assert `verified is False`; append the
fifth and assert `verified is True`. Insert an incomplete frame, wrong profile,
missing heatmap, wrong dimensions, and wrong range step separately and assert
each resets `consecutive_good_frames` to zero.

- [ ] **Step 3: Write firmware-marker tests**

Append the confirmed marker in two writes:

```python
raw.write_bytes(b"binary-prefixError: No Sufficient Time ")
append_bytes(raw, b"for getting into Low Power Modes.\n")
self.assertEqual(
    watchdog.poll(0.2).fault_reason,
    "firmware_low_power_timing_assert",
)
```

Assert that arbitrary binary containing neither full marker nor a recognized
TI fatal line does not fault.

- [ ] **Step 4: Run tests and confirm the watchdog is absent**

Run:

```powershell
python -m unittest tests.test_radar_watchdog -v
```

Expected: ERROR importing `sensors.radar_watchdog`.

- [ ] **Step 5: Implement bounded incremental readers**

Implement:

```python
@dataclass(frozen=True)
class ExpectedRadarEvidence:
    profile_id: str
    heatmap_azimuth_bins: int
    heatmap_range_bins: int
    heatmap_range_step_m: float


@dataclass(frozen=True)
class RadarWatchdogSnapshot:
    verified: bool
    consecutive_good_frames: int
    last_frame_observed_s: float | None
    latest_frame_number: int | None
    fault_reason: str | None
```

Keep byte offsets and partial-line buffers per epoch. Decode only newline-
terminated mission entries with `decode_log_entry`; update freshness only for
`RadarFrame`. Keep enough raw overlap to detect a marker split across reads.
Bound each retained partial buffer and turn an over-limit line into a
`mission_evidence_invalid` fault.

- [ ] **Step 6: Implement timeout and verification policy**

Before the first frame, fault at `started_at_s + 3.0`. After any valid radar
frame, fault when its supervisor observation age exceeds 2.5 seconds.
Verification succeeds only after five consecutive qualifying frames before
the three-second verification deadline.

- [ ] **Step 7: Run focused watchdog tests**

Run:

```powershell
python -m unittest tests.test_radar_watchdog -v
```

Expected: all watchdog tests PASS.

- [ ] **Step 8: Commit the evidence watchdog**

```powershell
git add -- `
  sensors/radar_watchdog.py `
  tests/test_radar_watchdog.py
git commit -m "feat: detect stale and asserted radar epochs"
```

---

### Task 5: Implement the Recovery State Machine and Epoch Manifest

**Files:**

- Create: `sensors/radar_supervisor.py`
- Create: `tests/test_radar_supervisor.py`

**Interfaces:**

- Consumes:
  - Task 3 port/reset/profile functions;
  - Task 4 `RadarEpochWatchdog`;
  - an injected process manager, clock, sleeper, and port inventory.
- Produces:
  - `SupervisorState`
  - `RadarSupervisorConfig`
  - `EpochPaths`
  - `allocate_epoch_paths(root: Path, run_id: str, epoch: int) -> EpochPaths`
  - `manifest_path(root: Path, run_id: str) -> Path`
  - `RadarSupervisor.run(stop_requested: Callable[[], bool]) -> None`
  - atomic `radar-supervisor-<run_id>.json` manifest.

- [ ] **Step 1: Write epoch allocation and no-overwrite tests**

Test:

```python
paths = allocate_epoch_paths(
    root,
    run_id="20260729-010000",
    epoch=1,
)
self.assertEqual(
    paths.mission.name,
    "radar-board-live-20260729-010000-e001.jsonl",
)
self.assertEqual(paths.mission.parent, root / "missions")
self.assertEqual(paths.raw.parent, root / "captures")
self.assertEqual(
    manifest_path(root, "20260729-010000"),
    root / "runtime" / "radar-supervisor-20260729-010000.json",
)
self.assertFalse(paths.mission.exists())

paths.mission.parent.mkdir(parents=True)
paths.mission.write_text("owned", encoding="utf-8")
with self.assertRaisesRegex(FileExistsError, "epoch artifact"):
    allocate_epoch_paths(root, run_id="board-live", epoch=1)
```

Assert raw and raw-index names match the design and that run IDs pass the
existing sensor ID validation.

- [ ] **Step 2: Write deterministic state transition tests**

Use fake dependencies that record actions. The healthy path must be:

```python
self.assertEqual(
    events,
    [
        "wait_port:COM3",
        "reset:RI32",
        "wait_port:COM3",
        "configure:COM3",
        "capture:e001",
        "verified:5",
        "viewer:e001",
        "running:e001",
    ],
)
```

The supervisor must not expose the new viewer before verification.
Add a variant where the second discovery returns COM9 and assert
configuration/capture use COM9 while the serial identity remains `RI32`.

- [ ] **Step 3: Write recovery and retry tests**

Cover:

- health-only frame timeout;
- known firmware assert;
- capture exit;
- Application/User port disappearance and COM3-to-COM9 reappearance;
- reset failure;
- reset tool unavailable after a silent assert, remaining fail-closed until an
  observed USB disappearance/reappearance;
- reset tool unavailable after a physical USB cycle, then configuration and
  recovery on the re-enumerated Application/User port;
- profile application result with missing magic/start evidence;
- first-frame verification failure;
- viewer exit while radar remains healthy; and
- explicit shutdown.

For retry timing assert the sequence:

```python
self.assertEqual(sleeper.delays[:5], [0.5, 1.0, 2.0, 4.0, 5.0])
```

On a radar fault, assert capture stops before reset. On viewer-only exit, assert
the viewer restarts on the same verified epoch without resetting the board.

- [ ] **Step 4: Write manifest tests**

After each state change, parse the manifest and assert it contains:

```python
{
    "schema_version": 1,
    "run_id": "board-live",
    "state": "RUNNING",
    "epoch": 1,
    "recovery_count": 0,
    "port": "COM3",
    "xds_serial": "RI32",
    "mission_path": str(paths.mission),
    "raw_path": str(paths.raw),
    "last_reason": "verified_frames",
    "epochs": [
        {
            "epoch": 1,
            "mission_path": str(paths.mission),
            "raw_path": str(paths.raw),
            "raw_index_path": str(paths.raw_index),
            "started_at": "2026-07-29T01:00:00Z",
            "ended_at": None,
            "end_reason": None,
            "capture_exit_code": None,
        },
    ],
    "process_events": [
        {
            "role": "capture",
            "pid": 41001,
            "action": "started",
            "escalation": None,
            "exit_code": None,
        },
    ],
}
```

Patch `os.replace` to prove the write uses a same-directory temporary file.
The temporary file must be flushed and `os.fsync`ed before replacement.
When a capture closes, assert its epoch entry receives `ended_at`,
`end_reason`, and the actual child exit code before another epoch becomes
`RUNNING`. Older epoch summaries are append-only after closure. Process event
rows may name only children returned by the injected process manager; test
that an arbitrary unrelated PID is rejected rather than recorded as a stop
target.

- [ ] **Step 5: Run tests and confirm the supervisor is missing**

Run:

```powershell
python -m unittest tests.test_radar_supervisor -v
```

Expected: ERROR importing `sensors.radar_supervisor`.

- [ ] **Step 6: Implement the state and dependency contracts**

Add:

```python
class SupervisorState(str, Enum):
    WAIT_PORT = "WAIT_PORT"
    RESET_TARGET = "RESET_TARGET"
    CONFIGURE = "CONFIGURE"
    START_CAPTURE = "START_CAPTURE"
    VERIFY_FRAMES = "VERIFY_FRAMES"
    SWITCH_VIEWER = "SWITCH_VIEWER"
    RUNNING = "RUNNING"
    RECOVERING = "RECOVERING"
    STOPPED = "STOPPED"


@dataclass(frozen=True)
class EpochPaths:
    mission: Path
    raw: Path
    raw_index: Path
    runtime_dir: Path
    capture_stdout: Path
    capture_stderr: Path
    viewer_stdout: Path
    viewer_stderr: Path


@dataclass(frozen=True)
class RadarSupervisorConfig:
    repository_root: Path
    output_root: Path
    profile_path: Path
    calibration_path: Path
    run_id: str
    mission_id: str = "radar-board-live"
    profile_id: str = (
        "lsdk-05.05.04.02-presence-near-"
        "heatmap16-elev8-cfar15-10hz-v1"
    )
    explicit_port: str | None = None
    xds_serial: str | None = None
    reset_executable: Path | None = None
    initial_baud: int = 115_200
    data_baud: int = 1_250_000
    heatmap_azimuth_bins: int = 16
    heatmap_range_bins: int = 128
    heatmap_range_step_m: float = 0.09765625
    first_frame_timeout_s: float = 3.0
    frame_timeout_s: float = 2.5
    verification_timeout_s: float = 3.0
    verification_frames: int = 5
    retry_initial_s: float = 0.5
    retry_max_s: float = 5.0
    poll_interval_s: float = 0.05
    http_bind: str = "127.0.0.1"
    http_port: int = 8081
    viewer_max_range_m: float = 3.0
    viewer_history_s: float = 0.3
```

`RadarSupervisorConfig` validates every positive timeout/count, ensures
`retry_initial_s <= retry_max_s`, binds the expected heatmap/profile contract,
and keeps output roots explicit.

- [ ] **Step 7: Implement one-epoch startup and verification**

The order is:

```python
port = dependencies.wait_for_port(config)
reset_result = dependencies.reset_target_if_available(port)
port = dependencies.wait_for_port(config)
profile_result = dependencies.configure(port)
paths = allocate_epoch_paths(config.output_root, config.run_id, epoch)
capture = dependencies.processes.start_capture(port, paths, config)
watchdog = dependencies.watchdog_factory(paths, config)
wait_until_verified(capture, watchdog, config)
viewer = dependencies.processes.switch_viewer(viewer, paths, config)
```

Call `validate_profile_result(..., require_first_magic=False)`. If
`first_magic_observed` is false, allow startup to continue only through the
three-second watchdog verification. Any command-count or prompt mismatch fails
immediately.

At initial startup, absence of a reset executable may proceed directly to one
configuration attempt because the board can already be freshly powered. After
a silent stall with the matching port continuously present, absence of the
reset tool must stay fail-closed with
`reset_tool_unavailable_waiting_for_usb_cycle`; it must not repeatedly send
configuration commands to asserted firmware. Once the supervisor observes
that Application/User port disappear and reappear, the physical power cycle
counts as the reset and configuration may proceed. After an executable-driven
reset, re-run metadata selection before configuration so a changed COM number
is accepted.

- [ ] **Step 8: Implement running monitoring and recovery**

Poll at 50–100 ms. If the capture exits, port disappears, watchdog faults, or
shutdown is requested, stop only owned children. Keep a healthy old viewer
serving its faulted old epoch until the new epoch verifies, then perform the
short viewer switch. Increment `recovery_count` once when leaving `RUNNING`
for a radar fault. Increment `epoch` only when allocating a new capture; reset
or configuration retries that create no files do not consume epoch numbers.
Retain prior files and reset backoff only after entering `RUNNING`.

- [ ] **Step 9: Run supervisor tests**

Run:

```powershell
python -m unittest `
  tests.test_radar_supervisor `
  tests.test_radar_watchdog `
  tests.test_ti_radar_control `
  -v
```

Expected: all tests PASS with no real COM access.

- [ ] **Step 10: Commit the supervisor core**

```powershell
git add -- `
  sensors/radar_supervisor.py `
  tests/test_radar_supervisor.py
git commit -m "feat: supervise radar recovery epochs"
```

---

### Task 6: Add Exact Capture/Viewer Process Commands and the One-Command Launcher

**Files:**

- Create: `sensors/radar_stack_processes.py`
- Create: `scripts/run_radar_stack.py`
- Create: `tests/test_radar_stack_processes.py`
- Create: `tests/test_radar_supervisor_integration.py`
- Modify: `sensors/cli.py`
- Modify: `tests/test_sensor_cli.py`
- Modify: `tests/test_radar_supervisor.py`

**Interfaces:**

- Consumes: `EpochPaths`, `RadarSupervisorConfig`, and the selected port.
- Produces:
  - `ManagedChild`
  - `ChildStopResult`
  - `build_capture_command(port: RadarPortIdentity, paths: EpochPaths, config: RadarSupervisorConfig) -> list[str]`
  - `build_viewer_command(paths: EpochPaths, config: RadarSupervisorConfig) -> list[str]`
  - `RadarStackProcesses.start_capture(port: RadarPortIdentity, paths: EpochPaths, config: RadarSupervisorConfig) -> ManagedChild`
  - `RadarStackProcesses.switch_viewer(current: ManagedChild | None, paths: EpochPaths, config: RadarSupervisorConfig) -> ManagedChild`
  - `RadarStackProcesses.stop_owned_children() -> tuple[ChildStopResult, ...]`
  - launcher `build_parser()` and `main()`.

- [ ] **Step 1: Write exact capture-command tests**

Assert that `build_capture_command(...)` produces:

```python
[
    sys.executable,
    "-m",
    "sensors",
    "radar-live",
    "--port",
    "COM3",
    "--baud",
    "1250000",
    "--allow-elided-empty-point-tlv",
    "--allow-nonzero-padding",
    "--heatmap-azimuth-bins",
    "16",
    "--heatmap-range-bins",
    "128",
    "--heatmap-range-step-m",
    "0.09765625",
    "--output",
    str(paths.mission),
    "--raw-output",
    str(paths.raw),
    "--raw-index",
    str(paths.raw_index),
    "--mission-id",
    config.mission_id,
    "--profile-id",
    config.profile_id,
    "--calibration-id",
    "uncalibrated",
]
```

Assert `--overwrite` is absent.

- [ ] **Step 2: Write exact viewer-command and switch-order tests**

Assert:

```python
[
    sys.executable,
    "monitor/radar_front.py",
    "--follow",
    str(paths.mission),
    "--clutter-calibration",
    str(config.calibration_path),
    "--bind",
    "127.0.0.1",
    "--http-port",
    "8081",
    "--max-range-m",
    "3",
    "--history-window",
    "0.3",
    "--quiet",
]
```

The fake process runner must record `stop:old-viewer` before
`start:new-viewer`, but this switch is invoked only after Task 5 reports
verification success.

- [ ] **Step 3: Write owned-child shutdown tests**

Model:

- graceful interrupt succeeds;
- graceful timeout requires terminate;
- terminate timeout requires kill; and
- an unrelated process handle is never touched.

On Windows assert capture starts with `CREATE_NEW_PROCESS_GROUP` and a hidden
window flag. On POSIX assert a new process session/group is created.
Assert each stop returns the exact owned role/PID, final exit code, and one of
`already_exited`, `graceful`, `terminate`, or `kill`; a
caller-supplied/unregistered PID cannot be passed to the stop API.

- [ ] **Step 4: Write a graceful radar-live signal test**

Patch `capture_radar_uart` with a fake that invokes the installed termination
handler. Assert the handler converts Windows `SIGBREAK` when available and
POSIX `SIGTERM` into `KeyboardInterrupt` inside the capture path, allowing
`capture_radar_uart` to write its final health/index footer. Assert previous
handlers are restored after `command_radar_live` returns.

Implement a private context manager in `sensors/cli.py`:

```python
@contextmanager
def _radar_shutdown_signals():
    previous: dict[int, object] = {}

    def interrupt_capture(signum, frame):
        raise KeyboardInterrupt

    for name in ("SIGTERM", "SIGBREAK"):
        signum = getattr(signal, name, None)
        if signum is not None:
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt_capture)
    try:
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
```

Wrap only the `capture_radar_uart(...)` call in `command_radar_live`. Do not
change signal behavior for other sensor commands.

- [ ] **Step 5: Write launcher parser tests**

Import `scripts.run_radar_stack` by file path and assert defaults:

```python
self.assertEqual(args.port, None)
self.assertEqual(args.xds_serial, None)
self.assertEqual(args.run_id, None)
self.assertEqual(args.output_root, REPOSITORY_ROOT)
self.assertEqual(args.frame_timeout, 2.5)
self.assertEqual(args.first_frame_timeout, 3.0)
self.assertEqual(args.verify_frames, 5)
self.assertEqual(args.http_port, 8081)
self.assertEqual(
    args.cfg.name,
    "iwrl6432_3d_operator_near_10hz.cfg",
)
```

Reject zero/negative timeouts, verification count below one, retry initial
greater than retry maximum, and a missing calibration path.
When `--run-id` is omitted, `main()` generates one UTC timestamp once and
reuses it for all mission, raw, log, and manifest paths. `--output-root`
contains `missions`, `captures`, and `runtime` subdirectories.

- [ ] **Step 6: Write a lightweight subprocess integration test**

In `tests/test_radar_supervisor_integration.py`, create a temporary helper
script that appends parent-provided, already encoded mission lines on a timed
schedule. Drive this sequence with real Python child processes:

```text
port absent
-> Application/User COM3 appears
-> epoch e001 emits five valid frames
-> health-only records continue until the 2.5 s frame timeout
-> COM3 disappears and the same RI32 Application/User port returns as COM7
-> epoch e002 emits five valid frames
```

Assert the old viewer remains owned until e002 verifies, the replacement
viewer receives e002, e001 bytes remain unchanged, the manifest reports one
recovery, and shutdown leaves no helper child alive.

Add a launcher subprocess check from a working directory outside the
repository. It must successfully print `--help`; this forces
`scripts/run_radar_stack.py` to bootstrap the repository root before importing
the `sensors` package.

- [ ] **Step 7: Run tests and confirm interfaces are missing**

Run:

```powershell
python -m unittest `
  tests.test_radar_stack_processes `
  tests.test_radar_supervisor_integration `
  tests.test_sensor_cli `
  tests.test_radar_supervisor `
  -v
```

Expected: import/attribute failures.

- [ ] **Step 8: Implement `ManagedChild`**

Add:

```python
@dataclass(frozen=True)
class ChildStopResult:
    role: str
    pid: int
    exit_code: int
    escalation: Literal["already_exited", "graceful", "terminate", "kill"]


@dataclass
class ManagedChild:
    role: str
    process: subprocess.Popen
    owned: bool = True

    def stop(self, grace_s: float = 2.0) -> ChildStopResult:
        if not self.owned:
            raise RuntimeError("refusing to signal an unowned process")
        exit_code = self.process.poll()
        if exit_code is not None:
            return ChildStopResult(
                self.role,
                self.process.pid,
                exit_code,
                "already_exited",
            )
        if os.name == "nt":
            self.process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        escalation = "graceful"
        try:
            exit_code = self.process.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            escalation = "terminate"
            self.process.terminate()
            try:
                exit_code = self.process.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                escalation = "kill"
                self.process.kill()
                exit_code = self.process.wait(timeout=grace_s)
        return ChildStopResult(
            self.role,
            self.process.pid,
            exit_code,
            escalation,
        )
```

Use `subprocess.Popen` with explicit working directory and stdout/stderr paths
under the run's runtime directory. `stop(grace_s=2.0)` sends
`CTRL_BREAK_EVENT` on Windows or `SIGINT` to the POSIX process group, waits,
then escalates to terminate and kill with bounded waits. It returns the final
exit code and escalation used for the manifest. Reject `owned=False` before
signaling.

- [ ] **Step 9: Implement capture and viewer commands**

Use argument lists only; never construct a shell command string. Create
stdout/stderr log paths per epoch. Start the capture only after configuration,
and start the replacement viewer only after supervisor verification.

- [ ] **Step 10: Implement the thin launcher**

`scripts/run_radar_stack.py` must:

1. resolve and add the repository root to `sys.path` before sensor imports;
2. parse and validate explicit arguments;
3. build `RadarSupervisorConfig`;
4. use pyserial `comports()` for discovery;
5. auto-discover `xds110reset` unless explicitly supplied;
6. install Ctrl+C/SIGTERM shutdown handling; and
7. call `RadarSupervisor.run`.

Do not open a browser from the supervisor; the operator URL remains
`http://127.0.0.1:8081/`.

Failure to discover `xds110reset` is a recoverable capability state, not a
launcher crash. Pass `None` into the supervisor, report the exact reason in
the manifest, and apply Task 5's USB power-cycle fallback.

- [ ] **Step 11: Verify command construction, integration, and help output**

Run:

```powershell
python -m unittest `
  tests.test_radar_stack_processes `
  tests.test_radar_supervisor_integration `
  tests.test_sensor_cli `
  tests.test_radar_supervisor `
  -v
python scripts\run_radar_stack.py --help
```

Expected: tests PASS; help shows every timeout, board identity, reset tool,
profile, calibration, output, and HTTP option with concrete defaults.

- [ ] **Step 12: Commit process lifecycle and launcher**

```powershell
git add -- `
  sensors/radar_stack_processes.py `
  sensors/cli.py `
  sensors/radar_supervisor.py `
  scripts/run_radar_stack.py `
  tests/test_radar_stack_processes.py `
  tests/test_radar_supervisor_integration.py `
  tests/test_sensor_cli.py `
  tests/test_radar_supervisor.py
git commit -m "feat: launch the self-recovering radar stack"
```

---

### Task 7: Make Recovery State Explicit to the Operator and Document Startup

**Files:**

- Modify: `monitor/radar_front.py:51`
- Modify: `monitor/web/radar_front.html:779-780`
- Modify: `monitor/web/radar_panel.js:804-818`
- Modify: `tests/test_radar_front.py`
- Create: `tests/web/radar_panel.test.js`
- Create: `docs/radar_auto_recovery.md`
- Modify: `docs/radar_front_view.md`

**Interfaces:**

- Consumes: existing `waiting`, `stale`, `fault`, `sensor_fault`, and
  `http_lost` presentation paths.
- Produces: explicit reconnecting/drive-stop copy, UI build
  `20260729-lidar-operator-r10`, and the one-command runbook.

- [ ] **Step 1: Write a served-asset copy regression**

Extend the existing radar web asset test:

```python
with urlopen(base + "/radar_panel.js", timeout=2) as response:
    javascript = response.read().decode("utf-8")
self.assertIn("RADAR RECONNECTING · DRIVE STOP", javascript)
self.assertIn("레이더 재연결 중 · 주행을 정지하세요", javascript)
self.assertIn("sensor_fault:", javascript)
self.assertIn(
    'const UI_BUILD_ID = "20260729-lidar-operator-r10"',
    javascript,
)
self.assertEqual(payload["ui_build_id"], "20260729-lidar-operator-r10")
self.assertIn(
    "/radar_scene.js?v=20260729-lidar-operator-r10",
    html,
)
self.assertIn(
    "/radar_panel.js?v=20260729-lidar-operator-r10",
    html,
)
```

- [ ] **Step 2: Run the focused test and confirm the copy is absent**

Run the exact existing HTTP asset test containing the new assertions:

```powershell
python -m unittest `
  tests.test_radar_front.RadarFrontHttpTests `
  -v
```

Expected: FAIL because the current build is `r9`, HTTP loss says
`DATA LINK LOST`, and `sensor_fault` falls through to `MAP BLOCKED`.

- [ ] **Step 3: Make every live-input fault explicitly movement-blocking**

Set:

```javascript
waiting: ["RADAR STARTING · DRIVE STOP", "레이더 준비 중 · 주행을 정지하세요"],
stale: ["RADAR RECONNECTING · DRIVE STOP", "레이더 재연결 중 · 주행을 정지하세요"],
fault: ["RADAR RECONNECTING · DRIVE STOP", "레이더 재연결 중 · 주행을 정지하세요"],
sensor_fault: ["RADAR RECONNECTING · DRIVE STOP", "레이더 재연결 중 · 주행을 정지하세요"],
http_lost: ["RADAR RECONNECTING · DRIVE STOP", "레이더 재연결 중 · 주행을 정지하세요"],
```

Keep the blocked presentation, canvases, map ranges, danger threshold, and
scene parser unchanged. Keep calibration/profile/replay errors distinct.

- [ ] **Step 4: Write browser-VM fail-closed behavior tests**

Create `tests/web/radar_panel.test.js` with a minimal fake DOM/canvas harness
that loads `radar_panel.js` through `node:vm`. Table-drive all blocked reasons:

```javascript
[
  "waiting",
  "stale",
  "fault",
  "sensor_fault",
  "http_lost",
  "replay_end",
  "calibration_required",
  "calibration_unavailable",
  "profile_mismatch",
  "invalid_scene",
]
```

For every row assert:

- `draw()` clears both canvases and calls `drawBlockingOverlay` twice;
- neither `drawLidarTopView` nor `drawCollisionInset` renders old scene data;
- nearest distance is `--`, all five sectors are `invalid/차단`, and
  `radar-mode` is `MAP BLOCKED`;
- `metric-hazard`, `collision-inset`, and diagnostic scene hazard report
  `SENSOR_FAULT`, never `NORMAL` or `UNKNOWN`.

For `waiting`, require the starting/drive-stop copy. For
`stale`, `fault`, `sensor_fault`, and `http_lost`, require the
reconnecting/drive-stop copy and assert the status badge cannot remain
`LIVE`. The remaining rows keep their distinct calibration/profile/replay/
contract overlay headings while remaining movement-blocking.

- [ ] **Step 5: Implement the explicit blocked-state presentation**

Add a small exact set for the five live-input reasons and use
`presentation.reason`, not only `snapshot.status`, when computing the badge.
When any presentation is blocked:

- set the collision and hazard metrics to `SENSOR_FAULT`;
- set the scene hazard diagnostic to `SENSOR_FAULT`;
- clear nearest-point guidance;
- invalidate all sectors; and
- draw only the blocking overlays on both canvases.

Extend the collision-inset CSS in `radar_front.html` so
`data-hazard="SENSOR_FAULT"` receives the existing blocked styling. Do not
reuse `UNKNOWN`, because an unavailable or rejected sensor is a fault, not an
unobserved but healthy region.

- [ ] **Step 6: Bump the UI contract identifier consistently**

Change the exact identifier from `20260728-lidar-operator-r9` to
`20260729-lidar-operator-r10` in:

1. `monitor/radar_front.py`;
2. `monitor/web/radar_panel.js`; and
3. both script query strings in `monitor/web/radar_front.html`.

Update the API and served-asset expectations in `tests/test_radar_front.py`.
All four locations must agree so an already-open page reloads after the
viewer process switches epochs.

- [ ] **Step 7: Write the operator runbook**

`docs/radar_auto_recovery.md` must contain:

- prerequisite pyserial and TI UniFlash/XDS reset tool checks;
- exact one-command Windows startup;
- the state meanings from `WAIT_PORT` through `RUNNING`;
- artifact and manifest locations;
- the fact that `SENSOR_FAULT` is movement-blocking;
- automatic COM renumbering behavior;
- behavior when reset tooling is unavailable;
- current Windows shutdown command/Ctrl+C behavior; and
- Raspberry Pi note that a reset command/GPIO must replace the Windows
  executable.

The normal-operation command uses a persistent calibrated model:

```powershell
python scripts\run_radar_stack.py `
  --xds-serial RI32 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --clutter-calibration `
    configs\radar\calibrations\head-near.json
```

If that persistent file is absent, the runbook must stop before normal
operation and link the controlled empty-scene capture plus
`python -m sensors radar-calibrate` commands already documented in
`docs/radar_front_view.md`. Put the current temporary fixture calibration in
a separate clearly labeled bench-acceptance section; never describe it as the
normal startup default.

- [ ] **Step 8: Link the supervised path from the existing viewer guide**

Add a top-level note in `docs/radar_front_view.md` that normal Windows
operation should use `docs/radar_auto_recovery.md`; retain the manual commands
as diagnostic fallback.

- [ ] **Step 9: Verify UI assets and docs**

Run:

```powershell
python -m unittest tests.test_radar_front -v
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --test `
  tests\web\radar_scene.test.js `
  tests\web\radar_panel.test.js
git diff --check -- `
  monitor/radar_front.py `
  monitor/web/radar_front.html `
  monitor/web/radar_panel.js `
  tests/test_radar_front.py `
  tests/web/radar_panel.test.js `
  docs/radar_auto_recovery.md `
  docs/radar_front_view.md
```

Expected: Python and Node tests PASS; diff check is clean.

- [ ] **Step 10: Commit operator-facing recovery behavior**

```powershell
git add -- `
  monitor/radar_front.py `
  monitor/web/radar_front.html `
  monitor/web/radar_panel.js `
  tests/test_radar_front.py `
  tests/web/radar_panel.test.js `
  docs/radar_auto_recovery.md `
  docs/radar_front_view.md
git commit -m "feat: expose radar reconnecting stop state"
```

---

### Task 8: Run Full Automated Verification and Independent Review

**Files:**

- Verify only; modify only if a test identifies a scoped defect.

**Interfaces:**

- Consumes: all preceding task outputs.
- Produces: a clean automated verification result before touching hardware.

- [ ] **Step 1: Run all focused radar recovery tests**

```powershell
python -m unittest `
  tests.test_configure_ti_radar `
  tests.test_ti_radar_control `
  tests.test_radar_watchdog `
  tests.test_radar_supervisor `
  tests.test_radar_supervisor_integration `
  tests.test_radar_stack_processes `
  tests.test_radar_capture `
  tests.test_radar_front `
  tests.test_radar_scene `
  tests.test_sensor_cli `
  -v
```

Expected: zero failures and zero errors.

- [ ] **Step 2: Run the entire Python suite**

```powershell
python -m unittest discover -s tests -v
```

Expected: zero failures and zero errors.

- [ ] **Step 3: Run the JavaScript contract suite**

Use the bundled Node executable already resolved for this workspace:

```powershell
$radarNode = "C:\Users\minho\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
& $radarNode --test `
  tests\web\radar_scene.test.js `
  tests\web\radar_panel.test.js
```

Expected: zero failed tests.

- [ ] **Step 4: Run static and command-contract checks**

```powershell
python -m compileall -q `
  sensors `
  scripts `
  monitor
python scripts\configure_ti_radar.py `
  --port COM3 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --dry-run
python scripts\run_radar_stack.py --help
git diff --check
git status --short
```

Expected: compile and dry-run exit zero; no whitespace errors; only intended
branch changes plus the five known unrelated dirty files.

- [ ] **Step 5: Request two-stage review**

Use `superpowers:requesting-code-review` with reviewers focused separately on:

1. safety/state-machine/serial ownership/artifact preservation; and
2. Windows process/XDS reset behavior, CLI contracts, UI fail-closed state,
   and test coverage.

Resolve critical and important findings using
`superpowers:receiving-code-review`, rerun Steps 1–4, and commit each scoped
fix separately.

---

### Task 9: Recover the Current Board and Perform Real-Hardware Acceptance

**Files:**

- Runtime artifacts under `missions/`, `captures/`, and the configured runtime
  log directory.
- No tracked source modification unless the board exposes a reproducible
  scoped defect.

**Interfaces:**

- Consumes: the approved supervisor and current XDS110 serial `RI32`.
- Produces: a live port-8081 viewer, preserved recovery epochs, and measured
  stability/recovery evidence.

- [ ] **Step 1: Stop only the verified old capture/viewer processes**

Resolve exact command lines for the current `radar-live` COM3 owner and
`radar_front.py --http-port 8081` process. Stop only those PIDs. Confirm COM3
and TCP 8081 are free before startup.

```powershell
$oldCapture = @(
  Get-CimInstance Win32_Process |
    Where-Object {
      $_.Name -match '^python(\.exe)?$' -and
      $_.CommandLine -match '-m\s+sensors\s+radar-live' -and
      $_.CommandLine -match '--port\s+COM3'
    }
)
$oldViewer = @(
  Get-CimInstance Win32_Process |
    Where-Object {
      $_.Name -match '^python(\.exe)?$' -and
      $_.CommandLine -match 'monitor[\\/]radar_front\.py' -and
      $_.CommandLine -match '--http-port\s+8081'
    }
)
$listeners = @(
  Get-NetTCPConnection -State Listen -LocalPort 8081 -ErrorAction SilentlyContinue
)
if ($oldCapture.Count -gt 1 -or $oldViewer.Count -gt 1) {
  throw "ambiguous old radar process selection"
}
$unrelatedListeners = @(
  $listeners |
    Where-Object { $oldViewer.ProcessId -notcontains $_.OwningProcess }
)
if ($unrelatedListeners.Count -gt 0) {
  throw "TCP 8081 belongs to an unrelated process"
}
@($oldCapture + $oldViewer) |
  Format-Table ProcessId, Name, CommandLine -AutoSize
$oldTargetIds = @($oldCapture + $oldViewer).ProcessId
$oldTargetIds | ForEach-Object { Stop-Process -Id $_ }
$stopDeadline = [DateTime]::UtcNow.AddSeconds(5)
do {
  $remainingOldTargets = @(
    $oldTargetIds |
      Where-Object { Get-Process -Id $_ -ErrorAction SilentlyContinue }
  )
  if ($remainingOldTargets.Count -eq 0) { break }
  Start-Sleep -Milliseconds 100
} while ([DateTime]::UtcNow -lt $stopDeadline)
if ($remainingOldTargets.Count -gt 0) {
  $remainingOldTargets | ForEach-Object { Stop-Process -Id $_ -Force }
}
if (Get-NetTCPConnection -State Listen -LocalPort 8081 -ErrorAction SilentlyContinue) {
  throw "TCP 8081 is still occupied"
}
python -c "import serial; p=serial.Serial('COM3',1250000,timeout=.1); p.close()"
```

The selection table is evidence: if either command-line predicate is
ambiguous or port 8081 belongs to something else, stop and refine the
read-only predicate rather than broadening termination.

- [ ] **Step 2: Start the supervised stack**

Run hidden/background with logs redirected to a unique runtime directory:

```powershell
$repositoryRoot = (Resolve-Path .).Path
$acceptanceRun = [DateTime]::UtcNow.ToString("yyyyMMdd-HHmmss")
$acceptanceRuntime = Join-Path `
  $repositoryRoot `
  "runtime\radar-acceptance-$acceptanceRun"
New-Item -ItemType Directory -Path $acceptanceRuntime | Out-Null
$pythonExecutable = (Get-Command python).Source
$stackArguments = @(
  "scripts\run_radar_stack.py",
  "--run-id", $acceptanceRun,
  "--output-root", $repositoryRoot,
  "--xds-serial", "RI32",
  "--cfg", "configs\radar\iwrl6432_3d_operator_near_10hz.cfg",
  "--clutter-calibration",
  "C:\Users\minho\AppData\Local\Temp\hansel-r9-fixture-calibration.json"
)
$stackProcess = Start-Process `
  -FilePath $pythonExecutable `
  -ArgumentList $stackArguments `
  -WorkingDirectory $repositoryRoot `
  -WindowStyle Hidden `
  -RedirectStandardOutput (Join-Path $acceptanceRuntime "supervisor.stdout.log") `
  -RedirectStandardError (Join-Path $acceptanceRuntime "supervisor.stderr.log") `
  -PassThru
$acceptanceState = [ordered]@{
  run_id = $acceptanceRun
  repository_root = $repositoryRoot
  runtime_dir = $acceptanceRuntime
  supervisor_pid = $stackProcess.Id
  manifest_path = Join-Path `
    $repositoryRoot `
    "runtime\radar-supervisor-$acceptanceRun.json"
}
$acceptanceState |
  ConvertTo-Json |
  Set-Content `
    -LiteralPath (Join-Path $repositoryRoot "runtime\radar-acceptance-current.json") `
    -Encoding utf8
```

Expected initial sequence: target reset, COM3 discovery, 25 completed profile
commands, capture epoch `e001`, five verified frames, viewer on port 8081.
The temporary `hansel-r9-fixture-calibration.json` is used only for this
bench acceptance. Normal operation remains blocked until the persistent
controlled calibration from Task 7 exists.

- [ ] **Step 3: Verify the live API before soak**

Load `runtime/radar-acceptance-current.json`, wait up to 30 seconds for its
manifest and API, then sample twice two seconds apart:

```powershell
$acceptance = Get-Content `
  -LiteralPath runtime\radar-acceptance-current.json `
  -Raw |
  ConvertFrom-Json
$deadline = [DateTime]::UtcNow.AddSeconds(30)
do {
  try {
    $api1 = Invoke-RestMethod `
      -Uri http://127.0.0.1:8081/api/radar `
      -TimeoutSec 2
  } catch {
    $api1 = $null
  }
  if ($api1) { break }
  Start-Sleep -Milliseconds 250
} while ([DateTime]::UtcNow -lt $deadline)
if (-not $api1) { throw "radar API did not become ready in 30 seconds" }
$manifest1 = Get-Content -LiteralPath $acceptance.manifest_path -Raw |
  ConvertFrom-Json
$missionSize1 = (Get-Item -LiteralPath $manifest1.mission_path).Length
$rawSize1 = (Get-Item -LiteralPath $manifest1.raw_path).Length
Start-Sleep -Seconds 2
$api2 = Invoke-RestMethod -Uri http://127.0.0.1:8081/api/radar -TimeoutSec 2
$missionSize2 = (Get-Item -LiteralPath $manifest1.mission_path).Length
$rawSize2 = (Get-Item -LiteralPath $manifest1.raw_path).Length
```

Require:

```text
status = live or degraded with rendering allowed
ui_build_id = 20260729-lidar-operator-r10
frame.complete = true
frame.heatmap_status = available
scene.calibration_status = ok
scene.pose_mode = robot_relative
health.status = ok or explained degraded
counters.frame_gaps_total = 0
counters.parse_errors_total = 0
```

Also require `api2.frame.number > api1.frame.number`,
`missionSize2 > missionSize1`, and `rawSize2 > rawSize1`. This checks the raw
path from the manifest rather than assuming raw byte counts are exposed by the
HTTP API.

- [ ] **Step 4: Run a 15-minute low-power-assert soak**

Run one yielding PowerShell loop for 90 samples at 10-second intervals and
collect `runtime/radar-acceptance-<run>/soak.jsonl`. While the process runs,
retrieve output at least every 50 seconds so the user continues receiving
progress updates. Each sample records API status/build/frame/age/FPS/gaps/
parse errors, manifest state/epoch/recovery count, and raw length.

```powershell
$acceptance = Get-Content `
  -LiteralPath runtime\radar-acceptance-current.json `
  -Raw |
  ConvertFrom-Json
$soakLog = Join-Path $acceptance.runtime_dir "soak.jsonl"
$baselineApi = Invoke-RestMethod `
  -Uri http://127.0.0.1:8081/api/radar `
  -TimeoutSec 2
$baselineManifest = Get-Content `
  -LiteralPath $acceptance.manifest_path `
  -Raw |
  ConvertFrom-Json
$expectedEpoch = [int]$baselineManifest.epoch
$expectedRecoveries = [int]$baselineManifest.recovery_count
$previousFrame = [int64]$baselineApi.frame.number
$previousRawLength = (
  Get-Item -LiteralPath $baselineManifest.raw_path
).Length
1..90 | ForEach-Object {
  Start-Sleep -Seconds 10
  $api = Invoke-RestMethod `
    -Uri http://127.0.0.1:8081/api/radar `
    -TimeoutSec 2
  $manifest = Get-Content `
    -LiteralPath $acceptance.manifest_path `
    -Raw |
    ConvertFrom-Json
  $rawLength = (Get-Item -LiteralPath $manifest.raw_path).Length
  $frameDelta = [int64]$api.frame.number - $previousFrame
  $failures = @()
  if ($api.status -notin @("live", "degraded")) {
    $failures += "status=$($api.status)"
  }
  if (
    $api.status -eq "degraded" -and
    [string]::IsNullOrWhiteSpace([string]$api.warning)
  ) {
    $failures += "unexplained degraded status"
  }
  if ($api.frame.complete -ne $true) { $failures += "incomplete frame" }
  if ($api.frame.heatmap_status -ne "available") {
    $failures += "heatmap unavailable"
  }
  if ([double]$api.fps -lt 8.0 -or [double]$api.fps -gt 12.0) {
    $failures += "fps=$($api.fps)"
  }
  if ([int]$api.age_ms -gt 750) { $failures += "age_ms=$($api.age_ms)" }
  if ($frameDelta -lt 80 -or $frameDelta -gt 120) {
    $failures += "frame_delta=$frameDelta"
  }
  if ($rawLength -le $previousRawLength) {
    $failures += "raw capture did not grow"
  }
  if ([int]$api.counters.frame_gaps_total -ne 0) {
    $failures += "frame gaps"
  }
  if ([int]$api.counters.parse_errors_total -ne 0) {
    $failures += "parse errors"
  }
  if ([int]$api.counters.writer_drops_total -ne 0) {
    $failures += "writer drops"
  }
  if (
    $manifest.state -ne "RUNNING" -or
    [int]$manifest.epoch -ne $expectedEpoch -or
    [int]$manifest.recovery_count -ne $expectedRecoveries
  ) {
    $failures += "supervisor epoch/state changed"
  }
  $sample = [ordered]@{
    sample = $_
    timestamp_utc = [DateTime]::UtcNow.ToString("o")
    status = $api.status
    warning = $api.warning
    frame = [int64]$api.frame.number
    frame_delta = $frameDelta
    age_ms = [int]$api.age_ms
    fps = [double]$api.fps
    frame_gaps = [int]$api.counters.frame_gaps_total
    parse_errors = [int]$api.counters.parse_errors_total
    writer_drops = [int]$api.counters.writer_drops_total
    supervisor_state = $manifest.state
    epoch = [int]$manifest.epoch
    recovery_count = [int]$manifest.recovery_count
    raw_length = $rawLength
    failures = $failures
  }
  $sample |
    ConvertTo-Json -Compress |
    Add-Content -LiteralPath $soakLog -Encoding utf8
  Write-Output (
    "sample {0}/90 frame={1} delta={2} fps={3:N1} age={4}ms" -f
      $_, $sample.frame, $frameDelta, $sample.fps, $sample.age_ms
  )
  if ($failures.Count -gt 0) {
    throw ($failures -join "; ")
  }
  $previousFrame = [int64]$api.frame.number
  $previousRawLength = $rawLength
}
```

After the first sample, fail the loop if:

- frame advance over a 10-second interval is outside 80–120 frames;
- reported FPS is outside 8.0–12.0;
- frame age exceeds 750 ms;
- raw length does not increase;
- API status is not `live`/explained `degraded`;
- supervisor state is not `RUNNING`;
- epoch or recovery count changes; or
- gaps, parser errors, or writer drops become nonzero.

Use this exact final marker check, scoped to the run:

```powershell
$acceptance = Get-Content `
  -LiteralPath runtime\radar-acceptance-current.json `
  -Raw |
  ConvertFrom-Json
$captureRoot = Join-Path $acceptance.repository_root "captures"
rg -a -F `
  -g "*$($acceptance.run_id)*.bin" `
  "Error: No Sufficient Time for getting into Low Power Modes." `
  $captureRoot
if ($LASTEXITCODE -eq 0) {
  throw "low-power timing assert returned"
}
if ($LASTEXITCODE -ne 1) {
  throw "raw marker scan failed"
}
```

Acceptance requires:

- no `Error: No Sufficient Time for getting into Low Power Modes.`;
- no automatic recovery epoch caused by that marker;
- frame advancement and reported FPS within the numeric bounds above;
- no frame gaps or parse errors; and
- no USB/device reset, disconnect, odor, or visible thermal malfunction.

The current telemetry does not expose board temperature. If an IR thermometer
is available, record board temperature at 0, 5, 10, and 15 minutes. Otherwise
report thermal temperature as **not measured** and do not claim a quantified
thermal pass.

- [ ] **Step 5: Inject a target reset while the supervisor is running**

Execute the same serial-scoped reset used by the supervisor:

```powershell
$acceptance = Get-Content `
  -LiteralPath runtime\radar-acceptance-current.json `
  -Raw |
  ConvertFrom-Json
$resetStartedUtc = [DateTime]::UtcNow
$recoveryClock = [Diagnostics.Stopwatch]::StartNew()
& 'C:\ti\uniflash_9.6.0\deskdb\content\TICloudAgent\win\ccs_base\common\uscif\xds110\xds110reset.exe' `
  -a toggle `
  -d 100 `
  -s RI32
if ($LASTEXITCODE -ne 0) { throw "injected XDS110 reset failed" }
$blockedMs = $null
$recoveredMs = $null
while ($recoveryClock.Elapsed.TotalSeconds -lt 15) {
  try {
    $recoveryApi = Invoke-RestMethod `
      -Uri http://127.0.0.1:8081/api/radar `
      -TimeoutSec 1
  } catch {
    $recoveryApi = $null
  }
  if (
    $null -eq $blockedMs -and
    (
      $null -eq $recoveryApi -or
      $recoveryApi.status -notin @("live", "degraded")
    )
  ) {
    $blockedMs = [int]$recoveryClock.Elapsed.TotalMilliseconds
  }
  $recoveryManifest = Get-Content `
    -LiteralPath $acceptance.manifest_path `
    -Raw |
    ConvertFrom-Json
  if (
    $recoveryManifest.state -eq "RUNNING" -and
    [int]$recoveryManifest.epoch -eq 2 -and
    $recoveryApi -and
    $recoveryApi.status -in @("live", "degraded") -and
    $recoveryApi.frame.complete -eq $true
  ) {
    $recoveredMs = [int]$recoveryClock.Elapsed.TotalMilliseconds
    break
  }
  Start-Sleep -Milliseconds 100
}
$recoveryClock.Stop()
if ($null -eq $blockedMs -or $blockedMs -gt 2500) {
  throw "operator API did not fail closed within 2.5 seconds"
}
if ($null -eq $recoveredMs -or $recoveredMs -gt 15000) {
  throw "radar stack did not recover within 15 seconds"
}
[ordered]@{
  reset_started_utc = $resetStartedUtc.ToString("o")
  blocked_ms = $blockedMs
  recovered_ms = $recoveredMs
  epoch = [int]$recoveryManifest.epoch
  recovery_count = [int]$recoveryManifest.recovery_count
} |
  ConvertTo-Json |
  Set-Content `
    -LiteralPath (Join-Path $acceptance.runtime_dir "reset-recovery.json") `
    -Encoding utf8
```

Expected: the UI enters a blocking fault/reconnecting state within 2.5
seconds, the supervisor creates `e002`, reapplies the profile, verifies five
frames, switches the viewer, and returns to live within 15 seconds after the
port is available. The measurement above starts before the injected reset, so
its 15-second bound is stricter than measuring only after port reappearance.
Automated browser-VM tests prove the corresponding non-live state renders no
old scene; the live browser is visually checked in Step 7.

- [ ] **Step 6: Verify evidence preservation**

Wait until the manifest reports `e002` `RUNNING` and the e001 entry has a
non-null `ended_at`, `end_reason`, and `capture_exit_code`. Only then compute
the e001 mission/raw/index baseline hashes, because graceful shutdown is
allowed to append its final health/index footer. Run:

```powershell
$acceptance = Get-Content `
  -LiteralPath runtime\radar-acceptance-current.json `
  -Raw |
  ConvertFrom-Json
$recoveryManifest = Get-Content `
  -LiteralPath $acceptance.manifest_path `
  -Raw |
  ConvertFrom-Json
$epoch1 = @($recoveryManifest.epochs | Where-Object { [int]$_.epoch -eq 1 })
$epoch2 = @($recoveryManifest.epochs | Where-Object { [int]$_.epoch -eq 2 })
if ($epoch1.Count -ne 1 -or $epoch2.Count -ne 1) {
  throw "expected exactly e001 and e002"
}
if (
  -not $epoch1[0].ended_at -or
  -not $epoch1[0].end_reason -or
  $null -eq $epoch1[0].capture_exit_code
) {
  throw "e001 was not finalized before preservation verification"
}
python -m sensors inspect $epoch1[0].mission_path
if ($LASTEXITCODE -ne 0) { throw "e001 mission validation failed" }
python -m sensors radar-index `
  $epoch1[0].raw_path `
  --index $epoch1[0].raw_index_path
if ($LASTEXITCODE -ne 0) { throw "e001 raw/index validation failed" }
$epoch1Paths = @(
  $epoch1[0].mission_path,
  $epoch1[0].raw_path,
  $epoch1[0].raw_index_path
)
$baselineHashes = @(
  $epoch1Paths |
    ForEach-Object { Get-FileHash -LiteralPath $_ -Algorithm SHA256 }
)
$baselineHashes |
  Select-Object Path, Hash |
  ConvertTo-Json |
  Set-Content `
    -LiteralPath (Join-Path $acceptance.runtime_dir "e001-hashes.json") `
    -Encoding utf8
Start-Sleep -Seconds 30
$currentHashes = @(
  $epoch1Paths |
    ForEach-Object { Get-FileHash -LiteralPath $_ -Algorithm SHA256 }
)
for ($index = 0; $index -lt $baselineHashes.Count; $index++) {
  if ($baselineHashes[$index].Hash -ne $currentHashes[$index].Hash) {
    throw "closed e001 artifact changed"
  }
}
```

Save all three `Get-FileHash -Algorithm SHA256` values, wait at least 30
seconds while e002 advances, recompute them, and require exact equality.
Then check that:

- `e001` mission/raw/index files still exist and their hashes have not changed;
- `e002` uses new paths and a new radar producer epoch;
- the manifest reports `epoch = 2` and `recovery_count = 1`; and
- supervisor process logs contain only owned capture/viewer PIDs in stop
  actions; no unrelated Python or port-8081 process was targeted.

- [ ] **Step 7: Open or refresh the operator browser**

Navigate the intended browser tab to `http://127.0.0.1:8081/`. If the browser
surface refuses localhost automation by policy, keep the server live and ask
the user for one manual `Ctrl+R`; do not use an external Playwright/CDP
workaround.

- [ ] **Step 8: Report the remaining physical USB test boundary**

The automated XDS target reset verifies the complete reset/configure/capture/
viewer path. A literal cable removal requires the user to unplug and reconnect
USB once. When they do, verify COM removal/reappearance, optional COM
renumbering, automatic new epoch creation, and recovery without pressing the
board RESET button.

---

## Final Completion Gate

Before claiming completion:

1. invoke `superpowers:verification-before-completion`;
2. rerun Task 8 Steps 1–4 after the final code change;
3. show fresh Task 9 API/process/artifact evidence;
4. confirm the supervisor remains running for the user;
5. push the intended branch only after the user-requested scope is verified;
   and
6. report any unperformed literal cable-removal test explicitly rather than
   implying it passed.
