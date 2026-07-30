# IWRL6432 Radar Automatic Recovery Design

Date: 2026-07-29

## 1. Goal

Keep the IWRL6432BOOST operator view usable without requiring a person to
repeatedly press RESET or restart capture and viewer processes.

The system must:

- prevent the currently observed low-power firmware assert while preserving
  the 10 Hz operator-view update rate;
- detect loss of real decoded radar frames, not merely process exit or mission
  file activity;
- fail closed while the radar is unavailable;
- reset and reconfigure the selected XDS110-connected board automatically;
- resume the viewer only after fresh complete radar evidence is verified; and
- preserve every capture epoch instead of overwriting prior evidence.

This is a Windows prototype design. The recovery core must keep the hardware
reset action injectable so a Raspberry Pi deployment can later use an
appropriate Linux/XDS110 utility or a dedicated reset GPIO.

## 2. Confirmed Failure

The current live capture stopped after 1,221 complete, consecutive frames.
The capture process remained alive and continued writing periodic health
records, but no new radar frames or raw UART bytes arrived.

The raw capture ends with:

```text
Error: No Sufficient Time for getting into Low Power Modes.
```

The matching TI MMWAVE-L-SDK 05.05.04.02 source prints that message and then
calls `DebugP_assert(0)` when low-power mode is enabled but the remaining
frame idle time is insufficient.

The active profile uses:

```text
frameCfg 2 8 600 16 100 0
lowPowerCfg 1
```

The final `frameCfg` value is `NumOfFrames`; zero means an infinite stream.
The stop was therefore not a finite-frame setting. TI's bundled profile with
the same chirp pattern uses a 200 ms frame period with low power enabled. The
current 100 ms period plus point-cloud and heatmap output leaves substantially
less idle time.

## 3. Chosen Approach

Use a separate, testable radar stack supervisor rather than embedding device
configuration and process lifecycle management into the capture parser.

This preserves the existing responsibility boundaries:

- `configure_ti_radar` applies one pinned board profile;
- `radar-live` captures one immutable device epoch;
- `radar_front` renders one followed mission log; and
- the new supervisor owns recovery, epoch rotation, and child processes.

An in-process reconnect inside `capture_radar_uart` was rejected for this
phase because it would mix configuration, hardware reset, parsing, raw index
continuity, and mission-log lifecycle in one large loop. A configuration-only
change was also rejected because it would not recover from USB removal or
future device stalls.

## 4. Prevention Change

Change only the active 10 Hz near-range 3D operator profile:

```text
configs/radar/iwrl6432_3d_operator_near_10hz.cfg
```

from `lowPowerCfg 1` to `lowPowerCfg 0`.

The 100 ms frame period, chirp pattern, heatmap dimensions, elevation FFT,
CFAR settings, and range selection remain unchanged. This trades additional
power consumption and heat for stable 10 Hz operator-view operation. Thermal
behavior must be observed during the real-board soak test.

Other TI example and archived profiles are not changed.

## 5. Components

### 5.1 Reusable TI control module

Move the reusable profile parsing and baud-switch control from
`scripts/configure_ti_radar.py` into a sensor-side module. Keep the existing
script as a compatible thin command-line wrapper.

The module provides:

- profile loading and partitioning at `baudRate`;
- cold profile application from 115,200 to 1,250,000 baud;
- strict success validation;
- Application/User UART discovery using pyserial metadata; and
- an injectable target-reset action.

Configuration is successful only when all expected commands complete, the new
baud prompt is established, and either the first TI magic word is observed or
the supervisor subsequently verifies complete frames within its first-frame
deadline.

An exit code of zero alone is not sufficient.

### 5.2 Radar stack supervisor

Add a testable supervisor core and a thin executable entry point. The
supervisor starts and monitors the existing configuration, capture, and viewer
programs.

The state machine is:

```text
WAIT_PORT
  -> RESET_TARGET
  -> CONFIGURE
  -> START_CAPTURE
  -> VERIFY_FRAMES
  -> SWITCH_VIEWER
  -> RUNNING
  -> RECOVERING
  -> WAIT_PORT
```

All retry delays use bounded exponential backoff, starting near 0.5 seconds
and capped at 5 seconds. Retries continue until the operator stops the
supervisor.

### 5.3 XDS110 target reset

On Windows, discover or accept an explicit path to TI's
`xds110reset.exe`. Always pass the selected probe serial number:

```text
xds110reset.exe -a toggle -d 100 -s RI32
```

The supervisor must never reset an unspecified "first" probe when a serial
number is available. The current board identity is:

- VID:PID `0451:BEF3`;
- XDS110 serial `RI32`;
- Application/User UART description;
- interface/location ending in `.0`.

The Auxiliary Data Port, currently COM4 and interface `.3`, is never selected.
If Windows assigns a different COM number after reconnect, the metadata match
finds the Application/User port again.

If no reset tool is available, physical USB removal and reappearance can still
recover automatically. A silent target assert remains fail-closed with a
clear `reset tool unavailable` reason until a reset mechanism becomes
available.

### 5.4 Per-epoch artifacts

Each capture/recovery epoch receives new timestamped paths:

```text
missions/radar-board-live-<run>-eNNN.jsonl
captures/radar-board-live-<run>-eNNN.bin
captures/radar-board-live-<run>-eNNN.bin.chunks.jsonl
```

No epoch overwrites an earlier one. A small supervisor manifest records the
current epoch, paths, start/end reason, recovery count, reset result, and
configuration result. This manifest is not treated as radar measurement
evidence.

The old viewer remains on its faulted epoch while the replacement capture is
being verified. Only after verification succeeds is the viewer restarted on
the new mission path while retaining port 8081. The already-loaded browser
page polls continuously, remains blocked on the old `SENSOR_FAULT` state,
briefly shows HTTP loss during the viewer switch, and resumes when the server
returns.

## 6. Fault Detection

The supervisor updates freshness only when it decodes a valid `RadarFrame`
record from the mission log.

The following trigger recovery:

- no valid radar frame within the 3-second initial first-frame deadline;
- no valid radar frame for 2.5 seconds while running;
- UART/capture child exit;
- viewer-independent firmware ASCII error detection in the raw stream,
  including the confirmed low-power timing message;
- selected Application/User port disappearance; or
- configuration or verification failure.

Mission file size, mission modification time, periodic `SensorHealth` records,
and child-process existence are not freshness evidence. The current failure
proved that all of those can continue while radar frames have stopped.

## 7. Recovery Flow

When a fault is detected:

1. Keep or enter the blocking `SENSOR_FAULT`/reconnecting presentation.
2. Stop the capture child gracefully where possible.
3. Keep the old viewer serving its faulted epoch whenever it is still alive.
4. Close the UART owner before invoking target reset.
5. Toggle the selected XDS110 target reset.
6. Wait for the matching Application/User UART to reappear.
7. Apply the pinned 10 Hz, low-power-disabled profile.
8. Start a new capture epoch without exposing it to the operator yet.
9. Require five consecutive complete radar frames with the configured
    heatmap, all within a 3-second verification deadline, before considering
    recovery successful.
10. Stop the old viewer and start the viewer on the verified mission path.

Any incomplete frame, wrong profile, missing expected heatmap, parse failure,
or renewed timeout resets the consecutive-frame count and returns to
recovery.

The first verified frame belongs to a new producer/device epoch. Old scene
evidence is not carried across the reset.

## 8. Safety Presentation

During `WAIT_PORT`, `RESET_TARGET`, `CONFIGURE`, `VERIFY_FRAMES`, and
`RECOVERING`:

- movement guidance is blocked;
- previous occupancy and tracks are not presented as current evidence;
- collision status is `SENSOR_FAULT`, not `NORMAL`; and
- the operator sees a clear reconnecting/stop message.

The display returns to live operation only after the five-frame verification
gate. Calibration and pose validation continue to fail closed under the
existing scene contract.

## 9. Command-Line Contract

The stack launcher accepts, at minimum:

- profile path;
- explicit port or automatic Application/User UART selection;
- optional XDS110 serial;
- optional reset executable/command;
- calibration path;
- output root/run name;
- frame timeout and first-frame timeout;
- verification frame count;
- HTTP bind and port; and
- existing capture/heatmap/profile identifiers.

Defaults target the current IWRL6432BOOST R9 operator profile but remain
visible in `--help`. Test-only process, clock, port enumeration, and reset
dependencies are injectable; production code does not depend on mocks.

## 10. Test Strategy

Unit tests use fake clocks, port inventories, reset actions, and child
processes.

Required tests:

- the active 10 Hz profile uses `lowPowerCfg 0`;
- COM3 Application/User is selected and COM4 Auxiliary is excluded;
- a renumbered Application/User port with the same identity is selected;
- a healthy frame stream does not reset;
- child exit triggers recovery;
- silent empty reads/health-only log growth trigger recovery after the last
  real frame;
- the confirmed firmware ASCII error triggers immediate recovery;
- reset is scoped to the requested XDS110 serial;
- reset/configuration failures retry with bounded backoff;
- exit code zero with an unverified configuration is rejected;
- five consecutive complete heatmap frames are required;
- a bad frame resets the verification count;
- each recovery creates new artifacts and preserves older files;
- viewer restart retains port 8081;
- the browser presentation remains blocked during HTTP loss and sensor fault;
  and
- shutdown terminates owned children without targeting unrelated processes.

Integration tests run the supervisor with simulated subprocesses and
disconnect/reconnect events before touching the real board.

## 11. Real-Board Acceptance

After automated tests pass:

1. Reset the current asserted target with the selected XDS110 probe.
2. Start the stack using the new supervisor.
3. Verify 10 Hz capture, heatmap availability, profile identity,
   calibration status, and zero frame gaps/parser errors.
4. Run at least a 15-minute stationary/motion soak and confirm that the
   low-power timing assert does not return.
5. Unplug and reconnect USB once.
6. Confirm the UI becomes blocking within 2.5 seconds.
7. Confirm automatic reset/configuration/capture/viewer recovery without
   pressing the board RESET button or restarting commands.
8. Confirm the UI unlocks only after five good frames.
9. Confirm a new epoch was created and old artifacts remain unchanged.

Target recovery time after the matching port reappears is no more than
15 seconds under normal Windows driver behavior.

## 12. Non-Goals

This change does not:

- claim SLAM or global pose continuity across a board reset;
- merge capture epochs into one raw binary;
- alter collision distance thresholds or scene geometry;
- change the radar heatmap/point-cloud visualization;
- implement the final Raspberry Pi reset transport; or
- treat the current temporary clutter calibration as a production
  calibration.

Future SLAM ingestion must treat each radar producer epoch explicitly and use
odometry/IMU constraints to bridge gaps when valid.
