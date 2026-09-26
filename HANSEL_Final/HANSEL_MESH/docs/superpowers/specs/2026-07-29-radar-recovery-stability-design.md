# Radar Recovery Stability Design

**Date:** 2026-07-29
**Scope:** IWRL6432BOOST/XDS110 radar stack only

## Problem

The current reconnect loop is not caused by point-count, heatmap-quality, or
parser thresholds. Runtime evidence shows that valid 10 Hz frames and raw UART
bytes stop completely while the XDS110 Application/User COM port and capture
process remain present.

Recent failed epochs produced 0, 1, 53, 217, 3,542, or 10,708 complete frames
before the UART stream stopped. Parser errors and writer drops were zero. The
watchdog correctly reported `radar_frame_timeout` about 2.5 seconds after the
last decoded frame.

The visible one-second flashing is amplified by a separate problem: five valid
frames, about 0.5 seconds at 10 Hz, currently promote a recovering epoch to
`RUNNING` and switch the viewer to it.

## Goals

- Do not expose a briefly revived but unstable epoch as a live driving view.
- Preserve fail-closed behavior whenever fresh radar evidence is unavailable.
- Continue automatic recovery with bounded, increasing retry delays.
- Update the old XDS110 firmware using the exact connected probe identity.
- Preserve the 10 Hz, 16-by-128 heatmap operator profile and current map UI.

## Non-goals

- Do not display stale radar evidence as current or free space.
- Do not hide UART stalls by increasing the UI stale/fault thresholds.
- Do not change collision thresholds, scene estimation, clutter calibration,
  or non-radar folders.
- Do not reduce frame rate or heatmap resolution in this change.

## Design

### 1. Recovery probation

A new radar epoch must produce 30 consecutive qualifying frames within five
seconds before the supervisor switches the viewer and publishes `RUNNING`.
At the configured 10 Hz rate this requires approximately three seconds of
continuous complete point-cloud and 16-by-128 heatmap evidence.

The existing qualification contract remains unchanged:

- complete frame;
- exact profile ID;
- expected heatmap dimensions;
- expected heatmap range step.

The first-frame deadline remains three seconds. The running frame timeout
remains 2.5 seconds. The viewer continues to mark evidence stale after
0.75 seconds and faulted after two seconds, so no old map is presented as live.

### 2. Stability-aware recovery backoff

Every faulted running epoch enters a bounded recovery backoff before another
target reset:

1. 0.5 seconds
2. 1 second
3. 2 seconds
4. 4 seconds
5. 5 seconds for later consecutive failures

Promotion to `RUNNING` does not immediately reset this delay. The retry delay
returns to 0.5 seconds only after one epoch has remained healthy for at least
30 continuous seconds. A short-lived epoch therefore cannot restart the rapid
reset loop.

Shutdown requests remain interruptible during every backoff. The viewer keeps
serving the fail-closed recovery state while the new epoch is verified.

### 3. XDS110 firmware update

After the code behavior passes automated tests:

1. Record the current supervisor manifest, connected XDS110 serial, COM ports,
   and firmware version.
2. Stop only the owned radar supervisor, capture, and viewer processes.
3. Confirm that the selected probe is the same XDS110 serial (`RI32`).
4. Use TI's installed XDS110 firmware utility and the bundled
   `firmware_3.0.0.43.bin`.
5. Re-enumerate the probe and verify firmware `3.0.0.43` before restarting.
6. Restart the radar stack with the same profile and calibration file.

If identity verification, flashing, re-enumeration, or version verification
fails, leave the radar stack stopped and report the exact recovery state.
Do not flash an ambiguous device and do not claim a successful update based
only on command exit status.

## Error handling

- A missing or ambiguous Application/User port remains fail-closed.
- A UART stall still triggers `radar_frame_timeout`; probation does not weaken
  the watchdog.
- Firmware update failure does not fall back to an unverified running stack.
- Existing immutable epoch artifacts and manifest history remain preserved.
- The browser must never alternate to `LIVE` for an epoch that has not passed
  probation.

## Testing

Automated tests must prove:

- five qualifying frames no longer promote a recovering epoch;
- 30 consecutive qualifying frames within five seconds do promote it;
- one bad frame resets the consecutive-frame count;
- short running epochs preserve and increase the recovery delay;
- 30 seconds of healthy running resets the delay to 0.5 seconds;
- recovery backoff remains bounded and shutdown-interruptible;
- stale and faulted API responses remain fail-closed;
- existing process ownership and immutable epoch guarantees still pass.

The test must be observed failing before each production change and passing
after the minimal implementation.

## Hardware acceptance

After the firmware update and stack restart:

- XDS110 firmware reports `3.0.0.43`;
- one epoch remains active for at least 15 minutes;
- no `radar_frame_timeout` or unexpected recovery occurs;
- API frame numbers increase near 10 Hz;
- parser errors and writer drops remain zero;
- COM3/COM4 remain present;
- the browser stays out of the reconnect loop.

Any remaining UART stall is reported rather than masked. If the 15-minute soak
fails, the next separately approved fallback is a lower-bandwidth radar
profile followed by a new clutter calibration.
