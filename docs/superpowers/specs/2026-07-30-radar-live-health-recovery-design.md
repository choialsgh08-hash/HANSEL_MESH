# Radar Live Health Recovery Design

**Date:** 2026-07-30

## Goal

Keep one-off UART corruption visible in mission diagnostics without making the
operator view look permanently disconnected after the live stream has
recovered.

## Evidence

The current hardware run stayed in supervisor epoch 1 with recovery count 0.
One device frame, 20796, was absent after frame 20795. Frame 20797 arrived
203.5 ms later and frame 20798 followed 101 ms after that. The decoder
discarded 5,318 additional post-sync bytes while finding the next TI magic
word. The producer, boot, capture process, viewer process, and supervisor
process did not change. More than 6,000 consecutive frames arrived after the
event at approximately 10 Hz.

The apparent persistent failure is therefore a status-model problem, not a
supervisor reconnect loop. Capture health currently derives `degraded` from
lifetime counters, and the viewer deliberately latches that state for its
entire process lifetime.

## Considered approaches

1. Lower the supervisor probation or frame timeout. Rejected because the
   watchdog did not fault and no reconnect occurred. This would weaken
   fail-closed recovery without affecting the sticky badge.
2. Clear degradation on the next good radar frame. Rejected because a
   short corruption could disappear between browser polls and because it
   could clear unrelated mission-log integrity failures.
3. Separate current health from cumulative evidence. Selected. A corrupted
   interval remains visibly degraded, a later quiet interval explicitly
   recovers the live view, and all lifetime counters remain available.

## Capture health

`capture_radar_uart` continues to publish cumulative values for frame gaps,
parser faults, discarded bytes, incomplete frames, missing TLVs, writer
drops, and device discontinuities.

Periodic `SensorHealth.status` describes only the interval since the last
successfully submitted periodic health record:

- `degraded` when any fault counter increased during the interval;
- `ok` when frames are present, point-cloud output is usable, and no fault
  counter increased;
- existing `starting` behavior remains for a capture with no decoded frame.

The interval baseline advances only after `MissionLogWriter.submit` accepts
the health record. A rejected health record increments `writer_drops`, and a
later successful record must still report a degraded interval.

Final health remains cumulative. Any fault anywhere in the capture keeps the
final record degraded, preserving whole-mission audit behavior.

## Viewer health

`RadarFrontState` keeps two independent active reasons:

- capture-derived degradation from sensor health or radar-frame anomalies;
- viewer-integrity degradation from invalid JSON records or mission-log
  sequence discontinuities.

A periodic `SensorHealth(status="ok")` clears only the capture-derived
reason. It never clears viewer-integrity failures. Counters use their existing
maximum/additive lifetime semantics and are never reset.

Consequently, the top-level API returns `live` after explicit capture recovery
while `counters.frame_gaps_total` still reports the historical gap. Persistent
missing point-cloud output remains degraded through the existing current-frame
check.

## Supervisor and operator safety

The supervisor remains unchanged:

- 30 consecutive qualifying frames are required before a new epoch is shown;
- a running epoch faults after 2.5 seconds without a radar frame;
- retry backoff resets only after 30 healthy running seconds.

The browser continues to block `waiting`, `stale`, `fault`, and supervisor
non-running states. `degraded` remains renderable. After a quiet capture
interval the badge returns to `LIVE`, while the diagnostics panel retains the
nonzero gap count.

## Verification

Automated tests must prove:

1. a gap produces a degraded periodic health record;
2. a later quiet interval produces `ok` with the cumulative gap still equal
   to one;
3. final health remains degraded for the whole artifact;
4. viewer status changes from degraded to live on explicit recovery health
   without clearing counters;
5. recovery health cannot clear invalid-log or log-sequence degradation;
6. existing supervisor, capture, viewer, and browser regressions remain green.

Live verification will deliberately restart only the manifest-owned capture
child so the existing supervisor exercises one automatic recovery epoch. The
new epoch must reach `RUNNING`, produce fresh complete frames near 10 Hz, and
remain `LIVE` with no increasing error counters. No unrelated repository
folders or robot-control processes are in scope.
