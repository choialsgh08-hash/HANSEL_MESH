# Radar transient scene hold design

## Goal

Prevent a single transient scene-contract error from blanking the operator map, without weakening radar-scene validation or delaying real sensor and collision faults.

## Observed cause

`RadarPanel.acceptSnapshot()` currently converts every parser exception directly into `invalid_scene`. The render path then clears all map, path, sector, and diagnostic state immediately. Live static sampling and replayed recent frames remain valid, so the repeated operator-visible message is caused by the UI treating a short transition error as a persistent scene failure.

## Behavior

- Keep the most recent valid presentation for the first two consecutive `invalid_scene` snapshots.
- During that hold, keep the map visible and show a non-blocking `SCENE VERIFYING 1/3` or `2/3` status.
- Accept the next valid snapshot immediately and clear the transient counter.
- On the third consecutive invalid snapshot, switch to the existing blocking `invalid_scene` presentation.
- Never apply the hold to HTTP loss, `waiting`, `stale`, `fault`, `sensor_fault`, replay end, calibration failure, profile mismatch, or any valid scene carrying a collision hazard. Those states remain immediate.
- Never reuse a held scene after its source frame is older than 500 ms.
- Do not relax or bypass `parseRadarScene()` validation.

## State and rendering

`RadarPanel` owns three small pieces of state: the last valid presentation, its acceptance time, and the consecutive invalid-scene count. `acceptSnapshot()` remains the only transition boundary. The existing blocking render path is unchanged once the threshold or age limit is reached.

The non-blocking hold exposes a `verifyingScene` marker on the presentation. Text rendering uses it only for the status badge and warning strip; map, path, hazard, and diagnostics come from the last valid presentation.

## Tests

- One and two parser failures retain the last valid presentation and show verifying state.
- A valid frame between failures resets the counter.
- The third consecutive failure blocks with `invalid_scene`.
- A held presentation older than 500 ms blocks immediately.
- HTTP and sensor faults bypass the hold.
- Existing blocked-state, map, path, hazard, parser, and live-producer tests remain green.
