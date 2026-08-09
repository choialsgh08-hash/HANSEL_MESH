# Radar Ground Suppression Design

## Goal

Remove traversable floor returns from the operator view and every driving
decision while retaining the untouched sensor frame in mission recordings for
diagnosis. The first hardware binding is an IWRL6432BOOST mounted at the front
of the robot with its center 0.065 m above the ground and facing horizontally.

## Scope

- Convert detected 3D points from the sensor frame into a robot/ground frame.
- Classify near-field points at or below the traversable ground allowance as
  ground.
- Exclude ground points before tracking, map updates, path planning, hazard
  evaluation, and operator rendering.
- Keep raw point TLVs unchanged in recordings and the diagnostic API.
- Expose a ground-rejection count so filtering can be audited during hardware
  tests.
- Bind the TI profile's `sensorPosition` height to 0.065 m and zero tilt.

Explicit body and bumper exclusion volumes are deferred until the final body
geometry exists. The existing profile-bound self-clutter model remains active.
Wheel step height is not modeled because the articulated head and high-torque
servo make floor shape a lower-priority driving constraint for this prototype.

## Coordinate Model

Host configuration uses:

- `sensor_height_m = 0.065`
- `sensor_pitch_deg = 0.0`
- `ground_rejection_height_m = 0.035`

Host pitch is positive when the radar looks upward. For sensor-frame forward
coordinate `f`, height coordinate `z`, and pitch `p`:

```text
robot_forward = cos(p) * f - sin(p) * z
robot_height  = sensor_height + sin(p) * f + cos(p) * z
```

Lateral position is unchanged. The current zero-pitch case therefore maps the
ground plane to sensor height `z = -0.065 m`. A future servo integration only
needs to replace the pitch input; the filtering and downstream contracts stay
unchanged. TI's `elevTilt` uses the opposite sign convention (positive toward
the ground), so a future upward host pitch maps to a negative TI elevation
tilt. Both are zero for this prototype.

## Ground Classification

A finite 3D point is classified as ground when:

1. its transformed forward/lateral position is within the local 0.60 m driving
   window; and
2. its transformed height is at or below 0.035 m above the ground plane.

The 0.035 m allowance covers the observed near-field elevation error and small
surface irregularities the robot is intended to traverse. Points above this
allowance remain obstacle candidates even if they are below the radar center.
The filter does not infer drop-offs or claim that unobserved space is free.

Ground classification runs after point decoding and axis mapping but before
point association and all scene evidence updates. It must not mutate or remove
the original frame points.

## Data Flow

```text
raw frame/recording
  -> axis mapping and finite validation
  -> robot-frame transform
  -> ground classification
       -> ground: diagnostic counter only
       -> non-ground: quality/self-clutter gates -> tracking
  -> confirmed non-ground evidence
  -> probability map / reactive occupancy / path / 10 cm hazard / renderer
```

Heatmap cells do not contain height. They may support a hard obstacle only when
correlated with a non-ground 3D point. A ground point removed before correlation
cannot confirm a heatmap obstacle. Heatmap-only reflection evidence remains
available for internal diagnostics but is not drawn as obstacle geometry in
the primary operator view.

## Operator and Diagnostic Contract

- Ground points are not rendered, including as tracks, contours, probability
  cells, path obstacles, or collision markers.
- The raw frame remains available to replay and hardware-analysis tools.
- Scene diagnostics add `ground_rejected_count` for the current frame.
- Mount diagnostics expose the configured sensor height, pitch, rejection
  height, and a ground-frame calibration status.
- Existing danger thresholds remain 0.10 m enter and 0.13 m release. Only
  confirmed non-ground point tracks can trigger danger.

## Validation and Failure Handling

- Mount height, pitch, and rejection height must be finite; height values must
  be non-negative and the rejection height must be lower than the sensor
  height.
- Invalid geometry configuration fails during construction rather than
  silently disabling filtering.
- A missing or non-finite point height is not classified as ground and follows
  the existing conservative obstacle path.
- Hardware validation compares the ground-rejection count and non-ground
  tracks while the fixed board views the current scene, then repeats with a
  raised obstacle at the same horizontal distance.

## Tests

Automated tests will prove:

- zero-pitch and nonzero-pitch robot-frame transforms;
- floor points near `z = -0.065 m` remain in raw data but never create tracks,
  maps, paths, or danger;
- a point at the same 2D distance but above the 0.035 m allowance is retained;
- ground-correlated heatmap evidence cannot become a hard obstacle;
- invalid mount configuration fails closed;
- parser and operator-view contracts accept and expose the new diagnostics
  without drawing ground geometry;
- existing non-ground tracking, 10 cm hazard, replay, and HTTP suites remain
  green.

## Non-Goals

- Automatic ground-plane fitting from sparse radar points.
- Wheel climb or terrain-cost modeling.
- Final body/bumper exclusion geometry.
- Live servo-angle transport in this change.
- Free-space proof, negative-obstacle detection, or camera-like reconstruction.
