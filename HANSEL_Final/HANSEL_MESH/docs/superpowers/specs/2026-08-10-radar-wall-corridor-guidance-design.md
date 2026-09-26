# Radar Two-Wall Corridor Guidance Design

## Goal

Use the calibrated short-memory reflectivity surface to recognize the two
visible sides of a corridor, show both observed wall segments clearly, and
guide the local path toward the corridor direction. Reflectivity geometry is a
soft heading cue only. Confirmed point and probability evidence remain the
only inputs allowed to declare a path collision-free.

## Live Evidence

The unchanged right-diagonal arrangement was sampled for 119 complete frames.
Every frame contained 44 stable reflectivity cells and the same dominant wall
axis at +63.435 degrees. A deterministic two-line fit found one 0.57 m segment
at +63.4 degrees and one 0.41 m segment at +67.4 degrees, separated by about
0.193 m. The hard planner nevertheless received only one occupied cell near
0.10 m forward and selected the zero-degree fallback in every frame. This
proves the fault is the boundary between display geometry and goal selection,
not absence of diagonal radar evidence.

## Separation of Responsibilities

The existing `reflectivity_surface` remains excluded from probability updates,
hazard latching, and collision membership. A new corridor estimator consumes
only its already persistent three-of-five cells and publishes a separate
`reflectivity_corridor` geometry product. The path planner may use a confirmed
corridor heading to choose a goal direction, but it must still test every path
sample against the unchanged confirmed-obstacle collision grid.

This revises the earlier absolute display-only rule only for goal orientation;
it does not promote reflectivity cells to occupied or free map cells.

## Two-Wall Estimator

The estimator performs a deterministic robust line search over stable cells:

- minimum cell intensity: 0.30;
- point-to-line tolerance: 0.026 m, approximately one surface cell;
- individual wall support: at least four cells and 0.14 m span;
- wall heading: within the trusted +/-75 degree fitting range;
- paired walls: heading difference at most 10 degrees;
- measured normal separation: 0.14--0.55 m;
- paired fits may share at most one support cell.

Candidate pairs are ranked by unique support intensity and observed span. Wall
endpoints are the extrema of each inlier set projected onto its fitted line;
the renderer must not extrapolate them into unobserved space. Wall order is
canonical by signed normal offset, and direction is normalized to have a
positive forward component.

The estimator keeps five complete frame results. Three mutually compatible
fits are required for `confirmed`; one or two compatible fits are `candidate`.
Dropped or missing frames contribute empty slots. Compatibility requires
heading within 5 degrees and width within 0.05 m. A confirmed result disappears
when it no longer satisfies the three-of-five rule.

The scene payload contains status, measured and guidance headings, measured
width, confidence, and two wall endpoint records. Guidance heading is clamped
to +/-55 degrees so the goal remains inside the trusted +/-60 degree planner
field of view.

## Path Integration

The hard-obstacle free-angle computation remains authoritative. When a
confirmed corridor is available, its guidance heading biases free-interval
selection and the goal angle within the selected interval. If hard obstacles
cover every interval, the planner reports `blocked_near_field` but retains the
corridor heading as the stopped opening direction. It must never turn a blocked
result into `clear` merely because a corridor fit exists.

Repeated stopped candidates refresh their current goal and corridor metadata
without counting a new accepted route switch. This prevents an old straight or
left stopped goal from freezing while new stable wall geometry points right.

## Operator Rendering

The main and collision views draw the two observed wall segments above the
grayscale reflectivity cells and below the path and confirmed tracks. Each wall
uses a dark underlay plus a crisp pale-cyan core. The region between their
observed endpoints receives a faint fill only; it is labelled `MEASURED
CORRIDOR`, never `FREE` or `SAFE`.

Confirmed walls are solid. Candidate walls are dashed and cannot influence the
planner. The measured width is shown in centimeters. A width below the 0.21 m
robot-plus-margin requirement is labelled `NARROW`, and the existing collision
logic remains responsible for stop/go status. The planned center path remains
yellow; a blocked heading remains dashed and explicitly labelled `STOP`.

## Validation

- Pure estimator tests cover the recorded right-diagonal geometry, canonical
  wall ordering, two-line support, three-of-five confirmation, misses, drops,
  reset, deterministic output, and rejection of random or single-wall input.
- Scene tests prove a confirmed +63--67 degree pair yields a positive guidance
  goal while a central 0.10 m obstacle still returns `blocked_near_field`.
- Regression tests prove reflectivity cells do not change probability cell
  counts, hazard state, or collision membership.
- Selector tests prove a stopped route refreshes heading metadata without a
  switch-count storm.
- Parser tests strictly validate and detach every corridor field and endpoint.
- Renderer tests verify two lines, candidate dashes, measured-width labelling,
  layer order, and right-side projection.
- The UI build identifier is bumped and the live board gate requires stable
  frames, a confirmed two-wall fit, a positive goal angle, and no new transport
  gaps. A persistent near point may keep the final route stopped.

## Non-Goals

- Declaring reflectivity-only space collision-free.
- Suppressing the current confirmed 0.09--0.10 m point to force motion.
- Inventing a second wall when only one wall is observed.
- Global SLAM, IMU/Doppler odometry, servo pitch transport, or elevation
  reconstruction.
