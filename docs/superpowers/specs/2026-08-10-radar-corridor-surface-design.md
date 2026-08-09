# Radar Corridor Surface and Anisotropic Planning Design

## Goal

Make a right-diagonal corridor recognizable in the radar-only driving screen
without allowing visually useful but unconfirmed heatmap energy to create a
false collision or safe-space claim. Reduce the planner's present circular
over-inflation while preserving the 0.15 m robot width, 0.03 m safety margin,
and 0.10/0.13 m danger latch. Extend ground suppression across the observed
3.5--5 cm measurement fringe.

## Observed Baseline

The fixed hardware arrangement was sampled for 100 complete live frames. All
frames were live, but the path was `blocked_near_field` with exactly one search
expansion in every frame. The point cloud averaged 10.8 returns per frame and
only 1.75 raw returns inside 0.50 m, so point markers cannot recover a wall
outline. The calibrated heatmap had 215 candidate cells in a representative
frame and three persistent near-field clusters. Two were point-correlated hard
obstacles; the largest cluster at about 0.41 m forward and 0.06 m right was
stable but unconfirmed and therefore invisible to the operator.

The largest collision radius was 0.358 m because a 0.20 m radial uncertainty
was converted to an isotropic circle. One repeated point was at about 0.095 m
with a transformed height near 0.041 m, only 6 mm above the configured 0.035 m
ground rejection height.

## Architecture

The scene will carry two deliberately separate spatial products:

1. `reflectivity_surface` is a display-only, calibrated, short-memory raster
   built from heatmap residual cells before cluster-centroid reduction. It may
   show stable unconfirmed reflections, but it is never consumed by hazard,
   probability occupancy, or path planning.
2. `probabilistic_local_map` remains the conservative driving map. Only
   confirmed points and point-correlated heatmap evidence may enter it.

This separation lets the operator see surfaces without silently weakening the
hard driving contract.

## Reflectivity Surface

Candidate heatmap residual cells are quantized into a 0.025 m Cartesian grid.
The surface keeps five complete frame slots and publishes a cell only after it
appears in at least three slots. Dropped frames insert empty slots, so transport
loss cannot manufacture persistence. Each published cell contains forward and
lateral position, normalized intensity, persistence, hit count, and age.

The payload contract is:

```text
reflectivity_surface:
  radius_m: 0.50
  resolution_m: 0.025
  history_frames: 5
  required_hits: 3
  retention: "three_of_five_complete_frames"
  pose_mode: "sensor_relative_short_memory"
  source: "calibrated_heatmap_residual"
  cells: [{forward_m, lateral_m, intensity, persistence, hits, age_ms}]
```

The renderer draws this product first as crisp grayscale cells and exposed
cell boundaries. It does not draw glow, trails, filled free space, or connect
cells across a gap. Confirmed probability cells, tracks, robot footprint, and
the planned path are drawn above it. Heatmap track centroids remain hidden
unless confirmed, because their display role is replaced by the richer surface.

## Ground Fringe

The physical allowance remains 0.035 m. A separate 0.015 m measurement fringe
extends the effective rejection height to 0.050 m for this 0.065 m-high mount.
Points at or below 0.035 m are classified `ground`; points above 0.035 m and at
or below 0.050 m are classified `ground_fringe`. Both are excluded before
tracking, heatmap point-correlation, maps, path, hazard, and rendering. The two
counts are exposed separately so hardware tests can distinguish definite floor
from uncertainty suppression. Raw recordings remain untouched.

The configuration must reject non-finite or negative fringe values and any
geometry where physical allowance plus fringe reaches the sensor center.

## Anisotropic Collision Geometry

Planning obstacles use a radial/tangential ellipse centered on each measured
obstacle instead of a circle formed from the maximum uncertainty. The radial
semi-axis is robot half-width plus safety margin, support radius, and radial
uncertainty. The tangential semi-axis uses the same physical base plus lateral
uncertainty. Grid padding is added to each axis only for raster collision.

Heatmap cluster range uncertainty represents range-bin localization, not the
full spatial extent of the cluster. It is therefore computed from one range
bin rather than cluster radius. The cluster extent remains visualized by the
individual reflectivity cells. Exact path postconditions use the same ellipse
as raster planning so a smoothed path cannot cross a boundary accepted by one
representation and rejected by the other.

Diagnostics retain the existing maximum scalar fields for compatibility and
add maximum radial and tangential collision semi-axes. The old field is the
maximum of those two axes, not a separate collision rule.

## Diagonal Corridor Selection

The existing hybrid-lattice selector is evaluated only after origin collision
geometry is correct. A deterministic regression builds two diagonal walls with
a corridor center that moves to positive lateral coordinates as forward range
increases. The accepted path must expand beyond the origin, end right of center,
and have a positive goal angle while maintaining the robot-width clearance.

Live hardware may still report `blocked_near_field` when a confirmed point is
truly inside the 0.10 m danger threshold. The implementation will not suppress
such a point merely to force a route; fixture/self-clutter calibration remains
the correct follow-up if the same return persists in an empty mounted capture.

## Validation

- Pure tests cover three-of-five surface admission, dropped/missed-frame
  removal, reset, deterministic ordering, and finite payload values.
- Scene tests prove that stable unconfirmed heatmap residuals appear only in
  the display surface and never affect hazard or path.
- Web parser mutation tests reject malformed surface metadata and cells.
- Renderer tests prove layer order and ensure no surface cell is treated as a
  track or probability obstacle.
- Ground tests cover the exact 0.035 m and 0.050 m boundaries and separate
  diagnostics.
- Planner tests cover radial/tangential membership, grid padding, exact-path
  consistency, and the right-diagonal synthetic corridor.
- Full Python, Node, syntax, compile, diff, and live API smoke checks run before
  completion. Hardware results are reported as observations, not proof of
  camera- or LiDAR-equivalent sensing.

## Non-Goals

- Inferring elevation from the range-azimuth heatmap.
- Treating unknown or weak reflection space as confirmed free space.
- Hiding arbitrary confirmed near points to make a path appear.
- Final bumper/body masks, servo pitch transport, Doppler odometry, IMU fusion,
  or global SLAM.
