# Independent Radar Boundary Display Design

## Problem

The current reflectivity estimator serializes wall geometry only after two
opposing walls form a validated corridor.  The live right-hand wall is present
in the reflectivity surface as a strong five-cell line (about -58 degrees and
0.29 m long), but one 0.090 m sampling gap exceeds the 0.080 m continuity
limit.  Even if that line passes, it is discarded when no matching wall exists.

## Design

- Increase the independent wall continuity allowance to 0.100 m.  Keep the
  existing support, span, residual, intensity, topology, and temporal checks.
- Produce a separate `reflectivity_boundaries` scene product for zero, one, or
  two independently measured walls.
- A boundary is visible only after compatible evidence appears in at least
  three of five frames.  Boundary-only admission also requires strong
  reflectivity so the relaxed continuity limit does not expose random scatter.
- Render confirmed independent boundaries as measured cyan wall segments.  If
  a confirmed two-wall corridor already contains the same geometry, suppress
  the duplicate boundary layer.
- Keep independent boundaries display-only.  Collision, hazard, and path
  planning continue to use point evidence, the probability map, and validated
  two-wall corridors.  Never synthesize a missing opposite wall.

## Transient Scene Validation

Implement the separately approved parser-failure hold: retain the most recent
valid presentation for at most two consecutive invalid snapshots and at most
500 ms.  A third invalid snapshot or the age limit blocks immediately.  Sensor,
HTTP, calibration, stale, waiting, and hazard states remain fail-closed.

## Verification

- A five-cell diagonal wall containing a 0.090 m gap confirms after three
  frames and is available to the browser while the corridor remains unavailable.
- Mirroring the wall mirrors the serialized geometry.
- Dense varied scatter does not produce a confirmed independent boundary.
- Parser and renderer reject malformed boundary payloads and draw confirmed
  single walls in both top views.
- A valid snapshot resets transient invalid-scene hold state; safety and
  transport failures never use the hold.
