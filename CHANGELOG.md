# Changelog

## 0.1.0 (unreleased)

First version: a pure-Python port of the certified continuous collision
checker of Drake PR #24924 (`certified-ccd` at `97d9d2cb08`) onto the stock
`drake` wheel (1.51.1 or later).

- `ContinuousCollisionChecker`, `Options`, `Result`, `Finding`, `Verdict` with
  the fork's names, defaults, error conditions and resolution contract.
- Trajectory normalization of `BezierCurve`, `BsplineTrajectory`,
  `PiecewisePolynomial` and `CompositeTrajectory` into Bézier segments.
- The fork's motion bound: subtree tables, reaches, lambda per joint type, the
  constant-coordinate carve-out and its residual, bounding spheres per shape.
- Distances: Drake's narrowphase for general shape pairs; exact closed forms
  for half-space (the fork's fallback), sphere-sphere, sphere-box and box-box
  pairs; a separating-axis lower bound for box-box pairs next to the bounding
  spheres.
- The search runs as rounds of up to 2048 nodes with batched forward
  kinematics; results do not depend on round size.
- The fork's test suite ported to pytest, plus the 436-curve corpus and the
  paper-benchmark analog against the fork.
