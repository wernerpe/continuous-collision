# API reference

```python
from continuous_collision import ContinuousCollisionChecker, Options, Result, Finding, Verdict
```

The module exports exactly these five names.
The documentation below is the header documentation of Drake PR #24924
(`planning/continuous_collision/continuous_collision_checker.h` at
`97d9d2cb08`), with C++ types replaced by their Python counterparts.

Every error listed below raises `RuntimeError`, as the fork's pydrake bindings
do, with a message that names the offending input.

## `Verdict`

Outcome of one check. An enum with three values.

| value | meaning |
|---|---|
| `Verdict.kCertifiedFree` | Proof: every unfiltered pair keeps signed distance > margin over the entire continuous time domain. |
| `Verdict.kViolationFound` | An exactly-on-trajectory configuration violates the threshold. |
| `Verdict.kInconclusive` | Some pair's clearance comes within `Options.distance_resolution` (plus the oracle tolerance) of the margin, so refining further cannot decide it: the trajectory grazes the margin. |

## `Finding`

Where the plan fails, or where it could not be decided.

| field | type | meaning |
|---|---|---|
| `time` | `float` | Trajectory time of the witness configuration. |
| `q` | `np.ndarray`, shape `(num_positions,)` | The witness configuration, exactly on the trajectory. |
| `geometry_a` | `pydrake.geometry.GeometryId` | First geometry of the pair. |
| `geometry_b` | `pydrake.geometry.GeometryId` | Second geometry of the pair. |
| `body_a` | `pydrake.multibody.tree.BodyIndex` | Body carrying `geometry_a`. |
| `body_b` | `pydrake.multibody.tree.BodyIndex` | Body carrying `geometry_b`. |
| `distance` | `float` | Signed distance of the pair at `q`, in meters. |
| `nearest_a_W` | `np.ndarray`, shape `(3,)`, or `None` | Closest point on `geometry_a` in the world frame at `q`. |
| `nearest_b_W` | `np.ndarray`, shape `(3,)`, or `None` | Closest point on `geometry_b` in the world frame at `q`. |

The closest points are present for violations, so that planners can push the
trajectory out of collision. They are `None` for an inconclusive witness.

For a violation, `distance` is below the margin and is the earliest violation
the search recorded.
For an inconclusive witness, `distance` lies in
$[\delta - \tau_p,\ \delta + \tau_p + \varepsilon + \sigma_p + r]$
(see the resolution contract below).

Names for reporting: `model.scene_graph().model_inspector().GetName(finding.geometry_a)`
and `model.plant().get_body(finding.body_a).name()`.

## `Options`

Options controlling one check.

```python
Options(margin=0.0, distance_resolution=1e-6, continuous_revolute_indices=(),
        parallelism=pydrake.common.Parallelism.Max())
```

All four are keyword arguments and read-write attributes.

**`margin`** (`float`, default `0.0`). Clearance margin $\delta$ in meters: the
check certifies signed distance > margin for every unfiltered pair at every
time. Must be finite and nonnegative.
A pair meant to touch must be collision-filtered, not given a negative margin.

**`distance_resolution`** (`float`, default `1e-6`). Resolution floor $r$ in
meters. A pair stops being refined on a node once its bounded relative motion
over that node is at most $r$; if it is still undecided there, the check
reports `Verdict.kInconclusive` with that node's midpoint as the witness.
Definitive verdicts are guaranteed for a trajectory whose clearance stays more
than $r$ (plus the oracle tolerance, see the class documentation) away from
the margin everywhere; the cost of a grazing trajectory grows roughly linearly
in $1/r$. Must be finite and positive.

**`continuous_revolute_indices`** (sequence of `int`, default empty). Position
coordinates whose junction continuity is checked modulo $2\pi$ (the
`GcsTrajectoryOptimization` continuous-revolute convention; see
`pydrake.planning.GetContinuousRevoluteJointIndices`).

**`parallelism`** (`pydrake.common.Parallelism`, default
`Parallelism.Max()`). The parallelism a check may use. The verdict and the
reported witness do not depend on it.

## `Result`

Result of one check.

| field | type | meaning |
|---|---|---|
| `verdict` | `Verdict` | The outcome. |
| `finding` | `Finding` or `None` | The earliest violation, or the inconclusive witness; `None` iff the verdict is `Verdict.kCertifiedFree`. |
| `num_nodes` | `int` | Nodes visited by the adaptive subdivision; a cost measure. |

When a trajectory has both a violation and an inconclusive witness, the
verdict is `kViolationFound`.
`num_nodes` counts subdivision nodes, not the breakpoint configurations
checked directly.

## `ContinuousCollisionChecker`

Certifies, rather than samples, that a trajectory is collision-free over its
entire continuous time domain.

**Guarantee.** If a check returns `Verdict.kCertifiedFree`, then for every
time $t$ in the trajectory's domain and every unfiltered geometry pair
$(A, B)$, the signed distance $\varphi_{AB}(q(t))$ exceeds `Options.margin`.
That holds under three assumptions: exact real arithmetic up to an internal
numerical slack, a distance oracle accurate to its stated tolerance, and
`Mesh` ≡ convex hull. The proof is a property of the path, so retiming the
trajectory afterwards does not invalidate it.

**Resolution contract.** Write $\delta$ for `Options.margin`, $r$ for
`Options.distance_resolution`, $\tau_p$ for the oracle tolerance of pair $p$
(at least 1 µm; Drake's documented signed-distance accuracy for that shape
combination), $\varepsilon$ for the internal slack (1 nm), and $\sigma_p$ for
the residual motion of coordinates the trajectory holds constant only to
within the continuity tolerance (exactly zero when they are exactly constant,
the common case). Then, for every pair,

- if $\varphi_p(q(t)) > \delta + r + \sigma_p + 2\tau_p + \varepsilon$ for
  every $t$, the pair is certified, so a trajectory that clears the margin by
  that much everywhere returns `Verdict.kCertifiedFree`;
- if $\varphi_p(q(t)) < \delta - (r + \sigma_p + 2\tau_p)$ for some $t$, the
  check returns `Verdict.kViolationFound`;
- `Verdict.kInconclusive` is therefore possible only when some pair's
  clearance comes within that band of the margin, and its `Finding` then
  names an on-trajectory configuration whose reported distance lies in
  $[\delta - \tau_p,\ \delta + \tau_p + \varepsilon + \sigma_p + r]$.

Resolutions below what double precision can represent along a segment are
capped by a floating-point backstop.

**Pairs.** The checked pairs are the proximity pairs left unfiltered by the
model's collision filters at construction. Filter changes applied to a
`Context` afterwards are not observed, so a checker keeps certifying the pair
set it was constructed with.

[algorithm.md](algorithm.md) derives the guarantee and the contract.

### `ContinuousCollisionChecker(model, default_options=None)`

Builds contexts, bounding spheres and topology tables, and runs the
capability probe.

- `model` (`pydrake.planning.RobotDiagram`): the robot and its scene. It is
  kept alive by the checker.
- `default_options` (`Options` or `None`): the options a check uses when it is
  given none. `None` means `Options()`.

Raises:

- if `model` is `None` or its plant is not finalized;
- if `default_options` is invalid; see `CheckTrajectory()`;
- if a pair's shape combination is unsupported, i.e. a deformable geometry or
  halfspace against halfspace;
- if the plant's topology or geometry defeats the motion bound: a rotating
  `HalfSpace`, a reversed joint, a kinematic loop, or a proximity shape with
  no bounding sphere.

### `CheckTrajectory(trajectory, options=None) -> Result`

Certifies a trajectory (`BezierCurve`, `BsplineTrajectory`,
`PiecewisePolynomial`, or a `CompositeTrajectory` of those, all from
`pydrake.trajectories`). `Finding.time` is in the trajectory's own time.

Raises:

- if `Options.margin` is not a finite nonnegative distance, or if
  `Options.distance_resolution` is not a finite positive distance;
- if the trajectory's row count differs from the plant's number of
  generalized positions;
- if the trajectory is not one of the supported types, has a segment of
  degree above 10, or is discontinuous at a junction (by more than $10^{-7}$
  per coordinate, modulo $2\pi$ for `Options.continuous_revolute_indices`);
- if `Options.continuous_revolute_indices` names a coordinate outside the
  plant's;
- if the trajectory moves a coordinate of an unsupported joint type
  (quaternion floating, ball), or moves a `HalfSpace` across a rotational
  coordinate.

The supported moving joint types are revolute, prismatic, planar and screw.
Any other joint type is accepted only while the trajectory holds its
coordinates constant.

### `CheckPath(waypoints, options=None) -> Result`

Certifies the piecewise-linear path through the given waypoint columns.
`waypoints` is an `np.ndarray` of shape `(num_positions, num_points)`.
Segment $k$ spans time $[k, k+1]$, so `Finding.time` is a fractional waypoint
index.

Raises:

- if `waypoints` has fewer than two columns, or does not have one row per
  generalized position of the plant;
- under every condition `CheckTrajectory()` lists.

### `CheckEdge(q1, q2, options=None) -> Result`

Certifies the straight configuration-space edge `q1` → `q2`.
It is `CheckPath` on the two-column matrix `[q1, q2]`, so the edge spans time
$[0, 1]$.

Raises:

- if `q1` or `q2` does not have one entry per generalized position of the
  plant;
- under every condition `CheckTrajectory()` lists.

### `model() -> RobotDiagram`

The `RobotDiagram` the checker was constructed with.

## Determinism

Every node's decisions depend only on its own control points and the set of
pairs it inherits.
So the verdict and the reported witness are a function of the trajectory and
the options alone.
`Result.num_nodes` is too on a run that certifies.
Where a violation prunes the search, the set of nodes skipped depends on
search order, so `num_nodes` can vary.
One exception: on a degenerate segment whose start and end times coincide,
every node maps to the same time, so the reported configuration (not its time)
may differ.

## Thread safety

The PR header states: the `Check*` methods are const, own no mutable state
outside per-call scratch, and may be called concurrently on one instance from
arbitrary threads; construction and destruction are not thread-safe.
This package targets the same rule, but it is pure Python: calls from several
Python threads share the interpreter lock.
