# continuous-collision

`continuous-collision` proves that a trajectory of a Drake robot model is
collision-free over its whole continuous time domain, not at samples. You give
it a `RobotDiagram` and a trajectory. It returns `kCertifiedFree`,
`kViolationFound` with a witness configuration that lies exactly on the
trajectory, or `kInconclusive` when the trajectory grazes the margin.
`kCertifiedFree` means: for every time $t$ in the trajectory's domain and every
unfiltered geometry pair $(A, B)$, the signed distance $\varphi_{AB}(q(t))$
exceeds `Options.margin`. That statement holds under three assumptions: exact
real arithmetic up to an internal slack of 1 nm, Drake's distance queries
accurate to their documented tolerance, and a `Mesh` meaning its convex hull.
The proof is a property of the path, so retiming the trajectory afterwards
does not invalidate it. The package reimplements the checker of
[Drake PR #24924](https://github.com/RobotLocomotion/drake/pull/24924) in pure
Python on the stock `drake` wheel from PyPI. It has the same API, the same
bounds and the same resolution contract.

## Install

```bash
pip install continuous-collision
```

The distribution may be published as `ccd-drake` if `continuous-collision` is
taken on PyPI; the import name is `continuous_collision` either way.
The only dependency is `drake` from PyPI (plus `numpy`, which it brings).
There is no compiled code.

## Example

<!-- Scene and expected output verified on 2026-11-25 against the fork's
pydrake.planning.continuous_collision (master-ccd, b6b672d232) with the import
line swapped; re-verify against this package. -->

```python
import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve
from continuous_collision import ContinuousCollisionChecker, Options

builder = RobotDiagramBuilder()
plant = builder.plant()
builder.parser().AddModels(
    url="package://drake_models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([0.6, 0.0, 0.5]),
                                Box(0.05, 1.0, 1.0), "wall", CoulombFriction(1.0, 1.0))
checker = ContinuousCollisionChecker(model=builder.Build(), default_options=Options(margin=0.01))

q_a = np.zeros(7)
q_b = np.array([1.5, -0.6, 0.0, -1.2, 0.0, 0.6, 0.0])
print(checker.CheckEdge(q_a, q_b).verdict)            # Verdict.kCertifiedFree

lean = np.array([0.0, 1.6, 0.0, -0.4, 0.0, 0.8, 0.0])  # pulls the curve into the wall
result = checker.CheckTrajectory(BezierCurve(0.0, 2.0, np.column_stack([q_a, lean, q_b])))
print(result.verdict)                                 # Verdict.kViolationFound
w = result.finding
print(w.time, w.distance, plant.get_body(w.body_a).name(), plant.get_body(w.body_b).name())
# 0.449... 0.00999... iiwa_link_7 world
```

The model is the iiwa14 with the dense all-sphere collision model from
`drake_models`; the first run downloads `drake_models`.
The wall is a 5 cm thick box welded to the world, 0.6 m in front of the robot.
The witness is the earliest violation the search found: at $t = 0.449$ s the
`iiwa_link_7` sphere comes within 10 mm of the wall.
`w.q` is that configuration, and `w.nearest_a_W`, `w.nearest_b_W` are the two
closest points in the world frame.

## Verdicts

| Verdict | Meaning | `Result.finding` |
|---|---|---|
| `kCertifiedFree` | Proof: every unfiltered pair keeps signed distance > margin over the entire continuous time domain. | `None` |
| `kViolationFound` | An exactly-on-trajectory configuration violates the margin. | The earliest violation found, with the closest points. |
| `kInconclusive` | Some pair's clearance comes within `Options.distance_resolution` (plus the oracle tolerance) of the margin, so refining further cannot decide it: the trajectory grazes the margin. | The inconclusive witness, an on-trajectory configuration; closest points `None`. |

A trajectory that clears the margin by more than
$r + \sigma_p + 2\tau_p + \varepsilon$ everywhere is certified, and one that
dips below it by more than $r + \sigma_p + 2\tau_p$ somewhere is refused.
$r$ is `distance_resolution` (default 1 µm), $\tau_p \ge 1$ µm is the pair's
distance-query tolerance, $\varepsilon$ is 1 nm, and $\sigma_p$ is zero unless
the trajectory holds a coordinate constant only to within $10^{-7}$.
[docs/algorithm.md](docs/algorithm.md#the-resolution-contract) states the
contract exactly.

## What is supported

**Trajectories.** `BezierCurve`, `BsplineTrajectory`, `PiecewisePolynomial`,
and `CompositeTrajectory` of those, through `CheckTrajectory`.
A waypoint matrix through `CheckPath`, and a straight edge through `CheckEdge`.
Every segment has degree at most 10.
Adjacent segments must agree at their junction to within $10^{-7}$ per
coordinate (modulo $2\pi$ for `Options.continuous_revolute_indices`).

**Joints the trajectory may move.** Revolute, prismatic, planar and screw.
Welds carry no coordinates and are always fine.

**Joints the trajectory must hold constant.** Quaternion floating, roll-pitch-yaw
floating, ball and universal joints are accepted only while their coordinates
stay constant along the trajectory. A moved coordinate of one of these raises.
Any other joint type raises.

**Topology.** A kinematic tree. Kinematic loops and reversed joints (a parent
body outboard of its child) raise at construction.

**Shapes.** `Sphere`, `Box`, `Capsule`, `Cylinder`, `Ellipsoid`, `Convex`,
`Mesh` and `HalfSpace`.
A `Mesh` is certified as its convex hull, matching Drake's own signed-distance
semantics for `Mesh`.
That is conservative in the certification direction, but a hollow or concave
mesh can produce false violations.
For tight nonconvex obstacles, register a convex decomposition as several
`Convex` geometries.
Deformable geometries raise.

**HalfSpace rules.** A pair of two half spaces raises.
A half space may be the static partner of a rotating body, such as an anchored
ground plane under an arm.
A half space on the moving side of a revolute, planar or screw joint, relative
to an unfiltered partner, raises at construction, because its reach is
unbounded.
A half space carried across a rotational coordinate of any other joint type is
accepted only if that coordinate is exactly constant along the trajectory.
The usual fixes are to anchor the half space, filter the pair, or replace the
half space with a large `Box`.

**Margins.** `Options.margin` is one finite, nonnegative number for every pair.
A negative margin raises; filter a pair that is meant to touch instead.
For different margins on different pairs, build one checker per margin, each on
a model whose collision filters leave exactly the pairs that carry it.

**Pairs.** The checked pairs are the unfiltered proximity pairs at
construction. Collision-filter changes made afterwards are not observed.

**Scope.** The certificate is a statement about the model.
Transferring it to hardware also needs a collision model that over-approximates
the physical robot and a tracking error that fits inside the margin.

## Relation to Drake

The checker was written for Drake as
`drake::planning::continuous_collision::ContinuousCollisionChecker`
([PR #24924](https://github.com/RobotLocomotion/drake/pull/24924), branch
`certified-ccd` of `github.com/wernerpe/drake`).
The Drake wheels on PyPI ship no C++ headers, so that code cannot be added to a
pip-installed Drake. Using it today means a source build of the fork.
This package runs the same algorithm in Python on top of stock pydrake distance
queries.
If the PR lands, switch by changing one import:

```python
from continuous_collision import ContinuousCollisionChecker, Options, Result, Finding, Verdict
# becomes
from pydrake.planning.continuous_collision import ContinuousCollisionChecker, Options, Result, Finding, Verdict
```

Pass `model=` and `default_options=` by keyword, as the example does; the
fork's binding accepts them only that way.
[docs/relation-to-drake.md](docs/relation-to-drake.md) lists the remaining
differences.

## Documentation

- [docs/api.md](docs/api.md): every class, method, option and error.
- [docs/algorithm.md](docs/algorithm.md): the method, the guarantee and the
  resolution contract.
- [docs/relation-to-drake.md](docs/relation-to-drake.md): PR #24924, the fork,
  and what differs.
- [docs/benchmarks.md](docs/benchmarks.md): agreement and timing against the
  fork.
- [docs/certified_ccd_paper.pdf](docs/certified_ccd_paper.pdf): the paper.

## Citation

```bibtex
@misc{werner2026certified,
  title  = {Certified Continuous Collision Checking for Drake},
  author = {Werner, Peter and Claude},
  year   = {2026},
  month  = aug,
  note   = {Preprint}
}
```

## License

MIT. See [LICENSE](LICENSE).
