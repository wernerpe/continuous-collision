# Relation to Drake

## The upstream PR

The checker was written as a Drake class:
`drake::planning::continuous_collision::ContinuousCollisionChecker`, bound in
Python as `pydrake.planning.continuous_collision`.

| | |
|---|---|
| PR | [RobotLocomotion/drake#24924](https://github.com/RobotLocomotion/drake/pull/24924), "[planning] Fast continuous collision checking for polynomial cspace trajectories" |
| state | open, opened 2026-08-28 |
| PR branch | `certified-ccd` of [github.com/wernerpe/drake](https://github.com/wernerpe/drake/tree/certified-ccd), head `97d9d2cb08` |
| merged build | `master-ccd`: Drake master `04db37019c` with `certified-ccd` merged (merge commit `b6b672d232`, no conflicts) |
| paper | [certified_ccd_paper.pdf](certified_ccd_paper.pdf), measured with a standalone build against prebuilt Drake `dc6688b` (near v1.45) |

The fork numbers in [benchmarks.md](benchmarks.md) were measured on
`master-ccd`.
This package's API, guarantee and resolution contract are the ones in the PR's
header documentation at `97d9d2cb08`.

## Why this package exists

The PR is not in any Drake release.
Drake's PyPI wheels ship pydrake without the C++ headers, so the checker
cannot be compiled as an extension against a pip-installed Drake.
Using the fork therefore means a from-source Bazel build of all of Drake,
about an hour.
Current Drake master supports only Ubuntu 24.04 and 26.04 and Python ≥ 3.12;
building it for Ubuntu 22.04 and Python 3.10 needs flags and overrides that
Drake does not support.
This package needs neither: it is pure Python and numpy on the stock `drake`
wheel.

## What is the same

- The public API: the five names, their fields, method names and argument
  names.
- The normalization of trajectories into Bézier segments, including the
  degree cap of 10 and the $10^{-7}$ continuity tolerance.
- The motion bound: $J(p)$, the reaches, the per-joint constants
  $\lambda(j,p)$, the carve-out residual $\sigma_p$, and the bounding-sphere
  formulas.
- The certificate test (12), the violation test (13), the per-pair tolerances
  $\tau_p$, the slack $\varepsilon = 1$ nm, and the per-pair distance
  resolution floor.
- The guarantee and the resolution contract.
- The verdict rules: earliest violation first, then the earliest inconclusive
  witness, then `kCertifiedFree`.
- The capability probe: the same models, shapes and joints raise.

## What differs

| | fork (PR #24924) | this package |
|---|---|---|
| language | C++, bound into pydrake | pure Python and numpy on the stock `drake` wheel (1.51.1 or later) |
| distance queries | `QueryObject.ComputeSignedDistancePairClosestPoints`, one pair at a time, plus an analytic half-space fallback | the same Drake query for general shape pairs, one configuration at a time; exact closed forms, batched over a round, for half-space pairs (the fork's fallback), sphere-sphere, sphere-box and box-box |
| box-box pairs | Drake's narrowphase | exact: the separating-axis depth when they interpenetrate, the nearest vertex-box or edge-edge distance when separated, with the separating-axis gap as a second free-ball lower bound next to the bounding spheres |
| search order | one node at a time on a shared LIFO stack, left child first | rounds of up to 2048 nodes, segments of one degree together, children pushed left of everything pending; see below |
| forward kinematics | Drake's, per node | a batched closed form per joint type, pinned to Drake's at 1e-12 by the tests |
| parallelism | call-scoped C++ worker threads sharing one stack; helpers hired after 64 nodes | accepted for API parity, evaluated serially (no batched per-pair query in pydrake, and its queries hold the GIL) |
| GIL | released for the whole check | held |
| constructor | keyword-only: `ContinuousCollisionChecker(model=..., default_options=...)`; `default_options` must be an `Options` | `model` and `default_options` may be positional; `default_options=None` means `Options()` |
| `Verdict` | pybind11 enum | Python enum with the same member names |
| `num_nodes` on a refused check | depends on thread timing | deterministic, but can exceed the fork's (rounds are decided before the earliest-violation time drops) |
| speed | see [benchmarks.md](benchmarks.md) | see [benchmarks.md](benchmarks.md) |

The verdict and the witness are a function of the trajectory and the options
in both, so the two agree on every input up to the oracle tolerances.
[benchmarks.md](benchmarks.md) measures that agreement: 436 of 436 corpus
curves, identical witnesses, identical node counts on every certified check.

### How this package searches

Algorithm 1 of the paper, run as rounds over a work list.
A round takes up to 2048 pending nodes, splits them all at once, poses them
with one batched forward-kinematics call, and decides every (node, active pair)
entry with array arithmetic: the motion bound, the bounding-sphere prefilter,
the closed-form distance routes, and the certificate and violation tests.
Only the entries no closed form can decide go to Drake's narrowphase, one
configuration at a time.
Children are pushed in time order to the left of everything still pending, so
the sweep runs left to right and the earliest-violation bound prunes as in the
fork.
Each node's decisions depend only on its control points and inherited pairs,
and each node's arithmetic is computed the same way whatever shares its round,
so the verdict and the witness are the fork's serial ones, and on a certified
run so is `num_nodes`.

### Drake's polytope distances can exceed their documented accuracy

The certificate assumes the oracle is accurate to $\tau_p$, which transcribes
Drake's documented accuracy (Box-Box $4 \times 10^{-15}$ m, Convex-Convex
$3 \times 10^{-15}$ m, charged as $10^{-6}$ m).
On rare poses Drake's narrowphase over-reports the distance of two separated
boxes by much more: 5 of 20,000 random box pairs by more than $10^{-6}$ m,
the worst by 0.63 mm (`benchmarks/drake_box_accuracy.py`; every case confirmed
by alternating projections). The same cuboids as `Convex` shapes showed 2
over-reports in 15,000 poses, the worst 1.1 mm. Drake 1.51.1 and the fork's
build agree on these values, so the fork inherits the gap for Box, Convex and
Mesh pairs.
This package computes box-box pairs exactly and so does not inherit it there;
Convex and Mesh pairs still go through Drake's narrowphase, in both.

## Switching when the PR lands

Change the import:

```python
from continuous_collision import ContinuousCollisionChecker, Options, Result, Finding, Verdict
```

to

```python
from pydrake.planning.continuous_collision import ContinuousCollisionChecker, Options, Result, Finding, Verdict
```

Code that works unchanged with both:

- passes `model=` and `default_options=` by keyword, and never passes
  `default_options=None`;
- compares verdicts by name (`result.verdict == Verdict.kCertifiedFree`), not
  by integer value or enum class;
- catches `RuntimeError` for refused inputs.

To use whichever is installed:

```python
try:
    from pydrake.planning.continuous_collision import ContinuousCollisionChecker, Options, Verdict
except ImportError:
    from continuous_collision import ContinuousCollisionChecker, Options, Verdict
```
