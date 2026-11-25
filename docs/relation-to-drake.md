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
| language | C++, bound into pydrake | pure Python and numpy |
| distance queries | `QueryObject.ComputeSignedDistancePairClosestPoints`, one pair at a time, plus an analytic half-space fallback | stock pydrake `SceneGraphCollisionChecker`, batched clearance queries |
| parallelism | call-scoped C++ worker threads sharing one stack; helpers hired after 64 nodes | `Options.parallelism` is passed to the batched distance queries |
| GIL | released for the whole check | held by the Python node loop |
| constructor | keyword-only: `ContinuousCollisionChecker(model=..., default_options=...)`; `default_options` must be an `Options` | `model` and `default_options` may be positional; `default_options=None` means `Options()` |
| inconclusive finding | `nearest_a_W` and `nearest_b_W` are filled | `nearest_a_W` and `nearest_b_W` are `None` |
| `Verdict` | pybind11 enum | Python enum with the same member names |
| speed | see [benchmarks.md](benchmarks.md) | see [benchmarks.md](benchmarks.md) |

The verdict and the witness are a function of the trajectory and the options
in both, so the two should agree on every input up to the oracle tolerances.
[benchmarks.md](benchmarks.md) measures that agreement.

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
- does not rely on the closest points being `None` for an inconclusive
  witness;
- catches `RuntimeError` for refused inputs.

To use whichever is installed:

```python
try:
    from pydrake.planning.continuous_collision import ContinuousCollisionChecker, Options, Verdict
except ImportError:
    from continuous_collision import ContinuousCollisionChecker, Options, Verdict
```
