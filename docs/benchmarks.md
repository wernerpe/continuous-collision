# Benchmarks

Two benchmarks compare this package with the fork's C++ checker
(PR #24924; see [relation-to-drake.md](relation-to-drake.md)).
Cells marked TODO are filled by the scripts in `benchmarks/`.
Every number states its machine, its thread count and the build it ran on.

## 1. The 436-curve corpus

**Provenance.** The curves are every candidate trajectory the certificate gate
of the bimanual Franka station's brick experiment saw: the straight line and
each optimizer iterate, for 84 leg-and-configuration cases, 436 curves in all.
They are piecewise Bézier curves over the 14 arm joints, planned among
shelves, table bricks and held bricks.
They were exported by `experiments/bricks/ccd_bench/legs.py` of
`bimanual-franka-cspace-experiments` as control points
(`curves/<leg>_<cfg>.npz`) with one scene description per world
(`specs/<key>.json`, 37 descriptions).
`benchmarks/drake_scene.py` rebuilds the Drake model from a description.

**Settings.** Margins are 10 mm on world, arm-to-arm and self pairs, and 0 on
the held bricks' pairs. `Options.margin` is one number per checker, so each
description gets one checker per distinct margin, each over exactly the pairs
that carry it; a curve is certified when every checker certifies it, and its
wall time is their sum. `Options.distance_resolution` is $10^{-5}$ m.

**Fork reference.** Measured on `master-ccd` (`b6b672d232`), Python 3.10,
Intel i9-10850K, `Parallelism.Max()` (20 threads), idle machine:

- 436 of 436 verdicts reproduced when re-run through the station's shipped
  certificate path.
- 433 of 436 verdicts agree with an independent GPU free-ball certificate. The
  3 others are certified by the fork and refused by the free-ball gate, whose
  box-box lower bound was 8.1 to 9.8 mm where Drake's exact clearance was
  10.4 to 13.1 mm against a 10 mm margin.
- Per curve, the checks alone: 17.7 ms median, 24.9 ms p95.
- Per leg, on the 30 legs whose final curve certified: 19 ms median, 27 ms
  p95, 28 ms max.
- Repeated runs gave identical verdicts, witness pairs, witness times and
  distances. The node count varied on 22 refused curves.

### Agreement

| | count |
|---|---|
| curves | 436 |
| fork verdict = package verdict | TODO |
| `kCertifiedFree`, fork / package | TODO / TODO |
| `kViolationFound`, fork / package | TODO / TODO |
| `kInconclusive`, fork / package | TODO / TODO |
| violations with the same witness pair and time | TODO |
| certified curves with the same `num_nodes` | TODO |

### Wall time per curve

| build | threads | median (ms) | p95 (ms) | max (ms) |
|---|---|---|---|---|
| fork, `master-ccd` | 20 | 17.7 | 24.9 | TODO |
| fork, `master-ccd` | 1 | TODO | TODO | TODO |
| this package | `Parallelism.Max()` | TODO | TODO | TODO |
| this package | 1 | TODO | TODO | TODO |

Machine for the TODO rows: TODO. drake wheel version: TODO.

## 2. The paper's iiwa14 shelf benchmark

**Provenance.** Table I of the paper
([certified_ccd_paper.pdf](certified_ccd_paper.pdf)).
The robot is an iiwa14 with the all-sphere dense collision model of
`drake_models`, in a shelf world with 936 unfiltered pairs.
The shelf trajectories are quintic composite Bézier curves, $C^2$ by
construction, through frozen inverse-kinematics and hand-picked waypoints; the
headline row is a curved six-segment trajectory.
The three shelf tiers differ in the trajectory's swept clearance (2 mm, 1 cm,
5 cm), measured as ground truth from $10^5$ dense samples with golden-section
refinement.
The piecewise-linear edge connects two shelf waypoints.
The dual-arm handover has 3384 pairs.
The grazing row is a trajectory whose clearance is 2.4 nm.
Paper timings are single-thread medians of 20 repetitions after 3 untimed
warmups (grazing: 3 repetitions, no warmup), minima in parentheses, on an AMD
Ryzen 9 7950X3D, unpinned, with the standalone build against Drake `dc6688b`.
The paper's checker used the earlier `min_interval` floor; `distance_resolution`
replaced it afterwards, so the grazing row's node count is not directly
comparable.
The paper gives no margin for these rows.
The benchmark scripts and frozen trajectories live outside the Drake tree and
are not in this repository: TODO (source of the frozen trajectories).

| scenario | pairs | swept clearance | verdict | nodes, paper | wall ms, paper (min) | nodes, package | wall ms, package, 1 thread |
|---|---|---|---|---|---|---|---|
| shelf, 2 mm tier | 936 | 2.0 mm | `kCertifiedFree` | 262 | 2.983 (2.904) | TODO | TODO |
| shelf, 1 cm tier | 936 | 10.0 mm | `kCertifiedFree` | 146 | 2.025 (1.988) | TODO | TODO |
| shelf, 5 cm tier | 936 | 50.0 mm (env) | `kCertifiedFree` | 96 | 1.486 (1.473) | TODO | TODO |
| PWL edge | 936 | 77.0 mm (env) | `kCertifiedFree` | 15 | 0.251 (0.243) | TODO | TODO |
| dual-arm handover | 3384 | 16.3 mm | `kCertifiedFree` | 110 | 1.506 (1.484) | TODO | TODO |
| grazing | 936 | 2.4 nm | `kInconclusive` | 5,785,680 | 42,037 (41,508) | TODO | TODO |

"env" marks clearance to the environment only; the iiwa14 sphere model carries
an intrinsic 25.2 mm self-clearance.
For comparison, Drake's sampled sweep at a 0.05 rad step checks 135
configurations in 1.860 ms (1.811) on the 1 cm tier and 37 configurations in
0.488 ms (0.476) on the PWL edge. A sampled sweep claims freedom at its
samples only.
Machine for the TODO columns: TODO. drake wheel version: TODO.
