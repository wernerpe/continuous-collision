# Benchmarks

Two benchmarks compare this package with the fork's C++ checker
(PR #24924; see [relation-to-drake.md](relation-to-drake.md)).
The scripts that produce every number are in `benchmarks/`.
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

Re-run on 2026-11-25 on this machine: Intel i9-10850K (10 cores, 20
threads), Ubuntu 22.04, Python 3.10, load average 2 to 3 from other work.
The fork is its pydrake (`drake_for_frankas/install-ccd`, built from
`master-ccd`) run in its own interpreter; the package ran on the `drake` 1.51.1
wheel. Both went through `benchmarks/corpus.py`;
[benchmarks/corpus_report.md](../benchmarks/corpus_report.md) has every curve.

| | count |
|---|---|
| curves | 436 |
| fork verdict = package verdict | 436 / 436 |
| `kCertifiedFree`, fork / package | 284 / 284 |
| `kViolationFound`, fork / package | 152 / 152 |
| `kInconclusive`, fork / package | 0 / 0 |
| violations with the same witness pair and time | 152 / 152 (configurations bit-identical, distances within 3e-16 m) |
| certified curves with the same `num_nodes` | 284 / 284 (684 / 684 per-margin checks) |

On 62 of the 188 refused per-margin checks the package visits more nodes than
the fork (1 to 139, mean 14), never fewer: a refused check's node count depends
on search order, and the fork's own count differs between 1 thread and
`Parallelism.Max()` on 33 of them.

### Wall time per curve

One untimed warmup, then the median of 5 timed runs per curve; a curve's wall
is the sum over its checkers.

| build | threads | median (ms) | p95 (ms) | max (ms) |
|---|---|---|---|---|
| fork, `master-ccd` | 20 (`Parallelism.Max()`) | 18.4 | 25.1 | 28.6 |
| fork, `master-ccd` | 1 | 31.7 | 66.9 | 159.7 |
| this package, drake 1.51.1 | `Parallelism.Max()` | 37.9 | 63.0 | 83.5 |
| this package, drake 1.51.1 | 1 | 37.6 | 60.9 | 86.1 |

Per curve, the package takes 1.12 times the fork's single-thread wall (median;
p95 2.07). It is relatively slowest on small checks, where its per-round
overhead dominates, and faster than the single-threaded fork on the largest
ones, where a round amortizes it over many nodes. The package evaluates
serially, so its two rows are the same code path.

## 2. The paper's iiwa14 shelf benchmark

**Provenance.** Table I of the paper
([certified_ccd_paper.pdf](certified_ccd_paper.pdf)): an iiwa14 with the
all-sphere dense collision model of `drake_models`, a shelf world with 936
unfiltered pairs, curved six-segment quintic trajectories at three swept
clearances, a piecewise-linear edge, a dual-arm handover (3384 pairs) and a
grazing trajectory. Paper timings are single-thread medians of 20 repetitions
after 3 untimed warmups on an AMD Ryzen 9 7950X3D, with the standalone build
against Drake `dc6688b`, and the paper's checker used the earlier
`min_interval` floor.

The paper's frozen scenes and trajectories are not available, so
`benchmarks/paper_bench.py` rebuilds the setup from that description on
Drake's bundled model: the iiwa14 in front of an eight-box shelf (936 pairs,
as in the paper: 600 self pairs plus 42 moving spheres times 8 boxes), one
curved trajectory (a clamped quintic B-spline, six Bézier segments) reaching
into the shelf, the shelf moved by bisection until the trajectory's swept
clearance to it is 2 mm, 10 mm and 50 mm (ground truth from dense sampling with
golden-section refinement), an edge between two shelf configurations, two
iiwa14 facing each other over a table (3384 pairs, as in the paper) at 16.3 mm
of clearance, and the shelf moved to a 2.4 nm clearance for the grazing row.
`benchmarks/paper_bench.json` freezes the result. Margin 0 throughout: the
table states swept clearances, not margins. Fork and package ran the same
scenes and trajectories on the machine above; medians of 20 repetitions after
3 warmups, grazing 3 repetitions without warmup.

| scenario | pairs | swept clearance | verdict | nodes, paper | ms, paper (1 thread) | nodes, fork = package | ms, fork 1 thread | ms, fork 20 threads | ms, package |
|---|---|---|---|---|---|---|---|---|---|
| shelf, 2 mm tier | 936 | 2.0 mm | `kCertifiedFree` | 262 | 2.983 | 466 | 6.21 | 2.46 | 7.78 |
| shelf, 1 cm tier | 936 | 10.0 mm | `kCertifiedFree` | 146 | 2.025 | 268 | 4.24 | 2.15 | 5.26 |
| shelf, 5 cm tier | 936 | 50.0 mm (env) | `kCertifiedFree` | 96 | 1.486 | 88 | 2.08 | 2.39 | 3.79 |
| PWL edge | 936 | 9.7 mm (env; paper 77 mm) | `kCertifiedFree` | 15 | 0.251 | 127 | 1.62 | 1.55 | 3.26 |
| dual-arm handover | 3384 | 16.3 mm | `kCertifiedFree` | 110 | 1.506 | 63 | 2.43 | 2.42 | 4.01 |
| grazing, `distance_resolution` 1e-6 | 936 | 2.4 nm | `kInconclusive` | | | 14,696 | 156.5 | 16.1 | 88.6 |
| grazing, `distance_resolution` 1e-8 | 936 | 2.4 nm | `kInconclusive` | 5,785,680 (`min_interval`) | 42,037 | 827,614 | 8,354 | 793 | 4,689 |

"env" marks clearance to the shelf only; the iiwa14 sphere model carries an
intrinsic 25.2 mm self-clearance. The scenes differ from the paper's, so only
the fork and package columns compare like with like: identical verdicts and
node counts on every row, and identical witnesses on the grazing rows. On this
machine the fork's 1 cm tier costs 4.24 ms on one thread; the paper reports
2.0 ms on a faster machine for a shallower tree. The package is 1.2 to 2 times
slower than the single-threaded fork on these sub-10 ms checks, and about half
its time on the grazing rows, where rounds hold thousands of nodes.
