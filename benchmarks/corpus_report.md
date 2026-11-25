# Corpus report: this package against the fork

The 436 candidate curves of the bimanual Franka brick experiment's certificate gate (`/home/peter/gitcspace/bfce-drake-ccd/experiments/bricks/out/ccd_bench`), re-checked by the fork's `pydrake.planning.continuous_collision` (the referee, run in its own interpreter) and by this package, both through `benchmarks/corpus.py`; this file is written by `benchmarks/report.py`.

436 curves, 872 checks (one per distinct margin per curve), distance_resolution 1e-5 m.

## Agreement

| | count |
|---|---|
| curve verdict = the recorded fork verdict (verdicts.jsonl) | 436 / 436 |
| per-margin verdicts = the fork re-run on this machine (1 thread) | 436 / 436 |
| fork 1 thread = fork Parallelism.Max() (per-margin verdicts) | 436 / 436 |
| kCertifiedFree curves, fork / package | 284 / 284 |
| kViolationFound curves, fork / package | 152 / 152 |
| kInconclusive curves, fork / package | 0 / 0 |
| refused curves with the same witness pair and time (1e-9) | 152 / 152 |
| largest witness-configuration difference, fork vs package | 0.0e+00 |
| largest witness-distance difference, fork vs package (m) | 2.7e-16 |
| certified checks with the same num_nodes | 684 / 684 |
| refused checks with the same num_nodes | 126 / 188 |

### Why node counts differ on refused checks

A violation lowers the earliest-violation time t*, and every node that starts at or after t* is skipped. Which nodes get skipped depends on the order the search visits them, so the node count of a refused check is a cost measure, not part of the answer (the fork documents the same for its threads). The fork walks one node at a time, left to right; this package decides a round of up to 2048 nodes at once (all segments of one degree together), so a few nodes that the fork would have skipped are already in flight when t* drops.

- refused checks whose count differs: 62 of 188; the package visits 1 to 139 more nodes (mean 13.6), never fewer;
- the fork against itself, 1 thread vs Parallelism.Max(): 33 of 188 differ;
- the verdict and the witness (pair, time, configuration) are identical on all of them.

## Wall time per curve

Each curve: one untimed warmup, then the median of 5 timed runs; a curve's wall is the sum over its checkers. The fork's binding releases the GIL and runs its C++ worker threads; this package is pure Python on stock pydrake, whose distance queries hold the GIL, so it evaluates serially whatever Options.parallelism says (the Max() row is the same code path, measured).

| build | threads | median (ms) | p95 (ms) | max (ms) |
|---|---|---|---|---|
| fork (pydrake.planning.continuous_collision) | 1 | 31.7 | 66.9 | 159.7 |
| fork (pydrake.planning.continuous_collision) | Parallelism.Max() | 18.4 | 25.1 | 28.6 |
| this package | 1 | 39.7 | 69.2 | 86.0 |
| this package | Parallelism.Max() | 39.2 | 69.0 | 84.4 |

Per-curve ratio package / fork (both 1 thread): median 1.22, p95 2.05, max 2.79.

Nodes per curve (package): median 469, p95 1182, max 3698.

Machine: x86_64, Linux 6.8.0-138-generic; fork pydrake /home/peter/gitcspace/drake_for_frankas/install-ccd/lib/python3.10/site-packages/pydrake/__init__.py; package pydrake /home/peter/gitcspace/continuous-collision/.venv/lib/python3.10/site-packages/pydrake/__init__.py.

## Per curve

| leg | cfg | candidate | recorded | fork | package | fork nodes | package nodes | fork 1t ms | fork max ms | package ms |
|---|---|---|---|---|---|---|---|---|---|---|
| e0l00_approach | A | iterate0 | certified | certified | certified | 58+312 | 58+312 | 25.2 | 17.0 | 33.1 |
| e0l00_approach | A | iterate1 | certified | certified | certified | 68+208 | 68+208 | 20.7 | 16.6 | 28.2 |
| e0l00_approach | A | iterate2 | refused | violation | violation | 56+397 | 56+404 | 20.0 | 14.1 | 32.6 |
| e0l00_approach | A | iterate3 | refused | violation | violation | 54+406 | 54+406 | 21.0 | 14.3 | 26.5 |
| e0l00_approach | A | straight | refused | violation | violation | 53+98 | 53+98 | 11.0 | 11.8 | 20.6 |
| e0l00_approach | B | iterate0 | certified | certified | certified | 58+312 | 58+312 | 25.2 | 17.2 | 32.7 |
| e0l00_approach | B | iterate1 | certified | certified | certified | 68+208 | 68+208 | 20.7 | 16.5 | 28.8 |
| e0l00_approach | B | iterate2 | refused | violation | violation | 56+401 | 56+408 | 20.1 | 14.3 | 32.0 |
| e0l00_approach | B | iterate3 | refused | violation | violation | 54+407 | 54+407 | 21.1 | 14.4 | 26.6 |
| e0l00_approach | B | straight | refused | violation | violation | 53+98 | 53+98 | 10.9 | 11.5 | 20.6 |
| e0l01_transport | A | iterate0 | certified | certified | certified | 580+336 | 580+336 | 64.9 | 28.2 | 68.3 |
| e0l01_transport | A | iterate1 | certified | certified | certified | 376+256 | 376+256 | 53.3 | 25.6 | 61.8 |
| e0l01_transport | A | iterate2 | certified | certified | certified | 280+268 | 280+268 | 46.7 | 24.9 | 59.7 |
| e0l01_transport | A | iterate3 | certified | certified | certified | 230+210 | 230+210 | 42.3 | 21.8 | 55.4 |
| e0l01_transport | A | iterate4 | certified | certified | certified | 250+356 | 250+356 | 45.0 | 22.4 | 55.9 |
| e0l01_transport | A | iterate5 | refused | violation | violation | 246+312 | 246+312 | 32.3 | 21.8 | 42.5 |
| e0l01_transport | A | iterate6 | certified | certified | certified | 246+500 | 246+500 | 48.4 | 22.4 | 58.8 |
| e0l01_transport | A | straight | refused | violation | violation | 275+159 | 275+159 | 25.5 | 19.0 | 40.0 |
| e0l01_transport | B | iterate0 | certified | certified | certified | 580+336 | 580+336 | 65.0 | 28.2 | 67.4 |
| e0l01_transport | B | iterate1 | certified | certified | certified | 364+250 | 364+250 | 52.8 | 25.2 | 61.1 |
| e0l01_transport | B | iterate2 | certified | certified | certified | 268+314 | 268+314 | 47.5 | 25.4 | 59.6 |
| e0l01_transport | B | iterate3 | certified | certified | certified | 244+228 | 244+228 | 43.5 | 21.7 | 55.5 |
| e0l01_transport | B | iterate4 | certified | certified | certified | 266+294 | 266+294 | 44.3 | 20.8 | 55.8 |
| e0l01_transport | B | iterate5 | certified | certified | certified | 246+640 | 246+640 | 50.0 | 24.9 | 58.1 |
| e0l01_transport | B | straight | refused | violation | violation | 275+159 | 275+159 | 25.6 | 19.1 | 38.8 |
| e0l02_approach | A | iterate0 | certified | certified | certified | 120+724 | 120+724 | 66.1 | 19.8 | 72.2 |
| e0l02_approach | A | iterate1 | certified | certified | certified | 106+420 | 106+420 | 48.0 | 19.2 | 56.5 |
| e0l02_approach | A | iterate2 | certified | certified | certified | 104+430 | 104+430 | 47.2 | 18.4 | 54.9 |
| e0l02_approach | A | iterate3 | certified | certified | certified | 106+608 | 106+608 | 53.6 | 18.8 | 59.9 |
| e0l02_approach | A | iterate4 | certified | certified | certified | 100+814 | 100+814 | 61.2 | 20.0 | 64.7 |
| e0l02_approach | A | straight | refused | violation | violation | 109+105 | 109+105 | 17.9 | 18.4 | 26.5 |
| e0l02_approach | B | iterate0 | certified | certified | certified | 120+724 | 120+724 | 66.2 | 20.7 | 72.0 |
| e0l02_approach | B | iterate1 | certified | certified | certified | 106+420 | 106+420 | 48.0 | 19.3 | 54.8 |
| e0l02_approach | B | iterate2 | certified | certified | certified | 104+428 | 104+428 | 46.9 | 18.2 | 53.9 |
| e0l02_approach | B | iterate3 | certified | certified | certified | 104+608 | 104+608 | 53.6 | 18.8 | 59.4 |
| e0l02_approach | B | iterate4 | certified | certified | certified | 104+894 | 104+894 | 63.3 | 19.8 | 64.4 |
| e0l02_approach | B | straight | refused | violation | violation | 109+105 | 109+105 | 17.9 | 18.0 | 26.6 |
| e0l03_transport | A | iterate0 | certified | certified | certified | 381+221 | 381+221 | 40.5 | 20.5 | 48.6 |
| e0l03_transport | A | iterate1 | certified | certified | certified | 305+187 | 305+187 | 35.5 | 21.6 | 43.8 |
| e0l03_transport | A | iterate2 | certified | certified | certified | 411+227 | 411+227 | 40.4 | 22.1 | 47.2 |
| e0l03_transport | A | iterate3 | certified | certified | certified | 445+223 | 445+223 | 45.6 | 19.8 | 51.4 |
| e0l03_transport | A | iterate4 | refused | violation | violation | 450+199 | 466+199 | 44.5 | 22.3 | 60.5 |
| e0l03_transport | A | straight | refused | violation | violation | 165+89 | 165+89 | 15.7 | 13.8 | 33.2 |
| e0l03_transport | B | iterate0 | certified | certified | certified | 381+221 | 381+221 | 40.7 | 20.6 | 48.5 |
| e0l03_transport | B | iterate1 | certified | certified | certified | 323+157 | 323+157 | 34.6 | 21.1 | 43.7 |
| e0l03_transport | B | iterate2 | certified | certified | certified | 409+175 | 409+175 | 41.6 | 22.3 | 48.4 |
| e0l03_transport | B | iterate3 | certified | certified | certified | 411+195 | 411+195 | 42.2 | 19.6 | 47.0 |
| e0l03_transport | B | straight | refused | violation | violation | 165+89 | 165+89 | 15.6 | 13.9 | 33.7 |
| e0l04_approach | A | iterate0 | certified | certified | certified | 47+537 | 47+537 | 28.3 | 15.1 | 33.9 |
| e0l04_approach | A | iterate1 | certified | certified | certified | 39+227 | 39+227 | 20.7 | 13.0 | 26.2 |
| e0l04_approach | A | iterate2 | refused | violation | violation | 47+289 | 47+289 | 13.8 | 10.9 | 21.8 |
| e0l04_approach | A | iterate3 | certified | certified | certified | 49+225 | 49+225 | 21.1 | 12.5 | 26.9 |
| e0l04_approach | A | straight | refused | violation | violation | 39+125 | 39+125 | 10.4 | 10.4 | 19.7 |
| e0l04_approach | B | iterate0 | certified | certified | certified | 47+537 | 47+537 | 28.5 | 15.1 | 33.4 |
| e0l04_approach | B | iterate1 | certified | certified | certified | 39+223 | 39+223 | 21.0 | 13.2 | 25.8 |
| e0l04_approach | B | iterate2 | refused | violation | violation | 45+395 | 45+425 | 15.9 | 11.1 | 30.4 |
| e0l04_approach | B | iterate3 | certified | certified | certified | 49+229 | 49+229 | 20.6 | 12.6 | 26.1 |
| e0l04_approach | B | straight | refused | violation | violation | 39+125 | 39+125 | 10.4 | 10.3 | 20.0 |
| e0l05_transport | A | iterate0 | certified | certified | certified | 332+704 | 332+704 | 72.1 | 23.2 | 73.8 |
| e0l05_transport | A | iterate1 | certified | certified | certified | 290+394 | 290+394 | 54.9 | 19.8 | 61.2 |
| e0l05_transport | A | iterate2 | certified | certified | certified | 338+324 | 338+324 | 51.5 | 21.5 | 58.5 |
| e0l05_transport | A | iterate3 | certified | certified | certified | 346+304 | 346+304 | 52.7 | 21.5 | 59.4 |
| e0l05_transport | A | iterate4 | certified | certified | certified | 302+1074 | 302+1074 | 66.4 | 21.6 | 67.1 |
| e0l05_transport | A | iterate5 | certified | certified | certified | 482+446 | 482+446 | 61.1 | 20.7 | 64.9 |
| e0l05_transport | A | straight | refused | violation | violation | 272+400 | 272+400 | 31.5 | 20.3 | 40.6 |
| e0l05_transport | B | iterate0 | certified | certified | certified | 332+704 | 332+704 | 72.0 | 23.4 | 73.6 |
| e0l05_transport | B | iterate1 | certified | certified | certified | 280+432 | 280+432 | 56.4 | 20.1 | 61.0 |
| e0l05_transport | B | iterate2 | certified | certified | certified | 266+304 | 266+304 | 49.6 | 21.6 | 55.4 |
| e0l05_transport | B | iterate3 | certified | certified | certified | 244+294 | 244+294 | 47.9 | 20.3 | 55.5 |
| e0l05_transport | B | iterate4 | certified | certified | certified | 338+312 | 338+312 | 52.6 | 20.5 | 57.1 |
| e0l05_transport | B | iterate5 | certified | certified | certified | 306+356 | 306+356 | 52.1 | 20.5 | 56.9 |
| e0l05_transport | B | straight | refused | violation | violation | 272+400 | 272+400 | 31.6 | 20.1 | 40.2 |
| e0l06_home | A | iterate0 | certified | certified | certified | 73+497 | 73+497 | 50.9 | 23.2 | 56.8 |
| e0l06_home | A | iterate1 | refused | violation | violation | 57+464 | 57+465 | 34.1 | 17.7 | 38.6 |
| e0l06_home | A | iterate2 | refused | violation | violation | 53+814 | 53+816 | 38.4 | 17.4 | 37.5 |
| e0l06_home | A | iterate3 | refused | violation | violation | 45+815 | 45+815 | 27.1 | 14.0 | 28.6 |
| e0l06_home | A | iterate4 | certified | certified | certified | 43+397 | 43+397 | 42.1 | 18.1 | 47.6 |
| e0l06_home | A | iterate5 | certified | certified | certified | 55+261 | 55+261 | 37.7 | 18.7 | 47.6 |
| e0l06_home | A | straight | refused | violation | violation | 59+88 | 59+88 | 15.6 | 16.1 | 31.0 |
| e0l06_home | B | iterate0 | certified | certified | certified | 73+497 | 73+497 | 50.8 | 23.6 | 56.7 |
| e0l06_home | B | iterate1 | refused | violation | violation | 57+464 | 57+465 | 33.9 | 17.6 | 39.0 |
| e0l06_home | B | iterate2 | refused | violation | violation | 53+1372 | 53+1372 | 63.5 | 20.3 | 67.5 |
| e0l06_home | B | iterate3 | certified | certified | certified | 47+961 | 47+961 | 62.6 | 21.4 | 62.7 |
| e0l06_home | B | iterate4 | refused | violation | violation | 43+504 | 43+504 | 36.6 | 19.2 | 43.8 |
| e0l06_home | B | iterate5 | certified | certified | certified | 47+335 | 47+335 | 39.9 | 19.0 | 45.4 |
| e0l06_home | B | straight | refused | violation | violation | 59+88 | 59+88 | 15.9 | 16.0 | 30.4 |
| e0l07_approach | A | iterate0 | certified | certified | certified | 50+176 | 50+176 | 20.8 | 17.1 | 28.9 |
| e0l07_approach | A | iterate1 | refused | violation | violation | 54+218 | 54+225 | 20.9 | 17.8 | 38.5 |
| e0l07_approach | A | iterate2 | refused | violation | violation | 50+150 | 50+150 | 16.8 | 15.7 | 33.5 |
| e0l07_approach | A | straight | refused | violation | violation | 47+132 | 47+132 | 14.9 | 14.1 | 29.7 |
| e0l07_approach | B | iterate0 | certified | certified | certified | 50+176 | 50+176 | 20.9 | 16.9 | 29.0 |
| e0l07_approach | B | iterate1 | refused | violation | violation | 54+220 | 54+227 | 21.1 | 17.3 | 39.0 |
| e0l07_approach | B | iterate2 | refused | violation | violation | 50+148 | 50+148 | 16.7 | 15.5 | 33.0 |
| e0l07_approach | B | straight | refused | violation | violation | 47+132 | 47+132 | 14.8 | 13.9 | 30.0 |
| e0l08_transport | A | iterate0 | certified | certified | certified | 344+274 | 344+274 | 36.7 | 19.5 | 41.5 |
| e0l08_transport | A | iterate1 | certified | certified | certified | 410+222 | 410+222 | 36.8 | 19.4 | 39.2 |
| e0l08_transport | A | iterate2 | certified | certified | certified | 214+140 | 214+140 | 26.2 | 18.2 | 34.1 |
| e0l08_transport | A | straight | refused | violation | violation | 186+75 | 196+75 | 16.6 | 14.2 | 32.9 |
| e0l08_transport | B | iterate0 | certified | certified | certified | 344+274 | 344+274 | 36.7 | 19.6 | 41.0 |
| e0l08_transport | B | iterate1 | certified | certified | certified | 426+222 | 426+222 | 37.1 | 19.1 | 39.7 |
| e0l08_transport | B | iterate2 | certified | certified | certified | 360+202 | 360+202 | 34.2 | 20.3 | 38.7 |
| e0l08_transport | B | straight | refused | violation | violation | 186+75 | 196+75 | 16.6 | 14.4 | 32.8 |
| e0l09_approach | A | iterate0 | certified | certified | certified | 101+451 | 101+451 | 38.6 | 17.1 | 39.7 |
| e0l09_approach | A | iterate1 | certified | certified | certified | 77+165 | 77+165 | 23.3 | 16.0 | 29.8 |
| e0l09_approach | A | iterate2 | certified | certified | certified | 71+157 | 71+157 | 21.9 | 16.0 | 29.4 |
| e0l09_approach | A | straight | refused | violation | violation | 103+179 | 103+179 | 17.1 | 14.6 | 33.4 |
| e0l09_approach | B | iterate0 | certified | certified | certified | 101+451 | 101+451 | 38.7 | 17.2 | 39.8 |
| e0l09_approach | B | iterate1 | certified | certified | certified | 77+163 | 77+163 | 23.0 | 16.1 | 29.4 |
| e0l09_approach | B | straight | refused | violation | violation | 103+179 | 103+179 | 17.0 | 14.7 | 32.9 |
| e0l10_transport | A | iterate0 | certified | certified | certified | 531+981 | 531+981 | 67.1 | 22.1 | 68.6 |
| e0l10_transport | A | iterate1 | refused | violation | violation | 263+456 | 263+463 | 30.6 | 17.5 | 46.4 |
| e0l10_transport | A | iterate2 | certified | certified | certified | 269+287 | 269+287 | 37.5 | 19.5 | 47.0 |
| e0l10_transport | A | straight | refused | violation | violation | 54+81 | 55+81 | 12.5 | 12.9 | 31.4 |
| e0l10_transport | B | iterate0 | certified | certified | certified | 531+981 | 531+981 | 67.1 | 22.0 | 67.8 |
| e0l10_transport | B | iterate1 | certified | certified | certified | 271+283 | 271+283 | 40.2 | 19.9 | 49.9 |
| e0l10_transport | B | iterate2 | certified | certified | certified | 259+211 | 259+211 | 35.6 | 19.9 | 46.5 |
| e0l10_transport | B | iterate3 | certified | certified | certified | 321+205 | 321+205 | 37.2 | 20.0 | 47.5 |
| e0l10_transport | B | straight | refused | violation | violation | 54+81 | 55+81 | 12.5 | 12.8 | 31.4 |
| e0l11_approach | A | iterate0 | certified | certified | certified | 114+650 | 114+650 | 59.4 | 24.0 | 58.8 |
| e0l11_approach | A | iterate1 | certified | certified | certified | 110+862 | 110+862 | 58.2 | 21.1 | 56.0 |
| e0l11_approach | A | iterate2 | refused | violation | violation | 114+364 | 114+364 | 19.9 | 15.4 | 26.0 |
| e0l11_approach | A | iterate3 | certified | certified | certified | 116+496 | 116+496 | 50.3 | 22.6 | 52.9 |
| e0l11_approach | A | straight | refused | violation | violation | 97+183 | 97+183 | 19.5 | 17.1 | 28.9 |
| e0l11_approach | B | iterate0 | certified | certified | certified | 114+650 | 114+650 | 58.9 | 24.2 | 58.8 |
| e0l11_approach | B | iterate1 | certified | certified | certified | 120+808 | 120+808 | 57.6 | 20.6 | 57.0 |
| e0l11_approach | B | iterate2 | certified | certified | certified | 118+542 | 118+542 | 50.8 | 19.3 | 52.2 |
| e0l11_approach | B | iterate3 | certified | certified | certified | 120+400 | 120+400 | 46.5 | 19.7 | 48.9 |
| e0l11_approach | B | straight | refused | violation | violation | 97+183 | 97+183 | 19.6 | 17.4 | 28.8 |
| e0l12_transport | A | iterate0 | certified | certified | certified | 303+333 | 303+333 | 55.8 | 25.8 | 64.7 |
| e0l12_transport | A | iterate1 | certified | certified | certified | 331+289 | 331+289 | 52.9 | 26.4 | 61.4 |
| e0l12_transport | A | iterate2 | certified | certified | certified | 717+261 | 717+261 | 59.7 | 25.1 | 59.2 |
| e0l12_transport | A | straight | refused | violation | violation | 93+119 | 93+119 | 16.7 | 16.0 | 36.4 |
| e0l12_transport | B | iterate0 | certified | certified | certified | 303+333 | 303+333 | 55.8 | 25.7 | 64.1 |
| e0l12_transport | B | iterate1 | certified | certified | certified | 337+301 | 337+301 | 53.3 | 26.2 | 61.3 |
| e0l12_transport | B | iterate2 | certified | certified | certified | 775+239 | 775+239 | 59.9 | 25.8 | 59.6 |
| e0l12_transport | B | iterate3 | certified | certified | certified | 475+225 | 475+225 | 52.9 | 24.1 | 57.4 |
| e0l12_transport | B | straight | refused | violation | violation | 93+119 | 93+119 | 16.6 | 16.4 | 36.3 |
| e0l13_home | A | iterate0 | certified | certified | certified | 60+696 | 60+696 | 29.0 | 10.6 | 32.8 |
| e0l13_home | A | iterate1 | certified | certified | certified | 56+366 | 56+366 | 22.0 | 12.1 | 27.5 |
| e0l13_home | A | iterate2 | certified | certified | certified | 56+206 | 56+206 | 18.5 | 11.9 | 24.5 |
| e0l13_home | A | straight | refused | violation | violation | 55+66 | 55+66 | 8.9 | 9.6 | 17.7 |
| e0l13_home | B | iterate0 | certified | certified | certified | 60+696 | 60+696 | 29.1 | 10.6 | 32.4 |
| e0l13_home | B | iterate1 | certified | certified | certified | 56+390 | 56+390 | 22.2 | 12.0 | 27.8 |
| e0l13_home | B | straight | refused | violation | violation | 55+66 | 55+66 | 9.1 | 9.6 | 18.0 |
| e1l00_approach | A | iterate0 | certified | certified | certified | 67+469 | 67+469 | 29.5 | 16.1 | 32.7 |
| e1l00_approach | A | iterate1 | certified | certified | certified | 61+101 | 61+101 | 15.5 | 14.4 | 23.7 |
| e1l00_approach | A | iterate2 | certified | certified | certified | 67+75 | 67+75 | 14.2 | 15.4 | 22.2 |
| e1l00_approach | A | iterate3 | certified | certified | certified | 69+111 | 69+111 | 14.9 | 13.9 | 22.7 |
| e1l00_approach | A | iterate4 | certified | certified | certified | 67+159 | 67+159 | 16.3 | 13.7 | 24.1 |
| e1l00_approach | A | iterate5 | certified | certified | certified | 67+145 | 67+145 | 16.3 | 13.7 | 23.7 |
| e1l00_approach | A | straight | refused | violation | violation | 51+114 | 51+114 | 11.2 | 11.9 | 20.7 |
| e1l00_approach | B | iterate0 | certified | certified | certified | 67+469 | 67+469 | 28.9 | 16.3 | 32.7 |
| e1l00_approach | B | iterate1 | certified | certified | certified | 63+101 | 63+101 | 15.5 | 14.2 | 23.9 |
| e1l00_approach | B | iterate2 | certified | certified | certified | 67+75 | 67+75 | 14.1 | 15.4 | 21.6 |
| e1l00_approach | B | iterate3 | certified | certified | certified | 67+109 | 67+109 | 14.9 | 13.7 | 22.7 |
| e1l00_approach | B | iterate4 | certified | certified | certified | 67+159 | 67+159 | 16.5 | 13.6 | 24.0 |
| e1l00_approach | B | straight | refused | violation | violation | 51+114 | 51+114 | 11.3 | 11.4 | 20.4 |
| e1l01_transport | A | iterate0 | certified | certified | certified | 499+627 | 499+627 | 59.7 | 16.5 | 60.9 |
| e1l01_transport | A | iterate1 | refused | violation | violation | 343+590 | 343+590 | 47.8 | 19.5 | 45.4 |
| e1l01_transport | A | iterate2 | certified | certified | certified | 317+293 | 317+293 | 40.1 | 19.0 | 47.0 |
| e1l01_transport | A | straight | refused | violation | violation | 81+123 | 92+123 | 14.0 | 14.3 | 32.0 |
| e1l01_transport | B | iterate0 | certified | certified | certified | 499+627 | 499+627 | 60.0 | 16.4 | 59.9 |
| e1l01_transport | B | iterate1 | refused | violation | violation | 327+662 | 327+662 | 49.8 | 19.7 | 46.4 |
| e1l01_transport | B | straight | refused | violation | violation | 81+123 | 92+123 | 13.9 | 14.5 | 31.9 |
| e1l02_approach | A | iterate0 | certified | certified | certified | 120+644 | 120+644 | 51.3 | 21.0 | 51.4 |
| e1l02_approach | A | iterate1 | certified | certified | certified | 104+1116 | 104+1116 | 62.8 | 18.4 | 52.0 |
| e1l02_approach | A | iterate2 | refused | violation | violation | 104+506 | 104+506 | 29.3 | 15.7 | 29.2 |
| e1l02_approach | A | straight | refused | violation | violation | 99+398 | 99+398 | 27.6 | 17.8 | 36.4 |
| e1l02_approach | B | iterate0 | certified | certified | certified | 120+644 | 120+644 | 51.3 | 21.5 | 51.4 |
| e1l02_approach | B | iterate1 | certified | certified | certified | 104+1116 | 104+1116 | 64.2 | 18.4 | 51.8 |
| e1l02_approach | B | iterate2 | refused | violation | violation | 104+509 | 104+509 | 29.4 | 15.7 | 29.4 |
| e1l02_approach | B | straight | refused | violation | violation | 99+398 | 99+398 | 27.4 | 18.2 | 36.2 |
| e1l03_transport | A | iterate0 | certified | certified | certified | 519+799 | 519+799 | 75.5 | 21.2 | 70.0 |
| e1l03_transport | A | iterate1 | refused | violation | violation | 315+853 | 315+868 | 52.0 | 19.8 | 61.6 |
| e1l03_transport | A | iterate2 | certified | certified | certified | 295+221 | 295+221 | 45.7 | 20.3 | 56.0 |
| e1l03_transport | A | straight | refused | violation | violation | 92+128 | 92+128 | 15.7 | 16.0 | 33.2 |
| e1l03_transport | B | iterate0 | certified | certified | certified | 519+799 | 519+799 | 75.9 | 21.5 | 71.8 |
| e1l03_transport | B | iterate1 | certified | certified | certified | 469+605 | 469+605 | 59.5 | 20.6 | 61.0 |
| e1l03_transport | B | iterate2 | certified | certified | certified | 455+213 | 455+213 | 50.0 | 20.4 | 55.5 |
| e1l03_transport | B | straight | refused | violation | violation | 92+128 | 92+128 | 15.7 | 16.0 | 33.2 |
| e1l04_approach | A | iterate0 | certified | certified | certified | 107+775 | 107+775 | 68.9 | 23.6 | 64.9 |
| e1l04_approach | A | iterate1 | certified | certified | certified | 91+421 | 91+421 | 47.5 | 22.2 | 49.9 |
| e1l04_approach | A | iterate2 | certified | certified | certified | 83+311 | 83+311 | 41.3 | 20.1 | 46.7 |
| e1l04_approach | A | iterate3 | certified | certified | certified | 87+413 | 87+413 | 45.9 | 17.3 | 49.3 |
| e1l04_approach | A | iterate4 | certified | certified | certified | 89+355 | 89+355 | 43.5 | 16.6 | 51.3 |
| e1l04_approach | A | straight | refused | violation | violation | 89+140 | 89+140 | 18.2 | 17.3 | 31.3 |
| e1l04_approach | B | iterate0 | certified | certified | certified | 107+775 | 107+775 | 69.0 | 24.2 | 65.0 |
| e1l04_approach | B | iterate1 | certified | certified | certified | 91+421 | 91+421 | 47.2 | 19.7 | 49.9 |
| e1l04_approach | B | iterate2 | certified | certified | certified | 83+301 | 83+301 | 41.0 | 20.2 | 46.2 |
| e1l04_approach | B | iterate3 | certified | certified | certified | 95+341 | 95+341 | 42.7 | 16.8 | 48.2 |
| e1l04_approach | B | iterate4 | certified | certified | certified | 89+395 | 89+395 | 44.3 | 16.7 | 47.9 |
| e1l04_approach | B | straight | refused | violation | violation | 89+140 | 89+140 | 18.2 | 17.6 | 31.1 |
| e1l05_transport | A | iterate0 | certified | certified | certified | 317+131 | 317+131 | 27.8 | 15.9 | 35.6 |
| e1l05_transport | A | iterate1 | certified | certified | certified | 259+117 | 259+117 | 29.8 | 20.5 | 38.3 |
| e1l05_transport | A | iterate2 | certified | certified | certified | 187+125 | 187+125 | 22.1 | 13.6 | 30.8 |
| e1l05_transport | A | straight | refused | violation | violation | 186+185 | 186+185 | 19.2 | 16.8 | 31.6 |
| e1l05_transport | B | iterate0 | certified | certified | certified | 317+131 | 317+131 | 27.8 | 16.0 | 35.4 |
| e1l05_transport | B | iterate1 | certified | certified | certified | 243+115 | 243+115 | 28.6 | 20.4 | 38.1 |
| e1l05_transport | B | iterate2 | certified | certified | certified | 255+101 | 255+101 | 23.2 | 15.7 | 34.9 |
| e1l05_transport | B | straight | refused | violation | violation | 186+185 | 186+185 | 19.4 | 14.7 | 32.2 |
| e1l06_home | A | iterate0 | certified | certified | certified | 24+172 | 24+172 | 14.8 | 10.1 | 21.1 |
| e1l06_home | A | iterate1 | certified | certified | certified | 26+150 | 26+150 | 14.8 | 11.7 | 22.0 |
| e1l06_home | A | straight | refused | violation | violation | 17+145 | 17+146 | 10.3 | 8.8 | 24.6 |
| e1l06_home | B | iterate0 | certified | certified | certified | 24+172 | 24+172 | 14.8 | 10.1 | 21.2 |
| e1l06_home | B | iterate1 | certified | certified | certified | 26+150 | 26+150 | 14.8 | 11.8 | 22.2 |
| e1l06_home | B | straight | refused | violation | violation | 17+145 | 17+146 | 10.3 | 8.9 | 24.5 |
| e1l07_approach | A | iterate0 | certified | certified | certified | 50+176 | 50+176 | 20.8 | 17.3 | 28.8 |
| e1l07_approach | A | iterate1 | refused | violation | violation | 54+218 | 54+225 | 20.9 | 17.7 | 37.6 |
| e1l07_approach | A | iterate2 | refused | violation | violation | 50+150 | 50+150 | 16.8 | 15.5 | 32.9 |
| e1l07_approach | A | straight | refused | violation | violation | 47+132 | 47+132 | 14.8 | 14.0 | 29.4 |
| e1l07_approach | B | iterate0 | certified | certified | certified | 50+176 | 50+176 | 20.8 | 17.4 | 28.6 |
| e1l07_approach | B | iterate1 | refused | violation | violation | 54+220 | 54+227 | 21.0 | 17.7 | 38.3 |
| e1l07_approach | B | iterate2 | refused | violation | violation | 50+148 | 50+148 | 16.6 | 15.4 | 32.2 |
| e1l07_approach | B | straight | refused | violation | violation | 47+132 | 47+132 | 14.9 | 13.9 | 29.2 |
| e1l08_transport | A | iterate0 | certified | certified | certified | 229+397 | 229+397 | 36.5 | 17.4 | 39.7 |
| e1l08_transport | A | iterate1 | certified | certified | certified | 133+291 | 133+291 | 29.2 | 19.9 | 34.8 |
| e1l08_transport | A | iterate2 | certified | certified | certified | 159+149 | 159+149 | 28.1 | 20.2 | 36.1 |
| e1l08_transport | A | straight | refused | violation | violation | 73+194 | 77+194 | 17.8 | 16.7 | 35.2 |
| e1l08_transport | B | iterate0 | certified | certified | certified | 229+397 | 229+397 | 36.6 | 18.7 | 40.2 |
| e1l08_transport | B | iterate1 | certified | certified | certified | 121+255 | 121+255 | 27.6 | 20.0 | 34.0 |
| e1l08_transport | B | iterate2 | certified | certified | certified | 169+157 | 169+157 | 28.9 | 19.9 | 36.4 |
| e1l08_transport | B | straight | refused | violation | violation | 73+194 | 77+194 | 17.5 | 16.7 | 35.5 |
| e1l09_approach | A | iterate0 | certified | certified | certified | 120+1504 | 120+1504 | 77.3 | 20.6 | 70.7 |
| e1l09_approach | A | iterate1 | refused | violation | violation | 124+752 | 124+752 | 56.4 | 21.6 | 60.3 |
| e1l09_approach | A | iterate2 | refused | violation | violation | 122+292 | 122+300 | 31.4 | 19.8 | 49.0 |
| e1l09_approach | A | iterate3 | certified | certified | certified | 116+590 | 116+590 | 51.0 | 20.1 | 57.2 |
| e1l09_approach | A | iterate4 | certified | certified | certified | 114+540 | 114+540 | 54.8 | 20.7 | 59.8 |
| e1l09_approach | A | iterate5 | refused | violation | violation | 128+480 | 128+480 | 41.9 | 18.2 | 50.3 |
| e1l09_approach | A | iterate6 | certified | certified | certified | 140+358 | 140+358 | 53.3 | 21.9 | 62.5 |
| e1l09_approach | A | iterate7 | certified | certified | certified | 140+494 | 140+494 | 60.4 | 23.0 | 70.2 |
| e1l09_approach | A | iterate8 | refused | violation | violation | 146+429 | 146+453 | 39.9 | 21.4 | 76.0 |
| e1l09_approach | A | straight | refused | violation | violation | 103+94 | 103+94 | 15.8 | 17.3 | 26.0 |
| e1l09_approach | B | iterate0 | certified | certified | certified | 120+1504 | 120+1504 | 77.3 | 20.5 | 71.1 |
| e1l09_approach | B | iterate1 | certified | certified | certified | 124+518 | 124+518 | 51.6 | 20.8 | 56.8 |
| e1l09_approach | B | iterate2 | certified | certified | certified | 120+320 | 120+320 | 41.8 | 20.0 | 51.4 |
| e1l09_approach | B | iterate3 | refused | violation | violation | 116+380 | 116+389 | 28.7 | 17.6 | 43.6 |
| e1l09_approach | B | iterate4 | refused | violation | violation | 122+305 | 122+329 | 26.2 | 18.2 | 49.6 |
| e1l09_approach | B | iterate5 | refused | violation | violation | 136+422 | 136+432 | 33.4 | 18.3 | 49.9 |
| e1l09_approach | B | iterate6 | refused | violation | violation | 136+304 | 136+359 | 32.2 | 21.6 | 62.9 |
| e1l09_approach | B | iterate7 | refused | violation | violation | 144+582 | 144+586 | 52.3 | 20.5 | 69.8 |
| e1l09_approach | B | iterate8 | refused | violation | violation | 150+566 | 150+570 | 50.7 | 20.2 | 69.8 |
| e1l09_approach | B | straight | refused | violation | violation | 103+94 | 103+94 | 15.9 | 15.8 | 25.6 |
| e1l10_transport | A | iterate0 | certified | certified | certified | 609+331 | 609+331 | 72.7 | 25.8 | 75.4 |
| e1l10_transport | A | iterate1 | certified | certified | certified | 497+1017 | 497+1017 | 86.3 | 26.5 | 73.3 |
| e1l10_transport | A | iterate2 | certified | certified | certified | 407+387 | 407+387 | 56.4 | 22.7 | 57.5 |
| e1l10_transport | A | iterate3 | certified | certified | certified | 303+277 | 303+277 | 47.3 | 22.1 | 53.3 |
| e1l10_transport | A | straight | refused | violation | violation | 67+726 | 67+726 | 39.0 | 19.3 | 43.2 |
| e1l10_transport | B | iterate0 | certified | certified | certified | 609+331 | 609+331 | 72.6 | 25.8 | 75.6 |
| e1l10_transport | B | iterate1 | certified | certified | certified | 497+923 | 497+923 | 81.9 | 26.7 | 71.9 |
| e1l10_transport | B | iterate2 | certified | certified | certified | 405+391 | 405+391 | 56.8 | 22.8 | 58.4 |
| e1l10_transport | B | iterate3 | certified | certified | certified | 279+313 | 279+313 | 47.4 | 22.4 | 51.6 |
| e1l10_transport | B | straight | refused | violation | violation | 67+726 | 67+726 | 39.2 | 19.2 | 43.3 |
| e1l11_approach | A | iterate0 | certified | certified | certified | 110+292 | 110+292 | 28.7 | 16.1 | 31.5 |
| e1l11_approach | A | iterate1 | certified | certified | certified | 90+120 | 90+120 | 20.2 | 19.5 | 26.6 |
| e1l11_approach | A | iterate2 | certified | certified | certified | 102+150 | 102+150 | 21.4 | 16.3 | 26.8 |
| e1l11_approach | A | iterate3 | certified | certified | certified | 96+174 | 96+174 | 22.4 | 17.4 | 27.4 |
| e1l11_approach | A | straight | refused | violation | violation | 101+222 | 101+222 | 16.4 | 13.2 | 22.8 |
| e1l11_approach | B | iterate0 | certified | certified | certified | 110+292 | 110+292 | 28.7 | 16.2 | 31.6 |
| e1l11_approach | B | iterate1 | certified | certified | certified | 90+120 | 90+120 | 20.2 | 19.3 | 26.5 |
| e1l11_approach | B | iterate2 | certified | certified | certified | 102+150 | 102+150 | 21.2 | 16.2 | 26.8 |
| e1l11_approach | B | straight | refused | violation | violation | 101+222 | 101+222 | 16.5 | 13.2 | 22.9 |
| e1l12_transport | A | iterate0 | certified | certified | certified | 361+277 | 361+277 | 38.4 | 20.4 | 42.0 |
| e1l12_transport | A | iterate1 | certified | certified | certified | 207+129 | 207+129 | 24.7 | 18.1 | 33.8 |
| e1l12_transport | A | iterate2 | certified | certified | certified | 173+97 | 173+97 | 21.9 | 17.9 | 31.1 |
| e1l12_transport | A | iterate3 | certified | certified | certified | 311+117 | 311+117 | 26.2 | 17.3 | 32.9 |
| e1l12_transport | A | straight | refused | violation | violation | 167+201 | 170+201 | 19.5 | 16.4 | 34.7 |
| e1l12_transport | B | iterate0 | certified | certified | certified | 361+277 | 361+277 | 38.2 | 20.3 | 41.9 |
| e1l12_transport | B | iterate1 | certified | certified | certified | 207+129 | 207+129 | 24.8 | 18.2 | 33.8 |
| e1l12_transport | B | iterate2 | certified | certified | certified | 173+97 | 173+97 | 21.9 | 17.7 | 30.7 |
| e1l12_transport | B | iterate3 | certified | certified | certified | 257+117 | 257+117 | 24.3 | 16.9 | 31.7 |
| e1l12_transport | B | iterate4 | certified | certified | certified | 289+107 | 289+107 | 25.7 | 18.4 | 32.4 |
| e1l12_transport | B | straight | refused | violation | violation | 167+201 | 170+201 | 19.4 | 16.5 | 34.6 |
| e1l13_home | A | iterate0 | certified | certified | certified | 60+696 | 60+696 | 28.9 | 10.7 | 32.1 |
| e1l13_home | A | iterate1 | certified | certified | certified | 56+366 | 56+366 | 21.8 | 12.1 | 27.3 |
| e1l13_home | A | iterate2 | certified | certified | certified | 56+206 | 56+206 | 18.6 | 12.0 | 24.4 |
| e1l13_home | A | straight | refused | violation | violation | 55+66 | 55+66 | 8.9 | 9.6 | 17.7 |
| e1l13_home | B | iterate0 | certified | certified | certified | 60+696 | 60+696 | 28.8 | 10.7 | 32.2 |
| e1l13_home | B | iterate1 | certified | certified | certified | 56+390 | 56+390 | 22.0 | 12.0 | 27.5 |
| e1l13_home | B | straight | refused | violation | violation | 55+66 | 55+66 | 9.0 | 9.6 | 17.6 |
| e2l00_approach | A | iterate0 | certified | certified | certified | 68+348 | 68+348 | 33.1 | 19.0 | 37.2 |
| e2l00_approach | A | iterate1 | refused | violation | violation | 62+141 | 62+176 | 15.1 | 14.2 | 29.6 |
| e2l00_approach | A | iterate2 | certified | certified | certified | 64+276 | 64+276 | 26.4 | 18.4 | 32.6 |
| e2l00_approach | A | iterate3 | certified | certified | certified | 58+396 | 58+396 | 31.5 | 18.4 | 35.9 |
| e2l00_approach | A | straight | refused | violation | violation | 47+92 | 47+92 | 11.9 | 13.2 | 20.7 |
| e2l00_approach | B | iterate0 | certified | certified | certified | 68+348 | 68+348 | 33.1 | 18.9 | 37.5 |
| e2l00_approach | B | iterate1 | refused | violation | violation | 62+131 | 62+227 | 14.6 | 14.4 | 35.5 |
| e2l00_approach | B | iterate2 | certified | certified | certified | 62+232 | 62+232 | 25.1 | 17.7 | 30.9 |
| e2l00_approach | B | iterate3 | certified | certified | certified | 58+228 | 58+228 | 24.9 | 18.2 | 30.5 |
| e2l00_approach | B | straight | refused | violation | violation | 47+92 | 47+92 | 12.0 | 12.8 | 20.6 |
| e2l01_transport | A | iterate0 | certified | certified | certified | 552+238 | 552+238 | 51.0 | 21.7 | 53.8 |
| e2l01_transport | A | iterate1 | certified | certified | certified | 264+184 | 264+184 | 34.5 | 17.2 | 41.5 |
| e2l01_transport | A | iterate2 | certified | certified | certified | 266+184 | 266+184 | 34.7 | 18.7 | 42.8 |
| e2l01_transport | A | straight | refused | violation | violation | 132+110 | 132+110 | 17.2 | 15.9 | 43.1 |
| e2l01_transport | B | iterate0 | certified | certified | certified | 552+238 | 552+238 | 51.0 | 22.0 | 53.1 |
| e2l01_transport | B | iterate1 | certified | certified | certified | 268+180 | 268+180 | 34.7 | 17.2 | 41.2 |
| e2l01_transport | B | iterate2 | certified | certified | certified | 276+198 | 276+198 | 35.6 | 18.8 | 43.2 |
| e2l01_transport | B | straight | refused | violation | violation | 132+110 | 132+110 | 17.2 | 16.1 | 43.3 |
| e2l02_approach | A | iterate0 | certified | certified | certified | 177+601 | 177+601 | 49.5 | 19.5 | 51.5 |
| e2l02_approach | A | iterate1 | refused | violation | violation | 131+629 | 131+635 | 34.1 | 16.3 | 40.2 |
| e2l02_approach | A | iterate2 | refused | violation | violation | 137+496 | 137+499 | 29.0 | 15.7 | 45.2 |
| e2l02_approach | A | iterate3 | refused | violation | violation | 139+713 | 139+713 | 36.7 | 16.4 | 31.3 |
| e2l02_approach | A | straight | refused | violation | violation | 115+107 | 115+107 | 15.6 | 15.7 | 24.5 |
| e2l02_approach | B | iterate0 | certified | certified | certified | 177+601 | 177+601 | 49.8 | 19.6 | 48.4 |
| e2l02_approach | B | iterate1 | refused | violation | violation | 131+629 | 131+635 | 34.1 | 16.1 | 40.2 |
| e2l02_approach | B | iterate2 | refused | violation | violation | 137+492 | 137+495 | 28.9 | 15.7 | 45.7 |
| e2l02_approach | B | iterate3 | refused | violation | violation | 139+711 | 139+711 | 37.1 | 16.2 | 31.3 |
| e2l02_approach | B | straight | refused | violation | violation | 115+107 | 115+107 | 15.6 | 15.6 | 24.4 |
| e2l03_transport | A | iterate0 | certified | certified | certified | 407+985 | 407+985 | 71.6 | 23.5 | 66.9 |
| e2l03_transport | A | iterate1 | certified | certified | certified | 221+341 | 221+341 | 42.4 | 18.3 | 49.1 |
| e2l03_transport | A | iterate2 | certified | certified | certified | 235+227 | 235+227 | 38.0 | 21.3 | 44.3 |
| e2l03_transport | A | iterate3 | certified | certified | certified | 265+243 | 265+243 | 41.3 | 18.2 | 46.0 |
| e2l03_transport | A | iterate4 | certified | certified | certified | 375+243 | 375+243 | 45.2 | 18.7 | 48.2 |
| e2l03_transport | A | straight | refused | violation | violation | 191+167 | 192+167 | 19.2 | 15.8 | 34.8 |
| e2l03_transport | B | iterate0 | certified | certified | certified | 407+985 | 407+985 | 72.3 | 23.5 | 66.9 |
| e2l03_transport | B | iterate1 | certified | certified | certified | 223+371 | 223+371 | 43.0 | 18.3 | 48.3 |
| e2l03_transport | B | iterate2 | certified | certified | certified | 251+253 | 251+253 | 39.1 | 18.3 | 45.0 |
| e2l03_transport | B | iterate3 | certified | certified | certified | 299+271 | 299+271 | 43.5 | 18.5 | 47.3 |
| e2l03_transport | B | iterate4 | certified | certified | certified | 329+341 | 329+341 | 46.8 | 18.1 | 49.1 |
| e2l03_transport | B | straight | refused | violation | violation | 191+167 | 192+167 | 19.2 | 16.0 | 35.0 |
| e2l04_approach | A | iterate0 | certified | certified | certified | 96+826 | 96+826 | 62.6 | 25.0 | 58.9 |
| e2l04_approach | A | iterate1 | certified | certified | certified | 94+336 | 94+336 | 40.6 | 19.8 | 43.2 |
| e2l04_approach | A | iterate2 | refused | violation | violation | 88+439 | 88+439 | 30.8 | 17.5 | 35.9 |
| e2l04_approach | A | iterate3 | refused | violation | violation | 90+321 | 90+342 | 27.0 | 17.3 | 48.2 |
| e2l04_approach | A | iterate4 | refused | violation | violation | 86+1179 | 86+1179 | 52.1 | 17.1 | 47.9 |
| e2l04_approach | A | iterate5 | certified | certified | certified | 82+408 | 82+408 | 42.1 | 17.6 | 44.9 |
| e2l04_approach | A | iterate6 | certified | certified | certified | 88+740 | 88+740 | 54.4 | 18.0 | 49.8 |
| e2l04_approach | A | straight | refused | violation | violation | 89+179 | 89+179 | 20.1 | 18.5 | 26.7 |
| e2l04_approach | B | iterate0 | certified | certified | certified | 96+826 | 96+826 | 62.6 | 24.8 | 59.1 |
| e2l04_approach | B | iterate1 | certified | certified | certified | 94+336 | 94+336 | 40.4 | 20.0 | 43.3 |
| e2l04_approach | B | iterate2 | refused | violation | violation | 88+427 | 88+427 | 30.9 | 17.6 | 36.0 |
| e2l04_approach | B | iterate3 | refused | violation | violation | 90+313 | 90+334 | 26.5 | 17.2 | 48.3 |
| e2l04_approach | B | iterate4 | refused | violation | violation | 86+465 | 86+465 | 26.6 | 15.0 | 36.5 |
| e2l04_approach | B | iterate5 | certified | certified | certified | 82+346 | 82+346 | 39.9 | 17.1 | 43.6 |
| e2l04_approach | B | iterate6 | certified | certified | certified | 96+526 | 96+526 | 46.5 | 17.6 | 46.6 |
| e2l04_approach | B | straight | refused | violation | violation | 89+179 | 89+179 | 20.4 | 18.9 | 26.7 |
| e2l05_transport | A | iterate0 | certified | certified | certified | 356+178 | 356+178 | 38.8 | 19.5 | 45.6 |
| e2l05_transport | A | iterate1 | certified | certified | certified | 272+144 | 272+144 | 30.5 | 19.6 | 38.0 |
| e2l05_transport | A | iterate2 | certified | certified | certified | 174+196 | 174+196 | 30.0 | 18.5 | 38.2 |
| e2l05_transport | A | iterate3 | certified | certified | certified | 210+250 | 210+250 | 32.4 | 17.5 | 40.1 |
| e2l05_transport | A | iterate4 | certified | certified | certified | 210+182 | 210+182 | 29.8 | 17.6 | 40.0 |
| e2l05_transport | A | straight | refused | violation | violation | 114+117 | 114+117 | 14.3 | 13.9 | 31.4 |
| e2l05_transport | B | iterate0 | certified | certified | certified | 356+178 | 356+178 | 38.9 | 19.9 | 46.0 |
| e2l05_transport | B | iterate1 | certified | certified | certified | 266+154 | 266+154 | 30.5 | 19.5 | 38.5 |
| e2l05_transport | B | iterate2 | certified | certified | certified | 200+152 | 200+152 | 29.5 | 18.4 | 37.9 |
| e2l05_transport | B | iterate3 | certified | certified | certified | 168+180 | 168+180 | 29.5 | 16.6 | 38.0 |
| e2l05_transport | B | straight | refused | violation | violation | 114+117 | 114+117 | 14.3 | 18.3 | 31.3 |
| e2l06_home | A | iterate0 | certified | certified | certified | 41+313 | 41+313 | 31.5 | 17.2 | 36.0 |
| e2l06_home | A | iterate1 | certified | certified | certified | 37+297 | 37+297 | 30.0 | 18.1 | 35.1 |
| e2l06_home | A | iterate2 | certified | certified | certified | 35+177 | 35+177 | 25.3 | 19.1 | 31.5 |
| e2l06_home | A | iterate3 | certified | certified | certified | 39+131 | 39+131 | 23.3 | 20.2 | 29.8 |
| e2l06_home | A | iterate4 | certified | certified | certified | 39+125 | 39+125 | 23.1 | 18.4 | 29.0 |
| e2l06_home | A | iterate5 | certified | certified | certified | 39+131 | 39+131 | 23.2 | 18.6 | 28.7 |
| e2l06_home | A | straight | refused | violation | violation | 35+83 | 35+83 | 11.4 | 12.3 | 26.1 |
| e2l06_home | B | iterate0 | certified | certified | certified | 41+313 | 41+313 | 31.2 | 16.8 | 36.0 |
| e2l06_home | B | iterate1 | certified | certified | certified | 37+423 | 37+423 | 34.6 | 18.6 | 38.4 |
| e2l06_home | B | iterate2 | certified | certified | certified | 33+237 | 33+237 | 27.5 | 19.4 | 33.7 |
| e2l06_home | B | iterate3 | certified | certified | certified | 37+155 | 37+155 | 24.3 | 19.4 | 30.2 |
| e2l06_home | B | iterate4 | certified | certified | certified | 37+189 | 37+189 | 25.9 | 18.3 | 30.9 |
| e2l06_home | B | iterate5 | certified | certified | certified | 37+185 | 37+185 | 25.8 | 18.2 | 31.0 |
| e2l06_home | B | straight | refused | violation | violation | 35+83 | 35+83 | 11.4 | 11.9 | 26.1 |
| e2l07_approach | A | iterate0 | certified | certified | certified | 96+442 | 96+442 | 41.6 | 25.1 | 42.0 |
| e2l07_approach | A | iterate1 | certified | certified | certified | 100+406 | 100+406 | 38.6 | 23.3 | 39.8 |
| e2l07_approach | A | iterate2 | certified | certified | certified | 92+332 | 92+332 | 36.2 | 22.5 | 38.9 |
| e2l07_approach | A | iterate3 | certified | certified | certified | 90+334 | 90+334 | 36.9 | 23.9 | 39.4 |
| e2l07_approach | A | straight | refused | violation | violation | 81+291 | 81+291 | 26.8 | 22.1 | 34.9 |
| e2l07_approach | B | iterate0 | certified | certified | certified | 96+442 | 96+442 | 42.2 | 25.2 | 42.0 |
| e2l07_approach | B | iterate1 | certified | certified | certified | 100+438 | 100+438 | 39.8 | 23.4 | 41.7 |
| e2l07_approach | B | iterate2 | certified | certified | certified | 92+338 | 92+338 | 36.8 | 22.8 | 38.9 |
| e2l07_approach | B | iterate3 | certified | certified | certified | 90+340 | 90+340 | 36.8 | 23.9 | 39.3 |
| e2l07_approach | B | straight | refused | violation | violation | 81+291 | 81+291 | 26.9 | 22.1 | 34.9 |
| e2l08_transport | A | iterate0 | certified | certified | certified | 454+728 | 454+728 | 68.1 | 24.9 | 71.3 |
| e2l08_transport | A | iterate1 | refused | violation | violation | 314+594 | 314+597 | 47.1 | 25.1 | 52.4 |
| e2l08_transport | A | iterate2 | certified | certified | certified | 540+308 | 540+308 | 58.1 | 24.2 | 61.7 |
| e2l08_transport | A | iterate3 | refused | violation | violation | 628+429 | 628+433 | 51.3 | 21.6 | 65.0 |
| e2l08_transport | A | iterate4 | refused | violation | violation | 554+341 | 554+372 | 45.7 | 21.4 | 75.9 |
| e2l08_transport | A | straight | refused | violation | violation | 81+139 | 81+139 | 18.5 | 17.9 | 37.5 |
| e2l08_transport | B | iterate0 | certified | certified | certified | 454+728 | 454+728 | 68.6 | 25.3 | 70.8 |
| e2l08_transport | B | iterate1 | refused | violation | violation | 318+593 | 318+596 | 47.5 | 24.6 | 53.4 |
| e2l08_transport | B | iterate2 | certified | certified | certified | 540+310 | 540+310 | 58.2 | 24.0 | 61.5 |
| e2l08_transport | B | iterate3 | refused | violation | violation | 736+410 | 736+414 | 55.3 | 22.3 | 67.7 |
| e2l08_transport | B | iterate4 | certified | certified | certified | 406+1032 | 406+1032 | 79.6 | 24.4 | 81.2 |
| e2l08_transport | B | iterate5 | certified | certified | certified | 356+548 | 356+548 | 59.8 | 22.8 | 61.5 |
| e2l08_transport | B | straight | refused | violation | violation | 81+139 | 81+139 | 18.4 | 17.8 | 37.5 |
| e2l09_approach | A | iterate0 | certified | certified | certified | 71+755 | 71+755 | 66.7 | 24.4 | 63.0 |
| e2l09_approach | A | iterate1 | refused | violation | violation | 57+368 | 57+416 | 29.8 | 21.6 | 53.1 |
| e2l09_approach | A | iterate2 | certified | certified | certified | 55+589 | 55+589 | 48.4 | 20.0 | 47.5 |
| e2l09_approach | A | iterate3 | refused | violation | violation | 53+1202 | 53+1208 | 56.4 | 18.1 | 46.7 |
| e2l09_approach | A | iterate4 | certified | certified | certified | 59+427 | 59+427 | 44.7 | 19.8 | 44.2 |
| e2l09_approach | A | straight | refused | violation | violation | 97+78 | 97+78 | 14.8 | 15.6 | 27.6 |
| e2l09_approach | B | iterate0 | certified | certified | certified | 71+755 | 71+755 | 66.8 | 24.3 | 62.1 |
| e2l09_approach | B | iterate1 | refused | violation | violation | 57+364 | 57+412 | 29.7 | 22.1 | 52.9 |
| e2l09_approach | B | iterate2 | refused | violation | violation | 53+381 | 53+390 | 27.2 | 18.9 | 39.2 |
| e2l09_approach | B | iterate3 | certified | certified | certified | 55+3643 | 55+3643 | 159.7 | 28.6 | 86.0 |
| e2l09_approach | B | iterate4 | certified | certified | certified | 51+417 | 51+417 | 44.5 | 18.9 | 45.5 |
| e2l09_approach | B | straight | refused | violation | violation | 97+78 | 97+78 | 14.9 | 15.6 | 27.6 |
| e2l10_transport | A | iterate0 | certified | certified | certified | 655+393 | 655+393 | 69.9 | 25.9 | 69.1 |
| e2l10_transport | A | iterate1 | certified | certified | certified | 457+537 | 457+537 | 57.9 | 24.3 | 60.5 |
| e2l10_transport | A | iterate2 | certified | certified | certified | 359+327 | 359+327 | 49.1 | 22.6 | 53.2 |
| e2l10_transport | A | iterate3 | refused | violation | violation | 431+242 | 431+242 | 36.1 | 20.6 | 42.2 |
| e2l10_transport | A | iterate4 | certified | certified | certified | 289+203 | 289+203 | 44.4 | 24.2 | 52.1 |
| e2l10_transport | A | straight | refused | violation | violation | 75+78 | 76+81 | 15.4 | 16.5 | 42.9 |
| e2l10_transport | B | iterate0 | certified | certified | certified | 655+393 | 655+393 | 69.2 | 26.6 | 69.5 |
| e2l10_transport | B | iterate1 | refused | violation | violation | 433+382 | 433+397 | 40.6 | 22.5 | 57.8 |
| e2l10_transport | B | iterate2 | certified | certified | certified | 313+345 | 313+345 | 47.6 | 22.1 | 53.1 |
| e2l10_transport | B | iterate3 | refused | violation | violation | 745+522 | 745+537 | 51.5 | 20.8 | 61.3 |
| e2l10_transport | B | straight | refused | violation | violation | 75+78 | 76+81 | 15.5 | 16.5 | 43.1 |
| e2l11_approach | A | iterate0 | certified | certified | certified | 98+300 | 98+300 | 23.1 | 13.1 | 28.0 |
| e2l11_approach | A | iterate1 | certified | certified | certified | 68+338 | 68+338 | 23.2 | 10.6 | 26.6 |
| e2l11_approach | A | straight | refused | violation | violation | 67+68 | 67+68 | 9.3 | 10.2 | 18.4 |
| e2l11_approach | B | iterate0 | certified | certified | certified | 98+300 | 98+300 | 23.0 | 10.6 | 28.5 |
| e2l11_approach | B | iterate1 | certified | certified | certified | 68+338 | 68+338 | 23.5 | 10.5 | 26.2 |
| e2l11_approach | B | iterate2 | certified | certified | certified | 64+362 | 64+362 | 23.9 | 10.4 | 26.5 |
| e2l11_approach | B | straight | refused | violation | violation | 67+68 | 67+68 | 9.1 | 10.2 | 18.5 |
| e2l12_transport | A | iterate0 | certified | certified | certified | 235+119 | 235+119 | 34.1 | 22.6 | 44.8 |
| e2l12_transport | A | iterate1 | certified | certified | certified | 203+91 | 203+91 | 27.0 | 21.0 | 34.7 |
| e2l12_transport | A | iterate2 | certified | certified | certified | 427+103 | 427+103 | 33.5 | 19.1 | 37.9 |
| e2l12_transport | A | iterate3 | certified | certified | certified | 271+109 | 271+109 | 26.4 | 17.9 | 34.9 |
| e2l12_transport | A | iterate4 | certified | certified | certified | 795+155 | 795+155 | 45.4 | 19.2 | 42.0 |
| e2l12_transport | A | iterate5 | refused | violation | violation | 1016+159 | 1155+159 | 51.3 | 27.1 | 47.3 |
| e2l12_transport | A | straight | refused | violation | violation | 159+98 | 160+98 | 15.0 | 13.3 | 31.1 |
| e2l12_transport | B | iterate0 | certified | certified | certified | 235+119 | 235+119 | 34.2 | 22.2 | 44.1 |
| e2l12_transport | B | iterate1 | certified | certified | certified | 205+93 | 205+93 | 27.0 | 21.2 | 35.0 |
| e2l12_transport | B | iterate2 | certified | certified | certified | 323+107 | 323+107 | 29.0 | 18.6 | 37.3 |
| e2l12_transport | B | iterate3 | certified | certified | certified | 371+103 | 371+103 | 29.0 | 18.3 | 35.2 |
| e2l12_transport | B | iterate4 | certified | certified | certified | 637+105 | 637+105 | 38.7 | 20.9 | 40.5 |
| e2l12_transport | B | iterate5 | certified | certified | certified | 619+105 | 619+105 | 38.1 | 19.2 | 40.3 |
| e2l12_transport | B | straight | refused | violation | violation | 159+98 | 160+98 | 15.2 | 13.5 | 31.0 |
| e2l13_home | A | iterate0 | certified | certified | certified | 66+506 | 66+506 | 31.9 | 22.1 | 34.4 |
| e2l13_home | A | iterate1 | certified | certified | certified | 64+244 | 64+244 | 22.4 | 17.0 | 28.2 |
| e2l13_home | A | iterate2 | refused | violation | violation | 68+187 | 68+187 | 15.5 | 14.1 | 30.4 |
| e2l13_home | A | straight | refused | violation | violation | 59+117 | 59+117 | 10.8 | 11.2 | 20.9 |
| e2l13_home | B | iterate0 | certified | certified | certified | 66+506 | 66+506 | 31.9 | 15.4 | 34.5 |
| e2l13_home | B | iterate1 | certified | certified | certified | 64+428 | 64+428 | 25.9 | 16.9 | 31.4 |
| e2l13_home | B | iterate2 | refused | violation | violation | 68+1291 | 68+1295 | 60.4 | 19.1 | 46.1 |
| e2l13_home | B | iterate3 | certified | certified | certified | 64+266 | 64+266 | 25.7 | 16.7 | 30.2 |
| e2l13_home | B | straight | refused | violation | violation | 59+117 | 59+117 | 10.8 | 11.0 | 21.1 |
