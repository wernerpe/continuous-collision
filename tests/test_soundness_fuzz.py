"""End-to-end soundness fuzz (port of soundness_fuzz_test.cc): random worlds x random
trajectories, cross-checked against Drake alone. A sampled configuration whose clearance
reaches the threshold refutes kCertifiedFree, so every certified case is searched for one
(1e4 configurations, 1e5 on every tenth); a kViolationFound witness is re-measured at its
configuration and must really violate; a kInconclusive witness must be backed by a clearance
within the contract band near the reported time. Every failure message is a complete repro.

The full corpus (200 cases, as in the fork) runs by default; CCD_FUZZ_CASES overrides it."""

import math
import os

import numpy as np
from pydrake.common import Parallelism

from continuous_collision import ContinuousCollisionChecker, Options, Verdict
from continuous_collision._numerics import NUMERICAL_SLACK
from continuous_collision.bezier_path import PiecewiseBezierPath

import helpers as h
from fuzz_worlds import SHAPES, build_trajectory, build_world, case as fuzz_case

NUM_CASES = int(os.environ.get("CCD_FUZZ_CASES", "200"))
MIN_CERTIFIED, MIN_VIOLATION, MIN_INCONCLUSIVE = NUM_CASES // 5, NUM_CASES // 10, NUM_CASES // 40
MIN_PER_FAMILY = NUM_CASES // 10
DENSE, DEEP_DENSE, DEEP_EVERY, GRAZE_PROBE = 10000, 100000, 10, 2000
WORST_TAU = 5e-5
SCAN_GUARD = 2e-3


def contract_band(resolution):
    return resolution + 2.0 * WORST_TAU + NUMERICAL_SLACK


def min_abs_slack_near(reference, path, pair, threshold, time, half_width, samples):
    lo, hi = max(path.start_time, time - half_width), min(path.end_time, time + half_width)
    return min(abs(reference.distance(path.value(lo + i / samples * (hi - lo)), *pair) - threshold)
               for i in range(samples + 1))


def test_random_worlds_and_trajectories():
    tally = dict(certified=0, violation=0, inconclusive=0, deep=0, graze=0, cert_guaranteed=0,
                 viol_guaranteed=0, band=0, PWL=0, Bezier=0, Bspline=0, floors=0, queries=0,
                 tightest=math.inf)
    shapes = dict.fromkeys(SHAPES, 0)
    failures = []
    for case in range(NUM_CASES):
        world, (kind, order, points) = fuzz_case(case)
        for link in world.links:
            shapes[link[6]] += 1
        for obstacle in world.obstacles:
            shapes[obstacle[2]] += 1
        tally["floors"] += world.floor
        tally[kind] += 1
        model = build_world(world)
        trajectory = build_trajectory(kind, order, points)
        path = PiecewiseBezierPath.from_trajectory(trajectory)
        reference = h.ReferenceOracle(model)

        margin = 0.0 if case % 2 == 0 else 0.01
        grazing = case % 5 == 3
        if grazing:
            lowest = h.scan(reference, path, GRAZE_PROBE, 0.0, reach=0.6)[0]
            if 0.01 < lowest < 0.5:
                margin = lowest
                tally["graze"] += 1
            else:
                grazing = False
        repro = (f"REPRO: case {case}, margin {margin}{' (grazing)' if grazing else ''}\n{world.describe()}\n"
                 f"trajectory {kind} order {order} points:\n{np.array2string(points, precision=17)}")
        resolution = 1e-3 if case % 4 == 1 else 1e-6
        options = Options(margin=margin, distance_resolution=resolution, parallelism=Parallelism(1))
        result = ContinuousCollisionChecker(model, options).CheckTrajectory(trajectory, options)

        band = contract_band(resolution)
        clear = h.scan(reference, path, DENSE, margin + band + SCAN_GUARD)
        at_margin = h.scan(reference, path, DENSE, margin)

        def check(condition, message):
            if not condition:
                failures.append(f"{message}\n{repro}")

        if clear[0] > 0.0:
            tally["cert_guaranteed"] += 1
            check(result.verdict == Verdict.kCertifiedFree,
                  f"every sampled clearance exceeds the margin by more than the contract band ({band}) plus "
                  f"the scan guard, so the contract promises kCertifiedFree; got {result.verdict}")
        elif at_margin[0] < -band:
            tally["viol_guaranteed"] += 1
            check(result.verdict == Verdict.kViolationFound,
                  f"a sample at t = {at_margin[1]} sits {-at_margin[0]} m below the margin, more than the band "
                  f"({band}), so the contract promises kViolationFound; got {result.verdict}")
        else:
            tally["band"] += 1

        if result.verdict == Verdict.kCertifiedFree:
            tally["certified"] += 1
            check(result.finding is None, "a certified result carries a finding")
            deep = tally["certified"] % DEEP_EVERY == 0
            tally["deep"] += deep
            sweep = h.scan(reference, path, DEEP_DENSE, margin) if deep else at_margin
            tally["tightest"] = min(tally["tightest"], sweep[0])
            check(sweep[0] > 0.0, f"CERTIFIED FREE but dense sampling ({DEEP_DENSE if deep else DENSE}) found "
                                  f"clearance {sweep[0]} m below the threshold at t = {sweep[1]}, pair {sweep[2]}")
        else:
            tally["violation" if result.verdict == Verdict.kViolationFound else "inconclusive"] += 1
            f = result.finding
            check(f is not None and f.q.shape == (len(world.links),), "missing or malformed finding")
            pair = (f.geometry_a, f.geometry_b)
            check(pair in reference.pairs, "the finding names an unknown pair")
            check(np.abs(path.value(f.time) - f.q).max() < 1e-9, "the witness is not on the trajectory")
            phi = reference.distance(f.q, *pair)
            check(abs(phi - f.distance) < 1e-9, f"reported distance {f.distance} does not reproduce: {phi}")
            check(f.nearest_a_W is not None and f.nearest_b_W is not None, "missing closest points")
            if result.verdict == Verdict.kViolationFound:
                check(phi < margin + WORST_TAU, f"violation at t = {f.time} re-measures at {phi} vs {margin}")
            else:
                check(margin - WORST_TAU <= f.distance <= margin + WORST_TAU + NUMERICAL_SLACK + resolution,
                      f"inconclusive distance {f.distance} outside the contract band")
                tolerance = resolution + 10.0 * (WORST_TAU + NUMERICAL_SLACK)
                window = 0.01 * max(1e-12, path.end_time - path.start_time)
                best = min_abs_slack_near(reference, path, pair, margin, f.time, window, 400)
                check(best <= tolerance, f"INCONCLUSIVE at t = {f.time} but the closest sampled clearance near "
                                         f"it is {best} m from the threshold (> {tolerance})")
        tally["queries"] += reference.queries

    print(f"\n[ fuzz summary ] cases = {NUM_CASES} {tally} shapes = {shapes}")
    assert not failures, failures[0] + f"\n({len(failures)} failures in all)"
    assert tally["certified"] >= MIN_CERTIFIED and tally["violation"] >= MIN_VIOLATION
    assert tally["inconclusive"] >= MIN_INCONCLUSIVE
    assert tally["cert_guaranteed"] >= MIN_CERTIFIED and tally["viol_guaranteed"] >= MIN_VIOLATION
    assert tally["band"] >= 1
    assert min(tally["PWL"], tally["Bezier"], tally["Bspline"]) >= MIN_PER_FAMILY
    assert tally["queries"] > 500 * NUM_CASES
    assert all(v > 0 for v in shapes.values()) and tally["floors"] > 0
