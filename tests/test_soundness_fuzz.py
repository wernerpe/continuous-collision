"""End-to-end soundness fuzz (port of soundness_fuzz_test.cc): random worlds x random
trajectories, cross-checked against Drake alone. A sampled configuration whose clearance
reaches the threshold refutes kCertifiedFree, so every certified case is searched for one
(1e4 configurations, 1e5 on every tenth); a kViolationFound witness is re-measured at its
configuration and must really violate; a kInconclusive witness must be backed by a clearance
within the contract band near the reported time. Every failure message is a complete repro.

The full corpus (200 cases, as in the fork) runs by default; CCD_FUZZ_CASES overrides it."""

import math
import os
from dataclasses import dataclass, field

import numpy as np
import pytest
from pydrake.common import Parallelism
from pydrake.geometry import Box, Capsule, Convex, Cylinder, Ellipsoid, HalfSpace, Sphere
from pydrake.math import BsplineBasis, RigidTransform, RollPitchYaw
from pydrake.multibody.tree import FixedOffsetFrame, PrismaticJoint, RevoluteJoint
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve, BsplineTrajectory, PiecewisePolynomial

from continuous_collision import ContinuousCollisionChecker, Options, Verdict
from continuous_collision._numerics import NUMERICAL_SLACK
from continuous_collision.bezier_path import PiecewiseBezierPath

import helpers as h

NUM_CASES = int(os.environ.get("CCD_FUZZ_CASES", "200"))
MIN_CERTIFIED, MIN_VIOLATION, MIN_INCONCLUSIVE = NUM_CASES // 5, NUM_CASES // 10, NUM_CASES // 40
MIN_PER_FAMILY = NUM_CASES // 10
BASE_SEED = 0x5EED000000000000
DENSE, DEEP_DENSE, DEEP_EVERY, GRAZE_PROBE = 10000, 100000, 10, 2000
WORST_TAU = 5e-5
SCAN_GUARD = 2e-3
SHAPES = ["Sphere", "Box", "Capsule", "Cylinder", "Ellipsoid", "ConvexTetra"]


def tetrahedron(scale):
    return scale * np.array([[1, 1, -1, -1], [1, -1, 1, -1], [1, -1, -1, 1]], dtype=float)


def make_shape(kind, dims):
    return {"Sphere": lambda: Sphere(dims[0]), "Box": lambda: Box(*dims),
            "Capsule": lambda: Capsule(dims[0], dims[1]), "Cylinder": lambda: Cylinder(dims[0], dims[1]),
            "Ellipsoid": lambda: Ellipsoid(*dims),
            "ConvexTetra": lambda: Convex(tetrahedron(dims[0]), "fuzz_tetra")}[kind]()


@dataclass
class World:
    seed: int
    links: list = field(default_factory=list)  # (parent, revolute?, axis, rpy_PF, p_PF, p_LG, kind, dims)
    obstacles: list = field(default_factory=list)  # (p_W, rpy_W, kind, dims)
    floor: bool = False
    floor_z: float = -0.45

    def describe(self):
        out = [f"world seed={self.seed} links={len(self.links)} obstacles={len(self.obstacles)} floor={self.floor}"]
        for i, l in enumerate(self.links):
            out.append(f"  link{i}: parent={l[0]} {'revolute' if l[1] else 'prismatic'} axis={list(l[2])} "
                       f"rpy_PF={list(l[3])} p_PF={list(l[4])} p_LG={list(l[5])} {l[6]}{list(l[7])}")
        for i, o in enumerate(self.obstacles):
            out.append(f"  obstacle{i}: p_W={list(o[0])} rpy_W={list(o[1])} {o[2]}{list(o[3])}")
        if self.floor:
            out.append(f"  floor: HalfSpace at z = {self.floor_z}")
        return "\n".join(out)


def direction(rng):
    while True:
        v = rng.uniform(-1, 1, 3)
        n = np.linalg.norm(v)
        if 1e-3 < n <= 1.0:
            return v / n


def link_shape(rng):
    roll = rng.integers(0, 12)
    if roll <= 2:
        return "Sphere", [rng.uniform(0.02, 0.05)]
    if roll <= 5:
        return "Box", list(rng.uniform(0.04, 0.10, 3))
    if roll <= 7:
        return "Capsule", [rng.uniform(0.02, 0.04), rng.uniform(0.04, 0.12)]
    if roll <= 9:
        return "Cylinder", [rng.uniform(0.02, 0.04), rng.uniform(0.04, 0.12)]
    if roll == 10:
        return "Ellipsoid", list(rng.uniform(0.02, 0.06, 3))
    return "ConvexTetra", [rng.uniform(0.02, 0.04)]


def obstacle_shape(rng):
    roll = rng.integers(0, 10)
    if roll <= 3:
        return "Box", list(rng.uniform(0.06, 0.22, 3))
    if roll <= 6:
        return "Sphere", [rng.uniform(0.04, 0.11)]
    if roll <= 8:
        return "Capsule", [rng.uniform(0.03, 0.08), rng.uniform(0.06, 0.20)]
    return "ConvexTetra", [rng.uniform(0.05, 0.10)]


def random_world(seed):
    rng = np.random.default_rng(seed)
    world = World(seed)
    for i in range(rng.integers(2, 6)):
        parent = -1 if i == 0 else (i - 1 if rng.uniform() < 0.72 else int(rng.integers(0, i)))
        revolute = rng.uniform() < 0.7
        axis, rpy = direction(rng), rng.uniform(-0.6, 0.6, 3)
        p_PF = direction(rng) * rng.uniform(0.25, 0.35)
        p_LG = direction(rng) * rng.uniform(0.12, 0.18)
        world.links.append((parent, revolute, axis, rpy, p_PF, p_LG, *link_shape(rng)))
    for _ in range(rng.integers(2, 7)):
        p_W = direction(rng) * rng.uniform(0.25, 0.80)
        world.obstacles.append((p_W, rng.uniform(-3.0, 3.0, 3), *obstacle_shape(rng)))
    world.floor = bool(rng.uniform() < 0.3)
    world.floor_z = float(rng.uniform(-0.6, -0.35))
    return world


def build_world(world):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    robot, env = plant.AddModelInstance("robot"), plant.AddModelInstance("env")
    bodies = []
    for i, (parent, revolute, axis, rpy, p_PF, p_LG, kind, dims) in enumerate(world.links):
        body = plant.AddRigidBody(f"link{i}", robot, h.inertia())
        P = plant.world_body() if parent < 0 else bodies[parent]
        F = plant.AddFrame(FixedOffsetFrame(f"F{i}", P.body_frame(), RigidTransform(RollPitchYaw(rpy), p_PF)))
        cls = RevoluteJoint if revolute else PrismaticJoint
        plant.AddJoint(cls(f"j{i}", F, body.body_frame(), axis))
        plant.RegisterCollisionGeometry(body, RigidTransform(p_LG), make_shape(kind, dims), f"link{i}_geom", h.friction())
        bodies.append(body)
    for i, (p_W, rpy, kind, dims) in enumerate(world.obstacles):
        body = plant.AddRigidBody(f"obstacle{i}", env, h.inertia())
        plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform(RollPitchYaw(rpy), p_W))
        plant.RegisterCollisionGeometry(body, RigidTransform(), make_shape(kind, dims), f"obstacle{i}_geom", h.friction())
    if world.floor:
        body = plant.AddRigidBody("floor", env, h.inertia())
        plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform([0, 0, world.floor_z]))
        plant.RegisterCollisionGeometry(body, RigidTransform(), HalfSpace(), "floor_geom", h.friction())
    return builder.Build()


def random_trajectory(world, rng):
    n = len(world.links)
    excursion = rng.uniform(0.15, 1.6)
    centre, amplitude = np.zeros(n), np.zeros(n)
    for i, link in enumerate(world.links):
        prismatic = not link[1]
        centre[i] = rng.uniform(-0.10, 0.10) if prismatic else rng.uniform(-2.0, 2.0)
        amplitude[i] = excursion * (0.15 if prismatic else 1.2)
    kind = ["PWL", "Bezier", "Bspline"][rng.integers(0, 3)]
    if kind == "PWL":
        order, columns = 1, int(rng.integers(2, 6))
    elif kind == "Bezier":
        order = int(rng.integers(1, 6))
        columns = order + 1
    else:
        order, columns = 4, int(rng.integers(4, 8))
    points = centre[:, None] + amplitude[:, None] * rng.uniform(-1.0, 1.0, (n, columns))
    return kind, order, points


def build_trajectory(kind, order, points):
    if kind == "PWL":
        return PiecewisePolynomial.FirstOrderHold(np.arange(points.shape[1], dtype=float), points)
    if kind == "Bezier":
        return BezierCurve(0.0, 1.0, points)
    return BsplineTrajectory(BsplineBasis(order, points.shape[1]), [points[:, [j]] for j in range(points.shape[1])])


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
        seed = BASE_SEED + case
        world = random_world(seed)
        for link in world.links:
            shapes[link[6]] += 1
        for obstacle in world.obstacles:
            shapes[obstacle[2]] += 1
        tally["floors"] += world.floor
        rng = np.random.default_rng(seed ^ 0x9E3779B97F4A7C15)
        kind, order, points = random_trajectory(world, rng)
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
