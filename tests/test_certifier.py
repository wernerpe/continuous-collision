"""End-to-end tests of the certifier on a hand-built corpus (port of certifier_test.cc). Every
cross-check densely samples the same path through the reference oracle (Drake only)."""

import math

import numpy as np
import pytest
from pydrake.geometry import Box, HalfSpace, Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.tree import FixedOffsetFrame, PrismaticJoint, RevoluteJoint
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve

from continuous_collision import Verdict
from continuous_collision.bezier_path import PiecewiseBezierPath

import helpers as h

MARGIN = 0.01


def add_arm(plant):
    l1, l2, tool = (plant.AddRigidBody(n, h.inertia()) for n in ("link1", "link2", "tool"))
    plant.AddJoint(RevoluteJoint("j1", plant.world_frame(), l1.body_frame(), [0, 0, 1]))
    F2 = plant.AddFrame(FixedOffsetFrame("F2", l1.body_frame(), RigidTransform([0.40, 0, 0])))
    plant.AddJoint(RevoluteJoint("j2", F2, l2.body_frame(), [0, 0, 1]))
    F3 = plant.AddFrame(FixedOffsetFrame("F3", l2.body_frame(), RigidTransform([0.30, 0, 0])))
    plant.AddJoint(PrismaticJoint("j3", F3, tool.body_frame(), [1, 0, 0]))
    plant.RegisterCollisionGeometry(l1, RigidTransform([0.20, 0, 0]), Box(0.40, 0.06, 0.06), "link1_geom", h.friction())
    plant.RegisterCollisionGeometry(l2, RigidTransform([0.15, 0, 0]), Box(0.30, 0.06, 0.06), "link2_geom", h.friction())
    plant.RegisterCollisionGeometry(tool, RigidTransform(), Sphere(0.05), "tool_geom", h.friction())


def add_welded_sphere(plant, name, p_W, radius):
    body = plant.AddRigidBody(name, h.inertia())
    plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform(p_W))
    plant.RegisterCollisionGeometry(body, RigidTransform(), Sphere(radius), f"{name}_geom", h.friction())


def arm_world():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_arm(plant)
    add_welded_sphere(plant, "post", [0.0, 0.75, 0.0], 0.10)
    add_welded_sphere(plant, "pillar", [-0.55, 0.35, 0.0], 0.08)
    ground = plant.AddRigidBody("ground", h.inertia())
    plant.WeldFrames(plant.world_frame(), ground.body_frame(), RigidTransform([0, 0, -0.50]))
    plant.RegisterCollisionGeometry(ground, RigidTransform(), HalfSpace(), "ground_geom", h.friction())
    ceiling = plant.AddRigidBody("ceiling", h.inertia())
    plant.WeldFrames(plant.world_frame(), ceiling.body_frame(), RigidTransform([0, 0, 0.90]))
    plant.RegisterCollisionGeometry(ceiling, RigidTransform(), Box(2.0, 2.0, 0.20), "ceiling_geom", h.friction())
    return builder.Build()


@pytest.fixture(scope="module")
def world():
    model = arm_world()
    return model, h.checker(model, margin=MARGIN), h.ReferenceOracle(model)


def make_bezier(start, end, order, t0=0.0, t1=1.0):
    u = np.linspace(0.0, 1.0, order + 1)
    return BezierCurve(t0, t1, np.outer(start, 1 - u) + np.outer(end, u))


def Q(t1, t2, d):
    return np.array([t1, t2, d])


def normalize(trajectory):
    return PiecewiseBezierPath.from_trajectory(trajectory)


def sampled(reference, path, samples, threshold):
    """(min clearance over every pair, time of the first sample below threshold)."""
    lowest, first = math.inf, math.nan
    for k, seg in enumerate(path.segments):
        for i in range(samples + 1):
            s = i / samples
            q = path.evaluate_segment(k, s)
            phi = min(reference.distance(q, a, b) for a, b in reference.pairs)
            lowest = min(lowest, phi)
            if phi < threshold and math.isnan(first):
                first = seg.time_of(s)
    return lowest, first


def test_free_trajectory_certified(world):
    model, checker, reference = world
    trajectory = make_bezier(Q(0, 0, 0), Q(0.8, -0.4, 0.10), 3)
    result = checker.CheckTrajectory(trajectory)
    assert result.verdict == Verdict.kCertifiedFree and result.finding is None and result.num_nodes > 0
    lowest, first = sampled(reference, normalize(trajectory), 10000, MARGIN)
    assert lowest > MARGIN and math.isnan(first)
    waypoints = np.column_stack((Q(0, 0, 0), Q(0.4, -0.2, 0.05), Q(0.8, -0.4, 0.10)))
    assert checker.CheckPath(waypoints).verdict == Verdict.kCertifiedFree
    assert checker.CheckEdge(Q(0, 0, 0), Q(0.8, -0.4, 0.10)).verdict == Verdict.kCertifiedFree


def test_narrow_gap_certified_by_subdivision():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_arm(plant)
    add_welded_sphere(plant, "gap_left", [0.80, 0.115, 0.0], 0.05)
    add_welded_sphere(plant, "gap_right", [0.80, -0.115, 0.0], 0.05)
    model = builder.Build()
    trajectory = make_bezier(Q(0, 0, 0), Q(0, 0, 0.20), 1)
    result = h.checker(model, margin=MARGIN).CheckTrajectory(trajectory)
    assert result.verdict == Verdict.kCertifiedFree and result.finding is None
    assert result.num_nodes >= 16
    lowest, _ = sampled(h.ReferenceOracle(model), normalize(trajectory), 10000, MARGIN)
    assert MARGIN < lowest < MARGIN + 0.01


def test_violation_witness_is_exact_and_earliest(world):
    model, checker, reference = world
    trajectory = make_bezier(Q(0, 0, 0), Q(2.0, 0, 0), 1)
    result = checker.CheckTrajectory(trajectory)
    assert result.verdict == Verdict.kViolationFound
    f = result.finding
    assert f.nearest_a_W is not None and f.nearest_b_W is not None
    path = normalize(trajectory)
    assert np.abs(path.value(f.time) - f.q).max() < 1e-9
    phi = reference.distance_at_finding(f)
    assert phi < MARGIN and abs(phi - f.distance) < 1e-12
    _, first = sampled(reference, path, 10000, MARGIN)
    assert not math.isnan(first)
    assert abs(f.time - first) < 5e-3 and f.time <= first + 1e-9


def test_grazing_tangency_is_inconclusive():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_arm(plant)
    add_welded_sphere(plant, "graze", [0.80, 0.11, 0.0], 0.05)
    model = builder.Build()
    trajectory = make_bezier(Q(0, 0, 0), Q(0, 0, 0.20), 1)
    result = h.checker(model, margin=MARGIN).CheckTrajectory(trajectory, h.serial(margin=MARGIN, distance_resolution=1e-4))
    assert result.verdict == Verdict.kInconclusive
    f = result.finding
    assert abs(f.distance - MARGIN) < 1e-3
    reference = h.ReferenceOracle(model)
    assert abs(reference.distance_at_finding(f) - f.distance) < 1e-12
    # The fork fills the closest points for an inconclusive witness too.
    assert f.nearest_a_W is not None and f.nearest_b_W is not None
    lowest, _ = sampled(reference, normalize(trajectory), 10000, MARGIN)
    assert abs(lowest - MARGIN) < 1e-6


def test_breakpoint_witnesses_are_reported(world):
    model, checker, reference = world
    at_start = checker.CheckTrajectory(make_bezier(Q(1.5708, 0, 0), Q(0.5, 0, 0), 1))
    assert at_start.verdict == Verdict.kViolationFound and at_start.finding.time == 0.0
    assert reference.distance_at_finding(at_start.finding) < MARGIN
    waypoints = np.column_stack((Q(0, 0, 0), Q(1.5708, 0, 0), Q(3.0, 0, 0)))
    at_junction = checker.CheckPath(waypoints)
    assert at_junction.verdict == Verdict.kViolationFound and at_junction.finding.time <= 1.0
    assert reference.distance_at_finding(at_junction.finding) < MARGIN


def test_static_pairs_are_resolved_at_the_start(world):
    model, checker, reference = world
    assert checker.CheckTrajectory(make_bezier(Q(0.3, -0.2, 0.0), Q(0.3, -0.2, 0.15), 2)).verdict == Verdict.kCertifiedFree
    result = checker.CheckTrajectory(make_bezier(Q(2.575, 0, 0), Q(2.575, 0, 0.15), 2))
    assert result.verdict == Verdict.kViolationFound and result.finding.time == 0.0
    assert reference.distance_at_finding(result.finding) < MARGIN


def test_retiming_invariance(world):
    _, checker, _ = world
    a = checker.CheckTrajectory(make_bezier(Q(0, 0, 0), Q(0.8, -0.4, 0.10), 3, 0.0, 1.0))
    b = checker.CheckTrajectory(make_bezier(Q(0, 0, 0), Q(0.8, -0.4, 0.10), 3, -2.5, 4.2))
    assert a.verdict == b.verdict == Verdict.kCertifiedFree and a.num_nodes == b.num_nodes


def test_random_trajectories_are_sound_against_dense_sampling(world):
    model, checker, reference = world
    rng = np.random.default_rng(1234)
    certified = violating = 0
    for trial in range(15):
        cps = np.column_stack([Q(rng.uniform(-3, 3), rng.uniform(-2, 2), rng.uniform(0, 0.25)) for _ in range(4)])
        trajectory = BezierCurve(0.0, 1.0, cps)
        result = checker.CheckTrajectory(trajectory)
        path = normalize(trajectory)
        if result.verdict == Verdict.kCertifiedFree:
            certified += 1
            lowest, _ = sampled(reference, path, 2000, MARGIN)
            assert lowest > MARGIN, f"trial {trial} certified but sampling found {lowest}"
        elif result.verdict == Verdict.kViolationFound:
            violating += 1
            assert np.abs(path.value(result.finding.time) - result.finding.q).max() < 1e-9
            assert reference.distance_at_finding(result.finding) < MARGIN
    assert certified > 0 and violating > 0
