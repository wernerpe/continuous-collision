"""The displacement lemma and the J(p) logic (port of motion_bound_test.cc). An under-bounding
lambda makes the certifier certify a colliding path with no other symptom, so a failure here
is a soundness bug in the kinematics module, not a test to loosen."""

import math

import numpy as np
import pytest
from pydrake.geometry import Box, Capsule, HalfSpace, Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.tree import (BallRpyJoint, FixedOffsetFrame, PlanarJoint, PrismaticJoint,
                                    QuaternionFloatingJoint, RevoluteJoint, RpyFloatingJoint,
                                    ScrewJoint, WeldJoint)
from pydrake.planning import RobotDiagramBuilder

from continuous_collision._numerics import CONTINUITY_TOLERANCE
from continuous_collision.motion_bound import KinematicsEngine

import helpers as h

SLACK = 1e-9


# ----------------------------------------------------------------------------- plumbing


def add_joint(plant, cls, name, parent, X_PF, child, X_CM, *args):
    F = plant.AddFrame(FixedOffsetFrame(f"{name}_F", parent.body_frame(), X_PF)) if X_PF is not None else parent.body_frame()
    M = plant.AddFrame(FixedOffsetFrame(f"{name}_M", child.body_frame(), X_CM)) if X_CM is not None else child.body_frame()
    return plant.AddJoint(cls(name, F, M, *args))


def collision_pairs(model):
    plant, inspector = model.plant(), model.scene_graph().model_inspector()
    pairs = []
    for a, b in inspector.GetCollisionCandidates():
        if b.get_value() < a.get_value():
            a, b = b, a
        pairs.append((a, b))
    pairs.sort(key=lambda p: (p[0].get_value(), p[1].get_value()))
    body = [[int(plant.GetBodyFromFrameId(inspector.GetFrameId(g)).index()) for g in p] for p in pairs]
    return pairs, np.array([b[0] for b in body], dtype=int), np.array([b[1] for b in body], dtype=int)


def table_for(engine, model, lower, upper, constant):
    _, ba, bb = collision_pairs(model)
    return engine.compute_motion_bound_table(np.asarray(lower, float), np.asarray(upper, float),
                                             np.asarray(constant, bool), ba, bb)


def displacement(plant, context, points_B, frame_d, frame_o, q, qp):
    plant.SetPositions(context, q)
    before = plant.CalcPointsPositions(context, frame_d, points_B, frame_o)
    plant.SetPositions(context, qp)
    after = plant.CalcPointsPositions(context, frame_d, points_B, frame_o)
    return float(np.linalg.norm(after - before, axis=0).max())


def coords_of(plant, name):
    j = plant.GetJointByName(name)
    return [j.position_start() + c for c in range(j.num_positions())]


# ----------------------------------------------------------------------------- Part 1: J(p)


def test_topology_determines_the_coordinate_set():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    env, b1, left, right, tip, hand, finger = (plant.AddRigidBody(n, h.inertia()) for n in
                                               ("env", "b1", "left", "right", "tip", "hand", "finger"))
    W = plant.world_body()

    def tx(x):
        return RigidTransform([x, 0.0, 0.0])

    plant.AddJoint(WeldJoint("w_env", W.body_frame(), env.body_frame(), tx(1.0)))
    plant.AddJoint(RevoluteJoint("j0", W.body_frame(), b1.body_frame(), [0, 0, 1]))
    plant.AddJoint(RevoluteJoint("jl", b1.body_frame(), left.body_frame(), [0, 1, 0]))
    plant.AddJoint(PlanarJoint("jr", b1.body_frame(), right.body_frame(), np.zeros(3)))
    plant.AddJoint(RevoluteJoint("jt", left.body_frame(), tip.body_frame(), [1, 0, 0]))
    plant.AddJoint(WeldJoint("w1", tip.body_frame(), hand.body_frame(), tx(0.2)))
    plant.AddJoint(WeldJoint("w2", hand.body_frame(), finger.body_frame(), tx(0.05)))
    model = builder.Build()
    engine = KinematicsEngine(model)
    p = model.plant()

    def affecting(a, b):
        return engine.coordinates_affecting_pair(int(p.GetBodyByName(a).index()), int(p.GetBodyByName(b).index()))

    j0, jl, jr, jt = (coords_of(p, n) for n in ("j0", "jl", "jr", "jt"))
    assert len(jr) == 3
    assert affecting("env", "tip") == sorted(j0 + jl + jt)
    assert engine.coordinates_affecting_pair(0, int(p.GetBodyByName("left").index())) == sorted(j0 + jl)
    assert affecting("b1", "tip") == sorted(jl + jt)
    assert affecting("left", "tip") == jt
    assert affecting("tip", "right") == sorted(jl + jr + jt)
    assert affecting("tip", "tip") == []
    assert engine.coordinates_affecting_pair(0, int(p.GetBodyByName("env").index())) == []
    assert affecting("tip", "finger") == [] and affecting("hand", "finger") == []
    assert affecting("env", "finger") == sorted(j0 + jl + jt)


def test_constant_coordinate_carve_out_empties_jp():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    l1, l2 = plant.AddRigidBody("l1", h.inertia()), plant.AddRigidBody("l2", h.inertia())
    plant.AddJoint(RevoluteJoint("j1", plant.world_frame(), l1.body_frame(), [0, 0, 1]))
    plant.AddJoint(RevoluteJoint("j2", l1.body_frame(), l2.body_frame(), [0, 1, 0]))
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), Sphere(0.1), "g_world", h.friction())
    plant.RegisterCollisionGeometry(l2, RigidTransform([0.3, 0, 0]), Sphere(0.05), "g_tip", h.friction())
    model = builder.Build()
    engine = KinematicsEngine(model)
    lo, hi = np.full(2, -0.5), np.full(2, 0.5)
    table = table_for(engine, model, lo, hi, [False, False])
    assert table.num_pairs == 1 and not table.pair_is_static(0) and len(table.entries(0)) == 2
    table = table_for(engine, model, lo, hi, [True, False])
    assert [c for c, _ in table.entries(0)] == [1]
    pinned = np.full(2, 0.25)
    table = table_for(engine, model, pinned, pinned, [True, True])
    assert table.pair_is_static(0) and table.slack[0] == 0.0 and table.motion_bound(0, np.ones(2)) == 0.0


def test_reversed_joint_throws_with_an_actionable_message():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    a, b = plant.AddRigidBody("body_a", h.inertia()), plant.AddRigidBody("body_b", h.inertia())
    plant.AddJoint(WeldJoint("w", plant.world_frame(), b.body_frame(), RigidTransform([0.1, 0, 0])))
    plant.AddJoint(RevoluteJoint("reversed", a.body_frame(), b.body_frame(), [0, 0, 1]))
    try:
        model = builder.Build()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"this Drake refuses the model outright: {e}")
    with pytest.raises(RuntimeError, match="reversed"):
        KinematicsEngine(model)


# ----------------------------------------------------------------------------- Part 1b


def halfspace_model(halfspace_on_link, prismatic):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    link = plant.AddRigidBody("link", h.inertia())
    if prismatic:
        plant.AddJoint(PrismaticJoint("j", plant.world_frame(), link.body_frame(), [0, 0, 1]))
    else:
        plant.AddJoint(RevoluteJoint("j", plant.world_frame(), link.body_frame(), [0, 1, 0]))
    if halfspace_on_link:
        plant.RegisterCollisionGeometry(link, RigidTransform(), HalfSpace(), "hs", h.friction())
        plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([0, 0, 1.0]), Sphere(0.1), "ball", h.friction())
    else:
        plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), HalfSpace(), "hs", h.friction())
        plant.RegisterCollisionGeometry(link, RigidTransform([0.3, 0, 0]), Sphere(0.1), "ball", h.friction())
    return builder.Build()


def test_only_rotation_relative_to_a_halfspace_is_refused():
    for prismatic in (False, True):
        model = halfspace_model(False, prismatic)
        table = table_for(KinematicsEngine(model), model, [-1.0], [1.0], [False])
        (c, lam), = table.entries(0)
        assert 0.0 < lam < math.inf
    model = halfspace_model(True, True)
    table = table_for(KinematicsEngine(model), model, [-1.0], [1.0], [False])
    assert table.entries(0) == [(0, 1.0)]
    with pytest.raises(RuntimeError, match=r"hs[\s\S]*HalfSpace|HalfSpace[\s\S]*hs"):
        KinematicsEngine(halfspace_model(True, False))


def mid_chain_floating_model():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    b1, b2, b3 = (plant.AddRigidBody(n, h.inertia()) for n in ("b1", "b2", "b3"))
    plant.AddJoint(RevoluteJoint("j0", plant.world_frame(), b1.body_frame(), [0, 0, 1]))
    add_joint(plant, QuaternionFloatingJoint, "jf", b1, RigidTransform([0.1, 0, 0]), b2, RigidTransform([0, 0.05, 0]))
    add_joint(plant, RevoluteJoint, "j1", b2, RigidTransform([0, 0, 0.15]), b3, RigidTransform([0.07, 0, 0]), [0, 1, 0])
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), Sphere(0.1), "g_world", h.friction())
    plant.RegisterCollisionGeometry(b3, RigidTransform([0.2, 0, 0]), Sphere(0.05), "g_tip", h.friction())
    return builder.Build()


def test_moving_quaternion_floating_joint_throws():
    model = mid_chain_floating_model()
    nq = model.plant().num_positions()
    with pytest.raises(RuntimeError, match=r"jf[\s\S]*quaternion_floating[\s\S]*constant"):
        table_for(KinematicsEngine(model), model, np.full(nq, -0.5), np.full(nq, 0.5), np.zeros(nq, bool))


def test_constant_floating_base_carve_out_is_sound_mid_chain():
    model = mid_chain_floating_model()
    plant = model.plant()
    engine = KinematicsEngine(model)
    nq = plant.num_positions()
    fs = plant.GetJointByName("jf").position_start()
    p_FM = np.array([0.9, -0.7, 0.4])
    q0 = np.zeros(nq)
    q0[fs] = 1.0
    q0[fs + 4:fs + 7] = p_FM
    constant = np.zeros(nq, bool)
    constant[fs:fs + 7] = True
    lower, upper = q0.copy(), q0.copy()
    movers = [plant.GetJointByName(n).position_start() for n in ("j0", "j1")]
    lower[movers], upper[movers] = -1.0, 1.0
    table = table_for(engine, model, lower, upper, constant)
    assert len(table.entries(0)) == 2
    assert dict(table.entries(0))[movers[0]] > np.linalg.norm(p_FM)
    context = plant.CreateDefaultContext()
    rng = np.random.default_rng(0xF10A7)
    points = h.sample_surface(rng, 128, lambda g: h.sample_sphere(g, 0.05)) + np.array([[0.2], [0], [0]])
    tip = plant.GetBodyByName("b3").body_frame()
    for _ in range(200):
        q, qp = q0.copy(), q0.copy()
        q[movers], qp[movers] = rng.uniform(-1, 1, 2), rng.uniform(-1, 1, 2)
        d = displacement(plant, context, points, tip, plant.world_frame(), q, qp)
        assert d <= table.motion_bound(0, np.abs(qp - q)) + SLACK


# ----------------------------------------------------------------------------- Part 1c


def tight_chain(screw_top, pitch):
    L1, D1, S, D2, E1, E2, E3, L3, RHO = 0.37, 0.29, 0.53, 0.19, 0.11, 0.23, 0.17, 0.31, 0.13

    def tx(x):
        return RigidTransform([x, 0.0, 0.0])

    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    b1, b2, b3 = (plant.AddRigidBody(n, h.inertia()) for n in ("b1", "b2", "b3"))
    W = plant.world_body()
    if screw_top:
        add_joint(plant, ScrewJoint, "j_top", W, tx(0.0), b1, tx(-L1), [0, 0, 1], pitch, 0.0)
    else:
        add_joint(plant, RevoluteJoint, "j_top", W, tx(0.0), b1, tx(-L1), [0, 0, 1])
    add_joint(plant, PrismaticJoint, "j_slide", b1, tx(D1), b2, tx(-D2), [1, 0, 0])
    add_joint(plant, WeldJoint, "j_weld", b2, tx(E1), b3, tx(-E3), tx(E2))
    plant.RegisterCollisionGeometry(W, RigidTransform(), Sphere(0.02), "g_world", h.friction())
    plant.RegisterCollisionGeometry(b3, tx(L3), Sphere(RHO), "g_tip", h.friction())
    expected = (L3 + RHO) + (E3 + E2 + E1) + (D2 + S + D1) + L1
    return builder.Build(), expected, S, np.array([L3 + RHO, 0.0, 0.0])


def probe_tight_chain(model, slide_max, far_point):
    plant = model.plant()
    engine = KinematicsEngine(model)
    nq = plant.num_positions()
    top, slide = (plant.GetJointByName(n).position_start() for n in ("j_top", "j_slide"))
    lower, upper = np.zeros(nq), np.zeros(nq)
    lower[top], upper[top], upper[slide] = -1.0, 1.0, slide_max
    table = table_for(engine, model, lower, upper, np.zeros(nq, bool))
    entries = dict(table.entries(0))
    q = np.zeros(nq)
    q[slide] = slide_max
    qp = q.copy()
    dtheta = 1e-4
    qp[top] = dtheta
    context = plant.CreateDefaultContext()
    d = displacement(plant, context, far_point[:, None], plant.GetBodyByName("b3").body_frame(),
                     plant.world_frame(), q, qp)
    return entries[top], entries[slide], d, dtheta, table.motion_bound(0, np.abs(qp - q))


def test_revolute_chain_is_exact_and_tight():
    model, expected, S, far = tight_chain(False, 0.0)
    lam_top, lam_slide, d, dtheta, whole = probe_tight_chain(model, S, far)
    assert abs(lam_top - expected) < 1e-12 and lam_slide == 1.0
    bound = lam_top * dtheta
    assert d <= bound + SLACK and d / bound > 1.0 - 1e-8
    assert d <= whole + SLACK


def test_screw_lambda_includes_pitch_and_is_necessary():
    pitch = 8.0
    model, expected, S, far = tight_chain(True, pitch)
    lam_top, _, d, dtheta, _ = probe_tight_chain(model, S, far)
    term = pitch / (2 * math.pi)
    assert abs(lam_top - (expected + term)) < 1e-12
    assert d <= lam_top * dtheta + SLACK
    assert d > expected * dtheta * (1.0 + 1e-6)
    assert abs(d - math.hypot(expected, term) * dtheta) < 1e-11


# ----------------------------------------------------------------------------- Part 2: the lemma


def add_random_geometry(rng, plant, body, name, num_samples, points):
    X_BG = h.random_transform(rng, 0.2)
    kind = rng.integers(0, 3)
    if kind == 0:
        r = rng.uniform(0.02, 0.15)
        g = plant.RegisterCollisionGeometry(body, X_BG, Sphere(r), name, h.friction())
        P = h.sample_surface(rng, num_samples, lambda x: h.sample_sphere(x, r))
    elif kind == 1:
        size = h.uniform_vector(rng, 0.02, 0.3)
        g = plant.RegisterCollisionGeometry(body, X_BG, Box(*size), name, h.friction())
        P = h.sample_surface(rng, num_samples, lambda x: h.sample_box(x, size))
    else:
        r, length = rng.uniform(0.02, 0.1), rng.uniform(0.05, 0.4)
        g = plant.RegisterCollisionGeometry(body, X_BG, Capsule(r, length), name, h.friction())
        P = h.sample_surface(rng, num_samples, lambda x: h.sample_capsule(x, r, length))
    points[g] = X_BG.multiply(P)


def random_plant(rng, allow_screw, num_samples):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    bodies, points, screws = [plant.world_body()], {}, 0
    for i in range(rng.integers(3, 8)):
        body = plant.AddRigidBody(f"b{i}", h.inertia())
        parent = bodies[rng.integers(0, len(bodies))]
        X_PF, X_CM = h.random_transform(rng, 0.25), h.random_transform(rng, 0.25)
        kind = rng.integers(0, 5 if allow_screw else 4)
        name = f"j{i}"
        if kind == 0:
            add_joint(plant, RevoluteJoint, name, parent, X_PF, body, X_CM, h.random_unit_vector(rng))
        elif kind == 1:
            add_joint(plant, PrismaticJoint, name, parent, X_PF, body, X_CM, h.random_unit_vector(rng))
        elif kind == 2:
            add_joint(plant, PlanarJoint, name, parent, X_PF, body, X_CM, np.zeros(3))
        elif kind == 3:
            add_joint(plant, WeldJoint, name, parent, X_PF, body, X_CM, h.random_transform(rng, 0.2))
        else:
            add_joint(plant, ScrewJoint, name, parent, X_PF, body, X_CM, h.random_unit_vector(rng), rng.uniform(0.05, 0.6), 0.0)
            screws += 1
        bodies.append(body)
    add_random_geometry(rng, plant, plant.world_body(), "g_world", num_samples, points)
    for i in range(1, len(bodies)):
        for g in range(1 if i == 1 else rng.integers(0, 3)):
            add_random_geometry(rng, plant, bodies[i], f"g{i}_{g}", num_samples, points)
    return builder.Build(), points, screws


class Stats:
    def __init__(self):
        self.plants = self.pairs = self.atomic = self.aggregate = self.one_sided = 0
        self.max_tightness = 0.0
        self.slack_pairs = 0
        self.max_slack = 0.0

    def observe(self, achieved, bound):
        if bound > 1e-12:
            self.max_tightness = max(self.max_tightness, achieved / bound)


def angular_coordinates(plant):
    angular = np.zeros(plant.num_positions(), bool)
    for ji in plant.GetJointIndices():
        j = plant.get_joint(ji)
        if j.type_name() in ("revolute", "screw"):
            angular[j.position_start()] = True
        elif j.type_name() == "planar":
            angular[j.position_start() + 2] = True
    return angular


def check_world(rng, model, points, carve_out, stats):
    plant = model.plant()
    engine = KinematicsEngine(model)
    pairs, body_a, body_b = collision_pairs(model)
    if not pairs:
        return
    nq = plant.num_positions()
    angular = angular_coordinates(plant)
    q0, lower, upper = np.zeros(nq), np.zeros(nq), np.zeros(nq)
    constant = np.zeros(nq, bool)
    for c in range(nq):
        q0[c] = rng.uniform(-math.pi, math.pi) if angular[c] else rng.uniform(-0.5, 0.5)
        constant[c] = carve_out != "none" and rng.uniform() < 0.3
        if constant[c]:
            half = 0.5 * rng.uniform(0.02 * CONTINUITY_TOLERANCE, CONTINUITY_TOLERANCE) if carve_out == "sub" else 0.0
        else:
            half = rng.uniform(0.05, 1.2) if angular[c] else rng.uniform(0.02, 0.4)
        lower[c], upper[c] = q0[c] - half, q0[c] + half
    table = engine.compute_motion_bound_table(lower, upper, constant, body_a, body_b)
    assert table.num_pairs == len(pairs)
    stats.plants += 1
    context = plant.CreateDefaultContext()
    subtree = engine.coordinate_subtree
    for k in range(table.num_pairs):
        expected = [c for c in engine.coordinates_affecting_pair(body_a[k], body_b[k]) if not constant[c]]
        entries = table.entries(k)
        assert [c for c, _ in entries] == expected
        assert all(math.isfinite(l) and l >= 0 for _, l in entries)
        assert table.pair_is_static(k) == (not expected)
        assert math.isfinite(table.slack[k]) and table.slack[k] >= 0
        if carve_out != "sub":
            assert table.slack[k] == 0.0
        if table.slack[k] > 0:
            stats.slack_pairs += 1
            stats.max_slack = max(stats.max_slack, table.slack[k])
    for _ in range(4):
        q, qp = rng.uniform(lower, upper), rng.uniform(lower, upper)
        dq = np.abs(qp - q)
        for k, (ga, gb) in enumerate(pairs):
            a, b = int(body_a[k]), int(body_b[k])
            pts = {a: points[ga], b: points[gb]}
            frame = {a: plant.get_body(_bi(a)).body_frame(), b: plant.get_body(_bi(b)).body_frame()}
            bound = table.motion_bound(k, dq)
            stats.pairs += 1
            distal_sides = set()
            for c, lam in table.entries(k):
                S = subtree[c]
                assert S[a] != S[b]
                distal, other = (a, b) if S[a] else (b, a)
                distal_sides.add(distal)
                q_step = q.copy()
                q_step[c] = qp[c]
                d = displacement(plant, context, pts[distal], frame[distal], frame[other], q, q_step)
                assert d <= lam * dq[c] + SLACK, f"atomic step: pair {k}, coordinate {c}"
                stats.observe(d, lam * dq[c])
                stats.atomic += 1
            # Aggregate: every A-point-to-B-point distance changes by at most the bound.
            A, B = points[ga][:, :28], points[gb][:, :28]
            fa, fb = plant.get_body(_bi(a)).body_frame(), plant.get_body(_bi(b)).body_frame()
            plant.SetPositions(context, q)
            Bq = plant.CalcPointsPositions(context, fb, B, fa)
            plant.SetPositions(context, qp)
            Bqp = plant.CalcPointsPositions(context, fb, B, fa)
            dq_ = np.linalg.norm(A[:, :, None] - Bq[:, None, :], axis=0)
            dqp = np.linalg.norm(A[:, :, None] - Bqp[:, None, :], axis=0)
            assert np.abs(dqp - dq_).max() <= bound + SLACK, f"aggregate: pair {k}"
            stats.aggregate += 1
            # The one-sided form needs every coordinate that moves the pair on one side, carved
            # ones included: a carved coordinate with nonzero width on the other side moves the
            # measuring frame, and its residual was charged with the other side's reach. (The
            # fork's test considers the table's entries only.)
            for c in engine.coordinates_affecting_pair(a, b):
                if upper[c] > lower[c]:
                    distal_sides.add(a if subtree[c][a] else b)
            if len(distal_sides) == 1:
                distal = distal_sides.pop()
                other = b if distal == a else a
                d = displacement(plant, context, pts[distal], frame[distal], frame[other], q, qp)
                assert d <= bound + SLACK, f"one-sided aggregate: pair {k}"
                stats.observe(d, bound)
                stats.one_sided += 1


def _bi(b):
    from pydrake.multibody.tree import BodyIndex

    return BodyIndex(int(b))


def test_displacement_lemma_random_plants():
    rng = np.random.default_rng(0xD15B0)
    stats, screws = Stats(), 0
    num = 1500
    for trial in range(num):
        model, points, s = random_plant(rng, trial % 3 == 0, 128)
        screws += s
        check_world(rng, model, points, ("none", "exact", "sub")[trial % 3], stats)
    scale = num / 1500
    assert stats.plants >= 1000 * scale
    assert stats.pairs >= 20000 * scale and stats.atomic >= 20000 * scale
    assert stats.aggregate >= 10000 * scale and stats.one_sided >= 2000 * scale
    assert screws > 0
    assert 0.9 < stats.max_tightness <= 1.0 + 1e-9
    assert stats.slack_pairs >= 200 * scale and stats.max_slack > 0.0


def test_displacement_lemma_screw_chain():
    rng = np.random.default_rng(0x5C2E7)
    stats = Stats()
    for _ in range(80):
        builder = RobotDiagramBuilder(time_step=0.0)
        plant = builder.plant()
        bodies, points = [plant.world_body()], {}
        for i in range(3):
            body = plant.AddRigidBody(f"b{i}", h.inertia())
            add_joint(plant, ScrewJoint, f"j{i}", bodies[-1], h.random_transform(rng, 0.25), body,
                      h.random_transform(rng, 0.25), h.random_unit_vector(rng), rng.uniform(-0.8, 0.8), 0.0)
            bodies.append(body)
        add_random_geometry(rng, plant, plant.world_body(), "g_world", 128, points)
        for i in range(1, len(bodies)):
            add_random_geometry(rng, plant, bodies[i], f"g{i}", 128, points)
        check_world(rng, builder.Build(), points, "none", stats)
    assert stats.plants >= 75 and stats.atomic > 500 and stats.max_tightness > 0.5


# ----------------------------------------------------------------------------- Part 3: carve-out


def test_tolerance_constant_coordinate_is_charged_at_lambda():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    l1, l2 = plant.AddRigidBody("l1", h.inertia()), plant.AddRigidBody("l2", h.inertia())
    add_joint(plant, RevoluteJoint, "j_rot", plant.world_body(), None, l1, RigidTransform([-0.2, 0, 0]), [0, 0, 1])
    add_joint(plant, PrismaticJoint, "j_slide", l1, RigidTransform([0.15, 0, 0]), l2, None, [1, 0, 0])
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), Sphere(0.1), "g_world", h.friction())
    plant.RegisterCollisionGeometry(l2, RigidTransform([0.3, 0, 0]), Sphere(0.05), "g_tip", h.friction())
    model = builder.Build()
    engine = KinematicsEngine(model)
    rot, slide = (model.plant().GetJointByName(n).position_start() for n in ("j_rot", "j_slide"))
    lower, upper = np.zeros(2), np.zeros(2)
    lower[slide], upper[slide] = 0.1, 0.4
    lower[rot], upper[rot] = -0.5, 0.5
    moving = table_for(engine, model, lower, upper, [False, False])
    assert len(moving.entries(0)) == 2 and moving.slack[0] == 0.0
    lam_rot = dict(moving.entries(0))[rot]
    assert lam_rot > 0
    rng_ = 5e-8
    constant = np.zeros(2, bool)
    constant[rot] = True
    lower[rot], upper[rot] = 0.0, rng_
    carved = table_for(engine, model, lower, upper, constant)
    assert [c for c, _ in carved.entries(0)] == [slide]
    assert abs(carved.slack[0] - lam_rot * rng_) <= 1e-15 * lam_rot * rng_
    w = np.zeros(2)
    w[slide] = 0.02
    assert carved.motion_bound(0, w) == pytest.approx(carved.slack[0] + 0.02, rel=1e-15)
    lower[rot] = upper[rot] = 0.0
    exact = table_for(engine, model, lower, upper, constant)
    assert exact.slack[0] == 0.0 and exact.motion_bound(0, w) == 0.02
    wide = table_for(engine, model, np.full(2, -2.0), np.full(2, 2.0), [False, False])
    assert wide.slack[0] == 0.0


def carved_halfspace_model(rpy):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    link = plant.AddRigidBody("link", h.inertia())
    if rpy:
        plant.AddJoint(RpyFloatingJoint("base", plant.world_frame(), link.body_frame()))
    else:
        plant.AddJoint(BallRpyJoint("base", plant.world_frame(), link.body_frame(), 0.0))
    plant.RegisterCollisionGeometry(link, RigidTransform(), HalfSpace(), "hs", h.friction())
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([0, 0, 1.0]), Sphere(0.1), "ball", h.friction())
    return builder.Build()


def test_halfspace_needs_exactly_constant_rotation():
    model = carved_halfspace_model(False)
    engine = KinematicsEngine(model)  # A ball joint is not a supported rotational kind.
    assert model.plant().num_positions() == 3
    with pytest.raises(RuntimeError, match=r"hs[\s\S]*base[\s\S]*EXACTLY constant"):
        table_for(engine, model, np.zeros(3), np.full(3, 5e-8), np.ones(3, bool))
    table = table_for(engine, model, np.full(3, 0.3), np.full(3, 0.3), np.ones(3, bool))
    assert table.pair_is_static(0) and table.slack[0] == 0.0


def test_halfspace_across_tolerance_constant_translation_is_accepted():
    model = carved_halfspace_model(True)
    engine = KinematicsEngine(model)
    assert model.plant().num_positions() == 6
    upper = np.zeros(6)
    upper[3:] = 4e-8
    table = table_for(engine, model, np.zeros(6), upper, np.ones(6, bool))
    assert table.pair_is_static(0) and table.slack[0] == pytest.approx(3 * 4e-8, rel=1e-15)


def floating_base_chain(rng, quaternion):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    b1, b2 = plant.AddRigidBody("b1", h.inertia()), plant.AddRigidBody("b2", h.inertia())
    cls = QuaternionFloatingJoint if quaternion else RpyFloatingJoint
    add_joint(plant, cls, "base", plant.world_body(), h.random_transform(rng, 0.2), b1, h.random_transform(rng, 0.2))
    add_joint(plant, RevoluteJoint, "jr", b1, h.random_transform(rng, 0.2), b2, h.random_transform(rng, 0.2),
              h.random_unit_vector(rng))
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), Sphere(0.1), "g_world", h.friction())
    plant.RegisterCollisionGeometry(b2, RigidTransform([0.25, 0, 0]), Sphere(0.06), "g_tip", h.friction())
    return builder.Build()


@pytest.mark.parametrize("quaternion,seed", [(False, 0x12F0BA5E), (True, 0x9A7E48A5E)])
def test_tolerance_constant_floating_base(quaternion, seed):
    rng = np.random.default_rng(seed)
    atomic_cases = aggregate_cases = 0
    max_ratio = 0.0
    for _ in range(250):
        model = floating_base_chain(rng, quaternion)
        plant = model.plant()
        engine = KinematicsEngine(model)
        nq = plant.num_positions()
        base = plant.GetJointByName("base")
        bs, nb = base.position_start(), base.num_positions()
        jr = plant.GetJointByName("jr").position_start()
        q0 = np.zeros(nq)
        if quaternion:
            q0[bs:bs + 4] = h.random_rotation(rng).ToQuaternion().wxyz()
            q0[bs + 4:bs + 7] = rng.uniform(-0.5, 0.5, 3)
        else:
            q0[bs:bs + 3] = rng.uniform(-math.pi, math.pi, 3)
            q0[bs + 3:bs + 6] = rng.uniform(-0.5, 0.5, 3)
        q0[jr] = rng.uniform(-1, 1)
        context = plant.CreateDefaultContext()
        points = h.sample_surface(rng, 64, lambda g: h.sample_sphere(g, 0.06)) + np.array([[0.25], [0], [0]])
        tip = plant.GetBodyByName("b2").body_frame()
        constant = np.zeros(nq, bool)
        constant[bs:bs + nb] = True

        c = bs + rng.integers(0, nb)
        width = rng.uniform(0.2 * CONTINUITY_TOLERANCE, CONTINUITY_TOLERANCE)
        lower, upper = q0.copy(), q0.copy()
        lower[c], upper[c] = q0[c] - 0.5 * width, q0[c] + 0.5 * width
        lower[jr], upper[jr] = q0[jr] - 0.8, q0[jr] + 0.8
        table = table_for(engine, model, lower, upper, constant)
        assert len(table.entries(0)) == 1
        slack = table.slack[0]
        assert 0.0 < slack < 1e-4
        atomic_cases += 1
        q = q0.copy()
        q[jr] = rng.uniform(lower[jr], upper[jr])
        qp = q.copy()
        q[c], qp[c] = lower[c], upper[c]
        d = displacement(plant, context, points, tip, plant.world_frame(), q, qp)
        assert d <= slack + SLACK
        max_ratio = max(max_ratio, d / slack)

        lower, upper = q0.copy(), q0.copy()
        for cc in range(bs, bs + nb):
            half = 0.5 * rng.uniform(0.2 * CONTINUITY_TOLERANCE, CONTINUITY_TOLERANCE)
            lower[cc], upper[cc] = q0[cc] - half, q0[cc] + half
        lower[jr], upper[jr] = q0[jr] - 0.8, q0[jr] + 0.8
        table = table_for(engine, model, lower, upper, constant)
        assert table.slack[0] > 0
        aggregate_cases += 1
        q, qp = rng.uniform(lower, upper), rng.uniform(lower, upper)
        assert displacement(plant, context, points, tip, plant.world_frame(), q, qp) <= \
            table.motion_bound(0, np.abs(qp - q)) + SLACK
        qp[jr] = q[jr]
        assert displacement(plant, context, points, tip, plant.world_frame(), q, qp) <= table.slack[0] + SLACK
    assert atomic_cases >= 200 and aggregate_cases >= 200
    assert 0.9 < max_ratio <= 1.0 + 1e-6


@pytest.mark.parametrize("quaternion", [False, True])
def test_floating_lambda_tilde_is_exact_and_tight(quaternion):
    radius, width = 0.4, 8e-8
    rng = np.random.default_rng(0x7168A7 if quaternion else 0x51DE12)
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    link = plant.AddRigidBody("link", h.inertia())
    cls = QuaternionFloatingJoint if quaternion else RpyFloatingJoint
    plant.AddJoint(cls("base", plant.world_frame(), link.body_frame()))
    plant.RegisterCollisionGeometry(link, RigidTransform(), Sphere(radius), "g_link", h.friction())
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([0, 0, 3.0]), Sphere(0.05), "g_world", h.friction())
    model = builder.Build()
    plant = model.plant()
    engine = KinematicsEngine(model)
    nq = plant.num_positions()
    assert nq == (7 if quaternion else 6)
    points = h.sample_surface(rng, 4096, lambda g: h.sample_sphere(g, radius))
    context = plant.CreateDefaultContext()
    frame = plant.GetBodyByName("link").body_frame()
    for off in range(nq):
        q0 = np.zeros(nq)
        if quaternion:
            qb = np.array([0.31, 0.53, -0.62, 0.49])
            if off < 4:
                qb[off] = 0.0
            q0[:4] = qb / np.linalg.norm(qb)
            q0[4:7] = rng.uniform(-0.5, 0.5, 3)
        else:
            q0[3:6] = rng.uniform(-0.5, 0.5, 3)
        lower, upper = q0.copy(), q0.copy()
        lower[off], upper[off] = q0[off] - 0.5 * width, q0[off] + 0.5 * width
        table = table_for(engine, model, lower, upper, np.ones(nq, bool))
        assert table.pair_is_static(0)
        slack = table.slack[0]
        d = displacement(plant, context, points, frame, plant.world_frame(), lower, upper)
        assert d <= slack + SLACK
        assert d / slack > 0.999
