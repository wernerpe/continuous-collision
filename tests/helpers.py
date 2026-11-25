"""Helpers shared by the tests (the port of the fork's test/test_utilities.h).

Seeded random primitives and surface samplers, model builders, the random world generator,
and a reference distance oracle built on Drake's QueryObject alone, so that every cross-check
is independent of the package's own forward kinematics and closed-form routes."""

from __future__ import annotations

import math
import os

import numpy as np
from pydrake.common import Parallelism
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import (Box, Capsule, CollisionFilterDeclaration, Convex, Cylinder, Ellipsoid,
                              GeometrySet, HalfSpace, Mesh, Sphere)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.tree import PrismaticJoint, RevoluteJoint, SpatialInertia
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve

from continuous_collision import ContinuousCollisionChecker, Options, Verdict

#: CCD_FULL=1 runs the property corpora at the fork's sizes instead of the quick defaults.
FULL = os.environ.get("CCD_FULL", "") not in ("", "0")


def friction():
    return CoulombFriction(1.0, 1.0)


def inertia():
    return SpatialInertia.SolidSphereWithMass(1.0, 0.05)


def serial(**kwargs) -> Options:
    return Options(parallelism=Parallelism(1), **kwargs)


# ----------------------------------------------------------------------------- random primitives


def uniform_vector(rng, lo, hi):
    return rng.uniform(lo, hi, 3)


def random_unit_vector(rng):
    while True:
        v = rng.normal(size=3)
        n = np.linalg.norm(v)
        if n > 1e-6:
            return v / n


def random_rotation(rng) -> RotationMatrix:
    q = rng.normal(size=4)
    return RotationMatrix(Quaternion(q / np.linalg.norm(q)))


def random_transform(rng, scale) -> RigidTransform:
    return RigidTransform(random_rotation(rng), uniform_vector(rng, -scale, scale))


# ----------------------------------------------------------------------------- surface samplers


def sample_sphere(rng, radius):
    return radius * random_unit_vector(rng)


def sample_box(rng, size):
    half = 0.5 * np.asarray(size, float)
    p = rng.uniform(-1.0, 1.0, 3) * half
    axis = rng.integers(0, 3)
    p[axis] = (-1.0 if rng.integers(0, 2) == 0 else 1.0) * half[axis]
    return p


def sample_capsule(rng, radius, length):
    half = 0.5 * length
    if rng.integers(0, 2) == 0:
        phi = rng.uniform(0.0, 2 * math.pi)
        return np.array([radius * math.cos(phi), radius * math.sin(phi), rng.uniform(-half, half)])
    u = random_unit_vector(rng)
    z0 = half if u[2] >= 0.0 else -half
    return np.array([radius * u[0], radius * u[1], z0 + radius * u[2]])


def sample_cylinder(rng, radius, length):
    half = 0.5 * length
    phi = rng.uniform(0.0, 2 * math.pi)
    if rng.integers(0, 2) == 0:
        return np.array([radius * math.cos(phi), radius * math.sin(phi), rng.uniform(-half, half)])
    rho = radius * math.sqrt(rng.uniform(0.0, 1.0))
    z = -half if rng.integers(0, 2) == 0 else half
    return np.array([rho * math.cos(phi), rho * math.sin(phi), z])


def sample_ellipsoid(rng, radii):
    return np.asarray(radii, float) * random_unit_vector(rng)


def sample_surface(rng, count, sampler):
    return np.stack([sampler(rng) for _ in range(count)], axis=1)


# ----------------------------------------------------------------------------- reference oracle


def _halfspace_reference(n, p0, X_WC, shape) -> float:
    """phi = min over the shape of n.(x - p0), from the textbook support functions."""
    return _halfspace_formula(shape)(n, p0, X_WC.rotation().matrix(), X_WC.translation())


def _halfspace_formula(shape):
    """The reference formula for one partner shape, as f(n, p0, R_WC, p_WC)."""
    if isinstance(shape, Sphere):
        r = shape.radius()
        return lambda n, p0, R, c: float(n @ (c - p0)) - r
    if isinstance(shape, Box):
        half = 0.5 * np.array([shape.width(), shape.depth(), shape.height()])
        return lambda n, p0, R, c: float(n @ (c - p0)) - float(half @ np.abs(R.T @ n))
    if isinstance(shape, Capsule):
        hl, r = 0.5 * shape.length(), shape.radius()
        return lambda n, p0, R, c: float(n @ (c - p0)) - hl * abs(n @ R[:, 2]) - r
    if isinstance(shape, Cylinder):
        hl, r = 0.5 * shape.length(), shape.radius()

        def cylinder(n, p0, R, c):
            axial = n @ R[:, 2]
            return float(n @ (c - p0)) - hl * abs(axial) - r * float(np.linalg.norm(n - axial * R[:, 2]))
        return cylinder
    if isinstance(shape, Ellipsoid):
        radii = np.array([shape.a(), shape.b(), shape.c()])
        return lambda n, p0, R, c: float(n @ (c - p0)) - float(np.linalg.norm(radii * (R.T @ n)))
    if isinstance(shape, (Convex, Mesh)):
        hull = shape.GetConvexHull()
        V = np.array([hull.vertex(i) for i in range(hull.num_vertices())]).T
        return lambda n, p0, R, c: float(n @ (c - p0)) + float(((R.T @ n) @ V).min())
    raise AssertionError(f"no reference for {type(shape).__name__}")


class ReferenceOracle:
    """Signed distances of a model's unfiltered pairs from Drake's QueryObject alone (half
    spaces by hand-derived support functions on Drake's poses), in a fresh private context."""

    def __init__(self, model):
        self.model = model
        self.plant, self.scene_graph = model.plant(), model.scene_graph()
        self.inspector = self.scene_graph.model_inspector()
        self.root = model.CreateDefaultContext()
        self.plant_context = self.plant.GetMyMutableContextFromRoot(self.root)
        self.sg_context = self.scene_graph.GetMyContextFromRoot(self.root)
        pairs = []
        for a, b in self.inspector.GetCollisionCandidates():
            if b.get_value() < a.get_value():
                a, b = b, a
            pairs.append((a, b))
        self.pairs = sorted(pairs, key=lambda p: (p[0].get_value(), p[1].get_value()))
        self.halfspace = [isinstance(self.inspector.GetShape(a), HalfSpace)
                          or isinstance(self.inspector.GetShape(b), HalfSpace) for a, b in self.pairs]
        # (pair, half space id, partner id, formula) for the half-space pairs.
        self._halves = []
        for (a, b), half in zip(self.pairs, self.halfspace):
            if half:
                hid, cid = (a, b) if isinstance(self.inspector.GetShape(a), HalfSpace) else (b, a)
                self._halves.append(((a, b), hid, cid, _halfspace_formula(self.inspector.GetShape(cid))))
        halves = [g for (a, b), h in zip(self.pairs, self.halfspace) if h
                  for g in (a, b) if isinstance(self.inspector.GetShape(g), HalfSpace)]
        if halves:
            # The pairwise query refuses half spaces against most shapes; they are measured
            # one by one instead, so keep them out of it in this private context.
            everything = list(self.inspector.GetAllGeometryIds())
            self.scene_graph.collision_filter_manager(self.sg_context).Apply(
                CollisionFilterDeclaration().ExcludeBetween(GeometrySet(halves), GeometrySet(everything)))
        self.queries = 0

    def query_object(self, q):
        self.plant.SetPositions(self.plant_context, np.asarray(q, float))
        return self.scene_graph.get_query_output_port().Eval(self.sg_context)

    def distance(self, q, a, b, qo=None) -> float:
        qo = self.query_object(q) if qo is None else qo
        self.queries += 1
        sa, sb = self.inspector.GetShape(a), self.inspector.GetShape(b)
        if isinstance(sa, HalfSpace) or isinstance(sb, HalfSpace):
            h, c = (a, b) if isinstance(sa, HalfSpace) else (b, a)
            X_WH = qo.GetPoseInWorld(h)
            return _halfspace_reference(X_WH.rotation().matrix()[:, 2], X_WH.translation(),
                                        qo.GetPoseInWorld(c), self.inspector.GetShape(c))
        return qo.ComputeSignedDistancePairClosestPoints(a, b).distance

    def distance_at_finding(self, finding) -> float:
        a, b = finding.geometry_a, finding.geometry_b
        assert (a, b) in self.pairs or (b, a) in self.pairs, "the finding names an unknown pair"
        return self.distance(finding.q, a, b)

    def worst(self, q, threshold, reach=0.02):
        """(min over pairs of phi - threshold, pair) at q. Pairs farther than threshold + reach
        are not reported by the pairwise query and count as +inf, which a threshold test
        tolerates and which keeps the dense scans affordable."""
        qo = self.query_object(q)
        best, arg = math.inf, None
        for r in qo.ComputeSignedDistancePairwiseClosestPoints(max(threshold, 0.0) + reach):
            self.queries += 1
            if r.distance - threshold < best:
                best, arg = r.distance - threshold, (r.id_A, r.id_B)
        for pair, hid, cid, formula in self._halves:
            self.queries += 1
            X_H, X_C = qo.GetPoseInWorld(hid), qo.GetPoseInWorld(cid)
            d = formula(X_H.rotation().matrix()[:, 2], X_H.translation(), X_C.rotation().matrix(),
                        X_C.translation()) - threshold
            if d < best:
                best, arg = d, pair
        return best, arg


def scan(reference: ReferenceOracle, path, samples: int, threshold: float, reach: float = 0.02):
    """Dense sampling of `path` (a PiecewiseBezierPath, evaluated by Drake's BezierCurve):
    (min slack, time, pair)."""
    per_segment = max(2, samples // len(path.segments))
    best = (math.inf, math.nan, None)
    s = np.linspace(0.0, 1.0, per_segment + 1)
    for seg in path.segments:
        Q = BezierCurve(0.0, 1.0, seg.control_points).vector_values(s)
        for i in range(per_segment + 1):
            slack, pair = reference.worst(Q[:, i], threshold, reach)
            if slack < best[0]:
                best = (slack, seg.time_of(s[i]), pair)
    return best


# ----------------------------------------------------------------------------- random worlds


def make_random_world(seed: int, num_links=4, num_obstacles=4, floor=True):
    """A chain of revolute joints (every third prismatic) carrying small primitives, anchored
    obstacles, and on odd seeds a HalfSpace floor (the concurrency corpus's world)."""
    rng = np.random.default_rng(seed)

    def offset(lo, hi):
        return random_unit_vector(rng) * rng.uniform(lo, hi)

    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    links = []
    for i in range(num_links):
        body = plant.AddRigidBody(f"link{i}", inertia())
        parent = plant.world_body() if i == 0 else links[-1]
        X_PF = RigidTransform(RollPitchYaw(uniform_vector(rng, -0.5, 0.5)), offset(0.22, 0.32))
        axis = random_unit_vector(rng)
        if i % 3 == 2:
            plant.AddJoint(PrismaticJoint(f"j{i}", _frame(plant, parent, X_PF, f"F{i}"), body.body_frame(), axis))
        else:
            plant.AddJoint(RevoluteJoint(f"j{i}", _frame(plant, parent, X_PF, f"F{i}"), body.body_frame(), axis))
        X_LG = RigidTransform(offset(0.10, 0.16))
        if i % 2 == 0:
            shape = Capsule(rng.uniform(0.02, 0.04), rng.uniform(0.05, 0.10))
        else:
            shape = Box(*uniform_vector(rng, 0.04, 0.09))
        plant.RegisterCollisionGeometry(body, X_LG, shape, f"link{i}_geom", friction())
        links.append(body)
    for i in range(num_obstacles):
        body = plant.AddRigidBody(f"obstacle{i}", inertia())
        plant.WeldFrames(plant.world_frame(), body.body_frame(),
                         RigidTransform(RollPitchYaw(uniform_vector(rng, -3, 3)), offset(0.30, 0.75)))
        if i % 3 == 0:
            shape = Sphere(rng.uniform(0.05, 0.12))
        elif i % 3 == 1:
            shape = Box(*uniform_vector(rng, 0.08, 0.20))
        else:
            shape = Cylinder(rng.uniform(0.04, 0.09), rng.uniform(0.08, 0.18))
        plant.RegisterCollisionGeometry(body, RigidTransform(), shape, f"obstacle{i}_geom", friction())
    if floor and seed % 2 == 1:
        body = plant.AddRigidBody("floor", inertia())
        plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform([0.0, 0.0, -0.5]))
        plant.RegisterCollisionGeometry(body, RigidTransform(), HalfSpace(), "floor_geom", friction())
    return builder.Build()


def _frame(plant, body, X, name):
    from pydrake.multibody.tree import FixedOffsetFrame

    return plant.AddFrame(FixedOffsetFrame(name, body.body_frame(), X))


def random_control_points(seed: int, num_positions: int) -> np.ndarray:
    rng = np.random.default_rng(seed ^ 0xA5A55A5A0F0FF0F0)
    return rng.uniform(-1.4, 1.4, (num_positions, 6))


def bezier(points, t0=0.0, t1=1.0) -> BezierCurve:
    return BezierCurve(t0, t1, np.asarray(points, float))


def checker(model, **options) -> ContinuousCollisionChecker:
    return ContinuousCollisionChecker(model=model, default_options=serial(**options))


__all__ = ["FULL", "Verdict", "friction", "inertia", "serial", "checker"]
