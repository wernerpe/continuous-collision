"""The soundness fuzz's random worlds and trajectories (pydrake and numpy only), shared by
tests/test_soundness_fuzz.py and benchmarks/fuzz_referee.py, which replays them on the fork."""

import math
from dataclasses import dataclass, field

import numpy as np
from pydrake.geometry import Box, Capsule, Convex, Cylinder, Ellipsoid, HalfSpace, Sphere
from pydrake.math import BsplineBasis, RigidTransform, RollPitchYaw
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.tree import FixedOffsetFrame, PrismaticJoint, RevoluteJoint, SpatialInertia
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve, BsplineTrajectory, PiecewisePolynomial

BASE_SEED = 0x5EED000000000000


def _friction():
    return CoulombFriction(1.0, 1.0)


def _inertia():
    return SpatialInertia.SolidSphereWithMass(1.0, 0.05)


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
        body = plant.AddRigidBody(f"link{i}", robot, _inertia())
        P = plant.world_body() if parent < 0 else bodies[parent]
        F = plant.AddFrame(FixedOffsetFrame(f"F{i}", P.body_frame(), RigidTransform(RollPitchYaw(rpy), p_PF)))
        cls = RevoluteJoint if revolute else PrismaticJoint
        plant.AddJoint(cls(f"j{i}", F, body.body_frame(), axis))
        plant.RegisterCollisionGeometry(body, RigidTransform(p_LG), make_shape(kind, dims), f"link{i}_geom", _friction())
        bodies.append(body)
    for i, (p_W, rpy, kind, dims) in enumerate(world.obstacles):
        body = plant.AddRigidBody(f"obstacle{i}", env, _inertia())
        plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform(RollPitchYaw(rpy), p_W))
        plant.RegisterCollisionGeometry(body, RigidTransform(), make_shape(kind, dims), f"obstacle{i}_geom", _friction())
    if world.floor:
        body = plant.AddRigidBody("floor", env, _inertia())
        plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform([0, 0, world.floor_z]))
        plant.RegisterCollisionGeometry(body, RigidTransform(), HalfSpace(), "floor_geom", _friction())
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


def case(case_index: int):
    """(world, (kind, order, points)) of one fuzz case."""
    seed = BASE_SEED + case_index
    world = random_world(seed)
    rng = np.random.default_rng(seed ^ 0x9E3779B97F4A7C15)
    return world, random_trajectory(world, rng)
