"""Distance oracle (port of distance_oracle_test.cc): accuracy against analytic ground truth,
the capability probe, the exact half-space, sphere-sphere and sphere-box routes, and Mesh as its
convex hull. Every route is checked twice: through `signed_distance` (Drake's poses, used for
findings) and through the batched path the certifier runs (numpy forward kinematics)."""

import math

import numpy as np
import pytest
from pydrake.common import MemoryFile
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import (Box, Capsule, Convex, Cylinder, Ellipsoid, GeometryInstance, HalfSpace,
                              InMemoryMesh, Mesh, ProximityProperties, Sphere, AddContactMaterial)
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.fem import DeformableBodyConfig
from pydrake.multibody.tree import SpatialInertia
from pydrake.planning import RobotDiagramBuilder

from continuous_collision.distance_oracle import DistanceOracle, Route
from continuous_collision.kinematics import BatchKinematics
from continuous_collision.motion_bound import KinematicsEngine

import helpers as h

EXACT, NATIVE = 1e-12, 1e-6


def make_oracle(model):
    engine = KinematicsEngine(model)
    kin = BatchKinematics(engine)
    return DistanceOracle(model, engine, kin), kin


class World:
    def __init__(self, model):
        self.model, self.plant = model, model.plant()
        self.context = self.plant.CreateDefaultContext()
        self.oracle, self.kin = make_oracle(model)
        self.lease = self.oracle.acquire()

    def set_pose(self, name, X):
        self.plant.SetFreeBodyPose(self.context, self.plant.GetBodyByName(name), X)

    def q(self):
        return self.plant.GetPositions(self.context)

    def pair(self, a, b):
        ga, gb = self.geometry(a), self.geometry(b)
        for p, (x, y) in enumerate(zip(self.oracle.geometry_a, self.oracle.geometry_b)):
            if {x, y} == {ga, gb}:
                return p
        raise AssertionError("pair not in the probe's snapshot")

    def geometry(self, body):
        ids = self.plant.GetCollisionGeometriesForBody(self.plant.GetBodyByName(body))
        assert len(ids) == 1
        return ids[0]

    def measure(self, p, want_points=False):
        """(signed_distance via Drake poses, the batched route's value) at the current q."""
        phi, pa, pb = self.oracle.signed_distance(self.lease, self.q(), p)
        return (phi, pa, pb) if want_points else phi

    def batched(self, p):
        o = self.oracle
        X = self.kin.anchor_transforms(self.q()[None])
        Xs = np.stack(X, axis=1)
        R, pos = Xs[:, :, :3, :3], Xs[:, :, :3, 3]
        C = o.centers(X)
        nodes, pairs = np.array([0]), np.array([p])
        if o.route[p] in (Route.HALFSPACE_A, Route.HALFSPACE_B):
            return float(o.halfspace_distance(R, pos, nodes, pairs)[0])
        if o.route[p] == Route.SPHERE_BOX:
            return float(o.sphere_box_distance(R, C, nodes, pairs)[0])
        if o.route[p] == Route.SPHERES:
            d = C[0, o.slot_a[p]] - C[0, o.slot_b[p]]
            return float(np.linalg.norm(d) - o.slot_radius[o.slot_a[p]] - o.slot_radius[o.slot_b[p]])
        return float(o.native_distances(self.lease, self.q()[None], nodes, pairs)[0])


def random_pose(rng, scale):
    q = rng.normal(size=4)
    return RigidTransform(RotationMatrix(Quaternion(q / np.linalg.norm(q))), rng.uniform(-scale, scale, 3))


def add_shape_body(plant, name, shape, p_WB):
    body = plant.AddRigidBody(name, SpatialInertia.SolidSphereWithMass(1.0, 0.1))
    plant.RegisterCollisionGeometry(body, RigidTransform(), shape, f"{name}_geometry", h.friction())
    plant.SetDefaultFloatingBaseBodyPose(body, RigidTransform(p_WB))
    return body


def check_against_reference(world, a, b, scale, expected, rng):
    p = world.pair(a, b)
    separated = penetrating = 0
    for _ in range(250):
        X_A, X_B = random_pose(rng, scale), random_pose(rng, scale)
        world.set_pose(a, X_A)
        world.set_pose(b, X_B)
        reference = expected(X_A, X_B)
        if reference is None:
            continue
        phi, pa, pb = world.measure(p, True)
        assert abs(phi - reference) < NATIVE
        assert abs(world.batched(p) - reference) < NATIVE
        if phi > 1e-9:
            separated += 1
            assert abs(np.linalg.norm(pa - pb) - phi) < NATIVE
        else:
            penetrating += 1
    assert separated > 0 and penetrating > 0


def test_sphere_sphere_matches_analytic_distance():
    builder = RobotDiagramBuilder(time_step=0.0)
    add_shape_body(builder.plant(), "sphere_a", Sphere(0.13), [-1, 0, 0])
    add_shape_body(builder.plant(), "sphere_b", Sphere(0.21), [1, 0, 0])
    world = World(builder.Build())
    assert world.oracle.route[0] == Route.SPHERES
    check_against_reference(world, "sphere_a", "sphere_b", 0.4,
                            lambda A, B: np.linalg.norm(B.translation() - A.translation()) - 0.34,
                            np.random.default_rng(20260826))


def test_sphere_box_matches_analytic_distance():
    builder = RobotDiagramBuilder(time_step=0.0)
    half = np.array([0.15, 0.2, 0.25])
    add_shape_body(builder.plant(), "sphere", Sphere(0.1), [-1, 0, 0])
    add_shape_body(builder.plant(), "box", Box(*(2 * half)), [1, 0, 0])
    world = World(builder.Build())
    assert world.oracle.route[0] == Route.SPHERE_BOX

    def expected(X_S, X_B):
        p_B = X_B.inverse().multiply(X_S.translation())
        d = np.linalg.norm(np.maximum(np.abs(p_B) - half, 0.0))
        return None if d <= 0.0 else d - 0.1  # A center inside the box has no such reference.

    check_against_reference(world, "sphere", "box", 0.5, expected, np.random.default_rng(881))


def test_sphere_box_route_matches_drake_inside_the_box_too():
    builder = RobotDiagramBuilder(time_step=0.0)
    add_shape_body(builder.plant(), "sphere", Sphere(0.07), [-1, 0, 0])
    add_shape_body(builder.plant(), "box", Box(0.4, 0.3, 0.2), [1, 0, 0])
    world = World(builder.Build())
    rng = np.random.default_rng(3)
    for _ in range(300):
        world.set_pose("sphere", random_pose(rng, 0.25))
        world.set_pose("box", random_pose(rng, 0.25))
        p = world.pair("sphere", "box")
        drake = world.oracle._query_object(world.lease, world.q()).ComputeSignedDistancePairClosestPoints(
            world.oracle.geometry_a[p], world.oracle.geometry_b[p]).distance
        assert abs(world.batched(p) - drake) < 1e-12


# ----------------------------------------------------------------------------- half spaces

RADIUS, LENGTH = 0.11, 0.34
HALF = np.array([0.15, 0.2, 0.25])
RADII = np.array([0.12, 0.19, 0.07])
TETRA = np.array([[0.0, 0.31, 0.04, -0.06], [0.0, 0.02, 0.27, 0.05], [0.0, -0.05, 0.03, 0.23]])


@pytest.fixture(scope="module")
def halfspace_world():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_shape_body(plant, "halfspace", HalfSpace(), [0, 0, -3])
    add_shape_body(plant, "sphere", Sphere(RADIUS), [-3, 0, 0])
    add_shape_body(plant, "box", Box(*(2 * HALF)), [-1, 0, 0])
    add_shape_body(plant, "capsule", Capsule(RADIUS, LENGTH), [1, 0, 0])
    add_shape_body(plant, "cylinder", Cylinder(RADIUS, LENGTH), [3, 0, 0])
    add_shape_body(plant, "ellipsoid", Ellipsoid(*RADII), [0, 3, 0])
    add_shape_body(plant, "tetra", Convex(TETRA, "tetra"), [0, -3, 0])
    return World(builder.Build())


def test_every_halfspace_partner_matches_hand_derived_formula_exactly(halfspace_world):
    world = halfspace_world
    rng = np.random.default_rng(4242)
    names = ["sphere", "box", "capsule", "cylinder", "ellipsoid", "tetra"]
    shapes = {n: world.model.scene_graph().model_inspector().GetShape(world.geometry(n)) for n in names}
    positive = negative = 0
    for _ in range(200):
        X_H = random_pose(rng, 0.3)
        world.set_pose("halfspace", X_H)
        poses = {}
        for n in names:
            poses[n] = random_pose(rng, 0.4)
            world.set_pose(n, poses[n])
        normal, p0 = X_H.rotation().matrix()[:, 2], X_H.translation()
        for n in names:
            p = world.pair("halfspace", n)
            assert world.oracle.route[p] in (Route.HALFSPACE_A, Route.HALFSPACE_B)
            reference = h._halfspace_reference(normal, p0, poses[n], shapes[n])
            phi, pa, pb = world.measure(p, True)
            assert abs(phi - reference) < EXACT
            assert abs(world.batched(p) - reference) < EXACT
            on_plane = pa if world.oracle.geometry_a[p] == world.geometry("halfspace") else pb
            assert abs(normal @ (on_plane - p0)) < EXACT
            assert abs(np.linalg.norm(pa - pb) - abs(phi)) < EXACT
            positive, negative = positive + (phi > 0), negative + (phi <= 0)
    assert positive > 0 and negative > 0


def test_axis_parallel_cylinder_direction_is_handled(halfspace_world):
    world = halfspace_world
    world.set_pose("halfspace", RigidTransform([0, 0, -0.5]))
    world.set_pose("cylinder", RigidTransform([0.0, 0.0, 0.4]))
    p = world.pair("halfspace", "cylinder")
    phi, pa, pb = world.measure(p, True)
    assert abs(phi - (0.9 - LENGTH / 2)) < EXACT
    assert abs(world.batched(p) - (0.9 - LENGTH / 2)) < EXACT
    assert abs(np.linalg.norm(pa - pb) - abs(phi)) < EXACT


# ----------------------------------------------------------------------------- the probe


def cube_obj(half):
    corners = [(sx * half[0], sy * half[1], sz * half[2]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = ["1 2 4 3", "5 7 8 6", "1 5 6 2", "3 4 8 7", "1 3 7 5", "2 6 8 4"]
    text = "".join(f"v {x} {y} {z}\n" for x, y, z in corners) + "".join(f"f {f}\n" for f in faces)
    return InMemoryMesh(mesh_file=MemoryFile(text, ".obj", "cube.obj"))


CUBE_HALF = np.array([0.15, 0.20, 0.25])
CUBE_CORNERS = np.array([[sx * CUBE_HALF[0], sy * CUBE_HALF[1], sz * CUBE_HALF[2]]
                         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]).T


@pytest.fixture(scope="module")
def all_shapes_world():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_shape_body(plant, "sphere", Sphere(0.1), [-1.5, 0, 1])
    add_shape_body(plant, "box", Box(0.2, 0.3, 0.4), [-0.5, 0, 1])
    add_shape_body(plant, "capsule", Capsule(0.08, 0.3), [0.5, 0, 1])
    add_shape_body(plant, "cylinder", Cylinder(0.09, 0.25), [1.5, 0, 1])
    add_shape_body(plant, "ellipsoid", Ellipsoid(0.12, 0.09, 0.07), [-1.5, 1.5, 1])
    add_shape_body(plant, "convex", Convex(CUBE_CORNERS, "convex"), [-0.5, 1.5, 1])
    add_shape_body(plant, "mesh", Mesh(cube_obj(CUBE_HALF)), [0.5, 1.5, 1])
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), HalfSpace(), "ground_geometry", h.friction())
    return World(builder.Build())


def test_probe_classifies_every_pair(all_shapes_world):
    world = all_shapes_world
    o = world.oracle
    ground = [g for g in o.slot_geometry if world.model.scene_graph().model_inspector().GetName(g) == "ground_geometry"][0]
    assert o.num_pairs == 21 + 7
    halfspace = native = 0
    for p in range(o.num_pairs):
        if o.geometry_a[p] == ground:
            assert o.route[p] == Route.HALFSPACE_A
            halfspace += 1
        elif o.geometry_b[p] == ground:
            assert o.route[p] == Route.HALFSPACE_B
            halfspace += 1
        else:
            # Drake's narrowphase, or the exact closed form for sphere-box (one sphere here).
            assert o.route[p] in (Route.NATIVE, Route.SPHERE_BOX)
            native += 1
        assert o.body_a[p] != o.body_b[p]
    assert (halfspace, native) == (7, 21)
    assert int((o.route == Route.SPHERE_BOX).sum()) == 1


def test_witness_points_are_consistent_for_separated_pairs(all_shapes_world):
    world = all_shapes_world
    rng = np.random.default_rng(31337)
    checked = 0
    for _ in range(25):
        for n in ["sphere", "box", "capsule", "cylinder", "ellipsoid", "convex", "mesh"]:
            world.set_pose(n, random_pose(rng, 0.6))
        for p in range(world.oracle.num_pairs):
            phi, pa, pb = world.measure(p, True)
            if phi <= 1e-6:
                continue
            assert abs(np.linalg.norm(pa - pb) - phi) < 1e-4
            checked += 1
    assert checked > 100


def test_id_ordering_is_symmetric(all_shapes_world):
    world = all_shapes_world
    rng = np.random.default_rng(777)
    for n in ["sphere", "box", "capsule", "cylinder", "ellipsoid", "convex", "mesh"]:
        world.set_pose(n, random_pose(rng, 0.5))
    o = world.oracle
    before = [(world.measure(p, True), world.batched(p)) for p in range(o.num_pairs)]
    swaps = [("geometry_a", "geometry_b"), ("body_a", "body_b"), ("slot_a", "slot_b"), ("class_a", "class_b")]
    saved = {k: getattr(o, k) for pair in swaps for k in pair}
    saved["route"] = o.route
    try:
        for x, y in swaps:
            setattr(o, x, saved[y])
            setattr(o, y, saved[x])
        o.route = np.where(saved["route"] == Route.HALFSPACE_A, Route.HALFSPACE_B,
                           np.where(saved["route"] == Route.HALFSPACE_B, Route.HALFSPACE_A, saved["route"]))
        for p in range(o.num_pairs):
            (phi, pa, pb), batched = before[p]
            phi2, pa2, pb2 = world.measure(p, True)
            assert phi == phi2 and np.array_equal(pa, pb2) and np.array_equal(pb, pa2)
            assert batched == world.batched(p)
    finally:
        for k, v in saved.items():
            setattr(o, k, v)


def test_halfspace_halfspace_pair_throws_at_construction():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), HalfSpace(), "ground_geometry", h.friction())
    add_shape_body(plant, "ceiling", HalfSpace(), [0, 0, 2])
    with pytest.raises(RuntimeError, match=r"two HalfSpace geometries[\s\S]*(ground_geometry[\s\S]*ceiling_geometry|"
                                           r"ceiling_geometry[\s\S]*ground_geometry)"):
        make_oracle(builder.Build())


def test_probe_snapshot_is_stable_across_constructions():
    builder = RobotDiagramBuilder(time_step=0.0)
    add_shape_body(builder.plant(), "sphere", Sphere(0.1), [-1, 0, 0])
    add_shape_body(builder.plant(), "box", Box(0.2, 0.2, 0.2), [1, 0, 0])
    model = builder.Build()
    first, _ = make_oracle(model)
    second, _ = make_oracle(model)
    assert first.geometry_a == second.geometry_a and first.geometry_b == second.geometry_b


def test_deformable_geometry_is_refused_by_name():
    builder = RobotDiagramBuilder(time_step=0.01)
    plant = builder.plant()
    add_shape_body(plant, "sphere", Sphere(0.1), [0, 0, 1])
    instance = GeometryInstance(RigidTransform([0, 0, 0.4]), Sphere(0.05), "squishy")
    props = ProximityProperties()
    AddContactMaterial(friction=h.friction(), properties=props)
    instance.set_proximity_properties(props)
    plant.mutable_deformable_model().RegisterDeformableBody(instance, DeformableBodyConfig(), 0.05)
    with pytest.raises(RuntimeError, match=r"deformable[\s\S]*squishy"):
        make_oracle(builder.Build())


def test_empty_world_probes_cleanly():
    oracle, _ = make_oracle(RobotDiagramBuilder(time_step=0.0).Build())
    assert oracle.num_pairs == 0


# ----------------------------------------------------------------------------- Mesh = convex hull


def test_mesh_distance_equals_convex_hull_distance():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_shape_body(plant, "mesh", Mesh(cube_obj(CUBE_HALF)), [-1, 0, 0])
    add_shape_body(plant, "convex", Convex(CUBE_CORNERS, "cube"), [1, 0, 0])
    add_shape_body(plant, "probe", Sphere(0.07), [0, 2, 0])
    world = World(builder.Build())
    rng = np.random.default_rng(9091)
    separated = penetrating = 0
    for _ in range(100):
        X = random_pose(rng, 0.3)
        world.set_pose("mesh", X)
        world.set_pose("convex", X)
        world.set_pose("probe", random_pose(rng, 0.5))
        phi = world.measure(world.pair("mesh", "probe"))
        assert abs(phi - world.measure(world.pair("convex", "probe"))) < EXACT
        separated, penetrating = separated + (phi > 0), penetrating + (phi <= 0)
    assert separated > 0 and penetrating > 0


def l_prism_mesh():
    profile = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)]
    n = len(profile)
    lines = [f"v {x} {y} {z}" for z in (-0.5, 0.5) for x, y in profile]
    for i in range(n):
        j = (i + 1) % n
        lines += [f"f {i + 1} {j + 1} {j + 1 + n}", f"f {i + 1} {j + 1 + n} {i + 1 + n}"]
    for i in range(1, n - 1):
        lines += [f"f 1 {i + 2} {i + 1}", f"f {1 + n} {i + 1 + n} {i + 2 + n}"]
    return InMemoryMesh(mesh_file=MemoryFile("\n".join(lines) + "\n", ".obj", "ccd_l_prism.obj"))


def test_nonconvex_mesh_is_measured_as_its_convex_hull():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_shape_body(plant, "l_mesh", Mesh(l_prism_mesh()), [0, 0, 0])
    add_shape_body(plant, "l_convex", Convex(l_prism_mesh()), [0, 0, 0])
    add_shape_body(plant, "probe", Sphere(0.05), [5, 5, 5])
    world = World(builder.Build())
    world.set_pose("l_mesh", RigidTransform())
    world.set_pose("l_convex", RigidTransform())
    for p_W, negative in (([1.4, 1.4, 0.0], True), ([1.9, 1.9, 0.0], False)):
        world.set_pose("probe", RigidTransform(p_W))
        phi = world.measure(world.pair("l_mesh", "probe"))
        assert abs(phi - world.measure(world.pair("l_convex", "probe"))) < EXACT
        assert (phi < 0.0) == negative


def test_halfspace_route_against_mesh_uses_the_same_hull():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    add_shape_body(plant, "mesh", Mesh(cube_obj(CUBE_HALF)), [0, 0, 1])
    add_shape_body(plant, "convex", Convex(cube_obj(CUBE_HALF)), [0, 3, 1])
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(), HalfSpace(), "ground", h.friction())
    world = World(builder.Build())
    o = world.oracle
    names = {world.model.scene_graph().model_inspector().GetName(g): g for g in o.slot_geometry}
    pm = [p for p in range(o.num_pairs) if {o.geometry_a[p], o.geometry_b[p]} == {names["ground"], world.geometry("mesh")}][0]
    pc = [p for p in range(o.num_pairs) if {o.geometry_a[p], o.geometry_b[p]} == {names["ground"], world.geometry("convex")}][0]
    rng = np.random.default_rng(5150)
    for _ in range(100):
        X = random_pose(rng, 0.3)
        world.set_pose("mesh", X)
        world.set_pose("convex", X)
        lowest = X.multiply(CUBE_CORNERS)[2].min()
        for p in (pm, pc):
            assert abs(world.measure(p) - lowest) < EXACT
            assert abs(world.batched(p) - lowest) < EXACT
