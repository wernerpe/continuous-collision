"""Batched forward kinematics against Drake's, on random trees of every accepted joint type.

The prefilter and the closed-form distance routes run on these poses, so a disagreement here
is a soundness bug."""

import numpy as np
import pytest
from pydrake.geometry import Sphere
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.tree import (BallRpyJoint, PlanarJoint, PrismaticJoint, QuaternionFloatingJoint,
                                    RevoluteJoint, RpyFloatingJoint, ScrewJoint, SpatialInertia,
                                    UniversalJoint, WeldJoint)
from pydrake.planning import RobotDiagramBuilder

from continuous_collision.kinematics import BatchKinematics
from continuous_collision.motion_bound import KinematicsEngine

KINDS = ["revolute", "prismatic", "planar", "screw", "weld", "rpy_floating", "quaternion_floating",
         "ball_rpy", "universal"]


def random_transform(rng, scale):
    return RigidTransform(RollPitchYaw(rng.uniform(-np.pi, np.pi, 3)), rng.uniform(-scale, scale, 3))


def unit(rng):
    v = rng.normal(size=3)
    return v / np.linalg.norm(v)


def random_tree(rng, num_bodies=12):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    inertia = SpatialInertia.SolidSphereWithMass(1.0, 0.05)
    bodies = [plant.world_body()]
    for i in range(num_bodies):
        body = plant.AddRigidBody(f"b{i}", inertia)
        parent = bodies[rng.integers(0, len(bodies))]
        F = plant.AddFrame(_offset(f"F{i}", parent, random_transform(rng, 0.3)))
        M = plant.AddFrame(_offset(f"M{i}", body, random_transform(rng, 0.3)))
        kind = KINDS[i % len(KINDS)] if i < len(KINDS) else KINDS[rng.integers(0, len(KINDS))]
        name = f"j{i}"
        if kind == "revolute":
            plant.AddJoint(RevoluteJoint(name, F, M, unit(rng)))
        elif kind == "prismatic":
            plant.AddJoint(PrismaticJoint(name, F, M, unit(rng)))
        elif kind == "planar":
            plant.AddJoint(PlanarJoint(name, F, M, np.zeros(3)))
        elif kind == "screw":
            plant.AddJoint(ScrewJoint(name, F, M, unit(rng), rng.uniform(-0.8, 0.8), 0.0))
        elif kind == "weld":
            plant.AddJoint(WeldJoint(name, F, M, random_transform(rng, 0.2)))
        elif kind == "rpy_floating":
            plant.AddJoint(RpyFloatingJoint(name, F, M))
        elif kind == "quaternion_floating":
            plant.AddJoint(QuaternionFloatingJoint(name, F, M))
        elif kind == "ball_rpy":
            plant.AddJoint(BallRpyJoint(name, F, M, 0.0))
        else:
            plant.AddJoint(UniversalJoint(name, F, M, 0.0))
        plant.RegisterCollisionGeometry(body, RigidTransform(), Sphere(0.01), f"g{i}", CoulombFriction(1.0, 1.0))
        bodies.append(body)
    return builder.Build()


def _offset(name, body, X):
    from pydrake.multibody.tree import FixedOffsetFrame

    return FixedOffsetFrame(name, body.body_frame(), X)


@pytest.mark.parametrize("seed", range(12))
def test_batch_kinematics_matches_drake(seed):
    rng = np.random.default_rng(seed)
    model = random_tree(rng)
    plant = model.plant()
    kin = BatchKinematics(KinematicsEngine(model))
    context = plant.CreateDefaultContext()
    Q = rng.uniform(-2.0, 2.0, (25, plant.num_positions()))
    R, p = kin.body_poses(Q)
    worst = 0.0
    for i, q in enumerate(Q):
        plant.SetPositions(context, q)
        for b in range(plant.num_bodies()):
            from pydrake.multibody.tree import BodyIndex

            X = plant.EvalBodyPoseInWorld(context, plant.get_body(BodyIndex(b)))
            worst = max(worst, np.abs(R[i, b] - X.rotation().matrix()).max(),
                        np.abs(p[i, b] - X.translation()).max())
    assert worst < 1e-12


@pytest.mark.parametrize("seed", range(6))
def test_batch_kinematics_is_batch_invariant(seed):
    """A round's poses must not depend on how many nodes share the round, bit for bit: that is
    what makes the verdict and the witness a function of the trajectory alone."""
    rng = np.random.default_rng(100 + seed)
    model = random_tree(rng)
    kin = BatchKinematics(KinematicsEngine(model))
    Q = rng.uniform(-2.0, 2.0, (37, model.plant().num_positions()))
    together = kin.anchor_transforms(Q)
    for i in range(len(Q)):
        alone = kin.anchor_transforms(Q[i:i + 1])
        assert np.array_equal(together[i], alone[0])
