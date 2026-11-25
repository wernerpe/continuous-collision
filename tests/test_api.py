"""What the checker accepts and how it refuses (port of api_test.cc). A refusal must name what
the caller has to fix, so these tests assert on message content."""

import math

import numpy as np
import pytest
from pydrake.common import Parallelism
from pydrake.geometry import (AddContactMaterial, Box, GeometryInstance, HalfSpace, ProximityProperties,
                              Sphere)
from pydrake.math import RigidTransform
from pydrake.multibody.fem import DeformableBodyConfig
from pydrake.multibody.tree import FixedOffsetFrame, PrismaticJoint, RevoluteJoint
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve

from continuous_collision import ContinuousCollisionChecker, Finding, Options, Result, Verdict

import helpers as h


def arm_world():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    link, tool = plant.AddRigidBody("link", h.inertia()), plant.AddRigidBody("tool", h.inertia())
    plant.AddJoint(RevoluteJoint("shoulder", plant.world_frame(), link.body_frame(), [0, 0, 1]))
    F = plant.AddFrame(FixedOffsetFrame("F", link.body_frame(), RigidTransform([0.30, 0, 0])))
    plant.AddJoint(PrismaticJoint("slide", F, tool.body_frame(), [1, 0, 0]))
    plant.RegisterCollisionGeometry(link, RigidTransform([0.15, 0, 0]), Box(0.30, 0.05, 0.05), "link_geom", h.friction())
    plant.RegisterCollisionGeometry(tool, RigidTransform(), Sphere(0.04), "tool_geom", h.friction())
    post = plant.AddRigidBody("post", h.inertia())
    plant.WeldFrames(plant.world_frame(), post.body_frame(), RigidTransform([0.0, 0.60, 0.0]))
    plant.RegisterCollisionGeometry(post, RigidTransform(), Sphere(0.08), "post_geom", h.friction())
    return builder.Build()


def floating_base_world():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    base, arm = plant.AddRigidBody("base", h.inertia()), plant.AddRigidBody("arm", h.inertia())
    F = plant.AddFrame(FixedOffsetFrame("F", base.body_frame(), RigidTransform([0.10, 0, 0])))
    plant.AddJoint(RevoluteJoint("elbow", F, arm.body_frame(), [0, 0, 1]))
    plant.RegisterCollisionGeometry(base, RigidTransform(), Sphere(0.05), "base_geom", h.friction())
    plant.RegisterCollisionGeometry(arm, RigidTransform([0.12, 0, 0]), Box(0.24, 0.04, 0.04), "arm_geom", h.friction())
    post = plant.AddRigidBody("post", h.inertia())
    plant.WeldFrames(plant.world_frame(), post.body_frame(), RigidTransform([0.0, 0.90, 0.0]))
    plant.RegisterCollisionGeometry(post, RigidTransform(), Sphere(0.06), "post_geom", h.friction())
    return builder.Build()


def floating_joint_name(plant):
    for ji in plant.GetJointIndices():
        if plant.get_joint(ji).type_name() == "quaternion_floating":
            return plant.get_joint(ji).name()
    raise AssertionError("no quaternion floating joint")


def floating_q(p, elbow):
    return np.array([1.0, 0.0, 0.0, 0.0, *p, elbow])


# --------------------------------------------------------------------------- joints


def test_moving_quaternion_base_throws_naming_the_joint():
    model = floating_base_world()
    checker = h.checker(model)
    name = floating_joint_name(model.plant())
    points = np.column_stack((floating_q([0, 0, 0], 0), floating_q([0, 0, 0], 0)))
    points[0, 1] = points[3, 1] = 0.7071067811865476
    with pytest.raises(RuntimeError) as e:
        checker.CheckTrajectory(BezierCurve(0.0, 1.0, points))
    assert name in str(e.value) and "quaternion_floating" in str(e.value)
    assert "constant-coordinate carve-out" in str(e.value)
    translated = np.column_stack((floating_q([0, 0, 0], 0), floating_q([0.2, 0, 0], 0)))
    with pytest.raises(RuntimeError) as e:
        checker.CheckTrajectory(BezierCurve(0.0, 1.0, translated))
    assert name in str(e.value) and "coordinate 4" in str(e.value)


@pytest.mark.parametrize("wobble", [0.0, 6e-8])
def test_constant_quaternion_base_is_accepted_end_to_end(wobble):
    checker = h.checker(floating_base_world())
    points = np.column_stack([floating_q([0.05, -0.10, 0.0], 0.0)] * 3)
    points[5, 1] += wobble
    points[7, 1], points[7, 2] = 0.35, 0.70
    assert checker.CheckTrajectory(BezierCurve(0.0, 1.0, points)).verdict == Verdict.kCertifiedFree


@pytest.mark.parametrize("kind", ["rpy_floating", "universal"])
def test_moving_other_unsupported_joints_throws(kind):
    from pydrake.multibody.tree import RpyFloatingJoint, UniversalJoint

    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    link = plant.AddRigidBody("link", h.inertia())
    if kind == "rpy_floating":
        plant.AddJoint(RpyFloatingJoint("base", plant.world_frame(), link.body_frame()))
    else:
        plant.AddJoint(UniversalJoint("base", plant.world_frame(), link.body_frame(), 0.0))
    plant.RegisterCollisionGeometry(link, RigidTransform([0.2, 0, 0]), Sphere(0.05), "g", h.friction())
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([0, 0, 1.0]), Sphere(0.05), "w", h.friction())
    model = builder.Build()
    nq = model.plant().num_positions()
    q1 = np.zeros(nq)
    q2 = q1.copy()
    q2[0] = 0.3
    with pytest.raises(RuntimeError, match=f"coordinate 0 of joint 'base', whose type '{kind}'"):
        h.checker(model).CheckEdge(q1, q2)
    assert h.checker(model).CheckEdge(q1, q1).verdict == Verdict.kCertifiedFree


# --------------------------------------------------------------------------- geometry


def test_rotating_halfspace_throws_at_construction():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    blade = plant.AddRigidBody("blade", h.inertia())
    plant.AddJoint(RevoluteJoint("spin", plant.world_frame(), blade.body_frame(), [1, 0, 0]))
    plant.RegisterCollisionGeometry(blade, RigidTransform(), HalfSpace(), "blade_halfspace", h.friction())
    post = plant.AddRigidBody("post", h.inertia())
    plant.WeldFrames(plant.world_frame(), post.body_frame(), RigidTransform([0.0, 0.5, 0.0]))
    plant.RegisterCollisionGeometry(post, RigidTransform(), Sphere(0.05), "post_geom", h.friction())
    with pytest.raises(RuntimeError) as e:
        h.checker(builder.Build())
    for needle in ("blade_halfspace", "post_geom", "spin", "Box"):
        assert needle in str(e.value)


def test_anchored_halfspace_under_a_rotating_arm_is_accepted():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    link = plant.AddRigidBody("link", h.inertia())
    plant.AddJoint(RevoluteJoint("shoulder", plant.world_frame(), link.body_frame(), [0, 0, 1]))
    plant.RegisterCollisionGeometry(link, RigidTransform([0.15, 0, 0]), Box(0.30, 0.05, 0.05), "link_geom", h.friction())
    ground = plant.AddRigidBody("ground", h.inertia())
    plant.WeldFrames(plant.world_frame(), ground.body_frame(), RigidTransform([0, 0, -0.4]))
    plant.RegisterCollisionGeometry(ground, RigidTransform(), HalfSpace(), "ground_halfspace", h.friction())
    assert h.checker(builder.Build()).CheckEdge([0.0], [1.5]).verdict == Verdict.kCertifiedFree


def test_deformable_geometry_is_refused_naming_it():
    builder = RobotDiagramBuilder(time_step=0.01)
    plant = builder.plant()
    post = plant.AddRigidBody("post", h.inertia())
    plant.WeldFrames(plant.world_frame(), post.body_frame(), RigidTransform([0.4, 0, 0]))
    plant.RegisterCollisionGeometry(post, RigidTransform(), Sphere(0.05), "post_geom", h.friction())
    link = plant.AddRigidBody("link", h.inertia())
    plant.AddJoint(RevoluteJoint("shoulder", plant.world_frame(), link.body_frame(), [0, 0, 1]))
    plant.RegisterCollisionGeometry(link, RigidTransform([0.15, 0, 0]), Box(0.30, 0.05, 0.05), "link_geom", h.friction())
    instance = GeometryInstance(RigidTransform([0, 0.5, 0]), Sphere(0.05), "squishy_blob")
    props = ProximityProperties()
    AddContactMaterial(1e8, None, h.friction(), props)
    instance.set_proximity_properties(props)
    config = DeformableBodyConfig()
    config.set_youngs_modulus(1e6)
    plant.mutable_deformable_model().RegisterDeformableBody(instance, config, 0.05)
    model = builder.Build()
    assert len(model.scene_graph().model_inspector().GetAllDeformableGeometryIds()) == 1
    with pytest.raises(RuntimeError, match=r"deformable[\s\S]*squishy_blob"):
        h.checker(model)


# --------------------------------------------------------------------------- dimensions, options


def test_dimension_mismatch_messages_name_the_sizes():
    model = arm_world()
    checker = h.checker(model)
    assert model.plant().num_positions() == 2
    with pytest.raises(RuntimeError) as e:
        checker.CheckPath(np.zeros((3, 4)))
    for needle in ("CheckPath", "3 rows", "2 generalized positions", "waypoints are columns"):
        assert needle in str(e.value)
    with pytest.raises(RuntimeError, match=r"CheckEdge[\s\S]*sizes 2 and 5"):
        checker.CheckEdge(np.zeros(2), np.zeros(5))
    with pytest.raises(RuntimeError, match=r"7 rows[\s\S]*2 generalized positions"):
        checker.CheckTrajectory(BezierCurve(0.0, 1.0, np.zeros((7, 3))))
    with pytest.raises(RuntimeError, match="at least 2 waypoints"):
        checker.CheckPath(np.zeros((2, 1)))


@pytest.mark.parametrize("field,value,needle", [
    ("distance_resolution", 0.0, "distance_resolution"),
    ("distance_resolution", -1e-3, "positive"),
    ("distance_resolution", math.nan, "distance_resolution"),
    ("distance_resolution", math.inf, "finite"),
    ("margin", -0.01, "nonnegative"),
    ("margin", math.nan, "margin"),
])
def test_options_validation_messages_are_actionable(field, value, needle):
    checker = h.checker(arm_world())
    trajectory = BezierCurve(0.0, 1.0, np.array([[0.0, 0.2], [0.0, 0.05]]))
    bad = h.serial()
    setattr(bad, field, value)
    with pytest.raises(RuntimeError, match=needle):
        checker.CheckTrajectory(trajectory, bad)
    with pytest.raises(RuntimeError, match=needle):
        ContinuousCollisionChecker(arm_world(), bad)


def test_null_model_is_refused():
    with pytest.raises(RuntimeError, match=r"model is null[\s\S]*finalized"):
        ContinuousCollisionChecker(None)


# --------------------------------------------------------------------------- the Python surface


def test_public_names_and_defaults():
    import continuous_collision

    assert sorted(continuous_collision.__all__) == ["ContinuousCollisionChecker", "Finding", "Options",
                                                    "Result", "Verdict"]
    options = Options()
    assert options.margin == 0.0 and options.distance_resolution == 1e-6
    assert tuple(options.continuous_revolute_indices) == ()
    assert options.parallelism.num_threads() == Parallelism.Max().num_threads()
    assert [v.name for v in Verdict] == ["kCertifiedFree", "kViolationFound", "kInconclusive"]
    assert Result().finding is None and Finding().nearest_a_W is None


def test_default_options_are_copied_and_positional_arguments_work():
    model = arm_world()
    options = Options(margin=0.0)
    checker = ContinuousCollisionChecker(model, options)
    options.margin = 1.0  # Mutating the caller's object afterwards does not reach the checker.
    assert checker.CheckEdge([0.0, 0.0], [0.0, 0.2]).verdict == Verdict.kCertifiedFree
    assert checker.model() is model
