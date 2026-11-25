"""A plate thin enough that Drake's sampled SceneGraphCollisionChecker steps over it, and the
mirror image: a millimetre-scale gap that is free and certifies with a bounded node budget
(port of thin_obstacle_test.cc). The robot is a 2-dof Cartesian gantry whose configuration is
the tool-sphere center, so every number below is exact."""

import math

import numpy as np
import pytest
from pydrake.common import Parallelism
from pydrake.geometry import Box, Sphere
from pydrake.math import RigidTransform
from pydrake.multibody.tree import PrismaticJoint
from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

from continuous_collision import Verdict

import helpers as h

TOOL_RADIUS = 0.005
PLATE_THICKNESS = 0.001
PLATE_X = 0.025
DRAKE_EDGE_STEP = 0.05
CONTACT_HALF_WIDTH = TOOL_RADIUS + 0.5 * PLATE_THICKNESS


def add_gantry(plant):
    robot = plant.AddModelInstance("robot")
    carriage = plant.AddRigidBody("carriage", robot, h.inertia())
    tool = plant.AddRigidBody("tool", robot, h.inertia())
    plant.AddJoint(PrismaticJoint("gantry_x", plant.world_frame(), carriage.body_frame(), [1, 0, 0]))
    plant.AddJoint(PrismaticJoint("gantry_y", carriage.body_frame(), tool.body_frame(), [0, 1, 0]))
    plant.RegisterCollisionGeometry(tool, RigidTransform(), Sphere(TOOL_RADIUS), "tool_geom", h.friction())


def add_anchored_box(plant, name, p_W, size):
    body = plant.AddRigidBody(name, plant.GetModelInstanceByName("env"), h.inertia())
    plant.WeldFrames(plant.world_frame(), body.body_frame(), RigidTransform(p_W))
    plant.RegisterCollisionGeometry(body, RigidTransform(), Box(*size), f"{name}_geom", h.friction())


def plate_world(plate_x, thickness):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    plant.AddModelInstance("env")
    add_gantry(plant)
    add_anchored_box(plant, "plate", [plate_x, 0.0, 0.0], [thickness, 0.6, 0.6])
    return builder.Build()


def slot_world(half_gap):
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    plant.AddModelInstance("env")
    add_gantry(plant)
    add_anchored_box(plant, "slot_left", [0.0, half_gap, 0.0], [0.8, PLATE_THICKNESS, 0.6])
    add_anchored_box(plant, "slot_right", [0.0, -half_gap, 0.0], [0.8, PLATE_THICKNESS, 0.6])
    return builder.Build()


def drake_checker(model, edge_step_size):
    return SceneGraphCollisionChecker(model=model,
                                      robot_model_instances=[model.plant().GetModelInstanceByName("robot")],
                                      edge_step_size=edge_step_size,
                                      implicit_context_parallelism=Parallelism(1))


Q1, Q2 = np.array([-0.5, 0.0]), np.array([0.5, 0.0])


def test_drake_sampled_checker_misses_the_thin_plate():
    drake = drake_checker(plate_world(PLATE_X, PLATE_THICKNESS), DRAKE_EDGE_STEP)
    assert abs(drake.ComputeConfigurationDistance(Q1, Q2) - 1.0) < 1e-15
    assert drake.CheckEdgeCollisionFree(Q1, Q2), "premise: default sampling misses the 1 mm plate"

    def gap(x):
        return abs(x - DRAKE_EDGE_STEP * round(x / DRAKE_EDGE_STEP))

    for i in range(21):
        offset = i * DRAKE_EDGE_STEP / 20
        probe = drake_checker(plate_world(offset, PLATE_THICKNESS), DRAKE_EDGE_STEP)
        assert probe.CheckEdgeCollisionFree(Q1, Q2) == (gap(offset) > CONTACT_HALF_WIDTH)
    intervals = math.ceil(1.0 / DRAKE_EDGE_STEP)
    nearest = math.inf
    for k in range(intervals + 1):
        q = drake.InterpolateBetweenConfigurations(Q1, Q2, k / intervals)
        nearest = min(nearest, abs(q[0] - PLATE_X))
        assert drake.CheckConfigCollisionFree(q)
    assert abs(nearest - 0.025) < 1e-12 and nearest > CONTACT_HALF_WIDTH
    assert not drake_checker(plate_world(PLATE_X, PLATE_THICKNESS), 0.002).CheckEdgeCollisionFree(Q1, Q2)


def test_certified_checker_catches_the_thin_plate():
    model = plate_world(PLATE_X, PLATE_THICKNESS)
    checker = h.checker(model)
    result = checker.CheckEdge(Q1, Q2)
    assert result.verdict == Verdict.kViolationFound
    f = result.finding
    assert f.q.shape == (2,)
    assert 0.5 + PLATE_X - CONTACT_HALF_WIDTH < f.time < 0.5 + PLATE_X + CONTACT_HALF_WIDTH
    assert abs(f.q[0] - PLATE_X) < CONTACT_HALF_WIDTH and abs(f.q[1]) < 1e-15
    assert np.abs(np.array([-0.5 + f.time, 0.0]) - f.q).max() < 1e-12
    phi = h.ReferenceOracle(model).distance_at_finding(f)
    assert phi < 0.0 and abs(phi - f.distance) < 1e-12
    assert f.nearest_a_W is not None and f.nearest_b_W is not None
    assert checker.CheckPath(np.column_stack((Q1, Q2))).verdict == Verdict.kViolationFound


def test_narrow_gap_certified_with_bounded_node_budget():
    half_gap = 0.0085
    assert half_gap - 0.5 * PLATE_THICKNESS - TOOL_RADIUS > 0.0
    q1, q2 = np.array([-0.3, 0.0]), np.array([0.3, 0.0])
    assert drake_checker(slot_world(half_gap), DRAKE_EDGE_STEP).CheckEdgeCollisionFree(q1, q2)
    result = h.checker(slot_world(half_gap)).CheckEdge(q1, q2)
    assert result.verdict == Verdict.kCertifiedFree and result.finding is None
    assert 64 <= result.num_nodes < 640


def test_thickness_sweep_brackets_the_resolution_gap():
    first_caught = math.nan
    for thickness in (0.001, 0.002, 0.005, 0.010, 0.020, 0.030, 0.038, 0.040, 0.042, 0.050, 0.080):
        drake_free = drake_checker(plate_world(PLATE_X, thickness), DRAKE_EDGE_STEP).CheckEdgeCollisionFree(Q1, Q2)
        if not drake_free and math.isnan(first_caught):
            first_caught = thickness
        assert h.checker(plate_world(PLATE_X, thickness)).CheckEdge(Q1, Q2).verdict == Verdict.kViolationFound
    assert 0.03 < first_caught < 0.06
