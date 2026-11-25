"""Certify a straight edge and refuse a curved trajectory of an iiwa14 next to a wall.

    python examples/iiwa_wall.py

The model is Drake's bundled iiwa14 with its dense all-sphere collision model (the first run
downloads drake_models). The wall is a 5 cm box welded 0.6 m in front of the robot.
"""

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.plant import CoulombFriction
from pydrake.planning import RobotDiagramBuilder
from pydrake.trajectories import BezierCurve

from continuous_collision import ContinuousCollisionChecker, Options, Verdict

builder = RobotDiagramBuilder()
plant = builder.plant()
builder.parser().AddModels(url="package://drake_models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([0.6, 0.0, 0.5]), Box(0.05, 1.0, 1.0), "wall",
                                CoulombFriction(1.0, 1.0))
model = builder.Build()
checker = ContinuousCollisionChecker(model=model, default_options=Options(margin=0.01))

q_a = np.zeros(7)
q_b = np.array([1.5, -0.6, 0.0, -1.2, 0.0, 0.6, 0.0])
edge = checker.CheckEdge(q_a, q_b)
print(f"edge: {edge.verdict.name} ({edge.num_nodes} nodes)")
assert edge.verdict == Verdict.kCertifiedFree

lean = np.array([0.0, 1.6, 0.0, -0.4, 0.0, 0.8, 0.0])  # pulls the curve into the wall
result = checker.CheckTrajectory(BezierCurve(0.0, 2.0, np.column_stack([q_a, lean, q_b])))
w = result.finding
inspector = model.scene_graph().model_inspector()
print(f"curve: {result.verdict.name} at t = {w.time:.4f} s: {model.plant().get_body(w.body_a).name()} "
      f"({inspector.GetName(w.geometry_a)}) vs {model.plant().get_body(w.body_b).name()} "
      f"({inspector.GetName(w.geometry_b)}), distance {w.distance * 1e3:.4f} mm against a 10 mm margin")
print(f"closest points: {np.round(w.nearest_a_W, 4)} and {np.round(w.nearest_b_W, 4)}")
