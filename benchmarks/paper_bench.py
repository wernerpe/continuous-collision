"""An analog of the paper's Table I on Drake's bundled iiwa14 (all-sphere dense collision model).

The paper's frozen scenes and trajectories are not available, so this rebuilds the setup from
its description: an iiwa14 with the dense sphere model in front of an eight-box shelf (936
unfiltered pairs), a curved six-segment quintic trajectory (a clamped quintic B-spline, C4)
reaching into the shelf, its swept clearance to the shelf tuned to 2 mm, 10 mm and 50 mm by
moving the shelf, a piecewise-linear edge between two shelf waypoints, a grazing case, and a
dual-arm handover (3384 pairs). Margin 0 everywhere (the table states clearances, not margins).

    .venv/bin/python benchmarks/paper_bench.py make                 # tune, write paper_bench.json
    .venv/bin/python benchmarks/paper_bench.py run --impl package   # either venv
    <fork venv>/bin/python benchmarks/paper_bench.py run --impl fork
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
URL = "package://drake_models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf"
DATA = HERE / "paper_bench.json"

# Clamped quintic B-spline control points (11 -> six Bezier segments): from a raised home pose,
# into the middle compartment at three lateral positions and back out between them.
SHELF_CONTROL_POINTS = np.array([
    [0.00, -0.30, 0.00, -1.90, 0.00, 1.00, 0.00],
    [0.10, 0.10, 0.00, -1.70, 0.00, 1.10, 0.00],
    [0.35, 0.55, 0.00, -1.15, 0.00, 1.20, 0.00],
    [0.45, 0.75, 0.00, -0.95, 0.00, 1.10, 0.00],
    [0.30, 0.20, 0.00, -1.60, 0.00, 1.15, 0.00],
    [0.00, 0.80, 0.00, -0.90, 0.00, 1.05, 0.00],
    [-0.05, 0.25, 0.00, -1.55, 0.00, 1.10, 0.00],
    [-0.35, 0.55, 0.00, -1.15, 0.00, 1.20, 0.00],
    [-0.45, 0.75, 0.00, -0.95, 0.00, 1.10, 0.00],
    [-0.20, 0.10, 0.00, -1.70, 0.00, 1.10, 0.00],
    [0.00, -0.30, 0.00, -1.90, 0.00, 1.00, 0.00],
]).T

# The two shelf waypoints the piecewise-linear edge connects.
EDGE = (np.array([0.40, 0.55, 0.0, -1.15, 0.0, 1.15, 0.0]), np.array([-0.40, 0.55, 0.0, -1.15, 0.0, 1.15, 0.0]))


def friction():
    from pydrake.multibody.plant import CoulombFriction

    return CoulombFriction(1.0, 1.0)


def shelf_scene(x0: float):
    """The iiwa14 at the origin and an eight-box shelf whose front face is at x = x0."""
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform
    from pydrake.planning import RobotDiagramBuilder

    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    builder.parser().AddModels(url=URL)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    depth, half_width, t = 0.40, 0.50, 0.02
    boxes = [("board_z%d" % i, [x0 + depth / 2, 0.0, z], [depth, 2 * half_width, t])
             for i, z in enumerate((0.0, 0.30, 0.78, 1.10, 1.40))]
    boxes += [("side_left", [x0 + depth / 2, half_width, 0.70], [depth, t, 1.40]),
              ("side_right", [x0 + depth / 2, -half_width, 0.70], [depth, t, 1.40]),
              ("back", [x0 + depth, 0.0, 0.70], [t, 2 * half_width, 1.40])]
    for name, p, size in boxes:
        plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(p), Box(*size), name, friction())
    return builder.Build()


def dual_arm_scene(separation: float):
    """Two iiwa14 facing each other `separation` apart over a table: 3384 unfiltered pairs."""
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform, RollPitchYaw
    from pydrake.planning import RobotDiagramBuilder

    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    parser = builder.parser()
    parser.SetAutoRenaming(True)
    left = parser.AddModels(url=URL)[0]
    right = parser.AddModels(url=URL)[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", left), RigidTransform([0.0, 0.0, 0.0]))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", right),
                     RigidTransform(RollPitchYaw(0, 0, math.pi), [separation, 0.0, 0.0]))
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform([separation / 2, 0.0, -0.06]),
                                    Box(0.5, 1.2, 0.1), "table", friction())
    return builder.Build()


def handover_points():
    """Both arms reach to the middle and back: one Bezier of order 5 over 14 coordinates."""
    reach = [0.0, 0.95, 0.0, -0.90, 0.0, 1.05, 0.0]
    home = [0.0, 0.10, 0.0, -1.60, 0.0, 1.10, 0.0]
    a = np.array([home, [0.15, 0.6, 0, -1.2, 0, 1.1, 0], reach, reach, [-0.15, 0.6, 0, -1.2, 0, 1.1, 0], home])
    b = np.array([home, [-0.15, 0.6, 0, -1.2, 0, 1.1, 0], reach, reach, [0.15, 0.6, 0, -1.2, 0, 1.1, 0], home])
    return np.hstack((a, b)).T


def bspline(points):
    from pydrake.math import BsplineBasis
    from pydrake.trajectories import BsplineTrajectory

    return BsplineTrajectory(BsplineBasis(6, points.shape[1]), [points[:, [j]] for j in range(points.shape[1])])


def swept_clearance(model, trajectory, samples, env_only=True):
    """Dense-sampled minimum distance (Drake only), refined by golden-section around the best
    sample, over pairs with an environment geometry (or all pairs)."""
    plant, sg = model.plant(), model.scene_graph()
    inspector = sg.model_inspector()
    context = model.CreateDefaultContext()
    pc, sc = plant.GetMyMutableContextFromRoot(context), sg.GetMyContextFromRoot(context)
    world = inspector.world_frame_id()

    def clearance(t):
        plant.SetPositions(pc, trajectory.value(t).ravel())
        qo = sg.get_query_output_port().Eval(sc)
        best = math.inf
        for r in qo.ComputeSignedDistancePairwiseClosestPoints(0.5):
            if env_only and world not in (inspector.GetFrameId(r.id_A), inspector.GetFrameId(r.id_B)):
                continue
            best = min(best, r.distance)
        return best

    t0, t1 = trajectory.start_time(), trajectory.end_time()
    ts = np.linspace(t0, t1, samples)
    values = np.array([clearance(t) for t in ts])
    i = int(np.argmin(values))
    lo, hi = ts[max(i - 1, 0)], ts[min(i + 1, samples - 1)]
    g = (math.sqrt(5) - 1) / 2
    for _ in range(60):
        a, b = hi - g * (hi - lo), lo + g * (hi - lo)
        if clearance(a) < clearance(b):
            hi = b
        else:
            lo = a
    return min(values[i], clearance(0.5 * (lo + hi)))


def make(args) -> int:
    from pydrake.trajectories import PiecewisePolynomial  # noqa: F401

    traj = bspline(SHELF_CONTROL_POINTS)
    out = {"shelf_control_points": SHELF_CONTROL_POINTS.tolist(), "edge": [e.tolist() for e in EDGE],
           "handover_points": handover_points().tolist(), "tiers": {}}
    for name, target in (("shelf_2mm", 0.002), ("shelf_1cm", 0.010), ("shelf_5cm", 0.050), ("grazing", 2.4e-9)):
        lo, hi = 0.30, 1.20  # shelf front x: lo collides, hi is clear
        for _ in range(60 if name == "grazing" else 40):
            mid = 0.5 * (lo + hi)
            c = swept_clearance(shelf_scene(mid), traj, args.samples)
            (lo, hi) = (lo, mid) if c > target else (mid, hi)
        x0 = hi
        c = swept_clearance(shelf_scene(x0), traj, args.samples)
        out["tiers"][name] = {"x0": x0, "swept_clearance_env": c}
        print(name, x0, c, flush=True)
    model = shelf_scene(out["tiers"]["shelf_1cm"]["x0"])
    from pydrake.trajectories import PiecewisePolynomial

    edge = PiecewisePolynomial.FirstOrderHold([0.0, 1.0], np.column_stack(EDGE))
    out["edge_clearance_env"] = swept_clearance(model, edge, args.samples)
    lo, hi = 1.5, 1.7  # base separation: lo collides, hi clears
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        c = swept_clearance(dual_arm_scene(mid), _handover(out), args.samples, env_only=False)
        (lo, hi) = (lo, mid) if c > 0.0163 else (mid, hi)
    out["separation"] = hi
    out["handover_clearance"] = swept_clearance(dual_arm_scene(hi), _handover(out), args.samples, env_only=False)
    print("edge", out["edge_clearance_env"], "handover", hi, out["handover_clearance"])
    DATA.write_text(json.dumps(out, indent=1))
    return 0


def _handover(data):
    from pydrake.trajectories import BezierCurve

    return BezierCurve(0.0, 1.0, np.asarray(data["handover_points"], float))


def run(args) -> int:
    if args.impl == "fork":
        from pydrake.planning import continuous_collision as api
    else:
        import continuous_collision as api
    from pydrake.common import Parallelism
    from pydrake.trajectories import PiecewisePolynomial

    data = json.loads(DATA.read_text())
    traj = bspline(np.asarray(data["shelf_control_points"], float))
    rows = []
    scenarios = [(name, shelf_scene(data["tiers"][name]["x0"]), traj, 20, 3) for name in ("shelf_2mm", "shelf_1cm", "shelf_5cm")]
    scenarios.append(("pwl_edge", shelf_scene(data["tiers"]["shelf_1cm"]["x0"]),
                      PiecewisePolynomial.FirstOrderHold([0.0, 1.0], np.column_stack([np.asarray(e) for e in data["edge"]])), 20, 3))
    scenarios.append(("dual_arm_handover", dual_arm_scene(data["separation"]), _handover(data), 20, 3))
    scenarios = [s + (1e-6,) for s in scenarios]
    if args.grazing:
        for resolution in (1e-6, 1e-8):
            scenarios.append(("grazing", shelf_scene(data["tiers"]["grazing"]["x0"]), traj, 3, 0, resolution))
    for name, model, trajectory, reps, warmups, resolution in scenarios:
        for threads in ([1] if args.impl == "package" else [1, None]):
            options = api.Options()
            options.margin = 0.0
            options.distance_resolution = resolution
            options.parallelism = Parallelism(threads) if threads else Parallelism.Max()
            checker = api.ContinuousCollisionChecker(model=model, default_options=options)
            pairs = len(model.scene_graph().model_inspector().GetCollisionCandidates())
            for _ in range(warmups):
                checker.CheckTrajectory(trajectory)
            walls = []
            for _ in range(reps):
                tick = time.perf_counter()
                result = checker.CheckTrajectory(trajectory)
                walls.append(time.perf_counter() - tick)
            row = {"scenario": name, "resolution": resolution, "impl": args.impl, "threads": threads, "pairs": pairs,
                   "verdict": result.verdict.name, "nodes": int(result.num_nodes),
                   "wall_median_ms": statistics.median(walls) * 1e3, "wall_min_ms": min(walls) * 1e3}
            rows.append(row)
            print(json.dumps(row), flush=True)
    Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("make")
    m.add_argument("--samples", type=int, default=2000)
    r = sub.add_parser("run")
    r.add_argument("--impl", choices=["fork", "package"], required=True)
    r.add_argument("--out", required=True)
    r.add_argument("--grazing", action="store_true")
    args = ap.parse_args(argv)
    return make(args) if args.cmd == "make" else run(args)


if __name__ == "__main__":
    sys.exit(main())
