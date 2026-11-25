"""How far Drake's box-box signed distance departs from the exact value (stock pydrake).

Random sizes and poses; the exact value is this package's box-box route, which is checked here
against alternating projections wherever the two disagree by more than 1e-9 m.

    .venv/bin/python benchmarks/drake_box_accuracy.py [--samples 50000]
"""

import argparse

import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import Box
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.tree import SpatialInertia
from pydrake.planning import RobotDiagramBuilder

from continuous_collision import ContinuousCollisionChecker


def pose(rng, scale):
    q = rng.normal(size=4)
    return RigidTransform(RotationMatrix(Quaternion(q / np.linalg.norm(q))), rng.uniform(-scale, scale, 3))


def projections(X_A, h_A, X_B, h_B, iterations=100000):
    RA, cA, RB, cB = X_A.rotation().matrix(), X_A.translation(), X_B.rotation().matrix(), X_B.translation()
    x, y = cA.copy(), cB.copy()
    for _ in range(iterations):
        y = cB + RB @ np.clip(RB.T @ (x - cB), -h_B, h_B)
        x = cA + RA @ np.clip(RA.T @ (y - cA), -h_A, h_A)
    return float(np.linalg.norm(x - y))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=50000)
    ap.add_argument("--worlds", type=int, default=50)
    args = ap.parse_args()
    rng = np.random.default_rng(2026)
    errors, worst = [], (0.0, None)
    for _ in range(args.worlds):
        sizes = [np.exp(rng.uniform(np.log(0.005), np.log(2.0), 3)) for _ in range(2)]
        builder = RobotDiagramBuilder(time_step=0.0)
        for name, size in zip("AB", sizes):
            body = builder.plant().AddRigidBody(name, SpatialInertia.SolidSphereWithMass(1.0, 0.1))
            builder.plant().RegisterCollisionGeometry(body, RigidTransform(), Box(*size), name, CoulombFriction(1, 1))
        model = builder.Build()
        plant, checker = model.plant(), ContinuousCollisionChecker(model)
        oracle, kin = checker._oracle, checker._kinematics
        lease, context = oracle.acquire(), plant.CreateDefaultContext()
        for _ in range(args.samples // args.worlds):
            X_A, X_B = pose(rng, 0.6), pose(rng, 0.6)
            plant.SetFreeBodyPose(context, plant.GetBodyByName("A"), X_A)
            plant.SetFreeBodyPose(context, plant.GetBodyByName("B"), X_B)
            q = plant.GetPositions(context)
            X = kin.anchor_transforms(q[None])
            Xs = np.stack(X, axis=1)
            exact = float(oracle.box_box_distance(Xs[:, :, :3, :3], oracle.centers(X), np.array([0]), np.array([0]))[0])
            drake = oracle.signed_distance(lease, q, 0)[0]
            error = drake - exact
            if abs(error) > 1e-9 and exact > 0:
                truth = projections(X_A, 0.5 * sizes[0], X_B, 0.5 * sizes[1])
                assert abs(truth - exact) < 1e-9, (truth, exact, drake)
            errors.append(error)
            if abs(error) > abs(worst[0]):
                worst = (error, (sizes, exact))
    e = np.abs(np.array(errors))
    print(f"{len(e)} box pairs: |Drake - exact| > 1e-6 m in {(e > 1e-6).sum()} "
          f"({100 * (e > 1e-6).mean():.3f}%), > 1e-9 m in {(e > 1e-9).sum()}; "
          f"largest {worst[0]:.3e} m (Drake {'over' if worst[0] > 0 else 'under'}-reports); "
          f"over-reports > 1e-6 m: {(np.array(errors) > 1e-6).sum()}")


if __name__ == "__main__":
    main()
