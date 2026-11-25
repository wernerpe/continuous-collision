"""Forward kinematics for a batch of configurations, in numpy.

Design note: the prefilter and the closed-form distance routes need geometry poses at every
node of a round. Asking Drake costs one Python round trip per body per configuration; the
joint maps X_FM(q) of the nine joint types the checker accepts are closed forms, so a round's
poses are a handful of vectorized operations per body instead. Agreement with Drake's own
forward kinematics is pinned to 1e-12 by the test suite on random plants of every joint type.
"""

from __future__ import annotations

import math

import numpy as np

from .motion_bound import JointKind, KinematicsEngine


def _skew(a) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


def _homogeneous(R, p) -> np.ndarray:
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, p
    return X


def _rot_x(t):
    c, s = np.cos(t), np.sin(t)
    R = np.zeros(t.shape + (3, 3))
    R[..., 0, 0] = 1.0
    R[..., 1, 1], R[..., 1, 2], R[..., 2, 1], R[..., 2, 2] = c, -s, s, c
    return R


def _rot_y(t):
    c, s = np.cos(t), np.sin(t)
    R = np.zeros(t.shape + (3, 3))
    R[..., 1, 1] = 1.0
    R[..., 0, 0], R[..., 0, 2], R[..., 2, 0], R[..., 2, 2] = c, s, -s, c
    return R


def _rot_z(t):
    c, s = np.cos(t), np.sin(t)
    R = np.zeros(t.shape + (3, 3))
    R[..., 2, 2] = 1.0
    R[..., 0, 0], R[..., 0, 1], R[..., 1, 0], R[..., 1, 1] = c, -s, s, c
    return R


def _quaternion_matrix(q):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - w * z)
    R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y)
    R[..., 2, 1] = 2 * (y * z + w * x)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


class BatchKinematics:
    """World poses of the plant's *anchors* for a batch of configurations Q of shape (N, n).

    An anchor is the world or a body whose inboard joint has coordinates; every other body is
    welded to an anchor at a fixed offset (`anchor_of_body`, `R_AB`, `p_AB`), so a round costs
    one step per mobile joint however many bodies hang off it."""

    def __init__(self, engine: KinematicsEngine):
        plant = engine.plant
        nb = engine.num_bodies
        self.anchor_of_body = np.full(nb, -1, dtype=int)
        self.R_AB = np.zeros((nb, 3, 3))
        self.p_AB = np.zeros((nb, 3))
        world = engine.body_order[0]
        self.anchor_of_body[world] = 0
        self.R_AB[world] = np.eye(3)
        self._steps = []  # (parent anchor, how, constants), one per non-world anchor in order
        context = plant.CreateDefaultContext()
        for b in engine.body_order[1:]:
            rec = engine.joints[engine.inboard_joint[b]]
            parent = rec.inboard
            # Fold the parent's offset in its anchor into the joint's parent-side frame.
            R_A = self.R_AB[parent] @ rec.X_PF.rotation().matrix()
            p_A = self.R_AB[parent] @ np.asarray(rec.X_PF.translation(), float) + self.p_AB[parent]
            X_MC = rec.X_CM.inverse()
            R_B, p_B = X_MC.rotation().matrix(), np.asarray(X_MC.translation(), float)
            joint = plant.get_joint(rec.index)
            if rec.kind is JointKind.WELD:
                X_FM = joint.X_FM()
                R_FM, p_FM = X_FM.rotation().matrix(), np.asarray(X_FM.translation(), float)
                self.anchor_of_body[b] = self.anchor_of_body[parent]
                self.R_AB[b] = R_A @ R_FM @ R_B
                self.p_AB[b] = p_A + R_A @ (p_FM + R_FM @ p_B)
                continue
            self.anchor_of_body[b] = len(self._steps) + 1
            self.R_AB[b] = np.eye(3)
            parent_anchor = int(self.anchor_of_body[parent])
            ps = rec.position_start
            A = _homogeneous(R_A, p_A)
            B = _homogeneous(R_B, p_B)
            if rec.kind in (JointKind.REVOLUTE, JointKind.SCREW):
                # X_FM(t) = [I + sin(t) K + (1 - cos(t)) K^2, t h], so A X_FM B is linear in
                # (1, sin t, 1 - cos t, t) with constant 4x4 coefficients.
                axis = (np.asarray(joint.revolute_axis(), float) if rec.kind is JointKind.REVOLUTE
                        else _screw_axis(joint, plant, context))
                K = _homogeneous(_skew(axis), np.zeros(3))
                K[3, 3] = 0.0
                H = np.zeros((4, 4))
                H[:3, 3] = axis * (rec.screw_pitch / (2 * math.pi))
                self._steps.append((parent_anchor, "rotary", ps, np.stack((A @ B, A @ K @ B, A @ K @ K @ B, A @ H @ B))))
            elif rec.kind is JointKind.PRISMATIC:
                H = np.zeros((4, 4))
                H[:3, 3] = np.asarray(joint.translation_axis(), float)
                self._steps.append((parent_anchor, "prismatic", ps, np.stack((A @ B, A @ H @ B))))
            else:
                self._steps.append((parent_anchor, rec.type_name, ps, (A, B)))
        self.num_anchors = len(self._steps) + 1
        rotary = [(ps, data) for _, how, ps, data in self._steps if how == "rotary"]
        self._rotary_coords = np.array([ps for ps, _ in rotary], dtype=int)
        self._rotary_D = np.array([d.reshape(4, 16) for _, d in rotary]).reshape(len(rotary), 4, 16)

    def anchor_transforms(self, Q: np.ndarray) -> list:
        """Homogeneous world poses of every anchor: a list of A arrays of shape (N, 4, 4),
        anchor 0 being the world. One small GEMM and one batched product per mobile joint."""
        Q = np.asarray(Q, dtype=float)
        N = Q.shape[0]
        t = Q[:, self._rotary_coords]
        coef = np.stack((np.ones_like(t), np.sin(t), 1.0 - np.cos(t), t), axis=1)  # (N, 4, r)
        X = [np.broadcast_to(np.eye(4), (N, 4, 4))]
        r = 0
        for parent, how, ps, data in self._steps:
            if how == "rotary":
                X_rel = (coef[:, :, r] @ self._rotary_D[r]).reshape(N, 4, 4)
                r += 1
            elif how == "prismatic":
                X_rel = data[0] + Q[:, ps, None, None] * data[1]
            else:
                R_FM, p_FM = _joint_map(how, Q, ps)
                X_FM = np.zeros((N, 4, 4))
                X_FM[:, :3, :3], X_FM[:, :3, 3], X_FM[:, 3, 3] = R_FM, p_FM, 1.0
                X_rel = data[0] @ X_FM @ data[1]
            X.append(X_rel if parent == 0 else X[parent] @ X_rel)
        return X

    def body_poses(self, Q: np.ndarray):
        """(R, p) of shapes (N, nb, 3, 3), (N, nb, 3): X_WB for every body (tests, examples)."""
        X = np.stack(self.anchor_transforms(Q), axis=1)
        R_A, p_A = X[:, :, :3, :3], X[:, :, :3, 3]
        a = self.anchor_of_body
        R = R_A[:, a] @ self.R_AB
        p = (R_A[:, a] @ self.p_AB[..., None])[..., 0] + p_A[:, a]
        return R, p


def _joint_map(type_name: str, Q: np.ndarray, ps: int):
    """X_FM for the multi-coordinate joint types, batched."""
    N = Q.shape[0]
    zero = np.zeros((N, 3))
    if type_name == "planar":
        p = zero.copy()
        p[:, :2] = Q[:, ps:ps + 2]
        return _rot_z(Q[:, ps + 2]), p
    if type_name == "universal":
        return _rot_x(Q[:, ps]) @ _rot_y(Q[:, ps + 1]), zero
    if type_name == "ball_rpy":
        return _rpy(Q[:, ps:ps + 3]), zero
    if type_name == "rpy_floating":
        return _rpy(Q[:, ps:ps + 3]), Q[:, ps + 3:ps + 6].copy()
    if type_name == "quaternion_floating":
        return _quaternion_matrix(Q[:, ps:ps + 4]), Q[:, ps + 4:ps + 7].copy()
    raise RuntimeError(f"BatchKinematics: no closed-form joint map for joint type '{type_name}'.")


def _rpy(rpy):
    return _rot_z(rpy[:, 2]) @ _rot_y(rpy[:, 1]) @ _rot_x(rpy[:, 0])


def _screw_axis(joint, plant, context) -> np.ndarray:
    """pydrake binds no screw-axis accessor, so read it off X_FM at one radian."""
    q = plant.GetPositions(context).copy()
    q[joint.position_start()] = 1.0
    plant.SetPositions(context, q)
    R = joint.frame_on_child().CalcPose(context, joint.frame_on_parent()).rotation().matrix()
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(1.0))
    return axis / np.linalg.norm(axis)
