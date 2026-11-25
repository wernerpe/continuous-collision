"""The motion bound: bounding spheres, J(p), reaches and the lambda table.

Design note: the free ball of the paper rests on one inequality per coordinate -- moving
coordinate j alone displaces every material point of pair p's distal side by at most
lambda(j, p)|dq_j| -- with lambda read off the kinematic tree once (reaches by summing
translation norms along the chain, which rotations cannot lengthen) and specialized per
trajectory only through the control box. An under-bounding lambda is a silent soundness bug,
so every rule here mirrors the fork line for line and refuses what it cannot bound.
"""

from __future__ import annotations

import enum
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from pydrake.geometry import Box, Capsule, Convex, Cylinder, Ellipsoid, Mesh, Role, Sphere

from ._numerics import ShapeClass, classify


@dataclass
class BoundingSphere:
    """A sphere in the body frame L containing one proximity geometry."""

    center_L: np.ndarray
    radius: float


def compute_bounding_sphere(shape, X_LG) -> BoundingSphere:
    """Exact-containment sphere of `shape` posed at X_LG (a RigidTransform) in its body frame.

    Primitives are centered at the geometry origin; Convex and Mesh at the centroid of the
    convex hull Drake actually collides. Raises on every other shape, HalfSpace included."""
    t = np.asarray(X_LG.translation(), dtype=float)
    if type(shape) is Sphere:
        return BoundingSphere(t.copy(), float(shape.radius()))
    if type(shape) is Box:
        w, d, h = shape.width(), shape.depth(), shape.height()
        return BoundingSphere(t.copy(), 0.5 * math.sqrt(w * w + d * d + h * h))
    if type(shape) is Capsule:
        return BoundingSphere(t.copy(), 0.5 * shape.length() + shape.radius())
    if type(shape) is Cylinder:
        return BoundingSphere(t.copy(), math.hypot(shape.radius(), 0.5 * shape.length()))
    if type(shape) is Ellipsoid:
        return BoundingSphere(t.copy(), max(shape.a(), shape.b(), shape.c()))
    if type(shape) in (Convex, Mesh):
        V = hull_vertices(shape)
        if V.shape[1] == 0:
            raise RuntimeError("ComputeBoundingSphere(): the convex hull has no vertices.")
        R = X_LG.rotation().matrix()
        P = R @ V + t[:, None]
        centroid = P.sum(axis=1) / V.shape[1]
        radius = float(np.sqrt(((P - centroid[:, None]) ** 2).sum(axis=0)).max())
        return BoundingSphere(centroid, radius)
    raise RuntimeError(
        f"ComputeBoundingSphere(): does not support the shape type '{type(shape).__name__}'. "
        "Supported proximity shapes are Sphere, Box, Capsule, Cylinder, Ellipsoid, Convex and "
        "Mesh. HalfSpace has no finite bounding sphere and is governed by dedicated rules "
        "instead: it must be anchored, or move only by translation relative to its partner. Any "
        "other shape must be replaced by a Convex/Mesh approximation before it can be certified.")


def hull_vertices(shape) -> np.ndarray:
    """(3, V) vertices of the convex hull Drake collides for a Convex or Mesh."""
    hull = shape.GetConvexHull()
    return np.array([hull.vertex(i) for i in range(hull.num_vertices())], dtype=float).T.reshape(3, -1)


class JointKind(enum.Enum):
    WELD = "weld"
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    PLANAR = "planar"
    SCREW = "screw"
    UNSUPPORTED = "unsupported"  # Accepted only while every coordinate is carved out.


class CoordRule(enum.Enum):
    """lambda-tilde rule of one coordinate once the carve-out has removed it from J(p)."""

    TRANSLATION = 0  # 1
    ROTATION = 1  # r
    SCREW = 2  # r + |pitch| / 2 pi
    QUATERNION = 3  # 2 r / m


_ROTATIONAL = {JointKind.REVOLUTE, JointKind.SCREW, JointKind.PLANAR}


@dataclass
class JointRecord:
    """One tree edge, oriented from its outboard body toward the world."""

    index: object  # pydrake JointIndex
    name: str
    type_name: str
    kind: JointKind
    position_start: int
    num_positions: int
    X_PF: object  # RigidTransform of the frame on the parent in the parent body
    X_CM: object  # RigidTransform of the frame on the child in the child body
    fixed_hop: float = 0.0
    p_CM_norm: float = 0.0
    screw_pitch: float = 0.0
    translation_offsets: tuple = ()
    translation_offsets_known: bool = False
    coord_rules: tuple = ()
    inboard: int = -1
    outboard: int = -1
    subtree: np.ndarray = field(default=None)


@dataclass
class MotionBoundTable:
    """lambda(j, p) for every pair, dense: row p, column j (zero for j outside J(p)).

    `in_J[p, j]` marks the coordinates that survive the carve-out, `slack[p]` is the
    carve-out residual sigma_p, charged unconditionally by every motion bound."""

    lam: np.ndarray  # (num_pairs, num_positions)
    in_J: np.ndarray  # (num_pairs, num_positions), bool
    slack: np.ndarray  # (num_pairs,)

    @property
    def num_pairs(self) -> int:
        return self.lam.shape[0]

    def pair_is_static(self, p: int) -> bool:
        """True iff no coordinate the trajectory moves changes the pair's relative pose."""
        return not self.in_J[p].any()

    def entries(self, p: int) -> list:
        """(coordinate, lambda) for one pair, in increasing coordinate order."""
        return [(int(j), float(self.lam[p, j])) for j in np.flatnonzero(self.in_J[p])]

    def __post_init__(self):
        self._used = np.flatnonzero(self.in_J.any(axis=0))
        self._lam_used = np.ascontiguousarray(self.lam[:, self._used])

    def travel_bound(self, pairs: np.ndarray, w: np.ndarray, nodes: np.ndarray) -> np.ndarray:
        """sum_j lambda(j, p) w_j for entry k = (nodes[k], pairs[k]), w of shape (N, n);
        accumulated in coordinate order, as the fork's sparse dot product is (a cumulative sum
        is strictly sequential)."""
        if len(self._used) == 0:
            return np.zeros(len(pairs))
        return np.cumsum(self._lam_used[pairs] * w[:, self._used][nodes], axis=1)[:, -1]

    def motion_bound(self, p: int, w: np.ndarray) -> float:
        """Delta_p = sigma_p + sum_j lambda(j, p) w_j for one pair."""
        w = np.asarray(w, float)[None]
        return float(self.slack[p] + self.travel_bound(np.array([p]), w, np.array([0]))[0])


class KinematicsEngine:
    """Construction-time analysis of a plant: joint classification, chain hops, bounding
    spheres per geometry, subtree tables for J(p), and the per-call lambda table."""

    def __init__(self, model):
        self.model = model
        self.plant = plant = model.plant()
        if not plant.is_finalized():
            raise RuntimeError(
                "KinematicsEngine: requires a finalized MultibodyPlant; call Finalize() before "
                "building the checker.")
        self.num_positions = plant.num_positions()
        self.num_bodies = plant.num_bodies()
        self._build_topology()
        self._build_geometry()
        self._check_halfspace_rule()
        self._tables: dict = {}

    # ------------------------------------------------------------------ topology

    def _build_topology(self) -> None:
        plant = self.plant
        self.joints: list[JointRecord] = []
        for ji in plant.GetJointIndices():
            joint = plant.get_joint(ji)
            rec = _classify_joint(joint)
            try:
                rec.X_PF = joint.frame_on_parent().GetFixedPoseInBodyFrame()
                rec.X_CM = joint.frame_on_child().GetFixedPoseInBodyFrame()
            except Exception as e:  # noqa: BLE001 -- Drake raises RuntimeError subclasses.
                raise RuntimeError(
                    f"KinematicsEngine: joint '{rec.name}' ({rec.type_name}) is mounted on a frame "
                    "whose pose in its body is not fixed, so its chain contribution to the reach "
                    "bound cannot be computed at construction time. Mount joints on body frames "
                    f"or FixedOffsetFrames. Underlying error: {e}") from e
            rec.p_CM_norm = _norm(rec.X_CM.translation())
            rec.fixed_hop = rec.p_CM_norm + _norm(rec.X_PF.translation())
            if rec.kind is JointKind.WELD:
                rec.fixed_hop += _norm(joint.X_FM().translation())
            self.joints.append(rec)

        # Orient the joint graph into the world-rooted tree by a breadth-first walk.
        nb = self.num_bodies
        incident = [[] for _ in range(nb)]
        for k, rec in enumerate(self.joints):
            joint = plant.get_joint(rec.index)
            incident[int(joint.parent_body().index())].append(k)
            incident[int(joint.child_body().index())].append(k)
        self.inboard_joint = [-1] * nb
        visited = [False] * nb
        world = int(plant.world_body().index())
        visited[world] = True
        self.body_order = [world]
        queue = deque([world])
        while queue:
            b = queue.popleft()
            for k in incident[b]:
                joint = plant.get_joint(self.joints[k].index)
                parent, child = int(joint.parent_body().index()), int(joint.child_body().index())
                other = child if parent == b else parent
                if visited[other]:
                    continue
                visited[other] = True
                self.inboard_joint[other] = k
                self.joints[k].inboard = b
                self.joints[k].outboard = other
                self.body_order.append(other)
                queue.append(other)
        for b in range(nb):
            if not visited[b]:
                raise RuntimeError(
                    f"KinematicsEngine: body '{plant.get_body(_body_index(b)).name()}' is not "
                    "connected to the world through the plant's joints; the kinematics module "
                    "requires the single world-rooted tree a finalized MultibodyPlant provides.")
        for rec in self.joints:
            if rec.outboard < 0:
                raise RuntimeError(
                    f"KinematicsEngine: joint '{rec.name}' ({rec.type_name}) closes a kinematic "
                    "loop (both of its bodies are already reachable from the world without it). "
                    "The motion bound is defined over a single world-rooted tree, so the "
                    "loop-closing joint must be removed; express the constraint it carried with "
                    "a MultibodyPlant constraint instead.")

        tree_subtree = np.zeros((len(self.joints), nb), dtype=bool)
        for b in range(nb):
            k = self.inboard_joint[b]
            while k >= 0:
                tree_subtree[k, b] = True
                k = self.inboard_joint[self.joints[k].inboard]

        self.positioned_order = []
        for k, rec in enumerate(self.joints):
            joint = plant.get_joint(rec.index)
            if joint.num_velocities() == 0:
                continue
            if rec.outboard != int(joint.child_body().index()):
                raise RuntimeError(
                    f"KinematicsEngine: joint '{rec.name}' ({rec.type_name}) is reversed: its "
                    f"declared parent body '{joint.parent_body().name()}' is outboard of its "
                    f"declared child body '{joint.child_body().name()}' in the multibody tree. "
                    "The frame that stays fixed under the joint's motion is then on the outboard "
                    "side, which the reach chain does not model. Re-declare the joint with the "
                    "inboard body as its parent.")
            subtree = np.zeros(nb, dtype=bool)
            for b in plant.GetBodiesKinematicallyAffectedBy([rec.index]):
                subtree[int(b)] = True
            if not np.array_equal(subtree, tree_subtree[k]):
                raise RuntimeError(
                    f"KinematicsEngine: the plant's kinematically-affected set for joint "
                    f"'{rec.name}' ({rec.type_name}) disagrees with the world-rooted tree walk "
                    "over the same joints, so which side of a pair is distal to this joint is "
                    "ambiguous. This model's topology is not supported.")
            rec.subtree = subtree
            self.positioned_order.append(k)
        self.positioned_order.sort(key=lambda k: self.joints[k].position_start)
        # Joint-by-coordinate subtree matrix: S[j, b] is True iff coordinate j moves body b.
        self.coordinate_subtree = np.zeros((self.num_positions, nb), dtype=bool)
        self.coordinate_joint = np.full(self.num_positions, -1, dtype=int)
        for k in self.positioned_order:
            rec = self.joints[k]
            for c in range(rec.position_start, rec.position_start + rec.num_positions):
                self.coordinate_subtree[c] = rec.subtree
                self.coordinate_joint[c] = k
        assert (self.coordinate_joint >= 0).all()

    # ------------------------------------------------------------------ geometry

    def _build_geometry(self) -> None:
        plant = self.plant
        inspector = self.model.scene_graph().model_inspector()
        nb = self.num_bodies
        self.body_radius = np.zeros(nb)
        self.body_has_halfspace = np.zeros(nb, dtype=bool)
        self.body_halfspace_name = [""] * nb
        self.geometry_spheres: dict = {}
        for b in range(nb):
            frame_id = plant.GetBodyFrameIdIfExists(_body_index(b))
            if frame_id is None:
                continue
            for gid in inspector.GetGeometries(frame_id, Role.kProximity):
                shape = inspector.GetShape(gid)
                if classify(shape) is ShapeClass.HALFSPACE:
                    self.body_has_halfspace[b] = True
                    if not self.body_halfspace_name[b]:
                        self.body_halfspace_name[b] = inspector.GetName(gid)
                    continue
                try:
                    sphere = compute_bounding_sphere(shape, inspector.GetPoseInFrame(gid))
                except RuntimeError as e:
                    raise RuntimeError(
                        f"KinematicsEngine: proximity geometry '{inspector.GetName(gid)}' on body "
                        f"'{plant.get_body(_body_index(b)).name()}' cannot be bounded. {e}") from e
                self.body_radius[b] = max(self.body_radius[b], _norm(sphere.center_L) + sphere.radius)
                self.geometry_spheres[gid] = sphere

    def _check_halfspace_rule(self) -> None:
        plant = self.plant
        inspector = self.model.scene_graph().model_inspector()
        for ga, gb in _sorted_candidates(inspector):
            a_half = classify(inspector.GetShape(ga)) is ShapeClass.HALFSPACE
            b_half = classify(inspector.GetShape(gb)) is ShapeClass.HALFSPACE
            if not (a_half or b_half):
                continue
            ia = int(plant.GetBodyFromFrameId(inspector.GetFrameId(ga)).index())
            ib = int(plant.GetBodyFromFrameId(inspector.GetFrameId(gb)).index())
            for k in self.positioned_order:
                rec = self.joints[k]
                in_a, in_b = rec.subtree[ia], rec.subtree[ib]
                if in_a == in_b:
                    continue
                if not (a_half if in_a else b_half) or rec.kind not in _ROTATIONAL:
                    continue
                offender, partner = (ga, gb) if in_a else (gb, ga)
                raise RuntimeError(
                    f"KinematicsEngine: HalfSpace geometry '{inspector.GetName(offender)}' (body "
                    f"'{plant.get_body(_body_index(ia if in_a else ib)).name()}') rotates relative "
                    f"to its unfiltered partner geometry '{inspector.GetName(partner)}' (body "
                    f"'{plant.get_body(_body_index(ib if in_a else ia)).name()}') through joint "
                    f"'{rec.name}' ({rec.type_name}). A half space has unbounded reach, so no "
                    "finite motion bound λ exists for that pair. Fix the model by anchoring the "
                    "half space, filtering the pair, or replacing the half space with a large Box.")

    def geometry_sphere(self, gid) -> BoundingSphere:
        if gid not in self.geometry_spheres:
            raise RuntimeError(
                f"KinematicsEngine: geometry {gid} has no bounding sphere; it is either not a "
                "proximity geometry of this model or it is a HalfSpace.")
        return self.geometry_spheres[gid]

    def coordinates_affecting_pair(self, body_a: int, body_b: int) -> list:
        """J(p) before any carve-out, ascending."""
        S = self.coordinate_subtree
        return [int(c) for c in np.flatnonzero(S[:, int(body_a)] != S[:, int(body_b)])]

    # ------------------------------------------------------------------ lambda table

    def compute_motion_bound_table(self, lower, upper, constant, body_a, body_b) -> MotionBoundTable:
        """The lambda table over the control box [lower, upper] for the pairs (body_a[p], body_b[p]),
        with the coordinates flagged `constant` carved out of J(p) and charged to sigma_p."""
        n = self.num_positions
        lower, upper = np.asarray(lower, float), np.asarray(upper, float)
        constant = np.asarray(constant, bool)
        if lower.shape != (n,) or upper.shape != (n,) or constant.shape != (n,):
            raise RuntimeError(
                f"KinematicsEngine: control-box size mismatch: got lower={lower.size}, "
                f"upper={upper.size}, constant_coordinates={constant.size} for a plant with {n} "
                "positions.")
        for c in range(n):
            if not (np.isfinite(lower[c]) and np.isfinite(upper[c])) or lower[c] > upper[c]:
                raise RuntimeError(
                    f"KinematicsEngine: the trajectory's global control box is invalid at "
                    f"coordinate {c}: [{lower[c]}, {upper[c]}].")

        def abs_max(c):
            return max(abs(lower[c]), abs(upper[c]))

        box_hop = np.zeros(len(self.joints))
        quat_min_norm = np.zeros(len(self.joints))
        for k, rec in enumerate(self.joints):
            ps = rec.position_start
            if rec.kind is JointKind.PRISMATIC:
                box_hop[k] = abs_max(ps)
            elif rec.kind is JointKind.PLANAR:
                box_hop[k] = math.hypot(abs_max(ps), abs_max(ps + 1))
            elif rec.kind is JointKind.SCREW:
                box_hop[k] = abs_max(ps) * abs(rec.screw_pitch) / (2 * math.pi)
            elif rec.kind is JointKind.UNSUPPORTED:
                for c in range(ps, ps + rec.num_positions):
                    if not constant[c]:
                        raise RuntimeError(
                            f"KinematicsEngine: this trajectory moves coordinate {c} of joint "
                            f"'{rec.name}', whose type '{rec.type_name}' is not supported. "
                            "Quaternion coordinates are not a vector space, so Bézier "
                            "interpolation of their components has no rotation-space meaning and "
                            "the convex-hull motion bound does not apply. Supported joint types "
                            "are revolute, prismatic, planar, screw and weld; a floating base "
                            "whose pose is *constant* along the trajectory is accepted via the "
                            "constant-coordinate carve-out.")
                if not rec.translation_offsets_known:
                    raise RuntimeError(
                        f"KinematicsEngine: joint '{rec.name}' has type '{rec.type_name}', which "
                        "this library does not know how to bound even when held constant. "
                        "Supported joint types are revolute, prismatic, planar, screw and weld.")
                box_hop[k] = math.sqrt(sum(abs_max(ps + o) ** 2 for o in rec.translation_offsets))
            if CoordRule.QUATERNION in rec.coord_rules:
                total = 0.0
                for off, rule in enumerate(rec.coord_rules):
                    if rule is CoordRule.QUATERNION:
                        c = ps + off
                        closest = 0.0 if (lower[c] <= 0.0 <= upper[c]) else min(abs(lower[c]), abs(upper[c]))
                        total += closest * closest
                quat_min_norm[k] = math.sqrt(total)

        body_a = np.asarray(body_a, dtype=int)
        body_b = np.asarray(body_b, dtype=int)
        # The table depends on the box only through these; a planner checking many curves of
        # one robot (all-revolute, nothing carved) reuses one table.
        key = (box_hop.tobytes(), quat_min_norm.tobytes(), constant.tobytes(),
               (upper - lower)[constant].tobytes(), body_a.tobytes(), body_b.tobytes())
        cached = self._tables.get(key)
        if cached is not None:
            return cached
        table = self._assemble(box_hop, quat_min_norm, lower, upper, constant, body_a, body_b)
        if len(self._tables) >= 16:
            self._tables.pop(next(iter(self._tables)))
        self._tables[key] = table
        return table

    def _assemble(self, box_hop, quat_min_norm, lower, upper, constant, body_a, body_b) -> MotionBoundTable:
        n = self.num_positions
        P = len(body_a)
        if P and (min(body_a.min(), body_b.min()) < 0 or max(body_a.max(), body_b.max()) >= self.num_bodies):
            raise RuntimeError(
                f"KinematicsEngine: pair references body indices outside the plant's "
                f"{self.num_bodies} bodies.")
        reach = self._reach_table(box_hop)  # reach[k, body]; NaN where body is not below k.
        lam = np.zeros((P, n))
        in_J = np.zeros((P, n), dtype=bool)
        slack = np.zeros(P)
        S = self.coordinate_subtree
        errors = []  # (pair, coordinate, message): the fork raises on the first in pair order.
        for c in range(n):
            moved = S[c, body_a] != S[c, body_b]
            if not moved.any():
                continue
            k = int(self.coordinate_joint[c])
            rec = self.joints[k]
            distal = np.where(S[c, body_a], body_a, body_b)
            r = reach[k, distal]
            hs = self.body_has_halfspace[distal] & moved
            if constant[c]:
                span = upper[c] - lower[c]
                if span == 0.0:
                    continue
                rule = rec.coord_rules[c - rec.position_start]
                if rule is CoordRule.TRANSLATION:
                    lam_tilde = np.ones(P)
                else:
                    for p in np.flatnonzero(hs)[:1]:
                        d = int(distal[p])
                        errors.append((int(p), c, (
                            f"KinematicsEngine: HalfSpace geometry '{self.body_halfspace_name[d]}' "
                            f"on body '{self.plant.get_body(_body_index(d)).name()}' is the distal "
                            f"side of coordinate {c} of joint '{rec.name}' ({rec.type_name}), which "
                            "rotates it, and this trajectory holds that coordinate constant only to "
                            f"within a tolerance: its control-point range is {span}, not 0. A half "
                            "space has unbounded reach, so the residual motion of a rotational "
                            "coordinate across it cannot be bounded by any finite λ. A half space "
                            "may only sit across a rotational coordinate that is EXACTLY constant. "
                            "Fix the trajectory so that coordinate's control points are identical, "
                            "anchor the half space, filter the pair, or replace the half space with "
                            "a large Box.")))
                    if rule is CoordRule.ROTATION:
                        lam_tilde = r
                    elif rule is CoordRule.SCREW:
                        lam_tilde = r + abs(rec.screw_pitch) / (2 * math.pi)
                    else:
                        m = quat_min_norm[k]
                        if not m > 0.0:
                            errors.append((int(np.flatnonzero(moved)[0]), c, (
                                f"KinematicsEngine: the trajectory's control box for the quaternion "
                                f"coordinates of joint '{rec.name}' ({rec.type_name}) contains the "
                                "zero quaternion, whose normalized rotation is undefined, so the "
                                "residual motion of its carved-out coordinates cannot be bounded. "
                                "Quaternion control points must be unit quaternions.")))
                            continue
                        lam_tilde = 2.0 * r / m
                slack[moved] += lam_tilde[moved] * span
                continue
            if rec.kind is JointKind.PRISMATIC or (rec.kind is JointKind.PLANAR and c != rec.position_start + 2):
                value = np.ones(P)
            else:
                if rec.kind not in _ROTATIONAL:
                    raise RuntimeError(
                        f"KinematicsEngine: internal error: joint '{rec.name}' ({rec.type_name}) "
                        "reached the λ assembly with an unsupported kind.")
                for p in np.flatnonzero(hs)[:1]:
                    d = int(distal[p])
                    errors.append((int(p), c, (
                        f"KinematicsEngine: HalfSpace geometry '{self.body_halfspace_name[d]}' on "
                        f"body '{self.plant.get_body(_body_index(d)).name()}' is the distal side of "
                        f"joint '{rec.name}' ({rec.type_name}), which rotates it. A half space has "
                        "unbounded reach, so no finite λ exists.")))
                value = r + abs(rec.screw_pitch) / (2 * math.pi) if rec.kind is JointKind.SCREW else r
            lam[moved, c] = value[moved]
            in_J[moved, c] = True
        if errors:
            raise RuntimeError(min(errors)[2])
        assert np.isfinite(lam).all() and np.isfinite(slack).all()
        return MotionBoundTable(lam, in_J, slack)

    def _reach_table(self, box_hop: np.ndarray) -> np.ndarray:
        """reach[k, b] = r(k, b) for every joint k above body b, accumulated from the body up
        exactly as the fork's chain walk does."""
        reach = np.full((len(self.joints), self.num_bodies), np.nan)
        for b in range(self.num_bodies):
            r = self.body_radius[b]
            k = self.inboard_joint[b]
            while k >= 0:
                rec = self.joints[k]
                reach[k, b] = r + rec.p_CM_norm
                r += rec.fixed_hop + box_hop[k]
                k = self.inboard_joint[rec.inboard]
        return reach


def _classify_joint(joint) -> JointRecord:
    T, R, Q = CoordRule.TRANSLATION, CoordRule.ROTATION, CoordRule.QUATERNION
    type_name = joint.type_name()
    rec = JointRecord(index=joint.index(), name=joint.name(), type_name=type_name,
                      kind=JointKind.UNSUPPORTED,
                      position_start=joint.position_start() if joint.num_positions() > 0 else 0,
                      num_positions=joint.num_positions(), X_PF=None, X_CM=None)
    known = True
    if type_name == "weld":
        rec.kind = JointKind.WELD
    elif type_name == "revolute":
        rec.kind, rec.coord_rules = JointKind.REVOLUTE, (R,)
    elif type_name == "prismatic":
        rec.kind, rec.coord_rules = JointKind.PRISMATIC, (T,)
    elif type_name == "planar":
        rec.kind, rec.coord_rules = JointKind.PLANAR, (T, T, R)
    elif type_name == "screw":
        rec.kind, rec.coord_rules = JointKind.SCREW, (CoordRule.SCREW,)
        rec.screw_pitch = float(joint.screw_pitch())
    elif type_name == "quaternion_floating":
        rec.translation_offsets = (4, 5, 6)
        rec.coord_rules = (Q, Q, Q, Q, T, T, T)
    elif type_name == "rpy_floating":
        rec.translation_offsets = (3, 4, 5)
        rec.coord_rules = (R, R, R, T, T, T)
    elif type_name in ("ball_rpy", "universal"):
        rec.coord_rules = (R,) * rec.num_positions
    else:
        known = False
    rec.translation_offsets_known = known
    return rec


def _sorted_candidates(inspector) -> list:
    """SceneGraph's unfiltered proximity pairs, each (smaller id, larger id), sorted by id value:
    the order std::set gives the fork."""
    pairs = []
    for ga, gb in inspector.GetCollisionCandidates():
        if gb.get_value() < ga.get_value():
            ga, gb = gb, ga
        pairs.append((ga, gb))
    pairs.sort(key=lambda p: (p[0].get_value(), p[1].get_value()))
    return pairs


def _body_index(b: int):
    from pydrake.multibody.tree import BodyIndex

    return BodyIndex(int(b))


def _norm(v) -> float:
    x, y, z = (float(c) for c in v)
    return math.sqrt(x * x + y * y + z * z)
