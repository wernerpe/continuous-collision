"""The distance oracle: the pair set, the capability probe, tau_p, and signed distances.

Design note: stock pydrake exposes no batched per-geometry-pair distance query, and every
query through a QueryObject first pays a full pose update of the scene. So the oracle splits
pairs by route, decided once at construction: pairs with an exact closed form -- a half space
against any convex shape (the fork's analytic fallback) and two spheres -- are evaluated for a
whole round of configurations at once in numpy from batched forward kinematics, and only the
remaining pairs go through Drake's narrowphase, one configuration at a time. The pair set is
SceneGraph's own unfiltered candidate set, snapshotted at construction, exactly as in the fork.
"""

from __future__ import annotations

import enum
import queue
from dataclasses import dataclass

import numpy as np

from ._numerics import QUERY_TOLERANCE, ShapeClass, classify, native_tolerance
from .kinematics import BatchKinematics
from .motion_bound import KinematicsEngine, _sorted_candidates, hull_vertices


class Route(enum.IntEnum):
    NATIVE = 0  # Drake's ComputeSignedDistancePairClosestPoints.
    HALFSPACE_A = 1  # Analytic support-function distance; geometry a is the half space.
    HALFSPACE_B = 2  # Same, geometry b is the half space.
    SPHERES = 3  # Two spheres: |c_a - c_b| - r_a - r_b, exact on both branches.
    SPHERE_BOX = 4  # A sphere and a box: point-to-box distance minus the radius, both branches.
    BOXES = 5  # Two boxes: separating-axis depth, or the nearest vertex-box / edge-edge distance.


_CLASS_NAME = {ShapeClass.SPHERE: "Sphere", ShapeClass.BOX: "Box", ShapeClass.CAPSULE: "Capsule",
               ShapeClass.CYLINDER: "Cylinder", ShapeClass.ELLIPSOID: "Ellipsoid",
               ShapeClass.CONVEX: "Convex", ShapeClass.MESH: "Mesh",
               ShapeClass.HALFSPACE: "HalfSpace", ShapeClass.UNSUPPORTED: "<unsupported>"}


@dataclass
class _Lease:
    root: object
    plant_context: object
    scene_graph_context: object


class DistanceOracle:
    """Unfiltered pairs of a RobotDiagram and their signed distances.

    Contract: for every pair, |phi_hat - phi| <= tau_p whenever phi >= -tau_p, and phi_hat is
    definitely negative when the shapes interpenetrate beyond tau_p."""

    def __init__(self, model, engine: KinematicsEngine, kinematics: BatchKinematics):
        self.model = model
        plant, scene_graph = model.plant(), model.scene_graph()
        inspector = scene_graph.model_inspector()
        self._plant, self._port = plant, scene_graph.get_query_output_port()

        deformables = inspector.GetAllDeformableGeometryIds()
        if deformables:
            names = "".join(f"\n  - {inspector.GetName(g)}" for g in deformables)
            raise RuntimeError(
                "DistanceOracle: deformable geometries are not supported (certified continuous "
                "collision checking assumes rigid bodies whose motion the plant's kinematics "
                f"describe). Offending geometries:{names}")

        def describe(g):
            return f"{inspector.GetName(g)} ({type(inspector.GetShape(g)).__name__})"

        self.geometry_a, self.geometry_b = [], []
        body_a, body_b, route, class_a, class_b = [], [], [], [], []
        combos = {}
        for ga, gb in _sorted_candidates(inspector):
            ba = plant.GetBodyFromFrameId(inspector.GetFrameId(ga))
            bb = plant.GetBodyFromFrameId(inspector.GetFrameId(gb))
            if ba is None or bb is None:
                raise RuntimeError(
                    f"DistanceOracle: collision geometry {describe(ga if ba is None else gb)} is "
                    "not attached to a MultibodyPlant body; the checker can only certify geometry "
                    "whose motion the plant describes.")
            ca, cb = classify(inspector.GetShape(ga)), classify(inspector.GetShape(gb))
            if ca is ShapeClass.HALFSPACE and cb is ShapeClass.HALFSPACE:
                raise RuntimeError(
                    "DistanceOracle: signed distance between two HalfSpace geometries is "
                    f"undefined, so the pair {describe(ga)} / {describe(gb)} cannot be certified. "
                    "Remove one halfspace, or filter the pair (CollisionFilterManager / a "
                    "collision filter group).")
            r = Route.NATIVE
            if ca is ShapeClass.HALFSPACE:
                r = Route.HALFSPACE_A
            elif cb is ShapeClass.HALFSPACE:
                r = Route.HALFSPACE_B
            elif ca is ShapeClass.SPHERE and cb is ShapeClass.SPHERE:
                r = Route.SPHERES
            elif {ca, cb} == {ShapeClass.SPHERE, ShapeClass.BOX}:
                r = Route.SPHERE_BOX
            elif ca is ShapeClass.BOX and cb is ShapeClass.BOX:
                r = Route.BOXES
            if r in (Route.HALFSPACE_A, Route.HALFSPACE_B):
                partner_class = cb if r is Route.HALFSPACE_A else ca
                if partner_class is ShapeClass.UNSUPPORTED:
                    partner = gb if r is Route.HALFSPACE_A else ga
                    raise RuntimeError(
                        "DistanceOracle: no closed-form support function for shape type "
                        f"'{type(inspector.GetShape(partner)).__name__}', so the halfspace pair "
                        f"{describe(ga)} / {describe(gb)} cannot be certified.")
            combos.setdefault(tuple(sorted((int(ca), int(cb)))), (r, ga, gb))
            self.geometry_a.append(ga)
            self.geometry_b.append(gb)
            body_a.append(int(ba.index()))
            body_b.append(int(bb.index()))
            route.append(int(r))
            class_a.append(int(ca))
            class_b.append(int(cb))
        P = len(self.geometry_a)
        self.body_a = np.array(body_a, dtype=int).reshape(P)
        self.body_b = np.array(body_b, dtype=int).reshape(P)
        self.route = np.array(route, dtype=int).reshape(P)
        self.class_a = np.array(class_a, dtype=int).reshape(P)
        self.class_b = np.array(class_b, dtype=int).reshape(P)
        # tau_p: Drake's documented accuracy for the shape classes; the closed forms are exact
        # but keep the baseline, and two spheres keep exactly what the native route would.
        self.tau = np.array([QUERY_TOLERANCE if r in (Route.HALFSPACE_A, Route.HALFSPACE_B)
                             else native_tolerance(ShapeClass(a), ShapeClass(b))
                             for r, a, b in zip(route, class_a, class_b)]).reshape(P)

        # Geometry slots: every geometry in some pair, with its body, pose in the body, class,
        # and bounding sphere (radius -1 for a half space, which has none).
        slot_of = {}
        self.slot_geometry, slot_body, slot_R, slot_p, slot_center, slot_radius, slot_class = \
            [], [], [], [], [], [], []
        self.slot_extent, self.slot_vertices = [], []

        def slot(g, body):
            if g in slot_of:
                return slot_of[g]
            shape = inspector.GetShape(g)
            X_BG = inspector.GetPoseInFrame(g)
            klass = classify(shape)
            slot_of[g] = len(self.slot_geometry)
            self.slot_geometry.append(g)
            # Everything is stored in the body's anchor frame (see BatchKinematics).
            R_AB, p_AB = kinematics.R_AB[body], kinematics.p_AB[body]
            slot_body.append(kinematics.anchor_of_body[body])
            slot_R.append(R_AB @ X_BG.rotation().matrix())
            slot_p.append(R_AB @ np.asarray(X_BG.translation(), float) + p_AB)
            slot_class.append(int(klass))
            if klass is ShapeClass.HALFSPACE:
                slot_center.append(np.zeros(3))
                slot_radius.append(-1.0)
            else:
                sphere = engine.geometry_sphere(g)
                slot_center.append(R_AB @ sphere.center_L + p_AB)
                slot_radius.append(sphere.radius)
            self.slot_extent.append(_extent(shape, klass))
            self.slot_vertices.append(hull_vertices(shape) if klass in (ShapeClass.CONVEX, ShapeClass.MESH) else None)
            return slot_of[g]

        self.slot_a = np.array([slot(g, b) for g, b in zip(self.geometry_a, body_a)], dtype=int).reshape(P)
        self.slot_b = np.array([slot(g, b) for g, b in zip(self.geometry_b, body_b)], dtype=int).reshape(P)
        G = len(self.slot_geometry)
        self.slot_body = np.array(slot_body, dtype=int).reshape(G)
        self.slot_R = np.array(slot_R).reshape(G, 3, 3)
        self.slot_p = np.array(slot_p).reshape(G, 3)
        self.slot_center = np.array(slot_center).reshape(G, 3)
        self.slot_radius = np.array(slot_radius).reshape(G)
        self.slot_class = np.array(slot_class, dtype=int).reshape(G)
        self.slot_extent = np.array(self.slot_extent).reshape(G, 3)
        # Bounding-sphere centers grouped by anchor for `centers`; the world anchor's are fixed.
        self._fixed = np.flatnonzero(self.slot_body == 0)
        # The others' centers in homogeneous anchor coordinates, padded to a common width, and
        # each moving slot's position in the flattened (anchor, column) product.
        self._moving = np.flatnonzero(self.slot_body != 0)
        self._anchors = np.unique(self.slot_body[self._moving])
        width = max([int((self.slot_body == a).sum()) for a in self._anchors], default=0)
        self._local = np.zeros((len(self._anchors), 4, width))
        self._column = np.zeros(len(self._moving), dtype=int)
        for i, a in enumerate(self._anchors):
            members = np.flatnonzero(self.slot_body[self._moving] == a)
            self._local[i, :3, :len(members)] = self.slot_center[self._moving[members]].T
            self._local[i, 3, :len(members)] = 1.0
            self._column[members] = i * width + np.arange(len(members))
        #: Pairs the sphere prefilter may test: neither side is a half space.
        self.has_spheres = (self.slot_radius[self.slot_a] >= 0) & (self.slot_radius[self.slot_b] >= 0)

        self._pool: queue.SimpleQueue = queue.SimpleQueue()
        lease = self.acquire()
        try:
            qo = self._query_object(lease, None)
            for (ka, kb), (r, ga, gb) in sorted(combos.items()):
                if r in (Route.HALFSPACE_A, Route.HALFSPACE_B):
                    continue
                try:
                    qo.ComputeSignedDistancePairClosestPoints(ga, gb)
                except Exception as e:  # noqa: BLE001
                    raise RuntimeError(
                        "DistanceOracle: this Drake build cannot compute signed distance for the "
                        f"shape combination ({_CLASS_NAME[ShapeClass(ka)]}, "
                        f"{_CLASS_NAME[ShapeClass(kb)]}); an offending pair is {describe(ga)} / "
                        f"{describe(gb)}. Filter the pair, or replace the geometry with a "
                        f"supported shape (Convex is always supported). Drake reported: {e}") from e
        finally:
            self.release(lease)

    @property
    def num_pairs(self) -> int:
        return len(self.geometry_a)

    # ------------------------------------------------------------------ contexts

    def acquire(self) -> _Lease:
        """A private context for one check; calls on several threads never share one."""
        try:
            return self._pool.get_nowait()
        except queue.Empty:
            root = self.model.CreateDefaultContext()
            return _Lease(root, self._plant.GetMyMutableContextFromRoot(root),
                          self.model.scene_graph().GetMyContextFromRoot(root))

    def release(self, lease: _Lease) -> None:
        self._pool.put(lease)

    def _query_object(self, lease: _Lease, q):
        if q is not None:
            self._plant.SetPositions(lease.plant_context, q)
        return self._port.Eval(lease.scene_graph_context)

    # ------------------------------------------------------------------ batched

    def geometry_poses(self, R_WB, p_WB, nodes, slots):
        """(R_WG, p_WG) for geometry `slots[k]` at node `nodes[k]`, from anchor poses."""
        R = R_WB[nodes, self.slot_body[slots]]
        p = p_WB[nodes, self.slot_body[slots]]
        return R @ self.slot_R[slots], np.einsum("kij,kj->ki", R, self.slot_p[slots]) + p

    def centers(self, X: np.ndarray) -> np.ndarray:
        """World centers of every slot's bounding sphere, shape (N, G, 3), from the anchors'
        homogeneous poses X (BatchKinematics.anchor_transforms). A stacked product, computed
        slice by slice, so a node's centers do not depend on its round."""
        N = X.shape[0]
        C = np.empty((N, len(self.slot_body), 3))
        C[:, self._fixed] = self.slot_center[self._fixed]
        if len(self._anchors):
            W = np.matmul(np.ascontiguousarray(X[:, self._anchors, :3]), self._local)  # (N, A', 3, w)
            C[:, self._moving] = np.swapaxes(W, 2, 3).reshape(N, -1, 3)[:, self._column]
        return C

    def halfspace_distance(self, R_WB, p_WB, nodes, pairs, want_points=False):
        """Exact signed distance of half-space pairs at the batched poses (see `_halfspace`)."""
        a_half = self.route[pairs] == Route.HALFSPACE_A
        h_slot = np.where(a_half, self.slot_a[pairs], self.slot_b[pairs])
        c_slot = np.where(a_half, self.slot_b[pairs], self.slot_a[pairs])
        R_H, p_H = self.geometry_poses(R_WB, p_WB, nodes, h_slot)
        R_C, c = self.geometry_poses(R_WB, p_WB, nodes, c_slot)
        return self._halfspace(a_half, c_slot, R_H, p_H, R_C, c, want_points)

    def _halfspace(self, a_half, c_slot, R_H, p_H, R_C, c, want_points):
        """phi = n.(x* - p0), x* the partner's support point in direction -n: the signed
        distance on both branches, with Drake's penetration-depth semantics."""
        n = R_H[:, :, 2]
        x = _support_points(self, c_slot, R_C, c, -n)
        phi = np.einsum("ki,ki->k", n, x - p_H)
        if not want_points:
            return phi
        plane = x - phi[:, None] * n
        pa = np.where(a_half[:, None], plane, x)
        pb = np.where(a_half[:, None], x, plane)
        return phi, pa, pb

    def sphere_box_distance(self, R_WB, C, nodes, pairs) -> np.ndarray:
        """Exact signed distance of sphere-box pairs: with p the sphere center in the box frame
        and h the half sizes, |max(|p| - h, 0)| - r outside the box, and -(min(h - |p|)) - r
        inside, which is the penetration depth of the minimal separating translation."""
        a_sphere = self.class_a[pairs] == ShapeClass.SPHERE
        s_slot = np.where(a_sphere, self.slot_a[pairs], self.slot_b[pairs])
        b_slot = np.where(a_sphere, self.slot_b[pairs], self.slot_a[pairs])
        # Both bounding spheres are centered on the geometry origins, so C holds the centers.
        v = C[nodes, s_slot] - C[nodes, b_slot]
        v = np.einsum("kji,kj->ki", R_WB[nodes, self.slot_body[b_slot]], v)
        local = np.abs(np.einsum("kji,kj->ki", self.slot_R[b_slot], v))
        h = self.slot_extent[b_slot]
        outside = np.maximum(local - h, 0.0)
        out = np.sqrt(outside[:, 0] ** 2 + outside[:, 1] ** 2 + outside[:, 2] ** 2)
        inside = -np.min(h - local, axis=1)
        return np.where((local > h).any(axis=1), out, inside) - self.slot_extent[s_slot, 0]

    def _box_frames(self, R_WB, C, nodes, pairs):
        """B's rotation and center in A's frame, and both half sizes, for box-box entries."""
        sa, sb = self.slot_a[pairs], self.slot_b[pairs]
        R_A = R_WB[nodes, self.slot_body[sa]] @ self.slot_R[sa]
        R_B = R_WB[nodes, self.slot_body[sb]] @ self.slot_R[sb]
        R = np.swapaxes(R_A, 1, 2) @ R_B  # B's axes in A's frame, as columns.
        t = np.einsum("kji,kj->ki", R_A, C[nodes, sb] - C[nodes, sa])  # Bounding centers = box centers.
        return R, t, self.slot_extent[sa], self.slot_extent[sb]

    def box_box_lower_bound(self, R_WB, C, nodes, pairs) -> np.ndarray:
        """The largest gap over the 15 separating axes: a lower bound on the signed distance
        of two boxes on both branches, and exactly it when they interpenetrate."""
        return _separating_axis_gap(*self._box_frames(R_WB, C, nodes, pairs))

    def box_box_bounds(self, R_WB, C, nodes, pairs):
        """(G, V, frames) with G <= phi <= V for box-box pairs, both exact when the boxes
        interpenetrate or when G = V. Penetrating: the minimal separating translation lies along
        one of the 15 separating axes, so phi = G, the largest axis gap. Separated: the closest
        points are a vertex against the other box or two edges, so phi <= V, the smallest
        vertex-box distance, with equality unless an edge pair is closer."""
        frames = self._box_frames(R_WB, C, nodes, pairs)
        G = _separating_axis_gap(*frames)
        V = G.copy()
        apart = np.flatnonzero(G > 0.0)
        if len(apart):
            V[apart] = _vertex_distance(*(f[apart] for f in frames))
        return G, V, frames

    def box_box_distance(self, R_WB, C, nodes, pairs) -> np.ndarray:
        """Exact signed distance of box-box pairs (see `box_box_bounds`)."""
        return box_box_exact(*self.box_box_bounds(R_WB, C, nodes, pairs))

    def native_distances(self, lease: _Lease, Q, nodes, pairs) -> np.ndarray:
        """Drake's narrowphase, one configuration at a time; entries grouped by node."""
        phi = np.empty(len(pairs))
        if len(pairs) == 0:
            return phi
        breaks = np.flatnonzero(np.diff(nodes)) + 1
        starts = np.concatenate(([0], breaks))
        ends = np.concatenate((breaks, [len(pairs)]))
        ga, gb = self.geometry_a, self.geometry_b
        for s, e in zip(starts.tolist(), ends.tolist()):
            qo = self._query_object(lease, Q[nodes[s]])
            for k in range(s, e):
                p = pairs[k]
                phi[k] = qo.ComputeSignedDistancePairClosestPoints(ga[p], gb[p]).distance
        return phi

    # ------------------------------------------------------------------ one pair

    def signed_distance(self, lease: _Lease, q, p: int):
        """(phi, nearest_a_W, nearest_b_W) for pair p at configuration q, from Drake's poses."""
        qo = self._query_object(lease, q)
        p = int(p)
        if self.route[p] in (Route.HALFSPACE_A, Route.HALFSPACE_B):
            a_half = self.route[p] == Route.HALFSPACE_A
            h, cs = (self.slot_a[p], self.slot_b[p]) if a_half else (self.slot_b[p], self.slot_a[p])
            X_H = qo.GetPoseInWorld(self.slot_geometry[h])
            X_C = qo.GetPoseInWorld(self.slot_geometry[cs])
            phi, pa, pb = self._halfspace(
                np.array([a_half]), np.array([cs]), X_H.rotation().matrix()[None],
                np.asarray(X_H.translation(), float)[None], X_C.rotation().matrix()[None],
                np.asarray(X_C.translation(), float)[None], True)
            return float(phi[0]), pa[0], pb[0]
        a, b = self.geometry_a[p], self.geometry_b[p]
        result = qo.ComputeSignedDistancePairClosestPoints(a, b)
        if result.id_A == a and result.id_B == b:
            p_ACa, p_BCb = result.p_ACa, result.p_BCb
        elif result.id_A == b and result.id_B == a:
            p_ACa, p_BCb = result.p_BCb, result.p_ACa
        else:
            raise RuntimeError("DistanceOracle: Drake returned a signed distance result for a "
                               "different geometry pair than the one queried.")
        pa = qo.GetPoseInWorld(a).multiply(np.asarray(p_ACa, float))
        pb = qo.GetPoseInWorld(b).multiply(np.asarray(p_BCb, float))
        return float(result.distance), np.asarray(pa, float), np.asarray(pb, float)


def _extent(shape, klass) -> np.ndarray:
    """Per-class parameters of the support function: radius / half-length / half sizes."""
    if klass is ShapeClass.SPHERE:
        return np.array([shape.radius(), 0.0, 0.0])
    if klass is ShapeClass.BOX:
        return 0.5 * np.array([shape.width(), shape.depth(), shape.height()])
    if klass in (ShapeClass.CAPSULE, ShapeClass.CYLINDER):
        return np.array([shape.radius(), 0.5 * shape.length(), 0.0])
    if klass is ShapeClass.ELLIPSOID:
        return np.array([shape.a(), shape.b(), shape.c()])
    return np.zeros(3)


def _support_points(oracle: DistanceOracle, slots, R, c, d) -> np.ndarray:
    """argmax over the shape of d.x for unit directions d (K, 3), per shape class."""
    x = np.empty_like(c)
    klass = oracle.slot_class[slots]
    ext = oracle.slot_extent[slots]
    for k in np.unique(klass):
        m = klass == k
        Rm, cm, dm, e = R[m], c[m], d[m], ext[m]
        if k == ShapeClass.SPHERE:
            x[m] = cm + e[:, :1] * dm
        elif k == ShapeClass.BOX:
            d_C = np.einsum("kji,kj->ki", Rm, dm)
            corner = np.where(d_C >= 0.0, e, -e)
            x[m] = cm + np.einsum("kij,kj->ki", Rm, corner)
        elif k == ShapeClass.CAPSULE:
            axis = Rm[:, :, 2]
            s = np.where(np.einsum("ki,ki->k", dm, axis) >= 0.0, 1.0, -1.0)
            x[m] = cm + (s * e[:, 1])[:, None] * axis + e[:, :1] * dm
        elif k == ShapeClass.CYLINDER:
            axis = Rm[:, :, 2]
            along = np.einsum("ki,ki->k", dm, axis)
            s = np.where(along >= 0.0, 1.0, -1.0)
            pt = cm + (s * e[:, 1])[:, None] * axis
            perp = dm - along[:, None] * axis
            norm = np.sqrt(np.einsum("ki,ki->k", perp, perp))
            big = norm > 1e-14
            pt[big] += (e[big, 0] / norm[big])[:, None] * perp[big]
            x[m] = pt
        elif k == ShapeClass.ELLIPSOID:
            M = Rm * e[:, None, :]
            Mt_d = np.einsum("kji,kj->ki", M, dm)
            norm = np.sqrt(np.einsum("ki,ki->k", Mt_d, Mt_d))
            safe = np.where(norm > 0.0, norm, 1.0)
            x[m] = cm + np.where((norm > 0.0)[:, None], np.einsum("kij,kj->ki", M, Mt_d) / safe[:, None], 0.0)
        elif k in (ShapeClass.CONVEX, ShapeClass.MESH):
            idx = np.flatnonzero(m)
            for i in idx:
                V = oracle.slot_vertices[slots[i]]
                d_C = R[i].T @ d[i]
                x[i] = c[i] + R[i] @ V[:, int(np.argmax(d_C @ V))]
        else:
            raise RuntimeError("DistanceOracle: internal error - no support function for this "
                               "shape class; the capability probe should have refused it.")
    return x


_SIGNS = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


_I1, _I2 = np.array([1, 2, 0]), np.array([2, 0, 1])


def _separating_axis_gap(R, t, h_A, h_B) -> np.ndarray:
    """max over the 15 OBB axes of |u.t| - r_A(u) - r_B(u) for unit u, in A's frame, with B at
    (R, t). Edge-pair axes e_i x b_j use the closed forms of Gottschalk's OBB test; near-parallel
    edge pairs are skipped, their directions being covered by the face normals."""
    abs_R = np.abs(R)
    face_A = np.abs(t) - h_A - np.einsum("kij,kj->ki", abs_R, h_B)
    face_B = np.abs(np.einsum("ki,kij->kj", t, R)) - np.einsum("ki,kij->kj", h_A, abs_R) - h_B
    # Index grids for the 9 edge pairs (i, j): rows i1 = i+1, i2 = i+2 and columns j1, j2.
    i1, i2 = _I1[:, None], _I2[:, None]
    j1, j2 = _I1[None, :], _I2[None, :]
    i, j = np.arange(3)[:, None], np.arange(3)[None, :]
    R_i1j, R_i2j = R[:, i1, j], R[:, i2, j]
    dist = np.abs(t[:, i2] * R_i1j - t[:, i1] * R_i2j)
    r_A = h_A[:, i1] * np.abs(R_i2j) + h_A[:, i2] * np.abs(R_i1j)
    r_B = h_B[:, j1] * abs_R[:, i, j2] + h_B[:, j2] * abs_R[:, i, j1]
    norm = np.sqrt(R_i1j * R_i1j + R_i2j * R_i2j)
    edge = np.where(norm > 1e-9, (dist - r_A - r_B) / np.where(norm > 1e-9, norm, 1.0), -np.inf)
    return np.maximum(np.maximum(face_A.max(axis=1), face_B.max(axis=1)), edge.reshape(len(t), 9).max(axis=1))


def _point_box(P, h) -> np.ndarray:
    """Distance from points P (K, V, 3) to boxes of half sizes h (K, 3), zero inside."""
    d = np.maximum(np.abs(P) - h[:, None, :], 0.0)
    return np.sqrt((d * d).sum(axis=2))


def _box_edges(h, R=None, t=None):
    """The 12 edges of each box as (start (K, 12, 3), direction (K, 12, 3)), optionally mapped by
    x -> R x + t."""
    starts, dirs = [], []
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        for sj in (-1.0, 1.0):
            for sk in (-1.0, 1.0):
                p = np.zeros((len(h), 3))
                p[:, i], p[:, j], p[:, k] = -h[:, i], sj * h[:, j], sk * h[:, k]
                d = np.zeros((len(h), 3))
                d[:, i] = 2.0 * h[:, i]
                starts.append(p)
                dirs.append(d)
    P, D = np.stack(starts, axis=1), np.stack(dirs, axis=1)
    if R is not None:
        P = P @ np.swapaxes(R, 1, 2) + t[:, None, :]
        D = D @ np.swapaxes(R, 1, 2)
    return P, D


def _segment_distance(p1, d1, p2, d2) -> np.ndarray:
    """Distance between segments p1 + s d1 and p2 + t d2, s, t in [0, 1] (Ericson, Real-Time
    Collision Detection, 5.1.9), elementwise over leading axes."""
    r = p1 - p2
    a = (d1 * d1).sum(-1)
    e = (d2 * d2).sum(-1)
    f = (d2 * r).sum(-1)
    c = (d1 * r).sum(-1)
    b = (d1 * d2).sum(-1)
    denom = a * e - b * b
    s = np.where(denom > 1e-12 * a * e, np.clip((b * f - c * e) / np.where(denom > 0, denom, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / e
    low, high = t < 0.0, t > 1.0
    s = np.where(low, np.clip(-c / a, 0.0, 1.0), np.where(high, np.clip((b - c) / a, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    d = p1 + s[..., None] * d1 - p2 - t[..., None] * d2
    return np.sqrt((d * d).sum(-1))


def box_box_exact(G, V, frames) -> np.ndarray:
    """phi from the bounds G <= phi <= V of `box_box_bounds`: V, unless an edge pair is closer."""
    phi = V.copy()
    edges = np.flatnonzero(V - G > 1e-12)
    if len(edges):
        phi[edges] = np.minimum(V[edges], _edge_distance(*(f[edges] for f in frames)))
    return phi


def _vertex_distance(R, t, h_A, h_B) -> np.ndarray:
    """Smallest distance from a vertex of either box to the other box (A at the origin,
    axis-aligned; B at (R, t))."""
    V_B = (_SIGNS[None] * h_B[:, None, :]) @ np.swapaxes(R, 1, 2) + t[:, None, :]
    t_A = -np.einsum("kji,kj->ki", R, t)  # A's center in B's frame
    V_A = (_SIGNS[None] * h_A[:, None, :]) @ R + t_A[:, None, :]
    return np.minimum(_point_box(V_B, h_A).min(axis=1), _point_box(V_A, h_B).min(axis=1))


def _edge_distance(R, t, h_A, h_B) -> np.ndarray:
    """Smallest distance between an edge of A and an edge of B."""
    P_A, D_A = _box_edges(h_A)
    P_B, D_B = _box_edges(h_B, R, t)
    d = _segment_distance(P_A[:, :, None], D_A[:, :, None], P_B[:, None], D_B[:, None])
    return d.reshape(len(t), -1).min(axis=1)
