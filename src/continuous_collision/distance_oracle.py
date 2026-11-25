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
        self._groups = [(int(a), g, np.vstack((self.slot_center[g].T, np.ones(len(g)))))
                        for a in np.unique(self.slot_body) if a != 0
                        for g in [np.flatnonzero(self.slot_body == a)]]
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

    def centers(self, X: list) -> np.ndarray:
        """World centers of every slot's bounding sphere, shape (N, G, 3), from the anchors'
        homogeneous poses (BatchKinematics.anchor_transforms)."""
        N = X[0].shape[0]
        C = np.empty((N, len(self.slot_body), 3))
        C[:, self._fixed] = self.slot_center[self._fixed]
        for a, g, local in self._groups:
            C[:, g] = np.swapaxes((X[a][:, :3, :].reshape(-1, 4) @ local).reshape(N, 3, -1), 1, 2)
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
