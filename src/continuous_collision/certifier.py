"""The adaptive certifier: Algorithm 1 of the paper, run as rounds over a work list.

Design note: the fork walks one node at a time on a shared LIFO stack across C++ threads. In
Python the per-node cost would be interpreter overhead, so the same search runs in rounds: a
round takes up to CHUNK pending nodes of one segment, splits them all at once, poses them all
with one batched forward-kinematics call, decides every (node, active pair) entry with array
arithmetic, and sends only the entries no closed form can decide to Drake's narrowphase.
Children are pushed back left of everything still pending, so the sweep runs left to right in
time and the earliest-violation bound prunes as in the fork. Every node's decisions depend
only on its own control points and inherited active set, so the verdict and the witness are
those of the fork's serial search; num_nodes matches whenever nothing is pruned.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ._numerics import NUMERICAL_SLACK, is_certified, is_definite_violation
from .bezier_path import PiecewiseBezierPath, split_at_half
from .distance_oracle import DistanceOracle, Route, box_box_exact
from .kinematics import BatchKinematics
from .motion_bound import MotionBoundTable

#: Largest number of nodes decided in one round.
CHUNK = 2048


@dataclass
class Witness:
    time: float
    q: np.ndarray
    pair: int
    distance: float


class _Sink:
    """The earliest violation and the earliest inconclusive witness seen so far."""

    def __init__(self):
        self.definite: Witness | None = None
        self.inconclusive: Witness | None = None

    @property
    def best_time(self) -> float:
        return self.definite.time if self.definite is not None else math.inf

    def add(self, kind: str, witness: Witness) -> None:
        current = getattr(self, kind)
        if current is None or witness.time < current.time:
            setattr(self, kind, witness)


@dataclass
class _Chunk:
    seg: np.ndarray  # (N,) segment index of each node
    s_lo: np.ndarray  # (N,)
    s_hi: np.ndarray  # (N,)
    cps: np.ndarray  # (N, n, m+1)
    node: np.ndarray  # (K,) entry -> node, nondecreasing
    pair: np.ndarray  # (K,) entry -> pair


class Certifier:
    """One check: the breakpoint pass, then the subdivision over every segment."""

    def __init__(self, oracle: DistanceOracle, kinematics: BatchKinematics, table: MotionBoundTable,
                 path: PiecewiseBezierPath, margin: float, resolution: float, lease):
        self.oracle, self.kin, self.table, self.path = oracle, kinematics, table, path
        self.margin, self.resolution, self.lease = margin, resolution, lease
        self.sink = _Sink()
        self.num_nodes = 0

    def run(self):
        """Returns (verdict name, Witness or None, num_nodes)."""
        if self.oracle.num_pairs > 0:
            self._breakpoints()
            moving = np.flatnonzero(self.table.in_J.any(axis=1))
            if len(moving):
                self._subdivide(moving)
        if self.sink.definite is not None:
            return "violation", self.sink.definite, self.num_nodes
        if self.sink.inconclusive is not None:
            return "inconclusive", self.sink.inconclusive, self.num_nodes
        return "certified", None, self.num_nodes

    # ------------------------------------------------------------------ shared pieces

    def _lower_bounds(self, C, nodes, pairs):
        """Free-sphere lower bound |c_A - c_B| - R_A - R_B (garbage where a side has no sphere)."""
        o = self.oracle
        G = C.shape[1]
        sa, sb = o.slot_a[pairs], o.slot_b[pairs]
        ia, ib = nodes * G + sa, nodes * G + sb
        flat = C.reshape(-1, 3).T
        dx, dy, dz = (flat[i][ia] - flat[i][ib] for i in range(3))
        dist = np.sqrt(dx * dx + dy * dy + dz * dz)
        return dist - o.slot_radius[sa] - o.slot_radius[sb]

    def _tighten(self, lower, X, C, nodes, pairs, settled):
        """Raises `lower` to the separating-axis bound on box-box entries `settled` rejects:
        a second, tighter free-ball lower bound where bounding spheres are poor (thin boxes)."""
        boxes = np.flatnonzero(~settled(lower) & (self.oracle.route[pairs] == Route.BOXES))
        if len(boxes) == 0:
            return lower
        lower = lower.copy()
        gap = self.oracle.box_box_lower_bound(X[:, :, :3, :3], C, nodes[boxes], pairs[boxes])
        lower[boxes] = np.maximum(lower[boxes], gap)
        return lower

    def _distances(self, Q, X, C, lower, nodes, pairs, tau, bound, at_floor):
        """phi_hat for each entry, by route; entries are grouped by node. Box-box entries get
        their exact distance only where the bounds G <= phi <= V could change a decision or a
        value is recorded; elsewhere phi_hat = V decides identically."""
        o = self.oracle
        phi = np.empty(len(pairs))
        route = o.route[pairs]
        spheres = route == Route.SPHERES
        phi[spheres] = lower[spheres]  # The bounding spheres are the spheres: exact.
        half = (route == Route.HALFSPACE_A) | (route == Route.HALFSPACE_B)
        box = route == Route.SPHERE_BOX
        boxes = route == Route.BOXES
        if half.any() or box.any() or boxes.any():
            R, p = X[:, :, :3, :3], X[:, :, :3, 3]
            if half.any():
                phi[half] = o.halfspace_distance(R, p, nodes[half], pairs[half])
            if box.any():
                phi[box] = o.sphere_box_distance(R, C, nodes[box], pairs[box])
            if boxes.any():
                m = self.margin
                G, V, frames = o.box_box_bounds(R, C, nodes[boxes], pairs[boxes])
                t, b, f = tau[boxes], bound[boxes], at_floor[boxes]
                lo_cert, hi_cert = is_certified(G, t, b, m), is_certified(V, t, b, m)
                lo_viol, hi_viol = is_definite_violation(G, t, m), is_definite_violation(V, t, m)
                exact = (lo_cert != hi_cert) | (lo_viol != hi_viol) | hi_viol | (~hi_cert & ~lo_viol & f)
                values = V.copy()
                k = np.flatnonzero(exact)
                if len(k):
                    values[k] = box_box_exact(G[k], V[k], tuple(x[k] for x in frames))
                phi[boxes] = values
        native = route == Route.NATIVE
        if native.any():
            phi[native] = o.native_distances(self.lease, Q, nodes[native], pairs[native])
        return phi

    def _record(self, kind, mask, times, Q, nodes, pairs, phi):
        if not mask.any():
            return
        idx = np.flatnonzero(mask)
        i = idx[int(np.argmin(times[idx]))]  # First entry among equal times wins.
        self.sink.add(kind, Witness(float(times[i]), Q[nodes[i]].copy(), int(pairs[i]), float(phi[i])))

    # ------------------------------------------------------------------ breakpoints

    def _breakpoints(self):
        """q(t0), every junction, and q(tf): the configurations node midpoints only approach,
        decided in one batch. Static pairs (empty J(p)) are settled once and for all at q(t0)."""
        segs = self.path.segments
        Q = np.array([s.control_points[:, 0] for s in segs] + [segs[-1].control_points[:, -1]])
        times = np.array([s.t_start for s in segs] + [segs[-1].t_end])
        X = self.kin.anchor_transforms(Q)
        C = self.oracle.centers(X)
        o, m = self.oracle, self.margin
        static = ~self.table.in_J.any(axis=1)
        moving = np.flatnonzero(~static)
        # Entries in the fork's order: breakpoint by breakpoint, pairs ascending; static pairs
        # only at q(t0).
        nodes = np.concatenate([np.zeros(o.num_pairs, dtype=int)] +
                               [np.full(len(moving), k) for k in range(1, len(Q))])
        pairs = np.concatenate([np.arange(o.num_pairs)] + [moving] * (len(Q) - 1))
        is_static = static[pairs]
        tau, bound = o.tau[pairs], self.table.slack[pairs]
        lower = np.where(o.has_spheres[pairs], self._lower_bounds(C, nodes, pairs), -np.inf)

        def settled(lb):
            # A static pair's bound is its carve-out residual; a moving pair cannot be a
            # violation once the lower bound clears the margin.
            return np.where(is_static, is_certified(lb, tau, bound, m), lb >= m)

        ask = np.flatnonzero(~settled(self._tighten(lower, X, C, nodes, pairs, settled)))
        if len(ask) == 0:
            return
        a_nodes, a_pairs, a_static = nodes[ask], pairs[ask], is_static[ask]
        # Breakpoints test violations, plus certification for static pairs, whose gray outcome
        # is recorded at once (as a floor node's is).
        phi = self._distances(Q, X, C, lower[ask], a_nodes, a_pairs, tau[ask],
                              np.where(a_static, bound[ask], np.inf), a_static)
        viol = is_definite_violation(phi, tau[ask], m)
        t = times[a_nodes]
        self._record("definite", viol, t, Q, a_nodes, a_pairs, phi)
        gray = ~viol & a_static & ~is_certified(phi, tau[ask], bound[ask], m)
        self._record("inconclusive", gray, t, Q, a_nodes, a_pairs, phi)

    # ------------------------------------------------------------------ subdivision

    def _subdivide(self, moving):
        """All segments' trees, segments of one degree sharing rounds; LIFO over chunks with
        children pushed in time order, so the sweep runs left to right."""
        segs = self.path.segments
        self._t0 = np.array([s.t_start for s in segs])
        self._dt = np.array([s.t_end - s.t_start for s in segs])
        groups: dict = {}
        for k, seg in enumerate(segs):
            groups.setdefault(seg.control_points.shape[1], []).append(k)
        stack = []
        for ks in reversed(list(groups.values())):
            N = len(ks)
            stack.append(_Chunk(np.array(ks), np.zeros(N), np.ones(N),
                                np.stack([segs[k].control_points for k in ks]).astype(float),
                                np.repeat(np.arange(N), len(moving)), np.tile(moving, N)))
        while stack:
            children = self._round(stack.pop())
            if children is None:
                continue
            N = len(children.s_lo)
            bounds = list(range(0, N, CHUNK)) + [N]
            cuts = np.searchsorted(children.node, bounds)
            for i in reversed(range(len(bounds) - 1)):
                a, b, ea, eb = bounds[i], bounds[i + 1], cuts[i], cuts[i + 1]
                stack.append(_Chunk(children.seg[a:b], children.s_lo[a:b], children.s_hi[a:b],
                                    children.cps[a:b], children.node[ea:eb] - a,
                                    children.pair[ea:eb]))

    def _round(self, ch: _Chunk):
        # Branch and bound on time: a node starting at or after the earliest violation found so
        # far cannot hold an earlier one.
        keep = ~(self._t0[ch.seg] + ch.s_lo * self._dt[ch.seg] >= self.sink.best_time)
        if not keep.any():
            return None
        if not keep.all():
            new_index = np.cumsum(keep) - 1
            e = keep[ch.node]
            ch = _Chunk(ch.seg[keep], ch.s_lo[keep], ch.s_hi[keep], ch.cps[keep],
                        new_index[ch.node[e]], ch.pair[e])
        N = len(ch.s_lo)
        self.num_nodes += N
        o, table, m = self.oracle, self.table, self.margin
        nodes, pairs = ch.node, ch.pair

        left, right, mid = split_at_half(ch.cps)
        w = np.abs(ch.cps - mid[:, :, None]).max(axis=2)
        s_mid = 0.5 * (ch.s_lo + ch.s_hi)
        t_mid = self._t0[ch.seg] + s_mid * self._dt[ch.seg]
        backstop = ~((s_mid > ch.s_lo) & (s_mid < ch.s_hi))

        travel = table.travel_bound(pairs, w, nodes)
        bound = table.slack[pairs] + travel
        # The resolution floor is per pair and tests the travel term only: the carve-out
        # residual does not shrink with splitting.
        at_floor = backstop[nodes] | (travel <= self.resolution)
        tau = o.tau[pairs]

        X = self.kin.anchor_transforms(mid)
        C = o.centers(X)
        lower = self._lower_bounds(C, nodes, pairs)
        settled = o.has_spheres[pairs] & is_certified(
            self._tighten(lower, X, C, nodes, pairs, lambda lb: is_certified(lb, tau, bound, m)), tau, bound, m)
        ask = np.flatnonzero(~settled)
        if len(ask) == 0:
            return None
        a_nodes, a_pairs, a_tau, a_bound, a_floor = nodes[ask], pairs[ask], tau[ask], bound[ask], at_floor[ask]
        phi = self._distances(mid, X, C, lower[ask], a_nodes, a_pairs, a_tau, a_bound, a_floor)

        viol = is_definite_violation(phi, a_tau, m)
        cert = ~viol & is_certified(phi, a_tau, a_bound, m)
        times = t_mid[a_nodes]
        self._record("definite", viol, times, mid, a_nodes, a_pairs, phi)
        self._record("inconclusive", ~viol & ~cert & a_floor, times, mid, a_nodes, a_pairs, phi)
        # Violating pairs stay active off the floor, so the recursion refines the witness
        # toward the earliest violating time.
        survive = ~cert & ~a_floor
        if not survive.any():
            return None

        s_nodes, s_pairs = a_nodes[survive], a_pairs[survive]
        has = np.zeros(N, dtype=bool)
        has[s_nodes] = True
        parents = np.flatnonzero(has)
        rank = np.cumsum(has) - 1
        child = np.concatenate((2 * rank[s_nodes], 2 * rank[s_nodes] + 1))
        order = np.argsort(child, kind="stable")
        M = 2 * len(parents)
        s_lo = np.empty(M)
        s_hi = np.empty(M)
        s_lo[0::2], s_hi[0::2] = ch.s_lo[parents], s_mid[parents]
        s_lo[1::2], s_hi[1::2] = s_mid[parents], ch.s_hi[parents]
        cps = np.empty((M,) + ch.cps.shape[1:])
        cps[0::2], cps[1::2] = left[parents], right[parents]
        return _Chunk(np.repeat(ch.seg[parents], 2), s_lo, s_hi, cps, child[order],
                      np.concatenate((s_pairs, s_pairs))[order])


__all__ = ["Certifier", "Witness", "NUMERICAL_SLACK"]
