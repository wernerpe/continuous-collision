"""Trajectory normalization into an ordered list of Bezier segments.

Design note: the certifier rests on two Bezier facts only -- a segment stays inside the box
of its control points, and de Casteljau subdivision at 1/2 yields two exact children whose
control points are convex combinations of the parent's, with the curve's midpoint as the
apex. Every accepted trajectory type is therefore converted exactly, up front, into Bezier
segments (B-splines by knot insertion, polynomials by an exact change of basis), and the
recursion never sees anything else.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from pydrake.trajectories import (
    BezierCurve,
    BsplineTrajectory,
    CompositeTrajectory,
    PiecewisePolynomial,
)

from ._numerics import CONTINUITY_TOLERANCE, MAX_CONVERSION_DEGREE

PARAMETER_SLACK = 1e-12
TIME_CONTIGUITY_SLACK = 1e-9


@dataclass
class BezierSegment:
    """q(s) = sum_j B_{j,m}(s) P_j for s in [0, 1], spanning [t_start, t_end] in time."""

    t_start: float
    t_end: float
    control_points: np.ndarray  # (num_positions, m + 1); column j is P_j.

    def time_of(self, s):
        """Trajectory time of segment parameter `s` (scalar or array)."""
        return self.t_start + s * (self.t_end - self.t_start)


def split_at_half(cps: np.ndarray):
    """de Casteljau at 1/2 on control points of shape (..., n, m+1).

    Returns (left, right, mid): the two children (same shape) and the curve value at the
    midpoint, which is the apex of the triangle. Batched over leading axes."""
    m = cps.shape[-1] - 1
    right = np.array(cps, dtype=float, copy=True)
    left = np.empty_like(right)
    left[..., 0] = cps[..., 0]
    for r in range(1, m + 1):
        # In-place sweep upward in j is safe: column j+1 is read before it is written.
        right[..., : m - r + 1] = 0.5 * (right[..., : m - r + 1] + right[..., 1 : m - r + 2])
        left[..., r] = right[..., 0]
    return left, right, right[..., 0].copy()


class PiecewiseBezierPath:
    """An ordered, C0-validated piecewise-Bezier path over the plant's positions."""

    def __init__(self, num_positions: int, segments: list[BezierSegment]):
        self.num_positions = num_positions
        self.segments = segments
        lower = np.full(num_positions, np.inf)
        upper = np.full(num_positions, -np.inf)
        for seg in segments:
            lower = np.minimum(lower, seg.control_points.min(axis=1))
            upper = np.maximum(upper, seg.control_points.max(axis=1))
        self.global_lower = lower
        self.global_upper = upper
        #: Coordinates whose whole control-point range is within the continuity tolerance.
        self.constant_coordinates = (upper - lower) <= CONTINUITY_TOLERANCE

    @property
    def start_time(self) -> float:
        return self.segments[0].t_start

    @property
    def end_time(self) -> float:
        return self.segments[-1].t_end

    @staticmethod
    def from_trajectory(trajectory, continuous_revolute_indices: Sequence[int] = ()) -> "PiecewiseBezierPath":
        """Normalizes a BezierCurve, BsplineTrajectory, PiecewisePolynomial or a
        CompositeTrajectory of those. Raises on unsupported types, degree above 10, or a
        junction discontinuity (modulo 2*pi for continuous-revolute coordinates)."""
        if trajectory.cols() != 1:
            raise RuntimeError(
                f"PiecewiseBezierPath::FromTrajectory: the trajectory is "
                f"{trajectory.rows()}x{trajectory.cols()}-valued; only column-vector-valued "
                "trajectories (cols() == 1) over the plant's generalized positions are supported.")
        n = int(trajectory.rows())
        if n < 1:
            raise RuntimeError(
                "PiecewiseBezierPath::FromTrajectory: the trajectory has zero rows; expected one "
                "row per generalized position.")
        segments: list[BezierSegment] = []
        _append_segments(trajectory, [0], segments)
        _validate(n, continuous_revolute_indices, segments)
        return PiecewiseBezierPath(n, segments)

    @staticmethod
    def from_waypoints(waypoints: np.ndarray) -> "PiecewiseBezierPath":
        """K-1 order-1 segments through the K waypoint columns; segment k spans [k, k+1]."""
        waypoints = np.asarray(waypoints, dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[0] < 1:
            raise RuntimeError(
                "PiecewiseBezierPath::FromWaypoints: the waypoint matrix has zero rows; expected "
                "one row per generalized position.")
        if waypoints.shape[1] < 2:
            raise RuntimeError(
                "PiecewiseBezierPath::FromWaypoints: at least 2 waypoints (columns) are required "
                f"to form a path; got {waypoints.shape[1]}.")
        segments = [BezierSegment(float(k), float(k + 1), waypoints[:, k:k + 2].copy())
                    for k in range(waypoints.shape[1] - 1)]
        _validate(waypoints.shape[0], (), segments)
        return PiecewiseBezierPath(waypoints.shape[0], segments)

    def value(self, t: float) -> np.ndarray:
        """q(t); at a junction the later segment is evaluated."""
        t0, tf = self.start_time, self.end_time
        slack = PARAMETER_SLACK * max(1.0, abs(t0), abs(tf))
        if not (t >= t0 - slack) or not (t <= tf + slack):
            raise RuntimeError(
                f"PiecewiseBezierPath::Value: time {t} is outside the path's domain [{t0}, {tf}].")
        t = min(max(t, t0), tf)
        index = 0
        for k, seg in enumerate(self.segments):
            if seg.t_start <= t:
                index = k
            else:
                break
        seg = self.segments[index]
        duration = seg.t_end - seg.t_start
        s = (t - seg.t_start) / duration if duration > 0.0 else 0.0
        return self.evaluate_segment(index, min(max(s, 0.0), 1.0))

    def evaluate_segment(self, index: int, s: float) -> np.ndarray:
        """Segment `index` at local parameter s in [0, 1], by de Casteljau."""
        if index < 0 or index >= len(self.segments):
            raise RuntimeError(
                f"PiecewiseBezierPath::EvaluateSegment: segment index {index} is out of range; "
                f"the path has {len(self.segments)} segments.")
        if not (s >= -PARAMETER_SLACK) or not (s <= 1.0 + PARAMETER_SLACK):
            raise RuntimeError(
                f"PiecewiseBezierPath::EvaluateSegment: parameter s = {s} is outside the "
                "segment's domain [0, 1].")
        u = min(max(s, 0.0), 1.0)
        work = self.segments[index].control_points.copy()
        m = work.shape[1] - 1
        for r in range(1, m + 1):
            for j in range(m - r + 1):
                work[:, j] = (1.0 - u) * work[:, j] + u * work[:, j + 1]
        return work[:, 0]


def _append_segments(trajectory, source_index: list, segments: list) -> None:
    if isinstance(trajectory, BezierCurve):
        cps = np.asarray(trajectory.control_points(), dtype=float)
        if cps.shape[1] < 1:
            raise RuntimeError(
                f"PiecewiseBezierPath: the BezierCurve at segment index {source_index[0]} has no "
                "control points.")
        segments.append(BezierSegment(float(trajectory.start_time()), float(trajectory.end_time()),
                                      cps.copy()))
        source_index[0] += 1
    elif isinstance(trajectory, CompositeTrajectory):
        num = trajectory.get_number_of_segments()
        if num < 1:
            raise RuntimeError(
                f"PiecewiseBezierPath: the CompositeTrajectory at segment index {source_index[0]} "
                "has no segments.")
        for i in range(num):
            _append_segments(trajectory.segment(i), source_index, segments)
    elif isinstance(trajectory, BsplineTrajectory):
        _append_bspline(trajectory, source_index[0], segments)
        source_index[0] += 1
    elif isinstance(trajectory, PiecewisePolynomial):
        _append_piecewise_polynomial(trajectory, source_index[0], segments)
        source_index[0] += 1
    else:
        raise RuntimeError(
            f"PiecewiseBezierPath: unsupported trajectory type '{type(trajectory).__name__}' at "
            f"segment index {source_index[0]}. Supported types are BezierCurve, "
            "BsplineTrajectory, PiecewisePolynomial, and CompositeTrajectory whose segments are "
            "themselves supported.")


def _append_bspline(bspline: BsplineTrajectory, source_index: int, segments: list) -> None:
    """Boehm knot insertion raises every in-domain knot to multiplicity >= degree, after which
    each nonempty span's p+1 active control points are exactly its Bezier control points."""
    if bspline.cols() != 1:
        raise RuntimeError(
            f"PiecewiseBezierPath: the BsplineTrajectory at segment index {source_index} is "
            f"{bspline.rows()}x{bspline.cols()}-valued; only column-vector-valued trajectories "
            "(cols() == 1) over the plant's generalized positions are supported.")
    traj = bspline.Clone()
    order = traj.basis().order()
    degree = order - 1
    if degree > 0:
        t0 = traj.basis().initial_parameter_value()
        tf = traj.basis().final_parameter_value()
        knots = list(traj.basis().knots())
        extra = []
        i = 0
        while i < len(knots):
            j = i
            while j < len(knots) and knots[j] == knots[i]:
                j += 1
            if t0 <= knots[i] <= tf:
                extra.extend([knots[i]] * max(0, degree - (j - i)))
            i = j
        if extra:
            traj.InsertKnots(extra)
    knots = list(traj.basis().knots())
    points = [np.asarray(p, dtype=float).reshape(-1) for p in traj.control_points()]
    before = len(segments)
    for i in range(order - 1, len(points)):
        if not knots[i] < knots[i + 1]:
            continue
        cps = np.stack(points[i - degree:i + 1], axis=1)
        segments.append(BezierSegment(float(knots[i]), float(knots[i + 1]), cps))
    if len(segments) == before:
        raise RuntimeError(
            f"PiecewiseBezierPath: the BsplineTrajectory at segment index {source_index} has an "
            "empty parameter domain; a trajectory must span a positive time interval.")


def _append_piecewise_polynomial(pp: PiecewisePolynomial, source_index: int, segments: list) -> None:
    """Monomial coefficients in relative time, rescaled to s in [0, 1], then the exact change
    of basis P_j = sum_{a<=j} C(j,a)/C(m,a) alpha_a."""
    if pp.cols() != 1:
        raise RuntimeError(
            f"PiecewiseBezierPath: the PiecewisePolynomial at segment index {source_index} is "
            f"{pp.rows()}x{pp.cols()}-valued; only column-vector-valued trajectories (cols() == 1) "
            "over the plant's generalized positions are supported.")
    n = int(pp.rows())
    if pp.get_number_of_segments() < 1:
        raise RuntimeError(
            f"PiecewiseBezierPath: the PiecewisePolynomial at segment index {source_index} has no "
            "segments.")
    for k in range(pp.get_number_of_segments()):
        m = max(pp.getSegmentPolynomialDegree(k, r, 0) for r in range(n))
        if m > MAX_CONVERSION_DEGREE:
            raise RuntimeError(
                f"PiecewiseBezierPath: PiecewisePolynomial segment {k} (source segment index "
                f"{source_index}) has polynomial degree {m}, above the supported maximum of "
                f"{MAX_CONVERSION_DEGREE}. The monomial-to-Bernstein change of basis is "
                "ill-conditioned at high degree; re-express the trajectory with more, "
                "lower-degree segments.")
        t_start, t_end = float(pp.start_time(k)), float(pp.end_time(k))
        duration = t_end - t_start
        if not duration > 0.0:
            raise RuntimeError(
                f"PiecewiseBezierPath: PiecewisePolynomial segment {k} (source segment index "
                f"{source_index}) has non-positive duration {duration}.")
        cps = np.zeros((n, m + 1))
        for r in range(n):
            coefficients = np.asarray(pp.getPolynomial(k, r, 0).GetCoefficients(), dtype=float)
            alpha = np.zeros(m + 1)
            scale = 1.0
            for a in range(min(len(coefficients) - 1, m) + 1):
                alpha[a] = coefficients[a] * scale
                scale *= duration
            for j in range(m + 1):
                total = 0.0
                for a in range(j + 1):
                    total += (math.comb(j, a) / math.comb(m, a)) * alpha[a]
                cps[r, j] = total
        segments.append(BezierSegment(t_start, t_end, cps))


def _validate(n: int, continuous_revolute_indices: Sequence[int], segments: list) -> None:
    if not segments:
        raise RuntimeError("PiecewiseBezierPath: the trajectory produced no Bezier segments.")
    for i, seg in enumerate(segments):
        if seg.control_points.shape[0] != n:
            raise RuntimeError(
                f"PiecewiseBezierPath: segment {i} has {seg.control_points.shape[0]} rows but the "
                f"trajectory declares {n} generalized positions; every segment must be valued in "
                "the same position space.")
        if seg.control_points.shape[1] < 1:
            raise RuntimeError(f"PiecewiseBezierPath: segment {i} has no control points.")
        if not seg.t_end >= seg.t_start:
            raise RuntimeError(
                f"PiecewiseBezierPath: segment {i} spans [{seg.t_start}, {seg.t_end}], which runs "
                "backwards in time.")
        if i > 0:
            prev_end = segments[i - 1].t_end
            slack = TIME_CONTIGUITY_SLACK * max(1.0, abs(prev_end), abs(seg.t_start))
            if abs(seg.t_start - prev_end) > slack:
                raise RuntimeError(
                    f"PiecewiseBezierPath: segments are not contiguous in time: segment {i - 1} "
                    f"ends at {prev_end} but segment {i} starts at {seg.t_start}. Segments must "
                    "be ordered and meet end-to-start.")
    revolute = np.zeros(n, dtype=bool)
    for index in continuous_revolute_indices:
        if int(index) < 0 or int(index) >= n:
            raise RuntimeError(
                "PiecewiseBezierPath: Options::continuous_revolute_indices contains "
                f"{index}, which is out of range for a trajectory with {n} generalized positions.")
        revolute[int(index)] = True
    for i in range(1, len(segments)):
        prev, nxt = segments[i - 1].control_points, segments[i].control_points
        for c in range(n):
            raw_gap = nxt[c, 0] - prev[c, -1]
            gap = raw_gap
            if revolute[c]:
                gap -= 2 * math.pi * round(gap / (2 * math.pi))
            if abs(gap) > CONTINUITY_TOLERANCE:
                modulo = f" ({gap} modulo 2π)" if revolute[c] else ""
                raise RuntimeError(
                    f"PiecewiseBezierPath: C0 discontinuity at the junction between segments "
                    f"{i - 1} and {i} in coordinate {c}: the gap is {raw_gap}{modulo}, which "
                    f"exceeds the continuity tolerance {CONTINUITY_TOLERANCE}. A discontinuous "
                    "trajectory teleports; per-segment certificates would not cover the jump. If "
                    f"coordinate {c} is a continuous revolute joint, list it in "
                    "Options::continuous_revolute_indices.")
