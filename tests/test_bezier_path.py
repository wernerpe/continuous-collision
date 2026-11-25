"""Trajectory normalization (port of piecewise_bezier_path_test.cc). Reference values come from
Drake's own trajectory classes, so the conversions are checked against an independent
implementation."""

import math

import numpy as np
import pytest
from pydrake.math import BsplineBasis, KnotVectorType
from pydrake.trajectories import (BezierCurve, BsplineTrajectory, CompositeTrajectory,
                                  PiecewisePolynomial, Trajectory)

from continuous_collision._numerics import MAX_CONVERSION_DEGREE
from continuous_collision.bezier_path import PiecewiseBezierPath, split_at_half

TWO_PI = 2 * math.pi


def random_matrix(rng, rows, cols):
    return rng.uniform(-1.0, 1.0, (rows, cols))


def drake_bezier_value(cps, s):
    return BezierCurve(0.0, 1.0, cps).value(s).ravel()


def make_bezier(rng, start, order, t0, t1):
    cps = random_matrix(rng, len(start), order + 1)
    cps[:, 0] = start
    return BezierCurve(t0, t1, cps)


def max_sampled_error(path, trajectory, num_samples):
    t0, tf = trajectory.start_time(), trajectory.end_time()
    worst = 0.0
    for i in range(num_samples):
        t = t0 + (tf - t0) * i / (num_samples - 1.0)
        worst = max(worst, np.abs(trajectory.value(t).ravel() - path.value(t)).max())
    return worst


class UnsupportedTrajectory(Trajectory):
    def __init__(self, rows, t0, t1):
        Trajectory.__init__(self)
        self._rows, self._t0, self._t1 = rows, t0, t1

    def do_value(self, t):
        return np.zeros((self._rows, 1))

    def do_rows(self):
        return self._rows

    def do_cols(self):
        return 1

    def do_start_time(self):
        return self._t0

    def do_end_time(self):
        return self._t1

    def DoClone(self):
        return UnsupportedTrajectory(self._rows, self._t0, self._t1)

    def __deepcopy__(self, memo):
        return UnsupportedTrajectory(self._rows, self._t0, self._t1)


# --------------------------------------------------------------------------- evaluation, split


def test_evaluation_matches_drake_bezier_curve():
    rng = np.random.default_rng(1234)
    for order in range(1, 6):
        for trial in range(5):
            n = 1 + trial % 6
            cps = random_matrix(rng, n, order + 1)
            t0 = -0.75 + 0.4 * trial
            t1 = t0 + 1.0 + 0.3 * trial
            curve = BezierCurve(t0, t1, cps)
            path = PiecewiseBezierPath.from_trajectory(curve)
            assert path.num_positions == n and len(path.segments) == 1
            assert path.start_time == t0 and path.end_time == t1
            assert np.array_equal(path.segments[0].control_points, cps)
            for i in range(1001):
                s = i / 1000
                t = t0 + s * (t1 - t0)
                expected = curve.value(t).ravel()
                assert np.abs(path.value(t) - expected).max() < 1e-12
                assert np.abs(path.evaluate_segment(0, s) - expected).max() < 1e-12


def test_de_casteljau_children_reproduce_parent():
    rng = np.random.default_rng(20260826)
    worst = 0.0
    for _ in range(1000):
        n, order = rng.integers(1, 8), rng.integers(0, 7)
        parent = random_matrix(rng, n, order + 1)
        left, right, mid = split_at_half(parent)
        assert left.shape == right.shape == parent.shape and mid.shape == (n,)
        worst = max(worst, np.abs(mid - drake_bezier_value(parent, 0.5)).max(),
                    np.abs(left[:, 0] - parent[:, 0]).max(), np.abs(right[:, -1] - parent[:, -1]).max(),
                    np.abs(left[:, -1] - mid).max(), np.abs(right[:, 0] - mid).max())
        for i in range(21):
            s = i / 20
            child = drake_bezier_value(left, 2 * s) if s <= 0.5 else drake_bezier_value(right, 2 * s - 1)
            worst = max(worst, np.abs(child - drake_bezier_value(parent, s)).max())
    assert worst < 1e-12


def test_de_casteljau_batched_equals_single():
    # The certifier splits a whole round at once; batching must not change a single bit.
    rng = np.random.default_rng(7)
    batch = rng.uniform(-1, 1, (9, 6, 4))
    L, R, M = split_at_half(batch)
    for k in range(9):
        l, r, m = split_at_half(batch[k])
        assert np.array_equal(L[k], l) and np.array_equal(R[k], r) and np.array_equal(M[k], m)


@pytest.mark.skip(reason="Allocation-freedom of the C++ hot loop (LimitMalloc) has no Python "
                         "counterpart; the batched split allocates one array per round by design.")
def test_pre_sized_outputs_do_not_allocate():
    pass


def test_control_boxes_contain_curve_after_random_subdivision():
    rng = np.random.default_rng(99991)
    tol = 1e-12
    for _ in range(1000):
        n, order, depth = rng.integers(1, 6), rng.integers(1, 7), rng.integers(1, 7)
        root = random_matrix(rng, n, order + 1)
        node, a, b = root, 0.0, 1.0
        for _ in range(depth):
            parent = node
            left, right, _ = split_at_half(parent)
            midpoint = 0.5 * (a + b)
            if rng.integers(0, 2) == 0:
                node, b = left, midpoint
            else:
                node, a = right, midpoint
            assert (node.min(axis=1) >= parent.min(axis=1) - tol).all()
            assert (node.max(axis=1) <= parent.max(axis=1) + tol).all()
        assert (node.min(axis=1) >= root.min(axis=1) - tol).all()
        assert (node.max(axis=1) <= root.max(axis=1) + tol).all()
        for i in range(41):
            local = i / 40
            from_node = drake_bezier_value(node, local)
            assert np.abs(from_node - drake_bezier_value(root, a + local * (b - a))).max() < 1e-12
            assert (from_node >= node.min(axis=1) - tol).all() and (from_node <= node.max(axis=1) + tol).all()


# --------------------------------------------------------------------------- B-spline -> Bezier


def bspline_from_basis(rng, basis, n):
    return BsplineTrajectory(basis, [random_matrix(rng, n, 1) for _ in range(basis.num_basis_functions())])


def check_bspline(bspline, expected_segments=0):
    path = PiecewiseBezierPath.from_trajectory(bspline)
    assert path.num_positions == bspline.rows()
    assert abs(path.start_time - bspline.start_time()) < 1e-14
    assert abs(path.end_time - bspline.end_time()) < 1e-14
    for seg in path.segments:
        assert seg.control_points.shape[1] == bspline.basis().order()
    if expected_segments:
        assert len(path.segments) == expected_segments
    assert max_sampled_error(path, bspline, 10001) < 1e-10


@pytest.mark.parametrize("order", range(2, 7))
def test_bspline_clamped_uniform(order):
    rng = np.random.default_rng(4242 + order)
    n_basis = order + 4
    check_bspline(bspline_from_basis(rng, BsplineBasis(order, n_basis, KnotVectorType.kClampedUniform, 0.0, 3.0), 3),
                  n_basis - order + 1)


@pytest.mark.parametrize("order", range(2, 7))
def test_bspline_non_uniform_knots(order):
    rng = np.random.default_rng(515151 + order)
    knots = [0.0] * order + [0.13, 0.29, 0.31, 1.70, 2.55] + [3.0] * order
    check_bspline(bspline_from_basis(rng, BsplineBasis(order, knots), 4), 6)


def test_bspline_repeated_interior_knots():
    rng = np.random.default_rng(606060)
    knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0]
    check_bspline(bspline_from_basis(rng, BsplineBasis(4, knots), 2), 5)


def test_bspline_kinematic_trajectory_optimization_style():
    rng = np.random.default_rng(777)
    check_bspline(bspline_from_basis(rng, BsplineBasis(4, 10, KnotVectorType.kClampedUniform, 0.0, 5.0), 7), 7)


@pytest.mark.parametrize("order", range(2, 6))
def test_bspline_unclamped_uniform(order):
    rng = np.random.default_rng(31337 + order)
    check_bspline(bspline_from_basis(rng, BsplineBasis(order, order + 5, KnotVectorType.kUniform, 0.0, 2.0), 3))


def test_bspline_segment_times_match_knot_spans():
    rng = np.random.default_rng(24680)
    knots = [0.0, 0.0, 0.0, 0.5, 1.25, 2.0, 2.0, 2.0]
    path = PiecewiseBezierPath.from_trajectory(bspline_from_basis(rng, BsplineBasis(3, knots), 2))
    expected = [0.0, 0.5, 1.25, 2.0]
    assert [(s.t_start, s.t_end) for s in path.segments] == list(zip(expected[:-1], expected[1:]))


def test_bspline_matrix_valued_throws():
    bspline = BsplineTrajectory(BsplineBasis(3, 6, KnotVectorType.kClampedUniform, 0.0, 1.0),
                                [np.zeros((2, 2))] * 6)
    with pytest.raises(RuntimeError, match="column-vector-valued"):
        PiecewiseBezierPath.from_trajectory(bspline)


# --------------------------------------------------------------------------- PiecewisePolynomial


def test_first_order_hold():
    rng = np.random.default_rng(11235)
    times = np.linspace(0.0, 2.5, 6)
    samples = random_matrix(rng, 4, 6)
    pp = PiecewisePolynomial.FirstOrderHold(times, samples)
    path = PiecewiseBezierPath.from_trajectory(pp)
    assert len(path.segments) == 5
    for k in range(5):
        cps = path.segments[k].control_points
        assert cps.shape[1] == 2
        assert np.abs(cps[:, 0] - samples[:, k]).max() < 1e-14
        assert np.abs(cps[:, 1] - samples[:, k + 1]).max() < 1e-14
    assert max_sampled_error(path, pp, 10001) < 1e-10


def test_cubic_splines():
    rng = np.random.default_rng(626262)
    times = np.linspace(-1.0, 3.0, 7)
    samples = random_matrix(rng, 3, 7)
    a = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(times, samples)
    path_a = PiecewiseBezierPath.from_trajectory(a)
    assert len(path_a.segments) == 6
    assert all(s.control_points.shape[1] == 4 for s in path_a.segments)
    assert max_sampled_error(path_a, a, 10001) < 1e-10
    b = PiecewisePolynomial.CubicShapePreserving(times, samples)
    assert max_sampled_error(PiecewiseBezierPath.from_trajectory(b), b, 10001) < 1e-10


@pytest.mark.parametrize("degree", range(1, MAX_CONVERSION_DEGREE + 2))
def test_lagrange_up_to_degree_cap_and_beyond(degree):
    assert MAX_CONVERSION_DEGREE == 10
    times = np.array([0.3 + 1.7 * i / degree for i in range(degree + 1)])
    samples = np.vstack((np.sin(3.0 * times), np.cos(2.0 * times) - 0.25 * times))
    pp = PiecewisePolynomial.LagrangeInterpolatingPolynomial(times, samples)
    if degree > MAX_CONVERSION_DEGREE:
        with pytest.raises(RuntimeError, match="polynomial degree 11"):
            PiecewiseBezierPath.from_trajectory(pp)
        return
    path = PiecewiseBezierPath.from_trajectory(pp)
    assert len(path.segments) == 1 and path.segments[0].control_points.shape[1] == degree + 1
    assert max_sampled_error(path, pp, 10001) < 1e-10


def test_piecewise_polynomial_matrix_valued_throws():
    pp = PiecewisePolynomial.FirstOrderHold([0.0, 1.0], [np.zeros((2, 2)), np.ones((2, 2))])
    with pytest.raises(RuntimeError, match="column-vector-valued"):
        PiecewiseBezierPath.from_trajectory(pp)


# --------------------------------------------------------------------------- junctions


def junction_case(rng, offset):
    n = len(offset)
    first, second = random_matrix(rng, n, 4), random_matrix(rng, n, 3)
    second[:, 0] = first[:, 3] + offset
    return CompositeTrajectory([BezierCurve(0.0, 1.0, first), BezierCurve(1.0, 2.5, second)])


def test_injected_discontinuity_throws():
    rng = np.random.default_rng(90210)
    with pytest.raises(RuntimeError, match=r"C0 discontinuity[\s\S]*coordinate 1"):
        PiecewiseBezierPath.from_trajectory(junction_case(rng, np.array([0.0, 1e-3, 0.0])))
    PiecewiseBezierPath.from_trajectory(junction_case(rng, np.array([0.0, 0.0, 9e-8])))


def test_two_pi_offset_accepted_only_when_declared_revolute():
    rng = np.random.default_rng(1357)
    trajectory = junction_case(rng, np.array([0.0, TWO_PI, 0.0]))
    with pytest.raises(RuntimeError, match="C0 discontinuity"):
        PiecewiseBezierPath.from_trajectory(trajectory)
    with pytest.raises(RuntimeError, match="C0 discontinuity"):
        PiecewiseBezierPath.from_trajectory(trajectory, [0, 2])
    PiecewiseBezierPath.from_trajectory(trajectory, [1])
    PiecewiseBezierPath.from_trajectory(junction_case(rng, np.array([0.0, -3 * TWO_PI, 0.0])), [1])
    with pytest.raises(RuntimeError, match="C0 discontinuity"):
        PiecewiseBezierPath.from_trajectory(junction_case(rng, np.array([0.0, TWO_PI + 1e-3, 0.0])), [1])


def test_control_points_are_not_realigned_across_two_pi_junction():
    rng = np.random.default_rng(864213)
    trajectory = junction_case(rng, np.array([TWO_PI, 0.0]))
    path = PiecewiseBezierPath.from_trajectory(trajectory, [0])
    first, second = path.segments[0].control_points, path.segments[1].control_points
    assert abs(second[0, 0] - first[0, -1] - TWO_PI) < 1e-14
    assert max_sampled_error(path, trajectory, 501) < 1e-12
    assert abs(path.value(1.0)[0] - second[0, 0]) < 1e-14
    assert abs(path.evaluate_segment(0, 1.0)[0] - first[0, -1]) < 1e-14
    assert path.global_upper[0] - path.global_lower[0] > 5.0


def test_out_of_range_revolute_index_throws():
    with pytest.raises(RuntimeError, match="continuous_revolute_indices"):
        PiecewiseBezierPath.from_trajectory(BezierCurve(0.0, 1.0, np.array([[0.0, 1.0, 2.0], [0.0, 0.0, 0.0]])), [2])


def test_zero_order_hold_is_rejected():
    samples = np.array([[0.0, 1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 0.0]])
    with pytest.raises(RuntimeError, match="C0 discontinuity"):
        PiecewiseBezierPath.from_trajectory(PiecewisePolynomial.ZeroOrderHold(np.linspace(0, 3, 4), samples))


# --------------------------------------------------------------------------- metadata


def test_global_control_box():
    rng = np.random.default_rng(5150)
    start, t, pieces, all_cps = random_matrix(rng, 3, 1)[:, 0], 0.0, [], []
    for _ in range(3):
        curve = make_bezier(rng, start, 3, t, t + 1.0)
        all_cps.append(curve.control_points())
        start = curve.control_points()[:, 3]
        t += 1.0
        pieces.append(curve)
    path = PiecewiseBezierPath.from_trajectory(CompositeTrajectory(pieces))
    assert np.array_equal(path.global_lower, np.min([c.min(axis=1) for c in all_cps], axis=0))
    assert np.array_equal(path.global_upper, np.max([c.max(axis=1) for c in all_cps], axis=0))
    for i in range(1001):
        q = path.value(3.0 * i / 1000)
        assert (q >= path.global_lower - 1e-12).all() and (q <= path.global_upper + 1e-12).all()


def test_constant_coordinate_flags():
    waypoints = np.array([[0.0, 0.5, -0.25, 1.0], [2.0, 2.0, 2.0, 2.0],
                          [1.0, 1.0 + 5e-9, 1.0 - 2e-8, 1.0], [0.0, 0.0, 2e-7, 0.0]])
    path = PiecewiseBezierPath.from_waypoints(waypoints)
    assert list(path.constant_coordinates) == [False, True, True, False]


# --------------------------------------------------------------------------- composite


def test_composite_bezier_segments_round_trip():
    rng = np.random.default_rng(31415)
    orders, breaks = [3, 5, 2, 1], [0.0, 0.4, 1.9, 2.0, 4.25]
    start, pieces = random_matrix(rng, 5, 1)[:, 0], []
    for i, order in enumerate(orders):
        curve = make_bezier(rng, start, order, breaks[i], breaks[i + 1])
        start = curve.control_points()[:, order]
        pieces.append(curve)
    trajectory = CompositeTrajectory(pieces)
    path = PiecewiseBezierPath.from_trajectory(trajectory)
    assert [s.control_points.shape[1] for s in path.segments] == [o + 1 for o in orders]
    assert [s.t_start for s in path.segments] == breaks[:-1]
    assert [s.t_end for s in path.segments] == breaks[1:]
    assert max_sampled_error(path, trajectory, 10001) < 1e-12


def test_nested_composite_recursion():
    rng = np.random.default_rng(2718)
    start = random_matrix(rng, 2, 1)[:, 0]
    a = make_bezier(rng, start, 2, 0.0, 1.0)
    b = make_bezier(rng, a.control_points()[:, 2], 3, 1.0, 2.0)
    c = make_bezier(rng, b.control_points()[:, 3], 1, 2.0, 3.0)
    trajectory = CompositeTrajectory([a, CompositeTrajectory([b, c])])
    path = PiecewiseBezierPath.from_trajectory(trajectory)
    assert [s.control_points.shape[1] for s in path.segments] == [3, 4, 2]
    assert max_sampled_error(path, trajectory, 5001) < 1e-12


def test_composite_mixed_segment_types():
    rng = np.random.default_rng(11111)
    bspline = bspline_from_basis(rng, BsplineBasis(4, 8, KnotVectorType.kClampedUniform, 0.0, 1.0), 2)
    end = bspline.FinalValue().ravel()
    samples = np.column_stack((end, end + 0.3, end - 0.1))
    pp = PiecewisePolynomial.FirstOrderHold([1.0, 1.5, 2.0], samples)
    trajectory = CompositeTrajectory([bspline, pp])
    path = PiecewiseBezierPath.from_trajectory(trajectory)
    assert len(path.segments) == 7
    assert max_sampled_error(path, trajectory, 10001) < 1e-10


def test_unknown_segment_type_throws_with_index_and_type_name():
    rng = np.random.default_rng(4321)
    first = make_bezier(rng, random_matrix(rng, 3, 1)[:, 0], 2, 0.0, 1.0)
    # Inside a CompositeTrajectory pydrake hands back its own wrapper type for a Python
    # subclass, so only the index is pinned there; the bare case names the class.
    with pytest.raises(RuntimeError, match=r"unsupported trajectory type[\s\S]*segment index 1"):
        PiecewiseBezierPath.from_trajectory(CompositeTrajectory([first, UnsupportedTrajectory(3, 1.0, 2.0)]))
    with pytest.raises(RuntimeError, match=r"UnsupportedTrajectory[\s\S]*segment index 0"):
        PiecewiseBezierPath.from_trajectory(UnsupportedTrajectory(3, 0.0, 1.0))


# --------------------------------------------------------------------------- waypoints, domain


def test_waypoint_segments_are_exact():
    rng = np.random.default_rng(19191)
    waypoints = random_matrix(rng, 6, 5)
    path = PiecewiseBezierPath.from_waypoints(waypoints)
    assert path.num_positions == 6 and len(path.segments) == 4
    assert path.start_time == 0.0 and path.end_time == 4.0
    for k, seg in enumerate(path.segments):
        assert (seg.t_start, seg.t_end) == (k, k + 1)
        assert np.array_equal(seg.control_points, waypoints[:, k:k + 2])
        for i in range(101):
            s = i / 100
            expected = (1.0 - s) * waypoints[:, k] + s * waypoints[:, k + 1]
            assert np.abs(path.value(k + s) - expected).max() < 1e-15
            assert np.abs(path.evaluate_segment(k, s) - expected).max() < 1e-15


def test_too_few_waypoints_throws():
    with pytest.raises(RuntimeError, match="at least 2 waypoints"):
        PiecewiseBezierPath.from_waypoints(np.zeros((3, 1)))
    with pytest.raises(RuntimeError, match="zero rows"):
        PiecewiseBezierPath.from_waypoints(np.zeros((0, 4)))


def test_domain_edges_clamp_and_outside_throws():
    waypoints = np.array([[0.0, 1.0, 3.0], [-1.0, 0.0, 1.0]])
    path = PiecewiseBezierPath.from_waypoints(waypoints)
    assert np.array_equal(path.value(0.0), waypoints[:, 0])
    assert np.array_equal(path.value(2.0), waypoints[:, 2])
    path.value(2.0 + 1e-13)
    assert np.array_equal(path.value(-1e-13), waypoints[:, 0])
    path.evaluate_segment(0, 1.0 + 1e-13)
    for t in (-1e-3, 2.5):
        with pytest.raises(RuntimeError, match="outside the path's domain"):
            path.value(t)
    for s in (-0.5, 1.5):
        with pytest.raises(RuntimeError, match="outside the segment's domain"):
            path.evaluate_segment(0, s)
    for k in (-1, 2):
        with pytest.raises(RuntimeError, match="out of range"):
            path.evaluate_segment(k, 0.5)


def test_junction_time_selects_the_later_segment():
    path = PiecewiseBezierPath.from_waypoints(np.array([[0.0, 1.0, 3.0, 6.0]]))
    assert [path.value(t)[0] for t in (0.0, 1.0, 2.0, 3.0)] == [0.0, 1.0, 3.0, 6.0]
    assert abs(path.value(1.5)[0] - 2.0) < 1e-15 and abs(path.value(2.5)[0] - 4.5) < 1e-15


def test_junction_times_are_consistent():
    rng = np.random.default_rng(606)
    start, pieces, t = random_matrix(rng, 3, 1)[:, 0], [], 0.0
    for _ in range(4):
        curve = make_bezier(rng, start, 3, t, t + 0.75)
        start = curve.control_points()[:, 3]
        t += 0.75
        pieces.append(curve)
    trajectory = CompositeTrajectory(pieces)
    path = PiecewiseBezierPath.from_trajectory(trajectory)
    for i in range(4):
        assert np.abs(path.value(0.75 * i) - trajectory.value(0.75 * i).ravel()).max() < 1e-13
