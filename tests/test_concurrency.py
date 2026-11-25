"""Determinism and concurrent use (port of concurrency_test.cc), on a corpus of ten random
worlds (a mix of free and violating) plus a deep, grazing workload found by bisection.

1. The answer does not depend on Options.parallelism, nor on the round size of the work list
   (this port's analog of the fork's thread count): verdict and witness are identical, and on a
   certified run so is num_nodes.
2. The public Check* methods are safe to call concurrently on one instance."""

import threading

import numpy as np
import pytest
from pydrake.common import Parallelism

from continuous_collision import ContinuousCollisionChecker, Options, Verdict, certifier

import helpers as h

MARGIN = 0.005
NUM_CASES, MIN_FREE, MIN_VIOLATING = 10, 3, 3
PROBE_BUDGET, MIN_DEEP_NODES, DEEP_RESOLUTION = 6000, 800, 1e-8


def options(threads, margin=MARGIN, resolution=1e-6):
    return Options(margin=margin, distance_resolution=resolution, parallelism=Parallelism(threads))


def same_result(a, b):
    if a.verdict != b.verdict:
        return False
    if a.verdict == Verdict.kCertifiedFree and a.num_nodes != b.num_nodes:
        return False
    if (a.finding is None) != (b.finding is None):
        return False
    if a.finding is None:
        return True
    f, g = a.finding, b.finding
    return (f.time == g.time and np.array_equal(f.q, g.q) and f.geometry_a == g.geometry_a
            and f.geometry_b == g.geometry_b and f.distance == g.distance
            and np.array_equal(f.nearest_a_W, g.nearest_a_W) and np.array_equal(f.nearest_b_W, g.nearest_b_W))


class Case:
    def __init__(self, seed):
        self.name = f"seed_{seed}"
        self.model = h.make_random_world(seed)
        self.checker = ContinuousCollisionChecker(self.model, options(1))
        self.control_points = h.random_control_points(seed, self.model.plant().num_positions())
        self.verdict = self.checker.CheckTrajectory(self.trajectory()).verdict

    def trajectory(self):
        return h.bezier(self.control_points)



@pytest.fixture(scope="module")
def corpus():
    cases, free, violating = [], 0, 0
    for seed in range(1, 201):
        if len(cases) >= NUM_CASES:
            break
        case = Case(seed)
        is_free, is_violating = case.verdict == Verdict.kCertifiedFree, case.verdict == Verdict.kViolationFound
        if not (is_free or is_violating):
            continue
        if is_free and free >= NUM_CASES - MIN_VIOLATING or is_violating and violating >= NUM_CASES - MIN_FREE:
            continue
        free, violating = free + is_free, violating + is_violating
        cases.append(case)
    return cases


def grazing_margin(entry):
    """The largest margin whose tree certifies within the probe budget, by bisection."""

    def certifiable(margin):
        r = entry.checker.CheckTrajectory(entry.trajectory(), options(1, margin, DEEP_RESOLUTION))
        return r.verdict == Verdict.kCertifiedFree and r.num_nodes <= PROBE_BUDGET

    ok, grazing = 0.0, MARGIN
    for _ in range(12):
        if not certifiable(grazing):
            break
        ok, grazing = grazing, 2 * grazing
    for _ in range(30):
        mid = 0.5 * (ok + grazing)
        if certifiable(mid):
            ok = mid
        else:
            grazing = mid
    return ok, entry.checker.CheckTrajectory(entry.trajectory(), options(1, ok, DEEP_RESOLUTION)).num_nodes


@pytest.fixture(scope="module")
def deep(corpus):
    """The deepest grazing workload over the corpus's free cases."""
    best = None
    for entry in corpus:
        if entry.verdict == Verdict.kCertifiedFree:
            margin, nodes = grazing_margin(entry)
            if best is None or nodes > best[2]:
                best = (entry, margin, nodes)
    return best


def test_corpus_is_balanced(corpus):
    assert len(corpus) == NUM_CASES
    free = sum(c.verdict == Verdict.kCertifiedFree for c in corpus)
    assert free >= MIN_FREE and NUM_CASES - free >= MIN_VIOLATING


def test_parallelism_invariant_and_serially_deterministic(corpus):
    for case in corpus:
        serial = case.checker.CheckTrajectory(case.trajectory(), options(1))
        assert same_result(serial, case.checker.CheckTrajectory(case.trajectory(), options(1))), case.name
        for threads in (2, 8, 16):
            assert same_result(serial, case.checker.CheckTrajectory(case.trajectory(), options(threads))), case.name


@pytest.mark.parametrize("chunk", [1, 3, 64])
def test_round_size_invariant(corpus, monkeypatch, chunk):
    expected = [c.checker.CheckTrajectory(c.trajectory(), options(1)) for c in corpus]
    monkeypatch.setattr(certifier, "CHUNK", chunk)
    for case, want in zip(corpus, expected):
        assert same_result(want, case.checker.CheckTrajectory(case.trajectory(), options(1))), case.name


def test_concurrent_calls_on_one_checker_match_sequential(corpus):
    opts = options(2)
    expected, waypoints = [], []
    for case in corpus:
        q1, q2 = case.control_points[:, 0], case.control_points[:, -1]
        w = np.column_stack((q1, 0.5 * (q1 + q2), q2))
        waypoints.append(w)
        expected.append((case.checker.CheckTrajectory(case.trajectory(), opts),
                         case.checker.CheckEdge(q1, q2, opts), case.checker.CheckPath(w, opts)))
    mismatches = [0] * 8

    def work(t):
        for _ in range(3):
            for k, case in enumerate(corpus):
                q1, q2 = case.control_points[:, 0], case.control_points[:, -1]
                got = (case.checker.CheckTrajectory(case.trajectory(), opts), case.checker.CheckEdge(q1, q2, opts),
                       case.checker.CheckPath(waypoints[k], opts))
                if not all(same_result(a, b) for a, b in zip(got, expected[k])):
                    mismatches[t] += 1

    threads = [threading.Thread(target=work, args=(t,)) for t in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert mismatches == [0] * 8


def test_deep_workload_is_invariant(deep, monkeypatch):
    entry, margin, nodes = deep
    assert nodes >= MIN_DEEP_NODES, f"grazing margin {margin}"
    serial = entry.checker.CheckTrajectory(entry.trajectory(), options(1, margin, DEEP_RESOLUTION))
    for threads in (4, 16):
        assert same_result(serial, entry.checker.CheckTrajectory(entry.trajectory(), options(threads, margin, DEEP_RESOLUTION)))
    monkeypatch.setattr(certifier, "CHUNK", 5)
    assert same_result(serial, entry.checker.CheckTrajectory(entry.trajectory(), options(1, margin, DEEP_RESOLUTION)))


def test_deep_workload_survives_concurrent_calls(deep):
    entry, margin, _ = deep
    expected = entry.checker.CheckTrajectory(entry.trajectory(), options(1, margin, DEEP_RESOLUTION))
    mismatches = [0] * 4

    def work(t):
        for _ in range(2):
            got = entry.checker.CheckTrajectory(entry.trajectory(), options(4, margin, DEEP_RESOLUTION))
            mismatches[t] += not same_result(got, expected)

    threads = [threading.Thread(target=work, args=(t,)) for t in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert mismatches == [0] * 4
