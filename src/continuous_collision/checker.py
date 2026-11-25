"""The public API: ContinuousCollisionChecker, Options, Result, Finding, Verdict.

Design note: names, fields, defaults, error conditions and messages follow the header of Drake
PR #24924 (pydrake.planning.continuous_collision) so that switching to the fork, or to Drake
if the PR lands, is a one-line import change. Everything the checker learns about the model is
computed once at construction and never mutated, so the Check* methods are safe to call from
several threads; each call leases its own Drake context.
"""

from __future__ import annotations

import copy
import enum
import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
from pydrake.common import Parallelism

from .bezier_path import PiecewiseBezierPath
from .certifier import Certifier
from .distance_oracle import DistanceOracle
from .kinematics import BatchKinematics
from .motion_bound import KinematicsEngine


class Verdict(enum.Enum):
    """Outcome of one check."""

    #: Proof: every unfiltered pair keeps signed distance > margin over the whole time domain.
    kCertifiedFree = 0
    #: An exactly-on-trajectory configuration violates the threshold.
    kViolationFound = 1
    #: Some pair's clearance comes within distance_resolution (plus the oracle tolerance) of
    #: the margin: the trajectory grazes the margin.
    kInconclusive = 2


@dataclass
class Finding:
    """Where the plan fails, or where it could not be decided."""

    time: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.zeros(0))
    geometry_a: object = None  # pydrake.geometry.GeometryId
    geometry_b: object = None
    body_a: object = None  # pydrake.multibody.tree.BodyIndex
    body_b: object = None
    distance: float = 0.0
    nearest_a_W: Optional[np.ndarray] = None
    nearest_b_W: Optional[np.ndarray] = None


@dataclass
class Options:
    """Options controlling one check."""

    #: Clearance margin in meters; finite and nonnegative.
    margin: float = 0.0
    #: Resolution floor in meters; finite and positive.
    distance_resolution: float = 1e-6
    #: Coordinates whose junction continuity is checked modulo 2 pi.
    continuous_revolute_indices: Sequence[int] = ()
    #: Accepted for API parity; see docs/api.md for what it does here.
    parallelism: Parallelism = field(default_factory=Parallelism.Max)


@dataclass
class Result:
    """Result of one check."""

    verdict: Verdict = Verdict.kCertifiedFree
    #: The earliest violation, or the inconclusive witness; None iff certified.
    finding: Optional[Finding] = None
    #: Nodes visited by the adaptive subdivision; a cost measure.
    num_nodes: int = 0


def _validate(options: Options) -> None:
    margin, resolution = options.margin, options.distance_resolution
    if not (margin >= 0.0) or not math.isfinite(margin):
        raise RuntimeError(
            "ContinuousCollisionChecker: Options::margin must be a finite nonnegative distance; "
            f"got {margin}. Filter a pair out instead of giving it a negative margin.")
    if not (resolution > 0.0) or not math.isfinite(resolution):
        raise RuntimeError(
            "ContinuousCollisionChecker: Options::distance_resolution must be a finite positive "
            f"distance in meters; got {resolution}.")


class ContinuousCollisionChecker:
    """Certifies, rather than samples, that a trajectory is collision-free over its entire
    continuous time domain. See docs/api.md for the guarantee and the resolution contract."""

    def __init__(self, model=None, default_options: Optional[Options] = None):
        """Builds bounding spheres and topology tables, and runs the capability probe."""
        if model is None:
            raise RuntimeError(
                "ContinuousCollisionChecker: the model is null; supply a RobotDiagram whose "
                "plant is finalized.")
        if not model.plant().is_finalized():
            raise RuntimeError(
                "ContinuousCollisionChecker: the plant is not finalized; call "
                "MultibodyPlant::Finalize() (or RobotDiagramBuilder::Build()) first.")
        options = Options() if default_options is None else copy.copy(default_options)
        _validate(options)
        self._model = model
        self._default_options = options
        self._engine = KinematicsEngine(model)
        self._kinematics = BatchKinematics(self._engine)
        self._oracle = DistanceOracle(model, self._engine, self._kinematics)

    def model(self):
        """The RobotDiagram the checker was constructed with."""
        return self._model

    def CheckTrajectory(self, trajectory, options: Optional[Options] = None) -> Result:
        """Certifies a BezierCurve, BsplineTrajectory, PiecewisePolynomial, or a
        CompositeTrajectory of those."""
        options = self._resolve(options)
        path = PiecewiseBezierPath.from_trajectory(trajectory, options.continuous_revolute_indices)
        return self._check(path, options)

    def CheckPath(self, waypoints, options: Optional[Options] = None) -> Result:
        """Certifies the piecewise-linear path through the waypoint columns
        (shape (num_positions, num_points)); segment k spans time [k, k+1]."""
        options = self._resolve(options)
        waypoints = np.asarray(waypoints, dtype=float)
        if waypoints.ndim == 1:
            waypoints = waypoints[:, None]
        expected = self._model.plant().num_positions()
        if waypoints.shape[0] != expected:
            raise RuntimeError(
                f"ContinuousCollisionChecker::CheckPath: the waypoint matrix has "
                f"{waypoints.shape[0]} rows but the plant has {expected} generalized positions "
                "(waypoints are columns).")
        return self._check(PiecewiseBezierPath.from_waypoints(waypoints), options)

    def CheckEdge(self, q1, q2, options: Optional[Options] = None) -> Result:
        """Certifies the straight configuration-space edge q1 -> q2 over time [0, 1]."""
        q1 = np.asarray(q1, dtype=float).reshape(-1)
        q2 = np.asarray(q2, dtype=float).reshape(-1)
        expected = self._model.plant().num_positions()
        if q1.size != expected or q2.size != expected:
            raise RuntimeError(
                f"ContinuousCollisionChecker::CheckEdge: the endpoints have sizes {q1.size} and "
                f"{q2.size} but the plant has {expected} generalized positions.")
        return self.CheckPath(np.column_stack([q1, q2]), options)

    def _resolve(self, options: Optional[Options]) -> Options:
        return self._default_options if options is None else options

    def _check(self, path: PiecewiseBezierPath, options: Options) -> Result:
        _validate(options)
        expected = self._model.plant().num_positions()
        if path.num_positions != expected:
            raise RuntimeError(
                f"ContinuousCollisionChecker: the trajectory has {path.num_positions} rows but "
                f"the plant has {expected} generalized positions.")
        oracle = self._oracle
        table = self._engine.compute_motion_bound_table(
            path.global_lower, path.global_upper, path.constant_coordinates,
            oracle.body_a, oracle.body_b)
        lease = oracle.acquire()
        try:
            certifier = Certifier(oracle, self._kinematics, table, path, float(options.margin),
                                  float(options.distance_resolution), lease)
            verdict, witness, num_nodes = certifier.run()
            if witness is None:
                return Result(Verdict.kCertifiedFree, None, num_nodes)
            _, pa, pb = oracle.signed_distance(lease, witness.q, witness.pair)
        finally:
            oracle.release(lease)
        from pydrake.multibody.tree import BodyIndex

        p = witness.pair
        finding = Finding(time=witness.time, q=witness.q, geometry_a=oracle.geometry_a[p],
                          geometry_b=oracle.geometry_b[p], body_a=BodyIndex(int(oracle.body_a[p])),
                          body_b=BodyIndex(int(oracle.body_b[p])), distance=witness.distance,
                          nearest_a_W=pa, nearest_b_W=pb)
        kind = Verdict.kViolationFound if verdict == "violation" else Verdict.kInconclusive
        return Result(kind, finding, num_nodes)
