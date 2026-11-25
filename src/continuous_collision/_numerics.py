"""Numerical policy, shape classes and the per-pair oracle tolerance.

Design note: every decision of the certifier is one of two inequalities,

    certified:          phi_hat - tau_p - Delta_p > margin + EPS
    definite violation: phi_hat + tau_p < margin

so the whole soundness story lives in the constants below and in the two functions that
encode those inequalities. tau_p transcribes Drake's documented signed-distance accuracy
(query_object.h, Table 4); a shape the checker cannot classify is charged the worst value.
"""

from __future__ import annotations

import enum

import numpy as np
from pydrake.geometry import Box, Capsule, Convex, Cylinder, Ellipsoid, HalfSpace, Mesh, Sphere

#: Junction C0 tolerance per coordinate, and the control-box width below which a
#: coordinate counts as constant (the carve-out).
CONTINUITY_TOLERANCE = 1e-7
#: The oracle's baseline accuracy contract in meters (tau never drops below it).
QUERY_TOLERANCE = 1e-6
#: Slack that swallows floating-point noise in the bound arithmetic.
NUMERICAL_SLACK = 1e-9
#: Highest polynomial degree accepted by the monomial-to-Bernstein conversion.
MAX_CONVERSION_DEGREE = 10


def is_certified(phi_hat, tau, motion_bound, threshold, slack=NUMERICAL_SLACK):
    """True iff the pair keeps clearance above `threshold` on the whole node."""
    return phi_hat - tau - motion_bound > threshold + slack


def is_definite_violation(phi_hat, tau, threshold):
    """True iff the (on-trajectory) configuration is definitely below `threshold`."""
    return phi_hat + tau < threshold


class ShapeClass(enum.IntEnum):
    SPHERE = 0
    BOX = 1
    CAPSULE = 2
    CYLINDER = 3
    ELLIPSOID = 4
    CONVEX = 5
    MESH = 6
    HALFSPACE = 7
    UNSUPPORTED = 8


_CLASS_OF = (
    (Sphere, ShapeClass.SPHERE),
    (Box, ShapeClass.BOX),
    (Capsule, ShapeClass.CAPSULE),
    (Cylinder, ShapeClass.CYLINDER),
    (Ellipsoid, ShapeClass.ELLIPSOID),
    (Convex, ShapeClass.CONVEX),
    (Mesh, ShapeClass.MESH),
    (HalfSpace, ShapeClass.HALFSPACE),
)


def classify(shape) -> ShapeClass:
    """The shape class of a pydrake Shape (exact type match; subclasses are unsupported)."""
    for cls, klass in _CLASS_OF:
        if type(shape) is cls:
            return klass
    return ShapeClass.UNSUPPORTED


#: Worst documented accuracy, charged to anything unclassified.
WORST_DOCUMENTED_ACCURACY = 5e-5


def _accuracy_table() -> np.ndarray:
    t = np.full((len(ShapeClass), len(ShapeClass)), WORST_DOCUMENTED_ACCURACY)
    S = ShapeClass

    def put(a, b, value):
        t[a, b] = t[b, a] = value

    put(S.SPHERE, S.SPHERE, 6e-15)
    put(S.SPHERE, S.BOX, 4e-15)
    put(S.SPHERE, S.CAPSULE, 6e-15)
    put(S.SPHERE, S.CYLINDER, 5e-15)
    put(S.SPHERE, S.ELLIPSOID, 4e-5)
    put(S.SPHERE, S.CONVEX, 3e-6)
    put(S.SPHERE, S.MESH, 3e-6)
    put(S.BOX, S.BOX, 4e-15)
    put(S.BOX, S.CAPSULE, 3e-6)
    put(S.BOX, S.CYLINDER, 6e-6)
    put(S.BOX, S.ELLIPSOID, 9e-6)
    put(S.BOX, S.CONVEX, 3e-15)
    put(S.BOX, S.MESH, 3e-15)
    put(S.CAPSULE, S.CAPSULE, 2e-5)
    put(S.CAPSULE, S.CYLINDER, 1e-5)
    put(S.CAPSULE, S.ELLIPSOID, 5e-6)
    put(S.CAPSULE, S.CONVEX, 2e-5)
    put(S.CAPSULE, S.MESH, 2e-5)
    put(S.CYLINDER, S.CYLINDER, 2e-5)
    put(S.CYLINDER, S.ELLIPSOID, 5e-5)
    put(S.CYLINDER, S.CONVEX, 6e-6)
    put(S.CYLINDER, S.MESH, 6e-6)
    put(S.ELLIPSOID, S.ELLIPSOID, 2e-5)
    put(S.ELLIPSOID, S.CONVEX, 9e-6)
    put(S.ELLIPSOID, S.MESH, 9e-6)
    put(S.CONVEX, S.CONVEX, 3e-15)
    put(S.CONVEX, S.MESH, 3e-15)
    put(S.MESH, S.MESH, 3e-15)
    put(S.SPHERE, S.HALFSPACE, 3e-15)
    t.setflags(write=False)
    return t


#: Drake's documented ComputeSignedDistancePairClosestPoints accuracy per shape-class pair.
DOCUMENTED_ACCURACY = _accuracy_table()


def native_tolerance(class_a: ShapeClass, class_b: ShapeClass) -> float:
    """tau_p for a pair measured by Drake's narrowphase."""
    return max(QUERY_TOLERANCE, float(DOCUMENTED_ACCURACY[class_a, class_b]))
