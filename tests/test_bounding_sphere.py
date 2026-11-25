"""Bounding-sphere containment (port of bounding_sphere_test.cc): for every supported shape, at
many random poses, every sampled surface point lies inside the reported sphere. An
under-bounding formula makes lambda unsound with no other symptom."""

import numpy as np
import pytest
from pydrake.geometry import (Box, Capsule, Convex, Cylinder, Ellipsoid, HalfSpace, InMemoryMesh,
                              Mesh, MeshcatCone, Sphere)
from pydrake.common import MemoryFile
from pydrake.math import RigidTransform

from continuous_collision.motion_bound import compute_bounding_sphere, hull_vertices

import helpers as h

NUM_POSES, NUM_SAMPLES = 100, 1000
RELATIVE_SLACK = 1e-12


def check_containment(rng, shape, sampler, translation_scale):
    for _ in range(NUM_POSES):
        X_LG = h.random_transform(rng, translation_scale)
        sphere = compute_bounding_sphere(shape, X_LG)
        assert np.isfinite(sphere.radius) and sphere.radius >= 0 and np.isfinite(sphere.center_L).all()
        origin_radius = np.linalg.norm(sphere.center_L) + sphere.radius
        limit = sphere.radius + RELATIVE_SLACK * origin_radius
        P = X_LG.multiply(h.sample_surface(rng, NUM_SAMPLES, sampler))
        assert np.linalg.norm(P - sphere.center_L[:, None], axis=0).max() <= limit
        # The reach chain consumes |c_L| + rho as an origin-centered radius.
        P = X_LG.multiply(h.sample_surface(rng, 32, sampler))
        assert np.linalg.norm(P, axis=0).max() <= origin_radius * (1.0 + RELATIVE_SLACK)


def primitive_cases():
    for r in (1e-4, 0.05, 1.0, 7.5):
        yield f"Sphere({r})", Sphere(r), lambda g, r=r: h.sample_sphere(g, r)
    for s in ((1.0, 1.0, 1.0), (0.01, 2.0, 0.3), (5.0, 0.002, 0.002), (0.4, 0.7, 1.9)):
        yield f"Box{s}", Box(*s), lambda g, s=s: h.sample_box(g, s)
    for r, length in ((0.1, 1.0), (1.0, 0.01), (0.001, 3.0), (0.5, 0.5)):
        yield f"Capsule({r}, {length})", Capsule(r, length), lambda g, r=r, l=length: h.sample_capsule(g, r, l)
    for r, length in ((0.1, 1.0), (2.0, 0.01), (0.002, 4.0), (0.5, 0.5)):
        yield f"Cylinder({r}, {length})", Cylinder(r, length), lambda g, r=r, l=length: h.sample_cylinder(g, r, l)
    for e in ((1.0, 1.0, 1.0), (0.01, 0.5, 2.0), (3.0, 0.001, 0.001), (0.2, 0.9, 0.05)):
        yield f"Ellipsoid{e}", Ellipsoid(*e), lambda g, e=e: h.sample_ellipsoid(g, e)


def test_primitives_contain_their_surface():
    rng = np.random.default_rng(0x5EED0001)
    for _, shape, sampler in primitive_cases():
        check_containment(rng, shape, sampler, 2.0)


def vertex_sets(rng):
    yield "generic", np.stack([rng.uniform(0.2, 1.0) * h.random_unit_vector(rng) for _ in range(30)], axis=1)
    corners = np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)]).T
    yield "redundant", np.hstack((corners, rng.uniform(-0.4, 0.4, (3, 40))))
    yield "planar", np.vstack((rng.uniform(-1, 1, (2, 16)), np.zeros((1, 16))))
    yield "thin-slab", np.vstack((rng.uniform(-1, 1, (2, 24)), rng.uniform(-5e-7, 5e-7, (1, 24))))
    yield "sliver", np.vstack((rng.uniform(-2, 2, (1, 20)), rng.uniform(-1e-5, 1e-5, (2, 20))))
    yield "tiny", np.stack([1e-4 * h.random_unit_vector(rng) for _ in range(20)], axis=1)


def hull_sampler(V):
    return lambda g: V[:, g.integers(0, V.shape[1])]


def test_convex_contains_hull_vertices():
    rng = np.random.default_rng(0x5EED0006)
    checked = 0
    for name, V in vertex_sets(rng):
        for scale in ((1.0, 1.0, 1.0), (2.0, 0.5, 1.3)):
            shape = Convex(V, name, np.array(scale))
            try:
                hull = hull_vertices(shape)
            except Exception:  # Drake refuses degenerate hulls; the checker inherits that.
                continue
            assert hull.shape[1] > 0
            check_containment(rng, shape, hull_sampler(hull), 1.0)
            checked += 1
    assert checked >= 4


def l_shaped_obj() -> InMemoryMesh:
    profile = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.3), (0.3, 0.3), (0.3, 1.2), (0.0, 1.2)]
    lines = [f"v {x} {y} {z}" for z in (-0.25, 0.25) for x, y in profile]
    for base in (1, 7):
        lines += [f"f {base} {base + i} {base + i + 1}" for i in range(1, 5)]
    for i in range(6):
        a, b = 1 + i, 1 + (i + 1) % 6
        lines += [f"f {a} {b} {b + 6}", f"f {a} {b + 6} {a + 6}"]
    return InMemoryMesh(mesh_file=MemoryFile("\n".join(lines) + "\n", ".obj", "ccd_l_prism.obj"))


def test_mesh_contains_hull_vertices():
    rng = np.random.default_rng(0x5EED0007)
    for scale in ((1.0, 1.0, 1.0), (0.4, 1.7, 1.0)):
        shape = Mesh(l_shaped_obj(), np.array(scale))
        hull = hull_vertices(shape)
        assert hull.shape[1] > 3
        check_containment(rng, shape, hull_sampler(hull), 1.0)


def test_throws_on_unsupported_shapes():
    with pytest.raises(RuntimeError, match="HalfSpace"):
        compute_bounding_sphere(HalfSpace(), RigidTransform())
    with pytest.raises(RuntimeError):
        compute_bounding_sphere(MeshcatCone(1.0, 0.5, 0.25), RigidTransform())
