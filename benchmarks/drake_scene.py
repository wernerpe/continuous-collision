"""Copied from bimanual-franka-cspace-experiments/experiments/bricks/ccd_bench/drake_scene.py (the csdecomp half dropped)."""

from __future__ import annotations

import re
from typing import Iterable, Optional

import numpy as np

SEP = "__"


def drake_name(name: str) -> str:
    return name.replace("::", SEP)


def sanitize(text: str) -> str:
    text = re.sub(r"<drake:collision_filter_group\b.*?</drake:collision_filter_group>", "", text,
                  flags=re.DOTALL)
    return text.replace("::", SEP)


# ----------------------------------------------------------------- either-venv half


def _shape_key(shape) -> tuple:
    from pydrake.geometry import Box, Sphere

    if isinstance(shape, Sphere):
        return ("sphere", round(shape.radius(), 6))
    if isinstance(shape, Box):
        return ("box",) + tuple(round(v, 6) for v in shape.size())
    raise ValueError(f"unexpected shape {type(shape).__name__} in a csdecomp description")


def _spec_key(g: dict) -> tuple:
    d = g["dims"]
    if g["type"] == "sphere":
        return ("sphere", round(d[0], 6))
    if g["type"] == "box":
        return ("box",) + tuple(round(v, 6) for v in d[:3])
    raise ValueError(f"unexpected csdecomp geometry type {g['type']!r}")


class DrakeScene:
    """A built model plus the maps between csdecomp's indices and Drake's ids."""

    def __init__(self, model, gids: list, pair_ids: list, pair_index: dict, spec: dict, keep: np.ndarray):
        self.model = model
        #: Drake GeometryId per csdecomp geometry index.
        self.gids = gids
        #: ``(gid_a, gid_b)`` per csdecomp pair index (all pairs, kept or not).
        self.pair_ids = pair_ids
        #: ``frozenset({gid_a, gid_b}) -> csdecomp pair index``.
        self.pair_index = pair_index
        self.spec = spec
        #: Which csdecomp pairs are candidates in this model.
        self.keep = keep

    @property
    def plant(self):
        return self.model.plant()

    @property
    def scene_graph(self):
        return self.model.scene_graph()

    def body_of_link(self, link: str):
        if link == "world":
            return self.plant.world_body()
        return self.plant.GetBodyByName(drake_name(link))


def build(spec: dict, keep: Optional[Iterable[bool]] = None, before_finalize=None,
          extra_filters=None) -> DrakeScene:
    """The model, with candidates = ``spec['pairs'][keep]`` exactly (all pairs when ``keep`` is None).

    ``before_finalize(plant)`` may add bodies (the movable-obstacle measurement); geometries it adds
    are left out of the filter surgery and keep Drake's own filters, plus whatever
    ``extra_filters(plant, scene_graph, gids)`` declares (``gids``: the csdecomp twins).  The
    equality check is over the twins' pairs only.
    """
    from pydrake.geometry import CollisionFilterDeclaration, GeometrySet, Role
    from pydrake.planning import RobotDiagramBuilder

    pairs = np.asarray(spec["pairs"], int)
    keep = np.ones(len(pairs), bool) if keep is None else np.asarray(list(keep), bool)
    builder = RobotDiagramBuilder(time_step=0.0)
    plant, scene_graph = builder.plant(), builder.scene_graph()
    builder.parser().AddModelsFromString(sanitize(spec["text"]), "urdf")
    if before_finalize is not None:
        before_finalize(plant)
    plant.Finalize()
    inspector = scene_graph.model_inspector()

    # csdecomp geometry index -> Drake GeometryId: same link, same primitive, same pose.
    by_frame: dict = {}
    gids = []
    for i, g in enumerate(spec["geoms"]):
        body = plant.world_body() if g["link"] == "world" else plant.GetBodyByName(drake_name(g["link"]))
        frame = plant.GetBodyFrameIdOrThrow(body.index())
        if frame not in by_frame:
            by_frame[frame] = list(inspector.GetGeometries(frame, Role.kProximity))
        X = np.asarray(g["X_L_G"], float)
        want = _spec_key(g)
        best, err = None, np.inf
        for gid in by_frame[frame]:
            if _shape_key(inspector.GetShape(gid)) != want:
                continue
            e = np.abs(inspector.GetPoseInFrame(gid).GetAsMatrix4() - X).max()
            if e < err:
                best, err = gid, e
        if best is None or err > 1e-5:
            raise RuntimeError(f"csdecomp geometry {i} ({g['link']}, {want}) has no Drake twin (pose err {err})")
        by_frame[frame].remove(best)
        gids.append(best)
    leftover = [gid for v in by_frame.values() for gid in v]
    if leftover:
        raise RuntimeError(f"{len(leftover)} Drake proximity geometries have no csdecomp twin")

    pair_ids = [(gids[a], gids[b]) for a, b in pairs]
    decl = CollisionFilterDeclaration().ExcludeWithin(GeometrySet(gids))
    for (a, b), k in zip(pair_ids, keep):
        if k:
            decl.AllowBetween(GeometrySet([a]), GeometrySet([b]))
    scene_graph.collision_filter_manager().Apply(decl)
    if extra_filters is not None:
        extra_filters(plant, scene_graph, gids)

    model = builder.Build()
    inspector = model.scene_graph().model_inspector()
    want = {frozenset(p) for p, k in zip(pair_ids, keep) if k}
    mine = set(gids)
    got = {frozenset(p) for p in inspector.GetCollisionCandidates() if p[0] in mine and p[1] in mine}
    if got != want:
        raise RuntimeError(f"Drake's candidates differ from csdecomp's pair list: {len(got - want)} extra, "
                           f"{len(want - got)} missing (of {len(want)})")
    pair_index = {frozenset(p): i for i, p in enumerate(pair_ids)}
    return DrakeScene(model, gids, pair_ids, pair_index, spec, keep)
