"""Replay the soundness fuzz's 200 random worlds and trajectories on the fork and the package.

The brick corpus has spheres and boxes only; these worlds add capsules, cylinders, ellipsoids,
convex tetrahedra, half-space floors, prismatic joints, B-splines and polynomials, and so
exercise Drake's narrowphase route and the half-space route against the fork.

    .venv/bin/python benchmarks/fuzz_referee.py make                    # margins -> fuzz_cases.json
    .venv/bin/python benchmarks/fuzz_referee.py run --impl package --out out/fuzz_package.jsonl
    <fork venv>/bin/python benchmarks/fuzz_referee.py run --impl fork --out out/fuzz_fork.jsonl
    .venv/bin/python benchmarks/fuzz_referee.py compare out/fuzz_fork.jsonl out/fuzz_package.jsonl
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tests"))
import fuzz_worlds as fw  # noqa: E402

CASES = HERE / "fuzz_cases.json"


def make(args) -> int:
    """Margins and resolutions exactly as tests/test_soundness_fuzz.py picks them."""
    import helpers as h
    from continuous_collision.bezier_path import PiecewiseBezierPath

    cases = []
    for i in range(args.cases):
        world, (kind, order, points) = fw.case(i)
        margin = 0.0 if i % 2 == 0 else 0.01
        if i % 5 == 3:
            model = fw.build_world(world)
            path = PiecewiseBezierPath.from_trajectory(fw.build_trajectory(kind, order, points))
            lowest = h.scan(h.ReferenceOracle(model), path, 2000, 0.0, reach=0.6)[0]
            if 0.01 < lowest < 0.5:
                margin = lowest
        cases.append({"case": i, "margin": margin, "resolution": 1e-3 if i % 4 == 1 else 1e-6})
    CASES.write_text(json.dumps(cases, indent=0))
    return 0


def run(args) -> int:
    if args.impl == "fork":
        from pydrake.planning import continuous_collision as api
    else:
        import continuous_collision as api
    from pydrake.common import Parallelism

    with open(args.out, "w") as fh:
        for c in json.loads(CASES.read_text()):
            world, (kind, order, points) = fw.case(c["case"])
            model = fw.build_world(world)
            inspector = model.scene_graph().model_inspector()
            options = api.Options()
            options.margin, options.distance_resolution = c["margin"], c["resolution"]
            options.parallelism = Parallelism(1)
            checker = api.ContinuousCollisionChecker(model=model, default_options=options)
            trajectory = fw.build_trajectory(kind, order, points)
            tick = time.perf_counter()
            r = checker.CheckTrajectory(trajectory)
            wall = time.perf_counter() - tick
            f = r.finding
            fh.write(json.dumps({**c, "kind": kind, "verdict": r.verdict.name, "nodes": int(r.num_nodes),
                                 "wall_s": wall,
                                 "finding": None if f is None else {
                                     "time": float(f.time), "distance": float(f.distance),
                                     "q": np.asarray(f.q, float).tolist(),
                                     "pair": sorted([inspector.GetName(f.geometry_a), inspector.GetName(f.geometry_b)])}})
                     + "\n")
    return 0


def compare(args) -> int:
    fork = [json.loads(l) for l in open(args.fork)]
    pkg = [json.loads(l) for l in open(args.package)]
    assert [r["case"] for r in fork] == [r["case"] for r in pkg]
    same_verdict = sum(a["verdict"] == b["verdict"] for a, b in zip(fork, pkg))
    certified = [(a, b) for a, b in zip(fork, pkg) if a["verdict"] == "kCertifiedFree"]
    refused = [(a, b) for a, b in zip(fork, pkg) if a["verdict"] != "kCertifiedFree" and a["verdict"] == b["verdict"]]
    same_witness = sum(a["finding"]["pair"] == b["finding"]["pair"] and abs(a["finding"]["time"] - b["finding"]["time"]) < 1e-9
                       for a, b in refused)
    dq = max([np.abs(np.subtract(a["finding"]["q"], b["finding"]["q"])).max() for a, b in refused], default=0.0)
    dd = max([abs(a["finding"]["distance"] - b["finding"]["distance"]) for a, b in refused], default=0.0)
    counts = {v: (sum(r["verdict"] == v for r in fork), sum(r["verdict"] == v for r in pkg))
              for v in ("kCertifiedFree", "kViolationFound", "kInconclusive")}
    ratio = np.array([b["wall_s"] / a["wall_s"] for a, b in zip(fork, pkg)])
    print(f"cases {len(fork)}; verdicts agree {same_verdict}; counts fork/package {counts}")
    print(f"certified with equal num_nodes {sum(a['nodes'] == b['nodes'] for a, b in certified)} / {len(certified)}")
    print(f"refused with the same witness pair and time {same_witness} / {len(refused)}; "
          f"max |dq| {dq:.1e}, max |d distance| {dd:.1e}")
    print(f"wall package/fork (1 thread): median {np.median(ratio):.2f}, p95 {np.percentile(ratio, 95):.2f}")
    for a, b in zip(fork, pkg):
        if a["verdict"] != b["verdict"] or (a["verdict"] == "kCertifiedFree" and a["nodes"] != b["nodes"]):
            print("DIFF", a["case"], a["verdict"], b["verdict"], a["nodes"], b["nodes"])
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("make")
    m.add_argument("--cases", type=int, default=200)
    r = sub.add_parser("run")
    r.add_argument("--impl", choices=["fork", "package"], required=True)
    r.add_argument("--out", required=True)
    c = sub.add_parser("compare")
    c.add_argument("fork")
    c.add_argument("package")
    args = ap.parse_args(argv)
    return {"make": make, "run": run, "compare": compare}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
