"""Run the 436-curve brick corpus through one checker implementation and write JSONL.

    .venv/bin/python benchmarks/corpus.py --impl package --out benchmarks/out/package.jsonl
    <fork venv>/bin/python benchmarks/corpus.py --impl fork --threads 1 --out benchmarks/out/fork_t1.jsonl

The same file runs in both interpreters: pydrake and numpy only, plus the implementation under
test. Per scene description, one checker per distinct margin over exactly the pairs carrying it
(the fork's Options.margin is one number), Options.distance_resolution = 1e-5; a curve is
certified iff every checker certifies it, and its wall time is the sum over checkers.
Adapted from bimanual-franka-cspace-experiments/experiments/bricks/ccd_bench/ccd.py.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drake_scene as ds  # noqa: E402

CORPUS = Path("/home/peter/gitcspace/bfce-drake-ccd/experiments/bricks/out/ccd_bench")
RESOLUTION = 1e-5


def load_api(impl: str):
    if impl == "fork":
        from pydrake.planning import continuous_collision as api
    else:
        import continuous_collision as api
    return api


def trajectory(npz, j: int):
    from pydrake.trajectories import BezierCurve, CompositeTrajectory

    segs = []
    k = 0
    while f"c{j}_s{k}" in npz:
        P = np.asarray(npz[f"c{j}_s{k}"], float)  # (n_cp, dim)
        t0, t1 = np.asarray(npz[f"c{j}_t{k}"], float)
        segs.append(BezierCurve(float(t0), float(t1), P.T.copy()))
        k += 1
    return CompositeTrajectory(segs) if len(segs) > 1 else segs[0]


class Certifier:
    """One checker per distinct margin of a scene description."""

    def __init__(self, api, spec: dict, threads):
        from pydrake.common import Parallelism

        bars = np.round(np.asarray(spec["bars"], float), 9)
        self.checkers = []
        for bar in sorted(set(bars.tolist())):
            scene = ds.build(spec, bars == bar)
            options = api.Options()
            options.margin = float(bar)
            options.distance_resolution = RESOLUTION
            options.parallelism = Parallelism.Max() if threads is None else Parallelism(threads)
            checker = api.ContinuousCollisionChecker(model=scene.model, default_options=options)
            self.checkers.append((float(bar), scene, checker))

    def check(self, api, traj) -> dict:
        out = {"walls": [], "nodes": [], "verdicts": [], "findings": []}
        for bar, scene, checker in self.checkers:
            tick = time.perf_counter()
            r = checker.CheckTrajectory(traj)
            out["walls"].append(time.perf_counter() - tick)
            out["nodes"].append(int(r.num_nodes))
            v = {api.Verdict.kCertifiedFree: "certified", api.Verdict.kViolationFound: "violation",
                 api.Verdict.kInconclusive: "inconclusive"}[r.verdict]
            out["verdicts"].append(v)
            f = r.finding
            if f is None:
                out["findings"].append(None)
            else:
                i = scene.pair_index.get(frozenset((f.geometry_a, f.geometry_b)))
                out["findings"].append({"time": float(f.time), "distance": float(f.distance),
                                        "q": np.asarray(f.q, float).tolist(), "pair_index": i})
        verdicts = out["verdicts"]
        out["verdict"] = ("violation" if "violation" in verdicts else
                          "inconclusive" if "inconclusive" in verdicts else "certified")
        out["wall_s"] = float(sum(out["walls"]))
        return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--impl", choices=["fork", "package"], required=True)
    ap.add_argument("--threads", type=int, default=None, help="Parallelism(threads); default Max()")
    ap.add_argument("--repeats", type=int, default=3, help="timed repetitions after one warmup")
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=None, help="first N legs only (smoke runs)")
    args = ap.parse_args(argv)
    api = load_api(args.impl)
    import pydrake

    rows = [json.loads(l) for l in (CORPUS / "legs.jsonl").read_text().splitlines() if l]
    if args.limit:
        rows = rows[:args.limit]
    certifiers = {}
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as fh:
        for row in rows:
            key = row["spec"]
            if key not in certifiers:
                spec = json.loads((CORPUS / "specs" / f"{key}.json").read_text())
                tick = time.perf_counter()
                certifiers[key] = Certifier(api, spec, args.threads)
                certifiers[key].setup_s = time.perf_counter() - tick
            cert = certifiers[key]
            npz = np.load(CORPUS / "curves" / f"{row['leg']}_{row['cfg']}.npz")
            for j, cand in enumerate(row["walk"]["candidates"]):
                traj = trajectory(npz, j)
                r = cert.check(api, traj)  # warmup, and the recorded verdict / finding
                walls = [cert.check(api, traj)["wall_s"] for _ in range(args.repeats)]
                rec = {"leg": row["leg"], "cfg": row["cfg"], "candidate": cand["name"], "spec": key,
                       "impl": args.impl, "threads": args.threads, "pydrake": pydrake.__file__,
                       **r, "wall_median_s": statistics.median(walls) if walls else r["wall_s"],
                       "wall_min_s": min(walls) if walls else r["wall_s"], "setup_s": cert.setup_s}
                fh.write(json.dumps(rec) + "\n")
                fh.flush()
            print(f"{row['leg']:24s} {row['cfg']} {len(row['walk']['candidates'])} curves", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
