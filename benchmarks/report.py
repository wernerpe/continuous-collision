"""Compare corpus runs (benchmarks/corpus.py output) and write benchmarks/corpus_report.md.

    .venv/bin/python benchmarks/report.py --fork-t1 out/fork_t1.jsonl --fork-tmax out/fork_tmax.jsonl \
        --package-t1 out/package_t1.jsonl [--package-tmax out/package_tmax.jsonl] --out corpus_report.md
"""

from __future__ import annotations

import argparse
import json
import platform
from pathlib import Path

import numpy as np

CORPUS = Path("/home/peter/gitcspace/bfce-drake-ccd/experiments/bricks/out/ccd_bench")


def load(path):
    rows = [json.loads(l) for l in Path(path).read_text().splitlines() if l]
    return {(r["leg"], r["cfg"], r["candidate"]): r for r in rows}


def stats(values):
    v = np.asarray(values) * 1e3
    return f"{np.median(v):.1f}", f"{np.percentile(v, 95):.1f}", f"{v.max():.1f}"


def witness(r):
    """(bar index, time, pair) of every non-certified checker of a curve."""
    return [None if f is None else (round(f["time"], 9), f["pair_index"]) for f in r["findings"]]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fork-t1", required=True)
    ap.add_argument("--fork-tmax", required=True)
    ap.add_argument("--package-t1", required=True)
    ap.add_argument("--package-tmax")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    recorded = {(r["leg"], r["cfg"], r["candidate"]): r for r in
                (json.loads(l) for l in (CORPUS / "verdicts.jsonl").read_text().splitlines() if l)}
    f1, fm, p1 = load(args.fork_t1), load(args.fork_tmax), load(args.package_t1)
    pm = load(args.package_tmax) if args.package_tmax else None
    keys = sorted(recorded)
    assert set(keys) == set(f1) == set(fm) == set(p1), "the runs do not cover the same 436 curves"

    lines = ["# Corpus report: this package against the fork", ""]
    agree_recorded = sum(p1[k]["verdict"] == "certified" for k in keys if recorded[k]["fork_ccd_certified"]) + \
        sum(p1[k]["verdict"] != "certified" for k in keys if not recorded[k]["fork_ccd_certified"])
    agree_fork = sum(p1[k]["verdicts"] == f1[k]["verdicts"] for k in keys)
    agree_fork_m = sum(f1[k]["verdicts"] == fm[k]["verdicts"] for k in keys)
    same_witness = [k for k in keys if p1[k]["verdict"] != "certified" and witness(p1[k]) == witness(f1[k])]
    refused = [k for k in keys if f1[k]["verdict"] != "certified"]
    certified_checks = [(k, i) for k in keys for i, v in enumerate(f1[k]["verdicts"]) if v == "certified"]
    same_nodes_cert = sum(p1[k]["nodes"][i] == f1[k]["nodes"][i] for k, i in certified_checks)
    viol_checks = [(k, i) for k in keys for i, v in enumerate(f1[k]["verdicts"]) if v != "certified"]
    same_nodes_viol = sum(p1[k]["nodes"][i] == f1[k]["nodes"][i] for k, i in viol_checks)

    def count(run, v):
        return sum(run[k]["verdict"] == v for k in keys)

    lines += [
        f"{len(keys)} curves, {sum(len(f1[k]['verdicts']) for k in keys)} checks (one per distinct margin "
        "per curve), distance_resolution 1e-5 m.",
        "",
        "## Agreement",
        "",
        "| | count |",
        "|---|---|",
        f"| curve verdict = the recorded fork verdict (verdicts.jsonl) | {agree_recorded} / {len(keys)} |",
        f"| per-margin verdicts = the fork re-run on this machine (1 thread) | {agree_fork} / {len(keys)} |",
        f"| fork 1 thread = fork Parallelism.Max() (per-margin verdicts) | {agree_fork_m} / {len(keys)} |",
        f"| kCertifiedFree curves, fork / package | {count(f1, 'certified')} / {count(p1, 'certified')} |",
        f"| kViolationFound curves, fork / package | {count(f1, 'violation')} / {count(p1, 'violation')} |",
        f"| kInconclusive curves, fork / package | {count(f1, 'inconclusive')} / {count(p1, 'inconclusive')} |",
        f"| refused curves with the same witness pair and time (1e-9) | {len(same_witness)} / {len(refused)} |",
        f"| certified checks with the same num_nodes | {same_nodes_cert} / {len(certified_checks)} |",
        f"| refused checks with the same num_nodes | {same_nodes_viol} / {len(viol_checks)} |",
        "",
    ]
    diffs = [k for k in keys if p1[k]["verdicts"] != f1[k]["verdicts"]]
    if diffs:
        lines += ["### Verdict disagreements", ""]
        for k in diffs:
            lines.append(f"- {k}: fork {f1[k]['verdicts']} vs package {p1[k]['verdicts']}")
        lines.append("")
    wdiffs = [k for k in refused if k not in same_witness]
    if wdiffs:
        lines += ["### Witness differences", ""]
        for k in wdiffs:
            lines.append(f"- {k}: fork {witness(f1[k])} vs package {witness(p1[k])}")
        lines.append("")
    ndiff = [(k, i) for k, i in certified_checks if p1[k]["nodes"][i] != f1[k]["nodes"][i]]
    if ndiff:
        lines += ["### Node-count differences on certified checks", ""]
        for k, i in ndiff:
            lines.append(f"- {k} margin #{i}: fork {f1[k]['nodes'][i]} vs package {p1[k]['nodes'][i]}")
        lines.append("")

    def walls(run):
        return [run[k]["wall_median_s"] for k in keys]

    lines += ["## Wall time per curve", "",
              "Each curve: one untimed warmup, then the median of 5 timed runs; a curve's wall is the sum "
              "over its checkers.", "",
              "| build | threads | median (ms) | p95 (ms) | max (ms) |", "|---|---|---|---|---|"]
    rows = [("fork (pydrake.planning.continuous_collision)", "1", f1),
            ("fork (pydrake.planning.continuous_collision)", "Parallelism.Max()", fm),
            ("this package", "1", p1)]
    if pm:
        rows.append(("this package", "Parallelism.Max()", pm))
    for name, threads, run in rows:
        lines.append(f"| {name} | {threads} | " + " | ".join(stats(walls(run))) + " |")
    ratio = np.asarray(walls(p1)) / np.asarray(walls(f1))
    lines += ["", f"Per-curve ratio package / fork (both 1 thread): median {np.median(ratio):.2f}, "
                  f"p95 {np.percentile(ratio, 95):.2f}, max {ratio.max():.2f}.", ""]
    nodes = [sum(p1[k]["nodes"]) for k in keys]
    lines += [f"Nodes per curve (package): median {int(np.median(nodes))}, p95 {int(np.percentile(nodes, 95))}, "
              f"max {max(nodes)}.", "",
              f"Machine: {platform.processor() or platform.machine()}, {platform.system()} {platform.release()}; "
              f"fork pydrake {f1[keys[0]]['pydrake']}; package pydrake {p1[keys[0]]['pydrake']}.", ""]

    lines += ["## Per curve", "",
              "| leg | cfg | candidate | recorded | fork | package | fork nodes | package nodes | "
              "fork 1t ms | fork max ms | package ms |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for k in keys:
        rec = "certified" if recorded[k]["fork_ccd_certified"] else "refused"
        lines.append(f"| {k[0]} | {k[1]} | {k[2]} | {rec} | {f1[k]['verdict']} | {p1[k]['verdict']} | "
                     f"{'+'.join(map(str, f1[k]['nodes']))} | {'+'.join(map(str, p1[k]['nodes']))} | "
                     f"{f1[k]['wall_median_s'] * 1e3:.1f} | {fm[k]['wall_median_s'] * 1e3:.1f} | "
                     f"{p1[k]['wall_median_s'] * 1e3:.1f} |")
    Path(args.out).write_text("\n".join(lines) + "\n")
    print("\n".join(lines[:40]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
