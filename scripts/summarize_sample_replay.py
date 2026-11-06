#!/usr/bin/env python3
"""Summarize eda_replay_sample_nets output: per class and search mode, how much a
one-net reroute changes length and score, and a rough whole-design projection
that scales each class's sampled mean by its net count. The projection keeps
only reroutes that raise the score (as an accept-if-better pass would) and
ignores interactions between nets, so it is an optimistic offline estimate."""
import csv
import json
import sys
from collections import defaultdict


def main(replay_dir):
    classes = json.load(open(f"{replay_dir}/classes.json"))
    L, R = classes["L"], classes["reference"]
    rows = defaultdict(list)
    failures = defaultdict(lambda: defaultdict(int))
    for r in csv.DictReader(open(f"{replay_dir}/replay.csv")):
        if r["status"] != "ok":
            failures[(r["class"], r["mode"])][r["status"][:60]] += 1
            continue
        rows[(r["class"], r["mode"])].append(r)
    modes = sorted({m for _, m in rows})
    out = {"L": L, "reference": R, "W": 100 * R / L, "classes": {}, "projection": {}}
    for mode in modes:
        total_dl = 0.0
        total_dq = 0.0
        for name, info in classes["classes"].items():
            sample = rows.get((name, mode), [])
            if not sample:
                continue
            n = len(sample)
            dl = [float(r["dL"]) for r in sample]
            ds = [float(r["dS"]) for r in sample]
            dq = [float(r["dQ"]) for r in sample]
            acc = [i for i in range(n) if ds[i] > 1e-12]
            acc_dl = sum(dl[i] for i in acc) / n
            acc_dq = sum(dq[i] for i in acc) / n
            total_dl += acc_dl * info["nets"]
            total_dq += acc_dq * info["nets"]
            entry = out["classes"].setdefault(name, {"nets": info["nets"], "L_share": info["L"] / L})
            entry[mode] = {
                "sampled_ok": n,
                "failed": dict(failures.get((name, mode), {})),
                "mean_dL_all": sum(dl) / n,
                "mean_dL_over_net_L": sum(dl) / max(1e-9, sum(float(r["old_L"]) for r in sample)),
                "share_score_improves": len(acc) / n,
                "mean_dL_accepted_per_sampled_net": acc_dl,
                "mean_d_planar_accepted": sum(float(sample[i]["d_planar"]) for i in acc) / n,
                "mean_d_via_accepted": sum(float(sample[i]["d_via"]) for i in acc) / n,
                "new_over_cells_total": sum(int(r["new_over_cells"] or 0) for r in sample),
                "old_over_cells_total": sum(int(r["old_over_cells"] or 0) for r in sample),
                "mean_seconds": sum(float(r["seconds"]) for r in sample) / n,
                "max_seconds": max(float(r["seconds"]) for r in sample),
            }
        new_l = L + total_dl
        out["projection"][mode] = {
            "dL": total_dl,
            "dL_over_L": total_dl / L,
            "W_after": 100 * R / new_l,
            "dS_from_W": 0.45 * 100 * (R / new_l - R / L),
            "sum_accepted_dQ": total_dq,
        }
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main(sys.argv[1])
