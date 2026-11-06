#!/usr/bin/env python3
"""Summarize eda_diagnose_two_tree_bound output for one case. Offline.

Per net class: the Guide's wire length per die (covered cells minus straight
runs) against a FLUTE tree on that die's snapped pins plus the TSV, and for
cross-die nets how much longer the two per-die FLUTE trees are than one FLUTE
tree on all pins. Gaps are expressed as a share of the case's total length L,
i.e. the most a shorter tree could remove if blockages and congestion allowed.
"""
import csv
import gzip
import json
import os
import sys
from collections import defaultdict


def rows(d):
    path = f"{d}/nets.csv"
    if not os.path.exists(path):
        path += ".gz"
    with (gzip.open(path, "rt") if path.endswith(".gz") else open(path)) as fh:
        return list(csv.DictReader(fh))


def main(d):
    totals = json.load(open(f"{d}/totals.json"))
    nets = rows(d)
    f = lambda r, k: float(r[k])
    L = sum(f(r, "L") for r in nets)
    # A die counts when it has pins; a die whose FLUTE was skipped (>150 points) is left out.
    def usable(r, s):
        return int(r[f"{s}_points"]) > 0 and r[f"{s}_skipped"] == "0"

    def agg(rs):
        a = defaultdict(float)
        a["nets"] = len(rs)
        for r in rs:
            a["L"] += f(r, "L")
            a["planar"] += f(r, "planar")
            a["via_L"] += f(r, "via_L")
            for s in ("b", "t"):
                if int(r[f"{s}_points"]) == 0:
                    continue
                if not usable(r, s):
                    a["skipped_dies"] += 1
                    continue
                a["edge"] += f(r, f"{s}_edge")
                a["flute"] += f(r, f"{s}_flute")
                a["runs"] += f(r, f"{s}_runs")
                a["cells"] += f(r, f"{s}_cells")
                a["die_planar"] += f(r, f"{s}_planar")
                a["vias"] += f(r, f"{s}_vias")
                a[f"{s}_edge"] += f(r, f"{s}_edge")
                a[f"{s}_flute"] += f(r, f"{s}_flute")
            if r["cross"] == "1" and r["one_tree_skipped"] == "0" and usable(r, "b") and usable(r, "t"):
                a["one_tree"] += f(r, "flute_one_tree")
                a["two_tree_paired"] += f(r, "b_flute") + f(r, "t_flute")
        gap = a["edge"] - a["flute"]
        out = {
            "nets": int(a["nets"]),
            "share_of_L": a["L"] / L,
            "edge_over_flute": a["edge"] / a["flute"] if a["flute"] else None,
            "edge_minus_flute_over_L": gap / L,
            "runs_per_net": a["runs"] / a["nets"],
            "in_die_vias_per_net": a["vias"] / a["nets"],
            "run_end_cells_over_L": (a["die_planar"] - a["edge"]) / L,
            "skipped_dies": int(a["skipped_dies"]),
        }
        if a["one_tree"]:
            out["two_trees_over_one_tree"] = a["two_tree_paired"] / a["one_tree"]
        if a["b_flute"] and a["t_flute"]:
            out["bottom_edge_over_flute"] = a["b_edge"] / a["b_flute"]
            out["top_edge_over_flute"] = a["t_edge"] / a["t_flute"]
        return out, gap

    cls = defaultdict(list)
    for r in nets:
        cls[r["class"]].append(r)
    table = {}
    for k, rs in sorted(cls.items()):
        table[k], _ = agg(rs)
    overall, gap_all = agg(nets)
    cross, gap_cross = agg([r for r in nets if r["cross"] == "1"])
    same, gap_same = agg([r for r in nets if r["cross"] == "0"])
    # How concentrated the per-net gap is.
    per = sorted(((sum(f(r, f"{s}_edge") - f(r, f"{s}_flute") for s in ("b", "t") if usable(r, s)), r) for r in nets),
                 key=lambda x: -x[0])
    pos = sum(g for g, _ in per if g > 0)
    top10 = sum(g for g, _ in per[: max(1, len(per) // 10)] if g > 0)
    neg = [g for g, _ in per if g < 0]
    out = {
        "scope": "offline read-only; FLUTE ignores blockages and congestion, so gaps are upper bounds",
        "L": L,
        "reference": totals["reference"],
        "checks": {k: totals[k] for k in ("planar_check", "via_check", "cross_nets_without_one_tsv")},
        "overall": overall,
        "cross_die": cross,
        "same_die": same,
        "positive_gap_top10pct_share": top10 / pos if pos else None,
        "nets_shorter_than_flute": len(neg),
        "shorter_than_flute_sum_over_L": sum(neg) / L,
        "classes": table,
    }
    json.dump(out, sys.stdout, indent=1, sort_keys=True)
    print()


if __name__ == "__main__":
    main(sys.argv[1])
