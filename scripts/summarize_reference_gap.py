#!/usr/bin/env python3
"""Summarize eda_diagnose_reference_gap output for one case: how a saved Guide's
length exceeds the official reference, split into planar / ordinary via / TSV
parts, by net class, by layer, and by how concentrated the excess is. Offline."""
import csv
import json
import sys
from collections import defaultdict


def main(d):
    totals = json.load(open(f"{d}/totals.json"))
    nets = list(csv.DictReader(open(f"{d}/nets.csv")))
    pitch = totals["pitch"]
    f = lambda r, k: float(r[k])
    L = sum(f(r, "L") for r in nets)
    R = sum(f(r, "ref") for r in nets)
    parts = {
        "planar": (sum(f(r, "planar") for r in nets), sum(f(r, "ref_planar") for r in nets)),
        "via": (sum(f(r, "via_L") for r in nets), sum(f(r, "ref_vias") * 3 * pitch for r in nets)),
        "tsv": (sum(f(r, "tsv_L") for r in nets), sum(f(r, "ref_tsv") for r in nets)),
    }
    E = L - R
    out = {
        "L": L, "reference": R, "ratio": L / R, "excess": E,
        "checks": {k: totals[k] for k in ("reference_split_mismatch_nets", "per_layer_planar_check", "connection_check")},
        "parts": {k: {"ours": a, "reference": b, "excess": a - b, "excess_over_R": (a - b) / R,
                      "share_of_excess": (a - b) / E, "ours_over_ref": a / b if b else None}
                  for k, (a, b) in parts.items()},
        "vias_per_net": {"ours": sum(f(r, "vias") for r in nets) / len(nets),
                         "reference": sum(f(r, "ref_vias") for r in nets) / len(nets)},
    }
    skipped = [r for r in nets if r["ref_skipped"] == "1"]
    out["reference_skipped_degree_over_150"] = {"nets": len(skipped), "L": sum(f(r, "L") for r in skipped),
                                                "share_of_excess": sum(f(r, "L") for r in skipped) / E}
    cls = defaultdict(list)
    for r in nets:
        cls[r["class"]].append(r)
    table = {}
    for k, rs in sorted(cls.items()):
        l = sum(f(r, "L") for r in rs)
        ref = sum(f(r, "ref") for r in rs)
        pl = sum(f(r, "planar") for r in rs)
        rpl = sum(f(r, "ref_planar") for r in rs)
        v = sum(f(r, "via_L") for r in rs)
        rv = sum(f(r, "ref_vias") * 3 * pitch for r in rs)
        t = sum(f(r, "tsv_L") for r in rs)
        rt = sum(f(r, "ref_tsv") for r in rs)
        hp = sum(f(r, "hpwl") for r in rs)
        table[k] = {
            "nets": len(rs), "L_share": l / L, "ratio": l / ref if ref else None,
            "planar_over_ref_planar": pl / rpl if rpl else None,
            "planar_over_hpwl": pl / hp if hp else None,
            "vias_per_net": sum(f(r, "vias") for r in rs) / len(rs),
            "ref_vias_per_net": sum(f(r, "ref_vias") for r in rs) / len(rs),
            "share_of_excess": (l - ref) / E,
            "excess_planar_share": (pl - rpl) / E, "excess_via_share": (v - rv) / E, "excess_tsv_share": (t - rt) / E,
            "mean_guide_layers": sum(f(r, "guide_layers") for r in rs) / len(rs),
        }
    out["classes"] = table
    ex = sorted((f(r, "L") - f(r, "ref") for r in nets), reverse=True)
    n = len(ex)
    out["excess_concentration"] = {f"top_{p}pct_nets": sum(ex[:max(1, n * p // 100)]) / E for p in (1, 5, 10, 25, 50)}
    out["nets_shorter_than_reference"] = sum(1 for x in ex if x < 0)
    ratios = sorted(f(r, "L") / f(r, "ref") for r in nets if f(r, "ref") > 0)
    out["per_net_ratio_percentiles"] = {str(p): ratios[min(len(ratios) - 1, len(ratios) * p // 100)] for p in (10, 25, 50, 75, 90, 99)}
    hist = defaultdict(float)
    for r in nets:
        hist[int(r["guide_layer_max_in_die"])] += f(r, "L") / L
    out["L_share_by_highest_layer_used_in_die"] = dict(sorted(hist.items()))
    out["planar_by_layer_share"] = {k: v / max(1e-9, totals["per_layer_planar_check"]["recomputed"])
                                    for k, v in totals["planar_by_layer"].items() if v}
    out["connections_by_lower_layer"] = {k: v for k, v in totals["connections_by_lower_layer"].items() if v}
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main(sys.argv[1])
