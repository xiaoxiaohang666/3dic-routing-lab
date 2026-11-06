#!/usr/bin/env python3
"""Summarize eda_diagnose_net_wirelength CSV: where the Guide's length exceeds the official reference.

Read-only. Groups nets by pin count and cross-die status and splits the excess L - reference
into planar (Guide planar minus the reference FLUTE planar tree) and via/TSV (the rest).
For cross-die nets also reports the per-die FLUTE trees joined at the Guide's TSV, a comparison
value only, not an official quantity. Output JSON goes to stdout.
"""
import csv, gzip, json, sys

BINS = [(2, 2), (3, 4), (5, 8), (9, 16), (17, 32), (33, 150), (151, 10**9)]


def label(pins):
    for low, high in BINS:
        if low <= pins <= high:
            return f"{low}" if low == high else (f"{low}+" if high >= 10**9 else f"{low}-{high}")
    return "1"


def main(path):
    opener = gzip.open if path.endswith(".gz") else open
    groups, total = {}, {"nets": 0, "reference": 0.0, "L": 0.0}
    with opener(path, "rt") as stream:
        for row in csv.DictReader(stream):
            pins, cross = int(row["pins"]), int(row["cross"])
            ref, length = float(row["reference"]), float(row["L"])
            planar, via, tsv = float(row["planar"]), float(row["via"]), float(row["tsv"])
            flute = float(row["flute_wire"])
            key = f"{'cross' if cross else 'same'}:{label(pins)}"
            g = groups.setdefault(key, {"nets": 0, "reference": 0.0, "L": 0.0, "planar": 0.0, "via": 0.0,
                                        "tsv": 0.0, "flute_planar": 0.0, "per_die_wire": 0.0, "zero_reference": 0})
            g["nets"] += 1; g["reference"] += ref; g["L"] += length; g["planar"] += planar
            g["via"] += via; g["tsv"] += tsv; g["flute_planar"] += flute; g["per_die_wire"] += float(row["per_die_wire"])
            g["zero_reference"] += ref <= 0
            total["nets"] += 1; total["reference"] += ref; total["L"] += length
    excess_total = total["L"] - total["reference"]
    out = {"scope": "read-only per-net breakdown; offline, not an official result", "total": total,
           "ratio": total["L"] / total["reference"], "groups": {}}
    for key in sorted(groups, key=lambda k: -(groups[k]["L"] - groups[k]["reference"])):
        g = groups[key]
        excess = g["L"] - g["reference"]
        out["groups"][key] = {
            "nets": g["nets"], "ratio": g["L"] / g["reference"] if g["reference"] else None,
            "share_of_L": g["L"] / total["L"], "share_of_excess": excess / excess_total,
            "planar_over_flute": g["planar"] / g["flute_planar"] if g["flute_planar"] else None,
            "planar_excess_share_of_group_excess": (g["planar"] - g["flute_planar"]) / excess if excess else None,
            "via_tsv_share_of_L": (g["via"] + g["tsv"]) / g["L"] if g["L"] else None,
            "per_die_flute_over_flute": g["per_die_wire"] / g["flute_planar"] if g["flute_planar"] else None,
            "zero_reference": g["zero_reference"]}
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main(sys.argv[1])
