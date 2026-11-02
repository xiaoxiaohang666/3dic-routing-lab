"""Summarize read-only attribution on Titan; never writes Guides or resources."""
from pathlib import Path
from collections import Counter, defaultdict
import argparse
import csv
import hashlib
import json
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attribution", required=True, type=Path)
    parser.add_argument("--official-report", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output must be new")
    root = args.attribution
    summary = json.loads((root / "summary.json").read_text())
    report = json.loads(args.official_report.read_text())["raw_report"]
    rows = lambda name: list(csv.DictReader((root / name).open()))
    net_rows = rows("nets.csv")
    net = {}
    for row in net_rows:
        row = {k: (v if k == "net" else float(v)) for k, v in row.items()}
        net[int(row["net_index"])] = row
    cells = {}
    for row in rows("cells.csv"):
        row = {k: int(v) for k, v in row.items()}
        key = tuple(row[k] for k in ("layer", "col", "row"))
        cells[key] = row
    owners = defaultdict(list)
    for row in rows("owners.csv"):
        row = {k: (v if k in ("net", "kind") else int(v)) for k, v in row.items()}
        owners[tuple(row[k] for k in ("layer", "col", "row"))].append(row)

    def q(cell, removed=0):
        return (max(0, cell["official_total"] - removed - cell["capacity"]) / cell["capacity"]) ** 2

    Q = sum(q(c) for c in cells.values())
    assert len(cells) == summary["official_hot_cells"] == report["congestion"]["overflow_entries"]
    assert math.isclose(100 / math.sqrt(1 + Q), report["score"]["congestion_score"], abs_tol=1e-10)
    assert sum(n["L"] for n in net.values()) == summary["L"] == report["wirelength"]["total_dbu"]
    for key, cell in cells.items():
        assert sum(o["demand"] for o in owners[key]) == cell["route"], key
        assert sum(o["mandatory_tsv"] for o in owners[key]) == cell["mandatory_tsv"], key
        assert cell["official_total"] >= cell["route"], key

    def ordinary(o):
        return o["demand"] - o["mandatory_tsv"]

    layers = []
    for layer in range(1, 21):
        keys = [k for k in cells if k[0] == layer]
        billed = [o for k in keys for o in owners[k] if ordinary(o) > 0]
        total = sum(q(cells[k]) for k in keys)
        movable = sum(q(cells[k]) - q(cells[k], sum(ordinary(o) for o in owners[k] if not o["pin_geometry"])) for k in keys)
        layers.append({"layer": layer, "overflow_cells": len(keys), "Q": total, "Q_share": total / Q,
                       "no_pin_ordinary_deletion_relief_upper_bound": movable,
                       "billed_ordinary_owner_occurrences": len(billed),
                       "billed_ordinary_nets": len({o["net_index"] for o in billed}),
                       "topology": dict(Counter(o["kind"] for o in billed)),
                       "local_contacts": dict(Counter(str(o["planar_neighbors"]) + "_planar_" + str(o["layer_contacts"]) + "_vertical" for o in billed)),
                       "pin_geometry_occurrences": sum(o["pin_geometry"] for o in billed)})
    legacy = sorted((n for n in net if net[n]["legacy_rank"] > 0), key=lambda n: (-net[n]["legacy_rank"], n))
    caps = []
    for count in [512, 1024, 2048, 4096, len(legacy)]:
        selected = set(legacy[:count])
        touched = 0
        relief = 0
        for key, cell in cells.items():
            removed = sum(ordinary(o) for o in owners[key] if o["net_index"] in selected and not o["pin_geometry"])
            touched += removed > 0
            relief += q(cell) - q(cell, removed)
        caps.append({"limit": count, "selected": len(selected), "official_hot_cells_with_selected_no_pin_ordinary_owner": touched,
                     "joint_deletion_Q_relief_upper_bound": relief})
    top = sorted(net.values(), key=lambda n: (-n["ordinary_official_marginal"], n["net_index"]))[:15]
    L = summary["L"]
    W = report["score"]["wirelength_score"]
    C = report["score"]["congestion_score"]
    scenarios = []
    for loss in [0, 0.00025, 0.001, 0.005]:
        for drop in [1, 2, 3]:
            delta = .3 * (100 / math.sqrt(1 + Q - drop) - C) + .45 * W * (1 / (1 + loss) - 1)
            scenarios.append({"Q_reduction": drop, "L_growth_fraction": loss, "delta_single_case_if_T_fixed": delta, "delta_public_mean": delta / 3})
    result = {"scope": "Read-only official sparse-hotspot attribution, not optimizer input or candidate score",
              "baseline_L": L, "baseline_Q": Q, "baseline_W": W, "baseline_C": C,
              "checks": {"official_C_reconstructed": True, "all_net_L_matches_official": True, "all_hotspot_owner_demands_sum_to_ledger": True},
              "layers": layers, "top_nets_by_ordinary_deletion_marginal": top,
              "legacy_positive_rank_nets": len(legacy), "legacy_shortlist_coverage": caps,
              "ordinary_via_count": sum(int(n["ordinary_vias"]) for n in net.values()),
              "ordinary_via_L_fraction": sum(n["via_L"] for n in net.values()) / L,
              "line_growth_allowance_DBU": {str(f): L * f for f in [0.00025, 0.001, 0.005]},
              "conditional_score_sensitivity": scenarios,
              "caveats": ["Per-net deletion marginals overlap and must not be summed.", "Joint deletion bounds ignore connectivity and new route/via costs.", "Sparse official hotspots cannot determine the full resource map or candidate scores.", "Degree classification describes billed Guide-cell contacts, not a proof that an edge is removable."],
              "source_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [root / n for n in ["nets.csv", "cells.csv", "owners.csv", "summary.json"]]}}
    with args.output.open("x") as f:
        json.dump(result, f, indent=2)
        f.write("\n")
    print(json.dumps({"layers": [r for r in layers if r["Q"] > 1], "top_nets": top[:8], "shortlists": caps,
                      "ordinary_via_L_fraction": result["ordinary_via_L_fraction"]}, indent=2))


if __name__ == "__main__":
    main()
