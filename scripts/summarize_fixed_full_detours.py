#!/usr/bin/env python3
"""Summarize eda_diagnose_fixed_full_detours output for one case.

Length gap per search mode is the class-weighted mean of one-net reroutes (each
class's sum dL / sum old L, weighted by the class's share of total length), so it
reads as "fraction of the design's length". The fixed-full share is
fixed_walls - no_congestion. Cell breakdowns use the class-weighted attribution
written by the C++ tool. Offline, one net at a time, not an achievable total."""
import csv
import json
import sys
from collections import defaultdict


def main(replay_dir):
    classes = json.load(open(f"{replay_dir}/classes.json"))
    L, R = classes["L"], classes["reference"]
    rows = defaultdict(list)
    failures = defaultdict(int)
    for r in csv.DictReader(open(f"{replay_dir}/replay.csv")):
        if r["status"] != "ok":
            failures[(r["mode"], r["status"][:60])] += 1
            continue
        rows[(r["class"], r["mode"])].append(r)
    modes = sorted({m for _, m in rows})
    gap = {}
    per_net = {}
    for mode in modes:
        frac = 0.0
        full_entries = 0
        n_total = 0
        zero = 0
        for name, info in classes["classes"].items():
            sample = rows.get((name, mode), [])
            if not sample:
                continue
            dl = sum(float(r["dL"]) for r in sample)
            old = sum(float(r["old_L"]) for r in sample)
            frac += (info["L"] / L) * (dl / max(old, 1e-9))
            full_entries += sum(int(r["fixed_full_cells"] or 0) for r in sample)
            zero += sum(int(r["zero_capacity_cells"] or 0) for r in sample)
            n_total += len(sample)
        gap[mode] = frac
        per_net[mode] = {"sampled_ok": n_total,
                         "fixed_full_cells_per_net": full_entries / max(1, n_total),
                         "zero_capacity_cells_total": zero}
    old_full = 0
    n_old = 0
    for (name, mode), sample in rows.items():
        if mode == "maze_large":
            old_full += sum(int(r["old_fixed_full_cells"] or 0) for r in sample)
            n_old += len(sample)
    out = {
        "L": L, "reference": R, "ratio": L / R, "W": 100 * R / L,
        "model_mismatch_cells": classes["model_mismatch_cells"],
        "official_capacity_mismatch": classes["official_capacity_mismatch"],
        "official_lists": classes["official_lists"],
        "failures": {f"{m}: {s}": c for (m, s), c in failures.items()},
        "length_change_fraction_by_mode": gap,
        "per_net": per_net,
        "current_route_fixed_full_cells_per_net": old_full / max(1, n_old),
    }
    if "maze_large_fixed_walls" in gap and "maze_large_no_congestion" in gap:
        out["detour_from_fixed_full_cells"] = gap["maze_large_fixed_walls"] - gap["maze_large_no_congestion"]
    if "maze_large" in gap and "maze_large_fixed_walls" in gap:
        out["detour_from_other_routes"] = gap["maze_large"] - gap["maze_large_fixed_walls"]

    cells = list(csv.DictReader(open(f"{replay_dir}/fixed_full_cells.csv")))
    entered = [c for c in cells if int(c["nets_no_congestion"]) > 0]
    total_dl = sum(float(c["attributed_dL"]) for c in entered)
    total_w = sum(float(c["weighted_entries"]) for c in entered)

    def share(pred):
        sel = [c for c in entered if pred(c)]
        return {
            "cells": len(sel),
            "share_of_entries": sum(float(c["weighted_entries"]) for c in sel) / max(total_w, 1e-9),
            "share_of_detour": sum(float(c["attributed_dL"]) for c in sel) / max(total_dl, 1e-9),
        }

    def capped(c, key):
        return min(int(c[key]), int(c["capacity"])) >= int(c["capacity"])

    by_layer = {}
    for layer in sorted({int(c["layer"]) for c in entered}):
        by_layer[layer] = share(lambda c, layer=layer: int(c["layer"]) == layer)
    out["fixed_full_cells_entered_by_no_congestion_route"] = {
        "cells": len(entered),
        "attributed_detour_dbu": total_dl,
        "attributed_detour_fraction_of_L": total_dl / L,
        "by_layer": by_layer,
        "what_fills_them": {
            "obs_alone_fills": share(lambda c: capped(c, "obs_tracks")),
            "pins_alone_fill": share(lambda c: c["full_without_obs"] == "1"),
            "obs_and_pins_only_together": share(lambda c: not capped(c, "obs_tracks") and c["full_without_obs"] != "1"),
            "not_full_without_unused_pins": share(lambda c: c["full_without_unused_pins"] != "1"),
            "not_full_without_fallback_track": share(lambda c: c["full_without_fallback"] != "1"),
            "not_full_without_signal_pins": share(lambda c: c["full_without_signal_pins"] != "1"),
            "own_pin_cell_of_an_entering_net": share(lambda c: int(c["own_pin_nets"]) > 0),
        },
        "official_lists": {
            "listed_confirmed_full": share(lambda c: int(c["official_lists"]) > 0),
            "never_listed_untested": share(lambda c: int(c["official_lists"]) == 0),
        },
    }
    hist = defaultdict(int)
    for c in entered:
        hist[int(c["capacity"])] += 1
    out["fixed_full_cells_entered_by_no_congestion_route"]["capacity_histogram"] = dict(sorted(hist.items()))
    top = sorted(entered, key=lambda c: -float(c["attributed_dL"]))[:15]
    out["top_cells"] = [{k: c[k] for k in ("col", "row", "layer", "capacity", "fixed", "obs_tracks", "signal_pin_tracks",
                                             "unused_pin_tracks", "io_pin_tracks", "fallback_shapes", "nets_no_congestion",
                                             "attributed_dL", "official_lists", "examples")} for c in top]
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main(sys.argv[1])
