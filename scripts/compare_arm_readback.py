#!/usr/bin/env python3
"""Compare two arm result directories (readback-observed, stats, route log)."""
import json, re, sys
from pathlib import Path

KEYS = ["S", "W", "C", "T", "Q", "rho", "L", "reference", "overflow", "planar_L", "via_L", "tsv_L", "ordinary_connections"]
COUNTERS = ["initial_maze_seconds", "v7_rrr_seconds", "v9_full_validation_seconds", "v7_tsv_seconds",
            "overflow_cleanup_seconds", "wirelength_polish_seconds", "wirelength_polish_attempted",
            "wirelength_polish_accepted", "wirelength_polish_gain_dbu"]

def load(d):
    d = Path(d)
    out = {}
    rb = json.loads((d / "readback-observed.json").read_text())
    out["G"] = rb.get("G")
    out.update({k: rb["metrics"].get(k) for k in KEYS})
    arm = json.loads((d / "arm.json").read_text())
    out["route_exit"] = arm["route"]["exit"]
    out["route_seconds"] = arm["route"]["seconds"]
    out["route_rss_gib"] = arm["route"]["max_rss_kib"] / 2**20
    out["guide_sha256"] = arm["guide_sha256"][:12]
    out["validation"] = json.loads((d / "validation.json").read_text()).get("status")
    st = json.loads((d / "stats-summary.json").read_text())
    c = st.get("counters", {})
    for k in COUNTERS:
        out[k] = c.get(k)
    log = (d / "route.log").read_text(errors="replace")
    m = re.findall(r"(?i)(thermal[_ -]gain[^\n]{0,120})", log)
    out["gain_log"] = m[0][:120] if m else None
    out["cleanup_lines"] = [l for l in log.splitlines() if l.startswith(("[OVERFLOW_CLEANUP]", "[WIRELENGTH_POLISH]", "[FINAL]", "[Reserved RRR]"))]
    return out

a, b = load(sys.argv[1]), load(sys.argv[2])
for k in a:
    if k == "cleanup_lines":
        continue
    va, vb = a[k], b.get(k)
    diff = ""
    if isinstance(va, (int, float)) and isinstance(vb, (int, float)) and not isinstance(va, bool):
        diff = f"  diff={vb - va:+.6g}"
    print(f"{k:32s} {str(va):>28s} {str(vb):>28s}{diff}")
for name, x in (("A", a), ("B", b)):
    print(f"--- {name} stage lines")
    for l in x["cleanup_lines"]:
        print("  ", l[:200])
