#!/usr/bin/env python3
"""Measure a serialized Guide under the current documented production contract.

Run on benchmark-host. This is an offline model, not official equivalence certification.
"""
import argparse
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--def", dest="def_file", required=True, type=Path)
    p.add_argument("--lef", action="append", required=True, type=Path)
    p.add_argument("--thermal", required=True, type=Path)
    p.add_argument("--guide", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--thermal-gain", required=True, type=float)
    p.add_argument("--runtime", type=Path, default=ROOT / "output/runtime/official-flute")
    p.add_argument("--binary", type=Path, default=ROOT / "build/bin/eda_measure_documented")
    p.add_argument("--cells-csv", type=Path)
    a = p.parse_args()
    import math
    if not math.isfinite(a.thermal_gain) or a.thermal_gain <= 0:
        p.error("--thermal-gain must be finite and positive")
    for f in [a.def_file, *a.lef, a.thermal, a.guide, a.binary]:
        if not f.is_file(): p.error("missing file: " + str(f))
    for f in [a.output] + ([a.cells_csv] if a.cells_csv else []):
        if f.exists(): p.error("output must be new: " + str(f))
        if not f.parent.is_dir(): p.error("output parent must exist: " + str(f.parent))
    cmd = [str(a.binary.resolve()), "--def", str(a.def_file), "--thermal", str(a.thermal),
           "--guide", str(a.guide), "--runtime", str(a.runtime.resolve()), "--output", str(a.output),
           "--rules-profile", "a150-20261029", "--tsv-obstacle-scope", "all-layers",
           "--penalty-scale", "1", "--thermal-gain", str(a.thermal_gain)]
    for f in a.lef: cmd += ["--lef", str(f)]
    if a.cells_csv: cmd += ["--cells-csv", str(a.cells_csv)]
    env = os.environ.copy()
    lib = a.runtime.resolve().parent.parent / "lib"
    if (lib / "libpython3.10.so.1.0").is_file():
        env["LD_LIBRARY_PATH"] = str(lib) + ":" + str(a.runtime.resolve()) + ":" + env.get("LD_LIBRARY_PATH", "")
    return subprocess.run(cmd, env=env).returncode

if __name__ == "__main__":
    sys.exit(main())
