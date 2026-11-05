#!/usr/bin/env python3
"""Assemble a code-channel submission ZIP (run on benchmark-host after a Release build).

Third-party runtime files (lib/, runtime/official-flute/, licenses/) are copied
from a previously submitted reference package and checked against that
package's manifest; they are never stored in Git. The router binary, sources
and entry scripts come from the current checkout. Uploading is a separate,
explicitly authorised user action.
"""
import argparse
import hashlib
import json
import pathlib
import stat
import subprocess
import sys
import zipfile

ENTRY_SCRIPTS = ["run_package.sh", "run_a150_20261029.sh", "run_tsv_resource_v7.sh",
                 "run_tsv_resource_v6.sh", "run_tsv_resource.sh", "run_v9.sh"]
THIRD_PARTY_PREFIXES = ("lib/", "runtime/official-flute/", "licenses/")
RUN_SH = """#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
chmod u+x "$ROOT/router"
export LD_LIBRARY_PATH="$ROOT/lib:$ROOT/runtime/official-flute${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export EDA_ROUTER_BINARY="$ROOT/router"
export EDA_OFFICIAL_RUNTIME="$ROOT/runtime/official-flute"
exec bash "$ROOT/scripts/run_package.sh" --time-limit __TIME_LIMIT__ "$@"
"""


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=pathlib.Path, required=True, help="repository checkout used for the build")
    p.add_argument("--router", type=pathlib.Path, required=True, help="Release eda_router built from --root")
    p.add_argument("--reference-package", type=pathlib.Path, required=True)
    p.add_argument("--output", type=pathlib.Path, required=True)
    p.add_argument("--label", required=True)
    p.add_argument("--source-commit", required=True)
    p.add_argument("--time-limit", type=int, default=3000)
    a = p.parse_args()
    if a.output.exists():
        p.error("output must be new: " + str(a.output))
    if not 60 <= a.time_limit <= 43200:
        p.error("time limit must be 60..43200 seconds")
    files = {}
    reference = zipfile.ZipFile(a.reference_package)
    ref_manifest = json.loads(reference.read("manifest.json"))["files"]
    for info in reference.infolist():
        name = info.filename
        if name.endswith("/") or not name.startswith(THIRD_PARTY_PREFIXES):
            continue
        data = reference.read(name)
        if ref_manifest.get(name) != sha256(data):
            raise SystemExit("reference package file does not match its manifest: " + name)
        files[name] = (data, 0o644)
    for required in ("lib/libpython3.10.so.1.0", "runtime/official-flute/libflute.so",
                     "runtime/official-flute/POWV9.dat", "runtime/official-flute/POST9.dat"):
        if required not in files:
            raise SystemExit("reference package lacks " + required)
    router = a.router.read_bytes()
    files["router"] = (router, 0o755)
    files["run.sh"] = (RUN_SH.replace("__TIME_LIMIT__", str(a.time_limit)).encode(), 0o755)
    files["build.sh"] = ((a.root / "submission/build.sh").read_bytes(), 0o755)
    for script in ENTRY_SCRIPTS:
        files["scripts/" + script] = ((a.root / "scripts" / script).read_bytes(), 0o755)
    for path in sorted((a.root / "src").rglob("*")):
        if path.is_file() and path.suffix in (".cpp", ".hpp", ".h"):
            files[str(path.relative_to(a.root))] = (path.read_bytes(), 0o644)
    help_text = subprocess.run([str(a.router), "--help"], capture_output=True, text=True).stdout
    for flag in ("--thermal-gain-estimate", "--objective-profile", "--climb-escape-radius", "--fixed-resource-model",
                 "--congestion-target-q", "--rrr-reserved-seconds"):
        if flag not in help_text:
            raise SystemExit("router binary does not support " + flag)
    readme = f"""# 3D IC global router package {a.label}

Linux x86-64, Ubuntu 22.04. Run: `bash run.sh --def INPUT.def --lef INPUT.lef --thermal-report THERMAL.json --output OUTPUT.guide`.
Paths are relative to the caller. Time budget {a.time_limit} seconds (internal deadline keeps an export reserve).

Settings of the research entry `run_fixed_resource_candidate.sh` (documented resources, A150 rules with
the October 29 PIN floor, TSV ban only on cells with PIN/OBS on M10/M11, site-20261029 scalar objective,
fixed occupancy read as `observed-20261104`, which comes from returned platform samples and is not a
published rule), with these points:

- the platform passes no per-case thermal gain, so the router uses G = 1.5 / hottest-K Pearson bound
  (K = cross-die nets) as an optimisation proxy; it is not an official calibration;
- each TSV landing keeps one net;
- a via climb that would pass through full cells may first step sideways on the pin layer, up to 3
  cells (`--climb-escape-radius 3`); the candidate is taken only when its total cost is lower.

Source commit {a.source_commit}. Offline readback scores are model estimates, not official results.
"""
    files["README.md"] = (readme.encode(), 0o644)
    manifest = {"label": a.label, "source_commit": a.source_commit, "router_sha256": sha256(router),
                "time_limit": a.time_limit, "reference_package_sha256": sha256(a.reference_package.read_bytes()),
                "files": {name: sha256(data) for name, (data, _) in sorted(files.items())}}
    files["manifest.json"] = (json.dumps(manifest, indent=1).encode(), 0o644)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(a.output, "x", compression=zipfile.ZIP_DEFLATED) as out:
        for name, (data, mode) in sorted(files.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 10, 31, 0, 0, 0))
            info.external_attr = (stat.S_IFREG | mode) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            out.writestr(info, data)
    print(json.dumps({"output": str(a.output), "sha256": sha256(a.output.read_bytes()),
                      "router_sha256": manifest["router_sha256"], "files": len(files)}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
