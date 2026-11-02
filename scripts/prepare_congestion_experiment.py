"""Freeze a planned experiment on Titan. Does not route or build a ZIP."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import socket


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--plan", type=Path, required=True)
    p.add_argument("--baseline-manifest", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    if socket.gethostname() != "benchmark-host":
        p.error("preparation must run on benchmark-host")
    if args.output_dir.exists():
        p.error("output directory must be new")
    plan = json.loads(args.plan.read_text())
    baseline = json.loads(args.baseline_manifest.read_text())
    for profile in plan["profiles"]:
        assert profile["total_wall_seconds"] <= 36000
        assert sum(profile["operator_seconds"].values()) + profile["parse_export_verify_reserve_seconds"] == profile["total_wall_seconds"]
        assert profile["soft_peak_memory_GiB"] < 128
    assert plan["fixed_tsv"] and plan["preserve_all_existing_pin_access_cells"]
    checked = {}
    for path, expected in baseline["protected_paths_sha256"].items():
        actual = digest(Path(path))
        assert actual == expected, path
        checked[path] = actual
    assert baseline["input_sha256"]
    for path, expected in baseline["input_sha256"].items():
        assert digest(Path(path)) == expected, path
        checked[path] = expected
    assert not plan["official_sparse_csv_used_by_optimizer"]
    memory = {line.split(":")[0]: int(line.split()[1]) for line in Path("/proc/meminfo").read_text().splitlines()}
    args.output_dir.mkdir()
    result = {"status": "PREPARED_NOT_RUN", "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "host": socket.gethostname(), "plan_sha256": digest(args.plan),
              "baseline_manifest_sha256": digest(args.baseline_manifest),
              "protected_files_unchanged": checked,
              "MemAvailable_KiB": memory["MemAvailable"],
              "minimum_launch_available_GiB": plan["minimum_launch_available_GiB"],
              "memory_launch_gate_now": memory["MemAvailable"] >= plan["minimum_launch_available_GiB"] * 1024 ** 2,
              "optimizer_implemented": False, "launch_ready": False,
              "remaining": ["Implement opt-in via-stack relocation and local-subtree operator", "Pass synthetic transaction/PIN/TSV/overlap-accounting/deadline checks on Titan", "Check runtime memory availability and actual incumbent export reserve"],
              "routing_run": False, "guide_written": False, "zip_written": False,
              "profiles": plan["profiles"]}
    (args.output_dir / "prepared-experiment.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("status", "launch_ready", "memory_launch_gate_now", "MemAvailable_KiB")}))


if __name__ == "__main__":
    main()
