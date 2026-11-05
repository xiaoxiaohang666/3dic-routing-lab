#!/usr/bin/env python3
"""Package explicitly selected, previously checked Guides on benchmark-host.

Selection JSON: {"candidates": [{"case": "open_3d_case001", "guide": "...",
"readback": "...", "validation": "...", "manifest": "..."}]}. Relative paths
are resolved against the selection file. The ZIP contains only root-level
<case>.output.guide files; hashes and offline evidence stay in a separate JSON.
No routing, scoring, input modification or platform upload is performed.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import zipfile


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def checked_json(path: Path) -> tuple[dict, str]:
    data = path.read_bytes()
    value = json.loads(data)
    require(isinstance(value, dict), f"expected JSON object: {path}")
    return value, hashlib.sha256(data).hexdigest()


def candidate(item: dict, base: Path, scope: str = "all-layers") -> dict:
    require(isinstance(item, dict), "candidate must be an object")
    case = item.get("case")
    require(isinstance(case, str) and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", case)
            is not None, f"invalid case identifier: {case!r}")
    paths = {}
    for key in ("guide", "readback", "validation", "manifest"):
        value = item.get(key)
        require(isinstance(value, str) and bool(value), f"{case}: missing {key} path")
        path = Path(value)
        paths[key] = (base / path).resolve(strict=True)
        require(paths[key].is_file(), f"{case}: not a file: {paths[key]}")
    require(len(set(paths.values())) == len(paths), f"{case}: evidence paths must differ")
    guide_hash = digest(paths["guide"])
    manifest_data = paths["manifest"].read_bytes()
    manifest = manifest_data.decode("utf-8")
    bound = []
    for line in manifest.splitlines():
        match = re.fullmatch(r"([0-9a-fA-F]{64}) [ *](.+)", line)
        if match:
            source = (paths["manifest"].parent / match[2]).resolve()
            if source == paths["guide"]:
                bound.append(match[1].lower())
    require(bool(bound) and all(value == guide_hash for value in bound),
            f"{case}: Guide hash/path is absent or mismatched in manifest")
    for stage in ("route", "score", "validate"):
        exits = re.findall(r"(?m)^" + stage + r"_exit=(-?\d+)(?:\s|$)", manifest)
        require(bool(exits) and all(value == "0" for value in exits),
                f"{case}: manifest must record successful {stage}_exit")
    validation, validation_hash = checked_json(paths["validation"])
    require(validation.get("status") == "PASS", f"{case}: validation is not PASS")
    require(validation.get("errors", {}).get("total") == 0,
            f"{case}: validation errors are nonzero or missing")
    a150 = validation.get("a150", {})
    if scope == "interface":
        # Official Q&A: the TSV ban checks only M10 and M11. See docs/rules/qa-notes-20261105.md.
        require(a150.get("scope") == "interface-m10-m11",
                f"{case}: validation must use the M10/M11 scope")
        require(a150.get("interface_violations") == 0,
                f"{case}: M10/M11 validation violations are nonzero or missing")
    else:
        require(a150.get("scope") == "all-routing-layers-both-dies",
                f"{case}: validation must cover all routing layers")
        require(a150.get("all_layers_violations", a150.get("all_layer_violations")) == 0,
                f"{case}: A150 validation violations are nonzero or missing")
    readback, readback_hash = checked_json(paths["readback"])
    a150 = readback.get("a150", {})
    require(a150.get("enabled") is True and a150.get("obstacle_scope") == scope,
            f"{case}: readback must use A150 {scope}")
    violations = a150.get("interface_violations") if scope == "interface" else \
        a150.get("all_layer_violations", a150.get("all_layers_violations"))
    require(violations == 0, f"{case}: readback A150 violations are nonzero or missing")
    require(readback.get("congestion_overflow_scale_proxy") == 1
            and readback.get("all_adjacent_overlaps") is True,
            f"{case}: readback must use scale=1 and all adjacent overlaps")
    require(readback.get("official_equivalence") is False,
            f"{case}: offline evidence must retain official_equivalence=false")
    metrics = readback.get("metrics", {})
    values = {key: metrics.get(key) for key in ("W", "C", "T", "S")}
    values["G"] = readback.get("G")
    require(all(isinstance(value, (float, int)) and not isinstance(value, bool)
                and math.isfinite(value) for value in values.values()),
            f"{case}: nonfinite or missing score/G")
    require(values["G"] > 0, f"{case}: G must be positive")
    require(math.isclose(values["S"], .45 * values["W"] + .30 * values["C"]
                         + .25 * values["T"], rel_tol=1e-10, abs_tol=1e-8),
            f"{case}: S disagrees with W/C/T")
    return {"case": case, "member": case + ".output.guide",
            "sources": {key: str(path) for key, path in paths.items()},
            "sha256": {"guide": guide_hash, "readback": readback_hash,
                       "validation": validation_hash,
                       "manifest": hashlib.sha256(manifest_data).hexdigest()},
            "metrics": values, "validation_status": "PASS"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--tsv-obstacle-scope", choices=("all-layers", "interface"), default="all-layers")
    args = parser.parse_args()
    selection = args.selection.resolve(strict=True)
    output, summary = args.output.resolve(), args.summary.resolve()
    require(output != summary, "ZIP and summary paths must differ")
    for path in (args.output, args.summary):
        require(not os.path.lexists(path), f"output must be new: {path}")
        require(path.parent.is_dir(), f"output parent must exist: {path.parent}")
    document, selection_hash = checked_json(selection)
    entries = document.get("candidates")
    require(isinstance(entries, list) and bool(entries), "selection needs a nonempty candidates list")
    candidates = [candidate(item, selection.parent, args.tsv_obstacle_scope) for item in entries]
    names = [item["case"].casefold() for item in candidates]
    require(len(set(names)) == len(names), "duplicate case identifier")
    inputs = {selection} | {Path(path) for item in candidates for path in item["sources"].values()}
    require(output not in inputs and summary not in inputs, "output aliases a source file")
    candidates.sort(key=lambda item: item["case"])
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for item in candidates:
            info = zipfile.ZipInfo(item["member"], date_time=(2026, 10, 31, 0, 0, 0))
            info.external_attr = 0o100644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            with open(item["sources"]["guide"], "rb") as source, archive.open(info, "w") as target:
                for block in iter(lambda: source.read(1024 * 1024), b""):
                    target.write(block)
    with zipfile.ZipFile(output) as archive:
        require(archive.namelist() == [item["member"] for item in candidates],
                "ZIP members differ from selection")
        for item in candidates:
            h = hashlib.sha256()
            with archive.open(item["member"]) as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    h.update(block)
            require(h.hexdigest() == item["sha256"]["guide"],
                    f"Guide changed during packaging: {item['case']}")
    report = {"format_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
              "selection": str(selection), "selection_sha256": selection_hash,
              "archive": str(output), "archive_sha256": digest(output),
              "archive_size": output.stat().st_size, "candidates": candidates,
              "official_equivalence": False, "platform_uploaded": False,
              "tsv_obstacle_scope": args.tsv_obstacle_scope,
              "evidence_boundary": "The run manifest binds each Guide path and SHA256. "
              "Readback/validation JSON has no embedded input identity or Guide hash; "
              "their association is the selected run directory, not an independent cryptographic "
              "proof. Scores use the offline production model, not official reevaluation. "
              "Engineering validation does not certify full DRC or platform acceptance."}
    with summary.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"archive": str(output), "sha256": report["archive_sha256"],
                      "summary": str(summary), "guides": len(candidates)}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError, TypeError, zipfile.BadZipFile) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
