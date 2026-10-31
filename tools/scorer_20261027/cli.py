"""Isolated CLI; original inputs, dependency package and older scorers stay intact."""
from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import sys
import tempfile
import time

from . import __version__
from .profile import A150Violation, Options, RULES, HISTORICAL_RULES, OCT29_RULES, NEW_PROFILE, install


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                          encoding="utf-8")


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("Expected finite nonnegative number")
    return number


class ProfileParser(argparse.ArgumentParser):
    def parse_args(self, args=None, namespace=None):
        value = super().parse_args(args, namespace)
        a150 = value.rules_profile in ("a150",NEW_PROFILE)
        for key,default in {"tsv_demand_model":"landing-union",
                "fixed_track_boundary":"closed" if a150 else "half-open",
                "landing_track_boundary":"closed" if a150 else "open",
                "tsv_pin_exemption":"none" if a150 else "own-pin",
                "thermal_gain_policy":"default20" if a150 else "archived-identity"}.items():
            if getattr(value,key) is None:
                setattr(value,key,default)
        if a150 and value.tsv_congestion_ratio != 1:
            self.error("A150 direct track counting requires --tsv-congestion-ratio 1")
        return value


def create_parser(default_dependency):
    p = ProfileParser(description="Document-based 20261027 compatible scorer; not official.",
                                allow_abbrev=False)
    p.add_argument("guide", type=Path)
    p.add_argument("--rules-profile", choices=(NEW_PROFILE,"a150","inferred-platform"), default=NEW_PROFILE,
                   help="a150-20261029 adds logical-PIN zero-track floors; a150 and inferred-platform preserve historical contracts")
    p.add_argument("--tsv-obstacle-scope", choices=("all-layers","interface"), default="all-layers",
                   help="Conservative all-layer PIN/OBS prohibition; interface tests only metal10/metal11")
    p.add_argument("--lefs", nargs="+", required=True)
    p.add_argument("--input-def", type=Path, required=True)
    p.add_argument("--thermal-json", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True, help="New or empty directory")
    p.add_argument("--dependency-root", type=Path, default=default_dependency,
                   help="Untouched community-score-0.4.2 package")
    p.add_argument("--runtime", type=Path, help="Optional vendor runtime directory; defaults to dependency-root/vendor")
    p.add_argument("--mode", choices=("20261027", "platform"), default="20261027",
                   help="platform is a CLI alias for this 20261027 profile")
    p.add_argument("--tsv-cost-ratio", type=nonnegative, default=6.)
    p.add_argument("--via-cost-ratio", type=nonnegative, default=3.)
    p.add_argument("--tsv-congestion-ratio", type=nonnegative, default=1.)
    p.add_argument("--thermal-gain", type=nonnegative,
                   help="Explicit gain override; otherwise use the stated gain policy")
    p.add_argument("--thermal-gain-policy", choices=("archived-identity", "default20"),
                   default=None,
                   help="Archived content-hash configuration is an unconfirmed compatibility assumption")
    p.add_argument("--tsv-width-um", type=nonnegative, default=5.)
    p.add_argument("--tsv-demand-model", choices=("physical-tracks", "cell-track", "ceil", "landing-union"),
                   default=None)
    p.add_argument("--capacity-boundary", choices=("half-open", "closed"), default="half-open")
    p.add_argument("--fixed-track-boundary", choices=("half-open", "open", "closed"), default=None)
    p.add_argument("--landing-track-boundary", choices=("half-open", "open", "closed"), default=None)
    p.add_argument("--tsv-pin-exemption", choices=("own-pin", "none"), default=None)
    p.add_argument("--pin-credit-policy", choices=("all-layers", "first-layer"), default="all-layers")
    p.add_argument("--zero-capacity-penalty", type=nonnegative, default=10000.)
    p.add_argument("--dump-congestion-state", action="store_true",
                   help="Write congestion_state.json.gz for offline diagnostic sweeps")
    p.add_argument("--version", action="version", version="score-20261027 " + __version__)
    return p


def resolve_gain(args, hashes, dependency):
    if args.thermal_gain is not None:
        return args.thermal_gain, "explicit-command-line-override", None
    if args.thermal_gain_policy == "default20":
        return 20., "explicit-default20-policy", None
    identities = json.loads((dependency / "evaluation/repaired/case-identities.json").read_text())
    identity = identities.get(hashes["def"])
    if identity:
        lef_hashes = sorted(value for key, value in hashes.items() if key.startswith("lef"))
        if hashes["thermal"] != identity["thermal_sha256"] or lef_hashes != sorted(identity["lef_sha256"]):
            raise ValueError("Known DEF does not match its archived LEF and thermal configuration")
        return identity["gain"], "archived-content-identity-configuration:unverified-under-20261027", identity["case"]
    return 20., "archived-default20:unregistered-input", None


def main(argv=None, *, default_dependency=None):
    if default_dependency is None:
        default_dependency = Path(__file__).resolve().parents[2] / "references/community-score-0.4.2"
    args = create_parser(default_dependency).parse_args(argv)
    rules = OCT29_RULES if args.rules_profile == NEW_PROFILE else RULES if args.rules_profile == "a150" else HISTORICAL_RULES
    calibrated_resource_profile = (args.rules_profile=="inferred-platform" and args.tsv_demand_model=="landing-union" and
        args.capacity_boundary=="half-open" and args.fixed_track_boundary=="half-open" and
        args.landing_track_boundary=="open" and args.pin_credit_policy=="all-layers" and
        args.tsv_pin_exemption=="own-pin" and args.tsv_width_um==5 and args.tsv_congestion_ratio==1)
    calibration_status = ("A150_1029_PIN_FLOOR_CONGESTION_FORMULA_UNCONFIRMED" if args.rules_profile==NEW_PROFILE else
                          "A150_TEXT_WITH_EXPLICIT_ASSUMPTIONS" if args.rules_profile=="a150" else
                          "A1_REFERENCE_PROFILE_WITH_HISTORICAL_CONFLICT" if calibrated_resource_profile
                          else "UNCALIBRATED_DIAGNOSTIC_MODEL")
    print("Independent " + rules + " resource model; official source equivalence remains uncertified.", file=sys.stderr)
    args.lefs = [Path(item) for value in args.lefs for item in value.split(",") if item]
    dependency = args.dependency_root.resolve(strict=True)
    sys.path.insert(0, str(dependency))
    from evaluation.repaired.input_guard import validate_lef_geometry
    from evaluation.repaired.runtime import ARCHIVE_SHA256, SCORE_SHA256, load
    from evaluation.repaired.cli import validate_finite

    output = args.output_dir.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        print("output-dir must be new or empty", file=sys.stderr)
        return 2
    output.mkdir(parents=True, exist_ok=True)
    sources = [("guide", args.guide), ("def", args.input_def), ("thermal", args.thermal_json)]
    sources += [(f"lef{i}", path) for i, path in enumerate(args.lefs)]
    started = time.perf_counter()
    audit = {"version": __version__, "rules": rules, "mode": "20261027",
             "started_utc": datetime.now(timezone.utc).isoformat(),
             "official_equivalence": False, "scope": "document-based-compatible-evaluator-not-official",
             "calibration_status": calibration_status,
             "calibration_scope": ("1029 PIN zero-track floor; strengthened congestion formula unpublished; no official 1029 scalar match"
                                   if args.rules_profile==NEW_PROFILE else
                                   "A134/A146/A149/A150 text with recorded scope/boundary assumptions; not official score parity"
                                   if args.rules_profile=="a150" else
                                   "A1 public Guides: user-provided 2026-10-28 Guide log; historical fixed-version3 checked separately"),
             "arguments": list(sys.argv[1:] if argv is None else argv),
             "production_candidate_acceptance_supported": False,
             "dependency_root": str(dependency), "dependency_archive_sha256": ARCHIVE_SHA256,
             "dependency_score_sha256": SCORE_SHA256, "input_sha256": {},
             "program_sha256": {str(path): sha256(path) for path in
                                (Path(__file__), Path(__file__).with_name("profile.py"),
                                 Path(__file__).with_name("__init__.py"),
                                 Path(__file__).resolve().parents[2] / "scripts/score_20261027.py")},
             "complete_evaluation": False,
             "cost_parameters": {"tsv": args.tsv_cost_ratio, "via": args.via_cost_ratio,
                                 "tsv_congestion": args.tsv_congestion_ratio}}
    code = 2
    phase = "INPUT"
    before = {}
    previous = Path.cwd()
    with tempfile.TemporaryDirectory(prefix=".score-work-", dir=output) as temporary:
        stage = Path(temporary)
        try:
            paths = {}
            for label, original in sources:
                original = original.resolve(strict=True)
                before[str(original)] = sha256(original)
                target = stage / "inputs" / label / ("input.lef" if label.startswith("lef") else "input." + label)
                target.parent.mkdir(parents=True)
                shutil.copyfile(original, target)
                if sha256(target) != before[str(original)]:
                    raise ValueError("Input changed during snapshot: " + label)
                paths[label] = target
                audit["input_sha256"][label] = before[str(original)]
            payload = json.loads(paths["thermal"].read_text(encoding="utf-8-sig"))
            validate_finite(payload)
            del payload
            normalized = {}
            counts, geometry = validate_lef_geometry([paths[f"lef{i}"] for i in range(len(args.lefs))],
                                                    collect=True, normalized_lefs=normalized)
            audit["supported_input_geometry"] = counts
            normalization = {}
            for index in range(len(args.lefs)):
                key = f"lef{index}"
                replacement = normalized.get(paths[key])
                if replacement is not None:
                    target = stage / "canonical_lefs" / (key + ".lef")
                    target.parent.mkdir(exist_ok=True)
                    target.write_text(replacement["text"], encoding="utf-8")
                    paths[key] = target
                    normalization[key] = {"sha256": sha256(target),
                                          "rectangles_normalized": replacement["rectangles_normalized"]}
            audit["private_lef_normalization"] = {"original_inputs_modified": False,
                                                    "canonical_snapshots": normalization,
                                                    "rule": "exact RECT corner ordering and LEF ORIGIN translation"}
            gain, gain_source, case = resolve_gain(args, audit["input_sha256"], dependency)
            audit["thermal_gain_identity"] = {"gain": gain, "source": gain_source, "case": case,
                                                "official_20261027_certified": False}
            phase = "INTERNAL"
            modules = load((args.runtime or dependency / "vendor").resolve())
            common, checker, _, _, score = modules
            options = Options(rules_profile=args.rules_profile, tsv_obstacle_scope=args.tsv_obstacle_scope,
                              tsv_width_um=args.tsv_width_um, tsv_demand_model=args.tsv_demand_model,
                              tsv_pin_exemption=args.tsv_pin_exemption, pin_credit_policy=args.pin_credit_policy,
                              zero_capacity_penalty=args.zero_capacity_penalty,
                              dump_state=stage / "congestion_state.json.gz" if args.dump_congestion_state else None,
                              capacity_boundary=args.capacity_boundary,
                              fixed_track_boundary=args.fixed_track_boundary,
                              landing_track_boundary=args.landing_track_boundary)
            install(modules, audit, geometry, options)
            score.resolve_thermal_gain = lambda table, input_def: (gain, gain_source)
            work = stage / "runtime_work"
            work.mkdir()
            vendor = (args.runtime or dependency / "vendor").resolve()
            for name in ("POST9.dat", "POWV9.dat"):
                (work / name).symlink_to(vendor / name)
            os.chdir(work)
            vendor_args = [str(paths["guide"]), "--lefs", ",".join(str(paths[f"lef{i}"]) for i in range(len(args.lefs))),
                           "--input-def", str(paths["def"]), "--thermal-json", str(paths["thermal"]),
                           "--output-dir", str(stage), "--tsv-cost-ratio", str(args.tsv_cost_ratio),
                           "--via-cost-ratio", str(args.via_cost_ratio),
                           "--tsv-congestion-ratio", str(args.tsv_congestion_ratio)]
            with (stage / "vendor_diagnostics.txt").open("w", encoding="utf-8") as capture:
                with redirect_stdout(capture), redirect_stderr(capture):
                    try:
                        score.run(vendor_args)
                        vendor_exit = 0
                    except SystemExit as error:
                        vendor_exit = int(error.code or 0)
            report = json.loads((stage / "evaluation_report.json").read_text(encoding="utf-8"))
            validate_finite(report)
            passed = report.get("status") == "PASS" and vendor_exit == 0
            violation = not passed and report.get("category") in (checker.LEGALITY, score.EFFICIENCY)
            report["status"] = "PASS" if passed else "FAIL"
            report["evaluation_outcome"] = "SCORED" if passed else "VIOLATION" if violation else "UNSCORABLE"
            report["valid_score"] = passed
            if not passed:
                report["score"] = 0.0 if violation else None
            if passed:
                wire_report = report.setdefault("wirelength", {})
                wire_report["tsv_cost_ratio"] = args.tsv_cost_ratio
                wire_report["tsv_cost_dbu_each"] = args.tsv_cost_ratio * max(
                    audit["congestion"]["gcell_width_dbu"], audit["congestion"]["gcell_height_dbu"])
                score_report = report.get("score")
                if isinstance(score_report, dict):
                    fields = ("wirelength_score", "congestion_score", "thermal_score")
                    if all(key in score_report for key in fields):
                        score_report["quality"] = math.fsum(
                            weight * score_report[key] for weight, key in zip((.45, .30, .25), fields))
                        score_report["quality_source"] = "derived:0.45W+0.30C+0.25T"
            report["efficiency_score"] = None
            report["compatibility_20261027"] = audit["compatibility_20261027"]
            report["thermal_gain_configuration"] = audit["thermal_gain_identity"]
            report["official_equivalence"] = False
            report["calibration_status"] = calibration_status
            report["calibration_scope"] = audit["calibration_scope"]
            report["production_candidate_acceptance_supported"] = False
            report["repair"] = {"version": __version__, "rules": rules, "mode": "20261027",
                                "all_evaluation_stages_completed": passed, "S4": None,
                                "official_final_rank_score": None,
                                "physical_DRC": "not_checked", "solver_resources": "not_derivable_from_guide"}
            audit["complete_evaluation"] = passed
            audit["vendor_exit_code"] = vendor_exit
            code = 0 if passed else 1 if violation else 2
        except A150Violation as error:
            report = {"status":"FAIL","evaluation_outcome":"VIOLATION","valid_score":False,
                      "score":0.0,"category":"LEGALITY","error":str(error),
                      "error_type":type(error).__name__,"official_equivalence":False,
                      "repair":{"version":__version__,"rules":rules,
                                "all_evaluation_stages_completed":False,"S4":None,"official_final_rank_score":None}}
            code = 1
        except Exception as error:
            report = {"status": "FAIL", "evaluation_outcome": "UNSCORABLE", "valid_score": False,
                      "score": None, "category": phase, "error": str(error), "error_type": type(error).__name__,
                      "official_equivalence": False, "repair": {"version": __version__, "rules": rules,
                      "all_evaluation_stages_completed": False, "S4": None, "official_final_rank_score": None}}
            code = 2
        finally:
            os.chdir(previous)
        changed = [path for path, digest in before.items() if not Path(path).is_file() or sha256(path) != digest]
        audit["changed_original_inputs"] = changed
        report["calibration_status"] = calibration_status
        report["production_candidate_acceptance_supported"] = False
        if args.rules_profile == NEW_PROFILE:
            formula_status="UNCONFIRMED_20261029_CONGESTION_FORMULA"
            audit["congestion_formula_status"]=formula_status
            report["congestion_formula_status"]=formula_status
            report["official_20261029_match"]=False
        if changed:
            report.update(status="FAIL", evaluation_outcome="UNSCORABLE", valid_score=False,
                          score=None, category="INTEGRITY", error="An original input changed during evaluation")
            code = 2
        audit.update(exit_code=code, wall_seconds=time.perf_counter() - started,
                     peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                     completed_utc=datetime.now(timezone.utc).isoformat())
        report["inputs"] = {"guide": str(args.guide), "lefs": [str(p) for p in args.lefs],
                            "defs": [str(args.input_def)], "thermal_json": str(args.thermal_json)}
        if not report["valid_score"]:
            for path in stage.glob("*.csv"):
                path.unlink()
        dump(stage / "evaluation_report.json", report)
        dump(stage / "run_record.json", audit)
        (stage / "evaluation_report.txt").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                     encoding="utf-8")
        artifacts = [p for p in stage.iterdir() if p.is_file() and p.name != "evaluation_report.json"]
        for path in artifacts:
            os.replace(path, output / path.name)
        os.replace(stage / "evaluation_report.json", output / "evaluation_report.json")
    print(json.dumps({"status": report["status"], "evaluation_outcome": report["evaluation_outcome"],
                      "valid_score": report["valid_score"], "rule_profile": rules,
                      "report": str(output / "evaluation_report.json")}, ensure_ascii=False))
    return code
