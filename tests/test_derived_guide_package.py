#!/usr/bin/env python3
"""Provenance rejection tests for derived Guides and legacy compatibility."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT, ARTIFACTS = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
ARTIFACTS.mkdir(parents=True, exist_ok=True)
spec = importlib.util.spec_from_file_location("guide_builder", ROOT / "scripts/build_guide_submission.py")
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


class DerivedGuideEvidence(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ARTIFACTS)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.paths = {k: self.root / n for k, n in (("guide", "result.guide"), ("readback", "readback.json"),
                      ("validation", "validation.json"), ("manifest", "manifest.json"))}
        self.origin = self.root / "original.guide"
        self.origin.write_text("original fixture\n")
        self.paths["guide"].write_text("derived fixture\n")
        self.paths["readback"].write_text(json.dumps({"a150": {"enabled": True, "obstacle_scope": "all-layers", "all_layer_violations": 0},
            "congestion_overflow_scale_proxy": 1, "all_adjacent_overlaps": True, "official_equivalence": False,
            "metrics": {"W": 50, "C": 50, "T": 50, "S": 50}, "G": 3}))
        self.paths["validation"].write_text(json.dumps({"status": "PASS", "errors": {"total": 0},
            "a150": {"scope": "all-routing-layers-both-dies", "all_layers_violations": 0}}))
        self.item = {"case": "test", **{k: str(v) for k, v in self.paths.items()}}
        original_hash = builder.digest(self.origin)
        self.record = {"status": "PASS_OFFLINE_ONLY", "platform_run": False, "router_run": False,
            "inputs_unchanged": True, "inputs_before": {str(self.origin): original_hash}, "inputs_after": {str(self.origin): original_hash},
            "guide_sha256": builder.digest(self.paths["guide"]),
            "outputs_sha256": {p.name: builder.digest(p) for k, p in self.paths.items() if k != "manifest"},
            "events": [{"name": "optimize", "exit": 0, "argv": ["optimizer", "--guide", str(self.origin), "--output", str(self.paths["guide"])]},
                       {"name": "readback", "exit": 0, "argv": ["measure", "--guide", str(self.paths["guide"]), "--output", str(self.paths["readback"])]},
                       {"name": "validate", "exit": 0, "argv": ["validator", "--guide", str(self.paths["guide"]), "--report", str(self.paths["validation"])]}]}

    def check(self, record=None):
        self.paths["manifest"].write_text(json.dumps(self.record if record is None else record))
        return builder.candidate(self.item, self.root)

    def test_actual_stages_without_fictional_router_run(self):
        self.assertEqual(self.check()["provenance"]["kind"], "frozen-guide-postprocess")

    def test_reject_changed_guide(self):
        self.paths["guide"].write_text("tampered\n")
        with self.assertRaises(ValueError): self.check()

    def test_reject_rebound_readback(self):
        r = copy.deepcopy(self.record);r["events"][1]["argv"][2] = str(self.origin)
        with self.assertRaises(ValueError): self.check(r)

    def test_reject_changed_origin(self):
        self.origin.write_text("changed source\n")
        with self.assertRaises(ValueError): self.check()

    def test_reject_failed_validation(self):
        r = copy.deepcopy(self.record);r["events"][2]["exit"] = 1
        with self.assertRaises(ValueError): self.check(r)

    def test_reject_changed_input_record(self):
        r = copy.deepcopy(self.record);r["inputs_after"] = {}
        with self.assertRaises(ValueError): self.check(r)

    def test_reject_changed_evidence_bytes(self):
        with self.paths["readback"].open("a") as out: out.write("\n")
        with self.assertRaises(ValueError): self.check()

    def test_legacy_success_still_supported(self):
        self.paths["manifest"].write_text(f"{builder.digest(self.paths['guide'])}  {self.paths['guide']}\nroute_exit=0\nscore_exit=0\nvalidate_exit=0\n")
        self.assertEqual(builder.candidate(self.item, self.root)["provenance"]["kind"], "historical-router-manifest")


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
