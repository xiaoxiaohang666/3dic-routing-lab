"""Synthetic physical oracles; run on benchmark-host, never on the local workstation."""
from __future__ import annotations

from copy import deepcopy
import gzip
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
DEPENDENCY = Path(os.environ.get("SCORER_20261027_DEPENDENCY_ROOT",
                                ROOT / "references/community-score-0.4.2"))
sys.path.insert(0, str(DEPENDENCY))
from scorer_20261027.profile import (A150Violation, Options, congestion, covered_cells,
                                     first_layer_credit_source, physical_track_count,
                                     tsv_footprint_demands, interval_track_count,
                                     set_capacity_boundary, landing_tsv_demands,
                                     a150_mask_cells, full_grid_boundaries, install_full_grid_boundaries,
                                     check_a150_guide_geometry, install_a150_geometry, pin_zero_track_floor, configuration)


def historical_options(**kwargs):
    """Keep the archived diagnostic profile explicit in pre-A150 regressions."""
    return Options(rules_profile="inferred-platform", **kwargs)


def a150_options(**kwargs):
    """Frozen pre-1029 PIN-floor A150 contract stays explicit."""
    return Options(rules_profile="a150", **kwargs)


class Pattern:
    def __init__(self, coords):
        self.coords = coords

    def coordinates(self):
        return self.coords


class GeometryOracles(unittest.TestCase):
    def make_run(self, *, components=(), guides=None, pins=None):
        grid = NS(x_lines=(0, 3, 6), y_lines=(0, 3, 6), rows=2, cols=2,
                  die=NS(x1=0, y1=0, x2=6, y2=6))
        tracks = {"metal10": {"Y": [Pattern(tuple(range(7)))]},
                  "metal11": {"X": [Pattern(tuple(range(7)))]}}
        run = NS(gcell_grid=grid, def_data=NS(tracks=tracks), gcell_width=3, gcell_height=3,
                 layer_directions={"metal10": "HORIZONTAL", "metal11": "VERTICAL"},
                 def_definition=NS(GetDatabaseUnits=lambda: 1, getComponents=lambda: components),
                 net_guides=guides or {}, _repair_pin_cells=pins or {}, _repair_io_positions={},
                 tsv_congestion_ratio=1.)
        common = NS(rectangle_cell_indices=covered_cells)
        return run, common

    def capacity(self, run):
        from evaluation.repaired.core import Capacity
        return Capacity(run.def_data.tracks, run.gcell_grid)

    def point(self, net="n", x=3, y=3):
        return NS(net=net, x_dbu=x, y_dbu=y, bottom_layer="metal10", top_layer="metal11",
                  gcell_col=1, gcell_row=1)

    def test_five_micron_footprint_crosses_cells_and_counts_both_layers(self):
        run, _ = self.make_run()
        got = dict(tsv_footprint_demands(self.point(), run.gcell_grid, self.capacity(run),
                                        run.layer_directions, 1))
        # [0.5,5.5) intersects two rows/cols; H counts 2 then 3, V likewise.
        self.assertEqual(got, {(0, 0, "metal10"): 2, (1, 0, "metal10"): 2,
                               (0, 1, "metal10"): 3, (1, 1, "metal10"): 3,
                               (0, 0, "metal11"): 2, (0, 1, "metal11"): 2,
                               (1, 0, "metal11"): 3, (1, 1, "metal11"): 3})
        self.assertEqual(sum(got.values()), 20)

    def test_physical_upper_edge_excluded_but_interior_final_track_retained(self):
        tracks = tuple(range(7))
        self.assertEqual(physical_track_count(tracks, 0, 6, 3, 6, final=True), 3)
        self.assertEqual(physical_track_count(tracks, -1, 7, 3, 6, final=True), 4)
        self.assertEqual(physical_track_count(tracks, 1, 3, 0, 3), 2)
        self.assertEqual(physical_track_count(tracks, 1, 3, 3, 6, final=True), 0)

    def test_die_clipping_does_not_clamp_outside_tracks_into_edge_cells(self):
        run, _ = self.make_run()
        got = dict(tsv_footprint_demands(self.point(x=1, y=1), run.gcell_grid,
                                        self.capacity(run), run.layer_directions, 1))
        self.assertEqual(sum(v for k, v in got.items() if k[2] == "metal10"), 8)
        self.assertEqual(sum(v for k, v in got.items() if k[2] == "metal11"), 8)
        self.assertEqual(list(covered_cells(-8, -2, run.gcell_grid.x_lines)), [])
        self.assertEqual(list(covered_cells(6, 8, run.gcell_grid.x_lines)), [])

    def test_cell_track_and_ceil_are_explicit_distinct_models(self):
        run, _ = self.make_run()
        cap = self.capacity(run)
        proportional = dict(tsv_footprint_demands(self.point(), run.gcell_grid, cap,
                                                 run.layer_directions, 1, model="cell-track"))
        rounded = dict(tsv_footprint_demands(self.point(), run.gcell_grid, cap,
                                            run.layer_directions, 1, model="ceil"))
        self.assertAlmostEqual(proportional[0, 1, "metal10"], 10 / 3)
        self.assertEqual(rounded[0, 1, "metal10"], 4)
        self.assertAlmostEqual(proportional[1, 0, "metal11"], 10 / 3)

    def test_multilayer_fixed_pin_uses_each_original_layer(self):
        comp = NS(macro_name="M", x=0, y=0, orient=0)
        run, common = self.make_run(components=[comp])
        box = ("0", "0", "2", "2")
        source = {"M": {"size": ("3", "3"),
                        "rectangles": [("metal10", box), ("metal11", box)],
                        "pins": {"P": [("metal10", box), ("metal11", box)]}}}
        original = deepcopy(source)
        audit = {}
        congestion(run, [], common, audit, source, historical_options())
        self.assertEqual(audit["congestion"]["fixed_track_demand_by_layer"],
                         {"metal10": 2, "metal11": 2})
        self.assertEqual(source, original)

    def test_first_layer_credit_keeps_geometry_without_projecting_other_shapes(self):
        source = {"M": {"size": ("3", "3"), "rectangles": [],
                        "pins": {"P": [("metal11", (0, 0, 1, 1)),
                                       ("metal10", (1, 1, 2, 2)),
                                       ("metal11", (2, 2, 3, 3))]}}}
        original = deepcopy(source)
        credit = first_layer_credit_source(source)
        self.assertEqual(credit["M"]["pins"]["P"],
                         [("metal11", (0, 0, 1, 1)), ("metal11", (2, 2, 3, 3))])
        self.assertEqual(source, original)

    def test_tsv_pin_exemption_is_per_net_cell_and_layer(self):
        run, common = self.make_run(pins={"n": {(0, 0, "metal10")}})
        audit = {}
        congestion(run, [self.point()], common, audit, {}, historical_options())
        self.assertEqual(audit["congestion"]["tsv_demand"], 18)
        self.assertEqual(audit["congestion"]["own_pin_tsv_exempt_demand"], 2)
        audit = {}
        congestion(run, [self.point(net="other")], common, audit, {}, historical_options())
        self.assertEqual(audit["congestion"]["tsv_demand"], 20)
        audit = {}
        congestion(run, [self.point()], common, audit, {}, historical_options(tsv_pin_exemption="none"))
        self.assertEqual(audit["congestion"]["tsv_demand"], 20)

    def test_distinct_tsv_instances_are_additive(self):
        run, common = self.make_run()
        audit = {}
        congestion(run, [self.point("a"), self.point("b")], common, audit, {}, historical_options())
        self.assertEqual(audit["congestion"]["tsv_demand"], 40)

    def test_route_same_net_cell_dedup_and_pin_exemption(self):
        rect = NS(xl=0, yl=0, xh=3, yh=3, layer="metal10")
        run, common = self.make_run(guides={"n": [rect, rect]})
        audit = {}
        congestion(run, [], common, audit, {}, historical_options())
        self.assertEqual(audit["congestion"]["guide_net_cell_demand"], 1)
        run._repair_pin_cells = {"n": {(0, 0, "metal10")}}
        congestion(run, [], common, audit, {}, historical_options())
        self.assertEqual(audit["congestion"]["guide_net_cell_demand"], 0)
        self.assertEqual(audit["congestion"]["own_pin_guide_exempt_demand"], 1)

    def test_capture_preserves_replayable_geometry_and_masks(self):
        run, common = self.make_run()
        run._scorer_20261027_pin_masks = {"all-layers": {"n": {(0, 0, "metal10")}},
                                         "first-layer": {"n": set()}}
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "state.json.gz"
            congestion(run, [self.point()], common, {}, {}, historical_options(dump_state=target))
            with gzip.open(target, "rt", encoding="utf-8") as stream:
                state = json.load(stream)
            self.assertEqual(state["tsv_points"][0]["x_dbu"], 3)
            self.assertEqual(state["tsv_points"][0]["net"], "n")
            self.assertEqual(state["pin_masks"]["all-layers"]["n"], [[0, 0, "metal10"]])
            self.assertFalse(state["official_equivalence"])
            self.assertEqual(state["tracks"]["metal10"]["Y"], list(range(7)))

    def test_rejects_undefined_model_parameters(self):
        for kwargs in ({"tsv_width_um": 0}, {"tsv_width_um": float("nan")},
                       {"tsv_demand_model": "case-special"}, {"zero_capacity_penalty": 0}):
            with self.assertRaises(ValueError):
                historical_options(**kwargs)

    def test_boundary_capacity_and_interior_reservation_are_distinct(self):
        run,_ = self.make_run()
        cap = self.capacity(run)
        self.assertEqual(cap[0,0,"metal10","HORIZONTAL"],3)
        set_capacity_boundary(cap,run.gcell_grid,"closed")
        self.assertEqual(cap[0,0,"metal10","HORIZONTAL"],4)
        self.assertEqual(interval_track_count(tuple(range(7)),3,6,"open"),2)
        self.assertEqual(interval_track_count(tuple(range(7)),3,6,"closed"),4)

    def test_half_open_shared_boundary_track_has_one_owner(self):
        run, _ = self.make_run()
        # A physical track on the shared edge belongs only to the upper band.
        run.def_data.tracks = {"metal10": {"Y": [Pattern((3,))]},
                               "metal11": {"X": [Pattern((3,))]}}
        cap = self.capacity(run)
        set_capacity_boundary(cap, run.gcell_grid, "half-open")
        self.assertEqual([cap[0, row, "metal10", "HORIZONTAL"] for row in range(2)],
                         [0, 1])
        self.assertEqual([cap[col, 0, "metal11", "VERTICAL"] for col in range(2)],
                         [0, 1])

    def test_half_open_capacity_conserves_tracks_on_nonuniform_grid(self):
        run, _ = self.make_run()
        run.gcell_grid = NS(x_lines=(0, 2, 7, 10), y_lines=(0, 4, 9), rows=2, cols=3,
                           die=NS(x1=0, y1=0, x2=10, y2=9))
        run.def_data.tracks = {
            "metal10": {"Y": [Pattern((-1, 0, 4, 5, 9, 10))]},
            "metal11": {"X": [Pattern((-1, 0, 2, 3, 7, 10, 11))]}}
        cap = self.capacity(run)
        horizontal = [cap[0, row, "metal10", "HORIZONTAL"] for row in range(2)]
        vertical = [cap[col, 0, "metal11", "VERTICAL"] for col in range(3)]
        self.assertEqual(horizontal, [1, 3])
        self.assertEqual(vertical, [1, 2, 2])
        # Each in-DIE physical track appears once across perpendicular bands.
        self.assertEqual(sum(horizontal), 4)
        self.assertEqual(sum(vertical), 5)

    def test_duplicate_track_declarations_do_not_add_capacity(self):
        run, _ = self.make_run()
        patterns = [Pattern((0, 1, 2, 3)), Pattern((2, 3, 4, 5, 6)),
                    Pattern((0, 1, 2, 3))]
        run.def_data.tracks = {"metal10": {"Y": patterns},
                               "metal11": {"X": patterns}}
        cap = self.capacity(run)
        self.assertEqual(cap.tracks["metal10"]["Y"], list(range(7)))
        self.assertEqual(cap.tracks["metal11"]["X"], list(range(7)))
        self.assertEqual([cap[0, row, "metal10", "HORIZONTAL"] for row in range(2)],
                         [3, 4])
        self.assertEqual([cap[col, 0, "metal11", "VERTICAL"] for col in range(2)],
                         [3, 4])

    def test_half_open_final_die_edge_is_retained_without_outside_tracks(self):
        run, _ = self.make_run()
        run.def_data.tracks = {"metal10": {"Y": [Pattern((-1, 0, 6, 7))]},
                               "metal11": {"X": [Pattern((-1, 0, 6, 7))]}}
        cap = self.capacity(run)
        # Interior bands are [lo, hi); only the exterior DIE endpoint is closed.
        for layer, axis in (("metal10", "Y"), ("metal11", "X")):
            self.assertEqual(cap.tracks[layer][axis], [0, 6])
        self.assertEqual([cap[0, row, "metal10", "HORIZONTAL"] for row in range(2)],
                         [1, 1])
        self.assertEqual([cap[col, 0, "metal11", "VERTICAL"] for col in range(2)],
                         [1, 1])

    def test_cli_defaults_to_unique_track_capacity_and_keeps_closed_diagnostic(self):
        from scorer_20261027.cli import create_parser
        parser = create_parser(DEPENDENCY)
        arguments = ["input.guide", "--lefs", "input.lef", "--input-def", "input.def",
                     "--thermal-json", "thermal.json", "--output-dir", "new-result"]
        self.assertEqual(parser.parse_args(arguments).capacity_boundary, "half-open")
        self.assertEqual(parser.parse_args(arguments).rules_profile, "a150-20261029")
        self.assertEqual(parser.parse_args(arguments+["--rules-profile","a150"]).rules_profile,"a150")
        self.assertEqual(parser.parse_args(arguments).fixed_track_boundary, "closed")
        self.assertEqual(parser.parse_args(arguments).landing_track_boundary, "closed")
        self.assertEqual(parser.parse_args(arguments).tsv_pin_exemption, "none")
        self.assertEqual(parser.parse_args(arguments).thermal_gain_policy, "default20")
        old = parser.parse_args(arguments + ["--rules-profile", "inferred-platform"])
        self.assertEqual(old.fixed_track_boundary,"half-open")
        self.assertEqual(old.landing_track_boundary,"open")
        self.assertEqual(old.tsv_pin_exemption,"own-pin")
        self.assertEqual(old.thermal_gain_policy,"archived-identity")
        self.assertEqual(parser.parse_args(arguments + ["--capacity-boundary", "closed"])
                         .capacity_boundary, "closed")

    def test_landing_tsv_does_not_recharge_own_guide(self):
        guides={"n":[NS(xl=3,yl=3,xh=6,yh=6,layer=layer)
                     for layer in ("metal10","metal11")]}
        run,common=self.make_run(guides=guides)
        options=historical_options(tsv_demand_model="landing-union",capacity_boundary="closed",
                        fixed_track_boundary="open",landing_track_boundary="open")
        audit={}
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,audit,{},options)
        # Two interior tracks per landing layer; one Guide track was already paid.
        self.assertEqual(audit["congestion"]["tsv_demand"],2)
        self.assertEqual(result[4][1,1,"metal10"],2)
        self.assertEqual(result[4][1,1,"metal11"],2)

    def test_landing_tsv_extends_partial_pin_reservation_without_double_charge(self):
        component=NS(macro_name="M",x=3,y=3,orient=0)
        guides={"n":[NS(xl=3,yl=3,xh=6,yh=6,layer=layer)
                     for layer in ("metal10","metal11")]}
        run,common=self.make_run(components=[component],guides=guides,
                                 pins={"n":{(1,1,"metal10")}})
        box=("0","0","2","2")
        source={"M":{"size":("3","3"),"rectangles":[("metal10",box)],
                     "pins":{"P":[("metal10",box)]}}}
        audit={}
        options=historical_options(tsv_demand_model="landing-union",capacity_boundary="closed",
                        fixed_track_boundary="open",landing_track_boundary="open")
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,audit,source,options)
        # PIN reserves y=4 only; TSV landing adds y=5. The owned Guide stays exempt.
        self.assertEqual(result[4][1,1,"metal10"],2)
        self.assertEqual(audit["congestion"]["fixed_track_demand"],1)
        self.assertEqual(audit["congestion"]["tsv_demand"],2)

    def test_landing_reservation_clips_to_actual_entity_when_gcell_is_larger(self):
        run,_=self.make_run()
        cap=self.capacity(run)
        point=self.point(x=4.5,y=4.5)
        result=dict(landing_tsv_demands(point,run.gcell_grid,cap,run.layer_directions,
                                        1,.5,"open"))
        self.assertEqual(result,{(1,1,"metal10"):0,(1,1,"metal11"):0})


    def a150_guides(self):
        return {"n":[NS(xl=3,yl=3,xh=6,yh=6,layer=layer)
                     for layer in ("metal10","metal11")]}

    def test_a150_direct_tracks_replace_guide_and_clip_to_landing(self):
        run,common=self.make_run(guides=self.a150_guides())
        audit={}
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,audit,{},a150_options())
        self.assertEqual(result[4],{(1,1,"metal10"):4,(1,1,"metal11"):4})
        self.assertEqual(audit["congestion"]["tsv_demand"],8)
        self.assertEqual(audit["congestion"]["guide_net_cell_demand"],0)
        self.assertEqual(audit["congestion"]["own_pin_tsv_exempt_demand"],0)
        self.assertEqual(audit["a150_legality"]["violations"],0)

    def test_a150_resource_merge_has_no_tsv_own_pin_credit(self):
        # Deliberately inject a credit mask without fixed geometry: legality
        # and resource accounting are separate contracts, both exercised.
        run,common=self.make_run(guides=self.a150_guides(),pins={"n":{(1,1,"metal10")}})
        audit={}
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,audit,{},a150_options())
        self.assertEqual(result[4][1,1,"metal10"],4)
        self.assertEqual(audit["congestion"]["own_pin_tsv_exempt_demand"],0)

    def test_a150_zero_track_marker_removes_the_ordinary_one(self):
        run,common=self.make_run(guides=self.a150_guides())
        run.def_data.tracks["metal11"]["X"]=[Pattern(())]
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,{}, {},a150_options())
        self.assertEqual(result[4],{(1,1,"metal10"):4})
        self.assertNotIn((1,1,"metal11"),result[4])

    def test_a150_internal_cell_boundary_track_has_one_owner(self):
        run,_=self.make_run()
        cap=self.capacity(run)
        low=self.point(x=1.5,y=1.5);low.gcell_col=low.gcell_row=0
        lower=dict(landing_tsv_demands(low,run.gcell_grid,cap,run.layer_directions,1,5,"closed",a150=True))
        upper=dict(landing_tsv_demands(self.point(x=4.5,y=4.5),run.gcell_grid,cap,
                                      run.layer_directions,1,5,"closed",a150=True))
        self.assertEqual(lower,{(0,0,"metal10"):3,(0,0,"metal11"):3})
        self.assertEqual(upper,{(1,1,"metal10"):4,(1,1,"metal11"):4})

    def test_a150_pin_obs_geometric_mask_does_not_require_covered_tracks(self):
        component=NS(macro_name="M",x=3,y=3,orient=0)
        run,common=self.make_run(components=[component],guides=self.a150_guides())
        box=("0.25","0.25","0.5","0.5")
        source={"M":{"size":("3","3"),"rectangles":[("metal10",box)],"pins":{}}}
        audit={}
        with self.assertRaises(A150Violation):
            congestion(run,[self.point(x=4.5,y=4.5)],common,audit,source,a150_options())
        self.assertEqual(audit["a150_legality"]["examples"][0]["layers"],["metal10"])
        self.assertTrue(audit["a150_legality"]["track_independent"])

    def test_a150_all_layers_default_and_interface_alternative(self):
        component=NS(macro_name="M",x=3,y=3,orient=0)
        run,common=self.make_run(components=[component],guides=self.a150_guides())
        run.layer_directions["metal3"]="HORIZONTAL"
        box=("0.25","0.25","0.5","0.5")
        source={"M":{"size":("3","3"),"rectangles":[("metal3",box)],"pins":{"P":[("metal3",box)]}}}
        with self.assertRaises(A150Violation):
            congestion(run,[self.point(x=4.5,y=4.5)],common,{},source,a150_options())
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,{},source,
                          a150_options(tsv_obstacle_scope="interface"))
        self.assertEqual(result[4][1,1,"metal10"],4)

    def test_a150_cross_net_same_cell_is_a_hard_violation(self):
        run,common=self.make_run(guides=self.a150_guides())
        audit={}
        with self.assertRaises(A150Violation):
            congestion(run,[self.point("n"),self.point("other")],common,audit,{},a150_options())
        self.assertEqual(audit["a150_legality"]["examples"][0]["kind"],"CROSS_NET_TSV_GCELL")
        self.assertEqual(audit["a150_legality"]["pin_obs_rule_violations"],0)
        self.assertEqual(audit["a150_legality"]["conservative_exclusive_policy_violations"],1)

    def test_a150_fixed_shape_closed_but_internal_cell_upper_open(self):
        component=NS(macro_name="M",x=0,y=0,orient=0)
        run,common=self.make_run(components=[component])
        box=("0","0","2","4")
        source={"M":{"size":("3","6"),"rectangles":[("metal10",box)],"pins":{}}}
        result=congestion(run,[],common,{},source,a150_options())
        self.assertEqual(result[4],{(0,0,"metal10"):3,(0,1,"metal10"):2})

    def test_a150_rejects_legacy_resource_overrides(self):
        for kwargs in ({"tsv_pin_exemption":"own-pin"},{"capacity_boundary":"closed"},
                       {"landing_track_boundary":"open"},{"tsv_demand_model":"physical-tracks"}):
            with self.assertRaises(ValueError):
                a150_options(**kwargs)


class GeometryFailure(ValueError):
    def __init__(self, message, **kwargs):
        super().__init__(message)
        self.detail=kwargs


class FullGridOracles(unittest.TestCase):
    make_run=GeometryOracles.make_run
    capacity=GeometryOracles.capacity
    point=GeometryOracles.point
    # Keep this class independent of native vendor imports; the real runtime
    # integration is checked separately on the frozen exported Guides.
    def geometry_checker(self):
        return NS(EvaluationError=GeometryFailure,LEGALITY="LEGALITY")

    def grid(self, x=(0,3,6,9), y=(0,3,6,9), die=(0,0,7,7)):
        return NS(x_lines=x,y_lines=y,cols=len(x)-1,rows=len(y)-1,
                  die=NS(x1=die[0],y1=die[1],x2=die[2],y2=die[3]))

    def test_full_grid_overshoot_is_preserved_without_inserting_DIE_edges(self):
        original=(-3,0,3,6,9,12)
        self.assertEqual(full_grid_boundaries(original,0,7),original)
        self.assertNotIn(7,full_grid_boundaries(original,0,7))
        score=NS(expand_grid_lines=lambda statements,low,high,axis:(original,["original-warning"]))
        audit={};install_full_grid_boundaries(score,audit)
        lines,warnings=score.expand_grid_lines(object(),0,7,"X")
        self.assertEqual(lines,original)
        self.assertEqual(warnings,["original-warning"])
        self.assertTrue(audit["effective_gcell_grid"]["X"]["outside_DIE_boundaries_preserved"])
        self.assertEqual(audit["effective_gcell_grid"]["X"]["last"],12)

    def test_full_grid_undershoot_adds_only_missing_envelope_ends(self):
        self.assertEqual(full_grid_boundaries((0,3,6),-2,8),(-2,0,3,6,8))
        self.assertEqual(full_grid_boundaries((-3,0,3,6,9),-2,8),(-3,0,3,6,9))
        self.assertEqual(full_grid_boundaries((0,3,6),0,6),(0,3,6))

    def test_DIE_clipped_last_parent_cell_is_legal_without_full_grid_DIE_line(self):
        grid=self.grid();audit={}
        raw={"n":((0,6,7,7,"metal10"),(6,0,7,7,"metal11"))}
        check_a150_guide_geometry(raw,grid,{"metal10":"HORIZONTAL","metal11":"VERTICAL"},
                                  self.geometry_checker(),audit)
        self.assertEqual(audit["a150_guide_geometry"]["violations"],0)
        self.assertEqual(grid.x_lines,(0,3,6,9))
        self.assertNotIn(7,grid.x_lines)

    def test_nonzero_negative_DIE_origin_keeps_parent_grid_intervals(self):
        grid=self.grid(x=(-3,0,3,6,9),y=(-3,0,3,6,9),die=(-2,-1,7,8))
        raw={"n":((-2,-1,7,0,"metal10"),(-2,-1,0,8,"metal11"))}
        check_a150_guide_geometry(raw,grid,{"metal10":"HORIZONTAL","metal11":"VERTICAL"},
                                  self.geometry_checker(),{})
        self.assertEqual(grid.x_lines[0],-3)
        self.assertEqual(grid.die.x1,-2)

    def test_internal_shift_multicell_and_raw_outside_DIE_are_rejected(self):
        grid=self.grid();checker=self.geometry_checker()
        for bounds in ((0,.5,6,3.5),(0,0,6,6),(0,6,9,7)):
            audit={}
            with self.assertRaises(GeometryFailure):
                check_a150_guide_geometry({"n":((*bounds,"metal10"),)},grid,
                                          {"metal10":"HORIZONTAL"},checker,audit)
            self.assertEqual(audit["a150_guide_geometry"]["violations"],1)
            self.assertFalse(audit["a150_guide_geometry"]["source_geometry_clipped_or_discarded"])

    def test_full_final_band_capacity_is_separate_from_DIE_clipped_TSV_tracks(self):
        run,common=self.make_run()
        run.gcell_grid=self.grid()
        run.def_data.tracks={"metal10":{"Y":[Pattern((6,7,8,9))]},
                             "metal11":{"X":[Pattern((6,7,8,9))]}}
        cap=self.capacity(run)
        self.assertEqual(cap[2,2,"metal10","HORIZONTAL"],4)
        self.assertEqual(cap[2,2,"metal11","VERTICAL"],4)
        run.net_guides={"n":[NS(xl=6,yl=6,xh=7,yh=7,layer=layer)
                              for layer in ("metal10","metal11")]}
        point=self.point(x=6.5,y=6.5);point.gcell_col=point.gcell_row=2
        audit={};result=congestion(run,[point],common,audit,{},a150_options())
        # Capacity is the declared full band [6,9], while the TSV is clipped
        # to DIE high=7 and covers only tracks 6 and 7. Never mix the grids.
        self.assertEqual(result[4],{(2,2,"metal10"):2,(2,2,"metal11"):2})
        self.assertEqual(audit["congestion"]["tsv_demand"],4)
        self.assertEqual(audit["a150_legality"]["violations"],0)

    def test_geometric_mask_clips_outside_shapes_and_handles_degenerate_edges(self):
        grid=self.grid()
        self.assertEqual(list(a150_mask_cells((7,6,8,7),grid)),[])
        self.assertEqual(list(a150_mask_cells((6.5,6.5,8,8),grid)),[(2,2)])
        self.assertEqual(list(a150_mask_cells((6,6,6,6),grid)),[(2,2)])
        self.assertEqual(list(a150_mask_cells((7,7,7,7),grid)),[(2,2)])
        self.assertEqual(list(a150_mask_cells((8,8,8,8),grid)),[])

    def test_install_shares_full_grid_with_resources_and_thermal_vector(self):
        from evaluation.repaired import core
        original=(-3,0,3,6,9,12)
        seen=[]
        common=NS(parse_guide=lambda path:{"n":[NS(xl=0,yl=0,xh=7,yh=3,layer="metal10")]})
        checker=self.geometry_checker();checker.check=lambda run:seen.append(run.gcell_grid)
        score=NS(expand_grid_lines=lambda statements,low,high,axis:(original,[]),
                 compute_score=lambda *args:{},TSVPoint=NS)
        audit={}
        with patch.object(core,"install",lambda *args,**kwargs:None):
            install_a150_geometry((common,checker,NS(),NS(),score),audit,{},
                                   lambda run,*args:seen.append(run.gcell_grid))
        x,_=score.expand_grid_lines(object(),0,7,"X")
        y,_=score.expand_grid_lines(object(),0,7,"Y")
        grid=self.grid(x=x,y=y)
        run=NS(gcell_grid=grid,layer_directions={"metal10":"HORIZONTAL"},
               def_definition=NS(getNets=lambda:[NS(name="n")]))
        common.parse_guide("unused")
        score.check(run)
        # A clipped grid would have 3x3 cells; this full grid has 5x5.
        thermal_vector=[0.]*(run.gcell_grid.rows*run.gcell_grid.cols)
        self.assertEqual(len(thermal_vector),25)
        self.assertEqual(audit["resource_grid_shape"]["thermal_vector_cells"],25)
        self.assertEqual(audit["resource_grid_shape"]["resource_cells"],25)
        from evaluation.repaired.legality import install_grid_boundaries
        legacy_score=NS(expand_grid_lines=lambda statements,low,high,axis:(original,[]))
        install_grid_boundaries(legacy_score,{})
        clipped,_=legacy_score.expand_grid_lines(object(),0,7,"X")
        self.assertEqual(clipped,(0,3,6,7))
        self.assertEqual((len(clipped)-1)**2,9)
        # This contrast uses the unchanged historical dependency policy and
        # guards against accidentally sending its clipped grid to A150 heat.
        self.assertNotEqual(len(thermal_vector),(len(clipped)-1)**2)
        self.assertTrue(all(value is grid for value in seen))
        self.assertIs(score.check,checker.check)


class PinFloor1029Oracles(unittest.TestCase):
    make_run=GeometryOracles.make_run
    capacity=GeometryOracles.capacity
    point=GeometryOracles.point
    a150_guides=GeometryOracles.a150_guides

    def fixture(self, pins, rectangles=None, *, orientation=0):
        component=NS(macro_name="M",instance_name="u",x=0,y=0,orient=orientation)
        run,common=self.make_run(components=[component])
        if rectangles is None:
            rectangles=[shape for shapes in pins.values() for shape in shapes]
        source={"M":{"size":("6","6"),"rectangles":rectangles,"pins":pins}}
        return run,common,source

    def test_new_default_adds_zero_track_pin_floor_but_preserves_old_a150(self):
        box=(".2",".2",".4",".4")
        run,common,source=self.fixture({"P":[("metal10",box)]})
        old_audit={};new_audit={}
        old=congestion(run,[],common,old_audit,source,a150_options())
        new=congestion(run,[],common,new_audit,source,Options())
        self.assertEqual(old[4],{})
        self.assertEqual(new[4],{(0,0,"metal10"):1})
        self.assertNotIn("pin_zero_track_floor",old_audit)
        self.assertEqual(new_audit["pin_zero_track_floor"]["virtual_floor_demand"],1)
        self.assertEqual(Options().rules_profile,"a150-20261029")

    def test_zero_track_OBS_does_not_receive_a_floor(self):
        box=(".2",".2",".4",".4")
        run,common,source=self.fixture({},[("metal10",box)])
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{})
        self.assertEqual(audit["pin_zero_track_floor"]["virtual_floor_demand"],0)

    def test_all_rectangles_of_one_logical_pin_union_before_floor(self):
        zero=(".2",".2",".4",".4");real=(".2","1",".4","1.2")
        run,common,source=self.fixture({"P":[("metal10",zero),("metal10",zero),("metal10",real)]})
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{(0,0,"metal10"):1})
        self.assertEqual(audit["pin_zero_track_floor"]["virtual_floor_demand"],0)
        run,common,source=self.fixture({"P":[("metal10",zero),("metal10",zero)]})
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{(0,0,"metal10"):1})
        self.assertEqual(audit["pin_zero_track_floor"]["logical_pin_layer_cell_entries"],1)

    def test_multilayer_pin_floor_uses_actual_layers(self):
        box=(".2",".2",".4",".4")
        run,common,source=self.fixture({"P":[("metal10",box),("metal11",box)]})
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{(0,0,"metal10"):1,(0,0,"metal11"):1})
        self.assertEqual(audit["pin_zero_track_floor"]["floor_demand_by_layer"],{"metal10":1,"metal11":1})

    def test_distinct_same_net_pin_floors_remain_fixed_and_only_one_guide_is_credited(self):
        box=(".2",".2",".4",".4")
        run,common,source=self.fixture({"P":[("metal10",box)],"Q":[("metal10",box)]})
        run._repair_pin_cells={"n":{(0,0,"metal10")}}
        run.net_guides={"n":[NS(xl=0,yl=0,xh=3,yh=3,layer="metal10")]}
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{(0,0,"metal10"):2})
        self.assertEqual(audit["congestion"]["fixed_track_demand"],2)
        self.assertEqual(audit["congestion"]["guide_net_cell_demand"],0)
        self.assertEqual(audit["congestion"]["own_pin_guide_exempt_demand"],1)
        self.assertEqual(audit["pin_zero_track_floor"]["virtual_floor_demand"],2)

    def test_virtual_floor_is_not_hidden_by_other_PIN_or_OBS_real_track_union(self):
        zero=(".2",".2",".4",".4");real=("0","0","2","1")
        pins={"P":[("metal10",zero)],"Q":[("metal10",real)]}
        run,common,source=self.fixture(pins,[("metal10",zero),("metal10",real),("metal10",real)])
        audit={};result=congestion(run,[],common,audit,source,Options())
        # Real tracks y=0,1 are globally unioned once. P has no own tracks,
        # so its independent virtual floor adds a third fixed resource unit.
        self.assertEqual(result[4],{(0,0,"metal10"):3})
        self.assertEqual(audit["pin_zero_track_floor"]["virtual_floor_demand"],1)
        self.assertEqual(congestion(run,[],common,{},source,a150_options())[4],{(0,0,"metal10"):2})

    def test_unconnected_IO_pin_rectangles_share_logical_identity(self):
        run,common=self.make_run()
        box=NS(xl=.2,yl=.2,xh=.4,yh=.4)
        run._repair_io_positions={"unconnected":[(box,"metal10","a"),(box,"metal10","b")],
                                  "supply":[(box,"metal10","c")]}
        audit={};result=congestion(run,[],common,audit,{},Options())
        self.assertEqual(result[4],{(0,0,"metal10"):2})
        self.assertEqual(audit["pin_zero_track_floor"]["logical_pin_instances"],2)

    def test_one_logical_pin_spanning_two_cells_has_two_independent_floors(self):
        box=(".2",".2","5.8",".4")
        run,common,source=self.fixture({"P":[("metal10",box)]})
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{(0,0,"metal10"):1,(1,0,"metal10"):1})
        self.assertEqual(audit["pin_zero_track_floor"]["virtual_floor_demand"],2)

    def test_zero_capacity_pin_floor_is_not_capped_and_zero_track_TSV_stays_zero(self):
        box=(".2",".2",".4",".4")
        run,common,source=self.fixture({"P":[("metal10",box)]})
        run.def_data.tracks["metal10"]["Y"]=[Pattern(())]
        audit={};result=congestion(run,[],common,audit,source,Options())
        self.assertEqual(result[4],{(0,0,"metal10"):1})
        self.assertEqual(audit["congestion"]["zero_capacity_demand_cells"],1)
        run,common=self.make_run(guides=self.a150_guides())
        run.def_data.tracks["metal11"]["X"]=[Pattern(())]
        result=congestion(run,[self.point(x=4.5,y=4.5)],common,{}, {},Options())
        self.assertNotIn((1,1,"metal11"),result[4])

    def test_degenerate_pin_floor_and_orientation_use_fixed_resource_coordinates(self):
        box=(".2",".2",".2",".4")
        run,common,source=self.fixture({"P":[("metal10",box)]})
        floor,detail=pin_zero_track_floor(run,self.capacity(run),source)
        self.assertEqual(floor,{(0,0,"metal10"):1})
        box=(".2",".2",".4",".4")
        run,common,source=self.fixture({"P":[("metal10",box)]},orientation=1)
        floor,_=pin_zero_track_floor(run,self.capacity(run),source)
        # A W rotation moves the original rectangle into x=5.6..5.8, y=.2...4.
        self.assertEqual(floor,{(1,0,"metal10"):1})

    def test_congestion_strengthening_is_flagged_without_inventing_a_new_formula(self):
        run,common=self.make_run();audit={}
        congestion(run,[],common,audit,{},Options())
        self.assertEqual(audit["congestion_formula"]["normalized_overflow_scale"],.1)
        self.assertFalse(audit["congestion_formula"]["official_20261029_match"])
        self.assertIn("UNCONFIRMED",audit["congestion_formula_status"])
        current=configuration(Options());old=configuration(a150_options())
        self.assertIn("pin_zero_track_floor",current)
        self.assertNotIn("pin_zero_track_floor",old)
        self.assertEqual(current["congestion_formula_status"],"UNCONFIRMED_20261029_CONGESTION_FORMULA")
        self.assertIn("not published",current["congestion_formula_detail"])


if __name__ == "__main__":
    unittest.main()
