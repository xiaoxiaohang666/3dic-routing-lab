"""October 27 resource corrections with explicit, inspectable assumptions.

The untouched 0.4.2 dependency supplies input parsing and Guide legality.
These functions correct PIN layer accounting and project a physical TSV
footprint onto the real track grid. Diagnostic alternatives are explicit;
none depends on a case name or target score.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from collections import defaultdict
from dataclasses import dataclass
import gzip
import json
import math
from pathlib import Path
import time


RULES = "a150-20261029-v2-full-grid"
HISTORICAL_RULES = "inferred-platform-20261027-v3"
OCT29_RULES = "a150-20261029-pin-floor-v1"
NEW_PROFILE = "a150-20261029"


def uses_a150(options):
    return options.rules_profile in ("a150",NEW_PROFILE)


class A150Violation(ValueError):
    """An independently checked A150 hard legality failure."""


def rule_name(options):
    return OCT29_RULES if options.rules_profile == NEW_PROFILE else RULES if uses_a150(options) else HISTORICAL_RULES


@dataclass(frozen=True)
class Options:
    rules_profile: str = NEW_PROFILE
    tsv_obstacle_scope: str = "all-layers"
    tsv_width_um: float = 5.0
    tsv_demand_model: str | None = None
    tsv_pin_exemption: str | None = None
    pin_credit_policy: str = "all-layers"
    zero_capacity_penalty: float = 10000.0
    dump_state: Path | None = None
    capacity_boundary: str = "half-open"
    fixed_track_boundary: str | None = None
    landing_track_boundary: str | None = None

    def __post_init__(self):
        if self.rules_profile not in ("a150", NEW_PROFILE, "inferred-platform"):
            raise ValueError("Unknown rules profile")
        defaults = {"tsv_demand_model": "landing-union" if uses_a150(self) else "physical-tracks",
                    "tsv_pin_exemption": "none" if uses_a150(self) else "own-pin",
                    "fixed_track_boundary": "closed" if uses_a150(self) else "half-open",
                    "landing_track_boundary": "closed" if uses_a150(self) else "open"}
        for key, value in defaults.items():
            if getattr(self, key) is None:
                object.__setattr__(self, key, value)
        if self.tsv_obstacle_scope not in ("all-layers", "interface"):
            raise ValueError("Unknown TSV obstacle scope")
        if uses_a150(self) and (
                self.tsv_demand_model != "landing-union" or self.tsv_pin_exemption != "none" or
                self.capacity_boundary != "half-open" or self.fixed_track_boundary != "closed" or
                self.landing_track_boundary != "closed" or self.tsv_width_um != 5):
            raise ValueError("A150 uses a 5um clipped landing, physical tracks, closed shapes, half-open cells and no TSV PIN credit; use inferred-platform for diagnostics")
        if not math.isfinite(self.tsv_width_um) or self.tsv_width_um <= 0:
            raise ValueError("TSV width must be finite and positive")
        if self.tsv_demand_model not in ("physical-tracks", "cell-track", "ceil", "landing-union"):
            raise ValueError("Unknown TSV demand model")
        if self.tsv_pin_exemption not in ("own-pin", "none"):
            raise ValueError("Unknown TSV PIN exemption policy")
        if self.pin_credit_policy not in ("all-layers", "first-layer"):
            raise ValueError("Unknown PIN credit policy")
        if not math.isfinite(self.zero_capacity_penalty) or self.zero_capacity_penalty <= 0:
            raise ValueError("Zero-capacity penalty must be finite and positive")
        if self.capacity_boundary not in ("half-open", "closed"):
            raise ValueError("Unsupported capacity boundary policy")
        if self.fixed_track_boundary not in ("half-open", "open", "closed"):
            raise ValueError("Unsupported fixed-track boundary policy")
        if self.landing_track_boundary not in ("half-open", "open", "closed"):
            raise ValueError("Unsupported landing-track boundary policy")


def interval_track_count(tracks, lower, upper, boundary):
    if upper <= lower:
        return 0
    left = (bisect_right if boundary == "open" else bisect_left)(tracks, lower)
    right = (bisect_right if boundary == "closed" else bisect_left)(tracks, upper)
    return max(0, right-left)


def set_capacity_boundary(capacity, grid, policy):
    if policy == "half-open":
        return
    for layer, axes in capacity.tracks.items():
        for axis, direction, lines in (("Y", "HORIZONTAL", grid.y_lines),
                                        ("X", "VERTICAL", grid.x_lines)):
            values = axes.get(axis, ())
            capacity.counts[layer, direction] = tuple(
                interval_track_count(values, lo, hi, "closed") for lo, hi in zip(lines, lines[1:]))


def fixed_demand_with_boundary(run, common, capacity, source, boundary, *, a150=False):
    from evaluation.repaired import core, platform
    if boundary == "half-open" and not a150:
        return platform.fixed_track_demand(run, common, capacity, source)
    from types import SimpleNamespace
    view = SimpleNamespace(def_definition=run.def_definition, layer_directions=run.layer_directions,
                           _repair_lef_geometry=source, _repair_io_positions=getattr(run,"_repair_io_positions",{}))
    grid, bits = run.gcell_grid, {}
    for layer, (xl,yl,xh,yh) in core.fixed_rectangles(view, common):
        horizontal = run.layer_directions[layer] == "HORIZONTAL"
        coords = capacity.tracks.get(layer,{}).get("Y" if horizontal else "X",())
        for col in covered_cells(xl,xh,grid.x_lines):
            for row in covered_cells(yl,yh,grid.y_lines):
                lo,hi = (max(yl,grid.y_lines[row]),min(yh,grid.y_lines[row+1])) if horizontal else (
                         max(xl,grid.x_lines[col]),min(xh,grid.x_lines[col+1]))
                begin = (bisect_right if boundary == "open" else bisect_left)(coords,lo)
                end = (bisect_right if boundary == "closed" else bisect_left)(coords,hi)
                if a150:
                    # A shape's own edge is closed; internal GCell upper edges
                    # belong to the next band, exactly as in the C++ model.
                    lines = grid.y_lines if horizontal else grid.x_lines
                    band = row if horizontal else col
                    stop = (bisect_right if band == len(lines)-2 else bisect_left)(coords,lines[band+1])
                    end = min(end,stop)
                base = bisect_left(coords, grid.y_lines[row] if horizontal else grid.x_lines[col])
                key = col,row,layer
                bits[key] = bits.get(key,0) | (((1 << (end-begin))-1) << (begin-base) if end>begin else 0)
    return {key:mask.bit_count() for key,mask in bits.items()}


def fixed_shape_cells(low, high, lines):
    """Full-grid fixed-resource coverage, including degenerate containing cells."""
    if high < low:
        low,high=high,low
    if high < lines[0] or low > lines[-1]:
        return range(0)
    if low == high:
        cell=max(0,min(len(lines)-2,bisect_right(lines,low)-1))
        return range(cell,cell+1)
    return covered_cells(low,high,lines)


def logical_pin_geometry(run, source):
    """Every physical logical macro/IO PIN, regardless of NETS ownership.

    Input-guard source already includes LEF ORIGIN translation. Decimal source
    coordinates are scaled exactly before the eight DEF orientation transforms.
    """
    from fractions import Fraction
    from evaluation.repaired.announcement import transform_box
    dbu=Fraction(str(run.def_definition.GetDatabaseUnits()))

    def exact(value):
        value=Fraction(str(value))*dbu
        return value.numerator if value.denominator == 1 else value

    templates={}
    for index,component in enumerate(run.def_definition.getComponents()):
        name=component.macro_name
        if name not in templates:
            macro=source[name]
            width,height=(exact(value) for value in macro["size"])
            pins={pin:[(layer,tuple(exact(value) for value in box)) for layer,box in shapes]
                  for pin,shapes in macro["pins"].items()}
            templates[name]=width,height,pins
        width,height,pins=templates[name]
        orientation=int(getattr(component.orient,"value",component.orient))
        instance=getattr(component,"instance_name",f"@component{index}")
        for pin,shapes in pins.items():
            yield ("macro",instance,pin),[(layer,transform_box(box,width,height,component.x,component.y,orientation))
                                           for layer,box in shapes if layer in run.layer_directions]
    for name,positions in getattr(run,"_repair_io_positions",{}).items():
        yield ("io",name),[(layer,(box.xl,box.yl,box.xh,box.yh)) for box,layer,_ in positions
                           if layer in run.layer_directions]


def pin_zero_track_floor(run, capacity, source):
    """One virtual unit per logical PIN/layer/cell with empty own track union.

    Multiple RECTs/PORTs of one logical PIN are unioned first. Different PINs
    add separate virtual units, even when their floor cells overlap each other
    or somebody else's actual track. OBS never receives a virtual floor.
    """
    grid=run.gcell_grid;floors=defaultdict(int)
    logical_pins=pin_cells=zero_cells=0
    by_layer=defaultdict(int)
    for identity,shapes in logical_pin_geometry(run,source):
        logical_pins += 1
        own={}
        for layer,(xl,yl,xh,yh) in shapes:
            horizontal=run.layer_directions[layer] == "HORIZONTAL"
            lines=grid.y_lines if horizontal else grid.x_lines
            tracks=capacity.tracks.get(layer,{}).get("Y" if horizontal else "X",())
            for col in fixed_shape_cells(xl,xh,grid.x_lines):
                for row in fixed_shape_cells(yl,yh,grid.y_lines):
                    key=col,row,layer
                    band=row if horizontal else col
                    low,high=(max(yl,lines[band]),min(yh,lines[band+1])) if horizontal else (
                              max(xl,lines[band]),min(xh,lines[band+1]))
                    begin=bisect_left(tracks,low)
                    end=min(bisect_right(tracks,high),
                            (bisect_right if band == len(lines)-2 else bisect_left)(tracks,lines[band+1]))
                    # Only emptiness matters after union, so OR presence avoids
                    # materializing per-PIN track sets without losing identity.
                    own[key]=own.get(key,False) or end > begin
        pin_cells += len(own)
        for key,occupied in own.items():
            if not occupied:
                floors[key] += 1;zero_cells += 1;by_layer[key[2]] += 1
    return dict(floors),{"enabled":True,"logical_pin_instances":logical_pins,
        "logical_pin_layer_cell_entries":pin_cells,"zero_track_pin_layer_cell_entries":zero_cells,
        "virtual_floor_demand":sum(floors.values()),"floor_demand_by_layer":dict(by_layer),
        "model":"per-physical-pin-layer-cell-empty-track-union-additive",
        "overlap_assumption":"distinct logical PIN virtual floors add; never unioned with another PIN/OBS real tracks",
        "real_track_union_unchanged":True,"OBS_zero_track_floor":False,
        "ordinary_own_pin_credit":"removes at most ordinary Guide=1; global fixed floors remain"}


def landing_tsv_demands(point, grid, capacity, directions, dbu, width_um, boundary, *, a150=False):
    """Guide-level landing-cell reservation; no detailed TSV placement claim."""
    col,row = point.gcell_col, point.gcell_row
    radius = width_um*dbu/2
    for layer in (point.bottom_layer, point.top_layer):
        horizontal = directions[layer] == "HORIZONTAL"
        lines = grid.y_lines if horizontal else grid.x_lines
        band = row if horizontal else col
        center = point.y_dbu if horizontal else point.x_dbu
        lo,hi = max(center-radius,lines[band]),min(center+radius,lines[band+1])
        tracks = capacity.tracks.get(layer,{}).get("Y" if horizontal else "X",())
        if a150:
            die_lo,die_hi = (grid.die.y1,grid.die.y2) if horizontal else (grid.die.x1,grid.die.x2)
            lo,hi = max(lo,die_lo),min(hi,die_hi)
            begin = bisect_left(tracks,lo)
            end = (bisect_left if hi == lines[band+1] and band+2 < len(lines) else bisect_right)(tracks,hi)
            amount = max(0,end-begin) if hi >= lo else 0
            yield (col,row,layer), amount
        else:
            yield (col,row,layer), interval_track_count(tracks,lo,hi,boundary)


def covered_cells(low, high, lines):
    """Positive-area intersections, without grid snapping or endpoint caps."""
    if high <= low:
        return range(0)
    return range(max(0, bisect_right(lines, low) - 1),
                 min(len(lines) - 1, bisect_left(lines, high)))


def physical_track_count(tracks, low, high, cell_low, cell_high, *, final=False):
    """Half-open physical footprint, half-open cells, final cell boundary closed."""
    begin = max(bisect_left(tracks, low), bisect_left(tracks, cell_low))
    end = min(bisect_left(tracks, high),
              (bisect_right if final else bisect_left)(tracks, cell_high))
    return max(0, end - begin)


def tsv_footprint_demands(point, grid, capacity, directions, dbu_per_micron,
                          *, width_um=5.0, model="physical-tracks"):
    """Yield (cell-layer, demand) for both layers of one centered TSV.

    physical-tracks: exact track identities intersecting the 5x5 um square.
    cell-track: cell capacity times perpendicular covered-width fraction.
    ceil: ceiling of cell-track, per intersected cell-layer.
    Demand from separate TSV instances is additive, including overlap.
    """
    if model not in ("physical-tracks", "cell-track", "ceil"):
        raise ValueError("Unknown TSV demand model")
    radius = width_um * dbu_per_micron / 2
    x0, x1 = point.x_dbu - radius, point.x_dbu + radius
    y0, y1 = point.y_dbu - radius, point.y_dbu + radius
    for layer in (point.bottom_layer, point.top_layer):
        direction = directions[layer]
        if direction not in ("HORIZONTAL", "VERTICAL"):
            raise ValueError("Unsupported TSV layer direction")
        horizontal = direction == "HORIZONTAL"
        lines = grid.y_lines if horizontal else grid.x_lines
        tracks = capacity.tracks.get(layer, {}).get("Y" if horizontal else "X", ())
        low, high = (y0, y1) if horizontal else (x0, x1)
        for row in covered_cells(y0, y1, grid.y_lines):
            for col in covered_cells(x0, x1, grid.x_lines):
                band = row if horizontal else col
                if model == "physical-tracks":
                    amount = physical_track_count(tracks, low, high, lines[band], lines[band+1],
                                                  final=band == len(lines)-2)
                else:
                    span = lines[band+1] - lines[band]
                    fraction = max(0, min(high, lines[band+1]) - max(low, lines[band])) / span
                    amount = capacity.get((col, row, layer, direction), 0) * fraction
                    if model == "ceil":
                        amount = math.ceil(amount)
                yield (col, row, layer), amount


def first_layer_credit_source(source):
    """Keep original-layer shapes on each PIN's first declared geometry layer."""
    result = {}
    for name, macro in source.items():
        pins = {}
        for pin, shapes in macro["pins"].items():
            first = shapes[0][0] if shapes else None
            pins[pin] = [(layer, box) for layer, box in shapes if layer == first]
        result[name] = dict(macro, pins=pins)
    return result


def _rows(mapping):
    return [[*key, value] for key, value in sorted(mapping.items())]


def a150_mask_cells(box, grid):
    """C++ geometric mask: DIE clip, positive interiors, degenerate ownership."""
    xl,yl,xh,yh=box
    x0,x1=max(xl,grid.die.x1),min(xh,grid.die.x2)
    y0,y1=max(yl,grid.die.y1),min(yh,grid.die.y2)
    if (x1 < x0 or y1 < y0 or (xl < xh and x0 == x1) or (yl < yh and y0 == y1)):
        return

    def axis(low, high, lines):
        if high < lines[0] or low > lines[-1]:
            return range(0)
        if low == high:
            band=max(0,min(len(lines)-2,bisect_right(lines,low)-1))
            return range(band,band+1)
        return covered_cells(low,high,lines)

    for row in axis(y0,y1,grid.y_lines):
        for col in axis(x0,x1,grid.x_lines):
            yield col,row


def check_a150_landings(run, points, common, source, options, audit):
    """Check geometry presence independently of resource tracks or PIN credit."""
    from evaluation.repaired import core
    from types import SimpleNamespace
    requested = {(point.gcell_col,point.gcell_row) for point in points}
    blocked = defaultdict(set)
    view = SimpleNamespace(def_definition=run.def_definition, layer_directions=run.layer_directions,
                           _repair_lef_geometry=source,
                           _repair_io_positions=getattr(run,"_repair_io_positions",{}))
    grid = run.gcell_grid
    for layer,(xl,yl,xh,yh) in core.fixed_rectangles(view,common):
        if options.tsv_obstacle_scope == "interface" and layer not in ("metal10","metal11"):
            continue
        for col,row in a150_mask_cells((xl,yl,xh,yh),grid):
            if (col,row) in requested:
                blocked[col,row].add(layer)
    owners,violations = {},[]
    for point in points:
        cell = point.gcell_col,point.gcell_row
        layers = sorted(blocked.get(cell,()))
        if layers:
            violations.append({"net":point.net,"col":cell[0],"row":cell[1],
                               "kind":"PIN_OR_OBS_GCELL","layers":layers})
        other = owners.setdefault(cell,point.net)
        if other != point.net:
            violations.append({"net":point.net,"other_net":other,"col":cell[0],"row":cell[1],
                               "kind":"CROSS_NET_TSV_GCELL",
                               "basis":"conservative-exclusive-solver-policy; A150 only specifies additive demand"})
    audit["a150_legality"] = {"scope":options.tsv_obstacle_scope,"geometry_based":True,
        "track_independent":True,"boundary_contact":"DIE-clipped-positive-interior;degenerate-half-open-final-closed-assumption",
        "checked_tsvs":len(points),"violations":len(violations),
        "pin_obs_rule_violations":sum(item["kind"] == "PIN_OR_OBS_GCELL" for item in violations),
        "conservative_exclusive_policy_violations":sum(item["kind"] == "CROSS_NET_TSV_GCELL" for item in violations),
        "cross_net_same_cell_basis":"conservative solver policy, not an A150 hard prohibition",
        "examples":violations[:50]}
    if violations:
        raise A150Violation(f"TSV placement violates the selected A150/solver policy: {len(violations)} PIN/OBS or conservative exclusivity violations")


def congestion(run, points, common, audit, source, options):
    from evaluation.repaired import core, platform

    started = time.perf_counter()
    if uses_a150(options):
        if run.tsv_congestion_ratio != 1:
            raise ValueError("A150 TSV demand is a direct track count and requires ratio one")
        check_a150_landings(run,points,common,source,options,audit)
    capacity = core.Capacity(run.def_data.tracks, run.gcell_grid)
    set_capacity_boundary(capacity, run.gcell_grid, options.capacity_boundary)
    # Original geometry: each PIN rectangle keeps its own real layer.
    fixed = fixed_demand_with_boundary(run, common, capacity, source, options.fixed_track_boundary,
                                       a150=uses_a150(options))
    floor={}
    if options.rules_profile == NEW_PROFILE:
        floor,floor_audit=pin_zero_track_floor(run,capacity,source)
        audit["pin_zero_track_floor"]=floor_audit
        fixed=dict(fixed)
        for key,value in floor.items():
            fixed[key]=fixed.get(key,0)+value
        audit["congestion_formula_status"]="UNCONFIRMED_20261029_CONGESTION_FORMULA"
        audit["congestion_formula"]={"used":"sum((0.1*overflow/capacity)^2);C=100/sqrt(1+Q)",
            "normalized_overflow_scale":0.1,"official_20261029_match":False,
            "reason":"1029 update says stronger congestion penalty but supplies no replacement formula; coefficient not guessed"}
    demand = defaultdict(float, fixed)
    fixed_layers = defaultdict(int)
    for (_, _, layer), value in fixed.items():
        fixed_layers[layer] += value
    masks = getattr(run, "_scorer_20261027_pin_masks", {})
    pin_cells = run._repair_pin_cells
    route = defaultdict(int)
    used_by_net = {}
    landing_keys = defaultdict(set)
    if uses_a150(options):
        for point in points:
            landing_keys[point.net].update((point.gcell_col,point.gcell_row,layer)
                                           for layer in (point.bottom_layer,point.top_layer))
    route_unexempted = defaultdict(int)
    exemptions = {policy: defaultdict(int) for policy in masks}
    exempt_total = 0
    grid = run.gcell_grid
    for name, rectangles in run.net_guides.items():
        used = set()
        for rect in rectangles:
            cols = common.rectangle_cell_indices(max(rect.xl, grid.die.x1),
                                                min(rect.xh, grid.die.x2), grid.x_lines)
            rows = common.rectangle_cell_indices(max(rect.yl, grid.die.y1),
                                                min(rect.yh, grid.die.y2), grid.y_lines)
            used.update((col, row, rect.layer) for col in cols for row in rows)
        if options.dump_state:
            for key in used:
                route_unexempted[key] += 1
            for policy, owned in masks.items():
                for key in used & owned.get(name, set()):
                    exemptions[policy][key] += 1
        if name in landing_keys:
            used_by_net[name] = used & landing_keys[name]
        exempt = pin_cells.get(name, set())
        exempt_total += len(used & exempt)
        for key in used - exempt:
            route[key] += 1
            demand[key] += 1
    tsv_total = tsv_exempt = 0.0
    landing_reserved = landing_guide_credit = 0.0
    tsv = defaultdict(float)
    dbu = run.def_definition.GetDatabaseUnits()
    replaced = set()
    for point in points:
        owned = pin_cells.get(point.net, set())
        if uses_a150(options):
            for key,occupied in landing_tsv_demands(point,grid,capacity,run.layer_directions,dbu,
                    options.tsv_width_um,options.landing_track_boundary,a150=True):
                if key not in used_by_net.get(point.net,set()):
                    raise A150Violation("A150 TSV landing lacks its own Guide coverage")
                # Replace this net's ordinary Guide contribution, even when
                # the exact covered-track count is zero. Fixed demand stays.
                owner_key = point.net,key
                credit = int(key not in owned and owner_key not in replaced)
                replaced.add(owner_key)
                route[key] -= credit
                demand[key] += occupied-credit
                landing_guide_credit += credit
                landing_reserved += occupied
                tsv[key] += occupied
                tsv_total += occupied
            continue
        if options.tsv_demand_model == "landing-union":
            for key, occupied in landing_tsv_demands(point,grid,capacity,run.layer_directions,dbu,
                                                     options.tsv_width_um,options.landing_track_boundary):
                # Merge the existing own-net cell contribution with the TSV.
                # PIN-covered cells already hold a fixed-track union. Other
                # cells already contain this net's one Guide demand.
                credit = (fixed.get(key,0) if options.tsv_pin_exemption=="own-pin" else 0) if key in owned else 1
                reserved = occupied*run.tsv_congestion_ratio
                amount = max(0., reserved-credit)
                landing_reserved += reserved
                if key in owned:
                    tsv_exempt += min(reserved,credit)
                else:
                    landing_guide_credit += min(reserved,credit)
                demand[key] += amount
                tsv[key] += amount
                tsv_total += amount
            continue
        for key, amount in tsv_footprint_demands(point, grid, capacity, run.layer_directions, dbu,
                                                width_um=options.tsv_width_um,
                                                model=options.tsv_demand_model):
            amount *= run.tsv_congestion_ratio
            if options.tsv_pin_exemption == "own-pin" and key in owned:
                tsv_exempt += amount
                continue
            demand[key] += amount
            tsv[key] += amount
            tsv_total += amount
    if uses_a150(options):
        # Zero markers have already replaced ordinary Guide coverage. Keep
        # the exported ledger sparse, matching the C++ aggregate contract.
        demand = defaultdict(float,{key:value for key,value in demand.items() if value > 0})
        route = defaultdict(int,{key:value for key,value in route.items() if value > 0})
        tsv = defaultdict(float,{key:value for key,value in tsv.items() if value > 0})
    if options.dump_state:
        state = {
            "schema": "eda-congestion-state-20261027-v1", "official_equivalence": False,
            "grid": {"x_lines": list(grid.x_lines), "y_lines": list(grid.y_lines),
                     "die": [grid.die.x1, grid.die.y1, grid.die.x2, grid.die.y2],
                     "dbu_per_micron": dbu},
            "directions": run.layer_directions, "tracks": capacity.tracks,
            "fixed_demand": _rows(fixed), "pin_zero_track_floor":_rows(floor), "route_demand": _rows(route),
            "route_unexempted": _rows(route_unexempted),
            "route_pin_exemptions": {p: _rows(cells) for p, cells in exemptions.items()},
            "pin_masks": {p: {net: [list(k) for k in sorted(cells)] for net, cells in owned.items()}
                          for p, owned in masks.items()},
            "tsv_points": [{"net": p.net, "x_dbu": p.x_dbu, "y_dbu": p.y_dbu,
                            "gcell_col": p.gcell_col, "gcell_row": p.gcell_row,
                            "bottom_layer": p.bottom_layer, "top_layer": p.top_layer} for p in points],
            "tsv_congestion_ratio": run.tsv_congestion_ratio,
            "configuration": configuration(options),
            "fixed_demand_variants": {policy:_rows(fixed if policy==options.fixed_track_boundary else
                                        fixed_demand_with_boundary(run,common,capacity,source,policy,
                                            a150=uses_a150(options) and policy == "closed"))
                                       for policy in ("half-open","open","closed")},
        }
        with gzip.open(options.dump_state, "xt", encoding="utf-8") as target:
            json.dump(state, target, separators=(",", ":"), allow_nan=False)
    overflow, penalties = [], []
    zero_capacity = 0
    for key in sorted(demand):
        amount = demand[key]
        direction = run.layer_directions[key[2]]
        cap = capacity.get((*key, direction), 0)
        over = max(0.0, amount - cap)
        if over <= 0:
            continue
        if cap == 0:
            penalty = options.zero_capacity_penalty
            zero_capacity += 1
        else:
            penalty = (0.1 * over / cap) ** 2
        penalties.append(penalty)
        overflow.append(((*key, direction), amount, cap, over))
    total = math.fsum(item[3] for item in overflow)
    audit["congestion"] = {
        "gcell_width_dbu": run.gcell_width, "gcell_height_dbu": run.gcell_height,
        "fixed_track_demand": sum(fixed.values()), "fixed_track_demand_by_layer": dict(fixed_layers),
        "guide_net_cell_demand": sum(route.values()), "own_pin_guide_exempt_demand": exempt_total,
        "tsv_demand": tsv_total, "own_pin_tsv_exempt_demand": tsv_exempt,
        "landing_reserved_track_demand": landing_reserved,
        "existing_own_guide_landing_credit": landing_guide_credit,
        "tsv_demand_entries": len(tsv), "total_demand": math.fsum(demand.values()),
        "penalty_sum": math.fsum(penalties), "zero_capacity_demand_cells": zero_capacity,
        "all_occupied_cells_checked": len(demand), "capacity_entries_represented": len(capacity),
        "seconds": time.perf_counter() - started,
    }
    return (max((item[3] for item in overflow), default=0),
            total / len(demand) if demand else 0.0, total, overflow,
            dict(demand), math.fsum(penalties), len(overflow))


def configuration(options):
    if options.rules_profile == NEW_PROFILE:
        previous=configuration(Options(rules_profile="a150",tsv_obstacle_scope=options.tsv_obstacle_scope,
            tsv_width_um=options.tsv_width_um,tsv_demand_model=options.tsv_demand_model,
            tsv_pin_exemption=options.tsv_pin_exemption,pin_credit_policy=options.pin_credit_policy,
            zero_capacity_penalty=options.zero_capacity_penalty,capacity_boundary=options.capacity_boundary,
            fixed_track_boundary=options.fixed_track_boundary,landing_track_boundary=options.landing_track_boundary))
        previous.update(rules_profile=OCT29_RULES,
            basis="official 1029 update PIN floor plus A134/A146/A149/A150; congestion formula remains unpublished",
            pin_zero_track_floor="per-physical-pin-layer-cell-empty-track-union-additive",
            pin_floor_overlap_assumption="distinct logical PIN floors additive; real PIN/OBS tracks globally unioned",
            congestion_formula_status="UNCONFIRMED_20261029_CONGESTION_FORMULA",
            congestion_formula="legacy 0.1 normalized overflow scale retained without claiming official 1029 match",
            congestion_formula_detail="1029 strengthened penalty formula not published; no replacement coefficient guessed")
        previous["unconfirmed"]=[*previous["unconfirmed"],
            "1029 stronger congestion penalty formula not published; scalar score not official 1029 matched",
            "logical-PIN/layer/GCell floor aggregation and virtual overlap policy are explicit assumptions"]
        return previous
    if uses_a150(options):
        return {"rules_profile":RULES,"basis":"official A134/A146/A149/A150 text; independent implementation",
            "fixed_PIN_layers":"each geometry's original layer","fixed_track_interval":"closed-shape-half-open-cell",
            "capacity_interval":"half-open-internal-final-outer-closed-assumption",
            "landing_track_interval":"closed-shape-half-open-cell","tsv_width_um":5,"tsv_height_um":5,
            "tsv_demand_model":"landing-cell-direct-tracks","tsv_instances":"additive",
            "resource_grid":"C++-OFFICIAL_FULL;declared-or-extended-lines-preserved",
            "Guide_geometry":"DIE-clipped-parent-GCell;source-outside-DIE-rejected",
            "tsv_layers":["metal10","metal11"],"tsv_pin_exemption":"none",
            "landing_credit":"replace this net's Guide with exact TSV tracks, including zero",
            "tsv_obstacle_scope":options.tsv_obstacle_scope,
            "cross_net_same_cell_policy":"conservative-exclusive-solver-policy; A150 demand remains additive",
            "pin_credit_policy":options.pin_credit_policy,"zero_capacity_penalty":options.zero_capacity_penalty,
            "unconfirmed":["all-layer obstacle scope is a conservative assumption; interface is explicit alternative",
                "positive-area PIN/OBS cell intersection; shape own edge closed; final outer grid edge closed",
                "ordinary Guide own-PIN credit retained separately pending A148",
                "FLUTE, via, thermal and final ranking/S4 equivalence"]}
    return {"rules_profile":HISTORICAL_RULES,"basis": "uniform resource model calibrated against public Guide logs; not official source",
            "fixed_PIN_layers": "each geometry's original layer",
            "fixed_track_interval": options.fixed_track_boundary,
            "capacity_interval": options.capacity_boundary,
            "landing_track_interval": options.landing_track_boundary,
            "tsv_width_um": options.tsv_width_um, "tsv_height_um": options.tsv_width_um,
            "tsv_demand_model": options.tsv_demand_model, "tsv_instances": "additive",
            "tsv_layers": ["metal10", "metal11"],
            "tsv_pin_exemption": options.tsv_pin_exemption,
            "landing_credit": "existing fixed demand for own-PIN cells, otherwise one own Guide contribution",
            "pin_credit_policy": options.pin_credit_policy,
            "zero_capacity_penalty": options.zero_capacity_penalty,
            "unconfirmed": ["exact updated TSV accounting, overlap union and PIN exemptions",
                            "archived FLUTE, via and thermal implementation details",
                            "zero-capacity penalty and final ranking/S4"]}


def full_grid_boundaries(lines, die_lower, die_upper):
    """C++ OFFICIAL_FULL envelope: retain declared lines, extend only if short.

    DIE edges inside the declared grid are geometry clipping boundaries, not
    extra resource grid lines. Keep outside bands in every resource/heat view.
    """
    if not math.isfinite(die_lower) or not math.isfinite(die_upper) or die_lower >= die_upper:
        raise ValueError("DIEAREA must have finite positive extent")
    values = sorted(set(lines))
    if len(values) < 2 or any(not math.isfinite(value) for value in values):
        raise ValueError("A150 requires at least two finite GCELLGRID lines")
    if values[0] > die_lower:
        values.insert(0,die_lower)
    if values[-1] < die_upper:
        values.append(die_upper)
    return tuple(values)


def install_full_grid_boundaries(score, audit):
    original = score.expand_grid_lines

    def expand(statements, die_lower, die_upper, axis):
        declared, warnings = original(statements,die_lower,die_upper,axis)
        full = full_grid_boundaries(declared,die_lower,die_upper)
        audit.setdefault("effective_gcell_grid",{})[axis] = {
            "basis":"C++-OFFICIAL_FULL-declared-or-extended-grid",
            "declared_or_extended_boundaries":len(declared),"effective_boundaries":len(full),
            "die_lower":die_lower,"die_upper":die_upper,"first":full[0],"last":full[-1],
            "changed":tuple(declared) != full,"outside_DIE_boundaries_preserved":True,
            "DIE_edges_inserted_only_when_envelope_short":True}
        return full,warnings

    score.expand_grid_lines = expand


def check_a150_guide_geometry(raw_guides, grid, directions, checker, audit):
    """Check source geometry against full-grid cells with DIE-clipped edges.

    Do not clip/drop malformed source rectangles. Transverse coverage must be
    exactly one parent GCell after DIE clipping, as in validate_gcell_encoding.
    """
    xs,ys = set(grid.x_lines),set(grid.y_lines)
    xs.update((grid.die.x1,grid.die.x2));ys.update((grid.die.y1,grid.die.y2))
    violations=[];checked=0
    for name,rectangles in raw_guides.items():
        for xl,yl,xh,yh,layer in rectangles:
            checked += 1
            reason = None
            if (not all(math.isfinite(value) for value in (xl,yl,xh,yh)) or xl >= xh or yl >= yh):
                reason = "non-finite coordinates or non-positive area"
            elif xl < grid.die.x1 or yl < grid.die.y1 or xh > grid.die.x2 or yh > grid.die.y2:
                reason = "rectangle extends outside DIEAREA"
            elif xl not in xs or xh not in xs or yl not in ys or yh not in ys:
                reason = "edge is neither a declared GCell line nor a DIE clipping edge"
            elif layer not in directions or directions[layer] not in ("HORIZONTAL","VERTICAL"):
                reason = "unknown or non-routing layer direction"
            else:
                horizontal = directions[layer] == "HORIZONTAL"
                low,high = (yl,yh) if horizontal else (xl,xh)
                lines = grid.y_lines if horizontal else grid.x_lines
                band = bisect_right(lines,(low+high)/2)-1
                if not 0 <= band < len(lines)-1:
                    reason = "no transverse parent GCell"
                else:
                    die_low,die_high = ((grid.die.y1,grid.die.y2) if horizontal else
                                        (grid.die.x1,grid.die.x2))
                    expected = max(lines[band],die_low),min(lines[band+1],die_high)
                    if (low,high) != expected:
                        reason = "transverse interval must equal one DIE-clipped full-grid GCell"
            if reason is not None:
                violations.append({"net":name,"layer":layer,"bounds":[xl,yl,xh,yh],"reason":reason})
    audit["a150_guide_geometry"] = {"grid":"C++-OFFICIAL_FULL","checked_rectangles":checked,
        "violations":len(violations),"examples":violations[:10],
        "source_geometry_clipped_or_discarded":False,
        "boundary_rule":"declared GCell lines or DIE clipping edges; exact transverse parent cell"}
    if violations:
        raise checker.EvaluationError("A150 Guide geometry: " + str(violations[0]),
            category=checker.LEGALITY,rule="GCell-aligned Guide with DIE-clipped transverse parent cell",
            hint="Correct source geometry; preserve declared resource grid and clip only Guide geometry to DIEAREA.")


def install_a150_geometry(modules, audit, source, pin_builder):
    from evaluation.repaired import core
    from evaluation.repaired.legality import check_submitted_names, extract_tsv_points
    common,checker,_,_,score = modules
    core.install(modules,audit,source,"announcement")
    # Unlike install_announcement, never replace an overshooting resource grid
    # by DIE bounds. All downstream resource and thermal stages see this grid.
    install_full_grid_boundaries(score,audit)
    score.extract_tsv_points = lambda run,grid: extract_tsv_points(
        run,grid,common,checker,score.TSVPoint,audit)
    raw_guides={}
    original_parse=common.parse_guide

    def parse(path):
        result=original_parse(path)
        raw_guides.update({name:tuple((r.xl,r.yl,r.xh,r.yh,r.layer) for r in rectangles)
                           for name,rectangles in result.items()})
        return result

    common.parse_guide=parse
    original_check=checker.check

    def full_check(run):
        check_submitted_names(raw_guides,run,checker,audit)
        check_a150_guide_geometry(raw_guides,run.gcell_grid,run.layer_directions,checker,audit)
        original_check(run)
        pin_builder(run,source,common,audit)
        audit["preliminary_legality_completed"]=True
        audit["resource_grid_shape"] = {"cols":run.gcell_grid.cols,"rows":run.gcell_grid.rows,
            "resource_cells":run.gcell_grid.cols*run.gcell_grid.rows,
            "thermal_vector_cells":run.gcell_grid.cols*run.gcell_grid.rows,
            "shared_run_gcell_grid":True}

    checker.check=full_check;score.check=full_check
    original_score=score.compute_score

    def compute(length,overflow,correlation,gain):
        value=float(length)
        if not math.isfinite(value) or value < 0:
            raise ValueError("Invalid normalized wire length")
        result=original_score(length,overflow,correlation,gain)
        quality=min(1.,1./max(value,1e-9))
        result["wirelength_quality"]=quality;result["wirelength_score"]=100.*quality
        return result

    score.compute_score=compute
    audit["legality_rules"]="A150-full-grid-source-guide-with-DIE-clipping-v1"


def install(modules, audit, source, options):
    from evaluation.repaired.announcement import build_pin_exemptions, install_announcement

    def pins(run, original, common, record):
        records = {}
        masks = {}
        for policy, geometry in (("all-layers", original),
                                 ("first-layer", first_layer_credit_source(original))):
            detail = {}
            build_pin_exemptions(run, geometry, common, detail)
            masks[policy] = run._repair_pin_cells
            records[policy] = detail["pin_exemption"]
        run._scorer_20261027_pin_masks = masks
        run._repair_pin_cells = masks[options.pin_credit_policy]
        record["pin_exemption"] = dict(records[options.pin_credit_policy],
                                        policy=options.pin_credit_policy)
        record["pin_exemption_diagnostics"] = records

    if uses_a150(options):
        install_a150_geometry(modules,audit,source,pins)
    else:
        install_announcement(modules, audit, source, "announcement", pin_builder=pins)
    common, _, _, _, score = modules
    score.compute_congestion = lambda run, points: congestion(run, points, common, audit, source, options)
    audit["rules"] = rule_name(options)
    audit["compatibility_20261027"] = configuration(options)
    audit["official_equivalence"] = False
