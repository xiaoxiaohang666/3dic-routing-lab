#!/usr/bin/env python3
"""October 29 A150 geometric/resource baseline verification on benchmark-host.

The enhanced congestion formula is unpublished. Scores retain the existing
explicit proxy and cannot unlock new subtree optimization or official alignment.

All three supply audits finish before any timed route starts. All routing
finishes before engineering, geometry and serialized-resource readback starts.
No compatibility scorer, platform submission or new subtree operator is used.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import traceback

import run_tsv_resource_ablation as base

CASES = ('001', '003', '005')
SCOPE_NAMES = {'all-layers': 'all-routing-layers-both-dies', 'interface': 'interface-m10-m11'}
base.VARIANTS = {'BASE': (1, 1, 0)}
A150_CONTRACT = 'a150-20261029-pin-floor-tsv-tracks'
RULES_PROFILE = 'a150-20261029'
SCORE_ALIGNMENT = 'UNCONFIRMED_20261029_CONGESTION_FORMULA'
RULE_ASSUMPTIONS = {
    'tsv_footprint': '5um-square-at-GCell-center-clipped-to-landing-cell',
    'gcell_track_boundary': 'lower-inclusive-upper-exclusive;last-exterior-upper-inclusive',
    'shape_track_boundary': 'closed-shape-edges;internal-gcell-upper-exclusive',
    'geometry_boundary': 'positive-area-cell-interior;degenerate-half-open-last-closed',
    'ordinary_guide_own_pin_credit': 'same-owner-actual-layer-cell-Guide-unit-exemption;fixed-demand-retained',
    'pin_zero_track_floor': 'one-per-logical-PIN-actual-layer-covered-GCell-only-when-own-track-union-empty;granularity-assumption',
    'zero_track_OBS': 'no-floor',
    'zero_track_TSV': 'actual-zero-no-extra-unit',
    'congestion_formula': 'existing-Q=sum((0.1*overflow/capacity)^2)-is-an-explicit-proxy;20261029-enhanced-penalty-unpublished',
    'thermal_gain': 'uniform-G20-proxy;not-official-G_case',
    'cross_net_tsv_ownership': 'one-per-cell-conservative-policy;A150-only-specifies-additive-demand',
    'unresolved': [
        '20261029 enhanced congestion penalty has no published numeric formula in the supplied update',
        'zero-track PIN floor granularity and overlapping logical-PIN floor combination remain assumptions',
        'PIN/OBS prohibition layer domain and PIN-type coverage are not explicitly bounded by A150',
        'physical-shape boundary coverage is distinct from GCell track ownership',
        'duplicate TRACKS and TSV rectangle-edge track ownership are not fully specified',
        'aligned fixed M10/M11 source PINs may conflict with unconditional PIN prohibition',
    ],
}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument('--job-root', type=Path, default=base.SOURCE.parent)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--budget', type=float, default=1800)
    p.add_argument('--budget-reserve', type=float, default=60)
    p.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    p.add_argument('--workers', type=int, choices=(1, 2, 3), default=3)
    p.add_argument('--evaluation-workers', type=int, choices=(1, 2, 3), default=3)
    p.add_argument('--tsv-obstacle-scope', choices=('all-layers', 'interface'), default='all-layers')
    p.add_argument('--supply-only', action='store_true')
    p.add_argument('--case-root', type=Path, default=base.CACHE / 'full-suite-20261018-04ab4298/cases')
    p.add_argument('--runtime', type=Path, default=base.CACHE / 'community040-fixed-20261024-341df8d5/submission/runtime/official-flute')
    # base.Experiment records this existing dependency path, but never executes
    # its scorer in this harness.
    p.add_argument('--dependency-root', type=Path, default=base.CACHE / 'community042-audit-dff686f959/package')
    a = p.parse_args(argv)
    if not math.isfinite(a.budget) or not 60 <= a.budget <= 43200:
        p.error('budget must be finite and 60..43200 seconds')
    if not math.isfinite(a.budget_reserve) or not 0 <= a.budget_reserve < a.budget - 30:
        p.error('budget reserve must leave at least 30 internal seconds')
    if len(set(a.cases)) != len(a.cases):
        p.error('duplicate cases')
    a.variants = ['BASE']
    return a


def integer(mapping, key):
    value = mapping.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError('missing or invalid nonnegative integer: ' + key)
    return value


def normalized_scope(value):
    for short, name in SCOPE_NAMES.items():
        if value in (short, name):
            return short
    raise ValueError('unknown recorded TSV obstacle scope: ' + repr(value))


def recorded_resource_configuration(report):
    floor = report.get('pin_zero_track_floor', {})
    if not isinstance(floor, dict) or floor.get('enabled') is not True:
        raise ValueError('October 29 PIN zero-track floor is not enabled')
    integer(floor, 'demand')
    if floor.get('granularity_assumption') != 'per-logical-PIN-layer-GCell':
        raise ValueError('October 29 PIN floor granularity assumption differs')
    if report.get('congestion_formula_status') != SCORE_ALIGNMENT:
        raise ValueError('unpublished October 29 congestion formula must remain explicitly unconfirmed')
    return floor


def recorded_profile(report, scope, *, serialized=False):
    recorded_resource_configuration(report)
    if report.get('model' if serialized else 'rules_contract') != A150_CONTRACT:
        raise ValueError('recorded result does not use the A150 resource contract')
    data = report.get('a150', {})
    if data.get('enabled') is not True:
        raise ValueError('recorded A150 profile is not enabled')
    if normalized_scope(data.get('obstacle_scope')) != scope:
        raise ValueError('recorded result has different TSV obstacle scope')
    if serialized:
        interface = integer(data, 'interface_violations')
        all_layers = integer(data, 'all_layer_violations')
        if interface > all_layers or (all_layers if scope == 'all-layers' else interface):
            raise ValueError('serialized readback has forbidden TSV landings')
    return data


def supply_contract(report, scope):
    if report.get('rules_profile') != RULES_PROFILE or report.get('supply_only') is not True:
        raise ValueError('supply report does not use the A150 preflight profile')
    floor = recorded_resource_configuration(report)
    data = report.get('a150', {})
    if data.get('scope') != SCOPE_NAMES[scope]:
        raise ValueError('supply report has different obstacle scope')
    cross = integer(data, 'cross_die_nets')
    interface = integer(data, 'interface_eligible_cells')
    all_layers = integer(data, 'all_layers_eligible_cells')
    selected = integer(data, 'selected_eligible_cells')
    if all_layers > interface or selected != (all_layers if scope == 'all-layers' else interface):
        raise ValueError('inconsistent supply counts')
    enough = selected >= cross
    if data.get('selected_scope_supply_sufficient') is not enough:
        raise ValueError('supply sufficient flag differs from counts')
    fixed_interface = integer(data, 'fixed_interface_violations')
    fixed_all = integer(data, 'fixed_all_layers_violations')
    if fixed_interface > fixed_all:
        raise ValueError('fixed interface conflicts exceed all-layer conflicts')
    fixed = fixed_all if scope == 'all-layers' else fixed_interface
    expected_status = 'PASS' if enough and not fixed else 'FAIL'
    if report.get('status') != expected_status:
        raise ValueError('supply status differs from supply/fixed-conflict findings')
    return dict(data, selected_scope_supply_sufficient=enough, selected_scope_fixed_conflicts=fixed,
                pin_zero_track_floor_demand=floor['demand'])


def legal_contract(report, scope, supply):
    if report.get('rules_profile') != RULES_PROFILE:
        raise ValueError('engineering report does not use the October 29 profile')
    if report.get('status') != 'PASS':
        raise ValueError('engineering validator did not pass')
    floor = recorded_resource_configuration(report)
    data = report.get('a150', {})
    if data.get('scope') != SCOPE_NAMES[scope]:
        raise ValueError('engineering report has different obstacle scope')
    for key in ('interface_eligible_cells', 'all_layers_eligible_cells', 'selected_eligible_cells'):
        if integer(data, key) != supply[key]:
            raise ValueError('engineering supply differs from preflight: ' + key)
    if floor['demand'] != supply['pin_zero_track_floor_demand']:
        raise ValueError('engineering PIN floor demand differs from preflight')
    interface = integer(data, 'interface_violations')
    all_layers = integer(data, 'all_layers_violations')
    if interface > all_layers:
        raise ValueError('interface violations exceed all-layer violations')
    selected = all_layers if scope == 'all-layers' else interface
    if selected:
        raise ValueError('TSV intersects forbidden PIN/OBS cells in selected scope')
    return dict(data, selected_scope_violations=selected)


def qualifies_for_subtree(rows, strict_baseline):
    # The supplied update raises congestion penalties without publishing a
    # formula. Legal Guides do not resolve candidate selection under that score.
    # Preserve the check so malformed/violating rows cannot masquerade as ready.
    if strict_baseline:
        if len(rows) != 3 or any(integer(row, 'all_layers_violations') for row in rows):
            return False
    return False


class Experiment(base.Experiment):
    def __init__(self, args):
        super().__init__(args)
        self.measurer = self.job / 'build-release/bin/eda_measure_documented'
        self.preset = base.SOURCE / 'scripts/run_a150_20261029.sh'
        for path in (self.measurer, self.preset):
            if not path.is_file():
                raise ValueError('Missing A150 program: ' + str(path))
        self.programs_before[str(self.measurer)] = base.sha(self.measurer)
        self.programs_before[str(self.preset)] = base.sha(self.preset)
        self.state.update(
            schema_version=3, rules_profile=RULES_PROFILE, tsv_obstacle_scope=args.tsv_obstacle_scope,
            assumptions=RULE_ASSUMPTIONS, selection_model='a150-20261029-pin-floor-existing-Q-proxy-G20',
            score_alignment=SCORE_ALIGNMENT, congestion_formula_status=SCORE_ALIGNMENT,
            official_score_alignment_confirmed=False,
            pin_zero_track_floor=True, subtree_optimization_allowed_after_this_gate=False,
            official_equivalence=False, compatibility_used_for_selection=False,
            external_thermal_gain=None, external_scorer_used=False,
            external_wall_budget_seconds=args.budget,
            internal_time_limit_seconds=args.budget - args.budget_reserve,
            baseline_differences_from_previous_Q1={
                'PIN_OBS_hard_prohibition': args.tsv_obstacle_scope,
                'resource_rules_profile': RULES_PROFILE,
                'pin_zero_track_floor': True,
                'v9_subtree_recovery': 0,
                'low_layer_mask': 0,
            },
            supply_only=args.supply_only, available_cpus=sorted(os.sched_getaffinity(0)),
            host_load_average=list(os.getloadavg()), programs_before=self.programs_before)
        self.publish()

    def common_validator(self):
        return [self.validator, '--model', 'official', '--score-profile', '20261020',
                '--tsv-resources', 'global-grid', '--rules-profile', RULES_PROFILE,
                '--tsv-obstacle-scope', self.args.tsv_obstacle_scope]

    def run_preflight(self, name):
        record = self.state['runs'][name]
        case = self.inputs[record['case']]
        folder = Path(record['folder'])
        self.update(record, status='SUPPLY_AUDIT')
        command = self.common_validator() + ['--tsv-supply-only', '1', '--def', case['def'][0],
                                             '--report', folder / 'supply.json']
        for lef in case['lef']:
            command += ['--lef', lef]
        self.execute(name, 'supply', command, 1800, self.environment(), allowed=(0, 1, 2))
        supply = supply_contract(base.read_json(folder / 'supply.json'), self.args.tsv_obstacle_scope)
        status = ('SUPPLY_INFEASIBLE' if not supply['selected_scope_supply_sufficient'] else
                  'FIXED_INTERFACE_INFEASIBLE' if supply['selected_scope_fixed_conflicts'] else 'SUPPLY_READY')
        self.update(record, status=status, active_step=None, supply=supply)
        print(name, record['status'], 'eligible=', supply['selected_eligible_cells'],
              'cross_die_nets=', supply['cross_die_nets'], flush=True)

    def run_route(self, name):
        record = self.state['runs'][name]
        if record['status'] != 'SUPPLY_READY':
            raise ValueError('routing requires successful supply preflight')
        case = self.inputs[record['case']]
        folder = Path(record['folder'])
        command = ['bash', self.preset, '--time-limit', str(self.args.budget - self.args.budget_reserve),
                   '--rules-profile', RULES_PROFILE, '--tsv-obstacle-scope', self.args.tsv_obstacle_scope,
                   '--thermal-gain', '20', '--v9-subtree-recovery', '0', '--low-layer-mask', '0',
                   '--rrr-reserved-seconds', '0', '--tsv-track-demand', '1',
                   '--tsv-landing-pricing', '1', '--tsv-landing-rrr', '0',
                   '--def', case['def'][0], '--thermal-report', case['thermal'][0],
                   '--output', folder / 'result.guide', '--stats', folder / 'stats.json']
        for lef in case['lef']:
            command += ['--lef', lef]
        self.update(record, status='ROUTING')
        self.execute(name, 'route', command, self.args.budget,
                     self.environment(True, self.runtime, self.binary))
        guide = folder / 'result.guide'
        stats = base.read_json(folder / 'stats.json')
        recorded_profile(stats, self.args.tsv_obstacle_scope)
        if stats['pin_zero_track_floor']['demand'] != record['supply']['pin_zero_track_floor_demand']:
            raise ValueError(name + ': router PIN floor demand differs from preflight')
        metrics = stats.get('metrics', {})
        if not guide.is_file() or not guide.stat().st_size:
            raise ValueError(name + ': no nonempty Guide')
        if not all(base.finite(metrics.get(k)) for k in
                   ('wirelength_score', 'congestion_score', 'thermal_score', 'correlation',
                    'guide_mst_dbu', 'squared_normalized_overflow', 'quality')):
            raise ValueError(name + ': incomplete router metrics')
        self.update(record, status='ROUTED', active_step=None,
                    guide_sha256=base.sha(guide), stats_sha256=base.sha(folder / 'stats.json'),
                    router_metrics=metrics, resource_v6=stats.get('resource_v6'),
                    counters=stats.get('counters'), options=stats.get('options'),
                    rules_contract=stats.get('rules_contract'), thermal_gain=stats.get('thermal_gain'))
        print(name, 'ROUTED', round(record['steps']['route']['seconds'], 2), flush=True)

    def run_evaluation(self, name):
        record = self.state['runs'][name]
        if record['status'] != 'ROUTED':
            return
        case = self.inputs[record['case']]
        folder = Path(record['folder'])
        guide = folder / 'result.guide'
        env = self.environment(True, self.runtime, self.binary)
        self.update(record, status='ENGINEERING_AUDIT')
        command = self.common_validator() + ['--def', case['def'][0], '--thermal', case['thermal'][0],
                                             '--guide', guide, '--report', folder / 'engineering.json']
        for lef in case['lef']:
            command += ['--lef', lef]
        self.execute(name, 'engineering', command, 1800, env)
        engineering = base.read_json(folder / 'engineering.json')
        legal = legal_contract(engineering, self.args.tsv_obstacle_scope, record['supply'])
        command = [sys.executable, base.SOURCE / 'scripts/audit_guide_geometry.py', '--def', case['def'][0],
                   '--guide', guide, '--report', folder / 'geometry.json']
        for lef in case['lef']:
            command += ['--lef', lef]
        self.execute(name, 'geometry', command, 1800, self.environment(), allowed=(0, 2))
        geometry = base.read_json(folder / 'geometry.json')
        if geometry.get('summary', {}).get('errors') != 0:
            raise ValueError(name + ': geometry errors')
        self.update(record, status='SERIALIZED_READBACK')
        command = [self.measurer, '--def', case['def'][0], '--thermal', case['thermal'][0], '--guide', guide,
                   '--runtime', self.runtime, '--rules-profile', RULES_PROFILE,
                   '--tsv-obstacle-scope', self.args.tsv_obstacle_scope,
                   '--output', folder / 'documented.json']
        for lef in case['lef']:
            command += ['--lef', lef]
        self.execute(name, 'documented', command, 1800, env)
        documented = base.read_json(folder / 'documented.json')
        recorded_profile(documented, self.args.tsv_obstacle_scope, serialized=True)
        if documented['pin_zero_track_floor']['demand'] != record['supply']['pin_zero_track_floor_demand']:
            raise ValueError(name + ': serialized PIN floor demand differs from preflight')
        m, router = documented['metrics'], record['router_metrics']
        for left, right in (('W', 'wirelength_score'), ('C', 'congestion_score'), ('T', 'thermal_score'),
                            ('rho', 'correlation'), ('L', 'guide_mst_dbu'), ('Q', 'squared_normalized_overflow')):
            if not base.finite(m.get(left)) or not math.isclose(m[left], router[right], rel_tol=1e-10, abs_tol=1e-8):
                raise ValueError(f'{name}: serialized {left} differs: {m.get(left)} vs {router[right]}')
        if not math.isclose(m['S'], 100 * router['quality'], rel_tol=1e-10, abs_tol=1e-8):
            raise ValueError(name + ': serialized S differs')
        resource = record.get('resource_v6')
        if resource is not None and resource.get('reservation_remaining') != 0:
            raise ValueError(name + ': nonzero future promise')
        if base.sha(guide) != record['guide_sha256'] or base.sha(folder / 'stats.json') != record['stats_sha256']:
            raise ValueError(name + ': Guide or stats changed during readback')
        row = dict(m, name=name, case=record['case'], variant='BASE',
                   pin_zero_track_floor_demand=documented['pin_zero_track_floor']['demand'],
                   score_alignment=SCORE_ALIGNMENT,
                   tsv_obstacle_scope=self.args.tsv_obstacle_scope,
                   interface_violations=legal['interface_violations'],
                   all_layers_violations=legal['all_layers_violations'],
                   selected_scope_violations=legal['selected_scope_violations'],
                   interface_eligible_cells=legal['interface_eligible_cells'],
                   all_layers_eligible_cells=legal['all_layers_eligible_cells'],
                   route_seconds=record['steps']['route']['seconds'],
                   score_seconds=record['steps']['documented']['seconds'],
                   geometry_unresolved=geometry['summary'].get('unresolved'),
                   guide_sha256=record['guide_sha256'])
        self.update(record, status='COMPLETE', active_step=None, row=row,
                    engineering_report=engineering, geometry_report=geometry, documented_report=documented)
        print(name, 'COMPLETE', 'A150 selected-scope violations=0', 'S=', round(m['S'], 6), flush=True)

    def finish(self):
        integrity = {
            'inputs_unchanged': base.files_hash(Path(p) for p in self.inputs_before) == self.inputs_before,
            'source_unchanged': base.source_hashes() == self.sources_before,
            'programs_unchanged': base.files_hash(Path(p) for p in self.programs_before) == self.programs_before,
            'runtime_unchanged': base.files_hash(Path(p) for p in self.runtime_before) == self.runtime_before,
        }
        runs = list(self.state['runs'].values())
        rows = [r['row'] for r in runs if r['status'] == 'COMPLETE']
        audit_ready = all(r['status'] in ('SUPPLY_READY', 'SUPPLY_INFEASIBLE', 'FIXED_INTERFACE_INFEASIBLE') for r in runs)
        insufficient = any(r['status'] == 'SUPPLY_INFEASIBLE' for r in runs)
        fixed_conflict = any(r['status'] == 'FIXED_INTERFACE_INFEASIBLE' for r in runs)
        infeasible_status = 'SUPPLY_INFEASIBLE' if insufficient else 'FIXED_INTERFACE_INFEASIBLE'
        all_complete = len(rows) == len(runs) and all(integrity.values())
        strict_baseline = all_complete and set(self.args.cases) == set(CASES) and len(rows) == 3
        subtree_gate = qualifies_for_subtree(rows, strict_baseline)
        if self.args.supply_only and audit_ready and all(integrity.values()):
            status = infeasible_status if insufficient or fixed_conflict else 'SUPPLY_AUDIT_COMPLETE'
        elif (insufficient or fixed_conflict) and audit_ready and all(integrity.values()):
            status = infeasible_status
        else:
            status = 'COMPLETE' if all_complete else 'ERROR'
        summary = {
            'status': status, 'rules_profile': RULES_PROFILE, 'tsv_obstacle_scope': self.args.tsv_obstacle_scope,
            'official_equivalence': False, 'official_platform_submission': False,
            'score_alignment': SCORE_ALIGNMENT, 'congestion_formula_status': SCORE_ALIGNMENT,
            'official_score_alignment_confirmed': False,
            'pin_zero_track_floor': True,
            'geometric_and_resource_baseline_confirmed': strict_baseline,
            'baseline_confirmed_under_recorded_rules': strict_baseline,
            'conservative_all_layer_baseline': strict_baseline and self.args.tsv_obstacle_scope == 'all-layers',
            'subtree_optimization_allowed_after_this_gate': subtree_gate,
            'external_scorer_used': False, 'compatibility_used_for_selection': False,
            'external_wall_budget_seconds': self.args.budget,
            'internal_time_limit_seconds': self.args.budget - self.args.budget_reserve,
            'assumptions': RULE_ASSUMPTIONS, 'integrity': integrity,
            'supply': {r['case']: r.get('supply') for r in runs},
            'failures': {r['case']: r.get('error', r['status']) for r in runs
                         if r['status'] not in ('COMPLETE', 'SUPPLY_READY')},
            'rows': rows,
        }
        if strict_baseline:
            summary['means'] = {key: sum(r[key] for r in rows) / 3 for key in ('W', 'C', 'T', 'S', 'route_seconds')}
        with (self.out / 'summary.json').open('x', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2, allow_nan=False)
            stream.write('\n')
        table = ['October 29 geometric/resource baseline under explicit assumptions; official_equivalence=false.',
                 'Score alignment: '+SCORE_ALIGNMENT+'; existing congestion proxy only; subtree gate remains closed.', '',
                 f'Status: {status}; scope: {self.args.tsv_obstacle_scope}; three-case gate: {strict_baseline}.', '',
                 '| case | W | C (proxy) | T (G20) | S (proxy) | interface forbidden | all-layer forbidden | route s |',
                 '|---|---:|---:|---:|---:|---:|---:|---:|']
        for row in sorted(rows, key=lambda r: r['case']):
            table.append('| {case} | {W:.4f} | {C:.4f} | {T:.4f} | {S:.4f} | {interface_violations} | {all_layers_violations} | {route_seconds:.2f} |'.format(**row))
        if not rows:
            table += ['', 'No baseline was certified. Supply audits and failures are preserved in summary.json.']
        (self.out / 'summary.md').write_text('\n'.join(table) + '\n', encoding='utf-8')
        self.update(self.state, status=status, finished_utc=base.utc(), integrity=integrity,
                    rows=rows, baseline_confirmed_under_recorded_rules=strict_baseline,
                    geometric_and_resource_baseline_confirmed=strict_baseline,
                    official_score_alignment_confirmed=False, score_alignment=SCORE_ALIGNMENT,
                    congestion_formula_status=SCORE_ALIGNMENT,
                    subtree_optimization_allowed_after_this_gate=subtree_gate)
        return 2 if status in ('SUPPLY_INFEASIBLE', 'FIXED_INTERFACE_INFEASIBLE') else (0 if status in ('COMPLETE', 'SUPPLY_AUDIT_COMPLETE') else 1)

    def run(self):
        names = list(self.state['runs'])
        try:
            self.update(self.state, status='SUPPLY_AUDIT')
            self.phase(self.run_preflight, names, self.args.workers)
            if not self.args.supply_only and all(r['status'] == 'SUPPLY_READY' for r in self.state['runs'].values()):
                self.update(self.state, status='ROUTING')
                self.phase(self.run_route, names, self.args.workers)
                self.update(self.state, status='EVALUATING')
                self.phase(self.run_evaluation, names, self.args.evaluation_workers)
            return self.finish()
        except Exception as error:
            self.update(self.state, status='ERROR', error=str(error), traceback=traceback.format_exc(),
                        finished_utc=base.utc())
            raise


def main(argv=None):
    args = parse_args(argv)
    try:
        return Experiment(args).run()
    except Exception as error:
        print('October 29 geometric/resource baseline ERROR:', error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())