#!/usr/bin/env python3
"""Same-binary A150 v11 BASE/TREE engineering and raw-metric diagnostics.

This experiment does not unlock a performance gate: October 29 congestion
penalties and official thermal gain remain unknown. It reuses the October 29
preflight, PIN-floor, actual Guide legality and serialized resource checks.
All requested routes run in one matched batch before evaluation begins.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading

import run_a150_20261029_baseline as baseline

base = baseline.base
VARIANTS = ('BASE', 'TREE')
DERIVED_OPTION_FIELDS = frozenset(('congestion_price_updates', 'congestion_price_min',
                                 'congestion_price_max'))
SWITCH_FIELDS = frozenset(('tsv_tree_move',))
IGNORED_CLI = frozenset(('--output', '--stats', '--tsv-tree-move'))
LENGTH_FIELDS = ('planar_dbu', 'ordinary_via_dbu', 'tsv_dbu', 'total_dbu')
RAW_FIELDS = ('L', 'rho', 'Q', 'overflow', 'W', 'C', 'T', 'S', 'route_seconds')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--job-root', type=Path, default=base.SOURCE.parent)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--budget', type=float, default=1800)
    parser.add_argument('--budget-reserve', type=float, default=60)
    parser.add_argument('--cases', nargs='+', choices=baseline.CASES, default=list(baseline.CASES))
    parser.add_argument('--workers', type=int, choices=(2, 4, 6), default=None)
    parser.add_argument('--evaluation-workers', type=int, choices=(1, 2, 3), default=3)
    parser.add_argument('--tree-seconds', type=float, default=.02)
    parser.add_argument('--tree-expansions', type=int, default=4096)
    parser.add_argument('--case-root', type=Path,
                        default=base.CACHE / 'full-suite-20261018-04ab4298/cases')
    parser.add_argument('--runtime', type=Path,
                        default=base.CACHE / 'community040-fixed-20261024-341df8d5/submission/runtime/official-flute')
    parser.add_argument('--dependency-root', type=Path,
                        default=base.CACHE / 'community042-audit-dff686f959/package')
    args = parser.parse_args(argv)
    if not math.isfinite(args.budget) or not 60 <= args.budget <= 43200:
        parser.error('budget must be finite and 60..43200 seconds')
    if not math.isfinite(args.budget_reserve) or not 0 <= args.budget_reserve < args.budget - 30:
        parser.error('budget reserve must leave at least 30 internal seconds')
    if len(set(args.cases)) != len(args.cases):
        parser.error('duplicate cases')
    required_workers = 2 * len(args.cases)
    if args.workers is None:
        args.workers = required_workers
    if args.workers != required_workers:
        parser.error('workers must equal twice the case count for a single matched batch')
    if not math.isfinite(args.tree_seconds) or not 0 < args.tree_seconds <= 1:
        parser.error('tree-seconds must be finite and >0..1 seconds')
    if not 1 <= args.tree_expansions <= 1000000:
        parser.error('tree-expansions must be 1..1000000')
    args.variants = list(VARIANTS)
    args.tsv_obstacle_scope = 'all-layers'
    args.supply_only = False
    return args


def normalized_command(command):
    """Compare effective wrapper arguments; repeated values use CLI last-wins."""
    if not isinstance(command, list) or len(command) < 2 or command[0] != 'bash':
        raise ValueError('missing actual preset route command')
    if (len(command) - 2) % 2:
        raise ValueError('malformed route command pairs')
    values, lefs = {}, []
    for index in range(2, len(command), 2):
        flag, value = command[index:index + 2]
        if not isinstance(flag, str) or not flag.startswith('--'):
            raise ValueError('malformed route flag')
        if flag == '--lef':
            lefs.append(value)
        elif flag not in IGNORED_CLI:
            values[flag] = value
    return {'entrypoint': command[:2], 'values': values, 'lefs': lefs}


def normalized_options(options):
    if not isinstance(options, dict) or not options:
        raise ValueError('missing actual router options')
    return {key: value for key, value in options.items()
            if key not in DERIVED_OPTION_FIELDS and key not in SWITCH_FIELDS}


def tree_configuration(stats, enabled, seconds, expansions):
    tree = stats.get('tsv_tree_move', {})
    if not isinstance(tree, dict) or tree.get('enabled') is not enabled:
        raise ValueError('tree-move stats do not match requested variant')
    # These are configuration values, not accumulated work counters.
    if not base.finite(tree.get('seconds_limit')) or not math.isclose(
            tree['seconds_limit'], seconds, rel_tol=0, abs_tol=1e-12):
        raise ValueError('tree-move per-candidate seconds differ')
    if type(tree.get('expansions_limit')) is not int or tree['expansions_limit'] != expansions:
        raise ValueError('tree-move expansion limit differs')
    if 'tsv_tree_move' in stats.get('options', {}) and stats['options']['tsv_tree_move'] is not enabled:
        raise ValueError('tree-move options conflict with stats')
    return tree


def raw_row(record):
    row = dict(record['row'])
    row['variant'] = record['variant']
    metrics = record.get('router_metrics', {})
    overflow = metrics.get('guide_overflow')
    if type(overflow) is not int or overflow < 0:
        raise ValueError('missing integer route overflow')
    row['overflow'] = overflow
    breakdown = record.get('wirelength_breakdown', {})
    if not all(base.finite(breakdown.get(key)) and breakdown[key] >= 0 for key in LENGTH_FIELDS):
        raise ValueError('missing raw charged-length decomposition')
    if not math.isclose(sum(breakdown[k] for k in LENGTH_FIELDS[:3]), breakdown['total_dbu'],
                        rel_tol=1e-10, abs_tol=1e-8):
        raise ValueError('charged-length decomposition does not sum')
    if not math.isclose(breakdown['total_dbu'], row['L'], rel_tol=1e-10, abs_tol=1e-8):
        raise ValueError('charged-length decomposition differs from serialized L')
    if not all(base.finite(row.get(k)) for k in RAW_FIELDS):
        raise ValueError('missing finite raw/explicit proxy result')
    row['wirelength_breakdown'] = breakdown
    row['tree_diagnostics'] = record['tsv_tree_move']
    return row


def compare_pair(left, right):
    """Reject confounded pairs before producing diagnostic deltas."""
    if left.get('variant') != 'BASE' or right.get('variant') != 'TREE':
        raise ValueError('pair needs BASE and TREE')
    if left.get('case') != right.get('case') or left.get('status') != 'COMPLETE' or right.get('status') != 'COMPLETE':
        raise ValueError('pair incomplete or cases differ')
    a, b = left['steps']['route'], right['steps']['route']
    if normalized_command(a['command']) != normalized_command(b['command']):
        raise ValueError('paired route commands differ beyond the single tree switch')
    for record, expected in ((left, '0'), (right, '1')):
        command = record['steps']['route']['command']
        flags = [command[i + 1] for i, value in enumerate(command[:-1]) if value == '--tsv-tree-move']
        if flags != [expected]:
            raise ValueError('missing or duplicated tree variant switch')
    if normalized_options(left.get('options')) != normalized_options(right.get('options')):
        raise ValueError('paired router options differ beyond the single tree switch')
    if a.get('timeout_seconds') != b.get('timeout_seconds'):
        raise ValueError('paired external budgets differ')
    if left.get('supply') != right.get('supply'):
        raise ValueError('paired geometry/fixed-resource supply differs')
    if left.get('thermal_gain') != 20 or right.get('thermal_gain') != 20:
        raise ValueError('paired results require explicit uniform G20 proxy')
    if left.get('rules_contract') != baseline.A150_CONTRACT or right.get('rules_contract') != baseline.A150_CONTRACT:
        raise ValueError('paired resource contracts differ from October 29')
    if left.get('route_binary_sha256') != right.get('route_binary_sha256') or not left.get('route_binary_sha256'):
        raise ValueError('paired route binaries differ')
    x, y = raw_row(left), raw_row(right)
    if x.get('pin_zero_track_floor_demand') != y.get('pin_zero_track_floor_demand'):
        raise ValueError('paired PIN floor totals differ')
    for row in (x, y):
        if any(row.get(key) != 0 for key in ('interface_violations', 'all_layers_violations', 'selected_scope_violations')):
            raise ValueError('paired result contains forbidden TSV landing')
    return {
        'case': left['case'], 'comparison_valid': True,
        'same_binary': True, 'same_options_except_tree_switch': True,
        'BASE': x, 'TREE': y,
        'delta_TREE_minus_BASE': {key: y[key] - x[key] for key in RAW_FIELDS},
        'delta_wirelength_breakdown': {key: y['wirelength_breakdown'][key] - x['wirelength_breakdown'][key]
                                       for key in LENGTH_FIELDS},
        'proxy_only': True, 'formal_performance_adoption_allowed': False,
    }


class Experiment(baseline.Experiment):
    def __init__(self, args):
        # Parent constructor uses the registry only to label runs. Restore it
        # immediately; do not change a historical harness's gate or globals.
        registry = base.VARIANTS
        try:
            base.VARIANTS = {variant: (1, 1, 0) for variant in VARIANTS}
            super().__init__(args)
        finally:
            base.VARIANTS = registry
        self.route_barrier = threading.Barrier(args.workers)
        self.update(self.state, schema_version=4, experiment='a150-v11-tree-move-diagnostic',
                    proxy_only=True, formal_performance_adoption_allowed=False,
                    official_alignment=False, official_score_alignment_confirmed=False,
                    subtree_optimization_allowed_after_this_gate=False,
                    variants={'BASE': {'tsv_tree_move': False}, 'TREE': {'tsv_tree_move': True}},
                    tree_seconds_limit=args.tree_seconds, tree_expansions_limit=args.tree_expansions,
                    comparison_scope='same-binary-single-batch; raw metrics and existing score proxy only')

    def execute(self, name, step, command, timeout, env, allowed=(0,)):
        if step == 'route':
            variant = self.state['runs'][name]['variant']
            command = list(command) + ['--tsv-tree-move', '1' if variant == 'TREE' else '0',
                                      '--tsv-tree-move-seconds', str(self.args.tree_seconds),
                                      '--tsv-tree-move-expansions', str(self.args.tree_expansions)]
        return super().execute(name, step, command, timeout, env, allowed)

    def run_route(self, name):
        self.route_barrier.wait(timeout=120)
        super().run_route(name)
        record = self.state['runs'][name]
        stats = base.read_json(Path(record['folder']) / 'stats.json')
        tree = tree_configuration(stats, record['variant'] == 'TREE',
                                  self.args.tree_seconds, self.args.tree_expansions)
        self.update(record, tsv_tree_move=tree,
                    wirelength_breakdown={k: v for k, v in stats.get('wirelength_breakdown', {}).items() if k != 'nets'},
                    resource_v7=stats.get('resource_v7'), diagnostics_v11=stats.get('diagnostics_v11'),
                    initial_metrics=stats.get('initial_metrics'),
                    route_binary_sha256=self.programs_before[str(self.binary)])

    def run_evaluation(self, name):
        super().run_evaluation(name)
        record = self.state['runs'][name]
        if record['status'] == 'COMPLETE':
            self.update(record, row=raw_row(record))

    def finish(self):
        integrity = {
            'inputs_unchanged': base.files_hash(Path(p) for p in self.inputs_before) == self.inputs_before,
            'source_unchanged': base.source_hashes() == self.sources_before,
            'programs_unchanged': base.files_hash(Path(p) for p in self.programs_before) == self.programs_before,
            'runtime_unchanged': base.files_hash(Path(p) for p in self.runtime_before) == self.runtime_before,
        }
        records = list(self.state['runs'].values())
        pairs, comparison_errors = [], {}
        for case in self.args.cases:
            try:
                pairs.append(compare_pair(self.state['runs'][case + '-BASE'], self.state['runs'][case + '-TREE']))
            except (KeyError, ValueError, TypeError) as error:
                comparison_errors[case] = str(error)
        complete = len(pairs) == len(self.args.cases) and all(integrity.values())
        status = 'COMPLETE' if complete else 'ERROR'
        summary = {
            'status': status, 'experiment': 'a150-v11-tree-move-diagnostic',
            'rules_profile': baseline.RULES_PROFILE, 'tsv_obstacle_scope': 'all-layers',
            'proxy_only': True, 'formal_performance_adoption_allowed': False,
            'official_alignment': False, 'official_score_alignment_confirmed': False,
            'official_equivalence': False, 'official_platform_submission': False,
            'subtree_optimization_allowed_after_this_gate': False,
            'baseline_confirmed_under_recorded_rules': False,
            'geometric_resource_pair_checks_complete': complete,
            'congestion_formula_status': baseline.SCORE_ALIGNMENT,
            'score_alignment': baseline.SCORE_ALIGNMENT,
            'assumptions': baseline.RULE_ASSUMPTIONS,
            'cases': self.args.cases, 'workers': self.args.workers,
            'external_wall_budget_seconds': self.args.budget,
            'internal_time_limit_seconds': self.args.budget - self.args.budget_reserve,
            'integrity': integrity, 'comparison_errors': comparison_errors,
            'failures': {r['case'] + '-' + r['variant']: r.get('error', r['status'])
                         for r in records if r['status'] != 'COMPLETE'},
            'pairs': pairs,
            'limitations': [
                'No formal gain gate: new congestion formula and official G_case are unconfirmed.',
                'A single matched batch is diagnostic; no statistical significance or default adoption is implied.',
                'Geometry unresolved classifications remain explicit; errors=0 is not full DRC certification.',
                'Independent Python reference conversion discrepancy is not repaired or waived by this C++ readback.',
            ],
        }
        if complete:
            summary['mean_deltas_TREE_minus_BASE'] = {
                key: sum(pair['delta_TREE_minus_BASE'][key] for pair in pairs) / len(pairs)
                for key in RAW_FIELDS}
        with (self.out / 'diagnostic-summary.json').open('x', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2, allow_nan=False)
            stream.write('\n')
        lines = ['A150 v11 tree-preserving TSV migration: engineering/raw-metric diagnostic.',
                 '', f'Status: {status}. Proxy only; formal performance adoption remains disabled.',
                 'All deltas are TREE minus BASE. C, T and S retain the explicitly unconfirmed proxy.', '',
                 '| case | delta L DBU | delta rho | delta Q proxy | delta overflow | delta S proxy | BASE/TREE seconds |',
                 '|---|---:|---:|---:|---:|---:|---:|']
        for pair in pairs:
            d = pair['delta_TREE_minus_BASE']
            lines.append(f"| {pair['case']} | {d['L']:.6f} | {d['rho']:.9g} | {d['Q']:.9g} | {d['overflow']} | {d['S']:.6f} | {pair['BASE']['route_seconds']:.2f}/{pair['TREE']['route_seconds']:.2f} |")
        if comparison_errors:
            lines += ['', 'Pair comparison errors:', json.dumps(comparison_errors, indent=2)]
        lines += ['', *summary['limitations']]
        with (self.out / 'diagnostic-summary.md').open('x', encoding='utf-8') as stream:
            stream.write('\n'.join(lines) + '\n')
        self.update(self.state, status=status, finished_utc=base.utc(), integrity=integrity,
                    pairs=pairs, comparison_errors=comparison_errors,
                    geometric_resource_pair_checks_complete=complete,
                    proxy_only=True, formal_performance_adoption_allowed=False,
                    official_alignment=False, official_score_alignment_confirmed=False,
                    baseline_confirmed_under_recorded_rules=False,
                    subtree_optimization_allowed_after_this_gate=False)
        return 0 if complete else 1


def main(argv=None):
    args = parse_args(argv)
    try:
        return Experiment(args).run()
    except Exception as error:
        print('A150 v11 diagnostic ERROR:', error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
