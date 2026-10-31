#!/usr/bin/env python3
"""A150 BASE/SUBTREE single-batch screening on benchmark-host.

Require a completed conservative all-layer baseline before creating output.
Six same-binary routes run together; their sole option difference is the existing
v9-subtree-recovery switch. All routes finish before any readback starts. A single
screening pass never enables the mechanism by default or certifies a new baseline.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading

import run_a150_baseline as baseline

base = baseline.base
VARIANTS = ('BASE', 'SUBTREE')
INTEGRITY_KEYS = ('inputs_unchanged', 'source_unchanged', 'programs_unchanged', 'runtime_unchanged')
PRESETS = ('scripts/run_a150_v9.sh', 'scripts/run_tsv_resource_v7.sh',
           'scripts/run_tsv_resource_v6.sh', 'scripts/run_tsv_resource.sh', 'scripts/run_v9.sh')
MEAN_GAIN = .05
MAX_CASE_REGRESSION = .05
DERIVED_OPTION_FIELDS = frozenset(('congestion_price_updates', 'congestion_price_min', 'congestion_price_max'))
CLI_IGNORED_FIELDS = frozenset(('--output', '--stats', '--v9-subtree-recovery'))


def normalized_route_command(command):
    """Effective last-value CLI contract; LEF inputs remain accumulative."""
    if not isinstance(command, list) or len(command) < 2:
        raise ValueError('missing actual route command')
    values, lefs = {}, []
    index = 2  # bash, immutable hashed preset; all following arguments are pairs.
    while index < len(command):
        flag = command[index]
        if not isinstance(flag, str) or not flag.startswith('--') or index + 1 >= len(command):
            raise ValueError('malformed actual route CLI')
        value = command[index + 1]
        if flag == '--lef':
            lefs.append(value)
        elif flag not in CLI_IGNORED_FIELDS:
            values[flag] = value
        index += 2
    return {'entrypoint': command[:2], 'last_value_options': values, 'lef_inputs': lefs}


def normalized_stats_options(options):
    return {key: value for key, value in options.items()
            if key != 'v9_subtree_recovery' and key not in DERIVED_OPTION_FIELDS}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--baseline-gate', type=Path, required=True,
                        help='completed all-layers baseline summary.json; sibling status.json is required')
    parser.add_argument('--job-root', type=Path, default=base.SOURCE.parent)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--budget', type=float, default=1800)
    parser.add_argument('--budget-reserve', type=float, default=60)
    parser.add_argument('--workers', type=int, choices=(6,), default=6)
    parser.add_argument('--evaluation-workers', type=int, choices=(1, 2, 3), default=3)
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
    args.cases = list(baseline.CASES)
    args.variants = list(VARIANTS)
    args.tsv_obstacle_scope = 'all-layers'
    args.supply_only = False
    return args


def _rows_by_case(rows, label):
    if not isinstance(rows, list) or len(rows) != 3:
        raise ValueError(label + ': exactly three baseline rows are required')
    result = {}
    for row in rows:
        case = row.get('case')
        if case not in baseline.CASES or case in result or row.get('variant') != 'BASE':
            raise ValueError(label + ': duplicate, missing or non-BASE case')
        if baseline.normalized_scope(row.get('tsv_obstacle_scope')) != 'all-layers':
            raise ValueError(label + ': baseline row does not use all-layers')
        for key in ('interface_violations', 'all_layers_violations', 'selected_scope_violations'):
            if baseline.integer(row, key):
                raise ValueError(label + ': baseline row contains forbidden TSV landings')
        if not all(base.finite(row.get(key)) for key in ('W', 'C', 'T', 'S', 'route_seconds')):
            raise ValueError(label + ': incomplete finite scores')
        if not isinstance(row.get('guide_sha256'), str) or len(row['guide_sha256']) != 64:
            raise ValueError(label + ': missing Guide hash')
        result[case] = row
    return result


def validate_gate_documents(summary, status, current_sources, current_inputs):
    """Validate frozen evidence against actual current inputs and source hashes."""
    if summary.get('status') != 'COMPLETE' or status.get('status') != 'COMPLETE':
        raise ValueError('baseline gate is incomplete')
    for key in ('baseline_confirmed_under_recorded_rules', 'conservative_all_layer_baseline',
                'subtree_optimization_allowed_after_this_gate'):
        if summary.get(key) is not True:
            raise ValueError('baseline gate did not qualify: ' + key)
    if status.get('baseline_confirmed_under_recorded_rules') is not True or status.get('supply_only') is not False:
        raise ValueError('baseline status is a supply-only or unqualified run')
    for document in (summary, status):
        if document.get('rules_profile') != 'a150' or document.get('tsv_obstacle_scope') != 'all-layers':
            raise ValueError('baseline gate must use the A150 all-layer scope')
        integrity = document.get('integrity', {})
        if any(integrity.get(key) is not True for key in INTEGRITY_KEYS):
            raise ValueError('baseline integrity is not fully verified')
        if document.get('official_equivalence') is not False or document.get('compatibility_used_for_selection') is not False:
            raise ValueError('baseline gate has an unexpected evidence contract')
    rows = _rows_by_case(summary.get('rows'), 'baseline summary')
    status_rows = _rows_by_case(status.get('rows'), 'baseline status')
    if rows != status_rows:
        raise ValueError('baseline summary and status rows differ')
    runs = status.get('runs', {})
    if not isinstance(runs, dict) or len(runs) != 3:
        raise ValueError('baseline status does not contain three completed runs')
    seen = set()
    for record in runs.values():
        case = record.get('case')
        if case not in rows or case in seen or record.get('variant') != 'BASE' or record.get('status') != 'COMPLETE':
            raise ValueError('baseline run is incomplete, duplicated or has the wrong variant')
        seen.add(case)
        if record.get('row') != rows[case] or record.get('guide_sha256') != rows[case]['guide_sha256']:
            raise ValueError('baseline completed run differs from its summary row')
        supply = record.get('supply', {})
        if supply.get('selected_scope_supply_sufficient') is not True or supply.get('selected_scope_fixed_conflicts') != 0:
            raise ValueError('baseline supply gate did not pass')
        baseline.legal_contract(record.get('engineering_report', {}), 'all-layers', supply)
        baseline.recorded_profile(record.get('documented_report', {}), 'all-layers', serialized=True)
        if record.get('geometry_report', {}).get('summary', {}).get('errors') != 0:
            raise ValueError('baseline geometry audit contains errors')
        options = record.get('options', {})
        command = record.get('steps', {}).get('route', {}).get('command', [])
        def last_value(flag):
            indices = [i for i, value in enumerate(command[:-1]) if value == flag]
            return command[indices[-1] + 1] if indices else None
        if (options.get('v9_subtree_recovery') is not False or
                last_value('--v9-subtree-recovery') != '0' or last_value('--low-layer-mask') != '0'):
            raise ValueError('baseline already enabled a new operator or did not record its route switches')
    frozen_sources = status.get('source_files_before', {})
    frozen_algorithm = {key: value for key, value in frozen_sources.items() if key.startswith('src/')}
    current_algorithm = {key: value for key, value in current_sources.items() if key.startswith('src/')}
    if not frozen_algorithm or frozen_algorithm != current_algorithm:
        raise ValueError('current src algorithm hashes differ from the qualified baseline')
    for relative in PRESETS:
        if not frozen_sources.get(relative) or current_sources.get(relative) != frozen_sources[relative]:
            raise ValueError('routing preset changed after the qualified baseline: ' + relative)
    if not current_inputs or status.get('inputs_before') != current_inputs:
        raise ValueError('current input hashes differ from the qualified baseline')
    return rows


def input_hashes(case_root):
    files = []
    for case in baseline.CASES:
        folder = case_root.resolve() / ('open_3d_case' + case)
        for key, pattern in (('def', '*.input.def'), ('lef', '*.input.lef'), ('thermal', '*.thermal.input.json')):
            matches = sorted(path.resolve() for path in folder.glob(pattern) if path.is_file())
            if not matches or (key != 'lef' and len(matches) != 1):
                raise ValueError(str(folder) + ': ambiguous or missing ' + key)
            files.extend(matches)
    return base.files_hash(files)


def load_gate(args):
    summary_path = args.baseline_gate.resolve()
    status_path = summary_path.parent / 'status.json'
    if not summary_path.is_file() or not status_path.is_file():
        raise ValueError('baseline summary and sibling status.json must both exist')
    summary, status = base.read_json(summary_path), base.read_json(status_path)
    current_sources, current_inputs = base.source_hashes(), input_hashes(args.case_root)
    rows = validate_gate_documents(summary, status, current_sources, current_inputs)
    # The original completed artifacts must still be the bytes that qualified.
    for record in status['runs'].values():
        folder = Path(record['folder'])
        for filename, key in (('result.guide', 'guide_sha256'), ('stats.json', 'stats_sha256')):
            path = folder / filename
            if not path.is_file() or base.sha(path) != record.get(key):
                raise ValueError('baseline artifact changed or is missing: ' + str(path))
        stats = base.read_json(folder / 'stats.json')
        baseline.recorded_profile(stats, 'all-layers')
        if stats.get('low_layer', {}).get('mask') != 0 or stats.get('options') != record.get('options'):
            raise ValueError('baseline stats differ from the qualified options')
    evidence_files = {str(summary_path): base.sha(summary_path), str(status_path): base.sha(status_path)}
    return {'summary_path': str(summary_path), 'status_path': str(status_path),
            'files': evidence_files, 'algorithm_hashes': {k: v for k, v in current_sources.items() if k.startswith('src/')},
            'input_hashes': current_inputs, 'rows': rows}


def screening(rows, integrity):
    complete = (len(rows) == 6 and all(integrity.get(key) is True for key in INTEGRITY_KEYS))
    grouped = {(row.get('case'), row.get('variant')): row for row in rows}
    expected = {(case, variant) for case in baseline.CASES for variant in VARIANTS}
    complete = complete and len(grouped) == 6 and set(grouped) == expected
    if not complete:
        return {'complete': False, 'screening_passed': False, 'case_deltas': [], 'mean_delta_S': None}
    for row in rows:
        if baseline.integer(row, 'all_layers_violations') or baseline.integer(row, 'selected_scope_violations'):
            raise ValueError('ablation contains forbidden TSV landings')
    deltas = [dict(case=case, delta_S=grouped[case, 'SUBTREE']['S'] - grouped[case, 'BASE']['S'])
              for case in baseline.CASES]
    mean = sum(row['delta_S'] for row in deltas) / 3
    return {'complete': True, 'screening_passed': mean >= MEAN_GAIN and
            all(row['delta_S'] >= -MAX_CASE_REGRESSION for row in deltas),
            'case_deltas': deltas, 'mean_delta_S': mean}


class Experiment(baseline.Experiment):
    def __init__(self, args):
        # No output, route, or scoring process can precede this evidence check.
        gate = load_gate(args)
        base.VARIANTS = {variant: (1, 1, 0) for variant in VARIANTS}
        super().__init__(args)
        self.route_barrier = threading.Barrier(6)
        self.gate = gate
        self.programs_before.update(gate['files'])
        self.state.update(experiment_kind='a150-existing-subtree-single-batch-screen',
                          baseline_gate=gate, programs_before=self.programs_before,
                          baseline_confirmed_under_recorded_rules=False,
                          conservative_all_layer_baseline=False,
                          single_switch='--v9-subtree-recovery', variants={variant: int(variant == 'SUBTREE') for variant in VARIANTS},
                          predeclared_screen={'minimum_three_case_mean_gain_S': MEAN_GAIN,
                                             'maximum_per_case_regression_S': MAX_CASE_REGRESSION,
                                             'all_six_validations_required': True,
                                             'single_batch_never_changes_default': True},
                          interpretation='end-to-end switch plus existing adaptive scheduling response;not isolated stage gain')
        self.publish()

    def execute(self, name, step, command, timeout, env, allowed=(0,)):
        command = list(command)
        if step == 'route':
            variant = self.state['runs'][name]['variant']
            command += ['--v9-subtree-recovery', str(int(variant == 'SUBTREE'))]
            self.route_barrier.wait(timeout=60)
        return super().execute(name, step, command, timeout, env, allowed)

    def run_route(self, name):
        super().run_route(name)
        record = self.state['runs'][name]
        stats = base.read_json(Path(record['folder']) / 'stats.json')
        if stats.get('options', {}).get('v9_subtree_recovery') is not (record['variant'] == 'SUBTREE'):
            raise ValueError(name + ': actual subtree option differs from the declared variant')
        self.update(record, budgeted_wirelength_recovery=stats.get('budgeted_wirelength_recovery'),
                    low_layer=stats.get('low_layer'), initial_metrics=stats.get('initial_metrics'))

    def run_evaluation(self, name):
        super().run_evaluation(name)
        record = self.state['runs'][name]
        if record['status'] == 'COMPLETE':
            row = dict(record['row'], variant=record['variant'])
            self.update(record, row=row)

    def finish(self):
        integrity = {
            'inputs_unchanged': base.files_hash(Path(path) for path in self.inputs_before) == self.inputs_before,
            'source_unchanged': base.source_hashes() == self.sources_before,
            'programs_unchanged': base.files_hash(Path(path) for path in self.programs_before) == self.programs_before,
            'runtime_unchanged': base.files_hash(Path(path) for path in self.runtime_before) == self.runtime_before,
        }
        records = list(self.state['runs'].values())
        rows = [record['row'] for record in records if record['status'] == 'COMPLETE']
        option_pairs_equal, command_pairs_equal = True, True
        command_contracts, derived_statistics = {}, {}
        for case in baseline.CASES:
            pair = {record['variant']: record for record in records if record['case'] == case}
            if all(isinstance(pair.get(variant, {}).get('options'), dict) for variant in VARIANTS):
                normalized = [normalized_stats_options(pair[variant]['options']) for variant in VARIANTS]
                option_pairs_equal &= normalized[0] == normalized[1]
                derived_statistics[case] = {
                    variant: {key: pair[variant]['options'].get(key) for key in sorted(DERIVED_OPTION_FIELDS)}
                    for variant in VARIANTS}
            else:
                option_pairs_equal = False
            try:
                command_contracts[case] = {
                    variant: normalized_route_command(pair[variant]['steps']['route']['command']) for variant in VARIANTS}
                command_pairs_equal &= command_contracts[case]['BASE'] == command_contracts[case]['SUBTREE']
            except (KeyError, ValueError, TypeError):
                command_pairs_equal = False
        option_pairs_equal &= command_pairs_equal
        outcome = screening(rows, integrity)
        outcome['screening_passed'] &= option_pairs_equal
        complete = outcome['complete'] and option_pairs_equal
        outcome['complete'] = complete
        status = 'COMPLETE' if complete else 'ERROR'
        summary = dict(status=status, experiment_kind='a150-existing-subtree-single-batch-screen',
                       rules_profile='a150', tsv_obstacle_scope='all-layers', baseline_gate=self.gate,
                       official_equivalence=False, official_platform_submission=False,
                       external_scorer_used=False, compatibility_used_for_selection=False,
                       baseline_confirmed_under_recorded_rules=False, default_subtree_enabled=False,
                       default_change_authorized_by_screen=False, confirmation_required=True,
                       single_batch_screen=True, option_pairs_equal_except_subtree_switch=option_pairs_equal,
                       normalized_cli_pairs_equal=command_pairs_equal, normalized_route_commands=command_contracts,
                       derived_execution_statistics=derived_statistics,
                       stats_configuration_ignored_derived_fields=sorted(DERIVED_OPTION_FIELDS),
                       cli_ignored_output_and_variant_fields=sorted(CLI_IGNORED_FIELDS),
                       predeclared_screen=self.state['predeclared_screen'], integrity=integrity,
                       external_wall_budget_seconds=self.args.budget,
                       internal_time_limit_seconds=self.args.budget - self.args.budget_reserve,
                       assumptions=baseline.RULE_ASSUMPTIONS, rows=rows, **outcome,
                       failures={name: record.get('error', record['status']) for name, record in self.state['runs'].items()
                                 if record['status'] != 'COMPLETE'})
        if complete:
            summary['means'] = {variant: {key: sum(row[key] for row in rows if row['variant'] == variant) / 3
                                         for key in ('W', 'C', 'T', 'S', 'route_seconds')}
                                for variant in VARIANTS}
        with (self.out / 'ablation-summary.json').open('x', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2, allow_nan=False)
            stream.write('\n')
        lines = ['A150 subtree single-batch screen; official_equivalence=false; default_subtree_enabled=false.', '',
                 'Only --v9-subtree-recovery differs. Adaptive scheduling response is included in final gains.', '',
                 f'Status: {status}; screening passed: {outcome["screening_passed"]}.', '',
                 '| case | BASE S | SUBTREE S | delta S |', '|---|---:|---:|---:|']
        indexed = {(row['case'], row['variant']): row for row in rows}
        for delta in outcome['case_deltas']:
            case = delta['case']
            lines.append(f'| {case} | {indexed[case,"BASE"]["S"]:.6f} | {indexed[case,"SUBTREE"]["S"]:.6f} | {delta["delta_S"]:+.6f} |')
        lines += ['', 'Predeclared screen: mean delta S >= 0.05; every case delta S >= -0.05.',
                  'A pass requires later confirmation; this experiment does not enable the operator by default.']
        with (self.out / 'ablation-summary.md').open('x', encoding='utf-8') as stream:
            stream.write('\n'.join(lines) + '\n')
        self.update(self.state, status=status, finished_utc=base.utc(), integrity=integrity, rows=rows,
                    ablation_summary='ablation-summary.json', screening_passed=outcome['screening_passed'],
                    baseline_confirmed_under_recorded_rules=False, default_subtree_enabled=False)
        return 0 if complete else 1


def main(argv=None):
    args = parse_args(argv)
    try:
        return Experiment(args).run()
    except Exception as error:
        print('A150 subtree ablation ERROR:', error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
