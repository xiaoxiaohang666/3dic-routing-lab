#!/usr/bin/env python3
"""Controlled low-layer experiments; execute on benchmark-host only.

Every route has the same external wall-clock watchdog and internal time limit.
Repetitions run in separate serial batches, each followed by validation/scoring.
Documented G=20 readback is the sole selection model. Compatibility scores are
reported separately and never influence the adoption decision.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

import run_tsv_resource_ablation as base
import run_tsv_resource_v6_ablation as v6

VARIANT_FLAGS = {
    'BASE': (0, 0),
    'M1': (1, 0),
    'M11': (2, 0),
    'BOTH': (3, 0),
    'SCHED200': (0, 200),
    'SCHED400': (0, 400),
    'BOTH200': (3, 200),
    'BOTH400': (3, 400),
}
v6.CONFIGS = {key: ((1, 1, 0), (0, 0, 0, 0, 0)) for key in VARIANT_FLAGS}
base.VARIANTS = {key: value[0] for key, value in v6.CONFIGS.items()}


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument('--job-root', type=Path, default=base.SOURCE.parent)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--budget', type=float, default=1800,
                   help='External route-process wall-clock watchdog, including parsing/export.')
    p.add_argument('--budget-reserve', type=float, default=60,
                   help='Uniform reserve: internal time-limit is budget minus this value.')
    p.add_argument('--variants', nargs='+', choices=tuple(VARIANT_FLAGS), default=['BASE', 'BOTH'])
    p.add_argument('--cases', nargs='+', choices=('001', '003', '005'), default=['001', '003', '005'])
    p.add_argument('--workers', type=int, default=6)
    p.add_argument('--evaluation-workers', type=int, choices=(1, 2, 3), default=3)
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--protocol', choices=('screen', 'confirmation'), default='screen')
    p.add_argument('--e1-evidence', type=Path,
                   help='Required for confirmation: reviewed JSON with documented_gate_passed=true.')
    p.add_argument('--low-layer-seconds', type=float, default=200)
    p.add_argument('--low-layer-max-attempts', type=int, default=20000)
    p.add_argument('--case-root', type=Path,
                   default=base.CACHE / 'full-suite-20261018-04ab4298/cases')
    p.add_argument('--runtime', type=Path,
                   default=base.CACHE / 'community040-fixed-20261024-341df8d5/submission/runtime/official-flute')
    p.add_argument('--dependency-root', type=Path,
                   default=base.CACHE / 'community042-audit-dff686f959/package')
    p.add_argument('--score-source', type=Path, default=base.SOURCE,
                   help='Frozen source containing calibrated diagnostic scorer 0.3.0.')
    p.add_argument('--skip-diagnostic', action='store_true',
                   help='Exploratory screen only; documented checks still run.')
    p.add_argument('--minimum-mean-gain', type=float, default=.05,
                   help='Predeclared engineering threshold in documented quality points.')
    p.add_argument('--maximum-case-regression', type=float, default=.05)
    a = p.parse_args(argv)
    for name in ('budget', 'budget_reserve', 'low_layer_seconds', 'minimum_mean_gain',
                 'maximum_case_regression'):
        if not math.isfinite(getattr(a, name)):
            p.error(name + ' must be finite')
    if not 60 <= a.budget <= 43200 or not 0 <= a.budget_reserve < a.budget - 30:
        p.error('budget must be 60..43200 and leave at least 30 internal seconds')
    if not 0 < a.low_layer_seconds <= 3600 or not 1 <= a.low_layer_max_attempts <= 1000000:
        p.error('invalid low-layer stage budget')
    if a.minimum_mean_gain < 0 or a.maximum_case_regression < 0:
        p.error('selection thresholds must be nonnegative')
    if not 1 <= a.repeats <= 10 or not 1 <= a.workers <= 12:
        p.error('repeats must be 1..10 and workers 1..12')
    if len(set(a.variants)) != len(a.variants) or len(set(a.cases)) != len(a.cases):
        p.error('duplicate variants or cases')
    if 'BASE' not in a.variants:
        p.error('every experiment must include BASE')
    if a.protocol == 'confirmation':
        if a.cases != ['001', '003', '005'] or a.repeats < 3:
            p.error('confirmation requires all three cases and at least three separate batches')
        if a.workers != len(a.cases) * len(a.variants):
            p.error('confirmation workers must match batch size, so each configuration starts together')
        if a.skip_diagnostic:
            p.error('confirmation must include diagnostic readback')
        if a.e1_evidence is None or not a.e1_evidence.is_file():
            p.error('confirmation requires existing E1 gate evidence')
        gate = base.read_json(a.e1_evidence)
        if gate.get('documented_gate_passed') is not True:
            p.error('E1 evidence must explicitly pass the documented physical gate')
    return a


class Experiment(v6.Experiment):
    def __init__(self, args, repeat):
        self.repeat = repeat
        super().__init__(args)
        self.score_source = args.score_source.resolve()
        self.diagnostic_program = self.score_source / 'scripts/score_20261027.py'
        if not args.skip_diagnostic:
            paths = [self.diagnostic_program]
            paths += list((self.score_source / 'tools/scorer_20261027').glob('*.py'))
            if not self.diagnostic_program.is_file() or len(paths) < 3:
                raise ValueError('Missing calibrated diagnostic scorer source')
            for path in paths:
                self.programs_before[str(path)] = base.sha(path)
        self.state.update(
            schema_version=2, repeat=repeat, protocol=args.protocol,
            external_wall_budget_seconds=args.budget,
            internal_time_limit_seconds=args.budget - args.budget_reserve,
            uniform_external_reserve_seconds=args.budget_reserve,
            watchdog_grace_policy='SIGTERM then up to 15s cleanup; any timeout is a failed run',
            compatibility_used_for_selection=False,
            host_load_average=list(os.getloadavg()),
            available_cpus=sorted(os.sched_getaffinity(0)),
            score_source=str(self.score_source),
            experimental_flags={v: self.flags(v) for v in args.variants},
            programs_before=self.programs_before,
            predeclared_selection={
                'model': 'documented-rules-G20',
                'minimum_three_case_mean_gain': args.minimum_mean_gain,
                'all_repetition_three_case_mean_gains_positive': True,
                'maximum_per_case_mean_regression': args.maximum_case_regression,
                'engineering_errors': 0,
                'geometry_errors': 0,
                'unresolved_geometry_is_reported_not_certified': True,
                'complete_guide_resource_readback_required': True,
            })
        self.publish()

    def flags(self, variant):
        mask, reserved = VARIANT_FLAGS[variant]
        return ['--tsv-index-reuse', '0', '--checkpoint-reuse', '0',
                '--checkpoint-amortization', '0',
                '--low-layer-mask', str(mask),
                '--low-layer-seconds', str(self.args.low_layer_seconds),
                '--low-layer-max-attempts', str(self.args.low_layer_max_attempts),
                '--rrr-reserved-seconds', str(reserved)]

    def execute(self, name, step, command, timeout, env, allowed=(0,)):
        command = list(command)
        if step == 'route':
            # Last CLI occurrence wins. Count every process second under one watchdog.
            command += ['--time-limit', str(self.args.budget - self.args.budget_reserve)]
            command += self.flags(self.state['runs'][name]['variant'])
            timeout = self.args.budget
        return super().execute(name, step, command, timeout, env, allowed)

    def run_route(self, name):
        super().run_route(name)
        record = self.state['runs'][name]
        stats = base.read_json(Path(record['folder']) / 'stats.json')
        extra = {k: v for k, v in stats.items()
                 if 'low_layer' in k or k in ('resource_v7', 'resource_v8', 'v8_low_layer')}
        self.update(record, low_layer_statistics=extra, repeat=self.repeat,
                    external_watchdog_passed=not record['steps']['route'].get('timeout', False))

    def run_evaluation(self, name):
        # v6 checks real own-pin resource ownership, full via demand, geometry and
        # exact exported Guide parity against the documented production model.
        super().run_evaluation(name)
        record = self.state['runs'][name]
        if record['status'] != 'COMPLETE':
            return
        record['row']['repeat'] = self.repeat
        record['row']['external_budget_seconds'] = self.args.budget
        record['row']['internal_time_limit_seconds'] = self.args.budget - self.args.budget_reserve
        if self.args.skip_diagnostic:
            self.update(record, diagnostic_status='SKIPPED_EXPLORATORY')
            return
        folder = Path(record['folder'])
        case = self.inputs[record['case']]
        guide = folder / 'result.guide'
        self.update(record, status='DIAGNOSTIC_SCORING', diagnostic_status='RUNNING')
        command = [sys.executable, self.diagnostic_program, guide,
                   '--input-def', case['def'][0], '--thermal-json', case['thermal'][0],
                   '--output-dir', folder / 'diagnostic-score', '--dependency-root', self.package,
                   '--rules-profile', 'inferred-platform',
                   '--capacity-boundary', 'half-open', '--fixed-track-boundary', 'half-open',
                   '--landing-track-boundary', 'open', '--lefs', *case['lef']]
        self.execute(name, 'diagnostic', command, 3600, self.environment())
        report = base.read_json(folder / 'diagnostic-score/evaluation_report.json')
        audit = base.read_json(folder / 'diagnostic-score/run_record.json')
        if (report.get('valid_score') is not True or report.get('evaluation_outcome') != 'SCORED'):
            raise ValueError(name + ': diagnostic scoring failed')
        if audit.get('production_candidate_acceptance_supported') is not False:
            raise ValueError(name + ': diagnostic scorer lacks non-production boundary')
        score = report.get('score', {})
        for key in ('wirelength_score', 'congestion_score', 'thermal_score', 'quality'):
            if not base.finite(score.get(key)):
                raise ValueError(name + ': nonfinite diagnostic ' + key)
        diag = {
            'W': score['wirelength_score'], 'C': score['congestion_score'],
            'T': score['thermal_score'], 'S': score['quality'],
            'overflow': report.get('congestion', {}).get('total_overflow'),
            'rho': report.get('thermal_tsv', {}).get('correlation_above_ambient'),
            'version': audit.get('version'), 'official_equivalence': False,
            'production_candidate_acceptance_supported': False,
            'seconds': record['steps']['diagnostic']['seconds'],
        }
        if base.sha(guide) != record['guide_sha256'] or base.sha(folder / 'stats.json') != record['stats_sha256']:
            raise ValueError(name + ': Guide or stats changed during diagnostic scoring')
        record['row']['diagnostic'] = diag
        self.update(record, status='COMPLETE', active_step=None, diagnostic_status='COMPLETE',
                    diagnostic_report=report, diagnostic_audit=audit)
        print(name, 'DIAGNOSTIC_COMPLETE', diag['S'], flush=True)


def summarize(rounds, args):
    rows = [row for result in rounds for row in result.get('rows', [])]
    result = {
        'selection_model': 'documented-rules-G20',
        'compatibility_used_for_selection': False,
        'official_equivalence': False,
        'protocol': args.protocol, 'independent_serial_batches': len(rounds),
        'external_budget_seconds': args.budget,
        'internal_time_limit_seconds': args.budget - args.budget_reserve,
        'all_rounds_complete': len(rounds) == args.repeats and all(r['status'] == 'COMPLETE' for r in rounds),
        'rows': rows, 'per_case': [], 'candidates': {},
    }
    for case in args.cases:
        for variant in args.variants:
            sample = [r for r in rows if r['case'] == case and r['variant'] == variant]
            if not sample:
                continue
            summary = {'case': case, 'variant': variant, 'count': len(sample)}
            for key in ('W', 'C', 'T', 'S', 'Q', 'rho', 'route_seconds', 'geometry_unresolved'):
                values = [r[key] for r in sample if base.finite(r.get(key))]
                if len(values) == len(sample):
                    summary[key] = sum(values) / len(values)
                    summary[key + '_range'] = [min(values), max(values)]
            d = [r['diagnostic'] for r in sample if 'diagnostic' in r]
            if len(d) == len(sample):
                summary['diagnostic'] = {
                    key: sum(r[key] for r in d) / len(d)
                    for key in ('W', 'C', 'T', 'S', 'overflow')
                    if all(base.finite(r.get(key)) for r in d)}
            result['per_case'].append(summary)
    by = {(r['repeat'], r['case'], r['variant']): r for r in rows}
    for variant in args.variants:
        if variant == 'BASE':
            continue
        pairs = []
        for repeat in range(1, args.repeats + 1):
            for case in args.cases:
                a, b = by.get((repeat, case, 'BASE')), by.get((repeat, case, variant))
                if a is None or b is None:
                    continue
                delta = {key: b[key] - a[key] for key in ('W', 'C', 'T', 'S', 'Q', 'rho', 'route_seconds')}
                delta.update(repeat=repeat, case=case,
                             geometry_unresolved_delta=b['geometry_unresolved'] - a['geometry_unresolved'])
                if 'diagnostic' in a and 'diagnostic' in b:
                    delta['diagnostic'] = {key: b['diagnostic'][key] - a['diagnostic'][key]
                                           for key in ('W', 'C', 'T', 'S')}
                pairs.append(delta)
        expected = args.repeats * len(args.cases)
        mean = sum(p['S'] for p in pairs) / len(pairs) if pairs else None
        case_delta = {case: sum(p['S'] for p in pairs if p['case'] == case) / args.repeats
                      for case in args.cases if sum(p['case'] == case for p in pairs) == args.repeats}
        repeat_delta = {str(n): sum(p['S'] for p in pairs if p['repeat'] == n) / len(args.cases)
                        for n in range(1, args.repeats + 1)
                        if sum(p['repeat'] == n for p in pairs) == len(args.cases)}
        numeric_pass = (len(pairs) == expected and mean >= args.minimum_mean_gain and
                        all(v > 0 for v in repeat_delta.values()) and
                        all(v >= -args.maximum_case_regression for v in case_delta.values()))
        result['candidates'][variant] = {
            'pairs': pairs, 'documented_mean_delta_S': mean,
            'documented_case_delta_S': case_delta,
            'documented_repeat_delta_S': repeat_delta,
            'numeric_engineering_threshold_passed': numeric_pass,
            'adoption_eligible': (args.protocol == 'confirmation' and result['all_rounds_complete'] and numeric_pass
                                  and not any(p['geometry_unresolved_delta'] > 0 for p in pairs)),
            'geometry_unresolved_review_required': any(p['geometry_unresolved_delta'] > 0 for p in pairs),
            'selection_note': 'No diagnostic score enters this decision. Unresolved geometry needs separate review.',
        }
    return result


def main(argv=None):
    args = parse_args(argv)
    if os.uname().nodename != 'benchmark-host':
        raise ValueError('Execute experiments only on benchmark-host')
    root = args.output_dir.resolve()
    if root.exists():
        raise ValueError('Output root must be new; existing evidence is never overwritten')
    root.mkdir(parents=True)
    original_variants = list(args.variants)
    declaration = {
        'started_utc': base.utc(), 'arguments': list(sys.argv[1:] if argv is None else argv),
        'external_budget_seconds': args.budget,
        'internal_time_limit_seconds': args.budget - args.budget_reserve,
        'repeats': args.repeats, 'workers_per_batch': args.workers,
        'separate_batches_not_independent_hardware': True,
        'paired_cases': args.cases, 'variants': args.variants,
        'low_layer_seconds': args.low_layer_seconds, 'low_layer_max_attempts': args.low_layer_max_attempts,
        'selection_model': 'documented-rules-G20', 'compatibility_used_for_selection': False,
        'minimum_mean_gain': args.minimum_mean_gain,
        'maximum_case_regression': args.maximum_case_regression,
        'all_paired_batch_mean_gains_must_be_positive': True,
        'e1_evidence': str(args.e1_evidence.resolve()) if args.e1_evidence else None,
        'e1_evidence_sha256': base.sha(args.e1_evidence) if args.e1_evidence else None,
    }
    dump(root / 'declaration.json', declaration)
    rounds = []
    cohort_reference = None
    failures = False
    for repeat in range(1, args.repeats + 1):
        # Reverse start ordering in alternate batches; all configurations fit the
        # confirmation worker pool, and no evaluation overlaps timed routing.
        args.variants = original_variants if repeat % 2 else list(reversed(original_variants))
        args.output_dir = root / ('round-' + str(repeat))
        experiment = Experiment(args, repeat)
        cohort = (experiment.inputs_before, experiment.sources_before, experiment.programs_before, experiment.runtime_before)
        if cohort_reference is None:
            cohort_reference = cohort
        elif cohort != cohort_reference:
            raise ValueError('Inputs, programs, runtime or source changed between independent batches')
        rc = experiment.run()
        rounds.append(base.read_json(args.output_dir / 'summary.json'))
        failures = failures or rc != 0
        args.variants = original_variants
        current = summarize(rounds, args)
        current['finished_utc'] = base.utc()
        dump(root / 'summary.json', current)
        if rc:
            break
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
