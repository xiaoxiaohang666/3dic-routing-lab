#!/usr/bin/env python3
"""E0: summarize frozen A150 evidence without routing or changing any score.

Only the Python standard library is required. --stats CASE=PATH adds a runtime
observation; it never replaces the frozen baseline or establishes input identity.
Outputs are always written into a new directory outside the evidence tree.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any

CONTRACT = 'a150-20261029-pin-floor-tsv-tracks'
FORMULA_STATUS = 'UNCONFIRMED_20261029_CONGESTION_FORMULA'
Q_FORMULA = 'sum((0.1*max(fixed+route-capacity,0)/capacity)^2); occupied zero-capacity cell contributes 10000'
LIMITATIONS = [
    'This is a read-only summary of supplied reports, not an independent Guide validation or an official score.',
    'PIN floor granularity and overlap combination, prohibition scope, and boundary rules remain recorded assumptions.',
    'Q/C and G20 thermal scores are proxies; the enhanced 20261029 congestion formula and official G_case remain unconfirmed.',
    'Added Q is Q(fixed+route)-Q(fixed), not Q(route); fixed demand cannot be removed by rerouting.',
    'The fixed-only C bound assumes the same inputs, capacity, fixed demand and old 0.1 proxy; it ignores compulsory route demand.',
    'The legal thermal upper bound ignores connectivity, both-die access, length and resources; it is not an achievable target.',
    'Length/Q rejection counters are overlapping associations among rejected trials, not mutually exclusive causal categories.',
    'Nested, attributed and alias timings must not be summed into process wall time. Queue visits are not unique nets.',
    'Optional stats retain their own provenance; matching profile/counts does not prove the same inputs or validation success.',
]


def number(value: Any, name: str, *, nonnegative: bool = True, integer: bool = False) -> float | int:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name}: expected a finite number')
    if nonnegative and value < 0:
        raise ValueError(f'{name}: expected a nonnegative number')
    if integer and int(value) != value:
        raise ValueError(f'{name}: expected an integer')
    return int(value) if integer else value


def get_path(obj: dict, path: str) -> Any:
    value: Any = obj
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def close(a: Any, b: Any, name: str) -> None:
    number(a, name, nonnegative=False)
    number(b, name, nonnegative=False)
    if not math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-8):
        raise ValueError(f'{name}: conflicting reports ({a} versus {b})')


def added(total: Any, fixed: Any, name: str) -> float | int:
    number(total, name + '.total')
    number(fixed, name + '.fixed')
    delta = total - fixed
    if delta < -1e-8:
        raise ValueError(f'{name}: total smaller than fixed lower bound')
    return max(0, delta)


def ratio(numerator: Any, denominator: Any, numerator_scope: str, denominator_scope: str) -> dict:
    result = {'numerator': numerator, 'denominator': denominator,
              'numerator_scope': numerator_scope, 'denominator_scope': denominator_scope,
              'fraction': None, 'status': 'UNAVAILABLE'}
    if numerator is None or denominator is None:
        return result
    n = number(numerator, numerator_scope, integer=True)
    d = number(denominator, denominator_scope, integer=True)
    if n > d:
        raise ValueError(f'{numerator_scope}: unique count exceeds denominator')
    result.update(fraction=n / d if d else None, status='AVAILABLE' if d else 'EMPTY_POPULATION')
    return result


def rejection_overlap(trials: Any, accepted: Any, length: Any, q: Any, both: Any = None) -> dict:
    result = {'positive_trials': trials, 'retained_positive_acceptances': accepted,
              'length_increased_rejections': length, 'Q_increased_rejections': q,
              'intersection_count': both, 'intersection_bounds': None, 'union_bounds': None,
              'categories_are_disjoint': False, 'is_causal_attribution': False,
              'scope': 'positive-rho trial events in the thermal queue; not distinct networks',
              'residual_is_rejection_count': False}
    if any(x is None for x in (trials, accepted, length, q)):
        result['status'] = 'INSUFFICIENT_COUNTERS'
        return result
    t, a, l, c = [number(v, n, integer=True) for v, n in
                  zip((trials, accepted, length, q), ('trials', 'accepted', 'length', 'Q'))]
    pool = t - a
    if pool < 0 or max(l, c) > pool:
        raise ValueError('rejection counters exceed non-retained positive trial population')
    lower, upper = max(0, l + c - pool), min(l, c)
    if both is not None:
        both = number(both, 'rejection intersection', integer=True)
        if not lower <= both <= upper:
            raise ValueError('rejection intersection violates marginal bounds')
        lower = upper = both
    result.update(status='EXACT_INTERSECTION' if both is not None else 'BOUNDED_INTERSECTION',
                  non_retained_positive_trial_population=pool,
                  intersection_bounds=[lower, upper], union_bounds=[l + c - upper, l + c - lower],
                  residual_scope='Includes failed/timed-out/rolled-back trials; not all are score rejections.')
    return result


def summarize_runtime(stats: dict, cross_nets: int, route_wall: Any = None) -> dict:
    """Keep unique-net coverage, candidate events and timing scopes separate."""
    k = number(cross_nets, 'cross_nets', integer=True)
    v11 = stats.get('diagnostics_v11', {})
    thermal = v11.get('thermal_queue', {})
    def value(name: str, fallback: str) -> Any:
        return thermal[name] if name in thermal else get_path(stats, fallback)
    queried = value('queried_unique', 'resource_v6.thermal_query_unique')
    positive = value('positive_unique', 'resource_v6.thermal_positive_unique')
    rerouted = value('rerouted_unique', 'resource_v6.thermal_rerouted_unique')
    eligible = thermal.get('eligible_cross_nets')
    if eligible is not None and number(eligible, 'eligible_cross_nets', integer=True) > k:
        raise ValueError('thermal eligible population exceeds all cross nets')
    population = 'all cross-die nets, one TSV per net; not thermal queue size'
    coverage = {
        'all_probe_unique_over_cross_nets': ratio(get_path(stats, 'counters.v9_tsv_queried_unique'), k,
            'unique nets receiving the cheap all-net TSV probe', population),
        'all_rerouted_unique_over_cross_nets': ratio(get_path(stats, 'counters.v9_tsv_rerouted_unique'), k,
            'unique nets entering at least one actual TSV route trial across all queues', population),
        'all_moved_unique_over_cross_nets': ratio(get_path(stats, 'counters.v9_tsv_moved_unique'), k,
            'unique nets with a retained accepted move across all queues', population),
        'thermal_queried_over_cross_nets': ratio(queried, k,
            'unique nets selected for a full thermal-queue candidate query', population),
        'thermal_positive_over_queried': ratio(positive, queried,
            'unique queried nets with at least one positive-rho candidate', 'unique full thermal-queue queried nets'),
        'thermal_rerouted_over_queried': ratio(rerouted, queried,
            'unique thermal-queue nets entering an actual trial (not necessarily positive-rho)',
            'unique full thermal-queue queried nets'),
        'thermal_queried_over_eligible': ratio(queried, eligible,
            'unique full thermal-queue queried nets', 'explicitly recorded thermal-eligible cross-net population'),
    }
    rejected = rejection_overlap(value('positive_trials', 'resource_v7.thermal_positive_trials'),
        value('positive_accepted', 'resource_v7.thermal_positive_accepted'),
        value('rejected_length', 'resource_v7.thermal_rejected_length'),
        value('rejected_q', 'resource_v7.thermal_rejected_q'), thermal.get('rejected_length_and_q'))
    # These are measured scopes, not a partition. In particular v8/v7 RRR are aliases.
    timing_specs = [
        ('elapsed_seconds', 'router-reported elapsed time; measurement origin comes from options.budget_origin', None),
        ('counters.v7_topology_seconds', 'accumulated topology work', 'router'),
        ('counters.v7_rrr_seconds', 'RRR inclusive phase time; v8_rrr_seconds may alias this value', 'router'),
        ('counters.v8_rrr_seconds', 'RRR alias; do not add to v7_rrr_seconds', 'counters.v7_rrr_seconds'),
        ('counters.v7_validation_seconds', 'full RRR validation time nested in RRR', 'counters.v7_rrr_seconds'),
        ('budgeted_wirelength_recovery.seconds', 'wirelength recovery inclusive elapsed time', 'router'),
        ('counters.v8_rematch_seconds', 'TSV rematch inclusive elapsed time', 'router'),
        ('counters.v7_tsv_seconds', 'dynamic TSV phase inclusive elapsed time including validation', 'router'),
        ('counters.v8_tsv_wirelength_seconds', 'wirelength queue attributed work, including allocated phase overhead', 'counters.v7_tsv_seconds'),
        ('counters.v8_tsv_thermal_seconds', 'thermal queue attributed work, including allocated phase overhead', 'counters.v7_tsv_seconds'),
        ('counters.v8_tsv_congestion_seconds', 'congestion queue attributed work, including allocated phase overhead', 'counters.v7_tsv_seconds'),
        ('counters.v9_tsv_explore_seconds', 'exploration attributed work, including allocated phase overhead', 'counters.v7_tsv_seconds'),
        ('resource_v6.thermal_query_seconds', 'thermal full query + regional query elapsed; already inside queue work', 'counters.v8_tsv_thermal_seconds'),
        ('resource_v6.thermal_route_seconds', 'thermal trial loop elapsed including validation/rollback; already inside queue work', 'counters.v8_tsv_thermal_seconds'),
        ('resource_v7.tsv_final_validation_seconds', 'final dynamic-TSV validation/checkpoint work; also attributed to queues', 'counters.v7_tsv_seconds'),
        ('resource_v7.index_seconds', 'candidate index construction work; may overlap phase/queue scopes', 'router'),
        ('counters.v8_tsv_probe_seconds', 'cheap candidate probe work; also attributed to queues', 'counters.v7_tsv_seconds'),
        ('counters.v9_tsv_refresh_seconds', 'proposal refresh bookkeeping; also attributed to queues', 'counters.v7_tsv_seconds'),
    ]
    timings = []
    for field, scope, parent in timing_specs:
        seconds = get_path(stats, field)
        if seconds is not None:
            timings.append({'field': field, 'seconds': number(seconds, field), 'scope': scope,
                            'parent_scope': parent, 'additive_with_other_rows': False})
    if route_wall is not None:
        timings.insert(0, {'field': 'baseline.summary.rows.route_seconds', 'seconds': number(route_wall, 'route_wall'),
            'scope': 'harness-measured external router process wall time; excludes later validators/scorers',
            'parent_scope': None, 'additive_with_other_rows': False})
    for item in v11.get('timings', []):
        if not isinstance(item, dict) or not item.get('name') or not item.get('scope'):
            raise ValueError('diagnostics_v11.timings require name, seconds and scope')
        timings.append({'field': 'diagnostics_v11.' + item['name'], 'seconds': number(item['seconds'], item['name']),
            'scope': item['scope'], 'parent_scope': item.get('parent'),
            'overlaps_with': item.get('overlaps_with'), 'additive_with_other_rows': False})
    stages = {}
    for path in ('initial_metrics', 'pre_recovery_metrics', 'budgeted_wirelength_recovery.before',
                 'budgeted_wirelength_recovery.after', 'pre_tsv_local_metrics', 'metrics'):
        item = get_path(stats, path)
        if isinstance(item, dict):
            stages[path] = {key: item[key] for key in ('guide_overflow', 'squared_normalized_overflow',
                'guide_mst_dbu', 'reference_dbu', 'correlation', 'zero_capacity_cells') if key in item}
    return {'coverage': coverage, 'thermal_rejections': rejected, 'timings': timings,
            'timings_are_additive': False, 'budget_origin': get_path(stats, 'options.budget_origin'),
            'internal_time_limit_seconds': get_path(stats, 'options.time_limit_seconds'),
            'stage_snapshots': stages, 'stage_deltas_are_isolated_operator_effects': False,
            'assignment_rho_before': get_path(stats, 'policy.assignment_polish_correlation_before'),
            'assignment_rho_after': get_path(stats, 'policy.assignment_polish_correlation_after'),
            'queue_visit_events': {key: get_path(stats, 'counters.' + key) for key in
                ('v8_tsv_wirelength_visits', 'v8_tsv_thermal_visits', 'v8_tsv_congestion_visits', 'v9_tsv_explore_visits')},
            'queue_visit_events_are_unique_net_counts': False}


class EvidenceReader:
    def __init__(self) -> None:
        self.hashes: dict[str, str] = {}

    def read(self, path: Path) -> dict:
        path = path.resolve(strict=True)
        data = path.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if str(path) in self.hashes and self.hashes[str(path)] != digest:
            raise ValueError(f'evidence changed while reading: {path}')
        self.hashes[str(path)] = digest
        result = json.loads(data)
        if not isinstance(result, dict):
            raise ValueError(f'expected JSON object: {path}')
        return result

    def verify(self) -> None:
        for path, digest in self.hashes.items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
                raise ValueError(f'evidence changed during summary: {path}')


def summarize_resources(metrics: dict, fixed: dict) -> dict:
    totals = fixed['totals']
    f_over = number(totals['new_fixed_only_overflow'], 'fixed overflow', integer=True)
    f_q = number(totals['new_fixed_only_Q_0p1_proxy'], 'fixed Q')
    total_over = number(metrics['overflow'], 'total overflow', integer=True)
    total_q = number(metrics['Q'], 'total Q')
    current_c, bound_c = 100 / math.sqrt(1 + total_q), 100 / math.sqrt(1 + f_q)
    close(current_c, metrics['C'], 'old proxy C')
    return {'fixed_overflow': f_over, 'total_overflow': total_over,
            'route_added_overflow': added(total_over, f_over, 'overflow'),
            'fixed_Q_0p1_proxy': f_q, 'total_Q_0p1_proxy': total_q,
            'route_added_Q_0p1_proxy': added(total_q, f_q, 'Q'),
            'fixed_fraction_of_total_overflow': f_over / total_over if total_over else None,
            'fixed_fraction_of_total_Q': f_q / total_q if total_q else None,
            'pin_floor_demand': totals['added_pin_floor'],
            'raw_fixed_track_union': totals['raw_fixed_track_union'],
            'new_fixed_total': totals['new_fixed_total'],
            'fixed_only_C_upper_bound_old_proxy': bound_c,
            'current_C_old_proxy': current_c,
            'C_only_weighted_S_gain_upper_bound_old_proxy': .30 * (bound_c - current_c),
            'bound_conditions': 'Identical inputs/capacity/fixed PIN floor; old Q scale 0.1 and C=100/sqrt(1+Q); nonnegative route demand. Ignores compulsory routing. Not official or attainable.',
            'added_Q_semantics': 'Q(F+R)-Q(F), including interactions; not Q(R)'}


def summarize_case(case: str, root: Path, baseline: dict, reader: EvidenceReader, extra: Path | None) -> dict:
    rows = [r for r in baseline['rows'] if r.get('case') == case]
    if len(rows) != 1:
        raise ValueError(f'{case}: expected exactly one frozen baseline row')
    row = rows[0]
    folder = root / 'baseline-all-layers-01' / row['name']
    documented = reader.read(folder / 'documented.json')
    stats_path = folder / 'stats-compact.json'
    if not stats_path.exists():
        stats_path = folder / 'stats.json'
    stats = reader.read(stats_path)
    fixed = reader.read(root / 'diagnostics-pin-floor-01' / f'{case}.json')
    thermal = reader.read(root / 'diagnostics-legal-thermal-01' / f'{case}.json')
    for doc, key in ((documented, 'model'), (stats, 'rules_contract')):
        if doc.get(key) != CONTRACT or doc.get('congestion_formula_status') != FORMULA_STATUS:
            raise ValueError(f'{case}: incompatible frozen profile or congestion status')
        if doc.get('congestion_overflow_scale_proxy') != .1:
            raise ValueError(f'{case}: incompatible proxy scale')
        floor = doc.get('pin_zero_track_floor', {})
        if floor.get('enabled') is not True or floor.get('demand') != fixed['totals']['added_pin_floor']:
            raise ValueError(f'{case}: PIN floor demand mismatch')
    if fixed.get('status') != 'COMPLETE' or thermal.get('status') != 'COMPLETE':
        raise ValueError(f'{case}: incomplete frozen diagnostics')
    if fixed.get('official_congestion_formula_status') != FORMULA_STATUS:
        raise ValueError(f'{case}: fixed-resource proxy status mismatch')
    metrics = documented['metrics']
    for key in ('overflow', 'Q', 'rho', 'L', 'C'):
        close(metrics[key], row[key], f'{case} baseline {key}')
    for key, stat_key in (('overflow', 'guide_overflow'), ('Q', 'squared_normalized_overflow'),
                          ('rho', 'correlation'), ('L', 'guide_mst_dbu')):
        close(metrics[key], stats['metrics'][stat_key], f'{case} stats {key}')
    k = number(thermal['K_cross_nets'], 'K', integer=True)
    close(k, stats['metrics']['evaluator_tsv_count'], f'{case} TSV population')
    n = number(thermal['N_full_XY_cells'], 'N', integer=True)
    if not 0 < k < n:
        raise ValueError(f'{case}: thermal relaxation needs 0 < K < N')
    rho = number(metrics['rho'], 'rho', nonnegative=False)
    upper = number(thermal['legal']['rho_upper_bound'], 'legal rho bound', nonnegative=False)
    if not -1 <= rho <= upper + 1e-10 or upper > 1:
        raise ValueError(f'{case}: rho outside recorded legal relaxation')
    resources = summarize_resources(metrics, fixed)
    fixed_layers = {int(re.sub(r'\D', '', str(x['layer']))): x['totals'] for x in fixed.get('layers', [])}
    layers = []
    for layer in documented.get('layers', []):
        f = fixed_layers.get(int(layer['layer']))
        if f is None:
            continue
        layers.append({'layer': layer['layer'], 'fixed_overflow': f['new_fixed_only_overflow'],
            'total_overflow': layer['overflow'],
            'route_added_overflow': added(layer['overflow'], f['new_fixed_only_overflow'], 'layer overflow'),
            'fixed_Q_0p1_proxy': f['new_fixed_only_Q_0p1_proxy'], 'total_Q_0p1_proxy': layer['Q'],
            'route_added_Q_0p1_proxy': added(layer['Q'], f['new_fixed_only_Q_0p1_proxy'], 'layer Q')})
    result = {'case': case, 'frozen_baseline': {
        'resources': resources, 'layers': layers,
        'thermal': {'rho': rho, 'legal_rho_upper_bound': upper, 'rho_relaxation_gap': upper - rho,
            'unrestricted_rho_upper_bound': thermal['unrestricted_full_grid']['rho_upper_bound'],
            'N_full_XY_cells': n, 'K_cross_nets': k, 'legal_all_layer_candidates': thermal['legal_all_layer_candidates'],
            'pearson_denominator': thermal['pearson_denominator'],
            'one_move_delta_rho': '(heat[new]-heat[old])/pearson_denominator, only for fixed K and binary occupancy',
            'same_site_set_owner_swap_changes_rho': False,
            'bound_scope': thermal['scope'], 'is_achievable_target': False},
        'runtime': summarize_runtime(stats, k, row.get('route_seconds')),
        'geometry_unresolved': row.get('geometry_unresolved'),
        'rules': {'contract': stats['rules_contract'], 'a150': stats.get('a150'),
                  'pin_zero_track_floor': stats['pin_zero_track_floor'], 'G_proxy': stats.get('thermal_gain')}}}
    if extra is not None:
        new = reader.read(extra)
        new_k = number(get_path(new, 'metrics.evaluator_tsv_count'), 'optional stats TSV population', integer=True)
        result['optional_stats'] = {'path': str(extra.resolve()), 'runtime': summarize_runtime(new, new_k),
            'reported_metrics': {key: new.get('metrics', {}).get(key) for key in
                ('guide_overflow', 'squared_normalized_overflow', 'guide_mst_dbu', 'correlation', 'zero_capacity_cells')},
            'reported_rules_contract': new.get('rules_contract'),
            'reported_congestion_formula_status': new.get('congestion_formula_status'),
            'same_input_identity_verified': False, 'serialized_validation_verified': False,
            'formal_adoption_allowed': False,
            'note': 'Runtime observation only. Frozen resource/thermal bounds are not applied to these unverified inputs.'}
    return result


def build_summary(root: Path, cases: list[str], stats: dict[str, Path] | None = None) -> dict:
    root = root.resolve(strict=True)
    reader = EvidenceReader()
    baseline = reader.read(root / 'baseline-all-layers-01/summary.json')
    if baseline.get('status') != 'COMPLETE' or baseline.get('rules_profile') != 'a150-20261029':
        raise ValueError('expected a COMPLETE a150-20261029 frozen baseline')
    if baseline.get('congestion_formula_status') != FORMULA_STATUS:
        raise ValueError('frozen baseline does not declare the unconfirmed congestion formula')
    results = [summarize_case(case, root, baseline, reader, (stats or {}).get(case)) for case in cases]
    python_path = root / 'python-baseline-01/summary.json'
    verification = None
    if python_path.exists():
        python = reader.read(python_path)
        verification = {key: python.get(key) for key in ('status', 'resource_and_legality_match',
            'full_metrics_match', 'official_score_alignment', 'reference_only_metric_differences', 'integrity_unchanged')}
    reader.verify()
    return {'schema_version': 1, 'status': 'DIAGNOSTIC_SUMMARY_COMPLETE', 'evidence_root': str(root),
            'official_equivalence': False, 'official_score_alignment_confirmed': False,
            'formal_adoption_allowed': False, 'proxy_S_is_formal_benefit_gate': False,
            'rule_profile': 'a150-20261029', 'congestion_formula_status': FORMULA_STATUS,
            'Q_formula_proxy': Q_FORMULA, 'thermal_gain_status': 'UNCONFIRMED_UNIFORM_G20_PROXY',
            'baseline_reported_integrity': baseline.get('integrity'),
            'baseline_reported_assumptions': baseline.get('assumptions'),
            'independent_python_reported_verification': verification,
            'cases': results, 'limitations': LIMITATIONS,
            'source_sha256': reader.hashes, 'source_bytes_unchanged_during_summary': True}


def render_markdown(report: dict) -> str:
    lines = ['# A150 v11 E0 diagnostics', '',
        'Read-only evidence summary. Official equivalence: **false**. Formal adoption: **not allowed by this report**.',
        'Q/C/S use the old 0.1 proxy. No score or route was recomputed.', '',
        '| Case | Fixed / total overflow | Added overflow | Fixed / total Q proxy | Added Q proxy | Fixed-only C upper bound (old proxy) | rho / legal relaxed upper bound |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for case in report['cases']:
        b = case['frozen_baseline']; r = b['resources']; t = b['thermal']
        lines.append(f"| {case['case']} | {r['fixed_overflow']} / {r['total_overflow']} | {r['route_added_overflow']} | {r['fixed_Q_0p1_proxy']:.9g} / {r['total_Q_0p1_proxy']:.9g} | {r['route_added_Q_0p1_proxy']:.9g} | {r['fixed_only_C_upper_bound_old_proxy']:.9g} | {t['rho']:.9g} / {t['legal_rho_upper_bound']:.9g} |")
    lines += ['', 'C bounds assume unchanged fixed demand/capacity and the old formula; they ignore mandatory route demand. Thermal bounds ignore access/connectivity/resources and are not achievable targets.', '']
    for case in report['cases']:
        observations = [('frozen baseline', case['frozen_baseline']['runtime'])]
        if 'optional_stats' in case:
            observations.append(('optional stats, input identity and validation unverified', case['optional_stats']['runtime']))
        for label, runtime in observations:
            lines += [f"## {case['case']} — {label}", '', '| Coverage | Numerator / denominator | Fraction | Denominator meaning |', '|---|---:|---:|---|']
            for name, cov in runtime['coverage'].items():
                fraction = 'unavailable' if cov['fraction'] is None else f"{100 * cov['fraction']:.3f}%"
                lines.append(f"| {name} | {cov['numerator']} / {cov['denominator']} | {fraction} | {cov['denominator_scope']} |")
            rej = runtime['thermal_rejections']
            lines += ['', f"Positive-rho trial events: {rej['positive_trials']}; retained acceptances: {rej['retained_positive_acceptances']}; length-increase rejections: {rej['length_increased_rejections']}; Q-increase rejections: {rej['Q_increased_rejections']}.",
                f"Length/Q intersection bounds: {rej['intersection_bounds']}; union bounds: {rej['union_bounds']}. Marginals overlap and are not causal categories; non-retained trials include failures/timeouts/rollbacks.", '',
                '| Time field | Seconds | Scope (rows are not additive) |', '|---|---:|---|']
            for timing in runtime['timings']:
                lines.append(f"| {timing['field']} | {timing['seconds']:.6f} | {timing['scope']} |")
            lines += ['', 'Stage raw-metric snapshots and per-layer fixed/added quantities are retained in summary.json; snapshots do not isolate operator effects.', '']
    lines += ['## Evidence limits', ''] + ['- ' + item for item in report['limitations']]
    verification = report['independent_python_reported_verification']
    lines += ['', 'Independent Python recorded state: `' + json.dumps(verification, ensure_ascii=False) + '`.', '',
              'All consumed JSON files have SHA256 records in summary.json; byte hashes were rechecked before writing.']
    return '\n'.join(lines) + '\n'


def write_outputs(report: dict, output: Path) -> None:
    output = output.resolve()
    evidence = Path(report['evidence_root']).resolve()
    if output == evidence or evidence in output.parents:
        raise ValueError('output directory must be outside the frozen evidence tree')
    if output.exists():
        raise FileExistsError(f'refusing to overwrite existing output directory: {output}')
    if output.parent == Path(output.anchor):
        raise ValueError('refusing to create a filesystem-root output directory')
    payload = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    markdown = render_markdown(report)
    output.mkdir(parents=True, exist_ok=False)
    with (output / 'summary.json').open('x', encoding='utf-8') as handle:
        handle.write(payload)
    with (output / 'summary.md').open('x', encoding='utf-8') as handle:
        handle.write(markdown)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--cases', default='001,003,005', help='Comma-separated case identifiers')
    parser.add_argument('--stats', action='append', default=[], metavar='CASE=PATH',
                        help='Optional additional stats; repeat once per selected case')
    args = parser.parse_args(argv)
    args.cases = args.cases.split(',')
    if not args.cases or len(set(args.cases)) != len(args.cases) or any(not re.fullmatch(r'[A-Za-z0-9_-]+', c) for c in args.cases):
        parser.error('--cases must contain unique simple identifiers')
    paths = {}
    for entry in args.stats:
        case, sep, path = entry.partition('=')
        if not sep or not path or case not in args.cases or case in paths:
            parser.error('--stats must be a unique selected CASE=PATH')
        paths[case] = Path(path)
    args.stats = paths
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        # Fail before reading large evidence if the destination is already used.
        if args.output_dir.exists():
            raise FileExistsError(f'refusing to overwrite existing output directory: {args.output_dir}')
        report = build_summary(args.evidence_root, args.cases, args.stats)
        write_outputs(report, args.output_dir)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'E0 diagnostic summary failed: {error}', file=sys.stderr)
        return 2
    print(str(args.output_dir.resolve() / 'summary.json'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
