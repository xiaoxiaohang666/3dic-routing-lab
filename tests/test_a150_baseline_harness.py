"""Baseline gate regressions; no real route or scoring process is launched."""
from pathlib import Path
import copy
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import run_a150_baseline as harness
import run_a150_subtree_ablation as subtree_harness


def supply(scope='all-layers', cross=3, interface=10, all_layers=5):
    selected = all_layers if scope == 'all-layers' else interface
    return {'rules_profile': 'a150', 'supply_only': True,
            'status': 'PASS' if selected >= cross else 'FAIL', 'a150': {
        'scope': harness.SCOPE_NAMES[scope], 'cross_die_nets': cross,
        'interface_eligible_cells': interface, 'all_layers_eligible_cells': all_layers,
        'selected_eligible_cells': selected, 'selected_scope_supply_sufficient': selected >= cross,
        'fixed_interface_violations': 0, 'fixed_all_layers_violations': 0,
    }}


class A150BaselineHarnessTests(unittest.TestCase):
    def test_default_budget_and_scope(self):
        args = harness.parse_args(['--output-dir', '/unused/new-results'])
        self.assertEqual(args.tsv_obstacle_scope, 'all-layers')
        self.assertEqual(args.budget - args.budget_reserve, 1740)
        self.assertEqual(args.cases, ['001', '003', '005'])
        self.assertEqual(args.variants, ['BASE'])

    def test_supply_rejects_inconsistent_eligibility_and_boolean_counts(self):
        report = supply()
        report['a150']['selected_eligible_cells'] = 10
        with self.assertRaisesRegex(ValueError, 'inconsistent supply'):
            harness.supply_contract(report, 'all-layers')
        report = supply()
        report['a150']['cross_die_nets'] = True
        with self.assertRaisesRegex(ValueError, 'nonnegative integer'):
            harness.supply_contract(report, 'all-layers')

    def test_insufficient_supply_and_fixed_pin_conflict_are_explicit(self):
        result = harness.supply_contract(supply(cross=6), 'all-layers')
        self.assertFalse(result['selected_scope_supply_sufficient'])
        report = supply()
        report['a150']['fixed_all_layers_violations'] = 1
        report['status'] = 'FAIL'
        result = harness.supply_contract(report, 'all-layers')
        self.assertEqual(result['selected_scope_fixed_conflicts'], 1)

    def test_engineering_pass_cannot_hide_mask_violation(self):
        audited = harness.supply_contract(supply(), 'all-layers')
        report = supply()
        report['status'] = 'PASS'
        report['a150'].update(interface_violations=0, all_layers_violations=1)
        with self.assertRaisesRegex(ValueError, 'forbidden PIN/OBS'):
            harness.legal_contract(report, 'all-layers', audited)
        report['a150']['all_layers_violations'] = 0
        self.assertEqual(harness.legal_contract(report, 'all-layers', audited)['selected_scope_violations'], 0)

    def test_scope_and_source_supply_must_match_readback(self):
        audited = harness.supply_contract(supply(), 'all-layers')
        report = supply()
        report['status'] = 'PASS'
        report['a150'].update(interface_violations=0, all_layers_violations=0)
        report['a150']['all_layers_eligible_cells'] = 6
        with self.assertRaisesRegex(ValueError, 'differs from preflight'):
            harness.legal_contract(report, 'all-layers', audited)
        with self.assertRaisesRegex(ValueError, 'different obstacle scope'):
            harness.supply_contract(supply('interface'), 'all-layers')

    def test_supply_rejects_unexplained_failure_and_wrong_profile(self):
        report = supply()
        report['status'] = 'FAIL'
        with self.assertRaisesRegex(ValueError, 'status differs'):
            harness.supply_contract(report, 'all-layers')
        report = supply(cross=6)
        report['status'] = 'PASS'
        with self.assertRaisesRegex(ValueError, 'status differs'):
            harness.supply_contract(report, 'all-layers')
        report = supply()
        report['rules_profile'] = 'historical'
        with self.assertRaisesRegex(ValueError, 'preflight profile'):
            harness.supply_contract(report, 'all-layers')

    def test_recorded_route_and_readback_normalize_scope_names(self):
        stats = {'rules_contract': harness.A150_CONTRACT,
                 'a150': {'enabled': True, 'obstacle_scope': 'all-routing-layers-both-dies'}}
        harness.recorded_profile(stats, 'all-layers')
        readback = {'model': harness.A150_CONTRACT,
                    'a150': {'enabled': True, 'obstacle_scope': 'all-layers',
                             'interface_violations': 0, 'all_layer_violations': 0}}
        harness.recorded_profile(readback, 'all-layers', serialized=True)
        stats['a150']['enabled'] = False
        with self.assertRaisesRegex(ValueError, 'not enabled'):
            harness.recorded_profile(stats, 'all-layers')
        stats['a150']['enabled'] = True
        stats['rules_contract'] = 'documented-rules-tsv-landing-tracks-v1'
        with self.assertRaisesRegex(ValueError, 'resource contract'):
            harness.recorded_profile(stats, 'all-layers')
        readback['a150']['obstacle_scope'] = 'interface'
        with self.assertRaisesRegex(ValueError, 'different TSV obstacle scope'):
            harness.recorded_profile(readback, 'all-layers', serialized=True)
        readback['a150'].update(obstacle_scope='all-layers', all_layer_violations=1)
        with self.assertRaisesRegex(ValueError, 'forbidden TSV landings'):
            harness.recorded_profile(readback, 'all-layers', serialized=True)

    def test_interface_baseline_cannot_unlock_subtrees_with_all_layer_violations(self):
        rows = [{'selected_scope_violations': 0, 'all_layers_violations': n} for n in (0, 2, 0)]
        self.assertFalse(harness.qualifies_for_subtree(rows, True))
        rows[1]['all_layers_violations'] = 0
        self.assertTrue(harness.qualifies_for_subtree(rows, True))
        self.assertFalse(harness.qualifies_for_subtree(rows, False))
        self.assertFalse(harness.qualifies_for_subtree(rows[:2], True))

    def test_any_failed_supply_prevents_all_routes(self):
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(supply_only=False, workers=3, evaluation_workers=3)
        experiment.state = {'runs': {case: {'status': 'PENDING'} for case in harness.CASES}}
        phases = []

        def update(record, **values):
            record.update(values)

        def phase(function, names, workers):
            phases.append(function.__name__)
            for name in names:
                experiment.state['runs'][name]['status'] = 'SUPPLY_READY'
            experiment.state['runs']['003']['status'] = 'SUPPLY_INFEASIBLE'

        experiment.update, experiment.phase, experiment.finish = update, phase, lambda: 2
        self.assertEqual(experiment.run(), 2)
        self.assertEqual(phases, ['run_preflight'])
        self.assertEqual(experiment.state['runs']['001']['status'], 'SUPPLY_READY')


class A150SubtreeGateTests(unittest.TestCase):
    @staticmethod
    def documents():
        integrity = {key: True for key in subtree_harness.INTEGRITY_KEYS}
        rows, runs = [], {}
        for case in harness.CASES:
            row = {'case': case, 'variant': 'BASE', 'tsv_obstacle_scope': 'all-layers',
                   'interface_violations': 0, 'all_layers_violations': 0, 'selected_scope_violations': 0,
                   'W': 70., 'C': 80., 'T': 90., 'S': 78., 'route_seconds': 1790., 'guide_sha256': case[-1] * 64}
            audited = harness.supply_contract(supply(), 'all-layers')
            engineering = supply()
            engineering['a150'].update(interface_violations=0, all_layers_violations=0)
            documented = {'model': harness.A150_CONTRACT,
                          'a150': {'enabled': True, 'obstacle_scope': 'all-layers',
                                   'interface_violations': 0, 'all_layer_violations': 0}}
            rows.append(row)
            runs[case + '-BASE'] = {'case': case, 'variant': 'BASE', 'status': 'COMPLETE', 'row': row,
                                   'guide_sha256': row['guide_sha256'], 'supply': audited,
                                   'engineering_report': engineering, 'documented_report': documented,
                                   'geometry_report': {'summary': {'errors': 0}},
                                   'options': {'v9_subtree_recovery': False},
                                   'steps': {'route': {'command': ['router', '--v9-subtree-recovery', '0',
                                                                  '--low-layer-mask', '0']}}}
        sources = {'src/router.cpp': 'algorithm', 'tests/new_test.cpp': 'test-before'}
        sources.update({relative: relative + '-unchanged' for relative in subtree_harness.PRESETS})
        inputs = {'/data/input.def': 'frozen-input'}
        common = {'status': 'COMPLETE', 'rules_profile': 'a150', 'tsv_obstacle_scope': 'all-layers',
                  'official_equivalence': False, 'compatibility_used_for_selection': False,
                  'baseline_confirmed_under_recorded_rules': True, 'integrity': integrity, 'rows': rows}
        summary = dict(common, conservative_all_layer_baseline=True,
                       subtree_optimization_allowed_after_this_gate=True)
        status = dict(common, supply_only=False, runs=runs, source_files_before=sources, inputs_before=inputs)
        return copy.deepcopy(summary), copy.deepcopy(status), copy.deepcopy(sources), copy.deepcopy(inputs)

    def test_incomplete_or_forged_summary_cannot_unlock_ablation(self):
        for mutate in (
                lambda summary, status: summary.update(status='ROUTING'),
                lambda summary, status: summary.update(conservative_all_layer_baseline=False),
                lambda summary, status: status['integrity'].update(source_unchanged=1),
                lambda summary, status: summary['rows'][1].update(all_layers_violations=1),
                lambda summary, status: status['runs']['003-BASE']['documented_report']['a150'].update(all_layer_violations=1)):
            summary, status, sources, inputs = self.documents()
            mutate(summary, status)
            with self.assertRaises(ValueError):
                subtree_harness.validate_gate_documents(summary, status, sources, inputs)

    def test_algorithm_or_input_drift_blocks_but_new_tests_are_allowed(self):
        summary, status, sources, inputs = self.documents()
        sources['tests/new_test.cpp'] = 'new-regression-after-baseline'
        sources['scripts/run_a150_subtree_ablation.py'] = 'new-screening-harness'
        self.assertEqual(set(subtree_harness.validate_gate_documents(summary, status, sources, inputs)), set(harness.CASES))
        changed = dict(sources, **{'src/router.cpp': 'different-algorithm'})
        with self.assertRaisesRegex(ValueError, 'algorithm hashes differ'):
            subtree_harness.validate_gate_documents(summary, status, changed, inputs)
        with self.assertRaisesRegex(ValueError, 'input hashes differ'):
            subtree_harness.validate_gate_documents(summary, status, sources, {'/data/input.def': 'modified-input'})
        changed = dict(sources, **{subtree_harness.PRESETS[0]: 'changed-stage-preset'})
        with self.assertRaisesRegex(ValueError, 'preset changed'):
            subtree_harness.validate_gate_documents(summary, status, changed, inputs)

    def test_pair_contract_uses_effective_flags_and_excludes_execution_statistics(self):
        first = ['bash', '/preset.sh', '--time-limit', '1740', '--lef', '/tech.lef', '--lef', '/cell.lef',
                 '--output', '/BASE.guide', '--stats', '/BASE.json', '--v9-subtree-recovery', '0']
        second = first + ['--output', '/SUBTREE.guide', '--stats', '/SUBTREE.json', '--v9-subtree-recovery', '1']
        self.assertEqual(subtree_harness.normalized_route_command(first), subtree_harness.normalized_route_command(second))
        second += ['--time-limit', '1800']
        self.assertNotEqual(subtree_harness.normalized_route_command(first), subtree_harness.normalized_route_command(second))
        common = {'v9_subtree_branches': 4, 'v9_subtree_recovery': False, 'congestion_price_updates': 2,
                  'congestion_price_min': .1, 'congestion_price_max': 2.}
        derived_change = dict(common, v9_subtree_recovery=True, congestion_price_updates=20,
                              congestion_price_min=.2, congestion_price_max=3.)
        self.assertEqual(subtree_harness.normalized_stats_options(common), subtree_harness.normalized_stats_options(derived_change))
        derived_change['v9_subtree_branches'] = 8
        self.assertNotEqual(subtree_harness.normalized_stats_options(common), subtree_harness.normalized_stats_options(derived_change))

    def test_six_completed_rows_and_case_regression_are_required_for_screen(self):
        integrity = {key: True for key in subtree_harness.INTEGRITY_KEYS}
        rows = [dict(case=case, variant=variant, S=80. + (.1 if variant == 'SUBTREE' else 0),
                     all_layers_violations=0, selected_scope_violations=0)
                for case in harness.CASES for variant in subtree_harness.VARIANTS]
        self.assertFalse(subtree_harness.screening(rows[:-1], integrity)['screening_passed'])
        self.assertTrue(subtree_harness.screening(rows, integrity)['screening_passed'])
        # Mean improvement still exceeds 0.05, but one case loses more than the cap.
        rows[1]['S'], rows[3]['S'], rows[5]['S'] = 80.2, 79.94, 80.2
        outcome = subtree_harness.screening(rows, integrity)
        self.assertGreater(outcome['mean_delta_S'], .05)
        self.assertFalse(outcome['screening_passed'])


if __name__ == '__main__':
    unittest.main()