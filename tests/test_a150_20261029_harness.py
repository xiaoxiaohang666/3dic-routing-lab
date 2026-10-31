"""October 29 baseline evidence gates; no route/scoring process is launched."""
from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import run_a150_20261029_baseline as harness


def metadata(demand=7):
    return {'pin_zero_track_floor': {'enabled': True, 'demand': demand,
                                    'granularity_assumption': 'per-logical-PIN-layer-GCell'},
            'congestion_formula_status': harness.SCORE_ALIGNMENT}


def supply():
    return dict(metadata(), rules_profile=harness.RULES_PROFILE, supply_only=True,
                status='PASS', a150={'scope': harness.SCOPE_NAMES['all-layers'],
                    'cross_die_nets': 3, 'interface_eligible_cells': 10,
                    'all_layers_eligible_cells': 5, 'selected_eligible_cells': 5,
                    'selected_scope_supply_sufficient': True,
                    'fixed_interface_violations': 0, 'fixed_all_layers_violations': 0})


def recorded(serialized=False):
    report = dict(metadata(), a150={'enabled': True, 'obstacle_scope': 'all-layers',
                                   'interface_violations': 0, 'all_layer_violations': 0})
    report['model' if serialized else 'rules_contract'] = harness.A150_CONTRACT
    return report


class October29BaselineHarnessTests(unittest.TestCase):
    def test_budget_scope_and_named_proxy_contract(self):
        args = harness.parse_args(['--output-dir', '/unused/new-results'])
        self.assertEqual(args.budget, 1800)
        self.assertEqual(args.budget - args.budget_reserve, 1740)
        self.assertEqual(args.tsv_obstacle_scope, 'all-layers')
        self.assertEqual(args.cases, ['001', '003', '005'])
        self.assertEqual(harness.RULES_PROFILE, 'a150-20261029')
        self.assertIn('0.1', harness.RULE_ASSUMPTIONS['congestion_formula'])

    def test_old_profile_or_contract_cannot_qualify_even_with_clean_geometry(self):
        report = supply(); report['rules_profile'] = 'a150'
        with self.assertRaisesRegex(ValueError, 'preflight profile'):
            harness.supply_contract(report, 'all-layers')
        report = recorded(); report['rules_contract'] = 'a150-v9-tsv-tracks-geometric-obstacles'
        with self.assertRaisesRegex(ValueError, 'resource contract'):
            harness.recorded_profile(report, 'all-layers')
        report = supply(); report['rules_profile'] = 'historical'
        with self.assertRaisesRegex(ValueError, 'October 29 profile'):
            harness.legal_contract(report, 'all-layers', {})

    def test_new_name_cannot_hide_disabled_floor_or_changed_granularity(self):
        for mutate in (lambda r: r['pin_zero_track_floor'].update(enabled=False),
                       lambda r: r['pin_zero_track_floor'].update(demand=True),
                       lambda r: r['pin_zero_track_floor'].update(granularity_assumption='one-per-cell')):
            report = recorded(); mutate(report)
            with self.assertRaises(ValueError):
                harness.recorded_profile(report, 'all-layers')
        harness.recorded_profile(recorded(), 'all-layers')

    def test_proxy_cannot_be_silently_claimed_as_confirmed_new_score(self):
        for status in (None, 'CONFIRMED', '20261020'):
            report = supply(); report['congestion_formula_status'] = status
            with self.assertRaisesRegex(ValueError, 'explicitly unconfirmed'):
                harness.supply_contract(report, 'all-layers')

    def test_engineering_checks_floor_totals_and_forbidden_landings(self):
        audited = harness.supply_contract(supply(), 'all-layers')
        report = supply(); report['supply_only'] = False
        report['a150'].update(interface_violations=0, all_layers_violations=0)
        self.assertEqual(harness.legal_contract(report, 'all-layers', audited)['selected_scope_violations'], 0)
        report['pin_zero_track_floor']['demand'] += 1
        with self.assertRaisesRegex(ValueError, 'floor demand differs'):
            harness.legal_contract(report, 'all-layers', audited)
        report['pin_zero_track_floor']['demand'] -= 1
        report['a150']['all_layers_violations'] = 1
        with self.assertRaisesRegex(ValueError, 'forbidden PIN/OBS'):
            harness.legal_contract(report, 'all-layers', audited)
        report = recorded(True); report['a150']['all_layer_violations'] = 1
        with self.assertRaisesRegex(ValueError, 'forbidden TSV landings'):
            harness.recorded_profile(report, 'all-layers', serialized=True)

    def test_unknown_penalty_keeps_subtree_gate_closed_even_for_legal_three_cases(self):
        rows = [{'all_layers_violations': 0} for _ in harness.CASES]
        self.assertFalse(harness.qualifies_for_subtree(rows, True))
        self.assertFalse(harness.qualifies_for_subtree(rows, False))
        rows[1]['all_layers_violations'] = 1
        self.assertFalse(harness.qualifies_for_subtree(rows, True))

    def test_supply_failure_stops_every_route_and_evaluation(self):
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(supply_only=False, workers=3, evaluation_workers=3)
        experiment.state = {'runs': {case: {'status': 'PENDING'} for case in harness.CASES}}
        phases = []
        def update(record, **values): record.update(values)
        def phase(function, names, workers):
            phases.append(function.__name__)
            for name in names: experiment.state['runs'][name]['status'] = 'SUPPLY_READY'
            experiment.state['runs']['003']['status'] = 'SUPPLY_INFEASIBLE'
        experiment.update, experiment.phase, experiment.finish = update, phase, lambda: 2
        self.assertEqual(experiment.run(), 2)
        self.assertEqual(phases, ['run_preflight'])

    def test_complete_output_separates_verified_resources_from_unconfirmed_score(self):
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(cases=list(harness.CASES), tsv_obstacle_scope='all-layers',
                                          supply_only=False, budget=1800, budget_reserve=60)
        rows = [dict(case=case, W=70., C=80., T=90., S=78., route_seconds=100.,
                     interface_violations=0, all_layers_violations=0) for case in harness.CASES]
        experiment.state = {'runs': {row['case']: {'case': row['case'], 'status': 'COMPLETE', 'row': row}
                                    for row in rows}}
        experiment.inputs_before = experiment.sources_before = experiment.programs_before = experiment.runtime_before = {}
        experiment.update = lambda record, **values: record.update(values)
        with tempfile.TemporaryDirectory() as folder:
            experiment.out = Path(folder)
            with mock.patch.object(harness.base, 'files_hash', return_value={}), mock.patch.object(harness.base, 'source_hashes', return_value={}):
                self.assertEqual(experiment.finish(), 0)
            result = json.loads((experiment.out / 'summary.json').read_text())
            self.assertTrue(result['geometric_and_resource_baseline_confirmed'])
            self.assertTrue(result['baseline_confirmed_under_recorded_rules'])
            self.assertFalse(result['official_score_alignment_confirmed'])
            self.assertFalse(result['subtree_optimization_allowed_after_this_gate'])
            self.assertFalse(result['official_equivalence'])
            self.assertEqual(result['score_alignment'], harness.SCORE_ALIGNMENT)


if __name__ == '__main__':
    unittest.main()
