"""v11 diagnostic pair contracts; no router or scoring process is launched."""
from pathlib import Path
import contextlib
import copy
import io
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import run_a150_v11_diagnostic as harness


def records(case='005'):
    pair = []
    for variant, flag in (('BASE', '0'), ('TREE', '1')):
        row = dict(case=case, variant=variant, L=100., rho=-.08, Q=3., overflow=20,
                   W=70., C=50., T=4., S=47.5, route_seconds=1790.,
                   pin_zero_track_floor_demand=100,
                   interface_violations=0, all_layers_violations=0, selected_scope_violations=0)
        command = ['bash', '/source/scripts/run_a150_20261029.sh',
                   '--time-limit', '1740.0', '--rules-profile', 'a150-20261029',
                   '--thermal-gain', '20', '--def', '/inputs/case.def',
                   '--lef', '/inputs/a.lef', '--lef', '/inputs/b.lef',
                   '--output', '/results/' + variant + '/result.guide',
                   '--stats', '/results/' + variant + '/stats.json',
                   '--tsv-tree-move', flag, '--tsv-tree-move-seconds', '0.02',
                   '--tsv-tree-move-expansions', '4096']
        pair.append(dict(case=case, variant=variant, status='COMPLETE', row=row,
                         router_metrics={'guide_overflow': 20},
                         steps={'route': {'command': command, 'timeout_seconds': 1800}},
                         options={'time_limit_seconds': 1740., 'v9_subtree_recovery': False,
                                  'tsv_tree_move': variant == 'TREE', 'congestion_price_updates': 2},
                         supply={'cross_die_nets': 4, 'pin_zero_track_floor_demand': 100},
                         thermal_gain=20, rules_contract=harness.baseline.A150_CONTRACT,
                         route_binary_sha256='a' * 64,
                         tsv_tree_move={'enabled': variant == 'TREE', 'seconds_limit': .02,
                                        'expansions_limit': 4096, 'attempted': 0},
                         wirelength_breakdown={'planar_dbu': 60., 'ordinary_via_dbu': 30.,
                                               'tsv_dbu': 10., 'total_dbu': 100.}))
    return pair


class V11HarnessTests(unittest.TestCase):
    def test_default_and_small_screening_use_single_matched_batch(self):
        args = harness.parse_args(['--output-dir', '/unused/new'])
        self.assertEqual(args.workers, 6)
        self.assertEqual(args.variants, ['BASE', 'TREE'])
        self.assertEqual(args.budget - args.budget_reserve, 1740)
        self.assertEqual(args.tsv_obstacle_scope, 'all-layers')
        small = harness.parse_args(['--output-dir', '/unused/new', '--cases', '005', '--budget', '600', '--workers', '2'])
        self.assertEqual(small.budget - small.budget_reserve, 540)
        self.assertEqual(small.workers, 2)

    def test_rejects_confounded_batch_and_invalid_limits(self):
        bad = [ ['--workers', '2'], ['--cases', '005', '005'],
                ['--tree-seconds', 'nan'], ['--tree-seconds', '0'],
                ['--tree-expansions', '0'], ['--budget-reserve', '1800'] ]
        for suffix in bad:
            with self.subTest(suffix=suffix), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                harness.parse_args(['--output-dir', '/unused/new'] + suffix)

    def test_single_switch_pair_and_actual_raw_length_deltas(self):
        left, right = records()
        right['row'].update(L=95., rho=-.07, Q=2.8, S=48.)
        right['wirelength_breakdown'].update(planar_dbu=58., ordinary_via_dbu=27., total_dbu=95.)
        right['router_metrics']['guide_overflow'] = 18
        right['options']['congestion_price_updates'] = 999
        result = harness.compare_pair(left, right)
        self.assertEqual(result['delta_TREE_minus_BASE']['L'], -5)
        self.assertAlmostEqual(result['delta_TREE_minus_BASE']['rho'], .01)
        self.assertEqual(result['delta_TREE_minus_BASE']['overflow'], -2)
        self.assertEqual(result['delta_wirelength_breakdown']['ordinary_via_dbu'], -3)
        self.assertTrue(result['same_options_except_tree_switch'])
        self.assertFalse(result['formal_performance_adoption_allowed'])
        self.assertEqual(left['row']['L'], 100.)

    def test_rejects_second_cli_change_and_missing_or_duplicate_switch(self):
        left, right = records()
        right['steps']['route']['command'] += ['--thermal-coverage', '1']
        with self.assertRaisesRegex(ValueError, 'commands differ'):
            harness.compare_pair(left, right)
        left, right = records()
        right['steps']['route']['command'] += ['--tsv-tree-move', '1']
        with self.assertRaisesRegex(ValueError, 'duplicated'):
            harness.compare_pair(left, right)
        left, right = records()
        index = right['steps']['route']['command'].index('--tsv-tree-move')
        del right['steps']['route']['command'][index:index + 2]
        with self.assertRaisesRegex(ValueError, 'missing'):
            harness.compare_pair(left, right)

    def test_rejects_runtime_option_confounds(self):
        left, right = records()
        right['options']['v9_subtree_recovery'] = True
        with self.assertRaisesRegex(ValueError, 'router options differ'):
            harness.compare_pair(left, right)
        left, right = records()
        right['steps']['route']['timeout_seconds'] = 2000
        with self.assertRaisesRegex(ValueError, 'budgets differ'):
            harness.compare_pair(left, right)

    def test_rejects_binary_resource_contract_supply_and_floor_drift(self):
        mutations = [lambda r: r.update(route_binary_sha256='b' * 64),
                     lambda r: r.update(rules_contract='historical'),
                     lambda r: r.update(thermal_gain=10),
                     lambda r: r['supply'].update(pin_zero_track_floor_demand=101),
                     lambda r: r['row'].update(pin_zero_track_floor_demand=101),
                     lambda r: r['row'].update(all_layers_violations=1)]
        for mutate in mutations:
            left, right = records()
            mutate(right)
            with self.assertRaises(ValueError):
                harness.compare_pair(left, right)

    def test_raw_decomposition_must_sum_and_match_serialized_length(self):
        left, right = records()
        right['wirelength_breakdown']['ordinary_via_dbu'] -= 1
        with self.assertRaisesRegex(ValueError, 'does not sum'):
            harness.compare_pair(left, right)
        left, right = records()
        right['row']['L'] -= 1
        with self.assertRaisesRegex(ValueError, 'serialized L'):
            harness.compare_pair(left, right)
        left, right = records()
        right['router_metrics']['guide_overflow'] = True
        with self.assertRaisesRegex(ValueError, 'integer route overflow'):
            harness.compare_pair(left, right)

    def test_stats_must_record_actual_tree_configuration(self):
        stats = {'tsv_tree_move': records()[1]['tsv_tree_move'], 'options': {'tsv_tree_move': True}}
        harness.tree_configuration(stats, True, .02, 4096)
        for change in ({'enabled': False}, {'seconds_limit': .03}, {'expansions_limit': True}):
            modified = copy.deepcopy(stats)
            modified['tsv_tree_move'].update(change)
            with self.assertRaises(ValueError):
                harness.tree_configuration(modified, True, .02, 4096)
        stats['options']['tsv_tree_move'] = False
        with self.assertRaisesRegex(ValueError, 'conflict'):
            harness.tree_configuration(stats, True, .02, 4096)

    def test_execute_only_adds_tree_flags_to_route_and_does_not_mutate_command(self):
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(tree_seconds=.02, tree_expansions=4096)
        experiment.state = {'runs': {'005-TREE': {'variant': 'TREE'}}}
        command = ['bash', 'preset', '--rules-profile', 'a150-20261029']
        with mock.patch.object(harness.baseline.Experiment, 'execute', return_value={}) as execute:
            experiment.execute('005-TREE', 'route', command, 1800, {})
            actual = execute.call_args.args[2]
            self.assertEqual(actual[-6:], ['--tsv-tree-move', '1', '--tsv-tree-move-seconds', '0.02', '--tsv-tree-move-expansions', '4096'])
            self.assertEqual(command, ['bash', 'preset', '--rules-profile', 'a150-20261029'])
            experiment.execute('005-TREE', 'engineering', command, 1800, {})
            self.assertEqual(execute.call_args.args[2], command)

    def test_any_failed_preflight_stops_all_route_and_readback_work(self):
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(supply_only=False, workers=2, evaluation_workers=1)
        experiment.state = {'runs': {'005-BASE': {'status': 'PENDING'}, '005-TREE': {'status': 'PENDING'}}}
        phases = []
        experiment.update = lambda record, **values: record.update(values)
        def phase(function, names, workers):
            phases.append(function.__name__)
            experiment.state['runs']['005-BASE']['status'] = 'SUPPLY_READY'
            experiment.state['runs']['005-TREE']['status'] = 'FIXED_INTERFACE_INFEASIBLE'
        experiment.phase, experiment.finish = phase, lambda: 1
        self.assertEqual(experiment.run(), 1)
        self.assertEqual(phases, ['run_preflight'])

    def test_readback_starts_only_after_the_route_phase(self):
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(supply_only=False, workers=2, evaluation_workers=1)
        experiment.state = {'runs': {'005-BASE': {'status': 'PENDING'}, '005-TREE': {'status': 'PENDING'}}}
        phases = []
        experiment.update = lambda record, **values: record.update(values)
        def phase(function, names, workers):
            phases.append(function.__name__)
            status = {'run_preflight': 'SUPPLY_READY', 'run_route': 'ROUTED', 'run_evaluation': 'COMPLETE'}[function.__name__]
            for record in experiment.state['runs'].values(): record['status'] = status
        experiment.phase, experiment.finish = phase, lambda: 0
        self.assertEqual(experiment.run(), 0)
        self.assertEqual(phases, ['run_preflight', 'run_route', 'run_evaluation'])

    def test_finish_never_opens_formal_gate_even_for_positive_proxy_gain(self):
        left, right = records()
        right['row']['S'] += 10
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(cases=['005'], workers=2, budget=1800, budget_reserve=60)
        experiment.state = {'runs': {'005-BASE': left, '005-TREE': right}}
        experiment.inputs_before = experiment.sources_before = experiment.programs_before = experiment.runtime_before = {}
        experiment.update = lambda record, **values: record.update(values)
        with tempfile.TemporaryDirectory() as folder:
            experiment.out = Path(folder)
            with mock.patch.object(harness.base, 'files_hash', return_value={}), mock.patch.object(harness.base, 'source_hashes', return_value={}):
                self.assertEqual(experiment.finish(), 0)
                with self.assertRaises(FileExistsError): experiment.finish()
            report = json.loads((experiment.out / 'diagnostic-summary.json').read_text())
            self.assertEqual(report['status'], 'COMPLETE')
            self.assertEqual(report['mean_deltas_TREE_minus_BASE']['S'], 10)
            self.assertTrue(report['geometric_resource_pair_checks_complete'])
            for key in ('formal_performance_adoption_allowed', 'official_alignment',
                        'official_score_alignment_confirmed', 'official_equivalence',
                        'baseline_confirmed_under_recorded_rules', 'subtree_optimization_allowed_after_this_gate'):
                self.assertIs(report[key], False)

    def test_integrity_failure_preserves_diagnostic_but_returns_error(self):
        left, right = records()
        experiment = object.__new__(harness.Experiment)
        experiment.args = SimpleNamespace(cases=['005'], workers=2, budget=1800, budget_reserve=60)
        experiment.state = {'runs': {'005-BASE': left, '005-TREE': right}}
        experiment.inputs_before = experiment.programs_before = experiment.runtime_before = {}
        experiment.sources_before = {'file': 'old'}
        experiment.update = lambda record, **values: record.update(values)
        with tempfile.TemporaryDirectory() as folder:
            experiment.out = Path(folder)
            with mock.patch.object(harness.base, 'files_hash', return_value={}), mock.patch.object(harness.base, 'source_hashes', return_value={'file': 'changed'}):
                self.assertEqual(experiment.finish(), 1)
            report = json.loads((experiment.out / 'diagnostic-summary.json').read_text())
            self.assertEqual(report['status'], 'ERROR')
            self.assertFalse(report['integrity']['source_unchanged'])
            self.assertNotIn('mean_deltas_TREE_minus_BASE', report)


if __name__ == '__main__':
    unittest.main()
