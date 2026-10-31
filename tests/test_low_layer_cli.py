"""Contract and serialized readback for opt-in low-layer and RRR phases."""
import json, os, subprocess, sys, tempfile, unittest
from pathlib import Path
from test_official_cli import design, technology
ROUTER, MEASURER, RUNTIME = [Path(p).resolve() for p in sys.argv[1:4]]
del sys.argv[1:4]
ROOT = Path(__file__).resolve().parents[1]

class LowLayerCliTests(unittest.TestCase):
    def test_bad_options_and_required_resource_model(self):
        for flag, values in {
            '--low-layer-mask': ('4', '-1', 'nan'),
            '--low-layer-seconds': ('0', 'nan', '3601'),
            '--low-layer-max-attempts': ('0', '1000001'),
            '--rrr-reserved-seconds': ('nan', '3601'),
        }.items():
            for value in values:
                result = subprocess.run([str(ROUTER), flag, value], capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0, (flag, value))
                self.assertNotIn('required: --def', result.stderr)
        result = subprocess.run([str(ROUTER), '--low-layer-mask', '3'], capture_output=True, text=True)
        self.assertIn('require official documented', result.stderr)

    def test_combined_phases_preserve_serialized_ledger(self):
        with tempfile.TemporaryDirectory(prefix='low-layer-cli-') as tmp:
            root = Path(tmp)
            lef, df, heat = root/'in.lef', root/'in.def', root/'heat.json'
            lef.write_text(technology()); df.write_text(design())
            heat.write_text(json.dumps({'meta': {'ambientTemp': 25},
                'nodes': [[5+10*c,5+10*r,0,30+c+2*r] for r in range(4) for c in range(4)]}))
            env = dict(os.environ, EDA_ROUTER_BINARY=str(ROUTER), EDA_OFFICIAL_RUNTIME=str(RUNTIME))
            env['LD_LIBRARY_PATH'] = str(RUNTIME.parent.parent/'lib')+':'+str(RUNTIME)
            command = ['bash', str(ROOT/'scripts/run_tsv_resource_v7.sh'),
                '--def', str(df), '--lef', str(lef), '--thermal-report', str(heat),
                '--output', str(root/'result.guide'), '--stats', str(root/'stats.json'),
                '--time-limit', '10', '--low-layer-mask', '3', '--low-layer-seconds', '1',
                '--low-layer-max-attempts', '20', '--rrr-reserved-seconds', '1']
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            saved = json.loads((root/'stats.json').read_text())
            self.assertEqual(saved['low_layer']['mask'], 3)
            self.assertEqual(saved['low_layer']['rrr_reserved_seconds'], 1)
            self.assertFalse(saved['official_scorer_run'])
            result = subprocess.run([str(MEASURER), "--rules-profile", "historical", '--def', str(df), '--lef', str(lef),
                '--thermal', str(heat), '--guide', str(root/'result.guide'), '--runtime', str(RUNTIME),
                '--output', str(root/'readback.json')], env=env, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            fresh = json.loads((root/'readback.json').read_text())['metrics']
            for key, metric in [('W','wirelength_score'), ('C','congestion_score'), ('T','thermal_score'),
                                ('Q','squared_normalized_overflow'), ('rho','correlation')]:
                self.assertAlmostEqual(fresh[key], saved['metrics'][metric], places=8)

if __name__ == '__main__':
    unittest.main()
