#!/usr/bin/env python3
import json,os,subprocess,sys,tempfile,unittest
from pathlib import Path
from test_official_cli import design,technology
ROUTER,MEASURER,VALIDATOR,RUNTIME=map(lambda p:Path(p).resolve(),sys.argv[1:5]);del sys.argv[1:5]
ROOT=Path(__file__).resolve().parents[1]
class ResourceV7Tests(unittest.TestCase):
 def test_option_contract(self):
  flags=('checkpoint-amortization','tsv-index-reuse','checkpoint-reuse','future-tsv-reservation','resource-local-rrr','joint-layer-recovery','resource-candidate-pool','thermal-coverage')
  help=subprocess.run([str(ROUTER),'--help'],capture_output=True,text=True,check=True).stdout
  for flag in flags:
   self.assertIn('--'+flag,help)
   for value in ('2','-1','nan'):
    p=subprocess.run([str(ROUTER),'--'+flag,value],capture_output=True,text=True)
    self.assertNotEqual(p.returncode,0)
 def test_uniform_wrapper_serialized_readback(self):
  with tempfile.TemporaryDirectory(prefix='resource v7 ') as tmp:
   d=Path(tmp);lef=d/'input.lef';df=d/'input.def';heat=d/'heat.json';guide=d/'result.guide';stats=d/'stats.json'
   lef.write_text(technology());df.write_text(design());heat.write_text(json.dumps({'meta':{'ambientTemp':25},
    'nodes':[[5+10*c,5+10*r,0,30+c+2*r] for r in range(4) for c in range(4)]}))
   env=dict(os.environ,EDA_ROUTER_BINARY=str(ROUTER),EDA_OFFICIAL_RUNTIME=str(d/'missing runtime'))
   def run(cmd):
    p=subprocess.run(list(map(str,cmd)),cwd=d,env=env,capture_output=True,text=True,timeout=60)
    self.assertEqual(p.returncode,0,p.stdout+p.stderr)
   saved_ld=env.pop('LD_LIBRARY_PATH',None)
   run(['bash',ROOT/'scripts/run_tsv_resource_v7.sh','--official-runtime',RUNTIME,'--def',df,'--lef',lef,'--thermal-json',heat,
    '--output',guide,'--stats',stats,'--time-limit','10','--large-recovery-seconds','1',
    '--v8-tsv-rematch-seconds','1','--v7-tsv-seconds','1'])
   if saved_ld is not None:env['LD_LIBRARY_PATH']=saved_ld
   saved=json.loads(stats.read_text());self.assertEqual(saved['thermal_gain'],20)
   self.assertEqual(saved['resource_v7']['checkpoint_amortization'],bool(0))
   self.assertFalse(saved['resource_v7']['tsv_index_reuse'])
   self.assertFalse(saved['resource_v7']['checkpoint_reuse'])
   self.assertEqual(saved['resource_v6']['reservation_remaining'],0)
   self.assertFalse(saved['official_scorer_run'])
   self.assertTrue(saved['options']['tsv_track_demand'])
   self.assertTrue(saved['options']['tsv_landing_pricing'])
   self.assertFalse(saved['options']['tsv_landing_rrr'])
   for flag in ('future_tsv_reservation','resource_local_rrr','joint_layer_recovery','resource_candidate_pool','thermal_coverage'):
    self.assertFalse(saved['resource_v6'][flag])
   run([MEASURER, "--rules-profile", "historical",'--def',df,'--lef',lef,'--thermal',heat,'--guide',guide,'--runtime',RUNTIME,'--output',d/'fresh.json'])
   fresh=json.loads((d/'fresh.json').read_text())['metrics']
   for key,other in [('W','wirelength_score'),('C','congestion_score'),('T','thermal_score'),('rho','correlation'),('Q','squared_normalized_overflow')]:
    self.assertAlmostEqual(fresh[key],saved['metrics'][other],places=8)
   self.assertAlmostEqual(fresh['S'],100*saved['metrics']['quality'],places=8)
if __name__=='__main__':unittest.main()