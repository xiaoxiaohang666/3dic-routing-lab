#!/usr/bin/env python3
"""A150 production defaults and independent serialized Guide checks."""
import hashlib,json,os,subprocess,sys,tempfile,unittest
from pathlib import Path
from test_official_cli import technology,design
ROUTER,VALIDATOR,MEASURER,RUNTIME,ARTIFACTS=[Path(p).resolve() for p in sys.argv[1:6]]
del sys.argv[1:6]

class A150Cli(unittest.TestCase):
 def test_a150_rejects_disabled_or_incompatible_contract(self):
  for flags,message in [
   (['--model','official','--tsv-track-demand','0'],'A150 requires --tsv-track-demand 1'),
   (['--model','official','--tsv-resource-mode','paired'],'A150 requires --tsv-resource-mode global-grid'),
   (['--model','official','--tsv-obstacle-scope','disabled'],'invalid TSV obstacle scope'),
   (['--model','official','--rules-profile','historical','--tsv-obstacle-scope','interface'],'historical replay cannot')]:
   p=subprocess.run([str(ROUTER),*flags],capture_output=True,text=True)
   self.assertNotEqual(p.returncode,0)
   self.assertIn(message,p.stderr)

 def test_default_a150_route_supply_and_readback(self):
  ARTIFACTS.mkdir(parents=True,exist_ok=True)
  with tempfile.TemporaryDirectory(prefix='a150-',dir=ARTIFACTS) as temp:
   d=Path(temp);lf=d/'input.lef';df=d/'input.def';heat=d/'heat.json';guide=d/'output.guide';stats=d/'stats.json'
   lf.write_text(technology());df.write_text(design())
   heat.write_text(json.dumps({'meta':{'ambientTemp':25},'nodes':[[5+10*c,5+10*r,0,30+c+2*r] for r in range(4) for c in range(4)]}))
   inputs={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in (lf,df,heat)}
   env=dict(os.environ);env['LD_LIBRARY_PATH']=str(RUNTIME.parent.parent/'lib')+':'+str(RUNTIME)+':'+env.get('LD_LIBRARY_PATH','')
   def run(args,run_env=env):
    p=subprocess.run(list(map(str,args)),env=run_env,capture_output=True,text=True,timeout=60)
    self.assertEqual(p.returncode,0,p.stdout+p.stderr)
   run([VALIDATOR,'--def',df,'--lef',lf,'--model','official','--tsv-resources','global-grid','--rules-profile','a150-20261029','--tsv-supply-only','1','--report',d/'supply.json'])
   supply=json.loads((d/'supply.json').read_text())['a150']
   self.assertTrue(supply['selected_scope_supply_sufficient'])
   self.assertLessEqual(supply['all_layers_eligible_cells'],supply['interface_eligible_cells'])
   run([ROUTER,'--def',df,'--lef',lf,'--thermal-json',heat,'--model','official','--official-runtime',RUNTIME,'--output',guide,'--stats',stats,'--time-limit','10','--rrr-rounds','0','--tsv-rounds','0','--wirelength-rounds','0'])
   saved=json.loads(stats.read_text());self.assertTrue(saved['a150']['enabled'])
   self.assertTrue(saved['pin_zero_track_floor']['enabled'])
   self.assertEqual(saved['congestion_formula_status'],'UNCONFIRMED_20261029_CONGESTION_FORMULA')
   self.assertEqual(saved['a150']['obstacle_scope'],'interface-m10-m11')
   self.assertEqual(saved['rules_contract'],'a150-20261029-pin-floor-tsv-tracks')
   run([VALIDATOR,'--def',df,'--lef',lf,'--thermal',heat,'--guide',guide,'--report',d/'engineering.json','--model','official','--tsv-resources','global-grid','--rules-profile','a150-20261029'])
   checked=json.loads((d/'engineering.json').read_text());self.assertEqual(checked['status'],'PASS')
   self.assertEqual(checked['a150']['scope'],'interface-m10-m11')
   self.assertEqual(checked['a150']['interface_violations'],0)
   run([MEASURER,'--def',df,'--lef',lf,'--thermal',heat,'--guide',guide,'--runtime',RUNTIME,'--output',d/'fresh.json'])
   fresh=json.loads((d/'fresh.json').read_text());self.assertTrue(fresh['a150']['enabled'])
   self.assertTrue(fresh['pin_zero_track_floor']['enabled'])
   self.assertEqual(fresh['congestion_formula_status'],'UNCONFIRMED_20261029_CONGESTION_FORMULA')
   self.assertEqual(fresh['a150']['obstacle_scope'],'interface')
   self.assertEqual(fresh['a150']['interface_violations'],0)
   for key,other in [('W','wirelength_score'),('C','congestion_score'),('T','thermal_score'),('Q','squared_normalized_overflow'),('rho','correlation')]:
    self.assertAlmostEqual(fresh['metrics'][key],saved['metrics'][other],places=8)
   # The older all-layers rule stays selectable as the rollback path.
   strict=d/'all-layers.guide';strict_stats=d/'all-layers.json'
   run([ROUTER,'--def',df,'--lef',lf,'--thermal-json',heat,'--model','official','--official-runtime',RUNTIME,'--output',strict,'--stats',strict_stats,'--time-limit','10','--rrr-rounds','0','--tsv-rounds','0','--wirelength-rounds','0','--tsv-obstacle-scope','all-layers'])
   self.assertEqual(json.loads(strict_stats.read_text())['a150']['obstacle_scope'],'all-routing-layers-both-dies')
   run([VALIDATOR,'--def',df,'--lef',lf,'--thermal',heat,'--guide',strict,'--report',d/'strict-engineering.json','--model','official','--tsv-resources','global-grid','--rules-profile','a150-20261029','--tsv-obstacle-scope','all-layers'])
   checked=json.loads((d/'strict-engineering.json').read_text());self.assertEqual(checked['status'],'PASS')
   self.assertEqual(checked['a150']['scope'],'all-routing-layers-both-dies')
   self.assertEqual(checked['a150']['all_layers_violations'],0)
   run([MEASURER,'--def',df,'--lef',lf,'--thermal',heat,'--guide',strict,'--runtime',RUNTIME,'--output',d/'strict-fresh.json','--tsv-obstacle-scope','all-layers'])
   fresh=json.loads((d/'strict-fresh.json').read_text())
   self.assertEqual(fresh['a150']['obstacle_scope'],'all-layers')
   self.assertEqual(fresh['a150']['all_layer_violations'],0)
   # Official runs use the sideways step at radius 3; an explicit 0 turns it off.
   self.assertEqual(saved['options']['climb_escape_radius'],3)
   def options_of(name,*flags,command=(ROUTER,),run_env=env):
    out=d/f'{name}.json'
    run([*command,'--def',df,'--lef',lf,'--thermal-json',heat,*flags,'--output',d/f'{name}.guide','--stats',out,'--time-limit','10'],run_env)
    return json.loads(out.read_text())
   off=options_of('radius-off','--model','official','--official-runtime',RUNTIME,'--rrr-rounds','0','--tsv-rounds','0','--wirelength-rounds','0','--climb-escape-radius','0')
   self.assertEqual(off['options']['climb_escape_radius'],0)
   # The platform entry goes through the same scripts as a built package.
   package=dict(env,EDA_ROUTER_BINARY=str(ROUTER),EDA_OFFICIAL_RUNTIME=str(RUNTIME))
   script=Path(__file__).resolve().parent.parent/'scripts/run_package.sh'
   packaged=options_of('radius-package',command=('bash',script),run_env=package)
   self.assertEqual(packaged['options']['climb_escape_radius'],3)
   self.assertEqual(packaged['options']['rrr_rounds'],64)
   self.assertEqual(packaged['options']['rrr_net_limit'],8192)
   self.assertEqual(packaged['options']['v7_rrr_net_limit'],32768)
   self.assertTrue(packaged['resource_v6']['future_tsv_reservation'])
   for path,sha in inputs.items():self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),sha)
if __name__=='__main__':unittest.main()
