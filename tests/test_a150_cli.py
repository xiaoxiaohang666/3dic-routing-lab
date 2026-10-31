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
   def run(args):
    p=subprocess.run(list(map(str,args)),env=env,capture_output=True,text=True,timeout=60)
    self.assertEqual(p.returncode,0,p.stdout+p.stderr)
   run([VALIDATOR,'--def',df,'--lef',lf,'--model','official','--tsv-resources','global-grid','--rules-profile','a150','--tsv-supply-only','1','--report',d/'supply.json'])
   supply=json.loads((d/'supply.json').read_text())['a150']
   self.assertTrue(supply['selected_scope_supply_sufficient'])
   self.assertLessEqual(supply['all_layers_eligible_cells'],supply['interface_eligible_cells'])
   run([ROUTER,'--rules-profile','a150','--def',df,'--lef',lf,'--thermal-json',heat,'--model','official','--official-runtime',RUNTIME,'--output',guide,'--stats',stats,'--time-limit','10','--rrr-rounds','0','--tsv-rounds','0','--wirelength-rounds','0'])
   saved=json.loads(stats.read_text());self.assertTrue(saved['a150']['enabled'])
   self.assertEqual(saved['a150']['obstacle_scope'],'all-routing-layers-both-dies')
   self.assertEqual(saved['rules_contract'],'a150-v9-tsv-tracks-geometric-obstacles')
   run([VALIDATOR,'--def',df,'--lef',lf,'--thermal',heat,'--guide',guide,'--report',d/'engineering.json','--model','official','--tsv-resources','global-grid','--rules-profile','a150'])
   checked=json.loads((d/'engineering.json').read_text());self.assertEqual(checked['status'],'PASS')
   self.assertEqual(checked['a150']['all_layers_violations'],0)
   run([MEASURER,'--rules-profile','a150','--def',df,'--lef',lf,'--thermal',heat,'--guide',guide,'--runtime',RUNTIME,'--output',d/'fresh.json'])
   fresh=json.loads((d/'fresh.json').read_text());self.assertTrue(fresh['a150']['enabled'])
   self.assertEqual(fresh['a150']['all_layer_violations'],0)
   for key,other in [('W','wirelength_score'),('C','congestion_score'),('T','thermal_score'),('Q','squared_normalized_overflow'),('rho','correlation')]:
    self.assertAlmostEqual(fresh['metrics'][key],saved['metrics'][other],places=8)
   for path,sha in inputs.items():self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),sha)
if __name__=='__main__':unittest.main()
