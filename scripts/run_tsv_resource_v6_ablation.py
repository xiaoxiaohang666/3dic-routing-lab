#!/usr/bin/env python3
"""benchmark-host resource-v6 ablations; acceptance evidence uses documented readback.

Compatibility scorer results are deliberately excluded from variant selection.
G=20 and the complete resource contract are uniform for every input.
"""
from pathlib import Path
import json,math,sys
import run_tsv_resource_ablation as base

FEATURES=('future-tsv-reservation','resource-local-rrr','joint-layer-recovery','resource-candidate-pool','thermal-coverage')
CONFIGS={
 'A1': ((1,0,0),(0,0,0,0,0)),
 'Q1': ((1,1,0),(0,0,0,0,0)),
 'R2': ((1,0,0),(1,0,0,0,0)),
 'RR': ((1,0,0),(0,1,0,0,0)),
 'LY': ((1,0,0),(0,0,1,0,0)),
 'HT': ((1,1,0),(0,0,0,1,1)),
 'V6': ((1,1,0),(1,1,1,1,1)),
 'FQ': ((1,1,0),(1,0,0,0,0)),
 'FLQ': ((1,1,0),(1,0,1,0,0)),
}
base.VARIANTS={key:value[0] for key,value in CONFIGS.items()}

class Experiment(base.Experiment):
 def __init__(self,args):
  super().__init__(args)
  self.measurer=self.job/'build-release/bin/eda_measure_documented'
  if not self.measurer.is_file():raise ValueError('Missing documented Guide measurer')
  self.programs_before[str(self.measurer)]=base.sha(self.measurer)
  self.state.update(selection_model='documented-rules-tsv-landing-tracks-v1',
   external_scorer_used=False,external_thermal_gain=None,
   feature_flags={key:dict(zip(FEATURES,CONFIGS[key][1])) for key in args.variants})
  self.publish()
 def execute(self,name,step,command,timeout,env,allowed=(0,)):
  command=list(command)
  if step=='route':
   for key,value in zip(FEATURES,CONFIGS[self.state['runs'][name]['variant']][1]):command+=['--'+key,str(value)]
  return super().execute(name,step,command,timeout,env,allowed)
 def run_route(self,name):
  super().run_route(name)
  record=self.state['runs'][name];stats=base.read_json(Path(record['folder'])/'stats.json')
  self.update(record,resource_v6=stats.get('resource_v6'))
 def run_evaluation(self,name):
  record=self.state['runs'][name]
  if record['status']!='ROUTED':return
  folder=Path(record['folder']);case=self.inputs[record['case']];guide=folder/'result.guide'
  env=self.environment(True,self.runtime,self.binary)
  self.update(record,status='VALIDATING')
  command=[self.validator,'--model','official','--score-profile','20261020','--tsv-resources','global-grid',
   '--def',case['def'][0],'--thermal',case['thermal'][0],'--guide',guide,'--report',folder/'engineering.json']
  for lef in case['lef']:command+=['--lef',lef]
  self.execute(name,'engineering',command,1800,env)
  engineering=base.read_json(folder/'engineering.json')
  if engineering.get('status')!='PASS':raise ValueError(name+': engineering validation failed')
  command=[sys.executable,base.SOURCE/'scripts/audit_guide_geometry.py','--def',case['def'][0],
   '--guide',guide,'--report',folder/'geometry.json']
  for lef in case['lef']:command+=['--lef',lef]
  self.execute(name,'geometry',command,1800,self.environment(),allowed=(0,2))
  geometry=base.read_json(folder/'geometry.json')
  if geometry.get('summary',{}).get('errors')!=0:raise ValueError(name+': geometry errors')
  self.update(record,status='DOCUMENTED_READBACK')
  command=[self.measurer,'--def',case['def'][0],'--thermal',case['thermal'][0],'--guide',guide,
   '--runtime',self.runtime,'--output',folder/'documented.json']
  for lef in case['lef']:command+=['--lef',lef]
  self.execute(name,'documented',command,1800,env)
  documented=base.read_json(folder/'documented.json');m=documented['metrics'];router=record['router_metrics']
  for left,right in [('W','wirelength_score'),('C','congestion_score'),('T','thermal_score'),
    ('rho','correlation'),('L','guide_mst_dbu'),('Q','squared_normalized_overflow')]:
   if not math.isclose(m[left],router[right],rel_tol=1e-10,abs_tol=1e-8):
    raise ValueError(f'{name}: readback {left} differs: {m[left]} vs {router[right]}')
  if not math.isclose(m['S'],100*router['quality'],rel_tol=1e-10,abs_tol=1e-8):raise ValueError(name+': readback S differs')
  if base.sha(guide)!=record['guide_sha256'] or base.sha(folder/'stats.json')!=record['stats_sha256']:
   raise ValueError(name+': artifact changed during readback')
  if record['resource_v6']['reservation_remaining']!=0:raise ValueError(name+': nonzero future promise')
  row=dict(m,name=name,case=record['case'],variant=record['variant'],route_seconds=record['steps']['route']['seconds'],
   score_seconds=record['steps']['documented']['seconds'],geometry_unresolved=geometry['summary'].get('unresolved'),
   guide_sha256=record['guide_sha256'])
  self.update(record,status='COMPLETE',active_step=None,row=row,documented_report=documented,
   engineering_report=engineering,geometry_report=geometry)
  print(name,'COMPLETE','documented S=',round(row['S'],6),flush=True)

def main(argv=None):
 argv=list(sys.argv[1:] if argv is None else argv)
 if '--variants' not in argv:argv+=['--variants','A1','Q1']
 args=base.parse_args(argv)
 return Experiment(args).run()
if __name__=='__main__':sys.exit(main())