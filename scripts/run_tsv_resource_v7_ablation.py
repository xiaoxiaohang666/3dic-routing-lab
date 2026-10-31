#!/usr/bin/env python3
"""Same-binary lifetime reuse ablations, executed only on benchmark-host."""
import sys
from pathlib import Path
import run_tsv_resource_v6_ablation as v6
import run_tsv_resource_ablation as base

REUSE={'Q1':(0,0),'IX':(1,0),'CK':(0,1),'BOTH':(1,1),'LOCAL':(1,1),'HEAT':(1,1)}
v6.CONFIGS={k:((1,1,0),(0,int(k=='LOCAL'),0,0,int(k=='HEAT'))) for k in REUSE}
base.VARIANTS={k:v[0] for k,v in v6.CONFIGS.items()}

class Experiment(v6.Experiment):
 def execute(self,name,step,command,timeout,env,allowed=(0,)):
  command=list(command)
  if step=='route':
   reuse=REUSE[self.state['runs'][name]['variant']]
   command+=['--tsv-index-reuse',str(reuse[0]),'--checkpoint-reuse',str(reuse[1])]
  return super().execute(name,step,command,timeout,env,allowed)
 def run_route(self,name):
  super().run_route(name)
  record=self.state['runs'][name]
  self.update(record,resource_v7=base.read_json(Path(record['folder'])/'stats.json')['resource_v7'])

def main(argv=None):
 argv=list(sys.argv[1:] if argv is None else argv)
 if '--variants' not in argv:argv+=['--variants','Q1','IX','CK']
 args=base.parse_args(argv)
 return Experiment(args).run()
if __name__=='__main__':sys.exit(main())
