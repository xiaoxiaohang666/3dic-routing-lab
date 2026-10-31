#!/usr/bin/env python3
"""Read-only E0 replay of frozen v7 Guides. Run exclusively on benchmark-host.

Compatibility estimates are diagnostic only. Production acceptance remains the
uniform documented model with G=20. This script never edits input Guides.
"""
import argparse, concurrent.futures, csv, hashlib, json, os, socket, subprocess, time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

ROOT=(Path.home() / ".cache" / "eda-official-evaluations")
V7=ROOT/'tsv-resource-v7-20261028-3ea1'
SCORER=Path(__file__).resolve().parents[1]
CASES=ROOT/'full-suite-20261018-04ab4298/cases'
DEPENDENCY=ROOT/'community042-audit-dff686f959/package'

def read(path): return json.loads(Path(path).read_text())
def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def write(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def stamp(): return datetime.now(timezone.utc).isoformat()
def layer_summary(folder):
    layers=defaultdict(lambda:dict(Q=0.,overflow=0.,entries=0,one_track_entries=0,tiles={}))
    with (folder/'overflow_gcells.csv').open(newline='') as f:
        for r in csv.DictReader(f):
            x,y=int(r['gcell_col']),int(r['gcell_row'])
            cap,over=float(r['capacity']),float(r['overflow'])
            q=(.1*over/cap)**2 if cap>0 else 10000.
            d=layers[r['layer']];d['Q']+=q;d['overflow']+=over;d['entries']+=1
            d['one_track_entries']+=int(over==1)
            k=f'{x//32},{y//32}';d['tiles'][k]=d['tiles'].get(k,0.)+q
    for d in layers.values():
        tiles=d.pop('tiles');d['occupied_32x32_tiles']=len(tiles)
        d['top10_tile_Q_fraction']=sum(sorted(tiles.values(),reverse=True)[:10])/d['Q'] if d['Q'] else 0.
        d['one_track_fraction']=d['one_track_entries']/d['entries']
    return dict(layers)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--workers',type=int,default=3)
    args=p.parse_args()
    if socket.gethostname()!='benchmark-host': raise SystemExit('E0 must run on benchmark-host')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    plan={'phase':'RUNNING','started_utc':stamp(),'host':socket.gethostname(),'pid':os.getpid(),
          'official_equivalence':False,'production_candidate_acceptance_supported':False,
          'frozen_source':str(V7/'source'),'scorer_source':str(SCORER),'cases':{},
          'prior_run_warning':'v7 amort1800 had 18 concurrent route workers; replay establishes artifact identity, not new timed comparison',
          'official_program_guide_available':False}
    files=[V7/'build-release/bin/eda_router',V7/'delivered-source.tar.gz']
    for sub in ['scripts/score_20261027.py','tools/scorer_20261027/cli.py','tools/scorer_20261027/profile.py','tools/scorer_20261027/__init__.py']:files.append(SCORER/sub)
    inputs={}
    for case in ['001','003','005']:
        source=V7/'amort1800'/f'{case}-Q1_r1';case_dir=CASES/f'open_3d_case{case}'
        inputs[case]=dict(guide=source/'result.guide',lef=case_dir/f'open_3d_case{case}.input.lef',
                         def_file=case_dir/f'open_3d_case{case}.input.def',thermal=next(case_dir.glob('*.thermal.input.json')))
        files+=list(inputs[case].values())+[source/'documented.json',source/'stats.json',source/'route.log',source/'engineering.json',source/'geometry.json']
    plan['sha256_before']={str(f):sha(f) for f in files}
    plan['router_matches_prior_delivery']=plan['sha256_before'][str(V7/'build-release/bin/eda_router')]==read(V7/'delivery-integrity.json')['router_sha256']
    write(args.output_dir/'status.json',plan)
    def score(case):
        inp=inputs[case];folder=args.output_dir/case;source=V7/'amort1800'/f'{case}-Q1_r1'
        command=['python3',str(SCORER/'scripts/score_20261027.py'),str(inp['guide']),'--lefs',str(inp['lef']),
                 '--input-def',str(inp['def_file']),'--thermal-json',str(inp['thermal']),'--dependency-root',str(DEPENDENCY),'--output-dir',str(folder),
                 '--rules-profile','inferred-platform','--capacity-boundary','half-open','--fixed-track-boundary','half-open','--landing-track-boundary','open']
        start=time.monotonic()
        with (args.output_dir/f'{case}.log').open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,cwd=SCORER,timeout=1800)
        record={'command':command,'exit_code':result.returncode,'wall_seconds':time.monotonic()-start,
                'inputs_sha256':{k:plan['sha256_before'][str(v)] for k,v in inp.items()},
                'documented':read(source/'documented.json'),'engineering':read(source/'engineering.json'),'geometry':read(source/'geometry.json')}
        if (folder/'evaluation_report.json').exists():record['compatible_report']=read(folder/'evaluation_report.json')
        if (folder/'run_record.json').exists():record['compatible_run_record']=read(folder/'run_record.json')
        if (folder/'overflow_gcells.csv').exists():record['compatible_layers']=layer_summary(folder)
        write(args.output_dir/f'{case}-summary.json',record)
        return case,record
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in concurrent.futures.as_completed([pool.submit(score,c) for c in inputs]):
            case,record=future.result();plan['cases'][case]=record
            write(args.output_dir/'status.json',plan)
            print(case,record['exit_code'],record.get('compatible_report',{}).get('score'),flush=True)
    plan['sha256_after']={str(f):sha(f) for f in files}
    plan['changed_original_files']=[f for f in plan['sha256_before'] if plan['sha256_before'][f]!=plan['sha256_after'][f]]
    plan['phase']='COMPLETE' if not plan['changed_original_files'] and all(r['exit_code']==0 for r in plan['cases'].values()) else 'FAILED'
    plan['completed_utc']=stamp();write(args.output_dir/'status.json',plan)
    return 0 if plan['phase']=='COMPLETE' else 1
if __name__=='__main__':raise SystemExit(main())
