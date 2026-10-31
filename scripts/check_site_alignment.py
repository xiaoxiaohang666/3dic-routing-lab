#!/usr/bin/env python3
"""Strict frozen-Guide calibration gate, never an official certification.
Only the latest supplied official reports are accepted as witnesses. A failed
resource comparison prevents a downstream optimization runner from adopting
this model. No tolerances are enlarged to hide known reference differences.
"""
from pathlib import Path
import argparse,hashlib,json,math,zipfile

def compare(expected,actual):
 finite=lambda x:isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)
 ok=finite(expected) and finite(actual) and math.isclose(expected,actual,rel_tol=1e-11,abs_tol=1e-8)
 return {'expected':expected,'actual':actual,'difference':actual-expected if finite(actual) and finite(expected) else None,'pass':ok}

def evaluate(official_zip,root):
 rows=[]
 with zipfile.ZipFile(official_zip) as z:
  reports=json.loads(z.read('scores.json'))['cases']
  for name,record in reports.items():
   if record.get('status')!='success':continue
   case=name[-3:];raw=record['raw_report'];actual=json.loads((root/(case+'-verified-measure-01.json')).read_text());m=actual['metrics']
   expected={'L':raw['wirelength']['total_dbu'],'reference':raw['wirelength']['hpwl_reference_dbu'],'W':raw['score']['wirelength_score'],'C':raw['score']['congestion_score'],'T':raw['score']['thermal_score'],'rho':raw['thermal_tsv']['correlation_above_ambient'],'overflow':raw['congestion']['total_overflow']}
   checks={k:compare(v,m.get(k)) for k,v in expected.items()}
   checks['G']=compare(raw['thermal_tsv']['saturation_gain'],actual['G'])
   diagnosis=json.loads((root/(case+'-diagnosis-09.json')).read_text())['resources']['physical_floor_a150']
   cell_ok=diagnosis['extra']==diagnosis['missing']==diagnosis['different']==diagnosis['capdiff']==0
   rows.append({'case':case,'checks':checks,'physical_resources':diagnosis,'sparse_overflow_map_exact':cell_ok,'pass':all(v['pass'] for v in checks.values()) and cell_ok})
 return {'status':'PASS' if len(rows)==3 and all(r['pass'] for r in rows) else 'FAIL','formal_optimization_experiments_allowed':len(rows)==3 and all(r['pass'] for r in rows),'official_certification':False,'official_archive_sha256':hashlib.sha256(Path(official_zip).read_bytes()).hexdigest(),'scope':'same frozen Guides; source input byte hashes unavailable','rows':rows}

def main():
 p=argparse.ArgumentParser(allow_abbrev=False);p.add_argument('--official-zip',type=Path,required=True);p.add_argument('--measurement-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 result=evaluate(a.official_zip,a.measurement_root)
 with a.output.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps({'status':result['status'],'formal_optimization_experiments_allowed':result['formal_optimization_experiments_allowed'],'cases':[{'case':r['case'],'failed':[k for k,v in r['checks'].items() if not v['pass']],'map_exact':r['sparse_overflow_map_exact']} for r in result['rows']]}))
 return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
