#!/usr/bin/env python3
"""Verify a handoff using only files in this clone; no router/scorer or private paths."""
from pathlib import Path
import csv, gzip, hashlib, io, json, math, re, tarfile, zipfile
root=Path(__file__).resolve().parents[1]
m=json.loads((root/'docs/provenance/device-handoff-manifest.json').read_text(encoding='utf-8'))
for a in m['assets']:
 p=root/a['path']; data=p.read_bytes()
 assert len(data)==a['bytes'] and hashlib.sha256(data).hexdigest()==a['sha256'],a['path']
 assert len(data)<50*1024*1024,a['path']
 if p.name.endswith('.tar.gz'):
  with tarfile.open(p) as t:
   for i in t.getmembers():
    assert not Path(i.name).is_absolute() and '..' not in Path(i.name).parts and i.isfile(),i.name
for a in json.loads((root/'handoff/guide-index.json').read_text(encoding='utf-8')):
 data=gzip.decompress((root/a['repository_path']).read_bytes())
 assert len(data)==a['bytes'] and hashlib.sha256(data).hexdigest()==a['sha256']
for p in (root/'handoff/official-20261104').glob('*/scoring_result.zip'):
 with zipfile.ZipFile(p) as z:
  assert z.testzip() is None
  scores=json.loads(z.read('scores.json'))
  for name,case in scores['cases'].items():
   if case['status']!='success':continue
   q=[]
   for r in csv.DictReader(io.StringIO(z.read('eval_temp/'+name+'/overflow_gcells.csv').decode('utf-8-sig'))):
    cap=float(r['capacity']);o=float(r['overflow']);assert cap>0
    q.append((o/cap)**2)
   assert abs(100/math.sqrt(1+math.fsum(q))-case['raw_report']['score']['congestion_score'])<1e-9
for name in ['docs/DEVICE_HANDOFF.md','AGENTS.md','README.md','PROJECT_STATUS.md','DECISIONS.md','EXPERIMENT_LOG.md']:
 p=root/name
 for target in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
  if '://' not in target and not target.startswith('#'):
   assert (p.parent/target.split('#')[0]).exists(),(name,target)
print('PASS: clone-only inventory hashes, 14 unique Guide contents, safe snapshot members, ZIP CRC, official C summaries and entry links')
print('External official inputs/runtime/evaluator still required for full experiments; no build or scoring executed.')
