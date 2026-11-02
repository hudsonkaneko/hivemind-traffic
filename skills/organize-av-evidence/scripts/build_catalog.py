"""Build a local, read-only research inventory and causal-explanation catalog."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
from urllib.parse import urlparse

EXCLUDED={'.git','.venv','.tools','node_modules','runtimes','__pycache__','.pytest_cache',
    'nv_shadercache','shadercache','migration_archive','.venv-traffic','.venv-isaac7','.venv-ovrtx'}
IMAGES={'.png','.jpg','.jpeg','.gif','.webp','.svg','.bmp','.tif','.tiff'}
RAW={'.laz','.las','.npy','.npz','.zip','.7z','.parquet','.feather','.bin'}


def resolve(ref,roots):
    if ref.startswith(('https://','http://')):return None
    alias,sep,relative=ref.partition(':')
    if not sep or alias not in roots:raise ValueError('Unknown reference root: '+ref)
    root=roots[alias].resolve();path=(root/relative).resolve()
    if not path.is_relative_to(root):raise ValueError('Reference escapes declared root: '+ref)
    return path


def uri(ref,roots):
    path=resolve(ref,roots)
    return ref if path is None else path.as_uri()


def kind(path):
    if path.suffix.lower() in IMAGES:return 'graphic'
    if path.suffix.lower() in {'.mp4','.webm','.mov'}:return 'video'
    if path.suffix.lower() in RAW:return 'raw/source data'
    if path.suffix.lower() in {'.py','.ps1','.cmd','.js','.ts','.cpp'}:return 'code'
    if path.suffix.lower() in {'.md','.pdf','.docx','.pptx','.html'}:return 'document'
    if any(w in path.name.lower() for w in ('metrics','result','summary','evidence','verification','test')):return 'result/report'
    if any(w in path.name.lower() for w in ('trace','recording','actions')):return 'trajectory/log'
    if any(w in path.name.lower() for w in ('config','manifest','plan')):return 'configuration/manifest'
    if path.suffix.lower() in {'.usd','.usda','.usdc'}:return 'scene/asset'
    return 'data/metadata'


def validate(registry,roots):
    if registry.get('schema_version')!=1:raise ValueError('Unsupported catalog schema')
    ids=set();errors=[];artifacts=set()
    for activity in registry['activities']:
        for key in ('id','title','domain','question','why_now','trigger','method','status','result','limits','next_action','references','prefixes'):
            if key not in activity or not activity[key]:raise ValueError(f'{activity.get("id")}: missing {key}')
        if activity['id'] in ids:raise ValueError('Duplicate activity ID')
        ids.add(activity['id'])
        for ref in activity['references']+activity['trigger']['sources']:
            path=resolve(ref,roots)
            if path is not None and not path.exists():errors.append(dict(activity=activity['id'],reference=ref,error='missing_reference'))
        for artifact in activity.get('artifacts',[]):
            if artifact['path'] in artifacts:raise ValueError('Duplicate explicit artifact: '+artifact['path'])
            artifacts.add(artifact['path'])
            for key in ('path','generator','inputs','explanation','claim','limits','lineage_basis'):
                if not artifact.get(key):raise ValueError('Incomplete artifact record: '+artifact['path']+' '+key)
            for ref in [artifact['path'],artifact['generator'],*artifact['inputs']]:
                path=resolve(ref,roots)
                if path is not None and not path.exists():errors.append(dict(activity=activity['id'],reference=ref,error='missing_artifact_input'))
    for activity in registry['activities']:
        for parent in activity.get('follow_up_to',[])+activity.get('supersedes',[]):
            if parent not in ids:raise ValueError('Unknown parent activity: '+parent)
    return errors


def scan(registry,roots,output):
    explicit={a['path']:(activity,a) for activity in registry['activities'] for a in activity.get('artifacts',[])}
    prefixes=sorted(((prefix,activity) for activity in registry['activities'] for prefix in activity['prefixes']),key=lambda p:len(p[0]),reverse=True)
    files={};issues=[];skipped=Counter();manifest_cache={}
    scope=registry['scope']
    for alias,relative_roots in scope['roots'].items():
        base=roots[alias]
        for relative in relative_roots:
            start=(base/relative).resolve()
            if not start.is_relative_to(base):raise ValueError('Scan root escapes project')
            if not start.exists():issues.append(dict(path=f'{alias}:{relative}',error='scan_root_missing'));continue
            iterator=[(start.parent,[],[start.name])] if start.is_file() else os.walk(start,followlinks=False)
            for directory,dirs,names in iterator:
                directory=Path(directory)
                kept=[]
                for d in dirs:
                    q=directory/d
                    if d in EXCLUDED or q.is_symlink() or q.resolve().is_relative_to(output) or (alias=='project' and q.resolve()==roots['repo']) or 'outputs/evidence-catalog' in q.as_posix():
                        skipped['excluded_directories']+=1
                    else:kept.append(d)
                dirs[:]=kept
                for name in sorted(names):
                    path=directory/name
                    if path.is_symlink() or path.suffix in {'.pyc','.tmp'}:continue
                    if alias=='project' and path.resolve().is_relative_to(roots['repo']):continue
                    if path.resolve().is_relative_to(output):continue
                    ref=alias+':'+path.relative_to(base).as_posix()
                    if ref in files:continue
                    try:stat=path.stat()
                    except OSError as exc:issues.append(dict(path=ref,error=str(exc)));continue
                    activity,declared=explicit.get(ref,(None,None))
                    if activity is None:
                        activity=next((a for prefix,a in prefixes if ref==prefix.rstrip('/') or ref.startswith(prefix.rstrip('/')+'/')),None)
                    record=dict(id=hashlib.sha256(ref.encode()).hexdigest()[:16],ref=ref,path=path.relative_to(base).as_posix(),
                        root=alias,uri=path.as_uri(),kind=kind(path),bytes=stat.st_size,mtime_ns=stat.st_mtime_ns,
                        activity=activity['id'] if activity else None,domain=activity['domain'] if activity else 'Unassigned',
                        provenance='explicit_lineage' if declared else 'context_linked' if activity else 'needs_backfill',
                        sha256=None,verification='metadata_only',generator=None,inputs=[],explanation=None,claim=None,limits=None)
                    # Protected research inputs are inventoried by metadata only.
                    # Hash small generated/repository files, never raw observations.
                    hash_kinds={'graphic','code','document','result/report','configuration/manifest','scene/asset'}
                    if alias=='repo' and record['kind'] in hash_kinds and stat.st_size<=8*1024*1024:
                        try:record.update(sha256=hashlib.sha256(path.read_bytes()).hexdigest(),verification='content_hash_computed')
                        except OSError as exc:issues.append(dict(path=ref,error=str(exc)))
                    if declared:
                        record.update({k:declared[k] for k in ('generator','inputs','explanation','claim','limits','lineage_basis')})
                        record['generator_uri']=uri(declared['generator'],roots)
                        record['input_uris']=[uri(ref,roots) for ref in declared['inputs']]
                    elif activity and alias=='repo' and path.is_relative_to(roots['repo']/'outputs'):
                        parent=path.parent
                        for _ in range(5):
                            m=parent/'manifest.json'
                            if m not in manifest_cache:
                                try:manifest_cache[m]=json.loads(m.read_text()) if m.is_file() and m.stat().st_size<8*1024*1024 else None
                                except (ValueError,OSError):manifest_cache[m]=None
                            manifest=manifest_cache[m]
                            if isinstance(manifest,dict) and isinstance(manifest.get('outputs'),dict):
                                key=path.relative_to(parent).as_posix()
                                expected=manifest['outputs'].get(key)
                                if isinstance(expected,str):
                                    record.update(provenance='manifest_linked',manifest='repo:'+m.relative_to(roots['repo']).as_posix(),recorded_sha256=expected)
                                    if record['sha256'] and record['sha256']!=expected:
                                        record['verification']='manifest_hash_mismatch'
                                        issues.append(dict(path=ref,error='manifest_hash_mismatch',manifest=record['manifest']))
                                    elif record['sha256']:record['verification']='manifest_hash_matches'
                                    break
                            if parent==roots['repo']/'outputs':break
                            parent=parent.parent
                    files[ref]=record
    # Explicit records outside scan roots remain visible and are not silently lost.
    for ref in explicit:
        if ref not in files:issues.append(dict(path=ref,error='explicit_artifact_not_in_inventory'))
    return sorted(files.values(),key=lambda r:r['ref']),issues,dict(skipped)


def render_html(path,registry,records,coverage,roots):
    esc=html.escape
    cards=[]
    for a in sorted(registry['activities'],key=lambda a:not a.get('current',False)):
        links=' · '.join(f'<a href="{esc(uri(ref,roots),quote=True)}">{esc(ref)}</a>' for ref in a['references'])
        opened=' open' if a.get('current') else ''
        cards.append(f'<article id="{esc(a["id"])}"><details{opened}><summary><div class="tag">{esc(a["domain"])} · {esc(a["status"])}</div><h2>{esc(a["title"])}</h2></summary>'
            +''.join(f'<p><strong>{label}</strong> {esc(str(a[key]))}</p>' for key,label in [('question','Question:'),('why_now','Why now:'),('method','What we did:'),('result','Evidence / result:'),('limits','Limits:'),('next_action','Next:')])
            +f'<p><strong>Trigger:</strong> {esc(a["trigger"]["text"])}</p><small>{links}</small></details></article>')
    payload=json.dumps(records,ensure_ascii=False).replace('<','\\u003c')
    counts=esc(json.dumps(coverage['by_provenance'],ensure_ascii=False))
    content='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AV Research Evidence Catalog</title><style>
body{font:16px/1.5 system-ui,sans-serif;background:#f3f6fa;color:#142335;margin:0}main{max-width:1400px;margin:auto;padding:32px}
h1{font-size:34px;margin:0}h2{font-size:21px}header,p{max-width:1100px}.flow{padding:18px;background:#142f49;color:white;border-radius:10px;margin:22px 0}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:16px}article{background:white;border:1px solid #ced9e5;border-radius:10px;padding:22px}article p{margin:10px 0}.tag{color:#42637d;font-size:13px;text-transform:uppercase}
a{color:#0869a4;overflow-wrap:anywhere}small{font-size:12px}input,select,button{font:inherit;padding:9px;margin:6px;border:1px solid #aab9c8;border-radius:5px}input{width:45%}
table{width:100%;border-collapse:collapse;background:white;font-size:13px}th,td{text-align:left;vertical-align:top;padding:10px;border-bottom:1px solid #dce4ee;overflow-wrap:anywhere}th{background:#e4edf5}td:first-child{max-width:450px}.note{border-left:4px solid #d18a24;padding:12px;background:#fff6e8}
</style><main><header><h1>AV research: questions to evidence</h1><p>One local index of what we did, why we did it, and what each artifact can support. Source packages stay at their original stable paths.</p></header>
<div class="flow">Question → Trigger / reason → Method and inputs → Run → Graphics and results → Interpretation → Next decision</div>
<p class="note">Context grouping is not complete derivation. Exact lineage is declared separately; unresolved historical relationships remain visible. This catalog is local and includes links to local/private artifacts. It is not a publication or backup.</p>
<p>Inventory: COUNT files. Coverage: COUNTS. <a href="#artifacts">Find an artifact</a> · <a href="coverage.json">Coverage and gaps</a> · <a href="inventory.json">Machine-readable inventory</a> · <a href="index.md">Markdown index</a></p>
<div class="cards">CARDS</div><h2 id="artifacts">Find an artifact</h2><input id="search" placeholder="Search a filename, activity, explanation or result"><select id="kind"><option value="">All types</option></select><select id="coverage"><option value="">All provenance levels</option></select>
<p id="status"></p><button id="prev">Previous 100</button><button id="next">Next 100</button><table><thead><tr><th>Artifact</th><th>Activity / type</th><th>Provenance</th><th>Explanation and derivation</th></tr></thead><tbody id="rows"></tbody></table>
<script id="data" type="application/json">PAYLOAD</script><script>
const data=JSON.parse(document.getElementById('data').textContent);let page=0;
const by=id=>document.getElementById(id);
for(const [id,key] of [['kind','kind'],['coverage','provenance']])for(const value of [...new Set(data.map(r=>r[key]))].sort()){const o=document.createElement('option');o.value=value;o.textContent=value;by(id).append(o)}
function addLink(parent,label,url){const a=document.createElement('a');a.textContent=label;a.href=url;parent.append(a)}
function render(){const query=by('search').value.toLowerCase();const matches=data.filter(r=>(!by('kind').value||r.kind===by('kind').value)&&(!by('coverage').value||r.provenance===by('coverage').value)&&JSON.stringify(r).toLowerCase().includes(query));page=Math.max(0,Math.min(page,Math.ceil(matches.length/100)-1));by('status').textContent=`${matches.length} matching artifacts; page ${page+1}`;by('rows').replaceChildren();for(const r of matches.slice(page*100,page*100+100)){const row=document.createElement('tr');let c=document.createElement('td');addLink(c,r.ref,r.uri);row.append(c);c=document.createElement('td');c.textContent=r.kind+' · ';if(r.activity)addLink(c,r.activity,'#'+r.activity);else c.append('Unassigned');row.append(c);c=document.createElement('td');c.textContent=r.provenance+' / '+r.verification;row.append(c);c=document.createElement('td');c.textContent=r.explanation||'Exact artifact derivation has not yet been curated.';if(r.generator){c.append(' Generator: ');addLink(c,r.generator,r.generator_uri)}if(r.inputs.length){c.append(' Inputs: ');r.inputs.forEach((input,i)=>{if(i)c.append(' · ');addLink(c,input,r.input_uris[i])})}if(r.claim)c.append(' Claim: '+r.claim);if(r.limits)c.append(' Limits: '+r.limits);row.append(c);by('rows').append(row)}}
for(const id of ['search','kind','coverage'])by(id).addEventListener('input',()=>{page=0;render()});by('prev').onclick=()=>{page--;render()};by('next').onclick=()=>{page++;render()};render();
</script></main></html>'''
    content=content.replace('COUNT files',str(len(records))+' files').replace('COUNTS',counts).replace('CARDS',''.join(cards)).replace('PAYLOAD',payload)
    path.write_text(content,encoding='utf-8')


def build(project,repo,output,registry=None):
    roots={'project':Path(project).resolve(),'repo':Path(repo).resolve()}
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    registry=Path(registry) if registry else roots['repo']/'documentation/evidence-catalog/activities.json'
    data=json.loads(registry.read_text(encoding='utf-8'))
    errors=validate(data,roots)
    records,issues,skipped=scan(data,roots,output)
    coverage=dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),scope=data['scope'],
        files=len(records),by_kind=dict(Counter(r['kind'] for r in records)),
        by_provenance=dict(Counter(r['provenance'] for r in records)),
        by_activity=dict(Counter(r['activity'] or 'unassigned' for r in records)),
        validation_errors=errors,inventory_issues=issues,skipped=skipped,
        historical_backfill_complete=not any(r['provenance']!='explicit_lineage' for r in records if r['kind'] in {'graphic','video','result/report'}),
        registry_sha256=hashlib.sha256(registry.read_bytes()).hexdigest(),
        limits='Discovery/context association is not proof of causal derivation. Raw/project source content was not opened for inventory.')
    for name,value in [('registry-snapshot.json',data),('inventory.json',records),('coverage.json',coverage)]:
        (output/name).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    md=['# AV research evidence catalog','',f'Inventoried files: {len(records)}. See [coverage](coverage.json) and [searchable catalog](index.html).','',
        'Context links and explicit artifact derivation are distinct. Missing lineage stays visible.','']
    for a in data['activities']:
        md.extend([f'## {a["title"]} ({a["id"]})','',f'**Question:** {a["question"]}',f'**Why now:** {a["why_now"]}',
            f'**Method:** {a["method"]}',f'**Result:** {a["result"]}',f'**Limits:** {a["limits"]}',f'**Next:** {a["next_action"]}',''])
        for ref in a['references']:
            p=resolve(ref,roots);md.append(f'- [{ref}](<{p.as_posix() if p else ref}>)')
        md.append('')
    (output/'index.md').write_text('\n'.join(md),encoding='utf-8')
    render_html(output/'index.html',data,records,coverage,roots)
    print(json.dumps({k:coverage[k] for k in ('files','by_provenance','historical_backfill_complete','validation_errors')}))
    return coverage


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('project','repo','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--registry',type=Path)
    result=build(**vars(p.parse_args()))
    raise SystemExit(1 if result['validation_errors'] else 0)
