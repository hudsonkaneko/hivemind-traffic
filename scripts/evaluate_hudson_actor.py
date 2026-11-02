"""Reconstruct and evaluate the pinned v02 actor without a training process.

Default check never loads a model. Missing/wrong weights fail closed; native-check
only verifies the original environment/evaluator plumbing on development seeds.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.audit_hudson_checkpoint import EVIDENCE, REPORT_COMMIT, checkpoint_identity, git, recover_source_bytes


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def prepare(output, checkpoint=None):
    output=Path(output).resolve();output.mkdir(parents=True, exist_ok=False)
    raw=git('show', f'{REPORT_COMMIT}:{EVIDENCE}');evidence=json.loads(raw)
    (output/'published-evidence.json').write_bytes(raw)
    for name, expected in evidence['source_sha256'].items():
        data, recovery=recover_source_bytes(git('show', f"{evidence['source_commit']}:{name}"), expected)
        if hashlib.sha256(data).hexdigest()!=expected:
            raise ValueError('Recorded source cannot be reconstructed: '+name)
        target=output/'source'/name;target.parent.mkdir(parents=True, exist_ok=True);target.write_bytes(data)
    config=json.loads((output/'source/experiments/configs/sumo_mappo_v2.json').read_text())
    digest=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    if digest!=evidence['config_sha256']:
        raise ValueError('v02 configuration digest mismatch')
    checkpoint=Path(checkpoint).resolve() if checkpoint else ROOT/evidence['policy_path']
    identity=checkpoint_identity(checkpoint,evidence['policy_sha256'])
    if identity['matches_published_actor']:
        shutil.copyfile(checkpoint,output/'policy.pt')
        if sha(output/'policy.pt')!=evidence['policy_sha256']:
            raise ValueError('Checkpoint changed during copy')
    shutil.copyfile(ROOT/'scripts/hudson_actor_worker.py',output/'source/frozen_actor_driver.py')
    plan=dict(study_id='hudson-v02-frozen-reproduction',
        question='Can the frozen v02 actor be evaluated with its original sources/configuration, without retraining?',
        trigger='Sam forwarded Hudson confirmation and requested continuation in a new agent on 28 October 2026.',
        expected_policy_sha256=evidence['policy_sha256'],checkpoint=identity,
        report_commit=REPORT_COMMIT,source_commit=evidence['source_commit'],
        source_sha256=evidence['source_sha256'],config_sha256=digest,training=False,optimizer_steps=0,
        observations='Original 27-feature v02 environment; not i110_graph_v1',
        seeds=dict(smoke=config['preflight']['seeds'][:1],held_out=config['evaluation']['test_seeds']),
        limits=['No I-110 transfer or traffic benefit follows from original-fixture acceptance.',
                'Numerical reproduction needs original per-episode evaluation.json; public aggregate alone is insufficient.'])
    write(output/'plan.json',plan)
    return output,plan


def run(output, checkpoint=None, mode='check', suite='smoke', wall_seconds=180):
    if mode not in {'check','native-check','evaluate'} or suite not in {'smoke','held-out'}:
        raise ValueError('Unknown mode or suite')
    if not isinstance(wall_seconds,int) or not 1<=wall_seconds<=1800:
        raise ValueError('Use a bounded wall budget of 1..1800 seconds')
    if mode!='evaluate' and suite!='smoke':
        raise ValueError('Only explicit actor evaluation may use the fixed held-out seed set')
    output,plan=prepare(output,checkpoint)
    manifest=dict(started_utc=datetime.now(timezone.utc).isoformat(),status='prepared',mode=mode,suite=suite,
        training=False,optimizer_steps=0,actor_loaded=False,
        git_commit=git('rev-parse','HEAD').decode().strip(),git_status=git('status','--short').decode(),
        supervisor_sha256=sha(__file__),driver_sha256=sha(output/'source/frozen_actor_driver.py'),
        command=[sys.executable,*sys.argv],plan_sha256=sha(output/'plan.json'))
    (output/'source.diff').write_bytes(git('diff','HEAD'))
    shutil.copyfile(__file__,output/'supervisor.py')
    if mode in {'check','evaluate'} and not plan['checkpoint']['matches_published_actor']:
        manifest.update(status='blocked_missing_or_wrong_checkpoint',reason='No artifact loaded; exact published SHA256 required.')
        write(output/'manifest.json',manifest)
        return manifest
    if mode=='check':
        manifest.update(status='identity_verified_not_evaluated')
    else:
        command=[sys.executable,'-I',str(output/'source/frozen_actor_driver.py'),mode,suite,str(wall_seconds)]
        manifest['child_command']=command;write(output/'manifest.json',manifest)
        with (output/'process.log').open('x') as log:
            process=subprocess.Popen(command,cwd=output/'source',stdout=log,stderr=subprocess.STDOUT,
                start_new_session=os.name!='nt')
            try:
                code=process.wait(timeout=wall_seconds+20)
            except subprocess.TimeoutExpired:
                if os.name=='nt':
                    subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,timeout=15)
                else:
                    import signal
                    os.killpg(process.pid,signal.SIGKILL)
                process.wait(timeout=15);code=124
        manifest.update(status='completed' if code==0 else 'failed',exit_code=code)
        if (output/'runtime.json').exists():
            manifest['runtime']=json.loads((output/'runtime.json').read_text())
            manifest['actor_loaded']=manifest['runtime']['actor_loaded']
        if code==0:
            result=json.loads((output/'result.json').read_text())
            manifest['original_acceptance']=result['original_acceptance']
            if mode=='evaluate' and not result['original_acceptance']:
                manifest['status']='evaluated_acceptance_failed'
    manifest['finished_utc']=datetime.now(timezone.utc).isoformat()
    manifest['outputs']={p.relative_to(output).as_posix():sha(p) for p in output.rglob('*')
        if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
    write(output/'manifest.json',manifest)
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path)
    p.add_argument('--mode',choices=['check','native-check','evaluate'],default='check')
    p.add_argument('--suite',choices=['smoke','held-out'],default='smoke')
    p.add_argument('--wall-seconds',type=int,default=180)
    result=run(**vars(p.parse_args()));print(json.dumps({k:result[k] for k in ('status','training','actor_loaded')}))
    raise SystemExit(0 if result['status'] in {'completed','identity_verified_not_evaluated'} else 2)
