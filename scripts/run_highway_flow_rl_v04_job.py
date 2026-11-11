"""Durable unattended v04 launch; wait for exit without progress polling."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    if Path.cwd().resolve() != ROOT.resolve():
        raise ValueError('Launch from the canonical checkout')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--preflight',required=True)
    args=parser.parse_args()
    out=(ROOT/args.output).resolve()
    if not out.is_relative_to(ROOT/'outputs/highway-flow-rl/v04') or out.exists():
        raise ValueError('Use a fresh v04 run directory')
    name=out.name
    status_path=ROOT/'logs'/f'highway-flow-rl-v04-{name}.status.json'
    stdout_path=ROOT/'logs'/f'highway-flow-rl-v04-{name}.stdout.log'
    stderr_path=ROOT/'logs'/f'highway-flow-rl-v04-{name}.stderr.log'
    if any(p.exists() for p in (status_path,stdout_path,stderr_path)):
        raise ValueError('Preserve existing run logs')
    command=[sys.executable,'-u','experiments/run_highway_flow_rl_v04.py','train',
        '--config','configs/training/highway-flow-rl-v04-long.json',
        '--preflight',args.preflight,'--output',args.output]
    branch='highway-merge-rewards-v04'
    commit=subprocess.check_output(['git','rev-parse',branch],cwd=ROOT,text=True).strip()
    proof=json.loads((ROOT/args.preflight).read_text())
    for relative in [*proof['source_hashes'],'configs/training/highway-flow-rl-v04-long.json']:
        published=subprocess.check_output(['git','show',commit+':'+relative],cwd=ROOT)
        local=(ROOT/relative).read_bytes()
        if published.replace(b'\r\n',b'\n') != local.replace(b'\r\n',b'\n'):
            raise ValueError('Published source differs from launch workspace: '+relative)
    status=dict(schema='highway_flow_v04_job_v1',status='starting',command=command,
        output=str(out),started_unix=time.time(),publication_commit=commit,
        launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    def save():
        temporary=status_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(status,indent=2)+'\n',encoding='utf-8')
        temporary.replace(status_path)
    save()
    try:
        with stdout_path.open('x',encoding='utf-8') as stdout,stderr_path.open('x',encoding='utf-8') as stderr:
            process=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=stderr)
            status.update(status='running',pid=process.pid);save()
            code=process.wait()  # OS wait only: no metric polling or assistant calls.
        status.update(status='complete' if code==0 else 'failed',exit_code=code,finished_unix=time.time())
        save()
        return code
    except BaseException as error:
        status.update(status='launcher_failed',error=repr(error),finished_unix=time.time());save();raise


if __name__=='__main__':
    raise SystemExit(main())
