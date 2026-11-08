"""Collect matched training-only frozen batches and inspect losses; never learn."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))

import torch
from experiments import run_highway_smooth_control as runner
from experiments import run_highway_environment_v05 as old
from scripts.prepare_highway_smooth_control import rows, STUDY, sha


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=runner.DEFAULT)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-wall-seconds',type=float,default=90.)
    args=parser.parse_args()
    output=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not output.is_relative_to(STUDY) or output.exists():
        raise ValueError('Fresh canonical study output required')
    if not 0<args.max_wall_seconds<=180:
        raise ValueError('Gradient preflight has at most 180 seconds')
    cfg=runner.load_config(args.config)
    if cfg['training_authorized']:
        raise ValueError('Use the disabled preparation configuration')
    row=rows(cfg['base_config'])['i110-merge']
    hashes=runner.sources()
    hashes['scripts/check_highway_smooth_gradients.py']=sha(__file__)
    ledger_path=STUDY/'preparation-ledger.json'
    ledger=json.loads(ledger_path.read_text())
    if any(a['status']=='running' for a in ledger['attempts']):
        raise ValueError('Native preparation must run serially')
    prior=sum(a['charged_wall_seconds'] for a in ledger['attempts'])
    if prior+args.max_wall_seconds>2700:
        raise ValueError('No preparation allowance remains before the evaluation reserve')
    output.mkdir(parents=True)
    for name,digest in hashes.items():
        target=output/'source'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
        if sha(target)!=digest:
            raise RuntimeError('Source changed during snapshot')
    old.atomic_json(output/'config.json',cfg)
    old.atomic_json(output/'manifest.json',dict(training=False,optimizer_steps=0,source_hashes=hashes,
        preparation_sha256=runner.preparation_digest(cfg),selected_case=row,
        selection='Fixed train seed6800019, at most512 active decisions per arm, stochastic frozen actor; no optimizer step',
        command=[sys.executable,*sys.argv]))
    entry=dict(output=output.relative_to(ROOT).as_posix(),status='running',charged_wall_seconds=args.max_wall_seconds,
        reserved_wall_seconds=args.max_wall_seconds,source_hashes=hashes)
    ledger['attempts'].append(entry)
    old.atomic_json(ledger_path,ledger)
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    started=time.monotonic()
    error=None
    result={}
    try:
        arms=runner.initialize_arms(cfg)
        before={name:deepcopy(dict(model=a['model'].state_dict(),optimizer=a['trainer'].optimizer.state_dict())) for name,a in arms.items()}
        batches={}
        for name,arm in arms.items():
            with runner.arm_rng(arm):
                report,batch=old.episode(runner.env_for(cfg,output/name,row,name),row,model=arm['model'],
                    collect=True,limit=512,deadline=started+args.max_wall_seconds)
            report.update(training=False,optimizer_steps=0,scope='Frozen stochastic train-only collection; partial episode for gradient inspection, not driving acceptance')
            old.atomic_json(output/(name+'.json'),report)
            if batch is None:
                raise ValueError('No gradient batch')
            batches[name]=batch
            after=dict(model=arm['model'].state_dict(),optimizer=arm['trainer'].optimizer.state_dict())
            if not runner.tree_equal(before[name],after):
                raise ValueError('Collection changed parameters or optimizer')
        first,second=(batches[name] for name in runner.ARMS)
        invariant_keys=('local_obs','global_states','agent_indices','actions','pre_tanh_actions','active','old_log_probs')
        compared={key:torch.equal(first[key],second[key]) for key in invariant_keys}
        if not compared or not all(compared.values()):
            raise ValueError('Matched frozen collection changed actions/observations')
        torch.save(batches,output/'batches.pt')
        result=runner.frozen_loss_check(cfg,batches,arms)
        result.update(collection_invariance=compared,batches_sha256=sha(output/'batches.pt'))
        changed=[name for name,digest in hashes.items() if sha(ROOT/name)!=digest]
        if changed:
            raise ValueError('Sources changed during frozen check: '+str(changed))
    except Exception:
        error=traceback.format_exc()
    finally:
        elapsed=time.monotonic()-started
        entry.update(status='failed' if error else 'complete',charged_wall_seconds=elapsed)
        old.atomic_json(ledger_path,ledger)
        result.update(passed=error is None,training=False,optimizer_steps=0,error=error,
            wall_seconds=elapsed,source_hashes=hashes,preparation_sha256=runner.preparation_digest(cfg),
            preparation_total_wall_seconds=prior+elapsed)
        old.atomic_json(output/'summary.json',result)
    print(json.dumps(dict(passed=error is None,error=error,wall_seconds=elapsed)))
    return int(error is not None)


if __name__=='__main__':
    raise SystemExit(main())
