"""Matched train-only frozen loss/gradient inspection; no optimizer path."""
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
from experiments import run_highway_flow as runner
from experiments import run_highway_environment_v05 as old

STUDY=ROOT/'outputs/highway-flow/v01'
INVARIANT_KEYS=('local_obs','global_states','agent_indices','actions','pre_tanh_actions','active',
    'old_log_probs','teacher_actions','acceleration_supervision_eligible','lane_supervision_eligible')


def gradient_row(cfg):
    """Predeclared first dense-difficulty following row in the immutable train bank."""
    return next(row for row in runner.build_schedule(cfg)
        if row['case']['task']=='following' and row['difficulty']==2)


def verify_batches(first,second):
    compared={key:torch.equal(first[key],second[key]) for key in INVARIANT_KEYS}
    if not all(compared.values()):
        raise ValueError('Frozen paired observations/actions/qualified labels differ')
    return compared


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=runner.DEFAULT)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-wall-seconds',type=float,default=90.)
    args=parser.parse_args(argv)
    output=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not output.is_relative_to(STUDY) or output.exists():
        raise ValueError('Fresh canonical flow-study output required')
    if not 0<args.max_wall_seconds<=180:
        raise ValueError('Gradient preparation has at most180 seconds')
    cfg=runner.load_config(args.config)
    if cfg['training_authorized'] or cfg['initialization']['selected_id'] is None:
        raise ValueError('Disabled resolved configuration with frozen selected initializer required')
    ledger_path=runner.project_path(cfg['preparation_ledger'],outputs=True)
    ledger=json.loads(ledger_path.read_text(encoding='utf-8'))
    if ledger.get('schema')!='highway_flow_preparation_ledger_v01' or any(
            a['status'] not in ('complete','completed','failed') for a in ledger['attempts']):
        raise ValueError('Valid serial preparation ledger required')
    prior=sum(a['charged_wall_seconds'] for a in ledger['attempts'])
    if prior+args.max_wall_seconds>cfg['budget']['preparation_wall_seconds']:
        raise ValueError('Preparation1800s allowance would be exceeded')
    hashes=runner.sources(); row=gradient_row(cfg)
    entry=dict(output=output.relative_to(ROOT).as_posix(),status='running',charged_wall_seconds=args.max_wall_seconds,
        reserved_wall_seconds=args.max_wall_seconds,source_hashes=hashes)
    ledger['attempts'].append(entry); old.atomic_json(ledger_path,ledger)
    started=time.monotonic(); error=None; result={}; batches={}
    try:
        output.mkdir(parents=True)
        for name,checksum in hashes.items():
            target=output/'source'/name; target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(ROOT/name,target)
            if runner.file_digest(target)!=checksum:
                raise ValueError('Source changed during frozen snapshot')
        old.atomic_json(output/'config.json',cfg)
        old.atomic_json(output/'manifest.json',dict(training=False,optimizer_steps=0,source_hashes=hashes,
            preparation_sha256=runner.preparation_digest(cfg),selected_case=row,
            selection='First following/difficulty2 row in immutable training schedule, at most512 active decisions per arm; stochastic frozen actor; no driving acceptance from partial batch',
            command=[sys.executable,*sys.argv]))
        torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
        arms=runner.initialize_arms(cfg)
        outer_rng=runner.capture_rng()
        before={name:deepcopy(dict(model=a['model'].state_dict(),optimizer=a['trainer'].optimizer.state_dict())) for name,a in arms.items()}
        for name,arm in arms.items():
            old.check_deadline(started+args.max_wall_seconds)
            with runner.arm_rng(arm):
                report,batch=runner.episode(runner.env_for(cfg,output/name,row,name),row,
                    model=arm['model'],collect=True,limit=512,deadline=started+args.max_wall_seconds)
            report.update(training=False,optimizer_steps=0,
                scope='Frozen stochastic training-only partial collection for gradient inspection, not task acceptance')
            old.atomic_json(output/(name+'.json'),report)
            if batch is None:
                raise ValueError('No active gradient batch')
            batches[name]=batch
            torch.save(batches,output/'batches.pt')
            if not runner.tree_equal(before[name],dict(model=arm['model'].state_dict(),optimizer=arm['trainer'].optimizer.state_dict())):
                raise ValueError('Frozen collection changed model/optimizer')
        a,b=(arms[name] for name in runner.ARMS)
        if not runner.tree_equal(a['rng'],b['rng']) or not runner.tree_equal(outer_rng,runner.capture_rng()):
            raise ValueError('Matched collection RNG invariance failed')
        compared=verify_batches(*(batches[name] for name in runner.ARMS))
        result=runner.frozen_loss_check(cfg,batches,arms)
        result.update(collection_invariance=compared,batches_sha256=runner.file_digest(output/'batches.pt'),
            selected_initializer_id=cfg['initialization']['selected_id'])
        changed=[name for name,checksum in hashes.items() if runner.file_digest(ROOT/name)!=checksum]
        if changed:
            raise ValueError('Sources changed during frozen check: '+str(changed))
        old.check_deadline(started+args.max_wall_seconds)
    except Exception:
        error=traceback.format_exc()
    finally:
        elapsed=time.monotonic()-started
        entry.update(status='failed' if error else 'complete',charged_wall_seconds=elapsed)
        old.atomic_json(ledger_path,ledger)
        result.update(passed=error is None,training=False,optimizer_steps=0,error=error,wall_seconds=elapsed,
            source_hashes=hashes,preparation_sha256=runner.preparation_digest(cfg),
            preparation_total_wall_seconds=prior+elapsed)
        old.atomic_json(output/'summary.json',result)
    print(json.dumps(dict(passed=error is None,error=error,wall_seconds=elapsed)))
    return int(error is not None)


if __name__=='__main__':
    raise SystemExit(main())
