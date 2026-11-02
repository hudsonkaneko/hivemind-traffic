"""Isolated frozen-actor driver, copied beside the exact recovered source tree."""
from __future__ import annotations

import gzip
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import sys
import time
from types import SimpleNamespace


class CheckedActor:
    """Reject ABI/nonfinite/output violations, preserving the actor's numbers."""
    def __init__(self, actor):
        self.actor=actor

    def __call__(self, observations):
        import torch
        if observations.dtype!=torch.float32 or observations.ndim!=2 or observations.shape[1]!=27:
            raise ValueError('Frozen v02 requires float32 [N,27] observations')
        if not torch.isfinite(observations).all():
            raise ValueError('Nonfinite actor observations')
        with torch.no_grad():
            actions=self.actor(observations)
        if not isinstance(actions,torch.Tensor) or actions.shape!=(observations.shape[0],1):
            raise ValueError('Frozen v02 requires [N,1] actions')
        if actions.dtype!=torch.float32 or not torch.isfinite(actions).all() or (actions.abs()>1).any():
            raise ValueError('Actor output must be finite float32 normalized acceleration in [-1,1]')
        return actions


def main():
    source=Path(__file__).resolve().parent;output=source.parent
    sys.path.insert(0,str(source))
    plan=json.loads((output/'plan.json').read_text())
    mode,suite,wall=sys.argv[1:];deadline=time.monotonic()+int(wall)
    for name,expected in plan['source_sha256'].items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('Pinned source changed: '+name)
    # These imports resolve exclusively to the isolated source tree, not Sam's
    # current checkout. The original config, evaluator and safety filters remain.
    from experiments.sumo_mappo_support import load_config,config_digest,atomic_json
    from experiments.run_sumo_mappo import evaluate,seed_runtime
    for name in ('experiments.sumo_mappo_support','experiments.run_sumo_mappo'):
        if not Path(sys.modules[name].__file__).resolve().is_relative_to(source):
            raise ValueError('Imported evaluator escaped pinned source tree')
    config=load_config(source/'experiments/configs/sumo_mappo_v2.json')
    if config_digest(config)!=plan['config_sha256']:
        raise ValueError('Configuration changed')
    seed_runtime(config)
    import torch
    from traffic.sumo_backend import find_sumo_binary
    import subprocess
    runtime=dict(python=platform.python_version(),platform=platform.platform(),
        versions={name:importlib.metadata.version(name) for name in ('torch','numpy','gymnasium','pettingzoo','traci')},
        sumo=subprocess.check_output([str(find_sumo_binary()),'--version'],text=True).splitlines()[0],
        device='cpu',torch_threads=torch.get_num_threads(),deterministic=torch.are_deterministic_algorithms_enabled(),
        actor_loaded=False,training=False,optimizer_steps=0,
        metric_scope='Original fixed-horizon controlled-agent evaluation, not whole-network trip completion')
    published=json.loads((output/'published-evidence.json').read_text())
    expected=dict(published['versions'],python='3.11.9')
    actual=dict(runtime['versions'],python=runtime['python'],sumo=runtime['sumo'])
    runtime['reported_runtime_differences']={key:dict(reported=value,actual=actual.get(key))
        for key,value in expected.items() if actual.get(key)!=value}
    if runtime['versions']['traci']!='1.27.1' or ' 1.27.1' not in runtime['sumo']:
        raise ValueError('Pinned SUMO and TraCI 1.27.1 required')
    model=None;kind='native'
    if mode=='evaluate':
        policy=output/'policy.pt'
        if hashlib.sha256(policy.read_bytes()).hexdigest()!=plan['expected_policy_sha256']:
            raise ValueError('Actor changed before load')
        model=SimpleNamespace(actor=CheckedActor(torch.jit.load(str(policy),map_location='cpu').eval()))
        kind='learned';runtime['actor_loaded']=True
    elif mode!='native-check':
        raise ValueError('Unknown child mode')
    atomic_json(output/'runtime.json',runtime)
    seeds=plan['seeds']['held_out' if suite=='held-out' else 'smoke']
    # The unchanged original backend requires runs beneath its own ROOT/outputs.
    # Keep this constraint inside the isolated source bundle instead of patching it.
    evaluation_output=source/'outputs'
    with gzip.open(output/'actions.jsonl.gz','wt',encoding='utf-8') as journal:
        result=evaluate(config,kind,seeds,evaluation_output,output/'logs','evaluation',model=model,deadline=deadline,journal=journal)
        repeated=None
        if suite=='smoke':
            repeat=evaluate(config,kind,seeds,evaluation_output,output/'logs','repeat',model=model,deadline=deadline,journal=journal)
            repeated=result['episodes']==repeat['episodes']
            if not repeated:
                raise ValueError('Same-seed episode metrics changed')
    changed=[name for name,digest in plan['source_sha256'].items() if hashlib.sha256((source/name).read_bytes()).hexdigest()!=digest]
    if changed:
        raise ValueError('Pinned experiment source mutated: '+repr(changed))
    atomic_json(output/'result.json',dict(mode=mode,suite=suite,seeds=seeds,repeat_episode_metrics_equal=repeated,
        evaluation_file='source/outputs/evaluation/evaluation.json',
        original_acceptance=result['summary']['accepted'],summary=result['summary'],source_unchanged=True,
        reported_runtime_differences=runtime['reported_runtime_differences'],
        interpretation='Native scaffold verification only; no learned actor evaluated' if mode=='native-check'
            else 'Frozen actor on original v02 fixture; exact numerical reproduction unestablished without original per-episode reference'))


if __name__=='__main__':
    main()
