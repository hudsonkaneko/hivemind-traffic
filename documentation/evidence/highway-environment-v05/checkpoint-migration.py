"""Explicit pre-learning repair migration; refuses any optimized/collected run."""
from pathlib import Path
import hashlib,json,time,torch
from copy import deepcopy
from experiments.run_highway_environment_v05 import sources,readiness_check,preparation_digest
from experiments.sumo_mappo_support import config_digest
if not __debug__:
    raise RuntimeError('Recovery guards require normal Python mode')
ROOT=Path.cwd().resolve()
assert ROOT==Path(r'C:\Users\hudso\Documents\highwaysim')
old_path=ROOT/'outputs/highway-environment/v05/train-20261106T022222Z-0b8b611a/last.pt'
config_path=ROOT/'outputs/highway-environment/v05/authorized-config-recovery.json'
out=ROOT/'outputs/highway-environment/v05/recovery-departure-precision'
assert hashlib.sha256(old_path.read_bytes()).hexdigest()=='e4769d81655c323dfc6eaa261719704dbc27eeea7feab953b0764d6fcae03b12'
p=torch.load(old_path,map_location='cpu',weights_only=False)
cfg=json.loads(config_path.read_text())
readiness_check(cfg)
assert preparation_digest(cfg)==preparation_digest(p['config'])
state=p['state']
for key in ('spent','actual','critic_actual','ppo_actual','updates','optimizer_steps','critic_optimizer_steps','episode_index'):
    assert state[key]==0,(key,state[key])
assert state['pending'] is None and state['run_closed'] is True and state['initial_complete'] is False
assert state['final_complete'] is False
assert not p['optimizer']['state']
new_sources=sources()
changed={key for key,value in p['source_hashes'].items() if new_sources.get(key)!=value}
assert set(new_sources)==set(p['source_hashes'])
assert changed=={'traffic/highway_v05_demand.py','environments/highway_environment_v05.py'},changed
assert state['elapsed_seconds']>=125.343
original=deepcopy(p)
p.update(config=cfg,config_sha256=config_digest(cfg),source_hashes=new_sources,saved_at_unix=time.time())
target=out/'last.pt'
assert not target.exists()
torch.save(p,target)
loaded=torch.load(target,map_location='cpu',weights_only=False)
assert loaded['state']==original['state']
assert loaded['optimizer']==original['optimizer']
assert all(torch.equal(value,loaded['model'][key]) for key,value in original['model'].items())
assert torch.equal(loaded['torch_rng'],original['torch_rng'])
assert loaded['python_rng']==original['python_rng']
assert (loaded['numpy_rng'][1]==original['numpy_rng'][1]).all()
assert loaded['numpy_rng'][0]==original['numpy_rng'][0]
assert loaded['numpy_rng'][2:]==original['numpy_rng'][2:]
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
evidence=dict(reason='Pre-learning departure precision and interrupted-maneuver lifecycle repair; original checkpoint untouched',
    original_checkpoint=str(old_path.relative_to(ROOT)),original_sha256=sha(old_path),
    migrated_checkpoint=str(target.relative_to(ROOT)),migrated_sha256=sha(target),
    changed_sources={key:dict(before=original['source_hashes'][key],after=new_sources[key]) for key in changed},
    model_optimizer_rng_unchanged=True,counters_unchanged=True,charged_seconds=state['elapsed_seconds'],
    original_config_sha256=original['config_sha256'],new_config_sha256=p['config_sha256'],
    total_wall_budget_seconds=cfg['training']['max_wall_seconds'],training_decisions=0,
    readiness=cfg['readiness'],held_out_test_run=False)
(out/'migration.json').write_text(json.dumps(evidence,indent=2)+'\n')
print(json.dumps(evidence,indent=2))
