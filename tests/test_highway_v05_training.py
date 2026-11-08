"""V05 transfer, capacity, nominal acceptance and bounded-resume contracts."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from experiments import highway_v05_support as support
from policies.mappo import MappoActorCritic, MappoTrainer, export_actor


def model(width=99, count=24):
    return MappoActorCritic(observation_size=width,state_size=count*(width+1),num_agents=count,
        message_encoder=True,hybrid_actions=True,driving_local_size=width-48,
        driving_message_size=8,driving_mask_start=30,population_critic=True)


def test_actor_migration_preserves_old_features_and_zeros_only_new_columns(tmp_path):
    torch.manual_seed(19); old=model(93,8); new=model()
    support.migrate_actor(new,{'model':old.state_dict()})
    obs=torch.rand(31,93)*2-1
    obs[:,30:33]=torch.tensor([1.,1.,1.])
    obs[:,45:]=obs[:,45:].reshape(31,6,8).clamp(-1,1).reshape(31,48)
    new_obs=torch.cat((obs[:,:45],torch.randn(31,6),obs[:,45:]),dim=-1)
    assert torch.allclose(old.actor(obs),new.actor(new_obs),atol=1e-7,rtol=1e-6)
    assert not new.actor.mean_net[1][0].weight[:,45:51].count_nonzero()
    file=tmp_path/'actor.pt';export_actor(new,file)
    assert torch.equal(torch.jit.load(str(file))(new_obs),new.actor(new_obs))
    with pytest.raises(Exception):torch.jit.load(str(file))(obs)


def test_full_24_population_masked_critic_ignores_absent_rows():
    net=model()
    rows=torch.zeros(24,100); rows[:4,:99]=torch.rand(4,99); rows[:4,99]=1
    before=net.values(rows.reshape(1,-1),torch.tensor([2]))
    rows[4:,:99]=float('nan')
    after=net.values(rows.reshape(1,-1),torch.tensor([2]))
    assert torch.equal(before,after)
    assert net.values(rows.reshape(1,-1),torch.tensor([12])).item()==0


def test_near_limit_objective_uses_preserved_speed_fields_for_99_inputs():
    net=model(); trainer=MappoTrainer(net,near_limit_coef=.1)
    obs=torch.zeros(4,99);obs[:,30:33]=1;obs[:,0]=.5;obs[:,4]=.5
    labels=torch.tensor([[.3846154,0.]]).repeat(4,1)
    loss,stats=trainer._near_limit_terms(obs,labels)
    assert torch.isfinite(loss) and stats['near_limit_samples']==4
    loss.backward()
    assert any(p.grad is not None for p in net.actor.parameters())


def successful_report():
    return dict(case_id='test',seed=12,agent_steps=200,overrides=0,collisions=0,teleports=0,
        overspeed_steps=0,intervention_fraction=0.,native_emergencies={'available':True,'unique_events':0},
        primary_horizon_complete=True,accounting_comparison_valid=True,partial_control_horizon=False,
        task='merge',mean_fraction_of_speed_limit=.95,passes=0,successful_merges=2,merge_target_count=2,
        stable_merge={'stable':2},emergency_braking_observed=0,controlled_count=4)


@pytest.mark.parametrize('key,value', [('stable_merge',{'stable':1}),
    ('primary_horizon_complete',False),('native_emergencies',{'available':False}),
    ('collisions',1),('emergency_braking_observed',1)])
def test_entered_lane_does_not_erase_stability_or_native_failures(key,value):
    report=successful_report();assert support.acceptance([report])['all_gates_passed']
    report[key]=value
    assert not support.acceptance([report])['all_gates_passed']


@pytest.mark.parametrize('change', [{'merge_target_count':0},{'stable_merge':{'stable':float('nan')}},
    {'mean_fraction_of_speed_limit':float('nan')},{'intervention_fraction':.051}])
def test_missing_or_inconsistent_metrics_cannot_pass(change):
    report=successful_report();report.update(change)
    with pytest.raises(ValueError):support.acceptance([report])


def test_empty_evaluation_cannot_pass():
    with pytest.raises(ValueError):support.acceptance([])


def test_config_is_bounded_and_disjoint():
    cfg=json.loads(Path('experiments/configs/highway_environment_v05.json').read_text())
    support.validate_config(cfg)
    altered=deepcopy(cfg);altered['training']['max_agent_transitions']=250001
    with pytest.raises(ValueError):support.validate_config(altered)
    altered=deepcopy(cfg);altered['validation_cases'][0]['seed']=cfg['training']['seed_start']
    with pytest.raises(ValueError):support.validate_config(altered)
