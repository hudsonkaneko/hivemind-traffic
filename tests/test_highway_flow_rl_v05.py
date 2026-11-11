from copy import deepcopy
import inspect
import json
from pathlib import Path

import pytest

from traffic.highway_flow_rl_v05_rewards import condition_acceleration, stable_target, extra_rewards, WEIGHTS
from traffic.highway_flow_rl_v04_rewards import merge_speed_target, gap_plan
from traffic.highway_flow_rl_v05_contract import validate_config
from tests.test_highway_flow_rl_v04 import blocked
from tests.test_highway_flow_rl_v03_rewards import evidence


@pytest.mark.parametrize('requested,measured,expected', [(1.21,0.,.4),(.576,-4.5,-4.1),(-1.,0.,-.4),(.1,0.,.1)])
def test_start_and_brake_release_follow_measured_acceleration(requested,measured,expected):
    row=condition_acceleration(requested,measured,2.)
    assert row['conditioned']==pytest.approx(expected)
    assert row['raw_requested_jerk']==pytest.approx((requested-measured)/.1)


def test_safety_ceiling_and_maximum_braking_override_comfort():
    row=condition_acceleration(1.,1.,-4.5)
    assert row['expected_applied']==-4.5 and row['safety_exception']
    row=condition_acceleration(-4.5,1.,2.)
    assert row['expected_applied']==-4.5 and row['safety_exception']
    assert condition_acceleration(.58,-4.5,2.)['expected_applied']==pytest.approx(-4.1)
    assert condition_acceleration(1.,-9.,2.)['conditioned']==-4.5


@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_nonfinite_control_cannot_reach_sumo(value):
    with pytest.raises(ValueError): condition_acceleration(value,0.,2.)


def test_disabled_conditioning_preserves_legacy_request():
    assert condition_acceleration(1.21,0.,2.,enabled=False)['conditioned']==1.21


def test_stable_target_retains_original_acceptance_reference():
    s,c,p=blocked()
    target,plan,reference=stable_target(s,c,p)
    assert reference==merge_speed_target(s,c,p)[0]
    assert plan['lateral_permission'] is False
    history=dict(target=target,plan=plan)
    s2=deepcopy(s);s2['speed']+=.1
    target2,_,reference2=stable_target(s2,c,p,history)
    assert target2['target_m_s']<=target['target_m_s']+.15+1e-6
    assert reference2==merge_speed_target(s2,c,p)[0]


def test_target_never_smooths_through_new_stopping_constraint():
    s,c,p=blocked();target,plan,_=stable_target(s,c,p)
    c['corridors']['own']['front']=dict(bumper_gap_m=2.5,relative_speed_m_s=-s['speed'])
    current,_,_=stable_target(s,c,p,dict(target=dict(target_m_s=18.),plan=plan))
    assert current['target_m_s']<1e-5
    p['known']=False
    assert not stable_target(s,c,p,dict(target=target,plan=plan))[0]['eligible']


def test_missing_or_changed_slot_cannot_lock_planner():
    s,c,p=blocked();original=gap_plan(s,c)
    previous=dict(original,front_id='gone',rear_id='gone')
    assert gap_plan(s,c,previous_plan=previous)==original
    s,c,p=evidence()
    assert not gap_plan(s,c,previous_plan=previous)['required']
    assert stable_target(s,c,p)[0]==merge_speed_target(s,c,p)[0]


def test_rewards_penalize_harsh_raw_action_even_if_smoothed():
    target=dict(eligible=True,target_m_s=18.)
    smooth=extra_rewards(4.,4.,target,18.,merge_active=True)
    harsh=extra_rewards(4.,50.,target,18.,merge_active=True)
    slow=extra_rewards(4.,4.,target,0.,merge_active=True)
    assert smooth==dict(jerk_excess=0.,merge_tracking=0.)
    assert harsh['jerk_excess']<0 and slow['merge_tracking']==-WEIGHTS['merge_tracking']
    assert extra_rewards(4.,4.,target,0.,merge_active=False)['merge_tracking']==0


def test_config_training_disabled_and_checks_unchanged():
    cfg=json.loads(Path('configs/training/highway-flow-rl-v05-pilot.json').read_text())
    assert validate_config(cfg) is cfg
    with pytest.raises(ValueError,match='disabled'): validate_config(cfg,require_training=True)
    for key,value in [('variant','unknown'),('continuity_settings',{}),('v05_reward_weights',{}),('acceptance',{})]:
        bad=deepcopy(cfg);bad[key]=value
        with pytest.raises(ValueError):validate_config(bad)
    from experiments.run_highway_flow_rl_v04 import development_gates as old
    from experiments.run_highway_flow_rl_v05 import development_gates as new
    assert inspect.getsource(old)==inspect.getsource(new)


def test_rollouts_retain_raw_sampled_action_and_log_probability():
    from experiments.run_highway_flow_rl_v05 import episode
    source=inspect.getsource(episode)
    assert 'actions=actions, pre_tanh_actions=latent, old_log_probs=logp' in source
    assert 'env.step(chosen)' in source
