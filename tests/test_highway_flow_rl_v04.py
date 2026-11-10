from copy import deepcopy
import json
from pathlib import Path

import pytest

from traffic.highway_flow_rl_v04_rewards import (
    gap_plan, merge_speed_target, maneuver_gap_evidence, shaping_reward,
    quality_merge_eligible, validate_weights)
from tests.test_highway_flow_rl_v03_rewards import evidence
from traffic.highway_flow_rl_v04_contract import validate_config
from experiments.run_highway_flow_rl_v04 import full_episode_reservation, hashes


def blocked():
    s,c,p=evidence()
    c['corridors']['own']['route'].update(reachable=False,distance_m=300.)
    c['entry']=dict(known=True,target_lane='next',target_speed_limit=18.,distance_to_start_m=0.,
        entry_active=False,front=dict(id='front',bumper_gap_m=0.,relative_speed_m_s=0.),
        rear=dict(id='rear',bumper_gap_m=0.,relative_speed_m_s=4.))
    return s,c,p


def test_blocked_immediate_gap_keeps_longitudinal_guidance_without_lateral_permission():
    s,c,p=blocked(); target,plan=merge_speed_target(s,c,p)
    assert target['eligible'] and 0<=target['target_m_s']<=s['allowed_speed']
    assert target['original_target_reasons']==['no_current_admissible_merge_gap']
    assert plan['known'] and 0<plan['potential']<=1 and plan['lateral_permission'] is False
    assert not maneuver_gap_evidence(s['speed'],c['entry'])['permitted']


def test_unknown_and_occupied_lane_safety_are_not_replaced_by_gap_prediction():
    s,c,p=blocked();c['entry']['known']=False
    assert not merge_speed_target(s,c,p)[0]['eligible']
    s,c,p=blocked();c['corridors']['own']['front']=dict(bumper_gap_m=2.5,relative_speed_m_s=-s['speed'])
    assert merge_speed_target(s,c,p)[0]['target_m_s']<1e-5
    p['known']=False
    assert not merge_speed_target(s,c,p)[0]['eligible']


def test_no_merge_has_original_cruise_target():
    s,c,p=evidence();target,plan=merge_speed_target(s,c,p)
    assert target['eligible'] and target['clear_cruise'] and not plan['required']
    assert target['target_m_s']==18.


def test_projection_rejects_recorded_late_closing_gap_but_allows_open_gap():
    neighbors=dict(front=dict(bumper_gap_m=5.25,relative_speed_m_s=16.49),
                   rear=dict(bumper_gap_m=41.13,relative_speed_m_s=16.19))
    result=maneuver_gap_evidence(.407,neighbors)
    assert not result['permitted'] and result['checks']['rear']['projected_gap_m']<0
    assert maneuver_gap_evidence(18.,dict(front=dict(bumper_gap_m=70.,relative_speed_m_s=0.),
        rear=dict(bumper_gap_m=70.,relative_speed_m_s=0.)))['permitted']
    assert not maneuver_gap_evidence(18.,dict(front=dict(bumper_gap_m=50.,relative_speed_m_s=-10.)))['permitted']


def test_potential_telescopes_across_slot_changes_and_true_terminal():
    potentials=[.2,.8,.4,.9]
    rewards=[shaping_reward(dict(potential=a),dict(potential=b)) for a,b in zip(potentials,potentials[1:])]
    rewards.append(shaping_reward(dict(potential=potentials[-1]),dict(potential=.5),terminated=True))
    assert sum(.99**i*r for i,r in enumerate(rewards))==pytest.approx(-2*potentials[0])
    assert shaping_reward(dict(potential=.5),dict(potential=.5))<0
    assert shaping_reward(dict(potential=.2),dict(potential=.8))>0
    assert shaping_reward(dict(potential=.2),dict(potential=.8),terminated=True)==pytest.approx(-.4)


def test_quality_bonus_rejects_stops_severe_braking_and_unmatched_speed():
    good=[dict(stopped=False,jerk=1.,override=False,neighbor_acceleration=0.,speed_match=True) for _ in range(160)]
    assert quality_merge_eligible(good)
    stopped=deepcopy(good)
    for r in stopped[:14]:r['stopped']=True
    assert not quality_merge_eligible(stopped)
    for key,value in [('jerk',10.),('neighbor_acceleration',-4.)]:
        bad=deepcopy(good);bad[0][key]=value;assert not quality_merge_eligible(bad)
    assert not quality_merge_eligible([dict(r,speed_match=False) for r in good])
    assert not quality_merge_eligible([])


def config():
    return json.loads(Path('configs/training/highway-flow-rl-v04-long.json').read_text())


def test_full_episode_allocation_does_not_shrink_with_population():
    cfg=config();assert validate_config(cfg,require_training=True) is cfg
    for row in cfg['training_rows']:
        reserve=full_episode_reservation(row,100000)
        assert reserve/row['controlled_count']==row['horizon_steps']
        assert full_episode_reservation(row,reserve-1) is None
    bad=deepcopy(cfg);bad['training_rows'][0]['rollout_decisions']=2048
    with pytest.raises(ValueError,match='complete episode'):validate_config(bad)


@pytest.mark.parametrize('field,value',[('safety',{}),('gae',{}),('v04_reward_weights',{}),('optimizer',{}),('budget',{})])
def test_contract_rejects_unreviewed_changes(field,value):
    cfg=config();cfg[field]=value
    with pytest.raises(ValueError):validate_config(cfg)


def test_sources_include_new_and_inherited_reward_code():
    result=hashes()
    assert 'traffic/highway_flow_rl_v04_rewards.py' in result
    assert 'traffic/highway_flow_rl_v03_rewards.py' in result
    assert 'environments/highway_flow_rl_v04_env.py' in result
    with pytest.raises(ValueError):validate_weights({'gap_preparation':float('nan')})


def test_best_checkpoint_cannot_prefer_stalling_just_for_lower_overrides():
    from experiments.run_highway_flow_rl_v04 import validation_rank
    def result(arrived,overrides):
        return dict(reports=[dict(collisions=0,teleports=0,quality_merges={},pre_merge_stop_seconds={})],
            summary=dict(case_gates=[dict(passed=False,per_car={'a':dict(intervention_fraction=overrides,
                checks=dict(completed_route=arrived))})]))
    assert validation_rank(result(True,.1)) < validation_rank(result(False,0.))
