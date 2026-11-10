"""Offline reward qualification and fake-native invariance; no simulator/learning."""
from copy import deepcopy
import json

import numpy as np
import pytest

from environments.highway_flow_rl_env import HighwayFlowRLEnv
from environments.highway_flow_rl_v03_env import HighwayFlowRLV03Env
from traffic.highway_flow_rl_v03_rewards import (DEFAULT_WEIGHTS, BrakingDiagnostic,
    safe_speed_target, speed_error_reward, jerk_reward)
from tests.test_highway_flow_rl_env import rl_bare
from tests.test_highway_merge_context import graph
from tests.test_highway_smooth_control import fake_motion


def evidence():
    state = dict(speed=14., acceleration=0., allowed_speed=18.)
    own = dict(lane_id='through', known=True, legal_speed_m_s=18., front=None,
        route=dict(known=True, reachable=True, distance_m=500.))
    context = dict(corridors=dict(own=own), occupied_lanes=['through'],
        entry=dict(known=False, entry_active=False))
    return state, context, dict(known=True, lower_limits=[])


@pytest.mark.parametrize('j,cost', [(0,0), (1,.01125), (2,.045), (5,.18), (10,.405), (20,.855)])
def test_uncapped_huber_scale(j,cost):
    assert jerk_reward(j) == pytest.approx(-cost)
    assert jerk_reward(-j) == pytest.approx(-cost)


def test_spreading_same_acceleration_change_costs_less():
    assert sum(jerk_reward(j) for j in (1,1,1,1)) > sum(jerk_reward(j) for j in (4,0,0,0))


def test_target_is_pre_action_and_does_not_switch_off_during_braking():
    s,c,p = evidence()
    first = safe_speed_target(s,c,p)
    s['acceleration'] = -4.5
    assert safe_speed_target(s,c,p) == first
    assert first['target_m_s'] == 18.
    assert speed_error_reward(first,13.9) < speed_error_reward(first,14.1)
    assert speed_error_reward(first,18.5) == 0.
    assert speed_error_reward(first,100.) == -.30


def test_sequence_recovery_beats_stalling_and_oscillation_at_equal_progress():
    target = safe_speed_target(*evidence())
    def score(speeds, accelerations):
        jerk = [(b-a)/.1 for a,b in zip(accelerations,accelerations[1:])]
        return sum(v/26.8224 + speed_error_reward(target,v) for v in speeds) + sum(map(jerk_reward,jerk))
    assert score([14,15,16,17,18],[1,1,1,1,1]) > score([14]*5,[0]*5)
    assert score([16]*5,[0]*5) > score([16]*5,[1,-1,1,-1,1])


def test_close_leader_lowers_target_opening_leader_allows_recovery():
    s,c,p=evidence()
    c['corridors']['own']['front']=dict(bumper_gap_m=18.,relative_speed_m_s=-6.)
    target=safe_speed_target(s,c,p)
    assert target['eligible'] and target['target_m_s'] < s['speed']
    assert speed_error_reward(target,13.9) > speed_error_reward(target,14.1)
    c['corridors']['own']['front']=dict(bumper_gap_m=43.,relative_speed_m_s=2.5)
    target=safe_speed_target(s,c,p)
    assert target['target_m_s'] == 18.
    assert not target['clear_cruise']


def test_merge_front_and_rear_bound_incompatible_interval_abstains():
    s,c,p=evidence()
    c['corridors']['own']['route']['reachable']=False
    c['entry']=dict(known=True,target_lane='next',target_speed_limit=18.,distance_to_start_m=0.,
        entry_active=False,front=dict(bumper_gap_m=0.,relative_speed_m_s=0.),
        rear=dict(bumper_gap_m=0.,relative_speed_m_s=4.))
    target=safe_speed_target(s,c,p)
    assert not target['eligible'] and target['reasons']==['no_current_admissible_merge_gap']
    c['entry']['rear']['bumper_gap_m']=100.
    target=safe_speed_target(s,c,p)
    assert target['eligible'] and target['target_m_s'] < s['speed']
    assert 'ending_lane' in target['obligations']


def test_lower_limits_ending_runway_yield_and_occupied_lane_constraints():
    s,c,p=evidence()
    p['lower_limits']=[dict(distance_m=2.,limit_m_s=10.)]
    assert safe_speed_target(s,c,p)['target_m_s'] == 10.
    p['lower_limits']=[]
    c['entry']=dict(known=True,target_lane='next',target_speed_limit=18.,distance_to_start_m=10.,
        minor_link_visibility_m=5.,entry_active=True,front=None,rear=None)
    assert safe_speed_target(s,c,p)['target_m_s'] < 6.
    c['entry']['entry_active']=False
    c['corridors']['own']['route'].update(reachable=False,distance_m=45.)
    assert safe_speed_target(s,c,p)['target_m_s'] == 0.
    c['corridors']['own']['route']['reachable']=True
    c['corridors']['right']=deepcopy(c['corridors']['own'])
    c['corridors']['right'].update(lane_id='next',legal_speed_m_s=9.)
    c['occupied_lanes'].append('next')
    assert safe_speed_target(s,c,p)['target_m_s'] == 9.


@pytest.mark.parametrize('fault',['nan_speed','missing_occupancy','invalid_gap','unknown_map','cutin'])
def test_missing_or_unsafe_evidence_cannot_claim_target_coverage(fault):
    s,c,p=evidence()
    if fault=='nan_speed': s['speed']=float('nan')
    elif fault=='missing_occupancy': c['occupied_lanes'].append('missing')
    elif fault=='invalid_gap': c['corridors']['own']['front']=dict(bumper_gap_m=float('nan'),relative_speed_m_s=0.)
    elif fault=='unknown_map': p['known']=False
    else: c['corridors']['right']=dict(lane_id='other',front={},front_lateral_speed_over3=.1)
    t=safe_speed_target(s,c,p)
    assert not t['eligible'] and t['reasons'] and speed_error_reward(t,10.)==0.


def test_diagnostic_requires_persistent_eligible_braking_and_reports_coverage():
    target=safe_speed_target(*evidence()); diagnostic=BrakingDiagnostic()
    for i in range(3): diagnostic.observe('a',i*.1,target,14.,-.3,1.)
    assert diagnostic.rows['a']['events']==1
    diagnostic.observe('a',.3,target,14.,.3,1.)
    assert diagnostic.rows['a']['reaccelerations']==1
    target['obligations']=['ending_lane']
    for i in range(4,7): diagnostic.observe('a',i*.1,target,14.,-2.,1.)
    assert diagnostic.rows['a']['events']==1


def bare(graph):
    env=rl_bare(graph,jerk_reward_weight=0.)
    env.__class__=HighwayFlowRLV03Env
    env.v03_reward_weights=dict(DEFAULT_WEIGHTS)
    env._reset_v03()
    return env


def test_v03_changes_reward_only_and_journal_matches_total(graph):
    baseline,env=rl_bare(graph,jerk_reward_weight=0.),bare(graph)
    for item in (baseline,env): fake_motion(item,[.2,-4.5,.1])
    for _ in range(3):
        action={'agent_0':np.array([1.,0.],np.float32)}
        a,b=baseline.step(action),env.step(action)
        info=b[4]['agent_0']
        np.testing.assert_array_equal(a[0]['agent_0'],b[0]['agent_0'])
        assert env.state().shape==(2832,) and b[0]['agent_0'].shape==(117,)
        for k in ('requested_acceleration','filtered_acceleration','realized_acceleration','override','override_causes','lane_applied_delta'):
            assert a[4]['agent_0'][k]==info[k]
        assert b[1]['agent_0']==pytest.approx(sum(info['reward_terms'].values()))
        assert b[1]['agent_0']-a[1]['agent_0']==pytest.approx(sum(info['reward_terms'][k] for k in ('speed_error','jerk','route_detour')))
    assert env.conn.vehicle.setAcceleration.call_args_list==baseline.conn.vehicle.setAcceleration.call_args_list
    assert env.conn.vehicle.changeLane.call_args_list==baseline.conn.vehicle.changeLane.call_args_list
    assert env._mask.__func__ is baseline._mask.__func__
    journal=[json.loads(x) for x in env.journal.getvalue().splitlines()]
    assert sum(sum(x['agents']['agent_0']['reward_terms'].values()) for x in journal)==pytest.approx(sum(env.stats['reward_terms'].values()))


def test_first_motion_detour_once_and_native_refusal_uncharged(graph,monkeypatch):
    env=bare(graph)
    def begin(self,agent,delta): self.maneuvers[agent]=dict(started_at=1.,moved=False)
    monkeypatch.setattr(HighwayFlowRLEnv,'_begin_maneuver',begin)
    env._v02_pre_action['agent_0']=dict(time_s=1.,detours={'-1':dict(optional_detour=True)})
    env._begin_maneuver('agent_0',-1)
    assert env._consume_detour_event('agent_0') is None
    env.maneuvers['agent_0']['moved']=True
    assert env._consume_detour_event('agent_0')['evidence']['optional_detour']
    assert env._consume_detour_event('agent_0') is None
    env._begin_maneuver('agent_0',-1);env.maneuvers.clear()
    assert env._consume_detour_event('agent_0') is None


def test_legacy_jerk_cannot_be_double_charged(tmp_path):
    with pytest.raises(ValueError,match='base weight'):
        HighwayFlowRLV03Env(tmp_path/'missing',tmp_path,jerk_reward_weight=.03)
