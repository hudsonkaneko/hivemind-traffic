import copy
import pytest

from experiments.sumo_mappo_rollout import EpisodeMetrics, aggregate


def episode(seed,passes=1,changes=1):
    metrics=EpisodeMetrics(seed)
    for step in range(600):
        infos={}
        for index in range(4):
            infos[f'agent_{index}']=dict(progress_m=2.5,speed_m_s=25.,front_gap_m=30.,
                truncation_cause='time_limit' if step==599 else None,
                driving_metrics=dict(lane_change_requested=int(index==0 and step<changes),
                    lane_change_applied=int(index==0 and step<changes),lane_change_rejected=0,
                    pass_completed=int(index==0 and step<passes),headway_s=1.2,
                    cooperative_headway_active=step%2==0,cooperative_headway_applied=step%2==0,
                    applied_native_headway_s=.8 if step%2==0 else 1.5,
                    required_gap_m=25.,abs_jerk_m_s3=1.))
        metrics.add(dict(rewards={a:1. for a in infos},infos=infos))
    return metrics.result()


ACCEPTANCE=dict(collision_vehicles=0,overspeed_steps=0,minimum_mean_progress_m=600.,maximum_intervention_fraction=.05)
TARGETS=dict(minimum_passes_per_episode=1,maximum_lane_changes_per_vehicle_minute=6.)


def test_behavior_acceptance_is_each_episode_and_each_vehicle():
    good=episode(1)
    result=aggregate([good],ACCEPTANCE,TARGETS)
    assert result['accepted'] and result['driving_targets_met']
    assert result['driving']['cooperative_steps']==1200
    assert result['driving']['fallback_following_steps']==1200
    assert result['driving']['mean_headway_s']==pytest.approx(1.2)
    assert not aggregate([episode(1,passes=20),episode(2,passes=0)],ACCEPTANCE,TARGETS)['accepted']
    assert not aggregate([episode(1,changes=7)],ACCEPTANCE,TARGETS)['accepted']
    bad=copy.deepcopy(good);bad['driving']['maximum_lane_changes_per_vehicle_minute']=float('nan')
    assert not aggregate([bad],ACCEPTANCE,TARGETS)['accepted']
    assert not aggregate([good,bad],ACCEPTANCE,TARGETS)['accepted']


def test_no_behavior_claim_without_telemetry():
    row=episode(1);row.pop('driving')
    result=aggregate([row],ACCEPTANCE,TARGETS)
    assert result['basic_driving_accepted'] and not result['driving_targets_met'] and not result['accepted']


def test_actual_headway_commands_are_separate_from_next_observation_eligibility():
    metrics=EpisodeMetrics(1)
    base=dict(lane_change_requested=0,lane_change_applied=0,lane_change_rejected=0,
        pass_completed=0,headway_s=1.3,required_gap_m=25.,abs_jerk_m_s3=0.)
    for applied,eligible in [(False,True),(True,False)]:
        info=dict(progress_m=1.,speed_m_s=20.,front_gap_m=26.,driving_metrics=dict(base,
            cooperative_headway_active=eligible,cooperative_headway_applied=applied,
            applied_native_headway_s=.8 if applied else 1.5))
        metrics.add(dict(rewards={'agent_0':1.},infos={'agent_0':info}))
        if not applied:
            first=metrics.result()['driving']
            assert first['applied_cooperative_steps']==first['cooperative_steps']==0
            assert first['cooperative_eligible_steps']==1 and first['applied_fallback_steps']==1
    result=metrics.result()['driving']
    assert result['applied_cooperative_steps']==1 and result['applied_fallback_steps']==1
    assert result['cooperative_eligible_steps']==1 and result['fallback_following_steps']==1


def test_refined_selection_does_not_prefer_reward_without_passing_or_ignore_safety():
    rule='accepted_then_safety_then_driving_targets_then_fewer_interventions_then_reward'
    follows=episode(1,passes=0); follows['team_reward']=10000
    passes=episode(1); passes['interventions']=240  # 10%, still rejected
    safe=episode(1); safe['team_reward']=-10000
    score=lambda row: tuple(aggregate([row],ACCEPTANCE,TARGETS,selection=rule)['selection_score'])
    assert score(safe)>score(passes)>score(follows)
    crash=copy.deepcopy(passes);crash['collision_vehicles']=1
    assert score(follows)>score(crash)
    lower=copy.deepcopy(passes);lower['interventions']=144;lower['team_reward']=-20000
    assert score(lower)>score(passes)
