"""Bounded pure-data plotting checks; no renderer, native runtime, or policy."""
from copy import deepcopy
import math

import pytest

from experiments.lane_follow_ppo_evaluation import HELD_OUT_SEEDS, ROUTE_IDS, UPDATES, case_label
from scripts.plot_lane_follow_ppo import chart_data


def outcome(reason='success',progress=59.8,rms=.02):
    return dict(evidence_valid=True,terminal_reason=reason,driving_success=reason=='success',
        physical_failure=reason not in ('success','time_limit'),
        metrics=dict(progress_m=progress,driving_lateral_rms_m=rms))


@pytest.fixture
def inputs():
    training=[dict(global_control_step=100,**outcome('lane_departure',.1,None)),
        dict(global_control_step=200,**outcome('time_limit',.2,.01)),
        dict(global_control_step=65500,**outcome())]
    summary=dict(training_state=dict(control_steps=65536,completed_updates=32),partial_episode_control_steps=36)
    selection=dict(valid=True,selected_checkpoint=dict(update=8),rankings=[
        dict(checkpoint=dict(update=update),aggregate=dict(episodes=3,mean_progress_m=float(index),
            progress_m=dict(std_population=.1),driving_successes=index,physical_failures=3-index))
        for index,update in enumerate(UPDATES)])
    heldout=dict(valid=True,held_out_used_for_selection=False,selected_checkpoint=dict(update=8),cases={})
    for policy in ('ppo','scripted'):
        for route in ROUTE_IDS:
            for seed in HELD_OUT_SEEDS:
                item=(outcome('lane_departure',.1,None) if policy=='ppo' and seed==4101
                      else outcome('time_limit',.2,.01) if policy=='ppo' else outcome())
                heldout['cases'][case_label('heldout',policy,route,seed,8 if policy=='ppo' else None)]=dict(valid=True,outcome=item)
    return training,summary,selection,heldout


def test_plot_data_keeps_failures_missing_rms_pairs_and_partial_count(inputs):
    original=deepcopy(inputs)
    result=chart_data(*inputs)
    assert len(result['training'])==3 and len(result['validation'])==4 and len(result['pairs'])==15
    assert result['partial_training_steps']==36
    assert sum(row['ppo']['failure'] for row in result['pairs'])==3
    assert sum(row['ppo']['rms_m'] is None for row in result['pairs'])==3
    assert all(row['scripted']['success'] for row in result['pairs'])
    assert inputs==original


@pytest.mark.parametrize('mutation',['missing_pair','nonfinite_metric','partial_budget','episode_order'])
def test_incomplete_or_inconsistent_chart_inputs_reject(inputs,mutation):
    training,summary,selection,heldout=inputs
    if mutation=='missing_pair':heldout['cases'].pop(next(iter(heldout['cases'])))
    elif mutation=='nonfinite_metric':next(iter(heldout['cases'].values()))['outcome']['metrics']['progress_m']=math.nan
    elif mutation=='partial_budget':summary['partial_episode_control_steps']=35
    else:training[1]['global_control_step']=100
    with pytest.raises(ValueError):chart_data(training,summary,selection,heldout)
