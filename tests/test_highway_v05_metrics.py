import json
import math

import pytest

from traffic.highway_v05_metrics import PostMergeTracker


def sample(tracker, tick, **kwargs):
    values = dict(speed_mps=10., speed_limit_mps=10., distance_m=float(tick), lane='through-0')
    values.update(kwargs)
    return tracker.observe(tick / 10, **values)


def tracker():
    value = PostMergeTracker('car')
    value.mark_entry(0.)
    value.mark_merge_complete(0., 'through-0')
    sample(value, 0, count_decision=False)
    return value


def test_full_eight_second_window_and_once_only_bounded_bonus():
    value = tracker()
    for tick in range(1, 80):
        sample(value, tick)
    assert value.summary()['status'] == 'pending'
    assert value.consume_stable_bonus() == 0
    result = sample(value, 80)
    assert result['window_complete'] and result['status'] == 'succeeded'
    assert result['post_window']['elapsed_s'] == pytest.approx(8.)
    assert value.consume_stable_bonus() == 2.
    assert value.consume_stable_bonus() == 0.
    sample(value, 81, collision=True, emergency=True)
    result = value.finish(arrived=True)
    assert result['status'] == 'succeeded'
    assert result['post_window']['collisions'] == 0
    assert result['full_route']['collisions'] == 1
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('finish', [dict(arrived=True), dict(truncated=True), {}])
def test_arrival_or_truncation_before_complete_window_stays_pending(finish):
    value = tracker()
    for tick in range(1, 80):
        sample(value, tick)
    result = value.finish(**finish)
    assert result['status'] == 'pending' and not result['window_complete']
    assert value.consume_stable_bonus() == 0


def test_entry_without_completed_lateral_merge_is_never_stable():
    value = PostMergeTracker()
    assert value.mark_entry(0.) and not value.mark_entry(1.)
    for tick in range(90):
        sample(value, tick)
    result = value.finish(arrived=True)
    assert result['entry_event'] and not result['lateral_merge_completed']
    assert result['status'] == 'pending'


@pytest.mark.parametrize('extra,reason', [(dict(collision=True), 'collision'),
    (dict(emergency=True), 'emergency'), (dict(speed_mps=10.1), 'overspeed'),
    (dict(leader_present=True, leader_headway_s=1.49), 'headway'),
    (dict(leader_present=True, leader_headway_s=2., leader_ttc_s=2.99), 'ttc')])
def test_safety_failure_at_exact_boundary_cannot_receive_bonus(extra, reason):
    value = tracker()
    for tick in range(1, 80):
        sample(value, tick)
    result = sample(value, 80, **extra)
    assert result['status'] == 'failed' and reason in result['failure_reasons']
    assert value.consume_stable_bonus() == 0


def test_threshold_equality_and_nonclosing_leader_are_valid():
    value = tracker()
    for tick in range(1, 81):
        sample(value, tick, leader_present=True, leader_headway_s=1.5,
               leader_ttc_s=3. if tick == 1 else math.inf, intervention=tick <= 4)
    result = value.summary()
    assert result['status'] == 'succeeded'
    assert result['post_window']['intervention_fraction'] == pytest.approx(.05)
    json.dumps(result, allow_nan=False)


def test_sparse_observations_cannot_create_complete_window():
    value = tracker()
    sample(value, 80)
    result = sample(value, 81, collision=True)
    assert result['status'] == 'pending' and not result['window_complete']
    assert result['observation_coverage_gap']
    assert result['post_window']['collisions'] == 0


@pytest.mark.parametrize('reason,evidence,classification,penalty', [
    (None, False, 'unclassified', 0), ('unnecessary', False, 'unclassified', 0),
    ('route', True, 'justified', 0), ('safety', True, 'justified', 0),
    ('passing', True, 'justified', 0), ('unnecessary', True, 'unnecessary', 1)])
def test_reversals_are_reason_and_evidence_aware(reason, evidence, classification, penalty):
    value = tracker()
    sample(value, 1, lane='through-1')
    sample(value, 2, lane='through-0', reversal_reason=reason, reason_evidence=evidence)
    assert value.summary()['lane_reversals'][0]['classification'] == classification
    assert value.consume_lane_oscillation_penalty() == penalty
    assert value.consume_lane_oscillation_penalty() == 0
    assert (value.summary()['status'] == 'failed') == bool(penalty)


def test_braking_is_observed_without_causal_claim_and_control_paths_are_separate():
    value = tracker()
    sample(value, 1, neighbor_braking_mps2=-5., requested_acceleration=2.,
           filtered_acceleration=1., realized_acceleration=.5, native_intervention=True, intervention=True)
    result = value.summary()
    assert 'no causal attribution' in result['neighbor_braking_attribution']
    assert result['post_window']['observed_neighbor_braking_samples'] == 1
    assert result['full_route']['policy_filter_changes'] == 1
    assert result['full_route']['native_realization_changes'] == 1
    assert result['full_route']['mean_requested_acceleration'] == 2.


@pytest.mark.parametrize('problem', ['no_progress', 'too_many_interventions', 'missing_headway', 'missing_ttc'])
def test_remaining_window_gates_fail_closed(problem):
    value = tracker()
    for tick in range(1, 81):
        values = (dict(distance_m=0.) if problem == 'no_progress' else
                  dict(intervention=True) if problem == 'too_many_interventions' else
                  dict(leader_present=True, leader_headway_s=2.) if problem == 'missing_ttc' else
                  dict(leader_present=True, leader_ttc_s=math.inf))
        sample(value, tick, **values)
    assert value.summary()['status'] == 'failed'


def test_bad_time_and_nonfinite_evidence_are_rejected():
    value = tracker()
    with pytest.raises(ValueError, match='strictly'):
        sample(value, 0)
    with pytest.raises(ValueError, match='finite'):
        sample(value, 1, speed_mps=math.nan)
    with pytest.raises(ValueError, match='prior entry'):
        PostMergeTracker().mark_merge_complete(1.)
