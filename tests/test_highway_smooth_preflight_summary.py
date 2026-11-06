from copy import deepcopy

import pytest

from scripts.summarize_highway_smooth_preflight import (
    evaluate_candidate, mean_boundary_time, observed_merge_coverage, recording_audit, continuous_merge_gate, action_projection,
    drawdown_comparison,
)


def run(*, peak=2., completed=5, boundary=100., task='cruise', controller='smooth'):
    report = dict(agent_steps=100, overrides=0, collisions=0, teleports=0, overspeed_steps=0,
        emergency_braking_observed=0, passes=1, successful_merges=1, controlled_count=1,
        intervention_fraction=0., mean_fraction_of_speed_limit=.95, native_emergencies=dict(available=True, unique_events=0),
        primary_horizon_complete=True, accounting_comparison_valid=True, partial_control_horizon=False,
        task=task, case_id='fixture', seed=123, requested_trips=10, completed_trips=completed,
        accounting=dict(totals=dict(requested=10, completed=completed, boundary_time_s=boundary)),
        merge_target_count=1, merge_target_ids=['a'], stable_merge=dict(stable=1),
        smooth_control=dict(controlled=dict(jerk_m_s3={}), background=dict(jerk_m_s3={}),
            per_car=dict(a=dict(first_observed_time_s=.1, merge_slowdown=dict(
                observed_loss_m_s=3., requested_start_s=-1., requested_end_s=20.,
                available_start_s=.1, available_end_s=20., entry_time_s=4., settled_completion_time_s=12.,
                coverage_complete=False, nonconsecutive_gaps=0, samples=200)))))
    return dict(path=controller, report=report,
        audit=dict(controlled_peak_abs_jerk_m_s3=peak, controlled_measured_intervals=100, stopped_intervals=[],
            per_controlled_car=dict(a=dict(nominal_merge_stop=dict(coverage_complete=True, longest_consecutive_stop_s=0.)))),
        integrity_failures=[], hashes={}, manifest=dict(selected_case={'seed': 123}, network_sha256='network'),
        check=dict(demand_sha256='demand'))


def test_zero_intervention_does_not_override_failed_peak_gate():
    result = evaluate_candidate(run(peak=32.49), run(controller='teacher'))
    assert result['status'] == 'failed'
    assert result['gates']['unchanged_v05_safety_and_task']['status'] == 'passed'
    assert result['gates']['nominal_controlled_peak_jerk']['status'] == 'failed'


def test_mean_boundary_time_includes_unfinished_requests():
    assert mean_boundary_time(run(completed=2, boundary=200)['report']) == 20.
    invalid = run()['report']
    invalid['requested_trips'] = 9
    assert mean_boundary_time(invalid) is None


def test_missing_or_mismatched_baseline_is_incomplete():
    assert evaluate_candidate(run())['status'] == 'incomplete'
    baseline = run(controller='teacher')
    baseline['check']['demand_sha256'] = 'different'
    assert evaluate_candidate(run(), baseline)['gates']['boundary_time_vs_teacher']['status'] == 'incomplete'


def test_completed_trip_regression_and_slow_boundary_time_fail():
    result = evaluate_candidate(run(completed=4, boundary=106), run(completed=5, boundary=100))
    assert result['gates']['completed_trips_vs_teacher']['status'] == 'failed'
    assert result['gates']['boundary_time_vs_teacher']['status'] == 'failed'


def test_merge_boundary_censoring_retained_without_fabricated_history():
    candidate, teacher, actor = run(task='merge'), run(task='merge'), run(task='merge')
    result = evaluate_candidate(candidate, teacher, actor)
    gate = result['gates']['merge_drawdown_vs_frozen_actor']
    assert gate['status'] == 'passed'
    assert gate['comparisons'][0]['candidate_coverage']['boundary_censored']
    assert not gate['comparisons'][0]['candidate']['coverage_complete']
    candidate['report']['smooth_control']['per_car']['a']['merge_slowdown']['nonconsecutive_gaps'] = 1
    assert evaluate_candidate(candidate, teacher, actor)['gates']['merge_drawdown_vs_frozen_actor']['status'] == 'incomplete'


def test_existing_missing_history_and_missing_postwindow_do_not_pass():
    report = run(task='merge')['report']
    report['smooth_control']['per_car']['a']['merge_slowdown']['available_start_s'] = 1.
    assert not observed_merge_coverage(report, 'a')['usable']
    report['smooth_control']['per_car']['a']['merge_slowdown'].update(available_start_s=.1, available_end_s=19.9)
    assert not observed_merge_coverage(report, 'a')['usable']


def test_actual_startup_peak_is_retained_and_background_separated():
    frames = []
    for time, controlled_accel, background_accel in [(0.1, 0., 0.), (.2, -4.5, 8.), (.3, -4.4, 8.)]:
        frames.append(dict(time=time, vehicles=[dict(id=name, acceleration=accel, speed=10.,
            lane_id='road_0', lane_remaining_m=10., maneuver_in_progress=False)
            for name, accel in [('a', controlled_accel), ('human', background_accel)]]))
    recording = dict(frames=frames, controlled_ids=['a'])
    report = dict(smooth_control=dict(controlled=dict(jerk_m_s3=dict(realized=dict(peak_abs=45., samples=2)))))
    result = recording_audit(recording, report)
    assert result['controlled_peak_abs_jerk_m_s3'] == 45.
    assert result['controlled_startup_peak_abs_jerk_m_s3'] == 45.
    assert result['reported_controlled_jerk_agrees']


def test_native_warning_and_incomplete_horizon_still_fail_old_gates():
    candidate = run()
    candidate['report']['native_emergencies']['unique_events'] = 1
    candidate['report']['primary_horizon_complete'] = False
    result = evaluate_candidate(candidate, run())
    assert result['gates']['unchanged_v05_safety_and_task']['status'] == 'failed'


def test_new_nominal_stop_gate_rejects_long_stop_even_when_old_relative_bounds_pass():
    candidate = run(task='merge')
    candidate['audit']['per_controlled_car']['a']['nominal_merge_stop']['longest_consecutive_stop_s'] = 2.
    result = evaluate_candidate(candidate, run(task='merge'), run(task='merge'))
    assert result['gates']['nominal_continuous_merge']['status'] == 'failed'
    assert result['gates']['merge_drawdown_vs_frozen_actor']['status'] == 'passed'
    assert result['gates']['boundary_time_vs_teacher']['status'] == 'passed'


def test_new_stop_gate_keeps_short_stops_and_requires_complete_stable_coverage():
    candidate = run(task='merge')
    candidate['audit']['per_controlled_car']['a']['nominal_merge_stop'].update(longest_consecutive_stop_s=1.9)
    assert continuous_merge_gate(candidate)['status'] == 'passed'
    candidate['audit']['per_controlled_car']['a']['nominal_merge_stop']['coverage_complete'] = False
    assert continuous_merge_gate(candidate)['status'] == 'incomplete'


def test_invariance_action_projection_ignores_only_reward_metadata():
    first = [dict(time=.2, agents=dict(a=dict(requested_acceleration=1., lane_request=0,
        filtered_acceleration=.8, lane_applied_delta=0, reward_terms={'jerk': -.1}))) ]
    second = deepcopy(first)
    second[0]['agents']['a']['reward_terms']['jerk'] = -.3
    assert action_projection(first) == action_projection(second)
    assert action_projection(first, filtered=True) == action_projection(second, filtered=True)
    second[0]['agents']['a']['lane_request'] = 1
    assert action_projection(first) != action_projection(second)


def test_full_candidate_below_partial_actor_drawdown_proves_one_sided_bound():
    candidate = dict(observed_loss_m_s=1., coverage_complete=True)
    actor = dict(observed_loss_m_s=10., coverage_complete=False, requested_start_s=6.,
        requested_end_s=83.5, available_start_s=6., available_end_s=80.1, samples=742)
    result = drawdown_comparison(candidate, actor, {'usable': True}, {'usable': False})
    assert result['comparison_pass'] is True
    assert result['proof_type'] == 'one_sided_observed_baseline_lower_bound'
    assert result['baseline_full_window_claim'] is False and not result['complete_available_coverage']
    actor['observed_loss_m_s'] = .5
    assert drawdown_comparison(candidate, actor, {'usable': True}, {'usable': False})['comparison_pass'] is None


def test_one_sided_proof_never_accepts_censored_candidate_or_undefined_baseline_window():
    candidate = dict(observed_loss_m_s=1., coverage_complete=False)
    actor = dict(observed_loss_m_s=10., coverage_complete=False, requested_start_s=6.,
        requested_end_s=83.5, available_start_s=6., available_end_s=80.1, samples=742)
    assert drawdown_comparison(candidate, actor, {'usable': True}, {'usable': False})['comparison_pass'] is None
    candidate['coverage_complete'] = True
    actor['requested_end_s'] = None
    assert drawdown_comparison(candidate, actor, {'usable': True}, {'usable': False})['comparison_pass'] is None
