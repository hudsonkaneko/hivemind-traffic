from scripts.diagnose_highway_flow_rl_v05_jerk import classify, summarise


def sample(requested=0.5, **flags):
    base = dict(requested_acceleration=requested, override=False, policy_filter_changed=False,
                native_realization_changed=False, lane_change_observed=False,
                maneuver=dict(in_progress=False, completed_this_step=False))
    base.update(flags)
    return base


def test_first_decision_is_startup():
    assert classify(sample(), None, True) == 'startup'


def test_override_takes_priority_over_policy_swing():
    assert classify(sample(2.0, override=True), sample(-2.0), False) == 'safety_override'


def test_release_after_native_override():
    previous = sample(0.6, override=True, native_realization_changed=True)
    assert classify(sample(0.6), previous, False) == 'override_release'


def test_lane_change_and_policy_request():
    assert classify(sample(1.0, maneuver=dict(in_progress=True)), sample(0.0), False) == 'lane_change'
    assert classify(sample(1.0), sample(0.0), False) == 'policy_request'
    assert classify(sample(0.51), sample(0.5), False) == 'other'


def test_summary_counts_startup_only_cars():
    peak = lambda cause: dict(cause=cause, jerk_m_s3=9.0)
    cases = [dict(case_id='c', cars={'a': dict(peaks=[peak('startup')]),
                                     'b': dict(peaks=[peak('startup'), peak('safety_override')]),
                                     'c': dict(peaks=[])})]
    s = summarise(cases)
    assert s['failing_car_count'] == 2
    assert s['failing_only_at_startup'] == 1
    assert s['failing_with_native_or_override_peak'] == 1
