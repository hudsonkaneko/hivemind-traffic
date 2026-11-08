"""Pure/fake-native checks: these tests never launch SUMO or optimize a policy."""
import json

import numpy as np
import pytest

from environments.highway_environment_v05 import HighwayEnvironmentV05Env
from environments.highway_smooth_control import HighwaySmoothControlEnv, smooth_reward_terms
from traffic.highway_smooth_metrics import SmoothControlMetrics, distribution, huber, nominal_merge_stop
from tests.test_highway_environment_v05 import v05_bare, native_vehicle
from tests.test_highway_merge_context import graph


def smooth_bare(graph, objective='smooth_v01'):
    env = v05_bare(graph)
    env.__class__ = HighwaySmoothControlEnv
    env.objective_version = objective
    env.reference_parameters = {}
    if objective == 'smooth_v01':
        env.reward_weights.update(feasible_acceleration=1., jerk=.03,
            command_reference_residual_slew=.01)
    env._reset_smooth_tracking()
    return env


def fake_motion(env, accelerations):
    accelerations = iter(accelerations)
    def advance():
        acceleration = next(accelerations)
        env.time += .1
        state = env.states['agent_0']
        speed = state['speed'] + .1 * acceleration
        env.states = {'agent_0': dict(state, acceleration=acceleration, speed=speed,
            distance=state['distance'] + .1 * speed)}
    env._advance = advance


@pytest.mark.parametrize('value,expected', [(0, 0), (.5, .125), (1, .5), (5, 4.5), (-58.9257, 58.4257)])
def test_huber_units_and_unbounded_tail(value, expected):
    assert huber(value) == pytest.approx(expected)
    assert huber(1000) > huber(100)


def test_residual_slew_tracks_error_not_necessary_reference_change():
    first, missing = smooth_reward_terms(-4.5, -4.5, -45.)
    assert missing is None and first['command_reference_residual_slew'] == 0.
    next_terms, slew = smooth_reward_terms(2., 2., 65., previous_residual=0.)
    assert slew == 0 and next_terms['command_reference_residual_slew'] == 0
    assert next_terms['jerk'] == pytest.approx(-.03 * 64.5)
    terms, slew = smooth_reward_terms(1., 0., 0., previous_residual=0.)
    assert slew == 10 and terms['command_reference_residual_slew'] == pytest.approx(-.095)
    assert terms['feasible_acceleration'] == pytest.approx(-1 / 4.5)


def test_required_braking_alignment_prefers_braking_to_positive_request():
    brake, _ = smooth_reward_terms(-4.5, -4.5, -20.)
    positive, _ = smooth_reward_terms(2., -4.5, -20.)
    assert sum(brake.values()) > sum(positive.values())


@pytest.mark.parametrize('kwargs', [dict(objective_version='unknown'),
    dict(reference_parameters={'entry_preview_time_s': 3.}),
    dict(reference_parameters={'hidden_speed_override': 1.}),
    dict(reward_weights={'intervention': 0.})])
def test_pinned_configuration_rejects_confounded_arms_before_runtime(kwargs, tmp_path):
    with pytest.raises(ValueError):
        HighwaySmoothControlEnv(tmp_path / 'missing.xml', tmp_path, **kwargs)


def test_old_objective_exact_and_both_arms_preserve_commands_observations_motion(graph):
    base = v05_bare(graph)
    old = smooth_bare(graph, 'v05')
    new = smooth_bare(graph)
    for env in (base, old, new):
        fake_motion(env, [-1., .5, 2.])
    for raw in (.1, .4, .8):
        action = {'agent_0': np.array([raw, 0.], np.float32)}
        results = [env.step(action) for env in (base, old, new)]
        assert results[0][1] == results[1][1]
        assert results[0][4]['agent_0']['reward_terms'] == results[1][4]['agent_0']['reward_terms']
        for result in results[1:]:
            np.testing.assert_array_equal(result[0]['agent_0'], results[0][0]['agent_0'])
            assert result[2:4] == results[0][2:4]
        assert base.states == old.states == new.states
        assert base.previous == old.previous == new.previous
        assert base.conn.vehicle.setAcceleration.call_args_list == old.conn.vehicle.setAcceleration.call_args_list == new.conn.vehicle.setAcceleration.call_args_list
        assert base.conn.vehicle.changeLane.call_args_list == old.conn.vehicle.changeLane.call_args_list == new.conn.vehicle.changeLane.call_args_list
    assert base.stats['overrides'] == old.stats['overrides'] == new.stats['overrides']
    for agent in base.agents:
        np.testing.assert_array_equal(base.teacher_actions()[agent], old.teacher_actions()[agent])
    assert new.stats['reward_terms'] != base.stats['reward_terms']


def test_reward_journal_totals_and_reset_lifecycle(graph):
    env = smooth_bare(graph)
    fake_motion(env, [-4.5, 2.])
    total = 0.
    for index in range(2):
        _, rewards, _, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
        info = infos['agent_0']
        total += rewards['agent_0']
        assert sum(info['reward_terms'].values()) == pytest.approx(rewards['agent_0'])
        assert info['residual_history_available'] is (index == 1)
        assert json.loads(env.journal.getvalue().splitlines()[-1])['agents']['agent_0'] == info
    assert sum(env.stats['reward_terms'].values()) == pytest.approx(total)
    report = env.smooth_metrics.report()
    assert report['all']['jerk_m_s3']['realized']['peak_abs'] == pytest.approx(65.)
    assert report['startup']['jerk_m_s3']['realized']['peak_abs'] == 45.
    assert report['all']['jerk_m_s3']['requested']['samples'] == 1
    assert report['all']['jerk_m_s3']['requested']['missing_samples'] == 1
    env._reset_smooth_tracking()
    assert not env._previous_reference_residual and not env.smooth_metrics.traces


def test_terminal_arrival_does_not_invent_native_jerk_or_retain_reference(graph):
    env = smooth_bare(graph)
    fake_motion(env, [.2])
    env.step({'agent_0': np.array([0., 0.], np.float32)})
    def arrival():
        env.time += .1
        env.states = {}
        env.arrived = {'agent_0'}
    env._advance = arrival
    _, rewards, terminated, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
    assert terminated['agent_0'] and not env._previous_reference_residual
    assert infos['agent_0']['realized_jerk_m_s3'] is None
    assert infos['agent_0']['reward_terms']['jerk'] == 0.
    report = env.smooth_metrics.report()['per_car']['agent_0']
    assert report['route_completed']
    assert report['all']['decisions'] == 2 and report['all']['measured_samples'] == 1
    assert report['all']['jerk_m_s3']['realized']['samples'] == 1


def test_empty_agent_native_tail_is_measured_without_fake_decisions(graph):
    env = smooth_bare(graph)
    env.agents = []
    env.obs = {}
    env.states = {'native': dict(native_vehicle('native'), acceleration=-1.)}
    def advance():
        env.time += .1
        env.states = {'native': dict(env.states['native'], acceleration=2., speed=20.2, distance=102.)}
    env._advance = advance
    assert env.step({}) == ({}, {}, {}, {}, {})
    report = env.smooth_metrics.report()
    assert report['all']['decisions'] == 0
    assert report['all']['jerk_m_s3']['realized']['peak_abs'] == 30.
    assert report['constraint_unknown']['measured_samples'] == 1


def test_direct_native_tail_advance_hook_records_once_and_ignores_reset_warmup(graph, monkeypatch):
    env = smooth_bare(graph)
    env._measurement_phase = True
    env.agents, env.obs = [], {}
    env.states = {'native': dict(native_vehicle('native'), acceleration=-1.)}
    def advance(self):
        self.time += .1
        self.states = {'native': dict(self.states['native'], acceleration=2., speed=20.2, distance=102.)}
    monkeypatch.setattr(HighwayEnvironmentV05Env, '_advance', advance)
    env._advance()
    assert env.smooth_metrics.report()['all']['jerk_m_s3']['realized']['samples'] == 1
    env.step({})
    assert env.smooth_metrics.report()['all']['jerk_m_s3']['realized']['samples'] == 2
    env._measurement_phase = False
    env._advance()
    assert env.smooth_metrics.report()['all']['jerk_m_s3']['realized']['samples'] == 2


def test_assigned_controlled_identity_is_retained_during_native_baseline():
    metrics = SmoothControlMetrics(controlled_ids=['a'])
    before = {'a': state(), 'human': state()}
    after = {'a': state(acceleration=1.), 'human': state(acceleration=-4.5)}
    metrics.observe(.1, before, after, {})
    report = metrics.report()
    assert report['controlled']['jerk_m_s3']['realized']['peak_abs'] == 10.
    assert report['background']['jerk_m_s3']['realized']['peak_abs'] == 45.
    assert report['controlled']['decisions'] == 0
    assert report['per_car']['a']['assigned_controlled']


def state(speed=10., acceleration=0., lane='ramp_0', kind='ramp', distance=0., limit=20.):
    return dict(speed=speed, acceleration=acceleration, lane_id=lane,
        lane_identity={'kind': kind}, allowed_speed=limit, distance=distance)


def info(request=0., reference=0., **kwargs):
    return dict(requested_acceleration=request, filtered_acceleration=request,
        feasible_acceleration_target=reference, command_actually_applied=True, **kwargs)


def test_disappearance_and_measurement_gaps_do_not_bridge_derivatives():
    metrics = SmoothControlMetrics()
    a, b = state(), state(acceleration=2.)
    metrics.observe(.1, {'a': a}, {'a': b}, {'a': info()})
    metrics.observe(.2, {'a': b}, {}, {})
    metrics.observe(.3, {}, {'a': a}, {'a': info(request=1.)})
    metrics.observe(.6, {'a': a}, {'a': b}, {'a': info(request=2.)})
    rows = metrics.traces['a'][1:]
    assert rows[0]['realized_jerk_m_s3'] == 20.
    assert all(r['requested_jerk_m_s3'] is None for r in rows)
    assert rows[1]['realized_jerk_m_s3'] is None and rows[2]['realized_jerk_m_s3'] is None
    assert metrics.report()['all']['jerk_m_s3']['realized']['missing_samples'] == 2


def test_merge_slowdown_keeps_incomplete_coverage_and_braking_classification():
    metrics = SmoothControlMetrics(task='merge', merge_vehicle_ids=['a'])
    before = {'a': state(speed=10.)}
    for index, speed in enumerate((11., 9., 6., 7., 8.), 1):
        lane, kind = (':junction_0', 'transition') if index == 3 else ('aux_0', 'auxiliary') if index >= 4 else ('ramp_0', 'ramp')
        after = {'a': state(speed=speed, lane=lane, kind=kind, distance=index)}
        details = info(reference=-1. if index in (2, 3) else 1.,
            merge_completed=index == 5, maneuver={'in_progress': index == 4})
        metrics.observe(index * .1, before, after, {'a': details})
        before = after
    result = metrics.report()['per_car']['a']
    slowdown = result['merge_slowdown']
    assert slowdown['observed_loss_m_s'] == 5.
    assert slowdown['fraction_of_preceding_peak'] == pytest.approx(5 / 11)
    assert slowdown['peak_time_s'] == .1 and slowdown['trough_time_s'] == pytest.approx(.3)
    assert not slowdown['coverage_complete']
    assert slowdown['slowdown_interval']['reference_braking_samples'] == 2
    assert set(result['phases']) == {'approach', 'native_join', 'lateral_maneuver', 'post_merge'}


def test_missing_merge_events_are_incomplete_not_fake_zero():
    metrics = SmoothControlMetrics(task='merge', merge_vehicle_ids=['a'])
    metrics.observe(.1, {'a': state(speed=10.)}, {'a': state(speed=5.)}, {'a': info()})
    result = metrics.report()['per_car']['a']['merge_slowdown']
    assert result['observed_loss_m_s'] == 5 and not result['coverage_complete']
    assert result['entry_time_s'] is None and result['settled_completion_time_s'] is None


def test_distribution_denominators_include_missing_and_uncensored_tail():
    result = distribution([None, -1., 100.], eligible=3)
    assert result['samples'] == 2 and result['missing_samples'] == 1
    assert result['peak_abs'] == 100. and result['mean_abs'] == 50.5
    assert distribution([], eligible=0)['peak_abs'] is None


def test_nominal_merge_stop_measures_intervals_including_reset_until_stable():
    trace = [dict(time_s=i * .1, speed_m_s=0.) for i in range(26)]
    result = nominal_merge_stop(trace, 2.)
    assert result['coverage_complete'] and result['longest_consecutive_stop_s'] == 2.
    assert result['start_time_s'] == 0. and result['end_time_s'] == 2.
    assert result['observed_end_time_s'] == 2.
    pending = nominal_merge_stop(trace, None)
    assert not pending['coverage_complete'] and pending['longest_consecutive_stop_s'] == 2.5


def test_nominal_merge_stop_never_bridges_gaps_or_arrival_and_single_sample_is_zero():
    trace = [dict(time_s=t, speed_m_s=v) for t, v in [(0., 0.), (.1, 0.), (.3, 0.), (.4, 0.), (.5, None)]]
    result = nominal_merge_stop(trace, .5)
    assert not result['coverage_complete'] and result['nonconsecutive_gaps'] == 1
    assert result['longest_consecutive_stop_s'] == pytest.approx(.1)
    assert nominal_merge_stop([dict(time_s=0., speed_m_s=0.)], None)['longest_consecutive_stop_s'] == 0.


def test_report_pins_first_successful_stable_time_and_no_stop_zero():
    metrics = SmoothControlMetrics(task='merge', merge_vehicle_ids=['a'], controlled_ids=['a'])
    before = {'a': state()}
    for index in range(1, 4):
        metrics.observe(index * .1, before, before, {'a': info(stable_merge_status='succeeded' if index >= 2 else 'pending')})
    result = metrics.report()['per_car']['a']['nominal_merge_stop']
    assert result['stable_completion_time_s'] == .2 and result['coverage_complete']
    assert result['longest_consecutive_stop_s'] == 0 and result['start_time_s'] is None
