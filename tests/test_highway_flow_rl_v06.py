import xml.etree.ElementTree as ET

import numpy as np
import pytest

from traffic import highway_flow_rl_v06_rewards as v06
from traffic.highway_merge_context import Stage1LaneGraph
from traffic.highway_v05_demand import CAP, MAIN_ROUTE, write_demand


# --- faded actor merge inputs -------------------------------------------------
def test_first_observation_is_raw_and_input_is_not_modified():
    raw = np.zeros(117, np.float32)
    raw[28] = 1.
    out = v06.fade_observation(None, raw)
    assert np.array_equal(out, raw) and out is not raw


def test_binary_merge_flip_fades_over_ten_decisions():
    previous = np.zeros(117, np.float32)
    raw = previous.copy()
    raw[[28, 33, 39]] = 1.
    seen = []
    for _ in range(12):
        previous = v06.fade_observation(previous, raw)
        seen.append(round(float(previous[28]), 4))
    assert seen[0] == pytest.approx(.1) and seen[8] < 1. and seen[9] == pytest.approx(1.) and seen[11] == 1.


def test_mask_ceiling_and_non_merge_fields_are_never_faded():
    previous = np.zeros(117, np.float32)
    raw = np.zeros(117, np.float32)
    untouched = [i for i in range(117) if i not in v06.FADE_INDICES]
    raw[untouched] = .9
    out = v06.fade_observation(previous, raw)
    assert np.array_equal(out[untouched], raw[untouched])
    assert {30, 31, 32, 43}.isdisjoint(v06.FADE_INDICES)


def test_slow_continuous_changes_pass_through():
    previous = np.zeros(117, np.float32)
    previous[37] = .50
    raw = previous.copy()
    raw[37] = .53
    assert v06.fade_observation(previous, raw)[37] == pytest.approx(.53)


# --- merge-guidance potential -----------------------------------------------
def test_potential_fades_from_one_to_plan_value():
    plan = dict(required=True, known=True, potential=.2)
    values = [v06.blend_potential(plan, run)['potential'] for run in range(1, 13)]
    assert values[0] == 1. and values[10] == pytest.approx(.2) and values[11] == pytest.approx(.2)
    assert all(a >= b for a, b in zip(values, values[1:]))


def test_potential_without_required_merge_is_unchanged():
    plan = dict(required=False, known=True, potential=1.)
    assert v06.blend_potential(plan, 0) == plan


# --- reward terms -----------------------------------------------------------
def test_jerk_excess_at_observed_peak_is_comparable_to_a_guidance_step():
    cost = v06.jerk_excess(8.39, 2., v06.WEIGHTS['jerk_excess'])
    assert cost == pytest.approx(-1.5 * ((8.39 - 5.) / 5.) ** 2)
    assert v06.jerk_excess(4.9, -4.9, 1.5) == 0.
    # The raw request counts even when the executed command was smooth.
    assert v06.jerk_excess(1., -9., 1.5) < 0.
    # Capped at 10 m/s3: an emergency stop costs at most one weight per step.
    assert v06.jerk_excess(50., -45., 1.5) == v06.jerk_excess(10., 0., 1.5) == -1.5


def test_merge_tracking_uses_gate_tolerance_and_only_while_merging():
    target = dict(eligible=True, target_m_s=18.)
    assert v06.merge_tracking(target, 17., True, .5) == pytest.approx(-.25)
    assert v06.merge_tracking(target, 10., True, .5) == -.5
    assert v06.merge_tracking(target, 10., False, .5) == 0.
    assert v06.merge_tracking(dict(eligible=False), 10., True, .5) == 0.


def test_avoidable_braking_only_when_nothing_requires_braking():
    eligible = dict(eligible=True)
    far = dict(bumper_gap_m=100.)
    assert v06.avoidable_braking(eligible, -1.2, 20., far, .05) == pytest.approx(-.025)
    assert v06.avoidable_braking(eligible, -.1, 20., far, .05) == 0.           # not braking
    assert v06.avoidable_braking(dict(eligible=False), -2., 20., far, .05) == 0.  # bound or lower target
    assert v06.avoidable_braking(eligible, -2., 20., dict(bumper_gap_m=25.), .05) == 0.  # leader inside 1.5 s
    assert v06.avoidable_braking(eligible, -4.5, 20., None, .05) == -.05


def test_late_acceleration_lane_metric():
    context = lambda reachable, distance: dict(corridors=dict(own=dict(route=dict(reachable=reachable, distance_m=distance))))
    assert v06.late_in_acceleration_lane(context(False, 15.), 100., True)
    assert not v06.late_in_acceleration_lane(context(False, 30.), 100., True)
    assert not v06.late_in_acceleration_lane(context(True, 15.), 100., True)
    assert not v06.late_in_acceleration_lane(context(False, 15.), 100., False)


def _report(peaks):
    return dict(v05_metrics=dict(per_car={a: dict(peak_raw_requested_jerk_m_s3=p) for a, p in peaks.items() if p is not None}),
                smooth_control=dict(per_car={a: dict(assigned_controlled=True) for a in peaks}))


def test_raw_request_jerk_check_fails_on_spike_and_on_missing_evidence():
    assert v06.raw_request_jerk_check(_report(dict(agent_0=4.)))['passed']
    assert not v06.raw_request_jerk_check(_report(dict(agent_0=4., agent_1=8.4)))['passed']
    assert not v06.raw_request_jerk_check(_report(dict(agent_0=None)))['passed']
    assert not v06.raw_request_jerk_check(_report({}))['passed']


def test_round6_scope_skips_only_the_insertion_decision():
    report = _report(dict(agent_0=13.9, agent_1=4.))  # every-decision peaks (first decision 13.9)
    report['v06_metrics'] = dict(per_car={'agent_0': {v06.FIRST_DECISION_EXCLUDED_KEY: 2.1},
                                          'agent_1': {v06.FIRST_DECISION_EXCLUDED_KEY: 4.}})
    assert not v06.raw_request_jerk_check(report)['passed']
    later = v06.raw_request_jerk_check(report, scope='after_first_decision')
    assert later['passed'] and later['scope'] == 'after_first_decision'
    report['v06_metrics']['per_car']['agent_1'][v06.FIRST_DECISION_EXCLUDED_KEY] = 6.
    assert not v06.raw_request_jerk_check(report, scope='after_first_decision')['passed']
    del report['v06_metrics']['per_car']['agent_1']  # missing evidence still fails
    assert not v06.raw_request_jerk_check(report, scope='after_first_decision')['passed']
    with pytest.raises(ValueError):
        v06.raw_request_jerk_check(report, scope='none')


def test_round8_scope_reads_the_fixed_insertion_lag_key():
    report = _report(dict(agent_0=13.9))
    report['v06_metrics'] = dict(per_car={'agent_0': {v06.FIRST_DECISION_EXCLUDED_KEY: 12.,
                                                      v06.INSERTION_LAG_EXCLUDED_KEY: 4.2}})
    assert not v06.raw_request_jerk_check(report, scope='after_first_decision')['passed']
    lag = v06.raw_request_jerk_check(report, scope='after_insertion_lag')
    assert lag['passed'] and lag['scope'] == 'after_insertion_lag'
    assert lag['per_car']['agent_0']['peak_raw_requested_jerk_m_s3'] == 4.2
    del report['v06_metrics']['per_car']['agent_0'][v06.INSERTION_LAG_EXCLUDED_KEY]
    assert not v06.raw_request_jerk_check(report, scope='after_insertion_lag')['passed']
    assert v06.INSERTION_LAG_DECISIONS == 5


def test_environment_scores_requests_after_first_decision_and_after_insertion_lag():
    pytest.importorskip('torch')
    from types import SimpleNamespace
    from environments.highway_flow_rl_v06_env import HighwayFlowRLV06Env
    env = object.__new__(HighwayFlowRLV06Env)
    env.v06_weights, env._v05_seen = dict(v06.WEIGHTS), set()
    env.graph = SimpleNamespace(lanes={})
    env._reset_v06()
    snapshot = dict(ego_state=dict(speed=20., lane_id='x'), context={}, target={}, gap_plan={}, time_s=0.)
    for raw in (14., 12., 9., 6., 5.5, 4., 3.):  # an insertion lag, then ordinary driving
        info = dict(realized_jerk_m_s3=4., v04_reward_audit={})
        env._extra_rewards('agent_0', info, snapshot, dict(raw_requested_jerk=raw, requested=0.), None, False)
        env._v05_seen.add('agent_0')
    car = env.v06_metrics['per_car']['agent_0']
    assert car['decisions'] == 7 and car[v06.FIRST_DECISION_EXCLUDED_KEY] == 12.
    assert car[v06.INSERTION_LAG_EXCLUDED_KEY] == 4.


def test_v06_gates_take_scope_from_report_and_report_both(monkeypatch):
    pytest.importorskip('torch')
    import experiments.run_highway_flow_rl_v05 as runner
    monkeypatch.setattr(runner, 'development_gates', lambda report: dict(passed=True, failures=[]))
    report = _report(dict(agent_0=13.9))
    report['v06_metrics'] = dict(per_car={'agent_0': {v06.FIRST_DECISION_EXCLUDED_KEY: 2.}})
    assert not v06.v06_gates(report)['passed']  # round 5: no scope key -> every decision
    report['v06_raw_request_scope'] = 'after_first_decision'
    gates = v06.v06_gates(report)
    assert gates['passed'] and not gates['raw_request_jerk_all_decisions']['passed']


# --- environment guards (logic only, no SUMO) -------------------------------
def test_environment_rejects_unreviewed_weights_and_unknown_rate_limit():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v06_env import HighwayFlowRLV06Env
    with pytest.raises(ValueError):
        HighwayFlowRLV06Env('net', 'dir', v06_weights=dict(v06.WEIGHTS, jerk_excess=9.))
    with pytest.raises(ValueError):
        HighwayFlowRLV06Env('net', 'dir', rate_limit='none')
    with pytest.raises(ValueError):
        HighwayFlowRLV06Env('net', 'dir', v05_variant='legacy')
    with pytest.raises(ValueError):
        HighwayFlowRLV06Env('net', 'dir', raw_request_scope='second_half')


def test_pre_action_and_successor_potentials_agree_for_the_same_state(monkeypatch):
    pytest.importorskip('torch')
    import environments.highway_flow_rl_v06_env as module
    env = module.HighwayFlowRLV06Env.__new__(module.HighwayFlowRLV06Env)
    env._reset_v05(); env._reset_v06()
    env.graph, env.states = None, {}
    plans = iter([dict(required=False, potential=1.)] + [dict(required=True, known=True, potential=.3)] * 30)
    current = {}
    def fake_plan(*_args, **_kwargs):
        return current['plan']
    monkeypatch.setattr(module, 'gap_plan', fake_plan)
    monkeypatch.setattr(module, 'flow_context', lambda *a: None)
    monkeypatch.setattr(module, 'merge_speed_target', lambda *a: (dict(eligible=False), current['plan']))
    current['plan'] = next(plans)
    _, before = env._reward_target('agent_0', {}, None, None)
    for _ in range(15):
        current['plan'] = next(plans)
        successor = env._successor_plan('agent_0', {'speed': 1.})
        _, pre = env._reward_target('agent_0', {}, None, None)
        assert pre['potential'] == pytest.approx(successor['potential'])  # telescoping shaping
        before = pre
    assert before['potential'] == pytest.approx(.3)


# --- missed-gap demand ------------------------------------------------------
@pytest.fixture
def graph(tmp_path):
    root = ET.Element('net')
    for edge, count, length in [(edge, 3, 400.) for edge in MAIN_ROUTE] + [('mlk_on', 1, 350.)]:
        element = ET.SubElement(root, 'edge', id=edge)
        for index in range(count):
            ET.SubElement(element, 'lane', id=f'{edge}_{index}', index=str(index), length=str(length), speed=str(CAP))
    for source, target in zip(MAIN_ROUTE, MAIN_ROUTE[1:]):
        for lane in range(3):
            ET.SubElement(root, 'connection', {'from': source, 'to': target, 'fromLane': str(lane), 'toLane': str(lane)})
    ET.SubElement(root, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0'})
    path = tmp_path / 'network.xml'
    ET.ElementTree(root).write(path)
    return Stage1LaneGraph(path, 3)


def test_default_demand_is_byte_identical(graph, tmp_path):
    write_demand(tmp_path / 'a.xml', 11, graph, 'merge', 1, 8)
    write_demand(tmp_path / 'b.xml', 11, graph, 'merge', 1, 8, missed_gap=False)
    assert (tmp_path / 'a.xml').read_bytes() == (tmp_path / 'b.xml').read_bytes()


def test_missed_gap_shifts_only_the_right_lane_platoon(graph, tmp_path):
    for seed in range(5):
        normal = write_demand(tmp_path / f'n{seed}.xml', seed, graph, 'merge', 1, 8)
        missed = write_demand(tmp_path / f'm{seed}.xml', seed, graph, 'merge', 1, 8, missed_gap=True)
        info = missed.scenario['missed_gap']
        by_slot = {r['slot']: r for r in normal}
        moved = set()
        for row in missed:
            old = by_slot[row['slot']]
            assert row['speed'] == old['speed'] and row['driver_profile'] == old['driver_profile']
            if (row['lane_id'], row['position']) != (old['lane_id'], old['position']):
                moved.add(row['slot'])
                assert graph.identity(row['lane_id'])['index_from_right'] == 0
        assert info['blocker_slot'] in by_slot and info['predicted_arrival_s'] > 0.
        blocker = next(r for r in missed if r['slot'] == info['blocker_slot'])
        predicted = graph.lanes['sb1_0']['length'] - blocker['speed'] * info['predicted_arrival_s']
        assert blocker['edge'] == 0 and blocker['position'] == pytest.approx(predicted)
        assert normal.scenario['missed_gap'] is None and moved


def test_missed_gap_requires_merge_task(graph, tmp_path):
    with pytest.raises(ValueError):
        write_demand(tmp_path / 'x.xml', 1, graph, 'cruise', 1, 8, missed_gap=True)
