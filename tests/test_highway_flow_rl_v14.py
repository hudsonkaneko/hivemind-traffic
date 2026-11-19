"""V14 policy-only control and yield-entry merge target (no SUMO, no training)."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from traffic import highway_flow_rl_v04_rewards as v04
from traffic import highway_flow_rl_v14_rewards as v14

S = v14.SETTINGS
# Logged decision t = 0.2 s of the policy-only probe's focused northbound LA merge
# (outputs/test-runs/policy-only-probe1-stopped, case 10): the decision context in which
# v12's target sent the car through the yield line in front of main_13.
CRASH_STATE = dict(speed=9.0, allowed_speed=20.1)
CRASH_CONTEXT = dict(
    entry=dict(known=True, target_lane='sb2_0', target_index=0, lane_delta=0, distance_to_start_m=30.17,
               front=None, rear=dict(id='main_13', bumper_gap_m=27.98, relative_speed_m_s=10.62),
               target_speed_limit=26.8224, projection='shared_entry_stopline', source_lane='mlk_on_0',
               minor_link_visibility_m=100., entry_active=True),
    corridors=dict(own=dict(lane_id='mlk_on_0', known=True, legal_speed_m_s=20.1,
                            route=dict(known=True, reachable=True, distance_m=411.2), front=None)))


def accel_lane_context():
    context = deepcopy(CRASH_CONTEXT)
    context['entry'].update(entry_active=False, minor_link_visibility_m=0., lane_delta=-1)
    context['corridors']['own']['route'] = dict(known=True, reachable=False, distance_m=180.)
    return context


# --- the pure rule -------------------------------------------------------------------
def test_control_and_settings_are_pinned_to_their_sources():
    assert v14.CONTROL == dict(speed_mode=96, lane_change_mode=0)
    assert S == dict(crossing_window_s=3.1, crossing_step_s=.1, target_acceleration_m_s2=1.5,
                     target_deceleration_m_s2=2., initial_preview_s=3.)
    assert v14.validate_settings(dict(S)) == S
    for bad in ({}, dict(S, crossing_window_s=2.), dict(S, extra=1)):
        with pytest.raises(ValueError):
            v14.validate_settings(bad)


def test_crossing_time_follows_v04_candidate_motion():
    assert v14.crossing_time(10., 10., 3., 30.) == pytest.approx(3.)
    assert v14.crossing_time(0., 3., 3., 1.5) == pytest.approx(3 ** .5)     # 0.5 * 1 m/s^2 * t^2 = 1.5 m
    assert v14.crossing_time(0., 3., 3., 4.5) == pytest.approx(3.)
    assert v14.crossing_time(0., 3., 3., 7.5) == pytest.approx(4.)          # 4.5 m by 3 s, then 3 m/s
    assert v14.crossing_time(9., 0., 6., 30.) is None                         # stops 27 m out
    assert v14.crossing_time(9., 0., 6., 20.) is not None                     # cannot stop in 20 m at 1.5 m/s^2
    assert v14.crossing_time(5., 5., 3., 0.) == 0.


def test_v12_target_in_the_logged_crash_crosses_in_front_of_main_13():
    old = v04.gap_plan(CRASH_STATE, CRASH_CONTEXT)
    assert old['slot'] == 'behind_rear' and old['target_m_s'] == pytest.approx(16.33125)
    rear = dict(id='main_13', x=-27.98 - 4.8, speed=9. + 10.62)
    deficit, crossing = v14.crossing_deficit(9., old['target_m_s'], old['prediction_horizon_s'], 30.17, rear, None)
    assert deficit > .5 and crossing < 3.


def test_v14_plan_lets_main_13_pass_before_crossing():
    plan = v14.gap_plan(CRASH_STATE, CRASH_CONTEXT)
    assert plan['slot'] == 'behind_rear' and plan['crossing_deficit'] == 0.
    assert plan['target_m_s'] < 4. and plan['crossing_time_s'] > 4.


def test_away_from_yield_entries_the_plan_and_target_are_v04s():
    context = accel_lane_context()
    assert v14.gap_plan(CRASH_STATE, context) == v04.gap_plan(CRASH_STATE, context)
    target, plan, _ = v14.merge_speed_target(CRASH_STATE, context, None)
    old_target, old_plan = v04.merge_speed_target(CRASH_STATE, context, None)
    assert target == old_target and plan == old_plan


def test_yield_plan_is_v04s_plan_when_no_candidate_conflicts(monkeypatch):
    monkeypatch.setattr(v14, 'crossing_deficit', lambda *args, **kwargs: (0., None))
    for rear in (CRASH_CONTEXT['entry']['rear'], None, dict(id='far', bumper_gap_m=110., relative_speed_m_s=2.)):
        context = deepcopy(CRASH_CONTEXT)
        context['entry']['rear'] = rear
        new, old = v14.gap_plan(CRASH_STATE, context), v04.gap_plan(CRASH_STATE, context)
        for key in ('slot', 'target_m_s', 'prediction_horizon_s', 'cost', 'potential', 'front_id', 'rear_id'):
            assert new[key] == old[key]


def test_reachable_target_moves_at_v05_rates_and_obeys_the_ceiling_at_once():
    assert v14.reachable(16., None, 2., None, 5.) == pytest.approx(6.5)        # start: speed + 1.5 x 3 s
    assert v14.reachable(3., None, 9., None, 5.) == pytest.approx(3.)
    held = dict(time=5., value=0.)
    assert v14.reachable(13.8, None, 0., held, 5.1) == pytest.approx(.15)     # +1.5 m/s^2
    assert v14.reachable(0., None, 9., dict(time=5., value=9.), 5.1) == pytest.approx(8.8)  # -2.0 m/s^2
    assert v14.reachable(13.8, 4., 0., dict(time=5., value=9.), 5.1) == pytest.approx(4.)  # ceiling at once
    # A car already at speed past the line: the rise starts from its speed, not from a low target.
    assert v14.reachable(24.6, None, 17.4, dict(time=5., value=6.), 5.1) == pytest.approx(17.55)
    assert v14.reachable(12., None, 17.4, dict(time=5., value=6.), 5.1) == pytest.approx(12.)
    # A slow car cannot hold the target down: it still rises from the previous target.
    assert v14.reachable(13.8, None, 0., dict(time=5., value=6.), 5.1) == pytest.approx(6.15)
    with pytest.raises(ValueError):
        v14.reachable(float('nan'), None, 0., None, 0.)


# --- environment hooks (logic only) --------------------------------------------------
def _env(monkeypatch, context=None):
    pytest.importorskip('torch')
    from environments import highway_flow_rl_v14_env as module
    env = object.__new__(module.HighwayFlowRLV14Env)
    env.v14_settings = dict(S)
    env.time = 1.
    env.graph = None
    env.states = dict(agent_0=dict(CRASH_STATE))
    env.flow_contexts = dict(agent_0=deepcopy(context or CRASH_CONTEXT))
    env.merge_trackers = dict(agent_0=SimpleNamespace(status='pending'))
    env._reference_targets, env._required_run = {}, {}
    env._v14_cache, env._ramp = {}, {}
    env.v14_metrics = dict(yield_decisions=0, crossing_changed_target=0, crossing_limited_plans=0,
                           ramp_decisions=0, ramp_limited=0, hook_mismatch=0, per_car={})
    calls = []

    def target_stub(state, context, preview):
        # The map/occupied-lane bound needs live map evidence; here it is a fixed 20.1 m/s
        # ceiling and the target is the real v14 gap plan's, as merge_speed_target forms it.
        calls.append(env.time)
        plan = v14.gap_plan(state, context)
        value = plan.get('target_m_s')      # None once no merge is required (past the line)
        return dict(eligible=True, target_m_s=20.1 if value is None else min(20.1, value)), plan, 20.1
    monkeypatch.setattr(module.v14, 'merge_speed_target', target_stub)
    monkeypatch.setattr(module, 'map_evidence', lambda graph, state: None)
    return env, module, calls


def test_both_hooks_share_one_target_per_car_and_time(monkeypatch):
    env, module, calls = _env(monkeypatch)
    actor, pending, _ = env.current_target('agent_0')
    reward, _ = env._reward_target('agent_0', env.states['agent_0'], env.flow_contexts['agent_0'], None)
    assert pending and reward is actor and env._reference_targets['agent_0'] is actor
    assert actor['target_m_s'] < 4. and actor['v14']['yield_entry'] and actor['v14']['ramp_started']
    assert env.v14_metrics['hook_mismatch'] == 0 and env.v14_metrics['yield_decisions'] == 1
    assert env.v14_metrics['crossing_changed_target'] == 1


def test_ramp_continues_past_the_line_until_it_meets_the_plan(monkeypatch):
    env, module, _ = _env(monkeypatch)
    env._ramp['agent_0'] = dict(time=.9, value=0.)                          # held at the line
    past = deepcopy(CRASH_CONTEXT)
    past['entry'].update(entry_active=False, rear=None)
    env.flow_contexts['agent_0'] = past
    env.states['agent_0']['speed'] = 0.
    target, _, _ = env.current_target('agent_0')
    assert target['target_m_s'] == pytest.approx(.15) and not target['v14']['yield_entry']
    assert 'agent_0' in env._ramp
    env.time, env._ramp['agent_0'] = 1.1, dict(time=1., value=1e3)          # caught up: v12's target again
    target, _, _ = env.current_target('agent_0')
    assert 'agent_0' not in env._ramp
    env.time = 1.2
    plain, _, _ = env.current_target('agent_0')
    assert 'v14' not in plain


def test_acceleration_lane_merges_keep_v12s_target(monkeypatch):
    env, module, _ = _env(monkeypatch, context=accel_lane_context())
    target, _, _ = env.current_target('agent_0')
    plan = v04.gap_plan(env.states['agent_0'], env.flow_contexts['agent_0'])
    assert target == dict(eligible=True, target_m_s=min(20.1, plan['target_m_s']))
    assert env._ramp == {} and env.v14_metrics['ramp_decisions'] == 0


def test_reset_hands_every_controlled_car_to_the_policy_and_drain_restores_native(monkeypatch):
    pytest.importorskip('torch')
    from environments import highway_flow_rl_v12_env as m12
    from environments import highway_flow_rl_v14_env as module
    modes = {}
    vehicle = SimpleNamespace(
        setSpeedMode=lambda a, m: modes.setdefault(a, {}).update(speed=m),
        setLaneChangeMode=lambda a, m: modes.setdefault(a, {}).update(lane_change=m),
        getSpeedMode=lambda a: modes[a]['speed'], getLaneChangeMode=lambda a: modes[a]['lane_change'])
    env = object.__new__(module.HighwayFlowRLV14Env)
    env.conn = SimpleNamespace(vehicle=vehicle)
    env.states = dict(agent_0={}, agent_1={})
    env._controlled_agents = lambda: ('agent_0', 'agent_1')
    monkeypatch.setattr(m12.HighwayFlowRLV12Env, 'reset', lambda self, seed=None, options=None: ({}, {}))
    env.reset(seed=1)
    assert modes == dict(agent_0=dict(speed=96, lane_change=0), agent_1=dict(speed=96, lane_change=0))
    drained = []
    monkeypatch.setattr(m12.HighwayFlowRLV12Env, 'finish_accounting',
                        lambda self: drained.append({a: dict(m) for a, m in modes.items()}) or {})
    env.finish_accounting()
    assert drained[0]['agent_0']['speed'] == 31 and drained[0]['agent_1']['speed'] == 31


def test_environment_refuses_unreviewed_settings():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
    with pytest.raises(ValueError):
        HighwayFlowRLV14Env('missing.net.xml', 'outputs', v14_settings=dict(S, crossing_step_s=.5))
