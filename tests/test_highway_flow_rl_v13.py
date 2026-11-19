"""V13 yield-aware merge target: the pure rule and the environment hooks (no SUMO, no training)."""
import math
from types import SimpleNamespace

import pytest

from traffic import highway_flow_rl_v13_rewards as v13

S = v13.SETTINGS
TARGET = dict(eligible=True, target_m_s=16.3, reasons=[], bounds=[dict(reason='merge_target_limit', ceiling_m_s=20.1)])
ENTRY = dict(entry_active=True, target_lane='sb2_0', distance_to_start_m=27.5, known=True)
# TraCI getNextLinks order (traci/_lane.py): lane, hasPrio, isOpen, hasFoe, via, state, direction, length
CLOSED = ('sb2_0', False, False, True, ':n1_0_0', 'm', 's', 20.86)
OPEN = ('sb2_0', False, True, True, ':n1_0_0', 'm', 's', 20.86)
LATER = ('sb3_0', True, True, False, ':n2_0_0', 'M', 's', .32)


# --- the pure rule -------------------------------------------------------------------
def test_link_is_read_by_target_lane_in_traci_tuple_order():
    assert v13.yield_link((CLOSED, LATER), 'sb2_0') == dict(found=True, open=False, foe=True, priority=False,
                                                           via=':n1_0_0', state='m')
    assert v13.yield_link((OPEN, LATER), 'sb2_0')['open'] is True
    assert v13.yield_link((LATER,), 'sb2_0')['found'] is False
    assert v13.yield_link((), 'sb2_0')['found'] is False
    with pytest.raises(ValueError):
        v13.yield_link((('sb2_0', False),), 'sb2_0')


def test_stop_bound_is_v03s_unseen_yield_stop_form():
    for distance in (0., .1, 2.5, 3., 27.5, 100., 375.8):
        assert v13.stop_bound(distance) == pytest.approx(math.sqrt(4. * max(0., distance - 2.5)))
    assert v13.stop_bound(27.5) == pytest.approx(10.)
    with pytest.raises(ValueError):
        v13.stop_bound(float('nan'))


def test_closed_link_caps_the_target_and_never_raises_it():
    capped, applied = v13.apply_yield(TARGET, ENTRY, v13.yield_link((CLOSED,), 'sb2_0'))
    assert applied and capped['target_m_s'] == pytest.approx(10.) and capped['v12_target_m_s'] == 16.3
    assert capped['bounds'][-1] == dict(reason='closed_yield_stop', ceiling_m_s=pytest.approx(10.))
    assert capped['yield_link']['distance_to_line_m'] == 27.5
    assert TARGET['target_m_s'] == 16.3 and len(TARGET['bounds']) == 1        # input untouched
    held, _ = v13.apply_yield(TARGET, dict(ENTRY, distance_to_start_m=.1), v13.yield_link((CLOSED,), 'sb2_0'))
    assert held['target_m_s'] == 0.                                          # waiting at the line is on target
    far, applied = v13.apply_yield(TARGET, dict(ENTRY, distance_to_start_m=300.), v13.yield_link((CLOSED,), 'sb2_0'))
    assert applied and far['target_m_s'] == 16.3                             # cap above the target: no change


@pytest.mark.parametrize('target, entry, links', [
    (TARGET, ENTRY, (OPEN,)),                                                 # SUMO admits the car
    (TARGET, ENTRY, (LATER,)),                                                # link not listed
    (TARGET, dict(ENTRY, entry_active=False), (CLOSED,)),                     # not a yield entry
    (dict(TARGET, eligible=False), ENTRY, (CLOSED,)),                         # no target to cap
    (None, ENTRY, (CLOSED,)),
])
def test_everything_else_keeps_the_v12_target(target, entry, links):
    result, applied = v13.apply_yield(target, entry, v13.yield_link(links, 'sb2_0'))
    assert result is target and not applied


def test_settings_are_pinned():
    assert v13.validate_settings(dict(S)) == S
    for bad in ({}, dict(S, stop_decel_m_s2=3.), dict(S, extra=1)):
        with pytest.raises(ValueError):
            v13.validate_settings(bad)


# --- environment hooks (logic only) --------------------------------------------------
def _env(monkeypatch, links=(CLOSED,), entry=ENTRY, error=None):
    pytest.importorskip('torch')
    from environments import highway_flow_rl_v12_env as m12
    from environments import highway_flow_rl_v06_env as m06
    from environments import highway_flow_rl_v13_env as module
    env = object.__new__(module.HighwayFlowRLV13Env)
    env.v13_settings = dict(S)
    env.time = 1.2
    env.graph = None
    env.states = dict(agent_0=dict(speed=9., acceleration=1.5))
    env.flow_contexts = dict(agent_0=dict(entry=dict(entry)))
    env.merge_trackers = dict(agent_0=SimpleNamespace(status='pending'))
    env._reference_targets = {}
    env._links, env._links_time = {}, None
    env.v13_metrics = dict(yield_decisions=0, link_missing=0, link_closed=0, link_open=0,
                           bounded_decisions=0, open_while_braking=0, per_car={})
    calls = []

    def next_links(agent):
        calls.append((agent, env.time))
        if error:
            raise error
        return links
    env.conn = SimpleNamespace(vehicle=SimpleNamespace(getNextLinks=next_links))

    def parent_target(self, agent, state, context, preview):
        self._reference_targets[agent] = dict(TARGET)
        return dict(TARGET), .7
    monkeypatch.setattr(m06.HighwayFlowRLV06Env, '_reward_target', parent_target)
    monkeypatch.setattr(m12, 'map_evidence', lambda graph, state: None)
    monkeypatch.setattr(m12, 'merge_speed_target', lambda state, context, preview: (dict(TARGET), {}))
    return env, module, calls


def test_both_hooks_cap_with_one_query_per_car_and_step(monkeypatch):
    env, module, calls = _env(monkeypatch)
    context = env.flow_contexts['agent_0']
    target, potential = env._reward_target('agent_0', env.states['agent_0'], context, None)
    assert potential == .7 and target['target_m_s'] == pytest.approx(10.)
    assert env._reference_targets['agent_0'] is target                        # the gate grades the capped target
    actor_target, pending, _ = env.current_target('agent_0')
    assert pending and actor_target['target_m_s'] == pytest.approx(10.)      # the actor sees the same number
    assert calls == [('agent_0', 1.2)]
    env.time = 1.3
    env.current_target('agent_0')
    assert calls == [('agent_0', 1.2), ('agent_0', 1.3)]
    metrics = env.v13_metrics
    assert (metrics['yield_decisions'], metrics['link_closed'], metrics['bounded_decisions']) == (1, 1, 1)
    assert metrics['per_car']['agent_0']['link_closed'] == 1


def test_open_link_and_braking_are_counted_and_keep_the_v12_target(monkeypatch):
    env, module, calls = _env(monkeypatch, links=(OPEN,))
    env.states['agent_0']['acceleration'] = -4.5
    target, _ = env._reward_target('agent_0', env.states['agent_0'], env.flow_contexts['agent_0'], None)
    assert target['target_m_s'] == 16.3
    metrics = env.v13_metrics
    assert (metrics['link_open'], metrics['open_while_braking'], metrics['bounded_decisions']) == (1, 1, 0)


def test_missing_link_or_query_error_keeps_the_v12_target(monkeypatch):
    import traci
    for kwargs in (dict(links=(LATER,)), dict(error=traci.TraCIException('vehicle left'))):
        env, module, calls = _env(monkeypatch, **kwargs)
        target, _ = env._reward_target('agent_0', env.states['agent_0'], env.flow_contexts['agent_0'], None)
        assert target['target_m_s'] == 16.3 and env.v13_metrics['link_missing'] == 1


def test_acceleration_lane_merges_never_query_sumo(monkeypatch):
    env, module, calls = _env(monkeypatch, entry=dict(ENTRY, entry_active=False, minor_link_visibility_m=0.))
    target, _ = env._reward_target('agent_0', env.states['agent_0'], env.flow_contexts['agent_0'], None)
    actor_target, _, _ = env.current_target('agent_0')
    assert target == TARGET and actor_target == TARGET and calls == []
    assert env.v13_metrics['yield_decisions'] == 0


def test_environment_refuses_unreviewed_settings():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v13_env import HighwayFlowRLV13Env
    with pytest.raises(ValueError):
        HighwayFlowRLV13Env('missing.net.xml', 'outputs', v13_settings=dict(S, stop_margin_m=0.))
