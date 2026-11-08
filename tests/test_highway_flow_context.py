import numpy as np
import pytest

from environments.highway_environment_v05 import ContinuousMotionLaneGraph
from traffic.highway_flow_context import (FIELD_INDEX, LOCAL_SIZE, OBSERVATION_SIZE, STATE_SIZE,
    LEGACY_TO_FLOW, inject_observation, legacy_observation, flow_context, merge_context, route_continuation)
from tests.test_highway_merge_context import graph, RAMP
from tests.test_highway_environment_v05 import native_vehicle


def test_message_offsets_and_exact_explicit_roundtrip():
    old = np.linspace(-1., 1., 99, dtype=np.float32)
    obs = inject_observation(old, np.zeros(18))
    assert (LOCAL_SIZE, OBSERVATION_SIZE, STATE_SIZE) == (69, 117, 2832)
    np.testing.assert_array_equal(obs[:51], old[:51])
    np.testing.assert_array_equal(obs[69:], old[51:])
    np.testing.assert_array_equal(legacy_observation(obs), old)
    assert FIELD_INDEX['left_legal_speed_overcap'] == 68
    assert len(LEGACY_TO_FLOW) == 99
    with pytest.raises(ValueError):
        legacy_observation(old)
    with pytest.raises(ValueError):
        inject_observation(old, np.full(18, np.nan))


def test_farther_slower_occupied_corridor_is_not_dropped(graph):
    graph.__class__ = ContinuousMotionLaneGraph
    ego = native_vehicle('ego', 'sb2_1', 100., speed=16.)
    ego.update(lateral_position=-.15, lateral_speed=-1.)
    states = {'ego': ego, 'fast': native_vehicle('fast', 'sb2_1', 140., speed=15.5),
              'slow': native_vehicle('slow', 'sb2_0', 190., speed=10.)}
    context = flow_context(graph, ego, states)
    assert context['corridors']['own']['front']['id'] == 'fast'
    assert context['corridors']['right']['front']['id'] == 'slow'
    assert context['occupied_worst_acceleration_m_s2'] < context['corridors']['own']['native_response_estimate_m_s2']
    assert context['limiting_occupied_lane'] == 'sb2_0'
    assert len(context['extras']) == 18
    assert not any(not np.isfinite(x) or abs(x) > 1 for x in context['extras'])


def test_route_utility_follows_connections_past_edge_seams(graph):
    ego = native_vehicle('ego', 'sb2_1', 100.)
    through = route_continuation(graph, ego, 'sb2_1')
    auxiliary = route_continuation(graph, ego, 'sb2_0')
    assert through['reachable']
    assert not auxiliary['reachable']
    assert auxiliary['distance_m'] == 700.
    assert not route_continuation(graph, ego, None)['known']


@pytest.mark.parametrize('visibility', [4.5, 75., 100., 200.])
def test_declared_visibility_is_actor_visible_not_hardcoded(graph, visibility):
    graph.__class__ = ContinuousMotionLaneGraph
    graph.yield_entries['mlk_on_0'] = dict(target='sb2_0', via=None, visibility_m=visibility, stop_offset_m=.1)
    ego = native_vehicle('ego', 'mlk_on_0', 100., route=RAMP)
    context = flow_context(graph, ego, {'ego': ego})
    assert context['entry']['entry_active'] and context['entry']['distance_to_start_m'] == 50.
    assert context['extras'][6] == visibility / 200.


def test_unknown_visibility_fails_closed(graph):
    graph.yield_entries['mlk_on_0'] = dict(target='sb2_0', via=None, visibility_m=float('nan'), stop_offset_m=.1)
    ego = native_vehicle('ego', 'mlk_on_0', 100., route=RAMP)
    with pytest.raises(ValueError, match='visibility'):
        merge_context(graph, ego, {'ego': ego})


def test_adjacent_current_lateral_measurements_are_not_hidden_intent(graph):
    graph.__class__ = ContinuousMotionLaneGraph
    ego = native_vehicle('ego', 'sb2_1', 100.)
    other = native_vehicle('other', 'sb2_2', 150.)
    other.update(lateral_position=-.37, lateral_speed=-.6)
    flow = flow_context(graph, ego, {'ego': ego, 'other': other})
    assert flow['extras'][13] == pytest.approx(-.1)
    assert flow['extras'][14] == pytest.approx(-.2)
    other.update(future_lane='sb2_1', future_speed=0., planned_event_time=2.)
    np.testing.assert_array_equal(flow['extras'], flow_context(graph, ego, {'ego': ego, 'other': other})['extras'])
