import copy
import math
import xml.etree.ElementTree as ET

import pytest

from traffic.i110_convergence import ConvergenceGraph, stopping_profile, response_envelope, screen


@pytest.fixture
def graph(tmp_path):
    root = ET.Element('net')
    for name, n, length in [('up', 2, 100), ('ramp', 1, 60), ('down', 3, 100),
                            ('disconnected', 1, 100), (':join', 2, 2)]:
        edge = ET.SubElement(root, 'edge', id=name, **({'function': 'internal'} if name[0] == ':' else {}))
        for i in range(n):
            ET.SubElement(edge, 'lane', id=f'{name}_{i}', index=str(i), length=str(length))
    for i in range(2):
        ET.SubElement(root, 'connection', {'from': 'up', 'to': 'down', 'fromLane': str(i),
                      'toLane': str(i+1), 'via': f':join_{i}'})
        ET.SubElement(root, 'connection', {'from': ':join', 'to': 'down', 'fromLane': str(i),
                      'toLane': str(i+1)})
    ET.SubElement(root, 'connection', {'from': 'ramp', 'to': 'down', 'fromLane': '0', 'toLane': '0'})
    path = tmp_path/'net.xml'; ET.ElementTree(root).write(path)
    return ConvergenceGraph(path)


def state(lane, position, speed=20., acceleration=0., x=0.):
    return dict(lane_id=lane, lane_position=position, length=4.8, speed=speed,
                acceleration=acceleration, x=x, y=0.)


def test_incoming_adjacent_path_is_conditional_not_leader(graph):
    p = graph.pair(state('up_0', 70), state('ramp_0', 58))
    assert p['relation'] == 'adjacent_lane_path'
    assert p['projected_gap_m'] == pytest.approx(32-2-4.8)
    assert p['anchor_edge'] == 'down'
    assert not p['intent_known'] and not p['actual_gap_known']


def test_internal_and_current_lane_distances(graph):
    p = graph.pair(state(':join_0', 1), state('down_1', 10))
    assert p['projected_gap_m'] == pytest.approx(1+10-4.8)
    p = graph.pair(state('down_1', 10), state('down_1', 30))
    assert p['projected_gap_m'] == pytest.approx(20-4.8) and p['actual_gap_known']


def test_unknown_disconnected_and_nonadjacent_are_not_safe_labels(graph):
    assert graph.pair(state('bad', 0), state('down_1', 10))['status'] == 'unknown_lane_context'
    assert graph.pair(state('down_1', 101), state('down_1', 10))['status'] == 'unknown_lane_context'
    for a, b in [('disconnected_0', 'down_0'), ('up_1', 'ramp_0')]:
        assert graph.pair(state(a, 10), state(b, 10))['status'] == 'no_same_or_adjacent_path_within_limit'


@pytest.mark.parametrize('speed,acceleration', [(25., 1.5), (25., 0.), (15., -2.), (1., 0.), (0., 0.)])
def test_profile_continuity_bounds_and_exact_terminal_state(speed, acceleration):
    segments = stopping_profile(speed, acceleration)
    assert segments is not None
    for i, s in enumerate(segments):
        assert abs(s.j) <= 2
        for t in (s.start, s.start+s.duration/2, s.start+s.duration):
            _, v, a = s.at(t)
            assert v >= -1e-8 and a >= -2.5-1e-8
        if i:
            prev = segments[i-1]
            assert prev.at(prev.start+prev.duration) == pytest.approx((s.x, s.v, s.a))
    last = segments[-1]
    _, v, a = last.at(last.start+last.duration)
    assert (v, a) == pytest.approx((0., 0.), abs=1e-8)


def test_stopping_distance_closed_form():
    # Symmetric trapezoid from zero acceleration: total time v/b+b/j;
    # symmetry means mean speed v/2.
    profile = stopping_profile(20., 0., reaction=0.)
    envelope = response_envelope(profile, 0., model='constant_speed', headway=0.)
    assert envelope['stopping_time_s'] == pytest.approx(20/2.5+2.5/2)
    assert envelope['required_projected_gap_m'] == pytest.approx(2.5+10*(20/2.5+2.5/2))


@pytest.mark.parametrize('model', ['constant_speed', 'brake_9'])
@pytest.mark.parametrize('headway', [0., 2.])
def test_analytic_envelope_against_independent_time_integration(model, headway):
    profile = stopping_profile(23., .7)
    exact = response_envelope(profile, 7., model=model, headway=headway)
    # Independent small-step midpoint integration of the segment jerk inputs.
    xe = xo = t = 0.; ve, ae, vo = 23., .7, 7.
    maximum = headway*ve
    for segment in profile:
        n = max(1, math.ceil(segment.duration/.0005)); dt = segment.duration/n
        for _ in range(n):
            amid = ae+segment.j*dt/2
            xe += (ve+amid*dt/2)*dt
            ve += amid*dt; ae += segment.j*dt
            next_vo = vo if model == 'constant_speed' else max(0., vo-9*dt)
            xo += (vo+next_vo)*dt/2; vo = next_vo; t += dt
            maximum = max(maximum, xe-xo+headway*ve)
    assert exact['required_projected_gap_m'] == pytest.approx(2.5+maximum, abs=2e-4)


@pytest.mark.parametrize('speed,acceleration', [(10., -4.5), (.01, -2.), (-1., 0.), (math.nan, 0.)])
def test_infeasible_initial_state_stays_unknown(speed, acceleration):
    p = stopping_profile(speed, acceleration)
    assert p is None
    assert not response_envelope(p, 1., model='constant_speed', headway=0.)['known']


def test_signal_ignores_route_labels_event_schedule_and_future_context(graph):
    plan = dict(jerk_m_s3=2., comfortable_deceleration_m_s2=2.5, reaction_s=.1,
                ranges_m=[120., 240.], other_models=['constant_speed','brake_9'],
                headways_s=[0.,2.], standstill_gap_m=2.5)
    states = {'a': state('up_0', 70), 'b': state('ramp_0', 58, speed=0., x=30.)}
    original = screen('a', states, graph, plan)
    altered = copy.deepcopy(states)
    for s in altered.values():
        s.update(route=['fake', 'future'], leader=['future_leader', 1], event=True, scheduled_merge=3.)
    assert screen('a', altered, graph, plan) == original
    assert original['candidates'][0]['envelopes']['constant_speed-h0']['known']


def test_rear_and_out_of_range_are_not_front_envelopes(graph):
    plan = dict(jerk_m_s3=2., comfortable_deceleration_m_s2=2.5, reaction_s=.1,
                ranges_m=[120.,240.], other_models=['constant_speed'], headways_s=[0.], standstill_gap_m=2.5)
    states = {'a': state('down_1', 30), 'b': state('down_1', 10), 'c': state('down_1', 50, x=241.)}
    s = screen('a', states, graph, plan)
    assert len(s['candidates']) == 1
    assert not s['candidates'][0]['projection']['forward_candidate']
    assert not s['candidates'][0]['envelopes']
