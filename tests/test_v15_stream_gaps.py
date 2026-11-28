"""The stream-gap reading (experiments/v15_stream_gaps.py): its pure pieces, on made-up numbers."""
import math

import pytest

from experiments import v15_stream_gaps as G
from traffic import v15_rule_expert as v3

PARAMS = (2.0, 4.5, 1.75, 2.5)          # v3's balanced background profile


def test_the_needed_speed_is_where_the_rear_cars_braking_reaches_the_limit():
    u = G.needed_speed(23., 70., PARAMS, 2.)
    assert 0. < u < 23.
    assert v3.background_response(23., u, 70., 23., PARAMS) >= -2.
    assert v3.background_response(23., u - .01, 70., 23., PARAMS) < -2.
    assert G.needed_speed(23., 70., PARAMS, 3.) < u                 # a looser limit needs less speed
    assert G.needed_speed(23., 0., PARAMS, 2.) is None               # no room behind us
    assert G.needed_speed(23., 400., PARAMS, 2.) == 0.               # far enough back for any speed
    assert G.needed_speed(23., 6., PARAMS, 2., top=20.) is None      # too close for any speed up to 20 m/s


def test_the_highest_line_speed_falls_with_a_later_crossing_and_ends_at_stop_and_go():
    v0, d, cap = 13.2, 50., 19.8
    go, top = v3.run_up(v0, d, cap)
    assert G.line_speed_max(v0, d, go - .1, cap) is None            # can't get there that soon
    assert G.line_speed_max(v0, d, go, cap) == pytest.approx(top, abs=1e-3)
    speeds = [G.line_speed_max(v0, d, T, cap) for T in (go, go + 1., go + 3., go + 6., 30., 60.)]
    assert all(a >= b - 1e-9 for a, b in zip(speeds, speeds[1:]))
    stop_and_go = math.sqrt(2. * G.RUN_UP * (d - v0 * v0 / (2. * G.BRAKE)))
    assert speeds[-1] == pytest.approx(stop_and_go, abs=1e-3) == pytest.approx(speeds[-2], abs=1e-3)


def test_each_gap_is_judged_at_its_earliest_crossing_and_the_last_is_open():
    stream = [(10., 23., 'a'), (12., 23., 'b'), (17., 23., 'c')]
    rows = G.gaps(stream, 5., 5., dict(fast=lambda T: 18., slow=lambda T: 3.), {})
    assert [(g['ahead'], g['rear']) for g in rows] == [(None, 'a'), ('a', 'b'), ('b', 'c'), ('c', None)]
    assert [g['T'] for g in rows] == [5., 11., 13., 18.]               # 1 s after the car ahead, not before 5 s
    assert rows[1]['behind_m'] == pytest.approx(23. * 1. - 5.)
    assert not rows[1]['ok']['entry_safe']['fast']                    # 18 m: too close for any speed
    assert rows[2]['ok']['entry_safe']['fast'] and not rows[2]['ok']['entry_safe']['slow']
    assert rows[-1]['ok'] == {k: {'fast': True, 'slow': True} for k in G.SAFE}
    assert G.first_open(rows, 'entry_safe', 'fast') == (5., 'a')       # ahead of the stream: 110 m back
    assert rows[0]['ok']['entry_safe']['fast'] and not rows[0]['ok']['entry_safe']['slow']
    assert G.first_open(rows, 'entry_safe', 'slow') == (18., None)


def test_the_forced_time_below_a_speed():
    assert G.forced_below(50., 18.7) == pytest.approx(8.7)
    assert G.forced_below(100., 10.) == 0.


def test_passing_is_the_first_instant_its_estimate_is_not_later():
    class Tracks:
        p = {'m': [(0., 6.), (5., 6.2)]}

        def state(self, vid, t):
            return (6. if t < 5. else 6.2), 22.
    assert G.passing(Tracks(), 'm') == (6.2, 22., 5.)


def test_sumo_warnings_are_read_with_their_times(tmp_path):
    (tmp_path / 'sumo-errors.log').write_text(
        "Warning: Vehicle 'main_8' performs emergency braking on lane ':n1_1_0' with decel=9.00, wished=4.50, "
        "severity=1.00, time=11.00.\n"
        "Warning: Vehicle 'main_8'; junction collision with vehicle 'agent_2', lane=':n1_1_0', gap=-1.00, latGap=0.00, "
        "time=11.10, stage=move.\n", encoding='utf-8')
    assert G.sumo_log(tmp_path) == [
        dict(kind='emergency braking', vehicle='main_8', lane=':n1_1_0', decel_m_s2=9., wished_m_s2=4.5, t=11.),
        dict(kind='junction collision', vehicle='main_8', other='agent_2', lane=':n1_1_0', gap_m=-1., t=11.1)]
    assert G.sumo_log(tmp_path / 'none') == []


def _record(time, speed, front=None, rear=None, stop=40., active=True, own=None):
    entry = dict(entry_active=active, source_lane='r_0', target_lane='m_0', distance_to_start_m=stop, front=front,
                 rear=rear)
    return dict(time=time, agents={'agent_0': dict(
        flow_context=dict(entry=entry, corridors=dict(own=dict(front=own))),
        v04_reward_audit=dict(measured_speed_m_s=speed))})


def test_seen_mainline_uses_the_instants_own_speed_and_skips_overlaps():
    rear = dict(id='main_1', bumper_gap_m=10., relative_speed_m_s=8.)
    records = [_record(.1, 12., rear=rear), _record(.2, 11.7, rear=rear),
               _record(.3, 11.4, rear=dict(rear, bumper_gap_m=0.)), _record(.4, 11.1, active=False)]
    seen, crossing = G.seen_mainline(records, ('r_0', 'm_0'), {'agent_0': 5.}, ours={'agent_0'}, ramps={'agent_0'})
    assert seen == {('main_1', .1): (55., 12. + 8.)}                  # 0.0 s: no speed yet; 0.2 s: overlapping
    assert crossing == {'agent_0': (.3, 11.4)}


def test_compare_counts_only_readings_before_either_crossed():
    a = {('m', .1): (50., 20.), ('m', .2): (48., 20.), ('m', 5.): (0., 20.)}
    b = {('m', .1): (50., 20.), ('m', .2): (48.005, 20.), ('m', 5.): (9., 20.)}
    same = G.compare(a, b, 1.)
    assert same['matched'] == 2 and same['same'] and same['last_t'] == .2
    moved = dict(b)
    moved[('m', .2)] = (47., 20.)
    assert not G.compare(a, moved, 1.)['same']
    assert G.compare(a, {}, 1.)['same'] is None


def test_the_ramp_leader_and_when_it_crossed():
    lead = dict(id='ramp_0', bumper_gap_m=30., relative_speed_m_s=0.)
    records = [_record(.1, 12., own=lead, stop=40.), _record(.2, 12., own=dict(lead, bumper_gap_m=34.), stop=38.)]
    assert G.ramp_leader(records, 'agent_0', ('r_0', 'm_0'), {'ramp_0': 5.}, {}, set()) == ('ramp_0', .1)
    mainline = [_record(.1, 12., own=dict(lead, id='main_3'))]
    assert G.ramp_leader(mainline, 'agent_0', ('r_0', 'm_0'), {}, {}, set()) == (None, None)
    ours = [_record(.1, 12., own=dict(lead, id='agent_6'))]
    assert G.ramp_leader(ours, 'agent_0', ('r_0', 'm_0'), {}, {'agent_6': (7.5, 14.)}, {'agent_6'}) == ('agent_6', 7.5)
