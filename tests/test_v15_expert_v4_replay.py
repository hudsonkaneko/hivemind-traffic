"""The offline replay of expert v4 (experiments/v15_expert_v4_replay.py): its reconstruction and projection helpers on
synthetic inputs. No SUMO run and no saved record is read."""
import math
import random

import pytest

from experiments import v15_expert_v4_replay as X
from traffic import v15_rule_expert as v3

PARAMS = (2.0, 4.5, 1.75, 2.5)


def test_the_rear_speed_solves_v3s_recorded_response():
    rng = random.Random(7)
    for _ in range(200):
        speed, entry, gap = rng.uniform(8., 30.), rng.uniform(0., 20.), rng.uniform(10., 120.)
        # v3 records the response with the free speed at the car's own speed, a free term of 0: -a (desired / gap)^2
        response = v3.background_response(speed, entry, gap, max(speed, 1.), PARAMS)
        desired = PARAMS[3] + max(0., speed * PARAMS[2] + speed * (speed - entry) / (2. * math.sqrt(PARAMS[0] * PARAMS[1])))
        assert response == pytest.approx(-PARAMS[0] * (desired / gap) ** 2)
        if desired > PARAMS[3]:
            assert X.solve_rear_speed(gap, response, entry, PARAMS) == pytest.approx(speed, rel=1e-6)


def test_no_rear_speed_without_a_braking_response():
    assert X.solve_rear_speed(None, -1., 5., PARAMS) is None
    assert X.solve_rear_speed(40., None, 5., PARAMS) is None
    assert X.solve_rear_speed(40., 0., 5., PARAMS) is None
    assert X.solve_rear_speed(-1., -1., 5., PARAMS) is None


def test_tracks_interpolate_between_observations_and_hold_outside():
    tracks = X.Tracks({'main_1': [(1., 10., 20.), (3., 12., None), (3., 14., 24.), (5., None, 22.)]})
    assert tracks.cars() == ['main_1']
    assert tracks.state('main_1', 0.) == (10., 20.)                   # held before the first observation
    assert tracks.state('main_1', 2.) == pytest.approx((11.5, 22.))   # the mean of the two at t=3 is 13
    assert tracks.state('main_1', 9.) == (13., 22.)                   # held after the last
    assert tracks.state('main_2', 1.) is None


def test_a_path_point_walks_back_from_the_end_and_extends_past_the_start():
    line = [(0., 0.), (10., 0.), (10., 10.)]
    assert X.Paths.point(line, 0.) == (10., 10.)
    assert X.Paths.point(line, 5.) == (10., 5.)
    assert X.Paths.point(line, 15.) == (5., 0.)
    assert X.Paths.point(line, 25.) == (-5., 0.)


def test_crossing_safety_names_the_next_car_behind_and_the_last_ahead():
    tracks = X.Tracks({'main_1': [(0., 9., 22.)], 'main_2': [(0., 13., 22.)], 'main_3': [(0., 16., 25.)]})
    out = X.crossing_safety(tracks, 10., 15., 4.8, {'main_2': PARAMS})
    assert out['ahead'] == 'main_1' and out['ahead_headway_s'] == 1.
    assert out['rear'] == 'main_2' and out['rear_gap_m'] == pytest.approx(3 * 22. - 4.8)
    assert out['rear_response_m_s2'] == pytest.approx(v3.background_response(22., 15., 3 * 22. - 4.8, 22., PARAMS), abs=1e-3)
    empty = X.crossing_safety(X.Tracks({}), 10., 15., 4.8, {})
    assert empty['rear'] is None and empty['ahead'] is None


def test_the_ceiling_is_the_environments_bounds():
    me = dict(v=10., a=0., x=50.)
    assert X._ceiling(me, None, 20., None) == 2.                                      # speed bound well above
    assert X._ceiling(me, None, 20., 1.3) == 1.3                                      # the drive bound
    assert X._ceiling(dict(me, v=19.95), None, 20., None) == pytest.approx(.5)        # near the speed limit
    front = dict(v=0., gap=10.)
    safe = max(0., 0. + .5 * (10. - 2.5 - 15.))
    assert X._ceiling(me, front, 20., None) == max(-4.5, (safe - 10.) / X.DT)           # a stopped leader: -4.5


def test_the_world_splits_cars_into_the_stream_and_the_leaders():
    tracks = X.Tracks({'main_1': [(0., 2., 20.)], 'main_2': [(0., 8., 20.)], 'main_3': [(0., 5., 1.)]})
    stream, leaders = X.world(tracks, 0., 60., 20., {}, None, None)
    assert [vid for _, _, vid in stream] == ['main_2'] and stream[0][0] == pytest.approx(160.)
    assert [x[3] for x in leaders] == ['main_1'] and leaders[0][0] == pytest.approx(60. - 40. - 4.8)
    # not yet seen by this car's v3, and outside the 250 m radius
    assert X.world(tracks, 0., 60., 20., {}, None, {'main_2': 1.})[0] == []
    far = X.Tracks({'main_4': [(0., 20., 20.)]})
    assert X.world(far, 0., 60., 20., {}, None, None)[0] == []


def test_the_replay_drives_v3_v4_and_v4_as_written_and_only_v4_has_its_limits():
    assert set(X.DRIVERS) == {'v3', 'v4', 'v4_written'} and X.WITHOUT_LIMITS == ('v4_written',)
    params = {}
    for name in X.DRIVERS:
        junction = X.Junction(name, 20.86, params)
        decision = junction('agent_0', 12., 100., [], 20., 4.8, ceiling=1.3, curve=17.)
        assert decision['committed'] and decision['rear'] is None                     # a clear ramp: go now
        if name == 'v3':
            assert not hasattr(junction.driver, '_limits')
        else:
            assert junction.driver._limits['agent_0'] == ((1.3, 17.) if name == 'v4' else (None, None))
    assert X.Junction('v4', 20.86, params)('agent_0', 12., 100., [], 20., ceiling=1.3)['run_up_m_s2'] == \
        pytest.approx(1.3 - v3.SETTINGS['ceiling_margin_m_s2'])


def test_commits_count_starts_undone_and_dilemma_decisions():
    decisions = [dict(committed=c, dilemma=d) for c, d in
                 [(False, False), (True, False), (True, False), (False, False), (True, True), (True, True)]]
    assert X._commits(decisions) == dict(starts=2, undone=1, dilemma_decisions=2)
