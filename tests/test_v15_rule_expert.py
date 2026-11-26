"""The v15 rule expert (traffic/v15_rule_expert.py): its pure rules, and (with libsumo and the
networks) a 2-row pool check of the code on two of the cases' own v14 seeds, which are not
benchmark rows: an uncharged engineering check, as Phase 2's approval allows."""
import importlib.util
import json
import math
import shutil
import uuid
from pathlib import Path

import pytest

from experiments import v15_pool as pool
from traffic import v15_rule_expert as expert

ROOT = Path(__file__).resolve().parents[1]
HAS_LIBSUMO = importlib.util.find_spec('libsumo') is not None


def test_the_driver_is_registered_and_validates_only():
    assert expert.DRIVER == 'rule:expert-v3' and expert.DRIVER in pool.RULE_DRIVERS
    from traffic import v15_rule_expert_v1, v15_rule_expert_v2
    assert v15_rule_expert_v1.DRIVER == 'rule:expert-v1' and v15_rule_expert_v1.DRIVER in pool.RULE_DRIVERS
    assert v15_rule_expert_v2.DRIVER == 'rule:expert-v2' and v15_rule_expert_v2.DRIVER in pool.RULE_DRIVERS
    pool.Task('v15-development', 1, expert.DRIVER)
    with pytest.raises(ValueError):
        pool.Task('v15-development', 1, expert.DRIVER, mode='collect')


def test_idm_interaction_is_zero_at_the_desired_gap_and_near_full_on_a_free_road():
    s = expert.SETTINGS
    desired = s['idm_min_gap_m'] + 20. * s['idm_time_gap_s']
    assert expert.idm_interaction(20., 20., desired) == pytest.approx(0., abs=1e-9)
    assert expert.idm_interaction(20., 20., 1000.) == pytest.approx(s['idm_max_m_s2'], rel=.01)
    assert expert.idm_interaction(20., 10., desired) < -1.               # closing on a slower leader
    # larger than the environment's following bound (2.5 m + 1.5 s) at any speed
    assert all(s['idm_min_gap_m'] + v * s['idm_time_gap_s'] > 2.5 + 1.5 * v for v in range(0, 35))


def test_free_term_and_lane_end_dilemma_rule():
    assert expert.free_term(10., 10.5) == pytest.approx(1.)
    assert expert.free_term(10., 30.) == expert.SETTINGS['free_max_m_s2']
    assert expert.free_term(30., 0.) == expert.SETTINGS['free_min_m_s2']
    assert expert.lane_end_term(10., 200.) == (None, pytest.approx(100. / (2 * 194.)))
    acceleration, needed = expert.lane_end_term(15., 50.)                # 225 / 88 = 2.56 m/s^2
    assert acceleration == pytest.approx(-needed) and needed == pytest.approx(225. / 88.)
    assert expert.lane_end_term(15., 5.) == (-4.5, math.inf)


def test_mobil_safety_uses_the_background_cars_idm():
    assert expert.mobil_safe(20., None) == (True, None)
    far = dict(bumper_gap_m=80., relative_speed_m_s=0.)
    close = dict(bumper_gap_m=5., relative_speed_m_s=5.)                   # follower 25 m/s, 5 m behind
    assert expert.mobil_safe(20., far)[0] and not expert.mobil_safe(20., close)[0]
    slow_front = dict(bumper_gap_m=30., relative_speed_m_s=-10.)
    assert expert.mobil_incentive(20., 25., slow_front, None, None) > 1.    # an empty lane beats a slow leader
    assert expert.mobil_incentive(20., 25., None, slow_front, None) < 0.


def test_curvature_from_lane_shapes_and_the_cap():
    straight = [(0., 0.), (50., 0.), (100., 0.)]
    assert max(k for _, k in expert.lane_curvatures(straight)) == pytest.approx(0., abs=1e-9)
    radius = 50.
    arc = [(radius * math.cos(a / 100.), radius * math.sin(a / 100.)) for a in range(0, 157)]   # a quarter circle
    curvatures = [k for _, k in expert.lane_curvatures(arc)]
    assert curvatures and all(k == pytest.approx(1. / radius, rel=.05) for k in curvatures)
    assert expert.curve_cap([]) == math.inf and expert.curve_cap([(10., 0.)]) == math.inf
    at_curve = math.sqrt(2.5 * radius)
    assert expert.curve_cap([(0., 1. / radius)]) == pytest.approx(at_curve)
    assert expert.curve_cap([(40., 1. / radius)]) == pytest.approx(math.sqrt(at_curve ** 2 + 2. * 40.))


def test_compose_rate_band_ceiling_and_pull_away():
    request, terms = expert.compose(20., 0., 30., 2.)
    assert request == pytest.approx(.3) and set(terms) == {'free'}           # rate band
    request, _ = expert.compose(20., 1.9, 30., 1.)
    assert request == pytest.approx(.95)                                    # ceiling below the band wins
    request, terms = expert.compose(20., 0., 30., 2., leaders=[(10., 30., 0.)])
    assert terms['idm'] < 0. and terms['bound'] == expert.SETTINGS['bound_floor_m_s2'] and request == pytest.approx(-.3)
    request, terms = expert.compose(20., 0., 30., 2., extra=dict(yield_=-1., none=None))
    assert terms['yield_'] == -1. and 'none' not in terms and request == pytest.approx(-.3)
    request, _ = expert.compose(0., 0., 10., 2., leaders=[(0., 1., 0.)], pull_away=True)
    assert request == pytest.approx(.3)                                     # floor, still rate-banded
    assert expert.normalized(-4.5) == -1. and expert.normalized(2.) == 1.


def test_small_gap_corrections_coast_and_larger_ones_brake():
    s = expert.SETTINGS
    desired = s['idm_min_gap_m'] + s['idm_time_gap_s'] * 10.
    gentle = desired / math.sqrt(1. + .3 / s['idm_max_m_s2'])          # IDM asks -0.3 m/s^2 at equal speeds
    request, terms = expert.compose(10., -.2, 10., 2., leaders=[(10., gentle, 0.)])
    assert terms['idm'] == pytest.approx(-.3) and terms['bound'] > 0.
    assert request == pytest.approx(-s['coast_m_s2'])
    firm = desired / math.sqrt(1. + 1. / s['idm_max_m_s2'])            # IDM asks -1.0 m/s^2
    request, terms = expert.compose(10., -1., 10., 2., leaders=[(10., firm, 0.)])
    assert terms['idm'] == pytest.approx(-1.) and request < -.5


def test_following_bound_anticipation_holds_the_margin_under_the_environment_bound():
    s = expert.SETTINGS
    # the environment's bound: vL + 0.5 (gap - 2.5 - 1.5 v); at 34.5 m behind an equal-speed leader it is 1 m/s above
    assert expert.bound_speed(20., 20., 34.5) == pytest.approx(21.)
    assert expert.bound_term(20., 20., 0., 34.5) == (pytest.approx(0., abs=1e-9), pytest.approx(s['bound_margin_m_s']))
    assert expert.bound_term(20., 20., 1., 34.5)[0] == pytest.approx(1. / 1.75)      # follows the leader's acceleration
    wanted, headroom = expert.bound_term(20., 14., 0., 40.)                           # closing at 6 m/s, 2.25 over
    assert headroom == pytest.approx(-2.25) and wanted == pytest.approx((-3. - 3.25) / 1.75)
    assert expert.bound_term(20., 10., 0., 30.)[0] == s['bound_floor_m_s2']          # far over: the floor
    assert expert.bound_term(20., 20., 0., 100.)[0] > 0.                              # far behind: no constraint
    assert expert.bound_speed(20., 0., 0.) == 0.


def test_run_up_reaches_the_cap_or_the_line():
    a = expert.SETTINGS['run_up_m_s2']
    time, speed = expert.run_up(0., 12., 20.)
    assert speed == pytest.approx(math.sqrt(2. * a * 12.)) and time == pytest.approx(speed / a)
    time, speed = expert.run_up(20., 100., 20.)
    assert speed == 20. and time == pytest.approx(5.)
    # crawling for the wait leaves less room for the run-up, so a later arrival enters slower
    assert expert.entry_options(12., .5, 2., 20.)[-1][0] > expert.entry_options(12., .5, 10., 20.)[-1][0] >= .5
    # the gentlest way first: hold the steady speed that arrives on time, then the crawl and run-up
    (entry, hold), (run, crawl) = expert.entry_options(128., 11.4, 12., 20.)
    assert hold == pytest.approx(128. / 12.) == entry and crawl == expert.SETTINGS['crawl_m_s'] and run > entry
    assert expert.entry_options(10., 1., 20., 20.) == [(.5, .5)]


def test_gap_selection_goes_waits_and_runs_up():
    s = expert.SETTINGS
    assert expert.select_gap(27., 12., [])['waited_for'] == []
    far = expert.select_gap(27., 12., [(300., 22., 'far', None)])
    assert far['waited_for'] == [] and far['rear'] == 'far' and far['response'] >= -s['entry_safe_m_s2']
    # a car 60 m from our point at 22 m/s would brake hard behind us: wait for it to pass, then the next gap
    plan = expert.select_gap(12., 2., [(60., 22., 'close', None), (400., 22., 'next', None)])
    assert plan['waited_for'] == ['close'] and plan['rear'] == 'next'
    assert plan['T'] >= 60. / 22. + s['yield_headway_s'] - 1e-9 and plan['u'] > s['crawl_m_s']
    # far from the line a steady hold is enough: no braking to the crawl
    far_plan = expert.select_gap(128., 11.4, [(130., 22., 'a', None), (220., 22., 'b', None), (700., 22., 'c', None)])
    assert far_plan['waited_for'] and far_plan['hold'] > 5.
    # a cautious car (tau 2.0 s) needs a longer gap than the balanced profile at the same distance
    balanced = expert.background_response(22., 5., 90., 22.)
    cautious = expert.background_response(22., 5., 90., 22., (1.6, 4.5, 2.0, 2.5))
    assert cautious < balanced


def test_lane_change_needs_headroom_to_the_new_leader():
    assert expert.lane_change_ok(20., None, None)[0]
    slow_close = dict(bumper_gap_m=15., relative_speed_m_s=-5.)
    ok, record = expert.lane_change_ok(20., slow_close, None)
    assert not ok and record['front_headroom'] < expert.SETTINGS['new_front_margin_m_s']
    assert expert.lane_change_ok(20., dict(bumper_gap_m=80., relative_speed_m_s=0.), None)[0]


def _networks_present(rows):
    from traffic import highway_flow_rl_v14_contract as contract
    validation = contract.build_config(24)['validation_rows']
    return all((ROOT / validation[i]['case']['network']).is_file() for i in rows)


@pytest.mark.skipif(not HAS_LIBSUMO, reason='libsumo is not installed in this environment')
@pytest.mark.timeout(900)
def test_the_expert_through_the_pool_changes_lanes_inside_the_mask_and_matches_its_canary():
    rows = (6, 2)        # case 6 (ramp-started lane-change merge) and case 2 (passing), at their own v14 seeds
    if not _networks_present(rows):
        pytest.skip('the frozen networks are not in this checkout (outputs/ is not in Git)')
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-expert-{uuid.uuid4().hex[:8]}'
    try:
        with pool.WorkerPool(2, out, priority='below_normal') as workers:
            batch = workers.run([pool.Task('v14-validation', r, expert.DRIVER) for r in rows])
        assert batch.canary['matched'] and batch.canary['different_worker']
        import gzip
        for record in batch.records:
            episode = next(Path(record['directory']).glob('episode-*'))
            decisions = [json.loads(line) for line in (episode / 'expert.jsonl').read_text().splitlines()]
            assert len(decisions) == record['decisions'] > 0
            changes = 0
            with gzip.open(episode / 'actions.jsonl.gz', 'rt') as handle:
                for line in handle:
                    for decision in json.loads(line)['agents'].values():
                        if decision['lane_request']:
                            changes += 1
                            assert not decision.get('lane_rejected')      # requested only inside the mask
            assert changes >= 1                                        # both rows need a lane change
            for d in decisions:                                        # rate band, or the ceiling below it
                assert d['request'] <= d['ceiling'] - .05 + 1e-6 or d['request'] <= -4.5 + 1e-6
                assert abs(d['request'] - d['a']) <= expert.SETTINGS['rate_m_s2'] + 1e-6 \
                    or d['request'] < d['a'] - expert.SETTINGS['rate_m_s2']
    finally:
        shutil.rmtree(out, ignore_errors=True)
