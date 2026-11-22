"""V14 stop fix: merge_braking, target_band_tail and the stop measurements (no SUMO)."""
import pytest

from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v12_rewards as v12
from traffic import highway_flow_rl_v14_stop_fix as fix

MERGE = dict(eligible=True, target_m_s=16.5, obligations=['merge_or_yield'], reasons=[])
PLAN = dict(crossing_deficit=0.)


def charge(requested=-1., speed=12., target=MERGE, ceiling=1.3, front=None, plan=PLAN, merge_active=True):
    return fix.merge_braking(target, speed, requested, ceiling, front, plan, merge_active)


def test_settings_are_v06_avoidable_braking_values():
    assert fix.SETTINGS['weight'] == v06.WEIGHTS['avoidable_braking'] == .25
    assert fix.SETTINGS['braking_request_m_s2'] == v06.SETTINGS['braking_request_m_s2'] == -.2
    assert fix.SETTINGS['braking_scale_m_s2'] == v06.SETTINGS['braking_scale_m_s2'] == 2.
    assert fix.SETTINGS['free_headway_s'] == v06.SETTINGS['free_headway_s'] == 1.5
    assert fix.validate_settings(dict(fix.SETTINGS, obligations=list(fix.SETTINGS['obligations']))) == fix.SETTINGS
    with pytest.raises(ValueError):
        fix.validate_settings(dict(fix.SETTINGS, weight=.5))


def test_charge_matches_v06_formula_for_merge_decisions():
    assert charge(-.19) == 0.
    assert charge(-1.2) == pytest.approx(-.25 * .5)
    assert charge(-4.2) == pytest.approx(-.25)
    assert charge(-1., target=dict(MERGE, obligations=['ending_lane', 'merge_or_yield'])) == pytest.approx(-.1)


def test_no_charge_when_the_target_or_road_calls_for_braking():
    assert charge(target=dict(MERGE, target_m_s=12.4)) == 0.                     # target below speed + 0.5
    assert charge(ceiling=.19) == 0.                                             # safety ceiling binding
    assert charge(plan=dict(crossing_deficit=.3)) == 0.                          # yield-line conflict in the plan
    assert charge(front=dict(bumper_gap_m=17.9)) == 0.                           # leader inside 1.5 s at 12 m/s
    assert charge(front=dict(bumper_gap_m=18.1)) == pytest.approx(-.1)
    assert charge(merge_active=False) == 0.
    assert charge(target=dict(MERGE, eligible=False)) == 0.


def test_decisions_without_obligations_stay_v06s():
    assert charge(target=dict(MERGE, obligations=[])) == 0.
    assert charge(target=dict(MERGE, obligations=['merge_or_yield', 'other'])) == 0.


def test_band_tail_only_past_the_cap():
    target = dict(eligible=True, target_m_s=20.1)
    assert fix.target_band_tail(target, 11.1, True, v12.SETTINGS) == pytest.approx(0., abs=1e-12)  # 9 m/s off: the cap
    assert fix.target_band_tail(target, 12., True, v12.SETTINGS) == 0.
    stopped = fix.target_band_tail(target, 0., True, v12.SETTINGS)
    assert stopped == pytest.approx(-.25 * ((20.1 - 1.8) / 1.8 - 4.))
    assert v12.target_band(target, 0., True) + stopped == pytest.approx(-5.5417, abs=1e-3)
    assert fix.target_band_tail(target, 0., False, v12.SETTINGS) == 0.
    assert fix.target_band_tail(dict(target, eligible=False), 0., True, v12.SETTINGS) == 0.


def test_stop_sample():
    target = dict(eligible=True, target_m_s=14.6)
    assert fix.stop_sample(.2, -.3, None, target, True) == dict(stopped=0, asking_to_brake=0,
                                                                asking_to_brake_overrides=0, free_road=0)
    assert fix.stop_sample(0., -.3, None, target, True) == dict(stopped=1, asking_to_brake=1,
                                                                asking_to_brake_overrides=1, free_road=1)
    queued = fix.stop_sample(0., .1, dict(bumper_gap_m=6.), target, False)
    assert queued == dict(stopped=1, asking_to_brake=0, asking_to_brake_overrides=0, free_road=0)
    assert fix.stop_sample(0., -.3, None, dict(target, target_m_s=3.), False)['free_road'] == 0
