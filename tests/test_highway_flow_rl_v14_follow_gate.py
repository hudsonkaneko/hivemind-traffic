"""V14 follow gate: the followable reference, its restart, the gate's band and the reward cap (no SUMO)."""
import pytest

from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v14_follow_gate as follow
from traffic import highway_flow_rl_v14_rewards as v14

S = follow.SETTINGS
DT = .1


def test_settings_keep_v14_rates_and_the_gate_tolerance():
    assert S['rise_m_s2'] == v14.SETTINGS['target_acceleration_m_s2'] == 1.5
    assert S['fall_m_s2'] == v14.SETTINGS['target_deceleration_m_s2'] == 2.
    assert S['tolerance_m_s'] == v06.SETTINGS['tracking_tolerance_m_s'] == 2.
    assert S['jerk_m_s3'] < 4.          # the car's comfort rate limit: 0.4 m/s^2 per 0.1 s decision
    assert follow.validate_settings(dict(S, reward_caps=list(S['reward_caps']))) == S
    for bad in (dict(S, jerk_m_s3=4.), dict(S, tolerance_m_s=3.), dict(S, reward_caps=['occupied_leader']), {}):
        with pytest.raises(ValueError):
            follow.validate_settings(bad)


def test_rise_rate_is_capped_by_the_drive_ceiling():
    assert follow.rise_rate(None) == 1.5
    assert follow.rise_rate(2.) == 1.5
    assert follow.rise_rate(1.25) == pytest.approx(1.25)
    assert follow.rise_rate(-.2) == 0.
    with pytest.raises(ValueError):
        follow.rise_rate(float('inf'))


def test_start_is_the_car_speed_not_the_plan():
    # Logged case 11 agent_0 at 0.2 s: speed 11.05, plan 5.025; v14's gate target starts at the plan.
    assert v14.reachable(5.025, 20.1, 11.05, None, .2) == pytest.approx(5.025)
    ref = follow.start(11.05, 0., 20.1, 1.29, .2)
    assert ref == dict(time=.2, value=11.05, slope=0., below=False)
    assert follow.start(11.05, -3., None, 1.29, .2)['slope'] == -2.          # acceleration within [-fall, rise]
    assert follow.start(11.05, 1.8, None, 1.29, .2)['slope'] == pytest.approx(1.29)
    assert follow.start(11.05, 0., 9., 1.29, .2)['value'] == 9.              # the lane/map ceiling at once
    with pytest.raises(ValueError):
        follow.start(float('nan'), 0., None, 1.5, 0.)


def _run(plan, car, rise=1.29, steps=120, ceiling=None):
    """The reference over a plan profile (time -> road target) with a car model (state -> next state)."""
    speed, accel = car['speed'], 0.
    ref = follow.start(speed, accel, ceiling, rise, 0.)
    rows = []
    for k in range(1, steps + 1):
        now = k * DT
        speed, accel = car['move'](speed, accel, ref['value'])
        ref, restarted = follow.advance(ref, plan(now), ceiling, rise, speed, accel, now)
        rows.append(dict(time=now, ref=ref['value'], slope=ref['slope'], speed=speed, plan=plan(now),
                         restarted=restarted))
    return rows


def tracking_car(speed, gain=3., limit=(-4., 2.), rate=.4, drive=1.29):
    def move(v, a, target):
        want = max(limit[0], min(min(limit[1], drive), gain * (target - v)))
        a = a + max(-rate, min(rate, want - a))
        return max(0., v + a * DT), a
    return dict(speed=speed, move=move)


def case11_plan(now):
    """Case 11's logged plan: yield to 5.0 m/s for 1.2 s, then the ramp's 20.1 m/s."""
    return 5.025 if now < 1.2 else 20.1


def test_the_reference_is_jerk_and_rate_limited_and_does_not_overshoot():
    rows = _run(case11_plan, tracking_car(11.05))
    slopes = [0.] + [r['slope'] for r in rows]
    for before, after in zip(slopes, slopes[1:]):
        assert abs(after - before) <= S['jerk_m_s3'] * DT + 1e-9
    assert min(slopes) >= -2. - 1e-9 and max(slopes) <= 1.29 + 1e-9
    assert max(r['ref'] for r in rows) <= 20.1 + 1e-9


def test_a_car_that_follows_the_reference_matches_through_a_yield_but_not_v14s_target():
    rows = _run(case11_plan, tracking_car(11.05))
    assert all(follow.matches(r['speed'], *follow.band(r['ref'], r['plan'])) for r in rows)
    # The same car against v14's target, which starts at the plan and rises at 1.5 m/s^2:
    previous, misses = None, 0
    for r in rows:
        value = v14.reachable(r['plan'], 20.1, r['speed'], previous, r['time'])
        previous = dict(time=r['time'], value=value)
        misses += abs(r['speed'] - value) > 2.
    assert misses > .3 * len(rows)


def test_a_slow_or_stopped_car_leaves_the_band():
    lazy = _run(lambda now: 20.1, tracking_car(8., drive=.6))       # at most 0.6 m/s^2 against a 1.29 rise
    assert not follow.matches(lazy[-1]['speed'], *follow.band(lazy[-1]['ref'], 20.1))
    assert sum(follow.matches(r['speed'], *follow.band(r['ref'], 20.1)) for r in lazy) < .9 * len(lazy)
    stopped = dict(speed=8., move=lambda v, a, target: (max(0., v - .4), -4.))
    rows = _run(lambda now: 20.1, stopped)
    assert not follow.matches(rows[-1]['speed'], *follow.band(rows[-1]['ref'], 20.1))


def test_restart_after_a_yield_only_for_a_slower_car():
    ref = dict(time=1., value=9., slope=-2., below=True)                 # yielding: the plan was below
    moved, restarted = follow.advance(ref, 20.1, None, 1.29, 6., -3.5, 1.1)
    assert restarted and moved['value'] == 6. and moved['slope'] == -2. and not moved['below']
    moved, restarted = follow.advance(ref, 20.1, None, 1.29, 9.5, -1., 1.1)    # the car is not slower
    assert not restarted and moved['value'] == pytest.approx(9. - 1.75 * DT)
    moved, restarted = follow.advance(dict(ref, below=False), 20.1, None, 1.29, 6., -3.5, 1.1)
    assert not restarted                                                  # no yield before
    moved, restarted = follow.advance(ref, 5., None, 1.29, 6., -3.5, 1.1)   # still yielding
    assert not restarted and moved['below']
    with pytest.raises(ValueError):
        follow.advance(dict(ref, time=1.2), 20.1, None, 1.29, 6., 0., 1.1)


def test_a_deep_yield_then_restart_keeps_a_plan_following_car_in_the_band():
    # A car that brakes to the plan's 5.0 m/s (harder than the reference) and then accelerates.
    def plan_car():
        def move(v, a, target):
            goal = case11_plan(move.now) if move.now < 1.2 else 20.1
            move.now += DT
            want = max(-4., min(1.29, 3. * (goal - v)))
            a = a + max(-.4, min(.4, want - a))
            return max(0., v + a * DT), a
        move.now = DT
        return dict(speed=11.05, move=move)
    rows = _run(case11_plan, plan_car())
    assert any(r['restarted'] for r in rows)
    assert sum(follow.matches(r['speed'], *follow.band(r['ref'], r['plan'])) for r in rows) >= .9 * len(rows)


def test_ceiling_applies_at_once():
    ref = dict(time=0., value=15., slope=1., below=False)
    moved, _ = follow.advance(ref, 20., 12., 1.5, 15., 1., .1)
    assert moved['value'] == 12. and moved['slope'] == 0.


def test_band_and_acceptance_are_v04s_check_on_the_span():
    lower, upper = follow.band(10., 20.)
    assert (lower, upper) == follow.band(20., 10.) == (10., 20.)
    for speed, expected in ((7.9, False), (8., True), (15., True), (22., True), (22.1, False)):
        assert follow.matches(speed, lower, upper) is expected
        assert (abs(speed - follow.acceptance(speed, lower, upper)) <= 2.) is expected


def test_reward_cap_uses_only_stopping_bounds():
    target = dict(bounds=[dict(reason='occupied_lane_limit', ceiling_m_s=11.), dict(reason='occupied_leader', ceiling_m_s=14.),
                          dict(reason='ending_lane_runway', ceiling_m_s=12.5), dict(reason='merge_target_limit', ceiling_m_s=9.)])
    assert follow.reward_cap(target) == 12.5
    assert follow.reward_cap(dict(bounds=[dict(reason='occupied_lane_limit', ceiling_m_s=11.)])) is None
    assert follow.reward_cap(None) is None
