"""The v15 gate re-baseline (experiments/v15_regrade.py) on synthetic traces: the window and speed match as the
environment grades them, and each rule's effect, including cars it must still fail."""
import pytest

from experiments import v15_regrade as R
from traffic import highway_flow_rl_v14_follow_gate as F

DT = .1


def _sample(t, speed, match=False, jerk=0., stopped=None):
    return dict(time_s=t, stopped=speed < .1 if stopped is None else stopped, jerk=jerk, override=False,
                neighbor_acceleration=None, speed_match=match)


def _steps(speeds, targets, completed_at, stable_at, eligible=None, jerks=None):
    """A non-yield car: speeds after each decision, the recorded gate target, merge completion and stable times."""
    steps, previous = [], (speeds[0], 0.)
    for i, (speed, target) in enumerate(zip(speeds, targets)):
        t = round((i + 1) * DT, 6)
        ok = True if eligible is None else eligible[i]
        match = ok and abs(speed - target) <= 2.
        steps.append(dict(t=t, speed=speed, sample=_sample(t, speed, match, 0. if jerks is None else jerks[i]),
                          pre_time=round(t - DT, 6), pre_speed=previous[0], pre_acceleration=previous[1],
                          eligible=ok, target=target, bounds=[], follow=None,
                          completed=abs(t - completed_at) < 1e-6,
                          status='succeeded' if t >= stable_at - 1e-6 else 'pending', drive=None))
        previous = (speed, (speed - previous[0]) / DT)
    return steps


def _recorded(steps):
    """The environment's quality entry for a car (what grade() reads from the report)."""
    return dict(full_approach_stop_seconds=0.)


def _ramp(start, rate, n, after=None):
    out, v = [], start
    for i in range(n):
        out.append(v)
        if after is None or i >= after:
            v += rate * DT
    return out


def test_window_is_completion_minus_8_s_to_the_stable_merge():
    speeds = [10.] * 200
    steps = _steps(speeds, [10.] * 200, completed_at=12., stable_at=18.)
    idx = R.window(steps)
    assert steps[idx[0]]['t'] == pytest.approx(4.)
    assert steps[idx[-1]]['t'] == pytest.approx(18.)
    assert len(idx) == 141


def test_no_rule_grade_is_the_environment_fraction():
    targets = [10.] * 100 + [20.] * 100
    steps = _steps([10.] * 200, targets, completed_at=10., stable_at=18.)
    graded = R.grade(steps, _recorded(steps))
    idx = R.window(steps)
    expected = sum(steps[i]['sample']['speed_match'] for i in idx) / len(idx)
    assert graded['speed_match_fraction'] == pytest.approx(expected)
    assert graded['samples'] == len(idx) and not graded['qualified']


def test_every_merge_grades_an_acceleration_lane_jump_on_the_follow_reference():
    # The target jumps from 8 to 24.6 m/s at the merge completion (10 s); the car then accelerates at 1.6 m/s^2.
    n = 200
    targets = [8.] * 100 + [24.6] * 100
    speeds = [8.] * 100 + [min(24.6, 8. + 1.6 * DT * (i + 1)) for i in range(100)]
    steps = _steps(speeds, targets, completed_at=10., stable_at=18.)
    assert not R.grade(steps, _recorded(steps))['qualified']
    assert R.grade(steps, _recorded(steps), ('every_merge',))['qualified']
    # A car that stays at 8 m/s after the jump still fails.
    slow = _steps([8.] * n, targets, completed_at=10., stable_at=18.)
    assert not R.grade(slow, _recorded(slow), ('every_merge',))['qualified']


def _yield_steps(speeds, crossing_index, completed_at, stable_at, road=15., rise=1.3):
    """A yield-entry car whose follow audits are recorded exactly as the environment would write them."""
    steps, previous, reference = [], (speeds[0], 0.), None
    for i, speed in enumerate(speeds):
        t = round((i + 1) * DT, 6)
        pre_time, pre_speed, pre_acc = round(t - DT, 6), previous[0], previous[1]
        started = reference is None
        if started:
            reference = F.start(pre_speed, pre_acc, None, rise, pre_time)
        else:
            reference, _ = F.advance(reference, road, None, rise, pre_speed, pre_acc, pre_time)
        lower, upper = F.band(reference['value'], road)
        audit = dict(road_target_m_s=road, reference_m_s=reference['value'], lower_m_s=lower, upper_m_s=upper,
                     slope_m_s2=reference['slope'], rise_m_s2=rise, drive_ceiling_m_s2=rise, ceiling_m_s=None,
                     reward_cap_m_s=None, yield_entry=i < crossing_index, started=started, restarted=False, ended=False)
        match = lower - 2. <= speed <= upper + 2.
        steps.append(dict(t=t, speed=speed, sample=_sample(t, speed, match), pre_time=pre_time, pre_speed=pre_speed,
                          pre_acceleration=pre_acc, eligible=True, target=reference['value'], bounds=[], follow=audit,
                          completed=abs(t - completed_at) < 1e-6,
                          status='succeeded' if t >= stable_at - 1e-6 else 'pending', drive=rise))
        previous = (speed, (speed - pre_speed) / DT)
    return steps


def test_restart_at_stop_line_forgives_a_wait_but_not_a_slow_start():
    # The car crawls at 1 m/s for 12 s while the reference rises, crosses the line, then accelerates at 1.25 m/s^2.
    wait, n = 120, 260
    speeds = [1.] * wait + [1. + 1.25 * DT * (i + 1) for i in range(n - wait)]
    steps = _yield_steps(speeds, crossing_index=wait, completed_at=20., stable_at=24.)
    assert R.replay_parity(steps) == 0.
    assert not R.grade(steps, _recorded(steps))['qualified']
    graded = R.grade(steps, _recorded(steps), ('restart_at_stop_line',))
    assert graded['qualified'], graded
    # The same wait, then only 0.5 m/s^2 after the line: still fails.
    slow = [1.] * wait + [1. + .5 * DT * (i + 1) for i in range(n - wait)]
    steps = _yield_steps(slow, crossing_index=wait, completed_at=20., stable_at=24.)
    assert not R.grade(steps, _recorded(steps), ('restart_at_stop_line',))['qualified']


def test_eligible_samples_only_drops_ineligible_samples_from_the_fraction_only():
    n = 200
    eligible = [not (110 <= i < 135) for i in range(n)]     # 25 ineligible samples inside the window
    steps = _steps([10.] * n, [10.] * n, completed_at=10., stable_at=18., eligible=eligible)
    assert not R.grade(steps, _recorded(steps))['qualified']
    graded = R.grade(steps, _recorded(steps), ('eligible_samples_only',))
    assert graded['qualified'] and graded['counted_samples'] == graded['samples'] - 25
    # An ineligible sample's jerk still counts.
    jerks = [9. if i == 120 else 0. for i in range(n)]
    steps = _steps([10.] * n, [10.] * n, completed_at=10., stable_at=18., eligible=eligible, jerks=jerks)
    assert not R.grade(steps, _recorded(steps), ('eligible_samples_only',))['qualified']
    # Fewer eligible samples than half the window: not graded as a pass.
    sparse = [i < 60 or i > 150 for i in range(n)]
    steps = _steps([10.] * n, [10.] * n, completed_at=10., stable_at=18., eligible=sparse)
    assert not R.grade(steps, _recorded(steps), ('eligible_samples_only',))['qualified']


def test_parity_check_catches_a_speed_match_it_does_not_reproduce():
    steps = _steps([10.] * 200, [10.] * 200, completed_at=10., stable_at=18.)
    report = dict(quality_merges=dict(agent_0=dict(R.grade(steps, _recorded(steps)), full_approach_stop_seconds=0.)))
    assert R.check_parity(dict(agent_0=steps), report)['cars'] == 1
    steps[150]['sample']['speed_match'] = False
    with pytest.raises(R.ParityMismatch):
        R.check_parity(dict(agent_0=steps), report)


def test_unknown_rule_is_refused():
    with pytest.raises(ValueError):
        R.check_rules(('lenient',))
