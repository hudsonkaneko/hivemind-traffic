import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'merge_target_audit', Path(__file__).resolve().parents[1] / 'scripts' / 'audit_highway_flow_rl_v06_merge_target.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
LIMITS = audit.LIMITS['gate']


def test_target_already_matched_is_fully_reachable():
    fraction, count = audit.best_match_fraction([20.] * 30, 20., 0., **LIMITS)
    assert (fraction, count) == (1., 30)


def test_acceleration_limit_bounds_the_ceiling():
    # 10 m/s below target at 2 m/s2 max (jerk-limited ramp-up): the first ~4 s cannot match.
    fraction, _ = audit.best_match_fraction([26.] * 80, 16., 0., **LIMITS)
    assert .4 < fraction < .6


def test_ineligible_targets_never_match():
    fraction, count = audit.best_match_fraction([None] * 5 + [20.] * 5, 20., 0., **LIMITS)
    assert count == 5 and fraction == .5


def test_jerk_limit_matters():
    slow = audit.best_match_fraction([26.] * 40, 20., 0., accel_min=-2., accel_max=2., jerk_max=1.)[0]
    fast = audit.best_match_fraction([26.] * 40, 20., 0., accel_min=-2., accel_max=2., jerk_max=5.)[0]
    assert fast > slow
