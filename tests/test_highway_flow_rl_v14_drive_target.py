"""V14 drive-aware reward target: start lead, rise rate and fall rate (no SUMO)."""
import pytest

from traffic import highway_flow_rl_v14_drive_target as drive
from traffic import highway_flow_rl_v14_rewards as v14


def test_settings_keep_v14_rates_and_drop_the_lead():
    assert drive.SETTINGS['rise_m_s2'] == v14.SETTINGS['target_acceleration_m_s2'] == 1.5
    assert drive.SETTINGS['fall_m_s2'] == v14.SETTINGS['target_deceleration_m_s2'] == 2.
    assert drive.SETTINGS['initial_lead_m_s'] == 0.
    assert drive.validate_settings(dict(drive.SETTINGS)) == drive.SETTINGS
    with pytest.raises(ValueError):
        drive.validate_settings(dict(drive.SETTINGS, initial_lead_m_s=4.5))


def test_rise_rate_is_capped_by_the_drive_ceiling():
    assert drive.rise_rate(None) == 1.5
    assert drive.rise_rate(2.0) == 1.5
    assert drive.rise_rate(1.28) == pytest.approx(1.28)
    assert drive.rise_rate(-.3) == 0.
    with pytest.raises(ValueError):
        drive.rise_rate(float('nan'))


def test_start_is_the_car_speed_not_four_and_a_half_above():
    # Logged case 9 agent_5 at 0.2 s: speed 11.99, plan 20.1; v14 starts at 16.49.
    assert v14.reachable(20.1, None, 11.99, None, .2) == pytest.approx(16.49)
    assert drive.reachable(20.1, None, 11.99, None, .2, 1.28) == pytest.approx(11.99)
    assert drive.reachable(8., None, 11.99, None, .2, 1.28) == 8.            # a lower plan applies at once
    assert drive.reachable(20.1, 10., 11.99, None, .2, 1.28) == 10.          # and so does the lane/map ceiling


def test_a_car_at_its_drive_ceiling_stays_on_target_and_a_slow_car_falls_behind():
    target, ideal, slow, previous = None, 12., 12., None
    for step in range(50):
        now = .2 + .1 * step
        target = drive.reachable(26.8, None, ideal, previous, now, 1.28)
        previous = dict(time=now, value=target)
        assert abs(target - ideal) <= .13 + 1e-9
        ideal += 1.28 * .1
    previous = None
    for step in range(50):
        now = .2 + .1 * step
        target = drive.reachable(26.8, None, slow, previous, now, 1.28)
        previous = dict(time=now, value=target)
        slow += .5 * .1
    assert target - slow > 3.        # 5 s at 0.78 m/s^2 behind


def test_fall_and_a_car_above_the_target():
    previous = dict(time=1., value=20.)
    assert drive.reachable(10., None, 15., previous, 1.1, 1.3) == pytest.approx(19.8)    # falls at 2.0 m/s^2
    assert drive.reachable(26., None, 22., previous, 1.1, 1.3) == pytest.approx(22.13)   # rises from the car's speed
    with pytest.raises(ValueError):
        drive.reachable(26., None, 22., dict(time=1.2, value=20.), 1.1, 1.3)
