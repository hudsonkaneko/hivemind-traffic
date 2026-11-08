"""CPU checks for identification split, parameter recovery and case accounting."""
import copy
import json
from pathlib import Path

import pytest

from scripts.calibrate_mappo_physical import (
    ROOT, _scripted_request, assess_case, fit_calibration, validate_config,
)


def config():
    return json.loads((ROOT / 'experiments/configs/mappo_physical_adapter_v01.json').read_text())


def synthetic_rows():
    rows = []
    samples = [('half', .5, 0, 4., 'fit'), ('drive', 1, 0, 14., 'fit'),
        ('coast', 0, 0, 7., 'fit'), ('coast_fast', 0, 0, 17., 'fit'),
        ('brake', 0, .25, 13., 'fit'), ('heldout_drive', .75, 0, 11., 'validation'),
        ('heldout_brake', 0, .5, 9., 'validation'), ('confirmation', 0, .4, 10., 'confirmation')]
    for phase, throttle, brake, speed, split in samples:
        for tick in range(360):
            rows.append(dict(phase=phase, split=split, speed_m_s=speed, throttle=throttle, brake=brake,
                acceleration_m_s2=2.3 * throttle - 8.2 * brake - .07 - .013 * speed))
    return rows


def test_fit_recovers_dynamics_and_holds_out_pedal_levels():
    fit = fit_calibration(synthetic_rows())
    assert fit['parameters'] == pytest.approx(dict(throttle_gain_m_s2=2.3, brake_gain_m_s2=8.2,
        coast_m_s2=.07, coast_speed_gain_s_inv=.013))
    assert fit['heldout_rmse_m_s2'] < 1e-10
    assert fit['confirmation_rmse_m_s2'] < 1e-10
    assert fit['provisional_heldout_rmse_m_s2'] > .1
    assert all(r['throttle'] != .75 and r['brake'] != .5 for r in fit['windows'] if r['split'] == 'fit')


def test_holdout_corruption_changes_score_not_fit():
    original = fit_calibration(synthetic_rows())
    rows = synthetic_rows()
    for row in rows:
        if row['split'] == 'validation':
            row['acceleration_m_s2'] += 1
    changed = fit_calibration(rows)
    assert changed['parameters'] == original['parameters']
    assert changed['heldout_rmse_m_s2'] == pytest.approx(1)


def test_saturated_full_braking_cannot_distort_ordinary_fit():
    rows = synthetic_rows()
    original = fit_calibration(rows)
    for _ in range(360):
        rows.append(dict(phase='saturated', split='fit', speed_m_s=12, throttle=0, brake=1,
                         acceleration_m_s2=-6.8))
    changed = fit_calibration(rows)
    assert changed['parameters'] == original['parameters']
    assert changed['saturation_probe']['mean_acceleration_m_s2'] == pytest.approx(-6.8)
    assert changed['saturation_probe']['window_count'] > 0


def test_stopped_and_guarded_samples_do_not_calibrate_brakes():
    rows = synthetic_rows()
    for row in rows:
        row['speed_m_s'] = 0
    with pytest.raises(ValueError, match='Insufficient'):
        fit_calibration(rows)


def test_config_prohibits_training_and_budget_extension():
    cfg = config()
    assert validate_config(cfg) is cfg
    for key, value in [('training_enabled', True), ('max_process_seconds', 901), ('physics_hz', 60)]:
        changed = dict(cfg, **{key: value})
        with pytest.raises(ValueError):
            validate_config(changed)


@pytest.mark.parametrize('speed', [6, 12, 18, 20])
def test_lane_requests_reach_ten_hz_controller(speed):
    requests = [_scripted_request(tick / 10, speed, speed)[1] for tick in range(600)]
    assert requests.count(1) == 1
    assert requests.count(-1) == 1


def test_adapter_fallback_is_counted_even_with_valid_driver_command():
    cfg = config()
    rows = [dict(sim_time_s=40, speed_m_s=12, adapter={'is_fallback': True}, fallback=False,
                 throttle=0, brake=1)]
    result = assess_case(dict(passed=True, contact_count=0, static_files_unchanged=True), rows,
        [dict(causes=[], lane_delta=0)], dict(kind='frozen', duration_s=40, speed_limit_m_s=12), cfg)
    assert result['metrics']['adapter_fallback_ticks'] == 1
    assert result['metrics']['driver_fallback_ticks'] == 0
    assert result['gates']['no_adapter_fallback'] is False
    assert result['passed'] is False


def test_case_speed_ceiling_uses_local_limit():
    cfg = config()
    rows = [dict(sim_time_s=40, speed_m_s=13, adapter={}, fallback=False, throttle=0, brake=0)]
    result = assess_case(dict(passed=True, contact_count=0, static_files_unchanged=True), rows,
        [dict(causes=[], lane_delta=0)], dict(kind='frozen', duration_s=40, speed_limit_m_s=12), cfg)
    assert result['metrics']['overspeed_ticks'] == 1
    assert result['gates']['no_overspeed'] is False
