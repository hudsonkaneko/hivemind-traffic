"""CPU-only acceptance regressions; synthetic traces are not physics evidence."""
from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from traffic.lidar_braking import LidarBrakeConfig
from traffic.path_following import FollowerConfig
from traffic.visual_lidar_validation import assess_visual_lidar, barrier_plane_gap, validate_config


ROOT = Path(__file__).resolve().parents[1]


def configuration(mode='obstacle'):
    config = json.loads((ROOT/'experiments/physics-visual-lidar.json').read_text())
    config.update(mode=mode, sensor_hz=20, gui=False, points=False, paced=False,
                  capture=False, camera='follow', dropout_at_s=12,
                  braking=asdict(LidarBrakeConfig()), follower=asdict(FollowerConfig()))
    if mode == 'smoke':
        config['duration_s'] = 8
    return config


def fixture(mode='obstacle'):
    config = configuration(mode)
    total = round(config['duration_s']*120)
    frames = [dict(delivery_tick=t, acquisition_start_tick=t-12,
                   acquisition_end_tick=t-6, sensor_pose_error_m=0.)
              for t in range(12, total+1, 6)]
    # At dropout tick1440, latest delivered scan began at1428. Tick1452 is
    # still fresh; next controller tick1454 must brake. State rows lag by1.
    stop_tick = 1454 if mode == 'dropout' else 2760
    rows = []
    for i in range(total):
        stopped = mode != 'smoke' and i >= stop_tick
        progress = min(max(0., (i+1-240)/120*3), 30.4 if mode=='dropout' else 63.)
        status = ('stale_invalid' if mode=='dropout' else 'obstacle') if stopped else 'clear'
        rows.append(dict(tick=i+1, sim_time_s=(i+1)/120, control_tick=i%2==0,
            speed_m_s=0. if stopped or i<240 else 3., progress_m=progress,
            lateral_error_m=0., footprint_lateral_bound_m=.95, barrier_gap_m=66.6-progress,
            control=dict(is_fallback=False, throttle=0. if stopped else .1, brake=1. if stopped else 0.),
            driver=dict(fallback=False), lidar=dict(status=status,
                reason='stale_scan' if status=='stale_invalid' else 'fixture',
                stop_latched=stopped and mode=='obstacle',
                oldest_sample_age_s=None if status=='stale_invalid' else .1)))
    return config, rows, frames


def assess(config, rows, frames, **changes):
    kwargs = dict(contacts=[], errors=[], render_clock_checks=[True],
                  sensor_pose_errors=[0.], sensor_frames=frames)
    kwargs.update(changes)
    return assess_visual_lidar(rows, config, **kwargs)


@pytest.mark.parametrize('mode', ['smoke','obstacle','dropout'])
def test_ideal_declared_fixture_passes(mode):
    config, rows, frames = fixture(mode)
    assert validate_config(config) is config
    result = assess(config, rows, frames)
    assert result['passed'], result['gates']
    assert all(type(value) is bool for value in result['gates'].values())
    json.dumps(result, allow_nan=False)


def test_gap_and_gates_are_native_json_types():
    state = dict(position_m=np.array([0.,0.,1.]), quaternion_xyzw=np.array([0.,0.,0.,1.]))
    point = SimpleNamespace(yaw_rad=0., position_xy=(10.,0.))
    gap = barrier_plane_gap(state, point)
    assert type(gap) is float and gap == pytest.approx(6.6)
    config, rows, frames = fixture()
    rows[-1]['barrier_gap_m'] = np.float64(3.)
    result = assess(config, rows, frames)
    assert type(result['gates']['safe_gap']) is bool
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('key,value', [('sensor_hz',10), ('physics_hz',120.), ('control_hz',True),
    ('duration_s',math.nan), ('duration_s',61), ('duration_s',8), ('duration_s',40.001),
    ('max_process_seconds',600), ('settle_s',0), ('barrier_station_m',60),
    ('dropout_at_s',13), ('max_gpu_fraction',.95), ('route_file','different.json'),
    ('camera','other'), ('points',1), ('seed',True)])
def test_bad_config_is_rejected(key, value):
    config = configuration()
    config[key] = value
    with pytest.raises(ValueError):
        validate_config(config)


@pytest.mark.parametrize('section,key,value', [('braking','max_scan_age_ticks',25),
    ('braking','min_height_m',-.5), ('braking','front_bumper_offset_m',1.6),
    ('acceptance','sensor_max_age_s',.21), ('acceptance','minimum_barrier_gap_m',.49),
    ('acceptance','render_must_not_advance_physics',False), ('follower','wheelbase_m',2.)])
def test_relaxed_limits_and_wrong_geometry_are_rejected(section, key, value):
    config = configuration()
    config[section][key] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_unresolved_controller_settings_are_rejected():
    config = configuration()
    del config['follower']['lookahead_time_s']
    with pytest.raises(ValueError):
        validate_config(config)


@pytest.mark.parametrize('change', ['missing_pose','too_few','future_acquisition','high_pose'])
def test_sensor_frame_coverage_and_timing_are_required(change):
    config, rows, frames = fixture()
    if change == 'missing_pose':
        frames[-1]['sensor_pose_error_m'] = None
    elif change == 'too_few':
        frames = frames[-9:]
    elif change == 'future_acquisition':
        frames[-1]['acquisition_end_tick'] = frames[-1]['delivery_tick']+1
    else:
        frames[-1]['sensor_pose_error_m'] = .051
    result = assess(config, rows, frames)
    assert not result['passed']
    assert not result['gates']['sensor_pose']


def test_fresh_scan_age_gate_covers_obstacle_decisions_too():
    config, rows, frames = fixture()
    rows[2760]['lidar']['oldest_sample_age_s'] = .201
    assert not assess(config, rows, frames)['gates']['fresh_driving_scans']


def test_false_positive_early_obstacle_stop_cannot_pass():
    config, rows, frames = fixture()
    for row in rows[1200:]:
        row.update(progress_m=24., speed_m_s=0., barrier_gap_m=42.)
        row['lidar'].update(status='obstacle', stop_latched=True)
        row['control'].update(throttle=0., brake=1.)
    result = assess(config, rows, frames)
    assert not result['gates']['lidar_obstacle_stop']
    assert not result['gates']['obstacle_stop_location']


def test_obstacle_latch_cannot_release_and_drive_again():
    config, rows, frames = fixture()
    rows[-1]['control']['brake'] = 0.
    assert not assess(config, rows, frames)['gates']['lidar_obstacle_stop']


@pytest.mark.parametrize('gap', [.49,6.01])
def test_obstacle_final_gap_must_be_inside_declared_band(gap):
    config, rows, frames = fixture()
    rows[-1]['barrier_gap_m'] = gap
    assert not assess(config, rows, frames)['gates']['obstacle_stop_location']


def test_dropout_deadline_uses_original_oldest_tick_and_poststep_rows():
    config, rows, frames = fixture('dropout')
    result = assess(config, rows, frames)
    assert result['metrics']['expected_dropout_fault_tick'] == 1454
    assert result['metrics']['first_dropout_fault_tick'] == 1454
    assert rows[1452]['lidar']['status'] == 'clear'
    assert rows[1454]['lidar']['status'] == 'stale_invalid'


@pytest.mark.parametrize('fault', ['late','early','noncontrol_throttle','wrong_reason','already_stopped'])
def test_dropout_failures_do_not_pass(fault):
    config, rows, frames = fixture('dropout')
    if fault == 'late':
        for row in rows[1454:1456]:
            row['lidar'].update(status='clear', reason='fixture', oldest_sample_age_s=.1)
    elif fault == 'early':
        rows[1452]['lidar'].update(status='stale_invalid', reason='stale_scan', oldest_sample_age_s=None)
    elif fault == 'noncontrol_throttle':
        rows[1455]['control']['throttle'] = .1
    elif fault == 'wrong_reason':
        rows[1454]['lidar']['reason'] = 'missing_or_invalid_scan'
    else:
        for row in rows[1380:1440]:
            row['speed_m_s'] = 0.
    assert not assess(config, rows, frames)['passed']


@pytest.mark.parametrize('fault', ['missing_tick','nonfinite','clock','held_control_flag','bad_brake','spawn_at_finish'])
def test_trace_integrity_and_spawn_are_required(fault):
    config, rows, frames = fixture()
    if fault == 'missing_tick':
        rows = rows[:-1]
    elif fault == 'nonfinite':
        rows[400]['lateral_error_m'] = math.nan
    elif fault == 'clock':
        rows[400]['sim_time_s'] += .01
    elif fault == 'held_control_flag':
        rows[400]['control_tick'] = False
    elif fault == 'bad_brake':
        rows[400]['control']['brake'] = 2.
    else:
        rows[0]['progress_m'] = 63.
    assert not assess(config, rows, frames)['passed']


@pytest.mark.parametrize('change', [dict(contacts=[{}]), dict(errors=['capture fault']),
    dict(render_clock_checks=[]), dict(render_clock_checks=[True,False])])
def test_external_runtime_guard_failures_are_not_hidden(change):
    config, rows, frames = fixture()
    assert not assess(config, rows, frames, **change)['passed']


def test_missing_sensor_frames_cannot_be_replaced_by_a_good_pose_list():
    config, rows, _ = fixture()
    assert not assess(config, rows, [], sensor_pose_errors=[0.])['gates']['sensor_pose_coverage']
