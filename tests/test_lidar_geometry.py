import numpy as np
from scripts.lidar_geometry import directions, ray_box_ranges, fixture_positions
from scripts.lidar_geometry import analyze_scan, summarize
import json
from pathlib import Path


def test_ray_box_front_face_and_parallel_miss():
    origins = np.array([[0, 0, 0], [0, 4, 0], [0, 0, 0]])
    rays = directions(np.array([0, 0, 180]), np.zeros(3))
    actual = ray_box_ranges(origins, rays, np.tile([5, 0, 0], (3, 1)), [2, 2, 2])
    assert actual[0] == 4
    assert np.isinf(actual[1:]).all()


def test_direction_sign_and_units():
    np.testing.assert_allclose(directions([0, 90], [0, 0]), [[1, 0, 0], [0, 1, 0]], atol=1e-12)


def test_fixture_relative_motion():
    config = dict(sensor_mount_m=[0, 0, 1], target_initial_center_m=[12, 2, 1],
                  ego_velocity_mps=1, target_velocity_mps=3)
    sensor, target = fixture_positions(np.array([0, 2]), config)
    np.testing.assert_allclose(target - sensor, [[12, 2, 0], [16, 2, 0]])


def test_measured_range_does_not_select_good_points():
    config = json.loads((Path(__file__).parents[1] / 'experiments/configs/lidar_validation.json').read_text())
    az = np.array([10.0, 10.0])
    scan = dict(azimuth_deg=az, elevation_deg=np.zeros(2), timestamp_ns=1_000_000_000,
                offset_ns=np.zeros(2), flags=np.array([64, 64]), range_m=np.array([500.0, 501.0]))
    metrics, errors = analyze_scan(scan, config)
    assert metrics['expected_hits'] == 2
    assert errors.min() > 400


def test_empty_scans_cannot_pass():
    config = json.loads((Path(__file__).parents[1] / 'experiments/configs/lidar_validation.json').read_text())
    assert not summarize([], config)['passed']


def test_ray_inside_box_returns_exit():
    result = ray_box_ranges(np.zeros((1, 3)), np.array([[1, 0, 0]]), np.zeros((1, 3)), [2, 2, 2])
    assert result[0] == 1


def test_timing_gate_rejects_clock_drift_but_not_callback_delay():
    config = json.loads((Path(__file__).parents[1] / 'experiments/configs/lidar_validation.json').read_text())
    config = dict(config, warmup_scans=0, min_scans=2)
    records = []
    for i in range(2):
        ts = 1 + i * .1
        start, end = ts + .1 - 1/60, ts + .1
        a, _ = fixture_positions(np.array([start-config['sensor_to_usd_offset_s']]), config)
        b, _ = fixture_positions(np.array([end-config['sensor_to_usd_offset_s']]), config)
        records.append(dict(expected_hits=100, detection_fraction=1, direction_consistency_fraction=1,
                            range_p95_m=.01, range_max_m=.02, timestamp_ns=round(ts*1e9),
                            offset_min_ns=0, offset_max_ns=99_000_000,
                            frame_start_ns=round(start*1e9), frame_end_ns=round(end*1e9),
                            frame_start_position=a[0], frame_end_position=b[0], callback_time_s=ts+5,
                            coordinate_type='CoordsType.SPHERICAL', reference='FrameOfReference.SENSOR',
                            motion_compensation='MotionCompensationState.NONCOMPENSATED'))
    assert summarize(records, config)['passed']
    records[1]['frame_end_position'] = [100, 0, 1]
    assert not summarize(records, config)['timing_validated']
