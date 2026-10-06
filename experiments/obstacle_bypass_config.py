"""Frozen low-speed fixture settings; these are not general tuning knobs."""
from dataclasses import asdict

from traffic.lidar_braking import LidarBrakeConfig
from traffic.obstacle_bypass import BypassConfig
from traffic.path_following import FollowerConfig


FIXTURE = dict(schema_version=1, seed=101, physics_hz=120, control_hz=60,
    planner_hz=10, render_hz=30, duration_s=50, settle_s=2, dropout_at_s=12,
    max_process_seconds=540, max_gpu_fraction=.90, road_y_bounds_m=[-1.8, 5.4],
    nominal_stop_x_m=90, barrier_x_m=45, barrier_dimensions_m=[2, 1.6, 1.5],
    target_speed_m_s=3, minimum_clearance_m=.35, tracking_error_max_m=.5,
    sensor_pose_error_max_m=.05, hold_s=5)


def validate_config(config, *, resolved=False):
    expected = dict(FIXTURE)
    if resolved and config.get('real_time') is True:
        expected['render_hz'] = 20
    if resolved:
        expected.update(braking=asdict(LidarBrakeConfig()), follower=asdict(FollowerConfig()),
                        planner=asdict(BypassConfig()))
    options = {'mode', 'gui', 'camera', 'points', 'capture', 'paced'} if resolved else set()
    # Old resolved evidence remains readable; omission means the original RGB profile.
    if resolved and 'real_time' in config:
        options.add('real_time')
    if set(config) != set(expected) | options:
        raise ValueError('Unexpected or missing bounded bypass settings')
    for key, value in expected.items():
        if isinstance(config[key], bool) or config[key] != value:
            raise ValueError('Unsupported bounded bypass setting: '+key)
    if resolved:
        if config['mode'] not in ('pass', 'blocked', 'dropout') or config['camera'] not in ('follow', 'overview'):
            raise ValueError('Unsupported bypass mode/camera')
        if any(type(config[key]) is not bool for key in ('gui', 'points', 'capture', 'paced')):
            raise ValueError('Expected boolean viewer settings')
        if type(config.get('real_time', False)) is not bool:
            raise ValueError('Expected boolean real-time setting')
        if config.get('real_time', False) and not config['paced']:
            raise ValueError('Real-time preview requires pacing')
    return config
