"""CPU contracts for the opt-in fixture, not evidence of high-speed dynamics."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
from decimal import Decimal
import json
import math
from pathlib import Path

import pytest

from experiments.obstacle_bypass_config import (
    FIXTURE, FIXTURE_35, controller_settings, validate_config,
)
from traffic.bypass_validation import (
    OrientedBox, RouteSafetyBrake, assess_bypass, footprint_within_road, predicted_route_poses,
)
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.lane_validation import validate_config as validate_lane_config
from traffic.lidar_braking import LidarBrakeConfig, LidarEmergencyBrake, LidarScan
from traffic.obstacle_bypass import BypassConfig, ObstacleBypassPlanner, road_footprint_bounds
from traffic.path_following import FollowerConfig, VehicleState
from traffic.physical_lidar import (
    MAX_PACKET_ELEMENTS, MAX_SCAN_SPAN_S, SENSOR_RATE_HZ, _configure_rotary_profile,
)
from traffic.speed_profiles import MPH_TO_M_S, TARGET_35_MPH_M_S, speed_limit
from traffic.visual_lidar_validation import validate_config as validate_visual_lidar_config


SPEED = TARGET_35_MPH_M_S
ROOT = Path(__file__).resolve().parents[1]


def resolved(profile):
    config = deepcopy(FIXTURE if profile == 'low-speed' else FIXTURE_35)
    return dict(config, mode='pass', gui=False, camera='follow', points=False,
                capture=True, paced=False, **controller_settings(profile))


def brake_config():
    return LidarBrakeConfig(**controller_settings('35mph')['braking'])


def planner_config():
    return BypassConfig(**controller_settings('35mph')['planner'])


def nominal_route():
    return LaneRoute('straight', 3.6, (RouteSegment(550),))


def sample(tick=0, *, acquisition=None, x=None):
    x = 139 + SPEED*tick/120 if x is None else x
    acquisition = tick if acquisition is None else acquisition
    state = VehicleState('episode', 'car', tick, x, 0, 0, SPEED)
    scan = LidarScan('episode', 'car', acquisition, acquisition, tick,
                    [(299-x, y, 0) for y in (-.8, 0, .8)],
                    coordinate_reference_tick=tick)
    return scan, state


def confirmed_planner():
    planner = ObstacleBypassPlanner('episode', 'car', nominal_route(), planner_config())
    scan, state = sample()
    assert planner.update(scan, state, 0).status == 'confirming'
    scan, state = sample(12)
    decision = planner.update(scan, state, 12)
    assert decision.status == 'detour'
    return planner, decision


@pytest.mark.parametrize('filename,expected', [
    ('physics-obstacle-bypass.json', FIXTURE),
    ('physics-obstacle-bypass-35mph.json', FIXTURE_35),
])
def test_named_fixture_matches_checked_in_frozen_config(filename, expected):
    config = json.loads((ROOT/'experiments'/filename).read_text())
    assert validate_config(config) == expected


def test_35mph_conversion_and_fixture_geometry_are_explicit():
    assert MPH_TO_M_S == float(Decimal('1609.344')/Decimal('3600'))
    assert SPEED == float(Decimal('35')*Decimal('0.44704')) == 15.6464
    assert speed_limit('35mph') == 16.0  # Guard is distinct from cruise target.
    assert speed_limit('low-speed') == 3.0
    assert FIXTURE_35['road_length_m'] == 550
    assert FIXTURE_35['nominal_stop_x_m'] == 500
    assert FIXTURE_35['barrier_x_m'] == 300
    assert FIXTURE_35['duration_s'] == 60
    assert FIXTURE_35['dropout_at_s'] == 15
    cfg = planner_config()
    assert (cfg.shift_x_m, cfg.longitudinal_clearance_m, cfg.detection_range_m) == (70, 60, 160)


@pytest.mark.parametrize('key,value', [
    ('target_speed_m_s', 16), ('road_length_m', 500), ('barrier_x_m', 299),
    ('nominal_stop_x_m', 499), ('dropout_at_s', 12), ('duration_s', 50),
    ('speed_profile', 'low-speed'), ('physics_hz', 60), ('unknown', 1),
])
def test_high_speed_fixture_rejects_ad_hoc_changes(key, value):
    config = deepcopy(FIXTURE_35)
    config[key] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_resolved_profiles_preserve_legacy_evidence_without_profile_keys():
    for profile in ('low-speed', '35mph'):
        assert validate_config(resolved(profile), resolved=True)
    legacy = resolved('low-speed')
    for name in ('braking', 'follower', 'planner'):
        legacy[name].pop('speed_profile')
    assert validate_config(legacy, resolved=True)
    realtime = resolved('35mph')
    realtime.update(real_time=True, paced=True, render_hz=20)
    assert validate_config(realtime, resolved=True)
    with pytest.raises(ValueError):
        validate_config(dict(realtime, paced=False), resolved=True)


@pytest.mark.parametrize('section,key,value', [
    ('braking', 'max_scan_age_ticks', 25),
    ('braking', 'planning_deceleration_m_s2', 2),
    ('follower', 'max_speed_m_s', 17),
    ('planner', 'confirmation_scans', 1),
    ('planner', 'detection_range_m', 100),
    ('planner', 'speed_profile', 'low-speed'),
])
def test_resolved_high_speed_controller_settings_are_frozen(section, key, value):
    config = resolved('35mph')
    config[section][key] = value
    with pytest.raises(ValueError):
        validate_config(config, resolved=True)


@pytest.mark.parametrize('factory,key', [
    (LidarBrakeConfig, 'max_speed_m_s'), (FollowerConfig, 'max_speed_m_s'),
    (BypassConfig, 'target_speed_m_s'),
])
def test_speed_requires_explicit_profile_and_remains_capped(factory, key):
    assert getattr(factory(), key) == 3
    with pytest.raises(ValueError):
        factory(**{key: SPEED})
    cfg = factory(speed_profile='35mph', **{key: SPEED})
    assert getattr(cfg, key) == SPEED
    with pytest.raises(FrozenInstanceError):
        setattr(cfg, key, 100)
    for invalid in (16.00001, math.inf, math.nan, True):
        with pytest.raises(ValueError):
            factory(speed_profile='35mph', **{key: invalid})
    with pytest.raises(ValueError):
        factory(speed_profile='unbounded')


def test_high_speed_planner_validates_160m_scan_and_requires_distinct_confirmations():
    planner = ObstacleBypassPlanner('episode', 'car', nominal_route(), planner_config())
    scan, state = sample()
    assert scan.points[0][0] == 160
    assert planner.update(scan, state, 0).status == 'confirming'
    scan, state = sample(2, acquisition=0)
    assert planner.update(scan, state, 2).status == 'confirming'
    scan, state = sample(12)
    decision = planner.update(scan, state, 12)
    assert decision.status == 'detour'
    assert decision.target_speed_m_s == SPEED
    assert decision.sensed_bounds == pytest.approx((299, 299, -.8, .8))
    assert decision.sensed_acquisition_ticks == (0, 12)
    scan, state = sample(24)
    assert planner.update(scan, state, 24).route is decision.route
    low_speed = ObstacleBypassPlanner('episode', 'car', nominal_route())
    assert low_speed.update(scan, state, 24).reason == 'invalid_ego_odometry'


@pytest.mark.parametrize('changes,reason', [
    ({'episode_id': 'other'}, 'identity_mismatch'),
    ({'vehicle_id': 'other'}, 'identity_mismatch'),
    ({'frame': 'world'}, 'wrong_coordinate_frame'),
    ({'acquisition_start_tick': 0, 'acquisition_end_tick': 0}, 'stale_scan'),
    ({'acquisition_start_tick': 0}, 'excessive_acquisition_span'),
    ({'acquisition_end_tick': 37}, 'future_or_misordered_scan'),
    ({'coordinate_reference_tick': 35}, 'invalid_coordinate_reference_tick'),
    ({'acquisition_start_tick': 24, 'acquisition_end_tick': 24,
      'coordinate_reference_tick': 35}, 'scan_not_in_current_chassis_frame'),
    ({'points': []}, 'empty_point_cloud'),
    ({'points': [(10, 0, math.nan)]}, 'nonfinite_or_invalid_point'),
])
def test_high_speed_frozen_planner_still_fails_closed_on_bad_scans(changes, reason):
    planner, _ = confirmed_planner()
    scan, state = sample(36)
    decision = planner.update(replace(scan, **changes), state, 36)
    assert decision.status == 'stale_invalid'
    assert decision.reason == reason
    assert decision.target_speed_m_s == 0


@pytest.mark.parametrize('changes', [
    {'episode_id': 'other'}, {'vehicle_id': 'other'}, {'tick': 35},
    {'frame': 'world'}, {'source': 'lidar'}, {'x_m': math.nan},
    {'speed_m_s': 16.00001}, {'speed_m_s': -1},
])
def test_high_speed_planner_preserves_odometry_guards(changes):
    planner, _ = confirmed_planner()
    scan, state = sample(36)
    decision = planner.update(scan, replace(state, **changes), 36)
    assert (decision.status, decision.reason, decision.target_speed_m_s) == (
        'stale_invalid', 'invalid_ego_odometry', 0)


def test_extended_route_has_bounded_footprint_curvature_and_return():
    planner, decision = confirmed_planner()
    route, cfg = decision.route, planner.config
    end = route.evaluate(route.length_m)
    assert end.position_xy == pytest.approx((500, 0), abs=1e-8)
    assert end.yaw_rad == pytest.approx(0, abs=1e-10)
    assert decision.stop_s_m == pytest.approx(route.length_m)
    assert route.project(299, 3.8).distance_m < 1e-8
    assert route.segments[0].length_m == pytest.approx(167)
    radius = (70**2+3.8**2)/(4*3.8)
    curvature = max(abs(segment.curvature_rad_m) for segment in route.segments)
    assert curvature == pytest.approx(1/radius)
    assert SPEED**2*curvature < .8  # Ideal reference only, not a dynamics claim.
    lower, upper = road_footprint_bounds(route, cfg)
    assert -1.8 <= lower < upper <= 5.4
    for index in range(2001):
        point = route.evaluate(index*route.length_m/2000)
        box = OrientedBox(*point.position_xy, point.yaw_rad)
        assert footprint_within_road(box, x_min_m=-5, x_max_m=550)
        assert all(lower <= y <= upper for _, y in box.corners())


@pytest.mark.parametrize('speed', [SPEED, 16.0])
def test_stopping_horizon_accounts_for_oldest_sample_and_full_speed(speed):
    cfg = brake_config()
    assert cfg.max_scan_age_ticks == cfg.max_scan_span_ticks == 24
    assert cfg.max_points == MAX_PACKET_ELEMENTS == 500_000
    state = dict(position_m=[139, 0, 1], yaw_rad=0)
    scan = LidarScan('episode', 'car', 0, 6, 6, [(200, 10, 0)],
                     coordinate_reference_tick=24)
    poses = predicted_route_poses(nominal_route(), state, speed, config=cfg)
    assert poses[0] == (0, 0, 0)
    distances = [math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:])]
    assert max(distances) <= .250001
    expected = speed**2/(2*1.5)+speed*(.2+.2)+1
    assert expected > 88
    assert expected <= sum(distances) < expected+1.11
    decision = RouteSafetyBrake('episode', 'car', cfg).evaluate(
        scan, tick=24, speed_m_s=speed, requested_target_speed_m_s=SPEED,
        predicted_route=poses)
    assert decision.status == 'clear'
    assert decision.oldest_sample_age_s == .2
    assert decision.stopping_distance_m == pytest.approx(expected)
    assert decision.target_speed_m_s == SPEED


def test_high_speed_preview_requires_opt_in_and_short_route_fails_closed():
    state = dict(position_m=[139, 0, 1], yaw_rad=0)
    with pytest.raises(ValueError):
        predicted_route_poses(nominal_route(), state, SPEED)
    cfg = brake_config()
    scan = LidarScan('episode', 'car', 0, 0, 0, [(200, 10, 0)])
    short = [(index*.2, 0, 0) for index in range(30)]
    decision = RouteSafetyBrake('episode', 'car', cfg).evaluate(
        scan, tick=0, speed_m_s=SPEED, requested_target_speed_m_s=SPEED,
        predicted_route=short)
    assert decision.reason == 'route_shorter_than_stopping_horizon'
    assert (decision.status, decision.brake_override, decision.target_speed_m_s) == (
        'stale_invalid', 1, 0)
    stale = replace(scan, coordinate_reference_tick=25)
    decision = LidarEmergencyBrake('episode', 'car', cfg).evaluate(
        stale, tick=25, speed_m_s=SPEED, requested_target_speed_m_s=SPEED)
    assert (decision.reason, decision.brake_override) == ('stale_scan', 1)


def test_high_speed_brake_stops_inside_extended_corridor_and_keeps_latch():
    cfg = brake_config()
    scan = LidarScan('episode', 'car', 0, 0, 0, [(85+cfg.front_bumper_offset_m, 0, 0)])
    brake = LidarEmergencyBrake('episode', 'car', cfg)
    decision = brake.evaluate(scan, tick=0, speed_m_s=SPEED,
                              requested_target_speed_m_s=SPEED)
    assert (decision.reason, decision.brake_override, decision.target_speed_m_s) == (
        'obstacle_in_stopping_corridor', 1, 0)
    assert decision.nearest_gap_m == 85
    clear = replace(scan, points=[(200, 10, 0)], coordinate_reference_tick=2)
    decision = brake.evaluate(clear, tick=2, speed_m_s=SPEED,
                              requested_target_speed_m_s=SPEED)
    assert (decision.reason, decision.brake_override) == ('latched_obstacle_stop', 1)


class MockAttribute:
    def __init__(self, name, value=None, *, fail_set=False, fail_readback=False):
        self.name, self.value = name, value
        self.fail_set, self.fail_readback = fail_set, fail_readback

    def GetName(self):
        return self.name

    def Get(self):
        return -1 if self.fail_readback else self.value

    def Set(self, value):
        if self.fail_set:
            return False
        self.value = value
        return True


class MockPrim:
    def __init__(self):
        name = 'omni:sensor:Core:emitterState:s001:azimuthDeg'
        self.attrs = {name: MockAttribute(name, list(range(128)))}

    def GetAttributes(self):
        return list(self.attrs.values())

    def GetAttribute(self, name):
        return self.attrs.setdefault(name, MockAttribute(name))


@pytest.mark.parametrize('profile,rate', [('low-speed', 7200.), ('35mph', 72000.)])
def test_lidar_density_readback_preserves_20hz_frames_and_timing(profile, rate):
    prim = MockPrim()
    assert _configure_rotary_profile(prim, profile) == dict(
        sensor_tick=20., rotary_scan=20., pattern_firing=rate)
    assert SENSOR_RATE_HZ == 20
    assert MAX_SCAN_SPAN_S == pytest.approx(.06)
    for suffix, expected in [('numberOfEmitters', 32), ('elementsCoordsType', 'CARTESIAN'),
                             ('outputFrameOfReference', 'WORLD'),
                             ('outputMotionCompensationState', 'NONCOMPENSATED')]:
        assert prim.GetAttribute('omni:sensor:Core:'+suffix).Get() == expected
    assert len(prim.GetAttribute('omni:sensor:Core:emitterState:s001:azimuthDeg').Get()) == 32


@pytest.mark.parametrize('failure', ['fail_set', 'fail_readback'])
@pytest.mark.parametrize('name', ['omni:sensor:tickRate', 'omni:sensor:Core:scanRateBaseHz',
                                 'omni:sensor:Core:patternFiringRateHz'])
def test_high_speed_lidar_rejects_unverified_rates(name, failure):
    prim = MockPrim()
    prim.attrs[name] = MockAttribute(name, **{failure: True})
    with pytest.raises(RuntimeError, match='configure/read back'):
        _configure_rotary_profile(prim, '35mph')


def assess_high_speed(rows, **kwargs):
    settings = dict(mode='pass', obstacle=OrientedBox(300, 0, 0, 2, 1.6),
                    physics_hz=120, max_speed_m_s=16, stop_x_m=500,
                    adoption_before_x_m=165, dropout_at_s=15,
                    target_speed_m_s=SPEED, require_cruise=True, road_length_m=550)
    settings.update(kwargs)
    return assess_bypass(rows, **settings)


def synthetic_row(tick, position, *, speed=SPEED, phase='drive', safety='clear', yaw=0):
    """Acceptance-gate input only: no claim that these rows reproduce dynamics."""
    failed = safety != 'clear'
    return dict(tick=tick, position_m=[*position, 1], yaw_rad=yaw,
                speed_m_s=speed, upright_z=1, wheel_on_ground=[True]*4,
                obstacle_contacts=0, phase=phase,
                control=dict(brake=1 if failed or speed == 0 else 0, throttle=0),
                control_tick=tick % 2 == 1, safety_status=safety,
                route_adopted_from_lidar=position[0] >= 140,
                lidar=dict(status=safety, brake_override=1 if failed else 0,
                           target_speed_m_s=0 if failed else SPEED, oldest_sample_age_s=.2),
                planner=dict(status='detour' if position[0] >= 140 else 'nominal'))


@pytest.fixture(scope='module')
def synthetic_pass_trace():
    _, decision = confirmed_planner()
    route = decision.route
    rows = []
    for tick in range(1, 7201):
        distance = max(0, (tick-240)*SPEED/120)
        point = route.evaluate(min(route.length_m, distance))
        speed = SPEED if 240 < tick and distance < route.length_m else 0
        phase = 'settle' if tick <= 240 else 'hold' if tick > 6600 else 'drive'
        rows.append(synthetic_row(tick, point.position_xy, speed=speed,
                                  phase=phase, yaw=point.yaw_rad))
    return rows


def test_high_speed_synthetic_pass_satisfies_all_acceptance_gates(synthetic_pass_trace):
    result = assess_high_speed(synthetic_pass_trace)
    assert result['passed'], result['gates']
    assert result['gates']['endpoint_position']
    assert result['gates']['target_speed_past_obstacle']
    assert result['gates']['sustained_target_speed']
    assert result['metrics']['road_departure_samples'] == 0


@pytest.mark.parametrize('offset,accepted', [(-.5, True), (.5, True),
                                            (-.5001, False), (.5001, False)])
def test_high_speed_endpoint_tolerance_is_two_sided(synthetic_pass_trace, offset, accepted):
    rows = deepcopy(synthetic_pass_trace)
    rows[-1]['position_m'][0] = 500+offset
    result = assess_high_speed(rows)
    assert result['gates']['progress']  # The historical lower bound alone permits all four.
    assert result['gates']['endpoint_position'] is accepted
    assert result['passed'] is accepted
    assert all(value for key, value in result['gates'].items() if key != 'endpoint_position')


@pytest.mark.parametrize('center_x,accepted', [(-2.6, True), (552.6, True),
                                              (-2.6001, False), (552.6001, False)])
def test_longitudinal_padded_road_contains_entire_footprint(
        synthetic_pass_trace, center_x, accepted):
    rows = deepcopy(synthetic_pass_trace)
    # Isolate the road gate with one sample; both rejected centers remain on
    # the asphalt, but a 2.4 m chassis half-length extends beyond its end.
    rows[0]['position_m'] = [center_x, 0, 1]
    result = assess_high_speed(rows)
    assert result['gates']['road_footprint'] is accepted
    assert result['metrics']['road_departure_samples'] == (0 if accepted else 1)
    assert result['passed'] is accepted
    assert all(value for key, value in result['gates'].items() if key != 'road_footprint')
    if not accepted:
        historical = assess_high_speed(rows, road_length_m=None)
        assert historical['gates']['road_footprint']  # Legacy omission stays lateral-only.
        assert historical['metrics']['road_departure_samples'] == 0


def curved_braking_trace(horizontal_distance):
    """A deliberately curved synthetic path distinguishes travel from chord."""
    rows = [synthetic_row(tick, (100, 1.8)) for tick in range(1, 361)]
    count = 1000
    for step in range(1, count+1):
        fraction = step/count
        angle = 20*math.pi*fraction
        position = (100+horizontal_distance*fraction, 1.8+math.sin(angle))
        yaw = math.atan2(20*math.pi*math.cos(angle), horizontal_distance)
        speed = max(.1, SPEED*(1-fraction)) if step < count else 0
        rows.append(synthetic_row(360+step, position, speed=speed,
                                  safety='obstacle', yaw=yaw))
    final = rows[-1]
    for tick in range(1361, 1961):
        rows.append(synthetic_row(tick, final['position_m'][:2], speed=0,
                                  phase='hold', safety='obstacle', yaw=final['yaw_rad']))
    return rows


@pytest.mark.parametrize('horizontal_distance,accepted', [(60, True), (80, False)])
def test_braking_acceptance_uses_traveled_curve_not_endpoint_chord(horizontal_distance, accepted):
    rows = curved_braking_trace(horizontal_distance)
    result = assess_high_speed(rows, mode='blocked')
    metrics = result['metrics']
    assert metrics['brake_start_speed_m_s'] == SPEED
    assert metrics['planning_stop_envelope_m'] == pytest.approx(SPEED**2/3+SPEED*.4+1)
    assert horizontal_distance < metrics['planning_stop_envelope_m']
    assert metrics['braking_distance_m'] > horizontal_distance+10
    assert result['gates']['braking_within_planning_envelope'] is accepted
    assert result['passed'] is accepted
    assert all(value for key, value in result['gates'].items()
               if key != 'braking_within_planning_envelope')


def test_lane_validator_accepts_historical_and_explicit_low_speed_profile():
    config = json.loads((ROOT/'experiments/physics-lane-following.json').read_text())
    assert 'speed_profile' not in config['follower']
    assert validate_lane_config(config) is config
    explicit = deepcopy(config)
    explicit['follower']['speed_profile'] = 'low-speed'
    assert validate_lane_config(explicit) is explicit


@pytest.mark.parametrize('maximum', [3, SPEED, 16])
def test_lane_validator_rejects_high_profile_even_with_legacy_speed_limit(maximum):
    config = json.loads((ROOT/'experiments/physics-lane-following.json').read_text())
    config['follower'].update(speed_profile='35mph', max_speed_m_s=maximum)
    with pytest.raises(ValueError, match='bounded physical vehicle'):
        validate_lane_config(config)


def legacy_visual_lidar_config():
    config = json.loads((ROOT/'experiments/physics-visual-lidar.json').read_text())
    # Match the real launcher: follower comes from historical checked-in JSON,
    # rather than regenerating it with the current FollowerConfig defaults.
    follower = json.loads((ROOT/'experiments/physics-lane-following.json').read_text())['follower']
    braking = controller_settings('low-speed')['braking']
    braking.pop('speed_profile')
    assert 'speed_profile' not in follower
    config.update(mode='obstacle', gui=False, points=False, paced=False, capture=False,
                  camera='follow', dropout_at_s=12, braking=braking, follower=follower)
    return config


@pytest.mark.parametrize('explicit_profiles', [(), ('braking',), ('follower',),
                                                ('braking', 'follower')])
def test_visual_lidar_preserves_historical_and_explicit_low_speed_settings(explicit_profiles):
    config = legacy_visual_lidar_config()
    for section in explicit_profiles:
        config[section]['speed_profile'] = 'low-speed'
    before = deepcopy(config)
    assert validate_visual_lidar_config(config) is config
    assert config == before  # Validation must not rewrite historical evidence.


@pytest.mark.parametrize('section', ['braking', 'follower'])
@pytest.mark.parametrize('maximum', [3, SPEED, 16])
def test_visual_lidar_rejects_high_profile_even_at_old_speed(section, maximum):
    config = legacy_visual_lidar_config()
    config[section].update(speed_profile='35mph', max_speed_m_s=maximum)
    with pytest.raises(ValueError, match='low-speed profile'):
        validate_visual_lidar_config(config)


@pytest.mark.parametrize('section,key', [('braking', 'max_points'),
                                        ('follower', 'speed_deadband_m_s')])
def test_visual_lidar_legacy_compatibility_does_not_omit_other_settings(section, key):
    config = legacy_visual_lidar_config()
    config[section].pop(key)
    with pytest.raises(ValueError, match='Every .* setting must be resolved'):
        validate_visual_lidar_config(config)


def test_legacy_lane_source_capture_includes_shared_speed_profile_dependency():
    from experiments.verify_vehicle_stage import source_files

    sources = source_files('lane-following')
    assert 'traffic/path_following.py' in sources
    assert 'traffic/speed_profiles.py' in sources
    assert sources.count('traffic/speed_profiles.py') == 1
