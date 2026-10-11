"""CPU contracts and geometry only; runtime PhysX qualification is separate."""
from dataclasses import replace
import math
import random

import pytest

from traffic.driver_control import DriverControlGate
from traffic.loop_showcase_control import (
    ObjectTrack, RadialPath, ShowcaseConfig, ShowcaseController,
    circular_station, signed_station_gap, track_prediction,
)
from traffic.path_following import FRAME, SOURCE, VehicleState


def state(tick=0, station=0., lane=0, speed=15., **changes):
    point = RadialPath(lane, lane, 0, 80).pose(station)
    return replace(VehicleState('episode', 'ego', tick, *point, speed), **changes)


def track(tick=0, station=80., lane=0, speed=7., identity='slow_01', **changes):
    point = RadialPath(lane, lane, 0, 80).pose(station)
    return replace(ObjectTrack('episode', identity, tick, *point, speed), **changes)


@pytest.mark.parametrize('from_lane,to_lane', [(0, 1), (1, 0), (2, 3), (3, 2), (0, 0)])
def test_radial_path_is_smooth_and_stays_between_lane_centres(from_lane, to_lane):
    p = RadialPath(from_lane, to_lane, 100, 80)
    start = 500 + from_lane * 3.7
    end = 500 + to_lane * 3.7
    assert p.radius_and_derivative(99) == (start, 0)
    assert p.radius_and_derivative(100) == (start, 0)
    assert p.radius_and_derivative(180) == (end, 0)
    assert p.radius_and_derivative(190) == (end, 0)
    for i in range(81):
        radius, _ = p.radius_and_derivative(100 + i)
        assert min(start, end) <= radius <= max(start, end)
    assert p.radius_and_derivative(140)[0] == pytest.approx((start + end) / 2)
    for endpoint in (100, 180):
        before, after = p.pose(endpoint - 1e-5), p.pose(endpoint + 1e-5)
        assert math.dist(before[:2], after[:2]) < 3e-5
        assert abs(before[2] - after[2]) < 1e-6


def test_station_wrap_and_path_geometry_remain_continuous_across_seam():
    length = 500 * math.tau
    assert signed_station_gap(length - 2, 3) == pytest.approx(5)
    assert signed_station_gap(3, length - 2) == pytest.approx(-5)
    p = RadialPath(0, 1, length - 40, 80)
    assert math.dist(p.pose(length - .01)[:2], p.pose(length + .01)[:2]) < .021


def test_exact_background_prediction_uses_its_lane_radius_not_base_radius():
    obj = track(lane=3, station=500 * math.tau - 5)
    predicted = track_prediction(obj, 10)
    initial_angle = math.atan2(obj.y_m, obj.x_m)
    angle = initial_angle + 70 / 511.1
    assert (predicted.x_m, predicted.y_m) == pytest.approx((511.1 * math.cos(angle), 511.1 * math.sin(angle)))


@pytest.mark.parametrize('changes', [
    dict(radius_m=503), dict(lane_width_m=4), dict(lane_count=3), dict(physics_hz=60),
    dict(target_speed_m_s=20), dict(target_speed_m_s=math.nan), dict(lane_change_length_m=20),
    dict(planning_interval_ticks=24), dict(command_ttl_ticks=24), dict(prediction_step_s=.3),
    dict(ego_width_m=3.5), dict(rear_axle_offset_m=4), dict(max_steering_rad=.6),
])
def test_configuration_guards(changes):
    with pytest.raises(ValueError):
        ShowcaseConfig(**changes)


@pytest.mark.parametrize('args', [(-1, 0, 0, 80), (0, 2, 0, 80), (0, 1, math.nan, 80), (0, 1, 0, 0)])
def test_invalid_paths_are_rejected(args):
    with pytest.raises(ValueError):
        RadialPath(*args)


def test_empty_road_cruise_returns_bounded_command_with_correct_identity_and_ttl():
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(speed=3), 0, [])
    assert cmd.throttle > 0 and cmd.brake == 0
    assert 0 < cmd.steering_rad < .02
    assert cmd.expires_tick == 12
    assert cmd.episode_id == 'episode' and cmd.vehicle_id == 'ego'
    assert not control.diagnostics['is_fallback']
    assert control.diagnostics['source'] == SOURCE
    gate = DriverControlGate('episode', 'ego')
    assert not gate.step(tick=0, dt_s=1 / 120, command=cmd).is_fallback


@pytest.mark.parametrize('stop,delta,valid',[(True,-.055,True),(False,-.055,False),
                                         (True,.1,False),(True,-.2,False)])
@pytest.mark.parametrize('optimized', [False, True])
def test_bounded_background_braking_is_allowed_only_during_requested_stop(stop,delta,valid,optimized):
    control=ShowcaseController('episode','ego',optimized=optimized)
    control.command(state(speed=0),0,[track(station=200,lane=3)])
    control.command(state(tick=2,speed=0),2,[track(tick=2,station=200+7*2/120*500/511.1,
        lane=3,speed=7+delta)],stop=stop)
    assert (control.diagnostics['reason']!='object_motion_contract_changed') is valid


def test_slower_lead_causes_adjacent_lane_change_and_path_overlay_adapts():
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [track()])
    assert control.diagnostics['maneuver_active']
    assert control.diagnostics['target_lane'] == 1
    assert control.diagnostics['planner_reason'] == 'safe_adjacent_lane_selected'
    points = control.path_points(state())
    assert len(points) == 61
    assert math.hypot(*points[0][:2]) == pytest.approx(500)
    assert math.hypot(*points[-1][:2]) == pytest.approx(503.7)
    assert all(p[2] == .06 for p in points)


def test_nearby_target_lane_vehicle_blocks_change_without_forcing_weave():
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [track(station=30), track(lane=1, station=5, identity='target')])
    assert not control.diagnostics['maneuver_active']
    assert control.diagnostics['target_lane'] == 0
    assert control.diagnostics['target_speed_m_s'] < 15
    assert control.diagnostics['planner_reason'] in ('blocked_following', 'unsafe_closing_gap_brake')


def test_fast_target_lane_rear_vehicle_blocks_change():
    control = ShowcaseController('episode', 'ego')
    control.command(state(speed=10), 0, [track(station=80),
        track(lane=1, station=-30, speed=20, identity='fast_rear')])
    assert not control.diagnostics['maneuver_active']


def test_picks_safe_side_with_clearer_downstream_traffic():
    control = ShowcaseController('episode', 'ego', initial_lane=1)
    control.command(state(lane=1), 0, [track(lane=1), track(lane=0, station=5, identity='left_block')])
    assert control.diagnostics['target_lane'] == 2


def test_no_need_to_change_lane_for_distant_or_equal_speed_vehicle():
    for obj in (track(station=200), track(station=80, speed=15.6464)):
        control = ShowcaseController('episode', 'ego')
        control.command(state(), 0, [obj])
        assert not control.diagnostics['maneuver_active']


@pytest.mark.parametrize('changes', [
    dict(tick=1), dict(tick=-1), dict(episode_id='other'), dict(vehicle_id='ego'),
    dict(vehicle_id=''), dict(x_m=math.nan), dict(frame='wrong'), dict(source='lidar'),
    dict(speed_m_s=-1), dict(speed_m_s=30), dict(length_m=0), dict(width_m=5),
    dict(yaw_rad=0), dict(x_m=501),
])
def test_bad_object_frame_latches_brake_and_does_not_recover_silently(changes):
    control = ShowcaseController('episode', 'ego')
    obj = track(station=0, lane=1)
    cmd = control.command(state(), 0, [replace(obj, **changes)])
    assert cmd.throttle == 0 and cmd.brake == 1
    assert control.diagnostics['is_fallback']
    cmd = control.command(state(tick=2, station=.25), 2, [replace(obj, tick=2)])
    assert cmd.throttle == 0 and cmd.brake == 1


@pytest.mark.parametrize('tracks', [None, {}, 'missing'])
def test_missing_frame_is_not_an_empty_road(tracks):
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(), 0, tracks)
    assert cmd.brake == 1 and control.diagnostics['is_fallback']


def test_missing_id_duplicate_id_or_unexpected_new_id_fails_safe():
    for after in ([], [track(2), track(2)], [track(2), track(2, identity='new')]):
        control = ShowcaseController('episode', 'ego')
        control.command(state(speed=0), 0, [track()])
        cmd = control.command(state(tick=2, speed=0), 2, after)
        assert cmd.brake == 1 and control.diagnostics['is_fallback']


def test_too_close_object_triggers_emergency_brake():
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(), 0, [track(station=5)])
    assert cmd.brake == 1 and cmd.throttle == 0
    assert control.diagnostics['reason'] == 'object_clearance_emergency'


def test_requested_stop_is_latched_without_claiming_error_and_retains_steering():
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(), 0, [], stop=True)
    assert cmd.throttle == 0 and cmd.brake == 1 and cmd.steering_rad > 0
    assert not control.diagnostics['is_fallback']
    cmd = control.command(state(2, station=.25), 2, [], stop=False)
    assert cmd.brake == 1 and control.diagnostics['reason'] == 'requested_stop'


def test_clock_and_state_guardrails():
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [])
    with pytest.raises(ValueError):
        control.command(state(), 0, [])
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [])
    cmd = control.command(state(2, station=10), 2, [])
    assert cmd.brake == 1 and control.diagnostics['reason'] == 'implausible_or_missing_ego_progress'


def test_controller_tracks_reference_across_seam_and_counts_completed_change_once():
    control = ShowcaseController('episode', 'ego')
    start = 500 * math.tau - 30
    speed = 15.
    for tick in range(0, 800, 2):
        station = start + speed * tick / 120
        if tick == 0:
            ego = state(tick, station, speed=speed)
        else:
            pose = control.path.pose(station)
            ego = VehicleState('episode', 'ego', tick, *pose, speed)
        obj = track(tick, station=start + 80 + 7 * tick / 120)
        cmd = control.command(ego, tick, [obj])
        assert not control.diagnostics['is_fallback'], control.diagnostics
        assert math.isfinite(cmd.steering_rad)
    assert control.diagnostics['lane_changes'] == 1
    assert control.diagnostics['target_lane'] == 1
    assert control.diagnostics['seam_crossings'] == 1


def test_pass_counter_needs_initially_ahead_and_then_fully_behind():
    control = ShowcaseController('episode', 'ego', initial_lane=1)
    for tick in range(0, 700, 2):
        station = 15 * tick / 120
        obj = track(tick, station=30 + 7 * tick / 120)
        control.command(state(tick, station=station, lane=1), tick, [obj])
    assert control.diagnostics['passes'] == 1
    assert control.diagnostics['passed_ids'] == ['slow_01']


def test_deterministic_controller_repeats_identical_commands():
    def run():
        c = ShowcaseController('episode', 'ego')
        result = []
        for tick in range(0, 120, 2):
            station = 15 * tick / 120
            pose = c.path.pose(station)
            ego = VehicleState('episode', 'ego', tick, *pose, 15)
            result.append(c.command(ego, tick, [track(tick, station=80 + 7 * tick / 120)]))
        return result
    assert run() == run()


def test_station_only_projection_is_exact_at_boundaries_and_random_points():
    from traffic.loop_driving import CircularLoop
    route = CircularLoop()
    points = [(x, y) for x in (-511.1, -500., -0., 0., 500., 511.1)
              for y in (-511.1, -500., -0., 0., 500., 511.1) if math.hypot(x, y) > 1e-9]
    points += [(500., math.nextafter(0., side)) for side in (-math.inf, math.inf)]
    points += [(500., -1e-13), (1e300, -1e300), (math.nextafter(1e-9, math.inf), 0.)]
    rng = random.Random(101)
    points += [(rng.uniform(-600, 600), rng.uniform(-600, 600)) for _ in range(2000)]
    for x, y in points:
        assert circular_station(x, y) == route.project(x, y).s_m % route.length_m
    for point in ((0., 0.), (1e-9, 0.), (math.nan, 500.), (500., math.inf), (True, 500.), ('x', 500.)):
        with pytest.raises(ValueError):
            circular_station(*point)
        with pytest.raises(ValueError):
            route.project(*point)


def test_optimized_flag_requires_bool_and_legacy_is_default():
    assert not ShowcaseController('episode', 'ego').optimized
    for value in (1, 0, None, 'true'):
        with pytest.raises(ValueError):
            ShowcaseController('episode', 'ego', optimized=value)


def test_optimized_candidate_envelopes_match_randomized_and_ulp_boundaries():
    from types import SimpleNamespace
    rng = random.Random(101)
    legacy = ShowcaseController('episode', 'ego')
    optimized = ShowcaseController('episode', 'ego', optimized=True)
    decisions = set()
    for case in range(240):
        station = rng.choice((0., math.tau*500-1e-12, rng.uniform(-4000, 4000)))
        lane = rng.randrange(4)
        other_lane = lane + 1 if lane == 0 else lane - 1 if lane == 3 else lane + rng.choice((-1, 1))
        ego = state(station=station, lane=lane, speed=rng.choice((0., 1., 15.6464, 19., rng.uniform(0, 20))))
        legacy._unwrapped_s = optimized._unwrapped_s = station
        path = RadialPath(lane, other_lane, station + 2., 80.)
        objects = [track(station=station+rng.uniform(-180, 250), lane=rng.randrange(4),
                         speed=rng.uniform(0, 20), identity=f'car_{i}') for i in range(12)]
        # Near the first sample's exact longitudinal/radial envelope edges.
        cfg = legacy.config
        speed = 7.
        longitudinal = ((cfg.ego_length_m+4.8)/2+cfg.prediction_clearance_m
                        +cfg.prediction_tracking_margin_m+(cfg.max_observed_speed_m_s+speed)*cfg.prediction_step_s/2)
        radial_half = (cfg.ego_width_m/2+cfg.ego_length_m/2*(1.875*cfg.lane_width_m/path.length_m)
                       +cfg.prediction_tracking_margin_m+.02)
        radius = 500+lane*3.7 + radial_half+.9+cfg.prediction_clearance_m
        for direction in (-math.inf, math.inf):
            gap = math.nextafter(longitudinal, direction)
            angle = (station+gap)/500
            r = math.nextafter(radius, direction)
            objects.append(ObjectTrack('episode', f'edge_{direction}', 0,
                r*math.cos(angle), r*math.sin(angle), angle+math.pi/2, speed))
        samples = [[], []]
        def recording_path(index):
            def radius(station_m):
                samples[index].append(station_m)
                return path.radius_and_derivative(station_m)
            return SimpleNamespace(end_s_m=path.end_s_m, length_m=path.length_m,
                                   radius_and_derivative=radius)
        baseline = legacy._candidate_safe(recording_path(0), ego, objects)
        assert baseline == optimized._candidate_safe(recording_path(1), ego, objects)
        assert samples[0] == samples[1]  # Same sampled stations and early exit.
        decisions.add(baseline)
        geometry = {obj.vehicle_id: (math.hypot(obj.x_m, obj.y_m),
            signed_station_gap(station, circular_station(obj.x_m, obj.y_m))) for obj in objects}
        assert legacy._candidate_safe(path, ego, objects) == optimized._candidate_safe(path, ego, objects, geometry)
    assert decisions == {False, True}


@pytest.mark.parametrize('fixture', ['passing', 'blocked', 'rundown', 'empty'])
def test_optimized_commands_and_all_diagnostics_match_legacy_each_frame(fixture):
    legacy = ShowcaseController('episode', 'ego', initial_lane=1)
    optimized = ShowcaseController('episode', 'ego', initial_lane=1, optimized=True)
    start = math.tau*500-30
    for tick in range(0, 700, 2):
        station = start + 15*tick/120
        pose = legacy.path.pose(station)
        ego = VehicleState('episode', 'ego', tick, *pose, 15.)
        objects = [] if fixture == 'empty' else [
            track(tick, station=start+80+i*105+7*tick/120, lane=i%4,
                  speed=7*(500+(i%4)*3.7)/500, identity=f'car_{i}') for i in range(80)]
        if fixture == 'blocked':
            objects += [track(tick, station=start+60+7*tick/120, lane=lane,
                              speed=7*(500+lane*3.7)/500, identity=f'block_{lane}') for lane in range(4)]
        stop = fixture == 'rundown' and tick >= 240
        assert legacy.command(ego, tick, objects, stop=stop) == optimized.command(ego, tick, objects, stop=stop)
        assert legacy.diagnostics == optimized.diagnostics
        assert legacy.path == optimized.path
        assert legacy.path_points(ego) == optimized.path_points(ego)


@pytest.mark.parametrize('bad_frame', [None, [], 'missing', 'duplicate', 'stale', 'nan', 'circle', 'heading', 'identity'])
def test_optimized_validation_and_latched_brakes_match_legacy(bad_frame):
    controls = [ShowcaseController('episode', 'ego', optimized=value) for value in (False, True)]
    for control in controls:
        control.command(state(speed=0), 0, [track(station=200)])
    obj = track(2, station=200+7*2/120)
    frames = {'duplicate': [obj, obj], 'stale': [replace(obj, tick=0)],
              'nan': [replace(obj, x_m=math.nan)], 'circle': [replace(obj, x_m=0, y_m=0)],
              'heading': [replace(obj, yaw_rad=0)], 'identity': [replace(obj, vehicle_id='changed')]}
    frame = frames.get(bad_frame, bad_frame) if isinstance(bad_frame, str) else bad_frame
    commands = [control.command(state(2, speed=0), 2, frame) for control in controls]
    assert commands[0] == commands[1]
    assert controls[0].diagnostics == controls[1].diagnostics
