"""Synthetic saved-trace assessment; no native physics or training is launched."""
from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path

import pytest

from environments.lane_following_contract import LaneFollowingTask, make_reset_spec
from experiments.lane_following_lab_validation import assess_episode, compare_episode_traces
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import load_routes
from usd.physical_scene import PATHS

ROOT = Path(__file__).resolve().parents[1]
ROUTES = load_routes(ROOT/'scenarios/physics-road/routes.json')
ACCEPTANCE = json.loads((ROOT/'experiments/lab-lane-following-validation.json').read_text(encoding='utf-8'))['acceptance']
BRAKE = AppliedControl(0., 0., 1., 'accepted', 0, False)
COAST = AppliedControl(0., 0., 0., 'accepted', 1, False)


def physics(route, station=0., speed=0., lateral=0., side_speed=0.):
    p = route.evaluate(station)
    yaw = p.yaw_rad
    vx, vy = speed*math.cos(yaw)-side_speed*math.sin(yaw), speed*math.sin(yaw)+side_speed*math.cos(yaw)
    return dict(position_m=[p.position_xy[0]-lateral*math.sin(yaw), p.position_xy[1]+lateral*math.cos(yaw), .8],
                quaternion_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                velocity_m_s=[vx, vy, 0.], speed_m_s=math.hypot(vx, vy), yaw_rad=yaw,
                upright_z=1., wheel_on_ground=[True]*4)


def reset_record(route, *, serial=1):
    initial = physics(route)
    obs = LaneFollowingTask(route).initialize(initial, BRAKE)
    return dict(reset_spec=asdict(make_reset_spec(route, seed=101, episode_index=serial-1)),
                episode_serial=serial, episode_tick=0, reset_count=serial, physics_state=initial,
                observation=list(obs), wheel_rotation_speed_rad_s=[0.]*4)


def contact(tick):
    return dict(tick=tick, stage_id=1, event_type='found', actor0=PATHS.chassis,
                actor1=PATHS.barrier, collider0=PATHS.chassis+'/Collider',
                collider1=PATHS.barrier+'/Collider', contact_count=1,
                contacts=[dict(position=[1., 0., .5], normal=[-1., 0., 0.], impulse=[0., 0., 0.], separation_m=.01)])


def fixture(reason='success', route_id='straight100'):
    """Known numerical path, not a claim that these controls generate this motion."""
    route = ROUTES[route_id]
    reset = reset_record(route)
    task = LaneFollowingTask(route)
    task.initialize(reset['physics_state'], BRAKE)
    records, samples, events, first_failure = [], [], [], None
    for tick in range(1, 4201):
        if reason == 'time_limit':
            state = physics(route)
        elif reason == 'lane_departure':
            lateral = max(0, tick-240)*.02
            state = physics(route, lateral=lateral, side_speed=2.4 if tick > 240 else 0)
        else:
            station = min(59.8, max(0, tick-240)*.025)
            state = physics(route, station, 3. if 240 < tick <= 2632 else 0.)
        applied = BRAKE if tick <= 240 or tick > 2632 else COAST
        current_contacts = [contact(tick)] if reason == 'rigid_contact' and tick == 300 else []
        guard = task.physics_guard(state, tick=tick, contact_count=len(current_contacts))
        if guard and first_failure is None:
            first_failure = dict(tick=tick, reason=guard, physics_state=deepcopy(state))
        samples.append(dict(tick=tick, physics_state=deepcopy(state), contacts=current_contacts,
                            contact_count=len(current_contacts), unsupported_ticks=0, guard_reason=guard))
        events.extend(current_contacts)
        if tick % 2:
            continue
        transition = task.advance(state, applied, tick=tick, contact_count=len(events),
                                  failure_reason=None if first_failure is None else first_failure['reason'])
        row = transition.as_dict()
        row.update(episode_serial=1, episode_tick=tick, physics_state=deepcopy(state), control=asdict(applied),
                   requested_action=[applied.steering_rad, applied.throttle, applied.brake],
                   physics_samples=samples, contacts=events, first_guard_failure=deepcopy(first_failure),
                   returned_observation=list(transition.observation), returned_reward=transition.reward,
                   returned_terminated=transition.terminated, returned_truncated=transition.truncated,
                   returned_done=transition.terminated or transition.truncated,
                   sb3_time_limit_truncated=transition.truncated and not transition.terminated,
                   final_observation=list(transition.observation))
        records.append(row)
        samples, events = [], []
        if transition.terminated or transition.truncated:
            row['auto_reset'] = reset_record(route, serial=2)
            row['returned_observation'] = row['auto_reset']['observation'][:]
            row['sb3_terminal_observation'] = list(transition.observation)
            break
    return records, reset, route


@pytest.fixture(scope='module')
def successful():
    return fixture()


@pytest.mark.parametrize('reason', ['success', 'time_limit', 'lane_departure', 'rigid_contact'])
def test_supported_success_and_fault_episodes_require_their_actual_mechanism(reason):
    records, reset, route = fixture(reason)
    report = assess_episode(records, reset, route, ACCEPTANCE, expected_reason=reason)
    assert report['passed'], {k: v for k, v in report['checks'].items() if not v}
    assert report['metrics']['physics_samples'] == records[-1]['tick']
    json.dumps(report, allow_nan=False)
    if reason != 'success':
        assert 'forward_progress' not in report['checks']


@pytest.mark.parametrize('route_id', ['left_r60', 'right_r60'])
def test_curved_route_geometry_and_independent_rotated_footprint_agree(route_id):
    records, reset, route = fixture(route_id=route_id)
    report = assess_episode(records, reset, route, ACCEPTANCE)
    assert report['passed'], {k: v for k, v in report['checks'].items() if not v}
    assert report['metrics']['footprint_lateral_bound_max_m'] > .9


@pytest.mark.parametrize('mutation,check', [
    (lambda rows: rows[10]['physics_samples'].pop(), 'physics_tick_order'),
    (lambda rows: rows[10]['physics_samples'].reverse(), 'physics_tick_order'),
    (lambda rows: rows.pop(10), 'control_tick_order'),
    (lambda rows: rows[10].update(tick=24, episode_tick=24), 'control_tick_order'),
    (lambda rows: rows[10]['physics_state']['position_m'].__setitem__(0, .1), 'native_control_state_agreement'),
    (lambda rows: rows[10].update(episode_serial=2), 'episode_identity'),
    (lambda rows: rows[10]['physics_samples'][0]['physics_state']['position_m'].__setitem__(0, math.nan), 'finite_telemetry'),
    (lambda rows: rows[10]['physics_samples'][0]['physics_state'].update(wheel_on_ground=[1]*4), 'record_schema'),
    (lambda rows: rows[10]['physics_samples'][0]['physics_state'].update(upright_z=.99), 'record_schema'),
])
def test_missing_reordered_inconsistent_and_invalid_raw_samples_reject(successful, mutation, check):
    rows, reset, route = deepcopy(successful)
    mutation(rows)
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks'][check] is False


@pytest.mark.parametrize('mutation,check', [
    (lambda rows: rows[150].update(observation=[*rows[150]['observation'][:2], .3, *rows[150]['observation'][3:]]), 'observations_recomputed'),
    (lambda rows: rows[150]['route_state'].update(progress_m=30), 'route_state_recomputed'),
    (lambda rows: rows[150]['reward_components'].update(progress=1), 'reward_components_recomputed'),
    (lambda rows: rows[150].update(returned_reward=2), 'reward_components_recomputed'),
    (lambda rows: rows[150].update(returned_terminated=True), 'returned_flags'),
    (lambda rows: rows[150].update(returned_observation=[0.]*22), 'nonterminal_returned_observations'),
    (lambda rows: rows[-1].update(final_observation=rows[-1]['auto_reset']['observation'][:]), 'observations_recomputed'),
    (lambda rows: rows[-1].update(sb3_terminal_observation=rows[-1]['auto_reset']['observation'][:]), 'sb3_terminal_observation'),
    (lambda rows: rows[-1].pop('sb3_terminal_observation'), 'sb3_terminal_observation'),
    (lambda rows: rows[-1].update(returned_done=False), 'returned_done'),
    (lambda rows: rows[-1].update(sb3_time_limit_truncated=True), 'sb3_time_limit'),
    (lambda rows: rows[-1].pop('returned_done'), 'record_schema'),
    (lambda rows: rows[-1].pop('sb3_time_limit_truncated'), 'record_schema'),
    (lambda rows: rows[-1].update(returned_observation=list(rows[-1]['observation'])), 'terminal_return_is_reset'),
    (lambda rows: rows[-1].update(terminal_observation=None), 'terminal_snapshot'),
    (lambda rows: rows[-1]['auto_reset'].update(episode_serial=1), 'auto_reset_identity'),
    (lambda rows: rows[-1].pop('auto_reset'), 'auto_reset_schema'),
])
def test_reported_values_and_terminal_snapshots_cannot_override_native_evidence(successful, mutation, check):
    rows, reset, route = deepcopy(successful)
    mutation(rows)
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks'][check] is False


@pytest.mark.parametrize('mutation,check', [
    (lambda record: record['physics_state']['position_m'].__setitem__(0, .01), 'reset_native_position'),
    (lambda record: record['reset_spec'].update(x_m=1), 'reset_declared_pose'),
    (lambda record: record['reset_spec'].update(lateral_offset_m=.16), 'reset_bounded_spec'),
    (lambda record: record['wheel_rotation_speed_rad_s'].__setitem__(0, .1), 'reset_wheels_at_rest'),
    (lambda record: record['physics_state']['velocity_m_s'].__setitem__(2, .1), 'reset_native_velocity'),
    (lambda record: record.update(episode_tick=1), 'reset_at_tick_zero'),
])
def test_reset_pose_and_velocity_are_checked_from_native_record(successful, mutation, check):
    rows, reset, route = deepcopy(successful)
    mutation(reset)
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks'][check] is False


def test_physics_only_hold_violation_is_not_hidden_by_control_rate_success(successful):
    rows, reset, route = deepcopy(successful)
    # A missed odd-tick speed spike is below all ordinary guard thresholds, but
    # invalidates the independent120Hz stopped hold while leaving60Hz obs intact.
    sample = rows[-20]['physics_samples'][0]['physics_state']
    sample.update(velocity_m_s=[.1, 0., 0.], speed_m_s=.1)
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks']['stopped_hold'] is False
    assert report['checks']['terminal_decisions_recomputed']


def test_hold_includes_initial_boundary_of_full_one_second(successful):
    rows, reset, route = deepcopy(successful)
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert report['metrics']['hold_samples'] == 121
    assert report['metrics']['hold_duration_s'] == 1
    # The121st-from-last sample is outside a120sample shortcut, but inside the
    # actual continuous second. Changing it must invalidate stopped hold.
    sample = rows[-61]['physics_samples'][-1]['physics_state']
    sample.update(velocity_m_s=[.1, 0., 0.], speed_m_s=.1)
    rows[-61]['physics_state'] = deepcopy(sample)
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks']['stopped_hold'] is False


def test_physics_only_departure_and_support_gap_cannot_be_hidden_by_safe_even_tick(successful):
    rows, reset, route = deepcopy(successful)
    rows[200]['physics_samples'][0]['physics_state']['position_m'][1] = 1
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks']['footprint_containment'] is False
    assert report['checks']['guard_evidence'] is False
    rows, reset, route = deepcopy(successful)
    for row in rows[200:207]:
        for sample in row['physics_samples']:
            sample['physics_state']['wheel_on_ground'][-1] = False
        row['physics_state']['wheel_on_ground'][-1] = False
    report = assess_episode(rows, reset, route, ACCEPTANCE)
    assert not report['passed'] and report['checks']['unsupported_gap'] is False


def test_expected_failure_cannot_pass_only_by_renaming_final_reason(successful):
    rows, reset, route = deepcopy(successful)
    rows[-1]['reason'] = 'lane_departure'
    rows[-1]['success'] = False
    report = assess_episode(rows, reset, route, ACCEPTANCE, expected_reason='lane_departure')
    assert not report['passed'] and report['checks']['expected_physical_mechanism'] is False


def test_early_timeout_and_success_flag_at_start_do_not_pass():
    rows, reset, route = fixture('time_limit')
    rows = rows[:120]
    rows[-1].update(reason='time_limit', truncated=True, returned_truncated=True)
    report = assess_episode(rows, reset, route, ACCEPTANCE, expected_reason='time_limit')
    assert not report['passed'] and report['checks']['exact_timeout'] is False
    rows[-1].update(reason='success', success=True, terminated=True, truncated=False,
                    returned_terminated=True, returned_truncated=False)
    assert not assess_episode(rows, reset, route, ACCEPTANCE)['passed']


def test_deliberate_contact_requires_decoded_nonempty_points():
    rows, reset, route = fixture('rigid_contact')
    for event in rows[-1]['contacts']:
        event.update(contact_count=0, contacts=[])
    for sample in rows[-1]['physics_samples']:
        for event in sample['contacts']:
            event.update(contact_count=0, contacts=[])
    report = assess_episode(rows, reset, route, ACCEPTANCE, expected_reason='rigid_contact')
    assert not report['passed'] and report['checks']['decoded_nonempty_contact'] is False


@pytest.mark.parametrize('replacement,check', [
    ('foreign_vehicle', 'record_schema'), ('foreign_barrier', 'chassis_barrier_pair')])
def test_contact_control_requires_composed_chassis_and_barrier_identities(replacement, check):
    rows, reset, route = fixture('rigid_contact')
    for group in [rows[-1]['contacts'], *(sample['contacts'] for sample in rows[-1]['physics_samples'])]:
        for event in group:
            if replacement == 'foreign_vehicle':
                event.update(actor0='/World/Vehicles/Other', collider0='/World/Vehicles/Other/Collider')
            else:
                event.update(actor1='/World/Environment/Other', collider1='/World/Environment/Other/Collider')
    report = assess_episode(rows, reset, route, ACCEPTANCE, expected_reason='rigid_contact')
    assert not report['passed'] and report['checks'][check] is False


@pytest.mark.parametrize('value', [[], None, {}, [None]])
def test_empty_or_malformed_records_fail_with_json_report(value):
    result = assess_episode(value, reset_record(ROUTES['straight100']), ROUTES['straight100'], ACCEPTANCE)
    assert result['passed'] is False
    json.dumps(result, allow_nan=False)


def test_repeat_compares_every_physics_tick_and_ignores_episode_serial(successful):
    rows, _, _ = successful
    repeated = deepcopy(rows)
    for row in repeated:
        row['episode_serial'] = 200
    report = compare_episode_traces(rows, repeated, ACCEPTANCE)
    assert report['passed'] and report['metrics']['maximum_position_difference_m'] == 0
    repeated[200]['physics_samples'][0]['physics_state']['position_m'][1] += .002
    report = compare_episode_traces(rows, repeated, ACCEPTANCE)
    assert not report['passed'] and report['checks']['position_repeatability'] is False


@pytest.mark.parametrize('mutation', [
    lambda rows: rows.pop(), lambda rows: rows[20]['physics_samples'].pop(),
    lambda rows: rows[20]['physics_samples'].reverse(),
    lambda rows: rows[-1].update(reason='time_limit', success=False),
    lambda rows: rows[20]['physics_samples'][0]['physics_state']['velocity_m_s'].__setitem__(0, math.inf),
])
def test_repeat_rejects_missing_or_changed_terminal_and_native_data(successful, mutation):
    rows, _, _ = successful
    changed = deepcopy(rows)
    mutation(changed)
    assert not compare_episode_traces(rows, changed, ACCEPTANCE)['passed']


def test_identical_nonterminal_prefixes_and_reset_crossing_reject(successful):
    rows, _, _ = successful
    report = compare_episode_traces(rows[:20], rows[:20], ACCEPTANCE)
    assert not report['passed'] and report['checks']['complete_terminal_episodes'] is False
    changed = deepcopy(rows)
    changed[20]['terminated'] = True
    report = compare_episode_traces(changed, changed, ACCEPTANCE)
    assert not report['passed'] and report['checks']['complete_terminal_episodes'] is False
