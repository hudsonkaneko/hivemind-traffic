"""Synthetic CPU evidence, not native simulation or proof of driving skill."""
from copy import deepcopy
from dataclasses import asdict
import json

import pytest

from environments.loop_hazard_contract import HazardLaneTask
from experiments.loop_hazard_support import EpisodeAccumulator, summarize_episode, atomic_json
from tests.test_lane_follow_ppo_evaluation import state, ROUTES, BRAKE, COAST
from traffic.road_hazards import make_hazard_schedule
from usd.physical_scene import PATHS


def fixture(stage='clear', ending='success'):
    route = ROUTES['straight100']
    schedule = make_hazard_schedule(stage, 5101, route=route)
    task = HazardLaneTask(route, stage=stage)
    initial = state(route)
    task.set_objects(schedule.sample(route, 0.), 0.)
    first_observation = task.initialize(initial, BRAKE)
    reset = dict(physics_state=deepcopy(initial), episode_serial=1, observation=list(first_observation),
                 hazard_schedule=schedule.as_dict(), hazard_frames=[asdict(o) for o in schedule.sample(route, 0.)])
    goal = task.static_stop_station() if stage == 'static' else 60.
    rows, samples = [], []
    for tick in range(1, 4201):
        t = tick/120
        objects = schedule.sample(route, t)
        frames = [asdict(obj) for obj in objects]
        task.set_objects(objects, t)
        start = 14. if stage == 'crossing' else 2.
        station = min(goal, max(0., t-start)*3.) if ending == 'success' else 0.
        speed = 3. if ending == 'success' and t > start and station < goal else 0.
        native = state(route, station=station, speed=speed)
        applied = COAST if speed else BRAKE
        events = []
        if ending == 'rigid_contact' and tick == 242:
            events = [dict(actor0=PATHS.chassis, actor1='/World/Environment/Props/Hazard_000/Body',
                           contact_count=1)]
        reason = task.physics_guard(native, tick=tick, contact_count=len(events))
        samples.append(dict(tick=tick, physics_state=native, hazard_frames=frames,
                            contacts=events, contact_count=len(events), guard_reason=reason))
        if tick % 2:
            continue
        failure = next((s['guard_reason'] for s in samples if s['guard_reason']), None)
        transition = task.advance(native, applied, tick=tick,
            contact_count=sum(s['contact_count'] for s in samples), failure_reason=failure)
        row = transition.as_dict()
        row.update(episode_serial=1, physics_state=deepcopy(native), hazard_frames=deepcopy(frames),
                   physics_samples=deepcopy(samples), hazard_stage=stage, hazard_schedule=schedule.as_dict())
        rows.append(row)
        samples = []
        if transition.terminated or transition.truncated:
            break
    return rows, reset, route


@pytest.fixture(scope='module')
def clear_success():
    return fixture()


@pytest.fixture(scope='module')
def static_success():
    return fixture('static')


@pytest.fixture(scope='module')
def crossing_success():
    return fixture('crossing')


@pytest.mark.parametrize('which', ['clear_success', 'static_success', 'crossing_success'])
def test_success_semantics_independently_measured(request, which):
    rows, reset, route = request.getfixturevalue(which)
    result = summarize_episode(rows, reset, route=route)
    assert result['evidence_valid'], result['checks']
    assert result['task_success'] and result['driving_success']
    assert result['physics_samples'] == 2*len(rows)
    assert result['rigid_contact_events'] == 0
    assert result['post_settle_support_fraction'] == 1.
    if which == 'static_success':
        assert result['static_stopped_hold_s'] >= 1.
        assert abs(result['final_station_m']-result['static_stop_station_m']) <= .5
    elif which == 'crossing_success':
        assert result['crossing_encountered']
        assert result['first_corridor_clear_after_encounter_s'] is not None
        assert result['post_clear_stopped_seconds'] > 0
    else:
        assert result['progress_m'] >= 60.
        assert result['first_corridor_clear_after_encounter_s'] is None
        assert result['post_clear_stopped_seconds'] == 0.


def test_streaming_matches_batch_and_retains_bounded_state(clear_success):
    rows, reset, route = clear_success
    accumulator = EpisodeAccumulator(reset, route=route)
    for row in rows:
        accumulator.update(row)
    assert accumulator.finish() == summarize_episode(rows, reset, route=route)
    assert len(json.dumps(accumulator.finish())) < 5000
    assert not hasattr(accumulator, 'records')
    with pytest.raises(ValueError, match='terminal'):
        accumulator.update(rows[-1])


def test_valid_timeout_is_not_driving_success():
    rows, reset, route = fixture(ending='time_limit')
    result = summarize_episode(rows, reset, route=route)
    assert result['evidence_valid']
    assert result['terminal_reason'] == 'time_limit'
    assert not result['task_success'] and not result['driving_success']


def test_contact_failure_is_preserved():
    rows, reset, route = fixture(ending='rigid_contact')
    result = summarize_episode(rows, reset, route=route)
    assert result['evidence_valid'], result['checks']
    assert result['physical_failure'] and not result['driving_success']
    assert result['rigid_contact_events'] == result['contact_points'] == 1
    assert result['first_physical_failure']['tick'] == 242


def test_partial_episode_never_claims_completion(clear_success):
    rows, reset, route = clear_success
    result = summarize_episode(rows[:30], reset, route=route, complete=False)
    assert result['evidence_valid'] and not result['complete']
    assert not result['driving_success']
    assert not summarize_episode(rows[:30], reset, route=route)['evidence_valid']


def test_missing_route_never_qualifies_driving_success(clear_success):
    rows, reset, _ = clear_success
    result = summarize_episode(rows, reset)
    assert result['evidence_valid'] and result['task_success']
    assert not result['geometry_qualified'] and not result['driving_success']


@pytest.mark.parametrize('mutate', [
    lambda r: r[0].update(tick=4),
    lambda r: r[1].update(episode_serial=2),
    lambda r: r[0].update(observation=[0.]*22),
    lambda r: r[0]['observation'].__setitem__(0, float('nan')),
    lambda r: r[0]['physics_samples'].pop(),
    lambda r: r[0]['physics_samples'][0].update(tick=2),
    lambda r: r[0].update(hazard_stage='static'),
    lambda r: r[0].update(success=True),
    lambda r: r[0].update(terminated=True, truncated=True),
    lambda r: r[0]['physics_samples'][0].pop('hazard_frames'),
    lambda r: r[0]['physics_samples'][0].update(contact_count=1),
    lambda r: r[0]['physics_state'].update(speed_m_s=99.),
    lambda r: r[0]['hazard_schedule'].update(seed=9),
])
def test_malformed_or_inconsistent_rows_fail_closed(clear_success, mutate):
    rows, reset, route = deepcopy(clear_success)
    # asdict preserves tuples: normalize to the actual saved-JSON representation.
    rows = json.loads(json.dumps(rows[:2]))
    mutate(rows)
    with pytest.raises(ValueError):
        summarize_episode(rows, reset, route=route, complete=False)


def test_reward_arithmetic_and_terminal_snapshot_are_checked(clear_success):
    rows, reset, route = deepcopy(clear_success)
    rows[-1]['reward'] += 1.
    rows[-1]['terminal_observation'] = [0.]*42
    result = summarize_episode(rows, reset, route=route)
    assert not result['evidence_valid']
    assert not result['checks']['reward_component_sum']
    assert not result['checks']['pre_reset_terminal_snapshot']


def test_measured_object_observation_is_not_trusted(static_success):
    rows, reset, route = deepcopy(static_success)
    rows[0]['observation'] = list(rows[0]['observation'])
    rows[0]['observation'][23] += 1.
    result = summarize_episode(rows, reset, route=route)
    assert not result['evidence_valid']
    assert not result['checks']['object_observations_match']


def test_stale_frame_fails_closed(static_success):
    rows, reset, route = deepcopy(static_success)
    rows[0]['physics_samples'][0]['hazard_frames'][0]['timestamp_s'] = 0.
    with pytest.raises(ValueError, match='stale'):
        summarize_episode(rows[:1], reset, route=route, complete=False)


def test_atomic_json_rejects_nonfinite_without_replacing(tmp_path):
    output = tmp_path/'result.json'
    atomic_json(output, {'passed': False})
    before = output.read_bytes()
    with pytest.raises(ValueError):
        atomic_json(output, {'value': float('inf')})
    assert output.read_bytes() == before
