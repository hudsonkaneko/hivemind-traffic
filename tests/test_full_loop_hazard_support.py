"""Independent full-road evidence rejection and rolling-window replay; CPU only."""
from copy import deepcopy
from dataclasses import asdict
import json
import math

import pytest

from environments.continuous_loop_hazard_contract import ContinuousFullLoopHazardTask
from environments.full_loop_hazard_contract import FullLoopHazardTask
from environments.lane_following_contract import make_reset_spec, _state
from experiments.full_loop_hazard_support import FullEpisodeAccumulator, float32, reduce_episode, same, _contained, _demonstration_pair
from traffic.bypass_validation import OrientedBox, box_clearance
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.road_hazards import MeasuredObject, make_hazard_schedule
from usd.physical_scene import PATHS

ROAD = LaneRoute('loop_r500', 3.6, (RouteSegment(100., .002),), (500., 0.), math.pi/2)
LONG = LaneRoute('loop_r500', 3.6, (RouteSegment(300., .002),), (500., 0.), math.pi/2)
COAST = AppliedControl(0., 0., 0., 'held', 1, False)


def fixture(*, continuous=False, stage='clear', reset_count=1, serial=1, global_tick=0, seed=61001, contact=False):
    task = ContinuousFullLoopHazardTask(ROAD, stage=stage) if continuous else FullLoopHazardTask(ROAD, stage=stage)
    schedule = make_hazard_schedule(stage, seed, route=ROAD)
    spec = make_reset_spec(ROAD, seed=seed, perturb=True)

    def state(tick):
        progress = max(0, tick-240)*3/120
        p = LONG.evaluate(progress)
        c, s = math.cos(p.yaw_rad), math.sin(p.yaw_rad)
        yaw = p.yaw_rad+spec.yaw_error_rad
        speed = 0. if tick <= 240 else 3.
        return dict(position_m=[p.position_xy[0]-s*spec.lateral_offset_m, p.position_xy[1]+c*spec.lateral_offset_m, 1.],
            velocity_m_s=[c*speed, s*speed, 0.], speed_m_s=speed,
            quaternion_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)], yaw_rad=yaw, upright_z=1., wheel_on_ground=[True]*4)

    def objects(tick):
        hazards = schedule.sample(ROAD, tick/120)
        backgrounds = tuple(MeasuredObject(f'background_{i:03d}_g001', 5000.+i*20, 50., 0., 0., 0., 4.8, 1.9, tick/120) for i in range(12))
        task.set_objects(hazards, now_s=tick/120)
        task.set_background_objects(backgrounds, now_s=tick/120)
        return list(map(asdict, hazards)), list(map(asdict, backgrounds))

    hazards, backgrounds = objects(0)
    observation = task.initialize(state(0), AppliedControl(0., 0., 1., 'reset', None, False))
    reset = dict(reset_spec=asdict(spec), reset_count=reset_count, physical_reset_count=reset_count,
        episode_serial=serial, episode_tick=0, background_global_tick=global_tick,
        physics_state=state(0), observation=list(observation), wheel_rotation_speed_rad_s=[0.]*4,
        hazard_schedule=json.loads(json.dumps(schedule.as_dict())), hazard_frames=hazards, background_frames=backgrounds)

    def rows():
        previous_observation = observation
        for tick in range(2, 12601 if continuous else 4201, 2):
            samples, contacts = [], []
            failure = None
            for native_tick in (tick-1, tick):
                hazards, backgrounds = objects(native_tick)
                raw = state(native_tick)
                events = [dict(actor0=PATHS.chassis, actor1='/World/Environment/Props/Hazard/Body', contact_count=1)] if contact and native_tick == 1 else []
                reason = task.physics_guard(raw, tick=native_tick, contact_count=len(events))
                if reason is not None and failure is None:
                    failure = dict(tick=native_tick, reason=reason, physics_state=deepcopy(raw))
                ego = OrientedBox(*raw['position_m'][:2], raw['yaw_rad'], 4.8, 1.8)
                geometry = [box_clearance(ego, OrientedBox(o['x_m'], o['y_m'], o['yaw_rad'], o['length_m'], o['width_m'])) for o in hazards+backgrounds]
                sample = dict(tick=native_tick, global_tick=global_tick+native_tick, physics_state=raw,
                    contacts=events, contact_count=len(events), guard_reason=reason, unsupported_ticks=0,
                    hazard_frames=hazards, background_frames=backgrounds,
                    all_object_clearance=dict(clearance_m=min(g['clearance_m'] for g in geometry), overlap=any(g['overlap'] for g in geometry), obstacle_count=len(geometry)),
                    native_background=dict(passed=True, count=80, commanded_steps=global_tick+native_tick,
                        target_time_s=(global_tick+native_tick)/120, max_position_error_m=0., max_yaw_error_rad=0., max_speed_error_m_s=0.))
                samples.append(sample); contacts.extend(events)
            tr = task.advance(state(tick), COAST, tick=tick, contact_count=len(contacts), failure_reason=None if failure is None else failure['reason']).as_dict()
            tr.update(episode_serial=serial, episode_tick=tick, physical_reset_count=reset_count, global_physics_tick=global_tick+tick,
                physics_state=state(tick), control=asdict(COAST), requested_action=[0., 0., 0.], contacts=contacts,
                physics_samples=samples, first_guard_failure=failure, hazard_stage=stage,
                hazard_frames=hazards, background_frames=backgrounds, hazard_schedule=deepcopy(reset['hazard_schedule']),
                deployment_observation=deepcopy(task.last_deployment_observation))
            if continuous:
                context = deepcopy(task.last_continuation)
                roll = context['window_roll']
                if roll is not None:
                    context['native_boundary'] = dict(tick=tick, physical_reset_count=1, before=deepcopy(roll['physics_state_before']),
                        after=deepcopy(roll['physics_state_after']), equal=True, hazard_schedule_index=0)
                tr['continuation'] = context
            row = dict(transition=tr, input_observation=float32(previous_observation), returned_observation=float32(tr['observation']),
                final_observation=float32(tr['observation']), returned_reward=float32(tr['reward']),
                returned_terminated=tr['terminated'], returned_truncated=tr['truncated'])
            done = tr['terminated'] or tr['truncated']
            if done and continuous:
                row['autoreset_suppressed'] = dict(episode_tick=tick, physical_reset_count=1, terminal_state_retained=True)
            elif done:
                row['auto_reset'] = dict(reset_count=reset_count+1, episode_serial=serial+1, observation=[0.]*42)
                row['returned_observation'] = [0.]*42
            yield row
            previous_observation = tr['observation']
            if done:
                return
    return reset, rows()


def test_continuous_180m_replays_windows_reward_and_native_state():
    reset, rows = fixture(continuous=True)
    result = reduce_episode(rows, reset, route=ROAD, stage='clear', continuous=True)
    assert result['evidence_valid'] and result['driving_success']
    assert result['completed_windows'] == 3 and len(result['native_window_boundaries']) == 2
    assert 180 <= result['total_progress_m'] < 180.1
    assert result['physics_samples'] > 4200 and result['physical_reset_count_during_episode'] == 1
    assert result['contacts'] == 0


def test_raw_training_interface_partial_and_arbitrary_background_clock():
    reset, rows = fixture(reset_count=8, serial=8, global_tick=12345)
    audit = FullEpisodeAccumulator(reset, route=ROAD, stage='clear')
    for _, row in zip(range(4), rows):
        audit.update(row['transition'])
    result = audit.finish(complete=False)
    assert result['evidence_valid'] and not result['complete'] and not result['driving_success']
    assert result['control_steps'] == 4 and result['rigid_contact_events'] == 0
    with pytest.raises(ValueError, match='completion'):
        audit.finish()


def test_well_recorded_contact_failure_is_valid_not_driving_success():
    reset, rows = fixture(contact=True)
    result = reduce_episode(rows, reset, route=ROAD, stage='clear')
    assert result['evidence_valid'] and result['physical_failure'] and not result['driving_success']
    assert result['terminal_reason'] == 'rigid_contact' and result['contacts'] == 1


@pytest.mark.parametrize('mutation,match', [
    (lambda r: r['transition'].__setitem__('physical_reset_count', 2), 'reset'),
    (lambda r: r['transition']['physics_samples'][0].__setitem__('global_tick', 20), 'ticks'),
    (lambda r: r['transition']['physics_samples'][0]['background_frames'].pop(), 'inventory'),
    (lambda r: r['transition']['physics_samples'][0]['background_frames'][0].__setitem__('timestamp_s', 0.), 'Stale'),
    (lambda r: r['transition']['physics_samples'][0]['all_object_clearance'].__setitem__('clearance_m', 999.), 'clearance'),
    (lambda r: r['transition']['physics_samples'][0]['native_background'].__setitem__('max_position_error_m', .1), 'motion'),
    (lambda r: r['transition']['observation'].__setitem__(22, 0.), 'observation'),
    (lambda r: r['transition']['reward_components'].__setitem__('terminal', 10.), 'reward_components'),
    (lambda r: r.__setitem__('returned_terminated', True), 'flags'),
    (lambda r: r['input_observation'].__setitem__(0, 44.), 'Policy input'),
    (lambda r: r['transition'].__setitem__('first_guard_failure', {'reason':'lane_departure'}), 'Latched'),
    (lambda r: r['transition']['deployment_observation']['selected_ids'].reverse(), 'selection'),
])
def test_tampered_measurements_and_reports_rejected(mutation, match):
    reset, rows = fixture()
    row = next(rows)
    # JSON-shaped lists accurately model actual raw files.
    row = json.loads(json.dumps(row)); mutation(row)
    with pytest.raises(ValueError, match=match):
        FullEpisodeAccumulator(reset, route=ROAD, stage='clear').update(row)


def test_missing_initial_backgrounds_is_rejected_not_inferred():
    reset, _ = fixture()
    reset.pop('background_frames')
    with pytest.raises(ValueError, match='inventory'):
        FullEpisodeAccumulator(reset, route=ROAD, stage='clear')


def test_background_slots_use_actual_velocity_and_all_objects_for_clearance():
    reset, rows = fixture()
    row = next(rows)
    row['transition']['physics_samples'][0]['background_frames'][-1]['x_m'] = reset['physics_state']['position_m'][0]
    row['transition']['physics_samples'][0]['background_frames'][-1]['y_m'] = reset['physics_state']['position_m'][1]
    with pytest.raises(ValueError, match='clearance'):
        FullEpisodeAccumulator(reset, route=ROAD, stage='clear').update(row)


def test_native_roll_velocity_must_match_exactly():
    reset, rows = fixture(continuous=True)
    audit = FullEpisodeAccumulator(reset, route=ROAD, stage='clear', continuous=True)
    for row in rows:
        if row['transition']['continuation']['window_roll']:
            row = json.loads(json.dumps(row))
            row['transition']['continuation']['native_boundary']['after']['velocity_m_s'][0] += 1e-9
            with pytest.raises(ValueError, match='pose/velocity'):
                audit.update(row)
            return
        audit.update(row)
    raise AssertionError('No roll fixture')


@pytest.mark.parametrize('name', ['../outside', '/absolute', 'sub\\file', 'missing'])
def test_manifest_paths_fail_closed(tmp_path, name):
    with pytest.raises(ValueError):
        _contained(tmp_path, name)


def test_comparison_rejects_boolean_as_numeric_and_nonfinite():
    assert not same(True, 1)
    assert not same(float('nan'), 0.)
    assert not same([1.], [1., 2.])


def test_exact_npz_pair_uses_pre_action_observation_and_signed_requested_controls():
    np = pytest.importorskip('numpy')
    observation = np.arange(42, dtype=np.float32)
    action = np.asarray([.25, -.75], dtype=np.float32)
    transition = {'requested_action': [.125, 0., .75]}
    _demonstration_pair(observation, action, observation.tolist(), action.tolist(), transition)
    with pytest.raises(ValueError, match='pre-action'):
        _demonstration_pair(observation+1, action, observation.tolist(), action.tolist(), transition)
    with pytest.raises(ValueError, match='NPZ action'):
        _demonstration_pair(observation, -action, observation.tolist(), action.tolist(), transition)
    with pytest.raises(ValueError, match='requested native'):
        _demonstration_pair(observation, action, observation.tolist(), action.tolist(), {'requested_action':[.125, .75, 0.]})


def test_crossing_schedule_reset_and_object_slots_replay_without_schedule_pose_substitution():
    reset, rows = fixture(stage='crossing')
    audit = FullEpisodeAccumulator(reset, route=ROAD, stage='crossing')
    audit.update(next(rows))
    assert audit.finish(complete=False)['evidence_valid']
    assert len(audit.hazards) == 1 and audit.task.last_deployment_observation['hazard_ids'] == ['hazard_000']


def compatible_reset():
    from experiments.full_scene_hazard_preflight import background_plan, choose_schedule
    from environments.full_loop_hazard_contract import background_specs, background_settings
    from traffic.live_showcase import LiveTraffic
    plan = background_plan(LiveTraffic(background_specs(), background_settings()))
    schedule, preflight = choose_schedule('clear', 61001, 1, ROAD, plan)
    preflight.update(phase='before_physical_reset', native_current_background={'passed':True})
    objects = []
    for entry in plan['entries']:
        angle, radius = entry['station_m']/500, 500+3.7*entry['lane']
        objects.append(MeasuredObject(entry['object_id'], radius*math.cos(angle), radius*math.sin(angle),
            angle+math.pi/2, 0., 0., 4.8, 1.9, 0.))
    spec = make_reset_spec(ROAD, seed=61001, episode_index=1, perturb=True)
    state = dict(position_m=[spec.x_m,spec.y_m,1.], velocity_m_s=[0.,0.,0.], speed_m_s=0.,
        quaternion_xyzw=[0.,0.,math.sin(spec.yaw_rad/2),math.cos(spec.yaw_rad/2)],
        yaw_rad=spec.yaw_rad,upright_z=1.,wheel_on_ground=[True]*4)
    task = FullLoopHazardTask(ROAD, stage='clear')
    task.set_objects((), now_s=0.);task.set_background_objects(objects, now_s=0.)
    obs = task.initialize(state, AppliedControl(0.,0.,1.,'reset',None,False))
    return dict(reset_spec=asdict(spec), reset_count=2, episode_serial=2, episode_tick=0,
        background_global_tick=0, physics_state=state, observation=list(obs),
        wheel_rotation_speed_rad_s=[0.]*4, hazard_frames=[], background_frames=list(map(asdict,objects)),
        hazard_schedule=schedule.as_dict(),hazard_background_preflight=preflight)


def test_compatible_reset_replays_candidate_index_not_reset_index():
    reset = compatible_reset()
    assert reset['reset_spec']['episode_index'] == 1
    assert reset['hazard_schedule']['episode_index'] == 16
    value = FullEpisodeAccumulator(reset, route=ROAD, stage='clear')
    assert value.schedule['episode_index'] == 16


@pytest.mark.parametrize('mutation,match', [
    (lambda r: r['hazard_background_preflight'].__setitem__('selected_draw',1), 'preflight replay'),
    (lambda r: r['hazard_background_preflight'].pop('requested_side'), 'side'),
    (lambda r: r['background_frames'][0].__setitem__('x_m',0.), 'planned pose'),
])
def test_compatibility_evidence_cannot_erase_rejection_or_change_plan(mutation,match):
    reset = compatible_reset();mutation(reset)
    with pytest.raises(ValueError,match=match):
        FullEpisodeAccumulator(reset,route=ROAD,stage='clear')
