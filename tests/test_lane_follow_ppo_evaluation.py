"""Pure CPU evidence fixtures; these numerical traces are not native claims."""
from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path

import pytest

from environments.lane_following_contract import LaneFollowingTask, make_reset_spec
from experiments import lane_follow_ppo_evaluation as evaluation
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import load_routes
from usd.physical_scene import PATHS

ROOT = Path(__file__).resolve().parents[1]
ROUTES = load_routes(ROOT/'scenarios/physics-road/routes.json')
ACCEPTANCE = json.loads((ROOT/'experiments/lab-lane-following-validation.json').read_text(encoding='utf-8'))['acceptance']
CHECKPOINTS = [dict(update=n, control_steps=n*2048, model_sha256=f'{n:064x}',
                    normalization_sha256=f'{n+100:064x}') for n in evaluation.UPDATES]
BRAKE = AppliedControl(0., 0., 1., 'accepted', 0, False)
COAST = AppliedControl(0., 0., 0., 'accepted', 1, False)


def state(route, station=0., lateral=0., yaw_error=0., speed=0., height=.8):
    point = route.evaluate(station)
    yaw = point.yaw_rad+yaw_error
    return dict(position_m=[point.position_xy[0]-lateral*math.sin(point.yaw_rad),
                           point.position_xy[1]+lateral*math.cos(point.yaw_rad), height],
        quaternion_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)],
        velocity_m_s=[speed*math.cos(yaw), speed*math.sin(yaw), 0.], speed_m_s=abs(speed),
        yaw_rad=yaw, upright_z=1., wheel_on_ground=[True]*4)


def reset(route, seed, serial=1):
    spec = make_reset_spec(route, seed=seed, episode_index=serial-1, perturb=True)
    native = state(route, lateral=spec.lateral_offset_m, yaw_error=spec.yaw_error_rad)
    obs = LaneFollowingTask(route).initialize(native, BRAKE)
    return dict(reset_spec=asdict(spec), episode_serial=serial, reset_count=serial, episode_tick=0,
        physics_state=native, observation=list(obs), wheel_rotation_speed_rad_s=[0.]*4)


def fixture(reason='command_fallback', route_id='left_r60', seed=3101):
    route = ROUTES[route_id]
    initial = reset(route, seed)
    task = LaneFollowingTask(route)
    task.initialize(initial['physics_state'], BRAKE)
    rows, samples, events, first = [], [], [], None
    unsupported = 0
    for tick in range(1, 4201):
        native = deepcopy(initial['physics_state'])
        applied = BRAKE if tick <= 240 else COAST
        contacts = []
        if reason == 'success' and tick > 240:
            # Gradual perturbation recovery avoids an impossible control jump.
            fraction = max(0., 1-(tick-240)/120)
            native = state(route, station=min(59.8, (tick-240)*.025), speed=3. if tick <= 2632 else 0.,
                lateral=initial['reset_spec']['lateral_offset_m']*fraction,
                yaw_error=initial['reset_spec']['yaw_error_rad']*fraction)
        elif reason == 'wheel_support_loss' and tick > 240:
            native['wheel_on_ground'][0] = False
        elif reason == 'excessive_tilt' and tick >= 242:
            angle=.09
            native.update(quaternion_xyzw=[math.sin(angle/2),0.,0.,math.cos(angle/2)], yaw_rad=0., upright_z=math.cos(angle))
        elif reason == 'unsafe_height' and tick >= 242:
            native['position_m'][2] = 2.001
        elif reason == 'overspeed' and tick >= 242:
            native.update(velocity_m_s=[4.001,0.,0.],speed_m_s=4.001)
        elif reason == 'reverse_motion' and tick >= 242:
            native.update(velocity_m_s=[-.201,0.,0.],speed_m_s=.201)
        elif reason == 'lane_departure' and tick > 240:
            native['position_m'][1] += (tick-240)*.02
        elif reason == 'state_discontinuity' and tick == 242:
            native['position_m'][0] += .1
        elif reason == 'rigid_contact' and tick == 242:
            contacts = [dict(tick=tick, event_type='found', actor0=PATHS.chassis,
                actor1='/World/Environment/Props/Other', collider0=PATHS.chassis+'/Collider',
                collider1='/World/Environment/Props/Other/Collider', contact_count=1,
                contacts=[dict(position=[0.,0.,0.],normal=[1.,0.,0.],impulse=[1.,0.,0.],separation_m=-.001)])]
        if reason == 'command_fallback' and tick >= 241:
            applied = AppliedControl(0.,0.,1.,'watchdog',None,True)
        elif first is not None:
            applied = BRAKE
        unsupported = unsupported+1 if tick > 240 and not all(native['wheel_on_ground']) else 0
        guard = task.physics_guard(native,tick=tick,contact_count=len(contacts),
            command_fallback=applied.is_fallback,unsupported_ticks=unsupported)
        if guard and first is None:
            first = dict(tick=tick,reason=guard,physics_state=deepcopy(native))
        samples.append(dict(tick=tick,physics_state=deepcopy(native),contacts=contacts,
            contact_count=len(contacts),unsupported_ticks=unsupported,guard_reason=guard))
        events.extend(contacts)
        if tick%2:
            continue
        transition = task.advance(native,applied,tick=tick,contact_count=len(events),
            command_fallback=applied.is_fallback,failure_reason=first['reason'] if first else None)
        row = transition.as_dict()
        action=[0.,0.,1. if tick<=240 or reason=='command_fallback' else 0.]
        row.update(episode_serial=1,episode_tick=tick,physics_state=deepcopy(native),control=asdict(applied),
            physics_samples=samples,contacts=events,first_guard_failure=deepcopy(first),
            requested_action=action[:],policy_action_raw=action[:],env_action=action[:],
            returned_observation=list(transition.observation),final_observation=list(transition.observation),
            returned_reward=transition.reward,returned_terminated=transition.terminated,
            returned_truncated=transition.truncated,returned_done=transition.terminated or transition.truncated,
            sb3_time_limit_truncated=transition.truncated and not transition.terminated)
        rows.append(row)
        samples,events=[],[]
        if row['returned_done']:
            row['auto_reset']=reset(route,seed,2)
            row['returned_observation']=row['auto_reset']['observation'][:]
            row['sb3_terminal_observation']=list(transition.observation)
            break
    return rows,initial,route


@pytest.mark.parametrize('reason', ['success','time_limit','command_fallback','wheel_support_loss',
    'excessive_tilt','unsafe_height','overspeed','reverse_motion','lane_departure','state_discontinuity','rigid_contact'])
def test_valid_policy_endings_are_evidence_not_automatic_driving_passes(reason):
    records,initial,route=fixture(reason)
    report=evaluation.reduce_episode(records,initial,route,ACCEPTANCE)
    assert report['evidence_valid'], report['checks']
    assert report['terminal_reason']==reason
    assert report['driving_success']==(reason=='success')
    assert report['physical_failure']==(reason not in ('success','time_limit'))
    json.dumps(report,allow_nan=False)


@pytest.mark.parametrize('mutate',[
    lambda r:r[-1].update(reason='success'),
    lambda r:r[-1].update(reason='overspeed'),
    lambda r:r[-1].update(reason=['overspeed']),
    lambda r:r[-1].update(returned_done=False),
    lambda r:r[-1].update(reward=100.),
    lambda r:r[-1].pop('sb3_terminal_observation'),
    lambda r:r[0]['physics_samples'].pop(),
    lambda r:r[-1].update(policy_action_raw=[float('nan'),0.,0.]),
    lambda r:r[-1].pop('env_action'),
    lambda r:r[-1].update(env_action=[0.,1.,0.]),
    lambda r:r.pop(),
])
def test_forged_incomplete_and_nonfinite_failure_evidence_rejects(mutate):
    records,initial,route=fixture()
    mutate(records)
    report=evaluation.reduce_episode(records,initial,route,ACCEPTANCE)
    assert not report['evidence_valid']
    assert not report['driving_success']


def test_raw_clipping_and_brake_override_count_even_ignored_settling_requests():
    records,initial,route=fixture()
    records[0].update(policy_action_raw=[2.,2.,2.],env_action=[.5,1.,1.],requested_action=[.5,0.,1.])
    report=evaluation.reduce_episode(records,initial,route,ACCEPTANCE)
    assert report['evidence_valid']
    action=report['metrics']['actions']
    assert action['all_requested']['clipped_actions']==1
    assert action['all_requested']['clipped_components']==3
    assert action['all_requested']['brake_throttle_overrides']==1
    assert action['post_settle_requested']['clipped_actions']==0
    assert action['settling_requests_ignored']==120


def test_derivative_sampling_does_not_include_autoreset_jump():
    records,initial,route=fixture('overspeed')
    report=evaluation.reduce_episode(records,initial,route,ACCEPTANCE)
    metrics=report['metrics']['scalar_speed_derivatives']
    assert metrics['all_acceleration_m_s2']['count']==242
    assert metrics['all_jerk_m_s3']['count']==241
    assert metrics['post_settle_acceleration_m_s2']['count']==2
    assert metrics['post_settle_jerk_m_s3']['count']==1
    assert metrics['all_acceleration_m_s2']['maximum']==pytest.approx(4.001*120)
    assert metrics['all_jerk_m_s3']['maximum']==pytest.approx(4.001*120**2)


def test_first_substep_support_failure_applies_emergency_before_terminal_record():
    records,initial,route=fixture('wheel_support_loss')
    final=records[-1]
    assert final['tick']==254 and final['physics_samples'][0]['guard_reason']=='wheel_support_loss'
    assert final['requested_action']==[0.,0.,0.]
    assert final['control']['brake']==1 and not final['control']['is_fallback']
    report=evaluation.reduce_episode(records,initial,route,ACCEPTANCE)
    assert report['evidence_valid'] and report['physical_failure']
    assert report['metrics']['actions']['all_requested']['guard_brake_intervals']==1


def test_applied_control_cannot_ignore_a_valid_requested_action():
    records,initial,route=fixture('time_limit')
    records[121].update(policy_action_raw=[.5,0.,0.],env_action=[.5,0.,0.],requested_action=[.5,0.,0.])
    report=evaluation.reduce_episode(records,initial,route,ACCEPTANCE)
    assert not report['evidence_valid'] and not report['checks']['raw_and_supplied_actions']


def cases(stage='selection', checkpoint=None):
    answer=[]
    policies=('ppo',) if stage=='selection' else ('ppo','scripted')
    checkpoints=CHECKPOINTS if stage=='selection' else [checkpoint or CHECKPOINTS[0]]
    seeds=evaluation.SELECTION_SEEDS if stage=='selection' else evaluation.HELD_OUT_SEEDS
    route_ids=('left_r60',) if stage=='selection' else evaluation.ROUTE_IDS
    cache={}
    for policy in policies:
        for identity in checkpoints:
            for route_id in route_ids:
                for seed in seeds:
                    key=(route_id,seed)
                    if key not in cache:
                        cache[key]=fixture(route_id=route_id,seed=seed)
                    records,initial,_=cache[key]
                    answer.append(dict(label=evaluation.case_label(stage,policy,route_id,seed,identity['update'] if policy=='ppo' else None),
                        policy=policy,checkpoint=deepcopy(identity) if policy=='ppo' else None,route_id=route_id,
                        seed=seed,records=records,reset_record=initial))
    return answer


@pytest.fixture(scope='module')
def selection_cases():
    return cases()


def test_complete_failed_selection_retains_every_seed_and_earliest_exact_tie(selection_cases):
    report=evaluation.select_checkpoint(selection_cases,CHECKPOINTS,ROUTES,ACCEPTANCE)
    assert report['valid'],report['checks']
    assert report['selected_checkpoint']==CHECKPOINTS[0]
    assert [r['checkpoint']['update'] for r in report['rankings']]==[8,16,24,32]
    assert all(r['aggregate']['physical_failures']==3 for r in report['rankings'])
    assert all(r['aggregate']['driving_successes']==0 for r in report['rankings'])
    json.dumps(report,allow_nan=False)


@pytest.mark.parametrize('mutate',[
    lambda c:c.pop(),
    lambda c:c.append(deepcopy(c[0])),
    lambda c:c.__setitem__(1,deepcopy(c[0])),
    lambda c:c[0].update(seed=4101),
    lambda c:c[0].update(route_id='right_r60'),
    lambda c:c[0]['checkpoint'].update(model_sha256='f'*64),
    lambda c:c[0]['checkpoint'].update(normalization_sha256='f'*64),
    lambda c:c[0]['reset_record']['reset_spec'].update(episode_index=1),
    lambda c:c[0]['reset_record']['reset_spec'].update(lateral_offset_m=0.),
    lambda c:c[0]['records'][-1]['auto_reset']['reset_spec'].update(episode_index=0),
    lambda c:c[0].update(label='heldout/ppo/u08/left_r60/seed-4101'),
    lambda c:c[0].update(extra=True),
])
def test_selection_inventory_and_seed_or_checkpoint_relabeling_reject(selection_cases,mutate):
    selected=deepcopy(selection_cases)
    mutate(selected)
    report=evaluation.select_checkpoint(selected,CHECKPOINTS,ROUTES,ACCEPTANCE)
    assert not report['valid'] and report['selected_checkpoint'] is None


@pytest.mark.parametrize('change',[
    lambda c:c.pop(),lambda c:c.__setitem__(1,deepcopy(c[0])),
    lambda c:c[0].update(control_steps=1),lambda c:c[0].update(model_sha256='not-a-hash'),
    lambda c:c[0].update(update=True),
])
def test_checkpoint_manifest_inventory_rejects(selection_cases,change):
    checkpoints=deepcopy(CHECKPOINTS);change(checkpoints)
    assert not evaluation.select_checkpoint(selection_cases,checkpoints,ROUTES,ACCEPTANCE)['valid']


def rank_row(update,success=0,failures=0,progress=0.,rms=.1):
    return dict(checkpoint=dict(update=update),aggregate=dict(driving_successes=success,
        physical_failures=failures,mean_progress_m=progress,mean_driving_lateral_rms_m=rms))


@pytest.mark.parametrize('better,worse',[
    (rank_row(32,success=1,failures=2),rank_row(8,success=0)),
    (rank_row(32,failures=0,progress=1),rank_row(8,failures=1,progress=50)),
    (rank_row(32,progress=2,rms=1),rank_row(8,progress=1,rms=.01)),
    (rank_row(32,rms=.1),rank_row(8,rms=.2)),
    (rank_row(32,rms=.1),rank_row(8,rms=None)),
    (rank_row(8),rank_row(16)),
])
def test_predeclared_lexicographic_ranking_does_not_use_reward(better,worse):
    assert sorted([worse,better],key=evaluation._rank)==[better,worse]


@pytest.fixture(scope='module')
def held_out_cases():
    return cases('heldout')


def test_matched_heldout_failures_are_reported_without_reselection(held_out_cases):
    report=evaluation.assess_held_out(held_out_cases,CHECKPOINTS[0],ROUTES,ACCEPTANCE)
    assert report['valid'],report['checks']
    assert report['selected_checkpoint']==CHECKPOINTS[0]
    assert report['held_out_used_for_selection'] is False
    assert len(report['paired_differences'])==15
    assert report['aggregates']['ppo']['all_routes']['physical_failures']==15
    assert all(row['progress_m']==0 for row in report['paired_differences'])


@pytest.mark.parametrize('mutate',[
    lambda c:c.pop(),lambda c:c.append(deepcopy(c[0])),
    lambda c:c[0].update(checkpoint=CHECKPOINTS[1]),
    lambda c:c[-1].update(checkpoint=CHECKPOINTS[0]),
    lambda c:c[0].update(seed=3101),
    lambda c:c[0]['reset_record']['reset_spec'].update(yaw_error_rad=0.),
])
def test_heldout_inventory_and_matched_seed_checks_cannot_be_bypassed(held_out_cases,mutate):
    records=deepcopy(held_out_cases);mutate(records)
    report=evaluation.assess_held_out(records,CHECKPOINTS[0],ROUTES,ACCEPTANCE)
    assert not report['valid'] and report['aggregates']=={}


def test_reduction_does_not_mutate_raw_evidence(selection_cases):
    before=deepcopy(selection_cases)
    evaluation.select_checkpoint(selection_cases,CHECKPOINTS,ROUTES,ACCEPTANCE)
    assert selection_cases==before
