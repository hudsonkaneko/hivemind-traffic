"""Deterministic pre-reset scenario checks; no native runtime imports or launch."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from contextlib import nullcontext

import pytest

from environments.full_loop_hazard_contract import background_settings, background_specs
from experiments import full_scene_hazard_preflight as preflight
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.live_showcase import LiveTraffic
from traffic.road_hazards import HazardSchedule, HazardSpec


ROOT = Path(__file__).resolve().parents[1]
ROAD = LaneRoute('loop_r500', 3.6, (RouteSegment(100., 1/500),), (500., 0.), math.pi/2)


def live_plan():
    return LiveTraffic(background_specs(), background_settings())


def advance(live, seconds):
    for tick in range(1, round(seconds*120)+1):
        live.advance(tick/120)


def schedule(*, lateral=-3.7, end=3.7, onset=0., speed=1., station=25., index=0):
    return HazardSchedule('crossing', 101, index, (
        HazardSpec('hazard_000', 'crossing', station, lateral, end, onset, speed),))


def plan_with_near_car(station=25.):
    plan = preflight.background_plan(live_plan())
    # Other objects stay remote throughout this bounded episode.
    for index, row in enumerate(plan['entries']):
        row['station_m'] = 1000. + index*100.
    plan['entries'][0].update(lane=1, station_m=station)
    return plan


def test_live_plan_reads_actual_phase_and_does_not_mutate():
    live = live_plan()
    advance(live, 31.)
    before = deepcopy(live.__dict__)
    result = preflight.background_plan(live)
    assert live.__dict__ == before
    assert result['global_start_s'] == 31.
    assert result['entries'][0]['station_m'] == pytest.approx(40.+31*5*.44704)
    assert len(result['entries']) == 12
    assert [x['object_id'] for x in result['entries']] == sorted(x['object_id'] for x in result['entries'])


@pytest.mark.parametrize('mutation', [
    lambda live: setattr(live, 'stopping', True),
    lambda live: setattr(live, 'ramping', True),
    lambda live: setattr(live, 'speed_m_s', 3.),
    lambda live: live.events.append({'type': 'exit'}),
    lambda live: live.pending_lifecycle.append({'type': 'enter'}),
    lambda live: live.active.pop(),
])
def test_changed_background_premise_fails_closed(mutation):
    live = live_plan(); mutation(live)
    with pytest.raises(ValueError, match='fixed'):
        preflight.background_plan(live)


def test_clear_schedule_reports_no_hazard_pairs_and_explicit_scope():
    chosen, report = preflight.choose_schedule('clear', 101, 3, ROAD, preflight.background_plan(live_plan()))
    assert chosen.episode_index == 48 and report['selected_draw'] == 0
    assessment = report['draws'][0]['assessment']
    assert assessment['passed'] and assessment['samples_checked'] == 0
    assert assessment['minimum_checked_clearance_m'] is None
    assert 'no ego outcome' in assessment['scope']
    assert report['requested_side'] is None


def test_full_static_horizon_clearance_has_conservative_rotation_bound():
    static = HazardSchedule('static', 101, 0, (HazardSpec('hazard_000', 'static', 25., 0., 0., 0., 0.),))
    report = preflight.assess_schedule(static, ROAD, plan_with_near_car())
    assert report['passed'] and report['samples_checked'] == 351
    assert report['minimum_checked_clearance_m'] > 2.
    expected_padding = preflight.ANGULAR_SPEED_RAD_S*(511.1+math.hypot(4.8, 1.9)/2)/20
    assert report['between_sample_padding_m'] == pytest.approx(expected_padding)
    assert report['continuous_clearance_lower_bound_m'] == pytest.approx(
        report['minimum_checked_clearance_m']-expected_padding-.02)
    native_corner_bound = .002 + math.hypot(4.8, 1.9)/2*.0001 + .005 + math.sqrt(2)/2*.005
    assert native_corner_bound < preflight.NATIVE_TRACKING_PADDING_M


def test_initially_overlapping_hazard_rejected_at_zero():
    report = preflight.assess_schedule(schedule(onset=20.), ROAD, plan_with_near_car())
    assert not report['passed'] and report['samples_checked'] == 1
    assert report['first_rejection']['time_s'] == 0.
    assert report['first_rejection']['overlap']


def test_initially_safe_background_future_intersection_is_rejected():
    # Hazard remains parked in the adjacent lane until20 s. The car starts15 m
    # behind and reaches it before motion onset; checking reset alone is unsafe.
    report = preflight.assess_schedule(schedule(onset=20.), ROAD, plan_with_near_car(10.))
    assert not report['passed']
    assert 1 < report['first_rejection']['time_s'] < 10


def test_padding_rejects_a_sample_above_pointwise_clearance_threshold(monkeypatch):
    # Isolate the acceptance boundary: .9 m at samples is insufficient for the
    # declared continuous-motion uncertainty, even though pointwise .8 m passes.
    monkeypatch.setattr(preflight, 'nearest_clearance', lambda *args:
        {'clearance_m': .9, 'overlap': False, 'exact_pairs': 1})
    report = preflight.assess_schedule(schedule(), ROAD, plan_with_near_car())
    assert not report['passed']
    assert .8 < report['first_rejection']['clearance_m'] < report['required_sample_clearance_m']


@pytest.mark.parametrize('mutation', [
    lambda p: p.update(global_start_s=float('nan')),
    lambda p: p.update(angular_speed_rad_s=.1),
    lambda p: p['entries'].pop(),
    lambda p: p['entries'][0].update(lane=True),
    lambda p: p['entries'][0].update(station_m=float('inf')),
    lambda p: p['entries'][0].update(object_id=p['entries'][1]['object_id']),
    lambda p: p.update(extra='ignored?'),
])
def test_malformed_plan_cannot_pass_even_clear(mutation):
    plan = preflight.background_plan(live_plan()); mutation(plan)
    with pytest.raises(ValueError):
        preflight.choose_schedule('clear', 101, 0, ROAD, plan)


def test_first_accepted_draw_retains_rejected_draws_and_unchanged_seed(monkeypatch):
    calls = []
    def candidate(stage, seed, index, **kwargs):
        calls.append((stage, seed, index, kwargs['side']))
        return schedule(index=index, station=25. if index < 34 else 200.)
    monkeypatch.setattr(preflight, 'make_hazard_schedule', candidate)
    plan = plan_with_near_car()
    chosen, report = preflight.choose_schedule('crossing', 101, 2, ROAD, plan, side='right')
    assert calls == [('crossing', 101, i, 'right') for i in (32, 33, 34)]
    assert chosen.episode_index == 34 and report['selected_draw'] == 2
    assert [x['assessment']['passed'] for x in report['draws']] == [False, False, True]
    assert report['requested_side'] == 'right' and not report['actor_future_schedule_input']
    plan['entries'][0]['station_m'] = 999.
    assert report['background_plan']['entries'][0]['station_m'] == 25.
    json.dumps(report, allow_nan=False)


def test_all_rejected_candidates_fail_with_complete_bounded_durable_evidence(monkeypatch):
    calls = []
    def candidate(stage, seed, index, **kwargs):
        calls.append(index); return schedule(index=index)
    monkeypatch.setattr(preflight, 'make_hazard_schedule', candidate)
    with pytest.raises(preflight.ScenarioCompatibilityError) as caught:
        preflight.choose_schedule('crossing', 101, 2, ROAD, plan_with_near_car())
    assert calls == list(range(32, 48))
    report = caught.value.report
    assert report['outcome'] == 'invalid_scenario' and report['selected_draw'] is None
    assert len(report['draws']) == 16 and all(not x['assessment']['passed'] for x in report['draws'])
    assert json.loads(str(caught.value).split(': ', 1)[1])['candidate_limit'] == 16


@pytest.mark.parametrize('side', [None, 'left', 'right'])
def test_real_seeded_draws_are_reproducible(side):
    plan = preflight.background_plan(live_plan())
    first = preflight.choose_schedule('crossing', 61001, 0, ROAD, plan, side=side)
    second = preflight.choose_schedule('crossing', 61001, 0, ROAD, plan, side=side)
    assert first == second
    assert first[1]['draws'][-1]['assessment']['continuous_clearance_lower_bound_m'] >= .8


@pytest.fixture
def adapter(monkeypatch):
    class Parent:
        def _save_full_scene(self):
            self.order.append('export')
            return {'exported': True}
        def scene_contract(self):return {'passed':True}
        def _reset_idx(self, ids):
            self.order.append('before_schedule')
            self.schedule = self._make_schedule(2)
            self.order.append('physical_reset')
            self.last_reset = {'hazard_schedule': self.schedule.as_dict()}
    monkeypatch.setitem(sys.modules, 'environments.full_loop_hazard_lab',
        SimpleNamespace(FullLoopHazardLabCfg=object, FullLoopHazardLabEnv=Parent))
    spec = importlib.util.spec_from_file_location('_compatible_full_loop_test',
        ROOT/'environments/compatible_full_loop_hazard_lab.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    env = module.CompatibleFullLoopHazardLabEnv.__new__(module.CompatibleFullLoopHazardLabEnv)
    env.cfg = SimpleNamespace(hazard_stage='clear', hazard_side='left')
    env.route = ROAD; env._reset_seed = 101; env.order = []; env.extras = {}
    env.pending_hazard_background_preflight = None
    return env


def test_adapter_initial_authoring_and_reset_projection_are_consistent(adapter):
    adapter._make_schedule(0)
    assert adapter.pending_hazard_background_preflight['phase'] == 'pre_play_authoring'
    assert adapter.pending_hazard_background_preflight['requested_side'] is None
    adapter.live_traffic = live_plan(); advance(adapter.live_traffic, 31.)
    adapter._background_bound = True
    adapter.background = SimpleNamespace(pose_check=lambda t: {'passed': True, 'time': t})
    adapter._reset_idx([0])
    assert adapter.order == ['before_schedule', 'physical_reset']
    report = adapter.last_reset['hazard_background_preflight']
    assert report['phase'] == 'before_physical_reset'
    assert report['background_plan']['global_start_s'] == 31.
    assert report['native_current_background']['time'] == 31.
    adapter.pending_hazard_background_preflight['outcome'] = 'changed'
    assert adapter.last_reset['hazard_background_preflight']['outcome'] == 'accepted'
    adapter.last_reset['hazard_background_preflight']['outcome'] = 'changed'
    assert adapter.extras['reset']['hazard_background_preflight']['outcome'] == 'accepted'


def test_bad_native_background_prevents_any_physical_reset(adapter):
    adapter.live_traffic = live_plan(); adapter._background_bound = True
    adapter.background = SimpleNamespace(pose_check=lambda t: {'passed': False, 'max_yaw_error_rad': 3.047})
    with pytest.raises(RuntimeError, match='native background') as caught:
        adapter._reset_idx([0])
    assert adapter.order == ['before_schedule']
    assert adapter.pending_hazard_background_preflight is None
    details=json.loads(str(caught.value).split(': ',1)[1])
    assert details['native_pose_check']['max_yaw_error_rad']==3.047
    assert len(details['background_plan']['entries'])==12


def test_empty_reset_does_not_generate_candidate(adapter):
    adapter._reset_idx([])
    assert adapter.order == [] and adapter.pending_hazard_background_preflight is None


@pytest.fixture
def preplay(adapter, monkeypatch):
    from usd.showcase_fleet import fleet_pose
    adapter._background_bound=False
    adapter.live_traffic=live_plan()
    adapter.preplay_parking_initialization=None
    specs=list(adapter.live_traffic.specs.values())
    adapter.background=SimpleNamespace(_view=None,_target_count=0,specs=specs,
        dynamic_layer='runtime',is_active=lambda s:s.vehicle_id in adapter.live_traffic.active)
    attributes={}
    class Attribute:
        def __init__(self,path,value):self.path,self.value=path,value
        def Set(self,value):
            adapter.order.append(('orientation',self.path));self.value=value;return True
    for spec in specs:
        x,y,z,w=fleet_pose(spec,0)['quaternion_xyzw']
        for path in (spec.chassis_path,spec.render_path):attributes[path]=Attribute(path,(w,x,y,z))
    class Prim:
        def __init__(self,path):self.path=path
        def GetAttribute(self,name):
            assert name=='xformOp:orient'
            return attributes[self.path]
    adapter.sim=SimpleNamespace(stage=SimpleNamespace(GetPrimAtPath=Prim))
    monkeypatch.setitem(sys.modules,'pxr',SimpleNamespace(
        Gf=SimpleNamespace(Quatf=lambda *v:tuple(v),Quatd=lambda *v:tuple(v)),
        Usd=SimpleNamespace(EditContext=lambda *a:nullcontext())))
    return adapter,attributes


def test_dormant_authoring_fixes_initial_pose_without_active_writes_or_steps(preplay):
    env,attributes=preplay
    before={path:attr.value for path,attr in attributes.items()}
    assert env._save_full_scene()=={'exported':True}
    assert env.order[-1]=='export'
    assert len(env.order)==137 # 68 dormant chassis+proxy opinions, then export.
    for spec in env.background.specs:
        for path in (spec.chassis_path,spec.render_path):
            if env.background.is_active(spec):assert attributes[path].value==before[path]
            else:assert attributes[path].value==(1.,0.,0.,0.)
    proof=env.scene_contract()['preplay_parking_initialization']
    assert proof['dormant_slots']==68 and proof['active_pose_writes']==proof['native_pose_writes']==proof['physics_steps']==0
    assert env.live_traffic.time_s==0 and env.background._target_count==0


@pytest.mark.parametrize('mutation',[
    lambda e:setattr(e,'_background_bound',True),
    lambda e:setattr(e.background,'_view',object()),
    lambda e:setattr(e.background,'_target_count',1),
    lambda e:setattr(e.live_traffic,'time_s',1.),
])
def test_parking_authoring_cannot_reenter_after_binding_or_motion(preplay,mutation):
    env,attributes=preplay;before={p:a.value for p,a in attributes.items()}
    mutation(env)
    with pytest.raises(RuntimeError,match='pre-physics'):env._save_full_scene()
    assert env.order==[] and {p:a.value for p,a in attributes.items()}==before
