"""Full-road CPU contracts and native callback doubles; never launch Kit."""
from copy import deepcopy
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from environments.full_loop_hazard_contract import FullLoopHazardTask, background_settings, background_specs
from experiments import run_rsl_showcase as supervisor
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.road_hazards import MeasuredObject

ROOT=Path(__file__).resolve().parents[1]
ROAD=LaneRoute('straight',3.6,(RouteSegment(100.),))
COAST=AppliedControl(0.,0.,0.,'accepted',1,False)


def state(x=0.,speed=0.):
    return dict(position_m=[x,0.,.8],velocity_m_s=[speed,0.,0.],
        quaternion_xyzw=[0.,0.,0.,1.],yaw_rad=0.,upright_z=1.,
        speed_m_s=abs(speed),wheel_on_ground=[True]*4)


def obj(name,x,y=0.,*,t=0.,vx=0.,vy=0.,width=1.):
    return MeasuredObject(name,x,y,0.,vx,vy,1.,width,t)


def task(stage='crossing',backgrounds=()):
    value=FullLoopHazardTask(ROAD,stage=stage)
    value.set_objects(() if stage=='clear' else (obj('hazard_000',20.,5.),),now_s=0.)
    value.set_background_objects(backgrounds,now_s=0.)
    return value


def test_shared_nearest_two_has_no_privileged_hazard_priority_and_records_masking():
    value=task(backgrounds=(obj('background_b',5.,4.),obj('background_a',5.,-4.)))
    observation=value.initialize(state(),COAST)
    assert len(observation)==42
    assert value.last_hazard_observation.selected_ids==('background_a','background_b')
    assert value.last_deployment_observation['masked_hazard_ids']==['hazard_000']
    assert value.last_deployment_observation['background_count']==2
    assert observation[22:25]==pytest.approx((1.,5.,-4.))
    assert observation[32:35]==pytest.approx((1.,5.,4.))
    # These nearby cars do not count as the crossing object's encounter.
    assert not value.last_hazard_report['crossing_encountered']
    assert [o['object_id'] for o in value.last_hazard_report['objects']]==['hazard_000']


def test_actual_background_velocity_and_dimensions_survive_shared_observation():
    value=task('clear',(obj('background_a',5.,3.,vx=2.,vy=-1.,width=1.9),))
    value.initialize(state(),COAST)
    relative=value._relative(state(speed=1.),0)
    observation=value._observation(state(speed=1.),(0.,0.,0.),0,relative)
    assert observation[25:29]==pytest.approx((1.,-1.,.5,.95))
    assert observation[32:]==(0.,)*10


def test_background_cap_changes_reference_only_and_retains_requested_action():
    value=task('clear',(obj('background_a',4.),))
    value.initialize(state(),COAST)
    for tick in range(2,243,2):
        value.set_objects((),now_s=tick/120)
        value.set_background_objects((obj('background_a',4.,t=tick/120),),now_s=tick/120)
        tr=value.advance(state(),COAST,tick=tick)
    assert tr.route_state.target_speed_m_s<3.
    assert tr.observation[16:19]==(0.,0.,0.)
    assert not tr.terminated and tr.reason=='driving'


@pytest.mark.parametrize('objects',[(obj('a',1.),obj('a',2.)),('bad',),tuple(obj(str(i),i) for i in range(81))])
def test_invalid_background_frame_rejected(objects):
    with pytest.raises(ValueError):task('clear',objects)


def test_stale_background_frame_rejected_even_when_objects_empty():
    value=task('clear');value.initialize(state(),COAST)
    value.set_objects((),now_s=2/120)
    with pytest.raises(ValueError,match='stale'):value.advance(state(),COAST,tick=2)


def test_duplicate_identity_across_hazard_and_background_rejected():
    with pytest.raises(ValueError):task(backgrounds=(obj('hazard_000',5.),)).initialize(state(),COAST)


def test_fixed_background_pool_adjacent_only_and_no_hidden_lifecycle():
    from traffic.live_showcase import LiveTraffic
    from traffic.bypass_validation import OrientedBox
    from traffic.showcase_clearance import nearest_clearance
    specs=background_specs();live=LiveTraffic(specs,background_settings())
    assert len(specs)==80 and len(live.active)==12
    assert {live.specs[key].lane for key in live.active}=={1,2,3}
    membership=set(live.active)
    minimum=math.inf
    # Analytic placement check only, not a claim about native readback.
    for tick in range(1,4201):
        live.advance(tick/120)
        if tick%120:continue
        angle=3*max(0,tick/120-2)/500
        ego=OrientedBox(500*math.cos(angle),500*math.sin(angle),angle+math.pi/2,4.8,1.8)
        boxes=[live._box(live.specs[key].lane,live.station(key)) for key in sorted(live.active)]
        clearance=nearest_clearance(ego,boxes)
        assert not clearance['overlap']
        minimum=min(minimum,clearance['clearance_m'])
    assert minimum>.8 and live.active==membership and not live.events


@pytest.fixture
def adapter(monkeypatch):
    class Parent:
        def _setup_scene(self):self.bootstrap()
        def _refresh_objects(self):self.order.append('hazards')
        def _observe_completed_tick(self):
            if self._sampled_tick!=self.episode_tick:
                self._refresh_objects();self._step_samples.append({'tick':self.episode_tick})
                self._sampled_tick=self.episode_tick
        def _apply_action(self):self.order.append('ego_apply')
        def _reset_idx(self,env_ids):
            self.last_reset={'episode_serial':1};self.reset_count=1
        def _get_dones(self):
            self.extras={'lane_transition':{'tick':self.episode_tick}}
            return ('terminated','truncated')
        def close(self):self.order.append('stage_close')
    monkeypatch.setitem(sys.modules,'isaaclab.utils',SimpleNamespace(configclass=lambda x:x))
    monkeypatch.setitem(sys.modules,'environments.loop_hazard_lab',
        SimpleNamespace(LoopHazardLabCfg=object,LoopHazardLabEnv=Parent))
    spec=importlib.util.spec_from_file_location('_full_loop_adapter_test',ROOT/'environments/full_loop_hazard_lab.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    env=module.FullLoopHazardLabEnv.__new__(module.FullLoopHazardLabEnv)
    env.order=[];env._background_bound=True;env._total_physics_ticks=1001;env.episode_tick=1
    env._sampled_tick=0;env._step_samples=[];env.hazard_frames=();env._failure=None
    env.last_state=state();env.task=task('clear')
    env.reset_count=1
    spec=SimpleNamespace(vehicle_id='background_000')
    track=SimpleNamespace(vehicle_id='background_000_g001',x_m=4.,y_m=3.,yaw_rad=0.,length_m=4.8,width_m=1.9)
    env.background=SimpleNamespace(_ordered_specs=[spec],logical_identity=lambda s:s.vehicle_id+'_g001',
        tracks=lambda tick:[track],_measured=lambda:([],[[1.,-2.,0.,0.,0.,0.]]),
        pose_check=lambda time:dict(passed=True,target_time_s=time),
        update=lambda time:env.order.append(('background_target',time)),
        release=lambda:env.order.append('background_release'))
    return env


def test_refresh_preserves_measured_lateral_velocity_not_tangent_reconstruction(adapter):
    adapter._refresh_objects()
    frame=adapter.background_frames[0]
    assert (frame.vx_m_s,frame.vy_m_s)==(1.,-2.)
    assert frame.timestamp_s==1/120 # Episode clock, not global background clock.
    assert adapter.task.background_objects==(frame,)


def test_background_target_follows_completed_sample_and_uses_global_tick(adapter):
    adapter._apply_action()
    assert adapter.order==['hazards',('background_target',1002/120),'ego_apply']
    assert len(adapter._step_samples)==1
    sample=adapter._step_samples[0]
    assert sample['native_background']['target_time_s']==1001/120
    assert len(sample['background_frames'])==1 and not sample['all_object_clearance']['overlap']
    adapter._observe_completed_tick()
    assert len(adapter._step_samples)==1


def test_native_pose_error_aborts_before_next_target(adapter):
    adapter.background.pose_check=lambda time:{'passed':False}
    with pytest.raises(RuntimeError,match='motion contract'):adapter._apply_action()
    assert not any(isinstance(x,tuple) for x in adapter.order)


def test_overlap_latches_failure_and_preserves_earlier_failure(adapter):
    adapter.background.tracks=lambda tick:[SimpleNamespace(vehicle_id='background_000_g001',
        x_m=0.,y_m=0.,yaw_rad=0.,length_m=4.8,width_m=1.9)]
    adapter._observe_completed_tick()
    assert adapter._failure['reason']=='object_overlap' and adapter._failure['tick']==1
    adapter._failure={'reason':'rigid_contact','tick':0}
    adapter.episode_tick=2;adapter._observe_completed_tick()
    assert adapter._failure=={'reason':'rigid_contact','tick':0}


def test_terminal_evidence_detached_and_background_released_before_stage(adapter):
    adapter._refresh_objects()
    adapter.task.last_deployment_observation={'selected_ids':['background_000_g001']}
    assert adapter._get_dones()==('terminated','truncated')
    adapter.extras['lane_transition']['deployment_observation']['selected_ids'].clear()
    assert adapter.extras['final_info']['deployment_observation']['selected_ids']==['background_000_g001']
    adapter.close()
    assert adapter.order[-2:]==['background_release','stage_close']


def test_reset_retains_initial_background_frame_for_independent_replay(adapter):
    adapter._refresh_objects();adapter.extras={}
    adapter._reset_idx([0])
    assert adapter.last_reset['background_frames'][0]['vy_m_s']==-2.
    assert adapter.last_reset['physical_reset_count']==1
    assert adapter.last_reset['background_global_tick']==1001
    adapter.background_frames=()
    assert len(adapter.extras['reset']['background_frames'])==1


@pytest.fixture
def policy_file(tmp_path,monkeypatch):
    monkeypatch.setattr(supervisor,'ROOT',tmp_path)
    path=tmp_path/'outputs/export/policy.pt';path.parent.mkdir(parents=True)
    path.write_bytes(b'fixture only; never deserialize')
    return path


def test_job_hash_pins_policy_and_frozen_geometry(policy_file):
    job=supervisor.resolve_job(policy_file)
    assert supervisor.validate_job(job)==job
    assert job['observation_size']==42 and job['background_count']==12 and job['physics_car_count']==1
    assert not job['training'] and not job['continuous_windows'] and not job['safety_shield']
    policy_file.write_bytes(b'changed')
    with pytest.raises(ValueError,match='pinned policy'):supervisor.validate_job(job)


@pytest.mark.parametrize('options',[{'seed':True},{'seed':-1},{'max_seconds':901},{'capture':True},
    {'stage':'unknown'},{'side':'middle'},{'controller':'train'}])
def test_job_rejects_unsupported_modes(options):
    with pytest.raises(ValueError):supervisor.resolve_job(controller='scripted',**options) if 'controller' not in options else supervisor.resolve_job(**options)


@pytest.mark.parametrize('field,value',[('background_count',13),('continuous_windows',True),('training',True),('observation_size',22)])
def test_config_mutation_rejected(field,value):
    job=supervisor.resolve_job(controller='scripted');job[field]=value
    with pytest.raises(ValueError):supervisor.validate_job(job)


def test_scripted_mode_never_loads_policy(policy_file):
    with pytest.raises(ValueError):supervisor.resolve_job(policy_file,controller='scripted')
    job=supervisor.resolve_job(controller='scripted')
    assert job['policy_path'] is None and job['policy_sha256'] is None


@pytest.mark.parametrize('stage',['clear','static','crossing'])
def test_resolved_scenario_options_build_actual_schedule(stage):
    from traffic.road_hazards import make_hazard_schedule
    job=supervisor.resolve_job(controller='scripted',stage=stage,side='right')
    assert supervisor.validate_job(job)==job
    options={} if job['side'] is None else {'side':job['side']}
    schedule=make_hazard_schedule(job['stage'],job['seed'],0,route=ROAD,**options)
    assert schedule.stage==stage
    assert job['side']==('right' if stage=='crossing' else None)


def test_continuous_opt_in_declares_three_windows_and_preserves_default():
    ordinary=supervisor.resolve_job(controller='scripted',stage='clear')
    continuous=supervisor.resolve_job(controller='scripted',stage='clear',continuous=True,gui=True,capture=True)
    assert ordinary['episode_seconds']==35 and ordinary['task_progress_m']==60 and ordinary['window_count']==1
    assert not ordinary['continuous_windows']
    assert supervisor.validate_job(continuous)==continuous
    assert continuous['episode_seconds']==105 and continuous['task_progress_m']==180 and continuous['window_count']==3
    assert continuous['capture_seconds']==[3.,12.,20.,40.,60.]
    assert continuous['window_seconds']==35 and not continuous['training']
    with pytest.raises(ValueError):supervisor.resolve_job(controller='scripted',stage='static',continuous=True)
    with pytest.raises(ValueError):supervisor.resolve_job(controller='scripted',continuous=3)


def test_new_sources_compile_without_importing_native_modules():
    for name in ('environments/full_loop_hazard_contract.py','environments/full_loop_hazard_lab.py',
                 'experiments/run_rsl_showcase.py','scripts/physics_rsl_showcase.py'):
        compile((ROOT/name).read_text(encoding='utf-8'),name,'exec')


@pytest.fixture
def worker_package(tmp_path,monkeypatch):
    monkeypatch.setattr(supervisor,'ROOT',tmp_path)
    output=tmp_path/'outputs/rsl_full_sim/fixture';output.mkdir(parents=True)
    source=tmp_path/'example.py';source.write_text('pass\n',encoding='utf-8')
    captured=output/'source/example.py';captured.parent.mkdir();captured.write_bytes(source.read_bytes())
    hashes=lambda:{'example.py':supervisor.digest(source)}
    monkeypatch.setattr(supervisor,'source_hashes',hashes)
    job=supervisor.resolve_job(controller='scripted')
    supervisor.write_json(output/'resolved-config.json',job)
    supervisor.write_json(output/'manifest.json',dict(status='running',config=job,source_hashes=hashes()))
    return output,source,captured


def test_worker_requires_current_and_captured_source_match_before_native(worker_package):
    output,source,captured=worker_package
    assert supervisor.verify_worker_inputs(output)['controller']=='scripted'
    source.write_text('different\n',encoding='utf-8')
    with pytest.raises(ValueError,match='identity'):supervisor.verify_worker_inputs(output)


def test_worker_rejects_changed_snapshot_even_when_current_source_correct(worker_package):
    output,source,captured=worker_package
    captured.write_text('different\n',encoding='utf-8')
    with pytest.raises(ValueError,match='identity'):supervisor.verify_worker_inputs(output)


@pytest.mark.parametrize('field,value',[('status','completed'),('config',{})])
def test_worker_rejects_reused_or_mismatched_package(worker_package,field,value):
    output,_,_=worker_package
    manifest=json.loads((output/'manifest.json').read_text(encoding='utf-8'))
    manifest[field]=value;supervisor.write_json(output/'manifest.json',manifest)
    with pytest.raises(ValueError,match='identity'):supervisor.verify_worker_inputs(output)


def test_launch_failure_persists_failed_manifest_without_a_native_process(tmp_path,monkeypatch):
    from experiments import probe_support,verify_lane_following_lab
    from hivemind import launcher
    monkeypatch.setattr(supervisor,'ROOT',tmp_path)
    source=tmp_path/'example.py';source.write_text('pass\n',encoding='utf-8')
    monkeypatch.setattr(supervisor,'source_hashes',lambda:{'example.py':supervisor.digest(source)})
    monkeypatch.setattr(verify_lane_following_lab,'kit_process_snapshot',lambda:{'available':True,'count':0,'pids':[]})
    monkeypatch.setattr(launcher,'load_config',lambda root:{})
    monkeypatch.setattr(launcher,'isaac_runtime',lambda *args:tmp_path/'never-launch.bat')
    class Monitor:
        rows=[{'used_mib':1,'total_mib':10}];errors=[]
        def __enter__(self):return self
        def __exit__(self,*args):pass
    monkeypatch.setattr(probe_support,'GpuMonitor',Monitor)
    monkeypatch.setattr(supervisor.subprocess,'check_output',lambda *args,**kwargs:'fixture\n')
    def blocked(*args,**kwargs):raise OSError('synthetic launch failure; no process created')
    monkeypatch.setattr(supervisor.subprocess,'Popen',blocked)
    assert supervisor.supervise(supervisor.resolve_job(controller='scripted'))==1
    directories=list((tmp_path/'outputs/rsl_full_sim').iterdir());assert len(directories)==1
    output=directories[0]
    summary=json.loads((output/'summary.json').read_text(encoding='utf-8'))
    manifest=json.loads((output/'manifest.json').read_text(encoding='utf-8'))
    assert not summary['execution_passed'] and not summary['driving_passed']
    assert 'synthetic launch failure' in summary['launch_error'] and summary['returncode'] is None
    assert manifest['status']=='failed' and 'summary.json' in manifest['output_hashes']
    assert 'manifest.json' not in manifest['output_hashes']


def test_supervisor_preserves_native_startup_order_in_one_stream(tmp_path,monkeypatch):
    from experiments import probe_support,verify_lane_following_lab
    from hivemind import launcher
    monkeypatch.setattr(supervisor,'ROOT',tmp_path)
    source=tmp_path/'example.py';source.write_text('pass\n',encoding='utf-8')
    monkeypatch.setattr(supervisor,'source_hashes',lambda:{'example.py':supervisor.digest(source)})
    monkeypatch.setattr(verify_lane_following_lab,'kit_process_snapshot',lambda:{'available':True,'count':0,'pids':[]})
    monkeypatch.setattr(launcher,'load_config',lambda root:{})
    monkeypatch.setattr(launcher,'isaac_runtime',lambda *args:tmp_path/'never-launch.bat')
    class Monitor:
        rows=[{'used_mib':1,'total_mib':10}];errors=[]
        def __enter__(self):return self
        def __exit__(self,*args):pass
    monkeypatch.setattr(probe_support,'GpuMonitor',Monitor)
    monkeypatch.setattr(supervisor.subprocess,'check_output',lambda *args,**kwargs:'fixture\n')
    def fake_native(command,**options):
        assert options['stderr']==supervisor.subprocess.STDOUT
        output=Path(command[-1])
        for message in verify_lane_following_lab.KNOWN_STARTUP_ERRORS:
            options['stdout'].write('[Error] '+message+'\n')
        options['stdout'].write('LAB_LANE_TASK_READY full_highway\n')
        supervisor.write_json(output/'result.json',dict(completed=True,driving_passed=True,
            contact_cleanup={'passed':True},viewport_cleanup={'passed':True}))
        return SimpleNamespace(returncode=0,poll=lambda:0)
    monkeypatch.setattr(supervisor.subprocess,'Popen',fake_native)
    assert supervisor.supervise(supervisor.resolve_job(controller='scripted'))==0
    output=next((tmp_path/'outputs/rsl_full_sim').iterdir())
    summary=json.loads((output/'summary.json').read_text(encoding='utf-8'))
    assert summary['execution_passed'] and not summary['unexpected_runtime_errors']
    assert len(summary['known_baseline_startup_errors'])==2
    assert (output/'stderr.log').read_bytes()==b''


def test_plain_usd_full_assembly_export_inventory_and_dormant_visibility(adapter,request,monkeypatch):
    """Actual composition/export, no SimulationApp, physics stepping or renderer."""
    pytest.importorskip('pxr.Usd')
    import tempfile
    temporary=tempfile.TemporaryDirectory(prefix='full-scene-test-',dir=ROOT/'outputs')
    request.addfinalizer(temporary.cleanup)
    from pxr import Usd,UsdGeom,UsdPhysics
    from environments.lane_following_scene import author_lane_view,save_lane_scene
    from usd.physical_scene import compose_factory_vehicle
    from usd.loop_hazard_scene import author_hazards
    from usd.highway_loop_scene import validate_highway_loop_scene
    module_spec=importlib.util.spec_from_file_location('_full_road_factory_fixture',ROOT/'tests/test_highway_loop_scene.py')
    fixture=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(fixture)
    factory=SimpleNamespace(__file__=str(ROOT/'tests/test_highway_loop_scene.py'),DRIVE_NONE=0,
        AxesIndices=lambda *a:a,create4WheeledCarsScenario=fixture._factory)
    stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,'/World')
    adapter.sim=SimpleNamespace(stage=stage)
    adapter.scene_directory=Path(temporary.name)/'scene';adapter.route=ROAD
    adapter.cfg=SimpleNamespace(highway_directory='highway_usd/_v02',hazard_stage='clear')
    adapter.contact_reporting={};adapter._contact_reporting_snapshot=lambda phase:None
    monkeypatch.setitem(sys.modules,'environments.lane_following_lab',SimpleNamespace(ROOT=ROOT))
    prior={}
    def bootstrap():
        compose_factory_vehicle(stage,factory,120,adapter.scene_directory)
        adapter.road_view,adapter.runtime_layer=author_lane_view(stage,ROAD,adapter.scene_directory)
        save_lane_scene(stage,adapter.scene_directory,adapter.road_view,adapter.runtime_layer)
        adapter.hazards=author_hazards(stage,adapter.scene_directory,[])
        adapter.hazards.save_entry()
        prior.update(adapter._package_hashes())
    adapter.bootstrap=bootstrap
    adapter._setup_scene()
    report=adapter.scene_contract()
    assert len(report['inventory']['bodies'])==82 and report['background_active_count']==12
    assert all(adapter._package_hashes()[name]==value for name,value in prior.items())
    reopened=Usd.Stage.Open(adapter.full_scene_export['entry_path'])
    assert validate_highway_loop_scene(reopened)['passed']
    assert len([p for p in reopened.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)])==82
    assert all('road-layout.usda' not in layer for layer in adapter.full_scene_export['layers'])
    spec=next(s for s in adapter.background.specs if not adapter.background.is_active(s))
    with Usd.EditContext(stage,adapter.background.dynamic_layer):
        UsdGeom.Imageable(stage.GetPrimAtPath(spec.render_path)).GetVisibilityAttr().Set('inherited')
    with pytest.raises(RuntimeError,match='visibility'):adapter.scene_contract()
