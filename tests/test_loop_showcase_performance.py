"""Cross-profile motion assessment uses real hash-validated evidence directories."""
from copy import deepcopy
import hashlib
import json

import pytest

from experiments import loop_showcase_performance as performance
from experiments.loop_showcase_config import make_config
from tests.test_loop_showcase_repeatability import pair,rows,materialize,with_visual_evidence
from traffic.showcase_performance import audit_schedule


@pytest.fixture
def evidence(pair):
    with_visual_evidence((pair[0],),make_config('v05',gui=False,paced=False,drive_s=15))
    with_visual_evidence((pair[1],),make_config('v06',gui=False,paced=False,drive_s=15,render_hz=10))
    return prepare_physical_evidence(pair)


def prepare_physical_evidence(pair):
    for run in pair:
        run['manifest']['source_hashes'].update({name:hashlib.sha256(b'source').hexdigest()
                                                for name in performance.REQUIRED_COMMON_SOURCES})
        scene=run['scene_contract']
        scene.update(source_vehicle_hashes={'vehicle.usda':'a'*64},controller={'target_speed_m_s':15.6464},
                     environment={'road':'fixture'},spawn={'radius_m':503.7})
        scene['fleet'].update(episode_id=run['run_id'],cache_readback=run['config']['version'] in ('v06','v07'),
                             wheel_visual_hz=float(run['config']['render_hz']),
                             wheel_visual_update_every_steps=120//run['config']['render_hz'],
                             asset_hashes={'background-vehicle.usda':'a'*64})
        for row in run['trajectory']:
            row.update(steering_rad=0.,controller_fallback=False,controller_status='following',
                       controller_reason='fixture',lane=1)
        run['summary']['runtime_profile']=dict(iterations=len(run['trajectory']),loop_wall_s=30.,
            non_overlapping_stages=True,stages={'physics':dict(count=len(run['trajectory']),total_s=10.,p95_ms=1.)})
    return pair


@pytest.fixture(params=[('showcase',30),('editor',60)])
def responsive_evidence(pair,request):
    ui_profile,hz=request.param
    with_visual_evidence((pair[0],),make_config('v06',gui=False,paced=False,drive_s=15))
    with_visual_evidence((pair[1],),make_config('v07',gui=False,paced=False,drive_s=15,
        ui_profile=ui_profile,render_hz=hz))
    prepare_physical_evidence(pair)
    for run in pair:
        run['manifest']['source_hashes'].update({name:hashlib.sha256(b'source').hexdigest()
            for name in performance.RESPONSIVE_COMMON_SOURCES})
        run['scene_contract']['fleet']['collider_count']=80
        run['scene_contract']['fleet']['asset_hashes']={name:'a'*64 for name in
            performance.REQUIRED_FLEET_ASSETS|{performance.FLEET_LAYOUT_ASSET}}
        run['scene_contract']['fleet']['layout']=performance.FLEET_LAYOUT_ASSET
        run['scene_contract']['vehicle']['collider_count']=5
        run['summary']['preview_timing']={'completed_frames':run['config']['duration_s']*run['config']['render_hz']}
    pair[1]['manifest']['source_hashes'].update({name:hashlib.sha256(b'source').hexdigest()
        for name in performance.RESPONSIVE_ADDED_SOURCES})
    pair[1]['scene_contract']['fleet'].update(performance.INSTANCED_FLEET_METADATA)
    pair[1]['scene_contract']['fleet']['asset_hashes'].update({name:'b'*64 for name in
        performance.INSTANCED_FLEET_ASSETS|{performance.FLEET_LAYOUT_ASSET}})
    # The fixture retains the same sampled native/visual schedule at 30/60 Hz.
    pair[1]['background_visual_checks']=[check for check in pair[1]['background_visual_checks']
        if audit_schedule(pair[1]['config'],check['tick'])['visual']]
    return pair


def write_runs(tmp_path,monkeypatch,evidence):
    directories=[]
    for run in evidence:
        directory=materialize(tmp_path,monkeypatch,run)
        for name in run['manifest']['source_hashes']:
            source=directory/'source'/name
            source.parent.mkdir(parents=True,exist_ok=True);source.write_bytes(b'source')
        directories.append(directory)
    return directories


def test_matched_v05_v06_reports_motion_not_repeatability(tmp_path,monkeypatch,evidence):
    result=performance.compare_motion(*write_runs(tmp_path,monkeypatch,evidence))
    assert result['passed'],result
    assert result['max_position_difference_m']==result['max_speed_difference_m_s']==0.
    assert 'not configuration repeatability' in result['scope']
    assert result['timings']['first']['visual_audits']==270
    assert result['timings']['second']['visual_audits']==28
    assert result['timings']['first']['stages']['physics']['count']==3240


def test_matched_v06_v07_reports_declared_presentation_changes(tmp_path,monkeypatch,responsive_evidence):
    result=performance.compare_motion(*write_runs(tmp_path,monkeypatch,responsive_evidence))
    assert result['passed'],result
    assert result['max_position_difference_m']==result['max_speed_difference_m_s']==0.
    assert 'not configuration repeatability or source equivalence' in result['scope']
    assert result['config_differences']==['cpu_threads','frame_generation','instance_visuals','profile_render',
        'render_hz','statistics_hz','ui_profile','version']
    assert {entry['path'] for entry in result['source_differences']}==performance.RESPONSIVE_ADDED_SOURCES
    assert all(entry['first'] is None and entry['second'] for entry in result['source_differences'])
    assert result['timings']['first']['visual_audits']==result['timings']['second']['visual_audits']==28
    expected=27*responsive_evidence[1]['config']['render_hz']
    assert result['timings']['second']['measured_rendered_frames']==expected
    assert result['runtime_identities']['first']==result['runtime_identities']['second']
    assert {entry['path'] for entry in result['fleet_asset_differences']}==(
        performance.INSTANCED_FLEET_ASSETS|{performance.FLEET_LAYOUT_ASSET})
    assert {entry['field'] for entry in result['fleet_visual_differences']}==set(performance.INSTANCED_FLEET_METADATA)


@pytest.mark.parametrize('change',['physics_hz','drive_torque_nm','frame_generation','statistics_hz',
    'ego_colliders','fleet_colliders','fleet_identity','controller','asset','runtime',
    'missing_physics_source','missing_presentation_source','missing_profiler_source'])
def test_v07_cannot_relax_physics_assets_runtime_or_evidence(tmp_path,monkeypatch,responsive_evidence,change):
    run=responsive_evidence[1]
    if change in ('physics_hz','drive_torque_nm','frame_generation','statistics_hz'):
        run['config'][change]={'physics_hz':60,'drive_torque_nm':350.,'frame_generation':True,
                               'statistics_hz':1}[change]
        run['manifest']['config']=deepcopy(run['config'])
    elif change=='ego_colliders':run['scene_contract']['vehicle']['collider_count']=1
    elif change=='fleet_colliders':run['scene_contract']['fleet']['collider_count']=20
    elif change=='fleet_identity':run['scene_contract']['fleet']['specs'][0]['vehicle_id']='replacement'
    elif change=='controller':run['scene_contract']['controller']['target_speed_m_s']=10.
    elif change=='asset':run['scene_contract']['source_vehicle_hashes']['vehicle.usda']='b'*64
    elif change=='runtime':run['manifest']['isaac_version']='different-runtime'
    elif change=='missing_physics_source':run['manifest']['source_hashes'].pop('usd/showcase_fleet.py')
    elif change=='missing_presentation_source':run['manifest']['source_hashes'].pop('visualization/highwaysim.showcase.kit')
    elif change=='missing_profiler_source':run['manifest']['source_hashes'].pop('visualization/showcase_render_profile.py')
    result=performance.compare_motion(*write_runs(tmp_path,monkeypatch,responsive_evidence))
    assert not result['passed'],(change,result)


@pytest.mark.parametrize('name,allowed',[
    ('scripts/physics_loop_showcase.py',True),
    ('visualization/loop_showcase_view.py',True),
    ('traffic/loop_showcase_control.py',False),
    ('usd/showcase_fleet.py',True),
    ('traffic/physx_vehicle.py',False)])
def test_v07_source_allowlist_is_narrow_and_reports_each_change(tmp_path,monkeypatch,responsive_evidence,name,allowed):
    for run in responsive_evidence:
        run['manifest']['source_hashes'][name]=hashlib.sha256(b'source').hexdigest()
    paths=write_runs(tmp_path,monkeypatch,responsive_evidence)
    changed=b'new version source'
    (paths[1]/'source'/name).write_bytes(changed)
    manifest_path=paths[1]/'manifest.json';manifest=json.loads(manifest_path.read_text())
    manifest['source_hashes'][name]=hashlib.sha256(changed).hexdigest()
    manifest_path.write_text(json.dumps(manifest))
    result=performance.compare_motion(*paths)
    assert result['passed'] is allowed,result
    assert name in {entry['path'] for entry in result['source_differences']}
    assert (name in result['allowed_source_differences']) is allowed


@pytest.mark.parametrize('change',['entry','physics','geometry','missing_original','extra_asset',
    'missing_visual_asset','escape_path','duplicate_slashes','undeclared_visual_metadata','wrong_instance_flag'])
def test_v07_visual_exceptions_never_relax_original_asset_provenance(tmp_path,monkeypatch,responsive_evidence,change):
    fleet=responsive_evidence[1]['scene_contract']['fleet'];assets=fleet['asset_hashes']
    if change in ('entry','physics','geometry'):
        name={'entry':'assets/showcase-block-car.usda','physics':'assets/showcase-block-car-physics.usda',
              'geometry':'assets/showcase-block-car-geometry.usda'}[change]
        assets[name]='c'*64
    elif change=='missing_original':assets.pop('assets/showcase-block-car.usda')
    elif change=='extra_asset':assets['assets/unrelated-geometry.usda']='c'*64
    elif change=='missing_visual_asset':assets.pop('assets/showcase-block-car-visual-library.usda')
    elif change=='escape_path':
        assets['../assets/showcase-block-car-physics.usda']=assets.pop('assets/showcase-block-car-physics.usda')
    elif change=='duplicate_slashes':assets['assets\\showcase-block-car.usda']=assets['assets/showcase-block-car.usda']
    elif change=='undeclared_visual_metadata':fleet['visual_instance_policy']='whole colliders instanced'
    elif change=='wrong_instance_flag':fleet['instance_visuals']=False
    result=performance.compare_motion(*write_runs(tmp_path,monkeypatch,responsive_evidence))
    assert not result['passed'],(change,result)


def test_v07_normalizes_only_windows_slashes_and_reports_known_asset_changes(tmp_path,monkeypatch,responsive_evidence):
    for run in responsive_evidence:
        fleet=run['scene_contract']['fleet']
        fleet['asset_hashes']={name.replace('/','\\'):value for name,value in fleet['asset_hashes'].items()}
    responsive_evidence[1]['config']['cpu_threads']=16
    responsive_evidence[1]['manifest']['config']=deepcopy(responsive_evidence[1]['config'])
    result=performance.compare_motion(*write_runs(tmp_path,monkeypatch,responsive_evidence))
    assert result['passed'],result
    assert 'cpu_threads' in result['config_differences']
    assert {entry['path'] for entry in result['fleet_asset_differences']}==(
        performance.INSTANCED_FLEET_ASSETS|{performance.FLEET_LAYOUT_ASSET})


@pytest.mark.parametrize('where',['config','summary'])
def test_native_render_diagnostic_cannot_qualify_performance(tmp_path,monkeypatch,responsive_evidence,where):
    run=responsive_evidence[1]
    if where=='config':
        run['config']['profile_render']=True
        run['manifest']['config']=deepcopy(run['config'])
    else:run['summary']['display_qualification_exclusions']=['native_profiler_enabled']
    result=performance.compare_motion(*write_runs(tmp_path,monkeypatch,responsive_evidence))
    assert not result['passed'],result
    assert any('diagnostic and cannot qualify performance' in failure for failure in result['failures'])


@pytest.mark.parametrize('change',['same_id','gui','paced','controller','asset','fleet','physics_source',
    'missing_source','controls','position','speed','truncated','integrity','missing_timing',
    'bad_stage','missing_fleet_assets','incomplete_frames'])
def test_unmatched_or_incomplete_runs_fail(tmp_path,monkeypatch,evidence,change):
    run=evidence[1]
    if change=='same_id':
        paths=write_runs(tmp_path,monkeypatch,evidence)
        assert not performance.compare_motion(paths[0],paths[0])['passed'];return
    if change in ('gui','paced'):
        run['config'][change]=True;run['manifest']['config']=deepcopy(run['config'])
    elif change=='controller':run['scene_contract']['controller']['target_speed_m_s']=10.
    elif change=='asset':run['scene_contract']['source_vehicle_hashes']['vehicle.usda']='b'*64
    elif change=='fleet':run['scene_contract']['fleet']['specs'][0]['speed_m_s']=10.
    elif change=='missing_source':run['manifest']['source_hashes'].pop('traffic/driver_control.py')
    elif change=='controls':run['trajectory'][500]['controller_reason']='changed'
    elif change=='position':run['trajectory'][500]['position_m'][2]+=.0101
    elif change=='speed':run['trajectory'][500]['speed_m_s']+=.0101
    elif change=='truncated':run['trajectory'].pop()
    elif change=='missing_timing':run['summary'].pop('runtime_profile')
    elif change=='bad_stage':run['summary']['runtime_profile']['stages']['physics']['p95_ms']=-1.
    elif change=='missing_fleet_assets':run['scene_contract']['fleet'].pop('asset_hashes')
    elif change=='incomplete_frames':run['summary']['preview_timing']={'completed_frames':2}
    paths=write_runs(tmp_path,monkeypatch,evidence)
    if change in ('physics_source','integrity'):
        target=paths[1]/('source/traffic/driver_control.py' if change=='physics_source' else 'extra.txt')
        target.write_bytes(b'changed')
    result=performance.compare_motion(*paths)
    assert not result['passed'],(change,result)


def test_declared_source_changes_are_reported(tmp_path,monkeypatch,evidence):
    name='traffic/showcase_performance.py'
    for run in evidence:run['manifest']['source_hashes'][name]=hashlib.sha256(b'source').hexdigest()
    paths=write_runs(tmp_path,monkeypatch,evidence)
    changed=b'optimized source'
    (paths[1]/'source'/name).write_bytes(changed)
    import json
    manifest_path=paths[1]/'manifest.json';manifest=json.loads(manifest_path.read_text())
    manifest['source_hashes'][name]=hashlib.sha256(changed).hexdigest()
    manifest_path.write_text(json.dumps(manifest))
    result=performance.compare_motion(*paths)
    assert result['passed'],result
    assert result['source_differences'][0]['path']==name


def test_packaging_paths_and_runtime_layer_ids_normalize_only_known_locations(tmp_path,monkeypatch):
    from experiments import loop_showcase_repeatability as repeat
    monkeypatch.setattr(repeat,'ROOT',tmp_path)
    run=dict(relative_path='outputs/loop_showcase/first',scene_contract=dict(
        vehicle=dict(scene_composition=dict(scene_directory=str(tmp_path/'outputs/loop_showcase/first/scene'))),
        environment=dict(validation=dict(layers=[str(tmp_path/'outputs/loop_showcase/first/scene/assets/road.usda'),
            'anon:123ABC:World1-session.usda','anon:123DEF:World1.usd']))))
    normalized=performance._physical_contract(run)
    assert normalized['vehicle']['scene_composition']['scene_directory']=='scene/'
    assert normalized['environment']['validation']['layers']==[
        'anon:World1-session.usda','anon:World1.usd','scene/assets/road.usda']
    run['scene_contract']['vehicle']['scene_composition']['scene_directory']=str(tmp_path/'outside')
    with pytest.raises(ValueError):performance._physical_contract(run)
