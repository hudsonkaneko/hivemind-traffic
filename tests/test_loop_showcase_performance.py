"""Cross-profile motion assessment uses real hash-validated evidence directories."""
from copy import deepcopy
import hashlib

import pytest

from experiments import loop_showcase_performance as performance
from experiments.loop_showcase_config import make_config
from tests.test_loop_showcase_repeatability import pair,rows,materialize,with_visual_evidence


@pytest.fixture
def evidence(pair):
    with_visual_evidence((pair[0],),make_config('v05',gui=False,paced=False,drive_s=15))
    with_visual_evidence((pair[1],),make_config('v06',gui=False,paced=False,drive_s=15,render_hz=10))
    for run in pair:
        run['manifest']['source_hashes'].update({name:hashlib.sha256(b'source').hexdigest()
                                                for name in performance.REQUIRED_COMMON_SOURCES})
        scene=run['scene_contract']
        scene.update(source_vehicle_hashes={'vehicle.usda':'a'*64},controller={'target_speed_m_s':15.6464},
                     environment={'road':'fixture'},spawn={'radius_m':503.7})
        scene['fleet'].update(episode_id=run['run_id'],cache_readback=run['config']['version']=='v06',
                             wheel_visual_hz=10.,wheel_visual_update_every_steps=12,
                             asset_hashes={'background-vehicle.usda':'a'*64})
        for row in run['trajectory']:
            row.update(steering_rad=0.,controller_fallback=False,controller_status='following',
                       controller_reason='fixture',lane=1)
        run['summary']['runtime_profile']=dict(iterations=len(run['trajectory']),loop_wall_s=30.,
            non_overlapping_stages=True,stages={'physics':dict(count=len(run['trajectory']),total_s=10.,p95_ms=1.)})
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
