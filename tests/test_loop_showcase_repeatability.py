"""Fresh-process repeatability and corrupt-manifest negative controls."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from experiments import loop_showcase_repeatability as repeat
from experiments.loop_showcase_config import assess,make_config
from tests.test_loop_showcase_config import valid_rows


@pytest.fixture(scope='module')
def rows():
    cfg=make_config('v01',gui=False,paced=False,drive_s=15)
    records=valid_rows(cfg)
    for row in records:row.update(completed_passes=0,completed_lane_changes=0)
    return records


@pytest.fixture
def pair(rows):
    cfg=make_config('v01',gui=False,paced=False,drive_s=15)
    def run(name):
        summary=assess(rows,cfg,0,[],0)
        summary.update(exit_code=0,source_changed_during_run=[])
        return dict(run_id=name,config=deepcopy(cfg),summary=summary,trajectory=rows,load_errors=[],
            scene_contract=dict(vehicle=dict(factory_sha256='a'*64)),
            manifest=dict(status='passed',finished_utc='2026-10-09T00:00:00Z',config=deepcopy(cfg),
                source_hashes={'fixture.py':hashlib.sha256(b'source').hexdigest()},git_commit=name,
                isaac_version='6.0.1-test',python='3.11',platform='Windows-test',logical_cpus=32,
                gpu_before=dict(name='GPU-test',driver='test-driver'),
                command=['isaac-python','physics_loop_showcase.py','--output','outputs/loop_showcase/'+name]))
    return run('first'),run('second')


def edit_row(run,index,**changes):
    run['trajectory']=list(run['trajectory']);run['trajectory'][index]=dict(run['trajectory'][index],**changes)


def test_exact_repeats_pass_despite_different_git_labels(pair):
    result=repeat.compare_evidence(*pair)
    assert result['passed'],result
    assert result['max_position_difference_m']==result['max_speed_difference_m_s']==0
    assert result['samples']==3240


@pytest.mark.parametrize('field,value,passes',[('position',.01,True),('position',.0201,False),
                                             ('speed',.01,True),('speed',.0201,False)])
def test_tolerance(pair,field,value,passes):
    row=pair[1]['trajectory'][500]
    if field=='position':edit_row(pair[1],500,position_m=[*row['position_m'][:2],row['position_m'][2]+value])
    else:edit_row(pair[1],500,speed_m_s=row['speed_m_s']+value)
    assert repeat.compare_evidence(*pair)['passed'] is passes


@pytest.mark.parametrize('change',['same_run','missing_manifest','running','no_finish','config_mismatch',
    'source_mismatch','empty_sources','bad_source_hash','missing_launch','wrong_launch','failed_summary',
    'failed_exit','bool_exit','source_changed','false_gate','missing_gates','missing_runtime','gpu_changed',
    'factory_changed','truncated_rows','tick_changed','nan_position','wrong_count','duplicate_passes',
    'outcomes_changed','integrity_error'])
def test_bad_evidence_cannot_false_pass(pair,change):
    run=pair[1]
    if change=='same_run':run['run_id']=pair[0]['run_id']
    elif change=='missing_manifest':run.pop('manifest')
    elif change=='running':run['manifest']['status']='running'
    elif change=='no_finish':run['manifest'].pop('finished_utc')
    elif change=='config_mismatch':run['config']['capture']=True
    elif change=='source_mismatch':run['manifest']['source_hashes']['fixture.py']='b'*64
    elif change=='empty_sources':run['manifest']['source_hashes']={}
    elif change=='bad_source_hash':run['manifest']['source_hashes']['fixture.py']='invalid'
    elif change=='missing_launch':run['manifest'].pop('command')
    elif change=='wrong_launch':run['manifest']['command'][-1]='outputs/loop_showcase/first'
    elif change=='failed_summary':run['summary']['passed']=False
    elif change=='failed_exit':run['summary']['exit_code']=1
    elif change=='bool_exit':run['summary']['exit_code']=False
    elif change=='source_changed':run['summary']['source_changed_during_run']=['fixture.py']
    elif change=='false_gate':run['summary']['gates']['wheel_support']=False
    elif change=='missing_gates':run['summary'].pop('gates')
    elif change=='missing_runtime':run['manifest'].pop('isaac_version')
    elif change=='gpu_changed':run['manifest']['gpu_before']['driver']='new-driver'
    elif change=='factory_changed':run['scene_contract']['vehicle']['factory_sha256']='b'*64
    elif change=='truncated_rows':run['trajectory']=run['trajectory'][:-1]
    elif change=='tick_changed':edit_row(run,500,tick=1)
    elif change=='nan_position':edit_row(run,500,position_m=[0.,float('nan'),1.])
    elif change=='wrong_count':run['summary']['completed_passes']=2
    elif change=='duplicate_passes':run['summary'].update(completed_passes=2,passed_vehicle_ids=['a','a'])
    elif change=='outcomes_changed':
        run['summary']['completed_lane_changes']=1
        run['trajectory']=[dict(row,completed_lane_changes=1) for row in run['trajectory']]
    elif change=='integrity_error':run['load_errors']=['tampered output']
    assert not repeat.compare_evidence(*pair)['passed'],change


def materialize(tmp_path,monkeypatch,evidence):
    monkeypatch.setattr(repeat,'ROOT',tmp_path)
    directory=tmp_path/'outputs/loop_showcase'/evidence['run_id'];directory.mkdir(parents=True)
    source=directory/'source/fixture.py';source.parent.mkdir();source.write_bytes(b'source')
    documents={'summary.json':evidence['summary'],'trajectory.json':evidence['trajectory'],
        'resolved-config.json':evidence['config'],'scene-contract.json':evidence['scene_contract']}
    for name,document in documents.items():(directory/name).write_text(json.dumps(document),encoding='utf-8')
    (directory/'extra.txt').write_bytes(b'also hashed')
    manifest=deepcopy(evidence['manifest'])
    manifest['output_hashes']={p.relative_to(directory).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for p in directory.rglob('*') if p.is_file()}
    (directory/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
    return directory


def test_load_checks_all_hashes_and_does_not_mutate(tmp_path,monkeypatch,pair):
    directory=materialize(tmp_path,monkeypatch,pair[0])
    before={str(p):p.read_bytes() for p in directory.rglob('*') if p.is_file()}
    loaded=repeat.load_run(directory)
    assert not loaded['load_errors']
    assert before=={str(p):p.read_bytes() for p in directory.rglob('*') if p.is_file()}


@pytest.mark.parametrize('name',['summary.json','trajectory.json','source/fixture.py','extra.txt'])
def test_tampered_output_including_noncore_file_fails(tmp_path,monkeypatch,pair,name):
    directory=materialize(tmp_path,monkeypatch,pair[0])
    with (directory/name).open('ab') as stream:stream.write(b' ')
    loaded=repeat.load_run(directory)
    assert loaded['load_errors']
    assert not repeat.compare_evidence(loaded,pair[1])['passed']


@pytest.mark.parametrize('name',['../outside','/absolute/file','C:/outside','source/../../outside'])
def test_recorded_path_escape_rejected(tmp_path,monkeypatch,pair,name):
    directory=materialize(tmp_path,monkeypatch,pair[0])
    manifest=json.loads((directory/'manifest.json').read_text())
    manifest['output_hashes'][name]='a'*64
    (directory/'manifest.json').write_text(json.dumps(manifest))
    assert repeat.load_run(directory)['load_errors']


def test_missing_required_hash_fails(tmp_path,monkeypatch,pair):
    directory=materialize(tmp_path,monkeypatch,pair[0])
    manifest=json.loads((directory/'manifest.json').read_text())
    manifest['output_hashes'].pop('trajectory.json')
    (directory/'manifest.json').write_text(json.dumps(manifest))
    assert repeat.load_run(directory)['load_errors']


def test_wrong_study_or_root_input_rejected(tmp_path,monkeypatch,pair):
    directory=materialize(tmp_path,monkeypatch,pair[0])
    with pytest.raises(ValueError):repeat.resolve_run(directory.parent)
    other=tmp_path/'outputs/other_study/run';other.mkdir(parents=True)
    with pytest.raises(ValueError):repeat.resolve_run(other)
