"""Completed synthetic packages exercise the real verifier; no pickle is read."""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
import tempfile

import pytest

from experiments import lane_follow_ppo_protocol as protocol
from experiments import report_lane_follow_ppo as reporter
from experiments.lane_follow_ppo_evaluation import case_label, reduce_episode
from test_lane_follow_ppo_evaluation import ACCEPTANCE, ROUTES, fixture
from test_lane_follow_ppo_runner import completed_training_fixture, reseal_checkpoint, seal_run, write_json

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def project():
    parent=ROOT/'outputs/lab_lane_ppo/review/reporter-test-inputs'
    parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=parent) as temporary, pytest.MonkeyPatch.context() as patch:
        root=Path(temporary)
        _,training,manifest=completed_training_fixture(root,patch)
        source_names=list(protocol.source_files())+['scenarios/physics-road/routes.json',reporter.REDUCER]
        for name in ('scenarios/physics-road/routes.json',reporter.REDUCER,reporter.ASSEMBLER,reporter.ASSEMBLER_TEST):
            target=root/name;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((ROOT/name).read_bytes())
        patch.setattr(protocol,'source_files',lambda:source_names)
        patch.setattr(reporter,'source_files',lambda:source_names)
        hashes={name:protocol.digest(root/name) for name in source_names}
        manifest.update(source_hashes=hashes,finished_utc='2026-10-19T00:00:00Z',kit_before=dict(available=True,count=0))
        config=read(training/'resolved-config.json');config['acceptance']=dict(ACCEPTANCE)
        manifest['config']=config
        write_json(training/'resolved-config.json',config)
        for name in source_names:
            target=training/'source'/name;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((root/name).read_bytes())
        for checkpoint in (training/'checkpoints').iterdir():
            state=read(checkpoint/'state.json');state['provenance']['source_hashes']=hashes
            write_json(checkpoint/'state.json',state);reseal_checkpoint(checkpoint)
        summary=read(training/'summary.json')
        write_json(training/'native-result.json',summary)
        write_json(training/'gpu-samples.json',[dict(total_mib=10000,used_mib=1000)])
        (training/'runtime.log').write_text('synthetic completed run\n',encoding='utf-8')
        seal_run(training,manifest)
        identities={row['identity']['update']:row['identity'] for row in summary['checkpoints']}
        for update in (8,16,24,32):
            assert protocol.checkpoint_identity(training/'checkpoints'/f'update-{update:02d}',root=root)==identities[update]
        cache={}
        def evaluation_package(stage,controller,route,update=None):
            directory=root/'outputs/lab_lane_ppo'/f'{stage}-{controller}-{route}-{update}'
            directory.mkdir()
            seeds=[3101,3102,3103] if stage=='selection' else [4101,4102,4103,4104,4105]
            job=deepcopy(config)
            job.update(mode='evaluate',stage=stage,controller=controller,route_id=route,seeds=seeds,
                checkpoint_identity=identities[update] if update else None,
                checkpoint_path=(training/'checkpoints'/f'update-{update:02d}').relative_to(root).as_posix() if update else None)
            job.pop('smoke_evidence')
            cases=[]
            for seed in seeds:
                key=(route,seed)
                if key not in cache:
                    cache[key]=fixture(route_id=route,seed=seed)
                records,initial,lane=cache[key]
                assessment=reduce_episode(records,initial,lane,ACCEPTANCE)
                assert assessment['evidence_valid'] and not assessment['driving_success']
                trajectory=f'seed-{seed}-trajectory.json';reset=f'seed-{seed}-reset.json'
                write_json(directory/trajectory,records);write_json(directory/reset,initial)
                cases.append(dict(label=case_label(stage,controller,route,seed,update),policy=controller,
                    checkpoint=job['checkpoint_identity'],route_id=route,seed=seed,
                    trajectory_file=trajectory,reset_file=reset,loop_wall_seconds=1.,assessment=assessment))
            native=dict(passed=True,mode='evaluate',training_started=False,optimizer_steps=0,
                evaluation_frozen=True,cases=cases,runtime=summary['runtime'],versions=summary['versions'],
                scene_contract=summary['scene_contract'],viewport_cleanup=summary['viewport_cleanup'],
                contact_reporting=summary['contact_reporting'])
            supervised=dict(native,exit_code=0,stop_reason=None,source_unchanged=True,
                kit_after=dict(available=True,count=0),gpu_monitor_errors=[],unexpected_runtime_errors=[],usd_reference_warning_count=0)
            for name,value in (('resolved-config.json',job),('native-result.json',native),('summary.json',supervised),
                               ('evaluation-cases.json',cases),('gpu-samples.json',[dict(total_mib=10000,used_mib=1000)])):
                write_json(directory/name,value)
            (directory/'runtime.log').write_text('synthetic evaluated run\n',encoding='utf-8')
            for name in source_names:
                target=directory/'source'/name;target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes((root/name).read_bytes())
            package=dict(status='passed',finished_utc='2026-10-19T00:00:01Z',config=job,source_hashes=hashes,
                kit_before=dict(available=True,count=0))
            seal_run(directory,package)
            return directory
        selection=[evaluation_package('selection','ppo','left_r60',update) for update in (8,16,24,32)]
        heldout=[evaluation_package('heldout',policy,route,8 if policy=='ppo' else None)
                 for policy in ('ppo','scripted') for route in ('straight100','left_r60','right_r60')]
        yield dict(root=root,training=training,selection=selection,heldout=heldout)


@contextmanager
def replace_files(changes):
    original={path:path.read_bytes() if path.exists() else None for path in changes}
    try:
        for path,content in changes.items():
            path.write_bytes(content)
        yield
    finally:
        for path,content in original.items():
            if content is None:path.unlink()
            else:path.write_bytes(content)


@contextmanager
def resealed_changes(directory,changes):
    manifest=directory/'manifest.json'
    old=manifest.read_bytes()
    with replace_files({p:json.dumps(v).encode() for p,v in changes.items()}):
        try:
            seal_run(directory,read(manifest))
            yield
        finally:
            manifest.write_bytes(old)


def test_complete_selection_uses_real_checkpoint_verifier_and_retains_failed_driving(project):
    result=reporter.assemble_selection(project['training'],project['selection'],root=project['root'])
    assert result['report']['valid']
    assert result['report']['selected_checkpoint']['update']==8
    assert all(row['aggregate']['driving_successes']==0 and row['aggregate']['physical_failures']==3
               for row in result['report']['rankings'])
    assert result['assembler_source_sha256']==protocol.digest(project['root']/reporter.ASSEMBLER)
    assert result['assembler_test_source_sha256']==protocol.digest(project['root']/reporter.ASSEMBLER_TEST)


@pytest.mark.parametrize('mutation',['artifact','extra','missing','status','source_inventory','captured_source','duplicate_normalized','escape','boolean_exit'])
def test_package_integrity_rejects_corruption_relabeling_and_unsafe_inventory(project,mutation):
    directory=project['selection'][0]
    path=directory/'manifest.json';manifest=read(path)
    changes={}
    if mutation=='artifact':changes[directory/'runtime.log']=b'changed'
    elif mutation=='extra':changes[directory/'extra.txt']=b'extra'
    elif mutation=='missing':manifest['output_hashes'].pop('runtime.log')
    elif mutation=='status':manifest['status']='failed'
    elif mutation=='source_inventory':manifest['source_hashes'].pop('runtime.py')
    elif mutation=='captured_source':changes[directory/'source/runtime.py']=b'changed'
    elif mutation=='duplicate_normalized':manifest['output_hashes']['source\\runtime.py']=manifest['output_hashes']['source/runtime.py']
    elif mutation=='escape':manifest['output_hashes']['../outside.txt']='a'*64
    else:
        summary=read(directory/'summary.json');summary['exit_code']=False
        with resealed_changes(directory,{directory/'summary.json':summary}):
            with pytest.raises(ValueError):reporter.verify_package(directory,root=project['root'])
        return
    changes[path]=json.dumps(manifest).encode()
    with replace_files(changes),pytest.raises(ValueError):
        reporter.verify_package(directory,root=project['root'])


@pytest.mark.parametrize('field,value',[('evaluation_frozen',False),('optimizer_steps',1),('training_started',True)])
def test_resealed_evaluation_must_remain_frozen_and_optimizer_free(project,field,value):
    directory=project['selection'][0]
    summary,native=read(directory/'summary.json'),read(directory/'native-result.json')
    summary[field]=native[field]=value
    with resealed_changes(directory,{directory/'summary.json':summary,directory/'native-result.json':native}),pytest.raises(ValueError):
        reporter.assemble_selection(project['training'],project['selection'],root=project['root'])


@pytest.mark.parametrize('mutation',['raw_path','reset_seed','raw_reward','native_assessment'])
def test_resealed_semantically_forged_cases_still_fail_raw_reduction(project,mutation):
    directory=project['selection'][0]
    cases=read(directory/'evaluation-cases.json');summary=read(directory/'summary.json');native=read(directory/'native-result.json')
    changes={}
    if mutation=='raw_path':cases[0]['trajectory_file']='../outside.json'
    elif mutation=='reset_seed':
        path=directory/cases[0]['reset_file'];initial=read(path);initial['reset_spec']['seed']=4101;changes[path]=initial
    elif mutation=='raw_reward':
        path=directory/cases[0]['trajectory_file'];rows=read(path);rows[-1]['reward']=100.;changes[path]=rows
    else:cases[0]['assessment']['driving_success']=True
    summary['cases']=native['cases']=cases
    changes.update({directory/'evaluation-cases.json':cases,directory/'summary.json':summary,directory/'native-result.json':native})
    with resealed_changes(directory,changes),pytest.raises(ValueError):
        reporter.assemble_selection(project['training'],project['selection'],root=project['root'])


@pytest.mark.parametrize('which',['missing','duplicate','heldout'])
def test_selection_cannot_take_missing_duplicate_or_heldout_packages(project,which):
    packages=project['selection'][:]
    if which=='missing':packages.pop()
    elif which=='duplicate':packages[1]=packages[0]
    else:packages[0]=project['heldout'][0]
    with pytest.raises(ValueError):reporter.assemble_selection(project['training'],packages,root=project['root'])


def test_saved_selection_is_recomputed_before_complete_matched_heldout(project):
    root=project['root']
    selected=reporter.assemble_selection(project['training'],project['selection'],root=root)
    path=root/'outputs/lab_lane_ppo/reports/selection.json'
    reporter.write_report(path,selected,root=root)
    report=reporter.assemble_held_out(project['training'],path,project['heldout'],root=root)
    assert report['report']['valid'] and report['report']['held_out_used_for_selection'] is False
    assert len(report['report']['paired_differences'])==15
    assert report['report']['aggregates']['ppo']['all_routes']['physical_failures']==15
    selected['report']['selected_checkpoint']['update']=32
    with replace_files({path:json.dumps(selected).encode()}),pytest.raises(ValueError,match='does not reproduce'):
        reporter.assemble_held_out(project['training'],path,project['heldout'],root=root)


def test_exclusive_report_never_overwrites_or_enters_completed_package(project):
    root=project['root'];path=root/'outputs/lab_lane_ppo/reports/exclusive.json'
    reporter.write_report(path,dict(example=True),root=root)
    original=path.read_bytes()
    with pytest.raises(FileExistsError):reporter.write_report(path,dict(replaced=True),root=root)
    assert path.read_bytes()==original
    with pytest.raises(ValueError):reporter.write_report(project['training']/'report.json',{},root=root)
    with pytest.raises(ValueError):reporter.write_report(root/'report.json',{},root=root)


@pytest.mark.parametrize('content',['{"x":1,"x":2}','{"x":NaN}','{"x":Infinity}'])
def test_ambiguous_or_nonfinite_json_rejected(project,content):
    path=project['root']/'ambiguous.json'
    with replace_files({path:content.encode()}),pytest.raises(ValueError):reporter._read(path)


def test_recomputed_native_tuple_matches_saved_json_array_without_weakening_values():
    assert reporter._equivalent({'first':(241,'command_fallback')},{'first':[241,'command_fallback']})
    assert not reporter._equivalent({'first':(241,'command_fallback')},{'first':[242,'command_fallback']})
    assert not reporter._equivalent(True,1)
