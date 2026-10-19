"""Benchmark negative controls and serial-supervisor tests; no native launches."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from experiments import showcase_benchmark as benchmark
from experiments.loop_showcase_config import assess, fleet_specs, validate_config
from tests.test_loop_showcase_config import valid_rows
from tests.test_loop_showcase_repeatability import materialize
from traffic.showcase_performance import audit_schedule
from visualization.preview_timing import PreviewClock
from visualization.showcase_responsiveness import assess_responsiveness


QUIET = dict(available=True, pids=[], count=0)
LOAD = dict(before=QUIET, after=QUIET)


def set_timing(run, slowdown=1.):
    cfg, summary = run['config'], run['summary']
    frames, last = [], 0.
    count = cfg['duration_s'] * cfg['render_hz']
    for index in range(1, count + 1):
        sim = index / cfg['render_hz']
        wall = sim * slowdown
        frames.append(dict(sim_time_s=sim, active_wall_s=wall, lag_s=max(0., wall-sim),
                           frame_interval_s=wall-last, rtf=sim/wall))
        last = wall
    clock = PreviewClock(paced=True, clock=lambda: 0.)
    clock.frames = frames
    summary['preview_timing'] = clock.summary(expected_simulation_time_s=cfg['duration_s'], expected_frames=count)
    times = [i * slowdown / cfg['render_hz'] for i in range(count + 1)]
    event = dict(application_frames=[dict(pre=value) for value in times], native_completions=times)
    summary['responsiveness'] = assess_responsiveness(event['application_frames'], times,
        wall_s=last, simulation_s=cfg['duration_s'], render_hz=cfg['render_hz'], complete=True)
    summary['smooth_display_passed'] = bool(summary['preview_timing']['soft_realtime_passed']
        and summary['preview_timing']['p95_frame_interval_s'] <= 1.5/cfg['render_hz']
        and summary['preview_timing']['max_frame_interval_s'] <= .2 and summary['responsiveness']['passed'])
    run['preview-timing.json'], run['ui-event-timing.json'] = frames, event


def evidence(name, case='dense-hidden'):
    cfg = next(row['config'] for row in benchmark.benchmark_cases(drive_s=15) if row['name'] == case)
    rows = valid_rows(cfg)
    for row in rows:
        row.update(steering_rad=0., controller_fallback=False, controller_status='following',
                   controller_reason='fixture', lane=1, completed_lane_changes=0, completed_passes=0)
        if cfg['version'] == 'v09':
            row.update(active_background_count=cfg['controls']['traffic_count'],
                       requested_background_count=cfg['controls']['traffic_count'],
                       applied_ego_target_m_s=cfg['target_speed_m_s'], applied_traffic_speed_m_s=6.7056,
                       applied_following_s=cfg['controls']['following_s'])
    summary = dict(assess(rows, cfg, 0, [], 0), exit_code=0, source_changed_during_run=[],
        runtime_error_lines=[], usd_reference_warning_count=0, gpu_monitor_errors=[],
        cleanup_verified=True, stop_reason=None, display_qualification_exclusions=[], gpu_peak_mib=100)
    if cfg['version'] == 'v09':
        summary['sidebar'] = dict(display_events=[], event_count=0, active=deepcopy(cfg['controls']),
                                 pending=deepcopy(cfg['controls']))
        summary['live_traffic'] = dict(lifecycle_events=[], no_process_restart=True, final_settings=deepcopy(cfg['controls']))
    specs = fleet_specs(cfg)
    interval = cfg['physics_hz']//cfg['render_hz']
    visuals = [dict(tick=tick, passed=True, count=len(specs), publications=tick//interval+1,
        native_target_time_s=tick/cfg['physics_hz'], max_position_error_m=0., max_yaw_error_rad=0.,
        all_proxies_visible=True, visibility_matches_active=True)
        for tick in range(1, len(rows)+1) if audit_schedule(cfg, tick)['visual']]
    summary['evidence_policy'] = dict(native_audit_ticks=[tick for tick in range(1,len(rows)+1)
                                                        if audit_schedule(cfg,tick)['native']])
    run = dict(run_id=name, relative_path='outputs/loop_showcase/'+name, config=cfg, summary=summary,
        trajectory=rows, load_errors=[], background_visual_checks=visuals,
        scene_contract=dict(vehicle=dict(factory_sha256='a'*64), source_vehicle_hashes={},
            controller={'target': cfg['target_speed_m_s']}, environment={'road': 'fixture'}, spawn={'lane': 1},
            fleet=dict(specs=specs, count=80, collider_count=80, episode_id=name,
                       asset_hashes={'assets/block.usda':'b'*64}, wheel_visual_hz=cfg['render_hz'],
                       wheel_visual_update_every_steps=interval)),
        manifest=dict(status='passed', finished_utc='2026-10-16T00:00:00Z', config=deepcopy(cfg),
            source_hashes={'fixture.py':hashlib.sha256(b'source').hexdigest()},
            isaac_version='6.0.1-test', python='3.11', platform='Windows-test', logical_cpus=16,
            gpu_before=dict(name='GPU-test', driver='test-driver'),
            command=['python','physics_loop_showcase.py','--output','outputs/loop_showcase/'+name]))
    run.update({'background-visual-checks.json':visuals, 'lifecycle.json':[{'event':'closed'}],
                'startup-readiness.json':{'passed':True}, 'gpu-samples.json':[dict(used_mib=100, total_mib=1000)]})
    run['manifest']['source_hashes'].update({name:hashlib.sha256(b'source').hexdigest()
                                           for name in benchmark._required_sources(cfg)})
    set_timing(run)
    return run


def compare(first, second, allowed=()):
    return benchmark.compare_pair(first, second, allowed_config_fields=allowed,
                                  first_load=LOAD, second_load=LOAD)


def test_exact_matrix_and_no_density_comparison():
    cases = benchmark.benchmark_cases()
    assert len(cases) == 11
    assert [row['name'] for row in cases] == ['light-visible','light-hidden','dense-visible','dense-hidden',
        'dense-20hz','dense-captures','dense-full','editor-cost','minimal-cost','dense-repeat-1','dense-repeat-2']
    for row in cases:
        cfg=row['config'];validate_config(cfg)
        assert cfg['vehicle_profile']=='public' and cfg['seed']==101 and cfg['drive_s']==90
        assert cfg['gui'] and cfg['paced'] and cfg['traffic_statistics'] and cfg['cpu_threads']==8
    for _, left, right, _ in benchmark.PAIRS:
        assert left.startswith('light') == right.startswith('light')


@pytest.mark.parametrize('left,right,allowed',[(a,b,c) for _,a,b,c in benchmark.PAIRS])
def test_declared_pairs_accept_same_motion_even_when_timing_fails(left,right,allowed):
    first, second = evidence('first',left), evidence('second',right)
    set_timing(second,slowdown=1.2)
    result=compare(first,second,allowed)
    assert result['valid'],result
    assert result['physical_valid'] and result['load_qualified']
    assert not result['both_strict_display_qualified']
    assert result['observed_wall_change_percent']==pytest.approx(20)


def public_composition(run, *, windows_separators=False):
    directory = (benchmark.repeat.ROOT / run['relative_path'] / 'scene').resolve().as_posix()
    if windows_separators:
        directory = directory.replace('/', '\\')
    composition = dict(assembly_directory=directory,
        factory_sha256='a'*64, factory_contract=dict(passed=True, prims=31, attributes=173),
        paths=dict(chassis='/World/Vehicles/vehicle_ego/Chassis'),
        wheel_paths=['/World/Vehicles/vehicle_ego/Chassis/FrontLeftWheel'],
        references_not_instances=True)
    run['scene_contract']['vehicle']['scene_composition'] = composition
    run['scene_contract']['vehicle']['mass_kg'] = 1800.
    return composition


def test_public_assembly_packaging_normalization_preserves_inputs_and_semantics():
    first, second = evidence('first'), evidence('second')
    public_composition(first, windows_separators=True)
    public_composition(second)
    originals = deepcopy((first['scene_contract'], second['scene_contract']))
    result = compare(first, second)
    assert result['valid'], result
    normalized = benchmark._benchmark_physical_contract(first)
    normalized_composition = normalized['vehicle']['scene_composition']
    assert normalized_composition['assembly_directory'] == 'scene/'
    expected_composition = deepcopy(originals[0]['vehicle']['scene_composition'])
    expected_composition['assembly_directory'] = 'scene/'
    assert normalized_composition == expected_composition
    assert (first['scene_contract'], second['scene_contract']) == originals


@pytest.mark.parametrize('bad', ['other_run', 'outside', 'traversal', 'relative',
                                'suffix', 'empty', 'null', 'number', 'missing'])
def test_public_assembly_directory_is_not_blindly_discarded(bad):
    first, second = evidence('first'), evidence('second')
    first_composition = public_composition(first)
    composition = public_composition(second)
    correct = composition['assembly_directory']
    if bad == 'other_run':composition['assembly_directory'] = first_composition['assembly_directory']
    elif bad == 'outside':composition['assembly_directory'] = str(benchmark.repeat.ROOT.parent / 'foreign' / 'scene')
    elif bad == 'traversal':composition['assembly_directory'] = correct + '/../scene'
    elif bad == 'relative':composition['assembly_directory'] = second['relative_path'] + '/scene'
    elif bad == 'suffix':composition['assembly_directory'] = correct + '-foreign'
    elif bad == 'empty':composition['assembly_directory'] = ''
    elif bad == 'null':composition['assembly_directory'] = None
    elif bad == 'number':composition['assembly_directory'] = 7
    elif bad == 'missing':composition.pop('assembly_directory')
    result = compare(first, second)
    assert not result['valid'], result
    assert 'observed_wall_change_percent' not in result
    if bad != 'missing':assert 'assembly directory' in result['failures'][0]


@pytest.mark.parametrize('change', ['factory_hash', 'factory_contract', 'chassis_path',
                                  'wheel_path', 'mass', 'references', 'unexpected_field'])
def test_public_assembly_semantic_differences_remain_strict(change):
    first, second = evidence('first'), evidence('second')
    public_composition(first)
    composition = public_composition(second)
    if change == 'factory_hash':composition['factory_sha256'] = 'c'*64
    elif change == 'factory_contract':composition['factory_contract']['attributes'] = 172
    elif change == 'chassis_path':composition['paths']['chassis'] = '/World/Vehicles/different/Chassis'
    elif change == 'wheel_path':composition['wheel_paths'][0] = '/World/Vehicles/different/FrontLeftWheel'
    elif change == 'mass':second['scene_contract']['vehicle']['mass_kg'] = 1900.
    elif change == 'references':composition['references_not_instances'] = False
    elif change == 'unexpected_field':composition['new_physical_field'] = 'unmatched'
    result = compare(first, second)
    assert not result['valid'], result
    assert result['failures'] == ['Physical scene/assets/fleet contract differs']


@pytest.mark.parametrize('change',[
    'same_id','version','sources','runtime','collider','asset','fleet_cache','scene','control','live_target',
    'position','speed','truncated','runtime_error','cleanup','lifecycle','startup','pause','live_request',
    'live_population','visual_missing','visual_clock','visual_hidden','visual_error','native_schedule',
    'frame_missing','frame_clock','timing_forged','gpu_missing','gpu_guard','integrity','missing_source'])
def test_invalid_evidence_or_physical_mismatch_cannot_claim_cost(change):
    first,second=evidence('first'),evidence('second')
    if change=='same_id':second['run_id']='first'
    elif change=='version':second=evidence('second','editor-cost')
    elif change=='sources':second['manifest']['source_hashes']['fixture.py']='c'*64
    elif change=='runtime':second['manifest']['gpu_before']['driver']='different'
    elif change=='collider':second['scene_contract']['fleet']['collider_count']=1
    elif change=='asset':second['scene_contract']['fleet']['asset_hashes']['assets/block.usda']='c'*64
    elif change=='fleet_cache':second['scene_contract']['fleet']['cache_readback']=False
    elif change=='scene':second['scene_contract']['controller']['target']=1.
    elif change=='control':second['trajectory'][500]['steering_rad']=.1
    elif change=='live_target':second['trajectory'][500]['applied_ego_target_m_s']=.1
    elif change=='position':second['trajectory'][500]['position_m'][2]+=.0201
    elif change=='speed':second['trajectory'][500]['speed_m_s']+=.0201
    elif change=='truncated':second['trajectory'].pop()
    elif change=='runtime_error':second['summary']['runtime_error_lines']=['renderer failure']
    elif change=='cleanup':second['summary']['cleanup_verified']=False
    elif change=='lifecycle':second['lifecycle.json'][-1]['event']='closing'
    elif change=='startup':second['startup-readiness.json']['passed']=False
    elif change=='pause':second['summary']['display_qualification_exclusions']=['user_paused']
    elif change=='live_request':second['summary']['sidebar']['display_events']=[{'key':'ego_mph'}]
    elif change=='live_population':second['trajectory'][500]['active_background_count']=4
    elif change=='visual_missing':second['background_visual_checks'].pop()
    elif change=='visual_clock':second['background_visual_checks'][0]['native_target_time_s']=5
    elif change=='visual_hidden':second['background_visual_checks'][0]['visibility_matches_active']=False
    elif change=='visual_error':second['background_visual_checks'][0]['max_position_error_m']=.003
    elif change=='native_schedule':second['summary']['evidence_policy']['native_audit_ticks'].pop()
    elif change=='frame_missing':second['preview-timing.json'].pop()
    elif change=='frame_clock':second['preview-timing.json'][5]['lag_s']=999
    elif change=='timing_forged':second['summary']['preview_timing']['rtf']=5
    elif change=='gpu_missing':second['gpu-samples.json']=[]
    elif change=='gpu_guard':second['gpu-samples.json'][0]['used_mib']=900
    elif change=='integrity':second['load_errors']=['hash mismatch']
    elif change=='missing_source':second['manifest']['source_hashes'].pop('traffic/driver_control.py')
    result=compare(first,second)
    assert not result['valid'],(change,result)
    assert 'observed_wall_change_percent' not in result


def test_motion_tolerance_is_exactly_declared():
    first,second=evidence('first'),evidence('second')
    second['trajectory'][500]['position_m'][2]+=.01999
    second['trajectory'][500]['speed_m_s']+=.01999
    assert compare(first,second)['valid']


def test_missing_or_competing_load_invalidates_otherwise_valid_comparison():
    first,second=evidence('first'),evidence('second')
    for load in (None, {'before':QUIET,'after':{'available':True,'pids':[123],'count':1}},
                 {'before':{'available':False},'after':QUIET}):
        result=benchmark.compare_pair(first,second,first_load=LOAD,second_load=load)
        assert not result['valid'] and not result['load_qualified']


def test_hash_loader_requires_and_verifies_all_additional_evidence(tmp_path,monkeypatch):
    run=evidence('first')
    directory=materialize(tmp_path,monkeypatch,run)
    manifest=json.loads((directory/'manifest.json').read_text())
    for name in manifest['source_hashes']:
        path=directory/'source'/name
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'source')
        manifest['output_hashes'][path.relative_to(directory).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    for name in benchmark.EXTRA_FILES:
        target=directory/name
        target.write_text(json.dumps(run[name]))
        manifest['output_hashes'][name]=hashlib.sha256(target.read_bytes()).hexdigest()
    (directory/'manifest.json').write_text(json.dumps(manifest))
    loaded=benchmark._load_run(directory)
    assert not loaded['load_errors']
    assert benchmark._validate_run(loaded)
    (directory/'ui-event-timing.json').write_text('{}')
    with pytest.raises(ValueError,match='hash mismatch'):
        benchmark._load_run(directory)


def test_plan_never_launches(monkeypatch,capsys):
    monkeypatch.setattr(benchmark,'run_benchmark',lambda:pytest.fail('Unexpected native launch'))
    assert benchmark.main(['--plan'])==0
    assert json.loads(capsys.readouterr().out)['native_launches'] is False


def test_serial_runner_stops_at_competing_process_without_launch(tmp_path,monkeypatch):
    from scripts import demo_loop_showcase as demo
    output=tmp_path/'benchmark';output.mkdir()
    monkeypatch.setattr(benchmark,'run_package',lambda *args:(output,{'source_hashes':{'fixture.py':'a'*64}}))
    monkeypatch.setattr(benchmark,'_fingerprints',lambda paths:{'fixture.py':'a'*64})
    monkeypatch.setattr(benchmark,'kit_process_snapshot',lambda:dict(available=True,pids=[123],count=1))
    monkeypatch.setattr(benchmark,'desktop_load_snapshot',lambda:dict(available=False))
    monkeypatch.setattr(demo,'run_once',lambda *args:pytest.fail('Unexpected launch with active Kit'))
    _,result=benchmark.run_benchmark()
    index=json.loads((output/'index.json').read_text())
    assert index['status']=='stopped' and len(index['trials'])==1
    assert 'Competing' in index['stop_reason'] and not result['passed']
    assert (output/'manifest.json').is_file()


def test_serial_runner_stops_on_source_drift_and_preserves_completed_failure(tmp_path,monkeypatch):
    from scripts import demo_loop_showcase as demo
    from hivemind import launcher,showcase_setup
    output=tmp_path/'benchmark';output.mkdir()
    monkeypatch.setattr(benchmark,'run_package',lambda *args:(output,{'source_hashes':{'fixture.py':'a'*64}}))
    snapshots=iter([{'fixture.py':'a'*64},{'fixture.py':'b'*64}])
    monkeypatch.setattr(benchmark,'_fingerprints',lambda paths:next(snapshots))
    monkeypatch.setattr(benchmark,'kit_process_snapshot',lambda:QUIET)
    monkeypatch.setattr(benchmark,'desktop_load_snapshot',lambda:dict(available=False))
    monkeypatch.setattr(launcher,'load_config',lambda root:{})
    monkeypatch.setattr(showcase_setup,'check_showcase',lambda *args:('runtime',{}))
    calls=[]
    def run_once(cfg,runtime):
        calls.append(cfg)
        return dict(run_directory='outputs/loop_showcase/retained-failure',passed=False,
                    source_changed_during_run=['fixture.py'],cleanup_verified=True)
    monkeypatch.setattr(demo,'run_once',run_once)
    _,result=benchmark.run_benchmark()
    index=json.loads((output/'index.json').read_text())
    assert len(calls)==1 and index['trials'][0]['status']=='completed'
    assert index['trials'][0]['physical_passed'] is False
    assert 'Source drift' in result['stop_reason']


@pytest.mark.parametrize('physical_passed', [False, True])
def test_physical_failure_stops_but_timing_failure_can_continue(tmp_path,monkeypatch,physical_passed):
    from scripts import demo_loop_showcase as demo
    from hivemind import launcher,showcase_setup
    output=tmp_path/'benchmark';output.mkdir()
    monkeypatch.setattr(benchmark,'run_package',lambda *args:(output,{'source_hashes':{'fixture.py':'a'*64}}))
    monkeypatch.setattr(benchmark,'_fingerprints',lambda paths:{'fixture.py':'a'*64})
    monkeypatch.setattr(benchmark,'kit_process_snapshot',lambda:QUIET)
    monkeypatch.setattr(benchmark,'desktop_load_snapshot',lambda:dict(available=False))
    monkeypatch.setattr(launcher,'load_config',lambda root:{})
    monkeypatch.setattr(showcase_setup,'check_showcase',lambda *args:('runtime',{}))
    calls=[]
    def run_once(cfg,runtime):
        calls.append(cfg)
        return dict(run_directory='outputs/loop_showcase/fixture'+str(len(calls)),passed=physical_passed,
                    smooth_display_passed=False,source_changed_during_run=[],cleanup_verified=True)
    monkeypatch.setattr(demo,'run_once',run_once)
    benchmark.run_benchmark()
    assert len(calls)==(11 if physical_passed else 1)
    index=json.loads((output/'index.json').read_text())
    assert index['status']==('completed' if physical_passed else 'stopped')


def test_system_load_summary_uses_cpu_deltas_not_instantaneous_counter():
    before=dict(available=True,idle_100ns=100,kernel_100ns=200,user_100ns=100)
    after=dict(available=True,idle_100ns=200,kernel_100ns=400,user_100ns=300)
    summary=benchmark.desktop_load_summary(dict(desktop_before=before,desktop_after=after))
    assert summary['average_system_cpu_busy_percent']==75.
