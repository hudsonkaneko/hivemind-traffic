"""Pilot authorization/package/runner checks; no Isaac process or optimizer runs."""
from copy import deepcopy
from contextlib import nullcontext
import json
from pathlib import Path
import tempfile
from types import ModuleType, SimpleNamespace
import sys

import pytest
import numpy as np

from experiments import lane_follow_ppo_protocol as protocol
from experiments import run_lane_follow_ppo as runner
from scripts import run_lane_follow_ppo as worker


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def workspace():
    parent = ROOT / 'outputs' / 'lab_lane_ppo' / 'review' / 'cpu-test-inputs'
    parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=parent) as name:
        yield Path(name)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')


def approval_fixture(root):
    for name in (protocol.APPROVAL_PATH, 'experiments/lane-follow-ppo-proposal.json',
                 'documentation/evidence/lab-lane-following-20261019.json'):
        target = root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / name).read_bytes())


def checkpoint_fixture(root, *, update=8):
    directory = root / 'outputs' / 'lab_lane_ppo' / 'training' / 'checkpoints' / f'update-{update:02d}'
    directory.mkdir(parents=True)
    # Deliberately invalid pickle/zip bytes: validation must not deserialize.
    for name in ('model.zip', 'vecnormalize.pkl', 'rng-state.pkl'):
        (directory / name).write_bytes(('opaque ' + name).encode())
    state = dict(completed_updates=update, control_steps=update * 2048,
                 proposal_sha256=protocol.PROPOSAL_SHA256, training_seed=2101,
                 optimizer_steps=update * 160, checkpoint_phase='after_completed_train_update')
    write_json(directory / 'state.json', state)
    reseal_checkpoint(directory)
    return directory


def reseal_checkpoint(directory):
    inventory = {name: protocol.digest(directory / name) for name in
                 ('model.zip', 'vecnormalize.pkl', 'rng-state.pkl', 'state.json')}
    write_json(directory / 'checkpoint.json', dict(schema_version=1, complete=True,
        exact_native_mid_episode_resume=False, files=inventory))


def test_approval_binds_historical_proposal_without_rewriting_it(workspace):
    approval_fixture(workspace)
    old = (workspace / 'experiments/lane-follow-ppo-proposal.json').read_bytes()
    result = protocol.load_protocol(workspace)
    assert result['status'] == 'APPROVED_SINGLE_PILOT'
    assert result['launch_enabled'] is True
    assert result['authorization']['authorization']['approved'] is True
    assert (workspace / 'experiments/lane-follow-ppo-proposal.json').read_bytes() == old
    assert json.loads(old)['status'] == 'UNAPPROVED_PROPOSAL_NOT_LAUNCHED'


@pytest.mark.parametrize('name', [protocol.APPROVAL_PATH,
    'experiments/lane-follow-ppo-proposal.json',
    'documentation/evidence/lab-lane-following-20261019.json'])
def test_changed_approval_or_reviewed_bytes_fail_closed(workspace, name):
    approval_fixture(workspace)
    path = workspace / name
    if name == protocol.APPROVAL_PATH:
        changed = json.loads(path.read_text(encoding='utf-8'))
        changed['authorization']['approved'] = False
        write_json(path, changed)
    else:
        path.write_bytes(path.read_bytes() + b' ')
    with pytest.raises(ValueError):
        protocol.load_protocol(workspace)


@pytest.mark.parametrize('update', [8, 16, 24, 32])
def test_checkpoint_identity_is_bound_to_intact_post_update_package(workspace, update):
    directory = checkpoint_fixture(workspace, update=update)
    identity = protocol.checkpoint_identity(directory, root=workspace, require_completed_run=False)
    assert identity == dict(update=update, control_steps=update * 2048,
                           model_sha256=protocol.digest(directory / 'model.zip'),
                           normalization_sha256=protocol.digest(directory / 'vecnormalize.pkl'))


@pytest.mark.parametrize('field,value', [
    ('completed_updates', 7), ('completed_updates', True), ('control_steps', 16383),
    ('training_seed', 2102), ('proposal_sha256', '0' * 64),
    ('optimizer_steps', 0), ('optimizer_steps', 1281), ('optimizer_steps', True),
    ('optimizer_steps', 100.), ('control_steps', 16384.), ('training_seed', 2101.),
    ('checkpoint_phase', 'before_optimizer_update')])
def test_checkpoint_semantic_mismatch_rejected_even_if_file_hashes_match(workspace, field, value):
    directory = checkpoint_fixture(workspace)
    state = json.loads((directory / 'state.json').read_text(encoding='utf-8'))
    state[field] = value
    write_json(directory / 'state.json', state)
    reseal_checkpoint(directory)
    with pytest.raises(ValueError):
        protocol.checkpoint_identity(directory, root=workspace, require_completed_run=False)


@pytest.mark.parametrize('mutation', ['corrupt', 'missing', 'extra', 'inventory', 'subdirectory'])
def test_checkpoint_missing_or_altered_files_rejected(workspace, mutation):
    directory = checkpoint_fixture(workspace)
    if mutation == 'corrupt':
        (directory / 'model.zip').write_bytes(b'changed')
    elif mutation == 'missing':
        (directory / 'vecnormalize.pkl').unlink()
    elif mutation == 'extra':
        (directory / 'unexpected.bin').write_bytes(b'extra')
    elif mutation == 'subdirectory':
        (directory / 'unexpected-directory').mkdir()
    else:
        write_json(directory / 'checkpoint.json', {'files': {'../model.zip': '0' * 64}})
    with pytest.raises(ValueError):
        protocol.checkpoint_identity(directory, root=workspace, require_completed_run=False)


def test_checkpoint_outside_declared_output_root_rejected(workspace):
    with pytest.raises(ValueError):
        protocol.checkpoint_identity(workspace, root=workspace)


def seal_run(run, manifest):
    manifest['output_hashes'] = {path.relative_to(run).as_posix(): protocol.digest(path)
        for path in run.rglob('*') if path.is_file() and path != run / 'manifest.json'}
    write_json(run / 'manifest.json', manifest)


def completed_training_fixture(root, monkeypatch):
    approval_fixture(root)
    approved = protocol.load_protocol(root)
    source_names = [protocol.APPROVAL_PATH, 'experiments/lane-follow-ppo-proposal.json',
                    'documentation/evidence/lab-lane-following-20261019.json', 'runtime.py']
    (root / 'runtime.py').write_text('synthetic = True\n', encoding='utf-8')
    monkeypatch.setattr(protocol, 'source_files', lambda: source_names)
    hashes = {name: protocol.digest(root / name) for name in source_names}
    pinned = approved['read_only_runtime_inventory']
    versions = {package: pinned[key] for package, key in (
        ('stable-baselines3', 'stable_baselines3'), ('gymnasium', 'gymnasium'),
        ('torch', 'torch'), ('numpy', 'numpy'), ('isaaclab', 'isaaclab'), ('isaaclab_rl', 'isaaclab_rl'))}
    runtime = dict(python_version=pinned['python'], torch_version=pinned['torch'],
                   isaaclab_git_commit=pinned['isaaclab_git'], isaaclab_git_dirty=False)
    assets = {'scene/world.usda': 'c' * 64}
    provenance = dict(proposal_sha256=protocol.PROPOSAL_SHA256, source_hashes=hashes,
        runtime=runtime, versions=versions, asset_hashes=assets, git_commit='d' * 40, git_status='synthetic')
    entries = []
    for update in (8, 16, 24, 32):
        directory = checkpoint_fixture(root, update=update)
        state = json.loads((directory / 'state.json').read_text(encoding='utf-8'))
        state.update(config=approved, provenance=provenance)
        write_json(directory / 'state.json', state)
        reseal_checkpoint(directory)
        entries.append(dict(path=directory.relative_to(root).as_posix(),
            identity=protocol.checkpoint_identity(directory, root=root, require_completed_run=False)))
    run = directory.parent.parent
    for name in source_names:
        target = run / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((root / name).read_bytes())
    smoke = dict(run='outputs/lab_lane_ppo/smoke', manifest_sha256='e' * 64)
    resolved = dict(mode='train', pilot_id=protocol.PILOT_ID, protocol=approved,
        route_id='left_r60', seeds=[2101], stage=None, controller='ppo',
        checkpoint_path=None, checkpoint_identity=None, smoke_evidence=smoke)
    claim_path = root / 'outputs/lab_lane_ppo' / (protocol.PILOT_ID + '.claim.json')
    write_json(claim_path, dict(pilot_id=protocol.PILOT_ID, output=run.relative_to(root).as_posix(),
        approval_sha256=protocol.digest(root / protocol.APPROVAL_PATH), smoke=smoke))
    manifest = dict(status='passed', config=resolved, source_hashes=hashes,
        training_claim=claim_path.relative_to(root).as_posix(), git_commit='d' * 40, git_status='synthetic')
    summary = dict(passed=True, mode='train', training_started=True, optimizer_steps=5120,
        training_state=dict(status='completed', stop_reason=None, control_steps=65536,
            completed_updates=32, optimizer_steps=5120,
            checkpoints=['update-08', 'update-16', 'update-24', 'update-32']),
        exit_code=0, stop_reason=None, source_unchanged=True, kit_after=dict(available=True, count=0),
        viewport_cleanup=dict(passed=True), contact_reporting=dict(cleanup=dict(passed=True)),
        scene_contract=dict(passed=True, static_hashes_unchanged=True, static_hashes=assets),
        unexpected_runtime_errors=[], usd_reference_warning_count=0, gpu_monitor_errors=[],
        runtime=runtime, versions=versions, checkpoints=entries)
    write_json(run / 'resolved-config.json', resolved)
    write_json(run / 'summary.json', summary)
    write_json(run / 'initial-scene-contract.json', dict(passed=True, static_hashes=assets))
    seal_run(run, manifest)
    return run / 'checkpoints/update-08', run, manifest


def test_checkpoint_requires_completed_training_inventory_by_default(workspace, monkeypatch):
    directory, _, _ = completed_training_fixture(workspace, monkeypatch)
    assert protocol.checkpoint_identity(directory, root=workspace)['update'] == 8


@pytest.mark.parametrize('field,value', [
    ('status', 'stopped'), ('control_steps', 65535), ('control_steps', 65536.),
    ('completed_updates', 31), ('optimizer_steps', 5121), ('stop_reason', 'deadline')])
def test_resealed_incomplete_training_cannot_publish_checkpoint(workspace, monkeypatch, field, value):
    directory, run, manifest = completed_training_fixture(workspace, monkeypatch)
    summary = json.loads((run / 'summary.json').read_text(encoding='utf-8'))
    summary['training_state'][field] = value
    write_json(run / 'summary.json', summary)
    seal_run(run, manifest)
    with pytest.raises(ValueError):
        protocol.checkpoint_identity(directory, root=workspace)


@pytest.mark.parametrize('mutation', ['artifact', 'extra', 'source', 'captured_source',
    'checkpoint_resealed', 'runtime', 'assets', 'approval_config', 'claim', 'cleanup'])
def test_completed_checkpoint_rejects_provenance_and_inventory_mismatches(workspace, monkeypatch, mutation):
    directory, run, manifest = completed_training_fixture(workspace, monkeypatch)
    if mutation == 'artifact':
        (run / 'initial-scene-contract.json').write_text('{}', encoding='utf-8')
    elif mutation == 'extra':
        (run / 'unrecorded.txt').write_text('extra', encoding='utf-8')
    elif mutation == 'source':
        (workspace / 'runtime.py').write_text('changed = True', encoding='utf-8')
    elif mutation == 'captured_source':
        (run / 'source/runtime.py').write_text('changed = True', encoding='utf-8')
        seal_run(run, manifest)
    elif mutation == 'checkpoint_resealed':
        (directory / 'model.zip').write_bytes(b'replaced checkpoint')
        reseal_checkpoint(directory)
    elif mutation in ('runtime', 'assets', 'approval_config'):
        state = json.loads((directory / 'state.json').read_text(encoding='utf-8'))
        if mutation == 'runtime':
            state['provenance']['runtime']['isaaclab_git_dirty'] = True
        elif mutation == 'assets':
            state['provenance']['asset_hashes']['scene/world.usda'] = 'f' * 64
        else:
            state['config']['budget']['training_seed'] = 2102
        write_json(directory / 'state.json', state)
        reseal_checkpoint(directory)
        seal_run(run, manifest)
    elif mutation == 'claim':
        claim = workspace / manifest['training_claim']
        data = json.loads(claim.read_text(encoding='utf-8'))
        data['output'] = 'outputs/lab_lane_ppo/another-run'
        write_json(claim, data)
    else:
        summary = json.loads((run / 'summary.json').read_text(encoding='utf-8'))
        summary['contact_reporting']['cleanup']['passed'] = False
        write_json(run / 'summary.json', summary)
        seal_run(run, manifest)
    with pytest.raises(ValueError):
        protocol.checkpoint_identity(directory, root=workspace)


@pytest.mark.parametrize('mode', ['smoke', 'train'])
def test_non_evaluation_modes_have_one_seed_and_fixed_route(mode):
    job = runner.resolve_job(mode)
    assert job['route_id'] == 'left_r60' and job['seeds'] == [2101]
    assert job['checkpoint_identity'] is None and job['visible'] is False
    assert job['max_process_seconds'] == (300 if mode == 'smoke' else 3600)


@pytest.mark.parametrize('kwargs', [
    {'route': 'straight100'}, {'stage': 'heldout'}, {'controller': 'scripted'},
    {'checkpoint': Path('unused')}, {'route': 'unknown'}])
def test_training_cannot_be_repurposed_by_evaluation_overrides(kwargs):
    with pytest.raises(ValueError):
        runner.resolve_job('train', **kwargs)


def test_evaluation_jobs_bind_predeclared_seeds_and_checkpoint(monkeypatch):
    identity = dict(update=8, control_steps=16384, model_sha256='a' * 64,
                    normalization_sha256='b' * 64)
    monkeypatch.setattr(runner, 'checkpoint_identity', lambda path: deepcopy(identity))
    path = ROOT / 'outputs/lab_lane_ppo/example/update-08'
    selection = runner.resolve_job('evaluate', stage='selection', checkpoint=path)
    heldout = runner.resolve_job('evaluate', stage='heldout', route='right_r60', checkpoint=path)
    scripted = runner.resolve_job('evaluate', stage='heldout', route='right_r60', controller='scripted')
    assert selection['seeds'] == [3101, 3102, 3103]
    assert selection['checkpoint_identity'] == identity
    assert heldout['seeds'] == scripted['seeds'] == [4101, 4102, 4103, 4104, 4105]
    assert heldout['max_process_seconds'] == scripted['max_process_seconds'] == 600
    assert scripted['checkpoint_identity'] is None


@pytest.mark.parametrize('kwargs', [
    {}, {'stage': 'selection'}, {'stage': 'selection', 'controller': 'scripted'},
    {'stage': 'selection', 'route': 'straight100'},
    {'stage': 'heldout', 'controller': 'scripted', 'checkpoint': Path('unused')}])
def test_incomplete_or_changed_evaluation_matrix_rejected(kwargs):
    with pytest.raises(ValueError):
        runner.resolve_job('evaluate', **kwargs)


def smoke_fixture(root, monkeypatch):
    directory = root / 'outputs/lab_lane_ppo/smoke'
    directory.mkdir(parents=True)
    source = root / 'source.py'
    source.write_text('frozen = True\n', encoding='utf-8')
    summary = dict(passed=True, mode='smoke', optimizer_steps=0, training_started=False)
    write_json(directory / 'summary.json', summary)
    (directory / 'roundtrip.txt').write_text('no optimizer execution', encoding='utf-8')
    manifest = dict(status='passed', source_hashes={'source.py': protocol.digest(source)},
        output_hashes={p.name: protocol.digest(p) for p in directory.iterdir()})
    write_json(directory / 'manifest.json', manifest)
    monkeypatch.setattr(runner, 'ROOT', root)
    monkeypatch.setattr(runner, 'source_files', lambda: ['source.py'])
    monkeypatch.setattr(runner, 'verify_sources', lambda m: protocol.verify_sources(m, root=root))
    return directory


def test_intact_zero_optimizer_smoke_is_required_before_training(workspace, monkeypatch):
    directory = smoke_fixture(workspace, monkeypatch)
    verified = runner.verify_smoke(directory)
    assert verified['run'] == 'outputs/lab_lane_ppo/smoke'
    assert verified['manifest_sha256'] == protocol.digest(directory / 'manifest.json')


@pytest.mark.parametrize('mutation', ['artifact', 'source', 'extra', 'training', 'optimizer'])
def test_smoke_drift_or_hidden_training_cannot_authorize_training(workspace, monkeypatch, mutation):
    directory = smoke_fixture(workspace, monkeypatch)
    if mutation == 'artifact':
        (directory / 'roundtrip.txt').write_text('altered', encoding='utf-8')
    elif mutation == 'source':
        (workspace / 'source.py').write_text('frozen = False\n', encoding='utf-8')
    elif mutation == 'extra':
        (directory / 'unrecorded').write_text('extra', encoding='utf-8')
    else:
        summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
        summary['training_started' if mutation == 'training' else 'optimizer_steps'] = True if mutation == 'training' else 1
        write_json(directory / 'summary.json', summary)
    with pytest.raises(ValueError):
        runner.verify_smoke(directory)


def test_atomic_one_run_claim_survives_second_launch_attempt(workspace, monkeypatch):
    approval_fixture(workspace)
    output = workspace / 'outputs/lab_lane_ppo/run-one'
    output.mkdir(parents=True)
    monkeypatch.setattr(runner, 'ROOT', workspace)
    relative = runner.claim_training(output, {'run': 'smoke', 'manifest_sha256': 'a' * 64})
    before = (workspace / relative).read_bytes()
    with pytest.raises(FileExistsError):
        runner.claim_training(output.parent / 'run-two', {'run': 'replacement'})
    assert (workspace / relative).read_bytes() == before
    assert json.loads(before)['output'] == 'outputs/lab_lane_ppo/run-one'


def supervisor_fixture(root, monkeypatch, *, unsafe=None, source_drift=False):
    """No real Popen, GPU query, process inventory or runtime can be reached."""
    output = root / 'outputs/lab_lane_ppo/run'
    output.mkdir(parents=True)
    runtime = root / 'fake-runtime/python.bat'
    runtime.parent.mkdir()
    (runtime.parent / 'VERSION').write_text('synthetic', encoding='utf-8')
    config = runner.resolve_job('smoke')
    state = dict(launched=[], stopped=[], finished=[])
    manifest = dict(status='running', source_hashes={})
    monkeypatch.setattr(runner, 'ROOT', root)
    monkeypatch.setattr(runner, 'source_files', lambda: [])
    monkeypatch.setattr(runner, 'load_config', lambda root: {})
    monkeypatch.setattr(runner, 'isaac_runtime', lambda root, config: runtime)
    monkeypatch.setattr(runner, 'resolve_job', lambda *args, **kwargs: deepcopy(config))
    monkeypatch.setattr(runner, 'run_package', lambda *args: (output, manifest))
    monkeypatch.setattr(runner, 'finish', lambda out, man, result: state['finished'].append(deepcopy(result)))
    monkeypatch.setattr(runner, 'kit_process_snapshot', lambda: dict(available=True, count=1 if unsafe == 'kit_before' else 0, pids=[]))
    monkeypatch.setattr(runner, 'gpu_sample', lambda: dict(used_mib=95 if unsafe == 'gpu_before' else 20, total_mib=100))
    monkeypatch.setattr(runner.shutil, 'disk_usage', lambda root: SimpleNamespace(free=0 if unsafe == 'disk' else 10 * 1024**3))
    def verify(_manifest):
        if source_drift:
            raise ValueError('Pilot sources changed: synthetic.py')
    monkeypatch.setattr(runner, 'verify_sources', verify)
    increment = 400 if unsafe == 'deadline' else 11 if unsafe in ('gpu_during', 'monitor') else 1
    clock = iter(range(0, 10000, increment))
    monkeypatch.setattr(runner.time, 'perf_counter', lambda: next(clock))
    monkeypatch.setattr(runner.time, 'sleep', lambda duration: None)
    class Monitor:
        rows = [dict(used_mib=95 if unsafe == 'gpu_during' else 20, total_mib=100)]
        errors = ['synthetic sampler failure'] if unsafe == 'monitor' else []
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False
    monkeypatch.setattr(runner, 'GpuMonitor', Monitor)
    class Child:
        returncode = None if unsafe in ('deadline', 'gpu_during', 'monitor') else 0
        def poll(self):
            return self.returncode
    def popen(command, **kwargs):
        state['launched'].append((command, kwargs))
        write_json(output / 'native-result.json', dict(passed=True, mode='smoke',
            training_started=False, optimizer_steps=0))
        return Child()
    monkeypatch.setattr(runner.subprocess, 'Popen', popen)
    def stop(child):
        state['stopped'].append(child)
        child.returncode = -15
    monkeypatch.setattr(runner, 'stop_child', stop)
    monkeypatch.setattr(sys, 'argv', ['supervisor', '--mode', 'smoke'])
    return state, output


@pytest.mark.parametrize('unsafe', ['kit_before', 'gpu_before', 'disk'])
def test_supervisor_rejects_resource_interference_before_native_launch(workspace, monkeypatch, unsafe):
    state, _ = supervisor_fixture(workspace, monkeypatch, unsafe=unsafe)
    assert runner.main() == 1
    assert not state['launched'] and not state['stopped']
    assert not state['finished'][0]['passed']
    assert 'supervisor_error' in state['finished'][0]


def test_supervisor_accepts_clean_native_result_and_records_frozen_sources(workspace, monkeypatch):
    state, _ = supervisor_fixture(workspace, monkeypatch)
    assert runner.main() == 0
    assert len(state['launched']) == 1 and not state['stopped']
    command, kwargs = state['launched'][0]
    assert command[1] == str(workspace / 'scripts/run_lane_follow_ppo.py')
    assert kwargs['cwd'] == workspace
    assert state['finished'][0]['source_unchanged'] is True
    assert state['finished'][0]['optimizer_steps'] == 0


def test_completed_native_result_does_not_hide_source_drift(workspace, monkeypatch):
    state, _ = supervisor_fixture(workspace, monkeypatch, source_drift=True)
    assert runner.main() == 1
    assert len(state['launched']) == 1
    assert state['finished'][0]['source_unchanged'] is False
    assert not state['finished'][0]['passed']


@pytest.mark.parametrize('unsafe', ['deadline', 'gpu_during', 'monitor'])
def test_supervisor_persists_stop_signal_and_terminates_only_its_child(workspace, monkeypatch, unsafe):
    state, output = supervisor_fixture(workspace, monkeypatch, unsafe=unsafe)
    assert runner.main() == 1
    assert len(state['launched']) == len(state['stopped']) == 1
    assert (output / 'STOP').read_text(encoding='utf-8')
    assert state['finished'][0]['stop_reason'] == {
        'deadline': 'Process deadline reached', 'gpu_during': 'Whole-GPU memory stop',
        'monitor': 'GPU monitor failed'}[unsafe]
    assert not state['finished'][0]['passed']


class CpuArray(np.ndarray):
    """Only the read-only tensor boundary used by record/prediction helpers."""
    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self)


def cpu_array(value):
    return np.asarray(value).view(CpuArray)


@pytest.mark.parametrize('terminal', [False, True])
def test_raw_transition_copy_keeps_terminal_and_reset_observations_separate(terminal):
    terminal_obs = np.full(22, 9., dtype=np.float32)
    returned_obs = np.full((1, 22), -3. if terminal else 9., dtype=np.float32)
    extras = dict(lane_transition={'physics_samples': [{'tick': 1}]},
        final_terminated=[terminal], final_truncated=[False],
        final_observation={'policy': cpu_array([terminal_obs])},
        reset={'episode_serial': 2, 'observation': [-3.] * 22})
    infos = [{'TimeLimit.truncated': False}]
    if terminal:
        infos[0]['terminal_observation'] = terminal_obs.copy()
    returned = (returned_obs, np.array([1.25]), np.array([terminal]), infos)
    row = worker.transition_record(SimpleNamespace(extras=extras), returned,
        [[.8, -.1, 1.2]], [[.5, 0., 1.]], .005)
    assert row['returned_observation'] == returned_obs[0].tolist()
    assert row['final_observation'] == terminal_obs.tolist()
    assert row['returned_reward'] == 1.25 and row['returned_done'] is terminal
    assert row['policy_action_raw'] == [.8, -.1, 1.2]
    assert row['env_action'] == [.5, 0., 1.]
    if terminal:
        assert row['sb3_terminal_observation'] == terminal_obs.tolist()
        assert row['auto_reset']['episode_serial'] == 2
        infos[0]['terminal_observation'][:] = 100
        extras['reset']['observation'][0] = 500
        assert row['sb3_terminal_observation'][0] == 9.
        assert row['auto_reset']['observation'][0] == -3.
    else:
        assert 'sb3_terminal_observation' not in row and 'auto_reset' not in row
    extras['lane_transition']['physics_samples'][0]['tick'] = 99
    returned_obs[:] = -500
    assert row['physics_samples'][0]['tick'] == 1
    assert row['returned_observation'][0] != -500


def prediction_fixture(monkeypatch, values, *, mismatch=False):
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(no_grad=nullcontext))
    action = cpu_array(values)
    low, high = np.array([-.5, 0., 0.]), np.array([.5, 1., 1.])
    policy = SimpleNamespace(obs_to_tensor=lambda value: (value, False),
        get_distribution=lambda tensor: SimpleNamespace(get_actions=lambda deterministic: action))
    model = SimpleNamespace(policy=policy, action_space=SimpleNamespace(low=low, high=high),
        predict=lambda obs, deterministic: (np.zeros((1, 3)) if mismatch else np.clip(action, low, high), None))
    normalizer = SimpleNamespace(normalize_obs=lambda value: value.copy())
    return model, normalizer


def test_evaluation_records_unclipped_mean_and_matches_predict_without_optimizer(monkeypatch):
    model, normalizer = prediction_fixture(monkeypatch, [[.8, -.1, 1.2]])
    raw, clipped = worker.deterministic_action(model, normalizer, np.zeros((1, 22)))
    np.testing.assert_array_equal(raw, [[.8, -.1, 1.2]])
    np.testing.assert_array_equal(clipped, [[.5, 0., 1.]])


@pytest.mark.parametrize('values', [[[np.inf, 0., 0.]], [[np.nan, 0., 0.]], [0., 0., 0.]])
def test_nonfinite_or_malformed_policy_action_rejected_before_box_clip(monkeypatch, values):
    model, normalizer = prediction_fixture(monkeypatch, values)
    with pytest.raises(RuntimeError, match='malformed/nonfinite'):
        worker.deterministic_action(model, normalizer, np.zeros((1, 22)))


def test_evaluation_detects_predict_and_recorded_action_disagreement(monkeypatch):
    model, normalizer = prediction_fixture(monkeypatch, [[.8, -.1, 1.2]], mismatch=True)
    with pytest.raises(RuntimeError, match='differs'):
        worker.deterministic_action(model, normalizer, np.zeros((1, 22)))


class NativeBoundaryReached(BaseException):
    """Stop the test at a fake AppLauncher; no runtime import/launch occurs."""


def worker_fixture(root, monkeypatch):
    approval_fixture(root)
    base_job = runner.resolve_job('train')
    smoke = dict(run='outputs/lab_lane_ppo/smoke', manifest_sha256='e' * 64)
    job = dict(deepcopy(base_job), smoke_evidence=smoke)
    output = root / 'outputs/lab_lane_ppo/train-worker'
    output.mkdir(parents=True)
    claim_path = root / 'outputs/lab_lane_ppo' / (protocol.PILOT_ID + '.claim.json')
    claim = dict(pilot_id=protocol.PILOT_ID, output=output.relative_to(root).as_posix(),
                 smoke=smoke, approval_sha256=protocol.digest(root / protocol.APPROVAL_PATH))
    write_json(claim_path, claim)
    manifest = dict(config=job, status='running', source_hashes={'runtime.py': 'a' * 64},
                    training_claim=claim_path.relative_to(root).as_posix())
    write_json(output / 'manifest.json', manifest)
    write_json(output / 'resolved-config.json', job)
    monkeypatch.setattr(worker, 'ROOT', root)
    monkeypatch.setattr(worker, 'load_protocol', lambda: deepcopy(job['protocol']))
    monkeypatch.setattr(worker, 'source_files', lambda: ['runtime.py'])
    monkeypatch.setattr(worker, 'verify_sources', lambda manifest: None)
    monkeypatch.setattr(worker, 'process_sample', lambda: {'synthetic': True})
    monkeypatch.setattr(runner, 'resolve_job', lambda *args, **kwargs: deepcopy(base_job))
    monkeypatch.setattr(runner, 'verify_smoke', lambda directory: deepcopy(smoke))
    reached = []
    def fake_launcher(*args, **kwargs):
        reached.append(True)
        raise NativeBoundaryReached()
    fake_lab, fake_app = ModuleType('isaaclab'), ModuleType('isaaclab.app')
    fake_app.AppLauncher = fake_launcher
    monkeypatch.setitem(sys.modules, 'isaaclab', fake_lab)
    monkeypatch.setitem(sys.modules, 'isaaclab.app', fake_app)
    monkeypatch.setattr(sys, 'argv', ['worker', '--output', str(output)])
    return output, manifest, claim_path, reached


def test_worker_consumes_claim_once_before_native_boundary(workspace, monkeypatch):
    output, _, _, reached = worker_fixture(workspace, monkeypatch)
    with pytest.raises(NativeBoundaryReached):
        worker.main()
    assert reached == [True]
    assert (output / 'worker-started.json').is_file()
    # Even removing a failed result cannot consume the same claim twice.
    (output / 'native-result.json').unlink()
    with pytest.raises(FileExistsError):
        worker.main()
    assert reached == [True]


@pytest.mark.parametrize('mutation', ['manifest', 'sources', 'claim', 'parameters'])
def test_worker_refuses_changed_job_or_unowned_claim_before_native_boundary(workspace, monkeypatch, mutation):
    output, manifest, claim_path, reached = worker_fixture(workspace, monkeypatch)
    if mutation == 'manifest':
        manifest['status'] = 'passed'
        write_json(output / 'manifest.json', manifest)
    elif mutation == 'sources':
        manifest['source_hashes'] = {}
        write_json(output / 'manifest.json', manifest)
    elif mutation == 'claim':
        claim = json.loads(claim_path.read_text(encoding='utf-8'))
        claim['output'] = 'outputs/lab_lane_ppo/other'
        write_json(claim_path, claim)
    else:
        job = deepcopy(manifest['config'])
        job['route_id'] = 'right_r60'
        manifest['config'] = job
        write_json(output / 'resolved-config.json', job)
        write_json(output / 'manifest.json', manifest)
    with pytest.raises(ValueError):
        worker.main()
    assert not reached
