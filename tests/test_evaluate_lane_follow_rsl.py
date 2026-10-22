"""CPU-only boundary, provenance and supervisor fixtures; no native execution."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from experiments import evaluate_lane_follow_rsl as evaluation


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    monkeypatch.setattr(evaluation, 'ROOT', tmp_path)
    policy = tmp_path/'policy.pt'
    policy.write_bytes(b'trusted synthetic policy, never deserialized')
    (tmp_path/'source.py').write_text('frozen source\n', encoding='utf-8')
    monkeypatch.setattr(evaluation, 'source_files', lambda: ['source.py'])
    return tmp_path, policy


def test_resolve_exact_default_matrix(workspace):
    root, policy = workspace
    job = evaluation.resolve_job(policy)
    assert job['routes'] == list(evaluation.ROUTES)
    assert job['seeds'] == [5101, 5102, 5103, 5104, 5105]
    assert job['policy_sha256'] == evaluation.digest(policy)
    assert job['training'] is False and job['observations'] == 22
    assert job['acceptance'] == evaluation.EXPECTED_ACCEPTANCE


@pytest.mark.parametrize('overrides', [
    {'routes': []}, {'routes': ['unknown']}, {'routes': ['left_r60']*2},
    {'seeds': []}, {'seeds': [True]}, {'seeds': [-1]}, {'seeds': [2**32]},
    {'seeds': [5101, 5101]}, {'seeds': [1.5]}, {'max_episodes': 14},
    {'max_episodes': True}, {'max_episodes': 16}, {'max_process_seconds': 601},
    {'max_process_seconds': 29}, {'max_process_seconds': 600.0},
])
def test_rejects_invalid_or_silently_truncated_matrix(workspace, overrides):
    with pytest.raises(ValueError):
        evaluation.resolve_job(workspace[1], **overrides)


def test_rejects_foreign_and_wrong_policy(workspace, tmp_path):
    root, policy = workspace
    wrong = root/'policy.zip'
    wrong.write_bytes(b'x')
    with pytest.raises(ValueError):
        evaluation.resolve_job(wrong)
    with pytest.raises(ValueError):
        evaluation.resolve_job(root/'missing.pt')
    policy.write_bytes(b'')
    with pytest.raises(ValueError):
        evaluation.resolve_job(policy)


@pytest.mark.parametrize('signed,raw,clipped', [
    ([.4, .6], [.2, .6, 0.], [.2, .6, 0.]),
    ([-.4, -.6], [-.2, 0., .6], [-.2, 0., .6]),
    ([4., -2.], [2., 0., 2.], [.5, 0., 1.]),
    ([-4., 3.], [-2., 3., 0.], [-.5, 1., 0.]),
])
def test_action_mapping_preserves_unclipped_evidence(signed, raw, clipped):
    assert evaluation.mapped_actions(signed) == (raw, clipped)


@pytest.mark.parametrize('values', [None, [], [1.], [1., 2., 3.], [True, 0.],
                                  [float('inf'), 0.], [0., float('nan')], ['1', 0.]])
def test_action_rejects_bad_shape_nonfinite_before_clipping(values):
    with pytest.raises(ValueError):
        evaluation.mapped_actions(values)


class Vector(list):
    def tolist(self):
        return list(self)

    def cpu(self):
        return self


def test_terminal_capture_preserves_pre_reset_sentinels_and_flags():
    extras = dict(lane_transition={'observation': [7.]*22, 'reason': 'time_limit'},
        final_terminated=[False], final_truncated=[True],
        final_observation={'policy': [Vector([7.]*22)]}, reset={'reset_spec': {'seed': 5101}})
    env = SimpleNamespace(extras=extras)
    returned = ([Vector([2.]*22)], [-1.], [True],
                [{'TimeLimit.truncated': True, 'terminal_observation': Vector([7.]*22)}])
    row = evaluation.capture_transition(env, returned, [0., 0.], [0., 0., 0.], [0., 0., 0.], .01)
    extras['lane_transition']['observation'][0] = 99.
    extras['reset']['reset_spec']['seed'] = 99
    assert row['observation'] == row['final_observation'] == row['sb3_terminal_observation'] == [7.]*22
    assert row['returned_observation'] == [2.]*22
    assert row['returned_done'] and row['returned_truncated'] and not row['returned_terminated']
    assert row['auto_reset']['reset_spec']['seed'] == 5101


def test_nonterminal_capture_omits_terminal_only_keys():
    env = SimpleNamespace(extras=dict(lane_transition={}, final_terminated=[False],
        final_truncated=[False], final_observation={'policy': [Vector([3.]*22)]}))
    row = evaluation.capture_transition(env, ([Vector([3.]*22)], [0.], [False],
        [{'TimeLimit.truncated': False}]), [0., 0.], [0., 0., 0.], [0., 0., 0.], 0.)
    assert 'auto_reset' not in row and 'sb3_terminal_observation' not in row


def worker_fixture(workspace):
    root, policy = workspace
    package = root/'outputs/lane_follow_rsl_eval/synthetic'
    output = package/'left_r60'
    output.mkdir(parents=True)
    job = evaluation.resolve_job(policy, ['left_r60'], [5101], 1)
    manifest = dict(status='running', config=job, source_hashes={'source.py': evaluation.digest(root/'source.py')})
    evaluation.atomic_json(package/'manifest.json', manifest)
    evaluation.atomic_json(package/'resolved-config.json', job)
    (package/'policy.pt').write_bytes(policy.read_bytes())
    return output, job, manifest


def test_worker_claim_exact_once_before_native_import(workspace):
    output, job, manifest = worker_fixture(workspace)
    assert evaluation.verify_worker(output) == (job, manifest)
    with pytest.raises(FileExistsError):
        evaluation.verify_worker(output)


@pytest.mark.parametrize('tamper', ['policy', 'original_policy', 'source', 'inventory', 'status', 'config', 'route'])
def test_worker_rejects_tampering_before_claim(workspace, tamper):
    output, job, manifest = worker_fixture(workspace)
    root, policy = workspace
    if tamper == 'policy':
        (output.parent/'policy.pt').write_bytes(b'changed')
    elif tamper == 'original_policy':
        policy.write_bytes(b'changed')
    elif tamper == 'source':
        (root/'source.py').write_bytes(b'changed')
    elif tamper == 'inventory':
        manifest['source_hashes'] = {}
    elif tamper == 'status':
        manifest['status'] = 'passed'
    elif tamper == 'config':
        job['observations'] = 21
    elif tamper == 'route':
        output = output.parent/'right_r60'
        output.mkdir()
    evaluation.atomic_json(output.parent/'manifest.json', manifest)
    evaluation.atomic_json(output.parent/'resolved-config.json', job)
    with pytest.raises(ValueError):
        evaluation.verify_worker(output)
    assert not (output/'worker-started.json').exists()


def test_atomic_write_failure_preserves_published_json(tmp_path):
    path = tmp_path/'result.json'
    evaluation.atomic_json(path, {'valid': True})
    original = path.read_bytes()
    with pytest.raises(ValueError):
        evaluation.atomic_json(path, {'bad': float('nan')})
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]


@pytest.mark.parametrize('behavior', ['returns', 'exits', 'raises'])
def test_native_result_published_before_potential_interpreter_exit(tmp_path, behavior):
    original = dict(passed=True, cases=[{'seed': 5101}], frozen_inference=True,
                    contact_reporting={'cleanup': {'passed': True}})
    class App:
        def close(self):
            saved = json.loads((tmp_path/'native-result.json').read_text())
            assert saved['cases'] == original['cases']
            assert saved['contact_reporting']['cleanup']['passed']
            assert saved['application_close_requested']
            assert not saved['application_close_returned']
            if behavior == 'exits':
                raise SystemExit(0)
            if behavior == 'raises':
                raise RuntimeError('shutdown failed')
    if behavior == 'exits':
        with pytest.raises(SystemExit):
            evaluation.close_application(App(), tmp_path, original)
    else:
        evaluation.close_application(App(), tmp_path, original)
    saved = json.loads((tmp_path/'native-result.json').read_text())
    assert saved['passed'] == (behavior != 'raises')
    assert saved['application_close_returned'] == (behavior == 'returns')


@pytest.fixture(scope='module')
def timeout_case():
    from tests.test_lane_follow_ppo_evaluation import fixture, ROUTES
    rows, initial, route = fixture(reason='time_limit', seed=5101)
    for row in rows:
        steering, throttle, brake = row['policy_action_raw']
        row['rsl_action_raw'] = [2*steering, throttle-brake]
    return initial, rows, route


def test_actual_reducer_accepts_valid_timeout_but_not_driving_success(timeout_case):
    initial, rows, route = timeout_case
    report = evaluation.assess_case(rows, initial, route, 5101)
    assert report['valid']
    assert report['outcome']['terminal_reason'] == 'time_limit'
    assert not report['outcome']['driving_success']


@pytest.mark.parametrize('tamper', ['reset', 'autoreset', 'signed', 'terminal', 'prefix'])
def test_actual_reducer_rejects_false_evidence(timeout_case, tamper):
    initial, rows, route = deepcopy(timeout_case)
    if tamper == 'reset':
        initial['reset_spec']['seed'] = 5102
    elif tamper == 'autoreset':
        rows[-1]['auto_reset']['reset_spec']['seed'] = 5102
    elif tamper == 'signed':
        rows[130]['rsl_action_raw'] = [.5, 1.]
    elif tamper == 'terminal':
        rows[-1]['sb3_terminal_observation'] = rows[-1]['returned_observation']
    elif tamper == 'prefix':
        rows = rows[:-1]
    assert not evaluation.assess_case(rows, initial, route, 5101)['valid']


def supervisor_fixture(workspace, monkeypatch, *, native_pass=True, driving=False, stop=None):
    root, policy = workspace
    import hivemind.launcher
    monkeypatch.setattr(hivemind.launcher, 'load_config', lambda root: {})
    monkeypatch.setattr(hivemind.launcher, 'isaac_runtime', lambda root, config: root/'fake-runtime')
    monkeypatch.setattr(evaluation.subprocess, 'check_output', lambda *a, **k: 'fake-commit')
    monkeypatch.setattr(evaluation, 'kit_process_snapshot', lambda: dict(available=True, count=0, pids=[]))
    monkeypatch.setattr(evaluation, 'gpu_sample', lambda: dict(used_mib=1, total_mib=10))
    class Monitor:
        rows = [dict(used_mib=1, total_mib=10)]
        errors = []
        def __enter__(self): return self
        def __exit__(self, *args): pass
    monkeypatch.setattr(evaluation, 'GpuMonitor', Monitor)
    launches = []
    class Child:
        def __init__(self, command, **kwargs):
            folder = Path(command[-1]); launches.append(folder.name)
            result = dict(passed=native_pass, route_id=folder.name, cases=[dict(
                seed=5101, assessment=dict(outcome=dict(driving_success=driving)))])
            evaluation.atomic_json(folder/'native-result.json', result)
            kwargs['stdout'].write('LAB_LANE_TASK_READY\n')
            self.returncode = 0 if native_pass else 1
        def poll(self): return self.returncode
    monkeypatch.setattr(evaluation.subprocess, 'Popen', Child)
    return launches


def test_supervisor_valid_failure_is_complete_evaluation(workspace, monkeypatch):
    launches = supervisor_fixture(workspace, monkeypatch)
    job = evaluation.resolve_job(workspace[1], ['left_r60', 'right_r60'], [5101], 2)
    assert evaluation.supervise(job) == 0
    assert launches == ['left_r60', 'right_r60']
    package = next((workspace[0]/'outputs/lane_follow_rsl_eval').iterdir())
    summary = json.loads((package/'summary.json').read_text())
    manifest = json.loads((package/'manifest.json').read_text())
    assert summary['episodes'] == 2 and summary['driving_successes'] == 0
    assert summary['training_started'] is False and summary['optimizer_steps'] == 0
    assert manifest['status'] == 'passed'
    assert all(evaluation.digest(package/name) == sha for name, sha in manifest['output_hashes'].items())
    assert all(evaluation.digest(workspace[0]/name) == sha for name, sha in manifest['log_hashes'].items())


def test_supervisor_native_failure_stops_remaining_routes(workspace, monkeypatch):
    launches = supervisor_fixture(workspace, monkeypatch, native_pass=False)
    job = evaluation.resolve_job(workspace[1], ['left_r60', 'right_r60'], [5101], 2)
    assert evaluation.supervise(job) == 1
    assert launches == ['left_r60']


def test_supervisor_refuses_competing_kit(workspace, monkeypatch):
    launches = supervisor_fixture(workspace, monkeypatch)
    monkeypatch.setattr(evaluation, 'kit_process_snapshot', lambda: dict(available=True, count=1, pids=[123]))
    job = evaluation.resolve_job(workspace[1], ['left_r60'], [5101], 1)
    assert evaluation.supervise(job) == 1
    assert not launches


def test_check_does_not_launch(workspace, monkeypatch, capsys):
    monkeypatch.chdir(workspace[0])
    monkeypatch.setattr(evaluation, 'supervise', lambda job: pytest.fail('check launched native'))
    assert evaluation.main(['--policy', str(workspace[1]), '--check']) == 0
    assert json.loads(capsys.readouterr().out)['training'] is False
