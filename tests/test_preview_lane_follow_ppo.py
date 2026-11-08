"""Synthetic screenshot/evidence boundaries; no Kit, model or optimizer exists."""
from copy import deepcopy
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import zlib

import pytest

from experiments import preview_lane_follow_ppo as preview
from scripts import preview_lane_follow_ppo as worker

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def workspace():
    parent = ROOT / 'outputs/lab_lane_ppo/review/preview-test-inputs'
    parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=parent) as name:
        yield Path(name)


def capture_fixture(*, complete_after=2, result=True, exception=None):
    events, now = [], [0.]
    class Future:
        rendered = 0
        cancelled = False
        def done(self):
            return complete_after is not None and self.rendered >= complete_after
        def result(self):
            events.append('result')
            if exception:
                raise exception
            return result
        def cancel(self):
            self.cancelled = True
            events.append('cancel')
    future = Future()
    token = object()
    class Capture:
        def wait_for_result(self, *, completion_frames):
            assert completion_frames == 30
            events.append('wait_for_result')
            return token
    def schedule(value):
        assert value is token
        events.append('schedule')
        return future
    def render():
        events.append('render')
        future.rendered += 1
        now[0] += .1
    return Capture(), future, events, dict(render=render, guard=lambda: events.append('guard'),
        flush=lambda: events.append('flush'), clock=lambda: now[0], schedule=schedule)


def test_capture_waits_for_native_completion_then_flushes_before_return():
    capture, future, events, hooks = capture_fixture()
    worker.wait_for_capture(capture, **hooks)
    assert future.rendered == 2 and not future.cancelled
    assert events[:2] == ['wait_for_result', 'schedule']
    assert events.index('result') < events.index('flush')
    assert events[-1] == 'guard'


@pytest.mark.parametrize('result,exception', [(False, None), (True, RuntimeError('native write failed'))])
def test_capture_failure_does_not_flush_or_claim_completion(result, exception):
    capture, future, events, hooks = capture_fixture(result=result, exception=exception)
    with pytest.raises(RuntimeError):
        worker.wait_for_capture(capture, **hooks)
    assert 'flush' not in events and not future.cancelled


def test_capture_timeout_cancels_pending_native_future():
    capture, future, events, hooks = capture_fixture(complete_after=None)
    with pytest.raises(TimeoutError):
        worker.wait_for_capture(capture, timeout_s=.2, **hooks)
    assert future.rendered == 2 and future.cancelled
    assert 'flush' not in events and 'result' not in events


def test_supervisor_stop_cancels_pending_capture_before_more_rendering():
    capture, future, events, hooks = capture_fixture(complete_after=None)
    def stop():
        raise RuntimeError('supervisor requested stop')
    hooks['guard'] = stop
    with pytest.raises(RuntimeError, match='supervisor'):
        worker.wait_for_capture(capture, **hooks)
    assert future.cancelled and future.rendered == 0 and 'flush' not in events


def chunk(kind, data):
    return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)


def png_bytes():
    """One synthetic RGBA pixel; never edits a captured evidence image."""
    header = struct.pack('>IIBBBBB', 1, 1, 8, 6, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(b'\0\1\2\3\xff')) + chunk(b'IEND', b'')


def test_complete_png_has_verified_framing_dimensions_and_content_hash(workspace):
    path = workspace / 'synthetic.png'
    payload = png_bytes()
    path.write_bytes(payload)
    result = worker.verify_capture_png(path, expected_dimensions=(1, 1))
    assert result == dict(width=1, height=1, bytes=len(payload), sha256=preview.digest(path))
    assert path.read_bytes() == payload


@pytest.mark.parametrize('mutation', ['signature', 'crc', 'truncated_chunk', 'missing_iend', 'trailing_data', 'wrong_dimensions'])
def test_incomplete_or_corrupt_png_cannot_receive_capture_provenance(workspace, mutation):
    payload = bytearray(png_bytes())
    dimensions = (1, 1)
    if mutation == 'signature':
        payload[0] = 0
    elif mutation == 'crc':
        payload[20] ^= 1
    elif mutation == 'truncated_chunk':
        payload = payload[:-5]
    elif mutation == 'missing_iend':
        payload = payload[:-12]
    elif mutation == 'trailing_data':
        payload.extend(b'extra')
    else:
        dimensions = (1440, 900)
    path = workspace / 'invalid.png'
    path.write_bytes(payload)
    with pytest.raises(ValueError):
        worker.verify_capture_png(path, expected_dimensions=dimensions)


def preview_fixture(root, monkeypatch):
    training = root / 'outputs/lab_lane_ppo/training'
    training.mkdir(parents=True)
    report_path = root / 'outputs/lab_lane_ppo/reports/heldout.json'
    report_path.parent.mkdir()
    selected = dict(update=16, control_steps=32768, model_sha256='a' * 64, normalization_sha256='b' * 64)
    configs, packages = {}, []
    def manifest(directory):
        path = directory / 'manifest.json'
        path.write_text(json.dumps({'fixture': directory.name}), encoding='utf-8')
        return dict(path=directory.relative_to(root).as_posix(), manifest_sha256=preview.digest(path))
    training_ref = manifest(training)
    selection_packages = []
    for update in (8, 16, 24, 32):
        directory = root / 'outputs/lab_lane_ppo' / f'selection-{update}'
        directory.mkdir()
        selection_packages.append(manifest(directory))
    for policy in ('ppo', 'scripted'):
        for route in ('straight100', 'left_r60', 'right_r60'):
            directory = root / 'outputs/lab_lane_ppo' / f'{policy}-{route}'
            directory.mkdir()
            packages.append(manifest(directory))
            configs[directory / 'resolved-config.json'] = dict(controller=policy, route_id=route)
            if policy == 'ppo' and route == 'left_r60':
                (directory / 'seed-4101-trajectory.json').write_text('[]', encoding='utf-8')
                (directory / 'seed-4101-reset.json').write_text('{}', encoding='utf-8')
    selection_path = report_path.with_name('selection.json')
    selection = dict(kind='selection', report=dict(valid=True, selected_checkpoint=selected),
                     training=training_ref, packages=selection_packages)
    selection_path.write_text(json.dumps(selection), encoding='utf-8')
    saved = dict(kind='heldout', report=dict(valid=True, selected_checkpoint=selected), packages=packages,
                 training=training_ref, selection=dict(path=selection_path.relative_to(root).as_posix(),
                     sha256=preview.digest(selection_path)))
    report_path.write_text(json.dumps(saved), encoding='utf-8')
    calls = []
    def assemble(training_path, selection_path, package_paths):
        calls.append((training_path, selection_path, package_paths))
        return deepcopy(saved)
    monkeypatch.setattr(preview, 'ROOT', root)
    def read(path):
        path = Path(path)
        return deepcopy(saved if path == report_path else selection if path == selection_path else configs[path])
    monkeypatch.setattr(preview, '_read', read)
    monkeypatch.setattr(preview, 'assemble_held_out', assemble)
    monkeypatch.setattr(preview, 'checkpoint_identity', lambda directory: deepcopy(selected))
    return training, report_path, saved, configs, calls


def test_preview_recomputes_report_and_uses_only_fixed_selected_case(workspace, monkeypatch):
    training, path, saved, _, calls = preview_fixture(workspace, monkeypatch)
    job = preview.resolve_preview(training, path)
    assert calls == [(training, workspace / saved['selection']['path'],
                      [workspace / p['path'] for p in saved['packages']])]
    assert job['checkpoint_identity'] == saved['report']['selected_checkpoint']
    assert job['checkpoint_path'].endswith('/checkpoints/update-16')
    assert job['route_id'] == 'left_r60' and job['seed'] == 4101
    assert job['max_control_steps'] == 2100 and job['max_process_seconds'] == 300
    assert job['gpu_memory_fraction_stop'] == .9 and job['optimizer_steps'] == 0
    assert job['capture_sim_seconds'] == [0, 2, 5, 10, 15, 20, 25, 30, 35]
    assert job['excluded_from_selection_and_evaluation_counts'] is True
    assert 'post-auto-reset' in job['terminal_capture_rule']
    assert len(job['frozen_report_and_manifest_hashes']) == 13


def test_native_job_rechecks_exact_frozen_evidence_without_reducing_in_new_python(workspace, monkeypatch):
    training, path, _, _, calls = preview_fixture(workspace, monkeypatch)
    job = preview.resolve_preview(training, path)
    before = deepcopy(job)
    def forbidden(*args, **kwargs):
        raise AssertionError('Native worker must not numerically reassemble reports')
    monkeypatch.setattr(preview, 'assemble_held_out', forbidden)
    monkeypatch.setattr(preview, 'resolve_preview', forbidden)
    assert preview.verify_preview_job(job) == before
    assert job == before and len(calls) == 1
    assert 'without cross-Python numerical re-reduction' in job['verification_contract']


@pytest.mark.parametrize('mutation', [
    'heldout_report_bytes', 'selection_report_bytes', 'training_manifest',
    'selection_manifest', 'heldout_manifest', 'checkpoint_identity',
    'reference_trajectory', 'reference_reset', 'seed', 'route_id', 'physics_hz',
    'control_hz', 'optimizer_steps', 'max_control_steps', 'max_process_seconds',
    'gpu_memory_fraction_stop', 'captures', 'missing_bound_hash', 'outside_training',
    'outside_report', 'tiny_report_numeric_change',
])
def test_native_job_rejects_any_changed_bound_input_or_fixed_parameter(workspace, monkeypatch, mutation):
    training, path, saved, _, _ = preview_fixture(workspace, monkeypatch)
    job = preview.resolve_preview(training, path)
    def forbidden(*args, **kwargs):
        raise AssertionError('Unexpected numeric re-reduction')
    monkeypatch.setattr(preview, 'assemble_held_out', forbidden)
    append_path = None
    if mutation == 'heldout_report_bytes':
        append_path = path
    elif mutation == 'selection_report_bytes':
        append_path = workspace / saved['selection']['path']
    elif mutation == 'training_manifest':
        append_path = training / 'manifest.json'
    elif mutation == 'selection_manifest':
        append_path = workspace / 'outputs/lab_lane_ppo/selection-8/manifest.json'
    elif mutation == 'heldout_manifest':
        append_path = workspace / saved['packages'][0]['path'] / 'manifest.json'
    elif mutation == 'checkpoint_identity':
        monkeypatch.setattr(preview, 'checkpoint_identity', lambda directory: dict(update=8))
    elif mutation == 'reference_trajectory':
        append_path = workspace / job['reference_run'] / 'seed-4101-trajectory.json'
    elif mutation == 'reference_reset':
        append_path = workspace / job['reference_run'] / 'seed-4101-reset.json'
    elif mutation == 'captures':
        job['capture_sim_seconds'] = [0, 5]
    elif mutation == 'missing_bound_hash':
        job['frozen_report_and_manifest_hashes'].pop(next(iter(job['frozen_report_and_manifest_hashes'])))
    elif mutation == 'outside_training':
        job['training'] = str(workspace.parent)
    elif mutation == 'outside_report':
        job['heldout_report'] = str(workspace.parent / 'heldout.json')
    elif mutation == 'tiny_report_numeric_change':
        changed = deepcopy(saved)
        changed['report']['metric'] = 1e-16
        path.write_text(json.dumps(changed), encoding='utf-8')
    else:
        job[mutation] = {'seed':4102, 'route_id':'right_r60', 'physics_hz':60,
            'control_hz':30, 'optimizer_steps':1, 'max_control_steps':2200,
            'max_process_seconds':600, 'gpu_memory_fraction_stop':.99}[mutation]
    if append_path is not None:
        with append_path.open('a', encoding='utf-8') as stream:
            stream.write('\n')
    with pytest.raises(ValueError):
        preview.verify_preview_job(job)


@pytest.mark.parametrize('mutation', ['report_kind', 'recomputed_report', 'checkpoint', 'duplicate_reference'])
def test_preview_refuses_changed_selection_or_ambiguous_reference(workspace, monkeypatch, mutation):
    training, path, saved, configs, _ = preview_fixture(workspace, monkeypatch)
    if mutation == 'report_kind':
        saved['kind'] = 'selection'
    elif mutation == 'recomputed_report':
        monkeypatch.setattr(preview, 'assemble_held_out', lambda *args: dict(changed=True))
    elif mutation == 'checkpoint':
        monkeypatch.setattr(preview, 'checkpoint_identity', lambda directory: dict(update=8))
    else:
        configs[workspace / 'outputs/lab_lane_ppo/scripted-left_r60/resolved-config.json']['controller'] = 'ppo'
    with pytest.raises(ValueError):
        preview.resolve_preview(training, path)


def trace_fixture():
    row = dict(tick=2, physics_state={'position_m': [0., 0., .8]},
        physics_samples=[dict(tick=1, contacts=[]), dict(tick=2, contacts=[])],
        policy_action_raw=[.7, .2, -.1], env_action=[.5, .2, 0.], requested_action=[.5, .2, 0.],
        observation=[0.] * 22, route_state={'progress_m': 0.}, reward=0., reward_components={'terminal': 0.},
        terminated=False, truncated=False, reason='settling', success=False,
        episode_serial=1, step_wall_seconds=.01)
    reset = dict(reset_spec={'seed': 4101, 'episode_index': 0}, physics_state=deepcopy(row['physics_state']))
    return [row], reset


def test_replay_comparison_ignores_wall_and_episode_ids_without_changing_inputs():
    reference, reset = trace_fixture()
    actual = deepcopy(reference)
    actual[0].update(episode_serial=99, step_wall_seconds=100.)
    before = deepcopy(actual)
    result = preview.compare_trace(actual, reset, reference, reset)
    assert result['exact_selected_fields'] and actual == before
    assert result['performance_comparison'] is False and result['changes_heldout_outcomes'] is False


@pytest.mark.parametrize('mutation', ['physics', 'action', 'reward', 'decision', 'length', 'reset', 'contact_metadata'])
def test_replay_comparison_retains_real_differences_without_reranking(mutation):
    reference, reset = trace_fixture()
    actual, actual_reset = deepcopy(reference), deepcopy(reset)
    if mutation == 'physics':
        actual[0]['physics_state']['position_m'][0] += 1e-10
    elif mutation == 'action':
        actual[0]['policy_action_raw'][0] += .1
    elif mutation == 'reward':
        actual[0]['reward'] = 1.
    elif mutation == 'decision':
        actual[0]['success'] = True
    elif mutation == 'length':
        actual.append(deepcopy(actual[0]))
    elif mutation == 'reset':
        actual_reset['reset_spec']['seed'] = 4102
    else:
        actual[0]['physics_samples'][0]['contacts'] = [{'stage_id': 2}]
    result = preview.compare_trace(actual, actual_reset, reference, reset)
    assert not result['exact_selected_fields'] and not result['changes_heldout_outcomes']
    assert 'process-specific' in result['contact_metadata_caveat']


def test_helper_imports_do_not_load_kit_or_training_libraries():
    # A fresh project-Python process avoids dependence on other test modules'
    # imports. This is not the external Isaac Python executable (kit.exe).
    program = (
        'import sys,json; import experiments.preview_lane_follow_ppo; '
        'import scripts.preview_lane_follow_ppo; '
        'print(json.dumps([n for n in ("isaaclab.app","omni.kit.app",'
        '"stable_baselines3","torch") if n in sys.modules]))')
    result = subprocess.run([sys.executable, '-B', '-c', program], cwd=ROOT,
                            capture_output=True, text=True, check=True, timeout=10)
    assert json.loads(result.stdout) == []
