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
    for policy in ('ppo', 'scripted'):
        for route in ('straight100', 'left_r60', 'right_r60'):
            directory = root / 'outputs/lab_lane_ppo' / f'{policy}-{route}'
            directory.mkdir()
            packages.append(dict(path=directory.relative_to(root).as_posix(), manifest_sha256='c' * 64))
            configs[directory / 'resolved-config.json'] = dict(controller=policy, route_id=route)
            if policy == 'ppo' and route == 'left_r60':
                (directory / 'seed-4101-trajectory.json').write_text('[]', encoding='utf-8')
                (directory / 'seed-4101-reset.json').write_text('{}', encoding='utf-8')
    saved = dict(kind='heldout', report=dict(selected_checkpoint=selected), packages=packages,
                 selection=dict(path='outputs/lab_lane_ppo/reports/selection.json'))
    report_path.write_text(json.dumps(saved), encoding='utf-8')
    calls = []
    def assemble(training_path, selection_path, package_paths):
        calls.append((training_path, selection_path, package_paths))
        return deepcopy(saved)
    monkeypatch.setattr(preview, 'ROOT', root)
    monkeypatch.setattr(preview, '_read', lambda path: deepcopy(saved if Path(path) == report_path else configs[Path(path)]))
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
