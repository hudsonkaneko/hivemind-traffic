"""Supervisor tests use fake children only; never start Isaac or training."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from experiments import verify_sensor_perception as runner


def forbidden(*args, **kwargs):
    raise AssertionError('Native process must not launch in this test')


def setup_check(monkeypatch, argv):
    monkeypatch.setattr('sys.argv', ['verify_sensor_perception', *argv])
    monkeypatch.setattr(runner, 'load_config', lambda root: {})
    monkeypatch.setattr(runner, 'isaac_runtime', lambda root, config: Path('fake-runtime'))


def test_check_is_read_only_and_declares_no_training(monkeypatch, capsys):
    setup_check(monkeypatch, ['--check'])
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(runner, 'run_package', forbidden)
    assert runner.main() == 0
    config = json.loads(capsys.readouterr().out)
    assert config['training'] is False
    assert 0 < config['duration_s'] <= 15
    assert 0 < config['target_speed_m_s'] <= 2
    assert 0 < config['max_process_seconds'] <= 600


def test_cli_does_not_accept_training_override(monkeypatch):
    setup_check(monkeypatch, ['--train'])
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2


def test_windows_existing_kit_blocks_before_creating_run(monkeypatch):
    setup_check(monkeypatch, [])
    monkeypatch.setattr(runner, 'os', SimpleNamespace(name='nt'))
    monkeypatch.setattr(runner.subprocess, 'check_output', lambda *a, **k: '12345\n')
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(runner, 'run_package', forbidden)
    with pytest.raises(RuntimeError, match='Another Kit process'):
        runner.main()


def fake_run(monkeypatch, tmp_path, *, exit_code=0, report=None, timeout=False, mutate=False):
    setup_check(monkeypatch, [])
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    monkeypatch.setattr(runner, 'os', SimpleNamespace(name='posix'))
    output = tmp_path/'outputs'/'test-run'
    output.mkdir(parents=True)
    source = tmp_path/'source.py'
    source.write_text('original')
    manifest = {'source_hashes': {'source.py': hashlib.sha256(source.read_bytes()).hexdigest()}}
    if report is not None:
        (output/'probe-result.json').write_text(json.dumps(report))
    monkeypatch.setattr(runner, 'run_package', lambda *a: (output, manifest))
    calls, results, stopped = [], [], []

    class Child:
        returncode = None if timeout else exit_code
        def wait(self, timeout):
            assert timeout == runner.CONFIG['max_process_seconds']
            if mutate:
                source.write_text('changed during run')
            if self.returncode is None:
                raise runner.subprocess.TimeoutExpired('fake', timeout)
            return self.returncode
        def poll(self):
            return self.returncode

    child = Child()
    def launch(command, **kwargs):
        calls.append((command, kwargs))
        return child
    monkeypatch.setattr(runner.subprocess, 'Popen', launch)
    monkeypatch.setattr(runner, 'stop_child', lambda process: stopped.append(process))
    monkeypatch.setattr(runner, 'finish', lambda out, man, result: results.append(dict(result)))
    code = runner.main()
    assert len(calls) == len(results) == 1
    command, kwargs = calls[0]
    assert command[1] == str(tmp_path/'scripts/probe_sensor_perception.py')
    assert command[2:] == ['--output', str(output)]
    assert kwargs['cwd'] == tmp_path
    return code, results[0], stopped, child


@pytest.mark.parametrize('exit_code,report', [(3, {'passed': True}), (0, None), (0, {'passed': False})])
def test_nonzero_or_missing_or_failed_native_result_cannot_pass(monkeypatch, tmp_path, exit_code, report):
    code, result, stopped, _ = fake_run(monkeypatch, tmp_path, exit_code=exit_code, report=report)
    assert code == 1 and result['passed'] is False
    assert not stopped


def test_timeout_stops_only_owned_child_and_preserves_failure(monkeypatch, tmp_path):
    code, result, stopped, child = fake_run(monkeypatch, tmp_path, timeout=True)
    assert code == 1 and result['passed'] is False
    assert 'TimeoutExpired' in result['error']
    assert stopped == [child]


def test_source_mutation_invalidates_otherwise_passing_native_report(monkeypatch, tmp_path):
    code, result, _, _ = fake_run(monkeypatch, tmp_path, report={'passed': True}, mutate=True)
    assert code == 1 and result['passed'] is False
    assert result['source_changed_during_run'] == ['source.py']


def test_clean_native_report_is_required_for_success(monkeypatch, tmp_path):
    code, result, stopped, _ = fake_run(monkeypatch, tmp_path, report={'passed': True})
    assert code == 0 and result['passed'] is True
    assert result['source_changed_during_run'] == []
    assert not stopped
