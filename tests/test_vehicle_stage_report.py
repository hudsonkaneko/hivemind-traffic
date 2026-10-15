import hashlib
import json
import pytest

from experiments import report_vehicle_stages as report


def fixture(tmp_path, monkeypatch):
    monkeypatch.setattr(report, 'ROOT', tmp_path)
    run = tmp_path/'outputs'/'vehicle_resets'/'fixture'
    (run/'source').mkdir(parents=True)
    (run/'source'/'example.py').write_text('example')
    (run/'summary.json').write_text(json.dumps({'passed': False, 'error': 'retained failure'}))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = {'output_hashes': {p.relative_to(run).as_posix(): digest(p)
                                  for p in run.rglob('*') if p.is_file()},
                'source_hashes': {'example.py': digest(run/'source'/'example.py')},
                'config': {}, 'git_commit': 'fixture', 'git_status': 'dirty'}
    (run/'manifest.json').write_text(json.dumps(manifest))
    return run


def test_failed_attempt_is_not_omitted(tmp_path, monkeypatch):
    result = report.summarize(fixture(tmp_path, monkeypatch))
    assert not result['result']['passed']
    assert result['artifact_hashes_verified']
    assert result['verified_artifact_count'] == 2


def test_modified_artifact_rejected(tmp_path, monkeypatch):
    run = fixture(tmp_path, monkeypatch)
    (run/'summary.json').write_text('{}')
    with pytest.raises(ValueError, match='hash mismatch'):
        report.summarize(run)


def test_outside_stage_tree_rejected(tmp_path, monkeypatch):
    monkeypatch.setattr(report, 'ROOT', tmp_path)
    with pytest.raises(ValueError):
        report.summarize(tmp_path/'outside')
