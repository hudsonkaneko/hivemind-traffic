"""The v15 Phase 2 runner (experiments/v15_phase2.py): its resolved configuration and its guards.
Resolving reads Phase 1's records (outputs/, not in Git), so these tests skip without them."""
import json
from pathlib import Path

import pytest

from experiments import v15_phase2 as phase2

ROOT = Path(__file__).resolve().parents[1]
HAVE_PHASE1 = (ROOT / phase2.PHASE1_RESULTS).is_file()


@pytest.mark.skipif(not HAVE_PHASE1, reason="Phase 1's records are not in this checkout (outputs/ is not in Git)")
def test_resolve_covers_the_70_rows_in_two_batches_with_a_parity_task():
    config = phase2.resolve(1)
    assert config['phase'] == '2' and config['run'] == 1 and config['workers'] <= 16 and config['priority'] == 'below_normal'
    assert config['backgrounds'] == ['idm'] and config['held_out'] == 'not built, not run'
    expert_rows = [t['row'] for b in config['batches'] for t in b['tasks'] if t['driver'] == 'expert']
    assert sorted(expert_rows) == list(range(70))
    first, second = config['batches']
    assert [t['driver'] for t in first['tasks']].count('round-16') == 1 and first['tasks'][-1]['row'] == config['parity']['row']
    assert all(t['driver'] == 'expert' for t in second['tasks'])
    for batch in config['batches']:
        canary = batch['canary']
        assert batch['tasks'][0] == dict(driver='expert', row=canary['row'])     # the canary runs first
        assert {config['rows'][t['row']]['seed'] for t in batch['tasks'] if t['driver'] == 'expert'} == set(batch['seeds'])
    assert config['charge']['expected_decisions'] == sum(b['expected_decisions'] for b in config['batches'])
    assert config['charge']['expected_decisions'] <= phase2.ALLOWANCE
    assert sum(config['baseline']['passed'].values()) == 33 and config['baseline']['id'] == phase2.ROUND16
    assert config['expert']['id'] == 'rule:expert-v3' and len(config['expert']['source']['sha256']) == 64
    with pytest.raises(ValueError):
        phase2.resolve(6)


@pytest.mark.skipif(not HAVE_PHASE1, reason="Phase 1's records are not in this checkout (outputs/ is not in Git)")
def test_run_refuses_a_changed_configuration_or_the_wrong_run_number(tmp_path, monkeypatch):
    pinned = phase2.resolve(1)
    path = tmp_path / 'v15-phase2-run-01.json'
    monkeypatch.setattr(phase2, 'config_path', lambda run: path)
    path.write_text(json.dumps(dict(pinned, run=99)))
    with pytest.raises(ValueError, match='differs'):
        phase2.run(1, tmp_path / 'out')
    path.write_text(json.dumps(pinned))
    monkeypatch.setattr(phase2, '_ledger_phase2', lambda: dict(segments=1, decisions=10))
    with pytest.raises(ValueError, match='holds 1 Phase 2 runs'):
        phase2.run(1, tmp_path / 'out')
    monkeypatch.setattr(phase2, '_ledger_phase2', lambda: dict(segments=5, decisions=10))
    with pytest.raises(ValueError, match='All 5'):
        phase2.run(1, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()
