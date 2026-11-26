"""The v15 Phase 1 runner (experiments/v15_phase1.py): its batches, its report, and (where the policies and
Phase 0's records exist) that the pinned configuration is what it resolves. No simulation."""
import json
from pathlib import Path

import pytest

from experiments import v15_phase1 as phase1

ROOT = Path(__file__).resolve().parents[1]
CASES = [f'case-{c}' for c in range(14)]


def _rows():
    return [dict(case=dict(id=CASES[c] if c not in (0, 4) else ('v05-validation-i110-merge', 'ramp-eval-train-2-lane-ramp120-ramp-started')[c == 4]),
                 seed=6952001 + k, horizon_steps=800 if c in (0, 8, 9, 10, 11, 12, 13, 3) else 2000)
            for k in range(5) for c in range(14)]


def test_batches_go_seed_by_seed_with_the_cheapest_short_task_as_canary():
    rows = _rows()
    scored = {i for i in range(70) if i % 14 not in (0, 4)}
    per_row = [100. + i for i in range(70)]
    per_row[10] = 5.                                   # the cheapest 800-step row of the first seed
    ratios = {'round-16': 1., 'round-21-final': 1., 'plan-follower': 1.1}
    batches = phase1._batches(rows, scored, per_row, ratios)
    assert [b['seed'] for b in batches] == [6952001 + k for k in range(5)]
    assert all(len(b['tasks']) == 40 for b in batches) and sum(len(b['tasks']) for b in batches) == 200
    assert batches[0]['tasks'][0] == dict(driver='round-16', row=10) and batches[0]['canary']['expected'] == 5
    for batch in batches:
        assert {rows[t['row']]['seed'] for t in batch['tasks']} == {batch['seed']}
        assert not [t for t in batch['tasks'] if t['driver'] == 'plan-follower' and t['row'] % 14 in (0, 4)]
        first = batch['tasks'][0]
        assert rows[first['row']]['horizon_steps'] == 800
        expected = sum(per_row[t['row']] * ratios[t['driver']] for t in batch['tasks']) + batch['canary']['expected']
        assert batch['expected_decisions'] == pytest.approx(expected, abs=1)


def _record(driver, row, passed, events=None, failures=()):
    return dict(key=f'{driver}-{row}', fields=dict(row_set='v15-development', background='idm', mode='validate',
                                                   driver=driver, row=row), passed=passed, failures=list(failures),
                decisions=10, events=events or dict(collisions=0, stops=1))


def test_report_rates_cases_pairs_and_partial_runs(tmp_path):
    rows = [dict(row=i, case=('a', 'b', 'skip')[i % 3], seed=1 + i // 3, horizon_steps=800) for i in range(6)]
    config = dict(rows=rows, drivers={'round-16': dict(id='policy:r16'), 'round-21-final': dict(id='policy:r21'),
                                      'plan-follower': dict(id='rule:plan-follower-v1', rows=[0, 1, 3, 4])},
                  paired=[['round-21-final', 'round-16', 'all'], ['plan-follower', 'round-16', 'follower rows'],
                          ['plan-follower', 'round-21-final', 'follower rows']])
    (tmp_path / 'resolved-config.json').write_text(json.dumps(config))
    outcomes = {'policy:r16': [True, False, False, True, False, True], 'policy:r21': [True, True, False, False, False, True],
                'rule:plan-follower-v1': {0: True, 1: True, 3: True}}               # follower row 4 not run
    lines = []
    for driver, results in outcomes.items():
        items = results.items() if isinstance(results, dict) else enumerate(results)
        for row, passed in items:
            lines.append(_record(driver, row, passed, failures=() if passed else ('rolling_merge_quality',)))
    lines.append(_record('policy:r16', 0, False))                                    # a later record of a key is ignored
    (tmp_path / 'results.jsonl').write_text('\n'.join(json.dumps(line) for line in lines) + '\n')
    result = phase1.report(tmp_path)
    r16 = result['drivers']['round-16']
    assert r16['pooled']['passed'] == 3 and r16['pooled']['rows'] == 6 and r16['run_rows'] == 6
    assert r16['cases']['a'] == dict(passed=2, rows=2, rate=1., low=pytest.approx(2 / (2 + 1.959963984540054 ** 2), abs=1e-9), high=1.)
    assert r16['gate_failures'] == {'rolling_merge_quality': 3} and r16['events']['stops'] == dict(rows=6, count=6)
    follower = result['drivers']['plan-follower']
    assert follower['planned_rows'] == 4 and follower['run_rows'] == 3 and follower['pooled']['passed'] == 3
    pairs = {(p['first'], p['second']): p for p in result['paired']}
    assert pairs[('round-21-final', 'round-16')]['rows'] == 6
    assert (pairs[('round-21-final', 'round-16')]['first_only'], pairs[('round-21-final', 'round-16')]['second_only']) == (1, 1)
    follower_r16 = pairs[('plan-follower', 'round-16')]
    assert follower_r16['rows'] == 3 and follower_r16['difference'] == pytest.approx(1 / 3)    # rows 0, 1, 3: b=1, c=0
    assert result['simulator'].startswith('SUMO') and 'held-out rows not built' in result['rows']
    text = (tmp_path / 'report.md').read_text()
    assert '| round-16 | 3 of 6 |' in text and 'not scored' in text


@pytest.mark.skipif(not (ROOT / phase1.PHASE0_RECORDS).is_file() or not (ROOT / phase1.ROUND21_FINAL).is_file(),
                    reason='Phase 0 records and the round-21 policy are local outputs, not in Git')
def test_the_pinned_configuration_is_what_resolves_and_fits_the_approved_charge():
    config = json.loads(json.dumps(phase1.resolve()))
    assert config == json.loads(phase1.CONFIG.read_text())
    charge = config['charge']
    assert charge['expected_decisions'] <= charge['decision_allowance'] <= phase1.ALLOWANCE_CAP == 400_000
    assert config['backgrounds'] == ['idm'] and config['mode'] == 'validate' and config['row_set'] == 'v15-development'
    assert config['workers'] == 16 and config['priority'] == 'below_normal'     # the user's CPU limit
    assert config['drivers']['round-16']['id'] == phase1.PHASE0_ROUND16
    assert len(config['drivers']['plan-follower']['rows']) == 60 and len(config['rows']) == 70
    assert sum(len(b['tasks']) for b in config['batches']) == 200 and len(config['parity']['rows']) == 28


HAS_LIBSUMO = __import__('importlib').util.find_spec('libsumo') is not None


@pytest.mark.skipif(not HAS_LIBSUMO or not (ROOT / phase1.PHASE0_RECORDS).is_file() or not (ROOT / phase1.ROUND21_FINAL).is_file(),
                    reason='needs libsumo, Phase 0 records and the round-21 policy (local outputs, not in Git)')
@pytest.mark.timeout(900)
def test_the_charged_path_end_to_end_on_a_temporary_ledger(tmp_path, monkeypatch):
    """One 3-task batch (every driver on development row 10, which has a Phase 0 parity record), 2 workers,
    charged to a temporary ledger: lock, session, pool, canary, parity check, ledger segment and report."""
    import shutil
    import uuid
    from experiments import v15_ledger
    from experiments.run_highway_flow_rl_v05 import LEDGER_SCHEMA, ledger_totals
    config = json.loads(json.dumps(phase1.resolve()))
    tasks = [dict(driver='round-16', row=10), dict(driver='round-21-final', row=10), dict(driver='plan-follower', row=10)]
    config['batches'] = [dict(seed=6952001, tasks=tasks, expected_decisions=2000, canary=dict(tasks[0], expected=453))]
    config['workers'] = 2
    config['charge'].update(expected_decisions=2000, decision_allowance=5000, expected_clock_seconds=60., reserve_seconds=600.)
    pinned = tmp_path / 'v15-phase1.json'
    pinned.write_text(json.dumps(config))
    study = tmp_path / 'study'
    study.mkdir()
    (study / 'cost-ledger.json').write_text(json.dumps(dict(schema=LEDGER_SCHEMA, inherited_seconds=0., segments=[])))
    monkeypatch.setattr(phase1, 'resolve', lambda: json.loads(json.dumps(config)))
    monkeypatch.setattr(phase1, 'CONFIG', pinned)
    monkeypatch.setattr(v15_ledger, 'study_directory', lambda: study)
    from experiments import v15_pool
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-phase1-{uuid.uuid4().hex[:8]}'
    try:
        result = phase1.run(out)                  # lowers this process's priority, as the learner's
        segment = json.loads((study / 'cost-ledger.json').read_text())['segments'][0]
        assert segment['status'] == 'complete' and not (study / 'active.lock').exists()
        log = [json.loads(line) for line in (out / 'phase1-log.jsonl').read_text().splitlines()]
        assert log[0]['parity_rows'] == 1 and log[0]['parity_differing'] == [] and log[0]['canary']['matched']
        assert segment['decisions'] == log[0]['decisions'] + log[0]['canary_decisions'] and segment['evaluation_episodes'] == 4
        assert segment['workers'] == 2 and segment['worker_seconds'] > 0 and segment['simulated_seconds'] > 0
        assert segment['process_priority'] == 'below_normal' and segment['worker_priorities'] == ['below_normal']
        assert ledger_totals(json.loads((study / 'cost-ledger.json').read_text()), 24)['total'] == segment['charged_seconds']
        charge = json.loads((out / 'charge.json').read_text())
        assert charge['decisions'] == segment['decisions'] and charge['status'] == 'complete'
        assert {name: d['run_rows'] for name, d in result['drivers'].items()} == {name: 1 for name in phase1.DRIVERS}
        assert result['charge']['decisions'] == segment['decisions'] and (out / 'report.md').is_file()
    finally:
        v15_pool.set_priority('normal')
        shutil.rmtree(out, ignore_errors=True)
