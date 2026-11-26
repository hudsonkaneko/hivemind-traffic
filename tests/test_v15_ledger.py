"""The v15 evaluation ledger writer (experiments/v15_ledger.py), on a copy-shaped ledger in a temporary
study directory: never the shared ledger."""
import json
from types import SimpleNamespace

import pytest

from experiments import v15_ledger as ledger
from experiments.run_highway_flow_rl_v05 import LEDGER_SCHEMA, ledger_totals


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def _study(tmp_path, *, running=False):
    segments = [dict(command='train', category='training', round_index=24, status='complete', charged_seconds=1000.,
                     decisions=5000, episodes=0),
                dict(command='preflight', category='preparation', round_index=24, status='complete', charged_seconds=10.,
                     decisions=0, episodes=1)]
    if running:
        segments.append(dict(command='train', category='training', status='running', charged_seconds=5., decisions=0))
    (tmp_path / 'cost-ledger.json').write_text(json.dumps(dict(schema=LEDGER_SCHEMA, inherited_seconds=0., segments=segments)))
    return tmp_path


def _session(study, clock, **change):
    settings = dict(phase='1', output=study / 'out', config_sha256='abc', expected_decisions=900, decision_allowance=1000,
                    expected_clock_seconds=60., reserve_seconds=300., total_wall_seconds=1500., intended_episodes=4,
                    workers=24, authorization='test', round_index=24, study=study, clock=clock)
    settings.update(change)
    return ledger.EvaluationSession(**settings)


def _batch(decisions, canary_decisions=10, tasks=2):
    return SimpleNamespace(decisions=decisions, canary_decisions=canary_decisions, charged_decisions=decisions + canary_decisions,
                           simulated_tasks=tasks, canary=dict(matched=True), worker_seconds=40., simulated_seconds=200.)


def _read(study):
    return json.loads((study / 'cost-ledger.json').read_text())


def test_a_session_charges_clock_time_and_every_decision_and_keeps_the_ledger_readable(tmp_path):
    study, clock = _study(tmp_path), Clock()
    with _session(study, clock, priority='below_normal') as session:
        assert (study / 'active.lock').is_file()
        running = _read(study)['segments'][-1]
        assert running['status'] == 'running' and running['charged_seconds'] == 300.   # the reservation while running
        clock.now += 12.5
        session.add_batch(_batch(400))
        session.check_next(500)
        clock.now += 20.
        session.add_batch(_batch(300, canary_decisions=0, tasks=1))
    assert not (study / 'active.lock').exists()
    segment = _read(study)['segments'][-1]
    assert segment['status'] == 'complete' and segment['charged_seconds'] == pytest.approx(32.5)
    assert segment['clock_seconds'] == pytest.approx(32.5) and 'clock_seconds_so_far' not in segment
    assert segment['decisions'] == 710 and segment['canary_decisions'] == 10 and segment['episodes'] == 0
    assert segment['evaluation_episodes'] == 5 and segment['batches'] == 2 and segment['workers'] == 24
    assert segment['worker_seconds'] == 80. and segment['simulated_seconds'] == 400.
    assert segment['category'] == 'preparation' and segment['command'] == ledger.COMMAND and segment['evaluation_only']
    assert segment['process_priority'] == 'below_normal'
    base = ledger_totals(_read(study), 24)                     # every existing runner still reads the ledger
    assert base['total'] == pytest.approx(1042.5) and base['total_decisions'] == 5000 and base['diagnostic_episodes'] == 1
    totals = ledger.totals(_read(study))
    assert totals['v15']['1'] == dict(segments=1, decisions=710, seconds=pytest.approx(32.5), evaluation_episodes=5)


def test_a_held_lock_is_never_overridden(tmp_path):
    study = _study(tmp_path)
    (study / 'active.lock').write_text('outputs/other-run')
    before = (study / 'cost-ledger.json').read_text()
    with pytest.raises(ledger.LockHeld, match='other-run'):
        with _session(study, Clock()):
            pass
    assert (study / 'active.lock').read_text() == 'outputs/other-run' and (study / 'cost-ledger.json').read_text() == before


def test_budgets_are_checked_before_anything_is_written(tmp_path):
    study = _study(tmp_path)
    before = (study / 'cost-ledger.json').read_text()
    for change in (dict(expected_decisions=1001), dict(expected_clock_seconds=301.),
                   dict(total_wall_seconds=1050., expected_clock_seconds=60.)):
        with pytest.raises(ledger.BudgetExceeded):
            with _session(study, Clock(), **change):
                pass
        assert not (study / 'active.lock').exists() and (study / 'cost-ledger.json').read_text() == before


def test_an_unclosed_segment_stops_the_session(tmp_path):
    study = _study(tmp_path, running=True)
    with pytest.raises(ValueError, match='Unclosed'):
        with _session(study, Clock()):
            pass
    assert not (study / 'active.lock').exists()


def test_the_allowance_is_enforced_before_and_after_batches_and_shared_across_attempts(tmp_path):
    study, clock = _study(tmp_path), Clock()
    with pytest.raises(RuntimeError, match='crash'):
        with _session(study, clock) as session:
            session.add_batch(_batch(600))
            with pytest.raises(ledger.BudgetExceeded):
                session.check_next(400)                     # 390 left
            session.charge_estimate(395.2, 'batch 1 crashed')   # as the runner does for a batch that crashed
            raise RuntimeError('crash')
    segment = _read(study)['segments'][-1]
    assert segment['status'] == 'failed' and segment['decisions'] == 1006 and 'RuntimeError: crash' in segment['failure']
    assert segment['estimated_charges'] == [dict(decisions=396, note='batch 1 crashed')]
    with pytest.raises(ledger.BudgetExceeded, match='-6 of'):    # a second attempt sees the first one's charge
        with _session(study, clock, expected_decisions=0):
            pass


def test_charging_past_the_allowance_raises_and_fails_the_segment(tmp_path):
    study = _study(tmp_path)
    with pytest.raises(ledger.BudgetExceeded, match='1,005'):
        with _session(study, Clock()) as session:
            session.add_batch(_batch(995))
    segment = _read(study)['segments'][-1]
    assert segment['status'] == 'failed' and segment['decisions'] == 1005 and not (study / 'active.lock').exists()


def test_the_time_reservation_stops_further_batches(tmp_path):
    study, clock = _study(tmp_path), Clock()
    with _session(study, clock) as session:
        clock.now += 300.
        with pytest.raises(ledger.BudgetExceeded):
            session.check_next(1)
