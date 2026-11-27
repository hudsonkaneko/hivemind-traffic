"""v15 Stage 5 part 1, the held-out check: the guard, the resolve, the run-once rule and the reading.

No test here builds a held-out row: the guard's refusals are checked, and the resolve is checked with
traffic/v15_benchmark.held_out_rows replaced by a function that fails if it is called.
"""
from pathlib import Path
from types import SimpleNamespace

import pytest

from experiments import v15_pool as pool
from experiments import v15_stage5 as stage5
from traffic import highway_flow_rl_v14_contract as contract
from traffic import v15_benchmark as bench

ROOT = Path(__file__).resolve().parents[1]


def _never(*_, **__):
    raise AssertionError('a held-out row was built')


@pytest.fixture(scope='module')
def round24():
    return contract.build_config(24)


def test_only_an_authorized_pool_builds_the_held_out_rows(round24, monkeypatch):
    assert pool.HELD_OUT_ROW_SET == stage5.HELD and stage5.HELD not in pool.ROW_SETS
    monkeypatch.setattr(bench, 'held_out_rows', _never)
    for authorization in (None, '', '  ', True, 1):
        with pytest.raises(PermissionError):
            pool._held_out(round24, authorization)
    # no task on the held-out rows without an authorized pool in this process
    assert not pool._HELD_OUT
    with pytest.raises(ValueError):
        pool.Task(stage5.HELD, 0, stage5.EXPERT)
    with pytest.raises(ValueError):
        pool.Task('v15-held-out-2', 0, stage5.EXPERT)
    # and a pool built without the authorization refuses one, even if a task exists
    monkeypatch.setitem(pool._HELD_OUT, 'authorization', 'a test value, not the user\'s')
    task = pool.Task(stage5.HELD, 0, stage5.EXPERT)
    unauthorized = SimpleNamespace(rows={name: [] for name in pool.ROW_SETS}, round_index=24)
    with pytest.raises(PermissionError):
        pool.WorkerPool.key(unauthorized, task)


def test_an_authorized_pool_hands_the_authorization_to_every_worker(monkeypatch):
    """The pool's plumbing with stand-in workers (threads) and stand-in rows: no SUMO, no held-out row built."""
    import queue
    import shutil
    import threading
    import uuid
    fake = [dict(case=dict(id=f'case-{r % 14}'), seed=0, horizon_steps=800) for r in range(42)]

    def held_out(cfg, authorization):
        if not isinstance(authorization, str) or not authorization:
            raise PermissionError
        pool._HELD_OUT['authorization'] = authorization
        return fake

    seen = []

    class Process:
        def __init__(self, target, daemon, args):
            self.thread = threading.Thread(target=target, args=args, daemon=daemon)

        def start(self):
            self.thread.start()

        def is_alive(self):
            return self.thread.is_alive()

        def join(self, timeout=None):
            self.thread.join(timeout)

        def terminate(self):
            raise AssertionError('a stand-in worker did not stop')

    monkeypatch.setattr(pool, '_W', {})
    monkeypatch.setattr(pool, '_held_out', held_out)
    monkeypatch.setattr(pool, '_worker_setup', lambda backend, round_index, policy_dir, held_out=None: seen.append(held_out))
    monkeypatch.setattr(pool, 'set_priority', lambda priority: priority)
    monkeypatch.setattr(pool.mp, 'get_context', lambda method: SimpleNamespace(Queue=queue.Queue, Process=Process))
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-stage5-{uuid.uuid4().hex[:8]}'
    try:
        with pool.WorkerPool(2, out, priority='below_normal', held_out='a test value') as workers:
            assert workers.rows[stage5.HELD] is fake and pool._HELD_OUT == dict(authorization='a test value')
            assert workers.key(pool.Task(stage5.HELD, 3, stage5.EXPERT)) != \
                workers.key(pool.Task('v15-development', 3, stage5.EXPERT))
        assert seen == ['a test value', 'a test value'] and not pool._HELD_OUT
        with pool.WorkerPool(1, out, priority='below_normal') as plain:
            assert stage5.HELD not in plain.rows
        assert seen[-1] is None
    finally:
        shutil.rmtree(out, ignore_errors=True)


def test_the_canary_falls_on_the_development_check(round24):
    horizon = {c: bench.cases(round24)[c]['horizon_steps'] for c in range(14)}
    assert horizon[stage5.DEV_CHECK_ROW] == min(horizon.values())
    tasks = [dict(row_set=pool.ROW_SETS[2], row=3)] + stage5._held_out_tasks()
    assert stage5._canary_index(tasks, horizon) == 0
    # without the development check first, the canary would re-run a held-out row
    assert tasks[1:][stage5._canary_index(tasks[1:], horizon)]['row_set'] == stage5.HELD
    assert len(stage5._held_out_tasks()) == 126
    assert {(t['row'], t['driver']) for t in stage5._held_out_tasks()} == \
        {(r, d) for r in range(42) for d in stage5.DRIVERS}


def test_the_check_runs_once(tmp_path, monkeypatch):
    monkeypatch.setattr(stage5, 'out_dir', lambda step: tmp_path / step)
    ledger = dict(segments=[dict(v15_phase='4', stage5_step=None)])
    monkeypatch.setattr(stage5, '_ledger_stage5', lambda: (None, None, ledger))
    stage5._not_run_yet('held-out')
    (tmp_path / 'held-out').mkdir()
    (tmp_path / 'held-out' / 'resolved-config.json').write_text('{}')     # a run stopped before its session
    stage5._not_run_yet('held-out')
    for name in ('results.jsonl', 'charge.json', 'report.json'):
        (tmp_path / 'held-out' / name).write_text('{}')
        with pytest.raises(ValueError):
            stage5._not_run_yet('held-out')
        (tmp_path / 'held-out' / name).unlink()
    ledger['segments'].append(dict(v15_phase='5', stage5_step='held-out', status='failed'))
    with pytest.raises(ValueError):
        stage5._not_run_yet('held-out')


def test_the_development_checks_must_match_their_records():
    config = dict(dev_check=dict(report_sha='r', trace_sha='t'), parity=dict(report_sha='pr', trace_sha='pt'))
    tasks = [dict(row=3), dict(row=0), dict(row=52, parity=True)]
    records = [dict(key='k3', report_sha='r', trace_sha='t'), dict(key='k0', report_sha='x', trace_sha='y'),
               dict(key='k52', report_sha='pr', trace_sha='pt')]
    good = SimpleNamespace(records=records, canary=dict(key='k3', matched=True))
    assert stage5._checks(config, tasks, good) == dict(canary=True, dev_check=True, parity=True)
    elsewhere = SimpleNamespace(records=records, canary=dict(key='k0', matched=True))
    assert not stage5._checks(config, tasks, elsewhere)['canary']
    drifted = SimpleNamespace(records=[dict(records[0], trace_sha='u')] + records[1:], canary=good.canary)
    assert not stage5._checks(config, tasks, drifted)['dev_check']
    parity = SimpleNamespace(records=records[:2] + [dict(records[2], report_sha='q')], canary=good.canary)
    assert stage5._checks(config, tasks, parity)['parity'] is False


def test_the_frozen_regrade_is_pointed_at_the_held_out_rows_and_restored():
    from experiments import v15_regrade as R
    before = R.ROW_SET
    with stage5._HeldOutRegrade() as regrade:
        assert regrade.ROW_SET == stage5.HELD
    assert R.ROW_SET == before
    with pytest.raises(RuntimeError):
        with stage5._HeldOutRegrade():
            raise RuntimeError
    assert R.ROW_SET == before


def test_the_reading_set_before_the_run():
    calm = dict(collisions=dict(rows=0, count=0), cut_ins=dict(rows=1, count=1), stops=dict(rows=0, count=0))
    dev = (['rolling_merge_quality'], ['cut_ins'])
    holds = stage5._reading(dict(low=-.1, high=.05), dict(low=.02, high=.3), {4: ['rolling_merge_quality']}, calm, *dev)
    assert holds['holds_up'] and holds['beats_v14'] and not holds['new_gate_failures'] and not holds['new_events']
    lower = stage5._reading(dict(low=-.3, high=-.01), dict(low=-.02, high=.3), {}, calm, *dev)
    assert not lower['holds_up'] and not lower['beats_v14']
    new_gate = stage5._reading(dict(low=-.1, high=.05), dict(low=.02, high=.3), {4: ['headway']}, calm, *dev)
    assert not new_gate['holds_up'] and new_gate['new_gate_failures'] == ['headway']
    crash = dict(calm, collisions=dict(rows=1, count=1))
    crashed = stage5._reading(dict(low=-.1, high=.05), dict(low=.02, high=.3), {}, crash, *dev)
    assert not crashed['holds_up'] and crashed['policy_collisions'] == 1 and crashed['new_events'] == ['collisions']
    stopped = stage5._reading(dict(low=-.1, high=.05), dict(low=.02, high=.3), {}, dict(calm, stops=dict(rows=1, count=2)),
                              *dev)
    assert not stopped['holds_up'] and stopped['new_events'] == ['stops']


def test_the_resolve_builds_no_held_out_row(monkeypatch):
    needed = (stage5.P3.out_dir(stage5.POLICY_STEP) / 'results.jsonl', ROOT / stage5.ROUND16_RUN / 'results.jsonl',
              ROOT / stage5.REBASELINE)
    if not all(p.is_file() for p in needed):
        pytest.skip('the Phase 1-3 records are not in this checkout (outputs/ is not in Git)')
    try:
        stage5._ledger_stage5()
    except Exception:
        pytest.skip('no study ledger in this checkout')
    monkeypatch.setattr(bench, 'held_out_rows', _never)
    config = stage5.resolve('held-out')
    tasks = config['batches'][0]['tasks']
    assert tasks[0] == dict(row_set='v15-development', row=3, driver=stage5.POLICY, dev_check=True)
    assert tasks[-1]['parity'] and tasks[-1]['row'] == 52 and tasks[1:-1] == stage5._held_out_tasks()
    charge = config['charge']
    # the proposal's numbers (plan v15, "Stage 5 proposal"): 249,597 expected, 232,604-272,099, 129 episodes
    assert charge['expected_decisions'] == 249_597 and charge['range'] == dict(low=232_604, high=272_099)
    assert charge['expected_episodes'] == 129 and charge['projected_decisions'] == 262_077
    assert charge['projected_decisions'] <= stage5.ALLOWANCE and charge['reserve_seconds'] <= stage5.TIME_CAP
    assert [d['id'] for d in config['drivers']] == list(stage5.DRIVERS)
    assert config['held_out']['rows'] == 42 and 'resolve builds none' in config['held_out']['built']


def test_the_physics_drives_stop_before_a_drive_that_does_not_fit_and_after_a_failed_part_1(monkeypatch, tmp_path):
    from experiments import v15_stage5_physics as physics
    assert physics.ORDER == ('expert', 'policy') and physics.ROW == 11 and physics.TIME_CAP == 500.
    physics._drive_fits(0., 250., 500.)
    physics._drive_fits(240., 260., 500.)
    with pytest.raises(physics.StopAndAsk):
        physics._drive_fits(260., 250., 500.)
    monkeypatch.setattr(physics, 'OUT', tmp_path / 'physics')
    ledger = dict(segments=[])
    monkeypatch.setattr(stage5, '_ledger_stage5', lambda: (None, None, ledger))
    with pytest.raises(physics.StopAndAsk):                  # part 1 has not run
        physics._not_run_yet()
    ledger['segments'].append(dict(v15_phase='5', stage5_step='held-out', status='failed'))
    with pytest.raises(physics.StopAndAsk):                  # part 1 failed: the user decides
        physics._not_run_yet()
    ledger['segments'][0]['status'] = 'complete'
    physics._not_run_yet()
    ledger['segments'].append(dict(v15_phase='5', stage5_step='physics', status='failed'))
    with pytest.raises(ValueError):                          # the charged drives run once
        physics._not_run_yet()


def test_an_isaac_sim_that_the_run_did_not_start_is_found_and_never_closed(monkeypatch):
    from experiments import v15_stage5_physics as physics
    listing = '"python.exe","100","Console","1","10 K"\n"kit.exe","4242","Console","1","900 K"\n'
    monkeypatch.setattr(physics.subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=listing))
    assert physics.isaac_processes() == ['kit.exe:4242']
    with pytest.raises(RuntimeError):
        physics._check_no_isaac()
    monkeypatch.setattr(physics.subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=listing.splitlines()[0]))
    physics._check_no_isaac()
