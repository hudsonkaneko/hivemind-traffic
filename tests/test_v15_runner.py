"""The shared module for new v15 runners (experiments/v15_runner.py): configuration pinning, the lock and ledger, pool
dispatch and report writing. Every ledger here is a copy-shaped one in a temporary study directory, never the shared
ledger; no SUMO run. The recorded configurations are only read, and the recorded runners' helpers are only called, to
check that the shared copies agree with them."""
import gzip
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from experiments import v15_ledger as L
from experiments import v15_phase1 as P1
from experiments import v15_phase3 as P3
from experiments import v15_phase4 as P4
from experiments import v15_pool as P
from experiments import v15_runner as R
from experiments import v15_stage5 as S5
from experiments.run_highway_flow_rl_v05 import LEDGER_SCHEMA
from traffic import v15_level2 as V2

ROOT = Path(__file__).resolve().parents[1]
RECORDED = sorted((ROOT / 'configs' / 'evaluation').glob('v15-*.json'))


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def _study(path):
    path.mkdir(parents=True, exist_ok=True)
    segments = [dict(command='train', category='training', round_index=24, status='complete', charged_seconds=1000.,
                     decisions=5000, episodes=0)]
    (path / 'cost-ledger.json').write_text(json.dumps(dict(schema=LEDGER_SCHEMA, inherited_seconds=0., segments=segments)))
    return path


def _config(**change):
    config = dict(schema='test', workers=16, priority='below_normal', authorization='test', round_index=24,
                  charge=dict(expected_decisions=900, expected_clock_seconds=60., reserve_seconds=300.,
                              total_wall_seconds=1500., expected_episodes=4),
                  batches=[dict(name='a', expected_decisions=900, tasks=[dict(row_set='v15-development', row=3,
                                                                              driver='rule')])],
                  rows=[3], benchmark=dict(rows=[3, 4], sha='x'))
    config.update(change)
    return config


def _session(study, clock, config=None, allowance=1000):
    config = config or _config()
    return R.session(config, R.canonical_sha(config), study / 'out', phase='9', decision_allowance=allowance, study=study,
                     clock=clock)


def _result(rows=('v15-development', 'v15-training'), canary_row='v15-training', decisions=300, matched=True,
            reused=()):
    records = [dict(key=f'k{i}', fields=dict(row_set=r), reused=i in reused) for i, r in enumerate(rows)]
    canary = None if canary_row is None else dict(key='k%d' % list(rows).index(canary_row), matched=matched)
    simulated = sum(1 for r in records if not r['reused'])
    return SimpleNamespace(records=records, canary=canary, decisions=decisions, canary_decisions=10,
                           charged_decisions=decisions + 10, simulated_tasks=simulated, worker_seconds=40.,
                           simulated_seconds=200., clock_seconds=5.)


def _segment(study):
    return json.loads((study / 'cost-ledger.json').read_text())['segments'][-1]


# --------------------------------------------------------------------------- configurations

def test_the_hash_is_the_one_the_recorded_runners_give_the_ledger():
    for value in (dict(b=1, a=[1, 2., 'x']), [dict(z=None), True], _config()):
        assert R.canonical_sha(value) == P1._json_sha(value)


@pytest.mark.parametrize('path', RECORDED, ids=lambda p: p.name)
def test_every_recorded_configuration_has_a_slim_form_that_pins_it(path, tmp_path):
    """Read only: each recorded configuration splits into settings and generated sections that rebuild it, and its slim
    form pins the full configuration at the recorded runners' hash."""
    before = path.read_bytes()
    config = json.loads(before)
    form = R.write_slim(tmp_path / 'slim.json', config)
    rebuilt = dict(form['settings'], **{k: config[k] for k in form['generated']})
    assert rebuilt == config and set(form['generated']) <= set(R.GENERATED)
    assert form['resolved_sha256'] == P1._json_sha(config)
    assert R.pin(tmp_path / 'slim.json', config) == P1._json_sha(config)
    assert path.read_bytes() == before


def test_the_recorded_configurations_are_the_eighteen_measured():
    assert len(RECORDED) == 18


def test_the_slim_form_drops_the_generated_sections_and_is_deterministic(tmp_path):
    config = _config()
    a, b = R.write_slim(tmp_path / 'a.json', config), R.write_slim(tmp_path / 'b.json', json.loads(json.dumps(config)))
    assert (tmp_path / 'a.json').read_bytes() == (tmp_path / 'b.json').read_bytes()
    assert b'\r\n' not in (tmp_path / 'a.json').read_bytes()
    assert a['generated'] == ['batches', 'benchmark', 'rows'] and 'batches' not in a['settings']
    assert a['settings']['charge'] == config['charge'] and a['schema'] == R.SLIM_SCHEMA


@pytest.mark.parametrize('change', [
    lambda c: c.update(workers=8),                                       # a decided setting
    lambda c: c['charge'].update(expected_decisions=901),                # a nested decided setting
    lambda c: c.update(note='new'),                                      # a setting added
    lambda c: c.pop('authorization'),                                    # a setting dropped
    lambda c: c['batches'][0]['tasks'][0].update(row=4),                 # a generated section: only the hash shows it
    lambda c: c.update(parity=dict(row=3)),                              # a generated section added
    lambda c: c.pop('rows'),                                             # a generated section dropped
], ids=['setting', 'nested', 'added', 'dropped', 'generated', 'generated-added', 'generated-dropped'])
def test_pin_refuses_a_resolved_configuration_that_is_not_the_committed_one(change, tmp_path):
    R.write_slim(tmp_path / 'slim.json', _config())
    resolved = _config()
    change(resolved)
    with pytest.raises(R.ConfigMismatch):
        R.pin(tmp_path / 'slim.json', resolved)


def test_pin_refuses_a_file_that_is_not_a_slim_configuration(tmp_path):
    (tmp_path / 'full.json').write_text(json.dumps(_config()))
    with pytest.raises(R.ConfigMismatch, match='not a slim'):
        R.pin(tmp_path / 'full.json', _config())
    form = R.slim(_config())
    form.pop('resolved_sha256')
    (tmp_path / 'broken.json').write_text(json.dumps(form))
    with pytest.raises(R.ConfigMismatch, match='resolved_sha256'):
        R.pin(tmp_path / 'broken.json', _config())


def test_the_full_resolved_configuration_is_kept_in_the_run_outputs(tmp_path):
    config = _config()
    path, sha = R.write_resolved(tmp_path / 'outputs' / 'run', config)
    assert sha == R.canonical_sha(config) and json.loads(path.read_text()) == config


def test_line_counts_only_read(tmp_path):
    path = tmp_path / 'full.json'
    path.write_text(json.dumps(_config(), indent=1, sort_keys=True))
    before = path.read_bytes()
    full, slim = R.line_counts([path])[str(path)]
    assert full == len(before.decode().splitlines()) and 0 < slim < full and path.read_bytes() == before


# --------------------------------------------------------------------------- limits and charge

def test_the_cpu_limit_is_sixteen_workers_at_below_normal_priority():
    R.check_limits(_config())
    with pytest.raises(ValueError, match='16 workers'):
        R.check_limits(_config(workers=17))
    with pytest.raises(ValueError, match='below_normal'):
        R.check_limits(_config(priority='normal'))


def test_projected_and_check_fit_agree_with_phase_4():
    assert R.MARGIN == P3.MARGIN and R.TRAIN == P3.TRAIN
    for expected, ratio in ((1000, 1.), (1000, .5), (1234, 1.37)):
        assert R.projected(expected, ratio) == P4._projected(expected, ratio)
    fits = dict(expected_decisions=1000, decisions_left=1050, expected_clock_seconds=50., seconds_left=60.)
    for charge in (fits, dict(fits, decisions_left=1049), dict(fits, seconds_left=49.),
                   dict(fits, projected_decisions=2000, decisions_left=1999)):
        outcomes = []
        for check in (R.check_fit, P4._check_fit):
            try:
                check(charge)
                outcomes.append('fits')
            except L.BudgetExceeded as error:
                outcomes.append(str(error))
        assert outcomes[0] == outcomes[1]


# --------------------------------------------------------------------------- the lock and ledger

def test_the_session_takes_the_lock_and_charges_the_ledger(tmp_path):
    study, clock = _study(tmp_path / 'study'), Clock()
    with _session(study, clock) as session:
        assert (study / 'active.lock').is_file()
        assert session.segment['process_priority'] == 'below_normal' and session.segment['workers'] == 16
        clock.now += 10.
    assert not (study / 'active.lock').exists()
    segment = _segment(study)
    assert segment['status'] == 'complete' and segment['config_sha256'] == R.canonical_sha(_config())


def test_the_session_refuses_more_than_sixteen_workers_before_taking_the_lock(tmp_path):
    study = _study(tmp_path / 'study')
    with pytest.raises(ValueError, match='16 workers'):
        _session(study, Clock(), _config(workers=24))
    assert not (study / 'active.lock').exists()


def test_charge_agrees_with_phase_3(tmp_path):
    """The same batches charged through the shared copy and through the recorded runners' P3._add."""
    segments = []
    for name, add in (('shared', R.charge), ('phase3', P3._add)):
        study = _study(tmp_path / name)
        with _session(study, Clock()) as session:
            session.segment['training_episodes'] = 0
            add(session, _result(), [])
            add(session, _result(rows=('v15-training', 'v15-development'), canary_row='v15-development', reused=(0,)), [])
            add(session, _result(canary_row=None), [])
        segment = _segment(study)
        segments.append({k: v for k, v in segment.items() if k not in ('output', 'started_at_unix')})
    assert segments[0] == segments[1]
    assert segments[0]['training_episodes'] == 2 + 0 + 1


def test_charge_counts_training_episodes_when_the_runner_did_not_start_the_count(tmp_path):
    study = _study(tmp_path / 'study')
    with _session(study, Clock()) as session:
        R.charge(session, _result(), [])
    assert _segment(study)['training_episodes'] == 2 and _segment(study)['evaluation_episodes'] == 1


# --------------------------------------------------------------------------- pool dispatch

class Pool:
    def __init__(self, outcome):
        self.outcome, self.tasks = outcome, None

    def run(self, tasks):
        self.tasks = tasks
        if isinstance(self.outcome, BaseException):
            raise self.outcome
        return self.outcome


TASKS = [dict(row_set='v15-development', row=3, driver='rule:expert-v3'),
         dict(row_set='v15-training', row=7, driver=V2.RULE_DRIVER, mode='collect', sample_seed=2,
              label_with=V2.RULE_DRIVER, perturb=V2.PERTURBATION['version']),
         dict(row_set='v15-training', row=8, driver='policy:abc', mode='collect', sample_seed=1, label_with='policy:def')]


def test_make_task_agrees_with_phase_3():
    assert P3.BACKGROUND == 'idm'
    for t in TASKS:
        assert R.make_task(P, t, 'run-1', 'idm') == P3._task(P, t, 'run-1')


def test_dispatch_charges_a_reported_batch(tmp_path):
    study = _study(tmp_path / 'study')
    pool = Pool(_result())
    with _session(study, Clock()) as session:
        result, tasks = R.dispatch(session, pool, TASKS, index=0, run='r', background='idm', projected=500.)
    assert result is pool.outcome and tasks == pool.tasks and [t.row for t in tasks] == [3, 7, 8]
    assert _segment(study)['decisions'] == 310 and _segment(study)['batches'] == 1


def test_dispatch_refuses_a_batch_past_the_allowance_before_running_it(tmp_path):
    study = _study(tmp_path / 'study')
    pool = Pool(_result())
    with pytest.raises(L.BudgetExceeded):
        config = _config(charge=dict(_config()['charge'], expected_decisions=300))
        with _session(study, Clock(), config, allowance=400) as session:
            R.dispatch(session, pool, TASKS, index=0, run='r', background='idm', projected=500.)
    assert pool.tasks is None and _segment(study)['decisions'] == 0


def test_a_canary_mismatch_charges_the_batch_it_reports(tmp_path):
    study = _study(tmp_path / 'study')
    pool = Pool(P.CanaryMismatch('canary differs', result=_result(matched=False)))
    with pytest.raises(P.CanaryMismatch):
        with _session(study, Clock()) as session:
            R.dispatch(session, pool, TASKS, index=2, run='r', background='idm', projected=500.)
    segment = _segment(study)
    assert segment['decisions'] == 310 and 'estimated_charges' not in segment and segment['status'] == 'failed'


@pytest.mark.parametrize('error, note', [(P.CanaryMismatch('no result'), 'batch 2: no result'),
                                         (RuntimeError('worker died'), 'batch 2 failed before reporting: RuntimeError'),
                                         (KeyboardInterrupt(), 'batch 2 failed before reporting: KeyboardInterrupt')])
def test_a_batch_that_fails_before_reporting_is_charged_its_projection(error, note, tmp_path):
    study = _study(tmp_path / 'study')
    with pytest.raises(type(error)):
        with _session(study, Clock()) as session:
            R.dispatch(session, Pool(error), TASKS, index=2, run='r', background='idm', projected=500.2)
    segment = _segment(study)
    assert segment['decisions'] == 501 and segment['estimated_charges'] == [dict(decisions=501, note=note)]
    assert segment['status'] == 'failed' and not (study / 'active.lock').exists()


def test_check_canary_stops_on_an_unmatched_or_missing_canary():
    R.check_canary(_result(), 0)
    R.check_canary(SimpleNamespace(simulated_tasks=0, canary=None), 0)      # everything reused: nothing to check
    for result in (_result(matched=False), _result(canary_row=None)):
        with pytest.raises(P.CanaryMismatch):
            R.check_canary(result, 3)


# --------------------------------------------------------------------------- reports and results

def test_write_charge_agrees_with_stage_5(tmp_path):
    study = _study(tmp_path / 'study')
    with _session(study, Clock()) as session:
        session.segment['training_episodes'] = 0
        R.charge(session, _result(), [])
        R.write_charge(tmp_path / 'shared', session)
        (tmp_path / 'stage5').mkdir()
        S5._write_charge(tmp_path / 'stage5', session)
    shared = json.loads((tmp_path / 'shared' / 'charge.json').read_text())
    assert shared == json.loads((tmp_path / 'stage5' / 'charge.json').read_text())
    assert shared['decisions'] == 310 and shared['ledger'] == str(study / 'cost-ledger.json')


def test_write_report_and_log(tmp_path):
    R.write_report(tmp_path / 'a', dict(passed=3))
    assert json.loads((tmp_path / 'a' / 'report.json').read_text()) == dict(passed=3)
    assert not (tmp_path / 'a' / 'report.md').exists()
    R.write_report(tmp_path / 'b', dict(passed=3), markdown=lambda r: f'# {r["passed"]} passed\n')
    assert (tmp_path / 'b' / 'report.md').read_text() == '# 3 passed\n'
    log = R.Log(tmp_path / 'log' / 'run.jsonl', echo=False)
    log(dict(batch=0))
    log(dict(batch=1, note='é'))
    lines = (tmp_path / 'log' / 'run.jsonl').read_text(encoding='utf-8').splitlines()
    assert [json.loads(line) for line in lines] == [dict(batch=0), dict(batch=1, note='é')]


RESULTS = [dict(row=r, passed=r % 2 == 0, decisions=100 + r, trace=[0.1 * i for i in range(50)]) for r in range(30)]


def test_results_are_saved_compressed_with_a_summary_of_hashes(tmp_path):
    head = R.write_results(tmp_path / 'a', 'results', RESULTS, summary=dict(rows=30, passed=15))
    again = R.write_results(tmp_path / 'b', 'results', json.loads(json.dumps(RESULTS)), summary=dict(rows=30, passed=15))
    gz = (tmp_path / 'a' / 'results.json.gz').read_bytes()
    assert gz == (tmp_path / 'b' / 'results.json.gz').read_bytes()            # deterministic: no name or time in it
    assert head == again and head['entries'] == 30 and head['summary'] == dict(rows=30, passed=15)
    assert head['gzip_sha256'] == hashlib.sha256(gz).hexdigest() and head['content_sha256'] == R.canonical_sha(RESULTS)
    assert len(gz) < len(R.dumps(RESULTS).encode()) / 3
    assert R.read_results(tmp_path / 'a' / 'results.summary.json') == RESULTS


def test_reading_results_refuses_a_changed_file(tmp_path):
    R.write_results(tmp_path, 'results', RESULTS)
    summary = tmp_path / 'results.summary.json'
    gz = tmp_path / 'results.json.gz'
    original = gz.read_bytes()
    gz.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
    with pytest.raises(R.ResultsMismatch, match='gzip hash'):
        R.read_results(summary)
    changed = json.loads(gzip.decompress(original))
    changed[0]['passed'] = not changed[0]['passed']
    gz.write_bytes(gzip.compress(json.dumps(changed).encode()))
    head = json.loads(summary.read_text())
    head['gzip_sha256'] = hashlib.sha256(gz.read_bytes()).hexdigest()
    summary.write_text(json.dumps(head))
    with pytest.raises(R.ResultsMismatch, match='content hash'):
        R.read_results(summary)
    summary.write_text(json.dumps(dict(head, schema='other')))
    with pytest.raises(R.ResultsMismatch, match='not a results summary'):
        R.read_results(summary)
