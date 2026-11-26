"""v15 worker pool: result keys, cross-checks, background models and (where the frozen LA networks
and libsumo are installed) canaries, parity with the trainer's record and worker-independent sampling."""
import importlib.util
import json
import shutil
import uuid
from pathlib import Path

import pytest
import torch

from experiments import v15_pool as pool
from policies.highway_flow_v12 import FlowModelV12

ROOT = Path(__file__).resolve().parents[1]
HAS_LIBSUMO = importlib.util.find_spec('libsumo') is not None
TRAINER_RECORD = ROOT / 'outputs/highway-flow-rl/v14/round-24/initial-validation.json'
ROW = dict(case=dict(id='case-a'), seed=11)


def fields(**change):
    task = pool.Task(**{**dict(row_set='v14-validation', row=3, driver='policy:abc'), **change})
    return pool.key_fields(task, 24, ROW)


def test_the_key_ignores_who_asked_and_covers_what_decides_a_result():
    base = pool.task_key(fields())
    assert pool.task_key(fields(run='seed-2')) == base
    for change in (dict(row=4), dict(driver='policy:abd'), dict(background='krauss'), dict(mode='collect'),
                   dict(label_with='policy:abc'), dict(row_set='v15-development')):
        assert pool.task_key(fields(**change)) != base, change
    assert pool.task_key(fields(sample_seed=5)) == base                 # validation ignores the sample seed
    assert pool.task_key(fields(mode='collect', sample_seed=5)) != pool.task_key(fields(mode='collect'))
    assert fields()['seed'] == 11 and fields()['case'] == 'case-a'


def test_tasks_refuse_unknown_row_sets_backgrounds_modes_and_drivers():
    for change in (dict(row_set='v15-held-out'), dict(background='wiedemann'), dict(mode='train'),
                   dict(driver='rule:v1')):
        with pytest.raises(ValueError):
            pool.Task(**{**dict(row_set='v14-validation', row=0, driver='policy:abc'), **change})


def test_policy_hash_is_content_based():
    torch.manual_seed(0)
    a = FlowModelV12().state_dict()
    b = {k: v.clone() for k, v in a.items()}
    assert pool.state_hash(a) == pool.state_hash(b)
    name = next(iter(b))
    b[name] = b[name] + 1e-6
    assert pool.state_hash(a) != pool.state_hash(b)


def _routes(tmp_path, follow='IDM'):
    path = tmp_path / 'routes.rou.xml'
    types = ''.join(f'<vType id="{n}" carFollowModel="{follow}" tau="1.5"/>' for n in pool.BACKGROUND_PROFILES)
    path.write_text(f'<routes><vType id="controlled" carFollowModel="Krauss" tau="1.5"/>{types}</routes>')
    return path


def test_krauss_background_rewrites_only_the_background_profiles(tmp_path):
    path = _routes(tmp_path)
    pool.set_background(path, 'idm')
    assert path.read_text().count('carFollowModel="IDM"') == 3
    pool.set_background(path, 'krauss')
    text = path.read_text()
    assert 'carFollowModel="IDM"' not in text and text.count('carFollowModel="Krauss"') == 4
    assert 'tau="1.5"' in text


def test_krauss_background_refuses_an_unexpected_routes_file(tmp_path):
    with pytest.raises(ValueError):
        pool.set_background(_routes(tmp_path, follow='ACC'), 'krauss')
    with pytest.raises(ValueError):
        pool.set_background(_routes(tmp_path), 'wiedemann')


def test_interleave_merges_runs_row_by_row():
    assert pool.interleave(['a0', 'a1'], ['b0', 'b1', 'b2'], ['c0']) == ['a0', 'b0', 'c0', 'a1', 'b1', 'b2']


def test_paired_tasks_and_table_line_drivers_up_on_identical_rows():
    tasks = pool.paired_tasks('v15-development', [0, 1], ['policy:a', 'policy:b'])
    assert len(tasks) == 8 and {t.background for t in tasks} == {'idm', 'krauss'}
    records = []
    for t in tasks:
        f = pool.key_fields(t, 24, dict(case=dict(id='c'), seed=100 + t.row))
        records.append(dict(fields=f, passed=t.driver == 'policy:a', events={}))
    table = pool.paired_table(records, 'policy:a', 'policy:b')
    assert len(table) == 4 and all(r['a_passed'] and not r['b_passed'] for r in table)


def test_compact_samples_keep_one_centralized_state_per_step():
    states = torch.arange(6.).reshape(3, 1, 2).expand(3, 4, 2)
    batch = dict(global_states=states, local_obs=torch.zeros(3, 4, 5))
    compact = pool._compact(batch)
    assert compact['global_state_per_step'].shape == (3, 2)
    assert torch.equal(pool.expand(compact)['global_states'], states)
    with pytest.raises(ValueError):
        pool._compact(dict(global_states=torch.rand(3, 4, 2), local_obs=torch.zeros(3, 4, 5)))


def test_a_batch_simulates_each_key_once_and_reuses_known_results():
    keys = ['a', 'b', 'a', 'c', 'b']
    run, first = pool.plan_batch(keys, known={})
    assert run == [0, 1, 3] and first == {'a': 0, 'b': 1, 'c': 3}
    run, _ = pool.plan_batch(keys, known={'b': {}})
    assert run == [0, 3]                                   # 'b' already has a result
    run, _ = pool.plan_batch(keys, known={'b': {}}, reuse=False)
    assert run == [0, 1, 3]                                # re-run once, to be checked against it


def test_attempt_names_never_reuse_an_existing_directory(tmp_path):
    assert pool.free_attempt(tmp_path, 'first') == 'first'
    (tmp_path / 'first').mkdir()
    assert pool.free_attempt(tmp_path, 'first') == 'first-2'
    (tmp_path / 'first-2').mkdir()
    assert pool.free_attempt(tmp_path, 'first') == 'first-3'
    assert pool.free_attempt(tmp_path, 'canary') == 'canary'


def test_priority_is_set_and_read_back_in_a_separate_process():
    import subprocess
    import sys
    script = 'from experiments.v15_pool import set_priority; print(set_priority("below_normal"))'
    done = subprocess.run([sys.executable, '-c', script], cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert done.returncode == 0 and done.stdout.strip() == 'below_normal'
    with pytest.raises(ValueError):
        pool.set_priority('realtime')


def test_pool_outputs_must_stay_inside_project_outputs(tmp_path):
    with pytest.raises(ValueError):
        pool.WorkerPool(1, tmp_path / 'pool')


def _networks_present():
    from traffic import highway_flow_rl_v14_contract as contract
    rows = contract.build_config(24)['validation_rows']
    return all((ROOT / rows[i]['case']['network']).is_file() for i in (8, 9))


@pytest.mark.skipif(not HAS_LIBSUMO, reason='libsumo is not installed in this environment')
@pytest.mark.timeout(900)
def test_pool_canary_trainer_parity_and_worker_independent_sampling():
    if not _networks_present():
        pytest.skip('the frozen LA networks are not in this checkout (outputs/ is not in Git)')
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-pool-{uuid.uuid4().hex[:8]}'
    try:
        with pool.WorkerPool(2, out) as workers:
            from experiments import run_highway_flow_rl_v12 as runner
            model, _ = runner.load_warm_start(workers.cfg)
            driver = workers.publish(model.state_dict())
            assert workers.publish(model.state_dict()) == driver
            validation = workers.run([pool.Task('v14-validation', r, driver) for r in (8, 9)])
            assert validation.canary['matched'] and validation.canary['different_worker']
            canary_record = next(r for r in validation.records if r['key'] == validation.canary['key'])
            assert validation.canary_decisions == canary_record['decisions'] > 0
            assert validation.charged_decisions == validation.decisions + validation.canary_decisions
            if TRAINER_RECORD.is_file():
                reports = json.loads(TRAINER_RECORD.read_text())['reports']
                for record in validation.records:
                    expected = pool._sha(pool._strip(json.loads(json.dumps(reports[record['fields']['row']], default=str))))
                    assert record['report_sha'] == expected
            krauss = workers.run([pool.Task('v14-validation', 9, driver, background='krauss')], canary=False)
            assert 'carFollowModel="Krauss"' in next(Path(krauss.records[0]['directory']).glob('episode-*/routes.rou.xml')).read_text()
            collect = workers.run([pool.Task('v14-validation', 9, driver, mode='collect', sample_seed=1, label_with=driver)])
            assert collect.canary['matched'] and collect.canary['different_worker']
            samples = pool.load_samples(collect.records[0])
            assert samples['labels'].shape[:2] == samples['local_obs'].shape[:2]
            index = [json.loads(line) for line in (out / 'dataset' / 'index.jsonl').read_text().splitlines()]
            assert index[0]['key'] == collect.records[0]['key']
            trips = pool.learner_round_trips(workers, model, [8, 9], 1, row_set='v14-validation')
            assert trips[0]['episodes'] == 2 and trips[0]['overhead_seconds'] >= 0
            # A key that already has a result: reused (twice in one batch, at no cost), or re-run and checked.
            again = workers.run([pool.Task('v14-validation', 8, driver, run='again'),
                                 pool.Task('v14-validation', 8, driver, run='twice')])
            assert again.simulated_tasks == 0 and again.reused_tasks == 2 and again.decisions == 0
            assert again.canary is None and [r['run'] for r in again.records] == ['again', 'twice']
            assert all(r['reused'] and r['report_sha'] == validation.records[0]['report_sha'] for r in again.records)
            checked = workers.run([pool.Task('v14-validation', 8, driver, run='check')], canary=False, reuse=False)
            assert checked.simulated_tasks == 1 and checked.repeats == dict(tasks=1, matched=True, mismatched={})
            assert Path(checked.records[0]['directory']).name == 'repeat'
        table = pool.read_results(out / 'results.jsonl')
        # 2 + 1 + 1 + 2 first runs and 1 checked re-run (canaries apart): 7 records of 6 keys
        assert len(table) == 7 and len({r['key'] for r in table}) == 6
        with pool.WorkerPool(1, out) as reopened:                # a pool reopened on the directory knows them
            assert len(reopened._known) == 6
            assert reopened.run([pool.Task('v14-validation', 9, driver)]).reused_tasks == 1
    finally:
        shutil.rmtree(out, ignore_errors=True)
