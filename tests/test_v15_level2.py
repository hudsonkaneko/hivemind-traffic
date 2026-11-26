"""v15 Phase 3: the Level 2 split, the learned Level 2, the trainer and the training rows."""
import importlib.util
import json
import random
import shutil
import uuid
from pathlib import Path

import pytest
import torch

from experiments import v15_level2_train as T
from experiments import v15_ledger as ledger
from experiments import v15_pool as pool
from traffic import highway_flow_rl_v14_contract as contract
from traffic import v15_benchmark as bench
from traffic import v15_level2 as L
from traffic import v15_rule_expert as v3
from traffic import v15_training_rows as training

ROOT = Path(__file__).resolve().parents[1]
HAS_LIBSUMO = importlib.util.find_spec('libsumo') is not None


def _stream(rng):
    return [(rng.uniform(-50, 300), rng.uniform(0, 30), f'c{j}',
             None if rng.random() < .5 else (rng.uniform(1, 3), rng.uniform(3, 6), rng.uniform(.8, 2), rng.uniform(1, 3)))
            for j in range(rng.randint(0, 6))]


def test_gap_selection_with_a_minimum_gap_is_v3s_up_to_its_own_count():
    rng = random.Random(7)
    later = 0
    for _ in range(3000):
        stop, speed, stream = rng.uniform(1, 150), rng.uniform(0, 25), _stream(rng)
        plan = v3.select_gap(stop, speed, stream, 5., 20.)
        assert L.select_gap_from(stop, speed, stream, 5., 20.) == plan
        for skip in range(len(plan['waited_for']) + 1):
            assert L.select_gap_from(stop, speed, stream, 5., 20., skip=skip) == plan
        waited = L.select_gap_from(stop, speed, stream, 5., 20., skip=len(plan['waited_for']) + 1)
        assert len(waited['waited_for']) >= len(plan['waited_for'])          # a later gap only ever waits longer
        later += waited != plan
    assert later > 0


def test_a_later_gap_is_still_judged_by_the_safety_test():
    rng = random.Random(3)
    for _ in range(2000):
        stop, speed, stream = rng.uniform(1, 150), rng.uniform(0, 25), _stream(rng)
        for skip in range(4):
            plan = L.select_gap_from(stop, speed, stream, 5., 20., skip=skip)
            if plan['rear'] is not None:
                assert plan['response'] >= -v3.SETTINGS['entry_safe_m_s2'] and plan['gap_at_entry'] > 0
                assert len(plan['waited_for']) >= skip


def test_gap_choice_caps_at_three_and_offsets_never_pass_the_free_road_target():
    assert [L.gap_index(w) for w in range(6)] == [0, 1, 2, 3, 3, 3]
    assert L.apply_offset(20., 0., 26.) == 20.
    assert L.apply_offset(20., 2., 26.) == 22. and L.apply_offset(25.5, 2., 26.) == 26.
    assert L.apply_offset(27., 2., 26.) == 27.                     # never above the larger of the two
    assert L.apply_offset(1., -2., 26.) == 0.


def test_inputs_leave_out_own_acceleration_and_previous_request():
    observation = list(range(121))
    executor = dict(entry_active=1., committed=0., dilemma=0., mandatory_left=1., mandatory_right=0.)
    row = L.features(observation, executor)
    assert len(row) == L.INPUT_SIZE == 124
    assert 1 not in row[:119] and 5 not in row[:119] and row[:2] == [0., 2.] and row[-5:] == [1., 0., 0., 1., 0.]


def test_level2_net_heads_and_choice():
    torch.manual_seed(0)
    net = L.build_net()
    lane, gap, offset = net(torch.zeros(2, L.INPUT_SIZE))
    assert lane.shape == (2, 3) and gap.shape == (2, 4) and offset.shape == (2, 5)
    choice = L.net_choice(net, [0.] * L.INPUT_SIZE)
    assert choice['lane'] in L.LANES and 0 <= choice['gap'] < L.GAPS and choice['offset'] in L.OFFSETS


def test_level2_tasks_validate_collect_only_with_rule_labels_and_perturb_only_the_rule():
    rule = L.RULE_DRIVER
    pool.Task('v15-training', 0, rule, mode='collect', label_with=rule, perturb='p1')
    pool.Task('v15-training', 0, 'l2:0123456789abcdef', mode='collect', label_with=rule, sample_seed=1)
    pool.Task('v15-development', 0, 'l2:0123456789abcdef')
    for bad in (dict(driver=rule, mode='collect'), dict(driver=rule, label_with=rule),
                dict(driver='l2:0123456789abcdef', mode='collect', label_with=rule, perturb='p1'),
                dict(driver=rule, mode='collect', label_with=rule, perturb='p2'),
                dict(driver='policy:abc', perturb='p1'), dict(driver='l2:xyz', mode='collect', label_with=rule)):
        with pytest.raises(ValueError):
            pool.Task(**{**dict(row_set='v15-training', row=0), **bad})


def test_perturbation_enters_the_key_only_when_set():
    row = dict(case=dict(id='case-a'), seed=11)
    plain = pool.key_fields(pool.Task('v14-validation', 3, 'policy:abc'), 24, row)
    assert 'perturb' not in plain
    a = pool.Task('v15-training', 3, L.RULE_DRIVER, mode='collect', label_with=L.RULE_DRIVER)
    b = pool.Task('v15-training', 3, L.RULE_DRIVER, mode='collect', label_with=L.RULE_DRIVER, perturb='p1')
    assert pool.task_key(pool.key_fields(a, 24, row)) != pool.task_key(pool.key_fields(b, 24, row))


@pytest.fixture(scope='module')
def round24():
    return contract.build_config(24)


def test_training_rows_are_round24s_twenty_at_new_disjoint_seeds(round24):
    rows = training.rows(round24)
    assert len(rows) == 20 * training.SETS
    for k in (0, 7, 39):
        for i in range(20):
            row = rows[training.index(k, i)]
            assert row['seed'] == 9_700_000 + 1000 * k + i
            assert dict(row, seed=round24['training_rows'][i]['seed']) == round24['training_rows'][i]
    seeds = {r['seed'] for r in rows}
    used = set()
    for round_index in range(1, 25):
        for critic in (False, True):
            for cursor in range(400):
                base = round24['training_rows'][cursor % 20]['seed'] + 1000 * (cursor // 20)
                used.add(base + 10000 * (round_index - 1) + (50000 if critic else 0))
    used |= set(bench.DEVELOPMENT_SEEDS) | set(bench.HELD_OUT_SEEDS) | set(bench.V02_HELD_OUT) | set(bench.V14_DIAGNOSIS)
    used |= {r['seed'] for r in round24['validation_rows']} | {r['seed'] for r in round24['held_out_rows']}
    assert not seeds & used and min(seeds) >= bench.TRAINING_SEEDS_FROM
    with pytest.raises(ValueError):
        training.seed(40, 0)


def _dataset(n, seed=0):
    g = torch.Generator().manual_seed(seed)
    return dict(x=torch.rand(n, L.INPUT_SIZE, generator=g) * 2 - 1, lane=torch.randint(0, 3, (n,), generator=g),
                gap=torch.randint(-1, 4, (n,), generator=g), offset=torch.full((n,), 2),
                lane_valid=torch.rand(n, generator=g) > .1, gap_valid=torch.rand(n, generator=g) > .5,
                weight=torch.ones(n))


def test_the_fit_is_deterministic_and_learns_a_simple_rule():
    data = _dataset(3000)
    data['lane'] = (data['x'][:, 0] > 0).long() + 1                 # lane 0 or +1 by the first input
    settings = dict(T.FIT, epochs=8, batch=256)
    a, report = T.fit(data, seed=3, held_back=data, settings=settings)
    b, _ = T.fit(data, seed=3, settings=settings)
    assert all(torch.equal(x, y) for x, y in zip(a.state_dict().values(), b.state_dict().values()))
    assert report['train']['lane']['accuracy'] > .95 and report['train']['offset']['accuracy'] == 1.
    c, _ = T.fit(data, seed=4, settings=settings)
    assert not all(torch.equal(x, y) for x, y in zip(a.state_dict().values(), c.state_dict().values()))


def test_the_fit_caps_its_updates_on_large_data():
    settings = dict(T.FIT, epochs=2, batch=100, max_samples_per_epoch=1000)
    _, small = T.fit(_dataset(800), seed=0, settings=settings)
    _, large = T.fit(_dataset(5000), seed=0, settings=settings)
    assert small['steps'] == 2 * 8 and large['steps'] == 2 * 10


def test_invalid_choices_carry_no_loss():
    data = _dataset(500)
    data['gap_valid'][:] = False
    data['gap'][:] = -1
    _, report = T.fit(data, seed=0, settings=dict(T.FIT, epochs=1, batch=100))
    assert report['train']['gap']['samples'] == 0 and report['train']['gap']['loss'] is None


def test_the_smallest_gap_label_is_zero_where_the_gap_applies_and_leaves_the_data_alone():
    data = _dataset(600)
    recorded = data['gap'].clone()
    assert T.relabel_gap(data, 'rule') is data
    out = T.relabel_gap(data, 'smallest')
    assert torch.equal(data['gap'], recorded)                                 # the input is not changed
    assert (out['gap'][data['gap_valid']] == 0).all()
    assert torch.equal(out['gap'][~data['gap_valid']], recorded[~data['gap_valid']])
    assert all(out[k] is data[k] for k in T.FIELDS if k != 'gap')
    with pytest.raises(ValueError):
        T.relabel_gap(data, 'largest')
    _, report = T.fit(out, seed=0, settings=dict(T.FIT, epochs=3, batch=100))
    assert report['train']['gap']['accuracy'] == 1.


def test_gap_zero_gives_the_rules_plan_whatever_the_rule_waits_for():
    rng = random.Random(11)
    for _ in range(3000):
        stop, speed, stream = rng.uniform(1, 150), rng.uniform(0, 25), _stream(rng)
        label = L.gap_index(len(v3.select_gap(stop, speed, stream, 5., 20.)['waited_for']))
        rule = L.select_gap_from(stop, speed, stream, 5., 20., skip=label)
        assert L.select_gap_from(stop, speed, stream, 5., 20., skip=0) == rule


def test_phase3_fits_relabel_the_gap_only_when_the_spec_asks(monkeypatch):
    from experiments import v15_phase3 as phase3
    files = [dict(path='a', seed_set=0, decisions=1), dict(path='b', seed_set=phase3.HELD_BACK[0], decisions=1)]
    monkeypatch.setattr(phase3, '_data_files', lambda steps: files)
    monkeypatch.setattr(T, 'load', lambda paths: _dataset(300, seed=len(paths[0])))
    seen = []
    monkeypatch.setattr(T, 'fit', lambda data, seed, held_back, log: seen.append((data, held_back)) or (None, {}))
    _, report = phase3._fit('relabel', dict(name='r', seed=1, data_steps=['data'], gap_label='smallest'), None)
    data, held = seen[-1]
    assert report['gap_label'] == 'smallest'
    assert (data['gap'][data['gap_valid']] == 0).all() and (held['gap'][held['gap_valid']] == 0).all()
    _, report = phase3._fit('final', dict(name='f', seed=4, data_steps=['data']), None)
    assert report['gap_label'] == 'rule' and (seen[-1][0]['gap'][seen[-1][0]['gap_valid']] > 0).any()


def test_phase3_segments_pass_the_ledger_totals(tmp_path):
    from experiments.run_highway_flow_rl_v05 import LEDGER_SCHEMA
    book = dict(schema=LEDGER_SCHEMA, inherited_seconds=0., segments=[
        dict(command=ledger.COMMAND, category='preparation', v15_phase='3', status='complete', charged_seconds=120.,
             decisions=300_000, episodes=0, evaluation_episodes=1, training_episodes=200, fits=0, fit_seconds=0.),
        dict(command=ledger.COMMAND, category='preparation', v15_phase='3', status='complete', charged_seconds=400.,
             decisions=400_000, episodes=0, evaluation_episodes=211, training_episodes=0, fits=3, fit_seconds=90.5)])
    totals = ledger.totals(book)
    assert totals['v15']['3'] == dict(segments=2, decisions=700_000, seconds=520., evaluation_episodes=212,
                                      training_episodes=200, fits=3, fit_seconds=90.5)
    assert totals['total_decisions'] == 0          # the v05 training-decision total is untouched


def _networks_present(rows):
    validation = contract.build_config(24)['validation_rows']
    return all((ROOT / validation[i]['case']['network']).is_file() for i in rows)


@pytest.mark.skipif(not HAS_LIBSUMO, reason='libsumo is not installed in this environment')
@pytest.mark.timeout(900)
def test_the_split_reproduces_expert_v3_and_collects_labelled_samples():
    rows = (4,)
    if not _networks_present(rows):
        pytest.skip('the frozen networks are not in this checkout (outputs/ is not in Git)')
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-level2-{uuid.uuid4().hex[:8]}'
    try:
        with pool.WorkerPool(2, out, priority='below_normal') as workers:
            tasks = [pool.Task('v14-validation', 4, v3.DRIVER), pool.Task('v14-validation', 4, L.RULE_DRIVER),
                     pool.Task('v14-validation', 4, L.RULE_DRIVER, mode='collect', label_with=L.RULE_DRIVER),
                     pool.Task('v14-validation', 4, L.RULE_DRIVER, mode='collect', label_with=L.RULE_DRIVER, perturb='p1')]
            batch = workers.run(tasks)
        expert, split, collect, perturbed = batch.records
        assert batch.canary['matched']
        assert (split['report_sha'], split['trace_sha']) == (expert['report_sha'], expert['trace_sha'])
        assert collect['trace_sha'] == expert['trace_sha'] and collect['level2_samples'] > 0
        assert perturbed['level2']['acted']['perturbed'] > 0
        samples = torch.load(collect['samples'], weights_only=False)
        assert samples['x'].shape[1] == L.INPUT_SIZE and int(samples['offset'].unique().item()) == L.OFFSETS.index(0.)
        index = [json.loads(line) for line in (out / 'dataset' / 'index.jsonl').read_text().splitlines()]
        assert len(index) == 2
    finally:
        shutil.rmtree(out, ignore_errors=True)


def test_probes_change_one_choice_of_the_rules_and_validate_only():
    label = dict(lane=1, reason='rule', gap=1, waited=1, offset=0.)
    assert L.probe_choice(label, L.PROBES['l2:probe-offset-m2']) == dict(label, offset=-2.)
    assert L.probe_choice(label, L.PROBES['l2:probe-gap-p1']) == dict(label, gap=2)
    assert L.probe_choice(dict(label, gap=3, waited=5), L.PROBES['l2:probe-gap-p1'])['gap'] == 3     # Level 2's largest
    assert L.probe_choice(dict(label, gap=None), L.PROBES['l2:probe-gap-p1'])['gap'] is None
    with pytest.raises(ValueError):
        L.probe_choice(label, dict(offset=3.))
    assert sorted(L.PROBES) == ['l2:probe-gap-p1', 'l2:probe-offset-m1', 'l2:probe-offset-m2', 'l2:probe-offset-p1',
                                'l2:probe-offset-p2']
    for probe in L.PROBES:
        assert pool.is_level2(probe)
        pool.Task('v15-development', 0, probe)
        with pytest.raises(ValueError):
            pool.Task('v15-training', 0, probe, mode='collect', label_with=L.RULE_DRIVER)
    driver = L.Level2Driver(lambda: None, probe='l2:probe-offset-p1')
    assert driver.stats['acted'] == dict(rule=0, learner=0, perturbed=0, probe=0)
    assert 'probe' not in L.Level2Driver(lambda: None).stats['acted']
    for bad in (dict(probe='l2:probe-offset-p3'), dict(probe='l2:probe-gap-p1', collect=True),
                dict(probe='l2:probe-gap-p1', net=L.build_net())):
        with pytest.raises(ValueError):
            L.Level2Driver(lambda: None, **bad)
    from experiments import v15_phase3 as phase3
    phase3._check_policies(ROOT, [dict(row_set=phase3.DEV, row=0, driver=p) for p in L.PROBES])   # no weights needed


def test_probe_events_under_the_stop_rule_stop_step_a():
    from experiments import v15_phase3 as phase3
    from experiments import v15_phase4 as phase4
    phase4._stop_on_probe_events(dict(probes={'l2:probe-gap-p1': dict(new_failures=[])}))
    with pytest.raises(phase3.StopAndAsk):
        phase4._stop_on_probe_events(dict(probes={'l2:probe-gap-p1': dict(new_failures=[]),
                                                  'l2:probe-offset-p2': dict(new_failures=[dict(row=24, cut_ins=1)])}))


def test_a_probe_drives_the_rules_executor_with_its_change():
    rows = (4,)
    if not _networks_present(rows):
        pytest.skip('the frozen networks are not in this checkout (outputs/ is not in Git)')
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-probe-{uuid.uuid4().hex[:8]}'
    try:
        with pool.WorkerPool(2, out, priority='below_normal') as workers:
            batch = workers.run([pool.Task('v14-validation', 4, L.RULE_DRIVER),
                                 pool.Task('v14-validation', 4, 'l2:probe-offset-m2')])
        rule, probe = batch.records
        assert batch.canary['matched'] and probe['level2']['acted']['probe'] == probe['level2']['decisions'] > 0
        assert probe['trace_sha'] != rule['trace_sha']
        steps = [json.loads(line) for d in (out / 'episodes').iterdir() if d.name == probe['key'][:16]
                 for line in next(d.rglob('level2.jsonl')).read_text().splitlines()]
        assert steps and all(s['level2']['acted']['offset'] == -2. and s['level2']['actor'] == 'probe' for s in steps)
    finally:
        shutil.rmtree(out, ignore_errors=True)


def test_phase3_rollout_collisions_and_open_learner_failures_stop_the_run():
    from types import SimpleNamespace
    from experiments import v15_phase3 as phase3
    tasks = [dict(row_set=phase3.TRAIN, row=3), dict(row_set=phase3.DEV, row=52, parity=True)]
    calm = SimpleNamespace(records=[dict(events=dict(collisions=0, free_road_stops=2, cut_ins=1)), dict(events={})])
    phase3._rollout_checks(calm, tasks)                       # other rollout events are reported, not stop points
    crash = SimpleNamespace(records=[dict(events=dict(collisions=1)), dict(events={})])
    with pytest.raises(phase3.StopAndAsk):
        phase3._rollout_checks(crash, tasks)
    phase3._no_open_failures([dict(id='l2:a', step='clone', new_failures=[])])
    with pytest.raises(phase3.StopAndAsk):
        phase3._no_open_failures([dict(id='l2:a', step='clone', new_failures=[dict(row=1, collisions=1)])])


def test_phase3_publishes_an_earlier_steps_rollout_learner_and_checks_its_hash(monkeypatch, tmp_path):
    from types import SimpleNamespace
    from experiments import v15_phase3 as phase3
    torch.manual_seed(0)
    state = L.build_net().state_dict()
    learner = 'l2:' + pool.state_hash(state)
    (tmp_path / 'clone' / 'policies').mkdir(parents=True)
    torch.save(state, tmp_path / 'clone' / 'policies' / f'{learner[3:]}.pt')
    monkeypatch.setattr(phase3, 'out_dir', lambda step: tmp_path / step)
    published = []
    fake = SimpleNamespace(publish_level2=lambda s: published.append(s) or 'l2:' + pool.state_hash(s))
    assert phase3._bring_learner(fake, dict(inputs=dict(rollout_learner=learner, rollout_learner_step='clone'))) == learner
    assert phase3._bring_learner(fake, dict(inputs={})) is None and len(published) == 1
    with pytest.raises(ValueError):
        phase3._bring_learner(SimpleNamespace(publish_level2=lambda s: 'l2:0000'),
                              dict(inputs=dict(rollout_learner=learner, rollout_learner_step='clone')))


def test_phase3_refuses_to_dispatch_a_learner_whose_weights_are_not_published(tmp_path):
    from experiments import v15_phase3 as phase3
    (tmp_path / 'policies').mkdir()
    (tmp_path / 'policies' / '0123456789abcdef.pt').write_bytes(b'x')
    rule = dict(row_set=phase3.TRAIN, row=0, driver=L.RULE_DRIVER)
    present = dict(row_set=phase3.TRAIN, row=1, driver='l2:0123456789abcdef', label_with=L.RULE_DRIVER)
    phase3._check_policies(tmp_path, [rule, present])
    with pytest.raises(FileNotFoundError):
        phase3._check_policies(tmp_path, [rule, present, dict(present, driver='l2:fedcba9876543210')])
