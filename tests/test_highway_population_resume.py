"""Fault-injected checkpoint recovery; tiny synthetic tensors, no SUMO or policy training."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from experiments import run_highway_population as runner


class SyntheticModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.actor = torch.nn.Linear(1, 1)
        self.critic = torch.nn.Linear(1, 1)
        self.log_std = torch.nn.Parameter(torch.zeros(1))


@pytest.fixture
def harness(monkeypatch, tmp_path):
    """Exercise the production train/checkpoint state machine with fake episodes."""
    torch.set_num_threads(1)
    clock = SimpleNamespace(now=100., epoch=1700000000.)
    events = SimpleNamespace(episodes=[], updates=0, evaluations=[],
        fail_episode=False, fail_update=False, fail_final=False)
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    monkeypatch.setattr(runner, 'time', SimpleNamespace(
        monotonic=lambda: clock.now, time=lambda: clock.epoch + clock.now,
        sleep=lambda seconds: None))
    def check_deadline(deadline):
        if deadline is not None and clock.now >= deadline:
            raise runner.PilotBudgetExhausted('Synthetic aggregate deadline exhausted')
    monkeypatch.setattr(runner, 'check_deadline', check_deadline)
    monkeypatch.setattr(runner, 'model_for', lambda cfg: SyntheticModel())

    def warm_start(model, cfg):
        with torch.no_grad():
            for parameter in model.parameters():
                parameter.fill_(.125)
        return {'source': 'synthetic frozen parent'}

    class SyntheticTrainer:
        def __init__(self, model, **kwargs):
            self.model = model
            self.optimizer = torch.optim.Adam(model.parameters(), lr=.01)

        def update(self, batch, deadline=None):
            runner.check_deadline(deadline)
            # All three random streams influence the result, exposing recovery drift.
            noise = torch.rand(1) + np.random.random() + runner.random.random()
            loss = sum((parameter.square().sum() + parameter.sum() * noise.sum())
                       for parameter in self.model.parameters()) * batch['x'].sum()
            self.optimizer.zero_grad(set_to_none=True)
            loss.backward()
            self.optimizer.step()
            clock.now += 1.
            events.updates += 1
            if events.fail_update:
                events.fail_update = False
                raise RuntimeError('injected failure after optimizer step')
            return {'optimizer_steps': 1, 'budget_exhausted': False}

    row = {'case': {'id': 'synthetic-cruise', 'task': 'cruise'},
           'controlled_count': 2, 'total_vehicles': 8,
           'communication': 'local', 'packet_loss': 0., 'seed': 700}
    cfg = {'training': {'max_wall_seconds': 30., 'final_evaluation_reserve_seconds': 5.,
                       'max_agent_transitions': 16, 'critic_warmup_transitions': 0},
           'environment': {'horizon': 4}, 'optimizer': {},
           'population': {'total_vehicles': [8]}, 'validation_cases': [deepcopy(row)]}
    monkeypatch.setattr(runner, 'warm_start', warm_start)
    monkeypatch.setattr(runner, 'MappoTrainer', SyntheticTrainer)
    monkeypatch.setattr(runner, 'training_case',
                        lambda cfg, index: dict(deepcopy(row), seed=700 + index))
    monkeypatch.setattr(runner, 'env_for', lambda cfg, path, row: SimpleNamespace(row=row))

    def report(row):
        return {'case_id': row['case']['id'], 'seed': row['seed'],
                'population_setting': {key: row[key] for key in
                    ('controlled_count', 'total_vehicles', 'communication', 'packet_loss')},
                'intervention_fraction': .01, 'mean_fraction_of_speed_limit': .95,
                'passes': 0, 'successful_merges': 0}

    def episode(env, row, *, limit=None, deadline=None, **kwargs):
        runner.check_deadline(deadline)
        clock.now += 1.
        events.episodes.append(row['seed'])
        x = torch.rand(1) + np.random.random() + runner.random.random()
        if events.fail_episode:
            events.fail_episode = False
            raise RuntimeError('injected incomplete episode')
        return dict(report(row), agent_transitions=min(limit, 8)), {'x': x}

    def evaluate(cfg, run, model, *, label, deadline, **kwargs):
        runner.check_deadline(deadline)
        events.evaluations.append((label, deadline, clock.now))
        clock.now += 2.
        if label == 'final-validation' and events.fail_final:
            events.fail_final = False
            raise RuntimeError('injected final evaluation failure')
        value = {'summary': {'cases': 1}, 'reports': [report(row)]}
        runner.atomic_json(run / (label + '.json'), value)
        return value

    def export_actor(model, path):
        torch.save(model.actor.state_dict(), path)
        return path

    monkeypatch.setattr(runner, 'episode', episode)
    monkeypatch.setattr(runner, 'evaluate', evaluate)
    monkeypatch.setattr(runner, 'export_actor', export_actor)

    def seed():
        torch.manual_seed(123)
        np.random.seed(123)
        runner.random.seed(123)

    def run(name, resume=None, config=None):
        directory = tmp_path / 'outputs' / name
        directory.mkdir(parents=True)
        result = runner.train(config or cfg, directory, clock.now, {'source.py': 'fixed'}, resume)
        return directory, result

    seed()
    return SimpleNamespace(clock=clock, events=events, cfg=cfg, root=tmp_path,
                           run=run, seed=seed)


def load(path):
    return torch.load(path, map_location='cpu', weights_only=False)


def assert_nested_equal(left, right):
    if isinstance(left, torch.Tensor):
        torch.testing.assert_close(left, right, rtol=0, atol=0)
    elif isinstance(left, np.ndarray):
        np.testing.assert_array_equal(left, right)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            assert_nested_equal(left[key], right[key])
    elif isinstance(left, (tuple, list)):
        assert len(left) == len(right)
        for a, b in zip(left, right):
            assert_nested_equal(a, b)
    else:
        assert left == right


def test_incomplete_episode_reservation_stays_charged_on_resume(harness):
    h = harness
    h.events.fail_episode = True
    with pytest.raises(RuntimeError, match='incomplete episode'):
        h.run('failed-episode')
    path = h.root / 'outputs/failed-episode/last.pt'
    saved = load(path)
    assert saved['state']['run_closed'] is True
    assert saved['state']['pending']['kind'] == 'episode'
    assert saved['state']['spent'] == 8 and saved['state']['actual'] == 0
    assert saved['state']['elapsed_seconds'] == 5.
    h.clock.now = 200.
    directory, result = h.run('resumed-episode', path)
    state = load(directory / 'last.pt')['state']
    assert state['spent'] == 16 and state['actual'] == 8
    assert state['ppo_actual'] == 8 and state['optimizer_steps'] == 1
    assert state['pending'] is None and state['episode_index'] == 2
    assert h.events.episodes == [700, 701]  # Failed seed is not silently repeated.
    assert result['counters']['elapsed_seconds'] == 9.
    assert [label for label, _, _ in h.events.evaluations].count('initial-validation') == 1


def test_pending_update_recovery_matches_uninterrupted_model_optimizer_and_rng(harness):
    h = harness
    reference_dir, _ = h.run('reference')
    reference = load(reference_dir / 'last.pt')
    h.seed()
    h.clock.now = 200.
    h.events.fail_update = True
    with pytest.raises(RuntimeError, match='after optimizer step'):
        h.run('failed-update')
    path = h.root / 'outputs/failed-update/last.pt'
    saved = load(path)
    assert saved['state']['pending']['kind'] == 'update'
    assert saved['state']['spent'] == saved['state']['actual'] == 8
    assert saved['state']['updates'] == saved['state']['optimizer_steps'] == 0
    assert saved['optimizer']['state'] == {}  # Partial optimizer step was rolled back.
    assert saved['state']['run_closed'] is True
    h.clock.now = 300.
    resumed_dir, result = h.run('resumed-update', path)
    resumed = load(resumed_dir / 'last.pt')
    for field in ('model', 'optimizer', 'torch_rng', 'numpy_rng', 'python_rng'):
        assert_nested_equal(reference[field], resumed[field])
    assert resumed['state']['spent'] == resumed['state']['actual'] == 16
    assert resumed['state']['optimizer_steps'] == resumed['state']['updates'] == 2
    assert resumed['state']['pending'] is None
    # The failed physical optimizer attempt consumes time, not a second data allocation.
    assert result['counters']['elapsed_seconds'] == 11.


def test_final_evaluation_failure_closes_checkpoint_and_keeps_remaining_wall_budget(harness):
    h = harness
    h.events.fail_final = True
    with pytest.raises(RuntimeError, match='final evaluation'):
        h.run('failed-final')
    path = h.root / 'outputs/failed-final/last.pt'
    saved = load(path)
    assert saved['state']['run_closed'] is True
    assert saved['state']['final_complete'] is False
    assert saved['state']['spent'] == saved['state']['actual'] == 16
    assert saved['state']['elapsed_seconds'] == 10.
    h.clock.now = 300.
    resumed_dir, result = h.run('resumed-final', path)
    resumed = load(resumed_dir / 'last.pt')
    assert h.events.episodes == [700, 701]  # No retraining after completed learning.
    assert h.events.evaluations[-1] == ('final-validation', 320., 300.)
    assert result['counters']['elapsed_seconds'] == 12.
    assert_nested_equal(saved['model'], resumed['model'])
    assert_nested_equal(saved['optimizer'], resumed['optimizer'])
    assert resumed['state']['final_complete'] and resumed['state']['run_closed']


def test_resume_rejects_configuration_drift_before_collecting_more_data(harness):
    h = harness
    h.events.fail_episode = True
    with pytest.raises(RuntimeError):
        h.run('original')
    changed = deepcopy(h.cfg)
    changed['training']['max_agent_transitions'] = 24
    with pytest.raises(ValueError, match='configuration/source mismatch'):
        h.run('changed', h.root / 'outputs/original/last.pt', config=changed)
    assert h.events.episodes == [700]
