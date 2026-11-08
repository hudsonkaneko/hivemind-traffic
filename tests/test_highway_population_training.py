from copy import deepcopy
import json
from pathlib import Path
import time

import numpy as np
import pytest
import torch

from experiments import run_highway_population as runner
from policies.mappo import MappoTrainer


def config():
    return runner.load_config(runner.DEFAULT)


def test_configuration_is_disabled_bounded_and_partitioned():
    cfg = config()
    assert cfg['training_authorized'] is False
    assert cfg['training']['max_agent_transitions'] == 100000
    assert cfg['training']['max_wall_seconds'] == 1800
    assert cfg['environment']['horizon'] == 800
    assert set(r['controlled_count'] for r in cfg['validation_cases']) == {1, 3, 5, 7, 8}
    assert min(r['controlled_count'] / r['total_vehicles'] for r in cfg['validation_cases']) == 1 / 24
    assert max(r['controlled_count'] / r['total_vehicles'] for r in cfg['validation_cases']) == 1.
    rows = [runner.training_case(cfg, i) for i in range(180)]
    assert set(r['controlled_count'] for r in rows) == {1, 2, 4, 6, 8}
    assert set(r['total_vehicles'] for r in rows) == {8, 16, 24}
    assert set(r['case']['task'] for r in rows) == {'cruise', 'following', 'passing', 'merge'}
    assert rows == [runner.training_case(cfg, i) for i in range(180)]
    assert all(r['case']['split'] == 'train' for r in rows)
    assert not set(r['seed'] for r in rows) & set(r['seed'] for r in cfg['validation_cases'])


def test_training_schedule_never_uses_all_av_slow_leader_case():
    cfg = config()
    for i in range(180):
        row = runner.training_case(cfg, i)
        if row['case']['task'] in ('following', 'passing'):
            assert row['controlled_count'] < row['total_vehicles']


@pytest.mark.parametrize('edit', [
    lambda c: c['training'].update(max_agent_transitions=100001),
    lambda c: c['training'].update(max_wall_seconds=1801),
    lambda c: c['validation_cases'][0].update(seed=c['training']['seed_start']),
    lambda c: c['training_cases'][0].update(split='validation'),
    lambda c: c['model'].update(population_critic=False),
    lambda c: c['population'].update(training_counts=[1, 3, 5]),
])
def test_invalid_configuration_rejected(edit):
    cfg = config(); edit(cfg)
    with pytest.raises(ValueError):
        runner.validate_config(cfg)


def test_preparation_fingerprint_ignores_only_authorization_and_readiness():
    cfg = config(); other = deepcopy(cfg)
    other['training_authorized'] = True
    other['readiness'] = dict(summary_path='somewhere')
    assert runner.preparation_digest(cfg) == runner.preparation_digest(other)
    other['optimizer']['actor_lr'] *= 2
    assert runner.preparation_digest(cfg) != runner.preparation_digest(other)


def test_atomic_write_retries_transient_replacement_and_preserves_prior(tmp_path, monkeypatch):
    path = tmp_path / 'last.pt'; path.write_bytes(b'previous')
    real = runner.os.replace
    calls = []
    def retry(src, dst):
        calls.append(src)
        if len(calls) < 3:
            assert path.read_bytes() == b'previous'
            raise PermissionError('test busy reader')
        real(src, dst)
    monkeypatch.setattr(runner.os, 'replace', retry)
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None)
    runner.atomic_write(path, lambda p: p.write_bytes(b'next'))
    assert len(calls) == 3 and path.read_bytes() == b'next'
    assert list(tmp_path.iterdir()) == [path]


def test_atomic_write_persistent_failure_preserves_old_checkpoint(tmp_path, monkeypatch):
    path = tmp_path / 'last.pt'; path.write_bytes(b'previous')
    def fail(*_):
        raise PermissionError('persistent')
    monkeypatch.setattr(runner.os, 'replace', fail)
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None)
    with pytest.raises(PermissionError):
        runner.atomic_write(path, lambda p: p.write_bytes(b'next'), attempts=2)
    assert path.read_bytes() == b'previous'
    assert list(tmp_path.iterdir()) == [path]


def critic_batch():
    states = torch.zeros(1, 8, 752)
    states[..., 93] = 1.
    return dict(global_states=states, agent_indices=torch.arange(8)[None],
        active=torch.tensor([[True, False, False, False, False, False, False, False]]),
        returns=torch.ones(1, 8))


def test_critic_warmup_updates_values_but_freezes_actor_and_exploration():
    torch.set_num_threads(1)
    model = runner.model_for(config())
    trainer = MappoTrainer(model, **config()['optimizer'])
    before = runner.actor_digest(model)
    critic = {k: v.clone() for k, v in model.critic.state_dict().items()}
    result = runner.critic_update(model, trainer, critic_batch(), time.monotonic() + 30)
    assert result['optimizer_steps'] == 4
    assert runner.actor_digest(model) == before
    assert any(not torch.equal(v, model.critic.state_dict()[k]) for k, v in critic.items())


def test_expired_critic_update_returns_zero_count_without_hidden_mutations():
    model = runner.model_for(config())
    trainer = MappoTrainer(model, **config()['optimizer'])
    before = {k: v.clone() for k, v in model.state_dict().items()}
    result = runner.critic_update(model, trainer, critic_batch(), time.monotonic() - 1)
    assert result['budget_exhausted'] and result['optimizer_steps'] == 0
    assert all(torch.equal(v, model.state_dict()[k]) for k, v in before.items())


def report():
    return dict(agent_steps=100, overrides=0, collisions=0, teleports=0, overspeed_steps=0,
        intervention_fraction=0., mean_fraction_of_speed_limit=.95, passes=1, successful_merges=1,
        merge_target_count=1, task='merge', accounting_comparison_valid=True,
        partial_control_horizon=False, case_id='merge', seed=1, population_setting={})


def test_variable_merge_denominator_and_nonfinite_metrics():
    row = report()
    assert runner.summarize([row])['nominal_gates_passed']
    row['merge_target_count'] = 2
    assert not runner.summarize([row])['nominal_gates_passed']
    row['intervention_fraction'] = float('nan')
    with pytest.raises(ValueError, match='nonfinite'):
        runner.summarize([row])


def test_true_native_zero_population_is_not_actor_acceptance():
    row = report(); row.update(agent_steps=0, task='cruise', mean_fraction_of_speed_limit=0.)
    summary = runner.summarize([row])
    assert not summary['nominal_gates_passed']


def test_readiness_requires_exact_config_source_and_zero_learning(tmp_path, monkeypatch):
    cfg = config()
    root = tmp_path; output = root / 'outputs'; output.mkdir()
    summary = output / 'summary.json'
    value = dict(passed=True, optimizer_steps=0, source_hashes={'x': 'y'},
                 preparation_sha256=runner.preparation_digest(cfg))
    summary.write_text(json.dumps(value))
    cfg['readiness'] = dict(summary_path='outputs/summary.json', summary_sha256=runner.digest(summary))
    monkeypatch.setattr(runner, 'ROOT', root)
    monkeypatch.setattr(runner, 'sources', lambda: {'x': 'y'})
    runner.readiness_check(cfg)
    value['optimizer_steps'] = 1; summary.write_text(json.dumps(value))
    cfg['readiness']['summary_sha256'] = runner.digest(summary)
    with pytest.raises(ValueError):
        runner.readiness_check(cfg)


def test_reset_failure_is_not_masked_by_uninitialized_control_statistics(tmp_path):
    class Ledger:
        def snapshot(self):
            return {'comparison_valid': False}
    class FailedReset:
        directory = tmp_path
        ledger = Ledger()
        closed = False
        def reset(self, seed):
            raise RuntimeError('Controlled spawn timeout')
        def report(self):
            raise KeyError('abs_acceleration_sum')
        def close(self):
            self.closed = True
    env = FailedReset()
    with pytest.raises(RuntimeError, match='Controlled spawn timeout'):
        runner.episode(env, {'seed': 1}, kind='teacher')
    retained = json.loads((tmp_path / 'interrupted-accounting.json').read_text())
    assert 'Controlled spawn timeout' in retained['original_error']
    assert 'abs_acceleration_sum' in retained['report_error']
    assert not retained['accounting_comparison_valid']
    assert env.closed


def test_collector_bootstraps_truncation_and_masks_true_terminal(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    def observation(value):
        obs = np.zeros(93, np.float32); obs[0] = value; obs[31] = 1.
        return obs
    class Env:
        directory = tmp_path
        case_id = 'fake'
        agents = ['agent_0', 'agent_1']
        def reset(self, seed):
            return {a: observation(.1) for a in self.agents}, {}
        def state(self):
            value = np.zeros((8, 94), np.float32); value[:2, 93] = 1.
            return value.reshape(-1)
        def step(self, _):
            self.agents = []
            return ({'agent_0': observation(.7), 'agent_1': observation(.9)},
                {'agent_0': 1., 'agent_1': 2.}, {'agent_0': False, 'agent_1': True},
                {'agent_0': True, 'agent_1': False}, {})
        def close(self):
            pass
    class Model:
        seen = []
        def act(self, local, deterministic):
            return torch.zeros(8, 2), torch.zeros(8), torch.zeros(8), torch.zeros(8, 2)
        def values(self, state, indices):
            self.seen.append(state.clone())
            return state[:, 0]
    monkeypatch.setattr(runner, 'primary_accounting', lambda *a, **k: {})
    model = Model()
    row = dict(seed=1, controlled_count=2, total_vehicles=8, communication='none', packet_loss=0.)
    _, batch = runner.episode(Env(), row, model=model, collect=True)
    final = model.seen[-1][0].reshape(8, 94)
    assert final[0, 0] == pytest.approx(.7) and final[0, 93] == 1
    assert final[1, 93] == 0
    assert batch['returns'][0, 0] == pytest.approx(1. + .99 * .7)
    assert batch['returns'][0, 1] == 2.
    assert not batch['active'][0, 2:].any()
