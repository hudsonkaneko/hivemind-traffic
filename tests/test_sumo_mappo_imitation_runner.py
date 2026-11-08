"""Budget, provenance and resume checks; synthetic packets never launch training."""
import copy
import gzip
import hashlib
import json
import time

import numpy as np
import pytest
import torch

from experiments import run_sumo_mappo as runner
from experiments import sumo_mappo_imitation as imitation
from experiments import sumo_mappo_rollout as rollout
from experiments.sumo_mappo_support import load_config
from policies import mappo


def config():
    value = load_config(runner.ROOT / 'experiments/configs/sumo_mappo_v4.json')
    value['model']['actor_hidden_sizes'] = [8]
    value['model']['critic_hidden_sizes'] = [8]
    value['training']['imitation'] = dict(enabled=True, episodes=2, epochs=1, minibatch_size=4,
        learning_rate=.0003, initial_log_std=-2., coefficient=1., lane_change_weight=10.)
    value['training']['max_agent_transitions'] = 64
    value['environment']['horizon'] = 2
    return value


class FakePool:
    forced_cleanup = []

    def __init__(self, *args):
        self.step = 0
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def request(self, commands):
        command, value = commands[0]
        agents = ['agent_' + str(i) for i in range(4)]
        if command == 'reset':
            self.step = 0
            return {0: dict(agents=agents, observations={a: np.full(81, self.step, np.float32) for a in agents})}
        if command == 'scripted_action':
            assert value is False
            return {0: dict(actions={a: np.array([self.step / 10., 0.], np.float32) for a in agents})}
        assert command == 'step'
        self.step += 1
        return {0: dict(agents=agents if self.step < 2 else [],
            observations={a: np.full(81, self.step, np.float32) for a in agents},
            rewards={a: 1. for a in agents}, infos={a: dict(progress_m=1.,
                truncation_cause='time_limit' if self.step == 2 else None) for a in agents})}


def test_collects_only_training_seeds_and_pairs_pre_step_labels(tmp_path, monkeypatch):
    cfg = config()
    monkeypatch.setattr(imitation, 'aggregate', lambda *args: {'accepted': True})
    obs, actions, record = imitation.collect_demonstrations(cfg, tmp_path / 'out', tmp_path / 'logs',
                                                           time.monotonic() + 30, pool_factory=FakePool)
    assert obs.shape == (16, 81) and actions.shape == (16, 2)
    torch.testing.assert_close(obs[:, 0] / 10., actions[:, 0])
    assert record['agent_transitions'] == 16 and record['next_episode_seed'] == cfg['training']['episode_seed_start'] + 2
    path = tmp_path / 'out/imitation/demonstrations.npz'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == record['dataset_sha256']
    with np.load(path) as data:
        assert set(data['seeds']) == set(record['training_seeds'])
        np.testing.assert_array_equal(data['observations'], obs.numpy())
    with gzip.open(tmp_path / 'out/imitation/actions.jsonl.gz', 'rt') as stream:
        assert len(stream.readlines()) == 4
    assert not record['teacher_at_inference']


def test_failed_demo_gate_preserves_actual_transition_evidence(tmp_path, monkeypatch):
    monkeypatch.setattr(imitation, 'aggregate', lambda *args: {'accepted': False})
    with pytest.raises(RuntimeError, match='demonstrations failed'):
        imitation.collect_demonstrations(config(), tmp_path / 'out', tmp_path / 'logs',
                                         time.monotonic() + 30, pool_factory=FakePool)
    record = json.loads((tmp_path / 'out/imitation/collection.json').read_text())
    assert record['status'] == 'failed' and record['agent_transitions'] == 16
    assert len(record['episodes']) == 2


@pytest.mark.parametrize('case', ['seed_leak', 'budget', 'deadline'])
def test_invalid_demo_request_never_opens_workers(tmp_path, case):
    cfg = config()
    deadline = time.monotonic() + 30
    if case == 'seed_leak':
        cfg['evaluation']['test_seeds'] = [cfg['training']['episode_seed_start']]
    elif case == 'budget':
        cfg['training']['max_agent_transitions'] = 16
    else:
        deadline = time.monotonic() - 1
    def forbidden(*args):
        pytest.fail('Invalid demonstration request opened workers')
    with pytest.raises((ValueError, TimeoutError)):
        imitation.collect_demonstrations(cfg, tmp_path / 'out', tmp_path / 'logs', deadline, pool_factory=forbidden)


@pytest.mark.parametrize('ppo_lane_weight', [None, 2.0])
def test_runner_counts_demos_retains_warmstart_candidate_and_skips_bc_on_resume(tmp_path, monkeypatch, ppo_lane_weight):
    cfg = config()
    if ppo_lane_weight is not None:
        cfg['training']['imitation']['ppo_lane_change_weight'] = ppo_lane_weight
    cfg['training']['max_agent_transitions'] = 8
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    counts = dict(demos=0, bc=0, updates=0)
    class Trainer:
        def __init__(self, model, **options):
            self.optimizer = torch.optim.Adam(model.parameters())
            assert options['imitation_coef'] == 1.
            self.imitation_lane_change_weight = options['imitation_lane_change_weight']
            assert self.imitation_lane_change_weight == 10.
        def behavior_clone(self, obs, targets, **options):
            counts['bc'] += 1
            assert options['seed'] == cfg['training']['seed']
            assert self.imitation_lane_change_weight == 10.
            return dict(optimizer_steps=3)
        def update(self, batch):
            counts['updates'] += 1
            assert self.imitation_lane_change_weight == (10. if ppo_lane_weight is None else ppo_lane_weight)
            return dict(optimizer_steps=1)
    class Pool:
        forced_cleanup = []
        def __init__(self, *args): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
    class Collector:
        total_agent_transitions = 4
        def __init__(self, *args, **kwargs):
            assert kwargs['seed_start'] == cfg['training']['episode_seed_start'] + 1
            self.next_seed = kwargs['seed_start'] + 4
        def collect(self, steps, remaining):
            assert remaining == 4
            return {'synthetic': True}, [], 4
    def collect(config, out, logs, deadline):
        counts['demos'] += 1
        (out / 'imitation').mkdir()
        return torch.zeros(4, 81), torch.zeros(4, 2), dict(agent_transitions=4,
            next_episode_seed=cfg['training']['episode_seed_start'] + 1, teacher_at_inference=False)
    def evaluate(config, kind, seeds, out, logs, tag, **kwargs):
        return dict(summary=dict(selection_score=[10 if tag == 'validation-0' else 9], accepted=True))
    monkeypatch.setattr(mappo, 'MappoTrainer', Trainer)
    monkeypatch.setattr(mappo, 'export_actor', lambda model, path: path.write_bytes(b'actor'))
    monkeypatch.setattr(imitation, 'collect_demonstrations', collect)
    monkeypatch.setattr(rollout, 'RolloutCollector', Collector)
    monkeypatch.setattr(runner, 'SumoWorkerPool', Pool)
    monkeypatch.setattr(runner, 'evaluate', evaluate)
    out, logs = tmp_path / 'outputs/first', tmp_path / 'logs/first'
    out.mkdir(parents=True); logs.mkdir(parents=True)
    result = runner.train(cfg, out, logs, time.monotonic() + 30)
    assert result['agent_transitions'] == 8 and result['ppo_agent_transitions'] == 4
    assert result['optimizer_steps'] == 4 and result['imitation']['warm_start_completed']
    best = torch.load(out / 'checkpoints/best.pt', weights_only=True)
    latest = torch.load(out / 'checkpoints/latest.pt', weights_only=True)
    assert best['update'] == 0 and best['agent_transitions'] == 4
    assert latest['update'] == 1 and latest['agent_transitions'] == 8
    new_out, new_logs = tmp_path / 'outputs/resume', tmp_path / 'logs/resume'
    new_out.mkdir(parents=True); new_logs.mkdir(parents=True)
    resumed = runner.train(cfg, new_out, new_logs, time.monotonic() + 30, resume=out / 'checkpoints/best.pt')
    assert counts == dict(demos=1, bc=1, updates=2)
    assert resumed['agent_transitions'] == 8 and resumed['imitation'] == result['imitation']
    assert resumed['optimizer_steps'] == 4  # Three restored BC steps plus one new PPO step.
