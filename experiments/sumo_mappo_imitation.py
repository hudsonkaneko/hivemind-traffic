"""Training-only scripted demonstrations; no teacher is part of exported inference."""
from __future__ import annotations

import gzip
import hashlib
import time
from pathlib import Path

import numpy as np
import torch

from experiments.sumo_mappo_rollout import EpisodeMetrics, aggregate, journal_packet
from experiments.sumo_mappo_support import SumoWorkerPool, atomic_json


def collect_demonstrations(config, output, logs, deadline, *, pool_factory=SumoWorkerPool):
    """Collect actual training-seed transitions and retain data even on failure.

    Labels use the existing ordinary-headway scripted reference, from the same
    pre-step local scene available to the actor. They are requested separately
    from stepping and never substituted for learned actions during evaluation.
    """
    settings = config['training']
    imitation = settings['imitation']
    seeds = list(range(settings['episode_seed_start'], settings['episode_seed_start'] + imitation['episodes']))
    excluded = set(config['evaluation']['validation_seeds'] + config['evaluation']['test_seeds'] + config['preflight']['seeds'])
    if not seeds or seeds[-1] > settings['episode_seed_end'] or excluded.intersection(seeds):
        raise ValueError('Demonstrations require disjoint in-range training seeds')
    maximum = len(seeds) * config['environment']['horizon'] * config['model']['num_agents']
    if maximum + config['model']['num_agents'] > settings['max_agent_transitions']:
        raise ValueError('Demonstrations leave no PPO transition budget')
    def check_deadline():
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError('Imitation wall-time budget reached')
    check_deadline()
    directory, log_directory = Path(output) / 'imitation', Path(logs) / 'imitation'
    directory.mkdir(parents=True, exist_ok=False)
    log_directory.mkdir(parents=True, exist_ok=False)
    observations, targets, sample_seeds, agent_indices, episodes = [], [], [], [], []
    started = time.monotonic()
    metadata = dict(status='collecting', teacher='scripted_ordinary_headway', training_seeds=seeds,
                    next_episode_seed=seeds[-1] + 1, agent_transitions=0,
                    dataset='imitation/demonstrations.npz', journal='imitation/actions.jsonl.gz',
                    teacher_at_inference=False)
    try:
        with gzip.open(directory / 'actions.jsonl.gz', 'wt', encoding='utf-8') as journal:
            with pool_factory(config['environment'], 1, directory, log_directory,
                              config['execution']['worker_timeout_seconds']) as pool:
                for seed in seeds:
                    check_deadline()
                    packet = pool.request({0: ('reset', seed)})[0]
                    metrics = EpisodeMetrics(seed)
                    while packet['agents']:
                        check_deadline()
                        agents = list(packet['agents'])
                        local = np.stack([packet['observations'][agent] for agent in agents]).astype(np.float32)
                        actions = pool.request({0: ('scripted_action', False)})[0]['actions']
                        target = np.stack([actions[agent] for agent in agents]).astype(np.float32)
                        if (local.shape != (len(agents), config['model']['observation_size']) or
                            target.shape != (len(agents), 2) or not np.isfinite(local).all() or
                            not np.isfinite(target).all()):
                            raise ValueError('Invalid demonstration observations/actions')
                        if len(observations) + len(agents) > maximum:
                            raise RuntimeError('Demonstration transition accounting exceeded its bound')
                        packet = pool.request({0: ('step', actions)})[0]
                        observations.extend(local)
                        targets.extend(target)
                        sample_seeds.extend([seed] * len(agents))
                        agent_indices.extend(int(agent.rsplit('_', 1)[1]) for agent in agents)
                        metrics.add(packet)
                        journal_packet(journal, 'imitation', seed, actions, packet)
                    episodes.append(metrics.result())
            if pool.forced_cleanup:
                raise RuntimeError('Imitation workers required forced cleanup')
        summary = aggregate(episodes, config['evaluation']['acceptance'], config['evaluation'].get('driving_targets'))
        metadata.update(status='collected', summary=summary)
        if not summary['accepted']:
            raise RuntimeError('Scripted training demonstrations failed their driving/safety gate')
    except BaseException as error:
        metadata.update(status='failed', error=repr(error))
        raise
    finally:
        width = config['model']['observation_size']
        local_array = np.asarray(observations, dtype=np.float32).reshape(-1, width)
        action_array = np.asarray(targets, dtype=np.float32).reshape(-1, 2)
        np.savez_compressed(directory / 'demonstrations.npz', observations=local_array, teacher_actions=action_array,
                            seeds=np.asarray(sample_seeds, dtype=np.int64), agent_indices=np.asarray(agent_indices, dtype=np.int64))
        metadata.update(agent_transitions=len(local_array), collection_wall_seconds=time.monotonic() - started,
                        lane_change_labels=int(np.count_nonzero(action_array[:, 1])))
        metadata['dataset_sha256'] = hashlib.sha256((directory / 'demonstrations.npz').read_bytes()).hexdigest()
        if (directory / 'actions.jsonl.gz').exists():
            metadata['journal_sha256'] = hashlib.sha256((directory / 'actions.jsonl.gz').read_bytes()).hexdigest()
        atomic_json(directory / 'collection.json', dict(metadata, episodes=episodes))
    return torch.from_numpy(local_array), torch.from_numpy(action_array), metadata
