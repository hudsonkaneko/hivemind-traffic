"""MAPPO rollout collection with actual-agent budgets and terminal bootstraps."""
from __future__ import annotations

import json
import time

import numpy as np
import torch

from experiments.sumo_mappo_support import jsonable
from policies.mappo import compute_gae

AGENTS = tuple('agent_' + str(i) for i in range(4))


class EpisodeMetrics:
    def __init__(self, seed):
        self.seed = seed
        self.steps = self.transitions = self.interventions = self.overspeed = 0
        self.reward = 0.
        self.progress = {agent: 0. for agent in AGENTS}
        self.collisions = set()
        self.causes = {}
        self.peak_speed = 0.
        self.minimum_ttc = None
        self.background_peak = 0

    def add(self, packet):
        self.steps += 1
        rewards = packet['rewards']
        self.transitions += len(rewards)
        self.reward += sum(rewards.values()) / len(AGENTS)
        for agent, info in packet['infos'].items():
            self.progress[agent] += info.get('progress_m', 0.)
            self.interventions += int(info.get('safety_intervention', False))
            speed = info.get('speed_m_s') or 0.
            self.peak_speed = max(self.peak_speed, speed)
            self.overspeed += int(speed > 26.8224 + 1e-5)
            self.collisions.update(info.get('collisions', []))
            self.background_peak = max(self.background_peak, info.get('background_vehicle_count', 0))
            ttc = info.get('min_ttc_s')
            if ttc is not None:
                self.minimum_ttc = ttc if self.minimum_ttc is None else min(self.minimum_ttc, ttc)
            cause = info.get('termination_cause') or info.get('truncation_cause')
            if cause:
                self.causes[agent] = cause

    def result(self):
        return dict(seed=self.seed, steps=self.steps, agent_transitions=self.transitions,
                    team_reward=self.reward, mean_progress_m=float(np.mean(list(self.progress.values()))),
                    progress_by_agent_m=self.progress, collision_vehicles=len(self.collisions),
                    collision_ids=sorted(self.collisions), interventions=self.interventions,
                    intervention_fraction=self.interventions / max(1, self.transitions),
                    overspeed_steps=self.overspeed, peak_speed_m_s=self.peak_speed,
                    min_ttc_s=self.minimum_ttc, background_peak=self.background_peak,
                    end_causes=self.causes,
                    nominal_end=len(self.causes) == 4 and all(c in ('time_limit', 'route_complete') for c in self.causes.values()))


def journal_packet(stream, worker, seed, actions, packet):
    if stream is None:
        return
    infos = {agent: {key: value for key, value in info.items() if key != 'terminal_state'}
             for agent, info in packet['infos'].items()}
    stream.write(json.dumps(jsonable(dict(worker=worker, seed=seed, actions=actions, infos=infos)),
                            separators=(',', ':'), allow_nan=False) + '\n')


class RolloutCollector:
    def __init__(self, pool, model, config, *, journal=None, deadline=None, seed_start=None, seed_end=None):
        self.pool, self.model, self.config = pool, model, config
        self.journal, self.deadline = journal, deadline
        self.next_seed = config['training']['episode_seed_start'] if seed_start is None else seed_start
        self.seed_end = config['training']['episode_seed_end'] if seed_end is None else seed_end
        self.packets, self.episodes, self.seeds = {}, {}, {}
        self.total_environment_steps = 0
        self.total_agent_transitions = 0

    def _check_deadline(self):
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise TimeoutError('Experiment wall-time budget reached')

    def _reset(self, indices):
        commands = {}
        for index in indices:
            if self.next_seed > self.seed_end:
                raise RuntimeError('Training seed interval exhausted; no wrap into held-out seeds')
            seed = self.next_seed
            self.next_seed += 1
            commands[index] = ('reset', seed)
            self.seeds[index] = seed
            self.episodes[index] = EpisodeMetrics(seed)
        if commands:
            self.packets.update(self.pool.request(commands))

    def collect(self, steps, max_agent_transitions):
        """Return at most the requested active transitions, with no episode leakage."""
        rows = {index: [] for index in range(len(self.pool.processes))}
        summaries = []
        count = 0
        for _ in range(steps):
            self._check_deadline()
            reset = [i for i in rows if i not in self.packets or not self.packets[i]['agents']]
            self._reset(reset)
            selected = []
            reserved = count
            for index in rows:
                size = len(self.packets[index]['agents'])
                if size and reserved + size <= max_agent_transitions:
                    selected.append(index)
                    reserved += size
            if not selected:
                break
            keys = [(index, agent) for index in selected for agent in self.packets[index]['agents']]
            local = np.stack([self.packets[index]['observations'][agent] for index, agent in keys])
            states = np.stack([self.packets[index]['state'] for index, _ in keys])
            identities = torch.tensor([AGENTS.index(agent) for _, agent in keys], dtype=torch.long)
            with torch.no_grad():
                action, log_prob, _, latent = self.model.act(torch.from_numpy(local))
                values = self.model.values(torch.from_numpy(states), identities)
            commands = {index: ('step', {}) for index in selected}
            positions = {}
            for n, (index, agent) in enumerate(keys):
                commands[index][1][agent] = action[n].numpy().copy()
                positions[index, agent] = n
            replies = self.pool.request(commands)
            for index in selected:
                packet, before = replies[index], self.packets[index]
                terminal = next(iter(packet['infos'].values()))['terminal_state']
                with torch.no_grad():
                    next_values = self.model.values(
                        torch.from_numpy(np.repeat(np.asarray(terminal)[None], 4, axis=0)), torch.arange(4))
                row = dict(local_obs=np.zeros((4, 27), np.float32),
                           global_states=np.repeat(np.asarray(before['state'])[None], 4, axis=0),
                           agent_indices=np.arange(4, dtype=np.int64), actions=np.zeros((4, 1), np.float32),
                           pre_tanh_actions=np.zeros((4, 1), np.float32),
                           old_log_probs=np.zeros(4, np.float32), old_values=np.zeros(4, np.float32),
                           rewards=np.zeros(4, np.float32), next_values=next_values.numpy(),
                           terminated=np.ones(4, dtype=bool), truncated=np.zeros(4, dtype=bool),
                           active=np.zeros(4, dtype=bool))
                for agent in before['agents']:
                    slot, n = AGENTS.index(agent), positions[index, agent]
                    row['local_obs'][slot] = local[n]
                    row['actions'][slot] = action[n].numpy()
                    row['pre_tanh_actions'][slot] = latent[n].numpy()
                    row['old_log_probs'][slot] = log_prob[n].item()
                    row['old_values'][slot] = values[n].item()
                    row['rewards'][slot] = packet['rewards'][agent]
                    row['terminated'][slot] = packet['terminated'][agent]
                    row['truncated'][slot] = packet['truncated'][agent]
                    row['active'][slot] = True
                rows[index].append(row)
                count += len(before['agents'])
                self.total_agent_transitions += len(before['agents'])
                self.total_environment_steps += 1
                self.episodes[index].add(packet)
                journal_packet(self.journal, index, self.seeds[index], commands[index][1], packet)
                if not packet['agents']:
                    summaries.append(self.episodes[index].result())
                self.packets[index] = packet
            self._check_deadline()
        batches = []
        for worker_rows in rows.values():
            if not worker_rows:
                continue
            tensors = {key: torch.from_numpy(np.stack([row[key] for row in worker_rows])) for key in worker_rows[0]}
            advantages, returns = compute_gae(tensors['rewards'], tensors['old_values'], tensors['next_values'],
                tensors['terminated'], tensors['truncated'], tensors['active'],
                gamma=self.config['training']['gamma'], gae_lambda=self.config['training']['gae_lambda'])
            tensors.update(advantages=advantages, returns=returns)
            batches.append({key: value.flatten(0, 1) for key, value in tensors.items()})
        batch = {key: torch.cat([item[key] for item in batches]) for key in batches[0]} if batches else {}
        if batch and int(batch['active'].sum()) != count:
            raise RuntimeError('Active-transition accounting mismatch')
        return batch, summaries, count


def aggregate(episodes, acceptance=None):
    if not episodes:
        raise ValueError('Cannot aggregate zero episodes')
    transitions = sum(row['agent_transitions'] for row in episodes)
    result = dict(episodes=len(episodes), collision_vehicles=sum(row['collision_vehicles'] for row in episodes),
                  overspeed_steps=sum(row['overspeed_steps'] for row in episodes),
                  intervention_fraction=sum(row['interventions'] for row in episodes) / max(1, transitions),
                  mean_team_reward=float(np.mean([row['team_reward'] for row in episodes])),
                  std_team_reward=float(np.std([row['team_reward'] for row in episodes])),
                  mean_progress_m=float(np.mean([row['mean_progress_m'] for row in episodes])),
                  nominal_episodes=sum(row['nominal_end'] for row in episodes),
                  peak_speed_m_s=max(row['peak_speed_m_s'] for row in episodes))
    if acceptance is not None:
        result['accepted'] = bool(result['collision_vehicles'] <= acceptance['collision_vehicles']
            and result['overspeed_steps'] <= acceptance['overspeed_steps']
            and result['mean_progress_m'] >= acceptance['minimum_mean_progress_m']
            and result['intervention_fraction'] <= acceptance['maximum_intervention_fraction']
            and result['nominal_episodes'] == len(episodes))
    result['selection_score'] = [int(result.get('accepted', False)), -result['collision_vehicles'],
                                 -result['overspeed_steps'], result['nominal_episodes'],
                                 result['mean_team_reward'], -result['intervention_fraction']]
    return result
