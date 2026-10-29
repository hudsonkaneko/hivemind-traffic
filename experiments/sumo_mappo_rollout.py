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
        self.wave = None
        self.driving = None

    def add(self, packet):
        self.steps += 1
        rewards = packet['rewards']
        self.transitions += len(rewards)
        self.reward += sum(rewards.values()) / len(AGENTS)
        # The environment repeats team telemetry in each agent's info. Count once.
        shared = next(iter(packet['infos'].values()), {}).get('wave_metrics')
        if shared is not None:
            if self.wave is None:
                self.wave = {key: 0. for key in (
                    'downstream_speed_sum', 'downstream_speed_squared_sum',
                    'downstream_speed_count', 'throughput_count', 'elapsed_s', 'traffic_progress_m')}
            for key in self.wave:
                value = float(shared.get(key, 0.))
                if not np.isfinite(value):
                    raise ValueError('Nonfinite wave metric: ' + key)
                self.wave[key] += value
        for agent, info in packet['infos'].items():
            driving = info.get('driving_metrics')
            if driving is not None:
                if self.driving is None:
                    self.driving = dict(lane_change_requested=0,lane_change_applied=0,lane_change_rejected=0,
                        pass_completed=0,cooperative_steps=0,fallback_following_steps=0,
                        applied_cooperative_steps=0,applied_fallback_steps=0,cooperative_eligible_steps=0,
                        headway_count=0,headway_sum_s=0.,minimum_headway_s=None,
                        cooperative_headway_count=0,cooperative_headway_sum_s=0.,
                        required_gap_deficit_steps=0,abs_jerk_sum_m_s3=0.,max_abs_jerk_m_s3=0.,
                        measured_agent_steps=0,shield_reason_counts={},lane_changes_by_agent={},steps_by_agent={})
                d = self.driving
                for key in ('lane_change_requested','lane_change_applied','lane_change_rejected','pass_completed'):
                    d[key] += int(driving[key])
                cooperative = bool(driving['cooperative_headway_active'])
                applied_cooperative = bool(driving['cooperative_headway_applied'])
                native_headway = driving['applied_native_headway_s']
                if not np.isfinite(native_headway) or native_headway <= 0:
                    raise ValueError('Invalid applied native headway')
                # Native commands belong to the pre-step control interval; fresh
                # eligibility belongs to the resulting observation. Keep both.
                d['cooperative_steps'] += int(applied_cooperative)
                d['applied_cooperative_steps'] += int(applied_cooperative)
                d['applied_fallback_steps'] += int(not applied_cooperative)
                d['cooperative_eligible_steps'] += int(cooperative)
                headway = driving.get('headway_s')
                if headway is not None:
                    if not np.isfinite(headway):raise ValueError('Nonfinite driving headway')
                    d['headway_count'] += 1; d['headway_sum_s'] += headway
                    d['minimum_headway_s'] = headway if d['minimum_headway_s'] is None else min(headway,d['minimum_headway_s'])
                    if cooperative:
                        d['cooperative_headway_count'] += 1;d['cooperative_headway_sum_s'] += headway
                    if not applied_cooperative:d['fallback_following_steps'] += 1
                gap=info.get('front_gap_m')
                d['required_gap_deficit_steps'] += int(gap is not None and gap < driving['required_gap_m']-1e-6)
                jerk=float(driving['abs_jerk_m_s3'])
                if not np.isfinite(jerk):raise ValueError('Nonfinite driving jerk')
                d['abs_jerk_sum_m_s3'] += jerk;d['max_abs_jerk_m_s3']=max(d['max_abs_jerk_m_s3'],jerk)
                d['measured_agent_steps'] += 1
                d['steps_by_agent'][agent]=d['steps_by_agent'].get(agent,0)+1
                d['lane_changes_by_agent'][agent]=d['lane_changes_by_agent'].get(agent,0)+int(driving['lane_change_applied'])
                for reason in info.get('safety_reasons',()):
                    d['shield_reason_counts'][reason]=d['shield_reason_counts'].get(reason,0)+1
                if info.get('sumo_safety_intervention'):
                    d['shield_reason_counts']['sumo_override']=d['shield_reason_counts'].get('sumo_override',0)+1
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
        result = dict(seed=self.seed, steps=self.steps, agent_transitions=self.transitions,
                    team_reward=self.reward, mean_progress_m=float(np.mean(list(self.progress.values()))),
                    progress_by_agent_m=self.progress, collision_vehicles=len(self.collisions),
                    collision_ids=sorted(self.collisions), interventions=self.interventions,
                    intervention_fraction=self.interventions / max(1, self.transitions),
                    overspeed_steps=self.overspeed, peak_speed_m_s=self.peak_speed,
                    min_ttc_s=self.minimum_ttc, background_peak=self.background_peak,
                    end_causes=self.causes,
                    nominal_end=len(self.causes) == 4 and all(c in ('time_limit', 'route_complete') for c in self.causes.values()))
        if self.wave is not None:
            count = self.wave['downstream_speed_count']
            mean = self.wave['downstream_speed_sum'] / count if count else 0.
            result['wave'] = dict(self.wave,
                downstream_speed_variance_m2_s2=max(0., self.wave['downstream_speed_squared_sum'] / count - mean**2) if count else None,
                throughput_vehicles_per_hour=3600. * self.wave['throughput_count'] / self.wave['elapsed_s'] if self.wave['elapsed_s'] else None)
        if self.driving is not None:
            d=self.driving
            result['driving']=dict(d,
                mean_headway_s=d['headway_sum_s']/d['headway_count'] if d['headway_count'] else None,
                mean_cooperative_headway_s=d['cooperative_headway_sum_s']/d['cooperative_headway_count'] if d['cooperative_headway_count'] else None,
                cooperative_headway_measurement_scope='post-step eligible leader; applied command counts are separate',
                mean_abs_jerk_m_s3=d['abs_jerk_sum_m_s3']/max(1,d['measured_agent_steps']),
                maximum_lane_changes_per_vehicle_minute=max(
                    d['lane_changes_by_agent'][a]*600/d['steps_by_agent'][a] for a in d['steps_by_agent']))
        return result


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
                row = dict(local_obs=np.zeros((4, local.shape[-1]), np.float32),
                           global_states=np.repeat(np.asarray(before['state'])[None], 4, axis=0),
                           agent_indices=np.arange(4, dtype=np.int64), actions=np.zeros((4, action.shape[-1]), np.float32),
                           pre_tanh_actions=np.zeros((4, latent.shape[-1]), np.float32),
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


def aggregate(episodes, acceptance=None, driving_targets=None):
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
    if all('wave' in row for row in episodes):
        waves = [row['wave'] for row in episodes]
        valid = all(row['downstream_speed_count'] > 0 and row['elapsed_s'] > 0 for row in waves)
        result['wave'] = dict(
            valid=valid,
            mean_downstream_speed_variance_m2_s2=float(np.mean([row['downstream_speed_variance_m2_s2'] for row in waves])) if valid else None,
            mean_throughput_vehicles_per_hour=float(np.mean([row['throughput_vehicles_per_hour'] for row in waves])) if valid else None,
            mean_traffic_progress_m=float(np.mean([row['traffic_progress_m'] for row in waves])))
    if all('driving' in row for row in episodes):
        driving=[row['driving'] for row in episodes]
        counts={key:sum(row[key] for row in driving) for key in (
            'lane_change_requested','lane_change_applied','lane_change_rejected','pass_completed',
            'cooperative_steps','fallback_following_steps','applied_cooperative_steps','applied_fallback_steps',
            'cooperative_eligible_steps','headway_count','headway_sum_s',
            'cooperative_headway_count','cooperative_headway_sum_s','required_gap_deficit_steps',
            'abs_jerk_sum_m_s3','measured_agent_steps')}
        reason_counts={}
        for row in driving:
            for reason,count in row['shield_reason_counts'].items():
                reason_counts[reason]=reason_counts.get(reason,0)+count
        minimum=[r['minimum_headway_s'] for r in driving if r['minimum_headway_s'] is not None]
        result['driving']=dict(counts,shield_reason_counts=reason_counts,
            minimum_headway_s=min(minimum) if minimum else None,
            mean_headway_s=counts['headway_sum_s']/counts['headway_count'] if counts['headway_count'] else None,
            mean_cooperative_headway_s=counts['cooperative_headway_sum_s']/counts['cooperative_headway_count'] if counts['cooperative_headway_count'] else None,
            cooperative_headway_measurement_scope='post-step eligible leader; applied command counts are separate',
            mean_abs_jerk_m_s3=counts['abs_jerk_sum_m_s3']/max(1,counts['measured_agent_steps']),
            max_abs_jerk_m_s3=max(r['max_abs_jerk_m_s3'] for r in driving),
            minimum_passes_per_episode=min(r['pass_completed'] for r in driving),
            maximum_lane_changes_per_vehicle_minute=max(r['maximum_lane_changes_per_vehicle_minute'] for r in driving))
    if driving_targets is not None:
        d=result.get('driving',{})
        values=[d.get('minimum_passes_per_episode'),d.get('maximum_lane_changes_per_vehicle_minute')]
        per_episode_values=[row.get('driving',{}).get(key) for row in episodes
                            for key in ('pass_completed','maximum_lane_changes_per_vehicle_minute')]
        result['driving_targets_met']=bool(all(v is not None and np.isfinite(v) for v in values+per_episode_values)
            and values[0]>=driving_targets['minimum_passes_per_episode']
            and values[1]<=driving_targets['maximum_lane_changes_per_vehicle_minute'])
        result['basic_driving_accepted']=result.get('accepted',False)
        result['accepted']=bool(result['basic_driving_accepted'] and result['driving_targets_met'])
    result['selection_score'] = [int(result.get('accepted', False)), -result['collision_vehicles'],
                                 -result['overspeed_steps'], result['nominal_episodes'],
                                 result['mean_team_reward'], -result['intervention_fraction']]
    return result


def compare_wave_episodes(candidate, baseline, *, minimum_variance_reduction=.15,
                          maximum_throughput_loss=.02):
    """Matched-seed comparison; caller must identify a separately trained baseline.

    A zero baseline variance or throughput cannot establish a relative gain.
    This helper does not assert policy provenance or replace the safety gate.
    """
    def indexed(rows):
        result = {row['seed']: row for row in rows}
        if not result or len(result) != len(rows):
            raise ValueError('Wave comparison requires nonempty unique episode seeds')
        return result
    left, right = indexed(candidate), indexed(baseline)
    if set(left) != set(right):
        raise ValueError('Wave comparison requires identical episode seeds')
    pairs = []
    for seed in sorted(left):
        a, b = left[seed]['wave'], right[seed]['wave']
        if a['elapsed_s'] != b['elapsed_s'] or a['elapsed_s'] <= 0:
            raise ValueError('Wave comparison requires equal positive episode duration')
        av, bv = a['downstream_speed_variance_m2_s2'], b['downstream_speed_variance_m2_s2']
        at, bt = a['throughput_vehicles_per_hour'], b['throughput_vehicles_per_hour']
        if any(value is None or not np.isfinite(value) for value in (av, bv, at, bt)):
            raise ValueError('Wave comparison requires finite populated metrics')
        pairs.append(dict(seed=seed, candidate_variance=av, baseline_variance=bv,
                          candidate_throughput=at, baseline_throughput=bt))
    means = {key: float(np.mean([row[key] for row in pairs])) for key in pairs[0] if key != 'seed'}
    valid = means['baseline_variance'] > 0 and means['baseline_throughput'] > 0
    reduction = 1. - means['candidate_variance'] / means['baseline_variance'] if valid else None
    loss = 1. - means['candidate_throughput'] / means['baseline_throughput'] if valid else None
    return dict(pairs=pairs, means=means, valid=valid, variance_reduction_fraction=reduction,
                throughput_loss_fraction=loss, wave_targets_met=bool(valid and
                reduction >= minimum_variance_reduction - 1e-12 and
                loss <= maximum_throughput_loss + 1e-12))
