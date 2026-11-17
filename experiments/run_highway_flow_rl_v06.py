"""V06 merge training runner: the v05 MAPPO loop with v06 environment, gates and stop rules.

Reuses experiments/run_highway_flow_rl_v05.py unchanged (ledger, preflight,
complete-episode rollouts, checkpoints, periodic validation) and replaces only:
- the environment (HighwayFlowRLV06Env: rate limit, missed-gap rows, faded inputs);
- the config/preflight contract (traffic/highway_flow_rl_v06_contract.py);
- case gates (original development gates plus the raw-request jerk check);
- stop rules: any collision in any episode, or periodic-validation merge
  speed matching below the declared floor, stops the run for diagnosis; round 6
  also stops when periodic-validation pooled interventions exceed the declared
  maximum;
- round 6: several complete episodes per PPO update (BufferedMappoTrainer) and
  the raw-request jerk scope declared in the configuration;
- round 8 (final round): the merge-speed floor is reported, not enforced, at
  periodic validation (stop_rules.merge_floor_action='report');
- outputs under outputs/highway-flow-rl/v06 and a v06 checkpoint schema.
No training happens on import; `train` still requires training_authorized=true
and a measured zero-update preflight bound to this config and these sources.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import torch  # noqa: E402

from experiments import run_highway_flow_rl_v05 as base  # noqa: E402
from policies.mappo import MappoTrainer  # noqa: E402
from traffic import highway_flow_rl_v06_contract as contract  # noqa: E402
from traffic.highway_flow_rl_v06_rewards import v06_gates  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

V06_SOURCES = ('experiments/run_highway_flow_rl_v06.py', 'environments/highway_flow_rl_v06_env.py',
               'traffic/highway_flow_rl_v06_rewards.py', 'traffic/highway_flow_rl_v06_contract.py',
               'traffic/highway_v05_demand.py')


class StopRuleTriggered(RuntimeError):
    """A declared v06 stop rule fired; the run stops for diagnosis (checkpoint kept)."""


def merge_speed_match(reports):
    """Mean rolling merge speed-match over every merge target; no stable merge counts as 0."""
    values = []
    for report in reports:
        quality = report.get('quality_merges') or {}
        for agent in report.get('merge_target_ids') or []:
            values.append((quality.get(agent) or {}).get('speed_match_fraction', 0.) or 0.)
    return sum(values) / len(values) if values else None


class BufferedMappoTrainer(MappoTrainer):
    """MAPPO trainer that runs one PPO update per `episodes_per_update` complete episodes.

    Each episode keeps its own GAE (computed at collection). Buffered episodes
    were all collected by the same, not yet updated policy, so the merged batch
    is on-policy; advantages are normalized once over the merged batch. With
    episodes_per_update=1 this is exactly MappoTrainer.update (round 5).
    The buffer is in memory only: an interrupted run discards buffered episodes
    (their decisions stay charged), and episodes left over when the budget ends
    are never learned from; both show as trailing `deferred` update records.
    """

    def __init__(self, model, *, episodes_per_update=1, **kwargs):
        if type(episodes_per_update) is not int or episodes_per_update < 1:
            raise ValueError('episodes_per_update must be a positive integer')
        super().__init__(model, **kwargs)
        self.episodes_per_update = episodes_per_update
        self.buffer = []

    def update(self, batch, *, deadline=None, phase='ppo'):
        if self.episodes_per_update == 1:
            return super().update(batch, deadline=deadline, phase=phase)
        if not self.buffer or self.buffer[-1] is not batch:  # a retried update must not duplicate its batch
            self.buffer.append(batch)
        if len(self.buffer) < self.episodes_per_update:
            return dict(optimizer_steps=0, deferred=True, buffered_episodes=len(self.buffer),
                        episodes_per_update=self.episodes_per_update)
        merged = {key: torch.cat([b[key] for b in self.buffer]) for key in self.buffer[0]}
        stats = super().update(merged, deadline=deadline, phase=phase)
        stats.update(deferred=False, merged_episodes=len(self.buffer), episodes_per_update=self.episodes_per_update)
        self.buffer = []
        return stats


def env_for(cfg, row, directory):
    from environments.highway_flow_rl_v06_env import HighwayFlowRLV06Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayFlowRLV06Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], rate_limit=cfg['rate_limit'],
        v06_weights=cfg['v06_reward_weights'], missed_gap=row.get('missed_gap', False),
        controlled_car_follow_model=cfg['controlled_car_follow_model'],
        raw_request_scope=cfg.get('raw_request_scope', 'all'),
        road_end_leader_exit_rule=cfg['road_end_leader_exit_rule'])
    env.case_id = case['id']
    return env


_episode, _evaluate = base.episode, base.evaluate


def episode(cfg, row, directory, model, **kwargs):
    report, batch = _episode(cfg, row, directory, model, **kwargs)
    if cfg['stop_rules']['any_collision'] and report.get('collisions', 0):
        raise StopRuleTriggered(f"Collision in {row['case']['id']} seed {row['seed']}: {directory}")
    return report, batch


def evaluate(cfg, output, label, model, deadline, **kwargs):
    result = _evaluate(cfg, output, label, model, deadline, **kwargs)
    score = merge_speed_match(result['reports'])
    result['summary']['merge_speed_match'] = score
    below = score is None or score < cfg['stop_rules']['merge_speed_match_floor']
    result['summary']['merge_speed_match_below_floor'] = below
    if label.startswith('periodic-validation') and below and cfg['stop_rules'].get('merge_floor_action', 'stop') == 'stop':
        raise StopRuleTriggered(f'{label}: merge speed match {score} below the frozen-v04 baseline '
                                f"{cfg['stop_rules']['merge_speed_match_floor']}")
    limit = cfg['stop_rules'].get('max_pooled_intervention_fraction')
    pooled = result['summary'].get('pooled_intervention_fraction')
    if label.startswith('periodic-validation') and limit is not None and (pooled is None or pooled > limit):
        raise StopRuleTriggered(f'{label}: pooled intervention fraction {pooled} above {limit}')
    return result


def install():
    """Point the shared v05 loop at the v06 pieces (module globals are looked up at call time)."""
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v06'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v06'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *V06_SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.episode, base.evaluate = env_for, episode, evaluate
    base.ruled_gates = v06_gates
    base.MappoTrainer = BufferedMappoTrainer


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
