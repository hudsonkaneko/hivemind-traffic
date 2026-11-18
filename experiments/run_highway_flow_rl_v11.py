"""V11 merge runner: the v10 runner with the v11 environment and contract.

Reuses experiments/run_highway_flow_rl_v10.py (and through it the v09, v06 and
v05 loops: ledger, preflight, complete-episode rollouts, buffered PPO updates,
checkpoints, periodic validation, stop rules, the pinned round-9 warm-start
loader and the canonical ledger through Git's common directory) and replaces only:
- the environment (HighwayFlowRLV11Env: v10 terms plus the merge-speed precondition mask);
- the config contract (traffic/highway_flow_rl_v11_contract.py);
- outputs under this worktree's outputs/highway-flow-rl/v11 and a v11 checkpoint schema;
- the trainer, when a round declares `v11_training`: the buffered MAPPO trainer plus
  a shield-consistency loss (see ShieldConsistentTrainer).
No training happens on import.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import torch  # noqa: E402

from experiments import run_highway_flow_rl_v10 as previous  # noqa: E402
from experiments.run_highway_flow_rl_v06 import BufferedMappoTrainer  # noqa: E402
from traffic import highway_flow_rl_v11_contract as contract  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

base = previous.base
SOURCES = ('experiments/run_highway_flow_rl_v11.py', 'environments/highway_flow_rl_v11_env.py',
           'traffic/highway_flow_rl_v11_rewards.py', 'traffic/highway_flow_rl_v11_contract.py',
           'scripts/prepare_highway_flow_rl_v11.py', 'experiments/run_highway_flow_rl_v11_projection.py')
shared_study_directory = previous.shared_study_directory


def env_for(cfg, row, directory):
    from environments.highway_flow_rl_v11_env import HighwayFlowRLV11Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayFlowRLV11Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], rate_limit=cfg['rate_limit'],
        v06_weights=cfg['v06_reward_weights'], v10_settings=cfg['v10_reward_settings'],
        v11_settings=cfg['v11_settings'],
        missed_gap=row.get('missed_gap', False), controlled_car_follow_model=cfg['controlled_car_follow_model'],
        raw_request_scope=cfg.get('raw_request_scope', 'all'),
        road_end_leader_exit_rule=cfg['road_end_leader_exit_rule'])
    env.case_id = case['id']
    return env


ACCELERATION_INDEX, CEILING_INDEX, OBSERVATION_SCALE_M_S2 = 1, 43, 4.5  # actor117: both /4.5, neither is faded


def shield_consistency(request, measured, ceiling, *, ceiling_margin, rate_bound):
    """Squared distance of each raw request from what the command filters would pass (m/s2).

    The environment caps every request at the feasible ceiling (speed limit,
    following, lane end) and rate-limits commands to 0.4 m/s2 per decision around
    the measured acceleration; the gates count each such change as an override, and
    the raw-request check allows 0.5 m/s2 per decision. The allowed band here is
    [measured - rate_bound, min(measured + rate_bound, ceiling - ceiling_margin)].
    When the ceiling itself is below the band, the band collapses onto it, so a
    request may always drop as fast as the ceiling demands.
    """
    upper = torch.minimum(measured + rate_bound, ceiling - ceiling_margin)
    lower = torch.minimum(measured - rate_bound, upper)
    return torch.relu(request - upper).square() + torch.relu(lower - request).square()


class ShieldConsistentTrainer(BufferedMappoTrainer):
    """Buffered MAPPO plus a loss that keeps the actor's mean request inside the filters' own band.

    Not a teacher: the band comes from two inputs the actor already receives (its
    measured acceleration and the feasible acceleration ceiling) and constrains the
    request only where a filter would change it anyway. Inside the band the loss
    and its gradient are zero, so PPO alone decides the behavior there.
    """

    def __init__(self, model, *, shield_coef=0., ceiling_margin_m_s2=.05, rate_bound_m_s2=.35, **kwargs):
        if not all(isinstance(v, (int, float)) and v == v and v >= 0 for v in (shield_coef, ceiling_margin_m_s2, rate_bound_m_s2)):
            raise ValueError('Invalid shield-consistency settings')
        super().__init__(model, **kwargs)
        self.shield_coef, self.ceiling_margin, self.rate_bound = shield_coef, ceiling_margin_m_s2, rate_bound_m_s2

    def _loss_terms(self, part, ratio, entropy):
        loss, weighted, stats = super()._loss_terms(part, ratio, entropy)
        if not self.shield_coef:
            return loss, weighted, stats
        observations = part['local_obs']
        mean = self.model._distribution(observations).loc[..., 0]
        request = -4.5 + (mean.tanh() + 1.) * 3.25           # environments.sumo_mappo_env.action_to_acceleration
        distance = shield_consistency(request, observations[..., ACCELERATION_INDEX] * OBSERVATION_SCALE_M_S2,
            observations[..., CEILING_INDEX] * OBSERVATION_SCALE_M_S2,
            ceiling_margin=self.ceiling_margin, rate_bound=self.rate_bound)
        weighted['shield'] = self.shield_coef * distance.mean()
        loss = loss + weighted['shield']
        if not torch.isfinite(loss):
            raise ValueError('Nonfinite shield-consistency loss')
        stats.update(shield_loss_m2_s4=float(distance.mean().detach()),
                     shield_outside_fraction=float((distance > 0).float().mean().detach()))
        return loss, weighted, stats


def trainer_for(cfg):
    """Trainer class bound to the round's declared v11 training settings (plain buffered MAPPO without them)."""
    extra = dict(cfg.get('v11_training') or {})

    def build(model, **optimizer):
        return ShieldConsistentTrainer(model, **optimizer, **extra)
    return build


_build_model = previous.previous.build_model


def build_model(cfg, **kwargs):
    base.MappoTrainer = trainer_for(cfg)   # the v05/v09 builders look this name up at call time
    return _build_model(cfg, **kwargs)


def install():
    """Point the shared loop at the v11 pieces (module globals are looked up at call time)."""
    previous.install()
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v11'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v11'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.build_model = env_for, build_model


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
