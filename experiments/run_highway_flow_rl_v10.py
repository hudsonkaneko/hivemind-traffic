"""V10 merge training runner: the v09 runner with the v10 reward (Claude proposal).

Reuses experiments/run_highway_flow_rl_v09.py (and through it the v06 and v05 loops:
ledger, preflight, complete-episode rollouts, buffered PPO updates, checkpoints,
periodic validation, collision / merge-floor / pooled-intervention stop rules, the
pinned round-9 warm-start loader and the canonical ledger through Git's common
directory) and replaces only:
- the environment (HighwayFlowRLV10Env: v09 terms plus the post-merge override cost);
- the config contract (traffic/highway_flow_rl_v10_contract.py);
- outputs under this worktree's outputs/highway-flow-rl/v10 and a v10 checkpoint schema.
No training happens on import; `train` still requires training_authorized=true
and a measured zero-update preflight bound to this config and these sources.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import run_highway_flow_rl_v09 as previous  # noqa: E402
from traffic import highway_flow_rl_v10_contract as contract  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

base = previous.base
SOURCES = ('experiments/run_highway_flow_rl_v10.py', 'environments/highway_flow_rl_v10_env.py',
           'traffic/highway_flow_rl_v10_rewards.py', 'traffic/highway_flow_rl_v10_contract.py',
           'scripts/prepare_highway_flow_rl_v10.py', 'scripts/stage_highway_flow_rl_v10_inputs.py')
shared_study_directory = previous.shared_study_directory


def env_for(cfg, row, directory):
    from environments.highway_flow_rl_v10_env import HighwayFlowRLV10Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayFlowRLV10Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], rate_limit=cfg['rate_limit'],
        v06_weights=cfg['v06_reward_weights'], v10_settings=cfg['v10_reward_settings'],
        missed_gap=row.get('missed_gap', False), controlled_car_follow_model=cfg['controlled_car_follow_model'],
        raw_request_scope=cfg.get('raw_request_scope', 'all'),
        road_end_leader_exit_rule=cfg['road_end_leader_exit_rule'])
    env.case_id = case['id']
    return env


def install():
    """Point the shared loop at the v10 pieces (module globals are looked up at call time)."""
    previous.install()
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v10'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v10'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for = env_for


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
