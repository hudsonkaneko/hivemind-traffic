"""V06 merge training runner: the v05 MAPPO loop with v06 environment, gates and stop rules.

Reuses experiments/run_highway_flow_rl_v05.py unchanged (ledger, preflight,
complete-episode rollouts, checkpoints, periodic validation) and replaces only:
- the environment (HighwayFlowRLV06Env: rate limit, missed-gap rows, faded inputs);
- the config/preflight contract (traffic/highway_flow_rl_v06_contract.py);
- case gates (original development gates plus the raw-request jerk check);
- stop rules: any collision in any episode, or periodic-validation merge
  speed matching below the frozen-v04 baseline, stops the run for diagnosis;
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

from experiments import run_highway_flow_rl_v05 as base  # noqa: E402
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
    if label.startswith('periodic-validation') and (score is None or score < cfg['stop_rules']['merge_speed_match_floor']):
        raise StopRuleTriggered(f'{label}: merge speed match {score} below the frozen-v04 baseline '
                                f"{cfg['stop_rules']['merge_speed_match_floor']}")
    return result


def install():
    """Point the shared v05 loop at the v06 pieces (module globals are looked up at call time)."""
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v06'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v06'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *V06_SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.episode, base.evaluate = env_for, episode, evaluate
    base.ruled_gates = v06_gates


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
