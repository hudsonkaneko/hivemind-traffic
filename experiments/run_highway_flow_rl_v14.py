"""V14 runner pieces: the v12 loop with the v14 environment and policy-only SUMO modes.

install() points the shared v12 loop at the v14 environment, requires SUMO speed mode 96 and
lane change mode 0 on every controlled car at every decision, and writes under
outputs/highway-flow-rl/v14. No v14 training contract or configuration is declared yet, so
main() refuses to train; development checks and recordings use install() and env_for().
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import run_highway_flow_rl_v12 as v12runner  # noqa: E402
from traffic import highway_flow_rl_v14_rewards as v14  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

base = v12runner.base
_v12_install = v12runner.install   # kept, so callers may point v12runner.install at install() below
OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v14'
NATIVE_MODES = dict(speed=v14.CONTROL['speed_mode'], lane_change=v14.CONTROL['lane_change_mode'])
SOURCES = ('experiments/run_highway_flow_rl_v14.py', 'environments/highway_flow_rl_v14_env.py',
           'traffic/highway_flow_rl_v14_rewards.py')


def env_for(cfg, row, directory, env_class=None, **extra):
    """The v14 environment of a row; env_class/extra select a subclass such as the physical one."""
    from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = (env_class or HighwayFlowRLV14Env)(network, directory, **extra,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], rate_limit=cfg['rate_limit'],
        v06_weights=cfg['v06_reward_weights'], v10_settings=cfg['v10_reward_settings'],
        v11_settings=cfg['v11_settings'], v12_settings=cfg['v12_settings'],
        missed_gap=row.get('missed_gap', False), controlled_car_follow_model=cfg['controlled_car_follow_model'],
        raw_request_scope=cfg.get('raw_request_scope', 'all'),
        road_end_leader_exit_rule=cfg['road_end_leader_exit_rule'])
    env.case_id = case['id']
    return env


def install():
    """The v12 loop with the v14 environment, modes and output root (globals read at call time)."""
    _v12_install()
    v12runner.NATIVE_MODES = dict(NATIVE_MODES)
    base.OUTPUT_ROOT = OUTPUT_ROOT
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.env_for = env_for


def main(argv=None):
    raise SystemExit('No v14 training contract or configuration is declared yet.')


if __name__ == '__main__':
    raise SystemExit(main())
