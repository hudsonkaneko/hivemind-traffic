"""V09 merge training runner: the v06 round-9 loop with the v09 reward (Claude proposal).

Reuses experiments/run_highway_flow_rl_v06.py (and through it the v05 loop:
ledger, preflight, complete-episode rollouts, buffered PPO updates, checkpoints,
periodic validation, collision / merge-floor / pooled-intervention stop rules)
and replaces only:
- the environment (HighwayFlowRLV09Env: v06 dynamics, v09 reward terms);
- the config contract (traffic/highway_flow_rl_v09_contract.py);
- the warm start loader, which accepts the pinned complete v06 round-9 checkpoint;
- outputs under this worktree's outputs/highway-flow-rl/v09 and a v09 checkpoint schema;
- the study ledger and lock, resolved to the canonical checkout through Git's
  common directory (shared by every worktree, as in v07/v08).
No training happens on import; `train` still requires training_authorized=true
and a measured zero-update preflight bound to this config and these sources.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import torch  # noqa: E402

from experiments import run_highway_flow_rl_v06 as previous  # noqa: E402
from traffic import highway_flow_rl_v09_contract as contract  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

base = previous.base
SOURCES = ('experiments/run_highway_flow_rl_v09.py', 'environments/highway_flow_rl_v09_env.py',
           'traffic/highway_flow_rl_v09_rewards.py', 'traffic/highway_flow_rl_v09_contract.py',
           'scripts/prepare_highway_flow_rl_v09.py', 'scripts/stage_highway_flow_rl_v09_inputs.py')
_build_model = base.build_model


def env_for(cfg, row, directory):
    from environments.highway_flow_rl_v09_env import HighwayFlowRLV09Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayFlowRLV09Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], rate_limit=cfg['rate_limit'],
        v06_weights=cfg['v06_reward_weights'], v09_settings=cfg['v09_reward_settings'],
        missed_gap=row.get('missed_gap', False), controlled_car_follow_model=cfg['controlled_car_follow_model'],
        raw_request_scope=cfg.get('raw_request_scope', 'all'),
        road_end_leader_exit_rule=cfg['road_end_leader_exit_rule'])
    env.case_id = case['id']
    return env


def build_model(cfg, *, checkpoint=None, resume=False, fingerprint=None, sources=None, evaluation=False):
    """Load the pinned complete round-9 checkpoint (v06 schema) as warm start; everything else as v05."""
    path = base.project_path(checkpoint or cfg['warm_start']['path'])
    if resume or path != base.project_path(cfg['warm_start']['path']):
        return _build_model(cfg, checkpoint=checkpoint, resume=resume, fingerprint=fingerprint,
                            sources=sources, evaluation=evaluation)
    if file_digest(path) != cfg['warm_start']['sha256']:
        raise ValueError('Pinned warm-start checksum changed')
    saved = torch.load(path, map_location='cpu', weights_only=False)
    if saved.get('schema') != contract.WARM_START_SCHEMA or any(k not in saved for k in ('model', 'optimizer', 'rng', 'state')):
        raise ValueError('Warm start must be the complete v06 round-9 checkpoint')
    if saved.get('config_sha256') != cfg['parent_config_sha256']:
        raise ValueError('Warm start was not produced by the declared round-9 configuration')
    model = base.FlowModel()
    model.load_state_dict(saved['model'], strict=True)
    return model, base.MappoTrainer(model, **cfg['optimizer']), saved


def shared_study_directory():
    common = Path(subprocess.check_output(['git', 'rev-parse', '--git-common-dir'], cwd=ROOT, text=True).strip())
    if not common.is_absolute():
        common = ROOT / common
    return common.resolve().parent / 'outputs/highway-flow-rl/v02'


def install():
    """Point the shared loop at the v09 pieces (module globals are looked up at call time)."""
    previous.install()
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v09'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v09'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.build_model = env_for, build_model
    base.STUDY = shared_study_directory()


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
