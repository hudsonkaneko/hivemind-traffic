"""Source-bound merge recovery training, with mandatory cut-in coverage."""
from pathlib import Path
import math
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import torch
from experiments import run_highway_merge_cutin_v07 as v07
from traffic import highway_merge_recovery_v08_contract as contract
from traffic.highway_flow_contract import file_digest
base = v07.base
SOURCES = ('experiments/run_highway_merge_recovery_v08.py', 'environments/highway_merge_recovery_v08_env.py',
    'traffic/highway_merge_recovery_v08.py', 'traffic/highway_merge_recovery_v08_contract.py',
    'scripts/prepare_highway_merge_recovery_v08.py', 'scripts/run_highway_merge_recovery_v08_job.py')
_build_model = base.build_model
_episode = v07.previous.episode


def build_model(cfg, **kwargs):
    model, trainer, saved = _build_model(cfg, **kwargs)
    if not kwargs.get('resume') and kwargs.get('checkpoint') in (None, cfg['warm_start']['path']):
        with torch.no_grad():
            model.log_std.fill_(math.log(cfg['initial_policy_std']))
    return model, trainer, saved


def env_for(cfg, row, directory):
    from environments.highway_merge_recovery_v08_env import HighwayMergeRecoveryV08Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayMergeRecoveryV08Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=0., rate_limit=cfg['rate_limit'], v06_weights=cfg['v06_reward_weights'],
        missed_gap=row.get('missed_gap', False), cutin_stress=row.get('cutin_stress', False),
        controlled_car_follow_model='Krauss', raw_request_scope='all', road_end_leader_exit_rule=False)
    env.case_id = case['id']
    return env


def episode(cfg, row, directory, model, **kwargs):
    report, batch = _episode(cfg, row, directory, model, **kwargs)
    if row.get('cutin_stress'):
        proof = report.get('cutin_fixture', {})
        if not proof.get('moving_at_handoff') or not proof.get('completed_into_target'):
            raise RuntimeError(f'Cut-in exposure missing in {directory}; no empty stress episode accepted')
    return report, batch


def evaluate(cfg, output, label, model, deadline, **kwargs):
    result = v07.evaluate(cfg, output, label, model, deadline, **kwargs)
    pooled = result['summary'].get('pooled_intervention_fraction')
    if label.startswith('periodic-validation') and (pooled is None or pooled > .05):
        raise v07.previous.StopRuleTriggered(f'{label}: interventions {pooled} exceed 5%')
    return result


def install():
    v07.install()
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v08'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v08'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES, 'policies/mappo.py')))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.episode, base.evaluate, base.build_model = env_for, episode, evaluate, build_model


if __name__ == '__main__':
    install()
    raise SystemExit(base.main())
