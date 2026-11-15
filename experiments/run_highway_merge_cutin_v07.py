"""Approved bounded MAPPO merge/cut-in round; no launch on import."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments import run_highway_flow_rl_v06 as previous
from traffic import highway_merge_cutin_v07_contract as contract
from traffic.highway_merge_cutin_v07 import training_row
from traffic.highway_flow_contract import file_digest

base = previous.base
SOURCES = ('experiments/run_highway_merge_cutin_v07.py', 'environments/highway_merge_cutin_v07_env.py',
    'traffic/highway_merge_cutin_v07.py', 'traffic/highway_merge_cutin_v07_contract.py',
    'scripts/prepare_highway_merge_cutin_v07.py', 'scripts/run_highway_merge_cutin_v07_job.py')


def env_for(cfg, row, directory):
    from environments.highway_merge_cutin_v07_env import HighwayMergeCutinV07Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayMergeCutinV07Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=0., rate_limit=cfg['rate_limit'], v06_weights=cfg['v06_reward_weights'],
        missed_gap=row.get('missed_gap', False), cutin_stress=row.get('cutin_stress', False),
        controlled_car_follow_model='Krauss', road_end_leader_exit_rule=False)
    env.case_id = case['id']
    return env


def evaluate(cfg, output, label, model, deadline, **kwargs):
    result = previous._evaluate(cfg, output, label, model, deadline, **kwargs)
    nominal = [r for r in result['reports'] if not r.get('cutin_stress')]
    score = previous.merge_speed_match(nominal)
    result['summary'].update(merge_speed_match=score,
        nominal=base.summarize(nominal),
        stress=base.summarize([r for r in result['reports'] if r.get('cutin_stress')]))
    base.save(output / f'{label}.json', result)
    if label.startswith('periodic-validation'):
        import json
        initial = json.loads((output / 'initial-validation.json').read_text())
        floor = initial['summary']['merge_speed_match']
        if score is None or floor is None or score < floor:
            raise previous.StopRuleTriggered(f'{label}: nominal merge match {score} below matched baseline {floor}')
    return result


def install():
    previous.install()
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v07'
    base.CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v07'
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.training_row, base.evaluate = env_for, training_row, evaluate
    # All worktrees share the canonical append-only study ledger and lock.
    # Runtime inputs and new outputs stay inside this branch's worktree.
    import subprocess
    common = Path(subprocess.check_output(['git', 'rev-parse', '--git-common-dir'], cwd=ROOT, text=True).strip())
    if not common.is_absolute():
        common = ROOT / common
    base.STUDY = common.resolve().parent / 'outputs/highway-flow-rl/v02'


if __name__ == '__main__':
    install()
    raise SystemExit(base.main())
