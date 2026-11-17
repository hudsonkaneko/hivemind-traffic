"""Write the resolved v10 round-12 proposal configuration (training_authorized stays false).

Derived, like v09, from the committed v06 round-9 configuration
(configs/training/highway-flow-rl-v06-round9.json). Rows, observation, actions,
safety, rate limit, optimizer, acceptance and raw-request scope are unchanged;
traffic/highway_flow_rl_v10_contract.py checks that, and lists every change.
Round index 12 shifts every training seed; the missed-gap seeds of the curriculum
cycles a 200k-decision round can reach (PPO cycles 0-1, critic cycle 0; 4 cycles
checked as for rounds 9 and 11) are rebuilt to prove they build.
No simulation and no ledger charge.
"""
import argparse
import hashlib
import json
import sys
import tempfile
from copy import deepcopy
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from traffic import highway_flow_rl_v10_contract as contract  # noqa: E402
from traffic import highway_flow_rl_v10_rewards as v10  # noqa: E402
from traffic.highway_merge_context import Stage1LaneGraph  # noqa: E402
from traffic.highway_v05_demand import write_demand  # noqa: E402

CYCLES_CHECKED = 4
ROUND9_RESULTS = HERE / 'documentation/evidence/highway-merge-training-v06/round-09-results.json'
ROUND11_RESULTS = HERE / 'documentation/evidence/highway-merge-training-v09/round-11-results.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True, help='canonical checkout holding outputs/')
    parser.add_argument('--round9', type=Path, default=HERE / 'configs/training/highway-flow-rl-v06-round9.json')
    parser.add_argument('--output', type=Path, default=HERE / 'configs/training/highway-flow-rl-v10-round12.json')
    args = parser.parse_args()
    raw = args.round9.read_bytes().replace(b'\r\n', b'\n')  # Windows checkouts may hold CRLF copies
    if hashlib.sha256(raw).hexdigest() != contract.PARENT_CONFIG_SHA256:
        raise ValueError('Round-9 configuration changed')
    cfg = json.loads(raw.decode('utf-8'))
    results = json.loads(ROUND9_RESULTS.read_text(encoding='utf-8'))
    if round(results['validations']['final-validation']['merge_speed_match'], 4) != contract.ROUND9_FINAL_MERGE_SPEED_MATCH:
        raise ValueError('Round-9 final merge speed match differs from the contract floor source')
    round11 = json.loads(ROUND11_RESULTS.read_text(encoding='utf-8'))
    if round11['outcome']['stopped_by'] != 'merge_floor' or round11['outcome']['decisions'] != 100457:
        raise ValueError('Round-11 evidence differs from the recorded result')
    warm = args.project_root / contract.WARM_START['path']
    if hashlib.sha256(warm.read_bytes()).hexdigest() != contract.WARM_START['sha256']:
        raise ValueError('Round-9 final checkpoint differs from the pinned warm start')

    seed_check = []
    for index, row in enumerate(cfg['training_rows']):
        if not row.get('missed_gap'):
            continue
        graph = Stage1LaneGraph(args.project_root / row['case']['network'], row['case']['through_lanes'])
        for cycle, critic in [(c, 0) for c in range(CYCLES_CHECKED)] + [(0, 50000)]:
            seed = row['seed'] + 1000 * cycle + 10000 * (contract.ROUND_INDEX - 1) + critic
            with tempfile.TemporaryDirectory() as tmp:
                manifest = write_demand(Path(tmp) / 'r.xml', seed, graph, 'merge', 1, 8,
                    speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
                    horizon_seconds=row['horizon_steps'] * .1, controlled_car_follow_model='Krauss', missed_gap=True)
            seed_check.append(dict(row=index, seed=seed, blocker=manifest.scenario['missed_gap']['blocker_slot']))

    stop_rules = dict(any_collision=True, merge_speed_match_floor=contract.MERGE_FLOOR,
        merge_floor_source=(f'round-9 final validation (the warm start) in the same v06 environment '
                            f'({contract.ROUND9_FINAL_MERGE_SPEED_MATCH}) minus {contract.FLOOR_MARGIN}'),
        merge_floor_action='stop',
        merge_floor_action_source='a reward change that lowers merge matching should stop early and save budget',
        max_pooled_intervention_fraction=.05,
        intervention_source='per-case development gate value (5%)',
        checked_at=('every periodic validation (100k decisions); the final validation records a below-floor score '
                    'without stopping; collisions stop the run at any episode'))
    provenance = dict(parent_policy_sha256=contract.WARM_START['sha256'],
        parent_run='v06 round 9 final (outputs/highway-flow-rl/v06/round-09/last.pt, canonical checkout)',
        parent_config_sha256=contract.PARENT_CONFIG_SHA256,
        previous_round=('v09 round 11: stopped by the merge floor at 100,457 decisions (pooled match 0.211); '
                        'two of four validation merging cars failed the stable-merge gate on post-merge override '
                        'counts; training episodes 16-43 had 6 of 17 stable-merge failures (round 9: 2 of 17)'),
        evidence='documentation/highway-merge-training-plan-v10.md (round-11 diagnosis)',
        changes=['all v09 reward terms kept (unsaturated tracking, graded quality, lag-aware jerk excess)',
                 'post-merge override cost: +1.0 per decision the stable-merge tracker counts as an intervention '
                 'in its 8 s post-merge window (total 1.2 with the unchanged intervention term)',
                 'warm start: the complete round-9 final checkpoint again (not the round-11 checkpoint)',
                 'round index 12; 200,000 decisions in the 207,597 left under the existing 5,230,000 ceiling; '
                 'diagnostic-episode cap +1 for the preflight'])
    cfg.update(schema=contract.SCHEMA, round_index=contract.ROUND_INDEX, training_authorized=False,
        learning_profile=contract.LEARNING_PROFILE, reward_version=v10.REWARD_VERSION,
        v10_reward_settings=deepcopy(v10.SETTINGS), warm_start=deepcopy(contract.WARM_START),
        parent_config_sha256=contract.PARENT_CONFIG_SHA256, budget=deepcopy(contract.BUDGET),
        stop_rules=stop_rules, provenance=provenance, publication_branch=contract.PUBLICATION_BRANCH,
        execution=deepcopy(contract.EXECUTION),
        missed_gap_seed_check=dict(cycles=CYCLES_CHECKED, built=len(seed_check), rows=seed_check),
        authorization='Proposed by Claude for user review; training_authorized=false until the user approves '
                      'this configuration and the use of the remaining decision budget')
    contract.validate_config(cfg)
    args.output.write_bytes((json.dumps(cfg, indent=2) + '\n').encode('utf-8'))
    print(json.dumps(dict(round_index=cfg['round_index'], reward_version=cfg['reward_version'],
        budget={k: cfg['budget'][k] for k in ('per_round_decisions', 'total_decisions', 'total_wall_seconds',
                                              'max_diagnostic_episodes')},
        stop_rules={k: v for k, v in stop_rules.items() if not k.endswith('source')},
        missed_gap_seeds_built=len(seed_check),
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()), indent=1))


if __name__ == '__main__':
    main()
