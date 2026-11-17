"""Write the resolved v09 round-11 proposal configuration (training_authorized stays false).

Derived from the committed v06 round-9 configuration
(configs/training/highway-flow-rl-v06-round9.json). Rows, observation, actions,
safety, rate limit, optimizer, acceptance and raw-request scope are unchanged;
traffic/highway_flow_rl_v09_contract.py checks that, and lists every change.
Round index 11 shifts every training seed; the missed-gap seeds of the curriculum
cycles a 300k-decision round can reach (PPO cycles 0-2, critic cycle 0; 4 cycles
checked as for round 9) are rebuilt to prove they build.
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
from traffic import highway_flow_rl_v09_contract as contract  # noqa: E402
from traffic import highway_flow_rl_v09_rewards as v09  # noqa: E402
from traffic.highway_merge_context import Stage1LaneGraph  # noqa: E402
from traffic.highway_v05_demand import write_demand  # noqa: E402

CYCLES_CHECKED = 4
ROUND9_RESULTS = HERE / 'documentation/evidence/highway-merge-training-v06/round-09-results.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True, help='canonical checkout holding outputs/')
    parser.add_argument('--round9', type=Path, default=HERE / 'configs/training/highway-flow-rl-v06-round9.json')
    parser.add_argument('--output', type=Path, default=HERE / 'configs/training/highway-flow-rl-v09-round11.json')
    args = parser.parse_args()
    raw = args.round9.read_bytes().replace(b'\r\n', b'\n')  # Windows checkouts may hold CRLF copies
    if hashlib.sha256(raw).hexdigest() != contract.PARENT_CONFIG_SHA256:
        raise ValueError('Round-9 configuration changed')
    cfg = json.loads(raw.decode('utf-8'))
    results = json.loads(ROUND9_RESULTS.read_text(encoding='utf-8'))
    final_match = results['validations']['final-validation']['merge_speed_match']
    if round(final_match, 4) != contract.ROUND9_FINAL_MERGE_SPEED_MATCH:
        raise ValueError('Round-9 final merge speed match differs from the contract floor source')
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
        checked_at='every periodic validation (100k, 200k decisions); collisions stop the run at any episode')
    provenance = dict(parent_policy_sha256=contract.WARM_START['sha256'],
        parent_run='v06 round 9 final (outputs/highway-flow-rl/v06/round-09/last.pt, canonical checkout)',
        parent_config_sha256=contract.PARENT_CONFIG_SHA256,
        previous_round=('v06 round 9: 157,957 decisions, merge speed match 0.437 to 0.454 (I-110 0.534 to 0.590), '
                        'gates unchanged; strongest actor in the shared 30-episode comparison (48.8%)'),
        evidence='documentation/highway-merge-training-plan-v09.md (round-9 reward analysis)',
        changes=['merge-speed tracking keeps its slope past 2 m/s (cap at 8 m/s; v06 saturated at 2 m/s)',
                 'graded merge-quality payment: 5 x speed-match fraction x neighbor-calm fraction, own-car conditions required',
                 'jerk excess does not charge the raw request during the fixed 5-decision insertion lag',
                 'warm start: complete round-9 final checkpoint (actor, critic); critic warm-up 20,000 decisions as round 9',
                 'merge-speed floor (round-9 final - 0.02) stops the run at periodic validation',
                 'round index 11; study decision ceiling +300,000; diagnostic-episode cap +1 for the preflight'])
    cfg.update(schema=contract.SCHEMA, round_index=contract.ROUND_INDEX, training_authorized=False,
        learning_profile=contract.LEARNING_PROFILE, reward_version=v09.REWARD_VERSION,
        v09_reward_settings=deepcopy(v09.SETTINGS), warm_start=deepcopy(contract.WARM_START),
        parent_config_sha256=contract.PARENT_CONFIG_SHA256, budget=deepcopy(contract.BUDGET),
        stop_rules=stop_rules, provenance=provenance, publication_branch=contract.PUBLICATION_BRANCH,
        execution=deepcopy(contract.EXECUTION),
        missed_gap_seed_check=dict(cycles=CYCLES_CHECKED, built=len(seed_check), rows=seed_check),
        authorization='Proposed by Claude for user review; training_authorized=false until the user approves '
                      'this configuration and the budget increase')
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
