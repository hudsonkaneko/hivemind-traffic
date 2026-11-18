"""Write the resolved configuration of one declared v11 round.

Derived, like v09 and v10, from the committed v06 round-9 configuration
(configs/training/highway-flow-rl-v06-round9.json). Rows, observation, actions,
safety, rate limit, optimizer, acceptance and raw-request scope are unchanged
unless the round's row in traffic/highway_flow_rl_v11_contract.py says so; that
contract checks it. The warm start and every declared network are verified by
SHA-256 in this worktree. No simulation and no ledger charge.
"""
import argparse
import hashlib
import json
import sys
from copy import deepcopy
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from traffic import highway_flow_rl_v10_rewards as v10  # noqa: E402
from traffic import highway_flow_rl_v11_contract as contract  # noqa: E402
from traffic import highway_flow_rl_v11_rewards as v11  # noqa: E402

USER_DECISION = ('User decision, November 17, 2026: "Test merge rule. also i want you to use some autonomy. '
                 'continue monitoring and modifting training and changing rewards or weights to achieve '
                 'acceptable conditions. keep iterating until you get to that point."')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(round_index, round9_path=HERE / 'configs/training/highway-flow-rl-v06-round9.json', check_inputs=True):
    row = contract.ROUNDS[round_index]
    raw = Path(round9_path).read_bytes().replace(b'\r\n', b'\n')  # Windows checkouts may hold CRLF copies
    if hashlib.sha256(raw).hexdigest() != contract.PARENT_CONFIG_SHA256:
        raise ValueError('Round-9 configuration changed')
    cfg = json.loads(raw.decode('utf-8'))
    if check_inputs:
        if sha256(HERE / row['warm_start']['path']) != row['warm_start']['sha256']:
            raise ValueError('Warm start differs from the declared checkpoint')
        for item in cfg['training_rows'] + cfg['validation_rows']:
            if sha256(HERE / item['case']['network']) != item['case']['network_sha256']:
                raise ValueError('A declared network is missing or changed: ' + item['case']['network'])
    stop_rules = dict(any_collision=True, merge_speed_match_floor=row['merge_floor'],
        merge_floor_source=row.get('merge_floor_source',
            f'round-9 final validation in the v06 environment ({contract.ROUND9_FINAL_MERGE_SPEED_MATCH}) minus 0.02'),
        merge_floor_action='stop', max_pooled_intervention_fraction=.05,
        intervention_source='per-case development gate value (5%)',
        checked_at='every periodic validation; collisions stop the run at any episode')
    cfg.update(schema=contract.SCHEMA, round_index=round_index, training_authorized=row['training'],
        learning_profile=row['learning_profile'], reward_version=v11.REWARD_VERSION,
        v10_reward_settings=deepcopy(v10.SETTINGS), v11_settings=deepcopy(v11.SETTINGS_BY_ROUND[round_index]),
        v11_mask_version=v11.MASK_VERSION, warm_start=deepcopy(row['warm_start']),
        parent_config_sha256=contract.PARENT_CONFIG_SHA256, budget=deepcopy(row['budget']),
        stop_rules=stop_rules, held_out_authorized=False, publication_branch=contract.PUBLICATION_BRANCH,
        execution=deepcopy(contract.EXECUTION),
        provenance=dict(parent_config_sha256=contract.PARENT_CONFIG_SHA256,
            evidence='documentation/highway-merge-training-plan-v11.md', round=row['learning_profile']),
        authorization=USER_DECISION + (' This round trains.' if row['training'] else
                                       ' This round is evaluation only; it makes no training decisions.'))
    if row.get('training_settings') is not None:
        cfg['v11_training'] = deepcopy(row['training_settings'])
    cfg.pop('missed_gap_seed_check', None)
    contract.validate_config(cfg)
    return cfg


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round', type=int, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    cfg = build(args.round)
    output = args.output or HERE / f'configs/training/highway-flow-rl-v11-round{args.round}.json'
    output.write_bytes((json.dumps(cfg, indent=2) + '\n').encode('utf-8'))
    print(json.dumps(dict(round_index=cfg['round_index'], profile=cfg['learning_profile'],
        training=cfg['training_authorized'], v11_settings=cfg['v11_settings'],
        budget={k: cfg['budget'][k] for k in ('per_round_decisions', 'total_decisions', 'total_wall_seconds',
                                              'max_diagnostic_episodes')},
        sha256=sha256(output)), indent=1))


if __name__ == '__main__':
    main()
