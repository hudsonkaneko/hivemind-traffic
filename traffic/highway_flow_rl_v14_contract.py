"""V14 launch contract: round 20 (documentation/highway-merge-training-plan-v14.md).

User decisions, November 19, 2026:
- "one controller on the car at all times" ... "Yes, policy only": v14, SUMO's own checks
  off for controlled cars (traffic/highway_flow_rl_v14_rewards.py);
- "Yes, build it": the optional grade-aware drive limit (traffic/road_grade.py);
- "Yes, raise it": raise the study budget so a v14 round can train with the drive limit on.
The standing autonomy decision of November 17 ("continue monitoring and modifting training
and changing rewards or weights ...") covers the round's settings. Held-out evaluation was
never authorized and stays off.

As in v12, the contract is the single source of truth: build_config(20) is round 19's
configuration (traffic/highway_flow_rl_v12_contract.build_config(19): rows, seeds, warm
start, rewards, losses, optimizer, validation cases) plus only the changes declared here,
and validate_config() requires the launched file to equal it exactly.
scripts/prepare_highway_flow_rl_v14.py only writes it out.

Round 20 changes against round 19:
- environment v14 (environments/highway_flow_rl_v14_env.py): the policy is the only
  controller of the controlled cars, and yield-entry targets check crossing gaps;
- the drive limit on (DRIVE_LIMIT): every controlled car's acceleration ceiling also holds
  the prepared Americano's drive on the road grade. LA rows take the grade from the road
  model; the other networks have no road model and count as flat;
- the collision rule. Round 19 stopped at any collision in any episode. The v14 starting
  policy already collides in one validation case (the focused northbound LA merge), so that
  rule would stop round 20 at its initial validation. Round 20 instead stops when a
  periodic validation is a safety regression against this round's own initial validation:
  a validation case collides that did not collide initially, or the validation has more
  collisions in total. Training-rollout collisions keep their -20 reward and are recorded;
  they do not stop the run by themselves;
- periodic validation every 50,000 decisions instead of 100,000. Round 19's acceleration-
  lane merges had broken by its first check at 106,915 decisions; checking twice as often
  keeps a checkpoint from before such a break;
- budget, below.
"""
from copy import deepcopy
import json

from traffic import highway_flow_rl_v12_contract as v12c
from traffic import highway_flow_rl_v14_rewards as v14
from traffic.highway_flow_contract import file_digest
from traffic.road_grade import DRIVE_LIMIT

SCHEMA = 'highway_flow_rl_v14'
BASE_ROUND = 19
# Round 19's launched configuration; build_config() requires v12's build_config(19) to equal it.
BASE_CONFIG = dict(path='configs/training/highway-flow-rl-v12-round19.json',
                   sha256='71b28804bbb66b8b4cd64de409d045906df4f5a252110923b1230d7b7f82db3f')
OUTPUT_ROOT = 'outputs/highway-flow-rl/v14'

# Shared ledger before round 20 (outputs/highway-flow-rl/v02/cost-ledger.json through Git's
# common directory): 6,129,132 of 6,230,000 training decisions, 52,939.6 of 61,200 s and 167 of
# 168 diagnostic episodes. 100,868 decisions remain, about half of round 19's 200,000.
# User decision "Yes, raise it" (November 19, 2026): total decisions +100,000 to 6,330,000, so a
# round-19-sized round fits; diagnostic episodes +2 to 170, for this round's preflight and one
# physics recording of its result. Wall time is not raised: 8,260.4 s remain, enough for the
# preflight, a 3,600 s round and the recording.
STUDY_BUDGET = dict(v12c.STUDY_BUDGET, total_decisions=6330000, max_diagnostic_episodes=170,
                    per_round_decisions=200000, per_round_wall_seconds=3600)

STOP_RULES = dict(deepcopy(v12c.ROUNDS[BASE_ROUND]['stop_rules']), any_collision=False,
    collision_regression=dict(scope='every periodic validation',
        stops_when='a validation case collides that did not collide in this round\'s initial validation, '
                   'or the validation has more collisions in total than the initial validation',
        training_rollouts='collisions keep their -20 reward and are recorded in updates/*.json; they do not '
                          'stop the run by themselves'),
    collision_source='the v14 starting policy already collides in the focused northbound LA merge '
                     '(validation case 10), so "any collision" would stop the round before it trains',
    checked_at='every periodic validation (interventions, relative merge floor, collision regression)')

ROUNDS = {
    20: dict(learning_profile='merge_v14_r20_policy_only_drive_limit', training=True, base_round=BASE_ROUND,
             budget=STUDY_BUDGET, evaluation_interval_decisions=50000, stop_rules=STOP_RULES,
             v14_drive_limit=json.loads(json.dumps(DRIVE_LIMIT))),
}


def base_config():
    """Round 19's configuration, checked against the launched file."""
    path = v12c.ROOT / BASE_CONFIG['path']
    if file_digest(path) != BASE_CONFIG['sha256']:
        raise ValueError('The pinned round-19 configuration changed')
    cfg = v12c.build_config(BASE_ROUND)
    if json.loads(path.read_text(encoding='utf-8')) != cfg:
        raise ValueError("v12's build_config(19) no longer equals the launched round-19 configuration")
    return cfg


def build_config(round_index):
    """The complete configuration for a declared v14 round."""
    row = ROUNDS[round_index]
    if row['base_round'] != BASE_ROUND:
        raise ValueError('Only round 19 is a pinned base')
    cfg = base_config()
    parent = cfg['learning_profile']
    cfg.update(schema=SCHEMA, training_authorized=row['training'], round_index=round_index,
        base_config=dict(BASE_CONFIG),
        learning_profile=row['learning_profile'], reward_version=v14.REWARD_VERSION,
        v14_settings=dict(v14.SETTINGS), v14_control=dict(v14.CONTROL),
        v14_drive_limit=deepcopy(row['v14_drive_limit']), budget=dict(row['budget']),
        evaluation_interval_decisions=row['evaluation_interval_decisions'],
        stop_rules=deepcopy(row['stop_rules']),
        execution=dict(cfg['execution'], output_root=OUTPUT_ROOT),
        provenance=dict(cfg['provenance'], parent_round=parent, round=row['learning_profile'],
                        evidence='documentation/highway-merge-training-plan-v14.md'),
        authorization=cfg['authorization'] + (
            ' Round 20, user decisions of November 19, 2026: "Yes, policy only" (v14), "Yes, build it" '
            '(the drive limit) and "Yes, raise it" (the study budget). This round trains; held-out '
            'evaluation stays unauthorized.'))
    return cfg


def validate_config(cfg, require_training=False):
    """A declared v14 round; a v12 configuration is accepted only for evaluation and recordings."""
    if cfg.get('schema') == v12c.SCHEMA:
        if require_training:
            raise ValueError('The v14 runner trains only declared v14 rounds')
        return v12c.validate_config(cfg)
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v14 schema or missing explicit authorization state')
    if cfg.get('round_index') not in ROUNDS:
        raise ValueError('Expected a declared v14 round')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('This v14 round is evaluation only')
    expected = build_config(cfg['round_index'])
    for key in sorted(set(expected) | set(cfg)):
        if cfg.get(key) != expected.get(key):
            raise ValueError(f'V14 configuration field {key!r} differs from the declared round')
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    return v12c.validate_preflight(proof, config_sha256, source_hashes)
