"""V14 launch contract: rounds 20, 21 and 22 (documentation/highway-merge-training-plan-v14.md).

User decisions, November 19, 2026:
- "one controller on the car at all times" ... "Yes, policy only": v14, SUMO's own checks
  off for controlled cars (traffic/highway_flow_rl_v14_rewards.py);
- "Yes, build it": the optional grade-aware drive limit (traffic/road_grade.py);
- "Yes, raise it": raise the study budget so a v14 round can train with the drive limit on.
User decision, November 21, 2026 ("Yes, build and train", after "why did interventions go up
by so much" and "Yes, design a fix"): round 21, the stop fix, with the decision budget raised.
User decision, November 22, 2026 ("Yes, raise it", asked whether to raise the study budget for
round 22 and a physics check): round 22, with the decision and diagnostic-episode budgets raised.
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

Round 21 changes against round 20 (same rows, seeds, warm start, optimizer, drive limit and
validation cases, so the two rounds are a matched comparison):
- the stop fix on (traffic/highway_flow_rl_v14_stop_fix.py, reward version
  highway_flow_rl_merge_v14_merge_braking): merge_braking, v06's avoidable-braking cost for
  pending merges whose target needs no slowing, and target_band_tail, the band's slope past
  v12's cap. Round 20's actors braked to a stop on open road while their merge target was
  14-20 m/s; v06's cost never applied during a pending merge;
- a free-road stop rule at every periodic validation (free_road_stop_regression): stop when
  free-road stopped decisions exceed both 20 and twice this round's initial validation. Round
  20 had 0, 0, 25, 64 and 180; this rule would have stopped it at update 56;
- budget, below.

Round 22 changes against round 21 (same rows, seeds, warm start, optimizer, drive limit, stop fix,
stop rules and validation cases):
- the drive-aware reward target on (traffic/highway_flow_rl_v14_drive_target.py, reward version
  highway_flow_rl_merge_v14_drive_target): at yield entries the reward target starts at the car's
  speed and rises no faster than min(1.5 m/s^2, the drive ceiling). v14's target starts up to
  4.5 m/s above the car and rises at 1.5 m/s^2, faster than the drive gives on the southbound LA
  ramp (1.0-1.4 m/s^2). The gate, the merge-quality payments and the free-road stop measurement
  keep v14's target, so gate counts stay comparable with rounds 20 and 21;
- budget, below.

Round 23 changes against round 22 (same rows, seeds, warm start, optimizer, drive limit, stop fix,
stop rules and validation cases):
- the follow gate on instead of the drive-aware target (traffic/highway_flow_rl_v14_follow_gate.py,
  reward version highway_flow_rl_merge_v14_follow_gate). After round 22 a scripted car that follows
  v14's gate target matched it in only 15-66% of the window in all four LA merge cases (0.9 needed):
  at yield entries the target starts at the plan's speed, turns from falling to rising in one
  decision, and after the line returns to v12's road target, which jumps toward the speed limit.
  From a car's first yield-entry decision the gate now grades a reference that starts at the car's
  speed, moves toward the plan/road target at bounded acceleration and jerk, restarts from the car's
  speed after a yield and lasts through the merge window; a car matches within 2 m/s of the span
  between the reference and the road target. The reward target and the actor target inputs are the
  reference capped by the road target's leader and lane-end bounds; the merge-quality payments follow
  the gate. Validation summaries keep a diagnostic count on v14's target for comparison;
- budget: not raised (the remaining 161,620 decisions, 3,306.7 s and one diagnostic episode).
"""
from copy import deepcopy
import json

from traffic import highway_flow_rl_v12_contract as v12c
from traffic import highway_flow_rl_v14_rewards as v14
from traffic import highway_flow_rl_v14_stop_fix as stop_fix
from traffic import highway_flow_rl_v14_drive_target as drive_target
from traffic import highway_flow_rl_v14_follow_gate as follow_gate
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

# Shared ledger before round 21: 6,323,988 of 6,330,000 training decisions, 55,145.6 of 61,200 s
# and 169 of 170 diagnostic episodes. User decision "Yes, build and train" (November 21, 2026),
# whose choice said it implies raising the decision budget: total decisions +200,000 to
# 6,530,000, so a 200,000-decision round fits. Wall time (6,054.4 s remain: the 3,600 s round and
# the 900 s final reserve) and diagnostic episodes (one left, for this round's preflight) are not
# raised.
ROUND21_BUDGET = dict(STUDY_BUDGET, total_decisions=6530000)

ROUND21_STOP_RULES = dict(deepcopy(STOP_RULES),
    free_road_stop_regression=dict(minimum=20, factor=2., scope='every periodic validation',
        stops_when="free-road stopped decisions exceed both the minimum and factor x this round's initial "
                   'validation',
        definition='speed before the decision below 0.1 m/s, no own-lane leader within 20 m and an eligible '
                   'target of at least 5 m/s (traffic.highway_flow_rl_v14_stop_fix.STOP_SETTINGS)',
        source='round 20 validations: 0, 0, 25, 64 and 180 free-road stopped decisions (initial, updates 31, '
               '56 and 91, final)'),
    checked_at='every periodic validation (interventions, relative merge floor, collision regression, '
               'free-road stops)')

# Shared ledger before round 22: 6,517,702 of 6,530,000 training decisions, 57,275.2 of 61,200 s
# and 170 of 170 diagnostic episodes. User decision "Yes, raise it" (November 22, 2026): total
# decisions +200,000 to 6,730,000, so a 200,000-decision round fits; diagnostic episodes +2 to 172,
# for this round's preflight and one physics recording of its result. Wall time is not raised:
# 3,924.8 s remain, enough for the preflight, a round of round 21's length (2,126.8 s), the
# recording and the 900 s final reserve.
ROUND22_BUDGET = dict(ROUND21_BUDGET, total_decisions=6730000, max_diagnostic_episodes=172)

# Shared ledger before round 23: 6,568,380 of 6,730,000 training decisions, 57,893.3 of 61,200 s and
# 171 of 172 diagnostic episodes. The budget is not raised: the round trains within the remaining
# 161,620 decisions (the runner stops at the total) and keeps the 900 s final reserve; the last
# diagnostic episode is this round's preflight, so no physics recording of its result fits.
ROUND23_BUDGET = dict(ROUND22_BUDGET)

ROUND20_AUTHORIZATION = (
    ' Round 20, user decisions of November 19, 2026: "Yes, policy only" (v14), "Yes, build it" '
    '(the drive limit) and "Yes, raise it" (the study budget). This round trains; held-out '
    'evaluation stays unauthorized.')
ROUND21_AUTHORIZATION = ROUND20_AUTHORIZATION + (
    ' Round 21, user decision of November 21, 2026: "Yes, build and train" (the stop fix, the '
    'free-road stop rule and 200,000 more training decisions). This round trains; held-out '
    'evaluation stays unauthorized.')

ROUND22_AUTHORIZATION = ROUND21_AUTHORIZATION + (
    ' Round 22, user decision of November 22, 2026: "Yes, raise it" (200,000 more training '
    'decisions and 2 more diagnostic episodes, for round 22 and a physics check). This round '
    'trains; held-out evaluation stays unauthorized.')

ROUNDS = {
    20: dict(learning_profile='merge_v14_r20_policy_only_drive_limit', training=True, base_round=BASE_ROUND,
             budget=STUDY_BUDGET, evaluation_interval_decisions=50000, stop_rules=STOP_RULES,
             v14_drive_limit=json.loads(json.dumps(DRIVE_LIMIT))),
    21: dict(learning_profile='merge_v14_r21_stop_fix', training=True, base_round=BASE_ROUND,
             parent_profile='merge_v14_r20_policy_only_drive_limit',
             budget=ROUND21_BUDGET, evaluation_interval_decisions=50000, stop_rules=ROUND21_STOP_RULES,
             v14_drive_limit=json.loads(json.dumps(DRIVE_LIMIT)),
             reward_version=stop_fix.REWARD_VERSION,
             v14_stop_fix=dict(stop_fix.SETTINGS, obligations=list(stop_fix.SETTINGS['obligations'])),
             authorization=ROUND21_AUTHORIZATION),
    22: dict(learning_profile='merge_v14_r22_drive_target', training=True, base_round=BASE_ROUND,
             parent_profile='merge_v14_r21_stop_fix',
             budget=ROUND22_BUDGET, evaluation_interval_decisions=50000, stop_rules=ROUND21_STOP_RULES,
             v14_drive_limit=json.loads(json.dumps(DRIVE_LIMIT)),
             reward_version=drive_target.REWARD_VERSION,
             v14_stop_fix=dict(stop_fix.SETTINGS, obligations=list(stop_fix.SETTINGS['obligations'])),
             v14_drive_target=dict(drive_target.SETTINGS),
             authorization=ROUND22_AUTHORIZATION),
    23: dict(learning_profile='merge_v14_r23_follow_gate', training=True, base_round=BASE_ROUND,
             parent_profile='merge_v14_r22_drive_target',
             budget=ROUND23_BUDGET, evaluation_interval_decisions=50000, stop_rules=ROUND21_STOP_RULES,
             v14_drive_limit=json.loads(json.dumps(DRIVE_LIMIT)),
             reward_version=follow_gate.REWARD_VERSION,
             v14_stop_fix=dict(stop_fix.SETTINGS, obligations=list(stop_fix.SETTINGS['obligations'])),
             v14_follow_gate=dict(follow_gate.SETTINGS, reward_caps=list(follow_gate.SETTINGS['reward_caps'])),
             authorization=ROUND22_AUTHORIZATION + (
                 ' Round 23, user decision of November 24, 2026: "yes redesign" (a merge-quality gate target '
                 'a car can follow, checked with a scripted follower, then round 23 within the remaining '
                 'budget). This round trains; held-out evaluation stays unauthorized.')),
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
        learning_profile=row['learning_profile'], reward_version=row.get('reward_version', v14.REWARD_VERSION),
        v14_settings=dict(v14.SETTINGS), v14_control=dict(v14.CONTROL),
        v14_drive_limit=deepcopy(row['v14_drive_limit']), budget=dict(row['budget']),
        evaluation_interval_decisions=row['evaluation_interval_decisions'],
        stop_rules=deepcopy(row['stop_rules']),
        execution=dict(cfg['execution'], output_root=OUTPUT_ROOT),
        provenance=dict(cfg['provenance'], parent_round=row.get('parent_profile', parent), round=row['learning_profile'],
                        evidence='documentation/highway-merge-training-plan-v14.md'),
        authorization=cfg['authorization'] + row.get('authorization', ROUND20_AUTHORIZATION))
    if 'v14_stop_fix' in row:
        cfg['v14_stop_fix'] = deepcopy(row['v14_stop_fix'])
    if 'v14_drive_target' in row:
        cfg['v14_drive_target'] = deepcopy(row['v14_drive_target'])
    if 'v14_follow_gate' in row:
        cfg['v14_follow_gate'] = deepcopy(row['v14_follow_gate'])
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
