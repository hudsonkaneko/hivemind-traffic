"""V11 launch contract (documentation/highway-merge-training-plan-v11.md).

User decision, November 17, 2026: "Test merge rule. also i want you to use some
autonomy. continue monitoring and modifting training and changing rewards or
weights to achieve acceptable conditions. keep iterating until you get to that
point." V11 therefore runs as a series of rounds that Claude declares one at a
time in ROUNDS below, each in a reviewed commit before it runs. Held-out
evaluation was not part of that decision and stays unauthorized.

Each round is the v06 round-9 profile with only the changes its row declares:
- v11 settings (traffic/highway_flow_rl_v11_rewards.py), pinned to the round;
- warm start, round index, learning profile, budget and merge floor;
- publication branch and execution location (Claude's own worktree; the shared
  ledger and lock are resolved through Git's common directory).
Every other field is validated by the unchanged v06 contract against the round-9
profile, so rows, seeds, safety, optimizer, acceptance and rate limit cannot
drift unless a round's row says so.
"""
from copy import deepcopy

from traffic import highway_flow_rl_v06_contract as v06c
from traffic import highway_flow_rl_v10_contract as v10c
from traffic import highway_flow_rl_v10_rewards as v10
from traffic import highway_flow_rl_v11_rewards as v11

SCHEMA = 'highway_flow_rl_v11'
PARENT_CONFIG_SHA256 = v10c.PARENT_CONFIG_SHA256
ROUND9_WARM_START = deepcopy(v10c.WARM_START)
PUBLICATION_BRANCH = v10c.PUBLICATION_BRANCH
EXECUTION = dict(v10c.EXECUTION, output_root='outputs/highway-flow-rl/v11',
                 inputs='hash-verified copies of the declared networks and warm start, already staged for '
                        'v09/v10 and re-verified by scripts/prepare_highway_flow_rl_v11.py')
# Shared ledger before v11: 5,123,695 training decisions, 43,781.5 s, 128 diagnostic
# episodes. The user's autonomy decision covers further rounds; Claude bounds them
# here: at most 1,000,000 more decisions than the old 5,230,000 ceiling, 4 more
# hours of charged wall clock and 40 more diagnostic episodes. Raising any of
# these again needs a new note to the user.
STUDY_BUDGET = dict(v10c.BUDGET, total_decisions=6230000, total_wall_seconds=61200,
                    preparation_wall_seconds=7200, diagnostic_wall_seconds=6300, max_diagnostic_episodes=168)
ROUND9_FINAL_MERGE_SPEED_MATCH = v10c.ROUND9_FINAL_MERGE_SPEED_MATCH
V11_ONLY_KEYS = ('v11_settings', 'v11_mask_version', 'v11_training', 'v11_projection')

ROUNDS = {
    # Rule test: the frozen round-9 policy under the merge-speed precondition.
    # Evaluation only (8 development validation episodes); no training decisions.
    13: dict(learning_profile='merge_v11_r13_rule_test_frozen_round9', training=False,
             warm_start=ROUND9_WARM_START, budget=dict(STUDY_BUDGET, per_round_decisions=0),
             merge_floor=v10c.MERGE_FLOOR),
    # Second rule test (15 s lane-end release), also evaluation only.
    14: dict(learning_profile='merge_v11_r14_rule_test_release15_frozen_round9', training=False,
             warm_start=ROUND9_WARM_START, budget=dict(STUDY_BUDGET, per_round_decisions=0),
             merge_floor=v10c.MERGE_FLOOR),
    # First training round under the rule. Round 14 left three failures, two of them
    # requests the command filters change: speed-ceiling clamps right after a merge
    # (ramp 180 m, 2 lanes: 6 of 81, gate allows 4) and a request step of 0.62 m/s2
    # when a leader leaves the road end (I-110 following: raw request jerk 6.1,
    # gate 5). The shield-consistency loss (experiments/run_highway_flow_rl_v11.py)
    # trains the mean request to stay inside the filters' own band. Rewards are
    # v10's, unchanged. The merge floor only catches a collapse: one ramp car
    # flipping its stable merge moves the pooled score from 0.62 to 0.38, which
    # stopped rounds 11 and 12 on single-episode noise.
    15: dict(learning_profile='merge_v11_r15_rule_shield_consistency', training=True,
             warm_start=ROUND9_WARM_START, budget=dict(STUDY_BUDGET, per_round_decisions=200000),
             merge_floor=.3,
             merge_floor_source='collapse guard: round 14 (frozen round 9 under the rule) scored 0.618; '
                                'losing two of its three stable ramp merges gives about 0.15 to 0.38',
             training_settings=dict(shield_coef=50., ceiling_margin_m_s2=.05, rate_bound_m_s2=.35)),
    # Round 15 result: 5 of 8 gates at the start, 3 of 8 at 100k, 2 of 8 at the end.
    # PPO at this scale changed far more than the added loss asked for (cars stopped
    # at lane ends, raw request jerk up to 22 m/s3). Round 16 therefore drops PPO:
    # a supervised fit of the round-9 actor onto the filters' band, held to round 9
    # everywhere else (experiments/run_highway_flow_rl_v11_projection.py). Two
    # passes over the 20 training rows; each pass collects with the current actor.
    16: dict(learning_profile='merge_v11_r16_shield_projection', training=True,
             warm_start=ROUND9_WARM_START, budget=dict(STUDY_BUDGET, per_round_decisions=203200),
             merge_floor=.3,
             merge_floor_source='collapse guard, as in round 15 (not checked by the projection runner, which '
                                'reports every validation and selects by the shared validation rank)',
             projection_settings=dict(iterations=2, episodes_per_iteration=20, epochs=30, batch_size=1024,
                                      learning_rate=3e-4, hinge_weight=50., preserve_weight=1., lane_weight=1.,
                                      ceiling_margin_m_s2=.05, rate_bound_m_s2=.35)),
}


def round9_view(cfg):
    """The configuration as the v06 round-9 contract would see it, with the v11 changes reverted."""
    return v10c.round9_view({k: v for k, v in cfg.items() if k not in V11_ONLY_KEYS})


def validate_config(cfg, require_training=False):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v11 schema or missing explicit authorization state')
    row = ROUNDS.get(cfg.get('round_index'))
    if row is None or cfg.get('learning_profile') != row['learning_profile']:
        raise ValueError('Expected a declared v11 round and its profile')
    if cfg['training_authorized'] is not row['training']:
        raise ValueError('Training authorization differs from the declared round')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('This v11 round is evaluation only')
    if cfg.get('reward_version') != v11.REWARD_VERSION or cfg.get('v10_reward_settings') != v10.SETTINGS:
        raise ValueError('V11 reward version or inherited v10 settings changed')
    if (cfg.get('v11_settings') != v11.SETTINGS_BY_ROUND[cfg['round_index']]
            or cfg.get('v11_mask_version') != v11.MASK_VERSION):
        raise ValueError('V11 settings differ from the row reviewed for this round')
    if cfg.get('v11_projection') != row.get('projection_settings'):
        raise ValueError('V11 projection settings differ from the declared round')
    if cfg.get('v11_training') != row.get('training_settings'):
        raise ValueError('V11 training settings differ from the declared round')
    if cfg.get('warm_start') != row['warm_start'] or cfg.get('parent_config_sha256') != PARENT_CONFIG_SHA256:
        raise ValueError('Warm start differs from the declared round')
    if cfg.get('budget') != row['budget']:
        raise ValueError('Declared v11 budget changed')
    stop = cfg.get('stop_rules', {})
    if (stop.get('any_collision') is not True or stop.get('merge_speed_match_floor') != row['merge_floor']
            or stop.get('merge_floor_action') != 'stop' or stop.get('max_pooled_intervention_fraction') != .05):
        raise ValueError('V11 stop rules: collision, the declared merge floor (stop), 5% pooled interventions')
    if cfg.get('held_out_authorized') is not False:
        raise ValueError('Held-out evaluation is not authorized')
    if cfg.get('publication_branch') != PUBLICATION_BRANCH or cfg.get('execution') != EXECUTION:
        raise ValueError('V11 runs only from the Claude worktree and publishes to its branch')
    v06c.validate_config(round9_view(cfg))
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    return v06c.validate_preflight(proof, config_sha256, source_hashes)
