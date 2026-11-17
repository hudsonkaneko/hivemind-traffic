"""V10 launch contract (Claude proposal, documentation/highway-merge-training-plan-v10.md).

Training stays disabled until the user approves the resolved configuration and
`training_authorized` is set to true in a reviewed commit. V10 is the v06 round-9
profile with exactly these changes, each checked here:
- reward version and settings: the v09 terms plus the post-merge override cost
  (traffic/highway_flow_rl_v10_rewards.py);
- warm start: the complete round-9 final checkpoint, as in v09 (not the round-11
  checkpoint, which regressed);
- round index 12 (11 = v09 round 11 in the ledger);
- budget: no new decision ceiling. One 200,000-decision round fits in the 207,597
  decisions left under the 5,230,000 ceiling the user raised for v09; wall clock
  unchanged; diagnostic-episode cap raised by the one preflight episode;
- the merge-speed floor stops the run at periodic validation, measured from
  round 9's final validation (the starting policy);
- publication branch and execution location (Claude's own worktree; the shared
  ledger and lock are resolved through Git's common directory).
Every other field is validated by the unchanged v06 contract against the round-9
profile, so rows, seeds, safety, optimizer, acceptance and rate limit cannot drift.
"""
from copy import deepcopy

from traffic import highway_flow_rl_v06_contract as v06c
from traffic import highway_flow_rl_v09_contract as v09c
from traffic import highway_flow_rl_v10_rewards as v10

SCHEMA = 'highway_flow_rl_v10'
ROUND_INDEX = 12
PARENT_ROUND = v09c.PARENT_ROUND
LEARNING_PROFILE = 'merge_v10_r12_post_merge_overrides'
WARM_START = deepcopy(v09c.WARM_START)
WARM_START_SCHEMA = v09c.WARM_START_SCHEMA
PARENT_CONFIG_SHA256 = v09c.PARENT_CONFIG_SHA256
# Shared ledger after v09 round 11: 5,022,403 training decisions, 42,599.8 s,
# 126 diagnostic episodes. A 200k round fits under the existing 5,230,000 ceiling.
BUDGET = dict(v09c.BUDGET, per_round_decisions=200000, max_diagnostic_episodes=127)
ROUND9_FINAL_MERGE_SPEED_MATCH = v09c.ROUND9_FINAL_MERGE_SPEED_MATCH
FLOOR_MARGIN = v09c.FLOOR_MARGIN
MERGE_FLOOR = v09c.MERGE_FLOOR
PUBLICATION_BRANCH = v09c.PUBLICATION_BRANCH
EXECUTION = dict(v09c.EXECUTION, output_root='outputs/highway-flow-rl/v10',
                 inputs='hash-verified copies of the declared networks and warm start, staged by '
                        'scripts/stage_highway_flow_rl_v10_inputs.py')
V10_ONLY_KEYS = ('v10_reward_settings', 'execution', 'parent_config_sha256')


def round9_view(cfg):
    """The configuration as the v06 round-9 contract would see it, with the v10 changes reverted."""
    return v09c.round9_view({k: v for k, v in cfg.items() if k not in V10_ONLY_KEYS})


def validate_config(cfg, require_training=False):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v10 schema or missing explicit authorization state')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('Training is disabled pending user approval of the resolved configuration')
    if cfg.get('round_index') != ROUND_INDEX or cfg.get('learning_profile') != LEARNING_PROFILE:
        raise ValueError('Expected the declared v10 round-12 profile')
    if cfg.get('reward_version') != v10.REWARD_VERSION or cfg.get('v10_reward_settings') != v10.SETTINGS:
        raise ValueError('V10 reward version/settings differ from the proposal')
    if 'v09_reward_settings' in cfg:
        raise ValueError('V10 declares its reward settings once, in v10_reward_settings')
    if cfg.get('warm_start') != WARM_START or cfg.get('parent_config_sha256') != PARENT_CONFIG_SHA256:
        raise ValueError('Expected the complete v06 round-9 final checkpoint as warm start')
    if cfg.get('budget') != BUDGET:
        raise ValueError('Declared v10 budget changed')
    stop = cfg.get('stop_rules', {})
    if (stop.get('any_collision') is not True or stop.get('merge_speed_match_floor') != MERGE_FLOOR
            or stop.get('merge_floor_action') != 'stop' or stop.get('max_pooled_intervention_fraction') != .05):
        raise ValueError('V10 stop rules: collision, merge floor from round 9 final (stop), 5% pooled interventions')
    if cfg.get('publication_branch') != PUBLICATION_BRANCH or cfg.get('execution') != EXECUTION:
        raise ValueError('V10 runs only from the Claude worktree and publishes to its branch')
    v06c.validate_config(round9_view(cfg))
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    return v06c.validate_preflight(proof, config_sha256, source_hashes)
