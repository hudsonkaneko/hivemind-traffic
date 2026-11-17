"""V09 launch contract (Claude proposal, documentation/highway-merge-training-plan-v09.md).

Training stays disabled until the user approves the resolved configuration and
`training_authorized` is set to true in a reviewed commit. V09 is the v06 round-9
profile with exactly these changes, each checked here:
- reward version and settings (traffic/highway_flow_rl_v09_rewards.py);
- warm start: the complete round-9 final checkpoint instead of the v04 actor;
- round index 11 (index 10 records the shared evaluation in the ledger);
- budget: the shared study decision ceiling raised by one 300k round, and the
  diagnostic-episode cap raised by the one preflight episode;
- the merge-speed floor stops the run at periodic validation again, measured
  from round 9's final validation (the new starting policy);
- publication branch and execution location (Claude's own worktree; the shared
  ledger and lock are resolved through Git's common directory).
Every other field is validated by the unchanged v06 contract against the round-9
profile, so rows, seeds, safety, optimizer, acceptance and rate limit cannot drift.
"""
from copy import deepcopy

from traffic import highway_flow_rl_v06_contract as v06c
from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v09_rewards as v09

SCHEMA = 'highway_flow_rl_v09'
ROUND_INDEX = 11
PARENT_ROUND = 9
LEARNING_PROFILE = 'merge_v09_r11_tracking_quality'
WARM_START = dict(path='outputs/highway-flow-rl/v06/round-09/last.pt',
                  sha256='8f056b401a37316f0daaa387dbe3b41116f17d631a501cb4cf723dd971d31ab0')
WARM_START_SCHEMA = 'highway_flow_rl_checkpoint_v06'
PARENT_CONFIG_SHA256 = '91947d58d48feabe1efa97bb0a678909a27ba419ffab86b124e506cf2da99217'
# Shared ledger after the common evaluation: 4,921,946 training decisions, 41,512 s,
# 125 diagnostic episodes. One more 300k round needs the decision ceiling at
# 5,230,000; the wall-clock ceiling is unchanged; the preflight uses 1 episode.
BUDGET = dict(v06c.SHARED_STUDY_BUDGET, total_decisions=5230000, max_diagnostic_episodes=126)
ROUND9_FINAL_MERGE_SPEED_MATCH = .4543  # documentation/evidence/highway-merge-training-v06/round-09-results.json
FLOOR_MARGIN = .02
MERGE_FLOOR = round(ROUND9_FINAL_MERGE_SPEED_MATCH - FLOOR_MARGIN, 4)
PUBLICATION_BRANCH = '2026-11-17-claude-policy-candidate'
EXECUTION = dict(worktree='outputs/worktrees/claude-policy-2026-11-17',
                 shared_study='git common directory parent / outputs/highway-flow-rl/v02',
                 output_root='outputs/highway-flow-rl/v09',
                 inputs='hash-verified copies of the declared networks and warm start, staged by '
                        'scripts/stage_highway_flow_rl_v09_inputs.py')
V09_ONLY_KEYS = ('v09_reward_settings', 'execution', 'parent_config_sha256')


def round9_view(cfg):
    """The configuration as the v06 round-9 contract would see it, with the v09 changes reverted."""
    view = deepcopy(cfg)
    for key in V09_ONLY_KEYS:
        view.pop(key, None)
    view.update(schema=v06c.SCHEMA, round_index=PARENT_ROUND,
                learning_profile=v06c.ROUNDS[PARENT_ROUND]['learning_profile'],
                reward_version=v06.REWARD_VERSION, warm_start=deepcopy(v06c.WARM_START),
                budget=deepcopy(v06c.ROUNDS[PARENT_ROUND]['budget']))
    view['stop_rules'] = dict(view.get('stop_rules', {}),
                              merge_floor_action=v06c.ROUNDS[PARENT_ROUND]['merge_floor_action'])
    return view


def validate_config(cfg, require_training=False):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v09 schema or missing explicit authorization state')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('Training is disabled pending user approval of the resolved configuration')
    if cfg.get('round_index') != ROUND_INDEX or cfg.get('learning_profile') != LEARNING_PROFILE:
        raise ValueError('Expected the declared v09 round-11 profile')
    if cfg.get('reward_version') != v09.REWARD_VERSION or cfg.get('v09_reward_settings') != v09.SETTINGS:
        raise ValueError('V09 reward version/settings differ from the proposal')
    if cfg.get('warm_start') != WARM_START or cfg.get('parent_config_sha256') != PARENT_CONFIG_SHA256:
        raise ValueError('Expected the complete v06 round-9 final checkpoint as warm start')
    if cfg.get('budget') != BUDGET:
        raise ValueError('Declared v09 budget changed')
    stop = cfg.get('stop_rules', {})
    if (stop.get('any_collision') is not True or stop.get('merge_speed_match_floor') != MERGE_FLOOR
            or stop.get('merge_floor_action') != 'stop' or stop.get('max_pooled_intervention_fraction') != .05):
        raise ValueError('V09 stop rules: collision, merge floor from round 9 final (stop), 5% pooled interventions')
    if cfg.get('publication_branch') != PUBLICATION_BRANCH or cfg.get('execution') != EXECUTION:
        raise ValueError('V09 runs only from the Claude worktree and publishes to its branch')
    v06c.validate_config(round9_view(cfg))
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    return v06c.validate_preflight(proof, config_sha256, source_hashes)
