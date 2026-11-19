"""V12 launch contract (documentation/highway-merge-training-plan-v12.md).

User requests, November 18, 2026:
- "lets add the target input then add a reward with decent weight to focus on matching
  that target speed. it should have some tolerance around it tho so it could be between
  like 60-70 mph instead of only 65 etc. do some training on that and report back"
- "the new 110 3d model should be the primary model used in the training and simulation"
The standing autonomy decision of November 17 ("continue monitoring and modifting
training and changing rewards or weights ...") covers further rounds. Held-out
evaluation was never authorized and stays off.

The contract is the single source of truth: build_config() derives the whole round
configuration from the pinned round-16 configuration (the one that produced the warm
start) plus the changes declared here, and validate_config() requires the launched
file to equal it exactly. scripts/prepare_highway_flow_rl_v12.py only writes it out.
Changes against round 16:
- environment v12: four target-speed inputs (121-field actor, 2928-field critic) and
  the target_band reward (traffic/highway_flow_rl_v12_rewards.py);
- primary road model: the LA I-10/I-110 OpenUSD model, through two corridors cut from
  its lane graph (scripts/build_la_10_110_corridors.py). Twelve of twenty training rows
  and six of fourteen validation rows run on them; the other rows are round 16's, so
  the eight round-16 validation episodes stay comparable;
- warm start: the round-16 pass-2 actor (7 of 8 development gates), migrated with zero
  weights on the new inputs;
- learning: buffered PPO with the shield-consistency loss and an anchor to the warm start
  outside active merge targets (experiments/run_highway_flow_rl_v12.py);
- budget and stop rules below.
"""
from copy import deepcopy
import json
from pathlib import Path

from traffic import highway_flow_rl_v06_contract as v06c
from traffic import highway_flow_rl_v12_rewards as v12
from traffic.highway_flow_contract import file_digest

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'highway_flow_rl_v12'
REFERENCE_CONFIG = dict(path='configs/training/highway-flow-rl-v11-round16.json',
                        sha256='b4c616142be73fcb191bd0d5ab65250f64d4692e629e74c7da147070d244cbe2')
WARM_START = dict(path='outputs/highway-flow-rl/v11/round-16/candidates/checkpoint-02.pt',
                  sha256='651d20fecaec4d6361f91ad00c103e4f3be6198fb64a8a87871d306aa553bed7')
WARM_START_SCHEMA = 'highway_flow_rl_checkpoint_v11_projection'
PUBLICATION_BRANCH = '2026-11-17-claude-policy-candidate'
EXECUTION = dict(worktree='outputs/worktrees/claude-policy-2026-11-17',
                 shared_study='git common directory parent / outputs/highway-flow-rl/v02',
                 output_root='outputs/highway-flow-rl/v12',
                 inputs='LA corridors from scripts/build_la_10_110_corridors.py (outputs/highway-flow-rl/v12/'
                        'la-corridors-01, checksums pinned below); round-16 networks and the warm start as '
                        'already staged in this worktree for v11')
MODEL = 'highway_usd/la_10_110'
LA_CORRIDORS = {
    'la-110-nb-pasadena-onramp': dict(short='nb-pasadena', direction='CA-110 northbound', through_lanes=3,
        network='outputs/highway-flow-rl/v12/la-corridors-01/la-110-nb-pasadena-onramp/network.net.xml',
        network_sha256='53d9edab6c1868d1a862b5ef613cf24b42d0199d7e01be1cc17cd79568b6b411'),
    'la-110-sb-adams-onramp': dict(short='sb-adams', direction='CA-110 southbound', through_lanes=3,
        network='outputs/highway-flow-rl/v12/la-corridors-01/la-110-sb-adams-onramp/network.net.xml',
        network_sha256='dffc3282177cd7272a89bee3567d7db472b672dfb97f7d571d31ebfbcaa81e05'),
}
LA_SPEED_LIMIT_M_S = 26.8224   # 60 mph: the corridors' mainline speed and the global safety ceiling
LA_HORIZON_STEPS = 800

# (corridor, task, density, controlled, total, difficulty, communication, packet loss, missed gap, seed)
LA_TRAINING = (
    ('la-110-nb-pasadena-onramp', 'merge', 'medium', 4, 16, 0, 'local', 0., False, 9300000),
    ('la-110-sb-adams-onramp', 'merge', 'medium', 1, 16, 1, 'local', 0., False, 9300001),
    ('la-110-nb-pasadena-onramp', 'following', 'dense', 4, 16, 2, 'local', 0., False, 9300002),
    ('la-110-sb-adams-onramp', 'merge', 'medium', 8, 24, 2, 'local', 0., False, 9300003),
    ('la-110-nb-pasadena-onramp', 'merge', 'medium', 1, 16, 1, 'local', 0., False, 9300004),
    ('la-110-sb-adams-onramp', 'passing', 'medium', 4, 16, 0, 'local', 0., False, 9300005),
    ('la-110-nb-pasadena-onramp', 'merge', 'medium', 1, 8, 1, 'local', .1, True, 9300006),
    ('la-110-sb-adams-onramp', 'merge', 'medium', 4, 16, 0, 'local', 0., False, 9300007),
    ('la-110-nb-pasadena-onramp', 'cruise', 'medium', 8, 16, 0, 'local', 0., False, 9300008),
    ('la-110-sb-adams-onramp', 'following', 'dense', 8, 16, 2, 'local', 0., False, 9300009),
    ('la-110-nb-pasadena-onramp', 'merge', 'medium', 8, 24, 2, 'local', 0., False, 9300010),
    ('la-110-sb-adams-onramp', 'merge', 'dense', 1, 24, 1, 'local', 0., False, 9300011),
)
LA_VALIDATION = (
    ('la-110-nb-pasadena-onramp', 'merge', 'medium', 8, 16, 2, 'local', 0., False, 6950001),
    ('la-110-sb-adams-onramp', 'merge', 'medium', 8, 16, 2, 'local', 0., False, 6950002),
    ('la-110-nb-pasadena-onramp', 'merge', 'medium', 1, 16, 1, 'local', 0., False, 6950003),
    ('la-110-sb-adams-onramp', 'merge', 'medium', 1, 16, 1, 'local', 0., False, 6950004),
    ('la-110-nb-pasadena-onramp', 'following', 'dense', 8, 16, 2, 'local', 0., False, 6950005),
    ('la-110-sb-adams-onramp', 'passing', 'medium', 4, 16, 0, 'local', 0., False, 6950006),
)
# Round-16 training rows kept (by index), interleaved after the LA rows at these positions.
KEPT_ROUND16_ROWS = (0, 1, 4, 5, 6, 13, 14, 18)
KEPT_POSITIONS = (1, 4, 7, 10, 12, 15, 17, 19)

# Shared ledger before v12: 5,627,084 training decisions, 48,250.8 s, 162 diagnostic
# episodes, against the v11 study budget (6,230,000 decisions, 61,200 s, 168 episodes).
# Round 18 stays inside it; nothing is raised.
STUDY_BUDGET = dict(per_round_decisions=400000, total_decisions=6230000, per_round_wall_seconds=5400,
    total_wall_seconds=61200, final_reserve_seconds=900, inherited_spent_seconds=0,
    preparation_wall_seconds=7200, diagnostic_wall_seconds=6300, rollout_decisions=38400,
    max_diagnostic_episodes=168, conditional_second_round_authorized=False)

ROUNDS = {
    18: dict(learning_profile='merge_v12_r18_target_band_la_primary', training=True,
             budget=STUDY_BUDGET, critic_warmup_decisions=30000, evaluation_interval_decisions=100000,
             optimizer_changes=dict(actor_lr=5e-5, target_kl=.05),
             v12_training=dict(anchor_coef=10., anchor_lane_coef=1., shield_coef=50., ceiling_margin_m_s2=.05,
                               rate_bound_m_s2=.35, comfort_speed_ceiling=True),
             stop_rules=dict(any_collision=True, merge_speed_match_floor=.3, merge_floor_action='record',
                 relative_merge_floor=dict(drop=.25, minimum=.1),
                 merge_floor_source='collapse guard relative to this round\'s initial validation (the LA rows have '
                                    'no earlier score); the static 0.3 floor is recorded only',
                 max_pooled_intervention_fraction=.05, intervention_source='per-case development gate value (5%)',
                 checked_at='every periodic validation; collisions stop the run at any episode')),
}
V11_ONLY = ('v11_projection', 'v11_training')


def reference_config():
    path = ROOT / REFERENCE_CONFIG['path']
    if file_digest(path) != REFERENCE_CONFIG['sha256']:
        raise ValueError('The pinned round-16 reference configuration changed')
    return json.loads(path.read_text())


def la_row(spec, split):
    corridor, task, density, controlled, total, difficulty, communication, loss, missed, seed = spec
    site = LA_CORRIDORS[corridor]
    focus = '-focused' if task == 'merge' and controlled == 1 else ''
    gap = '-missed-gap' if missed else ''
    prefix = 'la' if split == 'train' else 'v12-val-la'
    case = dict(id=f'{prefix}-{site["short"]}-{task}{focus}{gap}-{controlled}of{total}-{density}', split=split,
        network=site['network'], network_sha256=site['network_sha256'], through_lanes=site['through_lanes'],
        speed_limit_mph=60, task=task, density=density, road_model=MODEL, corridor=corridor,
        direction=site['direction'], geometry_version='la_10_110_corridor_v1',
        geometry_note='real CA-110 through lanes from the LA I-10/I-110 model; modeled 250 m auxiliary lane '
                      '(the model has none); OSM-derived, ODbL',
        visibility_assumption='SUMO default sensing on the cut corridor; not surveyed sight distance')
    row = dict(case=case, controlled_count=controlled, total_vehicles=total, seed=seed,
               speed_limit_m_s=LA_SPEED_LIMIT_M_S, communication=communication, packet_loss=loss,
               difficulty=difficulty, traffic_mode='initial', horizon_steps=LA_HORIZON_STEPS)
    if split == 'train':
        row['rollout_decisions'] = LA_HORIZON_STEPS * controlled
    if missed:
        row['missed_gap'] = True
    return row


def training_rows(reference):
    la = [la_row(spec, 'train') for spec in LA_TRAINING]
    kept = [deepcopy(reference['training_rows'][index]) for index in KEPT_ROUND16_ROWS]
    rows, la_iter, kept_iter = [], iter(la), iter(kept)
    for position in range(len(la) + len(kept)):
        rows.append(next(kept_iter) if position in KEPT_POSITIONS else next(la_iter))
    return rows


def build_config(round_index, reference=None):
    """The complete configuration for a declared v12 round."""
    reference = deepcopy(reference or reference_config())
    row = ROUNDS[round_index]
    cfg = {k: v for k, v in reference.items() if k not in V11_ONLY}
    cfg.update(schema=SCHEMA, training_authorized=row['training'], round_index=round_index,
        learning_profile=row['learning_profile'], reward_version=v12.REWARD_VERSION,
        observation_version=v12.OBSERVATION_VERSION, observation_size=121, state_size=2928,
        v12_settings=dict(v12.SETTINGS), v12_training=dict(row['v12_training']),
        optimizer=dict(reference['optimizer'], **row['optimizer_changes']),
        budget=dict(row['budget']), critic_warmup_decisions=row['critic_warmup_decisions'],
        evaluation_interval_decisions=row['evaluation_interval_decisions'], stop_rules=deepcopy(row['stop_rules']),
        warm_start=dict(WARM_START), parent_config_sha256=REFERENCE_CONFIG['sha256'],
        training_rows=training_rows(reference),
        validation_rows=deepcopy(reference['validation_rows']) + [la_row(s, 'validation') for s in LA_VALIDATION],
        held_out_authorized=False, development_gates_passed=False,
        publication_branch=PUBLICATION_BRANCH, execution=dict(EXECUTION), road_model=MODEL,
        provenance=dict(parent_config_sha256=REFERENCE_CONFIG['sha256'], parent_round=reference['learning_profile'],
                        evidence='documentation/highway-merge-training-plan-v12.md', round=row['learning_profile']),
        authorization='User requests, November 18, 2026: "lets add the target input then add a reward with decent '
                      'weight to focus on matching that target speed ... do some training on that and report back" '
                      'and "the new 110 3d model should be the primary model used in the training and simulation". '
                      'This round trains; held-out evaluation stays unauthorized.',
        coverage_note='Twelve LA corridor rows interleaved with eight round-16 rows; full declared horizons; '
                      'focused one-AV merges mixed with multi-AV populations.',
        split_note='Held-out demand seeds, not a claim of held-out geometry. The LA validation rows share the '
                   'training corridors with disjoint seeds. Do not tune on held-out results.')
    return cfg


def validate_config(cfg, require_training=False):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v12 schema or missing explicit authorization state')
    if cfg.get('round_index') not in ROUNDS:
        raise ValueError('Expected a declared v12 round')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('This v12 round is evaluation only')
    expected = build_config(cfg['round_index'])
    for key in sorted(set(expected) | set(cfg)):
        if cfg.get(key) != expected.get(key):
            raise ValueError(f'V12 configuration field {key!r} differs from the declared round')
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    return v06c.validate_preflight(proof, config_sha256, source_hashes)
