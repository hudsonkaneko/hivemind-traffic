"""V07 launch contract (documentation/highway-merge-training-plan-v06.md, Phase 3).

Separate from the v05 contract. Training stays disabled until the user approves
the resolved configuration and `training_authorized` is set to true in a reviewed
commit. The study ledger, safety settings, MAPPO optimizer and flow117 ABI are
those of v05; the reward, observation fading, rate limit, controlled
car-following model, missed-gap rows and stop rules are v06.
"""
from traffic.highway_flow_rl_contract import ACTION_CONTRACT, OPTIMIZER, REWARD_WEIGHTS
from traffic.highway_flow_rl_v04_rewards import TARGET_SETTINGS, MASK_VERSION
from traffic.highway_flow_rl_v05_contract import artifact, number
from traffic import highway_flow_rl_v06_rewards as v06

SCHEMA = 'highway_merge_cutin_v07'
PREFLIGHT_SCHEMA = 'highway_flow_rl_preflight_v05'  # produced by the shared runner; bound to the v06 config hash
PREFLIGHT_CHECKS = ('api', 'shapes', 'native_guards', 'finite_gradient', 'export', 'reward_version', 'mask_version')
ROUND_INDEX = 6
WARM_START = dict(path='outputs/highway-flow-rl/v04/long-round-03/best.pt',
                  sha256='79e36b5425fa8a37f02ca990af23854f2cc5d73d6e62d27738ea767a55c25f0f')
BUDGET = dict(per_round_decisions=300000, total_decisions=4930000,
              per_round_wall_seconds=5400, total_wall_seconds=46800, final_reserve_seconds=900,
              inherited_spent_seconds=0, preparation_wall_seconds=3600, diagnostic_wall_seconds=2700,
              rollout_decisions=38400, max_diagnostic_episodes=100,
              conditional_second_round_authorized=False)
ACCEPTANCE = dict(max_intervention_fraction=.05, max_peak_jerk_m_s3=5., collisions=0, teleports=0,
                  overspeed_steps=0, report_per_car=True, full_route=True, min_headway_s=1.5, min_ttc_s=3.,
                  max_optional_detour_events=0, max_raw_requested_jerk_m_s3=5., road_end_exemption=False)
RAMP_STARTED_PREFIX = 'outputs/highway-flow-rl/v05/ramp-networks-02/'
I110_NETWORK = 'outputs/highway-flow-rl/v02/network-screen-01/candidate/network.net.xml'


def validate_config(cfg, require_training=False):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v06 schema or missing explicit authorization state')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('Training is disabled pending user approval of the resolved configuration')
    if (cfg.get('observation_size') != 117 or cfg.get('state_size') != 2832 or cfg.get('max_agents') != 24
            or cfg.get('action_contract') != ACTION_CONTRACT):
        raise ValueError('Flow117 actor/action/critic ABI changed')
    if cfg.get('safety') != dict(speed_mode=31, lane_change_mode=512, native_guards_unchanged=True,
                                 safety_shield_unchanged=True, global_speed_ceiling_m_s=26.8224):
        raise ValueError('Native or explicit safety settings changed')
    if cfg.get('round_index') != ROUND_INDEX or cfg.get('learning_profile') != 'merge_cutin_v07_300k_20epochs':
        raise ValueError('Expected the v06 round-six profile')
    if cfg.get('optimizer') != dict(OPTIMIZER, epochs=20) or cfg.get('gae') != {'gamma': .99, 'lambda': .95}:
        raise ValueError('Declared MAPPO optimizer or GAE changed')
    if cfg.get('evaluation_interval_decisions') != 100000:
        raise ValueError('Unexpected automatic validation interval')
    if cfg.get('reward_weights') != dict(REWARD_WEIGHTS, jerk=0.):
        raise ValueError('Base physical reward changed')
    if cfg.get('v06_reward_weights') != v06.WEIGHTS or cfg.get('v06_settings') != v06.SETTINGS:
        raise ValueError('V06 reward weights/settings differ from the audited values')
    if cfg.get('reward_version') != v06.REWARD_VERSION or cfg.get('observation_version') != v06.OBSERVATION_VERSION:
        raise ValueError('Reward/observation semantics require a reviewed version')
    if cfg.get('mask_version') != MASK_VERSION + '_headway_ttc_v07' or cfg.get('target_settings') != TARGET_SETTINGS:
        raise ValueError('Mask or target settings changed')
    if cfg.get('rate_limit') != 'full':
        raise ValueError('User decision (2026-11-15): whole-episode command rate limit')
    if cfg.get('controlled_car_follow_model') != 'Krauss' or cfg.get('road_end_leader_exit_rule') is not False:
        raise ValueError('User decisions: Krauss controlled vehicles, original road-end rules')
    if cfg.get('shared_cost_ledger') != 'outputs/highway-flow-rl/v02/cost-ledger.json' or cfg.get('budget') != BUDGET:
        raise ValueError('Declared v06 budget or ledger changed')
    if type(cfg.get('critic_warmup_decisions')) is not int or not 0 <= cfg['critic_warmup_decisions'] <= 30000:
        raise ValueError('Critic warmup must be bounded and counted inside the round')
    if cfg.get('warm_start') != WARM_START:
        raise ValueError('Expected frozen v04 best initialization')
    if cfg.get('acceptance') != ACCEPTANCE:
        raise ValueError('Acceptance must be the original rules plus the raw-request jerk check')
    stop = cfg.get('stop_rules', {})
    if stop.get('any_collision') is not True or not isinstance(stop.get('merge_speed_match_floor'), (int, float)):
        raise ValueError('Stop rules missing')
    number(stop['merge_speed_match_floor'], 'merge speed-match floor', 0., 1.)
    if type(cfg.get('seed')) is not int or cfg['seed'] < 0:
        raise ValueError('Invalid training seed')
    for pin in cfg.get('evaluation_initializers', []):
        artifact(pin)
    split_seeds = {}
    for split in ('training_rows', 'validation_rows', 'held_out_rows'):
        rows = cfg.get(split)
        if not isinstance(rows, list) or not rows:
            raise ValueError('Missing explicit ' + split)
        seeds, identities = set(), set()
        for row in rows:
            case = row.get('case', {})
            artifact(case, 'network', 'network_sha256')
            if case.get('task') not in ('cruise', 'following', 'passing', 'merge') or case.get('density') not in ('light', 'medium', 'dense'):
                raise ValueError('Unsupported task or density')
            if case['task'] == 'merge' and not (case['network'].startswith(RAMP_STARTED_PREFIX) or case['network'] == I110_NETWORK):
                raise ValueError('Merge rows use ramp-started layouts or I-110 (historical ramp layouts are reference only)')
            total, av = row.get('total_vehicles'), row.get('controlled_count')
            if type(total) is not int or total not in (8, 16, 24) or type(av) is not int or not 1 <= av <= total:
                raise ValueError('Unsupported population')
            if case['task'] in ('following', 'passing') and av == total:
                raise ValueError('Following/passing requires a native leader')
            if row.get('missed_gap', False) is not False:
                if row['missed_gap'] is not True or split != 'training_rows' or case['task'] != 'merge' or av != 1:
                    raise ValueError('Missed-gap demand is for one-AV merge training rows only')
            seed = row.get('seed')
            if type(seed) is not int or seed < 0 or (case.get('id'), seed) in identities:
                raise ValueError('Invalid or duplicate scenario identity')
            seeds.add(seed)
            identities.add((case.get('id'), seed))
            if type(row.get('horizon_steps')) is not int or not 100 <= row['horizon_steps'] <= 2400:
                raise ValueError('Explicit bounded full-route horizon required')
            if split == 'training_rows' and row.get('rollout_decisions') != row['horizon_steps'] * av:
                raise ValueError('Training must allocate complete episode horizons')
            number(row.get('speed_limit_m_s'), 'speed cap', .1, 26.8224)
            number(row.get('packet_loss'), 'packet loss', 0., 1.)
            if row.get('communication') not in ('local', 'none') or row.get('traffic_mode') != 'initial':
                raise ValueError('Only explicit fixed-cohort initial traffic is supported')
            if row.get('difficulty') not in (0, 1, 2) or type(row.get('difficulty')) is not int:
                raise ValueError('Invalid difficulty')
            if row.get('fixture') or row.get('fixture_path'):
                raise ValueError('Fixtures are diagnostic only')
        split_seeds[split] = seeds
    for a, b in (('training_rows', 'validation_rows'), ('training_rows', 'held_out_rows'), ('validation_rows', 'held_out_rows')):
        if split_seeds[a] & split_seeds[b]:
            raise ValueError('Seeds overlap between ' + a + ' and ' + b)
    from traffic.highway_merge_cutin_v07 import MIX, SETTINGS, VERSION
    if cfg.get('curriculum_mix') != MIX or cfg.get('lane_admission_settings') != SETTINGS or cfg.get('lane_admission_version') != VERSION:
        raise ValueError('Unreviewed curriculum or admission settings')
    if {r.get('curriculum_group') for r in cfg['training_rows']} != set(MIX):
        raise ValueError('Missing curriculum category')
    for r in cfg['training_rows']:
        group = r['curriculum_group']
        if (group in ('normal_merge', 'missed_gap', 'cutin_recovery')) != (r['case']['task'] == 'merge'):
            raise ValueError('Curriculum task mismatch')
        if bool(r.get('cutin_stress')) != (group == 'cutin_recovery'):
            raise ValueError('Stress rows must be labeled')
        if group in ('missed_gap', 'cutin_recovery') and not r.get('missed_gap'):
            raise ValueError('Missing missed-gap recovery demand')
    if cfg.get('stop_rules', {}).get('merge_speed_match_reference') != 'matched_initial_validation':
        raise ValueError('Merge stop rule requires a matched initial baseline')
    for flag in ('held_out_authorized', 'development_gates_passed'):
        if type(cfg.get(flag)) is not bool:
            raise ValueError('Missing final-evaluation gate: ' + flag)
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    if (proof.get('schema') != PREFLIGHT_SCHEMA or proof.get('passed') is not True
            or proof.get('training') is not False or proof.get('optimizer_steps') != 0
            or type(proof.get('optimizer_steps')) is not int
            or proof.get('config_sha256') != config_sha256 or proof.get('source_hashes') != source_hashes
            or any(proof.get('checks', {}).get(key) is not True for key in PREFLIGHT_CHECKS)):
        raise ValueError('V06 preflight is missing, stale, learned or failed')
    return proof
