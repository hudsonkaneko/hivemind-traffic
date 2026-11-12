"""Separate v05 preparation/launch contract; historical v01 budgets stay closed."""
import math
import re
from pathlib import Path

from traffic.highway_flow_rl_contract import ACTION_CONTRACT, OPTIMIZER, REWARD_WEIGHTS
from traffic.highway_flow_contract import file_digest
from traffic.highway_flow_rl_v05_rewards import WEIGHTS as DEFAULT_WEIGHTS, SETTINGS, REWARD_VERSION
from traffic.highway_flow_rl_v04_rewards import TARGET_SETTINGS, MASK_VERSION

SCHEMA = 'highway_flow_rl_v05'
ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT_CHECKS = ('api', 'shapes', 'native_guards', 'finite_gradient', 'export', 'reward_version', 'mask_version')
PINNED_CHECKPOINT = 'bab474716a4bdbeb650b7af4aa3eaccac1744b05c7926c9c56c200296ca48742'
PINNED_ACTOR = '888a380b7c6741aea623e9b99ad155f7cb9a2ef8fdc2e590823b7e2629031819'


def number(value, name, low=0., high=math.inf):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('Invalid '+name)
    return value


def artifact(spec, path_key='path', hash_key='sha256'):
    path = spec.get(path_key)
    if not isinstance(path, str) or Path(path).is_absolute() or not (ROOT/path).resolve().is_relative_to(ROOT):
        raise ValueError('Artifact must use a relative path inside the checkout')
    if not re.fullmatch('[0-9a-f]{64}', str(spec.get(hash_key, ''))):
        raise ValueError('Artifact needs a SHA256')


def validate_config(cfg, require_training=False):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Wrong v05 schema or missing explicit authorization state')
    if require_training and cfg['training_authorized'] is not True:
        raise ValueError('Training is disabled pending resolved-configuration review')
    if (cfg.get('observation_size') != 117 or cfg.get('state_size') != 2832 or cfg.get('max_agents') != 24
            or cfg.get('action_contract') != ACTION_CONTRACT):
        raise ValueError('Flow117 actor/action/critic ABI changed')
    if cfg.get('safety') != dict(speed_mode=31, lane_change_mode=512, native_guards_unchanged=True,
                                safety_shield_unchanged=True, global_speed_ceiling_m_s=26.8224):
        raise ValueError('Native or explicit safety settings changed')
    if cfg.get('learning_profile') != 'continuity_merge_pilot_300k_20epochs' or cfg.get('round_index') != 4:
        raise ValueError('Expected separately authorized v05 long profile')
    if cfg.get('optimizer') != dict(OPTIMIZER, epochs=20):
        raise ValueError('Declared MAPPO optimizer changed')
    if cfg.get('gae') != dict(gamma=.99, **{'lambda':.95}):
        raise ValueError('GAE and shaping discount must match the declared baseline')
    if cfg.get('evaluation_interval_decisions') != 100000:
        raise ValueError('Unexpected automatic validation interval')
    if cfg.get('reward_weights') != dict(REWARD_WEIGHTS, jerk=0.):
        raise ValueError('Base physical reward changed')
    if cfg.get('v05_reward_weights') != DEFAULT_WEIGHTS:
        raise ValueError('Declared v05 reward coefficients changed')
    if cfg.get('target_settings') != TARGET_SETTINGS:
        raise ValueError('Pre-action target settings changed')
    if cfg.get('shared_cost_ledger') != 'outputs/highway-flow-rl/v02/cost-ledger.json':
        raise ValueError('V05 must retain the existing v02 study costs')
    if cfg.get('mask_version') != MASK_VERSION:
        raise ValueError('Mask semantics require a separately reviewed version')
    if cfg.get('reward_version') != REWARD_VERSION:
        raise ValueError('Reward semantics require a separately reviewed version')
    if type(cfg.get('seed')) is not int or cfg['seed'] < 0:
        raise ValueError('Invalid training seed')
    if type(cfg.get('round_index')) is not int or cfg['round_index'] != 4:
        raise ValueError('This profile is round four')
    budget = cfg.get('budget', {})
    expected = dict(per_round_decisions=300000, total_decisions=4630000,
        per_round_wall_seconds=5400, total_wall_seconds=41400, final_reserve_seconds=900,
        inherited_spent_seconds=0, preparation_wall_seconds=3600, diagnostic_wall_seconds=2700,
        rollout_decisions=38400, max_diagnostic_episodes=60,
        conditional_second_round_authorized=False)
    if budget != expected:
        raise ValueError('Declared v05 budget changed')
    if cfg.get('critic_warmup_decisions') != 0:
        raise ValueError('This pilot reuses the existing critic without a new warmup budget')
    warm = cfg.get('warm_start', {})
    artifact(warm)
    if warm['path'] != 'outputs/highway-flow-rl/v04/long-round-03/best.pt':
        raise ValueError('Expected frozen v04 best initialization')
    if cfg.get('continuity_settings') != SETTINGS or cfg.get('variant') not in ('legacy', 'continuity', 'full', 'startup'):
        raise ValueError('Unreviewed continuity variant/settings')
    if require_training and cfg['variant'] != 'full':
        raise ValueError('Only the resolved full v05 configuration may train')
    if cfg.get('acceptance') != dict(max_intervention_fraction=.05,max_peak_jerk_m_s3=5.,collisions=0,teleports=0,overspeed_steps=0,report_per_car=True,full_route=True,min_headway_s=1.5,min_ttc_s=3.,max_optional_detour_events=0):
        raise ValueError('Historical acceptance thresholds changed')
    if len(cfg.get('evaluation_initializers', [])) != 2:
        raise ValueError('Both frozen v04 checkpoints are required')
    for pin in cfg['evaluation_initializers']: artifact(pin)
    split_seeds = {}
    for split in ('training_rows','validation_rows','held_out_rows'):
        rows = cfg.get(split)
        if not isinstance(rows,list) or not rows:
            raise ValueError('Missing explicit '+split)
        seeds = set(); identities = set()
        for row in rows:
            case = row.get('case', {})
            artifact(case,'network','network_sha256')
            if case.get('task') not in ('cruise','following','passing','merge') or case.get('density') not in ('light','medium','dense'):
                raise ValueError('Unsupported task or density')
            total, av = row.get('total_vehicles'), row.get('controlled_count')
            if type(total) is not int or total not in (8,16,24) or type(av) is not int or not 1<=av<=total:
                raise ValueError('Unsupported population')
            if case['task'] in ('following','passing') and av==total:
                raise ValueError('Following/passing requires a native leader')
            if type(case.get('through_lanes')) is not int or not 1<=case['through_lanes']<=7:
                raise ValueError('Invalid through-lane count')
            seed = row.get('seed')
            if type(seed) is not int or seed<0 or (case.get('id'),seed) in identities:
                raise ValueError('Invalid or duplicate scenario identity')
            seeds.add(seed); identities.add((case.get('id'),seed))
            if type(row.get('horizon_steps')) is not int or not 100<=row['horizon_steps']<=2400:
                raise ValueError('Explicit bounded full-route horizon required')
            if 'rollout_decisions' in row and (type(row['rollout_decisions']) is not int or not 64<=row['rollout_decisions']<=57600):
                raise ValueError('Per-case rollout allocation exceeds the declared curriculum bound')
            if split=='training_rows' and row.get('rollout_decisions') != row['horizon_steps']*av:
                raise ValueError('Training must allocate complete episode horizons')
            number(row.get('speed_limit_m_s'),'speed cap',.1,26.8224)
            number(row.get('packet_loss'),'packet loss',0.,1.)
            if row.get('communication') not in ('local','none') or row.get('traffic_mode')!='initial':
                raise ValueError('Only explicit fixed-cohort initial traffic is supported')
            if type(row.get('difficulty')) is not int or row['difficulty'] not in (0,1,2):
                raise ValueError('Invalid difficulty')
            if row.get('fixture_path'):
                artifact(row,'fixture_path','fixture_sha256')
            if split=='training_rows' and (row.get('fixture') or row.get('fixture_path')):
                raise ValueError('Fixed-seed counterexamples are diagnostic only')
        split_seeds[split] = seeds
    for a,b in [('training_rows','validation_rows'),('training_rows','held_out_rows'),('validation_rows','held_out_rows')]:
        if split_seeds[a] & split_seeds[b]:
            raise ValueError('Seeds overlap between '+a+' and '+b)
    if not any(r['case']['task']=='merge' and 'i110' in r['case']['id'].lower() for r in cfg['training_rows']):
        raise ValueError('I-110 merge must receive learning exposure')
    for flag in ('held_out_authorized','development_gates_passed'):
        if type(cfg.get(flag)) is not bool:
            raise ValueError('Missing final-evaluation gate: '+flag)
    return cfg


def validate_preflight(proof, config_sha256, source_hashes):
    if (proof.get('schema') != 'highway_flow_rl_preflight_v05' or proof.get('passed') is not True
            or proof.get('training') is not False or type(proof.get('optimizer_steps')) is not int
            or proof['optimizer_steps'] != 0 or proof.get('config_sha256') != config_sha256
            or proof.get('source_hashes') != source_hashes
            or any(proof.get('checks',{}).get(key) is not True for key in PREFLIGHT_CHECKS)):
        raise ValueError('V05 preflight is missing, stale, learned or failed')
    return proof
