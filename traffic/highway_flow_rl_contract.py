"""Reference-free development pilot contracts; never a v01 teacher certificate."""
from copy import deepcopy
import math

from traffic.highway_flow_contract import INITIALIZERS, file_digest

SCHEMA = 'highway_flow_rl_v01'
INITIALIZER_ID = 'v05_initializer'
INHERITED_SPENT_SECONDS = 1018.6849924
TOTAL_WALL_SECONDS = 5400.
FINAL_RESERVE_SECONDS = 900.
ACTION_CONTRACT = dict(acceleration=[-1., 1.], lane_intent=[-1, 0, 1])
PREFLIGHT_CHECKS = ('api', 'shapes', 'native_guards', 'finite_gradient')
REVISION_METRICS = ('max_peak_jerk_m_s3', 'pooled_intervention_fraction', 'collisions')
REWARD_WEIGHTS = dict(progress=1., merge=5., passing=2., arrival=2., collision=20.,
    intervention=.2, jerk=.01, lane_change=.1, feasible_acceleration=0.,
    unsafe_request=.5, cooperation=.1)
OPTIMIZER = dict(actor_lr=1e-4, critic_lr=3e-4, clip=.2, value_clip=.2,
    entropy_coef=.001, value_coef=.5, max_grad_norm=.5, epochs=4,
    minibatch_size=256, target_kl=.02, imitation_coef=0.,
    imitation_lane_change_weight=10., gradient_diagnostics=True,
    imitation_acceleration_weight=1., near_limit_coef=0., near_limit_band_m_s=1.)


def _number(value, name, *, low=0., high=math.inf):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError('Invalid ' + name)
    return value


def _validate_rows(cfg):
    seeds = set()
    for key in ('training_rows', 'validation_rows'):
        if key not in cfg:
            continue
        rows = cfg[key]
        if not isinstance(rows, list) or not rows:
            raise ValueError(key + ' must contain explicit cases')
        for row in rows:
            if not isinstance(row, dict) or not isinstance(row.get('case'), dict):
                raise ValueError('Every runtime row needs its case')
            total, controlled = row.get('total_vehicles'), row.get('controlled_count')
            task, seed = row['case'].get('task'), row.get('seed')
            if type(total) is not int or total not in (8, 16, 24) or type(controlled) is not int or not 1 <= controlled <= total:
                raise ValueError('Population requires 8/16/24 vehicles and 1..total controlled')
            if task not in ('cruise', 'following', 'passing', 'merge') or (task in ('following', 'passing') and controlled == total):
                raise ValueError('Unknown task or missing native leader')
            if row.get('communication') not in ('none', 'local') or type(row.get('difficulty')) is not int or row['difficulty'] not in (0, 1, 2):
                raise ValueError('Unsupported communication or difficulty')
            _number(row.get('packet_loss'), 'packet loss', high=1.)
            if _number(row.get('speed_limit_m_s'), 'speed ceiling', high=26.8224) <= 0:
                raise ValueError('Speed ceiling must be positive')
            if type(seed) is not int or seed < 0 or seed in seeds:
                raise ValueError('Runtime seeds must be distinct nonnegative integers across both splits')
            seeds.add(seed)
        if key == 'training_rows' and not any(r['case']['task'] == 'merge' and 'i110' in str(r['case'].get('id', '')).lower() for r in rows):
            raise ValueError('Training requires an explicit I110 merge case')


def validate_config(cfg):
    """Validate declared settings; this is not a runtime readiness assertion."""
    if cfg.get('schema') != SCHEMA or cfg.get('training_authorized') is not True:
        raise ValueError('Reference-free pilot schema/authorization mismatch')
    if (cfg.get('initializer_id') != INITIALIZER_ID
            or cfg.get('initializer') != INITIALIZERS[INITIALIZER_ID]):
        raise ValueError('Predeclared frozen initializer changed')
    if (cfg.get('observation_size') != 117 or cfg.get('state_size') != 2832
            or cfg.get('action_contract') != ACTION_CONTRACT):
        raise ValueError('Actor/action/centralized critic contract changed')
    safety = cfg.get('safety', {})
    if any(safety.get(key) is not True for key in ('native_guards_unchanged', 'safety_shield_unchanged')):
        raise ValueError('Native safety and shield must remain unchanged')
    optimizer, rewards = cfg.get('optimizer', {}), cfg.get('reward_weights', {})
    if optimizer.get('actor_lr') not in (1e-4, 5e-5):
        raise ValueError('Only the declared actor learning rates are permitted')
    if optimizer != dict(OPTIMIZER, actor_lr=optimizer['actor_lr']):
        raise ValueError('Pure MAPPO optimizer changed or auxiliary targets enabled')
    if rewards.get('jerk') not in (.01, .03) or rewards != dict(REWARD_WEIGHTS, jerk=rewards.get('jerk')):
        raise ValueError('Only the declared jerk revision is permitted; no teacher reward')
    round_index = cfg.get('round_index', 1)
    if type(round_index) is not int or round_index not in (1, 2):
        raise ValueError('Only two rounds are authorized')
    if round_index == 1 and (optimizer['actor_lr'] != 1e-4 or rewards['jerk'] != .01):
        raise ValueError('Round one must use the initial objective and learning rate')
    budget = cfg.get('budget', {})
    fixed = dict(per_round_decisions=30000, total_decisions=60000,
        inherited_spent_seconds=INHERITED_SPENT_SECONDS,
        total_wall_seconds=TOTAL_WALL_SECONDS, final_reserve_seconds=FINAL_RESERVE_SECONDS)
    for key, value in fixed.items():
        actual = _number(budget.get(key), key)
        if actual != value:
            raise ValueError('Shared budget or inherited cost changed: ' + key)
    _number(budget.get('per_round_wall_seconds'), 'round wall bound', low=.001, high=1500.)
    _validate_rows(cfg)
    return cfg


def budget_remaining(history_elapsed, spent=0., *, total_wall_seconds=TOTAL_WALL_SECONDS,
                     final_reserve_seconds=FINAL_RESERVE_SECONDS):
    """Seconds left before the reserve; spent includes every newly timed phase.

    The caller must persist cumulative failed/revised segments in history and
    measure the active segment monotonically. Final evaluation may use reserve=0.
    """
    history = _number(history_elapsed, 'charged history', low=INHERITED_SPENT_SECONDS - 1e-6)
    current = _number(spent, 'current segment elapsed')
    cap = _number(total_wall_seconds, 'total wall ceiling', high=TOTAL_WALL_SECONDS)
    reserve = _number(final_reserve_seconds, 'final reserve', high=FINAL_RESERVE_SECONDS)
    if reserve not in (0., FINAL_RESERVE_SECONDS):
        raise ValueError('Only final evaluation may consume the full reserve')
    return max(0., cap - history - current - reserve)


def validate_preflight(proof, config_sha256, source_hashes):
    """Check measured zero-update evidence for this pilot, not teacher quality."""
    if (proof.get('schema') != 'highway_flow_rl_preflight_v01'
            or proof.get('passed') is not True or proof.get('training') is not False
            or type(proof.get('optimizer_steps')) is not int or proof['optimizer_steps'] != 0
            or proof.get('config_sha256') != config_sha256
            or proof.get('source_hashes') != source_hashes
            or any(proof.get('checks', {}).get(key) is not True for key in PREFLIGHT_CHECKS)):
        raise ValueError('Reference-free runtime preflight is missing, failed, learned or stale')
    return proof


def validate_revision(previous, current, report):
    """Allow the declared second-round changes only after terminal measurements."""
    validate_config(previous)
    validate_config(current)
    if previous.get('round_index', 1) != 1 or current.get('round_index', 1) != 2:
        raise ValueError('Revision must follow round one into round two')
    if report.get('completed') is not True and report.get('failed') is not True:
        raise ValueError('Inspect a completed or failed round before revision')
    for key in REVISION_METRICS:
        _number(report.get(key), 'measured ' + key, high=1. if key == 'pooled_intervention_fraction' else math.inf)
    before, after = deepcopy(previous), deepcopy(current)
    for value in (before, after):
        for key in ('round_index', 'preflight', 'warm_start', 'revision'):
            value.pop(key, None)
    after['optimizer']['actor_lr'] = before['optimizer']['actor_lr']
    after['reward_weights']['jerk'] = before['reward_weights']['jerk']
    if before != after:
        raise ValueError('Revision changed undeclared settings or reset the budget')
    if current['reward_weights']['jerk'] != previous['reward_weights']['jerk']:
        if report['max_peak_jerk_m_s3'] <= 5.:
            raise ValueError('Higher jerk penalty requires measured high jerk')
    if current['optimizer']['actor_lr'] != previous['optimizer']['actor_lr']:
        kl = _number(report.get('approximate_kl', 0.), 'measured KL')
        initial = _number(report.get('initial_pooled_intervention_fraction'), 'initial overrides', high=1.)
        if kl <= OPTIMIZER['target_kl'] and report['pooled_intervention_fraction'] <= initial:
            raise ValueError('Lower learning rate requires high KL or worsened interventions')
    return current
