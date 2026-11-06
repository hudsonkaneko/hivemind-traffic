"""Strict configuration, readiness and charged-budget contracts for paired runs."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from statistics import median

from experiments.highway_v05_support import validate_config as validate_v05
from traffic.highway_smooth_schedule import build_schedule, digest

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'highway_smooth_training_v01'
ARMS = ('unchanged_objective_control', 'anticipatory_alignment_comfort_bundle')
CHECKS = ('contract', 'safety', 'comfort', 'reference', 'invariance', 'gradient', 'coverage')
PARENT_CONFIG_PATH = 'policies/versions/highway-environment-v05/training-config.json'
PARENT_CONFIG_SHA = '17a28932f90c0ae64d875beab733c4d275de506487f7cad7b1a11909423121f9'
FROZEN_INITIALIZATION = dict(
    actor=dict(path='policies/versions/highway-environment-v05/policy.pt',
        sha256='fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec'),
    checkpoint=dict(path='outputs/highway-environment/v05/train-20261106T024828Z-90b25880/last.pt',
        sha256='9279b6103d13d2be3fd18fe05e9bae72462e581a16d881fc9e4ff87b0b2ac02d'))


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def number(value, name, *, low=0., high=math.inf, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            or not low <= value <= high or (integer and type(value) is not int)):
        raise ValueError('Invalid ' + name)
    return value


def project_path(value, *, outputs=False):
    path = (ROOT / value).resolve()
    if not path.is_relative_to(ROOT / 'outputs' if outputs else ROOT):
        raise ValueError('Path must remain inside the declared project scope')
    return path


def preparation_digest(cfg):
    return digest({key: value for key, value in cfg.items() if key not in ('training_authorized', 'readiness')})


def validate_config(cfg):
    from traffic.highway_smooth_reference import REFERENCE_VERSION, IMPLEMENTATION_NOTES
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Smooth paired schema/authorization mismatch')
    validate_v05(cfg['base_config'])
    if file_digest(ROOT / PARENT_CONFIG_PATH) != PARENT_CONFIG_SHA:
        raise ValueError('Frozen parent configuration checksum changed')
    parent = json.loads((ROOT / PARENT_CONFIG_PATH).read_text(encoding='utf-8'))
    for key in ('model', 'environment', 'reward_weights', 'optimizer', 'population', 'speed_curriculum'):
        if cfg['base_config'][key] != parent[key]:
            raise ValueError('Pinned runtime base changed: ' + key)
    if cfg.get('gae') != dict(gamma=.99, gae_lambda=.95):
        raise ValueError('Collector gamma/lambda must match the pinned implementation')
    if cfg.get('reference_version') != REFERENCE_VERSION or cfg.get('reference_implementation_notes') != list(IMPLEMENTATION_NOTES):
        raise ValueError('Resolved reference identity differs from implementation')
    if tuple(arm.get('id') for arm in cfg['arms']) != ARMS:
        raise ValueError('Exactly the reviewed two arms are required')
    if [arm['objective_version'] for arm in cfg['arms']] != ['v05', 'smooth_v01']:
        raise ValueError('Objective arm mismatch')
    if [arm['imitation_acceleration_weight'] for arm in cfg['arms']] != [1., 5.]:
        raise ValueError('Reviewed imitation weights changed')
    budget = cfg['budget']
    for key, ceiling in (('per_arm_decisions', 50000), ('total_decisions', 100000), ('wall_seconds', 3600), ('critic_warmup_per_arm', 2048)):
        number(budget[key], key, low=0 if key == 'critic_warmup_per_arm' else 1, high=ceiling, integer=True)
    if (budget['total_decisions'] != 2 * budget['per_arm_decisions']
            or budget['critic_warmup_per_arm'] >= budget['per_arm_decisions']
            or budget['final_evaluation_reserve_seconds'] != 900 or budget['wall_seconds'] <= 900):
        raise ValueError('Invalid shared budget or evaluation reserve')
    if cfg['optimizer'] != cfg['base_config']['optimizer']:
        raise ValueError('Common optimizer differs from frozen reviewed base')
    if cfg['base_config']['training_authorized'] is not False:
        raise ValueError('Embedded legacy configuration must stay disabled')
    if cfg['reference_parameters'] != dict(legal_speed_buffer_m_s=.1, entry_preview_time_s=2.,
            nominal_reference_jerk_m_s3=2., desired_speed_error_gain_s_inverse=.4):
        raise ValueError('Reviewed reference constants changed')
    for key, expected in (('feasible_acceleration', 1.), ('jerk', .03), ('command_reference_residual_slew', .01)):
        if cfg['reward_bundle'][key] != expected:
            raise ValueError('Reviewed reward bundle changed')
    if cfg['reward_bundle'] != dict(feasible_acceleration=1., jerk=.03, command_reference_residual_slew=.01,
            huber='H(x)=0.5*x^2 for abs(x)<=1; abs(x)-0.5 otherwise; uncapped',
            jerk_argument='realized jerk / (1 m/s^3)',
            residual_slew_argument='delta(requested acceleration - current reference) / (0.1 s * 1 m/s^3)'):
        raise ValueError('Reward formula metadata differs from implemented objective')
    if cfg.get('comparison') != comparison_settings():
        raise ValueError('Predeclared endpoint comparison changed')
    if cfg['output_directory'] != 'outputs/highway-smooth-control/v02':
        raise ValueError('Smooth outputs must be isolated')
    schedule = cfg['schedule']
    for key in ('seed', 'seed_start', 'seed_end'):
        number(schedule[key], key, integer=True)
    if schedule['seed_start'] != 7000000 or schedule['seed_end'] != 7000999:
        raise ValueError('Training seed bank differs from approved proposal')
    if cfg['schedule_sha256'] != digest(build_schedule(cfg)):
        raise ValueError('Immutable matched schedule changed')
    seeds = [row['seed'] for row in cfg['base_config']['validation_cases']]
    if seeds != list(range(6900000, 6900024)):
        raise ValueError('All 24 fixed validation rows are required')
    expected = 'ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836'
    for case in parent['training_cases'] + [r['case'] for r in parent['validation_cases']]:
        if 'i110' in case['id'].lower():
            case.update(network='outputs/i110-merge-blocking/v01/network-v02/network.net.xml', network_sha256=expected)
    if any(cfg['base_config'][key] != parent[key] for key in ('training_cases', 'validation_cases')):
        raise ValueError('Frozen case bank changed beyond corrected I-110 network substitution')
    for case in cfg['base_config']['training_cases'] + [r['case'] for r in cfg['base_config']['validation_cases']]:
        if 'i110' in case['id'].lower() and case['network_sha256'] != expected:
            raise ValueError('Every I-110 case requires the corrected network')
    for key in ('actor', 'checkpoint'):
        spec = cfg['initialization'][key]
        project_path(spec['path'])
        if spec != FROZEN_INITIALIZATION[key]:
            raise ValueError('Frozen input identity changed: ' + key)
    if cfg['initialization']['seed'] != 9507 or cfg['provenance']['parent_config'] != PARENT_CONFIG_PATH or cfg['provenance']['parent_config_sha256'] != PARENT_CONFIG_SHA:
        raise ValueError('Reviewed initialization seed or parent identity changed')
    project_path(cfg['preparation_ledger'], outputs=True)
    return cfg


def comparison_settings():
    return dict(median_paired_p95_improvement_min=.1, task_mean_jerk_regression_max=.05,
        task_mean_speed_ratio_drop_max=.02, nominal_approach_peak_max_m_s3=5.,
        nominal_approach_rows=[dict(index=i, seed=6900000+i) for i in (3, 7, 19)],
        nominal_selection='Every assigned controlled merge target, complete approach phase including startup; no retrospective exclusions',
        nominal_merge_stop=dict(threshold_speed_m_s=.1, reject_consecutive_stop_at_least_s=2.,
            window='From first reset observation through successful stable-merge completion, including startup',
            coverage='Complete consecutive measured intervals through stable completion required; missing/pending is not a pass',
            criterion_history='Added after inspection of v2 and before v3 evaluation to enforce continuous nominal merging; not a claim that v2 failed the earlier numeric gates'),
        realized_jerk_scope='Assigned controlled vehicles through the entire 80 s accounting horizon, including native tail; background/all retained in full reports',
        task_aggregation='Equal case means within each task; paired p95 improvement uses all 24 cases; zero/zero is zero improvement',
        no_regression_totals=['stable_merges', 'passes', 'completed_trips'])


def compare_endpoints(cfg, control, treatment):
    """Pure all-row proposal gates, separate from held-out acceptance."""
    from experiments.highway_v05_support import acceptance
    settings, rows = cfg['comparison'], cfg['base_config']['validation_cases']
    if len(control) != 24 or len(treatment) != 24:
        raise ValueError('Endpoint comparison requires all 24 rows for both arms')
    ordinary = dict(control=acceptance(control), treatment=acceptance(treatment))
    paired, tasks, nominal, stops = [], {}, [], []
    totals = {label: dict(stable_merges=0, passes=0, completed_trips=0) for label in ('control', 'treatment')}
    for index, (row, before, after) in enumerate(zip(rows, control, treatment)):
        values = []
        for label, report in (('control', before), ('treatment', after)):
            if report['seed'] != row['seed'] or report['case_id'] != row['case']['id'] or report['task'] != row['case']['task']:
                raise ValueError('Endpoint order/case/seed mismatch')
            jerk = report['smooth_control']['controlled']['jerk_m_s3']['realized']
            number(jerk['samples'], 'realized jerk samples', low=1, integer=True)
            values.append((number(jerk['p95_abs'], 'jerk p95'), number(jerk['mean_abs'], 'mean jerk'),
                number(report['mean_fraction_of_speed_limit'], 'speed ratio')))
            stable = number(report['stable_merge']['stable'], 'stable merges', integer=True)
            for key, value in (('stable_merges', stable), ('passes', report['passes']), ('completed_trips', report['completed_trips'])):
                totals[label][key] += number(value, key, integer=True)
            if any(item['index'] == index for item in settings['nominal_approach_rows']):
                targets = report['merge_target_ids']
                if not targets:
                    raise ValueError('Nominal approach lacks controlled merge targets')
                for vehicle in targets:
                    car = report['smooth_control']['per_car'][vehicle]
                    if car['assigned_controlled'] is not True:
                        raise ValueError('Nominal target is not assigned controlled')
                    approach = car['phases']['approach']['jerk_m_s3']['realized']
                    number(approach['samples'], 'approach samples', low=1, integer=True)
                    peak = number(approach['peak_abs'], 'approach peak')
                    nominal.append(dict(arm=label, index=index, seed=row['seed'], vehicle=vehicle,
                        peak_m_s3=peak, passed=peak <= settings['nominal_approach_peak_max_m_s3']))
                    stop = car['nominal_merge_stop']
                    if (type(stop.get('coverage_complete')) is not bool
                            or stop.get('threshold_speed_m_s') != settings['nominal_merge_stop']['threshold_speed_m_s']):
                        raise ValueError('Nominal merge stop definition/coverage is missing or changed')
                    longest = number(stop.get('longest_consecutive_stop_s'), 'longest nominal merge stop')
                    if stop['coverage_complete']:
                        stable_time = number(stop.get('stable_completion_time_s'), 'stable completion time')
                        start = number(stop.get('observed_start_time_s'), 'nominal observed window start')
                        end = number(stop.get('observed_end_time_s'), 'nominal observed window end')
                        if not start <= stable_time <= end + 1e-8:
                            raise ValueError('Nominal stop window does not cover stable completion')
                    stop_record = deepcopy(stop)
                    stop_record.update(arm=label, index=index, seed=row['seed'], vehicle=vehicle,
                        passed=stop['coverage_complete'] and longest <
                        settings['nominal_merge_stop']['reject_consecutive_stop_at_least_s']-1e-8)
                    stops.append(stop_record)
        (p0, m0, s0), (p1, m1, s1) = values
        improvement = (p0-p1)/p0 if p0 else (0. if p1 == 0 else None)
        paired.append(dict(index=index, seed=row['seed'], control_p95=p0, treatment_p95=p1,
            relative_improvement=improvement))
        tasks.setdefault(row['case']['task'], []).append((m0, m1, s0, s1))
    improvements = [r['relative_improvement'] for r in paired]
    med = median(improvements) if all(value is not None for value in improvements) else None
    task_results = {}
    for task, values in tasks.items():
        m0, m1, s0, s1 = [sum(row[i] for row in values)/len(values) for i in range(4)]
        task_results[task] = dict(control_mean_jerk=m0, treatment_mean_jerk=m1,
            control_speed_ratio=s0, treatment_speed_ratio=s1,
            jerk_passed=m1 <= m0*(1+settings['task_mean_jerk_regression_max'])+1e-12,
            speed_passed=s1 >= s0-settings['task_mean_speed_ratio_drop_max']-1e-12)
    checks = dict(paired_full_horizon_accounting=all(
            report.get('primary_horizon_complete') is True
            and report.get('accounting_comparison_valid') is True
            and report.get('partial_control_horizon') is False
            for report in (*control, *treatment)),
        ordinary_safety_and_task_gates=ordinary['treatment']['all_gates_passed'],
        median_paired_p95=med is not None and med >= settings['median_paired_p95_improvement_min']-1e-12,
        task_mean_jerk=all(value['jerk_passed'] for value in task_results.values()),
        task_speed=all(value['speed_passed'] for value in task_results.values()),
        nominal_approach_peak=all(row['passed'] for row in nominal if row['arm'] == 'treatment'),
        nominal_merge_no_prolonged_stop=all(row['passed'] for row in stops if row['arm'] == 'treatment'),
        total_task_outcomes=all(totals['treatment'][key] >= totals['control'][key] for key in totals['control']))
    return dict(proposal_gates_passed=all(checks.values()), checks=checks, accepted=False, held_out_test_run=False,
        ordinary=ordinary, median_paired_p95_relative_improvement=med, paired_cases=paired,
        tasks=task_results, nominal_approach=nominal, nominal_merge_stops=stops,
        totals=totals, settings=deepcopy(settings))


def readiness_check(cfg, source_hashes):
    ready = cfg.get('readiness')
    if not isinstance(ready, dict) or not ready:
        raise ValueError('Fresh passed zero-update readiness is required')
    path = project_path(ready.get('summary_path', ''), outputs=True)
    if not path.is_file() or file_digest(path) != ready.get('summary_sha256'):
        raise ValueError('Readiness proof checksum/path mismatch')
    proof = json.loads(path.read_text(encoding='utf-8'))
    if (proof.get('schema') != 'highway_smooth_preflight_v01' or proof.get('passed') is not True
            or proof.get('training') is not False or type(proof.get('optimizer_steps')) is not int
            or proof['optimizer_steps'] != 0 or proof.get('source_hashes') != source_hashes
            or proof.get('preparation_sha256') != preparation_digest(cfg)
            or any(proof.get('checks', {}).get(key) is not True for key in CHECKS)):
        raise ValueError('Readiness is incomplete, learned, failed or stale')
    ledger_path = project_path(cfg['preparation_ledger'], outputs=True)
    if not ledger_path.is_file() or file_digest(ledger_path) != proof.get('preparation_ledger_sha256'):
        raise ValueError('Preparation ledger changed or is missing')
    ledger = json.loads(ledger_path.read_text(encoding='utf-8'))
    if ledger.get('schema') != 'smooth_preparation_ledger_v01' or not ledger.get('attempts'):
        raise ValueError('Invalid preparation cost ledger')
    total = 0.
    for attempt in ledger['attempts']:
        if attempt.get('status') not in ('complete', 'completed', 'failed'):
            raise ValueError('Preparation ledger still has an unfinished attempt')
        total += number(attempt.get('charged_wall_seconds'), 'preparation cost')
    if not math.isclose(total, number(proof.get('preparation_wall_seconds'), 'proof preparation cost'), abs_tol=1e-6):
        raise ValueError('Preparation charges do not match the ledger')
    if total >= cfg['budget']['wall_seconds'] - 900:
        raise ValueError('Preparation exhausted learning allowance; no automatic extension')
    return dict(proof, charged_preparation_wall_seconds=total)


def require_training(cfg, approved_config_sha256, source_hashes):
    validate_config(cfg)
    if cfg['training_authorized'] is not True or approved_config_sha256 != digest(cfg):
        raise ValueError('Training requires explicit authorization and exact reviewed config checksum')
    return readiness_check(cfg, source_hashes)


def new_arm_state():
    return dict(spent=0, actual=0, critic_spent=0, ppo_spent=0, critic_actual=0, ppo_actual=0,
        schedule_index=0, episodes=0, updates=0, optimizer_steps=0, critic_optimizer_steps=0,
        critic_closed=False, learning_seconds=0., pending=None, failures=[],
        coverage={}, task_decisions=dict(cruise=0, following=0, passing=0, merge=0),
        difficulty_decisions={'0': 0, '1': 0, '2': 0}, dense_following_decisions=0)


def reserve_episode(cfg, state, row, *, critic_only):
    phase = 'critic' if critic_only else 'ppo'
    ceiling = cfg['budget']['critic_warmup_per_arm'] if critic_only else cfg['budget']['per_arm_decisions'] - cfg['budget']['critic_warmup_per_arm']
    remaining = min(ceiling - state[phase + '_spent'], cfg['budget']['per_arm_decisions'] - state['spent'])
    count = min(remaining, cfg['base_config']['environment']['horizon'] * row['controlled_count'])
    if count < row['controlled_count']:
        return 0
    state['spent'] += count
    state[phase + '_spent'] += count
    state['episodes'] += 1
    state['pending'] = dict(kind='episode', reserved=count, critic_only=critic_only, row=deepcopy(row))
    return count


def complete_episode(state, actual):
    pending = state['pending']
    if not pending or pending['kind'] != 'episode':
        raise ValueError('No reserved episode')
    number(actual, 'collected decisions', integer=True, high=pending['reserved'])
    phase = 'critic' if pending['critic_only'] else 'ppo'
    state['spent'] -= pending['reserved'] - actual
    state[phase + '_spent'] -= pending['reserved'] - actual
    state['actual'] += actual
    state[phase + '_actual'] += actual


def elapsed_on_resume(state, saved_at_unix, now_unix):
    elapsed = number(state['elapsed_seconds'], 'charged elapsed')
    # Unclean process loss charges all time since last durable generation.
    return elapsed + (max(0., now_unix - saved_at_unix) if not state['run_closed'] else 0.)


def recover_pending(arm):
    pending = arm.get('pending')
    if pending and pending['kind'] in ('episode', 'updating'):
        arm['failures'].append(dict(kind=pending['kind'], schedule_index=pending['row']['matched_schedule']['index'],
            reserved_decisions=pending.get('reserved'),
            observed_failed_decisions=pending.get('observed_failed_decisions'),
            reason='Interrupted episode stays charged; an uncommitted update is rolled back and never replayed',
            uncommitted_optimizer_steps_unknown=pending['kind'] == 'updating'))
        arm['pending'] = None
    return arm
