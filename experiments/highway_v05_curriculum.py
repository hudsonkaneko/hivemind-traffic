"""Resumable train-only competence curriculum and fresh scenario retries.

The state is plain JSON. Hash-derived independent permutations require no global
RNG state, and retry rows contain scenario settings only, never PPO transitions.
The caller charges every newly collected decision against its existing budget.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math


SCHEMA = 'highway_competence_curriculum_v05'
TASKS = ('cruise', 'following', 'passing', 'merge')
CURRICULUM_DEFAULTS = dict(
    recent_reports=32, minimum_reports_per_task=2, grade_success_fraction=.9,
    maximum_intervention_fraction=.05, cruise_speed_ratio_min=.9,
    following_speed_ratio_min=.25, stable_merge_fraction_min=.9,
    failed_training_resample_fraction=.2, failed_case_capacity=128,
    task_weights=[dict(cruise=4, following=4, passing=1, merge=1),
                  dict(cruise=3, following=3, passing=3, merge=2),
                  dict(cruise=2, following=2, passing=3, merge=5)])


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _permuted(values, index, seed, stream):
    """One independent balanced block per dimension, reproducible across resumes."""
    block, offset = divmod(index, len(values))
    order = sorted(range(len(values)), key=lambda i: _hash([seed, stream, block, i]))
    return deepcopy(values[order[offset]])


def _number(value, name, *, low=0., high=math.inf, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (integer and int(value) != value)):
        raise ValueError(f'Invalid {name}')
    return value


def _settings(cfg):
    supplied = cfg.get('curriculum', {})
    settings = deepcopy(CURRICULUM_DEFAULTS)
    settings.update(supplied)
    for key in ('recent_reports', 'minimum_reports_per_task', 'failed_case_capacity'):
        _number(settings[key], key, low=1, integer=True)
    for key in ('grade_success_fraction', 'maximum_intervention_fraction', 'cruise_speed_ratio_min',
                'following_speed_ratio_min', 'stable_merge_fraction_min'):
        _number(settings[key], key, low=0, high=1)
    _number(settings['failed_training_resample_fraction'], 'retry fraction', low=0, high=.2)
    weights = settings['task_weights']
    if len(weights) != 3:
        raise ValueError('Exactly three curriculum grades are required')
    for grade in weights:
        if set(grade) != set(TASKS):
            raise ValueError('Every grade must retain all driving tasks')
        for value in grade.values():
            _number(value, 'task weight', low=1, integer=True)
    return settings


def _domain(cfg):
    population, training = cfg['population'], cfg['training']
    totals = population.get('total_vehicles', [8, 16, 24])
    fractions = population.get('training_fractions', [.125, .25, .5, .75, 1.])
    channels = population.get('channels', [dict(communication='none', packet_loss=0.),
        dict(communication='local', packet_loss=0.), dict(communication='local', packet_loss=.3)])
    speeds = cfg.get('speed_curriculum', {}).get('ceilings_m_s', [12., 18., 22.352, 26.8224])
    if totals != [8, 16, 24] or fractions != [.125, .25, .5, .75, 1.]:
        raise ValueError('V05 needs independent totals 8/16/24 and five nonzero AV fractions')
    if channels != [dict(communication='none', packet_loss=0.), dict(communication='local', packet_loss=0.), dict(communication='local', packet_loss=.3)]:
        raise ValueError('V05 communication domain mismatch')
    if sorted(speeds) != [12., 18., 22.352, 26.8224]:
        raise ValueError('V05 requires all four speed ceilings')
    if cfg.get('model', {}).get('num_agents', 24) != 24:
        raise ValueError('V05 requires 24 controlled-agent capacity')
    bank = cfg['training_cases']
    if not bank or any(c.get('split') != 'train' for c in bank) or {c['task'] for c in bank} != set(TASKS):
        raise ValueError('Curriculum may only use a complete training case bank')
    ids = [c['id'] for c in bank]
    if len(ids) != len(set(ids)):
        raise ValueError('Training case IDs must be unique')
    for key in ('seed', 'seed_start', 'seed_end'):
        _number(training[key], key, integer=True)
    if training['seed_end'] < training['seed_start']:
        raise ValueError('Invalid training seed allocation')
    if any(training['seed_start'] <= r['seed'] <= training['seed_end'] for r in cfg.get('validation_cases', [])):
        raise ValueError('Training and validation seeds overlap')
    return dict(totals=totals, fractions=fractions, channels=channels, speeds=speeds,
                bank=bank, seed=training['seed'], seed_start=training['seed_start'],
                seed_end=training['seed_end'], traffic_mode=cfg.get('environment', {}).get('traffic_mode', 'initial'))


def _initialize(cfg, state):
    settings, domain = _settings(cfg), _domain(cfg)
    fingerprint = _hash(dict(settings=settings, domain=domain))
    if not state:
        state.update(schema=SCHEMA, config_sha256=fingerprint, settings=settings,
            deterministic_construction='sha256_independent_block_permutations_v1',
            grade=0, ordinary_index=0, rollouts_issued=0, retry_rollouts=0,
            case_indices={task: 0 for task in TASKS}, recent=[], failed_cases=[],
            issued={}, observed_rollout_ids=[], grade_history=[], actual_agent_decisions=0,
            seed_start=domain['seed_start'], seed_end=domain['seed_end'])
    if state.get('schema') != SCHEMA or state.get('config_sha256') != fingerprint:
        raise ValueError('Curriculum resume configuration differs from saved state')
    return settings, domain


def next_case(cfg, state):
    """Mutate resumable ``state`` and return one fresh training episode row.

    Persist the mutated state with the runner's pending episode reservation before
    executing the row. A retry retains its original training seed/settings, but
    receives a new rollout ID and must be simulated and budgeted again.
    """
    settings, domain = _initialize(cfg, state)
    issued = state['rollouts_issued']
    retry_allowed = state['retry_rollouts'] + 1 <= settings['failed_training_resample_fraction'] * (issued + 1) + 1e-12
    if state['failed_cases'] and retry_allowed:
        source = _permuted(state['failed_cases'], state['retry_rollouts'], domain['seed'], 'failed_training_scenarios')
        row = deepcopy(source['row'])
        row.pop('curriculum', None)
        source_rollout = source['source_rollout_id']
        state['retry_rollouts'] += 1
        retry = True
    else:
        index, grade = state['ordinary_index'], state['grade']
        seed = domain['seed_start'] + index
        if seed > domain['seed_end']:
            raise ValueError('Training seed allocation exhausted')
        weighted = [task for task in TASKS for _ in range(settings['task_weights'][grade][task])]
        task = ('cruise', 'following')[index] if index < 2 else _permuted(weighted, index, domain['seed'], f'task_grade_{grade}')
        bank = [case for case in domain['bank'] if case['task'] == task]
        case = _permuted(bank, state['case_indices'][task], domain['seed'], 'geometry_' + task)
        state['case_indices'][task] += 1
        total = _permuted(domain['totals'], index, domain['seed'], 'total_population')
        eligible = [fraction for fraction in domain['fractions'] if task not in ('following', 'passing') or fraction < 1]
        fraction = _permuted(eligible, index, domain['seed'], 'controlled_fraction_' + ('scripted_leader' if task in ('following', 'passing') else 'all'))
        count = round(total * fraction)
        if not 1 <= count <= total <= 24 or count / total != fraction:
            raise ValueError('Population fraction is not representable')
        channel = _permuted(domain['channels'], index, domain['seed'], 'channel')
        row = dict(case=case, controlled_count=count, total_vehicles=total, difficulty=grade,
                   requested_av_fraction=fraction, speed_limit_m_s=_permuted(domain['speeds'], index, domain['seed'], 'speed'),
                   traffic_mode='initial' if fraction == 1 else domain['traffic_mode'], seed=seed, **channel)
        state['ordinary_index'] += 1
        retry, source_rollout = False, None
    row['curriculum'] = dict(rollout_id=issued, grade=state['grade'], retry=retry,
                             source_rollout_id=source_rollout, fresh_rollout_required=True,
                             decision_budget='charge all newly collected active-agent decisions')
    state['rollouts_issued'] += 1
    state['issued'][str(issued)] = _hash(row)
    return row


def _gate(report, row, settings):
    """Missing safety evidence fails closed; completed reports alone can advance."""
    failures = []
    for field in ('collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed'):
        value = report.get(field)
        if value is None:
            failures.append('missing_' + field)
        elif _number(value, field, integer=True) > 0:
            failures.append(field)
    native = report.get('native_emergencies', {})
    if native.get('available') is not True or native.get('unique_events') is None:
        failures.append('missing_native_emergencies')
    elif _number(native['unique_events'], 'native emergency events', integer=True) > 0:
        failures.append('native_emergencies')
    fraction = report.get('intervention_fraction')
    if fraction is None:
        failures.append('missing_intervention_fraction')
    elif _number(fraction, 'intervention fraction', high=1) > settings['maximum_intervention_fraction']:
        failures.append('intervention_fraction')
    task = row['case']['task']
    if task in ('cruise', 'following'):
        speed = report.get('mean_fraction_of_speed_limit')
        minimum = settings['cruise_speed_ratio_min' if task == 'cruise' else 'following_speed_ratio_min']
        if speed is None or _number(speed, 'mean speed ratio') < minimum:
            failures.append('speed_progress')
    if task == 'passing' and (report.get('passes') is None or _number(report['passes'], 'passes', integer=True) < 1):
        failures.append('passing')
    if task == 'merge':
        merge = report.get('stable_merge', {})
        targets, stable = merge.get('targets'), merge.get('stable')
        if targets is None or stable is None:
            failures.append('missing_stable_merge')
        else:
            _number(targets, 'merge targets', integer=True)
            _number(stable, 'stable merges', high=targets, integer=True)
            if targets < 1 or stable / targets < settings['stable_merge_fraction_min']:
                failures.append('stable_merge')
    return failures


def _grade(recent, settings):
    groups = {task: [r for r in recent if r['task'] == task] for task in TASKS}
    def passed(tasks):
        return all(len(groups[task]) >= settings['minimum_reports_per_task'] and
                   sum(row['passed'] for row in groups[task]) / len(groups[task]) >= settings['grade_success_fraction']
                   for task in tasks)
    return 2 if passed(TASKS) else 1 if passed(('cruise', 'following')) else 0


def observe(state, report, row):
    """Update only from complete learned-training reports after a PPO update.

    The runner sets ``curriculum_training=True`` only after optimizing the freshly
    collected training rollout. Partial horizons, critic warmup, teachers, native
    controls, and validation reports cannot advance grades or populate retries.
    """
    if state.get('schema') != SCHEMA:
        raise ValueError('Initialize curriculum with next_case before observing')
    if row.get('case', {}).get('split') != 'train':
        raise ValueError('Validation/test scenarios may never enter the curriculum')
    if report.get('curriculum_training') is not True:
        return dict(eligible=False, reason='not_learned_training', grade=state['grade'])
    if report.get('controller') in ('native', 'teacher', 'random') or report.get('kind') in ('native', 'teacher', 'random'):
        raise ValueError('Only learned training evidence may update curriculum')
    complete = (report.get('partial_control_horizon') is False
                and report.get('primary_horizon_complete') is True
                and report.get('accounting_comparison_valid') is True)
    if not complete:
        return dict(eligible=False, reason='incomplete_report', grade=state['grade'])
    rollout_id = row.get('curriculum', {}).get('rollout_id')
    if type(rollout_id) is not int or state['issued'].get(str(rollout_id)) != _hash(row):
        raise ValueError('Report scenario does not match an issued training rollout')
    if rollout_id in state['observed_rollout_ids']:
        raise ValueError('Training rollout evidence was already observed')
    if report.get('seed', row['seed']) != row['seed'] or report.get('case_id', row['case']['id']) != row['case']['id']:
        raise ValueError('Report identity differs from training scenario')
    decisions = _number(report.get('agent_transitions', report.get('agent_steps', 0)), 'active-agent decisions', low=1, integer=True)
    settings = state['settings']
    failures = _gate(report, row, settings)
    evidence = dict(rollout_id=rollout_id, case_id=row['case']['id'], seed=row['seed'],
                    task=row['case']['task'], passed=not failures, failures=failures,
                    fresh_agent_decisions=decisions)
    state['recent'] = (state['recent'] + [evidence])[-settings['recent_reports']:]
    state['observed_rollout_ids'].append(rollout_id)
    state['actual_agent_decisions'] += decisions
    scenario = deepcopy(row)
    scenario.pop('curriculum', None)
    key = _hash(scenario)
    state['failed_cases'] = [item for item in state['failed_cases'] if item['scenario_sha256'] != key]
    if failures:
        state['failed_cases'].append(dict(row=scenario, scenario_sha256=key,
            source_rollout_id=rollout_id, failures=failures))
        state['failed_cases'] = state['failed_cases'][-settings['failed_case_capacity']:]
    old_grade = state['grade']
    state['grade'] = _grade(state['recent'], settings)
    if state['grade'] != old_grade:
        state['grade_history'].append(dict(after_rollout_id=rollout_id, previous=old_grade,
            grade=state['grade'], recent_reports=len(state['recent']), source='complete learned-training reports only'))
    return dict(eligible=True, passed=not failures, failures=failures, grade=state['grade'], previous_grade=old_grade)
