"""Immutable matched case sequence; planned exposure is not measured mastery."""
from copy import deepcopy
import hashlib
import json


STRATA = ([("cruise", d, False) for d in (0, 0, 1, 1)]
          + [("following", d, d == 2) for d in (2, 2, 2, 2, 0, 1)]
          + [("passing", d, False) for d in (0, 0, 1, 1)]
          + [("merge", d, False) for d in (0, 0, 0, 1, 2, 2)])


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def permuted(values, index, seed, stream):
    block, offset = divmod(index, len(values))
    order = sorted(range(len(values)), key=lambda i: digest([seed, stream, block, i]))
    return deepcopy(values[order[offset]])


def build_schedule(cfg):
    settings, base = cfg['schedule'], cfg['base_config']
    seed, start, stop = settings['seed'], settings['seed_start'], settings['seed_end']
    cases = base['training_cases']
    if not cases or any(case['split'] != 'train' for case in cases):
        raise ValueError('Matched schedule accepts training cases only')
    result, case_indices = [], {task: 0 for task in ('cruise', 'following', 'passing', 'merge')}
    for index in range(stop - start + 1):
        task, difficulty, dense = permuted(STRATA, index, seed, 'strata')
        eligible = [case for case in cases if case['task'] == task]
        if not eligible:
            raise ValueError('Missing training task: ' + task)
        case = permuted(eligible, case_indices[task], seed, 'geometry_' + task)
        case_indices[task] += 1
        total = 24 if dense else permuted((8, 16, 24), index, seed, 'population')
        fractions = (.125, .25, .5, .75) if task in ('following', 'passing') else (.125, .25, .5, .75, 1.)
        fraction = permuted(fractions, index, seed, 'fraction_' + task)
        count = round(total * fraction)
        channel = permuted((('none', 0.), ('local', 0.), ('local', .3)), index, seed, 'channel')
        result.append(dict(case=case, seed=start + index, controlled_count=count, total_vehicles=total,
            requested_av_fraction=fraction, difficulty=difficulty,
            speed_limit_m_s=permuted((12., 18., 22.352, 26.8224), index, seed, 'speed'),
            communication=channel[0], packet_loss=channel[1],
            traffic_mode='initial' if count == total else 'sustained',
            matched_schedule=dict(index=index, stratum_block=index // 20, dense_following=dense,
                planned_case_weight=1, definition='Equal planned CASE shares, not active-decision allocation; actual decisions depend on population, arrivals and budget')))
    return result


def next_case(schedule, state):
    index = state['schedule_index']
    if index >= len(schedule):
        raise ValueError('Matched training seed bank exhausted')
    state['schedule_index'] += 1
    return deepcopy(schedule[index])


def add_coverage(state, row, actual, *, critic_only):
    if type(actual) is not int or actual < 0:
        raise ValueError('Coverage requires nonnegative actual decisions')
    key = '|'.join(map(str, (row['case']['task'], row['difficulty'], row['total_vehicles'],
        row['controlled_count'], row['speed_limit_m_s'], row['communication'], row['packet_loss'])))
    bucket = state['coverage'].setdefault(key, dict(episodes=0, ppo_decisions=0, critic_decisions=0))
    bucket['episodes'] += 1
    bucket['critic_decisions' if critic_only else 'ppo_decisions'] += actual
    if not critic_only:
        state['task_decisions'][row['case']['task']] += actual
        state['difficulty_decisions'][str(row['difficulty'])] += actual
        state['dense_following_decisions'] += actual * int(row['matched_schedule']['dense_following'])


def coverage_report(state):
    actual = state['ppo_actual']
    target_tasks = dict(cruise=.2, following=.3, passing=.2, merge=.3)
    target_difficulty = {'0': .4, '1': .3, '2': .3}
    return dict(actual_ppo_decisions=actual,
        tasks={k: dict(decisions=state['task_decisions'][k], actual_fraction=state['task_decisions'][k] / actual if actual else None,
                       planned_case_fraction=v, decision_target_fraction=v,
                       decision_target_shortfall=max(0., v - state['task_decisions'][k] / actual) if actual else v) for k, v in target_tasks.items()},
        difficulties={k: dict(decisions=state['difficulty_decisions'][k], actual_fraction=state['difficulty_decisions'][k] / actual if actual else None,
                             planned_case_fraction=v, decision_target_fraction=v,
                             decision_target_shortfall=max(0., v - state['difficulty_decisions'][k] / actual) if actual else v) for k, v in target_difficulty.items()},
        dense_following_decisions=state['dense_following_decisions'], dense_following_target_fraction=.2,
        dense_following_actual_fraction=state['dense_following_decisions'] / actual if actual else None,
        detailed=deepcopy(state['coverage']), mastery_grade=None,
        limitation='Immutable matched CASE order and case shares, not planned decision allocation. Actual decision targets may be missed. Exposure does not assert competence or full factorial coverage.')
