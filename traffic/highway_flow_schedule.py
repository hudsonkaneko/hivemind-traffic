"""Deterministic longitudinal case exposure, with measured decision shortfalls."""
from copy import deepcopy
from traffic.highway_smooth_schedule import digest, permuted


def build_schedule(cfg):
    settings = cfg['schedule']
    cases = [c for c in cfg['base_config']['training_cases'] if c['task'] in ('cruise', 'following')]
    if not cases or any(c['split'] != 'train' for c in cases):
        raise ValueError('Flow requires training-only longitudinal cases')
    # Every geometry/task appears once per block; difficulty rotates by block.
    # Population, speed and channel streams are independent deterministic cycles.
    result = []
    for i in range(settings['seed_end']-settings['seed_start']+1):
        block, offset = divmod(i, len(cases))
        case = permuted(cases, i, settings['seed'], 'case')
        difficulty = (block + offset) % 3
        total = permuted((8, 16, 24), i, settings['seed'], 'population')
        fractions = (.125, .25, .5, .75) if case['task'] == 'following' else (.125, .25, .5, .75, 1.)
        fraction = permuted(fractions, i, settings['seed'], 'fraction_' + case['task'])
        count = max(1, round(total*fraction))
        if block==0:
            count,fraction=1,1./total
        channel = permuted((('none', 0.), ('local', 0.), ('local', .3)), i, settings['seed'], 'channel')
        result.append(dict(case=deepcopy(case), seed=settings['seed_start']+i,
            controlled_count=count, total_vehicles=total, requested_av_fraction=fraction,
            difficulty=difficulty, speed_limit_m_s=permuted((12.,18.,22.352,26.8224), i, settings['seed'], 'speed'),
            communication=channel[0], packet_loss=channel[1],
            traffic_mode='initial' if count == total else 'sustained',
            matched_schedule=dict(index=i, stratum_block=block,
                exposure_stage='single_av_full_case_coverage' if block==0 else 'varied_population',
                dense_following=case['task']=='following' and difficulty==2,
                planned_case_weight=1, definition='Equal geometry/task CASE allocation; variable actual active decisions')))
    return result


def next_case(schedule,state,*,critic_only=False):
    cursor='critic_schedule_index' if critic_only else 'schedule_index'
    index=state[cursor]
    if index>=len(schedule):
        raise ValueError('Matched training seed bank exhausted')
    state[cursor]+=1
    return deepcopy(schedule[index])


def add_coverage(state, row, actual, *, critic_only, report=None):
    if type(actual) is not int or actual < 0:
        raise ValueError('Nonnegative actual decisions required')
    case = row['case']
    key = '|'.join(map(str, (case['id'], row['difficulty'], row['total_vehicles'], row['controlled_count'],
        row['speed_limit_m_s'], row['communication'], row['packet_loss'])))
    bucket = state['coverage'].setdefault(key, dict(case_id=case['id'], task=case['task'], episodes=0,
        complete_ppo_episodes=0, ppo_decisions=0, critic_decisions=0))
    bucket['episodes'] += 1
    bucket['critic_decisions' if critic_only else 'ppo_decisions'] += actual
    if not critic_only:
        complete = bool(report and report.get('primary_horizon_complete') is True
            and report.get('accounting_comparison_valid') is True and report.get('partial_control_horizon') is False)
        bucket['complete_ppo_episodes'] += int(complete)
        state['task_decisions'][case['task']] += actual
        state['difficulty_decisions'][str(row['difficulty'])] += actual
        state['dense_following_decisions'] += actual*int(row['matched_schedule']['dense_following'])


def coverage_report(state, cfg):
    actual = state['ppo_actual']
    expected = {c['id'] for c in cfg['base_config']['training_cases'] if c['task'] in ('cruise','following')}
    complete = {b['case_id'] for b in state['coverage'].values() if b['complete_ppo_episodes']}
    tasks = {task: dict(planned_case_fraction=.5, actual_decisions=state['task_decisions'][task],
        actual_decision_fraction=state['task_decisions'][task]/actual if actual else None,
        decision_share_shortfall=max(0., .5-state['task_decisions'][task]/actual) if actual else .5)
        for task in ('cruise','following')}
    return dict(actual_ppo_decisions=actual, tasks=tasks,
        difficulty_decisions=deepcopy(state['difficulty_decisions']),
        required_complete_case_ids=sorted(expected), completed_ppo_case_ids=sorted(complete),
        missing_complete_case_ids=sorted(expected-complete), full_case_coverage=expected <= complete,
        detailed=deepcopy(state['coverage']), mastery_grade=None,
        limitation='Case and decision exposure are distinct. Warmup/partial episodes do not satisfy full PPO case coverage; shortfalls are retained, never padded or called mastery.')
