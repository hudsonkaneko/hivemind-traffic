"""v15 Phase 3: imitation of the rule expert (behaviour cloning, then DAgger). SUMO only, charged.

User decision, November 26, 2026 (relayed by the main Claude session, recorded in the coordination master):
"Recommended size", for plan v15's Phase 3 proposal (master hash 49d1d5de):
- build the uncharged pieces, then run the uncharged DAgger cost check (`check`); if it shows the
  recommended size no longer fits, stop and re-size before anything is charged;
- then the charged steps (data, 3 cloning fits, up to 4 DAgger iterations, the final check) under Phase 3's
  own allowance: at most 2,700,000 decisions and 3,000 s of clock time; a 5th DAgger iteration only if the
  earlier steps came in under their estimates and it fits the cap;
- at most 16 workers at below-normal priority, IDM background only, SUMO only, development rows only for
  evaluation, held-out rows never built or run, the shared lock, and each run's resolved configuration and
  expected charge in the master before it starts;
- stop and ask if: the next step won't fit; the lock is held; a canary or parity check mismatches; tests
  fail; the expert collides or stops on a free road on a training row; a learned policy collides, stops on a
  free road, or causes a cut-in or emergency in a one-car LA case; or the measured cost per decision after
  step 1 puts the remaining steps over the cap.
Not authorized: raising the study total, Phase 4, Krauss runs, held-out evaluation, merging to main.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_phase3.py check NAME           the uncharged code checks on v14-validation rows 0 and 4
  python experiments/v15_phase3.py resolve STEP         write configs/evaluation/v15-phase3-STEP.json (no simulation)
  python experiments/v15_phase3.py run STEP             check the pinned configuration, take the lock, run (charged)
  python experiments/v15_phase3.py report STEP          rebuild the step's report (and the re-baselined verdicts)

STEP is data, clone, dagger-1 ... dagger-5 or final; each writes outputs/highway-merge-v15/phase-3/STEP.
- data: the rule Level 2 ('l2:rule-v3', which reproduces expert v3 exactly) collects on the 20 training rows at
  seed sets 0-9 (200 episodes; row i of set k perturbed when k + i is odd). Sets 8 and 9 are held back.
- clone: 3 fits (fit seeds 1-3) on sets 0-7, each evaluated on the 70 development rows.
- dagger-j: the latest learner rolls out on 5 new seed sets (10 + 5 (j - 1) onwards) acting with probability
  0.7, the rule labels every state; a refit on everything so far (the best clone's fit seed), evaluated.
- final: the selected learner's data refit at fit seeds 4 and 5, each evaluated.
Every run also re-runs 'l2:rule-v3' on development row 52 and its report and trace hashes must equal Phase 2
run 3's record of expert v3 there (parity), and every batch has a canary on another worker.
Selection is by closed-loop pass rate on the development rows under the frozen re-baselined gates
(experiments/v15_regrade.py, decision 3), with round 24's recorded gates second; never by held-back loss.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.v15_phase1 import _file_sha, _json_sha, _read_jsonl  # noqa: E402

SCHEMA = 'highway_merge_v15_phase3_v01'
CONFIG_DIR = ROOT / 'configs/evaluation'
OUT_ROOT = ROOT / 'outputs/highway-merge-v15/phase-3'
ROUND_INDEX = 24
DEV, TRAIN, CHECK_ROWS = 'v15-development', 'v15-training', 'v14-validation'
BACKEND, BACKGROUND = 'libsumo', 'idm'
WORKERS, PRIORITY = 16, 'below_normal'       # the user's CPU limit (see experiments/v15_phase1.py)
ALLOWANCE, TIME_CAP = 2_700_000, 3000.        # "Recommended size": Phase 3's own allowance
MARGIN = 0.05
DATA_SETS, HELD_BACK = tuple(range(10)), (8, 9)
DAGGER_ITERATIONS, DAGGER_MINIMUM, DAGGER_SETS = 5, 3, 5
CLONE_SEEDS, FINAL_SEEDS = (1, 2, 3), (4, 5)
STEPS = ('data', 'clone') + tuple(f'dagger-{j}' for j in range(1, DAGGER_ITERATIONS + 1)) + ('final',)
CHECK_ROW_IDS = (0, 4)
PHASE2_RUN3 = 'outputs/highway-merge-v15/phase-2/run-03'
PARITY_ROW = 52
EXPERT = 'rule:expert-v3'
ONE_CAR_LA = ('v12-val-la-nb-pasadena-model-merge-focused-1of16-medium', 'v12-val-la-sb-adams-model-merge-focused-1of16-medium')
# Estimates (plan v15, "Phase 3 proposal"): one training seed set about 32,000 decisions; a development
# evaluation of expert v3 135,428 (Phase 2 run 3); 1,300 decisions per second of clock time at 16 workers.
PER_SET, PER_DECISION_S, START_S = 32_000, 1 / 1300., 15.
FIT_S = dict(clone=35., dagger=40.)
COST_CHECK = 'cost-01'      # the uncharged check whose measured cost ratios scale the clock estimates


def cost_ratios():
    """Measured worker seconds per decision against expert v3's (the uncharged check), at least 1."""
    path = OUT_ROOT / 'checks' / COST_CHECK / 'check.json'
    if not path.is_file():
        return dict(rule_collect=1.3, dagger=1.3, learner_validate=1.3, source='proposal assumption (1.3)')
    ratio = json.loads(path.read_text())['cost_ratio']
    return dict(rule_collect=max(1., ratio['rule_collect_perturbed'], ratio['rule_collect']),
                dagger=max(1., ratio['dagger']), learner_validate=max(1., ratio['learner_validate']),
                source=f'outputs/highway-merge-v15/phase-3/checks/{COST_CHECK}/check.json')
AUTHORIZATION = ('User decision of November 26, 2026, relayed by the main Claude session: "Recommended size" '
                 '(plan v15 Phase 3 at the recommended size: at most 2,700,000 decisions and 3,000 s of clock time '
                 'as Phase 3\'s own allowance; IDM only, SUMO only, development rows only for evaluation, '
                 'held-out rows never built or run; at most 16 workers at below-normal priority).')


def config_path(step):
    return CONFIG_DIR / f'v15-phase3-{step}.json'


def out_dir(step):
    return OUT_ROOT / step


def _perturb(k, i):
    from traffic import v15_level2
    return v15_level2.PERTURBATION['version'] if (k + i) % 2 else None


def _ledger_phase3():
    from experiments import v15_ledger as L
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    totals = L.totals(ledger)
    return totals['v15'].get('3', dict(segments=0, decisions=0, seconds=0., evaluation_episodes=0)), totals


def _run3():
    """Phase 2 run 3's expert v3 records by development row, and its resolved configuration."""
    out = ROOT / PHASE2_RUN3
    records = {r['fields']['row']: r for r in _read_jsonl(out / 'results.jsonl')
               if r['attempt'] == 'first' and r['fields']['driver'] == EXPERT and r['fields']['row_set'] == DEV}
    if len(records) != 70:
        raise ValueError('Phase 2 run 3 has no expert v3 record for every development row')
    return records


def _report(step):
    path = out_dir(step) / 'report.json'
    if not path.is_file():
        raise ValueError(f'Step {step} has no report yet ({path})')
    return json.loads(path.read_text())


# --------------------------------------------------------------------------- resolve

def _previous_steps(step):
    index = STEPS.index(step)
    return STEPS[:index]


def _learner_history():
    """Every evaluated learner so far: dict(id, step, fit_seed, data, passed, rebaselined, ...), in order."""
    history = []
    for step in STEPS:
        path = out_dir(step) / 'report.json'
        if not path.is_file():
            break
        history.extend(_report(step).get('learners', []))
    return history


def _best(history):
    """The selected learner: most re-baselined passes, then most recorded passes, then the earliest."""
    if not history:
        return None
    return max(enumerate(history), key=lambda e: (e[1]['rebaselined_passed'], e[1]['recorded_passed'], -e[0]))[1]


def _data_files(steps):
    """The level2.pt files and their seed sets of the given steps' collect tasks, in task order."""
    files = []
    for step in steps:
        index = out_dir(step) / 'dataset' / 'index.jsonl'
        if not index.is_file():
            continue
        for line in _read_jsonl(index):
            f = line['fields']
            if f['row_set'] != TRAIN:
                continue
            k = f['row'] // 20
            files.append(dict(path=line['samples'], samples_sha=line['samples_sha'], seed_set=k, row=f['row'], step=step,
                              decisions=line['decisions']))
    return files


def _split(files):
    train = [f for f in files if f['seed_set'] not in HELD_BACK]
    held = [f for f in files if f['seed_set'] in HELD_BACK]
    return train, held


def _dev_tasks(drivers):
    return [dict(row_set=DEV, row=r, driver=d) for r in range(70) for d in drivers]


def _measured_per_set():
    """Step 1's decisions per training seed set (simulated collect tasks), or the proposal's estimate."""
    path = out_dir('data') / 'results.jsonl'
    if not path.is_file():
        return PER_SET, 'proposal estimate (round 24 training episodes)'
    records = [r for r in _read_jsonl(path) if r['fields']['row_set'] == TRAIN and r['attempt'] == 'first']
    sets = {r['fields']['row'] // 20 for r in records}
    return sum(r['decisions'] for r in records) / max(len(sets), 1), f'measured in step data ({len(sets)} seed sets)'


def _measured_dev():
    """Decisions of one 70-row development evaluation: the latest learner's, else expert v3's (Phase 2 run 3)."""
    history = _learner_history()
    if history:
        return history[-1]['decisions'], f'learner {history[-1]["id"]} in step {history[-1]["step"]}'
    return sum(r['decisions'] for r in _run3().values()), 'expert v3, Phase 2 run 3'


def resolve(step):
    """A step's resolved configuration and expected charge (no simulation)."""
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2, v15_training_rows
    from experiments import v15_level2_train as T
    if step not in STEPS:
        raise ValueError(f'Unknown step {step!r}; expected one of {STEPS}')
    for earlier in _previous_steps(step):
        if earlier.startswith('dagger-') and not (out_dir(earlier) / 'report.json').is_file():
            break                       # DAgger may stop early; later steps then follow the last one run
        if not earlier.startswith('dagger-') and not (out_dir(earlier) / 'report.json').is_file():
            raise ValueError(f'Step {earlier} has not run yet')
    cfg = contract.build_config(ROUND_INDEX)
    run3 = _run3()
    per_set, per_set_source = _measured_per_set()
    dev_decisions, dev_source = _measured_dev()
    dev_rows = v15_benchmark.development_rows(cfg)
    parity = dict(driver=v15_level2.RULE_DRIVER, row=PARITY_ROW, report_sha=run3[PARITY_ROW]['report_sha'],
                  trace_sha=run3[PARITY_ROW]['trace_sha'], decisions=run3[PARITY_ROW]['decisions'],
                  source=f'{PHASE2_RUN3}/results.jsonl, {EXPERT}')
    batches, fits, inputs = [], [], {}
    if step == 'data':
        tasks = [dict(row_set=TRAIN, row=v15_training_rows.index(k, i), driver=v15_level2.RULE_DRIVER, mode='collect',
                      label_with=v15_level2.RULE_DRIVER, perturb=_perturb(k, i), seed_set=k)
                 for k in DATA_SETS for i in range(20)]
        first = [t for t in tasks if t['seed_set'] == DATA_SETS[0]]
        rest = [t for t in tasks if t['seed_set'] != DATA_SETS[0]]
        batches = [dict(name='data-set-0', tasks=first + [dict(parity, row_set=DEV, mode='validate', parity=True)],
                        expected_decisions=round(per_set * 1 + parity['decisions'] + 800)),
                   dict(name='data-sets-1-9', tasks=rest, expected_decisions=round(per_set * (len(DATA_SETS) - 1) + 800))]
    elif step == 'clone':
        files = _data_files(['data'])
        train, held = _split(files)
        inputs = dict(train_files=len(train), held_back_files=len(held),
                      train_decisions=sum(f['decisions'] for f in train), data_index=_index_sha(['data']))
        fits = [dict(name=f'clone-seed-{s}', seed=s, data_steps=['data']) for s in CLONE_SEEDS]
        batches = [dict(name='clone-evaluations', learners_from_fits=[f['name'] for f in fits],
                        tasks=[dict(parity, row_set=DEV, mode='validate', parity=True)],
                        evaluate_fits=True,
                        expected_decisions=round(len(fits) * dev_decisions + parity['decisions'] + 800))]
    elif step.startswith('dagger-'):
        j = int(step.split('-')[1])
        history = _learner_history()
        if not history:
            raise ValueError('DAgger needs the cloning step first')
        latest, best = history[-1], _best(history)
        clone_best = _best([h for h in history if h['step'] == 'clone'])
        sets = tuple(range(10 + DAGGER_SETS * (j - 1), 10 + DAGGER_SETS * j))
        tasks = [dict(row_set=TRAIN, row=v15_training_rows.index(k, i), driver=latest['id'], mode='collect',
                      label_with=v15_level2.RULE_DRIVER, sample_seed=j, seed_set=k) for k in sets for i in range(20)]
        data_steps = ['data'] + [f'dagger-{i}' for i in range(1, j + 1)]
        fits = [dict(name=f'dagger-{j}-refit', seed=clone_best['fit_seed'], data_steps=data_steps)]
        inputs = dict(rollout_learner=latest['id'], rollout_learner_step=latest['step'], best_so_far=best['id'],
                      refit_seed=clone_best['fit_seed'], seed_sets=list(sets), data_index=_index_sha(data_steps[:-1]))
        batches = [dict(name=f'dagger-{j}-rollouts', tasks=tasks + [dict(parity, row_set=DEV, mode='validate', parity=True)],
                        expected_decisions=round(per_set * len(sets) + parity['decisions'] + 800)),
                   dict(name=f'dagger-{j}-evaluation', learners_from_fits=[fits[0]['name']], evaluate_fits=True, tasks=[],
                        expected_decisions=round(dev_decisions + 400))]
    else:   # final
        history = _learner_history()
        selected = _best(history)
        data_steps = selected['data_steps']
        fits = [dict(name=f'final-seed-{s}', seed=s, data_steps=data_steps) for s in FINAL_SEEDS]
        inputs = dict(selected=selected['id'], selected_step=selected['step'], selected_fit_seed=selected['fit_seed'],
                      data_index=_index_sha(data_steps))
        batches = [dict(name='final-evaluations', learners_from_fits=[f['name'] for f in fits], evaluate_fits=True,
                        tasks=[dict(parity, row_set=DEV, mode='validate', parity=True)],
                        expected_decisions=round(len(fits) * dev_decisions + parity['decisions'] + 800))]
    expected = sum(b['expected_decisions'] for b in batches)
    fit_seconds = sum(FIT_S['clone' if step in ('clone', 'final') else 'dagger'] for _ in fits)
    ratios = cost_ratios()
    kind = dict(data='rule_collect', clone='learner_validate', final='learner_validate').get(step, 'dagger')
    clock = START_S + expected * PER_DECISION_S * ratios[kind] + fit_seconds + 10. * len(batches)
    phase, totals = _ledger_phase3()
    decisions_left, seconds_left = ALLOWANCE - phase['decisions'], TIME_CAP - phase['seconds']
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    reserve = min(round(clock * 1.6 + 30.), seconds_left, study_left)
    episodes = sum(len(b['tasks']) + 1 for b in batches) + sum(70 for b in batches if b.get('evaluate_fits')
                                                                for _ in b['learners_from_fits'])
    return dict(
        schema=SCHEMA, phase='3', step=step, round_index=ROUND_INDEX, backend=BACKEND, workers=WORKERS, priority=PRIORITY,
        backgrounds=[BACKGROUND], benchmark=v15_benchmark.manifest(cfg), training_rows=v15_training_rows.manifest(cfg),
        level2=dict(rule=v15_level2.RULE_DRIVER, expert=EXPERT, settings_digest=v15_level2.settings_digest(),
                    mixing=v15_level2.MIXING, perturbation=v15_level2.PERTURBATION, sampling=v15_level2.SAMPLING,
                    offsets=list(v15_level2.OFFSETS), gaps=v15_level2.GAPS, excluded_inputs=list(v15_level2.EXCLUDED),
                    executor_inputs=list(v15_level2.EXECUTOR_INPUTS),
                    sources={p: _file_sha(ROOT / p) for p in ('traffic/v15_level2.py', 'traffic/v15_rule_expert.py',
                                                              'experiments/v15_level2_train.py', 'experiments/v15_pool.py',
                                                              'traffic/v15_training_rows.py')}),
        fit_settings=T.FIT, data_split=dict(held_back_seed_sets=list(HELD_BACK), rule='whole seed sets'),
        inputs=inputs, fits=fits, batches=batches, parity=parity,
        selection='most passes on the 70 development rows under the frozen re-baselined gates (experiments/v15_regrade.py, '
                  'all three rules), then round 24\'s recorded gates, then the earliest; never held-back loss',
        dagger_stop=f'at least {DAGGER_MINIMUM} iterations (plan v15 Stage 3: 3-5), then stop at the first whose '
                    f're-baselined pass count is not above the best so far; at most {DAGGER_ITERATIONS}; or when the '
                    'next will not fit the cap',
        stop_checks=dict(expert_on_training_rows='any collision, or a free-road stop in an unperturbed episode',
                         learner_on_development_rows='any collision or free-road stop, or a cut-in or emergency in a '
                                                     'one-car LA case', one_car_la_cases=list(ONE_CAR_LA)),
        charge=dict(expected_decisions=expected, expected_episodes=episodes, expected_clock_seconds=round(clock, 1),
                    expected_fit_seconds=fit_seconds, reserve_seconds=reserve, phase_allowance=ALLOWANCE,
                    phase_time_cap=TIME_CAP, phase_decisions_used=phase['decisions'],
                    phase_seconds_used=round(phase['seconds'], 3), decisions_left=decisions_left,
                    seconds_left=round(seconds_left, 3), study_seconds_left=round(study_left, 3), guard_margin=MARGIN,
                    estimate_source=dict(per_training_set=round(per_set), per_training_set_source=per_set_source,
                                         development_evaluation=dev_decisions, development_source=dev_source,
                                         decisions_per_clock_second=1300, start_seconds=START_S, fit_seconds=FIT_S,
                                         cost_ratio=ratios[kind], cost_ratios=ratios),
                    total_wall_seconds=cfg['budget']['total_wall_seconds']),
        authorization=AUTHORIZATION, held_out='not built, not run', simulator='SUMO (libsumo); no physics')


def _index_sha(steps):
    import hashlib
    digest = hashlib.sha256()
    for step in steps:
        path = out_dir(step) / 'dataset' / 'index.jsonl'
        if path.is_file():
            digest.update(step.encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


# --------------------------------------------------------------------------- run

class ParityMismatch(RuntimeError):
    """The parity task differs from Phase 2 run 3's record of expert v3."""


class StopAndAsk(RuntimeError):
    """A stop-and-ask condition of the approval was met: the run stops and the user is asked."""


def _task(P, t, run):
    return P.Task(t['row_set'], t['row'], t['driver'], background=BACKGROUND, mode=t.get('mode', 'validate'),
                  sample_seed=t.get('sample_seed', 0), label_with=t.get('label_with'), perturb=t.get('perturb'), run=run)


def _fit(step, spec, log):
    from experiments import v15_level2_train as T
    files = _data_files(spec['data_steps'])
    train, held = _split(files)
    data, held_data = T.load([f['path'] for f in train]), (T.load([f['path'] for f in held]) if held else None)
    net, report = T.fit(data, seed=spec['seed'], held_back=held_data, log=log)
    report.update(name=spec['name'], data_steps=spec['data_steps'], train_files=len(train), held_back_files=len(held))
    return net, report


def run(step):
    from experiments import v15_ledger as L
    from experiments import v15_pool as P
    config = json.loads(json.dumps(resolve(step)))
    pinned = json.loads(config_path(step).read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {config_path(step)}; re-resolve and record it first')
    sha = _json_sha(pinned)
    charge = config['charge']
    if config['workers'] > 16:
        raise ValueError('At most 16 workers (the user\'s CPU limit)')
    if charge['expected_decisions'] * (1 + MARGIN) > charge['decisions_left'] or \
            charge['expected_clock_seconds'] > charge['seconds_left']:
        raise L.BudgetExceeded(f'Step {step} will not fit what is left of Phase 3')
    out = out_dir(step)
    if (out / 'charge.json').is_file():
        raise ValueError(f'Step {step} has already run ({out})')
    learner_priority = P.set_priority(config['priority'])
    if learner_priority != config['priority']:
        raise RuntimeError(f'The learner runs at {learner_priority} priority, not {config["priority"]}')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    log_path = out / 'phase3-log.jsonl'

    def log(entry):
        with open(log_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(entry) + '\n')
        print(json.dumps(entry), flush=True)

    fit_reports, learners = [], {}
    with L.EvaluationSession(phase='3', output=out, config_sha256=sha, expected_decisions=charge['expected_decisions'],
                             decision_allowance=ALLOWANCE, expected_clock_seconds=charge['expected_clock_seconds'],
                             reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                             intended_episodes=charge['expected_episodes'], workers=config['workers'],
                             priority=config['priority'], authorization=config['authorization'],
                             round_index=config['round_index']) as session:
        session.segment.update(phase3_step=step, evaluation_only=False, v15_work=_work(config),
                               fits=0, fit_seconds=0., training_episodes=0)
        with P.WorkerPool(config['workers'], out, backend=config['backend'], round_index=config['round_index'],
                          priority=config['priority']) as pool:
            session.segment['worker_priorities'] = sorted(set(pool.worker_priorities.values()))
            pool.charged_by = f'cost-ledger {L.COMMAND} phase 3 step {step}, started_at_unix {session.segment["started_at_unix"]}'
            expected_so_far = charged_so_far = 0
            fitted = False
            for index, batch in enumerate(config['batches']):
                if batch.get('evaluate_fits') and not fitted:
                    fitted = True
                    for spec in config['fits']:
                        if session.elapsed() + FIT_S['dagger'] * 2 > session.allowed_seconds:
                            raise L.BudgetExceeded(f'No time left for fit {spec["name"]}')
                        started = time.perf_counter()
                        net, report = _fit(step, spec, None)
                        driver = pool.publish_level2(net.state_dict())
                        seconds = time.perf_counter() - started
                        report.update(driver=driver, fit_clock_seconds=seconds)
                        fit_reports.append(report)
                        learners[spec['name']] = driver
                        session.segment['fits'] += 1
                        session.segment['fit_seconds'] += seconds
                        log(dict(fit=spec['name'], driver=driver, seed=spec['seed'], samples=report['samples'],
                                 seconds=round(seconds, 2), held_back={k: v['accuracy'] for k, v in report['held_back'].items()}
                                 if report['held_back'] else None, elapsed_seconds=session.elapsed()))
                    (out / 'fits.json').write_text(json.dumps(fit_reports, indent=1))
                tasks = list(batch['tasks'])
                if batch.get('evaluate_fits'):
                    drivers = [learners[name] for name in batch['learners_from_fits']]
                    tasks = P.interleave(*[[dict(row_set=DEV, row=r, driver=d) for r in range(70)] for d in drivers]) + tasks
                ratio = max(1., charged_so_far / expected_so_far) if expected_so_far else 1.
                projected = batch['expected_decisions'] * ratio * (1 + MARGIN)
                session.check_next(projected)
                pool_tasks = [_task(P, t, f'phase3-{step}:{batch["name"]}') for t in tasks]
                try:
                    result = pool.run(pool_tasks)
                except P.CanaryMismatch as error:
                    if error.result is not None:
                        _add(session, error.result, pool_tasks)
                    else:
                        session.charge_estimate(projected, f'batch {index}: {error}')
                    raise
                except BaseException as error:
                    session.charge_estimate(projected, f'batch {index} failed before reporting: {type(error).__name__}')
                    raise
                _add(session, result, pool_tasks)
                expected_so_far += batch['expected_decisions']
                charged_so_far += result.charged_decisions
                parity = None
                for t, record in zip(tasks, result.records):
                    if t.get('parity'):
                        parity = (record['report_sha'], record['trace_sha']) == (config['parity']['report_sha'],
                                                                                 config['parity']['trace_sha'])
                log(dict(batch=index, name=batch['name'], tasks=len(pool_tasks), simulated=result.simulated_tasks,
                         clock_seconds=result.clock_seconds, worker_seconds=result.worker_seconds,
                         decisions=result.decisions, canary_decisions=result.canary_decisions,
                         expected_decisions=batch['expected_decisions'], canary=result.canary, parity_matched=parity,
                         elapsed_seconds=session.elapsed(), charged_decisions_so_far=session.segment['decisions']))
                if result.simulated_tasks and (result.canary is None or not result.canary['matched']):
                    raise P.CanaryMismatch(f'Batch {index}: canary {result.canary}')
                if parity is False:
                    raise ParityMismatch(f'Batch {index}: l2:rule-v3 on development row {PARITY_ROW} differs from '
                                         f'Phase 2 run 3\'s expert v3')
                if step == 'data':
                    _expert_checks(result, tasks)
    segment = session.segment
    summary = {k: segment[k] for k in ('status', 'charged_seconds', 'clock_seconds', 'decisions', 'canary_decisions',
                                       'evaluation_episodes', 'training_episodes', 'fits', 'fit_seconds', 'worker_seconds',
                                       'simulated_seconds', 'workers', 'batches', 'decision_allowance', 'expected_decisions',
                                       'started_at_unix', 'config_sha256')}
    summary['ledger'] = str(session.path)
    (out / 'charge.json').write_text(json.dumps(summary, indent=1))
    (out / 'learners.json').write_text(json.dumps(learners, indent=1))
    return report(step)


def _work(config):
    work = []
    if any(t.get('mode') == 'collect' for b in config['batches'] for t in b['tasks']):
        work.append('collect')
    if config['fits']:
        work.append('fit')
    if any(b.get('evaluate_fits') for b in config['batches']) or any(t.get('parity') for b in config['batches'] for t in b['tasks']):
        work.append('evaluation')
    return work


def _add(session, result, pool_tasks):
    """Charge a batch; its training-row episodes are counted as training episodes, not evaluation episodes."""
    session.add_batch(result)
    simulated = [r for r in result.records if not r.get('reused')]
    training = sum(1 for r in simulated if r['fields']['row_set'] == TRAIN)
    canary_training = int(result.canary is not None and any(r['key'] == result.canary['key'] and r['fields']['row_set'] == TRAIN
                                                            for r in simulated))
    session.segment['evaluation_episodes'] -= training + canary_training
    session.segment['training_episodes'] += training + canary_training
    from experiments.v15_ledger import _save
    _save(session.path, session.ledger)


def _expert_checks(result, tasks):
    """Stop and ask if the expert collides anywhere, or stops on a free road in an unperturbed episode."""
    problems = []
    for t, record in zip(tasks, result.records):
        if t.get('parity') or t['row_set'] != TRAIN:
            continue
        events = record['events']
        if events.get('collisions', 0):
            problems.append(dict(row=t['row'], perturbed=bool(t.get('perturb')), collisions=events['collisions']))
        if events.get('free_road_stops', 0) and not t.get('perturb'):
            problems.append(dict(row=t['row'], perturbed=False, free_road_stops=events['free_road_stops']))
    if problems:
        raise StopAndAsk(f'The expert on training rows: {problems}')


# --------------------------------------------------------------------------- report

def _events(records):
    out = {}
    for record in records:
        for event, count in record['events'].items():
            slot = out.setdefault(event, dict(rows=0, count=0))
            slot['rows'] += int(count > 0)
            slot['count'] += int(count)
    return out


def _new_failures(records, rows):
    found = []
    for record in records:
        e, case = record['events'], rows[record['fields']['row']]['case']['id']
        if e.get('collisions', 0) or e.get('free_road_stops', 0) or \
                (case in ONE_CAR_LA and (e.get('cut_ins', 0) or e.get('emergencies', 0))):
            found.append(dict(row=record['fields']['row'], case=case, **{k: e.get(k, 0) for k in
                                                                         ('collisions', 'free_road_stops', 'cut_ins', 'emergencies')}))
    return found


def report(step):
    from experiments import v15_pool as P
    from experiments import v15_regrade as R
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    from traffic import v15_stats as S
    out = out_dir(step)
    config = json.loads((out / 'resolved-config.json').read_text())
    cfg = contract.build_config(ROUND_INDEX)
    dev_rows = v15_benchmark.development_rows(cfg)
    records = P.read_results(out / 'results.jsonl')
    first = {}
    for record in records:
        first.setdefault(record['key'], record)
    records = list(first.values())
    learners = json.loads((out / 'learners.json').read_text()) if (out / 'learners.json').is_file() else {}
    fits = {f['driver']: f for f in json.loads((out / 'fits.json').read_text())} if (out / 'fits.json').is_file() else {}
    regraded = {}
    if learners:
        drivers, parity = R.regrade_run(out, dict(none=(), rebaselined=R.RULES))
        regraded = dict(drivers=drivers, parity=parity)
    expert = _run3()
    expert_regrade_path = ROOT / PHASE2_RUN3 / 'rebaseline' / 'rebaseline.json'
    result = dict(schema=SCHEMA + '_report', step=step, simulator='SUMO (libsumo); no physics', background=BACKGROUND,
                  rows='v15 training rows (collect) and development rows (evaluation); held-out rows not built or run')
    training = [r for r in records if r['fields']['row_set'] == TRAIN]
    if training:
        unperturbed = [r for r in training if not r['fields'].get('perturb') and r['fields']['driver'] == 'l2:rule-v3']
        result['collect'] = dict(
            episodes=len(training), decisions=sum(r['decisions'] for r in training),
            samples=sum(r.get('level2_samples', 0) for r in training),
            per_seed_set=sum(r['decisions'] for r in training) / max(len({r['fields']['row'] // 20 for r in training}), 1),
            worker_seconds_per_decision=sum(r['seconds'] for r in training) / max(sum(r['decisions'] for r in training), 1),
            events=_events(training), perturbed_events=_events([r for r in training if r['fields'].get('perturb')]),
            unperturbed_events=_events([r for r in training if not r['fields'].get('perturb')]),
            expert_unperturbed=S.rate([r['passed'] for r in unperturbed]) if unperturbed else None,
            learner_share=_share(training))
    out_learners = []
    for name, driver in learners.items():
        mine = sorted((r for r in records if r['fields']['driver'] == driver and r['fields']['row_set'] == DEV),
                      key=lambda r: r['fields']['row'])
        if len(mine) != 70:
            continue
        g = regraded['drivers'][driver]
        passes = [g['passed']['none'][r] for r in range(70)]
        rebased = [g['passed']['rebaselined'][r] for r in range(70)]
        fit = fits.get(driver, {})
        out_learners.append(dict(
            id=driver, step=step, name=name, fit_seed=fit.get('seed'), data_steps=fit.get('data_steps'),
            samples=fit.get('samples'), held_back=fit.get('held_back'), train=fit.get('train'),
            fit_seconds=fit.get('fit_clock_seconds'), decisions=sum(r['decisions'] for r in mine),
            recorded_passed=sum(r['passed'] for r in mine), rebaselined_passed=sum(rebased),
            recorded=S.rate([r['passed'] for r in mine]), rebaselined=S.rate(rebased),
            paired_with_expert_rebaselined=S.paired(rebased, _expert_rebaselined(expert_regrade_path)),
            events=_events(mine), new_failures=_new_failures(mine, dev_rows),
            failing_rows=[dict(row=r['fields']['row'], case=dev_rows[r['fields']['row']]['case']['id'],
                               failures=g['failures']['rebaselined'][r['fields']['row']]) for r in mine
                          if not g['passed']['rebaselined'][r['fields']['row']]],
            choices=_share(mine)))
        if passes != [r['passed'] for r in mine]:
            raise ValueError(f'{driver}: the re-grade without rules does not reproduce the recorded verdicts')
    result['learners'] = out_learners
    result['regrade_parity'] = regraded.get('parity')
    parity = [r for r in records if r['fields']['row_set'] == DEV and r['fields']['driver'] == 'l2:rule-v3']
    result['parity'] = [dict(row=r['fields']['row'], matched=(r['report_sha'], r['trace_sha']) ==
                             (config['parity']['report_sha'], config['parity']['trace_sha'])) for r in parity]
    result['charge'] = json.loads((out / 'charge.json').read_text()) if (out / 'charge.json').is_file() else None
    result['expert'] = dict(recorded_passed=sum(r['passed'] for r in expert.values()),
                            rebaselined_passed=sum(_expert_rebaselined(expert_regrade_path)),
                            source=f'{PHASE2_RUN3} and its rebaseline')
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown(result))
    return result


def _expert_rebaselined(path):
    data = json.loads(Path(path).read_text())
    rows = data['changed_rows'][EXPERT]
    recorded = {r: rec['passed'] for r, rec in _run3().items()}
    for change in rows:
        recorded[change['row']] = change['rebaselined']
    return [recorded[r] for r in range(70)]


def _share(records):
    totals = dict(rule=0, learner=0, perturbed=0)
    disagree, compared = dict(lane=0, gap=0, offset=0), dict(lane=0, gap=0, offset=0)
    for record in records:
        stats = record.get('level2') or {}
        for k, v in (stats.get('acted') or {}).items():
            totals[k] += v
        for k in disagree:
            disagree[k] += (stats.get('disagree') or {}).get(k, 0)
            compared[k] += (stats.get('compared') or {}).get(k, 0)
    return dict(acted=totals, disagreement={k: (disagree[k] / compared[k] if compared[k] else None) for k in disagree},
                compared=compared)


def _markdown(result):
    lines = [f'# v15 Phase 3 step {result["step"]}', '', f'{result["simulator"]}; background {result["background"]}; '
             f'{result["rows"]}.', '']
    if result.get('collect'):
        c = result['collect']
        lines += [f'- Collected {c["episodes"]} training-row episodes, {c["decisions"]:,} decisions, {c["samples"]:,} '
                  f'Level 2 samples ({c["per_seed_set"]:,.0f} decisions per seed set).',
                  f'- Events: {c["events"]}', f'- Unperturbed events: {c["unperturbed_events"]}']
        if c.get('expert_unperturbed'):
            e = c['expert_unperturbed']
            lines.append(f'- Expert on unperturbed training rows: {e["passed"]} of {e["rows"]}.')
        lines.append('')
    if result['learners']:
        lines += ['| Learner | Fit seed | Re-baselined | Recorded | New failures | Gap disagreement |', '|---|---|---|---|---|---|']
        for l in result['learners']:
            d = l['choices']['disagreement']
            lines.append(f'| {l["id"]} | {l["fit_seed"]} | {l["rebaselined_passed"]}/70 | {l["recorded_passed"]}/70 | '
                         f'{len(l["new_failures"])} | {d["gap"]} |')
        lines.append('')
    lines.append(f'Parity: {result["parity"]}')
    lines.append(f'Charge: {result["charge"]}')
    return '\n'.join(lines) + '\n'


# --------------------------------------------------------------------------- uncharged checks

def check(name):
    """Uncharged code checks on v14-validation rows 0 and 4 (2 workers): parity of the split, collect parity,
    a throwaway fit, and the DAgger cost per decision against the expert's."""
    from experiments import v15_level2_train as T
    from experiments import v15_pool as P
    from traffic import v15_level2
    out = OUT_ROOT / 'checks' / name
    if out.exists():
        raise ValueError(f'{out} exists')
    P.set_priority(PRIORITY)
    result = dict(schema=SCHEMA + '_check', rows=list(CHECK_ROW_IDS), row_set=CHECK_ROWS, charged=False,
                  note='uncharged engineering checks on v14-validation rows only; the throwaway network is never '
                       'evaluated on development rows')
    with P.WorkerPool(2, out, backend=BACKEND, round_index=ROUND_INDEX, priority=PRIORITY) as pool:
        result['start_seconds'] = pool.start_seconds
        rule = v15_level2.RULE_DRIVER
        validate = [P.Task(CHECK_ROWS, r, d, run='phase3-check') for r in CHECK_ROW_IDS for d in (EXPERT, rule)]
        collect = [P.Task(CHECK_ROWS, r, rule, mode='collect', label_with=rule, perturb=p, run='phase3-check')
                   for r in CHECK_ROW_IDS for p in (None, v15_level2.PERTURBATION['version'])]
        first = pool.run(validate + collect)
        by = {(t.row, t.driver, t.mode, t.perturb): rec for t, rec in zip(validate + collect, first.records)}
        result['parity'] = [dict(row=r, split=by[(r, EXPERT, 'validate', None)]['report_sha'] == by[(r, rule, 'validate', None)]['report_sha']
                                 and by[(r, EXPERT, 'validate', None)]['trace_sha'] == by[(r, rule, 'validate', None)]['trace_sha'],
                                 collect=by[(r, rule, 'collect', None)]['trace_sha'] == by[(r, EXPERT, 'validate', None)]['trace_sha'],
                                 decisions=by[(r, EXPERT, 'validate', None)]['decisions']) for r in CHECK_ROW_IDS]
        result['canary_1'] = first.canary
        files = [rec['samples'] for rec in first.records if rec.get('samples')]
        data = T.load(files)
        started = time.perf_counter()
        net, fit = T.fit(data, seed=0, held_back=data)
        result['fit'] = dict(samples=fit['samples'], seconds=time.perf_counter() - started, train=fit['train'])
        driver = pool.publish_level2(net.state_dict())
        learned = [P.Task(CHECK_ROWS, r, driver, mode='collect', label_with=rule, sample_seed=1, run='phase3-check')
                   for r in CHECK_ROW_IDS] + [P.Task(CHECK_ROWS, r, driver, run='phase3-check') for r in CHECK_ROW_IDS]
        second = pool.run(learned)
        result['canary_2'] = second.canary
        kinds = dict(expert=[by[(r, EXPERT, 'validate', None)] for r in CHECK_ROW_IDS],
                     rule_validate=[by[(r, rule, 'validate', None)] for r in CHECK_ROW_IDS],
                     rule_collect=[by[(r, rule, 'collect', None)] for r in CHECK_ROW_IDS],
                     rule_collect_perturbed=[by[(r, rule, 'collect', 'p1')] for r in CHECK_ROW_IDS],
                     dagger=second.records[:len(CHECK_ROW_IDS)], learner_validate=second.records[len(CHECK_ROW_IDS):])
        result['cost'] = {k: dict(decisions=sum(r['decisions'] for r in v), worker_seconds=sum(r['seconds'] for r in v),
                                  seconds_per_decision=sum(r['seconds'] for r in v) / max(sum(r['decisions'] for r in v), 1),
                                  per_row=[dict(row=r['fields']['row'], decisions=r['decisions'], seconds=round(r['seconds'], 3),
                                                passed=r['passed'], events=r['events'], level2=r.get('level2'),
                                                samples=r.get('level2_samples')) for r in v])
                          for k, v in kinds.items()}
        e = result['cost']['expert']['seconds_per_decision']
        result['cost_ratio'] = {k: v['seconds_per_decision'] / e for k, v in result['cost'].items()}
    (out / 'check.json').write_text(json.dumps(result, indent=1))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('check', 'resolve', 'run', 'report'))
    parser.add_argument('step')
    args = parser.parse_args(argv)
    if args.command == 'check':
        result = check(args.step)
        print(json.dumps({k: result[k] for k in ('parity', 'cost_ratio', 'fit')}, indent=1))
    elif args.command == 'resolve':
        config = resolve(args.step)
        path = config_path(args.step)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(config, indent=1, sort_keys=True) + '\n')
        print(json.dumps(dict(config=str(path.relative_to(ROOT)), sha256=_json_sha(config),
                              **{k: config['charge'][k] for k in ('expected_decisions', 'expected_episodes',
                                                                  'expected_clock_seconds', 'reserve_seconds',
                                                                  'decisions_left', 'seconds_left')})))
    elif args.command == 'run':
        result = run(args.step)
        print(json.dumps(dict(collect=result.get('collect'), learners=[{k: l[k] for k in ('id', 'rebaselined_passed',
                              'recorded_passed', 'new_failures')} for l in result['learners']], parity=result['parity'])))
    else:
        report(args.step)
    return 0


if __name__ == '__main__':
    sys.exit(main())
