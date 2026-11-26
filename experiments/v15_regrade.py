"""v15 gate re-baseline (plan v15, decision 3): rolling merge quality re-graded from recorded traces.

User decision 3 (November 25, 2026, recorded in the coordination master and in "Decisions for you" of
documentation/highway-merge-training-plan-v15.md): re-baseline the gates once, after the rule expert
qualifies, on the 70 development rows; when the expert fails a gate, check whether the gate or the driver
is wrong; the gates are frozen before any learning, and the held-out seeds stay untouched. The Phase 2
approval relayed by the main Claude session repeats it: "once, after the expert qualifies. Record it in the
master, then freeze it."

What is re-graded: only round 24's rolling merge quality gate (experiments/run_highway_flow_rl_v05.ruled_gates,
'rolling_merge_quality'), and within it only the speed match. Each graded car's window, its stop, jerk,
override and neighbour checks, its approach stops, every other gate and every per-car check are taken from
the recorded episode unchanged. No simulation runs: the input is each episode's action journal
(actions.jsonl.gz) and report.json, so a re-grade charges nothing.

Parity before any rule. With no rule the re-grade must reproduce, for every graded car, the environment's
window (samples from the merge completion - 8 s to the stable merge), its speed-match fraction and its
qualified flag, and then ruled_gates must give the recorded pass and failures. Every follow-gate reference is
also rebuilt from its recorded inputs (traffic/highway_flow_rl_v14_follow_gate start/advance) and must equal
the recorded value. Checked on Phase 2 run 2 (24,551 steps of 46 graded cars; 9,823 reference decisions) and
on Phase 1 (45,262 steps of 95 graded cars): every step's speed match and every window matched exactly.
Any mismatch stops the re-grade (ParityMismatch).

The rules, each from the audit of Phase 2 run 2's 16 merge-quality failures (plan v15, "Phase 2 result"):

R1 every_merge: a merge without a yield entry is graded on the follow gate's reference band too.
   Round 23 gave yield entries a reference that starts at the car's speed and rises no faster than
   min(1.5 m/s^2, the drive ceiling) (traffic/highway_flow_rl_v14_follow_gate.py), because v12's target
   "after the line ... jumps toward the speed limit". Merges from an acceleration lane kept that target:
   at the merge completion it returns to the road target at once (run 2 row 4: 15.9 -> 22.35 m/s; row 20:
   6.8 -> 24.6 m/s). A car at the 2.0 m/s^2 comfort cap then needs 3-8 s to come within 2 m/s, so the 90%
   match cannot be met. Here the reference starts at the car's speed and acceleration at the first eligible
   decision of the graded window and moves exactly as the follow gate's does, toward the recorded gate target,
   capped at once by that target's occupied-lane, occupied-leader and ending-lane bounds, at the drive ceiling
   the applied command got (acceleration_bounds.drive_grade).
R2 restart_at_stop_line: a yield entry's reference also restarts from the car's speed and acceleration at the
   first decision after the car crosses the stop line (the follow audit's yield_entry turns false). While a
   car waits for a gap the reference keeps rising toward the gap planner's target
   (traffic/highway_flow_rl_v14_rewards.gap_plan judges only the nearest front and rear car); once the car is
   2 m/s behind, a car limited by the same drive ceiling cannot catch up. Run 2 row 53: the expert waited
   for main_0 (22 m/s), which passed the junction about 0.5 s before our car crossed; the reference had
   restarted 2 s earlier and rose at the drive ceiling of 1.32 m/s^2, and the car, accelerating at its
   ceiling from the line, stayed 3.4-3.8 m/s behind for the whole window (speed match 0.00).
   Samples before the crossing are still graded against the rising reference.
R3 eligible_samples_only: the speed-match fraction counts only samples whose gate target is eligible. An
   ineligible target (for example 'required_merge_target_unknown' on a junction lane, or
   'unresolved_neighbor_lateral_motion' while a neighbour changes lanes) counts as a miss today, whatever the
   car does: 32 of agent_0's 161 samples in run 2 row 0. Those samples stay in every other check. A window
   with fewer eligible samples than half its samples does not qualify (a guard against grading a merge on a
   sliver of its window; the fewest in the recorded runs is 108 of 161).

Not changed: following behind a slower leader. The gate's target there is the leader's stopping bound, the
speed whose 1.5 s + 2.5 m gap equals the current gap (traffic/highway_flow_rl_v03_rewards.required_gap), and
the per-car check needs a 1.5 s minimum headway, so the gates accept gaps from 1.5 s x speed to about
1.5 s x speed + 5.5 m. The expert's IDM keeps 1.8 s + 4 m; that is a driver-side finding.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_regrade.py summary EXPERT_RUN PHASE1_RUN OUT   parity, then each rule alone and all
                                                                        together, for every driver; writes
                                                                        OUT/rebaseline.json and OUT/rebaseline.md
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic import highway_flow_rl_v14_follow_gate as F  # noqa: E402

SCHEMA = 'highway_merge_v15_gate_rebaseline_v01'
RULES = ('every_merge', 'restart_at_stop_line', 'eligible_samples_only')
WINDOW_S = 8.                    # v04's quality_window_s
TOLERANCE_M_S = 2.               # v04's speed-match tolerance (F.SETTINGS['tolerance_m_s'])
STOPPING_BOUNDS = ('occupied_lane_limit', 'occupied_leader', 'ending_lane_runway')
ROW_SET, BACKGROUND = 'v15-development', 'idm'


class ParityMismatch(RuntimeError):
    """The re-grade without rules does not reproduce the recorded episode."""


def check_rules(rules):
    rules = tuple(rules)
    unknown = [r for r in rules if r not in RULES]
    if unknown:
        raise ValueError(f'Unknown re-baseline rules: {unknown}')
    return rules


# --------------------------------------------------------------------------- reading an episode

def step_of(entry):
    """One car's recorded decision as the fields the re-grade uses (None without a quality sample)."""
    audit = entry.get('v04_reward_audit') or {}
    sample = audit.get('quality_sample')
    pre = audit.get('pre_action')
    if not sample or not pre:
        return None
    target = pre.get('target') or {}
    ego = pre.get('ego_state') or {}
    return dict(t=sample['time_s'], speed=audit.get('measured_speed_m_s'), sample=dict(sample),
                pre_time=pre['time_s'], pre_speed=ego.get('speed'), pre_acceleration=ego.get('acceleration', 0.),
                eligible=bool(target.get('eligible')), target=target.get('target_m_s'),
                bounds=list(target.get('bounds') or ()), follow=target.get('follow_gate'),
                completed=bool(entry.get('merge_completed')), status=entry.get('stable_merge_status'),
                drive=(entry.get('acceleration_bounds') or {}).get('drive_grade'))


def read_steps(trace, agents):
    """{agent: [step, ...]} from an action journal (a path to actions.jsonl.gz, or an iterable of decoded lines)."""
    agents = set(agents)
    steps = {a: [] for a in agents}
    if isinstance(trace, (str, Path)):
        with gzip.open(trace, 'rt', encoding='utf-8') as handle:
            lines = [json.loads(line) for line in handle]
    else:
        lines = list(trace)
    for line in lines:
        for agent in agents:
            entry = (line.get('agents') or {}).get(agent)
            step = step_of(entry) if entry else None
            if step is not None:
                steps[agent].append(step)
    return steps


def window(steps):
    """The graded window's step indices: the environment's history at the stable merge, i.e. samples with time at
    least (merge completion - 8 s) up to the first decision whose stable-merge status is no longer pending."""
    stable = next((i for i, s in enumerate(steps) if s['status'] not in (None, 'pending')), None)
    if stable is None:
        return None
    done = next((s['t'] for s in steps[:stable + 1] if s['completed']), None)
    start = (done if done is not None else steps[stable]['t']) - WINDOW_S
    return [i for i in range(stable + 1) if steps[i]['t'] >= start]


# --------------------------------------------------------------------------- the reference band

def stopping_ceiling(step):
    """R1's at-once cap: the lowest occupied-lane, occupied-leader or ending-lane bound of the recorded target."""
    values = [b['ceiling_m_s'] for b in step['bounds']
              if b.get('reason') in STOPPING_BOUNDS and F.finite(b.get('ceiling_m_s'))]
    return min(values) if values else None


def bands(steps, rules=(), first=None):
    """Per step: (lower, upper) of the follow band the step is graded on, or None (graded on the plain target).

    Yield-entry cars (any step with a follow audit) replay the environment's reference from its recorded inputs;
    with restart_at_stop_line it also restarts at the stop-line crossing. Without a rule the replay equals the
    recorded reference (checked by replay_parity). Cars without a yield entry get a band only with every_merge,
    from the first eligible step at or after index `first` (the graded window's start).
    """
    rules = check_rules(rules)
    out = [None] * len(steps)
    if any(s['follow'] for s in steps):
        reference, previous = None, None
        for i, s in enumerate(steps):
            audit = s['follow']
            if audit is None:
                continue
            road, ceiling, rise = audit['road_target_m_s'], audit['ceiling_m_s'], audit['rise_m_s2']
            crossed = ('restart_at_stop_line' in rules and previous is not None and previous['yield_entry']
                       and not audit['yield_entry'])
            if audit['started'] or reference is None or crossed:
                reference = F.start(s['pre_speed'], s['pre_acceleration'], ceiling, rise, s['pre_time'])
            else:
                reference, _ = F.advance(reference, road, ceiling, rise, s['pre_speed'], s['pre_acceleration'],
                                         s['pre_time'])
            out[i] = F.band(reference['value'], road)
            previous = audit
            if audit['ended']:
                reference, previous = None, None
        return out
    if 'every_merge' not in rules or first is None:
        return out
    reference = None
    for i in range(first, len(steps)):
        s = steps[i]
        if not s['eligible'] or not F.finite(s['target']):
            continue
        ceiling, rise = stopping_ceiling(s), F.rise_rate(s['drive'])
        if reference is None:
            reference = F.start(s['pre_speed'], s['pre_acceleration'], ceiling, rise, s['pre_time'])
        else:
            reference, _ = F.advance(reference, s['target'], ceiling, rise, s['pre_speed'], s['pre_acceleration'],
                                     s['pre_time'])
        out[i] = F.band(reference['value'], s['target'])
    return out


def replay_parity(steps):
    """The largest |rebuilt - recorded| follow reference of a car (0 for a car without a yield entry)."""
    worst, reference = 0., None
    for s in steps:
        audit = s['follow']
        if audit is None:
            continue
        road, ceiling, rise = audit['road_target_m_s'], audit['ceiling_m_s'], audit['rise_m_s2']
        if audit['started'] or reference is None:
            reference = F.start(s['pre_speed'], s['pre_acceleration'], ceiling, rise, s['pre_time'])
        else:
            reference, _ = F.advance(reference, road, ceiling, rise, s['pre_speed'], s['pre_acceleration'], s['pre_time'])
        worst = max(worst, abs(reference['value'] - audit['reference_m_s']))
        if audit['ended']:
            reference = None
    return worst


def speed_match(step, band):
    """v04's sample check: an eligible target and |speed - target| <= 2 m/s, on the band's nearest point."""
    speed = step['speed']
    if speed is None or not step['eligible']:
        return False
    if band is not None:
        lower, upper = band
        return lower - TOLERANCE_M_S <= speed <= upper + TOLERANCE_M_S
    return step['target'] is not None and abs(speed - step['target']) <= TOLERANCE_M_S


# --------------------------------------------------------------------------- grading

def grade(steps, recorded, rules=()):
    """One car's rolling merge quality under `rules`, in the environment's quality_results form.

    recorded: the episode's quality_merges entry for the car (its approach stops are taken from it).
    """
    rules = check_rules(rules)
    indices = window(steps)
    if indices is None:
        raise ParityMismatch('A graded car has no stable merge in its trace')
    graded = bands(steps, rules, first=indices[0])
    history = [steps[i]['sample'] for i in indices]
    matches = [speed_match(steps[i], graded[i]) for i in indices]
    counted = [m for i, m in zip(indices, matches) if steps[i]['eligible']] if 'eligible_samples_only' in rules \
        else matches
    fraction = sum(counted) / len(counted) if counted else 0.
    approach = recorded['full_approach_stop_seconds']
    others = (sum(s['stopped'] for s in history) * .1 <= 1. + 1e-8
              and all(s['jerk'] is not None and abs(s['jerk']) <= 5. for s in history)
              and sum(s['override'] for s in history) / len(history) <= .05
              and all(s['neighbor_acceleration'] is None or s['neighbor_acceleration'] >= -3. for s in history))
    enough = bool(counted) and len(counted) * 2 >= len(history)
    qualified = bool(approach <= 1. + 1e-8 and others and enough and fraction >= .9)
    return dict(qualified=qualified, samples=len(history), counted_samples=len(counted),
                eligible_samples=sum(1 for i in indices if steps[i]['eligible']),
                speed_match_fraction=fraction, stop_seconds=sum(s['stopped'] for s in history) * .1,
                full_approach_stop_seconds=approach, rules=list(rules),
                recorded_speed_matches=sum(s['speed_match'] for s in history),
                scope='Re-graded from the recorded trace (experiments/v15_regrade.py)')


def episode_steps(directory, report):
    """{agent: [step, ...]} of an episode's graded cars ({} when it grades none)."""
    directory = Path(directory)
    quality = report.get('quality_merges') or {}
    if not quality:
        return {}
    traces = sorted(directory.glob('episode-*/actions.jsonl.gz'))
    if len(traces) != 1:
        raise ParityMismatch(f'Expected one trace in {directory}')
    return read_steps(traces[0], quality)


def check_parity(steps, report, where=''):
    """Without a rule the re-grade reproduces every recorded speed match, window, fraction and qualified flag, and
    every follow reference is rebuilt; returns counts, or raises ParityMismatch."""
    quality = report.get('quality_merges') or {}
    parity = dict(cars=len(quality), steps=sum(len(s) for s in steps.values()), worst_reference=0.)
    for agent, recorded in quality.items():
        car = steps[agent]
        worst = replay_parity(car)
        parity['worst_reference'] = max(parity['worst_reference'], worst)
        if worst > 1e-9:
            raise ParityMismatch(f'{where} {agent}: a follow reference is not rebuilt ({worst})')
        for s in car:
            band = None if s['follow'] is None else (s['follow']['lower_m_s'], s['follow']['upper_m_s'])
            if speed_match(s, band) != bool(s['sample']['speed_match']):
                raise ParityMismatch(f'{where} {agent}: the recorded speed match at {s["t"]} is not reproduced')
        base = grade(car, recorded)
        if (base['samples'] != recorded['samples'] or base['qualified'] != recorded['qualified']
                or abs(base['speed_match_fraction'] - recorded['speed_match_fraction']) > 1e-12):
            raise ParityMismatch(f'{where} {agent}: window or quality not reproduced ({base["samples"]}/'
                                 f'{recorded["samples"]}, {base["speed_match_fraction"]}/{recorded["speed_match_fraction"]})')
    return parity


def episode_quality(steps, report, rules=()):
    """{agent: grade} for every graded car of an episode under `rules`."""
    return {agent: grade(steps[agent], recorded, rules)
            for agent, recorded in (report.get('quality_merges') or {}).items()}


_GATES = []


def gates():
    """Round 24's installed gates: the function the worker pool calls (experiments/v15_pool._worker_setup installs
    the v14 runner, whose chain wraps run_highway_flow_rl_v05.ruled_gates with the later checks)."""
    if not _GATES:
        from experiments import run_highway_flow_rl_v12 as runner
        from experiments import run_highway_flow_rl_v14 as runner14
        runner.install = runner14.install
        runner.install()
        _GATES.append(runner.base.ruled_gates)
    return _GATES[0]


def verdict(report, quality):
    """Round 24's gates on the episode with its quality merges replaced."""
    return gates()(dict(report, quality_merges=quality))


# --------------------------------------------------------------------------- runs

def first_records(out):
    """{driver: {row: record}}: each driver's first record per development row (the Phase 1/2 reports' choice)."""
    records = {}
    for line in (Path(out) / 'results.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        f = record['fields']
        if f.get('row_set') != ROW_SET or f.get('background') != BACKGROUND:
            continue
        records.setdefault(f['driver'], {}).setdefault(f['row'], record)
    return records


def regrade_run(out, variants):
    """Every driver's rows of a run under each variant (name -> rules), after the no-rule parity check."""
    out = Path(out)
    result = {}
    parity = dict(cars=0, steps=0, episodes=0, worst_reference=0., verdicts=0, fewest_eligible=None)
    for driver, rows in first_records(out).items():
        per = {name: {} for name in variants}
        failures = {name: {} for name in variants}
        recorded = {}
        for row, record in sorted(rows.items()):
            directory = out / 'episodes' / record['key'][:16] / record['attempt']
            report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
            steps = episode_steps(directory, report)
            check = check_parity(steps, report, where=str(directory))
            gate = verdict(report, episode_quality(steps, report))
            if bool(gate['passed']) != bool(record['passed']) or list(gate['failures']) != list(record['failures']):
                raise ParityMismatch(f'{directory}: the gates do not reproduce the recorded verdict')
            parity['cars'] += check['cars']
            parity['steps'] += check['steps']
            parity['episodes'] += 1
            parity['verdicts'] += 1
            parity['worst_reference'] = max(parity['worst_reference'], check['worst_reference'])
            recorded[row] = bool(record['passed'])
            for name, rules in variants.items():
                quality = episode_quality(steps, report, rules)
                for agent, g in quality.items() if name == 'none' else ():
                    share = g['eligible_samples'] / g['samples']
                    if parity['fewest_eligible'] is None or share < parity['fewest_eligible']['share']:
                        parity['fewest_eligible'] = dict(share=share, eligible=g['eligible_samples'],
                                                         samples=g['samples'], driver=driver, row=row, agent=agent)
                gate = verdict(report, quality)
                per[name][row] = bool(gate['passed'])
                failures[name][row] = list(gate['failures'])
        result[driver] = dict(recorded=recorded, passed=per, failures=failures)
    return result, parity


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summary(expert_run, phase1_run, out):
    """The re-baseline: parity, then each rule alone and all three, for the expert's run and Phase 1's drivers."""
    from traffic import v15_stats as S
    variants = dict(none=(), **{rule: (rule,) for rule in RULES}, rebaselined=RULES)
    runs = {}
    for label, path in (('expert', expert_run), ('phase1', phase1_run)):
        drivers, parity = regrade_run(path, variants)
        runs[label] = dict(path=str(Path(path)), parity=parity, drivers=drivers)
    expert_id = next(iter(runs['expert']['drivers']))
    round16 = 'policy:f0b11fa577b33763'
    rows = sorted(runs['expert']['drivers'][expert_id]['recorded'])
    table = {}
    for label, run in runs.items():
        for driver, d in run['drivers'].items():
            if label == 'expert' and driver != expert_id:
                continue                      # the run's parity task (round-16 on one row) is not a driver here
            table[driver] = {name: S.rate([d['passed'][name][r] for r in sorted(d['passed'][name])]) for name in variants}
            table[driver]['recorded'] = S.rate([d['recorded'][r] for r in sorted(d['recorded'])])
    e = runs['expert']['drivers'][expert_id]
    b = runs['phase1']['drivers'][round16]
    paired = {name: S.paired([e['passed'][name][r] for r in rows], [b['passed'][name][r] for r in rows])
              for name in variants}
    changes = {}
    for driver, d in [(expert_id, e)] + [(k, v) for k, v in runs['phase1']['drivers'].items()]:
        changes[driver] = [dict(row=r, recorded=d['recorded'][r], rebaselined=d['passed']['rebaselined'][r],
                                by_rule=[rule for rule in RULES if d['passed'][rule][r] != d['recorded'][r]],
                                failures=d['failures']['rebaselined'][r])
                           for r in sorted(d['recorded']) if d['passed']['rebaselined'][r] != d['recorded'][r]]
    result = dict(schema=SCHEMA, rules=list(RULES), module_sha256=_sha(__file__),
                  follow_gate_sha256=_sha(F.__file__),
                  gates_sha256=_sha(ROOT / 'experiments/run_highway_flow_rl_v05.py'),
                  simulator='none: re-graded from recorded SUMO (libsumo) traces; no simulation, no physics',
                  rows='v15 development rows; held-out rows not built or run',
                  parity={k: v['parity'] for k, v in runs.items()}, pass_rates=table,
                  paired_expert_vs_round16=paired, changed_rows=changes, expert=expert_id,
                  remaining_expert_failures={r: e['failures']['rebaselined'][r] for r in rows
                                             if not e['passed']['rebaselined'][r]})
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'rebaseline.json').write_text(json.dumps(result, indent=1), encoding='utf-8')
    (out / 'rebaseline.md').write_text(_markdown(result), encoding='utf-8')
    return result


def _markdown(result):
    lines = ['# v15 gate re-baseline (decision 3)', '',
             f"Rules: {', '.join(result['rules'])}. Re-graded from recorded traces; no simulation.", '',
             '| Driver | Recorded | ' + ' | '.join(['none'] + list(RULES) + ['rebaselined']) + ' |',
             '|---|---|' + '---|' * (len(RULES) + 2)]
    for driver, rates in result['pass_rates'].items():
        cells = [f"{rates[k]['passed']}/{rates[k]['rows']}" for k in ['none'] + list(RULES) + ['rebaselined']]
        lines.append(f"| {driver} | {rates['recorded']['passed']}/{rates['recorded']['rows']} | " + ' | '.join(cells) + ' |')
    lines += ['', '## Expert against the round-16 policy (paired, same rows)', '']
    for name, p in result['paired_expert_vs_round16'].items():
        lines.append(f"- {name}: both {p['both']}, expert only {p['first_only']}, round-16 only {p['second_only']}, "
                     f"neither {p['neither']}; {100 * p['difference']:+.1f} points ({100 * p['low']:+.1f} to "
                     f"{100 * p['high']:+.1f}), McNemar p = {p['mcnemar_p']:.3g}")
    lines += ['', '## Parity', '']
    for label, p in result['parity'].items():
        lines.append(f"- {label}: {p['episodes']} episodes, {p['cars']} graded cars, {p['steps']} steps; worst "
                     f"rebuilt reference difference {p['worst_reference']}; every verdict reproduced")
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    s = sub.add_parser('summary')
    s.add_argument('expert_run')
    s.add_argument('phase1_run')
    s.add_argument('out')
    args = parser.parse_args(argv)
    if args.command == 'summary':
        result = summary(args.expert_run, args.phase1_run, args.out)
        print((Path(args.out) / 'rebaseline.md').read_text(encoding='utf-8'))
        return result


if __name__ == '__main__':
    main()
