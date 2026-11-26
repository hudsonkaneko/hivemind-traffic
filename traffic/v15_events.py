"""v15 event metrics: each stop, cut-in, emergency, collision and override episode counted once.

v15 plan, "Events first": one stuck car used to count up to 100 times as interventions, once per
decision. Here a run of consecutive decisions (one simulation step apart) in the same condition
is one event, and the per-decision fractions stay as secondary numbers.

Inputs are what a v14 episode already records: the report (report.json, or a training round's
validation reports) and the per-decision trace (actions.jsonl.gz: one line per step with `time`
and `agents`). Nothing here changes a v14 definition:
- stops use the v14 stop audit per decision (`v14_stop`: `stopped` below 0.1 m/s, and
  `free_road` = stopped with no leader within 20 m and a target of at least 5 m/s;
  report['v14_stop_settings']). A stop event is a run of stopped decisions; it is a free-road
  stop event if any of its decisions is free-road (so a leader passing within 20 m mid-stop
  does not split one stop into two events). Every stop counts, however short: the user decided
  (November 25, plan v15 decision 2) that a stuck car pulls away automatically and that every
  such stop still counts as an event. The pull-away belongs to the Phase 2 rule executor, and no
  threshold for it has been set, so each stop event carries its duration and `stops_longer_than`
  counts any threshold on request;
- override episodes use the per-decision `override` flag (the v14 intervention definition,
  report['override_metric']);
- cut-ins use report['v14_cut_in']['events'] (one row per charged decision, per follower);
- emergencies use SUMO's own warnings, report['native_emergencies']['events'];
- collisions: the v05 step ends every agent on the first step with any colliding vehicle
  (environments/highway_environment_v05.py, `terminated`), and report['collisions'] counts
  colliding vehicles per step (environments/i110_merge_env.py, `_advance`). So an episode has
  at most one collision event, and the vehicle count stays secondary.
"""
from __future__ import annotations

import gzip
import json
from collections import Counter
from pathlib import Path

SCHEMA = 'highway_merge_v15_events_v01'
_TOLERANCE = 1e-6


def step_seconds(trace):
    """The simulation step, read from the trace's own times (the most common spacing)."""
    times = sorted({round(float(line['time']), 6) for line in trace})
    gaps = Counter(round(b - a, 6) for a, b in zip(times, times[1:]))
    if not gaps:
        raise ValueError('A trace needs at least two steps to give its step length')
    return gaps.most_common(1)[0][0]


def runs(times, step):
    """Group sorted times into runs whose neighbours are one step apart: [(start, end, count)]."""
    out = []
    for time in sorted(times):
        if out and time - out[-1][1] <= step + _TOLERANCE:
            out[-1] = (out[-1][0], time, out[-1][2] + 1)
        else:
            out.append((time, time, 1))
    return out


def _per_agent_runs(trace, step, test):
    times = {}
    for line in trace:
        for agent, decision in line['agents'].items():
            if test(decision):
                times.setdefault(agent, []).append(float(line['time']))
    events = []
    for agent, values in sorted(times.items()):
        for start, end, count in runs(values, step):
            events.append(dict(agent=agent, start_s=round(start, 6), end_s=round(end, 6), decisions=count,
                               seconds=round(count * step, 6)))
    return events


def _stop(decision, key):
    audit = decision.get('v14_stop') or {}
    return bool(audit.get(key))


def _grouped(rows, step, key_fields, time_field):
    grouped = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in key_fields), []).append(float(row[time_field]))
    events = []
    for key, values in sorted(grouped.items()):
        for start, end, count in runs(values, step):
            events.append(dict(zip(key_fields, key), start_s=round(start, 6), end_s=round(end, 6), decisions=count))
    return events


def episode_events(report, trace):
    """Events for one episode, from its report and its per-decision trace."""
    trace = list(trace)
    step = step_seconds(trace)
    decisions = sum(len(line['agents']) for line in trace)
    stops = _per_agent_runs(trace, step, lambda d: _stop(d, 'stopped'))
    free_times = {}
    for line in trace:
        for agent, decision in line['agents'].items():
            if _stop(decision, 'free_road'):
                free_times.setdefault(agent, []).append(float(line['time']))
    for event in stops:
        inside = [t for t in free_times.get(event['agent'], ())
                  if event['start_s'] - _TOLERANCE <= t <= event['end_s'] + _TOLERANCE]
        event['free_road_decisions'] = len(inside)
        event['free_road_seconds'] = round(len(inside) * step, 6)
    free_road = [event for event in stops if event['free_road_decisions']]
    overrides = _per_agent_runs(trace, step, lambda d: bool(d.get('override')))
    for event in overrides:
        causes = set()
        for line in trace:
            decision = line['agents'].get(event['agent'])
            if decision and decision.get('override') and event['start_s'] - _TOLERANCE <= float(line['time']) <= event['end_s'] + _TOLERANCE:
                causes.update(decision.get('override_causes') or ())
        event['causes'] = sorted(causes)
    cut_in = report.get('v14_cut_in') or {}
    cut_ins = _grouped(cut_in.get('events') or [], step, ('car', 'follower'), 'time_s')
    native = report.get('native_emergencies') or {}
    emergencies = _grouped(native.get('events') or [], step, ('vehicle_id', 'controlled'), 'time_s')
    colliding_vehicles = int(report.get('collisions') or 0)
    override_decisions = sum(event['decisions'] for event in overrides)
    return dict(
        schema=SCHEMA, case_id=report.get('case_id'), seed=report.get('seed'), step_s=step, decisions=decisions,
        events=dict(
            stops=len(stops), free_road_stops=len(free_road), cut_ins=len(cut_ins),
            emergencies=len(emergencies),
            controlled_emergencies=sum(1 for event in emergencies if event['controlled']),
            collisions=int(colliding_vehicles > 0), override_episodes=len(overrides)),
        secondary=dict(
            override_decisions=override_decisions,
            override_fraction=override_decisions / decisions if decisions else None,
            report_intervention_fraction=report.get('intervention_fraction'),
            free_road_stop_decisions=sum(len(v) for v in free_times.values()),
            cut_in_charged_decisions=cut_in.get('charged'),
            emergency_warning_lines=native.get('unique_events'),
            colliding_vehicle_steps=colliding_vehicles),
        detail=dict(stops=stops, free_road_stops=free_road, cut_ins=cut_ins, emergencies=emergencies,
                    override_episodes=overrides))


def stops_longer_than(result, seconds, *, free_road_only=True):
    """Stop events lasting longer than `seconds` (a reporting threshold, not a control rule)."""
    events = result['detail']['free_road_stops' if free_road_only else 'stops']
    return sum(1 for event in events if event['seconds'] > seconds + _TOLERANCE)


def pool(results):
    """Totals over episodes: event counts, the episodes with each kind of event, and decisions."""
    results = list(results)
    kinds = sorted({k for r in results for k in r['events']})
    return dict(schema=SCHEMA, episodes=len(results), decisions=sum(r['decisions'] for r in results),
                events={k: sum(r['events'][k] for r in results) for k in kinds},
                episodes_with={k: sum(1 for r in results if r['events'][k]) for k in kinds})


def read_trace(path):
    with gzip.open(Path(path), 'rt') as handle:
        return [json.loads(line) for line in handle]
