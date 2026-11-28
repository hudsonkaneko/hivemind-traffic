"""The stream merges, questioned (decision 14; asked by the main Claude session on November 27, 2026): offline, from
saved SUMO records only. Nothing is simulated or charged; every result is post hoc.

For the 12 stream cars (9 development rows; the situations of experiments/v15_expert_v4_replay):
- **Who drives the stream.** Each mainline car that reaches our car's shared lane, as the replay reconstructs the
  stream, is ours (a controlled agent in the action records and `routes.rou.xml`) or SUMO's (a native car).
- **Is there a gap?** Each gap in that stream is judged by expert v3's own gap test (`select_gap`: the car behind
  the gap may brake at most `entry_safe_m_s2`, 2.0 m/s^2, by its own SUMO IDM, and the car ahead must have passed
  `yield_headway_s`, 1 s, before) at its best case:
  - the earliest crossing the gap allows (1 s after the car ahead, and no sooner than the full run-up from the car's
    first decision);
  - the highest speed the car could have crossing the line at exactly that time, from its first decision: brake at
    once (at most the environment's 4.5 m/s^2), or stop and wait, then run up at v3's 1.5 m/s^2 to the line, up to
    the free-road speed; and no more than the curve cap at the line (expert v4's change 4) in the second variant.
    Curves before the line, the ramp car ahead and the 0.5 m/s crawl are ignored, so this is an upper bound.
  No driver that keeps v3's 2.0 m/s^2 rule and its run-up could take a gap that fails this. The same is repeated at
  `entry_abort_m_s2`, 3.0 m/s^2 (the merge gate's limit, which v3 uses only to hold a started commit).
- **How open the loop is.** The SUMO mainline cars as round 16's ramp car saw them (Phase 1 run 1, a driver that
  enters much sooner) against the same cars as expert v3's saw them (Phase 2 run 3), at the same instants before
  either car crossed its stop line: their distance to the stop point and their speed (our car's speed is the one
  at the instant of its neighbour readings, the previous record's measured speed; overlapping readings, a bumper gap
  clamped at 0, are left out). Then what SUMO logged once
  round 16's car crossed (emergency braking and junction collisions, `sumo-errors.log`), beside the braking the
  open-loop stream says the car behind would have needed at that crossing.
- **What the next steps would cost**, from recorded decisions with the re-check's own sizing
  (experiments/v15_expert_v4_recheck.charge).

Usage: python experiments/v15_stream_gaps.py   (writes documentation/evidence/v15-stream-gaps/)
"""
import collections
import gzip
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_expert_v4_replay as X             # noqa: E402
from experiments import v15_runner as R                       # noqa: E402
from traffic import v15_rule_expert as v3                     # noqa: E402

SCHEMA = 'highway_merge_v15_stream_gaps_v01'
OUT = 'documentation/evidence/v15-stream-gaps'
LABEL = ('offline reading of saved SUMO records (expert v3: Phase 2 run 3; round 16: Phase 1 run 1) and of the '
         'stream the offline replay reconstructs from them; nothing simulated; post hoc')
ROUND16_RUN, ROUND16 = 'outputs/highway-merge-v15/phase-1/run-01', 'policy:f0b11fa577b33763'
DT = X.DT
HORIZON_S = 90.
RUN_UP, BRAKE = v3.SETTINGS['run_up_m_s2'], -v3.ACCELERATION_MIN     # v3's run-up; the environment's hardest braking
SAME_M, SAME_M_S = .01, .01                                         # "the same car state" in both records
SAFE = {'entry_safe': v3.SETTINGS['entry_safe_m_s2'], 'gate_limit': v3.SETTINGS['entry_abort_m_s2']}
NUMBER = r'(-?\d+(?:\.\d+)?)'
EMERGENCY = re.compile(rf"Vehicle '([^']+)' performs emergency braking on lane '([^']+)' with decel={NUMBER}, "
                       rf"wished={NUMBER}, severity={NUMBER}, time={NUMBER}")
COLLISION = re.compile(rf"Vehicle '([^']+)'; (\w+) collision with vehicle '([^']+)', lane='([^']+)', "
                       rf"gap={NUMBER}.*time={NUMBER}")


# --------------------------------------------------------------------------- pure pieces

def passing(tracks, vid, horizon=HORIZON_S):
    """(passing time at the stop point, speed then, time of the last observation) of a reconstructed mainline car: the
    first instant whose estimated passing time is not later than the instant itself, else the last estimate."""
    t = 0.
    while t <= horizon + 1e-9:
        p, v = tracks.state(vid, t)
        if p <= t + 1e-9:
            return round(p, 2), round(tracks.state(vid, max(p, 0.))[1], 2), tracks.p[vid][-1][0]
        t = round(t + DT, 1)
    p, v = tracks.state(vid, horizon)
    return round(p, 2), round(v, 2), tracks.p[vid][-1][0]


def needed_speed(rear_speed, behind, params, safe, top=40.):
    """The lowest speed at the line for which the car `behind` m behind us at `rear_speed` would brake at most `safe`
    (v3's background_response, the car's own IDM, free term 0 as v3 computes it); None if no speed up to `top` will
    do, 0.0 if any will."""
    if behind <= 0.:
        return None

    def ok(u):
        return v3.background_response(rear_speed, u, behind, max(rear_speed, 1.), params) >= -safe - 1e-12
    if ok(0.):
        return 0.
    if not ok(top):
        return None
    lo, hi = 0., top
    for _ in range(60):
        mid = (lo + hi) / 2.
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return math.ceil(hi * 1000.) / 1000.                       # rounded up, so the speed still passes


def line_speed_max(v0, distance, T, cap, a=RUN_UP, b=BRAKE):
    """The highest speed a car at `v0`, `distance` m from the line, can have crossing it exactly `T` s from now: brake
    at once to some speed w (at most `b`), then run up to `cap` (v3's run_up, at run_up_m_s2 = `a`), or stop and
    wait, then run up. None if it cannot reach the line by T."""
    if abs(a - v3.SETTINGS['run_up_m_s2']) > 1e-12:
        raise ValueError('v3.run_up accelerates at run_up_m_s2')
    go, top = v3.run_up(v0, distance, cap)
    if T < go - 1e-9:
        return None

    def profile(w):
        rest = distance - (v0 * v0 - w * w) / (2. * b)
        t_up, u = v3.run_up(w, max(rest, 0.), cap)
        return (v0 - w) / b + t_up, u
    floor = math.sqrt(max(0., v0 * v0 - 2. * b * distance))         # braking all the way to the line
    t_floor, u_floor = profile(floor)
    if T >= t_floor - 1e-9:
        return round(u_floor, 3)                # stopped: wait, then run up (or, braking to the line, its speed)
    lo, hi = floor, v0                                              # t(w) falls as w rises
    for _ in range(80):
        mid = (lo + hi) / 2.
        lo, hi = (mid, hi) if profile(mid)[0] > T else (lo, mid)
    return round(profile(hi)[1], 3)


def gaps(stream, earliest, length, best, params, s=v3.SETTINGS):
    """Every gap of a stream [(passing time, speed, id)] judged by v3's gap test at its best case: crossing at the
    earliest the gap allows (`earliest`, and yield_headway_s after the car ahead) at the highest line speed then,
    `best` {name: function of the crossing time}. Returns [dict(ahead, rear, opens, closes, headway_s, T, behind_m,
    best: {name: speed}, needed: {safe: speed}, ok: {safe: {name: bool}})]; the last gap has no rear car and is
    always open."""
    cars = sorted(stream)
    out = []
    for i in range(len(cars) + 1):
        ahead = cars[i - 1] if i else None
        rear = cars[i] if i < len(cars) else None
        T = max(earliest, ahead[0] + s['yield_headway_s'] if ahead else -math.inf)
        speeds = {n: f(T) for n, f in best.items()}
        row = dict(ahead=ahead and ahead[2], rear=rear and rear[2], opens=ahead and ahead[0], closes=rear and rear[0],
                   headway_s=round(rear[0] - ahead[0], 2) if ahead and rear else None, T=round(T, 2), best=speeds)
        if rear is None:
            row.update(behind_m=None, needed={k: 0. for k in SAFE}, ok={k: {n: True for n in best} for k in SAFE})
        else:
            behind = rear[1] * (rear[0] - T) - length
            need = {k: needed_speed(rear[1], behind, params.get(rear[2]), safe) for k, safe in SAFE.items()}
            row.update(behind_m=round(behind, 2), needed=need,
                       ok={k: {n: need[k] is not None and u is not None and need[k] <= u + 1e-9
                               for n, u in speeds.items()} for k in SAFE})
        out.append(row)
    return out


def forced_below(distance, wait, speed=5.):
    """The least time a car `distance` m from the line must spend below `speed` if it may not cross for `wait` s:
    above that speed it covers more than `speed` m each second, so at most distance / speed s of the wait can be
    spent there."""
    return round(max(0., wait - distance / speed), 2)


def first_open(rows, safe, name):
    """The first gap acceptable under `safe` at the line speed `name`: (its crossing time, its rear car)."""
    for g in rows:
        if g['ok'][safe][name]:
            return g['T'], g['rear']
    return None, None


# --------------------------------------------------------------------------- records

def episode_of(run, driver, row):
    for line in (ROOT / run / 'results.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        f = r['fields']
        if (f['driver'] == driver and f['row_set'] == X.ROW_SET and f['background'] == X.BACKGROUND
                and r['attempt'] == X.ATTEMPT and f['row'] == row):
            found = sorted((ROOT / run / 'episodes' / r['key'][:16] / X.ATTEMPT).glob('episode-*'))
            if len(found) != 1:
                raise ValueError(f'{run} row {row}: {len(found)} episodes')
            return r, found[0]
    raise ValueError(f'{run}: no {driver} record for row {row}')


def actions(episode):
    with gzip.open(Path(episode) / 'actions.jsonl.gz', 'rt', encoding='utf-8') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def vehicles(episode):
    root = ET.parse(Path(episode) / 'routes.rou.xml').getroot()
    return {v.get('id'): dict(type=v.get('type'), route=v.get('route'), lane=v.get('departLane'),
                              length=float({t.get('id'): t for t in root.iter('vType')}[v.get('type')].get('length')))
            for v in root.iter('vehicle')}


def sumo_log(episode):
    """SUMO's emergency-braking and collision warnings."""
    path = Path(episode) / 'sumo-errors.log'
    text = path.read_text(encoding='utf-8', errors='replace') if path.is_file() else ''
    out = []
    for line in text.splitlines():
        if m := EMERGENCY.search(line):
            out.append(dict(kind='emergency braking', vehicle=m[1], lane=m[2], decel_m_s2=float(m[3]),
                            wished_m_s2=float(m[4]), t=float(m[6])))
        elif m := COLLISION.search(line):
            out.append(dict(kind=f'{m[2]} collision', vehicle=m[1], other=m[3], lane=m[4], gap_m=float(m[5]),
                            t=float(m[6])))
    return out


def seen_mainline(records, approach, lengths, ours, ramps):
    """{(SUMO mainline car, instant): (distance to the stop point, speed)} from every ramp car's entry neighbours on
    the approach (the decision's pre-action state, as the replay reads it), and {ramp car: crossing (instant, speed)}:
    the first state after its last active entry."""
    seen, crossing, last, previous = {}, {}, {}, {}
    for r in records:
        tau = X._t(r['time'] - DT)
        for agent, a in r['agents'].items():
            e = ((a.get('flow_context') or {}).get('entry') or {})
            speed = previous.get(agent)                 # the record's measured speed is its post-action one
            previous[agent] = (a.get('v04_reward_audit') or {}).get('measured_speed_m_s')
            active = e.get('entry_active') and (e.get('source_lane'), e.get('target_lane')) == approach
            if agent in last and not active and agent not in crossing:
                crossing[agent] = (tau, speed)
            if not active or speed is None:
                continue
            last[agent] = tau
            stop = e['distance_to_start_m']
            for kind in ('front', 'rear'):
                x = e.get(kind)
                if (not x or x['id'] in ours or x['id'] in ramps or x['id'].startswith('ramp')
                        or x['bumper_gap_m'] <= 0.):
                    continue
                if kind == 'rear':
                    d = x['bumper_gap_m'] + lengths.get(agent, 4.8) + stop
                else:
                    d = stop - x['bumper_gap_m'] - lengths.get(x['id'], 4.8)
                seen.setdefault((x['id'], tau), (d, speed + x['relative_speed_m_s']))
    return seen, crossing


def ramp_leader(records, agent, approach, lengths, crossings, ramps):
    """The car ahead of `agent` on the ramp at its first active entry (its own-lane front), and when that car crossed
    the stop line in these records: our cars' own crossing (`crossings`), else the first reading that puts the
    leader's front at or past the line (our stop distance less the bumper gap and its length). (None, None) if
    there is none."""
    leader, crossed = None, None
    for r in records:
        tau = X._t(r['time'] - DT)
        a = r['agents'].get(agent)
        if not a:
            continue
        fc = a.get('flow_context') or {}
        e = fc.get('entry') or {}
        if not (e.get('entry_active') and (e.get('source_lane'), e.get('target_lane')) == approach):
            continue
        front = ((fc.get('corridors') or {}).get('own') or {}).get('front')
        if leader is None:
            if not front or not (front['id'].startswith('ramp') or front['id'] in ramps):
                return None, None                      # none, or a mainline car beyond the junction
            leader = front['id']
            if leader in crossings:
                return leader, crossings[leader][0]
        if front and front['id'] == leader:
            if e['distance_to_start_m'] - front['bumper_gap_m'] - lengths.get(leader, 4.8) <= 0.:
                return leader, tau
        elif crossed is None:
            return leader, tau                         # no longer our front: it has gone
    return leader, crossed


def compare(a, b, before):
    """Matched observations of the same SUMO car at the same instant before `before`: count, largest differences."""
    keys = sorted(k for k in set(a) & set(b) if k[1] < before - 1e-9)
    if not keys:
        return dict(matched=0, cars=0, max_distance_m=None, max_speed_m_s=None, last_t=None, same=None)
    dd = max(abs(a[k][0] - b[k][0]) for k in keys)
    dv = max(abs(a[k][1] - b[k][1]) for k in keys)
    return dict(matched=len(keys), cars=len({k[0] for k in keys}), max_distance_m=round(dd, 4),
                max_speed_m_s=round(dv, 4), last_t=max(k[1] for k in keys), same=dd <= SAME_M and dv <= SAME_M_S)


# --------------------------------------------------------------------------- per row

def row_result(job):
    row, record, cars, replay = job
    import ctypes
    if hasattr(ctypes, 'windll'):
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x4000)
    ep3 = X.episode_dir(record)
    data = X.load(ep3)
    veh = vehicles(ep3)
    ours = {a for r in data['actions'].values() for a in r}
    rec16, ep16 = episode_of(ROUND16_RUN, ROUND16, row)
    if vehicles(ep16) != veh:
        raise ValueError(f'row {row}: round 16 and expert v3 have different route files')
    out = dict(row=row, case=record['fields']['case'], controlled=sorted(ours),
               mainline_vehicles=dict(ours=sorted(v for v, x in veh.items() if x['route'] == 'main' and v in ours),
                                      sumo=sorted(v for v, x in veh.items() if x['route'] == 'main' and v not in ours)),
               events=dict(v3=record['events'], round16=rec16['events']),
               sumo_log=dict(v3=sumo_log(ep3), round16=sumo_log(ep16)), cars={})
    by_approach = collections.defaultdict(list)
    for agent in cars:
        by_approach[X.approach_of(data, agent)].append(agent)
    a3, a16 = actions(ep3), actions(ep16)
    for approach, members in by_approach.items():
        tracks = X.Tracks(X.observe(data, approach))
        ramps = set(X.ramp_agents(data, approach))
        stream = []
        for vid in tracks.cars():
            p, v, last = passing(tracks, vid)
            stream.append(dict(id=vid, driver='ours' if vid in ours else 'SUMO', type=veh.get(vid, {}).get('type'),
                               passing_s=p, speed_m_s=v, last_seen_s=last))
        seen3, cross3 = seen_mainline(a3, approach, data['lengths'], ours, ramps)
        seen16, cross16 = seen_mainline(a16, approach, data['lengths'], ours, ramps)
        first16 = min((c[0] for c in cross16.values()), default=math.inf)
        first3 = min((c[0] for c in cross3.values()), default=math.inf)
        loop = dict(before=round(min(first16, first3), 1), **compare(seen3, seen16, min(first16, first3)))
        for agent in members:
            t0 = min(t for t, r in data['expert'][agent].items() if r.get('junction'))
            rec = data['expert'][agent][t0]
            v0, stop0 = rec['v'], rec['junction']['stop_line']
            cap = v3.RuleExpert._free_road(dict(allowed_speed=X._allowed(data, agent, t0)))
            go_s, u_go = v3.run_up(v0, stop0, cap)
            at_line = X.line_caps(data, agent)
            best = dict(v3_run_up=round(u_go, 2), curve_capped=round(min(u_go, at_line), 2))

            def run_up(T, v0=v0, stop0=stop0, t0=t0, cap=cap):
                return line_speed_max(v0, stop0, T - t0, cap)

            def capped(T, at_line=at_line, run_up=run_up):
                u = run_up(T)
                return None if u is None else min(u, at_line)
            length = data['lengths'].get(agent, 4.8)
            judged = gaps([(x['passing_s'], x['speed_m_s'], x['id']) for x in stream], t0 + go_s, length,
                          dict(v3_run_up=run_up, curve_capped=capped), data['params'])
            leader, leader_cross = ramp_leader(a3, agent, approach, data["lengths"], cross3, ramps)
            after = None
            if leader is not None:
                after = gaps([(x['passing_s'], x['speed_m_s'], x['id']) for x in stream],
                             max(t0 + go_s, (math.inf if leader_cross is None else leader_cross)
                                 + v3.SETTINGS['yield_headway_s']),
                             length, dict(curve_capped=capped), data['params'])
            rep = replay[agent]
            c16 = cross16.get(agent)
            open_loop16 = None
            if c16 and c16[1] is not None and loop['same']:          # only where the stream was the same
                open_loop16 = X.crossing_safety(tracks, c16[0], c16[1], length, data['params'])
            out['cars'][agent] = dict(
                approach=list(approach), start=dict(t=t0, v=round(v0, 2), stop_m=round(stop0, 2), free_road=round(cap, 2),
                                                    line_cap=None if math.isinf(at_line) else round(at_line, 2),
                                                    go_s=round(go_s, 2)),
                best_line_speed=best, stream=stream, gaps=judged,
                first_open={f'{k}/{n}': dict(zip(('T', 'rear'), first_open(judged, k, n))) for k in SAFE for n in best},
                ramp_leader=None if leader is None else dict(id=leader, crossed_t=leader_cross,
                                                             driver='ours' if leader in ours else 'SUMO'),
                first_open_after_leader=None if after is None else {
                    k: dict(zip(('T', 'rear'), first_open(after, k, 'curve_capped'))) for k in SAFE},
                forced_below_5_s=forced_below(stop0, (first_open(after or judged, 'entry_safe', 'curve_capped')[0]
                                                      or math.inf) - t0),
                crossed=dict(v3_recorded=rep['recorded']['describe']['cross_s'], v3_projected=rep['v3']['describe']['cross_s'],
                             v4_projected=rep['v4']['describe']['cross_s'],
                             round16=None if not c16 else round(c16[0] - t0, 1)),
                round16_crossing=None if not c16 else dict(t=c16[0], v=None if c16[1] is None else round(c16[1], 2),
                                                           open_loop=open_loop16),
                open_loop=loop)
    return out


def sizes():
    """The SUMO cost of each next step, from recorded decisions (the re-check's own sizing)."""
    from experiments import v15_expert_v4_recheck as RC
    from experiments import v15_phase4 as P4
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    cfg = contract.build_config(24)
    dev = v15_benchmark.development_rows(cfg)
    horizons = {r: dev[r]['horizon_steps'] for r in range(len(dev))}
    per_row, _ = RC._recorded()
    slim = R.read_slim(RC.CONFIG)
    left, total = slim['settings']['charge']['study_seconds_left'], slim['settings']['charge']['total_wall_seconds']
    rate = P4._measured_rate()
    stream_rows = sorted({8, 9, 22, 24, 37, 38, 39, 53, 67})
    out = {}
    for name, rows in (('step 2 as configured (70 rows)', list(range(70))), ('stream rows only (9 rows)', stream_rows)):
        tasks, canary = RC.tasks_for(rows, horizons)
        c = RC.charge(per_row, tasks, canary, rate, left, total)
        out[name] = {k: c[k] for k in ('expected_decisions', 'projected_decisions', 'expected_episodes',
                                       'expected_clock_seconds', 'proposed_allowance')}
    out['recorded per driver, 70 rows'] = {d: sum(per_row[d].values()) for d in (RC.EXPERT_V4, RC.POLICY_V4)}
    out['study_seconds_left'] = left
    return out


def main(root=ROOT, out=None):
    import ctypes
    from concurrent.futures import ProcessPoolExecutor
    if hasattr(ctypes, 'windll'):
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x4000)
    out = Path(out or Path(root) / OUT)
    head = json.loads((Path(root) / X.OUT / 'replay.summary.json').read_text(encoding='utf-8'))
    replay = R.read_results(Path(root) / X.OUT / 'replay.summary.json')
    rows = X.source_rows(root)
    jobs = []
    for e in replay:
        cars = {a: c for a, c in e['cars'].items() if c['situation'] == 'stream'}
        if cars:
            jobs.append((e['row'], rows[e['row']], sorted(cars), cars))
    with ProcessPoolExecutor(max_workers=min(8, len(jobs))) as pool:
        results = list(pool.map(row_result, jobs))
    summary = dict(schema=SCHEMA, label=LABEL, sources=dict(
        expert_v3=X.SOURCE_RUN, round16=ROUND16_RUN, replay=f'{X.OUT}/replay.summary.json',
        replay_content_sha256=head['content_sha256'],
        code={p: X._digest(Path(root) / p) for p in ('experiments/v15_stream_gaps.py', 'experiments/v15_expert_v4_replay.py',
                                                     'traffic/v15_rule_expert.py')}),
        safe_m_s2=SAFE, sizes=sizes())
    R.write_results(out, 'stream_gaps', results, summary=summary)
    (out / 'report.md').write_text(report(summary, results), encoding='utf-8', newline='\n')
    return summary, results


# --------------------------------------------------------------------------- report

def _f(x, nd=1):
    return '—' if x is None else (f'{x:.{nd}f}' if isinstance(x, float) else str(x))


def report(summary, results):
    lines = [f'# The stream merges: who drives the stream, whether a gap exists, how open the loop is', '',
             f'Label: {summary["label"]}.', '',
             'Times are seconds from the car\'s first decision at the entry. "Best case": v3\'s gap test at the '
             'earliest crossing each gap allows and the highest speed the car could have at the line.', '',
             '## Who drives the stream', '',
             '| Row | Our controlled cars | Stream cars (passing order) | Ours in the stream |', '|---|---|---|---|']
    for r in results:
        car = next(iter(r['cars'].values()))
        order = sorted(car['stream'], key=lambda x: x['passing_s'])
        lines.append(f'| {r["row"]} | {len(r["controlled"])} | '
                     + ', '.join(f'{x["id"]} ({x["driver"]})' for x in order)
                     + f' | {sum(x["driver"] == "ours" for x in order)} of {len(order)} |')
    lines += ['', '## Is there a gap? (v3\'s test, best case)', '',
              '| Row | Car | Start speed / distance | Best line speed (v3 run-up / curve cap) | Stream: passing s (speed) '
              '| First gap at 2.0, v3 run-up | at 2.0, curve cap | at 3.0, curve cap | Car ahead on the ramp (crossed, '
              'v3 record) | at 2.0, curve cap, after it | Least time below 5 m/s before it | v3 crossed (record) '
              '| v4 (projection) | Round 16 crossed |',
              '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        for agent, c in r['cars'].items():
            t0 = c['start']['t']
            order = sorted(c['stream'], key=lambda x: x['passing_s'])
            fo, fa, lead = c['first_open'], c['first_open_after_leader'], c['ramp_leader']

            def rel(x):
                T = x['T']
                return '—' if T is None else f'{T - t0:.1f}' + ('' if x['rear'] else ' (after the last car)')
            ahead = 'none' if lead is None else f'{lead["id"]}, {lead["driver"]} ({_f(None if lead["crossed_t"] is None else lead["crossed_t"] - t0)})'
            lines.append(
                f'| {r["row"]} | {agent} | {c["start"]["v"]:.1f} m/s, {c["start"]["stop_m"]:.0f} m | '
                f'{c["best_line_speed"]["v3_run_up"]:.1f} / {c["best_line_speed"]["curve_capped"]:.1f} | '
                + ', '.join(f'{x["passing_s"] - t0:.1f} ({x["speed_m_s"]:.0f})' for x in order)
                + f' | {rel(fo["entry_safe/v3_run_up"])} | {rel(fo["entry_safe/curve_capped"])} '
                f'| {rel(fo["gate_limit/curve_capped"])} | {ahead} | {"—" if fa is None else rel(fa["entry_safe"])} '
                f'| {_f(c["forced_below_5_s"])} s '
                f'| {_f(c["crossed"]["v3_recorded"])} | {_f(c["crossed"]["v4_projected"])} | {_f(c["crossed"]["round16"])} |')
    lines += ['', '## How open the loop is', '',
              '| Row | Same SUMO car, same instant, before either crossed: matched (cars) | Largest difference: '
              'distance / speed | Up to | Round 16: crossed at (speed) | Open loop: the car behind would need | '
              'What SUMO logged with round 16 |', '|---|---|---|---|---|---|---|']
    for r in results:
        for agent, c in r['cars'].items():
            lo, t0 = c['open_loop'], c['start']['t']
            x = c['round16_crossing']
            need = '—' if lo['same'] is None else ('—' if lo['same'] else 'not projected: the stream differed')
            if x and x['open_loop'] and x['open_loop']['rear']:
                need = f'{x["open_loop"]["rear_response_m_s2"]:.1f} m/s² ({x["open_loop"]["rear"]})'
            elif x and x['open_loop']:
                need = 'no car behind'
            log = '; '.join(f'{e["vehicle"]} {e["kind"]}' + (f' {e["decel_m_s2"]:.1f} m/s²' if 'decel_m_s2' in e
                                                              else f' with {e["other"]}') + f' at {e["t"] - t0:.1f} s'
                            for e in r['sumo_log']['round16']) or 'nothing'
            crossed = '—' if not x else '{:.1f} s ({} m/s)'.format(x['t'] - t0, _f(x['v']))
            lines.append(f'| {r["row"]} {agent} | {lo["matched"]} ({lo["cars"]}) | {_f(lo["max_distance_m"], 3)} m / '
                         f'{_f(lo["max_speed_m_s"], 3)} m/s | {_f(None if lo["last_t"] is None else lo["last_t"] - t0)} s '
                         f'| {crossed} | {need} | {log} |')
    s = summary['sizes']
    lines += ['', '## What the next steps would cost (SUMO, from recorded decisions)', '',
              '| Option | Decisions (projected) | Episodes | Clock (expected) | Decision allowance |', '|---|---|---|---|---|']
    for name in ('step 2 as configured (70 rows)', 'stream rows only (9 rows)'):
        x = s[name]
        lines.append(f'| {name} | {x["expected_decisions"]:,} ({x["projected_decisions"]:,}) | {x["expected_episodes"]} '
                     f'| {x["expected_clock_seconds"]:.0f} s | {x["proposed_allowance"]["decisions"]:,} |')
    lines += ['', 'The clock is the re-check\'s own estimate: the decisions at the measured rate, or the longest episode at '
              'one worker\'s rate, whichever is longer, plus start and settling time. Step 2\'s proposed time allowance '
              'is 300 s. A stream-only run could not apply the no-regression rule, which needs every development row.',
              '', 'In the open-loop column, -99 means the car behind would have overlapped ours.', '',
              f'Study time left: {s["study_seconds_left"]:.1f} s.', '']
    return '\n'.join(lines)


if __name__ == '__main__':
    main()
