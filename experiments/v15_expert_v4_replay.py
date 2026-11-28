"""Offline replay of expert v4 on the ramp cars of the v15 development rows (plan v15, "Ramp driving", decision 14,
step 1). Nothing runs in SUMO or Isaac Sim: it reads expert v3's saved SUMO records (Phase 2 run 3, IDM background,
first attempt) and computes.

User decision of November 27, 2026, relayed by the main Claude session: "Yes, as recommended" (decisions 13-20).
Decision 14, step 1: expert v4's code, its tests, "and the offline replay of v4's gap choice over the saved stream
rows"; the user will judge v4 "by the movement as well as the numbers".

What it computes, and how each part is labelled:
- **A reconstructed stream** (inferred from SUMO records, post hoc). SUMO's own state of the mainline cars isn't
  recorded, so each mainline car's passing time at the ramp's stop point, and its speed, are rebuilt from what the
  records show of it:
  - the yield entry's front and rear cars seen by every ramp car on the same approach (actions.jsonl.gz);
  - the last car each v3 decision waited for, whose passing time is that decision's arrival less the 1 s headway;
  - the rear car of each v3 decision's accepted gap, whose speed solves the IDM response v3 recorded (expert.jsonl);
  - each car's depart speed (routes.rou.xml), which anchors its speed before it is first seen.
  Between observations the passing time and speed are interpolated, outside them held. Part A measures how well
  this reproduces v3's own recorded choices.
- **A. Choices at v3's recorded states.** v3's and v4's own junction code (traffic/v15_rule_expert.py and
  traffic/v15_rule_expert_v4.py, their `_junction`, with the episode's geometry and the cars' IDM parameters filled
  in) on the reconstructed stream, at every decision v3 made at an active yield entry. v3's replayed choice is
  compared with its record (the reconstruction's fidelity), and v4's choice is set beside it.
- **B. An open-loop kinematic projection** (neither SUMO nor physics). Each ramp car on the approach is driven from
  its first active-entry decision to the stop line by v3 or by v4, with v3's Level 1 (`compose`) and the request
  realized exactly (as SUMO realized v3's requests on these ramps). The world doesn't react:
  - mainline cars follow the reconstruction;
  - a SUMO ramp leader follows its recorded trace;
  - a controlled ramp leader on the same approach is projected with the same driver, and after its stop line
    follows its own recorded trace shifted by its change in crossing time.
  The gate's speed target comes from v3's record by time; the curve cap and the drive ceiling by distance to the
  line. Projected v3 against recorded v3 shows the projection's own error. v4 against projected v3 is the
  like-for-like comparison.
- **The drivers:** v3; v4 with its four changes; and `v4_written`, v4 with change 4's run-up limits off (the
  write-up's three changes as written), for the decision on change 4.
- **Movement checks** (traffic/v15_movement.py, decision 13). They are computed on each projected trace, and on
  v3's recorded trace, up to the stop line, not to merge completion (labelled). These are post hoc readings, not
  gates.

At most 8 processes, at below-normal priority (the user's CPU limit). Usage: python experiments/v15_expert_v4_replay.py
"""
from __future__ import annotations

import bisect
import collections
import gzip
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_runner as R                       # noqa: E402
from traffic import v15_movement as M                         # noqa: E402
from traffic import v15_rule_expert as v3                     # noqa: E402
from traffic import v15_rule_expert_v4 as v4                  # noqa: E402

SCHEMA = 'highway_merge_v15_expert_v4_replay_v01'
SOURCE_RUN = 'outputs/highway-merge-v15/phase-2/run-03'          # expert v3, the development rows
SOURCE_DRIVER = 'rule:expert-v3'
ROW_SET, BACKGROUND, ATTEMPT, ROWS = 'v15-development', 'idm', 'first', 70
OUT = 'documentation/evidence/v15-expert-v4-replay'
DT = .1
CAP = 26.8224                                     # the environments' speed cap (highway_stage1_env.CAP)
LABEL = ('open-loop kinematic projection on a stream reconstructed from saved SUMO records; '
         'neither SUMO nor physics; post hoc')
DRIVERS = {'v3': v3.RuleExpert, 'v4': v4.RuleExpert, 'v4_written': v4.RuleExpert}
# 'v4_written': v4 as the write-up specified it (changes 1-3): change 4's run-up limits off, so its gap test, wait
# plans and committed run-up assume v3's 1.5 m/s^2 up to the free-road speed. Shown for the decision on change 4.
WITHOUT_LIMITS = ('v4_written',)
SETTINGS = dict(radius_m=v3.SETTINGS['projection_radius_m'], moving_m_s=v3.SETTINGS['projection_moving_m_s'],
                idm_range_m=v3.SETTINGS['idm_range_m'], match_arrival_s=0.2, workers=8, horizon_s=60.)


# --------------------------------------------------------------------------- the saved records

def source_rows(root=ROOT):
    """{row: results record} of expert v3's development rows (IDM background, first attempt)."""
    rows = {}
    for line in (Path(root) / SOURCE_RUN / 'results.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        f = r['fields']
        if (f['driver'] == SOURCE_DRIVER and f['row_set'] == ROW_SET and f['background'] == BACKGROUND
                and r['attempt'] == ATTEMPT):
            rows.setdefault(f['row'], r)
    if sorted(rows) != list(range(ROWS)):
        raise ValueError(f'{SOURCE_RUN}: expected rows 0-{ROWS - 1}, found {len(rows)}')
    return rows


def episode_dir(record, root=ROOT):
    found = sorted((Path(root) / SOURCE_RUN / 'episodes' / record['key'][:16] / ATTEMPT).glob('episode-*'))
    if len(found) != 1:
        raise ValueError(f'{record["key"]}: {len(found)} episodes')
    return found[0]


def _t(x):
    return round(float(x), 1)


def load(episode):
    """An episode's records: actions by time, expert records by agent and time, vehicle IDM parameters and
    lengths (routes.rou.xml), and junction lane lengths by (source lane, target lane) (network.net.xml)."""
    episode = Path(episode)
    actions = {}
    with gzip.open(episode / 'actions.jsonl.gz', 'rt', encoding='utf-8') as handle:
        for line in handle:
            if line.strip():
                r = json.loads(line)
                actions[_t(r['time'])] = r['agents']
    expert = collections.defaultdict(dict)
    with open(episode / 'expert.jsonl', encoding='utf-8') as handle:
        for line in handle:
            if line.strip():
                r = json.loads(line)
                expert[r['agent']][_t(r['t'])] = r
    routes = ET.parse(episode / 'routes.rou.xml').getroot()
    types = {t.get('id'): t for t in routes.iter('vType')}
    params, lengths, departs = {}, {}, {}
    for v in routes.iter('vehicle'):
        t = types[v.get('type')]
        params[v.get('id')] = (float(t.get('accel')), float(t.get('decel')), float(t.get('tau')), float(t.get('minGap')))
        lengths[v.get('id')] = float(t.get('length'))
        if v.get('departSpeed') not in (None, 'max', 'desired', 'speedLimit', 'random', 'avg', 'last'):
            departs[v.get('id')] = (float(v.get('depart')), float(v.get('departSpeed')))
    net = ET.parse(episode / 'network.net.xml').getroot()
    lanes = {lane.get('id'): float(lane.get('length')) for edge in net.iter('edge') for lane in edge.iter('lane')}
    shapes = {lane.get('id'): [tuple(map(float, xy.split(','))) for xy in lane.get('shape').split()]
              for edge in net.iter('edge') for lane in edge.iter('lane')}
    junction, connections = {}, []
    for c in net.iter('connection'):
        if c.get('via') and not c.get('from', '').startswith(':'):
            key = (f'{c.get("from")}_{c.get("fromLane")}', f'{c.get("to")}_{c.get("toLane")}')
            junction[key] = lanes[c.get('via')]
            connections.append((key[0], key[1], c.get('via')))
    return dict(actions=actions, expert=dict(expert), params=params, lengths=lengths, junction=junction,
                shapes=shapes, connections=connections, departs=departs)


class Paths:
    """Positions on the approach's two paths into the target lane (the ramp's, and the mainline's), by distance before
    the target lane's start, from the network's lane shapes, for v3's 250 m radius (a straight-line distance)."""

    def __init__(self, data, approach):
        source, target = approach
        into = [(f, via) for f, t, via in data['connections'] if t == target]
        ramp = next(via for f, via in into if f == source)
        mainline = [(f, via) for f, via in into if f != source]
        self.ramp = self._line(data, source, ramp)
        self.mainline = self._line(data, *mainline[0]) if mainline else None

    @staticmethod
    def _line(data, lane, via, reach=1000.):
        points = list(data['shapes'][lane]) + list(data['shapes'][via])[1:]
        total, current = 0., lane
        while total < reach:
            before = [f for f, t, _ in data['connections'] if t == current]
            if len(before) != 1:
                break
            current = before[0]
            points = list(data['shapes'][current]) + points[1:]
            total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        return points

    @staticmethod
    def point(line, back):
        """The point `back` m before the line's end (extended straight past its start)."""
        rest = back
        for a, b in zip(reversed(line[:-1]), reversed(line[1:])):
            step = math.dist(a, b)
            if rest <= step:
                f = rest / step if step else 0.
                return (b[0] + (a[0] - b[0]) * f, b[1] + (a[1] - b[1]) * f)
            rest -= step
        a, b = line[0], line[1]
        step = math.dist(a, b) or 1.
        return (a[0] - (b[0] - a[0]) * rest / step, a[1] - (b[1] - a[1]) * rest / step)

    def distance(self, ramp_back, mainline_back):
        if self.mainline is None:
            return abs(ramp_back - mainline_back)
        return math.dist(self.point(self.ramp, ramp_back), self.point(self.mainline, mainline_back))


# --------------------------------------------------------------------------- the reconstructed stream

def approach_of(data, agent):
    """(source lane, target lane) of an agent's active yield entry, or None."""
    for t in sorted(data['actions']):
        a = data['actions'][t].get(agent)
        e = ((a or {}).get('flow_context') or {}).get('entry') or {}
        if e.get('entry_active'):
            return e['source_lane'], e['target_lane']
    return None


def ramp_agents(data, approach):
    return sorted(a for a in data['expert'] if approach_of(data, a) == approach)


def solve_rear_speed(gap, response, entry_speed, params):
    """The rear car's speed from v3's recorded IDM response (free term 0, as v3 computes it), or None."""
    if gap is None or response is None or response >= 0 or gap <= 0:
        return None
    a, b, tau, s0 = params
    excess = gap * math.sqrt(-response / a) - s0
    if excess <= 0:
        return None
    k = 1. / (2. * math.sqrt(a * b))
    # k V^2 + (tau - k u) V - excess = 0
    p, q = tau - k * entry_speed, -excess
    return (-p + math.sqrt(p * p - 4. * k * q)) / (2. * k)


class Tracks:
    """Each mainline car's passing time at the approach's stop point P(t) and speed V(t), from the observations."""

    def __init__(self, observations):
        self.p, self.v = {}, {}
        for vid, xs in observations.items():
            by = collections.defaultdict(list)
            for t, p, _ in xs:
                if p is not None:
                    by[t].append(p)
            self.p[vid] = sorted((t, statistics.fmean(ps)) for t, ps in by.items())
            byv = collections.defaultdict(list)
            for t, _, v in xs:
                if v is not None:
                    byv[t].append(v)
            self.v[vid] = sorted((t, statistics.fmean(vs)) for t, vs in byv.items())

    @staticmethod
    def _at(series, t):
        if not series:
            return None
        times = [x for x, _ in series]
        i = bisect.bisect_left(times, t)
        if i == 0:
            return series[0][1]
        if i == len(series):
            return series[-1][1]
        (t0, y0), (t1, y1) = series[i - 1], series[i]
        return y0 + (y1 - y0) * (t - t0) / (t1 - t0)

    def state(self, vid, t):
        """(passing time, speed) at t, or None where the car was never seen with both."""
        p, v = self._at(self.p.get(vid), t), self._at(self.v.get(vid), t)
        return None if p is None or v is None else (p, v)

    def cars(self):
        return sorted(set(self.p) & set(self.v))


def observe(data, approach):
    """{mainline car: [(time, passing time or None, speed or None)]} from every ramp car on the approach."""
    ramps = set(ramp_agents(data, approach))
    obs = collections.defaultdict(list)
    lengths = data['lengths']
    for t, agents in data['actions'].items():
        tau = _t(t - DT)                                  # flow_context is the decision's (pre-action) state
        for agent, a in agents.items():
            e = ((a.get('flow_context') or {}).get('entry') or {})
            if not (e.get('entry_active') and (e.get('source_lane'), e.get('target_lane')) == approach):
                continue
            rec = data['expert'].get(agent, {}).get(tau)
            if rec is None:
                continue
            v, stop = rec['v'], e['distance_to_start_m']
            for kind in ('front', 'rear'):
                x = e.get(kind)
                if not x or x['id'] in ramps or x['id'].startswith('ramp'):
                    continue
                speed = v + x['relative_speed_m_s']
                if kind == 'rear':
                    d = x['bumper_gap_m'] + lengths.get(agent, 4.8) + stop
                else:
                    d = stop - x['bumper_gap_m'] - lengths.get(x['id'], 4.8)
                obs[x['id']].append((tau, tau + d / max(speed, 1.), speed))
    for agent in ramps:
        for t, rec in data['expert'][agent].items():
            j = rec.get('junction')
            if not j:
                continue
            if j['waited_for'] and j['waited_for'][-1] not in ramps:
                obs[j['waited_for'][-1]].append((t, t + j['arrive_s'] - v3.SETTINGS['yield_headway_s'], None))
            if j['rear'] is not None and j['rear'] not in ramps:
                speed = solve_rear_speed(j['gap_at_entry'], j['response'], j['entry_m_s'], data['params'][j['rear']])
                if speed is not None:
                    when = j['arrive_s'] + (j['gap_at_entry'] + data['lengths'].get(agent, 4.8)) / max(speed, 1.)
                    obs[j['rear']].append((t, t + when, speed))
    for vid in list(obs):
        if vid in data['departs']:
            obs[vid].append((_t(data['departs'][vid][0]), None, data['departs'][vid][1]))   # the recorded depart speed
    return dict(obs)


def first_seen(data, approach, agent):
    """{mainline car: the first time the agent's own v3 decisions listed it (waited for, or the accepted gap's rear)}.

    v3's visibility (moving cars within 250 m, a straight-line distance) is taken from these records where they
    list a car, rather than rebuilt: the radius edge is too sensitive to the reconstructed speeds. Cars the agent's
    decisions never listed fall back to the radius from the lane shapes."""
    ramps, seen = set(ramp_agents(data, approach)), {}
    for t, rec in data['expert'][agent].items():
        j = rec.get('junction')
        if j:
            for vid in j['waited_for'] + ([j['rear']] if j['rear'] else []):
                if vid not in ramps and (vid not in seen or t < seen[vid]):
                    seen[vid] = t
    return seen


def world(tracks, t, stop, jl, lengths, paths=None, seen=None, s=SETTINGS):
    """At time t, with our front `stop` m from the line: the stream (v3's `rears`: cars that reach the shared lane
    after us) as (distance to our stop point, speed, id), and the projected leaders (those that reach it before us)
    as (gap now, speed, acceleration, id, their distance to the shared lane)."""
    stream, leaders = [], []
    for vid in tracks.cars():
        now, later = tracks.state(vid, t), tracks.state(vid, t + DT)
        if now is None:
            continue
        passing, speed = now
        if speed < s['moving_m_s']:
            continue
        d = (passing - t) * max(speed, 1.)
        if seen and vid in seen:
            if t < seen[vid] - 1e-9:
                continue                              # not yet seen by this car's v3
        elif (paths.distance(stop + jl, d + jl) if paths is not None else abs(d - stop)) > s['radius_m']:
            continue                                  # v3's 250 m radius, from the lane shapes
        if d >= stop:
            stream.append((d, speed, vid))
        else:
            accel = 0. if later is None else (later[1] - speed) / DT
            leaders.append((stop - d - lengths.get(vid, 4.8), speed, accel, vid, d + jl))
    return stream, leaders


# --------------------------------------------------------------------------- the drivers' own junction code

class Junction:
    """A driver's own `_junction` (v3's or v4's RuleExpert) with the episode's geometry filled in: the shared lane
    `jl` beyond the stop line, and each car's SUMO IDM parameters."""

    LANE = 'target'

    def __init__(self, name, jl, params):
        self.driver = DRIVERS[name](lambda: None, record=False)
        self.limited = name not in WITHOUT_LIMITS
        self.jl = jl
        self._stop = None
        self.driver._distances = lambda env, agent: {self.LANE: self._stop + self.jl}
        self.driver._curve_points = lambda env, agent, state: []    # the curve cap at the line comes in `curve`
        self.driver._idm = lambda env, vid: params.get(vid, None) or (
            v3.SETTINGS['background_accel_m_s2'], v3.SETTINGS['background_decel_m_s2'],
            v3.SETTINGS['background_tau_s'], v3.SETTINGS['background_min_gap_m'])

    def __call__(self, agent, speed, stop, stream, allowed, length=4.8, ceiling=None, curve=None):
        self._stop = stop
        if hasattr(self.driver, '_limits'):                                                # v4's change 4
            self.driver._limits[agent] = ((ceiling, math.inf if curve is None else curve) if self.limited
                                          else (None, None))
        state = dict(speed=speed, length=length, allowed_speed=allowed)
        entry = dict(known=True, entry_active=True, target_lane=self.LANE, distance_to_start_m=stop)
        rears = [(None, v, d + self.jl, vid, self.LANE) for d, v, vid in stream]
        return self.driver._junction(None, agent, state, entry, rears, False)

    def forget(self, agent):
        self.driver._commit.pop(agent, None)


def _allowed(data, agent, t):
    a = data['actions'].get(_t(t + DT), {}).get(agent) or data['actions'].get(_t(t), {}).get(agent)
    own = ((a or {}).get('flow_context') or {}).get('corridors', {}).get('own') or {}
    return own.get('legal_speed_m_s')


def _same(a, b):
    """Whether a replayed choice equals the recorded one (committed, rear, waited-for cars, arrival within 0.2 s)."""
    return (a['committed'] == b['committed'] and a['rear'] == b['rear'] and a['waited_for'] == b['waited_for']
            and abs(a['arrive_s'] - b['arrive_s']) <= SETTINGS['match_arrival_s'] + 1e-9)


def choices(data, agent, approach, tracks):
    """Part A: v3's and v4's choices at each of v3's recorded active-entry states, beside v3's record."""
    jl = data['junction'][approach]
    paths, seen = Paths(data, approach), first_seen(data, approach, agent)
    drivers = {name: Junction(name, jl, data['params']) for name in DRIVERS}
    at_line = line_caps(data, agent)
    out = []
    for t in sorted(data['expert'][agent]):
        rec = data['expert'][agent][t]
        j = rec.get('junction')
        if not j:
            for d in drivers.values():
                d.forget(agent)
            continue
        stream, _ = world(tracks, t, j['stop_line'], jl, data['lengths'], paths, seen)
        allowed = _allowed(data, agent, t)
        row = dict(t=t, v=rec['v'], stop=j['stop_line'], recorded=_brief(j))
        curve = min(math.inf if rec['curve'] is None else rec['curve'], at_line)
        for name, d in drivers.items():
            row[name] = _brief(d(agent, rec['v'], j['stop_line'], stream, allowed, data['lengths'].get(agent, 4.8),
                                 ceiling=rec['ceiling'], curve=curve))
        row['v3_matches_record'] = _same(row['v3'], row['recorded'])
        row['v4_differs'] = any(row['v4'][k] != row['v3'][k] for k in row['v3'])
        out.append(row)
    return out


def line_caps(data, agent):
    """v3's recorded curve cap at its decision nearest the stop line: the cap seen from the line (inf if none)."""
    near = [(j['stop_line'], r['curve']) for r in data['expert'][agent].values() if (j := r.get('junction'))]
    if not near:
        return math.inf
    curve = min(near)[1]
    return math.inf if curve is None else curve


def _brief(j):
    keys = ('committed', 'dilemma', 'arrive_s', 'entry_m_s', 'hold_m_s', 'rear', 'waited_for', 'kept')
    out = {k: j.get(k) for k in keys}
    out.update({k: j[k] for k in ('kind', 'keep_speed', 'hold_w', 'needs_run_up', 'run_up_m_s2') if k in j})
    return out


# --------------------------------------------------------------------------- part B: the projection

class Recorded:
    """A recorded car's trace by time: speed, acceleration, front-bumper distance to the line (`x`, positive ahead
    of the line), from v3's expert records (controlled cars) or a follower's own-front records (SUMO cars)."""

    def __init__(self, points):
        self.points = sorted(points)
        self.times = [p[0] for p in self.points]

    def at(self, t):
        i = bisect.bisect_left(self.times, t - 1e-9)
        if i >= len(self.points):
            return None
        if abs(self.times[i] - t) > 1e-6:
            if i == 0:
                return None
            j = i - 1
            if t - self.times[j] > .25:
                return None
            return self.points[j][1:]
        return self.points[i][1:]


def _by_distance(rec_series):
    """[(distance to the line, value)] sorted by distance, for lookups by position."""
    return sorted(rec_series)


def _lookup(series, x):
    if not series:
        return None
    xs = [p for p, _ in series]
    i = bisect.bisect_left(xs, x)
    if i == 0:
        return series[0][1]
    if i == len(series):
        return series[-1][1]
    (x0, y0), (x1, y1) = series[i - 1], series[i]
    return y0 if abs(x - x0) <= abs(x1 - x) else y1


def own_trace(data, agent):
    """v3's recorded trace of a controlled car: {t: (v, a, stop or None)} with stop from its junction records
    and continued past the line by its own speed (Euler with the new speed, as SUMO integrates)."""
    out, stop = {}, None
    for t in sorted(data['expert'][agent]):
        r = data['expert'][agent][t]
        j = r.get('junction')
        if j:
            stop = j['stop_line']
        elif stop is not None:
            stop -= r['v'] * DT
        out[t] = (r['v'], r['a'], stop)
    return out


def leader_trace(data, agent):
    """The car's own-lane leader as recorded: {t: (id, rear-bumper distance to the line, speed, acceleration)}."""
    trace = own_trace(data, agent)
    out = {}
    for t, (v, _, stop) in trace.items():
        a = data['actions'].get(_t(t + DT), {}).get(agent)
        front = (((a or {}).get('flow_context') or {}).get('corridors') or {}).get('own', {}).get('front')
        if front and stop is not None:
            out[t] = (front['id'], stop - front['bumper_gap_m'], v + front['relative_speed_m_s'])
    acc = {}
    for t, (vid, x, v) in out.items():
        nxt = out.get(_t(t + DT))
        acc[t] = (nxt[2] - v) / DT if nxt and nxt[0] == vid else 0.
    return {t: (vid, x, v, acc[t]) for t, (vid, x, v) in out.items()}


def project(data, approach, tracks, name, s=SETTINGS):
    """Part B for every controlled ramp car on the approach, with driver `name`: {agent: trace}."""
    jl = data['junction'][approach]
    paths = Paths(data, approach)
    agents = ramp_agents(data, approach)
    traces = {a: own_trace(data, a) for a in agents}
    starts = {a: min(t for t, r in data['expert'][a].items() if r.get('junction')) for a in agents}
    ends = {a: max(t for t, r in data['expert'][a].items() if r.get('junction')) for a in agents}
    leaders = {a: leader_trace(data, a) for a in agents}
    seen = {a: first_seen(data, approach, a) for a in agents}
    at_line = {a: line_caps(data, a) for a in agents}
    curves, drives, plans = {}, {}, {}
    for a in agents:
        rows = [(traces[a][t][2], data['expert'][a][t]) for t in sorted(traces[a]) if traces[a][t][2] is not None]
        curves[a] = _by_distance([(x, r['curve']) for x, r in rows])
        drive = []
        for t in sorted(traces[a]):
            x = traces[a][t][2]
            act = data['actions'].get(_t(t + DT), {}).get(a)
            bounds = (act or {}).get('acceleration_bounds') or {}
            if x is not None and bounds.get('drive_grade') is not None:
                drive.append((x, bounds['drive_grade']))
        drives[a] = _by_distance(drive)
        plans[a] = {t: r['plan'] for t, r in data['expert'][a].items()}
    junction = Junction(name, jl, data['params'])
    state = {}                                        # agent: dict(v, a, x) while projected
    crossed = {}                                      # agent: projected crossing time
    out = {a: dict(start=starts[a], samples=[], decisions=[], crossed=None, end=ends[a]) for a in agents}
    t = min(starts.values())
    last = max(ends.values()) + s['horizon_s']
    while t <= last and len(crossed) < len(agents):
        # leaders first: the car nearest the line moves first
        before = {a: dict(x) for a, x in state.items()}      # every car's leader as it was at the step's start
        for a in agents:
            if a in crossed or t < starts[a]:
                continue
            if a not in state:
                v, acc, x = traces[a][starts[a]]
                state[a] = dict(v=v, a=acc, x=x)
            me = state[a]
            front = _front(a, t, me, leaders, before, crossed, traces, data)
            stream, ahead = world(tracks, t, me['x'], jl, data['lengths'], paths, seen[a])
            allowed = _allowed(data, a, min(t, ends[a])) or 20.
            cap = _lookup(curves[a], me['x'])
            ceiling = _ceiling(me, front, allowed, _lookup(drives[a], me['x']))
            decision = junction(a, me['v'], me['x'], stream, allowed, data['lengths'].get(a, 4.8),
                                ceiling=ceiling, curve=min(math.inf if cap is None else cap, at_line[a]))
            target = _plan(plans[a], t, me['v'])
            free_road = max(0., min(v3.SETTINGS['free_road_m_s'], allowed) - v3.SETTINGS['speed_margin_m_s'])
            extra = {}
            if decision['committed'] and decision['rear'] is not None:
                target = (v4.committed_target(target, decision, me['v'], free_road) if name.startswith('v4')
                          else max(target, min(free_road, me['v'])))
            elif not decision['committed']:
                target = min(target, decision['hold_m_s'])
                extra['stop_line'] = v3.idm_interaction(
                    me['v'], 0., decision['stop_line'] + v3.SETTINGS['idm_min_gap_m'] - v3.SETTINGS['stop_line_margin_m'])
            if cap is not None:
                target = min(target, cap)
            lead = []
            if front and front['gap'] <= s['idm_range_m']:
                lead.append((front['v'], front['gap'], front['a']))
            projected = None
            for gap, vl, al, vid, their in ahead:
                at_shared = gap + (vl - me['v']) * max(0., their) / max(vl, 1.)
                if 0. < at_shared <= s['idm_range_m'] and (projected is None or at_shared < projected[0]):
                    projected = (at_shared, vl, al, vid)
            if projected is not None:
                lead.append((projected[1], projected[0], projected[2]))
            request, terms = v3.compose(me['v'], me['a'], target, ceiling, leaders=lead, extra=extra)
            v = max(0., me['v'] + request * DT)
            sample = dict(t=t, v=round(me['v'], 4), a=round(me['a'], 4), x=round(me['x'], 3),
                          front=None if not front else dict(id=front['id'], bumper_gap_m=round(front['gap'], 3),
                                                            relative_speed_m_s=round(front['v'] - me['v'], 4)))
            out[a]['samples'].append(sample)
            out[a]['decisions'].append(dict(t=t, request=round(request, 4), target=round(target, 4),
                                            projected=None if projected is None else projected[3],
                                            **_brief(decision)))
            me.update(v=v, a=(v - me['v']) / DT, x=me['x'] - v * DT)
            if me['x'] <= 0.:
                crossed[a] = _t(t + DT)
                out[a]['crossed'] = dict(t=crossed[a], v=round(v, 4), shift=round(crossed[a] - _crossing(traces[a], ends[a]), 2),
                                         **crossing_safety(tracks, crossed[a], v, data['lengths'].get(a, 4.8), data['params']))
                junction.forget(a)
        t = _t(t + DT)
    return out


def crossing_safety(tracks, t, speed, length, params):
    """At a projected crossing: the next mainline car behind (its gap to our rear bumper and the IDM braking it would
    need behind us, as v3's gap test computes it) and the time since the last mainline car passed (v3 asks 1 s).
    Open loop: the mainline cars don't react, so this is the response they would need, not one that happened."""
    behind, ahead = None, None
    for vid in tracks.cars():
        x = tracks.state(vid, t)
        if x is None:
            continue
        passing, v = x
        if passing >= t:
            gap = (passing - t) * max(v, 1.) - length
            if behind is None or gap < behind[1]:
                behind = (vid, gap, v)
        elif ahead is None or passing > ahead[1]:
            ahead = (vid, passing)
    out = dict(rear=None, rear_gap_m=None, rear_response_m_s2=None, ahead=None, ahead_headway_s=None)
    if behind is not None:
        vid, gap, v = behind
        out.update(rear=vid, rear_gap_m=round(gap, 2), rear_response_m_s2=round(max(-99., v3.background_response(
            v, speed, max(gap, .1), max(v, 1.), params.get(vid))), 3))
    if ahead is not None:
        out.update(ahead=ahead[0], ahead_headway_s=round(t - ahead[1], 2))
    return out


def _crossing(trace, end):
    """The recorded crossing time: the decision after the last active-entry decision."""
    return _t(end + DT)


def _plan(plans, t, default):
    """The gate's speed target recorded at t, else the latest before t, else the first."""
    if t in plans:
        return plans[t]
    earlier = [k for k in plans if k <= t]
    return plans[max(earlier)] if earlier else (plans[min(plans)] if plans else default)


def _front(a, t, me, leaders, state, crossed, traces, data):
    """The car's own-lane leader in the projection: a controlled leader on the approach as projected (after its line,
    its recorded trace shifted by its change in crossing time), any other as recorded. Its gap is from our projected
    position. Returns dict(id, gap, v, a) or None."""
    rec = leaders[a].get(t)
    if rec is None:
        return None
    vid, x_rear, v, acc = rec
    if vid in state and vid not in crossed:
        other = state[vid]
        x_rear = other['x'] + data['lengths'].get(vid, 4.8)
        v, acc = other['v'], other['a']
    elif vid in crossed:
        shift = crossed[vid] - _crossing(traces[vid], max(k for k, r in data['expert'][vid].items() if r.get('junction')))
        then = traces[vid].get(_t(t - shift))
        if then is None or then[2] is None:
            return None
        x_rear, v, acc = then[2] + data['lengths'].get(vid, 4.8), then[0], then[1]
    return dict(id=vid, gap=me['x'] - x_rear, v=v, a=acc)


def _ceiling(me, front, allowed, drive):
    """The environment's ceiling (environments/highway_merge_v15_env.acceleration_bounds): speed, following and drive
    bounds; the drive bound from v3's record at the same distance to the line."""
    bounds = [(min(CAP, allowed) - me['v']) / DT]
    if front:
        safe = max(0., front['v'] + .5 * (front['gap'] - 2.5 - 1.5 * me['v']))
        bounds.append((safe - me['v']) / DT)
    if drive is not None:
        bounds.append(drive)
    return max(-4.5, min(2., *bounds))


# --------------------------------------------------------------------------- one car's results

def recorded_samples(data, agent):
    """v3's recorded trace to the stop line, in the movement checks' sample form."""
    trace, leaders = own_trace(data, agent), leader_trace(data, agent)
    end = max(t for t, r in data['expert'][agent].items() if r.get('junction'))
    start = min(t for t, r in data['expert'][agent].items() if r.get('junction'))
    out = []
    for t in sorted(trace):
        if t < start or t > end:
            continue
        v, a, x = trace[t]
        lead = leaders.get(t)
        front = None if lead is None else dict(id=lead[0], bumper_gap_m=round(x - lead[1], 3),
                                                relative_speed_m_s=round(lead[2] - v, 4))
        out.append(dict(t=t, v=v, a=a, x=x, front=front))
    return out, _t(end + DT), trace.get(_t(end + DT), (None,))[0]


def mainline_speed(tracks, t, window=5.):
    """The nearby mainline speed for M3 at time t: the median speed of the reconstructed mainline cars that pass the
    stop point within `window` s of t (inferred)."""
    speeds = []
    for vid in tracks.cars():
        x = tracks.state(vid, t)
        if x and abs(x[0] - t) <= window:
            speeds.append(x[1])
    return statistics.median(speeds) if speeds else None


def checks(samples, crossed, mainline):
    """Movement checks on a trace to the stop line (decision 13's thresholds; to the line, not merge completion)."""
    rows = [dict(v=x['v'], a=x['a'], front=x.get('front')) for x in samples]
    merged = crossed is not None
    if merged:
        rows.append(dict(v=crossed['v'], a=None, front=None))
    return M.car_checks(rows, merged, mainline)


def describe(samples, crossed, start):
    """The speed trace in brief: start, lowest and when, time below 5 m/s, hardest braking, crossing."""
    vs = [x['v'] for x in samples]
    low = min(range(len(vs)), key=lambda i: vs[i]) if vs else None
    brakes = [x['a'] for x in samples if x['a'] is not None]
    return dict(start_v=round(vs[0], 2) if vs else None,
                min_v=round(vs[low], 2) if vs else None, min_at_s=round(samples[low]['t'] - start, 1) if vs else None,
                below_5_s=round(sum(v < 5. for v in vs) * DT, 1), hardest_brake=round(min(brakes), 2) if brakes else None,
                cross_s=None if crossed is None else round(crossed['t'] - start, 1),
                cross_v=None if crossed is None else round(crossed['v'], 2),
                profile=[round(vs[i], 1) for i in range(0, len(vs), 10)])


def episode(job):
    """All results of one row: part A for the given cars, part B for every controlled car on the approach."""
    row, record, cars, root = job
    import ctypes
    if hasattr(ctypes, 'windll'):
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x4000)
    data = load(episode_dir(record, root))
    out = dict(row=row, case=record['fields']['case'], cars={})
    approaches = {}
    for agent, situation in cars:
        approaches.setdefault(approach_of(data, agent), []).append((agent, situation))
    for approach, members in approaches.items():
        tracks = Tracks(observe(data, approach))
        projections = {name: project(data, approach, tracks, name) for name in DRIVERS}
        for agent, situation in members:
            a = choices(data, agent, approach, tracks)
            recorded, recorded_cross_t, recorded_cross_v = recorded_samples(data, agent)
            start = recorded[0]['t']
            car = dict(situation=situation, approach=list(approach), junction_lane_m=data['junction'][approach],
                       ramp_cars=ramp_agents(data, approach), choices=a,
                       fidelity=dict(decisions=len(a), v3_matches=sum(x['v3_matches_record'] for x in a),
                                     v4_differs=sum(x['v4_differs'] for x in a)))
            ms = mainline_speed(tracks, recorded_cross_t)
            rec_cross = None if recorded_cross_v is None else dict(t=recorded_cross_t, v=recorded_cross_v)
            car['recorded'] = dict(describe=describe(recorded, rec_cross, start),
                                   checks=checks(recorded, rec_cross, ms), mainline_speed=ms)
            for name, by in projections.items():
                p = by[agent]
                ms_p = mainline_speed(tracks, p['crossed']['t']) if p['crossed'] else None
                car[name] = dict(describe=describe(p['samples'], p['crossed'], start),
                                 checks=checks(p['samples'], p['crossed'], ms_p), mainline_speed=ms_p,
                                 crossed=p['crossed'], samples=p['samples'], decisions=p['decisions'],
                                 commits=_commits(p['decisions']))
            car['recorded']['samples'] = recorded
            out['cars'][agent] = car
    return out


def _commits(decisions):
    """Commit starts, undone commits and dilemma commits in a projected trace."""
    starts = undone = dilemma = 0
    previous = False
    for d in decisions:
        if d['committed'] and not previous:
            starts += 1
        if previous and not d['committed']:
            undone += 1
        dilemma += bool(d['dilemma'])
        previous = d['committed']
    return dict(starts=starts, undone=undone, dilemma_decisions=dilemma)


# --------------------------------------------------------------------------- the whole replay

def situations(rows, root=ROOT, workers=SETTINGS['workers']):
    """Decision 15's situation of each ramp car, from expert v3's saved trace (traffic/v15_movement)."""
    with ProcessPoolExecutor(max_workers=workers) as pool:
        found = list(pool.map(_situations_one, [(row, rec, root) for row, rec in sorted(rows.items())]))
    return {(row, agent): s for row, cars in found for agent, s in cars.items()}


def _situations_one(job):
    row, record, root = job
    import ctypes
    if hasattr(ctypes, 'windll'):
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x4000)
    cars = M.episode_cars(episode_dir(record, root))
    return row, {agent: M.situation(c, record['fields']['case']) for agent, c in cars.items()}


def summary(results):
    """Counts per situation and driver: movement checks (to the line), crossings, the reconstruction's fidelity."""
    out = {}
    for group in ('stream', 'clear'):
        cars = [(r['row'], agent, c) for r in results for agent, c in r['cars'].items() if c['situation'] == group]
        if not cars:
            continue
        g = dict(cars=len(cars), decisions=sum(c['fidelity']['decisions'] for _, _, c in cars),
                 v3_replay_matches_record=sum(c['fidelity']['v3_matches'] for _, _, c in cars),
                 v4_differs_from_v3=sum(c['fidelity']['v4_differs'] for _, _, c in cars))
        for name in ('recorded', *DRIVERS):
            ch = [c[name]['checks'] for _, _, c in cars]
            d = [c[name]['describe'] for _, _, c in cars]
            crossed = [x['cross_s'] for x in d if x['cross_s'] is not None]
            g[name] = dict(crossed=len(crossed), m1=sum(x['flags']['m1'] for x in ch), m2=sum(x['flags']['m2'] for x in ch),
                           m4=sum(x['flags']['m4'] for x in ch), fails=sum(x['fails'] for x in ch),
                           m3_flags=sum(1 for x in ch if x['m3'] and not x['m3']['passed']),
                           m3_applies=sum(1 for x in ch if x['m3']),
                           median_cross_s=round(statistics.median(crossed), 1) if crossed else None,
                           median_cross_v=round(statistics.median(x['cross_v'] for x in d if x['cross_v'] is not None), 2)
                           if crossed else None,
                           median_min_v=round(statistics.median(x['min_v'] for x in d), 2),
                           median_below_5_s=round(statistics.median(x['below_5_s'] for x in d), 1))
            if name != 'recorded':
                g[name]['undone_commits'] = sum(c[name]['commits']['undone'] for _, _, c in cars)
                g[name]['dilemma_cars'] = sum(c[name]['commits']['dilemma_decisions'] > 0 for _, _, c in cars)
        out[group] = g
    return out


def main(root=ROOT, out=None):
    import ctypes
    if hasattr(ctypes, 'windll'):
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x4000)
    out = Path(out or Path(root) / OUT)
    rows = source_rows(root)
    found = situations(rows, root)
    chosen = collections.defaultdict(list)
    for (row, agent), s in sorted(found.items()):
        if s in ('stream', 'clear'):
            chosen[row].append((agent, s))
    jobs = [(row, rows[row], cars, root) for row, cars in sorted(chosen.items())]
    with ProcessPoolExecutor(max_workers=SETTINGS['workers']) as pool:
        results = list(pool.map(episode, jobs))
    head = dict(schema=SCHEMA, label=LABEL, source_run=SOURCE_RUN, source_driver=SOURCE_DRIVER,
                situations={g: sum(s == g for s in found.values()) for g in M.SITUATIONS},
                settings=SETTINGS, expert_v4_settings={k: v4.SETTINGS[k] for k in v4.NEW_SETTINGS},
                sources={p: _digest(Path(root) / p) for p in ('traffic/v15_rule_expert.py', 'traffic/v15_rule_expert_v4.py',
                                                              'traffic/v15_movement.py', 'experiments/v15_expert_v4_replay.py')},
                summary=summary(results))
    R.write_results(out, 'replay', results, summary=head)
    (out / 'report.md').write_text(report(head, results), encoding='utf-8', newline='\n')
    return head, results


NAMES = {'recorded': 'v3 recorded (SUMO)', 'v3': 'v3 projected', 'v4': 'v4 projected',
         'v4_written': 'v4 as written (changes 1-3), projected'}


def _cell(x):
    return '-' if x is None else (f'{x:g}' if isinstance(x, (int, float)) and not isinstance(x, bool) else str(x))


def report(head, results):
    """The replay's tables as Markdown (generated from the results; the reading is in the plan)."""
    lines = [f'# Expert v4 offline replay: {LABEL}', '',
             f'Source: {SOURCE_RUN} ({SOURCE_DRIVER}). Situations of the 31 ramp cars (decision 15): '
             + ', '.join(f'{k} {v}' for k, v in head['situations'].items())
             + '. Only the stream and clear cars are replayed.', '',
             'Movement checks are to the stop line, not merge completion, and post hoc (reported, not gates). '
             'Open loop: the mainline cars do not react, so a rear response is the braking it would need.', '']
    summ = head['summary']
    lines += ['## Fidelity (part A: choices at v3\'s recorded states)', '',
              '| Situation | Cars | Decisions | v3 replay = record | v4 differs from v3 |', '|---|---|---|---|---|']
    for group, g in summ.items():
        lines.append(f"| {group} | {g['cars']} | {g['decisions']} | {g['v3_replay_matches_record']} | "
                     f"{g['v4_differs_from_v3']} |")
    lines += ['', '## Movement and crossings (part B: projection to the stop line)', '',
              '| Situation | Driver | Crossed | M1 | M2 | M4 | Fail any | M3 flags / applies | Median crossing (s) | '
              'Median crossing speed (m/s) | Median lowest speed (m/s) | Median time below 5 m/s (s) | '
              'Undone commits | Cars with a dilemma commit |', '|' + '---|' * 14]
    for group, g in summ.items():
        for name in NAMES:
            x = g[name]
            lines.append(f"| {group} | {NAMES[name]} | {x['crossed']} | {x['m1']} | {x['m2']} | {x['m4']} | "
                         f"{x['fails']} | {x['m3_flags']} / {x['m3_applies']} | {_cell(x['median_cross_s'])} | "
                         f"{_cell(x['median_cross_v'])} | {_cell(x['median_min_v'])} | {_cell(x['median_below_5_s'])} | "
                         f"{_cell(x.get('undone_commits'))} | {_cell(x.get('dilemma_cars'))} |")
    lines += ['', '## Each stream car', '',
              'Lowest speed, time below 5 m/s, hardest braking, crossing time from the first active-entry decision '
              'and crossing speed; flags 1-4 = M1-M4 failed; rear = the braking the next mainline car would need at '
              'the crossing (m/s^2, open loop; v3\'s gap test allows 2.0).', '',
              '| Row | Car | Driver | Lowest (m/s) | Below 5 m/s (s) | Hardest braking (m/s^2) | Crossing (s) | '
              'Crossing speed (m/s) | Flags | Rear response (m/s^2) |', '|' + '---|' * 10]
    for res in results:
        for agent, c in res['cars'].items():
            if c['situation'] != 'stream':
                continue
            for name in NAMES:
                d, ck = c[name]['describe'], c[name]['checks']
                flags = ''.join(k[1] for k, v in ck['flags'].items() if v)
                if ck['m3'] and not ck['m3']['passed']:
                    flags = ''.join(sorted(flags + '3'))
                rear = (c[name].get('crossed') or {}).get('rear_response_m_s2')
                lines.append(f"| {res['row']} | {agent} | {NAMES[name]} | {_cell(d['min_v'])} | {_cell(d['below_5_s'])} | "
                             f"{_cell(d['hardest_brake'])} | {_cell(d['cross_s'])} | {_cell(d['cross_v'])} | "
                             f"{flags or '-'} | {_cell(rear)} |")
    lines += ['', '## Speed every second (m/s), stream cars', '']
    for res in results:
        for agent, c in res['cars'].items():
            if c['situation'] != 'stream':
                continue
            lines.append(f"- **Row {res['row']}, {agent}**")
            for name in ('v3', 'v4', 'v4_written'):
                lines.append(f"  - {NAMES[name]}: " + ', '.join(f'{v:g}' for v in c[name]['describe']['profile']))
    return '\n'.join(lines) + '\n'


def _digest(path):
    from traffic.highway_flow_contract import file_digest
    return file_digest(path)


if __name__ == '__main__':
    head, _ = main()
    print(json.dumps(head['summary'], indent=1))
