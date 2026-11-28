"""v15 movement checks M1-M4 and the four ramp situations (plan v15, "Ramp driving", decisions 13 and 15).

User decision of November 27, 2026, relayed by the main Claude session: "Yes, as recommended" (decisions 13-20).
- **Decision 13:** M1, M2 and M4 are reported checks, with M3 reported alongside. They are added to the reports of
  new runs and used in the no-regression rule of any Level 1 change. They are not gates.
- **Decision 15:** ramp cars are reported in four situations, separately, and drivers are compared only on matched
  cars (the same row and car for every driver).

The thresholds were set in the write-up before any run (plan v15, "4. Movement checks, set now, before any run").
Each reuses a project value or is marked as a choice:
- **A close leader** (M1 and M2): within the stop audit's 20 m free gap, or a headway under 3 s, or a closing time
  under 6 s. The 3 s and 6 s are twice the per-car gate's 1.5 s headway and 3 s TTC minima; the factor of 2 is a
  choice.
- **M1, hard braking on an open road:** 2.0 m/s^2 or more of deceleration (expert v3's `idm_comfort_m_s2`) for 3
  decisions in a row (the gate's avoidable-braking count) before the merge, with no close leader.
- **M2, crawling on an open ramp:** more than 1 s (the merge-quality gate's stop limit) below 5 m/s (the free-road
  stop rule's minimum target) before the merge, with no close leader.
- **M3, merge speed against traffic** (report only): at merge completion the car's speed is at least the nearby
  mainline speed minus 2 m/s (the gate's speed-match tolerance). It applies only where mainline cars were in view.
- **M4, accelerate then brake:** on the ramp, the speed rises at least 1 m/s above its lowest point so far, then
  falls at least 1 m/s again. The 1 m/s is a choice.
- **A car fails** if M1, M2 or M4 flags it, or if it never merges.

The checks need only the car's own trace and its leader, so they apply to SUMO and physics drives alike. Run on
records saved before November 27, 2026 they are post hoc (seen after the fact, not a test) and must be labelled so.

Situations (decision 15), assigned per car from a reference driver's trace (expert v3's, so every driver is grouped
the same way):
- **blocked:** a SUMO ramp car ahead stood (at most 0.2 m/s) for at least 2 s before the merge;
- **i110:** the I-110 merge case (its own network: a 2 m junction lane and 5 mainline lanes);
- **clear:** otherwise, the reference driver merged within 7 s;
- **stream:** otherwise; the reference driver waited for mainline cars.
The 2 s, 0.2 m/s and 7 s are choices made for the write-up's grouping (outputs/highway-merge-v15/ramp-driving/groups.py).
"""
from __future__ import annotations

import gzip
import json
import statistics
from pathlib import Path

SCHEMA = 'highway_merge_v15_movement_v01'
SETTINGS = dict(dt_s=0.1,
                close_free_gap_m=20., close_headway_s=3., close_ttc_s=6.,
                m1_brake_m_s2=2.0, m1_decisions=3,
                m2_speed_m_s=5., m2_limit_s=1.0,
                m3_tolerance_m_s=2.0, mainline_window_s=10.,
                m4_swing_m_s=1.0,
                blocked_speed_m_s=0.2, blocked_s=2.0, clear_merge_s=7.)
SITUATIONS = ('clear', 'stream', 'blocked', 'i110')
I110_CASE = 'v05-validation-i110-merge'


def close_leader(front, speed, s=SETTINGS):
    """A close leader for M1 and M2: within 20 m, a headway under 3 s or a closing time under 6 s."""
    if not front:
        return False
    gap, relative = front['bumper_gap_m'], front['relative_speed_m_s']
    return (gap < s['close_free_gap_m'] or gap < s['close_headway_s'] * max(speed, .1)
            or (relative < 0 and gap / -relative < s['close_ttc_s']))


def car_checks(samples, merged, mainline_speed=None, s=SETTINGS):
    """M1-M4 for one ramp car.

    samples: the car's decisions up to merge completion (or the episode's end), in order, each
    dict(v=speed m/s, a=realized acceleration m/s^2 or None, front=own-lane leader record or None).
    merged: whether it completed the merge (then the last sample is the merge decision).
    mainline_speed: the nearby mainline speed for M3, or None where no mainline car was in view."""
    run = m1 = crawl = 0
    low = high = None
    reversals = 0
    for x in samples:
        v, a, front = x['v'], x.get('a'), x.get('front')
        near = close_leader(front, v, s)
        run = run + 1 if (a is not None and a <= -s['m1_brake_m_s2'] and not near) else 0
        m1 = max(m1, run)
        crawl += v < s['m2_speed_m_s'] and not near
        if high is None:
            low = v if low is None else min(low, v)
            if v >= low + s['m4_swing_m_s']:
                high = v
        else:
            high = max(high, v)
            if v <= high - s['m4_swing_m_s']:
                reversals += 1
                low, high = v, None
    crawl_s = round(crawl * s['dt_s'], 1)
    merge_speed = samples[-1]['v'] if (merged and samples) else None
    m3 = None
    if merged and mainline_speed is not None:
        m3 = dict(merge_speed=merge_speed, mainline_speed=mainline_speed,
                  shortfall=round(mainline_speed - merge_speed, 3),
                  passed=merge_speed >= mainline_speed - s['m3_tolerance_m_s'])
    flags = dict(m1=m1 >= s['m1_decisions'], m2=crawl_s > s['m2_limit_s'], m4=reversals > 0)
    return dict(merged=bool(merged), merge_speed=merge_speed, m1_brake_run=m1, m2_crawl_s=crawl_s,
                m4_reversals=reversals, m3=m3, flags=flags, fails=bool(any(flags.values()) or not merged))


def episode_cars(episode, s=SETTINGS):
    """The ramp cars of a saved episode (actions.jsonl.gz), with their checks and situation inputs.

    A ramp car is a controlled car that had an active yield entry at some decision. Returns {agent: dict(t0,
    t_merge, checks, ramp_leader_stopped_s, mainline_speed)}. Mainline cars are the yield entry's front and rear
    cars in the 10 s before the car's last active entry that were never its own ramp leader and are not ramp cars
    (ids starting 'ramp'); the mainline speed is their median speed (an inferred classification)."""
    episode = Path(episode)
    records = [json.loads(line) for line in gzip.open(episode / 'actions.jsonl.gz', 'rt', encoding='utf-8')
               if line.strip()]
    tracks = {}
    for record in records:
        for agent, a in record['agents'].items():
            tracks.setdefault(agent, []).append((record['time'], a))
    ramp = {agent for agent, track in tracks.items()
            if any(((a.get('flow_context') or {}).get('entry') or {}).get('entry_active') for _, a in track)}
    cars = {}
    for agent in sorted(ramp):
        track = tracks[agent]
        t0 = track[0][0]
        merge = next(((t, a) for t, a in track if a.get('merge_completed')), None)
        t_end = merge[0] if merge else track[-1][0]
        same_ramp, last_active, samples, stopped = set(ramp), None, [], 0
        for t, a in track:
            if t > t_end:
                break
            v = a['v04_reward_audit']['measured_speed_m_s']
            context = a.get('flow_context') or {}
            front = ((context.get('corridors') or {}).get('own') or {}).get('front')
            if (context.get('entry') or {}).get('entry_active'):
                last_active = t
                if front:
                    same_ramp.add(front['id'])
            samples.append(dict(t=t, v=v, a=a.get('realized_acceleration'), front=front))
            if front and not front['id'].startswith('agent') and v + front['relative_speed_m_s'] <= s['blocked_speed_m_s']:
                stopped += 1
        mainline = []
        if last_active is not None:
            for t, a in track:
                if last_active - s['mainline_window_s'] <= t <= last_active:
                    entry = (a.get('flow_context') or {}).get('entry') or {}
                    v = a['v04_reward_audit']['measured_speed_m_s']
                    for x in (entry.get('front'), entry.get('rear')):
                        if x and x['id'] not in same_ramp and not x['id'].startswith('ramp'):
                            mainline.append(v + x['relative_speed_m_s'])
        mainline_speed = statistics.median(mainline) if mainline else None
        cars[agent] = dict(t0=t0, t_merge=(merge[0] - t0) if merge else None, samples=samples,
                           ramp_leader_stopped_s=round(stopped * s['dt_s'], 1), mainline_speed=mainline_speed,
                           checks=car_checks(samples, merge is not None, mainline_speed, s))
    return cars


def situation(reference, case, s=SETTINGS):
    """The situation of one ramp car from the reference driver's measures (episode_cars' entry for it)."""
    if reference['ramp_leader_stopped_s'] >= s['blocked_s']:
        return 'blocked'
    if case == I110_CASE:
        return 'i110'
    t = reference['t_merge']
    return 'clear' if t is not None and t <= s['clear_merge_s'] else 'stream'


def summary(cars, situations):
    """Per situation and driver, on matched cars only (decision 15).

    cars: {driver: {(row, agent): checks}}; situations: {(row, agent): situation}. A car counts only if every
    driver has it. Returns {situation: {driver: dict(cars, merged, m1, m2, m4, fails, m3_applies, m3_flags)}}, plus
    'all'."""
    drivers = sorted(cars)
    matched = sorted(set(situations).intersection(*(set(c) for c in cars.values()))) if drivers else []
    out = {}
    for group in SITUATIONS + ('all',):
        keys = [k for k in matched if group == 'all' or situations[k] == group]
        if not keys:
            continue
        out[group] = {}
        for d in drivers:
            checks = [cars[d][k] for k in keys]
            m3 = [c['m3'] for c in checks if c['m3'] is not None]
            out[group][d] = dict(cars=len(keys), merged=sum(c['merged'] for c in checks),
                                 m1=sum(c['flags']['m1'] for c in checks), m2=sum(c['flags']['m2'] for c in checks),
                                 m4=sum(c['flags']['m4'] for c in checks), fails=sum(c['fails'] for c in checks),
                                 m3_applies=len(m3), m3_flags=sum(not x['passed'] for x in m3))
    return out


def markdown(table, label):
    """A summary (from `summary`) as Markdown rows; `label` says SUMO or physics, and post hoc where it applies."""
    lines = [f'Movement checks ({label}; reported, not gates; matched cars only):', '',
             '| Situation (cars) | Driver | Merged | M1 | M2 | M4 | Fails any | M3 flags / applies |',
             '|---|---|---|---|---|---|---|---|']
    for group, by in table.items():
        for i, (driver, x) in enumerate(by.items()):
            name = f'{group} ({x["cars"]})' if i == 0 else ''
            lines.append(f'| {name} | {driver} | {x["merged"]} | {x["m1"]} | {x["m2"]} | {x["m4"]} | {x["fails"]} | '
                         f'{x["m3_flags"]} / {x["m3_applies"]} |')
    return '\n'.join(lines) + '\n'
