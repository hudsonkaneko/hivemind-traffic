"""v15 decision 20: an offline look at the first-decision jerk in the saved physics drives (plan v15, "Physics gap
options"; user decision of November 27, 2026, relayed by the main Claude session: "Yes, as recommended": "an offline
look at the first-decision jerk on rows 53 and 67, from code and logs only"). No fix is built.

Reads only saved logs (no simulator; the drives are physics, PhysX in Isaac Sim, recorded earlier). For each
drive folder it compares
- the gate's realized acceleration: SUMO's getAcceleration of the shadow, as written to level2.jsonl ('a'),
- the physics log's measured acceleration: (speed at the end - speed at the start of the advance) / 0.1,
  where both speeds are the physics body's horizontal forward speed (scripts/physics_drive_server.py read()),
and splits the gap at the first decision into the spawn speed step (SUMO speed at the handoff vs the body's
horizontal speed right after spawn) and the physics car's own first-decision tracking error.

The drives' logs are under outputs/ (not in Git). The result is written to
documentation/evidence/v15-first-decision-jerk/.
Usage: python experiments/v15_first_decision_check.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'documentation/evidence/v15-first-decision-jerk'
BASE = ROOT / 'outputs/highway-merge-v15'
DRIVES = ['physics-gap/row11-c1', 'physics-gap/row11-c2', 'physics-gap/row25-stage5', 'physics-gap/row25-c2',
          'physics-gap/row39-stage5', 'physics-gap/row39-c2', 'physics-gap/row53-stage5', 'physics-gap/row53-c2',
          'physics-gap/row67-stage5', 'physics-gap/row67-c2', 'stage-5/physics/policy', 'stage-5/physics/expert']
DT = .1


def level2(directory):
    files = sorted(Path(directory).rglob('level2.jsonl')) or sorted(Path(directory).rglob('expert.jsonl'))
    if len(files) != 1:
        raise RuntimeError(f'{directory}: {len(files)} level2.jsonl files')
    rows = [json.loads(line) for line in files[0].read_text(encoding='utf-8').splitlines() if line.strip()]
    return {round(r['t'], 1): r for r in rows if r.get('agent') == 'agent_0'}


def sumo_dir(drive):
    summary = json.loads((drive / 'summary.json').read_text(encoding='utf-8'))
    d = (summary.get('sides') or {}).get('sumo', {}).get('directory')
    return (ROOT / d) if d else None, summary


def check(name):
    drive = BASE / name
    report = json.loads((drive / 'isaac/physics-report.json').read_text(encoding='utf-8'))
    log = json.loads((drive / 'isaac/physics-log.json').read_text(encoding='utf-8'))['rows']
    spawn = report['spawn']
    l2 = level2(drive / 'episodes')
    t0 = round(spawn['requested']['time'], 1)
    t1 = round(t0 + DT, 1)
    v_sumo0 = spawn['requested']['speed']                     # SUMO's speed at the handoff = shadow's speed
    v_body0 = spawn['after_start']['speed']                   # the body's horizontal speed after spawn
    pitch = spawn['pitch_rad']
    first = log[0]
    if abs(first['time'] - t1) > 1e-6:
        raise RuntimeError(f'{name}: first physics row at {first["time"]}, expected {t1}')
    gate_a1 = l2[t1]['a']
    gate_a0 = l2[t0]['a']
    out = dict(drive=name, controller=report.get('controller'), lane=first.get('lane_id'),
               spawn_time=t0, spawn_pitch_rad=round(pitch, 5), grade_used_first=first.get('grade_used'),
               v_sumo_at_handoff=round(v_sumo0, 5), v_level2_t0=l2[t0]['v'], v_body_after_spawn=round(v_body0, 5),
               body_over_sumo=round(v_body0 / v_sumo0, 6), cos_spawn_pitch=round(math.cos(pitch), 6),
               speed_t1_physics=round(first['speed'], 5), speed_t1_level2=l2[t1]['v'],
               commanded_t0=round(first['commanded_acceleration'], 4),
               physics_measured_t1=round(first['measured_acceleration'], 4),
               physics_recomputed_t1=round((first['speed'] - v_body0) / DT, 4),
               gate_realized_t1=gate_a1, gate_recomputed_t1=round((first['speed'] - v_sumo0) / DT, 4),
               gate_realized_t0=gate_a0,
               spawn_step_m_s2=round((v_body0 - v_sumo0) / DT, 4),
               predicted_spawn_step_m_s2=round(-v_sumo0 * (1 - math.cos(pitch)) / DT, 4),
               tracking_error_t1=round(first['measured_acceleration'] - first['commanded_acceleration'], 4),
               gate_jerk_t1=round((gate_a1 - gate_a0) / DT, 3),
               jerk_if_body_reference=round((first['measured_acceleration'] - gate_a0) / DT, 3),
               jerk_if_exact=round((first['commanded_acceleration'] - gate_a0) / DT, 3),
               request_t1=l2[t1]['request'])
    # every later decision: does the gate's value equal the physics log's? (level2 rounds to 4 decimals)
    diffs = []
    for row in log[1:]:
        t = round(row['time'], 1)
        if t in l2:
            diffs.append((abs(l2[t]['a'] - row['measured_acceleration']), t))
    out['later_max_abs_gate_minus_physics'] = round(max(diffs)[0], 5) if diffs else None
    out['later_max_at_t'] = max(diffs)[1] if diffs else None
    out['later_decisions_compared'] = len(diffs)
    # tracking errors of the next few decisions and the largest anywhere
    errors = [(round(r['measured_acceleration'] - r['commanded_acceleration'], 4), round(r['time'], 1)) for r in log]
    out['tracking_error_t2_t5'] = [e for e, _ in errors[1:5]]
    out['max_abs_tracking_error_later'] = max(((abs(e), t) for e, t in errors[1:]), default=None)
    out['first_rows'] = [dict(t=round(r['time'], 1), cmd=round(r['commanded_acceleration'], 4),
                              meas=round(r['measured_acceleration'], 4), v=round(r['speed'], 4),
                              drive_nm=round(r['drive_nm'], 2), brake_nm=round(r['brake_nm'], 2), bias_n=round(r['bias_n'], 1),
                              vz=round(r['vertical_speed'], 4), pitch=round(r['pitch_rad'], 5),
                              grade_used=r.get('grade_used'), jounce=[round(j, 4) for j in r['jounce']],
                              gate_a=l2.get(round(r['time'], 1), {}).get('a'))
                         for r in log[:4]]
    out['after_start'] = dict(vz=round(spawn['after_start']['vertical_speed'], 4),
                              expected_vz=round(v_sumo0 * math.sin(pitch), 4),
                              pitch=round(spawn['after_start']['pitch_rad'], 5),
                              jounce=[round(j, 4) for j in spawn['after_start']['jounce']],
                              start=spawn.get('start'))
    # the SUMO record of the same row
    sdir, summary = sumo_dir(drive)
    if sdir is not None and sdir.exists():
        s2 = level2(sdir)
        out['sumo_record'] = [dict(t=t, v=s2[t]['v'], a=s2[t]['a'], request=s2[t]['request'])
                              for t in (t0, t1, round(t1 + DT, 1)) if t in s2]
    out['gate_peak_abs_jerk'] = (summary.get('metrics') or {}).get('peak_abs_jerk')
    out['sumo_peak_abs_jerk'] = (summary.get('sumo_metrics') or {}).get('peak_abs_jerk')
    # where the gate's realized jerk is largest in this drive (from level2, 4-decimal 'a')
    times = sorted(l2)
    jerks = [(abs(l2[b]['a'] - l2[a]['a']) / DT, b) for a, b in zip(times, times[1:]) if round(b - a, 1) == DT]
    jerks.sort(reverse=True)
    out['top_gate_jerks'] = [(round(j, 3), t) for j, t in jerks[:3]]
    return out


def main():
    results = []
    for name in DRIVES:
        try:
            results.append(check(name))
        except Exception as exc:          # report and continue: some folders may lack a file
            results.append(dict(drive=name, error=repr(exc)))
    OUT.mkdir(parents=True, exist_ok=True)
    target = OUT / 'first_decision_check.json'
    target.write_text(json.dumps(results, indent=1) + '\n', encoding='utf-8', newline='\n')
    keys = ['controller', 'lane', 'spawn_pitch_rad', 'v_sumo_at_handoff', 'v_body_after_spawn', 'body_over_sumo',
            'cos_spawn_pitch', 'commanded_t0', 'physics_measured_t1', 'gate_realized_t1', 'spawn_step_m_s2',
            'predicted_spawn_step_m_s2', 'tracking_error_t1', 'gate_jerk_t1', 'jerk_if_body_reference', 'jerk_if_exact',
            'later_max_abs_gate_minus_physics', 'later_max_at_t', 'tracking_error_t2_t5', 'max_abs_tracking_error_later',
            'gate_peak_abs_jerk', 'sumo_peak_abs_jerk', 'top_gate_jerks', 'sumo_record']
    for r in results:
        print('==', r['drive'])
        if 'error' in r:
            print('  error', r['error'])
            continue
        for k in keys:
            print(f'  {k}: {r.get(k)}')


if __name__ == '__main__':
    main()
