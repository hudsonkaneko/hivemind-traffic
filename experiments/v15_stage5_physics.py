"""v15 Stage 5 part 2: expert v3 and the Phase 3 policy each drive the PhysX car on the LA model (physics, not replay).

User decision, November 27, 2026, typed in the Claude side session: "Approve recommended" (plan v15 decision 10).
Part 2 at the recommended size: after part 1 (experiments/v15_stage5.py), an uncharged build and development drive,
then two charged drives on development row 11 (the southbound Adams focused merge, 1 controlled car among 16, seed
6952001), at most 500 s for both, counted as Stage 5 evaluation episodes (ledger phase 5), with no raise of the study
total. Never on held-out rows; never alongside a SUMO run; if the user's own Isaac Sim is open, ask first and never
close it.

How a drive runs: the episode is the worker pool's own (experiments/v15_pool._execute, the same row, driver, seeds,
gates and event counts as the SUMO records), with one change: the environment is v14's physical one
(environments/highway_flow_rl_v14_physical_env.py). The v15 drivers send their requests through the environment, which
turns them into setAcceleration and changeLane; for the controlled car those go to the physics side instead of SUMO:
- 'ideal': traffic's own IdealDriver (exact acceleration, path-following kinematics; not physics), a code check;
- 'physx': scripts/physics_drive_server.py in Isaac Sim's Python (PhysX at 120 Hz, the low-level controller, the
  provisional Americano v14 package on the LA model's deck colliders; the model file is not modified). The car's
  physics pose goes back into SUMO as its shadow. The other cars are SUMO's, drawn at their SUMO poses (kinematic).

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_stage5_physics.py ideal DRIVER         uncharged code check, no Isaac Sim (outputs/test-runs)
  python experiments/v15_stage5_physics.py development DRIVER   uncharged PhysX drive without rendering (outputs/test-runs)
  python experiments/v15_stage5_physics.py resolve              write configs/evaluation/v15-stage5-physics.json
  python experiments/v15_stage5_physics.py run                  the two charged drives, rendered at 10 Hz (once)
  python experiments/v15_stage5_physics.py report               rebuild the comparison report
DRIVER is 'expert' (rule:expert-v3) or 'policy' (l2:bc7c050f6d3da3ce).
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_phase3 as P3                  # noqa: E402
from experiments import v15_stage5 as S5                  # noqa: E402
from experiments.v15_phase1 import _file_sha, _json_sha  # noqa: E402

SCHEMA = 'highway_merge_v15_stage5_physics_v01'
ROW = 11                                       # development row 11: v12-val-la-sb-adams-model-merge-focused-1of16-medium
CASE = 'v12-val-la-sb-adams-model-merge-focused-1of16-medium'
DRIVERS = {'expert': S5.EXPERT, 'policy': S5.POLICY}
ORDER = ('expert', 'policy')                   # expert v3 first, then the candidate
TIME_CAP = 500.                                # "Approve recommended": both charged drives
RENDER_HZ = 10
OUT = ROOT / 'outputs/highway-merge-v15/stage-5/physics'
TEST_ROOT = ROOT / 'outputs/test-runs'
CONFIG = P3.CONFIG_DIR / 'v15-stage5-physics.json'
MOTION = ('controlled car: Isaac Sim PhysX rigid-body vehicle (Americano v14, provisional physics) on the LA model\'s '
          'deck colliders, driven by the low-level controller; other cars: SUMO poses (kinematic). Physics, not a replay')
AUTHORIZATION = ('Stage 5 part 2, the physics drives: user decision of November 27, 2026, typed in the Claude side '
                 'session: "Approve recommended" (plan v15 decision 10: after part 1, an uncharged build and development '
                 'drive, then expert v3 and the Phase 3 policy each drive the PhysX car on development row 11, at most '
                 '500 s for both, counted as Stage 5 evaluation episodes, with no raise of the study total; never on '
                 'held-out rows, never alongside a SUMO run; not authorized: any raise of the study total, Phase 4 step B, '
                 'Krauss runs, Isaac Lab training, merging to main).')
STOP_RULE = ('a collision, or a link or physics failure, ends that drive: it is reported and the controller is not '
             'retuned; before each charged drive, stop and ask if its estimate does not fit what is left of the 500 s, or '
             'if an Isaac Sim this run did not start is open')


def _dev_row():
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    row = v15_benchmark.development_rows(contract.build_config(P3.ROUND_INDEX))[ROW]
    if row['case']['id'] != CASE:
        raise ValueError(f'Development row {ROW} is {row["case"]["id"]}, not {CASE}')
    return row


def _sumo_record(name):
    """The driver's SUMO record on row 11: expert v3 from Phase 2 run 3, the policy from Phase 3's relabel step."""
    if name == 'expert':
        return P3._run3()[ROW], ROOT / P3.PHASE2_RUN3
    return S5._first(P3.out_dir(S5.POLICY_STEP), S5.POLICY)[ROW], P3.out_dir(S5.POLICY_STEP)


def isaac_processes():
    """Isaac Sim (Kit) processes that are running, by image name (Windows tasklist; nothing is changed)."""
    found = subprocess.run(['tasklist', '/FO', 'CSV', '/NH'], capture_output=True, text=True, check=True).stdout
    return sorted({line.split('","')[0].strip('"') + ':' + line.split('","')[1] for line in found.splitlines()
                   if line.lower().startswith('"kit.exe"') or line.lower().startswith('"isaac-sim')})


def _publish_policy(out):
    """The policy's weights in this drive's policies/ folder, where the pool's worker code loads them (hash-checked)."""
    source = P3.out_dir(S5.POLICY_STEP) / 'policies' / f'{S5.POLICY[3:]}.pt'
    target = Path(out) / 'policies' / source.name
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.is_file():
        shutil.copyfile(source, target)


def drive(name, out, physical, *, record=True):
    """One episode of DRIVERS[name] on development row 11 with the controlled car handed to `physical`.

    The pool's worker code runs it in this process: the same task key, seeds, gates and events as the SUMO record.
    With `record`, the episode also writes recording.json (the frames the video's graph is drawn from), as v14's
    physics recordings did; it reads the simulation's state and adds full_route_metrics to the report."""
    from environments.highway_flow_rl_v14_physical_env import HighwayFlowRLV14PhysicalEnv
    from experiments import run_highway_flow_rl_v14 as runner14
    from experiments import v15_pool as P
    out = Path(out)
    _publish_policy(out)
    P._worker_setup(P3.BACKEND, P3.ROUND_INDEX, out / 'policies')
    P._W['worker'] = 'physics'
    base = P._W['base']
    original, original_episode = base.env_for, base.episode
    base.env_for = lambda cfg_, row_, directory: runner14.env_for(cfg_, row_, directory,
                                                                    env_class=HighwayFlowRLV14PhysicalEnv, driver=physical)
    if record:
        base.episode = lambda *args, **kwargs: original_episode(*args, **dict(kwargs, record=True))
    try:
        result = P._execute(P.Task(P3.DEV, ROW, DRIVERS[name], run=f'stage5-physics-{name}'), 'first', out)
    finally:
        base.env_for, base.episode = original, original_episode
    return result


def _regrade(directory, report):
    """The frozen re-baselined gates (experiments/v15_regrade.py, unchanged) on one episode."""
    from experiments import v15_regrade as R
    steps = R.episode_steps(directory, report)
    R.check_parity(steps, report, where=str(directory))
    gate = R.verdict(report, R.episode_quality(steps, report, R.RULES))
    return dict(passed=bool(gate['passed']), failures=list(gate['failures']))


def _episode_dir(out, record):
    return Path(out) / 'episodes' / record['key'][:16] / record['attempt']


def compare(name, out, record, physics=None):
    """SUMO against the physics drive on row 11, for one driver."""
    sumo, sumo_run = _sumo_record(name)
    if record['key'] != sumo['key']:
        raise ValueError(f'The drive\'s task key differs from the SUMO record\'s ({record["key"][:16]}, {sumo["key"][:16]})')
    sides = {}
    for side, run_dir, rec in (('sumo', sumo_run, sumo), ('physics', Path(out), record)):
        directory = _episode_dir(run_dir, rec)
        report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
        stable = (report.get('stable_merge') or {}).get('per_agent') or {}
        sides[side] = dict(
            recorded=dict(passed=bool(rec['passed']), failures=list(rec['failures'])), rebaselined=_regrade(directory, report),
            decisions=rec['decisions'], events={k: v for k, v in rec['events'].items() if v},
            intervention_fraction=report.get('intervention_fraction'), peak_abs_jerk=report.get('peak_abs_jerk'),
            collisions=report.get('collisions'),
            stable_merge={a: dict(succeeded=s.get('stable_merge_succeeded'), entry_time_s=s.get('entry_time_s'),
                                  lateral_merge_complete_time_s=s.get('lateral_merge_complete_time_s'))
                          for a, s in stable.items() if s.get('entry_event')},
            drive_limit=report.get('v14_drive_limit'), physical_steps=report.get('physical_steps'),
            directory=directory.relative_to(ROOT).as_posix())
    return dict(driver=DRIVERS[name], name=name, key=record['key'], sides=sides, physics=physics)


def _physics_summary(server_report, log_path):
    from scripts.record_highway_flow_v14_physics import physics_summary
    log = json.loads(Path(log_path).read_text(encoding='utf-8')) if Path(log_path).is_file() else {}
    return physics_summary(server_report and server_report.get('report'), log)


def _remote(out, render_hz):
    from traffic.physics_link import RemoteDriver, server_command
    row = _dev_row()
    network = ROOT / row['case']['network']
    manifest = network.parent / 'corridor-manifest.json'
    return RemoteDriver(server_command(Path(out) / 'isaac', network=network, manifest=manifest, render_hz=render_hz),
                        log_path=Path(out) / 'isaac-server.log', cwd=ROOT)


def _fresh_test_dir(prefix):
    for n in range(1, 100):
        out = TEST_ROOT / f'{prefix}-{n:02d}'
        if not out.exists():
            return out
    raise ValueError(f'No free {prefix} directory')


def _set_priority():
    """Below-normal priority for this process, so Isaac Sim, started from it, inherits it (the user's CPU limit)."""
    from experiments import v15_pool as P
    if P.set_priority(P3.PRIORITY) != P3.PRIORITY:
        raise RuntimeError(f'This process does not run at {P3.PRIORITY} priority')


def _write(out, name, value):
    (Path(out) / name).write_text(json.dumps(value, indent=1, default=str), encoding='utf-8')


def ideal(name):
    """Uncharged code check: the physical environment with the ideal driver (no physics, no Isaac Sim)."""
    from environments.highway_flow_rl_v14_physical_env import IdealDriver
    _set_priority()
    out = _fresh_test_dir(f'v15-stage5-physics-ideal-{name}')
    started = time.monotonic()
    record = drive(name, out, IdealDriver())
    result = dict(schema=SCHEMA + '_ideal', charged=False, simulator='SUMO (libsumo) with the ideal driver: exact '
                  'acceleration and path-following kinematics for the controlled car; not physics',
                  elapsed_seconds=time.monotonic() - started, **compare(name, out, record))
    _write(out, 'summary.json', result)
    return out, result


def development(name):
    """Uncharged PhysX drive without rendering (as v14's physics-stage2-01); it measures how long a drive takes."""
    _check_no_isaac()
    _set_priority()
    out = _fresh_test_dir(f'v15-stage5-physics-dev-{name}')
    out.mkdir(parents=True)
    started = time.monotonic()
    physical = _remote(out, 0)
    server = None
    try:
        record = drive(name, out, physical)
    finally:
        server = physical.close()
    elapsed = time.monotonic() - started
    physics = _physics_summary(server, out / 'isaac/physics-log.json')
    result = dict(schema=SCHEMA + '_development', charged=False, motion=MOTION, render_hz=0, elapsed_seconds=elapsed,
                  **compare(name, out, record, physics))
    _write(out, 'summary.json', result)
    return out, result


def _check_no_isaac():
    running = isaac_processes()
    if running:
        raise RuntimeError(f'An Isaac Sim this run did not start is open ({running}); ask the user first, never close it')


# --------------------------------------------------------------------------- the charged drives

# v14's rendered drive (physics-01, 10 Hz): charged 108.2 s for 226 decisions, Isaac Sim's start included
V14_RENDERED_S_PER_DECISION = 108.2 / 226
SOURCES = ('environments/highway_flow_rl_v14_physical_env.py', 'scripts/physics_drive_server.py', 'traffic/physics_link.py',
           'traffic/physical_drive.py', 'experiments/v15_pool.py', 'traffic/v15_level2.py', 'traffic/v15_rule_expert.py',
           'experiments/v15_regrade.py', 'experiments/v15_stage5_physics.py')


class StopAndAsk(RuntimeError):
    """A stop rule of the approval was met: the run stops and the user is asked."""


def _development_drives():
    """The uncharged development drives that completed, newest last."""
    found = []
    for path in sorted(TEST_ROOT.glob('v15-stage5-physics-dev-*/summary.json')):
        summary = json.loads(path.read_text(encoding='utf-8'))
        if summary.get('schema') == SCHEMA + '_development' and summary.get('physics', {}).get('decisions'):
            found.append((path, summary))
    return found


def resolve():
    """The two charged drives' configuration and expected charge (no simulation), from the development drive."""
    from traffic import highway_flow_rl_v14_contract as contract
    drives = _development_drives()
    if not drives:
        raise ValueError('Run the uncharged development drive first (python experiments/v15_stage5_physics.py '
                         'development expert)')
    path, dev = drives[-1]
    row = _dev_row()
    cfg = contract.build_config(P3.ROUND_INDEX)
    sumo = {name: _sumo_record(name)[0] for name in ORDER}
    decisions = {name: sumo[name]['decisions'] for name in ORDER}
    # each drive: the larger of the development drive's time (unrendered) scaled to the driver's decisions and v14's
    # measured rendered rate; the second drive is checked again against the first one's measured time
    per_drive = {name: round(max(dev['elapsed_seconds'] * decisions[name] / dev['sides']['physics']['decisions'],
                                 V14_RENDERED_S_PER_DECISION * decisions[name]), 1) for name in ORDER}
    expected_seconds = round(sum(per_drive.values()), 1)
    phase, totals, _ = S5._ledger_stage5()
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    reserve = round(min(TIME_CAP, study_left), 3)
    return dict(
        schema=SCHEMA, phase=S5.PHASE, step='physics', round_index=P3.ROUND_INDEX, backend=P3.BACKEND,
        priority=P3.PRIORITY, background=P3.BACKGROUND, row=ROW, case=CASE, seed=row['seed'],
        drivers=[dict(name=name, id=DRIVERS[name], sumo=dict(key=sumo[name]['key'], report_sha=sumo[name]['report_sha'],
                                                              trace_sha=sumo[name]['trace_sha'], decisions=decisions[name],
                                                              passed=bool(sumo[name]['passed'])))
                 for name in ORDER],
        render_hz=RENDER_HZ, motion=MOTION, road_model='highway_usd/la_10_110/la_10_110_flat.usdc (unchanged)',
        sources={p: _file_sha(ROOT / p) for p in SOURCES},
        development=dict(summary=path.relative_to(ROOT).as_posix(), file_sha256=_file_sha(path), driver=dev['name'],
                         elapsed_seconds=round(dev['elapsed_seconds'], 1), render_hz=dev['render_hz'],
                         decisions=dev['sides']['physics']['decisions'],
                         physics={k: v for k, v in dev['physics'].items() if k != 'server'}),
        stop_rule=STOP_RULE,
        charge=dict(expected_decisions=sum(decisions.values()), expected_episodes=len(ORDER),
                    per_drive_seconds=per_drive, expected_clock_seconds=expected_seconds, reserve_seconds=reserve,
                    time_cap=TIME_CAP, phase_allowance=S5.ALLOWANCE, phase_decisions_used=phase['decisions'],
                    phase_seconds_used=round(phase['seconds'], 3), study_seconds_left=round(study_left, 3),
                    total_wall_seconds=cfg['budget']['total_wall_seconds'],
                    estimate_source=dict(development_drive=dev['name'],
                                         v14_rendered_seconds_per_decision=round(V14_RENDERED_S_PER_DECISION, 4),
                                         rule='each drive: the larger of the development drive\'s time scaled to its '
                                              'decisions and v14\'s rendered rate; Isaac Sim\'s start is charged')),
        authorization=AUTHORIZATION, held_out='not used: development row 11 only')


def _not_run_yet():
    if (OUT / 'charge.json').exists() or any((OUT / name).exists() for name in ORDER):
        raise ValueError(f'The charged physics drives have already run ({OUT})')
    _, _, ledger = S5._ledger_stage5()
    if any(s.get('v15_phase') == S5.PHASE and s.get('stage5_step') == 'physics' for s in ledger['segments']):
        raise ValueError('The charged physics drives already have a ledger segment')
    held = [s for s in ledger['segments'] if s.get('v15_phase') == S5.PHASE and s.get('stage5_step') == 'held-out']
    if [s['status'] for s in held] != ['complete']:
        raise StopAndAsk(f'Part 1 (the held-out check) is {[s["status"] for s in held]}, not complete: ask the user '
                         'before part 2')


def _drive_fits(elapsed, estimate, reserve):
    """Before each charged drive: its estimate must fit what is left of the reservation (never trimmed)."""
    if elapsed + estimate > reserve:
        raise StopAndAsk(f'The next drive is estimated at {estimate:.0f} s with {reserve - elapsed:.0f} s of the '
                         f'{reserve:.0f} s left: stop and ask')


def run():
    from experiments import v15_ledger as L
    _not_run_yet()
    config = json.loads(json.dumps(resolve()))
    pinned = json.loads(CONFIG.read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {CONFIG}; re-resolve and record it first')
    charge = config['charge']
    _check_no_isaac()
    _set_priority()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    results = {}
    with L.EvaluationSession(phase=S5.PHASE, output=OUT, config_sha256=_json_sha(pinned),
                             expected_decisions=charge['expected_decisions'], decision_allowance=S5.ALLOWANCE,
                             expected_clock_seconds=charge['expected_clock_seconds'],
                             reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                             intended_episodes=charge['expected_episodes'], workers=1, priority=config['priority'],
                             authorization=config['authorization'], round_index=config['round_index']) as session:
        session.segment.update(stage5_step='physics', v15_work=['evaluation', 'physics'], training_episodes=0,
                               render_hz=config['render_hz'])
        measured = None
        for name in ORDER:
            estimate = charge['per_drive_seconds'][name] if measured is None else max(charge['per_drive_seconds'][name],
                                                                                      measured)
            _drive_fits(session.elapsed(), estimate, session.allowed_seconds)
            _check_no_isaac()
            out = OUT / name
            out.mkdir(parents=True)
            started = time.monotonic()
            physical = _remote(out, config['render_hz'])
            server, record, failure = None, None, None
            try:
                record = drive(name, out, physical)
            except Exception as error:        # a link or physics failure ends that drive; it is reported
                failure = f'{type(error).__name__}: {error}'[:1000]
            try:
                server = physical.close()         # stops only the server this run started
            except Exception as error:
                failure = failure or f'closing the physics server: {type(error).__name__}: {error}'[:1000]
            seconds = time.monotonic() - started
            measured = seconds
            physics = _physics_summary(server, out / 'isaac/physics-log.json')
            if record is None:
                decisions = config['drivers'][ORDER.index(name)]['sumo']['decisions']
                session.charge_estimate(decisions, f'{name}: the drive failed before reporting ({failure})')
                results[name] = dict(name=name, driver=DRIVERS[name], failure=failure, seconds=seconds, physics=physics)
            else:
                session.add_batch(SimpleNamespace(charged_decisions=record['decisions'], canary_decisions=0,
                                                  simulated_tasks=1, canary=None, worker_seconds=seconds,
                                                  simulated_seconds=record['simulated_seconds']))
                results[name] = dict(compare(name, out, record, physics), seconds=seconds, failure=failure)
            _write(out, 'summary.json', dict(results[name], schema=SCHEMA + '_drive', motion=MOTION,
                                             render_hz=config['render_hz'], charged=True))
    segment = session.segment
    _write(OUT, 'charge.json', {k: segment.get(k) for k in ('status', 'charged_seconds', 'clock_seconds', 'decisions',
                                                            'evaluation_episodes', 'worker_seconds', 'simulated_seconds',
                                                            'expected_decisions', 'started_at_unix', 'config_sha256',
                                                            'estimated_charges')})
    return report()


def report():
    config = json.loads((OUT / 'resolved-config.json').read_text())
    drives = {name: json.loads((OUT / name / 'summary.json').read_text(encoding='utf-8'))
              for name in ORDER if (OUT / name / 'summary.json').is_file()}
    result = dict(schema=SCHEMA + '_report', motion=MOTION, row=ROW, case=CASE, seed=config['seed'],
                  render_hz=config['render_hz'], drives=drives, development=config['development'],
                  charge=json.loads((OUT / 'charge.json').read_text()) if (OUT / 'charge.json').is_file() else None,
                  held_out='not used: development row 11 only')
    _write(OUT, 'report.json', result)
    lines = ['# v15 Stage 5 part 2: the physics drives (PhysX, not a replay)', '', MOTION + '.', '',
             f'Development row {ROW} ({CASE}, seed {config["seed"]}); rendered at {config["render_hz"]} Hz.', '',
             '| Driver | Side | Recorded gates | Re-baselined gates | Interventions | Peak jerk (m/s^3) | Events |',
             '|---|---|---|---|---|---|---|']
    for name, d in drives.items():
        if d.get('failure'):
            lines.append(f'| {name} | physics | failed: {d["failure"]} | | | | |')
            continue
        for side, s in d['sides'].items():
            lines.append(f'| {name} | {side} | {s["recorded"]} | {s["rebaselined"]} | {s["intervention_fraction"]} | '
                         f'{s["peak_abs_jerk"]} | {s["events"] or "none"} |')
    lines += ['', '| Driver | Mean acceleration error (m/s^2) | Under the torque limit | Decisions at the limit | '
              'Max lateral error (m) | Wheels off the deck |', '|---|---|---|---|---|---|']
    for name, d in drives.items():
        p = d.get('physics') or {}
        e = p.get('acceleration_error') or {}
        lines.append(f'| {name} | {e.get("mean_abs")} | {e.get("unsaturated_mean_abs")} | {e.get("saturated_decisions")} | '
                     f'{p.get("max_abs_lateral_error_m")} | {p.get("wheels_off_ground_decisions")} |')
    lines += ['', f'Charge: {result["charge"]}']
    (OUT / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('ideal', 'development', 'resolve', 'run', 'report'))
    parser.add_argument('driver', nargs='?', choices=sorted(DRIVERS))
    args = parser.parse_args(argv)
    if args.command == 'resolve':
        config = resolve()
        CONFIG.write_text(json.dumps(config, indent=1, sort_keys=True) + '\n')
        print(json.dumps(dict(config=CONFIG.relative_to(ROOT).as_posix(), sha256=_json_sha(config), **config['charge']),
                         indent=1))
        return 0
    if args.command in ('run', 'report'):
        result = run() if args.command == 'run' else report()
        print(json.dumps({k: v for k, v in result.items() if k in ('charge',)}, indent=1, default=str))
        return 0
    if args.driver is None:
        parser.error(f'{args.command} needs a driver')
    out, result = (ideal if args.command == 'ideal' else development)(args.driver)
    print(json.dumps(dict(output=str(out.relative_to(ROOT)), elapsed_seconds=round(result['elapsed_seconds'], 1),
                          sides={k: {f: v[f] for f in ('recorded', 'rebaselined', 'decisions', 'events',
                                                       'intervention_fraction', 'peak_abs_jerk')}
                                 for k, v in result['sides'].items()},
                          physics={k: v for k, v in (result.get('physics') or {}).items() if k != 'server'}),
                     indent=1, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
