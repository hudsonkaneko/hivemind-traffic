"""v15 physics gap: low-level controller versions C1 and C2 on the PhysX car (physics, not a replay).

User decision, November 27, 2026, relayed by the main Claude session: "Yes, as recommended" (plan v15 decisions 11
and 12, "Physics gap proposal"). Decision 11 at the recommended size: C1 and C2 (traffic/physical_controller.py,
frozen before any drive) each drive development row 11 rendered at 10 Hz; the pick between them, by the rule below,
and the current controller ('stage5') then drive southbound rows 25, 39, 53 and 67 unrendered. Row 11's "before" is
the Stage 5 drive, which is not re-run. 10 drives, at most 850 s and 6,000 decisions, counted as evaluation episodes
in their own allowance (ledger phase 6), with no raise of the study total or of the diagnostic episodes. The driver
is the Phase 3 policy l2:bc7c050f6d3da3ce only. Decision 12: the gates stay as they are (realization mismatches
still count as interventions). Never held-out rows; never alongside a SUMO run; if an Isaac Sim this run did not
start is open, stop and ask and never close it. Not authorized: the northbound rows, any Level 1 change, smoothing
the road model or changing the LA model, the car's physics parameters, gate changes, installs into Isaac Sim's
Python, training, any raise, merging to main.

Each drive is the worker pool's own episode (experiments/v15_stage5_physics.drive's method: the same task key,
seeds, gates and event counts as the policy's SUMO record on that row) with the controlled car handed to
scripts/physics_drive_server.py, run with --controller. The other cars are SUMO's, drawn at their SUMO poses.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_physics_gap.py ideal ROW     uncharged code check on a southbound row, no Isaac Sim
  python experiments/v15_physics_gap.py resolve       write configs/evaluation/v15-physics-gap.json
  python experiments/v15_physics_gap.py run           the 10 charged drives (once)
  python experiments/v15_physics_gap.py report        rebuild the report from the drives' summaries
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_phase3 as P3                  # noqa: E402
from experiments import v15_stage5 as S5                  # noqa: E402
from experiments import v15_stage5_physics as S5P         # noqa: E402
from experiments.v15_phase1 import _json_sha             # noqa: E402
from traffic.physical_controller import CONTROLLERS      # noqa: E402

SCHEMA = 'highway_merge_v15_physics_gap_v01'
PHASE = '6'
DRIVER = S5.POLICY                             # 'l2:bc7c050f6d3da3ce', the Phase 3 relabel refit
ROW11 = S5P.ROW                                # development row 11, seed 6952001
SOUTHBOUND = (25, 39, 53, 67)                  # the other southbound Adams focused rows (seeds 6952002-6952005)
CASE = S5P.CASE
VERSIONS = ('c1', 'c2')                        # in this order on row 11
CURRENT = 'stage5'
ALLOWANCE, TIME_CAP = 6_000, 850.              # "Yes, as recommended": the 10 drives
RENDERED_HZ, UNRENDERED_HZ = 10, 0
OUT = ROOT / 'outputs/highway-merge-v15/physics-gap'
TEST_ROOT = S5P.TEST_ROOT
CONFIG = P3.CONFIG_DIR / 'v15-physics-gap.json'
STAGE5_BEFORE = S5P.OUT / 'policy' / 'summary.json'   # row 11's "before": the Stage 5 policy drive (not re-run)
STAGE5_RENDERED = (S5P.OUT / 'expert' / 'summary.json', S5P.OUT / 'policy' / 'summary.json')
STAGE5_UNRENDERED = TEST_ROOT / 'v15-stage5-physics-dev-expert-01' / 'summary.json'
MOTION = S5P.MOTION
SOURCES = tuple(dict.fromkeys(S5P.SOURCES + ('traffic/physical_controller.py', 'experiments/v15_stage5_physics.py',
                                              'experiments/v15_physics_gap.py')))
AUTHORIZATION = ('v15 physics gap: user decision of November 27, 2026, relayed by the main Claude session: "Yes, as '
                 'recommended" (plan v15 decision 11, recommended: C1 and C2 on development row 11 rendered at 10 Hz, '
                 'then the pick and the current controller on southbound rows 25, 39, 53 and 67 unrendered; 10 drives, '
                 'at most 850 s and 6,000 decisions, evaluation episodes in their own allowance, no raise of the study '
                 'total or of the diagnostic episodes; the policy l2:bc7c050f6d3da3ce only; decision 12: keep the '
                 'gates; not authorized: the northbound rows, Level 1 changes, the road model or LA model, the car\'s '
                 'physics parameters, gate changes, held-out rows, installs into Isaac Sim\'s Python, training, any '
                 'raise, merging to main).')
PICK_RULE = ('on row 11: fewer failed gates, counted over both gate sets (recorded and re-baselined); then fewer '
             'realization flags (native_realization_changes); then C1, the smaller change. A drive that ended without '
             'a result (a link or physics failure) ranks after one with a result; if neither has one, stop and ask. '
             'The pick is not revisited after the other rows.')
STOP_RULE = ('C1 and C2 are frozen before the first drive and nothing is changed after a drive. A collision, or a link '
             'or physics failure, ends that drive; a collision with C1 or C2 also stops the rest of the run; it is '
             'reported and nothing is retuned. Before each drive: the larger of its estimate (88.4 s rendered, 70.8 s '
             'unrendered, the measured maxima) and the longest drive so far must fit what is left of the 850 s, and '
             'the larger of its row\'s SUMO decisions and the most decisions of a drive so far must fit what is left '
             'of the 6,000; otherwise stop and ask, never trim. If an Isaac Sim this run did not start is open, stop '
             'and ask, never close it. One process at below-normal priority under the shared lock; no SUMO run '
             'alongside; never held-out rows.')
READING = ('Row 11: closed if the pick passes both gate sets in PhysX. The southbound corridor (rows 11, 25, 39, 53, '
           '67): closed if the pick passes in PhysX every row the policy passes in SUMO, under each gate set, with no '
           'collision; improved if the pick passes more rows than the current controller under the re-baselined '
           'gates, loses none under either set (no row the current controller passes and the pick fails), and has no '
           'new event kind (no event kind in a pick drive that is not in the current controller\'s drive on that '
           'row); otherwise not improved. A pick drive without a result passes no gate. Reported per row, not gated: '
           'the flags and the stable-merge window intervention fraction, peak jerk, overspeed decisions, acceleration '
           'error and torque-limit decisions.')


class StopAndAsk(S5P.StopAndAsk):
    """A stop rule of the approval was met: the run stops and the user is asked."""


class RunStopped(RuntimeError):
    """A collision with C1 or C2: the rest of the run is stopped (reported, nothing retuned)."""


# --------------------------------------------------------------------------- the plan and its rules

def drive_name(row, controller):
    return f'row{row:02d}-{controller}'


def plan(pick=None):
    """The 10 drives in order: C1, C2 on row 11 (rendered); then per southbound row the current controller, the pick."""
    drives = [dict(row=ROW11, controller=c, render_hz=RENDERED_HZ, step='row11') for c in VERSIONS]
    for row in SOUTHBOUND:
        drives.append(dict(row=row, controller=CURRENT, render_hz=UNRENDERED_HZ, step='southbound'))
        drives.append(dict(row=row, controller=pick or 'pick', render_hz=UNRENDERED_HZ, step='southbound'))
    return drives


def failed_gates(result):
    """Failed gates over both gate sets of one drive's physics side (None for a drive without a result)."""
    if not result or result.get('failure') or 'sides' not in result:
        return None
    physics = result['sides']['physics']
    return len(physics['recorded']['failures']) + len(physics['rebaselined']['failures'])


def pick(results):
    """The pick between C1 and C2 from their row-11 drives (PICK_RULE)."""
    ranked = []
    for order, name in enumerate(VERSIONS):
        result = results.get(name)
        gates = failed_gates(result)
        if gates is None:
            ranked.append(((1, 0, 0, order), name))
        else:
            ranked.append(((0, gates, int(result['metrics']['flags']), order), name))
    ranked.sort()
    if ranked[0][0][0] == 1:
        raise StopAndAsk('Neither C1 nor C2 has a row-11 result: no pick can be made; stop and ask')
    return ranked[0][1]


def estimate_seconds(render_hz, longest):
    rule = 88.4 if render_hz else 70.8
    return max(rule, longest or 0.)


def check_fits(elapsed, seconds, allowed, decisions, decisions_left):
    """Before each drive: its time and decision estimates must fit what is left (never trimmed)."""
    if elapsed + seconds > allowed:
        raise StopAndAsk(f'The next drive is estimated at {seconds:.1f} s with {allowed - elapsed:.1f} s of the '
                         f'{allowed:.0f} s left: stop and ask')
    if decisions > decisions_left:
        raise StopAndAsk(f'The next drive is estimated at {decisions:,} decisions with {decisions_left:,} of the '
                         f'{ALLOWANCE:,} left: stop and ask')


def collided(result):
    return bool(result and 'sides' in result and (result['sides']['physics'].get('collisions') or 0) > 0)


def _passes(result, gates):
    return bool(result and not result.get('failure') and 'sides' in result
                and result['sides']['physics'][gates]['passed'])


def _kinds(result):
    if not result or 'sides' not in result:
        return set()
    return {k for k, v in result['sides']['physics']['events'].items() if v}


def reading(sumo, current, picked):
    """READING. sumo: {row: {'recorded': bool, 'rebaselined': bool}} (the policy in SUMO); current, picked:
    {row: drive result} for the current controller and the pick on rows 11, 25, 39, 53, 67."""
    rows = sorted(sumo)
    row11 = all(_passes(picked.get(ROW11), g) for g in ('recorded', 'rebaselined'))
    closed = (all(_passes(picked.get(r), g) for r in rows for g in ('recorded', 'rebaselined') if sumo[r][g])
              and not any(collided(picked.get(r)) for r in rows))
    passes = {who: sum(_passes(side.get(r), 'rebaselined') for r in rows) for who, side in (('current', current),
                                                                                           ('pick', picked))}
    lost = [dict(row=r, gates=g) for r in rows for g in ('recorded', 'rebaselined')
            if _passes(current.get(r), g) and not _passes(picked.get(r), g)]
    new_kinds = {r: sorted(_kinds(picked.get(r)) - _kinds(current.get(r))) for r in rows}
    new_kinds = {r: k for r, k in new_kinds.items() if k}
    improved = passes['pick'] > passes['current'] and not lost and not new_kinds
    corridor = 'closed' if closed else 'improved' if improved else 'not improved'
    return dict(row11_closed=row11, corridor=corridor, rebaselined_passes=passes, lost=lost,
                new_event_kinds=new_kinds, rows=rows)


# --------------------------------------------------------------------------- one drive

def _dev_row(row):
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    found = v15_benchmark.development_rows(contract.build_config(P3.ROUND_INDEX))[row]
    if found['case']['id'] != CASE:
        raise ValueError(f'Development row {row} is {found["case"]["id"]}, not {CASE}')
    return found


def _sumo_record(row):
    """The policy's SUMO record on a development row (Phase 3's relabel step)."""
    return S5._first(P3.out_dir(S5.POLICY_STEP), DRIVER)[row], P3.out_dir(S5.POLICY_STEP)


def drive(row, out, physical, *, run_name, record=True):
    """One episode of the policy on a development row with the controlled car handed to `physical`."""
    from environments.highway_flow_rl_v14_physical_env import HighwayFlowRLV14PhysicalEnv
    from experiments import run_highway_flow_rl_v14 as runner14
    from experiments import v15_pool as P
    out = Path(out)
    S5P._publish_policy(out)
    P._worker_setup(P3.BACKEND, P3.ROUND_INDEX, out / 'policies')
    P._W['worker'] = 'physics'
    base = P._W['base']
    original, original_episode = base.env_for, base.episode
    base.env_for = lambda cfg_, row_, directory: runner14.env_for(cfg_, row_, directory,
                                                                    env_class=HighwayFlowRLV14PhysicalEnv, driver=physical)
    if record:
        base.episode = lambda *args, **kwargs: original_episode(*args, **dict(kwargs, record=True))
    try:
        return P._execute(P.Task(P3.DEV, row, DRIVER, run=run_name), 'first', out)
    finally:
        base.env_for, base.episode = original, original_episode


def episode_metrics(directory):
    """The per-row numbers reported, not gated, from one episode's report.json."""
    report = json.loads((Path(directory) / 'report.json').read_text(encoding='utf-8'))
    window = {}
    for agent, s in ((report.get('stable_merge') or {}).get('per_agent') or {}).items():
        post = s.get('post_window') or {}
        if s.get('entry_event'):
            window[agent] = dict(succeeded=s.get('stable_merge_succeeded'), decisions=post.get('decisions'),
                                 intervention_fraction=post.get('intervention_fraction'),
                                 flags=post.get('native_realization_changes'), failure_reasons=s.get('failure_reasons'))
    return dict(flags=report.get('native_realization_changes'), override_causes=report.get('override_causes'),
                overspeed_steps=report.get('overspeed_steps'), peak_abs_jerk=report.get('peak_abs_jerk'),
                intervention_fraction=report.get('intervention_fraction'), collisions=report.get('collisions'),
                stable_merge_window=window)


def compare(row, out, record, physics=None):
    """The policy's SUMO record against one drive on a row (both gate sets, events, the per-row numbers)."""
    sumo, sumo_run = _sumo_record(row)
    if record['key'] != sumo['key']:
        raise ValueError(f'The drive\'s task key differs from the SUMO record\'s ({record["key"][:16]}, {sumo["key"][:16]})')
    sides, metrics = {}, {}
    for side, run_dir, rec in (('sumo', sumo_run, sumo), ('physics', Path(out), record)):
        directory = S5P._episode_dir(run_dir, rec)
        report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
        metrics[side] = episode_metrics(directory)
        sides[side] = dict(recorded=dict(passed=bool(rec['passed']), failures=list(rec['failures'])),
                           rebaselined=S5P._regrade(directory, report), decisions=rec['decisions'],
                           events={k: v for k, v in rec['events'].items() if v}, collisions=report.get('collisions'),
                           directory=directory.relative_to(ROOT).as_posix())
    return dict(driver=DRIVER, row=row, key=record['key'], sides=sides, metrics=metrics['physics'],
                sumo_metrics=metrics['sumo'], physics=physics)


def _remote(row, out, render_hz, controller):
    from traffic.physics_link import RemoteDriver, server_command
    network = ROOT / _dev_row(row)['case']['network']
    manifest = network.parent / 'corridor-manifest.json'
    command = server_command(Path(out) / 'isaac', network=network, manifest=manifest, render_hz=render_hz,
                             extra=('--controller', controller))
    return RemoteDriver(command, log_path=Path(out) / 'isaac-server.log', cwd=ROOT)


def _check_no_isaac():
    running = S5P.isaac_processes()
    if running:
        raise StopAndAsk(f'An Isaac Sim this run did not start is open ({running}): stop and ask; never close it')


def _check_server(server, controller):
    """The server ran the planned controller version (its report's name and gains)."""
    report = (server or {}).get('report') or {}
    gains = (((report.get('vehicle') or {}).get('spec') or {}).get('gains'))
    if report.get('controller') != controller or gains != asdict(CONTROLLERS[controller]):
        raise StopAndAsk(f'The physics server did not report controller {controller!r} with its frozen gains: stop '
                         'and ask')
    return dict(controller=controller, bias_paused_decisions=report.get('bias_paused_decisions'))


# --------------------------------------------------------------------------- uncharged: the ideal-driver check

def ideal(row):
    """Uncharged code check: the physical environment with the ideal driver on a row (no physics, no Isaac Sim)."""
    from environments.highway_flow_rl_v14_physical_env import IdealDriver
    if row not in SOUTHBOUND:
        raise ValueError(f'The ideal-driver check is for the new rows {SOUTHBOUND}')
    S5P._set_priority()
    out = S5P._fresh_test_dir(f'v15-physics-gap-ideal-row{row:02d}')
    started = time.monotonic()
    record = drive(row, out, IdealDriver(), run_name=f'physics-gap-ideal-row{row:02d}')
    result = dict(schema=SCHEMA + '_ideal', charged=False, simulator='SUMO (libsumo) with the ideal driver: exact '
                  'acceleration and path-following kinematics for the controlled car; not physics',
                  elapsed_seconds=time.monotonic() - started, **compare(row, out, record))
    S5P._write(out, 'summary.json', result)
    return out, result


def _ideal_checks():
    """The newest ideal-driver check of each southbound row; each must match the SUMO record's verdicts (both gate
    sets) and events. Its decisions are recorded, not required equal: on these rows the ideal driver's car ends 1-3
    decisions after SUMO's (0.1-0.3 s later; not investigated), where on row 11 in Stage 5 they were equal."""
    checks = {}
    for row in SOUTHBOUND:
        found = sorted(TEST_ROOT.glob(f'v15-physics-gap-ideal-row{row:02d}-*/summary.json'))
        if not found:
            raise ValueError(f'Run the ideal-driver check on row {row} first (python experiments/v15_physics_gap.py '
                             f'ideal {row})')
        summary = json.loads(found[-1].read_text(encoding='utf-8'))
        sumo, physics = summary['sides']['sumo'], summary['sides']['physics']
        same = all(sumo[k] == physics[k] for k in ('recorded', 'rebaselined', 'events'))
        if not same:
            raise ValueError(f'The ideal-driver check on row {row} differs from the SUMO record ({found[-1]})')
        checks[str(row)] = dict(summary=found[-1].relative_to(ROOT).as_posix(), file_sha256=_text_sha(found[-1]),
                                matches_sumo=same, recorded=physics['recorded'], rebaselined=physics['rebaselined'],
                                decisions=physics['decisions'], sumo_decisions=sumo['decisions'],
                                elapsed_seconds=round(summary['elapsed_seconds'], 1))
    return checks


# --------------------------------------------------------------------------- resolve and run

def _text_sha(path):
    """sha256 of a text file with CRLF read as LF (a Windows checkout's line endings do not change it)."""
    return hashlib.sha256(Path(path).read_bytes().replace(b'\r\n', b'\n')).hexdigest()


def _committed(paths):
    """The source files must be committed unchanged (C1 and C2 frozen before the first drive)."""
    status = subprocess.run(['git', 'status', '--porcelain', '--', *paths], cwd=ROOT, capture_output=True, text=True,
                            check=True).stdout.strip()
    if status:
        raise ValueError(f'Commit the sources before resolving: {status}')


def _stage5_estimates():
    rendered = max(json.loads(p.read_text(encoding='utf-8'))['seconds'] for p in STAGE5_RENDERED)
    unrendered = json.loads(STAGE5_UNRENDERED.read_text(encoding='utf-8'))['elapsed_seconds']
    return round(rendered, 1), round(unrendered, 1)


def _before_row11():
    """Row 11's "before": the Stage 5 policy drive with the current controller (not re-run)."""
    summary = json.loads(STAGE5_BEFORE.read_text(encoding='utf-8'))
    physics = summary['sides']['physics']
    return dict(summary, metrics=episode_metrics(ROOT / physics['directory']), controller=CURRENT, failure=None)


def resolve():
    """The 10 drives' configuration and expected charge (no simulation)."""
    from traffic import highway_flow_rl_v14_contract as contract
    _committed(SOURCES)
    rendered, unrendered = _stage5_estimates()
    if (rendered, unrendered) != (88.4, 70.8):
        raise ValueError(f'The measured maxima are {rendered} and {unrendered} s, not the approved 88.4 and 70.8 s')
    cfg = contract.build_config(P3.ROUND_INDEX)
    rows = (ROW11,) + SOUTHBOUND
    sumo = {}
    for row in rows:
        rec, run_dir = _sumo_record(row)
        directory = S5P._episode_dir(run_dir, rec)
        report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
        sumo[str(row)] = dict(key=rec['key'], report_sha=rec['report_sha'], trace_sha=rec['trace_sha'],
                              decisions=rec['decisions'], seed=_dev_row(row)['seed'],
                              recorded=bool(rec['passed']), rebaselined=S5P._regrade(directory, report)['passed'])
    drives = plan()
    decisions = [sumo[str(d['row'])]['decisions'] for d in drives]
    seconds = [rendered if d['render_hz'] else unrendered for d in drives]
    _, totals, _ = S5._ledger_stage5()
    phase = totals['v15'].get(PHASE, dict(decisions=0, seconds=0.))
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    before = _before_row11()
    return dict(
        schema=SCHEMA, phase=PHASE, round_index=P3.ROUND_INDEX, backend=P3.BACKEND, priority=P3.PRIORITY,
        background=P3.BACKGROUND, driver=DRIVER, case=CASE, rows=list(rows), drives=drives, sumo=sumo,
        controllers={name: dict(gains=asdict(gains), sha256=_json_sha(asdict(gains)))
                     for name, gains in CONTROLLERS.items()},
        sources={p: _text_sha(ROOT / p) for p in SOURCES},
        before_row11=dict(summary=STAGE5_BEFORE.relative_to(ROOT).as_posix(), file_sha256=_text_sha(STAGE5_BEFORE),
                          recorded=before['sides']['physics']['recorded'],
                          rebaselined=before['sides']['physics']['rebaselined'], flags=before['metrics']['flags']),
        ideal_checks=_ideal_checks(), motion=MOTION,
        road_model='highway_usd/la_10_110/la_10_110_flat.usdc (unchanged)', pick_rule=PICK_RULE, stop_rule=STOP_RULE,
        reading=READING,
        charge=dict(expected_decisions=sum(decisions), expected_episodes=len(drives),
                    expected_clock_seconds=round(sum(seconds), 1), reserve_seconds=round(min(TIME_CAP, study_left), 3),
                    time_cap=TIME_CAP, phase_allowance=ALLOWANCE, phase_decisions_used=phase['decisions'],
                    phase_seconds_used=round(phase['seconds'], 3), study_seconds_left=round(study_left, 3),
                    total_wall_seconds=cfg['budget']['total_wall_seconds'],
                    estimates=dict(rendered_seconds=rendered, unrendered_seconds=unrendered,
                                   source=dict(rendered=[p.relative_to(ROOT).as_posix() for p in STAGE5_RENDERED],
                                               unrendered=STAGE5_UNRENDERED.relative_to(ROOT).as_posix()),
                                   rule='the measured maxima whatever the row length; before each drive the larger '
                                        'of its estimate and the longest drive so far must fit')),
        authorization=AUTHORIZATION, held_out='not used: development rows 11, 25, 39, 53 and 67 only')


def _not_run_yet():
    if (OUT / 'charge.json').exists() or (OUT / 'resolved-config.json').exists():
        raise ValueError(f'The physics-gap drives have already run ({OUT})')
    _, _, ledger = S5._ledger_stage5()
    if any(s.get('v15_phase') == PHASE for s in ledger['segments']):
        raise ValueError('Phase 6 already has a ledger segment')


def run():
    from experiments import v15_ledger as L
    _not_run_yet()
    config = json.loads(json.dumps(resolve()))
    pinned = json.loads(CONFIG.read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {CONFIG}; re-resolve and record it first')
    charge = config['charge']
    _check_no_isaac()
    S5P._set_priority()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    results, longest, most, chosen, stopped = {}, None, 0, None, None
    with L.EvaluationSession(phase=PHASE, output=OUT, config_sha256=_json_sha(pinned),
                             expected_decisions=charge['expected_decisions'], decision_allowance=ALLOWANCE,
                             expected_clock_seconds=charge['expected_clock_seconds'],
                             reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                             intended_episodes=charge['expected_episodes'], workers=1, priority=config['priority'],
                             authorization=config['authorization'], round_index=config['round_index']) as session:
        session.segment.update(v15_work=['evaluation', 'physics'], physics_gap_step='drives', training_episodes=0)
        for item in plan():
            if item['controller'] == 'pick':
                if chosen is None:
                    chosen = pick({c: results[drive_name(ROW11, c)] for c in VERSIONS})
                    S5P._write(OUT, 'pick.json', dict(pick=chosen, rule=PICK_RULE, candidates={
                        c: dict(failed_gates=failed_gates(results[drive_name(ROW11, c)]),
                                flags=(results[drive_name(ROW11, c)].get('metrics') or {}).get('flags'))
                        for c in VERSIONS}))
                item = dict(item, controller=chosen)
            name = drive_name(item['row'], item['controller'])
            sumo_decisions = config['sumo'][str(item['row'])]['decisions']
            check_fits(session.elapsed(), estimate_seconds(item['render_hz'], longest), session.allowed_seconds,
                       max(sumo_decisions, most), session.decisions_left)
            _check_no_isaac()
            out = OUT / name
            out.mkdir(parents=True)
            started = time.monotonic()
            physical = server = record = failure = None
            try:
                physical = _remote(item['row'], out, item['render_hz'], item['controller'])
                record = drive(item['row'], out, physical, run_name=f'physics-gap-{name}')
            except Exception as error:        # a link or physics failure ends that drive; it is reported
                failure = f'{type(error).__name__}: {error}'[:1000]
            if physical is not None:
                try:
                    server = physical.close()     # stops only the server this run started
                except Exception as error:
                    failure = failure or f'closing the physics server: {type(error).__name__}: {error}'[:1000]
            seconds = time.monotonic() - started
            longest = max(longest or 0., seconds)
            physics = S5P._physics_summary(server, out / 'isaac/physics-log.json')
            if record is None:
                session.charge_estimate(sumo_decisions, f'{name}: the drive failed before reporting ({failure})')
                results[name] = dict(item, name=name, driver=DRIVER, failure=failure, seconds=seconds, physics=physics)
            else:
                most = max(most, int(record['decisions']))
                session.add_batch(SimpleNamespace(charged_decisions=record['decisions'], canary_decisions=0,
                                                  simulated_tasks=1, canary=None, worker_seconds=seconds,
                                                  simulated_seconds=record['simulated_seconds']))
                results[name] = dict(compare(item['row'], out, record, physics), **item, name=name, seconds=seconds,
                                     failure=failure)
            S5P._write(out, 'summary.json', dict(results[name], schema=SCHEMA + '_drive', motion=MOTION, charged=True))
            results[name]['server'] = _check_server(server, item['controller']) if server else None
            S5P._write(out, 'summary.json', dict(results[name], schema=SCHEMA + '_drive', motion=MOTION, charged=True))
            if item['controller'] in VERSIONS and collided(results[name]):
                stopped = f'{name}: a collision with {item["controller"]}; the rest of the run is stopped'
                break
        session.segment.update(stopped=stopped)
    segment = session.segment
    S5P._write(OUT, 'charge.json', {k: segment.get(k) for k in (
        'status', 'charged_seconds', 'clock_seconds', 'decisions', 'evaluation_episodes', 'worker_seconds',
        'simulated_seconds', 'expected_decisions', 'started_at_unix', 'config_sha256', 'estimated_charges', 'stopped')})
    return report()


def report():
    config = json.loads((OUT / 'resolved-config.json').read_text())
    drives = {}
    for path in sorted(OUT.glob('row*/summary.json')):
        drives[path.parent.name] = json.loads(path.read_text(encoding='utf-8'))
    chosen = json.loads((OUT / 'pick.json').read_text())['pick'] if (OUT / 'pick.json').is_file() else None
    sumo = {int(r): dict(recorded=s['recorded'], rebaselined=s['rebaselined']) for r, s in config['sumo'].items()}
    current = {ROW11: _before_row11()}
    current.update({r: drives.get(drive_name(r, CURRENT)) for r in SOUTHBOUND})
    picked = {r: drives.get(drive_name(r, chosen)) for r in (ROW11,) + SOUTHBOUND} if chosen else {}
    verdict = reading(sumo, current, picked) if chosen else None
    result = dict(schema=SCHEMA + '_report', motion=MOTION, driver=DRIVER, case=CASE, pick=chosen, reading=verdict,
                  drives=drives, before_row11=config['before_row11'],
                  charge=json.loads((OUT / 'charge.json').read_text()) if (OUT / 'charge.json').is_file() else None,
                  held_out=config['held_out'])
    S5P._write(OUT, 'report.json', result)
    lines = ['# v15 physics gap: controller versions on the PhysX car (physics, not a replay)', '', MOTION + '.', '',
             f'Driver {DRIVER}; pick: {chosen}; reading: {verdict}.', '',
             '| Row | Controller | Recorded gates | Re-baselined gates | Flags | Window interventions | Peak jerk | '
             'Overspeed | Mean accel error | Torque-limit decisions | Events | Seconds |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']

    def line(row, controller, d):
        if not d or d.get('failure'):
            return f'| {row} | {controller} | failed: {(d or {}).get("failure")} | | | | | | | | | |'
        p, m = d['sides']['physics'], d['metrics']
        window = [w.get('intervention_fraction') for w in m['stable_merge_window'].values()]
        error = ((d.get('physics') or {}).get('acceleration_error') or {})
        return (f'| {row} | {controller} | {p["recorded"]["failures"] or "pass"} | {p["rebaselined"]["failures"] or "pass"} '
                f'| {m["flags"]} | {window} | {m["peak_abs_jerk"]} | {m["overspeed_steps"]} | {error.get("mean_abs")} | '
                f'{error.get("saturated_decisions")} | {p["events"] or "none"} | {d.get("seconds")} |')
    lines.append(line(ROW11, 'stage5 (Stage 5 drive)', current[ROW11]))
    for c in VERSIONS:
        lines.append(line(ROW11, c, drives.get(drive_name(ROW11, c))))
    for r in SOUTHBOUND:
        lines.append(line(r, CURRENT, drives.get(drive_name(r, CURRENT))))
        if chosen:
            lines.append(line(r, chosen, drives.get(drive_name(r, chosen))))
    lines += ['', f'Charge: {result["charge"]}']
    (OUT / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('ideal', 'resolve', 'run', 'report'))
    parser.add_argument('row', nargs='?', type=int, choices=SOUTHBOUND)
    args = parser.parse_args(argv)
    if args.command == 'resolve':
        config = resolve()
        CONFIG.write_text(json.dumps(config, indent=1, sort_keys=True) + '\n')
        print(json.dumps(dict(config=CONFIG.relative_to(ROOT).as_posix(), sha256=_json_sha(config), **config['charge']),
                         indent=1))
        return 0
    if args.command in ('run', 'report'):
        result = run() if args.command == 'run' else report()
        print(json.dumps({k: result.get(k) for k in ('pick', 'reading', 'charge')}, indent=1, default=str))
        return 0
    if args.row is None:
        parser.error('ideal needs a southbound row')
    out, result = ideal(args.row)
    print(json.dumps(dict(output=str(out.relative_to(ROOT)), elapsed_seconds=round(result['elapsed_seconds'], 1),
                          sides={k: {f: v[f] for f in ('recorded', 'rebaselined', 'decisions', 'events')}
                                 for k, v in result['sides'].items()}), indent=1, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
