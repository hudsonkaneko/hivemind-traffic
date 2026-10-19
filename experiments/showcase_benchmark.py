"""Predeclared serial public-car display matrix; --plan never launches Isaac.

Use --run for the 11 trials, or --compare INDEX to re-evaluate retained evidence.
Comparison validity and the unchanged strict display thresholds are separate.
"""
from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

from experiments import loop_showcase_repeatability as repeat
from experiments.loop_showcase_config import assess, fleet_specs, make_config, validate_config
from experiments.loop_showcase_performance import CONTROL_FIELDS, _physical_contract
from experiments.probe_support import ROOT, finish, run_package, write_json
from experiments.showcase_controls import defaults
from traffic.showcase_performance import audit_schedule
from visualization.preview_timing import PreviewClock
from visualization.showcase_responsiveness import assess_responsiveness


PAIRS = (
    ('light-panel', 'light-hidden', 'light-visible', ('panel_hidden',)),
    ('dense-panel', 'dense-hidden', 'dense-visible', ('panel_hidden',)),
    ('display-cadence', 'dense-hidden', 'dense-20hz', ('render_hz', 'controls.render_hz')),
    ('capture-cost', 'dense-hidden', 'dense-captures', ('capture',)),
    ('evidence-cost', 'dense-hidden', 'dense-full', ('evidence_profile',)),
    ('editor-cost', 'minimal-cost', 'editor-cost', ('ui_profile',)),
    ('dense-repeat-1', 'dense-visible', 'dense-repeat-1', ()),
    ('dense-repeat-2', 'dense-visible', 'dense-repeat-2', ()),
)
LIVE_FIELDS = ('active_background_count', 'requested_background_count',
               'applied_ego_target_m_s', 'applied_traffic_speed_m_s', 'applied_following_s')
FLEET_CADENCE_FIELDS = {'episode_id', 'wheel_visual_update_every_steps', 'wheel_visual_hz'}
EXTRA_FILES = ('background-visual-checks.json', 'preview-timing.json', 'ui-event-timing.json',
               'lifecycle.json', 'startup-readiness.json', 'gpu-samples.json')
EVALUATOR_SOURCES = ['experiments/showcase_benchmark.py', 'experiments/loop_showcase_repeatability.py',
                     'experiments/loop_showcase_performance.py']


def _required_sources(config):
    from scripts.demo_loop_showcase import SOURCES
    required = set(SOURCES) | {'visualization/showcase_startup.py', 'traffic/showcase_metrics.py',
                               'visualization/highwaysim.showcase.kit', 'visualization/showcase_render_profile.py'}
    if config['version'] == 'v09':
        required.update(('experiments/showcase_controls.py', 'visualization/showcase_sidebar.py',
                         'traffic/live_showcase.py', 'usd/live_showcase_fleet.py'))
    return required


def benchmark_cases(*, drive_s=90):
    """Eleven ordered cases; density changes never become a matched pair."""
    definitions = [
        ('light-visible', 'v09', 4, False, 30, False, 'presentation', 'editor'),
        ('light-hidden', 'v09', 4, True, 30, False, 'presentation', 'editor'),
        ('dense-visible', 'v09', 80, False, 30, False, 'presentation', 'editor'),
        ('dense-hidden', 'v09', 80, True, 30, False, 'presentation', 'editor'),
        ('dense-20hz', 'v09', 80, True, 20, False, 'presentation', 'editor'),
        ('dense-captures', 'v09', 80, True, 30, True, 'presentation', 'editor'),
        ('dense-full', 'v09', 80, True, 30, False, 'full', 'editor'),
        ('editor-cost', 'v07', 80, False, 30, False, 'presentation', 'editor'),
        ('minimal-cost', 'v07', 80, False, 30, False, 'presentation', 'showcase'),
        ('dense-repeat-1', 'v09', 80, False, 30, False, 'presentation', 'editor'),
        ('dense-repeat-2', 'v09', 80, False, 30, False, 'presentation', 'editor'),
    ]
    result = []
    for name, version, count, hidden, hz, capture, evidence, ui in definitions:
        controls = {**defaults(), 'traffic_count': count, 'render_hz': hz} if version == 'v09' else None
        config = make_config(version, gui=True, paced=True, drive_s=drive_s, vehicle='public',
            statistics=True, controls=controls, panel_hidden=hidden, render_hz=hz,
            capture=capture, evidence=evidence, ui_profile=ui, cpu_threads=8)
        result.append(dict(name=name, config=config))
    return result


def kit_process_snapshot():
    """Read only process IDs; never terminate another viewer or expose command lines."""
    observed = dict(wall_monotonic_s=time.perf_counter(), available=False, pids=[], count=None)
    if os.name != 'nt':
        return dict(observed, error='Kit process isolation check requires Windows')
    try:
        command = "ConvertTo-Json -Compress -InputObject @(Get-Process -Name kit -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)"
        completed = subprocess.run(['powershell.exe', '-NoLogo', '-NoProfile', '-NonInteractive',
            '-Command', command], cwd=ROOT, text=True, capture_output=True, timeout=10,
            creationflags=subprocess.CREATE_NO_WINDOW, check=True)
        pids = json.loads(completed.stdout)
        if not isinstance(pids, list) or any(type(pid) is not int or pid <= 0 for pid in pids):
            raise ValueError('Invalid Kit process inventory')
        return dict(observed, available=True, pids=sorted(set(pids)), count=len(set(pids)))
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        return dict(observed, error=str(error))


def _quiet(snapshot):
    return (isinstance(snapshot, dict) and snapshot.get('available') is True
            and snapshot.get('pids') == [] and type(snapshot.get('count')) is int and snapshot['count'] == 0)


def desktop_load_snapshot():
    """Read system CPU counters and RAM without installing monitoring packages."""
    result = dict(available=False, wall_monotonic_s=time.perf_counter())
    if os.name != 'nt':
        return dict(result, error='Windows desktop counters unavailable')
    class MemoryStatus(ctypes.Structure):
        _fields_ = [('length', wintypes.DWORD), ('memory_load_percent', wintypes.DWORD)] + [
            (name, ctypes.c_ulonglong) for name in ('total_physical_bytes', 'available_physical_bytes',
                'total_pagefile_bytes', 'available_pagefile_bytes', 'total_virtual_bytes',
                'available_virtual_bytes', 'available_extended_virtual_bytes')]
    try:
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.GetSystemTimes.argtypes = [ctypes.POINTER(wintypes.FILETIME)] * 3
        kernel.GetSystemTimes.restype = wintypes.BOOL
        kernel.GlobalMemoryStatusEx.argtypes = [ctypes.POINTER(MemoryStatus)]
        kernel.GlobalMemoryStatusEx.restype = wintypes.BOOL
        times = [wintypes.FILETIME() for _ in range(3)]
        memory = MemoryStatus(); memory.length = ctypes.sizeof(memory)
        if (not kernel.GetSystemTimes(*[ctypes.byref(value) for value in times])
                or not kernel.GlobalMemoryStatusEx(ctypes.byref(memory))):
            raise OSError('Windows desktop counter query failed: ' + str(ctypes.get_last_error()))
        counters = {name: (value.dwHighDateTime << 32) + value.dwLowDateTime
                    for name, value in zip(('idle_100ns', 'kernel_100ns', 'user_100ns'), times)}
        return dict(result, available=True, **counters,
                    memory_load_percent=int(memory.memory_load_percent),
                    total_physical_bytes=int(memory.total_physical_bytes),
                    available_physical_bytes=int(memory.available_physical_bytes))
    except (OSError, AttributeError) as error:
        return dict(result, error=str(error))


def desktop_load_summary(load):
    before, after = load.get('desktop_before', {}), load.get('desktop_after', {})
    result = dict(before=before, after=after, average_system_cpu_busy_percent=None,
                  scope='Whole desktop including this trial; other application workload is uncontrolled')
    if before.get('available') is True and after.get('available') is True:
        elapsed = sum(after[key] - before[key] for key in ('kernel_100ns', 'user_100ns'))
        idle = after['idle_100ns'] - before['idle_100ns']
        if elapsed > 0 and 0 <= idle <= elapsed:
            result['average_system_cpu_busy_percent'] = 100 * (1 - idle / elapsed)
    return result


def _fingerprints(paths):
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in paths}


def _flatten(mapping, prefix=''):
    result = {}
    for key, value in mapping.items():
        path = prefix + key
        if isinstance(value, dict):
            result.update(_flatten(value, path + '.'))
        else:
            result[path] = value
    return result


def _benchmark_physical_contract(run):
    """Normalize the public Factory assembly's verified run-local packaging root.

    The shared normalizer already handles prepared-car scene_directory and
    runtime layer identifiers. The public Factory author records the equivalent
    location as assembly_directory. Retain every other composition/physics/asset
    field; a suffix match or an arbitrary external directory is never sufficient.
    This follows the existing ROOT/relative_path contract for reassessment: moved
    evidence with stale absolute paths must be explicitly resolved, not guessed.
    """
    scene = _physical_contract(run)
    composition = scene.get('vehicle', {}).get('scene_composition', {})
    if 'assembly_directory' in composition:
        recorded = composition['assembly_directory']
        expected = (repeat.ROOT / run['relative_path'] / 'scene').resolve().as_posix()
        if not isinstance(recorded, str) or recorded.replace('\\', '/') != expected:
            raise ValueError('Public vehicle assembly directory is outside its recorded run packaging')
        composition['assembly_directory'] = 'scene/'
    return scene


def _load_run(path):
    run = repeat.load_run(path)
    directory = repeat.resolve_run(path)
    hashes = run['manifest'].get('output_hashes', {})
    for name in EXTRA_FILES:
        if name not in hashes:
            raise ValueError('Missing required hashed benchmark output: ' + name)
        data = repeat._contained_file(directory, name).read_bytes()
        if hashlib.sha256(data).hexdigest() != hashes[name]:
            raise ValueError('Benchmark output hash mismatch: ' + name)
        run[name] = json.loads(data)
    run['background_visual_checks'] = run['background-visual-checks.json']
    return run


def _visual_validation(run):
    cfg, summary, rows = run['config'], run['summary'], run['trajectory']
    expected = [tick for tick in range(1, len(rows) + 1) if audit_schedule(cfg, tick)['visual']]
    checks = run['background_visual_checks']
    if not isinstance(checks, list) or [r.get('tick') for r in checks] != expected:
        raise ValueError('Visual audits do not cover the exact declared schedule')
    pool_count = len(fleet_specs(cfg))
    interval = cfg['physics_hz'] // cfg['render_hz']
    for row in checks:
        if (row.get('passed') is not True or row.get('count') != pool_count
                or row.get('publications') != row['tick'] // interval + 1
                or not math.isclose(row['native_target_time_s'], rows[row['tick'] - 1]['sim_time_s'],
                                    rel_tol=0, abs_tol=1e-6)):
            raise ValueError('Visual audit identity/publication/clock failed')
        for key, bound in (('max_position_error_m', .002), ('max_yaw_error_rad', .0001)):
            if type(row.get(key)) not in (int, float) or not math.isfinite(row[key]) or not 0 <= row[key] <= bound:
                raise ValueError('Visual pose tolerance failed: ' + key)
        key = 'visibility_matches_active' if cfg['version'] == 'v09' else 'all_proxies_visible'
        if row.get(key) is not True:
            raise ValueError('Visual active/dormant membership failed')
    expected_native = [tick for tick in range(1, len(rows) + 1) if audit_schedule(cfg, tick)['native']]
    if summary.get('evidence_policy', {}).get('native_audit_ticks') != expected_native:
        raise ValueError('Native audit schedule incomplete')


def _timing(run):
    """Recompute unchanged acceptance from retained frame/event timestamps."""
    cfg, summary = run['config'], run['summary']
    frames = run['preview-timing.json']
    if not isinstance(frames, list) or len(frames) != cfg['duration_s'] * cfg['render_hz']:
        raise ValueError('Incomplete preview timing samples')
    previous = 0.
    for index, row in enumerate(frames, 1):
        values = [row[key] for key in ('sim_time_s', 'active_wall_s', 'lag_s', 'frame_interval_s', 'rtf')]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError('Non-finite preview timing')
        sim, wall, lag, interval, rtf = values
        if (wall <= 0 or wall < previous or not math.isclose(sim, index / cfg['render_hz'], abs_tol=1e-6)
                or not math.isclose(lag, max(0., wall - sim), abs_tol=1e-6)
                or not math.isclose(interval, wall - previous, abs_tol=1e-6)
                or not math.isclose(rtf, sim / wall, abs_tol=1e-6)):
            raise ValueError('Inconsistent preview frame clocks')
        previous = wall
    clock = PreviewClock(paced=cfg['paced'], clock=lambda: 0.)
    clock.frames = frames
    preview = clock.summary(expected_simulation_time_s=cfg['duration_s'],
                            expected_frames=cfg['duration_s'] * cfg['render_hz'])
    event = run['ui-event-timing.json']
    responsiveness = assess_responsiveness(event['application_frames'], event['native_completions'],
        wall_s=preview['active_wall_s'], simulation_s=cfg['duration_s'], render_hz=cfg['render_hz'], complete=True)
    smooth = bool(preview['soft_realtime_passed']
                  and preview['p95_frame_interval_s'] <= 1.5 / cfg['render_hz']
                  and preview['max_frame_interval_s'] <= .2 and responsiveness['passed']
                  and summary.get('display_qualification_exclusions') == [])
    if (not repeat._same(preview, summary.get('preview_timing'))
            or not repeat._same(responsiveness, summary.get('responsiveness'))
            or summary.get('smooth_display_passed') is not smooth):
        raise ValueError('Stored timing qualification differs from raw timestamp reassessment')
    return dict(preview=preview, responsiveness=responsiveness, smooth_display_passed=smooth,
                gpu_peak_mib=summary.get('gpu_peak_mib'), gpu_before=run['manifest'].get('gpu_before'),
                gpu_sample_count=len(run['gpu-samples.json']))


def _validate_run(run):
    cfg, summary, manifest = run['config'], run['summary'], run['manifest']
    if run.get('load_errors'):
        raise ValueError('Evidence integrity errors: ' + repr(run['load_errors']))
    validate_config(cfg)
    if (cfg['version'] not in ('v07', 'v09') or cfg['vehicle_profile'] != 'public'
            or not cfg['gui'] or not cfg['paced'] or not cfg.get('traffic_statistics')
            or cfg.get('control_smoke') or cfg.get('control_validation') or cfg.get('profile_render')):
        raise ValueError('Run is outside the declared public paced benchmark protocol')
    if (manifest.get('status') != 'passed' or not manifest.get('finished_utc')
            or not repeat._same(manifest.get('config'), cfg) or not repeat._hashes(manifest.get('source_hashes'))):
        raise ValueError('Completed passed manifest/config/source evidence required')
    if not _required_sources(cfg).issubset(manifest['source_hashes']):
        raise ValueError('Required simulation/presentation source snapshots missing')
    command = manifest.get('command', [])
    if (len(command) < 4 or command[-2] != '--output'
            or Path(command[-1].replace('\\', '/')).name != run['run_id']):
        raise ValueError('Distinct supervised launch evidence required')
    if (summary.get('passed') is not True or type(summary.get('exit_code')) is not int or summary['exit_code'] != 0
            or summary.get('source_changed_during_run') != [] or summary.get('runtime_error_lines') != []
            or summary.get('usd_reference_warning_count') != 0 or summary.get('gpu_monitor_errors') != []
            or summary.get('cleanup_verified') is not True or summary.get('stop_reason') is not None
            or any(value for key, value in summary.items() if key == 'error' or key.endswith('_error'))):
        raise ValueError('Run process, source, runtime error or cleanup acceptance failed')
    if not run['lifecycle.json'] or run['lifecycle.json'][-1].get('event') != 'closed':
        raise ValueError('Native lifecycle did not close')
    if run['startup-readiness.json'].get('passed') is not True:
        raise ValueError('Startup readiness did not pass')
    gates = summary.get('gates', {})
    if not gates or any(value is not True for value in gates.values()):
        raise ValueError('Physical/scene acceptance gate failed')
    passes, changes = summary['passed_vehicle_ids'], summary['completed_lane_changes']
    if len(set(passes)) != len(passes) or summary['completed_passes'] != len(passes):
        raise ValueError('Pass identities/count inconsistent')
    reassessed = assess(run['trajectory'], cfg, summary['contact_report_count'], passes, changes)
    if not reassessed['passed']:
        raise ValueError('Raw physical trace failed reassessment: ' + repr(reassessed['gates']))
    for key, expected in (('completed_passes', len(passes)), ('completed_lane_changes', changes)):
        counts = [row.get(key) for row in run['trajectory']]
        if (any(type(value) is not int or value < 0 for value in counts) or counts[-1] != expected
                or any(b < a for a, b in zip(counts, counts[1:]))):
            raise ValueError('Trajectory event counter mismatch: ' + key)
    if summary.get('display_qualification_exclusions') != []:
        raise ValueError('Paused, interactive or otherwise excluded timing trial')
    if cfg['version'] == 'v09':
        sidebar, live = summary.get('sidebar', {}), summary.get('live_traffic', {})
        if (sidebar.get('display_events') != [] or sidebar.get('event_count') != 0
                or sidebar.get('active') != cfg['controls'] or sidebar.get('pending') != cfg['controls']
                or live.get('lifecycle_events') != [] or live.get('no_process_restart') is not True
                or live.get('final_settings') != cfg['controls']):
            raise ValueError('Live requests/restarts changed fixed benchmark conditions')
        for row in run['trajectory']:
            if any(row.get(key) != cfg['controls']['traffic_count'] for key in LIVE_FIELDS[:2]):
                raise ValueError('Live active/requested population changed')
    _visual_validation(run)
    samples = run['gpu-samples.json']
    if not isinstance(samples, list) or not samples:
        raise ValueError('Desktop GPU samples missing')
    for row in samples:
        if (not isinstance(row, dict) or not 0 <= row['used_mib'] < row['total_mib'] * cfg['max_gpu_fraction']
                or not math.isfinite(row['used_mib']) or not math.isfinite(row['total_mib'])):
            raise ValueError('Desktop GPU memory guard failed')
    return dict(runtime=repeat._runtime_identity(run), timing=_timing(run),
                physical_reassessment=reassessed)


def compare_pair(first, second, *, allowed_config_fields=(), first_load=None, second_load=None):
    """Compare loaded, hash-checked runs without granting timing success by validity."""
    failures, metrics = [], {}
    result = dict(valid=False, failures=failures, run_ids=[first['run_id'], second['run_id']],
                  timing={}, physical_valid=False, load_qualified=False,
                  acceptance=dict(position_tolerance_m=.02, speed_tolerance_m_s=.02,
                                  controls='exact every physics tick'))
    try:
        if first['run_id'] == second['run_id']:
            raise ValueError('Distinct fresh processes required')
        for label, run in (('first', first), ('second', second)):
            metrics[label] = _validate_run(run)
        result['physical_valid'] = True
        if first['config']['version'] != second['config']['version']:
            raise ValueError('Only same-version comparisons are allowed')
        a, b = _flatten(first['config']), _flatten(second['config'])
        differences = sorted(key for key in set(a) | set(b) if not repeat._same(a.get(key), b.get(key)))
        result['config_differences'] = differences
        if set(differences) - set(allowed_config_fields):
            raise ValueError('Undeclared configuration differences: ' + repr(differences))
        if first['manifest']['source_hashes'] != second['manifest']['source_hashes']:
            raise ValueError('Captured source hashes differ')
        if metrics['first']['runtime'] != metrics['second']['runtime']:
            raise ValueError('Runtime/machine identity differs')
        contracts = [_benchmark_physical_contract(run) for run in (first, second)]
        for contract in contracts:
            fleet = contract.get('fleet', {})
            if not repeat._hashes(fleet.get('asset_hashes')):
                raise ValueError('Fleet asset hash contract missing')
            for key in FLEET_CADENCE_FIELDS:
                fleet.pop(key, None)
        if not repeat._same(*contracts):
            raise ValueError('Physical scene/assets/fleet contract differs')
        rows = first['trajectory'], second['trajectory']
        if len(rows[0]) != len(rows[1]):
            raise ValueError('Trajectory lengths differ')
        fields = CONTROL_FIELDS + (LIVE_FIELDS if first['config']['version'] == 'v09' else ())
        max_position = max_speed = 0.
        for tick, (left, right) in enumerate(zip(*rows), 1):
            if any(key not in left or key not in right or not repeat._same(left[key], right[key]) for key in fields):
                raise ValueError('Per-tick controls/live targets differ at tick ' + str(tick))
            max_position = max(max_position, math.dist(left['position_m'], right['position_m']))
            max_speed = max(max_speed, abs(left['speed_m_s'] - right['speed_m_s']))
        result.update(max_position_difference_m=max_position, max_speed_difference_m_s=max_speed)
        if max_position > .02 or max_speed > .02:
            raise ValueError('Matched motion tolerance exceeded')
        result['load_qualified'] = all(isinstance(load, dict) and _quiet(load.get('before'))
                                      and _quiet(load.get('after')) for load in (first_load, second_load))
        if not result['load_qualified']:
            raise ValueError('Clean pre/post external Kit process checks are required')
        result['valid'] = True
    except (KeyError, ValueError, TypeError, AttributeError, IndexError, OverflowError) as error:
        failures.append(str(error))
    result['timing'] = {name: value['timing'] for name, value in metrics.items()}
    result['scope'] = 'Matched observed display costs; native notifications are not monitor scanouts'
    result['load_scope'] = 'Pre/post external Kit.exe inventory and whole-desktop GPU memory; transient or non-Kit work is not ruled out'
    result['desktop_load'] = {name: desktop_load_summary(load) for name, load in
        (('first', first_load), ('second', second_load)) if isinstance(load, dict)}
    result['inference_limit'] = 'One sample per variant under observed local desktop load; no statistical or causal benefit claim'
    if result['valid']:
        before = metrics['first']['timing']['preview']['active_wall_s']
        after = metrics['second']['timing']['preview']['active_wall_s']
        result['observed_wall_change_percent'] = 100 * (after / before - 1)
        result['both_strict_display_qualified'] = all(value['timing']['smooth_display_passed'] for value in metrics.values())
    return result


def evaluate_index(index):
    expected = {case['name']: case['config'] for case in benchmark_cases()}
    if (len(index.get('cases', [])) != len(expected)
            or {case['name']: case['config'] for case in index.get('cases', [])} != expected):
        raise ValueError('Index matrix/configuration differs from the declared 11-case protocol')
    trials = index.get('trials', [])
    if len({row['case'] for row in trials}) != len(trials):
        raise ValueError('Duplicate trial case identities')
    if any(row['case'] not in expected for row in trials):
        raise ValueError('Undeclared trial case identity')
    loaded, failures, trial_map = {}, {}, {row['case']: row for row in trials}
    for name, trial in trial_map.items():
        try:
            if name not in expected or trial.get('status') != 'completed':
                raise ValueError('Trial did not complete')
            run = _load_run(ROOT / trial['run_directory'])
            if run['config'] != expected[name]:
                raise ValueError('Captured configuration differs from declared case')
            loaded[name] = run
        except (OSError, ValueError, KeyError, TypeError) as error:
            failures[name] = str(error)
    comparisons = {}
    for name, left, right, allowed in PAIRS:
        if left not in loaded or right not in loaded:
            comparisons[name] = dict(valid=False, failures=[failures.get(key, 'Missing trial: ' + key)
                for key in (left, right) if key not in loaded])
        else:
            comparisons[name] = compare_pair(loaded[left], loaded[right], allowed_config_fields=allowed,
                first_load=trial_map[left].get('load'), second_load=trial_map[right].get('load'))
    return dict(passed=index.get('status') == 'completed' and all(row['valid'] for row in comparisons.values()), comparisons=comparisons,
                trial_load_errors=failures, all_trials_completed=len(loaded) == len(expected),
                scope='Passed means valid comparisons, not strict real-time display qualification')


def run_benchmark():
    """Serial native execution is opt-in; preserve every trial and stop unsafe work."""
    from scripts import demo_loop_showcase as demo
    from hivemind.launcher import load_config
    from hivemind.showcase_setup import check_showcase

    cases = benchmark_cases()
    sources = sorted(set(EVALUATOR_SOURCES).union(*(_required_sources(case['config']) for case in cases)))
    output, manifest = run_package('showcase_benchmark', dict(schema_version=1, cases=cases), sources)
    baseline = dict(manifest['source_hashes'])
    index = dict(schema_version=1, cases=cases, trials=[], status='running', stop_reason=None)
    write_json(output / 'index.json', index)
    try:
        for case in cases:
            trial = dict(case=case['name'], status='checking', load={})
            index['trials'].append(trial)
            trial['load']['before'] = kit_process_snapshot()
            trial['load']['desktop_before'] = desktop_load_snapshot()
            write_json(output / 'index.json', index)
            if _fingerprints(sources) != baseline:
                raise RuntimeError('Source drift before trial ' + case['name'])
            if not _quiet(trial['load']['before']):
                raise RuntimeError('Competing or unverified Kit processes before trial ' + case['name'])
            runtime, setup = check_showcase(ROOT, case['config'], load_config(ROOT))
            trial.update(status='running', setup=setup)
            write_json(output / 'index.json', index)
            result = demo.run_once(case['config'], runtime)
            trial.update(status='completed', run_directory=result['run_directory'],
                         physical_passed=result.get('passed', False),
                         smooth_display_passed=result.get('smooth_display_passed', False))
            trial['load']['after'] = kit_process_snapshot()
            trial['load']['desktop_after'] = desktop_load_snapshot()
            write_json(output / 'index.json', index)
            if (_fingerprints(sources) != baseline or result.get('source_changed_during_run')):
                raise RuntimeError('Source drift during trial ' + case['name'])
            if not _quiet(trial['load']['after']):
                raise RuntimeError('Competing or unverified Kit processes after trial ' + case['name'])
            if result.get('stop_reason') or result.get('gpu_monitor_errors') or result.get('cleanup_verified') is not True:
                raise RuntimeError('Resource/process/cleanup guard after trial ' + case['name'])
            if result.get('passed') is not True:
                raise RuntimeError('Physical/scene/process acceptance failed for trial ' + case['name'])
        index['status'] = 'completed'
    except (Exception, KeyboardInterrupt) as error:
        index.update(status='stopped', stop_reason=f'{type(error).__name__}: {error}')
        if index['trials'] and index['trials'][-1]['status'] != 'completed':
            index['trials'][-1].update(status='failed', error=index['stop_reason'])
    finally:
        write_json(output / 'index.json', index)
        try:
            result = evaluate_index(index)
        except Exception as error:
            result = dict(passed=False, error=repr(error))
        result.update(index='index.json', matrix_status=index['status'], stop_reason=index['stop_reason'])
        result['passed'] = bool(result.get('passed') and index['status'] == 'completed')
        finish(output, manifest, result)
    return output, result


def compare_index(path):
    path = Path(path).resolve()
    root = (ROOT / 'outputs/showcase_benchmark').resolve()
    if path.name != 'index.json' or path.parent.parent != root:
        raise ValueError('Index must be in a canonical outputs/showcase_benchmark run')
    manifest = json.loads((path.parent / 'manifest.json').read_text())
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if manifest.get('output_hashes', {}).get('index.json') != digest:
        raise ValueError('Benchmark index integrity is unverified or changed')
    return evaluate_index(json.loads(path.read_text()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--plan', action='store_true')
    mode.add_argument('--run', action='store_true')
    mode.add_argument('--compare', type=Path, metavar='INDEX')
    args = parser.parse_args(argv)
    if args.run:
        output, result = run_benchmark()
        print(json.dumps(dict(output=str(output), **result), indent=2, allow_nan=False))
        return 0 if result['passed'] else 1
    if args.compare:
        try:
            result = compare_index(args.compare)
        except (OSError, ValueError, KeyError, TypeError) as error:
            result = dict(passed=False, error=str(error))
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['passed'] else 1
    print(json.dumps(dict(cases=benchmark_cases(), comparisons=PAIRS, native_launches=False), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
