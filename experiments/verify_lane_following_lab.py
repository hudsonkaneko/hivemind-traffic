"""Bounded scripted Isaac Lab task validation. This module cannot launch training."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

from experiments.probe_support import ROOT, GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config

PROFILES = ('nominal', 'perturbed', 'diagnostics', 'contact-control', 'visible')
ROUTES = ('straight100', 'left_r60', 'right_r60')
EXPECTED_ACCEPTANCE = {
    'minimum_progress_m': 59.5, 'endpoint_error_max_m': 0.5,
    'stopped_speed_max_m_s': 0.05, 'hold_drift_max_m': 0.05,
    'driving_lateral_rms_max_m': 0.20, 'driving_lateral_max_m': 0.50,
    'repeat_position_tolerance_m': 0.001, 'repeat_speed_tolerance_m_s': 0.001,
    'repeat_yaw_tolerance_rad': 0.001, 'post_settle_wheel_support_fraction_min': 0.99,
    'full_episode_upright_z_min': 0.9961946980917455,
    'maximum_unsupported_gap_s': 0.10, 'full_footprint_containment_required': True,
    'rigid_contacts_max': 0,
}
KNOWN_STARTUP_ERRORS = [
    "[Error] [omni.kit.property.usd.attribute_menu_schema_registration] Cannot get API type for schema 'OmniPlaybackAPI'",
    "[Error] [omni.kit.property.material.scripts.extension] Failed to register schema handlers: Cannot get API type for schema 'OmniPlaybackAPI'",
]
EXPECTED_VISIBLE = {
    'window_width': 1440, 'window_height': 900, 'viewport_width': 960, 'viewport_height': 600,
    'capture_sim_seconds': [3.0, 12.0, 20.0], 'inspection_hold_wall_seconds': 45.0,
}


def source_files():
    """Explicit source inventory; native runs reject concurrent changes to it."""
    return [
        'experiments/lab-lane-following-validation.json',
        'experiments/verify_lane_following_lab.py',
        'experiments/lane_following_lab_validation.py',
        'experiments/probe_support.py', 'experiments/verify_physics_vehicle.py',
        'scripts/probe_lane_following_lab.py', 'scripts/probe_vehicle_lab.py',
        'hivemind/launcher.py', 'environments/__init__.py',
        'environments/lane_following_contract.py', 'environments/lane_following_lab.py',
        'environments/lane_following_scene.py',
        'environments/lane_follow_sb3.py', 'environments/physics_vehicle_contract.py',
        'traffic/physx_vehicle.py', 'traffic/physics_session.py', 'traffic/wheel_geometry.py',
        'traffic/driver_control.py', 'traffic/lane_geometry.py', 'traffic/path_following.py',
        'traffic/speed_profiles.py', 'traffic/vehicle_contacts.py', 'traffic/lane_validation.py',
        'traffic/dynamics_validation.py', 'usd/physical_scene.py', 'usd/scene_validation.py',
        'visualization/physics_road_view.py', 'scenarios/physics-road/routes.json',
    ]


def validate_config(config):
    if type(config.get('schema_version')) is not int or config['schema_version'] != 1 or config.get('training') is not False:
        raise ValueError('Only the versioned no-training validation protocol is supported')
    if (type(config.get('physics_hz')) is not int or type(config.get('control_hz')) is not int
            or config['physics_hz'] != 120 or config['control_hz'] != 60):
        raise ValueError('Validation requires 120 Hz physics and 60 Hz control')
    if config.get('route_ids') != list(ROUTES):
        raise ValueError('Validation routes must preserve the declared three-route set')
    if config.get('profile') not in PROFILES or config.get('route_id') not in ROUTES:
        raise ValueError('Select a declared validation profile and route')
    task = config.get('task', {})
    exact = {'target_speed_m_s': 3.0, 'stop_s_m': 60.0, 'episode_seconds': 35.0,
             'settle_seconds': 2.0, 'success_hold_seconds': 1.0,
             'lateral_reset_bound_m': 0.15, 'heading_reset_bound_rad': 0.035}
    if task != exact or any(isinstance(v, bool) for v in task.values()):
        raise ValueError('Task differs from the frozen version-one validation contract')
    deadline = config.get('max_process_seconds')
    if isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or not math.isfinite(deadline) or not 30 <= deadline <= 600:
        raise ValueError('Process deadline must be finite and bounded to 30..600 seconds')
    if config.get('gpu_memory_fraction_stop') != 0.9:
        raise ValueError('The 90 percent whole-GPU resource stop is required')
    if config.get('routes_file') != 'scenarios/physics-road/routes.json':
        raise ValueError('Validation uses the captured canonical routes file')
    route_file = (ROOT / config['routes_file']).resolve()
    if not route_file.is_relative_to(ROOT) or not route_file.is_file():
        raise ValueError('Routes must resolve to a project file')
    if (type(config.get('validation_seed')) is not int or config['validation_seed'] != 101
            or config.get('perturbation_seeds') != [1101, 1102]
            or any(type(seed) is not int for seed in config['perturbation_seeds'])):
        raise ValueError('Validation seeds differ from the declared protocol')
    if config.get('perturbation_corners') != [[0.15, 0.035], [-0.15, -0.035]]:
        raise ValueError('Perturbation corners differ from the declared protocol')
    acceptance = config.get('acceptance')
    if (acceptance != EXPECTED_ACCEPTANCE or
            any(type(acceptance[key]) is not type(value) for key, value in EXPECTED_ACCEPTANCE.items())):
        raise ValueError('Acceptance thresholds differ from the frozen protocol')
    if config.get('known_baseline_startup_errors') != KNOWN_STARTUP_ERRORS:
        raise ValueError('Only the two recorded baseline UI startup errors are recognized')
    visible = config.get('visible')
    if (visible != EXPECTED_VISIBLE or any(type(visible[key]) is not type(value)
                                         for key, value in EXPECTED_VISIBLE.items())
            or any(type(value) is not float for value in visible['capture_sim_seconds'])):
        raise ValueError('Visible dimensions, captures and inspection hold differ from the frozen protocol')
    return config


def kit_process_snapshot():
    """Inventory IDs only, following the existing showcase isolation check."""
    result = {'available': False, 'pids': [], 'count': None}
    if os.name != 'nt':
        return dict(result, error='Native Kit isolation check requires Windows')
    command = 'ConvertTo-Json -Compress -InputObject @(Get-Process -Name kit -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)'
    try:
        completed = subprocess.run(['powershell.exe', '-NoLogo', '-NoProfile', '-NonInteractive',
                                    '-Command', command], cwd=ROOT, text=True, capture_output=True,
                                   timeout=10, creationflags=subprocess.CREATE_NO_WINDOW, check=True)
        pids = json.loads(completed.stdout)
        if not isinstance(pids, list) or any(type(pid) is not int or pid <= 0 for pid in pids):
            raise ValueError('Invalid Kit inventory')
        return dict(available=True, pids=sorted(set(pids)), count=len(set(pids)))
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        return dict(result, error=repr(error))


def classify_runtime_log(text, known_messages):
    errors, known, unexpected = [], [], []
    seen = set()
    ready = False
    for line in text.splitlines():
        ready = ready or 'LAB_LANE_TASK_READY' in line
        if '[Error]' not in line:
            continue
        errors.append(line)
        match = next((message for message in known_messages if line.endswith(message)), None)
        if match is not None and match not in seen and not ready:
            known.append(line)
            seen.add(match)
        else:
            unexpected.append(line)
    return {'runtime_error_lines': errors, 'known_baseline_startup_errors': known,
            'unexpected_runtime_errors': unexpected,
            'usd_reference_warning_count': text.count('Unexpected reference count')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', choices=ROUTES, default='straight100')
    parser.add_argument('--profile', choices=PROFILES, default='nominal')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    config = json.loads((ROOT / 'experiments/lab-lane-following-validation.json').read_text(encoding='utf-8'))
    config.update(route_id=args.route, profile=args.profile)
    validate_config(config)
    sources = source_files()
    missing = [path for path in sources if not (ROOT / path).is_file()]
    if missing:
        raise FileNotFoundError(missing)
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps({'runtime': str(runtime), 'config': config, 'sources': sources,
                          'launches_training': False}, indent=2))
        return 0
    output, manifest = run_package('lab_lane_following', config, sources)
    command = [str(runtime), str(ROOT / 'scripts/probe_lane_following_lab.py'), '--output', str(output)]
    child = None
    result = {'passed': False, 'training_started': False}
    started = time.perf_counter()
    try:
        manifest.update(command=command, isaac_version=(runtime.parent / 'VERSION').read_text().strip(),
                        kit_before=kit_process_snapshot(), gpu_before=gpu_sample())
        write_json(output / 'manifest.json', manifest)
        if not manifest['kit_before']['available'] or manifest['kit_before']['count'] != 0:
            raise RuntimeError('Another Kit process is active or its inventory is unavailable')
        if manifest['gpu_before']['used_mib'] / manifest['gpu_before']['total_mib'] >= config['gpu_memory_fraction_stop']:
            raise RuntimeError('Preflight whole-GPU memory has reached the resource stop')
        reason = None
        with (output / 'runtime.log').open('w', encoding='utf-8') as log, GpuMonitor() as monitor:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            while child.poll() is None:
                if time.perf_counter() - started > config['max_process_seconds']:
                    reason = 'Process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib'] / monitor.rows[-1]['total_mib'] >= config['gpu_memory_fraction_stop']:
                    reason = 'Whole-GPU memory resource stop'
                if reason:
                    stop_child(child)
                    break
                time.sleep(0.2)
        probe = output / 'probe-result.json'
        result = json.loads(probe.read_text(encoding='utf-8')) if probe.exists() else {'passed': False, 'error': 'No probe result'}
        result.update(exit_code=child.returncode, stop_reason=reason, wall_process_seconds=time.perf_counter() - started,
                      gpu_peak_mib=max((r['used_mib'] for r in monitor.rows), default=None),
                      gpu_scope='Whole desktop GPU, sampled approximately once per second',
                      gpu_monitor_errors=monitor.errors,
                      **classify_runtime_log((output / 'runtime.log').read_text(encoding='utf-8', errors='replace'),
                                             config['known_baseline_startup_errors']))
        write_json(output / 'gpu-samples.json', monitor.rows)
        result['passed'] = bool(result.get('passed') and child.returncode == 0 and not reason
                                and monitor.rows and not monitor.errors and not result['unexpected_runtime_errors']
                                and not result['usd_reference_warning_count'])
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if child is not None and child.poll() is None:
            stop_child(child)
        result['kit_after'] = kit_process_snapshot()
        result['source_changed_during_run'] = [path for path, digest in manifest['source_hashes'].items()
            if not (ROOT / path).is_file() or hashlib.sha256((ROOT / path).read_bytes()).hexdigest() != digest]
        result['passed'] = bool(result.get('passed') and not result['source_changed_during_run']
                                and result['kit_after']['available'] and result['kit_after']['count'] == 0)
        result['training_started'] = False
        finish(output, manifest, result)
    print('LAB_LANE_FOLLOWING_RUN=' + str(output), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
