"""Bounded native PhysX identification and frozen MAPPO transfer; no optimizer.

Launch through run_mappo_physical_calibration.py. Pure helpers remain importable
in the traffic environment; Isaac imports occur only inside the native worker.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import itertools
import json
import math
from pathlib import Path
import platform
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.probe_support import write_json


def validate_config(cfg):
    if cfg.get('version') != 'mappo-physical-adapter-v01' or cfg.get('training_enabled') is not False:
        raise ValueError('This runner accepts only frozen v01 evaluation with training disabled')
    for key in ('physics_hz', 'control_hz', 'policy_hz', 'render_hz'):
        if type(cfg[key]) is not int or cfg[key] <= 0 or cfg['physics_hz'] % cfg[key]:
            raise ValueError('Each integer rate must divide physics_hz: ' + key)
    if cfg['physics_hz'] != 120 or cfg['control_hz'] != 60 or cfg['policy_hz'] != 10:
        raise ValueError('The existing vehicle and checkpoint rates must remain 120/60/10 Hz')
    if not 0 < cfg['max_process_seconds'] <= 900 or not 0 < cfg['max_gpu_fraction'] < 1:
        raise ValueError('Invalid resource budget')
    if cfg['lane_count'] != 3 or cfg['lane_width_m'] != 3.6 or cfg['road_length_m'] < 1000:
        raise ValueError('Only the declared three-lane straight fixture is supported')
    if not 0 < cfg['calibrated_max_speed_m_s'] <= 20:
        raise ValueError('First physical transfer is bounded at 20 m/s')
    if cfg['linear_brake_fit_max'] != .5:
        raise ValueError('Measured ordinary braking domain is restricted to pedal <= .5')
    for phase in cfg['calibration_phases']:
        if (phase['duration_s'] <= 0 or not 0 <= phase['throttle'] <= 1
                or not 0 <= phase['brake'] <= 1 or phase['throttle'] * phase['brake'] != 0
                or phase['split'] not in ('fit', 'validation', 'confirmation', 'saturation', 'excluded')):
            raise ValueError('Invalid open-loop calibration phase')
    checkpoint = (ROOT / cfg['checkpoint']).resolve()
    if not checkpoint.is_relative_to(ROOT) or checkpoint.suffix != '.pt':
        raise ValueError('Checkpoint must be a project-local policy.pt')
    for case in cfg['frozen_cases']:
        if not 0 < case['speed_limit_m_s'] <= cfg['calibrated_max_speed_m_s']:
            raise ValueError('Frozen case exceeds the calibrated speed envelope')
    if any(not 0 < speed <= cfg['calibrated_max_speed_m_s'] for speed in cfg['scripted_validation_speeds_m_s']):
        raise ValueError('Scripted validation exceeds calibration envelope')
    return cfg


def calibration_windows(rows, physics_hz=120):
    """Average 0.2 s responses after 0.5 s of unchanged pedal delivery.

    Fit and held-out control levels are predefined in the configuration. Stopped
    samples cannot identify braking gain and are explicitly excluded.
    """
    width = round(physics_hz * .2)
    result = []
    for _, grouped in itertools.groupby(rows, key=lambda r: r['phase']):
        group = list(grouped)
        for start in range(round(.5 * physics_hz), len(group) - width + 1, width):
            window = group[start:start + width]
            if min(r['speed_m_s'] for r in window) < 2 or window[0]['split'] == 'excluded':
                continue
            if any(r.get('guard_active', False) for r in window):
                continue
            result.append(dict(phase=window[0]['phase'], split=window[0]['split'],
                speed_m_s=float(np.mean([r['speed_m_s'] for r in window])),
                throttle=float(np.mean([r['throttle'] for r in window])),
                brake=float(np.mean([r['brake'] for r in window])),
                acceleration_m_s2=float(np.mean([r['acceleration_m_s2'] for r in window]))))
    return result


def fit_calibration(rows, physics_hz=120, *, linear_brake_fit_max=.5):
    """Four-parameter nonnegative least squares by enumerating active sets.

    a = throttle*gT - brake*gB - coast0 - coastSpeed*v. This identifies only
    this surrogate/configuration; it does not calibrate a real vehicle.
    """
    windows = calibration_windows(rows, physics_hz)
    fit = [r for r in windows if r['split'] == 'fit' and r['brake'] <= linear_brake_fit_max]
    heldout = [r for r in windows if r['split'] == 'validation' and r['brake'] <= linear_brake_fit_max]
    confirmation = [r for r in windows if r['split'] == 'confirmation' and r['brake'] <= linear_brake_fit_max]
    saturation = [r for r in windows if r['brake'] > linear_brake_fit_max]
    if len(fit) < 20 or len(heldout) < 4:
        raise ValueError('Insufficient moving calibration/held-out windows')
    def design(samples):
        return np.asarray([[r['throttle'], -r['brake'], -1., -r['speed_m_s']] for r in samples])
    x, y = design(fit), np.asarray([r['acceleration_m_s2'] for r in fit])
    best = None
    for active in itertools.product((False, True), repeat=4):
        if not active[0] or not active[1]:
            continue
        coefficients = np.zeros(4)
        selected = np.asarray(active)
        coefficients[selected] = np.linalg.lstsq(x[:, selected], y, rcond=None)[0]
        if np.any(coefficients < -1e-10):
            continue
        coefficients = np.maximum(coefficients, 0)
        error = float(np.mean((x @ coefficients - y) ** 2))
        if best is None or error < best[0]:
            best = error, coefficients
    if best is None:
        raise ValueError('No physically bounded calibration fit')
    mse, coefficients = best
    if not .1 < coefficients[0] < 10 or not .1 < coefficients[1] < 30:
        raise ValueError('Identified actuation gains outside the exploratory bounds')
    hy = np.asarray([r['acceleration_m_s2'] for r in heldout])
    error = design(heldout) @ coefficients - hy
    provisional_error = design(heldout) @ np.asarray([2., 6., 0., 0.]) - hy
    confirmation_error = (design(confirmation) @ coefficients
        - np.asarray([r['acceleration_m_s2'] for r in confirmation])) if confirmation else None
    parameters = dict(zip(('throttle_gain_m_s2', 'brake_gain_m_s2', 'coast_m_s2',
                          'coast_speed_gain_s_inv'), map(float, coefficients)))
    return dict(parameters=parameters, fit_rmse_m_s2=math.sqrt(mse),
        heldout_rmse_m_s2=float(np.sqrt(np.mean(error ** 2))),
        heldout_max_error_m_s2=float(np.max(np.abs(error))),
        provisional_heldout_rmse_m_s2=float(np.sqrt(np.mean(provisional_error ** 2))),
        confirmation_rmse_m_s2=float(np.sqrt(np.mean(confirmation_error ** 2))) if confirmation else None,
        confirmation_window_count=len(confirmation),
        linear_brake_fit_max=linear_brake_fit_max,
        ordinary_prediction_domain='throttle [0,1], brake [0,.5], never simultaneous; full stop/fallback brake is unmodeled',
        saturation_probe=dict(window_count=len(saturation),
            mean_acceleration_m_s2=float(np.mean([r['acceleration_m_s2'] for r in saturation])) if saturation else None,
            minimum_acceleration_m_s2=min((r['acceleration_m_s2'] for r in saturation), default=None),
            note='Full brake retained separately, excluded from unsaturated response fit and ordinary holdout'),
        fit_window_count=len(fit), heldout_window_count=len(heldout), windows=windows,
        observed_speed_range_m_s=[min(r['speed_m_s'] for r in windows), max(r['speed_m_s'] for r in windows)],
        split='Diagnostic holdout .75 throttle/.5 brake; fresh .4 brake confirmation; saturated full brake excluded',
        model='a = throttle*gT - brake*gB - coast0 - coastSpeed*v')


def _scene(stage, directory, cfg, slow_vehicle):
    """Compose decorative road/camera/traffic assets without moving the ego."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux
    from traffic.lane_geometry import LaneRoute, RouteSegment
    from visualization.physics_road_view import author_physics_road
    from usd.physical_scene import PATHS
    road = LaneRoute('physical_adapter_road', cfg['lane_count'] * cfg['lane_width_m'],
        (RouteSegment(cfg['road_length_m']),), origin_xy_m=(0., cfg['lane_width_m']))
    view = author_physics_road(stage, road, lane_dividers_m=(-1.8, 1.8),
        show_reference_centerline=False, clean_layout=True, asset_directory=directory / 'assets')
    layout = Sdf.Layer.FindOrOpen(str(directory / 'layout.usda'))
    with Usd.EditContext(stage, layout):
        UsdLux.DomeLight.Define(stage, '/World/Lighting/Dome').CreateIntensityAttr(1200.)
        camera = UsdGeom.Camera.Define(stage, PATHS.follow_camera)
        camera.CreateFocalLengthAttr(18.147562)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 10000.))
        if slow_vehicle:
            asset = Usd.Stage.CreateNew(str(directory / 'assets' / 'traffic-body.usda'))
            root = UsdGeom.Xform.Define(asset, '/TrafficBody')
            asset.SetDefaultPrim(root.GetPrim())
            Usd.ModelAPI(root).SetKind('component')
            UsdGeom.SetStageMetersPerUnit(asset, 1.)
            UsdGeom.SetStageUpAxis(asset, 'Z')
            body = UsdGeom.Cube.Define(asset, '/TrafficBody/Visuals/Body')
            body.CreateSizeAttr(1.)
            body.AddScaleOp().Set(Gf.Vec3f(4.8, 1.8, 1.4))
            body.CreateDisplayColorAttr([Gf.Vec3f(.85, .3, .08)])
            asset.GetRootLayer().Save()
            prim = stage.DefinePrim('/World/Vehicles/vehicle_slow')
            prim.GetReferences().AddReference('./assets/traffic-body.usda')
            prim.SetCustomDataByKey('movement_authority', 'scripted_analytic_traffic')
            prim.SetCustomDataByKey('collision_accounting', 'analytic_footprint_overlap; no rigid body')
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(50, 0, .8))
    layout.Save()
    # Export portable authored layers, preserving composition and references.
    view.dynamic_layer.Export(str(directory / 'debug.usda'))
    initial = Sdf.Layer.CreateNew(str(directory / 'initial-state.usda'))
    initial.TransferContent(stage.GetRootLayer())
    initial.subLayerPaths = ['./debug.usda', './physics.usda', './road-layout.usda', './layout.usda']
    initial.Save()
    entry = Usd.Stage.CreateNew(str(directory / 'world.usda'))
    entry.GetRootLayer().subLayerPaths = ['./initial-state.usda']
    entry.SetDefaultPrim(entry.GetPrimAtPath('/World'))
    UsdGeom.SetStageMetersPerUnit(entry, 1.)
    UsdGeom.SetStageUpAxis(entry, 'Z')
    entry.GetRootLayer().Save()
    errors = [str(e) for e in entry.GetCompositionErrors()]
    for path in (PATHS.chassis, PATHS.physics, PATHS.ground, '/World/Environment/Highway', PATHS.follow_camera):
        if not entry.GetPrimAtPath(path):
            errors.append('Missing ' + path)
    report = dict(passed=not errors, errors=errors,
        prims=[dict(path=str(p.GetPath()), type=p.GetTypeName(), kind=Usd.ModelAPI(p).GetKind(),
                    reference=p.HasAuthoredReferences()) for p in entry.Traverse()],
        layers=[str(Path(layer.identifier).name) for layer in entry.GetUsedLayers() if not layer.anonymous],
        ground='Existing infinite collision plane; finite road occupancy enforced by independent footprint guard',
        background='Scripted lightweight visual with analytic footprint collision accounting, not a physical agent')
    write_json(directory / 'structure.json', report)
    if errors:
        raise RuntimeError('Composed fixture failed: ' + '; '.join(errors))
    return view


def _camera(stage, state):
    from pxr import Gf, UsdGeom
    from usd.physical_scene import PATHS
    from visualization.physics_road_view import camera_pose_follow
    pose = camera_pose_follow(state['position_m'], state['yaw_rad'])
    transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*pose.eye), Gf.Vec3d(*pose.target), Gf.Vec3d(0, 0, 1)).GetInverse()
    camera = UsdGeom.Xformable(stage.GetPrimAtPath(PATHS.follow_camera))
    ops = camera.GetOrderedXformOps()
    op = ops[0] if ops else camera.AddTransformOp()
    op.Set(transform)


def _snapshot_hashes(directory):
    return {p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*.usda')}


def _scripted_request(time_s, target_speed, speed):
    ramp_s = target_speed / 1.5 + 3.
    left_s, right_s = ramp_s + 2., ramp_s + 11.
    brake_s = ramp_s + 22.
    if time_s < 2:
        return 0., 0, 'settle', (left_s, right_s, brake_s)
    if time_s >= brake_s:
        return max(-3., -speed * 1.2), 0, 'brake', (left_s, right_s, brake_s)
    acceleration = max(-2., min(1.5, (target_speed - speed) * 1.2))
    lane = 1 if -1e-9 <= time_s - left_s < .1 - 1e-9 else (-1 if -1e-9 <= time_s - right_s < .1 - 1e-9 else 0)
    return acceleration, lane, 'drive', (left_s, right_s, brake_s)


def _native_case(output, cfg, case, calibration, actor, deadline):
    from traffic.driver_control import AppliedControl, DriverCommand, DriverControlGate
    from traffic.lane_geometry import LaneRoute, RouteSegment
    from traffic.path_following import VehicleState
    from traffic.physx_vehicle import PhysxVehicle
    from traffic.rendered_physics_session import RenderedPhysicsSession
    from traffic.vehicle_contacts import VehicleContactMonitor
    from traffic.mappo_physical_adapter import AdapterConfig, LongitudinalCalibration, PhysicalPolicyAdapter, PolicyIntent
    from traffic.mappo_physical_policy import StraightRoadPolicyBridge
    from usd.physical_scene import PATHS
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from pxr import Gf, UsdGeom

    case_dir = output / case['name']
    case_dir.mkdir()
    hz, dt = cfg['physics_hz'], 1 / cfg['physics_hz']
    episode = output.name + '/' + case['name']
    rows, policy_rows, contacts, captures = [], [], [], []
    session = vehicle = view = monitor = adapter = None
    result = dict(name=case['name'], passed=False, no_training=True, optimizer_steps=0)
    static_hashes = None
    try:
        session = RenderedPhysicsSession(hz, cfg['render_hz'], physics_scene_path=PATHS.physics)
        vehicle = PhysxVehicle(session.stage, hz, scene_directory=case_dir / 'scene')
        view = _scene(session.stage, case_dir / 'scene', cfg, case.get('slow_vehicle', False))
        monitor = VehicleContactMonitor(session.stage, vehicle.path)
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError('No viewport for offscreen evidence')
        viewport.resolution = (960, 600)
        viewport.camera_path = PATHS.follow_camera
        gate = DriverControlGate(episode, 'ego')
        vehicle.apply(AppliedControl(0., 0., 1., 'initial_hold', None, False),
                      cfg['drive_torque_per_front_wheel_nm'], cfg['brake_torque_per_wheel_nm'])
        if calibration:
            cal = LongitudinalCalibration(**calibration['parameters'], max_speed_m_s=calibration['measured_max_speed_m_s'],
                max_linear_brake_fraction=cfg['linear_brake_fit_max'],
                source='Measured native PhysX identification: ' + output.name)
            adapter = PhysicalPolicyAdapter(LaneRoute('lane0', cfg['lane_width_m'],
                (RouteSegment(cfg['road_length_m']),)), episode, 'ego',
                config=AdapterConfig(physics_hz=hz, policy_hz=cfg['policy_hz'],
                    lane_width_m=cfg['lane_width_m'], lane_count=cfg['lane_count'],
                    max_speed_m_s=case.get('speed_limit_m_s', cfg['calibrated_max_speed_m_s'])), calibration=cal)
        bridge = StraightRoadPolicyBridge(road_length_m=cfg['road_length_m'], lane_count=cfg['lane_count'],
            lane_width_m=cfg['lane_width_m'], speed_limit_m_s=case.get('speed_limit_m_s', cfg['calibrated_max_speed_m_s'])) if case['kind'] == 'frozen' else None
        result['vehicle'] = vehicle.metadata
        static_hashes = _snapshot_hashes(case_dir / 'scene')
        session.start()
        state = vehicle.state()
        _camera(session.stage, state)
        previous_request = 0.
        latest_acceleration = 0.
        last_lane_change_s = -5.
        previous_lane = 0
        diagnostic = {}
        decision = None
        scheduled = [(p, sum(x['duration_s'] for x in cfg['calibration_phases'][:i]))
                     for i, p in enumerate(cfg['calibration_phases'])]
        duration = case['duration_s']
        for tick in range(round(duration * hz)):
            if time.perf_counter() > deadline:
                raise TimeoutError('Bounded native validation deadline')
            now = tick * dt
            command = None
            intent = None
            phase, split, requested, guard_active = 'drive', 'validation', 0., False
            traffic = []
            if case.get('slow_vehicle'):
                traffic = [dict(id='slow', x_m=case['slow_start_x_m'] + case['slow_speed_m_s'] * max(0, now - cfg['settle_s']),
                    y_m=0., yaw_rad=0., speed_m_s=case['slow_speed_m_s'], acceleration_m_s2=0., length_m=4.8, width_m=1.8)]
            if case['kind'] == 'calibration':
                phase_cfg, begin = next((p, start) for p, start in reversed(scheduled) if now >= start - 1e-9)
                phase, split = phase_cfg['name'], phase_cfg['split']
                throttle, brake = phase_cfg['throttle'], phase_cfg['brake']
                if state['speed_m_s'] >= cfg['calibrated_max_speed_m_s']:
                    throttle, brake, guard_active = 0., .1, True
                if tick % (hz // cfg['control_hz']) == 0:
                    command = DriverCommand(episode, 'ego', tick, tick, tick + 12, 0., throttle, brake)
            else:
                if tick % (hz // cfg['policy_hz']) == 0:
                    if case['kind'] == 'scripted':
                        requested, lane_delta, phase, lane_times = _scripted_request(now, case['target_speed_m_s'], state['speed_m_s'])
                    elif now < cfg['settle_s']:
                        requested, lane_delta, phase = 0., 0, 'settle'
                    else:
                        ego = dict(id='ego', x_m=state['position_m'][0], y_m=state['position_m'][1],
                            yaw_rad=state['yaw_rad'], speed_m_s=state['speed_m_s'], acceleration_m_s2=latest_acceleration,
                            length_m=4.8, width_m=1.8)
                        observed = bridge.observe(ego, traffic, time_s=now - cfg['settle_s'],
                            distance_travelled_m=max(0., state['position_m'][0]), previous_request_m_s2=previous_request,
                            last_lane_change_s=last_lane_change_s,
                            lane_change_active=bool(diagnostic.get('lane_change_active', False)))
                        prediction = actor.act(observed['observation'])
                        decision = bridge.filter_action(prediction, observed)
                        requested = decision['filtered_acceleration_m_s2']
                        lane_delta = decision['filtered_lane_delta']
                        previous_request = prediction['requested_acceleration_m_s2']
                        policy_rows.append(dict(tick=tick, time_s=now, **{**observed, **prediction, **decision}))
                    if lane_delta:
                        last_lane_change_s = now - cfg['settle_s']
                    intent = PolicyIntent(episode, 'ego', tick, tick, tick + 24, requested, lane_delta,
                                          lane_change_allowed=True)
                if tick % (hz // cfg['control_hz']) == 0:
                    command = adapter.command(VehicleState.from_physics(state, episode_id=episode, vehicle_id='ego', tick=tick), tick, intent)
                    command = replace(command, sequence=tick)
                    diagnostic = dict(adapter.last_diagnostics)
                    new_lane = int(diagnostic.get('lane_index', previous_lane))
                    previous_lane = new_lane
                if now < cfg['settle_s']:
                    phase = 'settle'
                    command = DriverCommand(episode, 'ego', tick, tick, tick + 12, 0., 0., 1.) if tick % 2 == 0 else None
            applied = gate.step(tick=tick, dt_s=dt, command=command)
            vehicle.apply(applied, cfg['drive_torque_per_front_wheel_nm'], cfg['brake_torque_per_wheel_nm'])
            previous = state
            session.step()
            state = vehicle.state()
            latest_acceleration = (state['speed_m_s'] - previous['speed_m_s']) / dt
            events = monitor.sample(session.sim, tick + 1)
            contacts.extend(events)
            row = dict(tick=tick + 1, sim_time_s=(tick + 1) * dt, phase=phase, split=split,
                guard_active=guard_active, **state, acceleration_m_s2=latest_acceleration,
                throttle=applied.throttle, brake=applied.brake, steering_rad=applied.steering_rad,
                fallback=applied.is_fallback, control_reason=applied.reason,
                requested_acceleration_m_s2=diagnostic.get('requested_acceleration_m_s2', requested),
                adapter=diagnostic, traffic=traffic)
            rows.append(row)
            half_width = .9 * abs(math.cos(state['yaw_rad'])) + 2.4 * abs(math.sin(state['yaw_rad']))
            lateral = state['position_m'][1]
            road_ok = (-cfg['lane_width_m'] / 2 <= lateral - half_width
                       and lateral + half_width <= (cfg['lane_count'] - .5) * cfg['lane_width_m'])
            a = cfg['acceptance']
            if now > cfg['settle_s'] and (not road_ok or state['upright_z'] < a['minimum_upright_z']
                    or not a['minimum_height_m'] < state['position_m'][2] < a['maximum_height_m']):
                raise RuntimeError('Physical road/attitude/height guard')
            if events:
                raise RuntimeError('Unexpected native rigid contact')
            if state['speed_m_s'] > cfg['calibrated_max_speed_m_s'] + a['speed_tolerance_m_s']:
                raise RuntimeError('Calibrated speed envelope exceeded')
            if state['position_m'][0] > cfg['road_length_m'] - 10:
                raise RuntimeError('Finite road endpoint reached')
            if traffic:
                other = traffic[0]
                if abs(state['position_m'][0] - other['x_m']) < 4.8 and abs(lateral - other['y_m']) < half_width + .9:
                    raise RuntimeError('Analytic traffic footprint overlap')
            if (tick + 1) % session.render_every_steps == 0:
                _camera(session.stage, state)
                if traffic:
                    prim = session.stage.GetPrimAtPath('/World/Vehicles/vehicle_slow')
                    UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(traffic[0]['x_m'], 0, .8))
                session.render()
                if cfg['capture'] and any(abs(row['sim_time_s'] - second) < dt / 2 for second in (5, 15, 25, 35, 45, 55)):
                    captures.append(capture_viewport_to_file(viewport, str(case_dir / f'capture-{round(row["sim_time_s"]):03d}s.png')))
            if (tick + 1) % (hz * 10) == 0:
                print(f'PHYSICAL_ADAPTER_CASE={case["name"]} TIME={now:.1f} SPEED={state["speed_m_s"]:.3f}', flush=True)
        for _ in range(3):
            session.render()
        result.update(passed=True, clock=session.snapshot())
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        result['completed_simulation_s'] = rows[-1]['sim_time_s'] if rows else 0.
        result['contact_count'] = len(contacts)
        result['static_files_unchanged'] = bool(static_hashes and _snapshot_hashes(case_dir / 'scene') == static_hashes)
        result['passed'] = bool(result['passed'] and result['static_files_unchanged'])
        write_json(case_dir / 'trajectory.json', rows)
        write_json(case_dir / 'policy-decisions.json', policy_rows)
        write_json(case_dir / 'contacts.json', contacts)
        result['captures'] = [p.name for p in case_dir.glob('capture-*.png')]
        if session and session.started:
            viewport.camera_path = '/OmniverseKit_Persp'
            session.render()
        vehicle = view = monitor = adapter = None
        prim = None
        if session:
            try:
                session.close()
            except Exception as error:
                result.update(passed=False, cleanup_error=repr(error))
            write_json(case_dir / 'lifecycle.json', session.lifecycle)
        write_json(case_dir / 'case-result.json', result)
    return result, rows, policy_rows


def assess_case(result, rows, decisions, case, cfg):
    """Predeclared case-level gates; failure results remain first-class evidence."""
    gates = dict(runtime_completed=result['passed'], no_contacts=result['contact_count'] == 0,
                 static_files_unchanged=result['static_files_unchanged'])
    metrics = {}
    active = [r for r in rows if r['sim_time_s'] > cfg['settle_s']]
    if case['kind'] != 'calibration':
        adapter_fallback_count = sum(bool(r['adapter'].get('is_fallback', False)) for r in active)
        driver_fallback_count = sum(r['fallback'] for r in active)
        limit = case.get('speed_limit_m_s', case.get('target_speed_m_s', cfg['calibrated_max_speed_m_s']))
        overspeed_count = sum(r['speed_m_s'] > limit + cfg['acceptance']['speed_tolerance_m_s'] for r in active)
        metrics.update(adapter_fallback_ticks=adapter_fallback_count, driver_fallback_ticks=driver_fallback_count,
            overspeed_ticks=overspeed_count, active_physics_ticks=len(active),
            full_throttle_ticks=sum(r['throttle'] >= 1 - 1e-9 for r in active),
            full_brake_ticks=sum(r['brake'] >= 1 - 1e-9 for r in active))
        gates.update(no_adapter_fallback=adapter_fallback_count == 0, no_driver_fallback=driver_fallback_count == 0,
                     no_overspeed=overspeed_count == 0)
    if rows and case['kind'] == 'scripted':
        _, _, _, (left_s, right_s, brake_s) = _scripted_request(0, case['target_speed_m_s'], 0)
        left = [r for r in rows if left_s + 7 <= r['sim_time_s'] < right_s]
        right = [r for r in rows if right_s + 7 <= r['sim_time_s'] < brake_s]
        metrics['left_lane_settled_max_error_m'] = max((abs(r['position_m'][1] - cfg['lane_width_m']) for r in left), default=None)
        metrics['right_lane_settled_max_error_m'] = max((abs(r['position_m'][1]) for r in right), default=None)
        cruise = [r for r in rows if left_s <= r['sim_time_s'] < brake_s]
        metrics['mean_cruise_speed_m_s'] = float(np.mean([r['speed_m_s'] for r in cruise])) if cruise else None
        # Aggregate derivative samples over 0.2s, excluding settle/stationary periods.
        errors = []
        width = round(cfg['physics_hz'] * .2)
        for i in range(round(3 * cfg['physics_hz']), len(rows) - width + 1, width):
            block = rows[i:i + width]
            if min(r['speed_m_s'] for r in block) > 2:
                errors.append(float(np.mean([r['acceleration_m_s2'] - r['requested_acceleration_m_s2'] for r in block])))
        metrics['acceleration_tracking_rmse_m_s2'] = float(np.sqrt(np.mean(np.square(errors)))) if errors else None
        a = cfg['acceptance']
        gates.update(left_lane=bool(left) and metrics['left_lane_settled_max_error_m'] <= a['lane_settled_error_m'],
            return_lane=bool(right) and metrics['right_lane_settled_max_error_m'] <= a['lane_settled_error_m'],
            acceleration_tracking=bool(errors) and metrics['acceleration_tracking_rmse_m_s2'] <= a['acceleration_tracking_rmse_m_s2'],
            stopped=rows[-1]['speed_m_s'] < .2,
            cruise_speed=bool(cruise) and metrics['mean_cruise_speed_m_s'] >= .9 * case['target_speed_m_s'])
    if rows and case['kind'] == 'frozen':
        override_count = sum(bool(d.get('causes')) for d in decisions)
        metrics.update(policy_decisions=len(decisions), override_count=override_count,
            override_fraction=override_count / len(decisions) if decisions else None,
            nonzero_lane_requests=sum(d.get('lane_delta', 0) != 0 for d in decisions))
        cruise = [r for r in rows if r['sim_time_s'] >= case['duration_s'] - 10]
        metrics['final_window_speed_ratio'] = float(np.mean([r['speed_m_s'] for r in cruise])) / case['speed_limit_m_s'] if cruise else None
        gates.update(actor_executed=bool(decisions), low_overrides=bool(decisions) and metrics['override_fraction'] <= cfg['acceptance']['maximum_override_fraction'],
            speed=bool(cruise) and metrics['final_window_speed_ratio'] >= cfg['acceptance']['cruise_minimum_speed_ratio'])
        if case.get('slow_vehicle'):
            final = rows[-1]
            clearance = final['position_m'][0] - final['traffic'][0]['x_m'] - 4.8
            metrics['final_passing_clearance_m'] = clearance
            gates['passed_slower_vehicle'] = clearance >= cfg['acceptance']['minimum_passing_clearance_m']
    return dict(result, gates=gates, metrics=metrics, passed=all(gates.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or not output.is_dir() or (output / 'probe-result.json').exists():
        parser.error('Use a fresh output directory from the supervisor')
    cfg = validate_config(json.loads((output / 'resolved-config.json').read_text(encoding='utf-8')))
    result = dict(passed=False, no_training=True, optimizer_steps=0, cases=[], python=platform.python_version(),
        observation_source=cfg['observation_source'], physics_authority='PhysX ego; analytic scripted background',
        limitations=['Factory surrogate calibration, not real-car system identification',
            'Straight road only, at most 20 m/s; no 60 mph qualification, curvature or merges',
            'Privileged simulator state; no camera/LiDAR perception policy',
            'Scripted lightweight background uses analytic footprint overlap; no traffic dynamics validation'])
    started = time.perf_counter()
    app = None
    try:
        checkpoint = ROOT / cfg['checkpoint']
        before = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        if before != cfg['checkpoint_sha256']:
            raise ValueError('Best checkpoint hash mismatch')
        from traffic.mappo_physical_policy import FrozenPhysicalActor
        actor = FrozenPhysicalActor(checkpoint)
        result['checkpoint_sha256'] = before
        from isaacsim import SimulationApp
        from visualization.preview_timing import rendering_profile
        app = SimulationApp({**rendering_profile(True), 'headless': cfg['headless'], 'disable_viewport_updates': False})
        from traffic.physics_session import prepare_vehicle_runtime
        result['runtime_setup'] = prepare_vehicle_runtime()
        deadline = started + cfg['max_process_seconds'] - 20
        case = dict(name='identification', kind='calibration', duration_s=sum(p['duration_s'] for p in cfg['calibration_phases']))
        report, rows, decisions = _native_case(output, cfg, case, None, actor, deadline)
        result['cases'].append(report)
        if not report['passed']:
            raise RuntimeError('Identification fixture failed; see retained case result')
        calibration = fit_calibration(rows, cfg['physics_hz'], linear_brake_fit_max=cfg['linear_brake_fit_max'])
        calibration['measured_max_speed_m_s'] = min(cfg['calibrated_max_speed_m_s'], max(r['speed_m_s'] for r in rows))
        maximum_nominal_speed = max(cfg['scripted_validation_speeds_m_s'] + [c['speed_limit_m_s'] for c in cfg['frozen_cases']])
        calibration['gates'] = dict(heldout_error=calibration['heldout_rmse_m_s2'] <= cfg['acceptance']['calibration_holdout_rmse_m_s2'],
            fresh_confirmation=calibration['confirmation_window_count'] >= 4 and calibration['confirmation_rmse_m_s2'] <= cfg['acceptance']['calibration_holdout_rmse_m_s2'],
            speed_coverage=calibration['measured_max_speed_m_s'] >= maximum_nominal_speed + cfg['acceptance']['speed_tolerance_m_s'])
        calibration['passed'] = all(calibration['gates'].values())
        write_json(output / 'calibration.json', calibration)
        result['calibration'] = {k: v for k, v in calibration.items() if k != 'windows'}
        if not calibration['passed']:
            raise RuntimeError('Identification holdout gate failed; adapter evaluation skipped')
        cases = [dict(name=f'scripted_{speed:g}mps', kind='scripted', target_speed_m_s=speed,
            duration_s=math.ceil(speed / 1.5 + 3 + 22 + 10)) for speed in cfg['scripted_validation_speeds_m_s']]
        cases.extend(dict(case, kind='frozen') for case in cfg['frozen_cases'])
        for case in cases:
            report, rows, decisions = _native_case(output, cfg, case, calibration, actor, deadline)
            assessed = assess_case(report, rows, decisions, case, cfg)
            write_json(output / case['name'] / 'case-result.json', assessed)
            result['cases'].append(assessed)
        result['checkpoint_unchanged'] = hashlib.sha256(checkpoint.read_bytes()).hexdigest() == before
        result['passed'] = all(case['passed'] for case in result['cases']) and result['checkpoint_unchanged']
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        result['total_wall_s'] = time.perf_counter() - started
        write_json(output / 'probe-result.json', result)
        print('MAPPO_PHYSICAL_RESULT=' + json.dumps(dict(passed=result['passed'], output=str(output), error=result.get('error'))), flush=True)
        if app:
            app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
