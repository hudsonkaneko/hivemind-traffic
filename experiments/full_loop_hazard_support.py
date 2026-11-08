"""CPU evidence replay for one physical ego and twelve full-road backgrounds.

This reads saved measurements; it never imports Isaac, loads a policy or launches
physics. Shared pure task code supplies reward/observation semantics; independent
per-tick geometry checks cover every object, including objects outside actor slots.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import struct

from environments.continuous_loop_hazard_contract import ContinuousFullLoopHazardTask
from environments.full_loop_hazard_contract import FullLoopHazardTask
from environments.lane_following_contract import _state, make_reset_spec
from traffic.bypass_validation import OrientedBox, box_clearance
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import load_routes, wrap_yaw
from traffic.lane_validation import footprint_lateral_bound
from traffic.road_hazards import MeasuredObject, make_hazard_schedule
from usd.physical_scene import PATHS

ROOT = Path(__file__).resolve().parents[1]
REPLAY_TOLERANCE = 1e-7
REPLAY_SOURCES = (
    'environments/lane_following_contract.py', 'environments/physics_vehicle_contract.py',
    'environments/loop_hazard_contract.py', 'environments/full_loop_hazard_contract.py',
    'environments/continuous_loop_hazard_contract.py', 'traffic/lane_geometry.py',
    'traffic/lane_validation.py', 'traffic/road_hazards.py', 'traffic/path_following.py',
    'traffic/bypass_validation.py', 'traffic/driver_control.py', 'usd/physical_scene.py',
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def same(actual, expected, *, tolerance=REPLAY_TOLERANCE):
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        return type(actual) is type(expected) and actual == expected
    if isinstance(expected, (int, float)):
        return (type(actual) in (int, float) and math.isfinite(actual)
                and math.isclose(actual, expected, rel_tol=0., abs_tol=tolerance))
    if isinstance(expected, dict):
        return (isinstance(actual, dict) and actual.keys() == expected.keys()
                and all(same(actual[k], v, tolerance=tolerance) for k, v in expected.items()))
    if isinstance(expected, (list, tuple)):
        return (isinstance(actual, (list, tuple)) and len(actual) == len(expected)
                and all(same(a, b, tolerance=tolerance) for a, b in zip(actual, expected)))
    return False


def float32(value):
    if isinstance(value, (list, tuple)):
        return [float32(x) for x in value]
    return struct.unpack('f', struct.pack('f', value))[0]


def _frames(raw, tick, *, background, stage):
    count = 12 if background else int(stage != 'clear')
    require(isinstance(raw, list) and len(raw) == count, 'Incomplete measured object inventory')
    objects = []
    for item in raw:
        require(isinstance(item, dict), 'Object measurement must be a dictionary')
        try:
            obj = MeasuredObject(**item)
        except (ValueError, TypeError) as exc:
            raise ValueError('Malformed measured object frame') from exc
        require(abs(obj.timestamp_s-tick/120) < 1e-9, 'Stale/future object sample')
        identity = bool(re.fullmatch(r'background_\d{3}_g001', obj.object_id)) if background else obj.object_id == 'hazard_000'
        require(identity and (obj.length_m, obj.width_m) == ((4.8, 1.9) if background else (1., 1.)), 'Object identity or dimensions changed')
        objects.append(obj)
    require(len({o.object_id for o in objects}) == len(objects), 'Duplicate object identity')
    return tuple(objects)


def _applied(raw):
    require(isinstance(raw, dict) and type(raw.get('is_fallback')) is bool, 'Missing applied controls')
    try:
        return AppliedControl(**{k: raw[k] for k in AppliedControl.__dataclass_fields__})
    except (TypeError, KeyError) as exc:
        raise ValueError('Malformed applied controls') from exc


class FullRoadAccumulator:
    """Bounded-memory replay of complete 60Hz rows, including both120Hz samples."""

    def __init__(self, reset_record, *, route, stage, continuous=False, seed=None):
        require(type(continuous) is bool and stage in ('clear', 'static', 'crossing'), 'Invalid replay protocol')
        require(not continuous or stage != 'static', 'Static stopping does not support rolling windows')
        self.reset = deepcopy(reset_record)
        self.initial_route, self.stage, self.continuous = route, stage, continuous
        self.task = (ContinuousFullLoopHazardTask(route, stage=stage) if continuous
                     else FullLoopHazardTask(route, stage=stage))
        self.reset_count = reset_record.get('reset_count')
        require(reset_record.get('episode_tick') == 0 and type(self.reset_count) is int and self.reset_count >= 1, 'Explicit physical reset required')
        require(not continuous or self.reset_count == 1, 'Continuous execution requires one physical reset')
        self.serial = reset_record['episode_serial']
        require(type(self.serial) is int and self.serial >= 1, 'Positive initial episode identity required')
        spec = reset_record['reset_spec']
        seed = spec['seed'] if seed is None else seed
        require(same(spec, asdict(make_reset_spec(route, seed=seed, episode_index=spec['episode_index'], perturb=True))), 'Seeded reset specification changed')
        self.schedule = reset_record['hazard_schedule']
        # Schedule sampling is not used as measured motion: only its immutable
        # declared identity is checked here. Native object poses drive replay.
        preflight = reset_record.get('hazard_background_preflight')
        if preflight is None:
            candidates = [make_hazard_schedule(stage, seed, spec['episode_index'], route=route, side=side).as_dict()
                          for side in ((None, 'left', 'right') if stage == 'crossing' else (None,))]
            require(any(same(self.schedule, value) for value in candidates), 'Original hazard schedule differs from declared seed')
        self._set_objects(reset_record, 0)
        self.ids = tuple(o.object_id for o in self.backgrounds)
        self.initial_global_tick = reset_record.get('background_global_tick')
        require(type(self.initial_global_tick) is int and self.initial_global_tick >= 0, 'Initial background clock required')
        if preflight is not None:
            self._preflight(preflight, seed, spec['episode_index'])
        state = _state(reset_record['physics_state'])
        require(math.dist(state['position_m'][:2], (spec['x_m'], spec['y_m'])) <= 1e-4
                and abs(wrap_yaw(state['yaw_rad']-spec['yaw_rad'])) <= 1e-5, 'Native reset pose differs from requested seed')
        require(same(state['velocity_m_s'], [0., 0., 0.]) and same(reset_record['wheel_rotation_speed_rad_s'], [0.]*4), 'Reset velocity/wheel state was not cleared')
        observation = self.task.initialize(state, AppliedControl(0., 0., 1., 'reset', None, False))
        require(same(reset_record['observation'], observation), 'Initial measured observation cannot be replayed')
        self.observation = observation
        self.previous = state
        self.tick = self.steps = self.contacts = self.contact_points = self.unsupported = 0
        self.supported = self.support_samples = self.longest_unsupported = 0
        self.minimum = None
        self.max_displacement = self.max_velocity_delta = self.max_footprint = 0.
        self.first_failure = None
        self.last = None
        self.reward = 0.
        self.masked_steps = 0
        self.rolls = []
        self.wrapper_mode = None

    def _preflight(self, recorded, seed, reset_index):
        from experiments.full_scene_hazard_preflight import choose_schedule
        require('requested_side' in recorded, 'Explicit preflight side is required')
        plan = recorded['background_plan']
        schedule, replay = choose_schedule(self.stage, seed, reset_index, self.initial_route,
                                           plan, side=recorded['requested_side'])
        require(same(schedule.as_dict(), self.schedule), 'Accepted candidate schedule differs from preflight replay')
        for key, value in replay.items():
            require(same(recorded.get(key), value), 'Scenario preflight replay differs: '+key)
        require(recorded.get('phase') == 'before_physical_reset'
                and same(plan['global_start_s'], self.initial_global_tick/120), 'Preflight phase or global background clock differs')
        native = recorded.get('native_current_background', {})
        require(native is not None and native.get('passed') is True, 'Native preflight plan check missing')
        measured = {o.object_id: o for o in self.backgrounds}
        require(set(measured) == {e['object_id'] for e in plan['entries']}, 'Preflight background membership differs')
        for entry in plan['entries']:
            obj = measured[entry['object_id']]
            angle, radius = entry['station_m']/500, 500+3.7*entry['lane']
            require(math.dist((obj.x_m, obj.y_m), (radius*math.cos(angle), radius*math.sin(angle))) <= .002
                    and abs(wrap_yaw(obj.yaw_rad-angle-math.pi/2)) <= .0001, 'Preflight planned pose differs from measured background')

    def _set_objects(self, record, tick):
        self.hazards = _frames(record.get('hazard_frames'), tick, background=False, stage=self.stage)
        self.backgrounds = _frames(record.get('background_frames'), tick, background=True, stage=self.stage)
        if hasattr(self, 'ids'):
            require(tuple(o.object_id for o in self.backgrounds) == self.ids, 'Background membership/order changed')
        self.task.set_objects(self.hazards, now_s=tick/120)
        self.task.set_background_objects(self.backgrounds, now_s=tick/120)

    def _sample(self, sample, fallback):
        tick = sample['tick']
        global_tick = self.initial_global_tick+tick
        require(type(tick) is int and tick == self.tick+1 and sample.get('global_tick') == global_tick, 'Native/global physics ticks are discontinuous')
        self._set_objects(sample, tick)
        state = _state(sample['physics_state'])
        events = sample.get('contacts')
        require(isinstance(events, list) and type(sample.get('contact_count')) is int and sample['contact_count'] == len(events), 'Native contacts/count disagree')
        for event in events:
            require(PATHS.chassis in (event.get('actor0'), event.get('actor1')) and type(event.get('contact_count')) is int and event['contact_count'] >= 0, 'Contact does not name measured ego and point count')
            self.contact_points += event['contact_count']
        self.contacts += len(events)
        self.unsupported = self.unsupported+1 if tick > 240 and not all(state['wheel_on_ground']) else 0
        require(sample.get('unsupported_ticks') == self.unsupported, 'Support-loss duration differs from native samples')
        if tick > 240:
            self.support_samples += 1
            self.supported += all(state['wheel_on_ground'])
        self.longest_unsupported = max(self.longest_unsupported, self.unsupported)
        route = self.task.route
        projection = route.project(*state['position_m'][:2])
        heading = wrap_yaw(state['yaw_rad']-projection.point.yaw_rad)
        # Same declared eight-corner 3D chassis envelope, including roll/pitch
        # and curvature remainder; a planar rectangle would miss departures.
        footprint = footprint_lateral_bound(state, projection, (4.8, 1.8, 1.4), self.task.max_curvature)
        self.max_footprint = max(self.max_footprint, footprint)
        forward = sum(v*d for v, d in zip(state['velocity_m_s'][:2], (math.cos(projection.point.yaw_rad), math.sin(projection.point.yaw_rad))))
        failures = [(bool(events), 'rigid_contact'), (fallback, 'command_fallback'),
            (self.unsupported > 12, 'wheel_support_loss'), (state['upright_z'] < math.cos(math.radians(5)), 'excessive_tilt'),
            (not 0 <= state['position_m'][2] <= 2, 'unsafe_height'), (footprint > route.width_m/2, 'lane_departure'),
            (abs(heading) > 1., 'heading_outside_envelope'), (state['speed_m_s'] > 4., 'overspeed'),
            (forward < -.2, 'reverse_motion'), (projection.s_m < -1., 'before_route_start'), (projection.s_m > 60.5, 'endpoint_overshoot')]
        reason = next((why for failed, why in failures if failed), None)
        require(sample.get('guard_reason') == reason, 'Recorded guard differs from raw native geometry/contact/support')
        ego = OrientedBox(*state['position_m'][:2], state['yaw_rad'], 4.8, 1.8)
        geometry = [box_clearance(ego, OrientedBox(o.x_m, o.y_m, o.yaw_rad, o.length_m, o.width_m)) for o in (*self.hazards, *self.backgrounds)]
        distance = min(g['clearance_m'] for g in geometry)
        overlap = any(g['overlap'] for g in geometry)
        saved = sample.get('all_object_clearance', {})
        require(same(saved.get('clearance_m'), distance) and saved.get('overlap') is overlap
                and saved.get('obstacle_count') == len(geometry), 'All-object clearance differs from exact oriented boxes')
        self.minimum = distance if self.minimum is None else min(self.minimum, distance)
        native = sample.get('native_background', {})
        require(native.get('passed') is True and native.get('count') == 80 and native.get('commanded_steps') == global_tick
                and same(native.get('target_time_s'), global_tick/120)
                and all(type(native.get(k)) in (int, float) and 0 <= native[k] <= limit
                        for k, limit in [('max_position_error_m', .002), ('max_yaw_error_rad', .0001), ('max_speed_error_m_s', .1)]), 'Background native motion clock/inventory failed')
        displacement = math.dist(state['position_m'][:2], self.previous['position_m'][:2])
        self.max_displacement = max(self.max_displacement, displacement)
        self.max_velocity_delta = max(self.max_velocity_delta, math.dist(state['velocity_m_s'], self.previous['velocity_m_s']))
        # Use the existing speed-envelope displacement bound at native cadence;
        # contact failures remain valid failure evidence, not hidden exclusions.
        if reason is None and overlap:
            reason = 'object_overlap'
        if reason is None and displacement > 4/120+.005:
            reason = 'state_discontinuity'
        if reason is not None and self.first_failure is None:
            self.first_failure = dict(tick=tick, reason=reason, physics_state=deepcopy(sample['physics_state']))
        self.tick, self.previous = tick, state
        return events

    def update(self, row):
        require(self.last is None or not (self.last['terminated'] or self.last['truncated']), 'Records continue after terminal')
        wrapped = 'transition' in row
        require(self.wrapper_mode is None or self.wrapper_mode == wrapped, 'Mixed raw/wrapped evidence')
        self.wrapper_mode = wrapped
        tr = row['transition'] if wrapped else row
        require(tr.get('episode_serial') == self.serial and tr.get('episode_tick') == tr.get('tick')
                and tr.get('physical_reset_count') == self.reset_count and tr.get('global_physics_tick') == self.initial_global_tick+tr.get('tick', -1), 'Episode/reset/global clock changed')
        require(same(tr.get('hazard_schedule'), self.schedule) and tr.get('hazard_stage') == self.stage, 'Original hazard schedule changed')
        if wrapped:
            require(same(row.get('input_observation'), float32(self.observation), tolerance=1e-6), 'Policy input was not the preceding physical observation')
        applied = _applied(tr.get('control'))
        samples = tr.get('physics_samples')
        require(isinstance(samples, list) and len(samples) == 2 and tr['tick'] == self.tick+2, 'Both consecutive120Hz samples required')
        contacts = []
        for sample in samples:
            contacts.extend(self._sample(sample, applied.is_fallback))
        require(same(tr.get('physics_state'), samples[-1]['physics_state']) and tr.get('contacts') == contacts, 'Control boundary disagrees with native samples')
        require(tr.get('hazard_frames') == samples[-1]['hazard_frames'] and tr.get('background_frames') == samples[-1]['background_frames'], 'Object control boundary disagrees with native sample')
        expected_failure = self.first_failure
        require(same(tr.get('first_guard_failure'), expected_failure), 'Latched failure evidence differs')
        expected = self.task.advance(tr['physics_state'], applied, tick=self.tick, contact_count=len(contacts),
            command_fallback=applied.is_fallback, failure_reason=None if expected_failure is None else expected_failure['reason'])
        expected_dict = expected.as_dict()
        for key, value in expected_dict.items():
            require(same(tr.get(key), value), 'Task replay differs: '+key)
        deployment = self.task.last_deployment_observation
        require(same(tr.get('deployment_observation'), deployment), 'Nearest-two selection/masking differs')
        self.masked_steps += bool(deployment['masked_hazard_ids'])
        if self.continuous:
            context = tr.get('continuation', {})
            for key, value in self.task.last_continuation.items():
                require(same(context.get(key), value), 'Continuous window replay differs: '+key)
            roll = self.task.last_continuation['window_roll']
            if roll is not None:
                native = context.get('native_boundary', {})
                require(native.get('equal') is True and native.get('physical_reset_count') == 1
                        and native.get('tick') == self.tick and native.get('hazard_schedule_index') == 0
                        and same(native.get('before'), roll['physics_state_before'], tolerance=0.)
                        and same(native.get('after'), native.get('before'), tolerance=0.), 'Native route roll changed pose/velocity or reset state')
                self.rolls.append(deepcopy(native))
        else:
            require('continuation' not in tr, 'Unexpected rolling-window evidence')
        if wrapped:
            self._wrapper(row, expected)
        self.last = expected_dict
        self.observation = expected.observation
        self.reward += expected.reward
        self.steps += 1
        return self

    def _wrapper(self, row, expected):
        require(type(row.get('returned_terminated')) is bool and type(row.get('returned_truncated')) is bool
                and row['returned_terminated'] == expected.terminated and row['returned_truncated'] == expected.truncated, 'Returned terminal flags differ')
        require(same(row.get('returned_reward'), float32(expected.reward), tolerance=1e-6), 'Returned reward differs')
        require(same(row.get('final_observation'), float32(expected.observation), tolerance=1e-6), 'Detached pre-reset observation differs')
        done = expected.terminated or expected.truncated
        if not done or self.continuous:
            require(same(row.get('returned_observation'), float32(expected.observation), tolerance=1e-6), 'Returned physical observation differs')
        if done and self.continuous:
            suppressed = row.get('autoreset_suppressed', {})
            require('auto_reset' not in row and suppressed.get('episode_tick') == self.tick
                    and suppressed.get('physical_reset_count') == 1 and suppressed.get('terminal_state_retained') is True, 'Final continuous reset was not explicitly suppressed')
        elif done:
            reset = row.get('auto_reset', {})
            require(reset.get('reset_count') == self.reset_count+1 and reset.get('episode_serial') == self.serial+1, 'Ordinary final auto-reset evidence missing')
            require(same(row.get('returned_observation'), float32(reset.get('observation', [])), tolerance=1e-6), 'Returned auto-reset observation differs')

    def finish(self, *, complete=True):
        require(type(complete) is bool and self.last is not None, 'Recorded exposure required')
        done = self.last['terminated'] or self.last['truncated']
        require(done == complete, 'Requested completion differs from terminal evidence')
        progress = (self.task.last_continuation['total_progress_m'] if self.continuous else self.last['route_state']['progress_m'])
        return dict(evidence_valid=True, complete=done, geometry_qualified=True, task_success=self.last['success'],
            driving_success=self.last['success'] and self.first_failure is None,
            physical_failure=self.last['terminated'] and not self.last['success'],
            terminal_reason=self.last['reason'], control_steps=self.steps, physics_samples=self.tick,
            total_progress_m=progress, progress_m=progress, simulated_seconds=self.tick/120,
            completed_windows=self.task.completed_windows if self.continuous else int(self.last['success']),
            continuous=self.continuous, physical_reset_count_during_episode=1, native_window_boundaries=deepcopy(self.rolls),
            contacts=self.contacts, rigid_contact_events=self.contacts, contact_points=self.contact_points, minimum_clearance_m=self.minimum,
            maximum_footprint_lateral_bound_m=self.max_footprint, maximum_native_xy_displacement_m=self.max_displacement,
            maximum_native_velocity_delta_m_s=self.max_velocity_delta, total_reward=self.reward,
            post_settle_support_fraction=self.supported/self.support_samples if self.support_samples else None,
            longest_unsupported_seconds=self.longest_unsupported/120, hazard_masked_control_steps=self.masked_steps,
            first_physical_failure=deepcopy(self.first_failure), replay_absolute_tolerance=REPLAY_TOLERANCE,
            limitations=[
                'Pure task semantics are replayed from saved native measurements; this is not an independent physics simulation.',
                'Every object participates in collision geometry; the actor still sees only the nearest two objects.',
                'Exact pose/velocity equality is required at route rolls. Ordinary native velocity deltas are reported without inventing an acceleration limit.',
                'Finite recorded data cannot prove sensor autonomy, collision prediction, learned background behavior or generalization.',
            ])


FullEpisodeAccumulator = FullRoadAccumulator


def reduce_episode(rows, reset_record, **kwargs):
    audit = FullRoadAccumulator(reset_record, **kwargs)
    for row in rows:
        audit.update(row)
    return audit.finish()


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _contained(base, name):
    require(isinstance(name, str) and name and '\\' not in name, 'Invalid manifest path')
    relative = Path(name)
    require(not relative.is_absolute() and '..' not in relative.parts, 'Manifest path escapes package')
    path = base/relative
    require(path.is_file() and path.resolve().is_relative_to(base.resolve())
            and all(not p.is_symlink() for p in (path, *path.parents) if p.is_relative_to(base)), 'Missing or linked manifest artifact')
    return path


def verify_package(package, *, root=ROOT):
    """Validate immutable provenance then stream a completed native package."""
    package, root = Path(package).resolve(), Path(root).resolve()
    def read(name):
        return json.loads(_contained(package, name).read_text(encoding='utf-8'))
    manifest, summary, result, job = (read(n) for n in ('manifest.json', 'summary.json', 'result.json', 'resolved-config.json'))
    require(manifest.get('status') == 'completed' and manifest.get('execution_passed') is True
            and summary.get('execution_passed') is True and result.get('completed') is True, 'Native package did not complete cleanly')
    require(same(manifest.get('config'), job) and same(summary.get('native'), result), 'Package native/config copies disagree')
    inventory = manifest['output_hashes']
    actual = {p.relative_to(package).as_posix() for p in package.rglob('*') if p.is_file() and p.name != 'manifest.json'}
    require(set(inventory) == actual, 'Output inventory is incomplete or has unexpected files')
    for name, expected in inventory.items():
        require(digest(_contained(package, name)) == expected, 'Output hash changed: '+name)
    sources = manifest['source_hashes']
    for name, expected in sources.items():
        require(digest(_contained(package/'source', name)) == expected, 'Captured source hash changed: '+name)
    for name in REPLAY_SOURCES:
        if name.startswith('environments/continuous_') and not job['continuous_windows']:
            continue
        require(name in sources and digest(_contained(root, name)) == sources[name], 'Current replay source differs from native source: '+name)
    if job['controller'] == 'rsl':
        require(digest(_contained(root, job['policy_path'])) == job['policy_sha256']
                and result.get('policy_state_unchanged') is True
                and result.get('policy_state_before') == result.get('policy_state_after'), 'Policy input/state changed')
    initial, final = result['scene_initial'], result['scene_final']
    require(initial['static_hashes'] == final['static_hashes'] and final.get('static_hashes_unchanged') is True, 'Static scene contract changed')
    for name, expected in initial['static_hashes'].items():
        require(digest(_contained(package/'scene', name)) == expected, 'Static scene file changed: '+name)
    require(initial['inventory'] == final['inventory'] and len(final['inventory']['bodies']) == 82
            and final.get('background_active_count') == 12 and final.get('physical_learned_car_count') == 1, 'Physical scene inventory changed')
    require(summary.get('returncode') == 0 and summary.get('sources_unchanged') is True
            and summary.get('policy_unchanged') is True and not summary.get('unexpected_runtime_errors')
            and summary.get('usd_reference_warning_count') == 0
            and summary.get('kit_after', {}).get('available') is True and summary['kit_after']['count'] == 0
            and result.get('contact_cleanup', {}).get('passed') is True
            and result.get('viewport_cleanup', {}).get('passed') is True
            and result.get('optimizer_steps') == 0 and result.get('training') is False, 'Process/cleanup/inference contract failed')
    route_path = _contained(package/'source', job['route_file'])
    route = load_routes(route_path)['loop_r500']
    reset = read('reset.json')
    if 'hazard_background_preflight' in reset:
        for name in ('experiments/full_scene_hazard_preflight.py', 'traffic/showcase_clearance.py'):
            require(sources.get(name) == digest(_contained(root, name)), 'Current preflight replay source differs: '+name)
    with _contained(package, 'trajectory.jsonl').open(encoding='utf-8') as stream:
        assessment = reduce_episode((json.loads(line) for line in stream), reset, route=route,
            stage=job['stage'], continuous=job['continuous_windows'], seed=job['seed'])
    require(assessment['control_steps'] == result['control_steps'] and assessment['contacts'] == result['contacts']
            and assessment['driving_success'] == result['driving_passed']
            and assessment['driving_success'] == summary['driving_passed'] == manifest['driving_passed']
            and same(assessment['minimum_clearance_m'], result['minimum_clearance_m']), 'Independent assessment differs from native result')
    if job['continuous_windows']:
        require(result.get('physical_reset_count') == 1 and same(result.get('native_window_boundaries'), assessment['native_window_boundaries'])
                and same(result.get('original_hazard_schedule'), reset['hazard_schedule']), 'Final continuous reset/window evidence differs')
    return dict(schema_version=1, package=str(package), evidence_valid=True, assessment=assessment,
        verified_outputs=len(inventory), verified_captured_sources=len(sources),
        manifest_sha256=digest(package/'manifest.json'), trajectory_sha256=digest(package/'trajectory.jsonl'),
        evaluator_sha256=digest(Path(__file__)))


def _demonstration_pair(observation, action, expected_observation, raw_action, transition):
    """Exact float32 supervision pair; raw requested controls remain distinct."""
    import numpy as np
    require(np.array_equal(observation, np.asarray(expected_observation, dtype=np.float32)), 'NPZ pre-action observation differs from native reset/previous boundary')
    require(np.array_equal(action, np.asarray(raw_action, dtype=np.float32)), 'NPZ action differs from raw scripted action')
    require(action.shape == (2,) and np.isfinite(action).all() and (np.abs(action) <= 1).all(), 'Demonstration action is nonfinite or outside signed bounds')
    physical = [float(action[0])*.5, max(0., float(action[1])), max(0., -float(action[1]))]
    require(same(transition.get('requested_action'), physical), 'Raw scripted action differs from requested native controls')


def audit_collection(package, *, root=ROOT, expected_episodes=40):
    """Qualify every saved full-road demonstration pair without loading JSONL whole.

    The bounded numeric NPZ arrays are loaded with pickle disabled. Raw physics
    is streamed episode by episode; all declared cases must pass, without drops.
    """
    import numpy as np
    package, root = Path(package).resolve(), Path(root).resolve()
    require(type(expected_episodes) is int and 1 <= expected_episodes <= 200, 'Bounded collection count required')
    def read(name):
        return json.loads(_contained(package, name).read_text(encoding='utf-8'))
    manifest, end, result = (read(n) for n in ('manifest.json', 'exit.json', 'result.json'))
    args = manifest['args']
    require(args.get('mode') == 'collect' and args.get('environment') == 'full'
            and args.get('controller') == 'scripted' and args.get('stage') == 'mixed', 'Expected scripted full-road mixed collection')
    seeds = args['seeds']
    require(len(seeds) == expected_episodes and len(set(seeds)) == len(seeds)
            and all(type(s) is int and 0 <= s < 2**32 for s in seeds), 'Declared collection seed inventory differs')
    inventory = end['output_sha256']
    actual = {p.relative_to(package).as_posix() for p in package.rglob('*') if p.is_file() and p.name != 'exit.json'}
    require(set(inventory) == actual, 'Collection output inventory differs')
    for name, expected in inventory.items():
        require(digest(_contained(package, name)) == expected, 'Collector output hash changed: '+name)
    for name, expected in manifest['source_sha256'].items():
        require(digest(_contained(package/'sources', name)) == expected, 'Collector captured source changed: '+name)
    for name in REPLAY_SOURCES:
        if name.startswith('environments/continuous_'):
            continue
        require(manifest['source_sha256'].get(name) == digest(_contained(root, name)), 'Current collector replay source differs: '+name)
    require(result.get('completed') is True and result.get('demonstrations_qualified') is True
            and result.get('environment') == 'full' and result.get('optimizer_steps') == 0
            and result.get('emergency_shield') is False and end.get('passed') is True
            and end.get('returncode') == 0 and end.get('sources_unchanged') is True
            and end.get('kit_after', {}).get('count') == 0 and end['kit_after'].get('available') is True
            and not end.get('timed_out') and not end.get('stop_reason')
            and result.get('contact_cleanup', {}).get('passed') is True
            and result.get('viewport_cleanup', {}).get('passed') is True
            and not end['runtime_log_assessment']['unexpected_runtime_errors']
            and end['runtime_log_assessment']['usd_reference_warning_count'] == 0, 'Collector native execution or cleanup failed')
    initial, final = read('scene-contract.json'), result['scene_final']
    require(initial['static_hashes'] == final['static_hashes'] and initial['inventory'] == final['inventory']
            and len(final['inventory']['bodies']) == 82 and final['background_active_count'] == 12, 'Collector scene contract changed')
    for name, expected in initial['static_hashes'].items():
        require(digest(_contained(package/'scene', name)) == expected, 'Collector static asset changed: '+name)
    cases = result['episodes']
    require([x['seed'] for x in cases] == seeds, 'Raw result case order differs')
    if any('hazard_background_preflight' in case['reset'] for case in cases):
        for name in ('experiments/full_scene_hazard_preflight.py', 'traffic/showcase_clearance.py'):
            require(manifest['source_sha256'].get(name) == digest(_contained(root, name)), 'Current preflight replay source differs: '+name)
    route = load_routes(_contained(package/'sources', 'scenarios/physics-road/loop-hazard-routes-v1.json'))['loop_r500']
    npz_path = _contained(package, 'demonstrations.npz')
    with np.load(npz_path, allow_pickle=False) as arrays:
        require(set(arrays.files) == {'observations', 'actions'}, 'Unexpected demonstration arrays')
        observations, actions = arrays['observations'], arrays['actions']
    count = result['demonstration_steps']
    require(type(count) is int and 0 < count <= expected_episodes*2100
            and observations.shape == (count, 42) and actions.shape == (count, 2)
            and observations.dtype == actions.dtype == np.dtype('float32')
            and np.isfinite(observations).all() and np.isfinite(actions).all(), 'Invalid bounded demonstration tensor inventory')
    reductions, row_index = [], 0
    with _contained(package, 'steps.jsonl').open(encoding='utf-8') as stream:
        for index, case in enumerate(cases):
            stage, side = [('clear', None), ('static', None), ('crossing', 'left'), ('crossing', 'right')][index % 4]
            require((case['stage'], case['side']) == (stage, side), 'Declared mixed case changed')
            reset = case['reset']
            accumulator = FullEpisodeAccumulator(reset, route=route, stage=stage, seed=case['seed'])
            observation, return_float32 = reset['observation'], 0.
            for step in range(2100):
                line = stream.readline()
                require(bool(line) and row_index < count, 'Collection raw stream ended early')
                row = json.loads(line)
                require(row['seed'] == case['seed'] and row['stage'] == stage, 'Collection raw episode order changed')
                tr = row['transition']
                _demonstration_pair(observations[row_index], actions[row_index], observation, row['raw_policy_action'], tr)
                accumulator.update(tr)
                row_index += 1
                observation = tr['observation']
                return_float32 += float32(tr['reward'])
                if tr['terminated'] or tr['truncated']:
                    break
            else:
                raise ValueError('Collection episode exceeded native35second bound')
            assessment = accumulator.finish()
            require(assessment['driving_success'] and assessment['contacts'] == 0 and case['success'] is True
                    and case['reason'] == 'success' and case['steps'] == assessment['control_steps']
                    and same(case['return'], return_float32) and same(case['final_transition'], tr)
                    and same(case['independent_assessment'], assessment), 'Collector case does not qualify independently')
            reductions.append(dict(seed=case['seed'], stage=stage, side=side, **assessment))
        require(not stream.readline() and row_index == count, 'Unaccounted demonstration/raw rows')
    names = ('manifest.json', 'exit.json', 'result.json', 'scene-contract.json', 'steps.jsonl', 'demonstrations.npz')
    artifacts = {name: dict(sha256=digest(package/name), bytes=(package/name).stat().st_size) for name in names}
    require(all(artifacts[name]['sha256'] == inventory[name] for name in names if name != 'exit.json'), 'Audited package changed during reduction')
    return dict(schema_version=1, package=package.relative_to(root).as_posix(), passed=True,
        training_warmstart_data_qualified=True, environment='full', episodes=expected_episodes,
        successes=sum(x['driving_success'] for x in reductions), demonstration_steps=count,
        demonstrations=dict(path=npz_path.relative_to(root).as_posix(), **artifacts['demonstrations.npz']),
        artifacts=artifacts, reductions=reductions, verified_outputs=len(inventory),
        verified_captured_sources=len(manifest['source_sha256']), evaluator_sha256=digest(Path(__file__)),
        limitations=['Scripted imitation data qualification, not evidence of learned policy performance.',
            'Privileged current simulator state and known map; backgrounds remain nonreactive kinematic actors.',
            'All declared40cases and every float32 supervision pair are retained; no failed case was dropped.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('package', type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(verify_package(args.package), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
