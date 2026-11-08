"""Streaming, CPU-only evidence summaries for the versioned native hazard task.

All completed 60 Hz boundaries must be supplied, including both 120 Hz native
samples, even when a training logger saves fewer rows. This checks raw geometry,
observations and explicit success conditions; it is not an independent replay
of the task's entire reward function or a collision-free driving certificate.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from numbers import Real
import os
from pathlib import Path
import uuid

from experiments.lane_following_lab_validation import _state_schema
from traffic.bypass_validation import OrientedBox, box_clearance
from usd.physical_scene import PATHS

STAGES = ('clear', 'static', 'crossing')
FAILURES = frozenset(('rigid_contact', 'command_fallback', 'wheel_support_loss',
    'excessive_tilt', 'unsafe_height', 'lane_departure', 'heading_outside_envelope',
    'overspeed', 'reverse_motion', 'before_route_start', 'endpoint_overshoot', 'state_discontinuity'))


def atomic_json(path, value):
    data = json.dumps(value, indent=2, allow_nan=False)+'\n'
    path = Path(path)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with temporary.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value, count):
    return isinstance(value, (list, tuple)) and len(value) == count and all(_number(x) for x in value)


def _close(a, b):
    return _number(a) and _number(b) and abs(a-b) <= 1e-5+1e-6*max(abs(a), abs(b))


def _same(a, b):
    return len(a) == len(b) and all(_close(x, y) for x, y in zip(a, b))


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _frames(value, tick, stage):
    if not isinstance(value, (list, tuple)) or len(value) != (0 if stage == 'clear' else 1):
        raise ValueError('Each native sample needs its zero/one measured hazard frame')
    names = ('x_m', 'y_m', 'yaw_rad', 'vx_m_s', 'vy_m_s', 'length_m', 'width_m', 'timestamp_s')
    for frame in value:
        if (not isinstance(frame, dict) or frame.get('object_id') != 'hazard_000'
                or not all(_number(frame.get(name)) for name in names)
                or frame['length_m'] != 1. or frame['width_m'] != 1.
                or abs(frame['timestamp_s']-tick/120) > 1e-9):
            raise ValueError('Malformed, stale or wrong-identity native hazard frame')
    return value


def _object_slots(state, frames):
    c, s = math.cos(state['yaw_rad']), math.sin(state['yaw_rad'])
    ego_vx, ego_vy = state['velocity_m_s'][:2]
    values = []
    ordered = sorted(frames, key=lambda o: ((o['x_m']-state['position_m'][0])**2+
                                          (o['y_m']-state['position_m'][1])**2, o['object_id']))
    for frame in ordered[:2]:
        dx, dy = frame['x_m']-state['position_m'][0], frame['y_m']-state['position_m'][1]
        vx, vy = frame['vx_m_s']-ego_vx, frame['vy_m_s']-ego_vy
        yaw = frame['yaw_rad']-state['yaw_rad']
        values.extend((1., c*dx+s*dy, -s*dx+c*dy, c*vx+s*vy, -s*vx+c*vy,
                       frame['length_m']/2, frame['width_m']/2, math.sin(yaw), math.cos(yaw), 0.))
    return values+[0.]*(10*(2-len(ordered)))


class EpisodeAccumulator:
    """Bounded-memory reduction; update exactly once for every completed step.

    Missing/malformed evidence raises ValueError. A well-recorded physical
    failure or timeout remains valid evidence and has driving_success=False.
    route is required for geometry-qualified success; omitting it is useful
    only for explicitly limited training counters.
    """
    def __init__(self, reset_record=None, *, route=None, stage=None):
        if stage is not None and stage not in STAGES:
            raise ValueError('Unknown hazard stage')
        self.reset = deepcopy(reset_record) if reset_record is not None else None
        self.route, self.stage = route, stage
        self.steps = self.tick = self.samples = 0
        self.serial = None if self.reset is None else self.reset.get('episode_serial')
        self.last = None
        self.reward = 0.
        self.parts = {}
        self.initial_progress = 0.
        self.first_physical_failure = None
        self.contacts = self.contact_points = 0
        self.minimum_clearance = None
        self.overlap_samples = 0
        self.maximum_lateral = self.lateral_sq = 0.
        self.lateral_count = 0
        self.minimum_upright = 1.
        self.maximum_footprint = 0.
        self.supported = self.support_samples = self.unsupported_ticks = self.longest_unsupported = 0
        self.encountered = False
        self.first_clear_s = None
        self.wait_ticks = self.max_wait_ticks = self.wait_total_ticks = 0
        self.previous_wait = False
        self.static_goal = None
        self.static_hold_ticks = 0
        self.previous_static_hold = False
        self.last_progress = 0.
        self.last_speed = 0.
        self.last_corridor_clear = True
        self.observation_checks = True
        self.reward_sum_checks = True
        self.terminal_snapshot_checks = True
        self.schedule_sha256 = None
        if self.reset is not None:
            state = self.reset.get('physics_state')
            if not _state_schema(state):
                raise ValueError('Reset must include a valid measured native state')
            if route is not None:
                self.initial_progress = route.project(*state['position_m'][:2]).s_m
            schedule = self.reset.get('hazard_schedule')
            if schedule is not None:
                if self.stage is None:
                    self.stage = schedule.get('stage')
                self.schedule_sha256 = self._schedule_hash(schedule)
            if self.stage is not None and 'hazard_frames' in self.reset:
                frames = _frames(self.reset['hazard_frames'], 0, self.stage)
                self._set_static_goal(frames)

    @staticmethod
    def _schedule_hash(schedule):
        return hashlib.sha256(json.dumps(schedule, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def _set_static_goal(self, frames):
        if self.stage != 'static' or self.route is None or self.static_goal is not None:
            return
        goals = []
        for obj in frames:
            projection = self.route.project(obj['x_m'], obj['y_m'])
            heading = obj['yaw_rad']-projection.point.yaw_rad
            lateral_extent = abs(math.sin(heading))*obj['length_m']/2+abs(math.cos(heading))*obj['width_m']/2
            if abs(projection.lateral_error_m) <= .9+.25+lateral_extent:
                extent = abs(math.cos(heading))*obj['length_m']/2+abs(math.sin(heading))*obj['width_m']/2
                goals.append(projection.s_m-extent-2.4-2.)
        if not goals or not 5. <= min(goals) <= 59.:
            raise ValueError('Static episode needs a measured forward stop window')
        self.static_goal = min(goals)

    def _sample(self, sample):
        tick = sample['tick']
        state = sample.get('physics_state')
        if not _state_schema(state):
            raise ValueError('Missing/malformed native physical state')
        frames = _frames(sample.get('hazard_frames'), tick, self.stage)
        self._set_static_goal(frames)
        events = sample.get('contacts')
        if not isinstance(events, list) or type(sample.get('contact_count')) is not int or sample['contact_count'] != len(events):
            raise ValueError('Native contact count must match retained events')
        for event in events:
            if (not isinstance(event, dict) or PATHS.chassis not in (event.get('actor0'), event.get('actor1'))
                    or type(event.get('contact_count')) is not int or event['contact_count'] < 0):
                raise ValueError('Contact must name canonical ego and measured point count')
            self.contact_points += event['contact_count']
        self.contacts += len(events)
        self.samples += 1
        self.minimum_upright = min(self.minimum_upright, state['upright_z'])
        self.last_speed = state['speed_m_s']
        supported = all(state['wheel_on_ground'])
        if tick > 240:
            self.support_samples += 1
            self.supported += supported
            self.unsupported_ticks = 0 if supported else self.unsupported_ticks+1
            self.longest_unsupported = max(self.longest_unsupported, self.unsupported_ticks)
        native_reason = sample.get('guard_reason')
        if native_reason is not None and native_reason not in FAILURES:
            raise ValueError('Unknown native guard reason')
        if native_reason is not None and self.first_physical_failure is None:
            self.first_physical_failure = dict(tick=tick, reason=native_reason, source='recorded_native_guard')
        if self.route is None:
            return
        projection = self.route.project(*state['position_m'][:2])
        heading = _angle(state['yaw_rad']-projection.point.yaw_rad)
        footprint = abs(projection.lateral_error_m)+2.4*abs(math.sin(heading))+.9*abs(math.cos(heading))
        self.maximum_footprint = max(self.maximum_footprint, footprint)
        self.last_progress, self.last_speed = projection.s_m, state['speed_m_s']
        if tick > 240:
            self.lateral_count += 1
            self.lateral_sq += projection.lateral_error_m**2
            self.maximum_lateral = max(self.maximum_lateral, abs(projection.lateral_error_m))
        c, s = math.cos(projection.point.yaw_rad), math.sin(projection.point.yaw_rad)
        forward = c*state['velocity_m_s'][0]+s*state['velocity_m_s'][1]
        checks = [(bool(events), 'rigid_contact'), (self.unsupported_ticks > 12, 'wheel_support_loss'),
            (state['upright_z'] < math.cos(math.radians(5)), 'excessive_tilt'),
            (not 0 <= state['position_m'][2] <= 2, 'unsafe_height'),
            (footprint > self.route.width_m/2, 'lane_departure'), (abs(heading) > 1., 'heading_outside_envelope'),
            (state['speed_m_s'] > 4., 'overspeed'), (forward < -.2, 'reverse_motion'),
            (projection.s_m < -1., 'before_route_start'), (projection.s_m > 60.5, 'endpoint_overshoot')]
        failure = next((reason for failed, reason in checks if failed), None)
        if failure and self.first_physical_failure is None:
            self.first_physical_failure = dict(tick=tick, reason=failure, source='independent_native_geometry')
        corridor_clear = True
        ego = OrientedBox(*state['position_m'][:2], state['yaw_rad'], 4.8, 1.8)
        for frame in frames:
            target = OrientedBox(frame['x_m'], frame['y_m'], frame['yaw_rad'], frame['length_m'], frame['width_m'])
            geometry = box_clearance(ego, target)
            clearance = geometry['clearance_m']
            self.overlap_samples += bool(geometry['overlap'])
            self.minimum_clearance = clearance if self.minimum_clearance is None else min(self.minimum_clearance, clearance)
            obj_projection = self.route.project(frame['x_m'], frame['y_m'])
            angle = frame['yaw_rad']-obj_projection.point.yaw_rad
            extent = abs(math.sin(angle))*frame['length_m']/2+abs(math.cos(angle))*frame['width_m']/2
            corridor_clear &= abs(obj_projection.lateral_error_m) > .9+.25+extent
        if self.stage == 'crossing' and not corridor_clear:
            self.encountered = True
        if self.stage == 'crossing' and self.encountered and corridor_clear and self.first_clear_s is None:
            self.first_clear_s = tick/120
        waiting = (self.stage == 'crossing' and self.encountered and corridor_clear
                   and state['speed_m_s'] <= .05 and tick > 240)
        if waiting and self.previous_wait:
            self.wait_ticks += 1
            self.wait_total_ticks += 1
        else:
            self.wait_ticks = 0
        self.max_wait_ticks = max(self.max_wait_ticks, self.wait_ticks)
        self.previous_wait = waiting
        holding = (self.stage == 'static' and self.static_goal is not None
                   and abs(self.static_goal-projection.s_m) <= .5 and state['speed_m_s'] <= .05
                   and tick > 240 and failure is None and native_reason is None)
        self.static_hold_ticks = self.static_hold_ticks+1 if holding and self.previous_static_hold else 0
        self.previous_static_hold = holding
        self.last_corridor_clear = bool(corridor_clear)

    def update(self, record):
        if self.last is not None and (self.last['terminated'] or self.last['truncated']):
            raise ValueError('A new accumulator is required after terminal/autoreset')
        if not isinstance(record, dict):
            raise ValueError('Expected a detached lane transition dictionary')
        tick = record.get('tick')
        if type(tick) is not int or tick != self.tick+2 or not _vector(record.get('observation'), 42):
            raise ValueError('Expected consecutive 60 Hz boundaries with42 finite observations')
        stage = record.get('hazard_stage', self.stage)
        if stage not in STAGES or self.stage is not None and stage != self.stage:
            raise ValueError('Hazard stage cannot change inside an episode')
        self.stage = stage
        serial = record.get('episode_serial')
        if type(serial) is not int or serial < 1 or self.serial is not None and serial != self.serial:
            raise ValueError('Episode identity changed or is missing')
        self.serial = serial
        if any(type(record.get(key)) is not bool for key in ('terminated', 'truncated', 'success')):
            raise ValueError('Explicit boolean terminal/success flags required')
        if record['terminated'] and record['truncated']:
            raise ValueError('Termination and pure timeout must be distinct')
        reason = record.get('reason')
        if not isinstance(reason, str) or reason not in FAILURES | {'success', 'time_limit', 'driving', 'settling'}:
            raise ValueError('Unknown transition reason')
        done = record['terminated'] or record['truncated']
        if ((reason == 'success') != record['success'] or record['success'] and not record['terminated']
                or (reason == 'time_limit') != record['truncated']
                or (reason in FAILURES) != (record['terminated'] and not record['success'])):
            raise ValueError('Terminal reason and flags disagree')
        if record['truncated'] and tick != 4200 or not done and tick >= 4200:
            raise ValueError('The35second episode deadline must be exact')
        samples = record.get('physics_samples')
        if (not isinstance(samples, list) or len(samples) != 2
                or any(type(sample.get('tick')) is not int for sample in samples)
                or [sample.get('tick') for sample in samples] != [tick-1, tick]):
            raise ValueError('Each step must retain both completed native samples')
        schedule = record.get('hazard_schedule')
        if schedule is not None:
            sha = self._schedule_hash(schedule)
            if self.schedule_sha256 is not None and sha != self.schedule_sha256:
                raise ValueError('Schedule changed within an episode')
            self.schedule_sha256 = sha
        for sample in samples:
            self._sample(sample)
        state = record.get('physics_state')
        if not _state_schema(state) or state != samples[-1]['physics_state']:
            raise ValueError('Control state must match the last native sample')
        frames = _frames(record.get('hazard_frames'), tick, self.stage)
        if frames != samples[-1]['hazard_frames']:
            raise ValueError('Control hazard readback must match the last native sample')
        self.observation_checks &= _same(record['observation'][22:], _object_slots(state, frames))
        if self.route is not None:
            self.observation_checks &= _close(record['observation'][0], self.last_progress)
        parts = record.get('reward_components')
        if not isinstance(parts, dict) or not parts or not all(_number(v) for v in parts.values()) or not _number(record.get('reward')):
            raise ValueError('Finite reward and named components required')
        self.reward_sum_checks &= _close(math.fsum(parts.values()), record['reward'])
        self.reward += record['reward']
        for name, value in parts.items():
            self.parts[name] = self.parts.get(name, 0.)+value
        if done:
            terminal = record.get('terminal_observation')
            self.terminal_snapshot_checks &= _vector(terminal, 42) and _same(terminal, record['observation'])
            if 'final_observation' in record:
                self.terminal_snapshot_checks &= _vector(record['final_observation'], 42) and _same(record['final_observation'], record['observation'])
        self.steps += 1
        self.tick = tick
        # Retain only the boundary required for final reporting, not entire scenes/traces.
        self.last = dict(tick=tick, terminated=record['terminated'], truncated=record['truncated'],
                         success=record['success'], reason=reason, observation=list(record['observation']))
        return self

    def finish(self, *, complete=True):
        if type(complete) is not bool:
            raise ValueError('complete must be boolean')
        done = self.last is not None and (self.last['terminated'] or self.last['truncated'])
        geometry = self.route is not None
        intended = False
        if geometry and self.last is not None:
            if self.stage == 'static':
                intended = self.static_hold_ticks >= 120
            else:
                intended = (self.last_progress >= 60. and self.last_progress-self.initial_progress >= 59.95
                            and (self.stage == 'clear' or self.encountered and self.last_corridor_clear))
            intended &= self.first_physical_failure is None
        reason = None if self.last is None else self.last['reason']
        checks = dict(records_present=self.steps > 0, requested_completion=done if complete else not done,
            object_observations_match=self.observation_checks, reward_component_sum=self.reward_sum_checks,
            pre_reset_terminal_snapshot=self.terminal_snapshot_checks,
            observed_success_semantics=not geometry or self.last is not None and self.last['success'] == intended)
        if self.first_physical_failure is not None and self.last is not None:
            checks['prompt_physical_failure'] = (self.last['terminated'] and not self.last['success']
                and self.last['reason'] == self.first_physical_failure['reason']
                and self.tick in (self.first_physical_failure['tick'], self.first_physical_failure['tick']+1))
        valid = all(checks.values())
        return dict(evidence_valid=valid, complete=done, geometry_qualified=geometry,
            task_success=bool(valid and done and self.last['success']),
            driving_success=bool(valid and done and geometry and intended and not self.overlap_samples),
            physical_failure=bool(done and reason in FAILURES), terminal_reason=reason,
            checks=checks, stage=self.stage, episode_serial=self.serial,
            control_steps=self.steps, physics_samples=self.samples, simulated_seconds=self.tick/120,
            progress_m=None if not geometry else self.last_progress-self.initial_progress,
            final_station_m=None if not geometry else self.last_progress,
            final_speed_m_s=None if self.last is None else self.last_speed,
            total_reward=self.reward, reward_components=dict(self.parts),
            rigid_contact_events=self.contacts, contact_points=self.contact_points,
            minimum_clearance_m=self.minimum_clearance,
            geometric_overlap_samples=self.overlap_samples,
            maximum_footprint_lateral_bound_m=None if not geometry else self.maximum_footprint,
            driving_lateral_rms_m=math.sqrt(self.lateral_sq/self.lateral_count) if self.lateral_count else None,
            driving_lateral_max_m=self.maximum_lateral if self.lateral_count else None,
            minimum_upright_z=self.minimum_upright,
            post_settle_support_fraction=self.supported/self.support_samples if self.support_samples else None,
            longest_unsupported_gap_s=self.longest_unsupported/120,
            first_physical_failure=deepcopy(self.first_physical_failure),
            static_stop_station_m=self.static_goal, static_stopped_hold_s=self.static_hold_ticks/120,
            crossing_encountered=self.encountered, first_corridor_clear_after_encounter_s=self.first_clear_s,
            post_clear_stopped_seconds=self.wait_total_ticks/120,
            longest_post_clear_stop_seconds=self.max_wait_ticks/120,
            schedule_sha256=self.schedule_sha256,
            validation_scope='All supplied60Hz boundaries;120Hz native geometry/contact/support; measured object slots; component arithmetic and explicit success conditions. No complete reward or controller replay.')


def summarize_episode(records, reset_record=None, *, route=None, stage=None, complete=True):
    accumulator = EpisodeAccumulator(reset_record, route=route, stage=stage)
    for record in records:
        accumulator.update(record)
    return accumulator.finish(complete=complete)
