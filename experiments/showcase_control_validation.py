"""Predeclared v09 native-widget validation, separate from the legacy smoke.

This module has no Isaac imports. Runtime callers supply actual native clock and
chassis observations; fake models in CPU tests prove only the harness contract.
"""
from copy import deepcopy
import math

from experiments.showcase_controls import MPH, defaults, validate_settings


VALIDATION_NAMES = ('endpoints',)
MINIMUM_DRIVE_S = 180
DRIVING_KEYS = ('traffic_count', 'ego_mph', 'traffic_mph', 'following_s')
INITIAL_DRIVING = dict(traffic_count=12, ego_mph=10., traffic_mph=5., following_s=2.4)
# Absolute simulation seconds, including the unchanged two-second settle.
PHASES = (
    dict(request_s=5, deadline_s=14, traffic_count=4, ego_mph=5., traffic_mph=5., following_s=3.),
    dict(request_s=15, deadline_s=50, traffic_count=80, ego_mph=35., traffic_mph=25., following_s=1.8),
    dict(request_s=55, deadline_s=100, traffic_count=12, ego_mph=35., traffic_mph=5., following_s=3.),
    dict(request_s=105, deadline_s=135, traffic_count=80, ego_mph=35., traffic_mph=15., following_s=1.8),
)
PASSING_START_S = 135
PASSING_END_S = 180


def validation_initial_controls(name, settings=None):
    """Return declared initial settings; reject conflicting supplied driving values.

    Omitting settings selects the scenario defaults. Explicit settings may tune
    viewing/rendering but must already have the declared driving initial state.
    """
    if name not in VALIDATION_NAMES:
        raise ValueError('Unknown control-validation scenario')
    if settings is None:
        return validate_settings({**defaults(), **INITIAL_DRIVING})
    values = validate_settings(settings)
    if any(values[key] != value for key, value in INITIAL_DRIVING.items()):
        raise ValueError('Control validation requires its declared initial driving settings')
    return values


def validation_snapshot(tick, state, live, clock):
    """Exclude display frame counts while retaining the actual native clock."""
    return dict(tick=tick, position_m=list(state['position_m']),
                speed_m_s=state['speed_m_s'], live_time_s=live.time_s,
                live_distance_m=live.distance_m, active_count=len(live.active),
                traffic_speed_m_s=live.speed_m_s,
                physics_steps=clock['physics_steps'],
                native_physics_steps=clock['native_physics_steps'],
                time_s=clock['time_s'], absolute_time_s=clock['absolute_time_s'])


class ControlValidationScenario:
    """Drive real widget callbacks at boundaries and audit pause without stepping.

    Call before_tick *before* the runner's paused loop. Call paused_frame after
    each UI-only render in that loop, supplying a fresh native snapshot. Consume
    the sidebar mailbox only after the paused loop returns.
    """
    def __init__(self, name, hz=120):
        if name not in VALIDATION_NAMES or type(hz) is not int or hz != 120:
            raise ValueError('Control validation requires endpoints at 120 Hz')
        self.name = name
        self.hz = hz
        self.stage = 0
        self.requests = []
        self.pause = None
        self.rejections = []
        self.errors = []

    def _set_model(self, sidebar, key, value):
        sidebar.models[key].set_value(value)
        if sidebar.settings.pending[key] != value:
            raise RuntimeError(f'Native callback did not accept {key}={value}')

    def _request(self, sidebar, phase):
        # Respect speed coupling at every intermediate widget callback.
        for key, value in (
                ('traffic_count', phase['traffic_count']),
                ('traffic_mph', min(phase['traffic_mph'], sidebar.settings.pending['traffic_mph'])),
                ('ego_mph', phase['ego_mph']), ('traffic_mph', phase['traffic_mph']),
                ('following_s', phase['following_s'])):
            self._set_model(sidebar, key, value)

    def before_tick(self, sidebar, tick, snapshot):
        if self.stage >= len(PHASES):
            return
        phase = PHASES[self.stage]
        due = phase['request_s'] * self.hz
        if tick < due:
            return
        if tick != due or sidebar.settings.locked or sidebar.settings.restart_request is not None:
            raise RuntimeError('Control-validation boundary missed or interrupted by restart')
        if sidebar.state['paused']:
            raise RuntimeError('Unexpected external pause during control validation')
        if self.stage == 0:
            validation_initial_controls(self.name, sidebar.settings.active)
        active_before = dict(sidebar.settings.active)
        if self.stage == 1:
            self.pause = dict(before=deepcopy(snapshot), frames=[], resumed=False,
                              active_before=active_before, event_count_before=len(sidebar.settings.events))
            sidebar.toggle_pause()
            if not sidebar.state['paused']:
                raise RuntimeError('Native pause callback did not pause')
            # A physically unsupported pair must be rejected and reset by the
            # actual model callback before the valid rapid request batch.
            pending_before = dict(sidebar.settings.pending)
            sidebar.models['traffic_mph'].set_value(25.)
            rejected = (sidebar.settings.pending == pending_before
                        and math.isclose(sidebar.models['traffic_mph'].as_float, 5., abs_tol=1e-5))
            self.rejections.append(dict(tick=tick, key='traffic_mph', requested=25.,
                                        reason='traffic target exceeds main-car target', rejected=rejected))
            if not rejected:
                raise RuntimeError('Unsupported endpoint combination was accepted')
            # All values stay in one bounded mailbox until resume. No physics
            # work or consumed event is allowed for intermediate slider values.
            for count in (4, 12, 40, 80) * 16:
                self._set_model(sidebar, 'traffic_count', count)
            for speed in (10., 20., 35.):
                self._set_model(sidebar, 'ego_mph', speed)
            for speed in (5., 15., 25.):
                self._set_model(sidebar, 'traffic_mph', speed)
            for gap in (3., 2.4, 1.8):
                self._set_model(sidebar, 'following_s', gap)
        else:
            self._request(sidebar, phase)
        expected = {key: phase[key] for key in DRIVING_KEYS}
        if any(sidebar.settings.pending[key] != value for key, value in expected.items()):
            raise RuntimeError('Native endpoint request differs from declared phase')
        self.requests.append(dict(tick=tick, expected=expected,
                                 changes={key: value for key, value in expected.items()
                                          if active_before[key] != value}))
        self.stage += 1

    def paused_frame(self, sidebar, tick, snapshot):
        if self.pause is None or self.pause['resumed']:
            raise RuntimeError('Unexpected paused frame during control validation')
        stable = (snapshot == self.pause['before']
                  and tick == 15 * self.hz
                  and sidebar.settings.active == self.pause['active_before']
                  and len(sidebar.settings.events) == self.pause['event_count_before']
                  and set(sidebar.settings.live_requests) <= set(DRIVING_KEYS)
                  and all(sidebar.settings.pending[key] == PHASES[1][key] for key in DRIVING_KEYS))
        self.pause['frames'].append(dict(snapshot=deepcopy(snapshot), stable=stable))
        if not stable:
            self.errors.append('Simulation clock, body state or applied settings advanced while paused')
            raise RuntimeError(self.errors[-1])
        if len(self.pause['frames']) == 4:
            sidebar.toggle_pause()
            self.pause['resumed'] = not sidebar.state['paused']
            if not self.pause['resumed']:
                raise RuntimeError('Native resume callback did not resume')

    def finish(self, rows, lifecycle_events, control_events, restarted=False):
        """Fail closed on missing evidence; physical safety gates remain separate."""
        checkpoints = []
        for phase in PHASES:
            row = next((r for r in rows if r.get('sim_time_s', -1) >= phase['deadline_s']), None)
            expected = dict(active_background_count=phase['traffic_count'],
                            requested_background_count=phase['traffic_count'],
                            applied_ego_target_m_s=phase['ego_mph'] * MPH,
                            applied_traffic_speed_m_s=phase['traffic_mph'] * MPH,
                            applied_following_s=phase['following_s'])
            actual = {key: None if row is None else row.get(key) for key in expected}
            reached = (row is not None and row.get('sim_time_s') <= phase['deadline_s'] + 1 / self.hz + 1e-6
                       and all(type(actual[key]) in (int, float) and math.isfinite(actual[key])
                               and math.isclose(actual[key], value, rel_tol=0, abs_tol=1e-6)
                               for key, value in expected.items()))
            checkpoints.append(dict(request_s=phase['request_s'], deadline_s=phase['deadline_s'],
                                    expected=expected, actual=actual, passed=reached))
        actual_events = [dict(tick=e.get('tick'), changes=e.get('changes')) for e in control_events]
        expected_events = [dict(tick=r['tick'], changes=r['changes']) for r in self.requests]
        pause_ok = (self.pause is not None and self.pause['resumed']
                    and len(self.pause['frames']) == 4 and all(f['stable'] for f in self.pause['frames']))
        tail = [r for r in rows if PASSING_START_S <= r.get('sim_time_s', -1) <= PASSING_END_S]
        tail_settled = (bool(tail) and tail[0]['sim_time_s'] <= PASSING_START_S + 1 / self.hz + 1e-6
                        and tail[-1]['sim_time_s'] >= PASSING_END_S - 1 / self.hz - 1e-6
                        and all(r.get('active_background_count') == 80
                                and r.get('requested_background_count') == 80
                                and math.isclose(r.get('applied_traffic_speed_m_s', -1), 15 * MPH, abs_tol=1e-6)
                                for r in tail))
        passes = tail[-1].get('completed_passes', 0) - tail[0].get('completed_passes', 0) if tail else 0
        lanes = tail[-1].get('completed_lane_changes', 0) - tail[0].get('completed_lane_changes', 0) if tail else 0
        lifecycle_ok = (bool(lifecycle_events)
                        and {'enter', 'exit'} <= {e.get('type') for e in lifecycle_events}
                        and all(type(e.get('tick')) is int and e.get('ego_exclusion_m') == 160.
                                and type(e.get('actual_arc_gap_m')) in (int, float)
                                and math.isfinite(e['actual_arc_gap_m']) and e['actual_arc_gap_m'] >= 160.
                                for e in lifecycle_events)
                        and all(b['tick'] - a['tick'] >= 30 for a, b in zip(lifecycle_events, lifecycle_events[1:])))
        gates = dict(all_phases_requested=self.stage == len(PHASES), no_restart=not restarted,
                     endpoint_deadlines=all(c['passed'] for c in checkpoints),
                     latest_requests_only=actual_events == expected_events and len(actual_events) == len(PHASES),
                     pause_clock_and_mailbox=pause_ok,
                     invalid_combination_rejected=len(self.rejections) == 1 and self.rejections[0]['rejected'],
                     bounded_remote_lifecycle=lifecycle_ok,
                     settled_passing_tail=tail_settled and passes >= 1 and lanes >= 1,
                     no_harness_errors=not self.errors)
        return dict(name=self.name, passed=all(gates.values()), gates=gates,
                    checkpoints=checkpoints, requests=self.requests, pause=self.pause,
                    rejections=self.rejections, errors=self.errors,
                    passing_tail=dict(start_s=PASSING_START_S, end_s=PASSING_END_S,
                                      completed_passes=passes, completed_lane_changes=lanes,
                                      settled=tail_settled),
                    scope='native widget callbacks and declared sequence; not mouse latency, all combinations or endurance')
