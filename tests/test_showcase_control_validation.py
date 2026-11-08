"""CPU evidence for scenario/acceptance contracts, not native GUI validation."""
import copy
from contextlib import redirect_stdout
import io
import json
import math
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.loop_showcase_config import fleet_specs, make_config, validate_config
from experiments.showcase_controls import SidebarState, defaults
from experiments.showcase_control_validation import (
    ControlValidationScenario, PHASES, validation_initial_controls, validation_snapshot)
from traffic.live_showcase import LiveShowcaseController, LiveTraffic
from traffic.loop_showcase_control import ShowcaseController


class Model:
    def __init__(self, owner, key):
        self.owner, self.key = owner, key
        self.value = owner.settings.pending[key]

    def set_value(self, value):
        self.value = value
        try:
            self.owner.settings.edit(self.key, value)
        except ValueError:
            self.value = self.owner.settings.pending[self.key]

    @property
    def as_float(self):
        return float(self.value)


class Sidebar:
    def __init__(self):
        self.settings = SidebarState(validation_initial_controls('endpoints'), live_driving=True)
        self.models = {key: Model(self, key) for key in self.settings.pending}
        self.state = dict(paused=False)

    def toggle_pause(self):
        self.state['paused'] = not self.state['paused']


def snapshot(tick=1800):
    return dict(tick=tick, native_physics_steps=tick + 2, physics_steps=tick,
                time_s=tick / 120, absolute_time_s=tick / 120 + 2 / 120,
                position_m=[500., 10., .3], speed_m_s=2., live_time_s=tick / 120,
                live_distance_m=10., active_count=4, traffic_speed_m_s=2.2352)


def complete_harness():
    scenario, sidebar = ControlValidationScenario('endpoints'), Sidebar()
    for phase in PHASES:
        tick = phase['request_s'] * 120
        state = snapshot(tick)
        scenario.before_tick(sidebar, tick, state)
        while sidebar.state['paused']:
            scenario.paused_frame(sidebar, tick, state)
        sidebar.settings.consume_live(tick)
    return scenario, sidebar


def good_rows():
    rows = []
    for phase in PHASES:
        rows.append(dict(sim_time_s=phase['deadline_s'], active_background_count=phase['traffic_count'],
                         requested_background_count=phase['traffic_count'],
                         applied_ego_target_m_s=phase['ego_mph'] * .44704,
                         applied_traffic_speed_m_s=phase['traffic_mph'] * .44704,
                         applied_following_s=phase['following_s'], completed_passes=0,
                         completed_lane_changes=0))
    rows.append({**rows[-1], 'sim_time_s': 180., 'completed_passes': 2, 'completed_lane_changes': 1})
    return rows


def good_events():
    return [dict(type='exit', tick=600, ego_exclusion_m=160., actual_arc_gap_m=800.),
            dict(type='enter', tick=1800, ego_exclusion_m=160., actual_arc_gap_m=700.)]


class ControlValidationTests(unittest.TestCase):
    def test_statistics_rejects_stationary_contact_fixture_without_changing_old_profile(self):
        old = make_config('v07', mode='contact-check')
        validate_config(old)
        self.assertNotIn('traffic_statistics', old)
        with self.assertRaises(ValueError):
            make_config('v07', mode='contact-check', statistics=True)
        for mode in ('blocked', 'dropout'):
            with self.subTest(mode=mode):
                cfg = make_config('v07', mode=mode, statistics=True)
                validate_config(cfg)
                self.assertTrue(cfg['traffic_statistics'])

    def test_opt_in_config_rejects_short_incompatible_and_legacy_requests(self):
        cfg = make_config('v09', control_validation='endpoints', drive_s=180,
                          gui=False, panel_hidden=True, vehicle='public')
        validate_config(cfg)
        self.assertEqual(cfg['controls'], validation_initial_controls('endpoints'))
        self.assertFalse(cfg['control_smoke'])
        self.assertNotIn('control_validation', make_config('v09'))
        for kwargs in (
                dict(version='v08'), dict(drive_s=179), dict(control_smoke=True),
                dict(gui=False, panel_hidden=False), dict(controls=defaults())):
            args = dict(version='v09', control_validation='endpoints', drive_s=180)
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                make_config(**args)
        damaged = copy.deepcopy(cfg)
        damaged['control_validation'] = 'unknown'
        with self.assertRaises(ValueError):
            validate_config(damaged)

    def test_cli_preflight_uses_declared_initial_settings_and_preserves_display_choice(self):
        from scripts import demo_loop_showcase
        command = ['demo_loop_showcase.py', '--version', 'v09', '--vehicle', 'public',
                   '--control-validation', 'endpoints', '--drive-seconds', '180',
                   '--headless', '--panel-hidden', '--render-hz', '20', '--check']
        output = io.StringIO()
        with patch.object(sys, 'argv', command), patch.object(demo_loop_showcase, 'load_config', return_value={}), \
                patch.object(demo_loop_showcase, 'check_showcase', return_value=(None, {})) as setup, \
                patch.object(demo_loop_showcase, 'run_once') as run, redirect_stdout(output):
            self.assertEqual(demo_loop_showcase.main(), 0)
        cfg = json.loads(output.getvalue())['config']
        validate_config(cfg)
        self.assertEqual(cfg['controls']['traffic_count'], 12)
        self.assertEqual(cfg['controls']['ego_mph'], 10.)
        self.assertEqual(cfg['controls']['render_hz'], 20)
        setup.assert_called_once()
        run.assert_not_called()
        with patch.object(sys, 'argv', command + ['--ego-mph', '35']), \
                patch.object(demo_loop_showcase, 'check_showcase') as setup:
            with self.assertRaisesRegex(ValueError, 'declared initial driving'):
                demo_loop_showcase.main()
        setup.assert_not_called()

    def test_declared_initial_state_and_unknown_profiles(self):
        values = validation_initial_controls('endpoints')
        self.assertEqual([values[k] for k in ('traffic_count', 'ego_mph', 'traffic_mph', 'following_s')],
                         [12, 10., 5., 2.4])
        self.assertEqual(validation_initial_controls('endpoints', {**values, 'render_hz': 20})['render_hz'], 20)
        for bad in ('smoke', '', None):
            with self.assertRaises(ValueError):
                validation_initial_controls(bad)
        with self.assertRaises(ValueError):
            validation_initial_controls('endpoints', defaults())
        with self.assertRaises(ValueError):
            ControlValidationScenario('endpoints', 60)

    def test_rapid_requests_wait_for_resume_and_only_latest_values_apply(self):
        scenario, sidebar = ControlValidationScenario('endpoints'), Sidebar()
        scenario.before_tick(sidebar, 600, snapshot(600))
        sidebar.settings.consume_live(600)
        before = dict(sidebar.settings.active)
        scenario.before_tick(sidebar, 1800, snapshot())
        self.assertTrue(sidebar.state['paused'])
        self.assertEqual(sidebar.settings.active, before)
        self.assertEqual(len(sidebar.settings.events), 1)
        self.assertEqual(len(sidebar.settings.live_requests), 4)
        for index in range(4):
            scenario.paused_frame(sidebar, 1800, snapshot())
            self.assertEqual(sidebar.settings.active, before)
            self.assertEqual(sidebar.state['paused'], index < 3)
        self.assertEqual(sidebar.settings.consume_live(1800),
                         dict(traffic_count=80, ego_mph=35., traffic_mph=25., following_s=1.8))
        self.assertEqual(len(sidebar.settings.events), 2)
        self.assertTrue(scenario.rejections[0]['rejected'])

    def test_pause_rejects_hidden_native_ticks_body_motion_or_mailbox_consumption(self):
        for corruption in ('native_physics_steps', 'position_m', 'consume'):
            with self.subTest(corruption=corruption):
                scenario, sidebar = ControlValidationScenario('endpoints'), Sidebar()
                scenario.before_tick(sidebar, 600, snapshot(600))
                sidebar.settings.consume_live(600)
                scenario.before_tick(sidebar, 1800, snapshot())
                changed = snapshot()
                if corruption == 'consume':
                    sidebar.settings.consume_live(1800)
                elif corruption == 'position_m':
                    changed['position_m'][0] += .0001
                else:
                    changed[corruption] += 1
                with self.assertRaisesRegex(RuntimeError, 'advanced while paused'):
                    scenario.paused_frame(sidebar, 1800, changed)

    def test_snapshot_checks_native_clock_but_allows_ui_frames(self):
        live = SimpleNamespace(time_s=1., distance_m=2., active={'a'}, speed_m_s=3.)
        state = dict(position_m=[1., 2., 3.], speed_m_s=4.)
        clock = dict(physics_steps=120, native_physics_steps=122, time_s=1., absolute_time_s=1.02, render_count=1)
        before = validation_snapshot(120, state, live, clock)
        after = validation_snapshot(120, state, live, {**clock, 'render_count': 5})
        self.assertEqual(before, after)
        self.assertNotEqual(before, validation_snapshot(120, state, live, {**clock, 'native_physics_steps': 123}))

    def test_deadlines_require_actual_population_and_ramps_plus_passing_afterward(self):
        scenario, sidebar = complete_harness()
        rows, events = good_rows(), good_events()
        result = scenario.finish(rows, events, sidebar.settings.events)
        self.assertTrue(result['passed'], result)
        mutations = (
            ('active_background_count', 79, 1, 'endpoint_deadlines'),
            ('applied_ego_target_m_s', 15., 1, 'endpoint_deadlines'),
            ('applied_following_s', 2.4, 0, 'endpoint_deadlines'),
            ('sim_time_s', 101., 2, 'endpoint_deadlines'),
            ('completed_passes', 0, -1, 'settled_passing_tail'),
            ('completed_lane_changes', 0, -1, 'settled_passing_tail'),
        )
        for key, value, index, gate in mutations:
            with self.subTest(key=key):
                changed = copy.deepcopy(rows)
                changed[index][key] = value
                self.assertFalse(scenario.finish(changed, events, sidebar.settings.events)['gates'][gate])
        self.assertFalse(scenario.finish(rows, events, sidebar.settings.events, restarted=True)['passed'])

    def test_remote_lifecycle_requires_actual_gap_and_rate_not_only_label(self):
        scenario, sidebar = complete_harness()
        for value in (None, float('nan'), 159.999):
            events = good_events()
            events[0]['actual_arc_gap_m'] = value
            self.assertFalse(scenario.finish(good_rows(), events, sidebar.settings.events)['gates']['bounded_remote_lifecycle'])
        events = good_events()
        events[1]['tick'] = events[0]['tick'] + 29
        self.assertFalse(scenario.finish(good_rows(), events, sidebar.settings.events)['gates']['bounded_remote_lifecycle'])

    def test_extra_requests_missing_pause_and_partial_trace_cannot_pass(self):
        scenario, sidebar = complete_harness()
        extra = sidebar.settings.events + [dict(tick=20000, changes={'traffic_count': 4})]
        self.assertFalse(scenario.finish(good_rows(), good_events(), extra)['gates']['latest_requests_only'])
        scenario.pause['frames'].pop()
        self.assertFalse(scenario.finish(good_rows(), good_events(), sidebar.settings.events)['gates']['pause_clock_and_mailbox'])
        self.assertFalse(ControlValidationScenario('endpoints').finish([], [], [])['passed'])

    def test_missed_boundary_or_restart_fails_explicitly(self):
        for tick, locked in ((601, False), (600, True)):
            scenario, sidebar = ControlValidationScenario('endpoints'), Sidebar()
            sidebar.settings.locked = locked
            with self.assertRaisesRegex(RuntimeError, 'boundary missed or interrupted'):
                scenario.before_tick(sidebar, tick, snapshot(tick))

    def test_protected_population_can_remain_above_four_without_unsafe_removal(self):
        cfg = make_config('v09')
        live = LiveTraffic(fleet_specs(cfg), cfg['controls'])
        live.configure({**live.settings, 'traffic_count': 4})
        # Fixed ego/background positions isolate the infeasible protected case.
        for tick in range(3600):
            live.reconcile(tick, (500., 0.))
        self.assertGreater(len(live.active), 4)
        self.assertLess(len(live.active), 12)
        self.assertTrue(all(e['actual_arc_gap_m'] >= 160. for e in live.events))
        self.assertEqual(live.reconcile(3600, (500., 0.)), [])
        self.assertTrue(all(abs((live.specs[k].station_m + math.pi * 500) % (math.tau * 500)
                               - math.pi * 500) < 160. for k in live.active))

    def test_new_lane_changes_are_reenabled_after_transitions_finish(self):
        controller = LiveShowcaseController('episode', 'ego', initial_lane=1, optimized=True)
        with patch.object(ShowcaseController, '_candidate_safe', return_value=True) as base:
            controller.traffic_transition_pending = True
            self.assertFalse(controller._candidate_safe(None, None, None))
            controller.traffic_transition_pending = False
            controller.changing_traffic_speed = True
            self.assertFalse(controller._candidate_safe(None, None, None))
            self.assertEqual(base.call_count, 0)
            controller.changing_traffic_speed = False
            self.assertTrue(controller._candidate_safe(None, None, None))
            self.assertEqual(base.call_count, 1)


if __name__ == '__main__':
    unittest.main()
