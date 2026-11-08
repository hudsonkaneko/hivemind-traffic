"""Fault-injected UI lifecycle contracts; no Isaac/GPU runtime needed."""
import copy
import math
import sys
import types
import unittest
from unittest.mock import Mock, patch

from visualization.showcase_startup import (
    ShowcaseStartupError, wait_for_native_showcase_ready,
    wait_for_showcase_ready, window_launch_config,
)


GOOD = dict(native_window_extent=(1440, 900), viewport_resolution=(960, 600),
            viewport_window_extent=(700, 500), viewport_visible=True,
            sidebar_docked=True)


class FakeLifecycle:
    def __init__(self, states=None, *, step_s=.01):
        self.states = states or [GOOD]
        self.updates = 0
        self.now = 0.
        self.step_s = step_s

    def pump(self):
        self.updates += 1
        self.now += self.step_s

    def sample(self):
        return dict(self.states[min(self.updates - 1, len(self.states) - 1)])

    def run(self, **options):
        return wait_for_showcase_ready(pump=self.pump, sample=self.sample,
            is_running=lambda: True, clock=lambda: self.now, **options)


class LaunchConfigTests(unittest.TestCase):
    def test_uses_existing_dimensions_without_changing_renderer_or_input(self):
        profile = dict(width=480, height=300, renderer='MinimalRendering',
                       extra_args=['--/app/vsync=false'])
        original = copy.deepcopy(profile)
        result = window_launch_config(profile, gui=True)
        self.assertEqual(profile, original)
        self.assertEqual((result['width'], result['height']), (480, 300))
        self.assertEqual((result['window_width'], result['window_height']), (1440, 900))
        self.assertEqual(result['extra_args'][-1], '--/app/window/saveSizeOnExit=false')
        self.assertFalse(any('/persistent/' in value for value in result['extra_args']))

    def test_explicit_dimensions_and_headless_profile_preserved(self):
        profile = dict(window_width=1000, window_height=700, extra_args=['--example'])
        self.assertEqual(window_launch_config(profile, gui=False), profile)
        self.assertEqual(window_launch_config(profile, gui=True)['window_width'], 1000)

    def test_bad_dimensions_rejected(self):
        for value in (0, -1, True, math.inf, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                window_launch_config({'window_height': value}, gui=True)


class ReadinessTests(unittest.TestCase):
    def test_requires_three_observed_lifecycle_frames(self):
        life = FakeLifecycle()
        result = life.run()
        self.assertTrue(result['passed'])
        self.assertEqual(life.updates, 3)
        self.assertEqual(result['actions'], [])

    def test_zero_native_height_repaired_once_then_observed(self):
        broken = {**GOOD, 'native_window_extent': (120, 0)}
        life = FakeLifecycle([broken] * 5 + [GOOD])
        repair = Mock()
        result = life.run(repair_window=repair)
        repair.assert_called_once_with()
        self.assertEqual(result['updates'], 8)
        self.assertEqual(result['actions'], [dict(update=3, action='resize_invalid_window')])
        self.assertEqual(result['observations'][0]['native_window_extent'], [120., 0.])

    def test_resize_request_alone_never_means_ready(self):
        life = FakeLifecycle([{**GOOD, 'native_window_extent': (120, 0)}])
        repair = Mock()
        with self.assertRaises(ShowcaseStartupError) as caught:
            life.run(repair_window=repair, max_updates=7)
        self.assertEqual(caught.exception.evidence['reason'], 'update_limit')
        self.assertFalse(caught.exception.evidence['passed'])
        repair.assert_called_once()

    def test_dock_retry_waits_for_native_layout_and_rechecks_docked(self):
        undocked = {**GOOD, 'sidebar_docked': False}
        life = FakeLifecycle([undocked] * 6 + [GOOD])
        dock_updates = []
        result = life.run(require_sidebar=True,
                          request_dock=lambda: dock_updates.append(life.updates))
        self.assertEqual(dock_updates, [1, 5])
        self.assertEqual(result['updates'], 9)

    def test_docking_not_attempted_against_invalid_native_window(self):
        life = FakeLifecycle([{**GOOD, 'sidebar_docked': False, 'native_window_extent': (0, 0)}])
        dock = Mock()
        with self.assertRaises(ShowcaseStartupError):
            life.run(require_sidebar=True, request_dock=dock, max_updates=4)
        dock.assert_not_called()

    def test_layout_regression_resets_consecutive_observation_count(self):
        life = FakeLifecycle([GOOD, GOOD, {**GOOD, 'viewport_window_extent': (0, 0)}, GOOD])
        result = life.run()
        self.assertEqual(result['updates'], 6)

    def test_headless_ignores_native_window_docking_and_visibility(self):
        life = FakeLifecycle([dict(viewport_resolution=(960, 600))])
        repair, dock = Mock(), Mock()
        result = life.run(gui=False, require_sidebar=True,
                          repair_window=repair, request_dock=dock)
        self.assertTrue(result['passed'])
        self.assertFalse(result['sidebar_required'])
        repair.assert_not_called()
        dock.assert_not_called()

    def test_invalid_headless_render_resolution_still_rejected(self):
        for resolution in (None, (960, 0), (math.nan, 600), (1, 2, 3), 'bad'):
            with self.subTest(resolution=resolution):
                life = FakeLifecycle([dict(viewport_resolution=resolution)])
                with self.assertRaises(ShowcaseStartupError) as caught:
                    life.run(gui=False, max_updates=3)
                self.assertIn('invalid_viewport_resolution',
                              caught.exception.evidence['observations'][-1]['pending'])

    def test_visible_but_unselected_viewport_is_not_ready(self):
        life = FakeLifecycle([{**GOOD, 'viewport_visible': False}])
        with self.assertRaises(ShowcaseStartupError):
            life.run(max_updates=3)

    def test_pump_cost_cannot_overrun_wall_deadline_and_pass(self):
        life = FakeLifecycle(step_s=6.)
        with self.assertRaises(ShowcaseStartupError) as caught:
            life.run(timeout_s=15.)
        self.assertEqual(caught.exception.evidence['reason'], 'wall_timeout')
        self.assertEqual(caught.exception.evidence['elapsed_s'], 18.)
        self.assertEqual(caught.exception.evidence['updates'], 3)

    def test_user_close_stops_before_any_pump(self):
        pump = Mock()
        with self.assertRaises(ShowcaseStartupError) as caught:
            wait_for_showcase_ready(pump=pump, sample=lambda: GOOD, is_running=lambda: False)
        self.assertEqual(caught.exception.evidence['reason'], 'application_closed')
        pump.assert_not_called()

    def test_native_errors_preserve_exception_and_fail_instead_of_hiding_it(self):
        for action in ('pump', 'sample', 'request_dock', 'repair_window'):
            with self.subTest(action=action):
                life = FakeLifecycle([{**GOOD, 'sidebar_docked': False,
                                       'native_window_extent': (0, 0) if action == 'repair_window' else (1440, 900)}])
                options = dict(pump=life.pump, sample=life.sample, is_running=lambda: True,
                               request_dock=lambda: None, repair_window=lambda: None,
                               require_sidebar=True, clock=lambda: life.now)
                options[action] = Mock(side_effect=RuntimeError('injected native failure'))
                with self.assertRaises(ShowcaseStartupError) as caught:
                    wait_for_showcase_ready(**options)
                self.assertIsInstance(caught.exception.__cause__, RuntimeError)
                self.assertIn('injected native failure', caught.exception.evidence['actions'][-1]['error'])

    def test_bad_bounds_rejected(self):
        for options in ({'max_updates': 0}, {'stable_updates': 0}, {'stable_updates': 4, 'max_updates': 3},
                        {'retry_every_updates': 0}, {'timeout_s': math.inf}, {'timeout_s': 0}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                FakeLifecycle().run(**options)


class NativeAdapterTests(unittest.TestCase):
    def test_adapter_observes_native_extents_and_uses_sidebar_lifecycle(self):
        omni = types.ModuleType('omni')
        appwindow = types.ModuleType('omni.appwindow')
        utility = types.ModuleType('omni.kit.viewport.utility')
        omni.appwindow = appwindow
        native = Mock()
        native.get_width.return_value = 1440
        native.get_height.return_value = 900
        appwindow.get_default_app_window = Mock(return_value=native)
        visible = types.SimpleNamespace(frame=types.SimpleNamespace(computed_width=640, computed_height=480),
                                        visible=True, docked=True, selected_in_dock=True)
        utility.get_active_viewport_window = Mock(return_value=visible)
        viewport = types.SimpleNamespace(resolution=(960, 600))
        sidebar = types.SimpleNamespace(window=types.SimpleNamespace(visible=True), docked=False)
        sidebar.dock = lambda: setattr(sidebar, 'docked', True)
        with patch.dict(sys.modules, {'omni': omni, 'omni.appwindow': appwindow,
                                      'omni.kit.viewport.utility': utility}):
            result = wait_for_native_showcase_ready(pump=lambda: None, is_running=lambda: True,
                                                   viewport=viewport, sidebar=sidebar)
            self.assertTrue(result['passed'])
            self.assertEqual(result['updates'], 4)
            self.assertEqual(result['observations'][-1]['viewport_window_extent'], [640., 480.])
            native.resize.assert_not_called()
            sidebar.window.visible = False
            sidebar.docked = False
            result = wait_for_native_showcase_ready(pump=lambda: None, is_running=lambda: True,
                                                   viewport=viewport, sidebar=sidebar)
            self.assertFalse(result['sidebar_required'])
            utility.get_active_viewport_window.reset_mock()
            appwindow.get_default_app_window.reset_mock()
            result = wait_for_native_showcase_ready(pump=lambda: None, is_running=lambda: True,
                                                   viewport=viewport, sidebar=sidebar, gui=False)
            self.assertTrue(result['passed'])
            appwindow.get_default_app_window.assert_not_called()
            utility.get_active_viewport_window.assert_not_called()


if __name__ == '__main__':
    unittest.main()
