"""CPU contracts for the v09 gap supervisor; these are not native dynamics tests."""
from dataclasses import replace
import ast
import json
import math
import unittest
from unittest.mock import patch

from traffic.live_showcase import LiveShowcaseController
from traffic.loop_showcase_control import ObjectTrack, ShowcaseConfig, ShowcaseController, RadialPath
from traffic.path_following import VehicleState


class GapCreationTests(unittest.TestCase):
    base_speed = 6.7056
    positions = ((0, -56.664), (0, 88.416), (1, -38.664), (1, 106.416),
                 (2, -449.363), (2, 17.876), (2, 162.956), (3, -.124), (3, 144.956))

    def make(self, optimized=True):
        cfg = replace(ShowcaseConfig(), ego_length_m=4.8, ego_width_m=1.8,
                      wheelbase_m=3.2, rear_axle_offset_m=1.6)
        return LiveShowcaseController('episode', 'ego', cfg, initial_lane=2, optimized=optimized)

    def frame(self, tick=0, lag=0., speed=None, rotation=0., base_speed=None):
        velocity = self.base_speed if base_speed is None else base_speed
        station = velocity * tick / 120 + rotation
        tracks = []
        for i, (lane, gap) in enumerate(self.positions):
            radius = 500 + lane * 3.7
            angle = (station + gap) / 500
            tracks.append(ObjectTrack('episode', 'car' + str(i), tick,
                radius * math.cos(angle), radius * math.sin(angle), angle + math.pi / 2,
                velocity * radius / 500, width_m=1.9))
        angle = (station - lag) / 500
        radius = 507.4
        state = VehicleState('episode', 'ego', tick, radius * math.cos(angle),
            radius * math.sin(angle), angle + math.pi / 2,
            velocity * radius / 500 if speed is None else speed)
        return state, tracks, station - lag

    def plan(self, controller, tick=0, **kwargs):
        state, tracks, station = self.frame(tick, **kwargs)
        controller._unwrapped_s = station
        reason = controller._plan(state, tick, tracks)
        return reason, state, tracks

    def started(self):
        controller = self.make()
        self.plan(controller)
        reason, state, tracks = self.plan(controller, 240)
        self.assertEqual(reason, 'gap_creation_slowing')
        return controller, state, tracks

    def test_repaired_geometry_has_bounded_opening_and_full_rear_window(self):
        controller = self.make()
        state, tracks, station = self.frame()
        controller._unwrapped_s = station
        rear = controller._gap_geometry(state, tracks, 21.)
        self.assertTrue(rear['eligible'])
        self.assertAlmostEqual(rear['rear_center_gap_m'], 449.363)
        self.assertGreater(rear['required_rear_center_gap_m'], 428.)
        self.assertLess(rear['minimum_rear_reserve_m'], 21.)
        with patch.object(ShowcaseController, '_plan', autospec=True, wraps=ShowcaseController._plan) as spy:
            # A failed feasibility scan is explicitly bounded even without an opening.
            spy.return_value = 'blocked_following'
            self.assertEqual(controller._gap_opening(state, tracks), dict(possible=False, queries=12))
            self.assertEqual(spy.call_count, 12)
        opening = controller._gap_opening(state, tracks)
        self.assertTrue(opening['possible'])
        self.assertLessEqual(opening['queries'], 12)
        self.assertEqual(opening['lane'], 1)
        self.assertEqual(controller.config.target_speed_m_s, 15.6464)
        self.assertFalse(controller._maneuver)

    def test_zero_lag_actual_speed_is_still_blocked_in_unchanged_planner(self):
        state, tracks, station = self.frame()
        for optimized in (False, True):
            with self.subTest(optimized=optimized):
                base = ShowcaseController('episode', 'ego', self.make().config, initial_lane=2, optimized=optimized)
                live = self.make(optimized)
                base._unwrapped_s = live._unwrapped_s = station
                for lane in (1, 3):
                    path = RadialPath(2, lane, station + 2, 80)
                    self.assertEqual(base._candidate_safe(path, state, tracks), live._candidate_safe(path, state, tracks))
                self.assertEqual(base._plan(state, 0, tracks), 'blocked_following')
                self.assertFalse(base._maneuver)

    def test_small_rear_window_rejects_without_lowering_follow_target(self):
        controller = self.make()
        self.positions = tuple((lane, -420 if lane == 2 and gap < 0 else gap) for lane, gap in self.positions)
        self.plan(controller)
        reason, _, _ = self.plan(controller, 240)
        self.assertEqual(reason, 'blocked_following')
        self.assertEqual(controller._gap_reason, 'rear_window')
        self.assertEqual(controller._gap_attempts, 0)
        self.assertIsNone(controller._gap_attempt)

    def test_wide_yawed_adjacent_object_is_included_in_rear_corridor(self):
        controller = self.make()
        state, tracks, station = self.frame()
        controller._unwrapped_s = station
        self.assertTrue(controller._gap_geometry(state, tracks, 21)['eligible'])
        tracks[2] = replace(tracks[2], width_m=2.5, length_m=8.)
        # Maximum valid dimensions plus heading tolerance overlap the certified corridor.
        evidence = controller._gap_geometry(state, tracks, 21)
        self.assertFalse(evidence['eligible'])
        self.assertEqual(evidence['rear_vehicle_id'], tracks[2].vehicle_id)

    def test_complete_object_radial_tolerance_enters_boundary_corridor(self):
        controller = self.make()
        state, tracks, station = self.frame()
        controller._unwrapped_s = station
        obj = tracks[2]
        angle = math.atan2(obj.y_m, obj.x_m)
        radius = 503.7 - .09
        tracks[2] = replace(obj, x_m=radius * math.cos(angle), y_m=radius * math.sin(angle),
                            width_m=2.15, length_m=8.)
        self.assertIsNone(controller._validate(state, 0, tracks))
        evidence = controller._gap_geometry(state, tracks, 21)
        self.assertFalse(evidence['eligible'])
        self.assertEqual(evidence['rear_vehicle_id'], obj.vehicle_id)

    def test_low_speed_does_not_attempt_or_search(self):
        controller = self.make()
        self.plan(controller, base_speed=2.2352)
        with patch.object(controller, '_gap_opening', side_effect=AssertionError('must not search')):
            self.plan(controller, 240, base_speed=2.2352)
        self.assertEqual(controller._gap_reason, 'not_settled_following_speed')
        self.assertEqual(controller._gap_attempts, 0)

    def test_no_opening_and_repeat_formation_do_not_repeat_search(self):
        controller = self.make()
        self.plan(controller)
        with patch.object(controller, '_gap_opening', return_value=dict(possible=False, queries=12)) as probe:
            self.plan(controller, 240)
            self.plan(controller, 600)
            self.assertEqual(probe.call_count, 1)
        self.assertEqual(controller._gap_reason, 'no_bounded_recovery_opening')

    def test_approach_transient_can_settle_without_consuming_formation(self):
        controller = self.make()
        self.plan(controller, speed=7.5)
        self.plan(controller, 240, speed=7.5)
        self.assertEqual(len(controller._gap_examined), 0)
        self.plan(controller, 252)
        self.plan(controller, 480)
        self.assertIsNone(controller._gap_attempt)
        self.plan(controller, 492)
        self.assertIsNotNone(controller._gap_attempt)

    def test_cap_slew_is_bounded_and_front_safety_keeps_authority(self):
        controller, initial, _ = self.started()
        reason, _, _ = self.plan(controller, 252)
        self.assertEqual(reason, 'gap_creation_slowing')
        self.assertAlmostEqual(controller._target_speed, initial.speed_m_s - .1)
        self.assertGreaterEqual(controller._gap_attempt['floor'], 2.5)
        self.assertLessEqual(initial.speed_m_s - controller._gap_attempt['floor'], 4.5)
        # A closing/front-brake condition releases the cap; it never overrides braking.
        reason, _, _ = self.plan(controller, 264, speed=15.)
        self.assertEqual(reason, 'unsafe_closing_gap_brake')
        self.assertEqual(controller._target_speed, 0.)
        self.assertEqual(controller._gap_state, 'recovering')

    def test_lag_release_then_actual_fresh_maneuver_ends_supervisor(self):
        controller, _, _ = self.started()
        reason, _, _ = self.plan(controller, 1440, lag=34., speed=2.5)
        self.assertEqual(reason, 'gap_creation_recovering')
        self.assertEqual(controller._gap_state, 'recovering')
        reason, _, _ = self.plan(controller, 1680, lag=34., speed=10.8)
        self.assertEqual(reason, 'safe_adjacent_lane_selected')
        self.assertTrue(controller._maneuver)
        self.assertIsNone(controller._gap_attempt)
        self.assertEqual(controller._gap_reason, 'ordinary_maneuver_accepted')
        self.assertEqual(controller.config.target_speed_m_s, 15.6464)

    def test_slow_budget_releases_even_if_lag_never_accumulates(self):
        controller, _, _ = self.started()
        self.plan(controller, 240 + 11 * 120)
        self.assertEqual(controller._gap_state, 'recovering')
        self.assertGreater(controller._target_speed, 6.)

    def test_config_ramp_and_membership_changes_release_to_guarded_recovery(self):
        for mutation in ('config', 'ramp', 'pending', 'lifecycle'):
            with self.subTest(mutation=mutation):
                controller, _, _ = self.started()
                if mutation == 'config':
                    controller.config = replace(controller.config, desired_time_gap_s=2.4)
                elif mutation == 'ramp':controller.changing_traffic_speed = True
                elif mutation == 'pending':controller.traffic_transition_pending = True
                else:controller._gap_lifecycle += 1
                self.plan(controller, 252)
                self.assertEqual(controller._gap_state, 'recovering')
                self.assertEqual(controller._gap_reason, 'controls_or_membership_changed')
                self.assertIsNotNone(controller._gap_attempt)

    def test_new_close_rear_terminates_experiment_instead_of_adding_brake(self):
        controller, _, _ = self.started()
        state, tracks, station = self.frame(252)
        angle = (station - 160) / 500
        tracks.append(ObjectTrack('episode', 'newrear', 252, 507.4 * math.cos(angle),
            507.4 * math.sin(angle), angle + math.pi / 2, 6.8, width_m=1.9))
        controller.accept_lifecycle(tracks, [dict(type='enter', vehicle_id='newrear')])
        controller._unwrapped_s = station
        with self.assertRaisesRegex(RuntimeError, 'rear_window'):
            controller._plan(state, 252, tracks)
        self.assertEqual(controller._gap_state, 'failed')

    def test_loss_of_strict_tracking_certificate_terminates(self):
        controller, _, _ = self.started()
        state, tracks, station = self.frame(252)
        state = replace(state, yaw_rad=state.yaw_rad + .05)
        controller._unwrapped_s = station
        with self.assertRaisesRegex(RuntimeError, 'straight_lane_tracking'):
            controller._plan(state, 252, tracks)

    def test_strict_guard_is_not_bypassed_by_newly_accepted_maneuver(self):
        controller, _, _ = self.started()
        state, tracks, station = self.frame(1680, lag=34., speed=10.8)
        state = replace(state, yaw_rad=state.yaw_rad + .05)
        controller._unwrapped_s = station
        with self.assertRaisesRegex(RuntimeError, 'straight_lane_tracking'):
            controller._plan(state, 1680, tracks)
        self.assertTrue(controller._maneuver)  # Ordinary planner accepted; handoff still rejected.
        self.assertEqual(controller._gap_state, 'failed')

    def test_lag_uses_observed_lead_progress_and_removed_lead_bound(self):
        controller, _, _ = self.started()
        state, tracks, station = self.frame(360)
        controller._unwrapped_s = station
        lead = tracks[5]
        angle = math.atan2(lead.y_m, lead.x_m) + 5. / 500
        tracks[5] = replace(lead, x_m=507.4 * math.cos(angle), y_m=507.4 * math.sin(angle))
        _, lag = controller._gap_guard(state, 360, tracks)
        self.assertAlmostEqual(lag, 5.)
        state, tracks, station = self.frame(372)
        controller._unwrapped_s = station
        tracks = [track for track in tracks if track.vehicle_id != lead.vehicle_id]
        _, removed_lag = controller._gap_guard(state, 372, tracks)
        self.assertGreater(removed_lag, lag + 1.)
        self.assertEqual(controller._gap_last['lag_reference'], 'removed_lead_maximum_speed_bound')

    def test_geometry_configuration_change_cannot_shrink_the_certificate(self):
        controller, _, _ = self.started()
        controller.config = replace(controller.config, max_observed_speed_m_s=19.)
        with self.assertRaisesRegex(RuntimeError, 'configuration_contract_changed'):
            self.plan(controller, 252)

    def test_time_lag_and_attempt_limits(self):
        controller, _, _ = self.started()
        with self.assertRaisesRegex(RuntimeError, 'added_lag_budget'):
            self.plan(controller, 1440, lag=51., speed=2.5)
        controller, _, _ = self.started()
        self.plan(controller, 1560, lag=34., speed=2.5)
        with self.assertRaisesRegex(RuntimeError, 'recovery_deadline'):
            self.plan(controller, 2760, lag=34., speed=2.5)
        controller, _, _ = self.started()
        self.plan(controller, 1560)
        self.plan(controller, 2760)
        self.assertIsNone(controller._gap_attempt)
        self.assertEqual(controller._gap_reason, 'ordinary_following_recovered')
        controller = self.make()
        controller._gap_attempts = 3
        self.plan(controller)
        self.plan(controller, 240)
        self.assertEqual(controller._gap_reason, 'attempt_budget')

    def test_seam_rotation_preserves_certificate_and_opening(self):
        results = []
        for rotation in (0., math.pi * 500 - 1, math.tau * 500 - 1):
            controller = self.make()
            state, tracks, station = self.frame(rotation=rotation)
            controller._unwrapped_s = station
            results.append((controller._gap_geometry(state, tracks, 21), controller._gap_opening(state, tracks)))
        for rear, opening in results:
            self.assertAlmostEqual(rear['minimum_rear_reserve_m'], results[0][0]['minimum_rear_reserve_m'])
            self.assertEqual(opening, results[0][1])

    def command_started(self):
        controller = self.make()
        for tick in range(0, 241, 2):
            state, tracks, _ = self.frame(tick)
            command = controller.command(state, tick, tracks)
            self.assertFalse(controller.diagnostics['is_fallback'])
        self.assertEqual(controller._gap_state, 'slowing')
        return controller, command

    def test_native_clock_only_and_base_command_identity_and_expiry(self):
        controller, command = self.command_started()
        before = dict(controller._gap_attempt)
        self.assertEqual(command.issued_tick, 240)
        self.assertEqual(command.expires_tick, 252)
        self.assertEqual(controller._gap_attempt, before)  # No wall/UI update API exists.
        with self.assertRaises(ValueError):
            state, tracks, _ = self.frame(240)
            controller.command(state, 240, tracks)
        self.assertEqual(controller._gap_attempt, before)
        with self.assertRaises(ValueError):
            controller.command(state, None, tracks)

    def test_failure_certificate_is_finite_json_preserved_in_error_repr(self):
        controller, _, _ = self.started()
        with self.assertRaisesRegex(RuntimeError, 'added_lag_budget') as raised:
            self.plan(controller, 1440, lag=51., speed=2.5)
        message = str(raised.exception)
        certificate = json.loads(message.split('; certificate=', 1)[1])
        self.assertEqual(certificate['state'], 'failed')
        self.assertEqual(certificate['reason'], 'added_lag_budget')
        self.assertEqual(certificate['start_tick'], 240)
        self.assertEqual(certificate['deadline_tick'], 2760)
        self.assertAlmostEqual(certificate['added_lag_m'], 51.)
        # repr(RuntimeError(...)) is the existing runner's persisted error path.
        restored_message = ast.literal_eval(repr(raised.exception)[len('RuntimeError('):-1])
        self.assertEqual(json.loads(restored_message.split('; certificate=', 1)[1]), certificate)

    def test_invalid_diagnostics_cannot_mask_original_safety_failure(self):
        for bad in (math.nan, math.inf, object()):
            with self.subTest(bad=repr(bad)):
                controller, _, _ = self.started()
                controller._gap_last['bad'] = bad
                with self.assertRaisesRegex(RuntimeError, 'rear_window') as raised:
                    controller._gap_failure('rear_window', 252)
                certificate = json.loads(str(raised.exception).split('; certificate=', 1)[1])
                self.assertEqual(certificate['reason'], 'rear_window')
                self.assertEqual(certificate['diagnostic_error'], 'nonfinite_or_unserializable_certificate')

    def test_every_command_rechecks_rear_and_settings_not_only_plan_ticks(self):
        controller, _ = self.command_started()
        controller.config = replace(controller.config, desired_time_gap_s=2.0)
        state, tracks, _ = self.frame(242)
        controller.command(state, 242, tracks)
        self.assertEqual(controller._gap_state, 'recovering')
        self.assertEqual(controller._last_planning_tick, 242)
        with patch.object(controller, '_gap_geometry', return_value=dict(eligible=False, reason='rear_window', minimum_rear_reserve_m=-1)):
            state, tracks, _ = self.frame(244)
            with self.assertRaisesRegex(RuntimeError, 'rear_window'):
                controller.command(state, 244, tracks)

    def test_base_invalid_stop_and_emergency_dominate_extra_cap(self):
        for failure in ('stale', 'nan', 'stop', 'tracking', 'close'):
            with self.subTest(failure=failure):
                controller, _ = self.command_started()
                state, tracks, station = self.frame(242)
                if failure == 'stale':tracks[0] = replace(tracks[0], tick=240)
                if failure == 'nan':state = replace(state, speed_m_s=math.nan)
                if failure == 'tracking':state = replace(state, yaw_rad=state.yaw_rad + .6)
                if failure == 'close':
                    controller._previous_tracks = None
                    tracks[5] = replace(tracks[5], x_m=state.x_m, y_m=state.y_m, yaw_rad=state.yaw_rad)
                if failure == 'stop':
                    command = controller.command(state, 242, tracks, stop=True)
                    self.assertEqual(command.throttle, 0.)
                    self.assertEqual(command.brake, 1.)
                    self.assertIsNone(controller._gap_attempt)
                    self.assertEqual(controller._gap_reason, 'coordinated_requested_stop')
                else:
                    with self.assertRaisesRegex(RuntimeError, 'base_stop:'):
                        controller.command(state, 242, tracks)
                    self.assertTrue(controller._stop_latched)
                    self.assertEqual(controller._gap_state, 'failed')

    def test_wider_base_tracking_emergency_at_handoff_and_deadline_keeps_abort(self):
        for deadline in (False, True):
            with self.subTest(deadline=deadline):
                controller, _ = self.command_started()
                tick = 2760 if deadline else 1680
                state, tracks, station = self.frame(tick, lag=0. if deadline else 34.,
                                                   speed=None if deadline else 10.8)
                # Isolate the later base tracking gate from the unrelated missing-
                # clock guard; the source trajectory is a counterfactual CPU fixture.
                controller._last_state = None
                controller._previous_tracks = None
                controller._last_tick = tick - 2
                controller._last_planning_tick = tick - 12
                controller.progress.update(station)
                controller._unwrapped_s = station
                state = replace(state, yaw_rad=state.yaw_rad + .6)
                with self.assertRaisesRegex(RuntimeError, 'base_stop:path_tracking_envelope'):
                    controller.command(state, tick, tracks)
                self.assertEqual(controller._gap_state, 'failed')

    def test_recovery_requires_matching_current_rear_speed(self):
        controller, _, _ = self.started()
        self.plan(controller, 1560)
        state, tracks, station = self.frame(2760)
        tracks[4] = replace(tracks[4], speed_m_s=9.)
        controller._unwrapped_s = station
        with self.assertRaisesRegex(RuntimeError, 'recovery_deadline'):
            controller._plan(state, 2760, tracks)


if __name__ == '__main__':
    unittest.main()
