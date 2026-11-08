"""Flow reference boundary checks; no SUMO connection or optimizer."""
import json
import unittest

import numpy as np

from traffic import highway_stage1_teacher as legacy
from traffic.highway_flow_reference import (
    FIELD_INDEX, OBSERVATION_SIZE, PLANNING_HORIZON_S,
    _gap_plan, reference_details, teacher_action, teacher_actions,
)


def observation(speed=15., acceleration=0., limit=18., hard=2.):
    obs = np.zeros(OBSERVATION_SIZE, dtype=np.float64)
    obs[0], obs[1], obs[4] = speed / legacy.SPEED_CAP, acceleration / 4.5, limit / legacy.SPEED_CAP
    obs[2], obs[26], obs[27], obs[31], obs[43], obs[44], obs[50] = 1/7., 3/7., 1., 1., hard/4.5, limit/legacy.SPEED_CAP, 1.
    for name in ('own', 'right', 'left'):
        obs[FIELD_INDEX[name + '_ACC_over4p5']] = 2. / 4.5
        obs[FIELD_INDEX[name + '_legal_speed_overcap']] = limit / legacy.SPEED_CAP
    obs[FIELD_INDEX['occupied_worst_ACC_over4p5']] = 2. / 4.5
    corridor(obs, 'own', reachable=True, distance=1000.)
    return obs


def corridor(obs, name, *, reachable, distance):
    obs[FIELD_INDEX[name + '_route_reachable']] = float(reachable)
    obs[FIELD_INDEX[name + '_route_end_distance_over1000']] = distance / 1000.


def neighbor(obs, start, gap, speed):
    obs[start:start + 3] = [gap / 120., (speed - obs[0] * legacy.SPEED_CAP) / legacy.SPEED_CAP, 1.]


def merge(obs, *, distance=200., delta=1):
    obs[28], obs[33], obs[35], obs[36] = 1., 1., float(delta), 0.
    corridor(obs, 'own', reachable=False, distance=distance)
    if delta:
        corridor(obs, 'left' if delta == 1 else 'right', reachable=True, distance=1000.)
        obs[delta + 31] = 1.
    return obs


class FlowReferenceTests(unittest.TestCase):
    def test_named_layout_matches_environment_contract(self):
        from traffic.highway_flow_context import FIELD_INDEX as actual
        self.assertEqual(FIELD_INDEX, actual)

    def test_native_float32_observations_produce_finite_json_journal_details(self):
        cases = [observation(), merge(observation()), merge(observation(), delta=0),
                 merge(observation(speed=12.), distance=10.)]
        cases[2][36], cases[2][57] = 40. / 240., 100. / 200.
        neighbor(cases[2], 37, 100., 15.)
        neighbor(cases[2], 40, 100., 15.)
        cases[3][33] = 0.
        for obs in cases:
            details = reference_details(obs.astype(np.float32))
            restored = json.loads(json.dumps(details, allow_nan=False))
            self.assertEqual(details, restored)
            self.assertIs(type(details['acceleration_supervision_eligible']), bool)

    def test_abi_rejects_old_width_nonfinite_and_out_of_box(self):
        for bad in (np.zeros(99), np.full(117, np.nan), np.full(117, 1.01)):
            with self.subTest(shape=bad.shape), self.assertRaises(ValueError):
                reference_details(bad)

    def test_clear_cruise_accelerates_with_measured_slew(self):
        d = reference_details(observation())
        self.assertAlmostEqual(d['acceleration_m_s2'], .2)
        self.assertEqual(d['lane_change_delta'], 0)
        self.assertTrue(d['acceleration_supervision_eligible'])

    def test_at_limit_does_not_target_negative_speed_or_excess(self):
        d = reference_details(observation(speed=18.))
        self.assertAlmostEqual(d['acceleration_m_s2'], -.04)
        stopped = reference_details(observation(speed=0., hard=-4.5))
        self.assertEqual(stopped['acceleration_m_s2'], 0.)

    def test_hard_guard_immediately_overrides_comfort_and_label_is_flagged(self):
        d = reference_details(observation(hard=-4.5))
        self.assertEqual(d['acceleration_m_s2'], -4.5)
        self.assertTrue(d['braking_override_soft_slew'])
        self.assertFalse(d['acceleration_supervision_eligible'])
        self.assertIn('mandatory_braking_overrides_comfort_target', d['supervision_ineligibility_reasons'])

    def test_through_car_does_not_pass_into_ending_auxiliary(self):
        obs = observation()
        neighbor(obs, 6, 55., 11.)
        corridor(obs, 'right', reachable=False, distance=180.)
        obs[30] = 1.
        d = reference_details(obs)
        self.assertEqual(d['lane_change_delta'], 0)
        self.assertTrue(any(r['cause'] == 'passing_into_ending_lane_has_no_route_benefit' for r in d['rejected_lanes']))
        self.assertTrue(d['lane_supervision_eligible'])

    def test_route_continuing_left_lane_can_be_used_for_real_pass(self):
        obs = observation(speed=14.)
        neighbor(obs, 6, 35., 12.)
        corridor(obs, 'left', reachable=True, distance=1000.)
        obs[32] = 1.
        d = reference_details(obs)
        self.assertEqual(d['lane_change_delta'], 1)
        self.assertTrue(d['selected_gap_plan']['feasible'])

    def test_empty_lanes_do_not_trigger_weaving(self):
        obs = observation()
        for name in ('left', 'right'):
            corridor(obs, name, reachable=True, distance=1000.)
        obs[30:33] = 1.
        self.assertEqual(reference_details(obs)['lane_change_delta'], 0)

    def test_feasible_highway_speed_merge_has_no_mandatory_slowdown(self):
        obs = merge(observation(speed=18.), distance=180.)
        d = reference_details(obs)
        self.assertEqual(d['lane_change_delta'], 1)
        self.assertGreaterEqual(d['acceleration_m_s2'], -.0400001)
        self.assertFalse(d['ending_lane']['active'])
        self.assertNotIn('ending_profile', d['soft_acceleration_components_m_s2'])

    def test_mask_cannot_be_overridden_by_predicted_gap(self):
        obs = merge(observation())
        obs[32] = 0.
        d = reference_details(obs)
        self.assertTrue(d['selected_gap_plan']['feasible'])
        self.assertEqual(d['lane_change_delta'], 0)

    def test_actual_maneuver_lock_disallows_new_lane_intent(self):
        obs = merge(observation())
        obs[47], obs[48] = 1., 1.
        d = reference_details(obs)
        self.assertEqual(d['lane_change_delta'], 0)
        self.assertEqual(d['considered_gap_plans'], [])

    def test_ending_profile_derivative_is_inactive_above_speed_target(self):
        obs = merge(observation(speed=12.), distance=240.)
        obs[33] = 0.  # No known exit; target remains well above legal speed.
        d = reference_details(obs)
        self.assertGreater(d['ending_lane']['profile_speed_m_s'], d['legal_limit_m_s'])
        self.assertFalse(d['ending_lane']['active'])
        self.assertGreater(d['acceleration_m_s2'], 0.)

    def test_binding_ending_profile_brakes_when_no_exit_exists(self):
        obs = merge(observation(speed=12.), distance=10.)
        obs[33] = 0.
        d = reference_details(obs)
        self.assertTrue(d['ending_lane']['active'])
        self.assertLess(d['acceleration_m_s2'], 0.)

    def test_destination_slower_farther_leader_prevents_immediate_lane_entry(self):
        obs = merge(observation(speed=17.9), distance=220.)
        neighbor(obs, 6, 32.478, 18.)
        neighbor(obs, 37, 84.698, 12.)
        obs[FIELD_INDEX['left_ACC_over4p5']] = -2.855 / 4.5
        d = reference_details(obs)
        self.assertEqual(d['lane_change_delta'], 0)
        self.assertLess(d['acceleration_m_s2'], 0.)
        if d['selected_gap_plan']:
            self.assertGreater(d['selected_gap_plan']['entry_after_s'], 0.)

    def test_occupied_worst_estimate_is_not_replaced_by_nearest_front(self):
        obs = observation(speed=17.9)
        neighbor(obs, 6, 32.478, 18.)
        obs[FIELD_INDEX['occupied_worst_ACC_over4p5']] = -2.855 / 4.5
        d = reference_details(obs)
        self.assertEqual(d['ordinary_response_estimate_m_s2'], -2.855)
        self.assertAlmostEqual(d['acceleration_m_s2'], -.2)
        self.assertEqual(d['hard_ceiling_m_s2'], 2.)

    def test_full_maneuver_prediction_rejects_closing_rear_that_is_safe_now(self):
        plan = _gap_plan(speed=12., measured=0., limit=12., free_acceleration=0.,
            front=None, rear=(45., 20.), own_front=None,
            own_runway=500., destination_runway=500., destination_response=2.)
        self.assertGreater(45., legacy.required_gap(20., 12.))
        self.assertFalse(plan['feasible'])

    def test_joint_planner_can_select_acceleration_to_preserve_rear_gap(self):
        plan = _gap_plan(speed=12., measured=0., limit=18., free_acceleration=2.,
            front=None, rear=(40., 16.), own_front=None,
            own_runway=500., destination_runway=500., destination_response=2.)
        self.assertTrue(plan['feasible'])
        self.assertGreater(plan['acceleration_m_s2'], 0.)
        self.assertEqual(plan['horizon_s'], PLANNING_HORIZON_S)

    def test_runway_must_fit_entire_maneuver_not_only_current_gap(self):
        plan = _gap_plan(speed=18., measured=0., limit=18., free_acceleration=0.,
            front=None, rear=None, own_front=None,
            own_runway=20., destination_runway=500., destination_response=2.)
        self.assertFalse(plan['feasible'])

    def test_cooldown_is_included_before_complete_maneuver_runway(self):
        obs = merge(observation(speed=18.), distance=60.)
        obs[29], obs[32] = 3. / 5., 0.
        d = reference_details(obs)
        self.assertIsNone(d['selected_gap_plan'])
        self.assertEqual(d['lane_change_delta'], 0)

    def test_equal_safe_passing_choices_use_deterministic_left_tie(self):
        obs = observation(speed=14.)
        neighbor(obs, 6, 35., 12.)
        for name in ('left', 'right'):
            corridor(obs, name, reachable=True, distance=1000.)
        obs[30:33] = 1.
        self.assertEqual(reference_details(obs)['lane_change_delta'], 1)

    def test_destination_lower_limit_is_checked_before_entry(self):
        obs = merge(observation(speed=18.))
        obs[FIELD_INDEX['left_legal_speed_overcap']] = 10. / legacy.SPEED_CAP
        self.assertEqual(reference_details(obs)['lane_change_delta'], 0)

    def test_visibility_is_explicit_map_input_not_fixed_four_point_five(self):
        obs = merge(observation(speed=12., limit=13.41), delta=0)
        obs[36] = 40. / 240.
        obs[57] = 100. / 200.
        visible = reference_details(obs)
        self.assertEqual(visible['entry']['visibility_m'], 100.)
        self.assertIsNone(visible['entry']['mandatory_stop_estimate_m_s2'])
        self.assertGreaterEqual(visible['acceleration_m_s2'], 0.)
        obs[57] = 4.5 / 200.
        limited = reference_details(obs)
        self.assertIsNotNone(limited['entry']['mandatory_stop_estimate_m_s2'])
        self.assertIn('entry_distance', limited['soft_acceleration_components_m_s2'])

    def test_visible_but_blocked_entry_preserves_stop_priority(self):
        obs = merge(observation(speed=10., hard=-4.5), delta=0)
        obs[36], obs[57] = 4. / 240., 100. / 200.
        neighbor(obs, 37, 3., 0.)
        d = reference_details(obs)
        self.assertFalse(d['entry']['gaps_safe'])
        self.assertEqual(d['acceleration_m_s2'], -4.5)
        self.assertFalse(d['acceleration_supervision_eligible'])

    def test_no_yield_projection_does_not_invent_stop_rule_or_lane_permission(self):
        obs = merge(observation(speed=16.), delta=0, distance=240.)
        obs[36] = 20. / 240.
        d = reference_details(obs)
        self.assertFalse(d['entry']['active'])
        self.assertEqual(d['entry']['visibility_m'], 0.)
        self.assertEqual(d['mandatory_components_m_s2'], {})
        self.assertNotIn('entry_distance', d['soft_acceleration_components_m_s2'])
        self.assertGreater(d['acceleration_m_s2'], 0.)
        self.assertEqual(d['lane_change_delta'], 0)
        self.assertFalse(d['ending_lane']['active'])

    def test_future_messages_and_absent_neighbors_do_not_change_planning(self):
        obs = merge(observation())
        baseline = reference_details(obs)
        obs[69:] = .5  # Present state-only implementation does not consume messages.
        obs[37:39] = [.001, -1.]  # Presence remains zero: ghost values ignored.
        self.assertEqual(baseline, reference_details(obs))

    def test_deterministic_pure_action_batch_preserves_input(self):
        obs = merge(observation())
        saved = obs.copy()
        first = teacher_action(obs)
        second = teacher_actions({'agent_0': obs})['agent_0']
        np.testing.assert_array_equal(first, second)
        np.testing.assert_array_equal(obs, saved)
        self.assertEqual(first.dtype, np.float32)
        self.assertTrue(np.all(np.abs(first) <= 1.))


if __name__ == '__main__':
    unittest.main()
