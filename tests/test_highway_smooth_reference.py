"""Current-observation reference boundaries; no simulator or policy updates."""
import unittest

import numpy as np

from traffic import highway_stage1_teacher as legacy
from traffic.highway_smooth_reference import (
    reference_details, smooth_acceleration_target, teacher_action, teacher_actions,
)


def observation(speed=12., acceleration=0., limit=18., hard_ceiling=2.):
    obs = np.zeros(99,dtype=np.float64)
    obs[0],obs[1],obs[4] = speed/legacy.SPEED_CAP, acceleration/4.5, limit/legacy.SPEED_CAP
    obs[27],obs[31],obs[43],obs[44] = 1.,1.,hard_ceiling/4.5,limit/legacy.SPEED_CAP
    return obs


def neighbor(obs,start,gap,speed):
    obs[start:start+3] = [gap/120.,(speed-obs[0]*legacy.SPEED_CAP)/legacy.SPEED_CAP,1.]


def minor_entry(obs,distance):
    obs[28],obs[33],obs[35],obs[36] = 1.,1.,0.,distance/240.
    return obs


class SmoothReferenceTests(unittest.TestCase):
    def test_cruise_start_uses_measured_acceleration_slew(self):
        details = reference_details(observation())
        self.assertAlmostEqual(details['desired_acceleration_m_s2'],2.)
        self.assertAlmostEqual(details['acceleration_m_s2'],.2)
        self.assertFalse(details['braking_override_soft_slew'])
        self.assertEqual(details['hard_ceiling_m_s2'],2.)

    def test_visibility_release_recovers_from_actual_braking(self):
        obs = minor_entry(observation(speed=6.,acceleration=-4.5,limit=13.41),4.4)
        details = reference_details(obs)
        self.assertFalse(details['entry']['native_stop_estimate_active'])
        self.assertAlmostEqual(details['acceleration_m_s2'],-4.3)
        self.assertAlmostEqual(details['reference_slew_m_s3'],2.)

    def test_hard_braking_overrides_soft_slew_immediately(self):
        details = reference_details(observation(speed=15.,hard_ceiling=-3.))
        self.assertAlmostEqual(details['acceleration_m_s2'],-3.)
        self.assertTrue(details['braking_override_soft_slew'])
        self.assertTrue(details['reference_ceiling_limited'])

    def test_ordinary_following_estimate_is_not_relabelled_hard_guard(self):
        obs = observation(speed=17.)
        neighbor(obs,6,30.,16.)
        details = reference_details(obs)
        self.assertAlmostEqual(details['native_response_estimate_m_s2'],-.72)
        self.assertEqual(details['hard_ceiling_m_s2'],2.)
        self.assertEqual(details['reference_ceiling_m_s2'],2.)
        self.assertAlmostEqual(details['acceleration_m_s2'],-.2)
        self.assertFalse(details['braking_override_soft_slew'])
        self.assertTrue(details['reference_exceeds_ordinary_response_estimate'])

    def test_ordinary_occupied_lane_switch_is_smoothed_but_estimate_is_retained(self):
        # Saved cruise31.5s: a neighboring car starts lateral motion. The
        # conservative own-leader observation changes before a hard violation.
        obs = observation(speed=17.899989236068723,acceleration=.00000428408387165,
            hard_ceiling=1.000104010105133)
        neighbor(obs,6,44.1271,14.416)
        details = reference_details(obs)
        self.assertLess(details['ordinary_native_response_estimate_m_s2'],-2.)
        self.assertAlmostEqual(details['acceleration_m_s2'],-.19999571591612835)
        self.assertEqual(details['mandatory_native_response_estimate_m_s2'],2.)
        self.assertTrue(details['reference_exceeds_ordinary_response_estimate'])
        self.assertFalse(details['braking_override_soft_slew'])
        # The same observed slowdown with a mandatory bound is not smoothed.
        obs[43] = -4.5/4.5
        mandatory = reference_details(obs)
        self.assertEqual(mandatory['acceleration_m_s2'],-4.5)
        self.assertTrue(mandatory['braking_override_soft_slew'])

    def test_stopped_follower_never_gets_negative_speed_label(self):
        obs = observation(speed=0.)
        neighbor(obs,6,2.5,0.)
        details = reference_details(obs)
        self.assertLess(details['raw_native_response_estimate_m_s2'],0.)
        self.assertEqual(details['native_response_estimate_m_s2'],0.)
        self.assertEqual(details['acceleration_m_s2'],0.)
        self.assertTrue(details['native_estimate_velocity_floor_applied'])

    def test_almost_stopped_braking_ends_at_zero_not_negative_speed(self):
        details = reference_details(observation(speed=.02,acceleration=-4.5))
        self.assertAlmostEqual(details['acceleration_m_s2'],-.2)
        self.assertAlmostEqual(details['speed_m_s']+.1*details['acceleration_m_s2'],0.)
        self.assertTrue(details['reference_velocity_floor_applied'])

    def test_unreachable_negative_hard_bound_at_rest_keeps_stopped(self):
        details = reference_details(observation(speed=0.,hard_ceiling=-4.5))
        self.assertEqual(details['hard_ceiling_m_s2'],-4.5)
        self.assertEqual(details['acceleration_m_s2'],0.)
        self.assertTrue(details['hard_ceiling_velocity_floor_applied'])

    def test_preview_uses_current_distance_and_two_seconds(self):
        details = reference_details(minor_entry(observation(speed=18.),60.))
        self.assertAlmostEqual(details['entry']['preview_distance_m'],24.)
        self.assertAlmostEqual(details['entry']['preview_speed_m_s'],legacy.discrete_stop_speed(23.9))
        self.assertEqual(details['entry']['preview_floor_m'],4.5)
        self.assertTrue(details['entry']['native_stop_estimate_active'])

    def test_unsafe_projected_front_uses_stop_offset_and_keeps_braking(self):
        obs = minor_entry(observation(speed=10.,limit=13.41),4.4)
        neighbor(obs,37,5.,0.)
        details = reference_details(obs)
        self.assertFalse(details['entry']['projected_front_safe'])
        self.assertEqual(details['entry']['preview_floor_m'],.1)
        self.assertEqual(details['entry']['preview_speed_m_s'],0.)
        self.assertTrue(details['entry']['native_stop_estimate_active'])
        self.assertEqual(details['acceleration_m_s2'],-4.5)
        self.assertTrue(details['braking_override_soft_slew'])

    def test_unsafe_projected_rear_does_not_grant_entry(self):
        obs = minor_entry(observation(speed=10.,limit=13.41),4.4)
        neighbor(obs,40,5.,18.)
        details = reference_details(obs)
        self.assertFalse(details['entry']['projected_rear_safe'])
        self.assertEqual(details['entry']['preview_floor_m'],.1)
        self.assertTrue(details['entry']['native_stop_estimate_active'])

    def test_native_visibility_boundary_is_unchanged(self):
        before = reference_details(minor_entry(observation(speed=10.),4.51))
        inside = reference_details(minor_entry(observation(speed=10.),4.49))
        self.assertTrue(before['entry']['native_stop_estimate_active'])
        self.assertFalse(inside['entry']['native_stop_estimate_active'])
        self.assertEqual(before['entry']['visibility_m'],4.5)
        # The estimate may still demand an abrupt brake: reference smoothing
        # must not silently discard that native-response limitation.
        self.assertTrue(before['braking_override_soft_slew'])

    def test_unknown_context_does_not_invent_entry_permission_or_preview(self):
        obs = minor_entry(observation(),10.)
        obs[33] = 0.
        details = reference_details(obs)
        self.assertFalse(details['entry']['active'])
        self.assertNotIn('entry_preview',details['soft_speed_components_m_s'])

    def test_auxiliary_lateral_merge_is_not_a_minor_entry(self):
        obs = minor_entry(observation(),10.)
        obs[35] = 1.
        details = reference_details(obs)
        self.assertFalse(details['entry']['active'])
        self.assertIn('target_lane_following',details['soft_speed_components_m_s'])

    def test_reference_respects_legal_and_ending_lane_targets(self):
        obs = observation(speed=12.,limit=13.41)
        obs[28],obs[27] = 1.,10./240.
        details = reference_details(obs)
        self.assertLessEqual(details['desired_speed_m_s'],13.31)
        self.assertAlmostEqual(details['desired_speed_m_s'],np.sqrt(4.*(7.5-1./12.)))
        self.assertLess(details['ending_lane']['feed_forward_m_s2'],0.)

    def test_entry_distance_braking_begins_before_native_stop_bound(self):
        # The v1 native fixture began here and accelerated. Its later speed-error
        # tracking reached the visibility boundary too fast, requiring -4.5.
        obs = minor_entry(observation(speed=8.612114,limit=13.41),38.934044)
        details = reference_details(obs)
        terminal = legacy.discrete_stop_speed(4.4)
        self.assertAlmostEqual(details['entry']['distance_braking_m_s2'],
            (terminal**2-8.612114**2)/(2.*(38.934044-4.5)))
        self.assertAlmostEqual(details['acceleration_m_s2'],-.2)
        self.assertGreater(details['native_response_components_m_s2']['minor_entry_stop'],2.)

    def test_late_braking_state_still_obeys_native_estimate(self):
        # Actual failed v1 state: the revision cannot undo an already-late
        # approach by suppressing the mandatory response or its measured jerk.
        obs = minor_entry(observation(speed=7.309395194435119,
            acceleration=-.5176052525639534,limit=13.41),5.79336404800415)
        details = reference_details(obs)
        self.assertAlmostEqual(details['acceleration_m_s2'],-3.766676644325253)
        self.assertTrue(details['braking_override_soft_slew'])

    def test_safe_entry_scalar_approach_has_no_late_full_brake(self):
        # Deterministic kinematic boundary check, not a replacement for SUMO.
        speed,acceleration,distance = 8.612114,0.,38.934044
        peak_jerk = 0.
        for _ in range(160):
            obs = minor_entry(observation(speed=speed,acceleration=acceleration,
                limit=13.41),max(0.,distance))
            details = reference_details(obs)
            next_acceleration = details['acceleration_m_s2']
            peak_jerk = max(peak_jerk,abs(next_acceleration-acceleration)/.1)
            speed += .1*next_acceleration
            distance -= .1*speed
            acceleration = next_acceleration
            if distance <= 0.:
                break
        self.assertLessEqual(distance,0.)
        self.assertLessEqual(peak_jerk,2.+1e-8)
        self.assertGreater(speed,5.9)

    def test_ending_lane_scalar_stop_recovers_without_velocity_floor_jump(self):
        for initial_speed,initial_distance in ((14.28,240.),(18.,120.),(8.,40.)):
            with self.subTest(speed=initial_speed,distance=initial_distance):
                speed,acceleration,distance = initial_speed,0.,initial_distance
                peak_jerk = 0.
                for _ in range(400):
                    # The unchanged analytical hard lane-end ceiling remains
                    # active at every step, including the terminal boundary.
                    hard = np.clip((np.sqrt(9.*max(0.,distance-2.5))-speed)/.1,-4.5,2.)
                    obs = observation(speed=speed,acceleration=acceleration,hard_ceiling=hard)
                    obs[28],obs[27] = 1.,max(0.,distance)/240.
                    details = reference_details(obs)
                    next_acceleration = details['acceleration_m_s2']
                    peak_jerk = max(peak_jerk,abs(next_acceleration-acceleration)/.1)
                    speed = max(0.,speed+.1*next_acceleration)
                    distance -= .1*speed
                    acceleration = next_acceleration
                    if speed < .001 and distance < 2.6:
                        break
                self.assertLess(speed,.001)
                self.assertGreaterEqual(distance,2.5)
                self.assertLessEqual(distance,2.6)
                self.assertLessEqual(peak_jerk,2.+1e-8)

    def test_unsafe_entry_and_stopped_endpoint_keep_safety_priority(self):
        obs = minor_entry(observation(speed=2.,acceleration=0.,hard_ceiling=-4.5),3.)
        neighbor(obs,37,2.5,0.)
        details = reference_details(obs)
        self.assertGreater(details['soft_braking_recovery_floor_m_s2'],-4.5)
        self.assertEqual(details['acceleration_m_s2'],-4.5)
        self.assertTrue(details['braking_override_soft_slew'])
        stopped = observation(speed=0.,hard_ceiling=0.)
        stopped[28],stopped[27] = 1.,2.5/240.
        self.assertEqual(reference_details(stopped)['acceleration_m_s2'],0.)

    def test_closing_rear_with_insufficient_front_room_yields_instead_of_pacing(self):
        # Probe02 paced a ~29 m front gap while a faster rear car was alongside.
        obs = minor_entry(observation(speed=15.77,acceleration=.01),0.)
        obs[35] = 1.
        neighbor(obs,37,29.44,15.67)
        neighbor(obs,40,0.,15.875)
        details = reference_details(obs)
        slot = details['merge_slot_alignment']
        self.assertTrue(slot['active'])
        self.assertGreater(slot['rear_deficit_m'],slot['front_surplus_m'])
        self.assertAlmostEqual(slot['speed_target_m_s'],14.875)
        self.assertAlmostEqual(details['acceleration_m_s2'],-.19)
        self.assertEqual(teacher_action(obs)[1],0.)

    def test_early_slot_yield_holds_speed_when_rear_is_already_passing(self):
        obs = minor_entry(observation(speed=7.6),0.)
        obs[35] = 1.
        neighbor(obs,37,1.,15.02)
        neighbor(obs,40,29.,15.45)
        details = reference_details(obs)
        self.assertTrue(details['merge_slot_alignment']['active'])
        self.assertAlmostEqual(details['merge_slot_alignment']['speed_target_m_s'],7.6)
        self.assertAlmostEqual(details['acceleration_m_s2'],0.)

    def test_slot_yield_releases_after_rear_passes_into_front_context(self):
        obs = minor_entry(observation(speed=7.6),0.)
        obs[35] = 1.
        neighbor(obs,37,15.,15.45)
        neighbor(obs,40,70.,15.5)
        details = reference_details(obs)
        self.assertFalse(details['merge_slot_alignment']['active'])
        self.assertGreater(details['acceleration_m_s2'],0.)

    def test_slot_yield_does_not_suppress_recoverable_gap_or_nonclosing_rear(self):
        for front_gap,rear_speed in ((100.,16.),(29.,14.)):
            with self.subTest(front_gap=front_gap,rear_speed=rear_speed):
                obs = minor_entry(observation(speed=15.),0.)
                obs[35] = 1.
                neighbor(obs,37,front_gap,16.)
                neighbor(obs,40,5.,rear_speed)
                self.assertFalse(reference_details(obs)['merge_slot_alignment']['active'])

    def test_slot_yield_does_not_apply_to_minor_entry_or_weaken_hard_braking(self):
        obs = minor_entry(observation(speed=10.,hard_ceiling=-3.),20.)
        neighbor(obs,37,20.,15.)
        neighbor(obs,40,0.,16.)
        self.assertFalse(reference_details(obs)['merge_slot_alignment']['active'])
        obs[35] = 1.
        details = reference_details(obs)
        self.assertTrue(details['merge_slot_alignment']['active'])
        self.assertEqual(details['acceleration_m_s2'],-3.)
        self.assertTrue(details['braking_override_soft_slew'])

    def test_lane_actions_match_unchanged_legacy_v3_and_inputs_are_immutable(self):
        passing = observation(speed=12.)
        neighbor(passing,6,30.,10.)
        passing[32] = 1.
        merging = minor_entry(observation(),80.)
        merging[35],merging[32] = 1.,1.
        for obs in (passing,merging,observation()):
            original = obs.copy()
            old = legacy.teacher_action(np.concatenate((obs[:45],obs[51:])),version='v3')
            action = teacher_action(obs)
            self.assertEqual(action.dtype,np.float32)
            self.assertEqual(action.shape,(2,))
            self.assertEqual(action[1],old[1])
            self.assertTrue(np.all(np.abs(action)<=1.))
            self.assertAlmostEqual(-1.25+3.25*float(action[0]),smooth_acceleration_target(obs),places=6)
            np.testing.assert_array_equal(obs,original)
        self.assertEqual(teacher_action(passing)[1],1.)
        self.assertEqual(teacher_action(merging)[1],1.)

    def test_batch_retains_fixed_id_mapping_without_history(self):
        items={'agent_7':observation(speed=5.),'agent_0':observation(speed=15.)}
        first=teacher_actions(items)
        second=teacher_actions(dict(reversed(list(items.items()))))
        self.assertEqual(list(first),list(items))
        for identity in first:
            np.testing.assert_array_equal(first[identity],second[identity])

    def test_invalid_observations_fail_closed(self):
        bad_size=np.zeros(93)
        bad_nan=observation();bad_nan[10]=np.nan
        bad_range=observation();bad_range[0]=1.01
        for bad in (bad_size,bad_nan,bad_range):
            with self.subTest(shape=bad.shape),self.assertRaises(ValueError):
                reference_details(bad)


if __name__=='__main__':
    unittest.main()
