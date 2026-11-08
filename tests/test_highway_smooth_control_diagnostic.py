"""Offline attribution must preserve recorded accounting and unknown causes."""
import unittest

from scripts.diagnose_highway_smooth_control import Aggregate, analytical_cause, distribution


class SmoothControlDiagnosticTests(unittest.TestCase):
    def test_acc_gap_response_is_distinct_from_free_speed_cap(self):
        ego = dict(speed=17.,lane_remaining_m=100.)
        leader = dict(bumper_gap_m=50.,relative_speed_m_s=-1.)
        info = dict(realized_acceleration=.08,acceleration_bounds={'speed_ceiling':10.})
        cause, prediction = analytical_cause(ego,leader,info,yield_entry=False)
        self.assertEqual(cause,'acc_following_response_match')
        self.assertAlmostEqual(prediction,.08)
        self.assertNotAlmostEqual(prediction,.4*(18.-17.))

    def test_unknown_realization_remains_unresolved(self):
        info = dict(realized_acceleration=.333,acceleration_bounds={'speed_ceiling':10.})
        cause, prediction = analytical_cause(dict(speed=17.,lane_remaining_m=100.),
            dict(bumper_gap_m=50.,relative_speed_m_s=-1.),info,yield_entry=False)
        self.assertEqual(cause,'unresolved_native_difference')
        self.assertIsNone(prediction)

    def test_minor_approach_matches_saved_nonclamped_braking(self):
        ego = dict(speed=11.117663681366654,lane_remaining_m=13.871036272174877)
        info = dict(realized_acceleration=-2.096491724967038,
                    acceleration_bounds={'speed_ceiling':22.92336318633346})
        cause, _ = analytical_cause(ego,None,info,yield_entry=True)
        self.assertEqual(cause,'minor_approach_stopping_response_match')

    def test_aggregate_does_not_redefine_override_from_residual(self):
        aggregate = Aggregate()
        info = dict(override=False,policy_filter_changed=False,native_realization_changed=False,
            requested_acceleration=2.,filtered_acceleration=2.,realized_acceleration=0.,
            feasible_acceleration_target=0.,reward_terms={})
        aggregate.add(info,dict(speed=10.,acceleration=0.),cause='unresolved_native_difference')
        report = aggregate.report()
        self.assertEqual(report['counts']['decisions'],1)
        self.assertEqual(report['counts']['override'],0)
        self.assertEqual(report['residuals']['absolute_request_target_error_m_s2']['mean'],2.)

    def test_arrival_without_realization_keeps_decision_denominator(self):
        aggregate = Aggregate()
        info = dict(override=False,policy_filter_changed=False,native_realization_changed=False,
            requested_acceleration=.2,filtered_acceleration=.2,realized_acceleration=None,
            feasible_acceleration_target=.1,reward_terms={})
        aggregate.add(info,{},cause='no_realization_after_arrival')
        report = aggregate.report()
        self.assertEqual(report['counts']['decisions'],1)
        self.assertEqual(report['residuals']['absolute_request_target_error_m_s2']['samples'],1)
        self.assertNotIn('absolute_native_target_error_m_s2',report['residuals'])
        self.assertEqual(distribution([])['samples'],0)


if __name__ == '__main__':
    unittest.main()
