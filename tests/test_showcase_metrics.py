"""Hand-computed checks for descriptive metrics, not controller mirroring."""
from dataclasses import replace
import json
import math
import unittest

from traffic.showcase_metrics import MetricsConfig, ShowcaseMetrics, VehicleSample, measured_leader


def car(identity='ego', *, angle=0.0, radius=500.0, speed=10.0, reference=10.0, role=None):
    return VehicleSample(identity, radius*math.cos(angle), radius*math.sin(angle),
                         angle+math.pi/2, speed, 4.0, 2.0, reference,
                         role or ('ego' if identity == 'ego' else 'background'))


def collector(**kwargs):
    return ShowcaseMetrics(MetricsConfig(max_interval_s=10.0, **kwargs))


class GeometryTests(unittest.TestCase):
    def test_geometric_bumper_headway_and_front_to_front(self):
        result = measured_leader(car(), [car('lead', angle=.04, speed=5)])
        expected_gap = 20-1000*math.atan2(2, 499)
        self.assertEqual(result['leader_id'], 'lead')
        self.assertAlmostEqual(result['bumper_gap_m'], expected_gap)
        self.assertAlmostEqual(result['net_headway_s'], expected_gap/10)
        self.assertAlmostEqual(result['front_to_front_headway_s'], 2)
        self.assertAlmostEqual(result['ttc_s'], expected_gap/5)

    def test_outer_lane_uses_actual_radius_and_wraps_seam(self):
        result = measured_leader(car(angle=math.tau-.01, radius=511.1),
                                 [car('lead', angle=.03, radius=511.1, speed=5)])
        self.assertAlmostEqual(result['bumper_gap_m'], 511.1*(.04-2*math.atan2(2,510.1)))
        self.assertAlmostEqual(result['front_to_front_headway_s'], 511.1*.04/10)

    def test_no_leader_other_lane_behind_or_beyond_horizon(self):
        others = [car('adjacent', angle=.04, radius=503.7), car('rear', angle=-.02),
                  car('far', angle=.4)]
        self.assertTrue(all(v is None for v in measured_leader(car(), others).values()))

    def test_partial_lane_change_radial_overlap_is_eligible(self):
        self.assertEqual(measured_leader(car(radius=501.85),
                         [car('lead', angle=.04, radius=503.7)])['leader_id'], 'lead')

    def test_stationary_headway_undefined_nonclosing_ttc_undefined(self):
        result = measured_leader(car(speed=0), [car('lead', angle=.04, speed=0)])
        self.assertIsNone(result['net_headway_s'])
        self.assertIsNone(result['front_to_front_headway_s'])
        self.assertIsNone(result['ttc_s'])
        self.assertIsNotNone(result['bumper_gap_m'])
        result = measured_leader(car(), [car('lead', angle=.04, speed=12)])
        self.assertIsNone(result['ttc_s'])
        self.assertLess(result['closing_speed_m_s'], 0)

    def test_nearest_eligible_leader_is_selected_independent_of_input_order(self):
        a, b = car('a', angle=.06), car('b', angle=.03)
        self.assertEqual(measured_leader(car(), [a,b])['leader_id'], 'b')
        self.assertEqual(measured_leader(car(), [b,a])['leader_id'], 'b')

    def test_longitudinal_overlap_zero_gap_is_finite(self):
        result = measured_leader(car(), [car('lead', angle=.002, speed=5)])
        self.assertEqual(result['bumper_gap_m'], 0)
        self.assertEqual(result['ttc_s'], 0)
        json.dumps(result, allow_nan=False)


class IntegrationTests(unittest.TestCase):
    def test_constant_speed_time_loss_and_fairness(self):
        metrics = collector()
        samples = [car(), car('b', speed=5)]
        metrics.update(0, 'drive', samples)
        metrics.update(2, 'drive', samples)
        report = metrics.summary()
        by_id = {r['vehicle_id']: r for r in report['vehicles']}
        self.assertEqual(by_id['ego']['free_flow_time_loss_s'], 0)
        self.assertEqual(by_id['b']['free_flow_time_loss_s'], 1)
        self.assertEqual(by_id['b']['speed_integral_distance_m'], 10)
        self.assertEqual(by_id['b']['reference_distance_m'], 20)
        fairness = report['fairness']['all_lifetimes']
        self.assertAlmostEqual(fairness['achieved_fraction_jain_index'], .9)
        self.assertEqual(fairness['time_loss_fraction_range'], .5)
        self.assertEqual(report['fairness']['by_role']['background']['eligible_lifetimes'], 1)
        self.assertIsNone(report['completed_trips'])

    def test_acceleration_and_midpoint_jerk_for_nonuniform_time(self):
        metrics = collector()
        metrics.update(0, 'drive', [car(speed=10)])
        first = metrics.update(1, 'drive', [car(speed=8)])
        second = metrics.update(3, 'drive', [car(speed=2)])
        self.assertEqual(first['longitudinal_acceleration_m_s2'], -2)
        self.assertIsNone(first['longitudinal_jerk_m_s3'])
        self.assertEqual(second['longitudinal_acceleration_m_s2'], -3)
        self.assertAlmostEqual(second['longitudinal_jerk_m_s3'], -2/3)
        ego = metrics.summary()['vehicles'][0]
        self.assertEqual(ego['speed_integral_distance_m'], 19)
        self.assertAlmostEqual(ego['free_flow_time_loss_s'], 1.1)
        self.assertEqual(ego['hard_braking_events'], 1)
        self.assertEqual(ego['hard_braking_exposure_s'], 2)

    def test_no_overspeed_credit_and_reference_changes_are_explicit(self):
        metrics = collector()
        metrics.update(0, 'drive', [car(speed=20, reference=10)])
        metrics.update(1, 'drive', [car(speed=20, reference=10)])
        metrics.update(2, 'drive', [car(speed=5, reference=20)])
        ego = metrics.summary()['vehicles'][0]
        self.assertEqual(ego['free_flow_time_loss_s'], .375)
        self.assertEqual(ego['reference_distance_m'], 25)

    def test_settle_and_forced_brake_do_not_enter_drive_statistics(self):
        metrics = collector()
        metrics.update(0, 'settle', [car(speed=0)])
        metrics.update(1, 'settle', [car(speed=0)])
        metrics.update(2, 'drive', [car(speed=10)])
        metrics.update(3, 'drive', [car(speed=10)])
        latest = metrics.update(4, 'brake', [car(speed=0)])
        metrics.update(5, 'brake', [car(speed=0)])
        ego = metrics.summary()['vehicles'][0]
        self.assertEqual(ego['drive_exposure_s'], 1)
        self.assertEqual(ego['free_flow_time_loss_s'], 0)
        self.assertEqual(ego['hard_braking_events'], 0)
        self.assertIsNone(latest['longitudinal_acceleration_m_s2'])
        self.assertEqual(metrics.summary()['excluded_phase_boundary_s'], 2)

    def test_near_miss_episodes_and_sampled_exposure(self):
        metrics = collector()
        for time, clearance in enumerate([2, .9, .8, 2, .5]):
            metrics.update(time, 'drive', [car()], nearest_clearance_m=clearance)
        report = metrics.summary()['near_miss_proxy']
        self.assertEqual(report['events'], 2)
        self.assertEqual(report['exposure_s'], 3)
        self.assertEqual(report['monitored_drive_s'], 4)

    def test_closing_ttc_triggers_near_proxy_without_small_clearance(self):
        metrics = collector()
        samples = [car(speed=10), car('lead', angle=.02, speed=0)]
        latest = metrics.update(0, 'drive', samples, nearest_clearance_m=6)
        self.assertTrue(latest['near_miss_proxy'])
        self.assertEqual(latest['near_miss_events'], 1)
        self.assertEqual(latest['near_miss_exposure_s'], 0)

    def test_nonclosing_leader_has_no_ttc_exposure(self):
        metrics = collector()
        for t in (0, 1):
            metrics.update(t, 'drive', [car(), car('lead', angle=.04, speed=10)])
        self.assertEqual(metrics.summary()['ego_closing_ttc_s']['observed_s'], 0)
        self.assertEqual(metrics.summary()['ego_net_headway_s']['observed_s'], 1)

    def test_missing_interval_breaks_derivatives_and_counts_exclusion(self):
        metrics = ShowcaseMetrics()
        metrics.update(0, 'drive', [car(speed=10)])
        latest = metrics.update(1, 'drive', [car(speed=0)])
        self.assertIsNone(latest['longitudinal_acceleration_m_s2'])
        self.assertEqual(metrics.summary()['excluded_gap_s'], 1)
        self.assertEqual(metrics.summary()['vehicles'][0]['drive_exposure_s'], 0)

    def test_lifecycle_exit_reentry_and_run_censoring(self):
        metrics = collector()
        metrics.update(0, 'drive', [car(), car('slot_g001', speed=5)])
        metrics.update(1, 'drive', [car(), car('slot_g001', speed=5)])
        metrics.update(2, 'drive', [car()])
        metrics.update(3, 'drive', [car(), car('slot_g002', speed=0)])
        metrics.update(4, 'drive', [car(), car('slot_g002', speed=0)])
        records = {r['vehicle_id']: r for r in metrics.summary()['vehicles']}
        self.assertEqual(records['slot_g001']['observation_end'], 'lifecycle_exit')
        self.assertEqual(records['slot_g001']['last_observed_s'], 1)
        self.assertEqual(records['slot_g001']['drive_exposure_s'], 1)
        self.assertEqual(records['slot_g002']['observation_end'], 'run_censored')
        self.assertEqual(records['slot_g002']['drive_exposure_s'], 1)
        self.assertEqual(records['slot_g002']['longitudinal_acceleration_m_s2']['mean'], 0)
        self.assertFalse(any(r['completed_trip'] for r in records.values()))

    def test_short_lifetimes_excluded_from_fairness_not_ledger(self):
        metrics = collector()
        metrics.update(0, 'drive', [car()])
        metrics.update(.5, 'drive', [car()])
        report = metrics.summary()
        self.assertEqual(len(report['vehicles']), 1)
        self.assertEqual(report['fairness']['all_lifetimes']['excluded_short_lifetimes'], 1)
        self.assertIsNone(report['fairness']['all_lifetimes']['achieved_fraction_jain_index'])

    def test_all_stopped_jain_is_undefined_not_perfect(self):
        metrics = collector()
        for t in (0, 1):
            metrics.update(t, 'drive', [car(speed=0), car('b', speed=0)])
        report = metrics.summary()
        self.assertIsNone(report['fairness']['all_lifetimes']['achieved_fraction_jain_index'])
        self.assertEqual(report['fairness']['all_lifetimes']['mean_time_loss_fraction'], 1)


class ContractTests(unittest.TestCase):
    def test_invalid_values_rejected(self):
        for key, value in [('speed_m_s', math.nan), ('speed_m_s', math.inf), ('speed_m_s', -1),
                           ('length_m', 0), ('width_m', True), ('free_flow_speed_m_s', 0),
                           ('free_flow_speed_m_s', 1e-300), ('x_m', 1e300), ('vehicle_id', '')]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                replace(car(), **{key: value})
        with self.assertRaises(ValueError):
            replace(car(), x_m=0, y_m=0)

    def test_time_duplicate_reverse_and_nonfinite_are_atomic(self):
        metrics = collector()
        metrics.update(1, 'drive', [car()])
        before = metrics.summary()
        for time in (1, 0, math.nan, math.inf, True, 1+1e-12):
            with self.subTest(time=time), self.assertRaises(ValueError):
                metrics.update(time, 'drive', [car()])
            self.assertEqual(metrics.summary(), before)

    def test_retired_id_rejected_without_mutating_state(self):
        metrics = collector()
        metrics.update(0, 'drive', [car(), car('slot_g001')])
        metrics.update(1, 'drive', [car()])
        before = metrics.summary()
        with self.assertRaises(ValueError):
            metrics.update(2, 'drive', [car(), car('slot_g001')])
        self.assertEqual(before, metrics.summary())

    def test_duplicates_missing_ego_and_role_changes_rejected(self):
        for samples in ([], [car(), car()], [car('b')], [car(), car('b', role='ego')]):
            with self.subTest(samples=samples), self.assertRaises(ValueError):
                collector().update(0, 'drive', samples)
        metrics = collector()
        metrics.update(0, 'drive', [car(), car('b')])
        with self.assertRaises(ValueError):
            metrics.update(1, 'drive', [car('ego', role='background'), car('b', role='ego')], ego_id='b')

    def test_capacity_is_explicit_and_keeps_report_recoverable(self):
        metrics = collector(max_active=2, max_lifetimes=2)
        metrics.update(0, 'drive', [car(), car('a')])
        metrics.update(1, 'drive', [car()])
        before = metrics.summary()
        with self.assertRaises(ValueError):
            metrics.update(2, 'drive', [car(), car('b')])
        self.assertEqual(before, metrics.summary())

    def test_invalid_config_phase_and_clearance(self):
        for kw in ({'max_active': True}, {'max_lifetimes': 2}, {'near_ttc_s': math.inf},
                   {'max_interval_s': 0}, {'max_active': 1.5}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                MetricsConfig(**kw)
        for clearance in (-1, math.inf, math.nan, True):
            with self.subTest(clearance=clearance), self.assertRaises(ValueError):
                collector().update(0, 'drive', [car()], nearest_clearance_m=clearance)
        with self.assertRaises(ValueError):
            collector().update(0, 'unknown', [car()])

    def test_reports_are_strict_json_and_do_not_expose_mutable_state(self):
        metrics = collector()
        for t in (0, 1, 2):
            latest = metrics.update(t, 'drive', [car(speed=0)])
        latest['ego_time_loss_s'] = -1
        report = metrics.summary()
        self.assertEqual(report['latest']['ego_time_loss_s'], 2)
        json.dumps(report, allow_nan=False)
        report['phase_sample_counts']['drive'] = 0
        self.assertEqual(metrics.summary()['phase_sample_counts']['drive'], 3)

    def test_no_per_tick_history_growth(self):
        metrics = ShowcaseMetrics()
        samples = [car(), car('a')]
        for tick in range(1000):
            metrics.update(tick/120, 'drive', samples)
        self.assertEqual(len(metrics._lifetimes), 2)
        self.assertEqual(len(metrics._active_ids), 2)
        self.assertAlmostEqual(metrics.summary()['observed_drive_s'], 999/120)

    def test_report_identifies_empty_and_partial_observation_end(self):
        metrics = collector()
        self.assertIsNone(metrics.summary()['last_observed_sim_time_s'])
        self.assertIsNone(metrics.summary()['last_observed_phase'])
        metrics.update(.5, 'settle', [car(speed=0)])
        metrics.update(1.0, 'drive', [car(speed=1)])
        report = metrics.summary()
        self.assertEqual(report['last_observed_sim_time_s'], 1.0)
        self.assertEqual(report['last_observed_phase'], 'drive')
        self.assertEqual(report['vehicles'][0]['observation_end'], 'run_censored')


if __name__ == '__main__':
    unittest.main()
