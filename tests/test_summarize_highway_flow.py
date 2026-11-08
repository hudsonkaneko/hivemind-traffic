"""Full-route flow gates from synthetic saved evidence, without native runs."""
from copy import deepcopy
import unittest

from scripts.summarize_highway_flow import summarize_recording


def evidence():
    vehicle = dict(id='agent_0', speed=10., acceleration=0., distance_m=0., legal_speed_m_s=18.,
        route_reachable_without_lateral=True, lane_id='sb2_1',
        lane_identity=dict(kind='through', index_from_right=1), lateral_position=0., maneuver_in_progress=False)
    frames = [dict(time=i*.1, vehicles=[dict(vehicle, distance_m=float(i))] if i <= 100 else [])
              for i in range(801)]
    metric = dict(controlled=True, native_samples=101, measured_intervals=100, missing_intervals=0,
        longest_consecutive_stop_s=0., below_half_local_limit_exposure_s=0., peak_abs_measured_jerk_m_s3=0.,
        ending_lane_excursions=[], first_observed_s=0., last_observed_s=10.)
    fixture = dict(nominal_unobstructed=True, expected_external_event_count=0, events=[],
        vehicles=[dict(vehicle_id='agent_0', controlled=True)])
    recording = dict(frames=frames, controlled_ids=['agent_0'], horizon_seconds=80., controller='reference',
        selected_case=dict(id='example', case=dict(task='following'), seed=123, controlled_count=1, fixture=fixture))
    report = dict(primary_horizon_complete=True, accounting_comparison_valid=True, partial_control_horizon=False,
        training=False, optimizer_steps=0, collisions=0, teleports=0, overspeed_steps=0, emergency_braking_observed=0,
        native_emergencies=dict(available=True, warning_lines=0), agent_steps=100, overrides=0, intervention_fraction=0.,
        fixture_events=[], completed_trips=1, full_route_metrics=dict(per_car={'agent_0':metric}),
        accounting=dict(comparison_valid=True, totals=dict(requested=1, boundary_time_s=10.1),
            trips=[dict(vehicle_id='agent_0', state='completed', departed_at=0., arrived_at=10.1, had_teleport=False)]))
    return report, recording, dict(error=None, source_changed=[])


class FlowSummaryTests(unittest.TestCase):
    def test_complete_nominal_evidence_passes_without_training(self):
        summary = summarize_recording(*evidence())
        self.assertTrue(summary['passed'])
        self.assertEqual(summary['integrity_failures'], [])
        self.assertTrue(summary['vehicle_coverage']['agent_0']['full_route_complete'])
        self.assertFalse(summary['training'])
        self.assertEqual(summary['optimizer_steps'], 0)

    def test_stop_at_two_seconds_fails_but_one_point_nine_passes(self):
        report, recording, check = evidence()
        metric = report['full_route_metrics']['per_car']['agent_0']
        metric['longest_consecutive_stop_s'] = 1.9
        self.assertTrue(summarize_recording(report, recording, check)['passed'])
        metric['longest_consecutive_stop_s'] = 2.
        summary = summarize_recording(report, recording, check)
        self.assertFalse(summary['passed'])
        self.assertFalse(summary['controlled_gates']['agent_0']['no_two_second_stop'])

    def test_low_speed_exposure_at_two_passes_above_two_fails(self):
        report, recording, check = evidence()
        metric = report['full_route_metrics']['per_car']['agent_0']
        metric['below_half_local_limit_exposure_s'] = 2.
        self.assertTrue(summarize_recording(report, recording, check)['passed'])
        metric['below_half_local_limit_exposure_s'] = 2.1
        self.assertFalse(summarize_recording(report, recording, check)['passed'])

    def test_nominal_comfort_is_separate_from_unobstructed_speed(self):
        report, recording, check = evidence()
        fixture = recording['selected_case']['fixture']
        fixture.update(nominal_unobstructed=False, nominal_comfort=True)
        metric = report['full_route_metrics']['per_car']['agent_0']
        metric['below_half_local_limit_exposure_s'] = 30.
        self.assertTrue(summarize_recording(report, recording, check)['passed'])
        metric['peak_abs_measured_jerk_m_s3'] = 6.
        summary = summarize_recording(report, recording, check)
        self.assertFalse(summary['passed'])
        self.assertNotIn('low_speed_exposure_at_most_two_seconds', summary['controlled_gates']['agent_0'])
        self.assertFalse(summary['controlled_gates']['agent_0']['peak_jerk_at_most_five'])

    def test_stress_peaks_are_retained_without_nominal_reclassification(self):
        report, recording, check = evidence()
        recording['selected_case']['fixture'].update(nominal_unobstructed=False, nominal_comfort=False)
        metric = report['full_route_metrics']['per_car']['agent_0']
        metric.update(peak_abs_measured_jerk_m_s3=45., longest_consecutive_stop_s=10.,
            below_half_local_limit_exposure_s=30.)
        summary = summarize_recording(report, recording, check)
        self.assertTrue(summary['passed'])
        self.assertIsNone(summary['nominal_controlled_gates_pass'])
        self.assertEqual(summary['all_car_metrics']['agent_0']['peak_abs_measured_jerk_m_s3'], 45.)

    def test_raw_override_gate_is_unchanged_and_denominator_must_match(self):
        report, recording, check = evidence()
        report.update(overrides=5, intervention_fraction=.05)
        self.assertTrue(summarize_recording(report, recording, check)['passed'])
        report.update(overrides=6, intervention_fraction=.06)
        self.assertFalse(summarize_recording(report, recording, check)['passed'])
        report['intervention_fraction'] = .04
        self.assertIn('raw_override_denominator_missing_or_inconsistent',
            summarize_recording(report, recording, check)['integrity_failures'])

    def test_native_safety_warnings_cannot_be_excused_as_stress(self):
        report, recording, check = evidence()
        recording['selected_case']['fixture'].update(nominal_unobstructed=False, nominal_comfort=False)
        report['native_emergencies']['warning_lines'] = 1
        self.assertFalse(summarize_recording(report, recording, check)['passed'])

    def test_missing_nominal_flags_or_warning_log_fails_closed(self):
        report, recording, check = evidence()
        recording['selected_case']['fixture'].pop('nominal_unobstructed')
        report.pop('native_emergencies')
        summary = summarize_recording(report, recording, check)
        self.assertFalse(summary['passed'])
        self.assertIn('nominal_classification_missing', summary['integrity_failures'])

    def test_observation_gap_or_uncaptured_arrival_is_incomplete(self):
        report, recording, check = evidence()
        recording['frames'][50]['vehicles'] = []
        summary = summarize_recording(report, recording, check)
        self.assertFalse(summary['passed'])
        self.assertFalse(summary['vehicle_coverage']['agent_0']['observed_interval_complete'])
        report, recording, check = evidence()
        report['accounting']['trips'][0]['arrived_at'] = 20.
        self.assertFalse(summarize_recording(report, recording, check)['passed'])

    def test_uncensored_horizon_is_required_even_if_all_cars_arrive_early(self):
        report, recording, check = evidence()
        recording['frames'] = recording['frames'][:102]
        recording['horizon_seconds'] = 10.1
        summary = summarize_recording(report, recording, check)
        self.assertFalse(summary['passed'])
        self.assertIn('incomplete_primary_horizon', summary['integrity_failures'])

    def test_controlled_car_never_observed_cannot_pass(self):
        report, recording, check = evidence()
        recording['controlled_ids'].append('agent_1')
        recording['selected_case']['controlled_count'] = 2
        self.assertIn('controlled_vehicle_never_observed',
            summarize_recording(report, recording, check)['integrity_failures'])

    def test_requested_event_needs_centered_target_lane_completion(self):
        report, recording, check = evidence()
        fixture = recording['selected_case']['fixture']
        fixture.update(expected_external_event_count=1,
            events=[dict(vehicle_id='agent_0', at_s=1., lane_delta=1)])
        report['fixture_events'] = [dict(index=0, requested_at=1., command_issued=True, movement_observed=True)]
        self.assertFalse(summarize_recording(report, recording, check)['event_completion']['complete'])
        for sample in recording['frames'][11:101]:
            sample['vehicles'][0]['lane_identity']['index_from_right'] = 2
            sample['vehicles'][0]['lane_id'] = 'sb2_2'
        # Each original row's nested identity is shared by the fixture helper;
        # set explicit independent source and target mappings for this boundary.
        for sample in recording['frames'][:101]:
            sample['vehicles'][0]['lane_identity'] = dict(kind='through', index_from_right=1 if sample['time'] <= 1. else 2)
        summary = summarize_recording(report, recording, check)
        self.assertTrue(summary['event_completion']['complete'])
        self.assertEqual(summary['event_completion']['details'][0]['settled_at_s'], 1.1)

    def test_route_excursion_is_not_hidden_by_other_good_metrics(self):
        report, recording, check = evidence()
        report['full_route_metrics']['per_car']['agent_0']['ending_lane_excursions'] = [dict(time_s=1.)]
        self.assertFalse(summarize_recording(report, recording, check)['route_utility_gate'])

    def test_task_first_merge_entry_does_not_replace_stable_completion(self):
        report, recording, check = evidence()
        recording['selected_case']['case']['task'] = 'merge'
        report.update(merge_target_count=1, stable_merge=dict(targets=1, stable=0, entered=1))
        self.assertFalse(summarize_recording(report, recording, check)['task_gate'])

    def test_no_mutation_and_missing_integrity_proof_cannot_pass(self):
        report, recording, check = evidence()
        before = deepcopy((report, recording, check))
        self.assertFalse(summarize_recording(report, recording)['passed'])
        self.assertEqual((report, recording, check), before)


if __name__ == '__main__':
    unittest.main()
