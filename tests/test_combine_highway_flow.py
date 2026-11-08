"""Fail-closed cached-evidence compatibility; no simulator or learning."""
from copy import deepcopy
import unittest
from unittest.mock import patch

from scripts.combine_highway_flow import (
    BUILDER, CONTROLLERS, allocation, normalized_config, verify_case, verify_sources,
    verify_deadline_repair_eligibility,
)


def row(index=0):
    return dict(id='case-'+str(index), seed=100+index, controlled_count=1, total_vehicles=8,
        communication='none', packet_loss=0., speed_limit_m_s=18., difficulty=0, traffic_mode='initial',
        nominal_unobstructed=True,
        case=dict(id='case-'+str(index), task='merge', network='bank-v03/network.net.xml', network_sha256='a'*64),
        fixture=dict(fixture_version='v2', vehicles=[dict(vehicle_id='agent_0', speed=16., position=40., route='ramp')],
            events=[], speed_controls=[], static_spawn_checks=dict(vehicles=[dict(vehicle_id='agent_0', speed_m_s=16.)])))


class CachedFlowEvidenceTests(unittest.TestCase):
    def test_only_explicit_builder_source_difference_is_allowed(self):
        previous = {BUILDER:'old', 'traffic/highway_flow_reference.py':'same', 'environments/highway_flow_env.py':'same2'}
        current = dict(previous, **{BUILDER:'new'})
        with self.assertRaises(ValueError):
            verify_sources(previous, current)
        proof = verify_sources(previous, current, allow_builder_change=True)
        self.assertEqual(proof['changed_sources'], [BUILDER])
        self.assertTrue(proof['explicit_exception'])

    def test_reference_or_environment_change_is_never_allowed(self):
        previous = {BUILDER:'old', 'traffic/highway_flow_reference.py':'same', 'environments/highway_flow_env.py':'same2'}
        for key in ('traffic/highway_flow_reference.py','environments/highway_flow_env.py'):
            current = dict(previous, **{key:'different'})
            with self.subTest(key=key), self.assertRaises(ValueError):
                verify_sources(previous,current,allow_builder_change=True)

    def test_missing_source_inventory_entry_is_rejected(self):
        with self.assertRaises(ValueError):
            verify_sources({BUILDER:'old','runtime':'same'},{BUILDER:'new'},allow_builder_change=True)

    def test_network_location_and_fixture_version_do_not_change_physics(self):
        previous, expected = row(), row()
        expected['case']['network'] = 'bank-v04/network.net.xml'
        expected['fixture']['fixture_version'] = 'v3'
        expected['fixture']['static_spawn_checks']['vehicles'][0]['yield_approaches'] = []
        with patch('scripts.combine_highway_flow._compute_yield_approaches',return_value={'agent_0':[]}):
            proof = verify_case(expected, previous,allow_added_yield_metadata=True)
        self.assertTrue(proof['physical_inputs_identical'])
        self.assertTrue(proof['network_path_changed'])
        self.assertTrue(proof['fixture_version_changed'])

    def test_network_content_must_be_identical(self):
        expected, actual = row(), row()
        actual['case']['network_sha256'] = 'b'*64
        with self.assertRaises(ValueError):
            verify_case(expected,actual)

    def test_old_dense_speed_is_not_reusable_even_if_new_builder_is_allowed(self):
        expected, old = row(), row()
        expected['fixture']['vehicles'][0]['speed'] = 8.
        with self.assertRaises(ValueError):
            verify_case(expected, old)

    def test_vehicle_events_speed_controls_and_case_conditions_all_matter(self):
        changes = [lambda r:r['fixture']['vehicles'][0].update(position=41.),
                   lambda r:r['fixture']['events'].append(dict(at_s=1.,lane_delta=1)),
                   lambda r:r['fixture']['speed_controls'].append(dict(speed_m_s=8.)),
                   lambda r:r.update(packet_loss=.3), lambda r:r.update(nominal_unobstructed=False),
                   lambda r:r.update(difficulty=2), lambda r:r.update(speed_limit_m_s=12.)]
        for change in changes:
            actual = row();change(actual)
            with self.subTest(change=repr(change)), self.assertRaises(ValueError):
                verify_case(row(),actual)

    def test_nonempty_new_validation_metadata_is_not_silently_ignored(self):
        expected, actual = row(), row()
        expected['fixture']['static_spawn_checks']['vehicles'][0]['yield_approaches'] = [dict(stop_room=39.9)]
        with self.assertRaises(ValueError):
            verify_case(expected, actual)

    def test_added_metadata_requires_exact_recomputed_values(self):
        expected, actual = row(), row()
        added = [dict(source_lane='mlk_on_0', available_stop_room_m=100., required_stop_room_m=54.94444444444444)]
        expected['fixture']['static_spawn_checks']['vehicles'][0]['yield_approaches'] = added
        with patch('scripts.combine_highway_flow._compute_yield_approaches',return_value={'agent_0':added}):
            proof=verify_case(expected,actual,allow_added_yield_metadata=True)
            self.assertTrue(proof['added_yield_validation_proof']['all_values_exactly_recomputed'])
        with patch('scripts.combine_highway_flow._compute_yield_approaches',return_value={'agent_0':[]}):
            with self.assertRaises(ValueError):
                verify_case(expected,actual,allow_added_yield_metadata=True)

    def test_other_static_metadata_cannot_be_discarded(self):
        expected, actual = row(), row()
        actual['fixture']['static_spawn_checks']['vehicles'][0]['speed_m_s'] = 8.
        with self.assertRaises(ValueError):
            verify_case(expected, actual)

    def test_configuration_exception_is_only_bank_manifest_identity(self):
        old = dict(network_bank=dict(manifest_path='old',manifest_sha256='old',schema='fixed'),
                   optimizer=dict(actor_lr=.0001),environment=dict(speed_mode=31))
        new = deepcopy(old)
        new['network_bank'].update(manifest_path='new',manifest_sha256='new')
        self.assertEqual(normalized_config(old),normalized_config(new))
        new['optimizer']['actor_lr'] = .0002
        self.assertNotEqual(normalized_config(old),normalized_config(new))

    def test_allocation_is_exact_54_cached_and_27_fresh_without_outcome_input(self):
        rows = [row(i) for i in range(27)]
        plan = allocation(rows)
        self.assertEqual(len(plan),81)
        self.assertEqual(sum(p[2]=='cached' for p in plan),54)
        self.assertEqual(sum(p[2]=='fresh' for p in plan),27)
        for i in range(27):
            self.assertEqual(tuple(p[1] for p in plan[i*3:(i+1)*3]),CONTROLLERS)
        self.assertIs(plan[54][0],rows[18])

    def test_missing_or_duplicate_expected_rows_cannot_pass(self):
        for rows in ([row(i) for i in range(26)], [row(i) for i in range(26)]+[row(0)]):
            with self.assertRaises(ValueError):
                allocation(rows)

    def test_case_normalization_does_not_mutate_provenance(self):
        actual, expected = row(), row()
        original = deepcopy((actual,expected))
        verify_case(expected,actual)
        self.assertEqual((actual,expected),original)

    def test_only_fixed_incomplete_deadline_failure_is_repairable(self):
        result=dict(row_id='legacy-i110-merge',controller='v05_final',source_changed=[],
            error='Traceback\nexperiments.run_i110_merge.PilotBudgetExhausted: Aggregate wall-time budget exhausted\n')
        report=dict(primary_horizon_complete=False,accounting_comparison_valid=False)
        recording=dict(controller='v05_final',selected_case=dict(id='legacy-i110-merge',seed=6800019))
        proof=verify_deadline_repair_eligibility(result,report,recording)
        self.assertTrue(proof['replacement_is_outcome_independent'])
        for changed in (dict(result,controller='reference'),dict(result,row_id='another-case'),
                        dict(result,error='RuntimeError: bad driving'),dict(result,source_changed=['controller.py'])):
            with self.subTest(changed=changed),self.assertRaises(ValueError):
                verify_deadline_repair_eligibility(changed,report,recording)
        with self.assertRaises(ValueError):
            verify_deadline_repair_eligibility(result,dict(report,primary_horizon_complete=True),recording)
        wrong=deepcopy(recording);wrong['selected_case']['seed']=6800020
        with self.assertRaises(ValueError):
            verify_deadline_repair_eligibility(result,report,wrong)


if __name__=='__main__':
    unittest.main()
