"""The diagnostic must not broaden its interventions beyond the named factor."""
import unittest
from types import SimpleNamespace

from scripts.probe_i110_merge_blocking import install_acceleration_probe, native_snapshot, variant_labels


class AccelerationProbeTests(unittest.TestCase):
    def api(self):
        calls = []
        # No safety-changing method is exposed: using one is a test failure.
        api = SimpleNamespace(
            setAcceleration=lambda *args: calls.append(('acceleration', *args)),
            setSpeed=lambda *args: calls.append(('speed', *args)))
        return api, calls

    def test_lifetime_changes_only_focal_and_preserves_requested_acceleration(self):
        api, calls = self.api()
        applied = {}
        install_acceleration_probe(api, 5., applied)
        api.setAcceleration('agent_0', 1.6, .1)
        api.setAcceleration('agent_1', -.7, .1)
        self.assertEqual(calls, [('acceleration','agent_0',1.6,5.),
                                 ('acceleration','agent_1',-.7,.1)])
        self.assertEqual(applied['agent_0'], {'acceleration':1.6,'duration_s':5.})

    def test_baseline_is_unchanged(self):
        api, calls = self.api()
        install_acceleration_probe(api, .1, {})
        api.setAcceleration('agent_0', -4.5, .1)
        self.assertEqual(calls, [('acceleration','agent_0',-4.5,.1)])

    def test_native_handoff_releases_only_focal_speed_override(self):
        api, calls = self.api()
        applied = {}
        install_acceleration_probe(api, None, applied)
        api.setAcceleration('agent_0', 1.6, .1)
        api.setAcceleration('agent_7', 1.4, .1)
        self.assertEqual(calls, [('speed','agent_0',-1),
                                 ('acceleration','agent_7',1.4,.1)])
        self.assertTrue(applied['agent_0']['native_longitudinal'])

    def test_departed_focal_is_not_queried(self):
        env = SimpleNamespace(states={}, time=80.1)
        self.assertEqual(native_snapshot(env), {'time':80.1,'present':False})

    def test_snapshot_never_calls_stateful_follow_speed_even_with_leader(self):
        class API:
            def getLeader(self, *args):
                return ('leader', 40.)
            def getFollowSpeed(self, *args):
                raise AssertionError('ACC follow-speed getter can mutate native state')
            def __getattr__(self, name):
                return lambda *args: None
        lane = SimpleNamespace(getLength=lambda _:100., getLinks=lambda *a,**k:[])
        state = dict(lane_id='mlk_on_0',lane_position=40.,speed=5.,acceleration=1.,x=0.,y=0.)
        env = SimpleNamespace(states={'agent_0':state},time=1.,
                              conn=SimpleNamespace(vehicle=API(),lane=lane))
        result = native_snapshot(env)
        self.assertEqual(result['leader'], ('leader',40.))
        self.assertIsNone(result['leader_follow_speed'])
        self.assertIn('not_queried', result['leader_follow_speed_status'])

    def test_network_treatment_is_all_traffic_despite_focal_telemetry(self):
        network = variant_labels('network_replacement')
        self.assertEqual(network['treatment_scope'], 'all traffic using the network')
        self.assertIsNone(network['directly_treated_vehicle_ids'])
        self.assertEqual(network['network_treatment'], 'comparison_network')
        self.assertEqual(network['telemetry_focal_vehicle'], 'agent_0')
        self.assertEqual(variant_labels('baseline')['directly_treated_vehicle_ids'], [])
        self.assertEqual(variant_labels('acceleration_command_lifetime')['directly_treated_vehicle_ids'], ['agent_0'])


if __name__ == '__main__':
    unittest.main()
