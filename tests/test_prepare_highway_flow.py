"""Recorder metrics/adapter checks without a native simulator."""
from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.prepare_highway_flow import capture_frame, route_metrics


def vehicle(identity='agent_0', *, speed=10., acceleration=0., limit=18.,
            kind='through', reachable=True, lane='sb2_1', distance=0.):
    return dict(id=identity, speed=speed, acceleration=acceleration,
        legal_speed_m_s=limit, lane_id=lane, lane_identity={'kind': kind},
        route_reachable_without_lateral=reachable, distance_m=distance)


def frame(time, *vehicles):
    return dict(time=time, vehicles=list(vehicles))


class PreparationMetricTests(unittest.TestCase):
    def test_reset_sample_is_not_an_extra_interval_and_startup_jerk_is_kept(self):
        rows = [frame(0., vehicle(speed=0., acceleration=0.)),
                frame(.1, vehicle(speed=0., acceleration=2.))]
        result = route_metrics(rows, ['agent_0'])['per_car']['agent_0']
        self.assertEqual(result['native_samples'], 2)
        self.assertEqual(result['measured_intervals'], 1)
        self.assertAlmostEqual(result['stopped_exposure_s'], .1)
        self.assertAlmostEqual(result['longest_consecutive_stop_s'], .1)
        self.assertEqual(result['peak_abs_measured_jerk_m_s3'], 20.)

    def test_two_second_stop_uses_twenty_one_samples_and_twenty_intervals(self):
        rows = [frame(i*.1, vehicle(speed=.099)) for i in range(21)]
        result = route_metrics(rows, ['agent_0'])['per_car']['agent_0']
        self.assertAlmostEqual(result['longest_consecutive_stop_s'], 2.)
        self.assertAlmostEqual(result['stopped_exposure_s'], 2.)

    def test_exact_threshold_speed_is_not_stopped(self):
        rows = [frame(i*.1, vehicle(speed=.1)) for i in range(21)]
        result = route_metrics(rows, ['agent_0'])['per_car']['agent_0']
        self.assertEqual(result['longest_consecutive_stop_s'], 0.)
        self.assertEqual(result['stopped_exposure_s'], 0.)

    def test_missing_observation_breaks_stop_and_jerk_without_fabrication(self):
        rows = [frame(0., vehicle(speed=0., acceleration=0.)),
                frame(.1, vehicle(speed=0., acceleration=1.)),
                frame(.2), frame(.3),
                frame(.4, vehicle(speed=0., acceleration=-4.)),
                frame(.5, vehicle(speed=0., acceleration=-3.))]
        result = route_metrics(rows, ['agent_0'])['per_car']['agent_0']
        self.assertEqual(result['measured_intervals'], 2)
        self.assertAlmostEqual(result['stopped_exposure_s'], .2)
        self.assertAlmostEqual(result['longest_consecutive_stop_s'], .1)
        self.assertAlmostEqual(result['peak_abs_measured_jerk_m_s3'], 10.)
        self.assertGreater(result['missing_intervals'], 0)

    def test_late_arrival_first_sample_has_no_invented_previous_acceleration(self):
        rows = [frame(0.), frame(.1, vehicle('native_1', acceleration=9.)),
                frame(.2, vehicle('native_1', acceleration=10.)), frame(.3)]
        result = route_metrics(rows, [])['per_car']['native_1']
        self.assertFalse(result['controlled'])
        self.assertEqual(result['native_samples'], 2)
        self.assertEqual(result['measured_intervals'], 1)
        self.assertAlmostEqual(result['peak_abs_measured_jerk_m_s3'], 10.)
        self.assertEqual(result['last_observed_s'], .2)

    def test_low_speed_exposure_uses_each_endpoint_current_legal_limit(self):
        rows = [frame(0., vehicle(speed=8., limit=30.)),
                frame(.1, vehicle(speed=8., limit=10.)),
                frame(.2, vehicle(speed=8., limit=30.))]
        result = route_metrics(rows, ['agent_0'])['per_car']['agent_0']
        self.assertAlmostEqual(result['below_half_local_limit_exposure_s'], .1)

    def test_ending_excursion_uses_route_and_lane_semantics_not_an_edge_seam(self):
        rows = [frame(0., vehicle(lane='sb2_1')),
                frame(.1, vehicle(lane='sb3_1')),
                frame(.2, vehicle(kind='auxiliary', reachable=False, lane='sb3_0'))]
        result = route_metrics(rows, ['agent_0'])['per_car']['agent_0']
        self.assertEqual(len(result['ending_lane_excursions']), 1)
        self.assertEqual(result['ending_lane_excursions'][0]['time_s'], .2)

    def test_duplicate_and_nonmonotonic_timestamps_fail_closed(self):
        for times in ((0., 0.), (.1, 0.)):
            with self.subTest(times=times), self.assertRaises(ValueError):
                route_metrics([frame(t, vehicle()) for t in times], ['agent_0'])

    def test_nonfinite_motion_fails_closed(self):
        for key in ('speed', 'acceleration', 'legal_speed_m_s', 'distance_m'):
            bad = vehicle()
            bad[key] = float('nan')
            with self.subTest(key=key), self.assertRaises(ValueError):
                route_metrics([frame(0., bad)], ['agent_0'])

    def test_capture_labels_every_car_from_map_even_without_actor_observation(self):
        source = frame(80., vehicle('agent_0'), vehicle('native_1'))
        env = SimpleNamespace(states={identity: dict(id=identity, lane_id='sb2_1', allowed_speed=21.)
            for identity in ('agent_0', 'native_1')}, obs={}, speed_limit_m_s=18.,
            graph=SimpleNamespace(speed_limits={'sb2_1':20.}))
        with patch('scripts.prepare_highway_flow.native_frame', return_value=deepcopy(source)), \
                patch('traffic.highway_flow_context.route_continuation', return_value={'reachable':False}) as route:
            result = capture_frame(env)
        self.assertEqual(route.call_count, 2)
        for row in result['vehicles']:
            self.assertFalse(row['route_reachable_without_lateral'])
            self.assertEqual(row['legal_speed_m_s'], 18.)
            self.assertEqual(row['native_allowed_speed_m_s'], 21.)

    def test_optional_episode_ceiling_does_not_change_lane_legal_speed(self):
        source = frame(0., vehicle())
        env = SimpleNamespace(states={'agent_0':dict(lane_id='sb2_1', allowed_speed=23.)},
            obs={}, speed_limit_m_s=None, graph=SimpleNamespace(speed_limits={'sb2_1':20.}))
        with patch('scripts.prepare_highway_flow.native_frame', return_value=deepcopy(source)), \
                patch('traffic.highway_flow_context.route_continuation', return_value={'reachable':True}):
            self.assertEqual(capture_frame(env)['vehicles'][0]['legal_speed_m_s'], 20.)


if __name__ == '__main__':
    unittest.main()
