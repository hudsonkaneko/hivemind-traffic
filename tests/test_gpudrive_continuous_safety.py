"""Continuous policy safeguards without SUMO, inference, or training."""
import unittest
from types import SimpleNamespace

import numpy as np

from traffic.gpudrive_continuous_safety import guarded_step, recover_lane_state
from traffic.gpudrive_sumo_bridge import corners


class OpenRoad:
    def offroad(self, car):
        return False


class NarrowRoad:
    def offroad(self, car):
        return any(abs(point[1]) > 1. for point in corners(car))


class ContinuousSafetyTests(unittest.TestCase):
    def car(self, **changes):
        return dict(dict(id='ego', x=0., y=0., yaw=0., speed=10.,
                         acceleration=0., length=4.8, width=1.8), **changes)

    def test_continuous_steering_survives_and_moves_vehicle_laterally(self):
        proposed, audit = guarded_step(self.car(), 0., .524, OpenRoad())
        self.assertTrue(audit['accepted'])
        self.assertEqual(audit['applied_steering_rad'], .524)
        self.assertGreater(proposed['y'], 0.)
        self.assertGreater(proposed['yaw'], 0.)
        self.assertGreater(proposed['x'], 0.)

    def test_sixty_mph_and_lower_road_limits_are_hard_caps(self):
        for initial_speed, allowed, cap in ((26.8, 40., 26.8224), (9.9, 10., 10.)):
            with self.subTest(allowed_speed=allowed):
                proposed, audit = guarded_step(self.car(speed=initial_speed), 2., 0.,
                                               OpenRoad(), allowed_speed=allowed)
                self.assertTrue(audit['accepted'])
                self.assertLessEqual(proposed['speed'], cap + 1e-10)
                self.assertTrue(audit['reasons'])

    def test_braking_stops_without_reverse_or_speed_bounce(self):
        proposed, audit = guarded_step(self.car(speed=.1), -4.5, 0., OpenRoad())
        self.assertTrue(audit['accepted'])
        self.assertAlmostEqual(proposed['speed'], 0.)
        self.assertGreaterEqual(proposed['x'], 0.)

    def test_acceleration_request_is_bounded(self):
        proposed, audit = guarded_step(self.car(), 50., 0., OpenRoad())
        self.assertTrue(audit['accepted'])
        self.assertAlmostEqual(audit['applied_acceleration'], 2.)
        self.assertAlmostEqual(proposed['speed'], 10.2)

    def test_close_slower_leader_forces_braking(self):
        proposed, audit = guarded_step(self.car(speed=15.), 2., 0., OpenRoad(),
            front={'bumper_gap_m': 4., 'relative_speed_m_s': -5.})
        self.assertTrue(audit['accepted'])
        self.assertAlmostEqual(audit['applied_acceleration'], -4.5)
        self.assertLess(proposed['speed'], 15.)
        self.assertTrue(audit['reasons'])

    def test_lane_end_and_blocked_entry_constrain_speed(self):
        for parameter in ('lane_end_distance', 'blocked_entry_distance'):
            with self.subTest(constraint=parameter):
                proposed, audit = guarded_step(self.car(speed=15.), 2., 0., OpenRoad(),
                                               **{parameter: 5.})
                self.assertTrue(audit['accepted'])
                self.assertLess(audit['applied_acceleration'], 0.)
                self.assertLess(proposed['speed'], 15.)

    def test_full_footprint_offroad_proposal_is_rejected(self):
        ego = self.car()
        self.assertFalse(NarrowRoad().offroad(ego))
        proposed, audit = guarded_step(ego, 0., .524, NarrowRoad())
        self.assertLess(abs(proposed['y']), 1.)  # Center alone remains on road.
        self.assertFalse(audit['accepted'])
        self.assertTrue(audit['rejection_reasons'])
        self.assertEqual(audit['applied_steering_rad'], .524)

    def test_full_footprint_contact_proposal_is_rejected(self):
        other = self.car(id='other', x=5., speed=0.)
        _, audit = guarded_step(self.car(), 0., 0., OpenRoad(), others=(other,))
        self.assertFalse(audit['accepted'])
        self.assertTrue(audit['rejection_reasons'])

    def lane_recovery_context(self):
        lane = dict(points=np.asarray([[0., 0.], [100., 0.]]),
                    width=3.2, length=100., speed=26.8224)
        road = SimpleNamespace(lanes={'road_0': lane})
        graph = SimpleNamespace(lanes={'road_0': {}}, allowed=lambda route: {'road_0'})
        return road, graph

    def test_missing_lane_recovery_uses_front_bumper_position(self):
        road, graph = self.lane_recovery_context()
        ego = self.car(x=10., lane_id='', lane_position=-1., route=('road',))
        recovered = recover_lane_state(ego, road, graph)
        self.assertEqual(recovered['lane_id'], 'road_0')
        self.assertAlmostEqual(recovered['lane_position'], 12.4)
        self.assertEqual(recovered['x'], 10.)
        self.assertEqual(recovered['y'], 0.)
        self.assertEqual(recovered['yaw'], 0.)
        self.assertEqual(ego['lane_id'], '')

    def test_missing_lane_recovery_rejects_pose_far_from_road(self):
        road, graph = self.lane_recovery_context()
        ego = self.car(x=10., y=20., lane_id='', lane_position=-1., route=('road',))
        with self.assertRaises(ValueError):
            recover_lane_state(ego, road, graph)


if __name__ == '__main__':
    unittest.main()
