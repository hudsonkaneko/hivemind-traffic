import math
import unittest
from traffic.gpudrive_native_adapter import native_request
from environments.sumo_mappo_env import action_to_acceleration
from environments.highway_stage1_env import acceleration_constraints


class NativeAdapterTests(unittest.TestCase):
    def test_acceleration_saturation_is_explicit(self):
        a,d=native_request(4.,0.,20.,3.5)
        self.assertAlmostEqual(action_to_acceleration(a[0]),2.)
        self.assertTrue(d['acceleration_range_clipped'])
        a,d=native_request(-4.,0.,20.,3.5)
        self.assertAlmostEqual(action_to_acceleration(a[0]),-4.,places=6)
        self.assertFalse(d['acceleration_range_clipped'])

    def test_periodic_steering_preserves_bicycle_semantics(self):
        for steer in (0.,math.pi,-math.pi,-3.142):
            self.assertEqual(native_request(0.,steer,26.8224,3.5)[0][1],0.)
        self.assertEqual(native_request(0.,.524,20.,3.5)[0][1],1.)
        self.assertEqual(native_request(0.,-.524,20.,3.5)[0][1],-1.)

    def test_no_lane_request_when_stationary(self):
        self.assertEqual(native_request(2.,.524,0.,3.5)[0][1],0.)

    def test_reused_speed_and_leader_guards_bind(self):
        _,c=acceleration_constraints({'speed':26.8224,'allowed_speed':40.},None,None)
        self.assertEqual(c,0.)
        _,c=acceleration_constraints({'speed':15.,'allowed_speed':26.8224},
            {'bumper_gap_m':4.,'relative_speed_m_s':-5.},None)
        self.assertEqual(c,-4.5)


if __name__=='__main__':unittest.main()
