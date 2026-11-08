import math
import unittest
from traffic.gpudrive_sumo_bridge import bicycle_step, center_from_front, front_from_center, overlaps


class BridgeTests(unittest.TestCase):
    def car(self, **kw):
        return dict(dict(x=0.,y=0.,yaw=0.,speed=10.,length=4.8,width=1.8),**kw)

    def test_front_center_roundtrip_all_directions(self):
        for heading in (0.,90.,180.,270.,123.4):
            x,y,yaw=center_from_front(20.,-50.,heading,4.8)
            fx,fy,h=front_from_center(self.car(x=x,y=y,yaw=yaw))
            self.assertAlmostEqual(fx,20.);self.assertAlmostEqual(fy,-50.)
            self.assertAlmostEqual(h,heading)

    def test_classic_straight_midpoint_integration(self):
        c=bicycle_step(self.car(),2.,0.)
        self.assertAlmostEqual(c['x'],1.01);self.assertAlmostEqual(c['speed'],10.2)
        self.assertEqual(c['y'],0.);self.assertEqual(c['yaw'],0.)

    def test_left_steering_moves_left_and_turns_left(self):
        c=bicycle_step(self.car(),0.,.2)
        self.assertGreater(c['y'],0.);self.assertGreater(c['yaw'],0.)

    def test_braking_through_zero_preserves_upstream_speed_magnitude(self):
        c=bicycle_step(self.car(speed=.1),-4.,0.)
        self.assertAlmostEqual(c['speed'],.3)
        self.assertAlmostEqual(c['signed_step_speed'],-.3)
        self.assertAlmostEqual(c['x'],-.01)

    def test_contact_detects_overlap_across_lane_ids(self):
        self.assertTrue(overlaps(self.car(),self.car(x=2.,y=.2,yaw=.3)))
        self.assertFalse(overlaps(self.car(),self.car(x=10.)))
        self.assertFalse(overlaps(self.car(),self.car(y=4.,yaw=math.pi/2)))


if __name__=='__main__': unittest.main()
