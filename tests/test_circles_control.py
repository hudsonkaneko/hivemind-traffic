import unittest
from types import SimpleNamespace
from traffic.circles_control import measured_leader_input, acceleration_to_pedals


class CirclesAdapterTests(unittest.TestCase):
    def test_bumper_gap_and_signed_velocity(self):
        state = dict(position_m=[0,0,0], yaw_rad=0., velocity_m_s=[3,1,0])
        lead = SimpleNamespace(x_m=20., y_m=0., yaw_rad=0., length_m=4.8,
                               width_m=1.8, vx_m_s=2., vy_m_s=0., timestamp_s=1.)
        values=measured_leader_input(state, lead, now_s=1.)
        self.assertEqual(values[:2], [3.,2.])
        self.assertAlmostEqual(values[2], 15.2)
        with self.assertRaises(ValueError): measured_leader_input(state, lead, now_s=1.3)
        lead.x_m=2.
        with self.assertRaises(ValueError): measured_leader_input(state, lead, now_s=1.)

    def test_actuation_sign_and_cap(self):
        throttle,brake,target=acceleration_to_pedals(1.,1.)
        self.assertAlmostEqual(throttle,.42)
        self.assertEqual(brake,0.)
        self.assertEqual(target,1.6)
        self.assertEqual(acceleration_to_pedals(3.,1.), (0.,0.,3.))
        self.assertEqual(acceleration_to_pedals(1.,-3.), (0.,.35,0.))
        with self.assertRaises(ValueError): acceleration_to_pedals(1.,float('nan'))


if __name__ == '__main__': unittest.main()
