import unittest

from traffic.gpudrive_longitudinal import shape_acceleration, control_metrics
from traffic.gpudrive_continuous_safety import guarded_step

SETTINGS = dict(standstill_gap_m=2.5, headway_s=1.8, gap_gain_s2=.15,
                relative_speed_gain_s=.8, speed_gain_s=.8, jerk_limit_m_s3=3.)


class OpenRoad:
    def offroad(self, car): return False


class LongitudinalTests(unittest.TestCase):
    def car(self, **kw):
        return dict(dict(id='ego',x=0.,y=0.,yaw=0.,speed=20.,length=4.8,width=1.8,
                         acceleration=0.),**kw)

    def test_raw_units_are_not_rescaled_and_routine_jerk_is_limited(self):
        value,audit=shape_acceleration(self.car(),4.,None,26.8224,SETTINGS)
        self.assertEqual(audit['raw_request'],4.)
        self.assertEqual(audit['range_bounded'],2.)
        self.assertAlmostEqual(value,.3)

    def test_gap_and_relative_speed_anticipate_braking(self):
        front=dict(id='lead',bumper_gap_m=25.,relative_speed_m_s=-3.)
        value,audit=shape_acceleration(self.car(),4.,front,26.8224,SETTINGS)
        self.assertLess(audit['following_target'],0.)
        self.assertAlmostEqual(value,-.3)
        self.assertEqual(audit['front']['id'],'lead')

    def test_emergency_guard_bypasses_comfort_smoothing(self):
        ego=self.car(acceleration=2.)
        front=dict(id='lead',bumper_gap_m=3.,relative_speed_m_s=-10.)
        value,_=shape_acceleration(ego,4.,front,26.8224,SETTINGS)
        self.assertAlmostEqual(value,1.7)
        _,audit=guarded_step(ego,value,0.,OpenRoad(),front=front)
        self.assertAlmostEqual(audit['applied_acceleration'],-4.5)

    def test_final_legal_cap_is_not_softened(self):
        ego=self.car(speed=26.8224,acceleration=2.)
        value,_=shape_acceleration(ego,4.,None,26.8224,SETTINGS)
        proposed,_=guarded_step(ego,value,0.,OpenRoad())
        self.assertLessEqual(proposed['speed'],26.8224)

    def test_empty_metrics(self):
        self.assertEqual(control_metrics([])['final_guard_changed_fraction'],0.)


if __name__=='__main__': unittest.main()
