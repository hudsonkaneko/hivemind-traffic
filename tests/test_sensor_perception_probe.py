"""Native probe acceptance and control tests without starting Isaac."""
from types import SimpleNamespace
import unittest

from scripts.probe_sensor_perception import assess, perception_control
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.bypass_validation import OrientedBox


class SensorProbeTests(unittest.TestCase):
    def rows(self):
        return [dict(time_s=i/10, injected_camera_fault=30<=i<35,
                     perception=dict(valid=not 30<=i<35,stop_required=i>=140,
                                     nearest_forward_gap_m=1.),
                     control=dict(throttle=0.,brake=1.),
                     evaluation=dict(progress_m=10.,speed_m_s=0.,upright_z=1.,
                                     true_barrier_gap_m=1.,footprint_within_lane=True)) for i in range(1,151)]

    def result(self, rows):
        return assess(rows,[],True,['camera-front-14s.png','overview-14s.png'])

    def test_controller_fails_closed(self):
        self.assertEqual(perception_control(None).brake,1.)
        estimate=SimpleNamespace(valid=False,stop_required=False)
        self.assertEqual(perception_control(estimate).throttle,0.)
        estimate=SimpleNamespace(valid=True,stop_required=True)
        self.assertEqual(perception_control(estimate).brake,1.)

    def test_controller_uses_only_local_perception(self):
        estimate=SimpleNamespace(valid=True,stop_required=False,
            lane=SimpleNamespace(center_offset_pixels=100.),ego=SimpleNamespace(speed_m_s=0.))
        command=perception_control(estimate)
        self.assertEqual(command.steering_rad,-.04)
        self.assertEqual(command.throttle,.5)

    def test_complete_fixture_passes(self):
        self.assertTrue(self.result(self.rows())['passed'])

    def test_stopped_due_to_lane_loss_does_not_pass(self):
        rows=self.rows()
        rows[-1]['perception']['valid']=False
        self.assertFalse(self.result(rows)['passed'])

    def test_false_progress_empty_evidence_and_ignored_fault_fail(self):
        self.assertFalse(self.result([])['passed'])
        for key,value in [('progress_m',0.),('true_barrier_gap_m',-1.),('footprint_within_lane',False)]:
            rows=self.rows();rows[-1]['evaluation'][key]=value
            self.assertFalse(self.result(rows)['passed'],key)
        rows=self.rows();rows[30]['control']['throttle']=.1
        self.assertFalse(self.result(rows)['passed'])
        self.assertFalse(assess(self.rows(),[],True,['overview-14s.png'])['passed'])

    def test_sparse_valid_measurements_fail(self):
        rows=self.rows()
        for row in rows[:90]:row['perception']['valid']=False
        self.assertFalse(self.result(rows)['passed'])

    def test_scene_geometry_constructor_contract(self):
        road=LaneRoute('sensor_straight',3.6,(RouteSegment(80.),),origin_xy_m=(0.,0.))
        self.assertEqual(road.length_m,80.)
        barrier=OrientedBox(14.,0.,0.,1.,3.)
        self.assertEqual(min(x for x,y in barrier.corners()),13.5)


if __name__=='__main__':unittest.main()
