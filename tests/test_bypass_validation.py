import math
import unittest
from traffic.bypass_validation import OrientedBox, box_clearance, footprint_within_road, RouteSafetyBrake, assess_bypass, predicted_route_poses
from traffic.lidar_braking import LidarScan


class BypassGeometryTests(unittest.TestCase):
    def test_exact_axis_and_diagonal_distance(self):
        a = OrientedBox(0,0,0,2,2)
        self.assertEqual(box_clearance(a,OrientedBox(3,0,0,2,2)),dict(clearance_m=1.,overlap=False))
        self.assertAlmostEqual(box_clearance(a,OrientedBox(3,3,0,2,2))['clearance_m'],math.sqrt(2))

    def test_rotated_overlap_touch_and_symmetry(self):
        a,b = OrientedBox(0,0,math.pi/4,2,2),OrientedBox(4,0,.3,2,2)
        self.assertEqual(box_clearance(a,b),box_clearance(b,a))
        self.assertTrue(box_clearance(a,OrientedBox(0,0,0))['overlap'])
        self.assertTrue(box_clearance(OrientedBox(0,0,0,2,2),OrientedBox(2,0,0,2,2))['overlap'])

    def test_full_footprint_not_center(self):
        self.assertTrue(footprint_within_road(OrientedBox(45,3.6,0)))
        self.assertFalse(footprint_within_road(OrientedBox(45,4.6,0)))
        self.assertFalse(footprint_within_road(OrientedBox(45,3.6,math.pi/2)))
        with self.assertRaises(ValueError):
            OrientedBox(float('nan'),0,0)


class RouteSafetyTests(unittest.TestCase):
    def scan(self, points):
        return LidarScan('e','v',0,0,0,points)

    def route(self, lateral=0):
        return [(i*.2,lateral*i*.2,0) for i in range(35)]

    def test_safe_scan_and_route(self):
        decision = RouteSafetyBrake('e','v').evaluate(self.scan([[20,0,0]]),tick=0,speed_m_s=3,predicted_route=self.route())
        self.assertEqual(decision.status,'clear')

    def test_lateral_hit_route_stop_and_latch(self):
        brake = RouteSafetyBrake('e','v')
        scan = self.scan([[3,2,0]])
        decision = brake.evaluate(scan,tick=0,speed_m_s=3,predicted_route=self.route(.5))
        self.assertEqual(decision.reason,'lidar_hit_in_predicted_body_envelope')
        self.assertEqual(decision.brake_override,1)
        again = brake.evaluate(LidarScan('e','v',0,0,0,[[20,0,0]],coordinate_reference_tick=1),tick=1,speed_m_s=3,predicted_route=self.route())
        self.assertEqual(again.reason,'latched_route_stop')

    def test_dropout_full_brake_and_sparse_route(self):
        for scan,route in ((None,self.route()),(self.scan([[20,0,0]]),[(0,0,0),(10,0,0)])):
            decision = RouteSafetyBrake('e','v').evaluate(scan,tick=0,speed_m_s=3,predicted_route=route)
            self.assertEqual(decision.brake_override,1)
            self.assertEqual(decision.target_speed_m_s,0)

    def test_stale_and_wrong_frame_do_not_plan(self):
        for scan,tick in ((self.scan([[20,0,0]]),25),(LidarScan('e','v',0,0,0,[[20,0,0]],frame='world'),0)):
            result = RouteSafetyBrake('e','v').evaluate(scan,tick=tick,speed_m_s=3,predicted_route=self.route())
            self.assertEqual(result.status,'stale_invalid')

    def test_preview_join_and_old_reference(self):
        from traffic.lane_geometry import LaneRoute, RouteSegment
        route = LaneRoute('straight',3.6,(RouteSegment(100),))
        poses = predicted_route_poses(route,dict(position_m=[10,.3,1],yaw_rad=.2),3)
        self.assertEqual(poses[0],(0.,0.,0.))
        self.assertTrue(all(math.dist(a[:2],b[:2]) <= .25 for a,b in zip(poses,poses[1:])))
        result = RouteSafetyBrake('e','v').evaluate(self.scan([[20,0,0]]),tick=1,speed_m_s=3,predicted_route=poses)
        self.assertEqual(result.reason,'scan_not_in_current_chassis_frame')

    def test_blocked_and_dropout_require_stop_not_pass(self):
        rows = [dict(tick=i+1,position_m=[40,0,1],yaw_rad=0,speed_m_s=2.5 if 241 <= i+1 < 1441 else 0,upright_z=1,
                     wheel_on_ground=[True]*4,obstacle_contacts=0,phase='hold',
                     control=dict(brake=1,throttle=0),safety_status='clear' if i+1 < 1440 else 'stale_invalid',
                     route_adopted_from_lidar=False) for i in range(2040)]
        for mode in ('blocked','dropout'):
            for row in rows:
                row['phase'] = 'hold' if row['tick'] >= 1441 else 'drive'
                row['control_tick'] = row['tick'] % 2 == 1
                row['lidar'] = dict(status='clear' if row['tick'] < 1441 else 'stale_invalid',
                                    oldest_sample_age_s=.2,brake_override=0 if row['tick'] < 1441 else 1,
                                    target_speed_m_s=3 if row['tick'] < 1441 else 0)
                row['safety_status'] = ('clear' if row['tick'] < 1441 else
                                       'obstacle' if mode == 'blocked' else 'stale_invalid')
            self.assertTrue(assess_bypass(rows,mode=mode)['passed'])
            rows[-1]['position_m'][0] = 85
            self.assertFalse(assess_bypass(rows,mode=mode)['passed'])
            rows[-1]['position_m'][0] = 40
            rows[-1]['control']['brake'] = 0
            self.assertFalse(assess_bypass(rows,mode=mode)['passed'])
            rows[-1]['control']['brake'] = 1

    def test_success_rejects_hidden_fault_and_unsafe_geometry(self):
        import copy
        rows = [dict(tick=i+1,position_m=[90 if i >= 400 else i*.2,3.6 if 160 <= i < 310 else 0,1],
                     yaw_rad=0,speed_m_s=0 if i >= 400 else 2.5,upright_z=1,
                     wheel_on_ground=[True]*4,obstacle_contacts=0,phase='hold' if i>=400 else 'drive',
                     control=dict(brake=1 if i>=400 else 0,throttle=0),
                     safety_status='clear',route_adopted_from_lidar=i>=50,
                     lidar=dict(status='clear',brake_override=0),planner=dict(status='detour'))
                for i in range(1000)]
        self.assertTrue(assess_bypass(rows)['passed'])
        for key,value in (('safety_status','stale_invalid'),('safety_status','no_safe_route'),
                          ('obstacle_contacts',1),('position_m',[45,0,1]),('position_m',[50,5,1])):
            altered = copy.deepcopy(rows)
            altered[300][key] = value
            self.assertFalse(assess_bypass(altered)['passed'])
        for nested,key,value in (('lidar','status','stale_invalid'),('lidar','brake_override',1),
                                 ('planner','status','no_safe_route')):
            altered = copy.deepcopy(rows)
            altered[300][nested][key] = value
            self.assertFalse(assess_bypass(altered)['passed'])

    def test_dropout_rejects_delayed_brake(self):
        rows = [dict(tick=i+1,position_m=[30,0,1],yaw_rad=0,speed_m_s=2.5 if i<1450 else 0,
                     upright_z=1,wheel_on_ground=[True]*4,obstacle_contacts=0,
                     phase='drive' if i<1450 else 'hold',control=dict(brake=1,throttle=0),
                     safety_status='clear' if i<1450 else 'stale_invalid',
                     control_tick=i%2==0,route_adopted_from_lidar=False,
                     lidar=dict(status='clear' if i<1450 else 'stale_invalid',oldest_sample_age_s=.2,
                                brake_override=0 if i<1450 else 1,target_speed_m_s=3 if i<1450 else 0))
                for i in range(2050)]
        result = assess_bypass(rows,mode='dropout')
        self.assertFalse(result['gates']['dropout_brake_timely'])
        self.assertFalse(result['passed'])

    def test_warmup_stale_is_not_dropout_success(self):
        rows = [dict(tick=i+1,position_m=[0,0,1],yaw_rad=0,speed_m_s=0,upright_z=1,
                     wheel_on_ground=[True]*4,obstacle_contacts=0,phase='hold',
                     control=dict(brake=1,throttle=0),safety_status='stale_invalid',
                     route_adopted_from_lidar=False) for i in range(2040)]
        for mode in ('blocked','dropout'):
            self.assertFalse(assess_bypass(rows,mode=mode)['passed'])


if __name__ == '__main__':
    unittest.main()
