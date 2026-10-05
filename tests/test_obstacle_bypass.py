import math
import unittest

from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.lidar_braking import LidarScan
from traffic.path_following import VehicleState
from traffic.obstacle_bypass import ObstacleBypassPlanner, BypassConfig, road_footprint_bounds, PLANNER_SOURCE


class BypassTests(unittest.TestCase):
    def setUp(self):
        self.nominal = LaneRoute('straight', 3.6, (RouteSegment(100),))
        self.planner = ObstacleBypassPlanner('episode', 'car', self.nominal)

    def update(self, tick, x=15, front=44, points=None, end=None):
        state = VehicleState('episode', 'car', tick, x, 0, 0, 2.5)
        points = points if points is not None else [(front-x, y, 0) for y in (-.8, 0, .8)]
        scan = LidarScan('episode', 'car', tick if end is None else end,
            tick if end is None else end, tick, points, coordinate_reference_tick=tick)
        return self.planner.update(scan, state, tick)

    def test_distinct_scans_and_frozen_path(self):
        self.assertEqual(self.update(0).status, 'confirming')
        self.assertEqual(self.update(1, end=0).status, 'confirming')
        decision = self.update(6, x=15.1)
        self.assertEqual(decision.status, 'detour')
        self.assertEqual(decision.sensed_bounds, (44., 44., -.8, .8))
        self.assertEqual(decision.extent_prior_m, (4., 3.))
        route = decision.route
        self.assertIs(self.update(12, x=16, front=45).route, route)
        self.assertAlmostEqual(route.evaluate(route.length_m).position_xy[0], 90)
        self.assertAlmostEqual(route.evaluate(route.length_m).position_xy[1], 0)
        self.assertAlmostEqual(decision.stop_s_m, route.length_m)
        passing = route.project(44, 3.8)
        self.assertLess(passing.distance_m, 1e-8)
        for i in range(501):
            point = route.evaluate(route.length_m*i/500)
            extent = .9*abs(math.cos(point.yaw_rad))+2.4*abs(math.sin(point.yaw_rad))
            self.assertGreaterEqual(point.position_xy[1]-extent, -1.8)
            self.assertLessEqual(point.position_xy[1]+extent, 5.4)
            self.assertLess(abs(point.curvature_rad_m), .04)

    def test_too_late_latches_stop(self):
        self.update(0, x=25)
        decision = self.update(6, x=25.1)
        self.assertEqual(decision.status, 'stop')
        self.assertEqual(decision.target_speed_m_s, 0)
        self.assertEqual(self.update(12, points=[(20, 0, -1)]).status, 'stop')

    def test_excludes_ground_ego_offroad_and_behind(self):
        points = [(29, 0, -1), (1, 0, 0), (-3, 0, 0), (29, 6, 0)]*3
        self.assertEqual(self.update(0, points=points).status, 'nominal')

    def test_bad_and_stale_scan_fail_closed_even_after_freeze(self):
        self.update(0)
        self.update(6)
        self.assertEqual(self.update(12, points=[(math.nan, 0, 0)]).status, 'stale_invalid')
        self.assertEqual(self.update(40, end=6).status, 'stale_invalid')

    def test_narrow_road_no_feasible_route(self):
        self.planner = ObstacleBypassPlanner('episode', 'car', self.nominal,
            BypassConfig(road_max_y_m=4.0))
        self.update(0)
        self.assertEqual(self.update(6).status, 'stop')

    def test_sparse_points_do_not_confirm(self):
        self.assertEqual(self.update(0, points=[(29, -.8, 0)]).status, 'nominal')

    def test_accumulated_distinct_scan_coverage(self):
        self.update(0, points=[(29, -.8, 0), (29, -.7, 0), (29, -.6, 0)])
        decision = self.update(6, points=[(29, .6, 0), (29, .7, 0), (29, .8, 0)])
        self.assertEqual(decision.status, 'detour')
        self.assertEqual(decision.sensed_bounds, (44., 44., -.8, .8))
        self.assertEqual(decision.sensed_acquisition_ticks, (0, 6))

    def test_waits_for_coverage_then_plans(self):
        center = [(29, y, 0) for y in (-.1, 0, .1)]
        self.update(0, points=center)
        self.assertEqual(self.update(6, points=center).status, 'confirming')
        self.assertEqual(self.update(12).status, 'detour')

    def test_confirmation_wait_has_deadline(self):
        self.planner = ObstacleBypassPlanner('episode', 'car', self.nominal,
            BypassConfig(max_confirmation_ticks=12))
        center = [(29, y, 0) for y in (-.1, 0, .1)]
        self.update(0, points=center)
        self.assertEqual(self.update(6, points=center).status, 'confirming')
        self.assertEqual(self.update(12, points=center).status, 'stop')

    def test_invalid_clock(self):
        self.update(6)
        decision = self.update(6)
        self.assertEqual(decision.status, 'stale_invalid')
        self.assertEqual(decision.reason, 'invalid_local_clock')

    def test_first_gpu_run_sensed_bounds_fit_conservative_reference_envelope(self):
        points = [(44.03-15, y, 0) for y in (-.5422439, 0, .523)]
        self.update(0, points=points)
        decision = self.update(6, points=points)
        self.assertEqual(decision.status, 'detour')
        self.assertEqual(decision.reason, 'confirmed_static_obstacle_bypass')
        self.assertEqual(decision.source, PLANNER_SOURCE)
        self.assertEqual(decision.sensed_bounds, (44.03, 44.03, -.5422439, .523))
        self.assertEqual(decision.extent_prior_m, (4., 3.))
        self.assertEqual(decision.target_speed_m_s, 3.)
        cfg = self.planner.config
        lower, upper = road_footprint_bounds(decision.route, cfg)
        self.assertGreaterEqual(lower, -1.8)
        self.assertLessEqual(upper, 5.4)
        # Dense independent samples must fit inside the outward certified bound.
        for i in range(20001):
            point = decision.route.evaluate(decision.route.length_m*i/20000)
            extent = .9*abs(math.cos(point.yaw_rad))+2.4*abs(math.sin(point.yaw_rad))
            self.assertGreaterEqual(point.position_xy[1]-extent, lower)
            self.assertLessEqual(point.position_xy[1]+extent, upper)
        self.assertGreaterEqual(3.8-(-.5422439+3), .9+.25)
        self.assertAlmostEqual(decision.route.evaluate(decision.stop_s_m).position_xy[0], 90)

    def test_reject_wrong_odometry_and_route(self):
        with self.assertRaises(ValueError):
            ObstacleBypassPlanner('episode', 'car', LaneRoute('curve', 3.6, (RouteSegment(90, .01),)))
        state = VehicleState('other', 'car', 0, 15, 0, 0, 2)
        self.assertEqual(self.planner.update(None, state, 0).status, 'stale_invalid')


if __name__ == '__main__':
    unittest.main()
