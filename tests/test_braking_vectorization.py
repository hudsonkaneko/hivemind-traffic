"""Scalar-oracle equivalence for the speedup; safety thresholds stay fixed."""
from dataclasses import asdict
import math

import numpy as np
import pytest

from traffic.bypass_validation import OrientedBox, points_hit_route
from traffic.lidar_braking import LidarBrakeConfig, LidarEmergencyBrake, LidarScan


def scalar_hit(points, boxes, cfg):
    for x, y, z in points:
        if not cfg.min_height_m <= z <= cfg.max_height_m:
            continue
        if abs(x) < 2.4 and abs(y) < .9:
            continue
        for box, padding in boxes:
            c, s = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
            dx, dy = x-box.x_m, y-box.y_m
            if (abs(c*dx+s*dy) <= 2.4+padding+cfg.lateral_margin_m
                    and abs(-s*dx+c*dy) <= .9+padding+cfg.lateral_margin_m):
                return True
    return False


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_array_brake_matches_scalar_for_random_and_boundary_points(dtype):
    cfg = LidarBrakeConfig()
    rng = np.random.default_rng(814)
    boundary = [[x, y, z] for x in (2.4, np.nextafter(2.4, math.inf), 7., 50.)
                for y in (.9, 1.15, np.nextafter(1.15, math.inf), -1.15)
                for z in (-.6, .6, np.nextafter(.6, math.inf))]
    clouds = [np.array(boundary, dtype=dtype)]
    clouds += [rng.uniform([-5, -8, -2], [40, 8, 2], (200, 3)).astype(dtype) for _ in range(40)]
    fast, scalar = LidarEmergencyBrake('e', 'v'), LidarEmergencyBrake('e', 'v')
    for tick, cloud in enumerate(clouds):
        a = fast.evaluate(LidarScan('e', 'v', tick, tick, tick, cloud), tick=tick, speed_m_s=3.)
        b = scalar.evaluate(LidarScan('e', 'v', tick, tick, tick, cloud.tolist()), tick=tick, speed_m_s=3.)
        assert asdict(a) == asdict(b)
        # Check the unlatched decision too, not only a persistent first stop.
        fast.reset(episode_id='e', vehicle_id='v')
        scalar.reset(episode_id='e', vehicle_id='v')


@pytest.mark.parametrize('points', [np.array([[10., 0., math.nan]]),
    np.array([[10., math.inf, 0.]]), np.array([[10., 0.]]),
    np.empty((0, 3)), np.array([[True, False, False]]), [[10., True, 0.]],
    np.array([['10', '0', '0']]), np.array([[10, object(), 0]], dtype=object)])
def test_bad_cloud_still_fails_safe(points):
    decision = LidarEmergencyBrake('e', 'v').evaluate(
        LidarScan('e', 'v', 0, 0, 0, points), tick=0, speed_m_s=3.)
    assert decision.status == 'stale_invalid'
    assert decision.brake_override == 1


def test_route_predicate_matches_scalar_for_random_and_edges():
    cfg = LidarBrakeConfig()
    rng = np.random.default_rng(901)
    for _ in range(100):
        boxes = [(OrientedBox(float(x), float(y), float(a)), float(p))
                 for x, y, a, p in rng.uniform([0, -2, -.5, 0], [6, 2, .5, .25], (30, 4))]
        cloud = rng.uniform([-5, -8, -2], [40, 8, 2], (250, 3))
        assert points_hit_route(cloud, boxes, cfg) == scalar_hit(cloud.tolist(), boxes, cfg)
    boxes = [(OrientedBox(0., 0., 0.), 0.)]
    for x in (2.4, 2.65, np.nextafter(2.65, math.inf), 20.):
        for y in (.9, 1.15, np.nextafter(1.15, math.inf), -1.15):
            for z in (-.6, .6, np.nextafter(.6, math.inf)):
                cloud = np.array([[x, y, z]])
                assert points_hit_route(cloud, boxes, cfg) == scalar_hit(cloud.tolist(), boxes, cfg)


def test_ndarray_subclasses_retain_scalar_validation():
    class MatrixShapedRows(np.ndarray):
        def __iter__(self):
            for row in super().__iter__():
                yield row.reshape(1, -1)

    points = np.array([[10., 0., 0.]]).view(MatrixShapedRows)
    decision = LidarEmergencyBrake('e', 'v').evaluate(
        LidarScan('e', 'v', 0, 0, 0, points), tick=0, speed_m_s=3.)
    assert decision.status == 'stale_invalid'
    assert decision.brake_override == 1
