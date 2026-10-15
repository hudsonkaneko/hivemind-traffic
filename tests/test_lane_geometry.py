"""CPU geometry/coordinate contracts; these tests do not run either simulator."""
from dataclasses import FrozenInstanceError
import json
import math
from pathlib import Path
import random

import pytest

from traffic.lane_geometry import LaneRoute, RigidTransform2D, RouteSegment, load_routes, wrap_yaw


ROOT = Path(__file__).parents[1]
FIXTURE = ROOT / 'scenarios/physics-road/routes.json'


@pytest.fixture
def routes():
    return load_routes(FIXTURE)


def definition(**changes):
    data = dict(route_id='test', width_m=3.6, origin_xy_m=[0, 0], initial_yaw_rad=0,
                segments=[dict(length_m=100, curvature_rad_m=0)])
    return {**data, **changes}


def offset_xy(point, lateral):
    return (point.position_xy[0] - math.sin(point.yaw_rad) * lateral,
            point.position_xy[1] + math.cos(point.yaw_rad) * lateral)


def test_shared_fixture_has_exact_arc_lengths_and_units(routes):
    assert set(routes) == {'straight100', 'left_r60', 'right_r60'}
    for route in routes.values():
        assert route.length_m == 100
        assert route.total_length_m == 100
        assert route.width_m == 3.6
        assert route.origin_xy_m == (0.0, 0.0)
        assert route.evaluate(0).position_xy == (0.0, 0.0)


@pytest.mark.parametrize('station', [-20, -1.6, 0, 19.5, 50, 100, 102, 120])
def test_straight_exact_evaluation_and_endpoint_extensions(routes, station):
    point = routes['straight100'].evaluate(station)
    assert point.s_m == station
    assert point.position_xy == (station, 0)
    assert point.yaw_rad == 0
    assert point.curvature_rad_m == 0


@pytest.mark.parametrize('station', [-3, 0, 20, 45, 80, 100, 104])
@pytest.mark.parametrize('lateral', [-1.8, 0, 1.8])
def test_straight_projection_sign_and_unclamped_station(routes, station, lateral):
    result = routes['straight100'].project(station, lateral)
    assert result.s_m == station
    assert result.lateral_error_m == lateral
    assert result.distance_m == abs(lateral)


@pytest.mark.parametrize('route_id,sign', [('left_r60', 1), ('right_r60', -1)])
def test_curve_has_analytic_anchor_points(routes, route_id, sign):
    route = routes[route_id]
    at_join = route.evaluate(20)
    assert at_join.position_xy == (20, 0)
    assert at_join.yaw_rad == 0
    assert at_join.curvature_rad_m == sign / 60
    arc_end = route.evaluate(80)
    assert arc_end.position_xy == pytest.approx((20 + 60 * math.sin(1), sign * 60 * (1 - math.cos(1))), abs=1e-12)
    assert arc_end.yaw_rad == pytest.approx(sign)
    assert arc_end.curvature_rad_m == 0
    end = route.evaluate(100)
    assert end.position_xy == pytest.approx((20 + 60 * math.sin(1) + 20 * math.cos(1),
                                           sign * (60 * (1 - math.cos(1)) + 20 * math.sin(1))), abs=1e-12)
    beyond = route.evaluate(105)
    assert beyond.position_xy == pytest.approx((end.position_xy[0] + 5 * math.cos(1),
                                              end.position_xy[1] + sign * 5 * math.sin(1)), abs=1e-12)
    assert beyond.curvature_rad_m == 0


@pytest.mark.parametrize('route_id', ['left_r60', 'right_r60'])
def test_position_and_heading_continuity_at_segment_joins(routes, route_id):
    route = routes[route_id]
    for station in (20, 80):
        left, right = route.evaluate(station - 1e-7), route.evaluate(station + 1e-7)
        assert math.dist(left.position_xy, right.position_xy) == pytest.approx(2e-7, abs=2e-13)
        assert abs(wrap_yaw(right.yaw_rad - left.yaw_rad)) < 1e-8


@pytest.mark.parametrize('curvature', [1e-14, -1e-14, 1e-9, -1e-9])
def test_small_curvature_retains_straight_limit_projection_accuracy(curvature):
    route = LaneRoute('nearly_straight', 3.6, (RouteSegment(100, curvature),))
    for station in (0.5, 20, 50, 99.5):
        point = route.evaluate(station)
        query = offset_xy(point, 1.2)
        projection = route.project(*query)
        assert projection.s_m == pytest.approx(station, abs=2e-11)
        assert projection.lateral_error_m == pytest.approx(1.2, abs=2e-11)


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
def test_dense_frenet_world_roundtrip(routes, route_id):
    route = routes[route_id]
    for index in range(241):
        station = -10 + index / 2
        for lateral in (-1.75, -0.2, 0.0, 0.2, 1.75):
            xy = offset_xy(route.evaluate(station), lateral)
            projected = route.project(*xy)
            assert projected.s_m == pytest.approx(station, abs=3e-11)
            assert projected.lateral_error_m == pytest.approx(lateral, abs=3e-11)
            assert projected.distance_m == pytest.approx(abs(lateral), abs=3e-11)
            assert offset_xy(projected.point, projected.lateral_error_m) == pytest.approx(xy, abs=3e-11)


@pytest.mark.parametrize('route_id', ['left_r60', 'right_r60'])
def test_projection_agrees_with_independent_dense_distance_search(routes, route_id):
    route = routes[route_id]
    randomizer = random.Random(301)
    sampled = [route.evaluate(index / 10).position_xy for index in range(-200, 1201)]
    for _ in range(20):
        point = route.evaluate(randomizer.uniform(0, 100))
        query = offset_xy(point, randomizer.uniform(-5, 5))
        projection = route.project(*query)
        sampled_min = min(math.dist(query, candidate) for candidate in sampled)
        assert projection.distance_m <= sampled_min + 1e-12
        assert sampled_min - projection.distance_m < 0.051


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
@pytest.mark.parametrize('lateral', [-1.8, 0, 1.8])
def test_export_samples_bound_spacing_preserve_joins_and_match_reference(routes, route_id, lateral):
    route = routes[route_id]
    points = route.sample(max_spacing_m=0.5, lateral_offset_m=lateral)
    assert isinstance(points, tuple)
    assert points[0].s_m == 0
    assert points[-1].s_m == 100
    if route_id != 'straight100':
        assert {20, 80}.issubset({point.s_m for point in points})
    assert all(math.dist(a.position_xy, b.position_xy) <= 0.5 + 1e-12 for a, b in zip(points, points[1:]))
    for point in points:
        center = route.evaluate(point.s_m)
        assert point.position_xy == pytest.approx(offset_xy(center, lateral), abs=1e-12)
        assert point.yaw_rad == center.yaw_rad
        assert point.curvature_rad_m == pytest.approx(center.curvature_rad_m / (1 - center.curvature_rad_m * lateral))
        assert point.lateral_offset_m == lateral


def test_road_art_and_control_can_share_the_same_sample_coordinates(routes):
    # Contract-level correspondence only: no SUMO network, USD scene, or live
    # simulator has been opened or verified by this numeric test.
    route = routes['left_r60']
    shared = route.sample()
    visual_world_xyz = [(p.position_xy[0], p.position_xy[1], 0) for p in shared]
    controller_xy = [route.evaluate(p.s_m).position_xy for p in shared]
    assert [point[:2] for point in visual_world_xyz] == controller_xy
    assert len(shared) == 201
    assert sum(math.dist(a.position_xy, b.position_xy) for a, b in zip(shared, shared[1:])) == pytest.approx(100, abs=0.001)


@pytest.mark.parametrize('angle', [0, math.pi / 2, -math.pi / 2, math.pi, 3.5])
def test_rigid_frame_point_and_heading_roundtrips(angle):
    transform = RigidTransform2D((24.5, -19), angle)
    for point in ((0, 0), (1, 0), (0, 1), (-1.6, 1.8), (100, -45)):
        assert transform.to_local(transform.to_world(point)) == pytest.approx(point, abs=1e-12)
    for yaw in (-3, -0.5, 0, 0.5, 3):
        assert wrap_yaw(transform.heading_to_local(transform.heading_to_world(yaw)) - yaw) == pytest.approx(0, abs=1e-12)


def test_two_frame_anchors_and_heading_are_explicit():
    frame = RigidTransform2D((10, -4), math.pi / 2)
    assert frame.to_world((0, 0)) == (10, -4)
    assert frame.to_world((10, 0)) == pytest.approx((10, 6))
    assert frame.to_world((0, 5)) == pytest.approx((5, -4))
    assert frame.heading_to_world(0) == pytest.approx(math.pi / 2)


def test_rotated_route_evaluation_and_projection_correspond_to_original(routes):
    source = routes['left_r60']
    frame = RigidTransform2D((-30, 17), 2.9)
    rotated = LaneRoute('rotated', source.width_m, source.segments, frame.origin_xy_m, frame.yaw_rad)
    for station in (-5, 0, 12, 20, 40, 80, 95, 100, 107):
        local, world = source.evaluate(station), rotated.evaluate(station)
        assert world.position_xy == pytest.approx(frame.to_world(local.position_xy), abs=1e-12)
        assert world.yaw_rad == pytest.approx(frame.heading_to_world(local.yaw_rad), abs=1e-12)
        query = frame.to_world(offset_xy(local, 1.2))
        assert rotated.project(*query).s_m == pytest.approx(station, abs=1e-11)
        assert rotated.project(*query).lateral_error_m == pytest.approx(1.2, abs=1e-11)


def test_immutable_route_detaches_input_lists():
    raw = definition()
    route = LaneRoute.from_dict(raw)
    raw['origin_xy_m'][0] = 999
    raw['segments'][0]['length_m'] = 1
    assert route.length_m == 100
    assert route.origin_xy_m == (0, 0)
    with pytest.raises(FrozenInstanceError): route.width_m = 1
    with pytest.raises(FrozenInstanceError): route.segments[0].length_m = 1
    with pytest.raises(FrozenInstanceError): route.evaluate(0).yaw_rad = 1
    with pytest.raises(FrozenInstanceError): route.project(0, 0).s_m = 1


@pytest.mark.parametrize('field,value', [
    ('route_id', ''), ('width_m', 0), ('width_m', True), ('width_m', math.inf),
    ('origin_xy_m', [0, math.nan]), ('origin_xy_m', [0, 0, 0]),
    ('initial_yaw_rad', math.nan), ('initial_yaw_rad', True), ('segments', []),
    ('segments', [dict(length_m=0, curvature_rad_m=0)]),
    ('segments', [dict(length_m=1, curvature_rad_m=math.inf)]),
    ('segments', [dict(length_m=10, curvature_rad_m=math.pi / 10)]),
    ('segments', [dict(length_m=1, curvature_rad_m=1)]),
])
def test_bad_route_definitions_are_rejected(field, value):
    with pytest.raises(ValueError): LaneRoute.from_dict(definition(**{field: value}))


def test_unknown_keys_do_not_silently_change_units():
    with pytest.raises(ValueError): LaneRoute.from_dict(definition(yaw_degrees=90))
    with pytest.raises(ValueError): LaneRoute.from_dict(definition(segments=[dict(length_m=10, radius_m=60)]))


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, True, '3'])
def test_invalid_geometry_queries_fail(routes, value):
    route = routes['straight100']
    with pytest.raises(ValueError): route.evaluate(value)
    with pytest.raises(ValueError): route.project(value, 0)
    with pytest.raises(ValueError): route.project(0, value)
    with pytest.raises(ValueError): route.sample(value)
    with pytest.raises(ValueError): route.sample(lateral_offset_m=value)
    with pytest.raises(ValueError): RigidTransform2D((0, 0), value)


def test_invalid_sampling_density_and_arc_offsets_fail(routes):
    for spacing in (0, -1, 1e-8):
        with pytest.raises(ValueError): routes['left_r60'].sample(spacing)
    with pytest.raises(ValueError): routes['left_r60'].sample(lateral_offset_m=60)
    with pytest.raises(ValueError): routes['right_r60'].sample(lateral_offset_m=-60)


def test_loader_rejects_wrong_frame_schema_or_duplicate_id(tmp_path):
    data = json.loads(FIXTURE.read_text())
    for field, bad in [('schema_version', 2), ('frame', 'pixels'), ('routes', []),
                       ('routes', [data['routes'][0], data['routes'][0]])]:
        path = tmp_path / 'bad.json'
        path.write_text(json.dumps({**data, field: bad}))
        with pytest.raises(ValueError): load_routes(path)
