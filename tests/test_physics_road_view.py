"""CPU visual geometry checks; optional USD checks require pxr, not a renderer."""
import math
from pathlib import Path

import pytest

from traffic.lane_geometry import load_routes
from visualization.physics_road_view import (
    author_physics_road, build_road_geometry, camera_pose_follow, camera_pose_overview,
    route_ribbon, target_ring,
    lane_divider_geometry,
)


@pytest.fixture
def routes():
    return load_routes(Path(__file__).parents[1] / 'scenarios/physics-road/routes.json')


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
def test_asphalt_exact_lane_width_padding_and_map_correspondence(routes, route_id):
    route = routes[route_id]
    mesh = build_road_geometry(route)['Asphalt']
    assert len(mesh.points) % 2 == 0
    assert len(mesh.face_counts) == len(mesh.points) // 2 - 1
    assert len(mesh.face_indices) == 4 * len(mesh.face_counts)
    for left, right in zip(mesh.points[::2], mesh.points[1::2]):
        assert math.dist(left, right) == pytest.approx(3.6, abs=1e-12)
        middle = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)
        assert route.project(*middle).distance_m < 1e-10
    first = tuple((mesh.points[0][i] + mesh.points[1][i]) / 2 for i in (0, 1))
    last = tuple((mesh.points[-2][i] + mesh.points[-1][i]) / 2 for i in (0, 1))
    assert first == pytest.approx(route.evaluate(-5).position_xy)
    assert last == pytest.approx(route.evaluate(105).position_xy)
    assert all(p[2] == 0.025 for p in mesh.points)


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
def test_ribbon_winding_faces_up_and_outer_spacing_is_bounded(routes, route_id):
    mesh = build_road_geometry(routes[route_id])['Asphalt']
    for i in range(0, len(mesh.face_indices), 4):
        a, b, c, _ = [mesh.points[index] for index in mesh.face_indices[i:i+4]]
        signed_z = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        assert signed_z > 0
    for edge in (mesh.points[::2], mesh.points[1::2]):
        assert max(math.dist(a, b) for a, b in zip(edge, edge[1:])) <= 0.5 + 1e-12


def test_lane_edge_marks_stay_inside_asphalt_and_above_it(routes):
    route = routes['left_r60']
    geometry = build_road_geometry(route)
    for name, sign in [('LeftEdge', 1), ('RightEdge', -1)]:
        for point in geometry[name].points:
            lateral = route.project(*point[:2]).lateral_error_m
            assert 1.68 - 1e-10 <= sign * lateral <= 1.8 + 1e-10
            assert point[2] == 0.035


def test_target_ring_is_flat_bounded_and_cannot_create_tall_debug_obstacle():
    mesh = target_ring((10, 20))
    assert len(mesh.points) == 64
    assert len(mesh.face_counts) == 32
    for i, point in enumerate(mesh.points):
        assert point[2] == 0.085
        assert math.hypot(point[0] - 10, point[1] - 20) == pytest.approx(.425 if i % 2 == 0 else .335)
    with pytest.raises(ValueError): target_ring((0, 0), z_m=1)


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
def test_overview_is_above_and_targets_route_bounds(routes, route_id):
    route = routes[route_id]
    pose = camera_pose_overview(route)
    points = build_road_geometry(route)['Asphalt'].points
    assert pose.eye[2] >= 25
    assert pose.target[2] == 0
    for axis in (0, 1):
        assert min(p[axis] for p in points) <= pose.target[axis] <= max(p[axis] for p in points)
    assert all(math.isfinite(v) for v in (*pose.eye, *pose.target))


def test_follow_camera_matches_offsets_at_cardinal_headings():
    pose = camera_pose_follow((10, 20, 1), 0)
    assert pose.eye == (-2, 11, 9)
    assert pose.target == (20, 20, 1)
    pose = camera_pose_follow((10, 20, 1), math.pi / 2)
    assert pose.eye == pytest.approx((19, 8, 9))
    assert pose.target == pytest.approx((10, 30, 1))


@pytest.mark.parametrize('bad', [math.nan, math.inf, True, '1'])
def test_nonfinite_camera_and_geometry_inputs_rejected(routes, bad):
    with pytest.raises(ValueError): camera_pose_follow((0, 0, 1), bad)
    with pytest.raises(ValueError): camera_pose_follow((0, bad, 1), 0)
    with pytest.raises(ValueError): build_road_geometry(routes['straight100'], endpoint_padding_m=bad)
    with pytest.raises(ValueError): target_ring((0, bad))
    with pytest.raises(ValueError): route_ribbon(routes['straight100'], 0, 10, width_m=bad)


@pytest.mark.parametrize('begin,end,width,spacing', [(1, 1, 1, .5), (2, 1, 1, .5),
                                                   (0, 1, 0, .5), (0, 1, 1, 0),
                                                   (0, 100, 1, 1e-8)])
def test_invalid_or_unbounded_ribbons_rejected(routes, begin, end, width, spacing):
    with pytest.raises(ValueError): route_ribbon(routes['straight100'], begin, end, width_m=width, max_spacing_m=spacing)


def _usd_stage():
    pytest.importorskip('pxr.Usd')
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.Xform.Define(stage, '/World')
    return stage


def test_usd_authoring_has_no_physics_and_updates_only_debug_layer(routes):
    from_import = _usd_stage()
    from pxr import UsdGeom, UsdPhysics, UsdShade
    stage = from_import
    original_target = stage.GetEditTarget().GetLayer().identifier
    view = author_physics_road(stage, routes['left_r60'])
    static_before = view.static_layer.ExportToString()
    view.update(dict(position_m=[10, 0, 1], yaw_rad=0), target_xy=(15, 0))
    assert view.static_layer.ExportToString() == static_before
    assert stage.GetEditTarget().GetLayer().identifier == original_target
    assert not stage.GetCompositionErrors()
    for prim in stage.Traverse():
        assert not prim.HasAPI(UsdPhysics.CollisionAPI)
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
        assert not prim.HasAPI(UsdPhysics.MassAPI)
    assert UsdGeom.Imageable(stage.GetPrimAtPath('/World/PhysicsRoad/Debug')).GetPurposeAttr().Get() == 'default'
    for name in ('ReferenceCenterline', 'UpcomingPath', 'PursuitTarget'):
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/World/PhysicsRoad/Debug/' + name))
        assert mesh.GetPurposeAttr().Get() == 'default'
        assert max(point[2] for point in mesh.GetPointsAttr().Get()) <= 0.1
    for name in ('Reference', 'Upcoming', 'Target'):
        shader = UsdShade.Shader(stage.GetPrimAtPath('/World/PhysicsRoad/Materials/' + name + '/Shader'))
        assert shader.GetInput('emissiveColor').Get() is not None
    asphalt_shader = UsdShade.Shader(stage.GetPrimAtPath('/World/PhysicsRoad/Materials/Asphalt/Shader'))
    assert not asphalt_shader.GetInput('emissiveColor')
    assert view.metadata['debug_purpose'] == 'default'
    assert not view.metadata['debug_sensor_exclusion_verified']
    assert 'raw RTX LiDAR' in view.metadata['debug_sensor_visibility']
    assert view.camera_pose('follow', dict(position_m=[0, 0, 1], yaw_rad=0)) == camera_pose_follow((0, 0, 1), 0)


def test_usd_reference_toggle_and_endpoint_update(routes):
    stage = _usd_stage()
    from pxr import UsdGeom
    view = author_physics_road(stage, routes['straight100'])
    view.update(dict(position_m=[105, 0, 1]), show_reference=False)
    assert UsdGeom.Imageable(stage.GetPrimAtPath('/World/PhysicsRoad/Debug')).GetVisibilityAttr().Get() == 'invisible'
    assert UsdGeom.Imageable(stage.GetPrimAtPath('/World/PhysicsRoad/Debug/UpcomingPath')).GetVisibilityAttr().Get() == 'invisible'
    assert UsdGeom.Imageable(stage.GetPrimAtPath('/World/PhysicsRoad/Debug/PursuitTarget')).GetVisibilityAttr().Get() == 'invisible'
    with pytest.raises(ValueError): author_physics_road(stage, routes['straight100'])


def test_lane_divider_has_actual_dash_gaps_and_relative_offset():
    from traffic.lane_geometry import LaneRoute, RouteSegment
    road = LaneRoute('two-lane', 7.2, (RouteSegment(100),), origin_xy_m=(0, 1.8))
    mesh = lane_divider_geometry(road, 0)
    assert all(p[2] == .04 for p in mesh.points)
    assert all(1.74 <= p[1] <= 1.86 for p in mesh.points)
    for face in range(len(mesh.face_counts)):
        xs = [mesh.points[i][0] for i in mesh.face_indices[4*face:4*face+4]]
        assert int(min(xs) // 6) == int((max(xs) - 1e-9) // 6)
        assert max(xs) % 6 <= 3 + 1e-9
    with pytest.raises(ValueError): lane_divider_geometry(road, 3.6)


def test_usd_actual_detour_preview_and_optional_static_divider():
    from traffic.lane_geometry import LaneRoute, RouteSegment
    stage = _usd_stage()
    from pxr import UsdGeom, UsdPhysics
    road = LaneRoute('two-lane', 7.2, (RouteSegment(100),), origin_xy_m=(0, 1.8))
    angle = 2 * math.atan(3.6 / 20)
    radius = 10 / math.sin(angle)
    detour = LaneRoute('detour', 3.6, (RouteSegment(10), RouteSegment(radius*angle, 1/radius),
                       RouteSegment(radius*angle, -1/radius), RouteSegment(70)))
    view = author_physics_road(stage, road, lane_dividers_m=(0,), show_reference_centerline=False)
    before = view.static_layer.ExportToString()
    state = dict(position_m=[10, 0, 1])
    view.update(state, target_xy=(30, 3.6), planned_route=detour, preview_distance_m=35)
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/World/PhysicsRoad/Debug/UpcomingPath'))
    expected = route_ribbon(detour, 10, 45, width_m=.16, z_m=.07)
    for actual, wanted in zip(mesh.GetPointsAttr().Get(), expected.points):
        assert tuple(actual) == pytest.approx(wanted, abs=2e-6)
    assert len(mesh.GetPointsAttr().Get()) == len(expected.points)
    assert max(p[1] for p in mesh.GetPointsAttr().Get()) > 3.6
    reference = UsdGeom.Imageable(stage.GetPrimAtPath('/World/PhysicsRoad/Debug/ReferenceCenterline'))
    assert reference.GetVisibilityAttr().Get() == 'invisible'
    assert stage.GetPrimAtPath('/World/PhysicsRoad/Static/LaneDivider0')
    assert view.static_layer.ExportToString() == before
    assert all(not prim.HasAPI(UsdPhysics.CollisionAPI) for prim in stage.Traverse())
    view.update(state)
    default = route_ribbon(road, 10, 35, width_m=.16, z_m=.07)
    for actual, wanted in zip(mesh.GetPointsAttr().Get(), default.points):
        assert tuple(actual) == pytest.approx(wanted, abs=2e-6)
    assert reference.GetVisibilityAttr().Get() == 'invisible'
    for bad in (0, -1, math.nan):
        with pytest.raises(ValueError): view.update(state, preview_distance_m=bad)


@pytest.mark.parametrize('remaining_m,faces', [(2., 4), (1.5, 3), (.5, 1), (0., 0)])
def test_usd_preview_shrinks_to_short_typed_arrays_at_endpoint(routes, remaining_m, faces):
    stage = _usd_stage()
    from pxr import UsdGeom, Vt
    route = routes['straight100']
    view = author_physics_road(stage, route)
    static_before = view.static_layer.ExportToString()
    view.update(dict(position_m=[route.length_m - remaining_m, 0, 1]))
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/World/PhysicsRoad/Debug/UpcomingPath'))
    counts = mesh.GetFaceVertexCountsAttr().Get()
    assert isinstance(counts, Vt.IntArray)
    assert isinstance(mesh.GetFaceVertexIndicesAttr().Get(), Vt.IntArray)
    assert isinstance(mesh.GetPointsAttr().Get(), Vt.Vec3fArray)
    if faces:
        assert len(counts) == faces
        assert list(counts) == [4] * faces
        assert len(mesh.GetFaceVertexIndicesAttr().Get()) == 4 * faces
        assert mesh.GetVisibilityAttr().Get() == 'inherited'
    else:
        assert mesh.GetVisibilityAttr().Get() == 'invisible'
    assert view.static_layer.ExportToString() == static_before
