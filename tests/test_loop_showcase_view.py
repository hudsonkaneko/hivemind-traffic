"""USD-only showcase camera/path checks; not RTX screenshot qualification."""
import math
from types import SimpleNamespace
import unittest

try:
    from pxr import Gf, Usd, UsdGeom
except ImportError:
    Usd = None

from usd.physical_scene import PATHS
from visualization.loop_showcase_view import LoopShowcaseView


class CircleFixture:
    def evaluate(self, station_m):
        angle = station_m / 500.
        return SimpleNamespace(position_xy=(500 * math.cos(angle), 500 * math.sin(angle)))


def points_for_shift(shift_m, count=61):
    points = []
    for i in range(count):
        q = i / (count - 1)
        radius = 500 + shift_m * q ** 3 * (10 - 15 * q + 6 * q * q)
        angle = 120 * q / 500
        points.append((radius * math.cos(angle), radius * math.sin(angle), .06))
    return points


@unittest.skipIf(Usd is None, 'USD authoring runtime required')
class LoopShowcaseViewTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        world = UsdGeom.Xform.Define(self.stage, '/World')
        self.stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.SetStageUpAxis(self.stage, 'Z')
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.)
        # A referenced asset must remain untouched by overlay/camera updates.
        self.asset = Usd.Stage.CreateInMemory('unchanged-vehicle.usda')
        body = UsdGeom.Cube.Define(self.asset, '/VehicleBody')
        self.asset.SetDefaultPrim(body.GetPrim())
        UsdGeom.Xform.Define(self.stage, '/World/Vehicles')
        car = self.stage.DefinePrim('/World/Vehicles/vehicle_ego')
        car.GetReferences().AddReference(self.asset.GetRootLayer().identifier)
        self.view = LoopShowcaseView(self.stage, CircleFixture())
        self.state = dict(position_m=[500., 0., 1.], yaw_rad=math.pi / 2)

    def assert_points_equal(self, expected):
        actual = self.view.path.GetPointsAttr().Get()
        self.assertEqual(len(actual), len(expected))
        self.assertEqual(list(self.view.path.GetCurveVertexCountsAttr().Get()), [len(expected)])
        for actual_point, expected_point in zip(actual, expected):
            self.assertLess(math.dist(actual_point, expected_point), 5e-5)

    def test_projected_path_equals_current_planner_points_not_nominal_circle(self):
        points = points_for_shift(3.7)
        self.view.update_showcase(self.state, 0., 'follow', points)
        self.assert_points_equal(points)
        self.assertAlmostEqual(math.hypot(*self.view.path.GetPointsAttr().Get()[-1][:2]), 503.7, places=4)
        self.assertEqual(self.view.path.GetWidthsInterpolation(), 'constant')
        self.assertAlmostEqual(self.view.path.GetWidthsAttr().Get()[0], .2)

    def test_changed_path_and_vertex_count_replace_the_previous_overlay(self):
        self.view.update_showcase(self.state, 0., 'follow', points_for_shift(0))
        old = tuple(tuple(point) for point in self.view.path.GetPointsAttr().Get())
        changed = points_for_shift(7.4, count=41)
        self.view.update_showcase(self.state, 0., 'traffic', changed)
        self.assert_points_equal(changed)
        self.assertNotEqual(old, tuple(tuple(point) for point in self.view.path.GetPointsAttr().Get()))

    def test_follow_camera_world_pose_tracks_ego_at_arbitrary_heading(self):
        for angle in (0., .5, 1., 2., 3., 4., 5., 6.):
            x, y = 507.4 * math.cos(angle), 507.4 * math.sin(angle)
            yaw = angle + math.pi / 2
            state = dict(position_m=[x, y, .98], yaw_rad=yaw)
            self.assertEqual(self.view.update_showcase(state, angle * 500, 'follow', points_for_shift(3.7)), PATHS.follow_camera)
            report = self.view.camera_check()
            self.assertTrue(report['passed'], report)
            self.assertLess(math.dist(report['actual_position_m'],
                (x - 20 * math.cos(yaw), y - 20 * math.sin(yaw), 13.98)), 1e-8)
            self.assertLess(report['direction_error'], 1e-8)

    def test_camera_toggle_selects_expected_world_eyes_and_paths(self):
        expected = {
            'follow': (PATHS.follow_camera, (500., -20., 14.)),
            'traffic': (PATHS.follow_camera, (499., 29., 90.)),
            'overview': (PATHS.overview_camera, (0., -850., 950.)),
        }
        for mode in ('follow', 'traffic', 'overview', 'traffic', 'follow'):
            path, eye = expected[mode]
            self.assertEqual(self.view.update_showcase(self.state, 0., mode, points_for_shift(3.7)), path)
            report = self.view.camera_check()
            self.assertTrue(report['passed'], report)
            self.assertLess(math.dist(report['actual_position_m'], eye), 1e-8)
            self.assertEqual(report['mode'], mode)

    def test_updates_do_not_edit_static_root_or_referenced_vehicle_asset(self):
        root_before = self.stage.GetRootLayer().ExportToString()
        asset_before = self.asset.GetRootLayer().ExportToString()
        for index in range(30):
            state = dict(position_m=[500 * math.cos(index / 100), 500 * math.sin(index / 100), 1.],
                         yaw_rad=index / 100 + math.pi / 2)
            mode = ('follow', 'traffic', 'overview')[index % 3]
            self.view.update_showcase(state, index * 5, mode, points_for_shift(3.7 * (index % 3)))
        self.assertEqual(root_before, self.stage.GetRootLayer().ExportToString())
        self.assertEqual(asset_before, self.asset.GetRootLayer().ExportToString())
        self.assertTrue(self.view.dynamic_layer.GetPropertyAtPath('/World/Debug/Route/ProjectedPath.points'))
        self.assertFalse(self.stage.GetRootLayer().GetPropertyAtPath('/World/Debug/Route/ProjectedPath.points'))

    def test_session_competing_pose_is_detected_then_corrected_by_toggle(self):
        self.view.update_showcase(self.state, 0., 'follow', points_for_shift(3.7))
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            camera = UsdGeom.Xformable(self.stage.GetPrimAtPath(PATHS.follow_camera))
            op = camera.AddTranslateOp(opSuffix='competingViewport')
            op.Set(Gf.Vec3d(0, 0, 1000))
            camera.SetXformOpOrder([op], resetXformStack=True)
        self.assertFalse(self.view.camera_check()['passed'])
        self.view.update_showcase(self.state, 0., 'traffic', points_for_shift(3.7))
        self.assertTrue(self.view.camera_check()['passed'])

    def test_saved_dynamic_layer_reproduces_camera_and_changed_path_without_session(self):
        points = points_for_shift(3.7)
        self.view.update_showcase(self.state, 0., 'traffic', points)
        reopened = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(reopened, '/World')
        reopened.GetRootLayer().subLayerPaths = [self.view.dynamic_layer.identifier]
        camera = UsdGeom.Xformable(reopened.GetPrimAtPath(PATHS.follow_camera))
        eye = camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertLess(math.dist(eye, (499., 29., 90.)), 1e-8)
        line = UsdGeom.BasisCurves(reopened.GetPrimAtPath('/World/Debug/Route/ProjectedPath'))
        self.assertEqual(list(line.GetCurveVertexCountsAttr().Get()), [61])
        for actual, expected in zip(line.GetPointsAttr().Get(), points):
            self.assertLess(math.dist(actual, expected), 5e-5)

    def test_follow_camera_includes_full_size_ego_body(self):
        self.view.update_showcase(self.state, 0., 'follow', points_for_shift(3.7))
        camera = UsdGeom.Camera(self.stage.GetPrimAtPath(PATHS.follow_camera)).GetCamera(Usd.TimeCode.Default())
        matrix = camera.frustum.ComputeViewMatrix() * camera.frustum.ComputeProjectionMatrix()
        for dx in (-1.11, 1.11):
            for dy in (-2.5, 2.5):
                for dz in (-.81, .81):
                    point = matrix.Transform(Gf.Vec3d(500 + dx, dy, 1 + dz))
                    self.assertLess(abs(point[0]), .99)
                    self.assertLess(abs(point[1]), .99)


if __name__ == '__main__':
    unittest.main()
