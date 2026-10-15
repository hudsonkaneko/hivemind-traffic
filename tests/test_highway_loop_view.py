"""USD-only camera tests; screenshots separately qualify rendered framing."""
import math
from types import SimpleNamespace
import unittest

try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError:
    Usd = None

from usd.physical_scene import PATHS
from visualization.highway_loop_view import HighwayLoopView


class CircleFixture:
    def evaluate(self, station_m):
        angle=station_m/500.
        return SimpleNamespace(position_xy=(500*math.cos(angle),500*math.sin(angle)))


@unittest.skipIf(Usd is None,'USD authoring runtime required')
class HighwayLoopViewTests(unittest.TestCase):
    def setUp(self):
        self.stage=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(self.stage,'/World')
        self.stage.SetDefaultPrim(self.stage.GetPrimAtPath('/World'))
        UsdGeom.SetStageUpAxis(self.stage,'Z')
        self.view=HighwayLoopView(self.stage,CircleFixture())
        self.state=dict(position_m=[500.,0.,1.],yaw_rad=math.pi/2)

    def test_follow_camera_tracks_arbitrary_world_pose(self):
        for angle in (0.,1.,2.,3.,4.,5.,6.):
            state=dict(position_m=[500*math.cos(angle),500*math.sin(angle),1.],yaw_rad=angle+math.pi/2)
            self.assertEqual(self.view.update(state,angle*500,'follow'),PATHS.follow_camera)
            report=self.view.camera_check()
            self.assertTrue(report['passed'],report)
            self.assertLess(report['position_error_m'],1e-8)
            self.assertLess(report['direction_error'],1e-8)
            self.assertAlmostEqual(math.dist(report['actual_position_m'],state['position_m']),math.hypot(14,9))

    def test_root_override_cannot_freeze_live_camera(self):
        self.view.update(self.state,0.,'follow')
        root_before=self.stage.GetRootLayer().ExportToString()
        with Usd.EditContext(self.stage,self.stage.GetRootLayer()):
            camera=UsdGeom.Xformable(self.stage.GetPrimAtPath(PATHS.follow_camera))
            op=camera.AddTranslateOp(opSuffix='viewportRootOverride')
            op.Set(Gf.Vec3d(0,0,1000))
            camera.SetXformOpOrder([op],resetXformStack=True)
        requested=dict(position_m=[400.,300.,.98],yaw_rad=2.)
        self.view.update(requested,200.,'follow')
        self.assertTrue(self.view.camera_check()['passed'])
        self.assertNotEqual(root_before,self.stage.GetRootLayer().ExportToString())

    def test_session_competitor_is_detected_then_replaced(self):
        self.view.update(self.state,0.,'follow')
        with Usd.EditContext(self.stage,self.stage.GetSessionLayer()):
            camera=UsdGeom.Xformable(self.stage.GetPrimAtPath(PATHS.follow_camera))
            op=camera.AddTranslateOp(opSuffix='viewportSessionOverride')
            op.Set(Gf.Vec3d(0,0,1000))
            camera.SetXformOpOrder([op],resetXformStack=True)
        self.assertFalse(self.view.camera_check()['passed'])
        self.view.update(self.state,0.,'follow')
        self.assertTrue(self.view.camera_check()['passed'])

    def test_camera_layer_policy_and_saved_preview_pose(self):
        root_before=self.stage.GetRootLayer().ExportToString()
        self.view.update(self.state,0.,'follow')
        self.assertEqual(root_before,self.stage.GetRootLayer().ExportToString())
        self.assertTrue(self.stage.GetSessionLayer().GetPropertyAtPath(
            PATHS.follow_camera+'.xformOp:transform:highwayLoopCamera'))
        self.assertTrue(self.view.dynamic_layer.GetPropertyAtPath(
            PATHS.follow_camera+'.xformOp:transform:highwayLoopCamera'))
        reopened=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(reopened,'/World')
        reopened.GetRootLayer().subLayerPaths=[self.view.dynamic_layer.identifier]
        matrix=UsdGeom.Xformable(reopened.GetPrimAtPath(PATHS.follow_camera)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertLess((matrix.ExtractTranslation()-Gf.Vec3d(*self.view.camera_check()['requested_position_m'])).GetLength(),1e-8)

    def test_car_bounds_are_in_follow_camera_frustum(self):
        self.view.update(self.state,0.,'follow')
        camera=UsdGeom.Camera(self.stage.GetPrimAtPath(PATHS.follow_camera)).GetCamera(Usd.TimeCode.Default())
        projection=camera.frustum.ComputeViewMatrix()*camera.frustum.ComputeProjectionMatrix()
        ndc=[]
        # Physical chassis 4.8m long, 1.8m wide, 1.4m high, pointing +Y here.
        for dx in (-.9,.9):
            for dy in (-2.4,2.4):
                for dz in (-.7,.7):
                    point=projection.Transform(Gf.Vec3d(500+dx,dy,1+dz))
                    ndc.append(point)
                    self.assertLess(abs(point[0]),.95)
                    self.assertLess(abs(point[1]),.95)
        # Bounds fill a meaningful but non-clipped portion of the image.
        self.assertGreater(max(p[1] for p in ndc)-min(p[1] for p in ndc),.25)

    def test_overview_mode_and_invalid_input(self):
        self.assertEqual(self.view.update(self.state,0.,'overview'),PATHS.overview_camera)
        self.assertTrue(self.view.camera_check()['passed'])
        self.assertEqual(self.view.camera_check()['requested_position_m'],[0.,-850.,950.])
        with self.assertRaises(ValueError):
            self.view.update(self.state,0.,'invalid')
        with self.assertRaises(ValueError):
            self.view.update(dict(position_m=[0,0,float('nan')],yaw_rad=0),0.,'follow')


if __name__=='__main__':
    unittest.main()
