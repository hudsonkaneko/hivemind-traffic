"""Source calibration/composition checks without launching Kit."""
import math
from pathlib import Path
import tempfile
import unittest
try:
    from pxr import Gf, Usd, UsdGeom
except ImportError:
    Usd = None
from usd.vehicle_sensor_rig import author_sensor_rig
from usd.seven_camera_rig import author_seven_camera_rig, camera_specs


@unittest.skipIf(Usd is None, "Requires isolated USD environment")
class SevenCameraTests(unittest.TestCase):
    def setUp(self):
        self.stage=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(self.stage,'/Vehicle/Chassis')
        self.stage.SetDefaultPrim(self.stage.GetPrimAtPath('/Vehicle'))
        author_sensor_rig(self.stage,'/Vehicle/Chassis')
        author_seven_camera_rig(self.stage,'/Vehicle/Chassis')

    def test_seven_calibrated_axes_and_optics(self):
        cameras=[p for p in self.stage.Traverse() if p.IsA(UsdGeom.Camera)]
        self.assertEqual(len(cameras),7)
        for spec in camera_specs():
            cam=UsdGeom.Camera(self.stage.GetPrimAtPath('/Vehicle/Chassis/Sensors/'+spec['name']))
            matrix=cam.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            yaw,pitch=map(math.radians,(spec['heading_deg'],spec['down_pitch_deg']))
            expected=Gf.Vec3d(math.cos(yaw)*math.cos(pitch),math.sin(yaw)*math.cos(pitch),-math.sin(pitch))
            self.assertTrue(Gf.IsClose(matrix.TransformDir(Gf.Vec3d(0,0,-1)),expected,1e-6))
            fov=math.degrees(2*math.atan(cam.GetHorizontalApertureAttr().Get()/2/cam.GetFocalLengthAttr().Get()))
            self.assertAlmostEqual(fov,spec['horizontal_fov_deg'],places=4)

    def test_payload_reload_and_body_inheritance(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]/'outputs') as temp:
            asset=Path(temp)/'asset.usda';self.stage.GetRootLayer().Export(str(asset))
            scene=Usd.Stage.CreateNew(str(Path(temp)/'world.usda'))
            prim=UsdGeom.Xform.Define(scene,'/World/Car').GetPrim();prim.GetPayloads().AddPayload('./asset.usda')
            scene.GetRootLayer().Save();scene=Usd.Stage.Open(str(Path(temp)/'world.usda'))
            self.assertFalse(scene.GetCompositionErrors())
            self.assertEqual(len([p for p in scene.Traverse() if p.IsA(UsdGeom.Camera)]),7)
            camera=UsdGeom.Xformable(scene.GetPrimAtPath('/World/Car/Chassis/Sensors/CameraRearTele'))
            before=camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            UsdGeom.Xformable(scene.GetPrimAtPath('/World/Car/Chassis')).AddTranslateOp().Set(Gf.Vec3d(10,0,0))
            after=camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            self.assertTrue(Gf.IsClose(after-before,Gf.Vec3d(10,0,0),1e-6))
            self.assertTrue(scene.GetPrimAtPath('/World/Car/Chassis/Sensors/LidarFront'))

    def test_horizontal_coverage_has_no_azimuth_gap(self):
        for azimuth in range(-180,180):
            self.assertTrue(any(abs((azimuth-s['heading_deg']+180)%360-180)<=s['horizontal_fov_deg']/2 for s in camera_specs()))

if __name__=='__main__':unittest.main()
