"""Plain USD sensor mounting/composition tests; no RTX/physics claims."""
import tempfile
from pathlib import Path
import unittest

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

from usd.vehicle_sensor_rig import author_sensor_rig, sensor_paths, CAMERA_TRANSLATION

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(Usd is None, 'Plain USD runtime required')
class VehicleSensorRigTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.chassis = UsdGeom.Xform.Define(self.stage, '/Vehicle/Chassis')
        self.paths = author_sensor_rig(self.stage, '/Vehicle/Chassis')

    def test_optical_axes_and_mount_follow_chassis(self):
        camera = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.paths['camera']))
        transform = camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertTrue(Gf.IsClose(transform.TransformDir(Gf.Vec3d(0, 0, -1)), Gf.Vec3d(1, 0, 0), 1e-6))
        self.assertTrue(Gf.IsClose(transform.TransformDir(Gf.Vec3d(0, 1, 0)), Gf.Vec3d(0, 0, 1), 1e-6))
        self.chassis.AddTranslateOp().Set(Gf.Vec3d(10, 20, 30))
        transformed = camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(Gf.IsClose(transformed, Gf.Vec3d(10, 20, 30) + Gf.Vec3d(*CAMERA_TRANSLATION), 1e-6))

    def test_lidar_scan_reserved_for_runtime_and_housings_visual_only(self):
        self.assertTrue(self.stage.GetPrimAtPath(self.paths['lidar_mount']))
        self.assertFalse(self.stage.GetPrimAtPath(self.paths['lidar']))
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.paths['root'])):
            self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))
            self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))

    def test_duplicate_rig_rejected_and_no_missing_chassis(self):
        with self.assertRaises(ValueError):
            author_sensor_rig(self.stage, '/Vehicle/Chassis')
        with self.assertRaises(ValueError):
            author_sensor_rig(self.stage, '/Missing')

    def test_saved_asset_is_payloadable_and_preserves_mount(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'outputs') as directory:
            source = Path(directory) / 'vehicle.usda'
            self.stage.SetDefaultPrim(self.stage.GetPrimAtPath('/Vehicle'))
            self.stage.GetRootLayer().Export(str(source))
            scene = Usd.Stage.CreateNew(str(Path(directory) / 'world.usda'))
            prim = UsdGeom.Xform.Define(scene, '/World/Vehicle').GetPrim()
            prim.GetPayloads().AddPayload('./vehicle.usda')
            scene.GetRootLayer().Save()
            reopened = Usd.Stage.Open(str(Path(directory) / 'world.usda'), load=Usd.Stage.LoadNone)
            path = sensor_paths('/World/Vehicle/Chassis')['camera']
            self.assertFalse(reopened.GetPrimAtPath(path))
            reopened.Load()
            self.assertTrue(reopened.GetPrimAtPath(path).IsA(UsdGeom.Camera))
            self.assertTrue(reopened.GetPrimAtPath('/World/Vehicle').HasAuthoredPayloads())
            self.assertFalse(reopened.GetCompositionErrors())


if __name__ == '__main__':
    unittest.main()
