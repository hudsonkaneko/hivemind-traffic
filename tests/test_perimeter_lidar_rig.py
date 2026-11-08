import unittest
try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None
from usd.perimeter_lidar_rig import author_perimeter_lidars, lidar_specs, PROFILE, SENSORS_PATH


@unittest.skipIf(Usd is None, 'Requires isolated USD environment')
class PerimeterLidarTests(unittest.TestCase):
    def test_mounts_housings_and_chassis_inheritance(self):
        stage = Usd.Stage.CreateInMemory()
        chassis = UsdGeom.Xform.Define(stage, '/Vehicle/Chassis')
        UsdGeom.Scope.Define(stage, SENSORS_PATH)
        author_perimeter_lidars(stage)
        for spec in lidar_specs():
            prim = stage.GetPrimAtPath(SENSORS_PATH+'/'+spec['name'])
            mount = UsdGeom.Xformable(prim)
            self.assertEqual(mount.GetLocalTransformation().ExtractTranslation(), Gf.Vec3d(*spec['translation_m']))
            self.assertEqual(prim.GetCustomDataByKey('runtimeProfile'), PROFILE)
            self.assertFalse(stage.GetPrimAtPath(str(prim.GetPath())+'/Scan'))
            housing = stage.GetPrimAtPath(str(prim.GetPath())+'Housing')
            self.assertFalse(housing.HasAPI(UsdPhysics.CollisionAPI))
            self.assertFalse(housing.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertFalse(stage.GetPrimAtPath(str(prim.GetPath())+'Support'))
            bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default']).ComputeWorldBound(housing).ComputeAlignedRange()
            self.assertAlmostEqual(bounds.GetSize()[1], .12)
            self.assertGreater(abs(spec['translation_m'][1]), max(abs(bounds.GetMin()[1]), abs(bounds.GetMax()[1])))
        chassis.AddTranslateOp().Set(Gf.Vec3d(10, 0, 0))
        for spec in lidar_specs():
            mounted = UsdGeom.Xformable(stage.GetPrimAtPath(SENSORS_PATH+'/'+spec['name']))
            self.assertEqual(mounted.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[0], 10)
        with self.assertRaisesRegex(ValueError, 'already exist'):
            author_perimeter_lidars(stage)

    def test_missing_hierarchy_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Missing chassis'):
            author_perimeter_lidars(Usd.Stage.CreateInMemory())


if __name__ == '__main__':
    unittest.main()
