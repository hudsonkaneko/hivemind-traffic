import unittest
try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None
from usd.full_perimeter_lidar_rig import author_end_lidars, end_lidar_specs, PROFILE, SENSORS_PATH
from usd.perimeter_lidar_rig import author_perimeter_lidars


@unittest.skipIf(Usd is None, 'Requires isolated USD environment')
class FullPerimeterLidarTests(unittest.TestCase):
    def test_front_rear_mounts_preserve_side_sensors(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, '/Vehicle/Chassis')
        UsdGeom.Scope.Define(stage, SENSORS_PATH)
        author_perimeter_lidars(stage)
        side_before = str(stage.GetPrimAtPath(SENSORS_PATH+'/LidarLeftNear').GetAttribute('xformOp:translate').Get())
        author_end_lidars(stage)
        self.assertEqual(str(stage.GetPrimAtPath(SENSORS_PATH+'/LidarLeftNear').GetAttribute('xformOp:translate').Get()), side_before)
        self.assertEqual(len([p for p in stage.GetPrimAtPath(SENSORS_PATH).GetChildren() if p.IsA(UsdGeom.Xform)]), 4)
        for spec in end_lidar_specs():
            prim = stage.GetPrimAtPath(SENSORS_PATH+'/'+spec['name'])
            matrix = UsdGeom.Xformable(prim).GetLocalTransformation()
            self.assertEqual(matrix.ExtractTranslation(), Gf.Vec3d(*spec['translation_m']))
            self.assertEqual(matrix.ExtractRotationMatrix(), Gf.Matrix3d(1))
            self.assertEqual(prim.GetCustomDataByKey('runtimeProfile'), PROFILE)
            self.assertFalse(stage.GetPrimAtPath(str(prim.GetPath())+'/Scan'))
            housing = stage.GetPrimAtPath(str(prim.GetPath())+'Housing')
            self.assertFalse(housing.HasAPI(UsdPhysics.CollisionAPI))
            self.assertFalse(housing.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertFalse(stage.GetPrimAtPath(str(prim.GetPath())+'Support'))
            for actual, expected in zip(housing.GetAttribute('xformOp:scale').Get(), (.12, .12, .1)):
                self.assertAlmostEqual(actual, expected)
        with self.assertRaisesRegex(ValueError, 'already exist'):
            author_end_lidars(stage)

    def test_missing_side_rig_rejected(self):
        with self.assertRaisesRegex(ValueError, 'existing side'):
            author_end_lidars(Usd.Stage.CreateInMemory())


if __name__ == '__main__':
    unittest.main()
