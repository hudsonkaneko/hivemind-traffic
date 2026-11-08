import unittest
try:
    from pxr import Gf, Usd, UsdGeom
except ImportError:
    Usd = None
from usd.near_field_sensor_rig import author_profile_contract, inspect_sensor_layout, PROFILE, SENSORS_PATH


@unittest.skipIf(Usd is None, 'Requires isolated USD environment')
class NearFieldSourceTests(unittest.TestCase):
    def setUp(self):
        from usd.vehicle_sensor_rig import author_sensor_rig
        from usd.seven_camera_rig import author_seven_camera_rig, camera_specs
        from usd.roof_lidar_rig import author_roof_lidar
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(self.stage, '/Vehicle/Chassis')
        author_sensor_rig(self.stage, '/Vehicle/Chassis')
        author_seven_camera_rig(self.stage, '/Vehicle/Chassis')
        author_roof_lidar(self.stage)
        for prim in list(self.stage.GetPrimAtPath(SENSORS_PATH).GetChildren()):
            if prim.GetName().startswith('Camera') and prim.GetName().endswith('Support'):
                self.stage.RemovePrim(prim.GetPath())
        for spec in camera_specs():
            housing = UsdGeom.Cube.Define(self.stage, SENSORS_PATH+'/'+spec['name']+'Housing').GetPrim()
            housing.SetCustomDataByKey('bodySurfaceAnchor', Gf.Vec3d(1, 0, 0))

    def test_profile_changes_no_mounts_or_optics(self):
        before = inspect_sensor_layout(self.stage)
        author_profile_contract(self.stage)
        self.assertEqual(inspect_sensor_layout(self.stage), before)
        mount = self.stage.GetPrimAtPath(SENSORS_PATH+'/LidarRoof')
        self.assertEqual(mount.GetCustomDataByKey('runtimeProfile'), PROFILE)
        self.assertFalse(self.stage.GetPrimAtPath(SENSORS_PATH+'/LidarRoof/Scan'))

    def test_reject_stalks_missing_cameras_and_runtime_scan(self):
        for path, message in [('CameraFrontSupport', 'stalks'), ('LidarRoof/Scan', 'Runtime LiDAR')]:
            UsdGeom.Xform.Define(self.stage, SENSORS_PATH+'/'+path)
            with self.assertRaisesRegex(ValueError, message):
                inspect_sensor_layout(self.stage)
            self.stage.RemovePrim(SENSORS_PATH+'/'+path)
        self.stage.RemovePrim(SENSORS_PATH+'/CameraRearTele')
        with self.assertRaisesRegex(ValueError, 'seven'):
            inspect_sensor_layout(self.stage)

    def test_reject_missing_body_anchor(self):
        self.stage.GetPrimAtPath(SENSORS_PATH+'/CameraFrontHousing').ClearCustomDataByKey('bodySurfaceAnchor')
        with self.assertRaisesRegex(ValueError, 'body-attached'):
            inspect_sensor_layout(self.stage)


if __name__ == '__main__':
    unittest.main()
