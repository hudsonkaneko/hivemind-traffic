import unittest
try:
    from pxr import Usd,UsdGeom
except ImportError:
    Usd=None
from usd.vehicle_sensor_rig import author_sensor_rig,sensor_paths
from usd.seven_camera_rig import author_seven_camera_rig
from usd.roof_lidar_rig import author_roof_lidar,ROOF_HEIGHT_M


@unittest.skipIf(Usd is None,'Requires isolated USD environment')
class RoofLidarTests(unittest.TestCase):
    def test_versioned_paths_and_clearance(self):
        stage=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage,'/Vehicle/Chassis')
        author_sensor_rig(stage,'/Vehicle/Chassis')
        author_seven_camera_rig(stage,'/Vehicle/Chassis')
        author_roof_lidar(stage)
        paths=sensor_paths('/Vehicle/Chassis',lidar_name='LidarRoof')
        mount=stage.GetPrimAtPath(paths['lidar_mount'])
        self.assertTrue(mount)
        self.assertFalse(stage.GetPrimAtPath('/Vehicle/Chassis/Sensors/LidarFront'))
        self.assertEqual(UsdGeom.Xformable(mount).GetLocalTransformation().ExtractTranslation()[2],ROOF_HEIGHT_M)
        self.assertEqual(len([p for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]),7)
        self.assertTrue(sensor_paths('/Vehicle/Chassis')['lidar'].endswith('LidarFront/Scan'))
        for p in stage.Traverse():
            if p.GetName().endswith(('Housing','Support')):
                bound=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default']).ComputeWorldBound(p).ComputeAlignedRange()
                self.assertLess(bound.GetMax()[2],ROOF_HEIGHT_M)


if __name__=='__main__':unittest.main()
