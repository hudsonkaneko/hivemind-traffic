"""Replay coordinate, clip and layered-scene regression checks; no simulator."""
from pathlib import Path
import json
import math
import subprocess
import sys
import tempfile
import unittest

from scripts.render_i110_merge_replay import (
    author_scene, elevation, road_quads, select_frames, vehicle_path, screenshot_samples,
    prepared_main_spec, sha256,
)


class MergeGeometryTests(unittest.TestCase):
    def setUp(self):
        self.lanes = {'road_0': {'width': 3.2, 'points': [(0., 0., 2.), (0., 20., 6.)]}}

    def test_elevation_uses_lane_projection_or_recorded_z(self):
        self.assertAlmostEqual(elevation({'x': 1., 'y': 10., 'lane_id': 'road_0'}, self.lanes), 4.)
        self.assertEqual(elevation({'x': 1., 'y': 10., 'z': 9.}, self.lanes), 9.)

    def test_road_vertices_keep_elevation_and_width(self):
        quads = road_quads(self.lanes['road_0'], 0., 3.2)
        self.assertEqual(quads[0], [(-1.6, 0., 2.), (1.6, 0., 2.), (1.6, 20., 6.), (-1.6, 20., 6.)])
        dashes = road_quads(self.lanes['road_0'], 1.6, .1, True)
        self.assertEqual(len(dashes), 3)
        self.assertAlmostEqual(dashes[0][-1][1], 3.)

    def test_window_and_budget_preserve_source_timestamps(self):
        data = {'frames': [{'time': n/10, 'vehicles': []} for n in range(400)],
                'clip_start_time': 3., 'clip_end_time': 4.}
        self.assertEqual(len(select_frames(data)), 11)
        with self.assertRaises(ValueError): select_frames(data, 0., 39.)
        with self.assertRaises(ValueError): select_frames(data, 50., 51.)

    def test_identifiers_are_deterministic_legal_and_distinct(self):
        self.assertEqual(vehicle_path('agent_2'), '/World/Vehicles/vehicle_6167656e745f32')
        self.assertNotEqual(vehicle_path('vehicle-1'), vehicle_path('vehicle_1'))

    def test_merge_screenshots_use_event_time_not_midpoint(self):
        frames = [{'time': n/10} for n in range(1, 111)]
        samples = dict(screenshot_samples(frames, [{'time':3.0}, {'time':9.7}]))
        self.assertEqual(frames[samples['merge']]['time'], 3.0)
        self.assertEqual(frames[samples['merge-2']]['time'], 9.7)
        self.assertIn('midpoint', dict(screenshot_samples(frames, [])))

    def test_composed_scene_retains_pose_and_reference(self):
        try:
            from pxr import Usd, UsdGeom
        except ImportError:
            self.skipTest('USD runtime unavailable; execute this test in the existing ovrtx environment')
        vehicle = dict(id='agent_2', x=0., y=10., heading=0., length=4.8, width=1.8, lane_id='road_0')
        frames = [{'time': 3., 'vehicles': [vehicle]}, {'time': 3.1, 'vehicles': []}]
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]/'outputs') as directory:
            folder = Path(directory)
            author_scene(folder, frames, self.lanes, 'agent_2', {'agent_2'})
            stage = Usd.Stage.Open(str(folder/'world.usda'))
            self.assertFalse(stage.GetCompositionErrors())
            self.assertEqual(stage.GetDefaultPrim().GetPath().pathString, '/World')
            self.assertEqual(len(stage.GetRootLayer().subLayerPaths), 3)
            prim = stage.GetPrimAtPath(vehicle_path('agent_2'))
            self.assertEqual(tuple(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.).ExtractTranslation()), (0., 10., 4.))
            self.assertEqual(UsdGeom.Imageable(prim).GetVisibilityAttr().Get(1.), 'invisible')
            model = stage.GetPrimAtPath(vehicle_path('agent_2')+'/Model')
            self.assertTrue(model.HasAuthoredReferences())
            color = UsdGeom.Gprim(stage.GetPrimAtPath(str(model.GetPath())+'/Body')).GetDisplayColorAttr().Get()[0]
            for actual, expected in zip(color, (1., .36, .035)):
                self.assertAlmostEqual(actual, expected, places=6)


class PreparedMainReplayTests(unittest.TestCase):
    def setUp(self):
        try:
            from pxr import Usd
        except ImportError:
            self.skipTest('USD runtime unavailable; use the existing ovrtx environment')
        self.temporary = tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]/'outputs')
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.source = self.folder/'prepared'; (self.source/'assets').mkdir(parents=True)
        self.output = self.folder/'scene'; self.output.mkdir()
        self.lanes = {'road_0': dict(width=3.2, points=[(0., 0., 2.), (0., 20., 2.)])}
        self.asset = self.make_source()

    def make_source(self):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        geometry = Usd.Stage.CreateNew(str(self.source/'assets/geometry.usda'))
        root = UsdGeom.Xform.Define(geometry, '/Vehicle').GetPrim()
        geometry.SetDefaultPrim(root)
        Usd.ModelAPI(root).SetKind('component')
        UsdGeom.SetStageMetersPerUnit(geometry, 1.)
        UsdGeom.SetStageUpAxis(geometry, 'Z')
        chassis = UsdGeom.Xform.Define(geometry, '/Vehicle/Chassis')
        chassis.AddTranslateOp().Set(Gf.Vec3d(0., 0., 1.))
        UsdPhysics.RigidBodyAPI.Apply(chassis.GetPrim()).CreateRigidBodyEnabledAttr(True)
        chassis.GetPrim().CreateAttribute('physxVehicle:vehicleEnabled', Sdf.ValueTypeNames.Bool).Set(True)
        body = UsdGeom.Cube.Define(geometry, '/Vehicle/Chassis/Body')
        body.CreateSizeAttr(1.)
        body.AddTranslateOp().Set(Gf.Vec3d(-.1, 0., 0.))
        body.AddScaleOp().Set(Gf.Vec3f(5., 2., 1.6))
        UsdPhysics.CollisionAPI.Apply(body.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.FixedJoint.Define(geometry, '/Vehicle/Chassis/FrameJoint').CreateJointEnabledAttr(True)
        camera = UsdGeom.Camera.Define(geometry, '/Vehicle/Chassis/Sensors/CameraFront')
        camera.AddTranslateOp().Set(Gf.Vec3d(2.3, 0., 0.))
        UsdGeom.Xform.Define(geometry, '/Vehicle/Chassis/Sensors/LidarRoof').AddTranslateOp().Set(Gf.Vec3d(0., 0., 1.3))
        geometry.GetRootLayer().Save()
        entry = self.source/'assets/vehicle.usda'
        stage = Usd.Stage.CreateNew(str(entry))
        stage.GetRootLayer().subLayerPaths = ['./geometry.usda']
        stage.SetDefaultPrim(stage.GetPrimAtPath('/Vehicle'))
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, 'Z')
        stage.GetRootLayer().Save()
        model = dict(version='test-v14', body_collision_dimensions_m=[5., 2., 1.6],
                     body_collision_center_m=[-.1, 0., 0.], chassis_origin_in_v05_m=[-2.4, 0., 1.],
                     wheel_centers_m={'FL': [1.5, 1., -.6], 'FR': [1.5, -1., -.6],
                                      'RL': [-1.5, 1., -.6], 'RR': [-1.5, -1., -.6]},
                     texture_status='fixture', license='fixture only')
        (self.source/'model.json').write_text(json.dumps(model), encoding='utf-8')
        return entry

    def frames(self):
        return [dict(time=index/10., vehicles=[
            dict(id='agent_0', x=20., y=10., heading=heading, length=4.8, width=1.8, lane_id='road_0'),
            dict(id='native_0', x=30., y=10., heading=heading, length=4.8, width=1.8, lane_id='road_0')])
                for index, heading in enumerate((0., 90., 180., 270.))]

    def test_cardinal_poses_keep_unscaled_prepared_body_at_front_bumper(self):
        from pxr import Gf, Usd, UsdGeom
        frames = self.frames()
        author_scene(self.output, frames, self.lanes, 'agent_0', {'agent_0'}, simready_main=self.asset)
        stage = Usd.Stage.Open(str(self.output/'world.usda'))
        self.assertFalse(stage.GetCompositionErrors())
        path = vehicle_path('agent_0')+'/Model'
        model = UsdGeom.Xformable(stage.GetPrimAtPath(path))
        self.assertTrue(model.GetPrim().HasAuthoredReferences())
        self.assertFalse(any(op.GetOpType() == UsdGeom.XformOp.TypeScale for op in model.GetOrderedXformOps()))
        for index, heading in enumerate((0., 90., 180., 270.)):
            chassis = UsdGeom.Xformable(stage.GetPrimAtPath(path+'/Chassis')).ComputeLocalToWorldTransform(index)
            front = chassis.Transform(Gf.Vec3d(2.4, 0., -1.))
            for actual, expected in zip(front, (20., 10., 2.)):
                self.assertAlmostEqual(actual, expected, places=6)
            forward = chassis.TransformDir(Gf.Vec3d(1., 0., 0.))
            for actual, expected in zip(forward, (math.sin(math.radians(heading)), math.cos(math.radians(heading)), 0.)):
                self.assertAlmostEqual(actual, expected, places=6)
        self.assertTrue(stage.GetPrimAtPath(path+'/Chassis/Sensors/CameraFront'))
        self.assertTrue(stage.GetPrimAtPath(path+'/Chassis/Sensors/LidarRoof'))
        self.assertFalse(stage.GetPrimAtPath(path+'/Body'))
        self.assertTrue(stage.GetPrimAtPath(vehicle_path('native_0')+'/Model/Body'))

    def test_physics_disabled_only_in_composed_replay_and_sources_unchanged(self):
        from pxr import Usd, UsdPhysics
        hashes = {p: sha256(p) for p in self.source.rglob('*') if p.is_file()}
        author_scene(self.output, self.frames(), self.lanes, 'agent_0', {'agent_0'}, simready_main=self.source)
        source = Usd.Stage.Open(str(self.asset))
        self.assertTrue(UsdPhysics.RigidBodyAPI(source.GetPrimAtPath('/Vehicle/Chassis')).GetRigidBodyEnabledAttr().Get())
        replay = Usd.Stage.Open(str(self.output/'world.usda'))
        root = vehicle_path('agent_0')+'/Model/Chassis'
        self.assertFalse(UsdPhysics.RigidBodyAPI(replay.GetPrimAtPath(root)).GetRigidBodyEnabledAttr().Get())
        self.assertFalse(replay.GetPrimAtPath(root).GetAttribute('physxVehicle:vehicleEnabled').Get())
        self.assertFalse(UsdPhysics.CollisionAPI(replay.GetPrimAtPath(root+'/Body')).GetCollisionEnabledAttr().Get())
        self.assertFalse(UsdPhysics.Joint(replay.GetPrimAtPath(root+'/FrameJoint')).GetJointEnabledAttr().Get())
        self.assertEqual(hashes, {p: sha256(p) for p in hashes})
        manifest = json.loads((self.output/'prepared-main-manifest.json').read_text())
        self.assertEqual(manifest['front_bumper_to_chassis_m'], 2.4)
        self.assertEqual(len(manifest['camera_prims']), 1)
        self.assertEqual(len(manifest['sensor_mount_prims']), 1)
        self.assertEqual(len(manifest['disabled_physics_attributes']), 4)
        self.assertTrue(manifest['source_unchanged'])
        self.assertFalse(manifest['sensor_capture'])
        self.assertFalse(manifest['physics_control'])
        self.assertFalse(Path(manifest['reference']).is_absolute())
        self.assertIn('references', (self.output/'prepared-main.usda').read_text())

    def test_fresh_process_reopens_relative_composition_from_another_directory(self):
        author_scene(self.output, self.frames(), self.lanes, 'agent_0', {'agent_0'}, simready_main=self.asset)
        command = [sys.executable, '-c',
            'from pxr import Usd,UsdUtils,Sdf; import sys; s=Usd.Stage.Open(sys.argv[1]); '
            'assert s and not s.GetCompositionErrors(); '
            'assert not UsdUtils.ComputeAllDependencies(Sdf.AssetPath(sys.argv[1]))[2]; '
            'assert s.GetPrimAtPath(sys.argv[2])',
            str(self.output/'world.usda'), vehicle_path('agent_0')+'/Model/Chassis/Sensors/CameraFront']
        subprocess.run(command, cwd=self.source, check=True, capture_output=True, text=True, timeout=30)

    def test_missing_asset_and_wrong_coordinate_contract_fail(self):
        with self.assertRaisesRegex(ValueError, 'requires a prepared'):
            prepared_main_spec(self.folder/'missing.usda')
        model = json.loads((self.source/'model.json').read_text())
        model['chassis_origin_in_v05_m'][0] = -1.
        (self.source/'model.json').write_text(json.dumps(model))
        with self.assertRaisesRegex(ValueError, 'coordinate contract'):
            prepared_main_spec(self.asset)

    def test_source_root_scale_is_rejected_without_silently_rescaling(self):
        from pxr import Gf, Usd, UsdGeom
        source = Usd.Stage.Open(str(self.asset))
        UsdGeom.Xformable(source.GetDefaultPrim()).AddScaleOp().Set(Gf.Vec3f(.01))
        source.GetRootLayer().Save()
        with self.assertRaisesRegex(ValueError, 'extra transform or scale'):
            author_scene(self.output, self.frames(), self.lanes, 'agent_0', {'agent_0'}, simready_main=self.asset)


if __name__ == '__main__':
    unittest.main()
