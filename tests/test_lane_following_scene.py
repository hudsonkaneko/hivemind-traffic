"""Real OpenUSD composition tests; no SimulationApp, renderer or physics step."""
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from environments.lane_following_scene import (
    author_lane_view, bind_composed_vehicle, save_lane_scene,
    update_lane_view, validate_lane_scene,
)
from traffic.lane_geometry import LaneRoute, RouteSegment
from usd.physical_scene import PATHS, asset_hashes

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
except ImportError:
    Usd = None

ROOT = Path(__file__).resolve().parents[1]


def factory_author(stage, *args, **kwargs):
    car = UsdGeom.Xform.Define(stage, '/World/Car_0')
    car.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
    UsdPhysics.RigidBodyAPI.Apply(car.GetPrim())
    UsdPhysics.MassAPI.Apply(car.GetPrim()).CreateMassAttr(1800.)
    UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisRender')
    collider = UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisCollision')
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    wheels = []
    for name in ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel'):
        path = '/World/Car_0/' + name
        wheels.append(path)
        wheel = UsdGeom.Xform.Define(stage, path)
        wheel.GetPrim().CreateRelationship('physxVehicleWheelAttachment:wheel').SetTargets(['/World/FrontWheel'])
        wheel.GetPrim().CreateRelationship('physxVehicleWheelAttachment:collisionGroup').SetTargets(['/World/VehicleGroundQueryGroup'])
        UsdGeom.Cylinder.Define(stage, path + '/Collision').CreateAxisAttr('Y')
    resource = UsdGeom.Scope.Define(stage, '/World/FrontWheel')
    resource.GetPrim().CreateAttribute('physxVehicleWheel:radius', Sdf.ValueTypeNames.Float).Set(.35)
    group = UsdPhysics.CollisionGroup.Define(stage, '/World/VehicleGroundQueryGroup')
    group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Car_0/ChassisCollision'])
    group.CreateFilteredGroupsRel().SetTargets(['/World/VehicleGroundQueryGroup'])
    UsdPhysics.Scene.Define(stage, '/World/PhysicsScene').CreateGravityMagnitudeAttr(9.81)
    UsdGeom.Mesh.Define(stage, '/World/GroundPlane')
    UsdLux.SphereLight.Define(stage, '/World/SphereLight')
    kwargs['vehiclePathsOut'].append('/World/Car_0')
    kwargs['wheelAttachmentPathsOut'].append(wheels)


class FakeVehicle:
    def __init__(self, stage, hz, scene_directory):
        from usd.physical_scene import compose_factory_vehicle
        factory = SimpleNamespace(__file__=__file__, DRIVE_NONE=0,
            AxesIndices=lambda *a: a, create4WheeledCarsScenario=factory_author)
        self.metadata = compose_factory_vehicle(stage, factory, hz, scene_directory)
        self.stage, self.path, self.wheel_paths = stage, PATHS.chassis, PATHS.wheels


@unittest.skipIf(Usd is None, 'OpenUSD environment required')
class LaneSceneTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='lane-scene-test-', dir=ROOT / 'outputs')
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / 'scene'
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(self.stage, '/World')
        UsdGeom.Xform.Define(self.stage, '/World/envs/env_0')
        physics = UsdPhysics.Scene.Define(self.stage, PATHS.physics)
        physics.CreateGravityMagnitudeAttr(9.8)  # The Lab root must stay strongest.
        physics.GetPrim().CreateAttribute('lab:solverMarker', Sdf.ValueTypeNames.Int).Set(73)
        self.route = LaneRoute('road', 7.2, (RouteSegment(100),))

    def compose(self, barrier=False):
        import pxr
        schema = SimpleNamespace(PhysxVehicleWheelControllerAPI=lambda p: p)
        with patch.dict(sys.modules, {'traffic.physx_vehicle': SimpleNamespace(PhysxVehicle=FakeVehicle)}), \
                patch.object(pxr, 'PhysxSchema', schema, create=True):
            vehicle = bind_composed_vehicle(self.stage, 120, self.directory)
        road, runtime = author_lane_view(self.stage, self.route, self.directory,
            diagnostic_contact_barrier=barrier)
        return vehicle, road, runtime

    def test_lab_solver_and_vehicle_handles_keep_composed_identity(self):
        vehicle, road, runtime = self.compose()
        self.assertIs(vehicle.stage, self.stage)
        self.assertEqual([str(p.GetPath()) for p in vehicle.controllers], PATHS.wheels)
        self.assertEqual(str(vehicle.body.GetPrim().GetPath()), PATHS.chassis)
        physics = self.stage.GetPrimAtPath(PATHS.physics)
        self.assertAlmostEqual(physics.GetAttribute('physics:gravityMagnitude').Get(), 9.8, places=6)
        self.assertEqual(physics.GetAttribute('lab:solverMarker').Get(), 73)
        report = validate_lane_scene(self.stage)
        self.assertEqual(report['rigid_bodies'], [PATHS.chassis])
        self.assertFalse(report['diagnostic_contact_barrier'])
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), runtime)

    def test_portable_save_reopen_and_live_camera_changes_leave_files_immutable(self):
        vehicle, road, runtime = self.compose()
        state = {'position_m': [0., 0., 1.], 'yaw_rad': 0.}
        self.assertEqual(update_lane_view(self.stage, road, runtime, state), PATHS.follow_camera)
        saved = save_lane_scene(self.stage, self.directory, road, runtime)
        entry = Usd.Stage.Open(saved['entry_path'])
        self.assertTrue(validate_lane_scene(entry)['passed'])
        self.assertFalse(entry.GetPrimAtPath('/World/envs'))
        self.assertTrue(entry.GetPrimAtPath(PATHS.vehicle).HasAuthoredReferences())
        before = asset_hashes(self.directory)
        state['position_m'] = [15., .1, 1.]
        update_lane_view(self.stage, road, runtime, state, mode='overview', target_xy=(20., 0.))
        self.assertEqual(asset_hashes(self.directory), before)
        for file in self.directory.rglob('*.usda'):
            self.assertNotIn(str(self.directory), file.read_text(encoding='utf-8'))
        code = "from pxr import Usd; s=Usd.Stage.Open(__import__('sys').argv[1]); assert s and not s.GetCompositionErrors(); assert s.GetPrimAtPath('/World/Vehicles/vehicle_ego/Chassis'); print('reopened')"
        result = subprocess.run([sys.executable, '-B', '-c', code, saved['entry_path']],
            cwd=self.temp.name, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('reopened', result.stdout)

    def test_contact_fixture_is_referenced_static_collider_and_recorded(self):
        vehicle, road, runtime = self.compose(barrier=True)
        report = validate_lane_scene(self.stage)
        self.assertTrue(report['diagnostic_contact_barrier'])
        barrier = self.stage.GetPrimAtPath(PATHS.barrier)
        self.assertTrue(barrier.HasAPI(UsdPhysics.CollisionAPI))
        self.assertFalse(barrier.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertEqual(barrier.GetMetadata('references').GetAppliedItems()[0].assetPath,
            './assets/diagnostic-contact-barrier.usda')
        matrix = UsdGeom.Xformable(barrier).ComputeLocalToWorldTransform(0)
        self.assertEqual(tuple(matrix.ExtractTranslation()), (8., 0., .75))
        entry = save_lane_scene(self.stage, self.directory, road, runtime)
        self.assertTrue(validate_lane_scene(Usd.Stage.Open(entry['entry_path']))['diagnostic_contact_barrier'])

    def test_validator_rejects_identity_extra_body_and_runtime_clone(self):
        self.compose()
        for mutation, message in [('identity', 'identity'), ('body', 'one native chassis'), ('clone', 'bookkeeping')]:
            with self.subTest(mutation=mutation):
                layer = Sdf.Layer.CreateAnonymous()
                self.stage.GetRootLayer().subLayerPaths.insert(0, layer.identifier)
                with Usd.EditContext(self.stage, layer):
                    if mutation == 'identity':
                        self.stage.GetPrimAtPath(PATHS.vehicle).SetCustomDataByKey('vehicle_id', 'other')
                    elif mutation == 'body':
                        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage, '/World/Environment/Extra').GetPrim())
                    else:
                        UsdGeom.Xform.Define(self.stage, '/World/envs/env_1')
                with self.assertRaisesRegex(ValueError, message):
                    validate_lane_scene(self.stage)
                self.stage.GetRootLayer().subLayerPaths.remove(layer.identifier)

    def test_validator_rejects_second_scene_unloaded_vehicle_and_unexpected_barrier(self):
        self.compose()
        for mutation, message in [('scene', 'one Lab physics'), ('unloaded', 'active/loaded'), ('barrier', 'Unexpected barrier')]:
            with self.subTest(mutation=mutation):
                layer = Sdf.Layer.CreateAnonymous()
                self.stage.GetRootLayer().subLayerPaths.insert(0, layer.identifier)
                with Usd.EditContext(self.stage, layer):
                    if mutation == 'scene':
                        UsdPhysics.Scene.Define(self.stage, '/World/Physics/Extra')
                    elif mutation == 'unloaded':
                        self.stage.GetPrimAtPath(PATHS.vehicle).SetActive(False)
                    else:
                        UsdGeom.Cube.Define(self.stage, PATHS.barrier)
                with self.assertRaisesRegex(ValueError, message):
                    validate_lane_scene(self.stage)
                self.stage.GetRootLayer().subLayerPaths.remove(layer.identifier)

    def test_bootstrap_refuses_existing_content_before_creating_assets(self):
        import pxr
        schema = SimpleNamespace(PhysxVehicleWheelControllerAPI=lambda p: p)
        UsdGeom.Xform.Define(self.stage, '/World/Foreign')
        with patch.dict(sys.modules, {'traffic.physx_vehicle': SimpleNamespace(PhysxVehicle=FakeVehicle)}), \
                patch.object(pxr, 'PhysxSchema', schema, create=True):
            with self.assertRaisesRegex(ValueError, 'fresh Lab project namespace'):
                bind_composed_vehicle(self.stage, 120, self.directory)
        self.assertFalse(self.directory.exists())


if __name__ == '__main__':
    unittest.main()
