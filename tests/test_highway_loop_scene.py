"""USD-only packaging checks; these do not establish moving-tire performance."""
import hashlib
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux
except ImportError:
    Usd = None

from usd.highway_loop_scene import (
    GROUND_GROUP, HIGHWAY, QUERY_GROUP, ROAD_COLLIDER, TARMAC,
    install_highway_loop, save_highway_loop_scene, spawn_vehicle_on_loop,
    validate_highway_loop_scene,
)
from usd.physical_scene import PATHS, compose_factory_vehicle

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'highway_usd/_v02'


def _factory(stage, *args, **kwargs):
    """Minimal read-only stand-in for testing authored integration contracts."""
    car = UsdGeom.Xform.Define(stage, '/World/Car_0')
    car.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0, 0, 1))
    car.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
    UsdPhysics.RigidBodyAPI.Apply(car.GetPrim())
    UsdPhysics.MassAPI.Apply(car.GetPrim()).CreateMassAttr(1800.)
    UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisRender')
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisCollision').GetPrim())
    wheels = []
    for name in ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel'):
        path = '/World/Car_0/' + name
        wheels.append(path)
        wheel = UsdGeom.Xform.Define(stage, path).GetPrim()
        wheel.CreateRelationship('physxVehicleWheelAttachment:collisionGroup').SetTargets(['/World/VehicleGroundQueryGroup'])
        wheel.CreateRelationship('physxVehicleTire:frictionTable').SetTargets(['/World/WinterTireFrictionTable'])
        UsdGeom.Cylinder.Define(stage, path + '/Collision').CreateAxisAttr('Y')
    for name in ('VehicleGroundQueryGroup', 'GroundSurfaceCollisionGroup', 'VehicleWheelCollisionGroup', 'VehicleChassisCollisionGroup'):
        UsdPhysics.CollisionGroup.Define(stage, '/World/' + name)
    ground_group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath('/World/GroundSurfaceCollisionGroup'))
    ground_group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/GroundPlane/CollisionPlane'])
    ground_group.CreateFilteredGroupsRel().SetTargets(['/World/VehicleWheelCollisionGroup'])
    UsdPhysics.CollisionGroup(stage.GetPrimAtPath('/World/VehicleWheelCollisionGroup')).CreateFilteredGroupsRel().SetTargets(['/World/GroundSurfaceCollisionGroup'])
    UsdPhysics.CollisionGroup(stage.GetPrimAtPath('/World/VehicleGroundQueryGroup')).CreateFilteredGroupsRel().SetTargets(['/World/VehicleWheelCollisionGroup', '/World/VehicleChassisCollisionGroup'])
    UsdGeom.Mesh.Define(stage, '/World/GroundPlane')
    ground = UsdGeom.Plane.Define(stage, '/World/GroundPlane/CollisionPlane')
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    material = UsdShade.Material.Define(stage, '/World/TarmacMaterial')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateStaticFrictionAttr(.9)
    table = stage.DefinePrim('/World/WinterTireFrictionTable', 'PhysxVehicleTireFrictionTable')
    table.CreateRelationship('groundMaterials').SetTargets(['/World/TarmacMaterial'])
    table.CreateAttribute('frictionValues', Sdf.ValueTypeNames.FloatArray).Set([.75])
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(material, materialPurpose='physics')
    UsdPhysics.Scene.Define(stage, '/World/PhysicsScene').CreateGravityMagnitudeAttr(9.81)
    UsdLux.SphereLight.Define(stage, '/World/SphereLight')
    kwargs['vehiclePathsOut'].append('/World/Car_0')
    kwargs['wheelAttachmentPathsOut'].append(wheels)


@unittest.skipIf(Usd is None, 'USD authoring runtime required; use highway_usd/_v01/.venv')
class HighwayLoopSceneTests(unittest.TestCase):
    def setUp(self):
        outputs = ROOT / 'outputs'
        outputs.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix='usd-loop-unit-', dir=outputs)
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / 'scene'
        self.stage = Usd.Stage.CreateInMemory()
        factory = SimpleNamespace(__file__=__file__, DRIVE_NONE=0, AxesIndices=lambda *a: a,
                                  create4WheeledCarsScenario=_factory)
        compose_factory_vehicle(self.stage, factory, 120, self.directory)
        self.report = install_highway_loop(self.stage, self.directory, SOURCE)

    def test_composition_and_single_motion_owner(self):
        self.assertTrue(self.report['validation']['passed'])
        self.assertEqual(self.report['validation']['physics_scenes'], [PATHS.physics])
        self.assertFalse(self.stage.GetPrimAtPath(PATHS.ground).IsActive())
        self.assertTrue(self.stage.GetPrimAtPath(HIGHWAY).HasAuthoredReferences())
        self.assertTrue(self.stage.GetPrimAtPath(HIGHWAY).HasPayload())
        self.assertTrue(self.stage.GetPrimAtPath(HIGHWAY).IsLoaded())
        self.assertFalse(self.stage.GetPrimAtPath(HIGHWAY + '/Ground').HasAPI(UsdPhysics.CollisionAPI))
        self.assertEqual(Usd.ModelAPI(self.stage.GetPrimAtPath(HIGHWAY)).GetKind(), 'component')

    def test_tire_material_and_ground_query_membership(self):
        road = self.stage.GetPrimAtPath(ROAD_COLLIDER)
        bound, _ = UsdShade.MaterialBindingAPI(road).ComputeBoundMaterial('physics')
        self.assertEqual(str(bound.GetPath()), TARMAC)
        group = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath(GROUND_GROUP))
        self.assertEqual(group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(), [Sdf.Path(ROAD_COLLIDER)])
        self.assertEqual(group.GetFilteredGroupsRel().GetTargets(),
                         [Sdf.Path('/World/Physics/Resources/VehicleWheelCollisionGroup')])
        for wheel in PATHS.wheels:
            self.assertEqual(self.stage.GetPrimAtPath(wheel).GetRelationship('physxVehicleWheelAttachment:collisionGroup').GetTargets(),
                             [Sdf.Path(QUERY_GROUP)])

    def test_source_mesh_is_exact_and_sources_remain_unchanged(self):
        original = Usd.Stage.Open(str(SOURCE / 'highway_v02.usda'))
        mesh = UsdGeom.Mesh(original.GetPrimAtPath('/World/Physics/RoadCollider'))
        current = UsdGeom.Mesh(self.stage.GetPrimAtPath(ROAD_COLLIDER))
        for attribute in ('points', 'faceVertexIndices', 'faceVertexCounts', 'extent', 'orientation'):
            self.assertEqual(current.GetPrim().GetAttribute(attribute).Get(), mesh.GetPrim().GetAttribute(attribute).Get())
        for name, expected in self.report['source_hashes'].items():
            self.assertEqual(hashlib.sha256((SOURCE / name).read_bytes()).hexdigest(), expected)

    def test_cross_subtree_navigation_and_appearance_relationships_remap(self):
        rel = self.stage.GetPrimAtPath(HIGHWAY + '/RoadNetwork').GetRelationship('road:collision')
        self.assertEqual(rel.GetTargets(), [Sdf.Path(ROAD_COLLIDER)])
        self.assertTrue(validate_highway_loop_scene(self.stage)['passed'])

    def test_spawn_has_correct_yaw_height_and_no_asset_mutation(self):
        before = {p.name: p.read_bytes() for p in (self.directory / 'assets').iterdir()}
        for angle in (0, math.pi / 2, math.pi, -math.pi / 2, -.04):
            meta = spawn_vehicle_on_loop(self.stage, angle)
            self.assertEqual(meta['position_m'][2], 1.)
            matrix = UsdGeom.Xformable(self.stage.GetPrimAtPath(PATHS.chassis)).GetLocalTransformation()
            pos = matrix.ExtractTranslation()
            forward = matrix.TransformDir(Gf.Vec3d(1, 0, 0))
            self.assertAlmostEqual(pos[0], 500 * math.cos(angle), delta=.00004)
            self.assertAlmostEqual(pos[1], 500 * math.sin(angle), delta=.00004)
            self.assertAlmostEqual(forward[0], -math.sin(angle), delta=.000001)
            self.assertAlmostEqual(forward[1], math.cos(angle), delta=.000001)
        self.assertEqual(before, {p.name: p.read_bytes() for p in (self.directory / 'assets').iterdir()})

    def test_invalid_spawn_is_rejected(self):
        for options in ({'angle_rad': float('nan')}, {'angle_rad': 0, 'radius_m': 0},
                        {'angle_rad': 0, 'center_xy': (1,)}, {'angle_rad': 0, 'center_xy': (0, float('inf'))}):
            with self.assertRaises(ValueError):
                spawn_vehicle_on_loop(self.stage, **options)

    def test_payload_unload_fails_readiness_and_reload_recovers(self):
        self.stage.Unload(HIGHWAY)
        # Collision opinions survive unloading the separately payloaded render
        # content, but driving is still prohibited until all content is loaded.
        self.assertTrue(self.stage.GetPrimAtPath(ROAD_COLLIDER))
        with self.assertRaisesRegex(ValueError, 'Missing/unloaded'):
            validate_highway_loop_scene(self.stage)
        self.stage.Load(HIGHWAY)
        self.assertTrue(validate_highway_loop_scene(self.stage)['passed'])

    def test_duplicate_scene_or_enabled_infinite_plane_is_rejected(self):
        self.stage.GetPrimAtPath(PATHS.ground).SetActive(True)
        with self.assertRaisesRegex(ValueError, 'infinite plane'):
            validate_highway_loop_scene(self.stage)
        self.stage.GetPrimAtPath(PATHS.ground).SetActive(False)
        UsdPhysics.Scene.Define(self.stage, '/World/Physics/SecondScene')
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            validate_highway_loop_scene(self.stage)

    def test_save_reopen_from_other_working_directory_and_relocation(self):
        spawn_vehicle_on_loop(self.stage, -.04)
        dynamic = Sdf.Layer.CreateAnonymous('loop-preview.usda')
        self.stage.GetRootLayer().subLayerPaths.insert(0, dynamic.identifier)
        with Usd.EditContext(self.stage, dynamic):
            UsdGeom.Camera.Define(self.stage, PATHS.follow_camera)
        saved = save_highway_loop_scene(self.stage, self.directory, dynamic)
        self.assertTrue(validate_highway_loop_scene(Usd.Stage.Open(saved))['passed'])
        moved = Path(self.temporary.name) / 'relocated'
        shutil.copytree(self.directory, moved)
        code = ('from pxr import Usd; import sys; s=Usd.Stage.Open(sys.argv[1]); '
                'assert s and not s.GetCompositionErrors(); '
                'assert s.GetPrimAtPath("/World/Environment/Highway/Colliders/RoadSurface"); '
                'assert s.GetPrimAtPath("/World/Cameras/Follow"); print("portable")')
        run = subprocess.run([sys.executable, '-c', code, str(moved / 'world.usda')],
                             cwd=Path(self.temporary.name), capture_output=True, text=True, timeout=30)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn('portable', run.stdout)
        for path in self.directory.rglob('*.usda'):
            self.assertNotIn(str(self.directory), path.read_text())
        with self.assertRaises(FileExistsError):
            save_highway_loop_scene(self.stage, self.directory, dynamic)

    def test_install_refuses_to_replace_existing_package(self):
        with self.assertRaisesRegex(ValueError, 'already installed'):
            install_highway_loop(self.stage, self.directory, SOURCE)

    def test_invalid_tire_table_or_filter_is_rejected(self):
        table = self.stage.GetPrimAtPath('/World/Physics/Resources/WinterTireFrictionTable')
        table.GetAttribute('frictionValues').Set([])
        with self.assertRaisesRegex(ValueError, 'friction-table'):
            validate_highway_loop_scene(self.stage)
        table.GetAttribute('frictionValues').Set([.75])
        group = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath(QUERY_GROUP))
        group.GetFilteredGroupsRel().AddTarget(GROUND_GROUP)
        with self.assertRaisesRegex(ValueError, 'filters out the road'):
            validate_highway_loop_scene(self.stage)

    def test_source_hash_mismatch_fails_before_asset_authoring(self):
        from usd.highway_loop_scene import _source_contract
        source_copy = Path(self.temporary.name) / 'modified-source'
        source_copy.mkdir()
        for name in ('manifest.json', 'highway_v02.usda', 'navigation.json', 'parameters.json'):
            shutil.copy2(SOURCE / name, source_copy / name)
        # Ordinary test-generated data, never a published source modification.
        (source_copy / 'navigation.json').write_bytes(b'{}')
        with self.assertRaisesRegex(ValueError, 'manifest mismatch: navigation.json'):
            _source_contract(source_copy)


if __name__ == '__main__':
    unittest.main()
