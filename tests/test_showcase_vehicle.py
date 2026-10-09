"""USD-only input/scene checks; no claim of native moving-car validation."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

from usd.physical_scene import PATHS, compose_factory_vehicle
from usd.showcase_vehicle import author_prepared_vehicle_scene, package_hashes

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(Usd is None, 'USD authoring runtime required')
class PreparedShowcaseVehicleTests(unittest.TestCase):
    def setUp(self):
        from tests.test_highway_loop_scene import _factory
        (ROOT / 'outputs').mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix='prepared-vehicle-unit-', dir=ROOT / 'outputs')
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / 'source'
        self.target = Path(self.temp.name) / 'scene'
        stage = Usd.Stage.CreateInMemory()
        factory = SimpleNamespace(__file__=__file__, DRIVE_NONE=0, AxesIndices=lambda *args: args,
                                  create4WheeledCarsScenario=_factory)
        compose_factory_vehicle(stage, factory, 120, self.source)
        for path in PATHS.wheels:
            wheel = stage.GetPrimAtPath(path)
            for name in ('driveTorque', 'brakeTorque', 'steerAngle'):
                wheel.CreateAttribute('physxVehicleWheelController:' + name, Sdf.ValueTypeNames.Float).Set(0.)
        stage.GetRootLayer().Export(str(self.source / 'world.usda'))
        entry = Sdf.Layer.FindOrOpen(str(self.source / 'world.usda'))
        entry.subLayerPaths = ['./physics.usda', './layout.usda']
        entry.Save()
        self.model = dict(chassis_path=PATHS.chassis, physics_path=PATHS.physics, wheel_paths=PATHS.wheels,
                          wheelbase_m=3., track_m=1.6, wheel_radius_m=.4,
                          body_collision_dimensions_m=[4.8, 1.8, 1.4], body_collision_center_m=[0., 0., 0.],
                          license='Unit test synthetic geometry, not a supplied vehicle')
        self._write_model()
        self.stage = Usd.Stage.CreateInMemory()

    def _write_model(self):
        (self.source / 'model.json').write_text(json.dumps(self.model), encoding='utf-8')

    def test_copy_preserves_source_and_assembly(self):
        before = package_hashes(self.source)
        report = author_prepared_vehicle_scene(self.stage, self.target, self.source)
        self.assertEqual(package_hashes(self.source), before)
        self.assertEqual(report['source_hashes'], before)
        self.assertTrue(self.stage.GetPrimAtPath(PATHS.vehicle).HasAuthoredReferences())
        self.assertEqual(str(self.stage.GetDefaultPrim().GetPath()), '/World')
        self.assertFalse((self.target / 'world.usda').exists())
        self.assertTrue((self.target / 'source-vehicle-world.usda').exists())
        self.assertFalse(report['redistribution_permitted'])
        self.assertEqual(report['license'], self.model['license'])

    def test_scene_has_explicit_typed_leaf_addition_notice(self):
        # Installed SimulationManager subscribes to scene-addition callbacks;
        # ancestor-only CopySpec resyncs did not register the first GPU attempt.
        # Reproduce that required discovery event without starting Kit.
        from pxr import Tf
        noticed=[]
        def changes(notice,sender):
            for path in notice.GetResyncedPaths():
                if str(path)==PATHS.physics:
                    prim=sender.GetPrimAtPath(path)
                    noticed.append(prim.GetTypeName() if prim else None)
        subscription=Tf.Notice.Register(Usd.Notice.ObjectsChanged,changes,self.stage)
        try:
            author_prepared_vehicle_scene(self.stage,self.target,self.source)
        finally:
            subscription.Revoke()
        self.assertIn('PhysicsScene',noticed)

    def test_missing_input_never_substitutes_factory(self):
        with self.assertRaisesRegex(ValueError, 'missing'):
            author_prepared_vehicle_scene(self.stage, self.target, self.source / 'absent')
        self.assertFalse(self.target.exists())

    def test_existing_destination_preserved(self):
        self.target.mkdir()
        with self.assertRaises(FileExistsError):
            author_prepared_vehicle_scene(self.stage, self.target, self.source)

    def test_source_cannot_be_destination_parent(self):
        with self.assertRaisesRegex(ValueError, 'separate'):
            author_prepared_vehicle_scene(self.stage, self.source / 'copied', self.source)

    def test_nonempty_project_stage_rejected(self):
        UsdGeom.Xform.Define(self.stage, '/World')
        with self.assertRaisesRegex(ValueError, 'empty'):
            author_prepared_vehicle_scene(self.stage, self.target, self.source)

    def test_wrong_vehicle_path_contract_rejected(self):
        self.model['chassis_path'] = '/World/LegacyCar'
        self._write_model()
        with self.assertRaisesRegex(ValueError, 'path contract'):
            author_prepared_vehicle_scene(self.stage, self.target, self.source)

    def test_nonfinite_geometry_rejected(self):
        self.model['wheelbase_m'] = float('nan')
        self._write_model()
        with self.assertRaisesRegex(ValueError, 'measurement'):
            author_prepared_vehicle_scene(self.stage, self.target, self.source)

    def test_road_install_and_final_world_save(self):
        from usd.highway_loop_scene import install_highway_loop, save_highway_loop_scene
        author_prepared_vehicle_scene(self.stage, self.target, self.source)
        report = install_highway_loop(self.stage, self.target, ROOT / 'highway_usd/_v02')
        self.assertTrue(report['validation']['passed'])
        path = save_highway_loop_scene(self.stage, self.target)
        reopened = Usd.Stage.Open(path)
        self.assertFalse(reopened.GetCompositionErrors())
        self.assertTrue(reopened.GetPrimAtPath(PATHS.chassis))
        for path in PATHS.wheels:
            self.assertTrue(reopened.GetPrimAtPath(path).GetAttribute('physxVehicleWheelController:driveTorque'))

    def test_invalid_physics_rate_rejected(self):
        for hz in (0, -1, True, 120.5):
            with self.assertRaisesRegex(ValueError, 'positive integer'):
                author_prepared_vehicle_scene(self.stage, self.target, self.source, hz)

    def test_units_mismatch_rejected(self):
        entry = Usd.Stage.Open(str(self.source / 'world.usda'))
        UsdGeom.SetStageMetersPerUnit(entry, .01)
        entry.GetRootLayer().Save()
        with self.assertRaisesRegex(ValueError, 'metre-scale'):
            author_prepared_vehicle_scene(self.stage, self.target, self.source)

    def test_external_dependency_rejected(self):
        outside = Usd.Stage.CreateNew(str(Path(self.temp.name) / 'outside.usda'))
        UsdGeom.Xform.Define(outside, '/Outside')
        outside.SetDefaultPrim(outside.GetPrimAtPath('/Outside'))
        outside.GetRootLayer().Save()
        entry = Usd.Stage.Open(str(self.source / 'world.usda'))
        entry.DefinePrim('/World/External').GetReferences().AddReference('../outside.usda')
        entry.GetRootLayer().Save()
        with self.assertRaisesRegex(ValueError, 'inside its package'):
            author_prepared_vehicle_scene(self.stage, self.target, self.source)

    def test_supplied_local_package_on_road_without_modification(self):
        # Optional local integration, not a repository dependency or redistributed
        # asset. Only a path is named here; no supplied geometry is embedded.
        supplied = ROOT / 'vehicles/sim_ready/americano-i7-ev/v07'
        if not (supplied / 'world.usda').exists():
            self.skipTest('User-supplied local-only vehicle package is absent')
        from usd.highway_loop_scene import install_highway_loop, save_highway_loop_scene
        before = package_hashes(supplied)
        report = author_prepared_vehicle_scene(self.stage, self.target, supplied)
        self.assertEqual(report['source_hashes'], before)
        self.assertTrue(install_highway_loop(self.stage, self.target, ROOT / 'highway_usd/_v02')['validation']['passed'])
        reopened = Usd.Stage.Open(save_highway_loop_scene(self.stage, self.target))
        self.assertFalse(reopened.GetCompositionErrors())
        for path in PATHS.wheels:
            self.assertTrue(reopened.GetPrimAtPath(path + report['model']['wheel_visual_suffix']))
        self.assertEqual(package_hashes(supplied), before)


if __name__ == '__main__':
    unittest.main()
