"""Pure geometry and USD authoring checks; native motion is a separate runtime gate."""
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from usd.showcase_fleet import (
    BACKGROUND_GROUP, BASE_RADIUS_M, BODY_Z_M, FleetSpec, KinematicFleet,
    LENGTH_M, QUERY_GROUP, WIDTH_M, WHEEL_NAMES, fleet_pose,
)
from usd.physical_scene import PATHS

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

ROOT = Path(__file__).resolve().parents[1]


class FleetGeometryTests(unittest.TestCase):
    def test_names_do_not_duplicate_background_prefix(self):
        self.assertEqual(FleetSpec('background_000',0,0,6).chassis_path,
            '/World/Vehicles/vehicle_background_000/Chassis')

    def test_initial_station_is_shared_base_circle_station(self):
        a = fleet_pose(FleetSpec('one', 0, 25., 6.), 0.)
        b = fleet_pose(FleetSpec('two', 3, 25., 6.), 0.)
        self.assertAlmostEqual(a['yaw_rad'], b['yaw_rad'])
        self.assertAlmostEqual(math.hypot(*a['position_m'][:2]), 500.)
        self.assertAlmostEqual(math.hypot(*b['position_m'][:2]), 511.1)

    def test_speed_is_actual_lane_arc_speed(self):
        spec = FleetSpec('car', 3, 0., 7.)
        a, b = fleet_pose(spec, 10.), fleet_pose(spec, 10.001)
        self.assertAlmostEqual(math.dist(a['position_m'], b['position_m']) / .001, 7., places=7)
        self.assertAlmostEqual(math.hypot(*b['velocity_m_s'][:2]), 7.)
        self.assertAlmostEqual(math.atan2(b['velocity_m_s'][1], b['velocity_m_s'][0]), b['yaw_rad'])

    def test_circle_seam_is_continuous(self):
        spec = FleetSpec('car', 0, 2*math.pi*500-.05, 6.)
        a, b = fleet_pose(spec, 0.), fleet_pose(spec, .02)
        self.assertLess(math.dist(a['position_m'], b['position_m']), .121)
        self.assertLess(abs(a['yaw_rad']-b['yaw_rad']), .001)

    def test_invalid_specs_rejected(self):
        for values in (('', 0, 0, 6), ('foo/bar', 0, 0, 6), ('0car', 0, 0, 6),
                       ('car', True, 0, 6), ('car', -1, 0, 6), ('car', 4, 0, 6),
                       ('car', 0, math.nan, 6), ('car', 0, 0, -1), ('car', 0, 0, math.inf)):
            with self.assertRaises(ValueError):
                FleetSpec(*values)
        for time_s in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                fleet_pose(FleetSpec('car', 0, 0, 6), time_s)

    def test_zero_speed_for_positive_contact_fixture(self):
        spec = FleetSpec('stationary', 0, 15., 0.)
        self.assertEqual(fleet_pose(spec, 0.), fleet_pose(spec, 600.))


class FakeBodies:
    def __init__(self, specs):
        self.specs = list(reversed(specs))  # Force ID mapping, never creation order.
        self.prim_paths = [s.chassis_path for s in self.specs]
        self.values = np.asarray([(*fleet_pose(s, 0.)['position_m'], *fleet_pose(s, 0.)['quaternion_xyzw'])
                                  for s in self.specs], dtype=np.float32)
        self.velocities = np.zeros((len(specs), 6), dtype=np.float32)
        self.targets = None
        self.target_calls = 0

    def get_transforms(self):
        return self.values

    def get_velocities(self):
        return self.velocities

    def set_kinematic_targets(self, values, indices):
        assert values.dtype == np.float32 and indices.dtype == np.uint32
        self.targets = values.copy()
        self.target_calls += 1

    def step(self, dt):
        self.velocities[:, :3] = (self.targets[:, :3] - self.values[:, :3]) / dt
        self.values = self.targets.copy()


@unittest.skipIf(Usd is None, 'USD authoring runtime required')
class FleetUsdTests(unittest.TestCase):
    def setUp(self):
        outputs = ROOT / 'outputs'
        outputs.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix='showcase-fleet-unit-', dir=outputs)
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.stage = Usd.Stage.CreateInMemory()
        world = UsdGeom.Xform.Define(self.stage, '/World').GetPrim()
        self.stage.SetDefaultPrim(world)
        Usd.ModelAPI(world).SetKind('assembly')
        Usd.ModelAPI(UsdGeom.Xform.Define(self.stage, '/World/Vehicles').GetPrim()).SetKind('group')
        UsdPhysics.Scene.Define(self.stage, PATHS.physics)
        UsdPhysics.CollisionGroup.Define(self.stage, QUERY_GROUP).CreateFilteredGroupsRel().SetTargets([
            '/World/Physics/Resources/VehicleWheelCollisionGroup'])
        self.specs = [FleetSpec('slow_001', 0, 20., 6.), FleetSpec('slow_002', 2, 50., 7.)]
        self.fleet = KinematicFleet(self.stage, self.directory, self.specs, episode_id='test')

    def bind(self):
        from types import SimpleNamespace
        self.bodies = FakeBodies(self.specs)
        self.assertTrue(self.fleet.bind(SimpleNamespace(create_rigid_body_view=lambda pattern: self.bodies))['passed'])

    def test_reusable_asset_and_kinematic_collision_hierarchy(self):
        self.assertTrue(self.fleet.validate()['passed'])
        for spec in self.specs:
            root = self.stage.GetPrimAtPath(spec.vehicle_path)
            self.assertTrue(root.HasAuthoredReferences())
            self.assertTrue(root.IsModel())
            self.assertFalse(root.IsInstance())
            body = UsdPhysics.RigidBodyAPI(self.stage.GetPrimAtPath(spec.chassis_path))
            self.assertTrue(body.GetKinematicEnabledAttr().Get())
            self.assertEqual(body.GetSimulationOwnerRel().GetTargets(), [Sdf.Path(PATHS.physics)])
            self.assertEqual(UsdGeom.Imageable(body.GetPrim()).GetVisibilityAttr().Get(), 'invisible')
            proxy = self.stage.GetPrimAtPath(spec.render_path)
            self.assertTrue(proxy.HasAuthoredReferences())
            self.assertNotEqual(UsdGeom.Imageable(proxy).ComputeVisibility(), 'invisible')
            self.assertFalse(any(name.startswith(('Physics', 'Physx'))
                                 for prim in Usd.PrimRange(proxy) for name in prim.GetAppliedSchemas()))
            for name in WHEEL_NAMES:
                self.assertEqual(UsdGeom.Cylinder(self.stage.GetPrimAtPath(spec.render_path+'/'+name+'/Tire')).GetAxisAttr().Get(), 'Y')

    def test_query_filter_excludes_roofs_without_ground_or_ego_filter(self):
        filters = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath(QUERY_GROUP)).GetFilteredGroupsRel().GetTargets()
        self.assertIn(Sdf.Path(BACKGROUND_GROUP), filters)
        group = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath(BACKGROUND_GROUP))
        self.assertFalse(group.GetFilteredGroupsRel().GetTargets())
        self.assertEqual(group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(),
                         [Sdf.Path(s.chassis_path + '/Colliders/Body') for s in self.specs])

    def test_target_is_not_observation_until_native_step(self):
        self.bind()
        initial = self.fleet.tracks(0)
        self.fleet.update(1./120)
        unchanged = self.fleet.tracks(0)
        self.assertEqual(initial, unchanged)
        self.assertFalse(self.fleet.pose_check()['passed'])
        self.bodies.step(1./120)
        self.assertTrue(self.fleet.pose_check()['passed'])
        actual = self.fleet.tracks(1)
        self.assertNotEqual(initial, actual)
        self.assertEqual([track.vehicle_id for track in actual], ['slow_002', 'slow_001'])
        self.assertTrue(all(t.episode_id == 'test' and t.tick == 1 and t.width_m == WIDTH_M for t in actual))
        self.assertAlmostEqual(actual[0].speed_m_s, 7., delta=.01)

    def test_static_assets_unchanged_and_only_dynamic_wheels_authored(self):
        self.bind()
        static_text = {layer.identifier: layer.ExportToString() for layer in self.stage.GetUsedLayers()
                       if not layer.anonymous}
        self.fleet.update(1./120)
        self.bodies.step(1./120)
        self.assertTrue(self.fleet.publish_visuals()['passed'])
        self.assertTrue(self.fleet.static_assets_unchanged())
        self.assertEqual(static_text, {layer.identifier: layer.ExportToString() for layer in self.stage.GetUsedLayers()
                                     if not layer.anonymous})
        for spec in self.specs:
            for name in WHEEL_NAMES:
                self.assertTrue(self.fleet.dynamic_layer.GetPropertyAtPath(spec.render_path+'/'+name+'.xformOp:rotateY:spin'))
            self.assertFalse(self.fleet.dynamic_layer.GetPropertyAtPath(spec.chassis_path+'.xformOp:translate'))

    def test_track_and_target_negative_controls(self):
        with self.assertRaises(RuntimeError):
            self.fleet.tracks(0)
        with self.assertRaises(RuntimeError):
            self.fleet.update(0.)
        self.bind()
        with self.assertRaises(RuntimeError):
            self.fleet.bind(None)
        self.fleet.update(.1)
        with self.assertRaises(ValueError):
            self.fleet.update(.05)
        for tick in (-1, True, .5):
            with self.assertRaises(ValueError):
                self.fleet.tracks(tick)
        self.fleet.tracks(10)
        with self.assertRaises(ValueError):
            self.fleet.tracks(9)
        self.bodies.values[0, 0] = math.nan
        with self.assertRaises(RuntimeError):
            self.fleet.tracks(11)

    def test_binding_must_match_exact_id_set(self):
        from types import SimpleNamespace
        bad = FakeBodies([FleetSpec('unexpected', 0, 0, 6.)])
        with self.assertRaisesRegex(RuntimeError, 'identity binding mismatch'):
            self.fleet.bind(SimpleNamespace(create_rigid_body_view=lambda pattern: bad))

    def test_duplicate_ids_and_assets_rejected(self):
        with self.assertRaises(ValueError):
            KinematicFleet(self.stage, self.directory, [self.specs[0], self.specs[0]])
        with self.assertRaises(ValueError):
            KinematicFleet(self.stage, self.directory, self.specs)

    def test_saved_layout_reopens_portably(self):
        reopened = Usd.Stage.Open(str(self.directory/'showcase-fleet-layout.usda'))
        self.assertFalse(reopened.GetCompositionErrors())
        for spec in self.specs:
            self.assertTrue(reopened.GetPrimAtPath(spec.chassis_path+'/Visuals/Body'))
            self.assertTrue(reopened.GetPrimAtPath(spec.vehicle_path).HasAuthoredReferences())
        self.assertEqual(UsdGeom.GetStageUpAxis(reopened), 'Z')
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(reopened), 1.)

    def test_visual_cadence_does_not_reduce_native_target_frequency(self):
        from types import SimpleNamespace
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, '/World')
        UsdPhysics.Scene.Define(stage, PATHS.physics)
        fleet = KinematicFleet(stage, self.directory/'cadence', self.specs,
                               episode_id='cadence', wheel_update_every=6)
        bodies = FakeBodies(self.specs)
        fleet.bind(SimpleNamespace(create_rigid_body_view=lambda pattern: bodies))
        wheel_path = self.specs[0].render_path + '/FrontLeftWheel.xformOp:rotateY:spin'
        for tick in range(1, 19):
            fleet.update(tick/120)
            self.assertEqual(bodies.target_calls, tick)
            bodies.step(1/120)
            self.assertTrue(fleet.pose_check()['passed'])
            if tick % 6 == 0:
                self.assertTrue(fleet.publish_visuals()['passed'])
            opinion = fleet.dynamic_layer.GetPropertyAtPath(wheel_path)
            if tick < 6:
                self.assertFalse(opinion)
            else:
                last_visual_time = (tick//6)*6/120
                self.assertAlmostEqual(opinion.default, fleet_pose(self.specs[0], last_visual_time)['wheel_spin_deg'], places=5)
            self.assertEqual(fleet._visual_update_count, tick//6)
        self.assertEqual(fleet.metadata['native_target_hz'], 120)
        self.assertEqual(fleet.metadata['wheel_visual_hz'], 20.)
        self.assertTrue(fleet.static_assets_unchanged())
        self.assertTrue(fleet._wheel_attributes)
        fleet.release()
        self.assertFalse(fleet._wheel_attributes)
        self.assertFalse(fleet._render_attributes)
        self.assertIsNone(fleet._view)
        self.assertIsNone(fleet.stage)
        self.assertIsNone(fleet.dynamic_layer)
        self.assertIsNone(fleet._float_tensor)

    def test_invalid_wheel_visual_cadence_rejected_before_authoring(self):
        for every in (True, 0, -1, .5, 7, 121):
            with self.assertRaisesRegex(ValueError, 'positive integer divisor'):
                KinematicFleet(self.stage, self.directory/'invalid-cadence', self.specs,
                               wheel_update_every=every)
        self.assertFalse((self.directory/'invalid-cadence').exists())

    def test_visible_proxy_follows_measured_body_not_unapplied_target(self):
        self.bind()
        self.assertTrue(self.fleet.visual_pose_check()['passed'])
        self.fleet.update(1./120)
        # The requested target is ahead, but native bodies have not stepped.
        self.assertTrue(self.fleet.publish_visuals()['passed'])
        proxy_before = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.specs[0].render_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.bodies.step(1./120)
        self.assertFalse(self.fleet.visual_pose_check()['passed'])
        self.assertTrue(self.fleet.publish_visuals()['passed'])
        proxy_after = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.specs[0].render_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.assertNotEqual(proxy_before, proxy_after)
        for spec in self.specs:
            self.assertFalse(self.fleet.dynamic_layer.GetPropertyAtPath(spec.chassis_path+'.xformOp:translate'))
            self.assertFalse(self.fleet.dynamic_layer.GetPropertyAtPath(spec.chassis_path+'.xformOp:orient'))

    def test_render_proxy_physics_and_competing_pose_negative_controls(self):
        self.bind()
        self.fleet.publish_visuals()
        proxy = self.stage.GetPrimAtPath(self.specs[0].render_path)
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            proxy.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0, 0, 0))
        self.assertFalse(self.fleet.visual_pose_check()['passed'])
        UsdPhysics.CollisionAPI.Apply(proxy)
        with self.assertRaisesRegex(ValueError, 'must not contain physics'):
            self.fleet.validate()


if __name__ == '__main__':
    unittest.main()
