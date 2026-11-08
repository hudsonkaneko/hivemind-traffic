"""Pure binding checks and plain OpenUSD composition; no Kit/physics launch."""
from copy import deepcopy
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from traffic.namespaced_physx_vehicle import bind_existing_vehicle, vehicle_binding_paths
from traffic.driver_control import AppliedControl
from usd.physical_fleet_scene import (
    CHASSIS_GROUP, GROUND_GROUP, QUERY_GROUP, WHEEL_GROUP, _pose,
    author_extra_vehicle, validate_fleet_scene,
)
from usd.physical_scene import PATHS, asset_hashes

ROOT = Path(__file__).resolve().parents[1]
SECOND = '/World/Vehicles/vehicle_rl_002'
POSE = (12., 4., 1., 0., 0., math.sin(.2), math.cos(.2))


@pytest.mark.parametrize('path', ['/', '/World/Vehicles/vehicle_*', '/World/Vehicles/vehicle_../X',
                                  '/World/Vehicles/vehicle_a/FrontLeftWheel', 'ego', None])
def test_binding_rejects_ambiguous_paths(path):
    with pytest.raises(ValueError):
        vehicle_binding_paths(path)


def test_paths_are_stable_and_wheel_order_is_not_creation_order():
    first = vehicle_binding_paths(PATHS.chassis)
    second = vehicle_binding_paths(SECOND)
    assert first.vehicle == PATHS.vehicle and first.vehicle_id == 'ego'
    assert second.vehicle_id == 'rl_002'
    assert [p.rsplit('/', 1)[1] for p in second.wheels] == [
        'FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel']
    assert not set(first.wheels) & set(second.wheels)


@pytest.mark.parametrize('pose', [(0, 0, 0), (0, 0, 0, 0, 0, 0, 0),
                                  (0, 0, 0, 0, 0, 0, 2), (False, 0, 0, 0, 0, 0, 1),
                                  (float('nan'), 0, 0, 0, 0, 0, 1)])
def test_pose_is_explicit_finite_world_xyzw(pose):
    with pytest.raises(ValueError):
        _pose(pose)


@pytest.fixture
def scene(tmp_path):
    pytest.importorskip('pxr.Usd')
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
    from usd.physical_scene import compose_factory_vehicle
    assert tmp_path.resolve().is_relative_to(ROOT / 'outputs'), 'Use a unique --basetemp under outputs'

    def author(stage, *args, **kwargs):
        car = UsdGeom.Xform.Define(stage, '/World/Car_0')
        car.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
        UsdPhysics.RigidBodyAPI.Apply(car.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(car.GetPrim())
        mass.CreateMassAttr(1800.)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(700, 2600, 2800))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisRender')
        collider = UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisCollision')
        UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
        UsdPhysics.Scene.Define(stage, '/World/PhysicsScene').CreateGravityMagnitudeAttr(9.81)
        UsdLux.SphereLight.Define(stage, '/World/SphereLight')
        UsdGeom.Mesh.Define(stage, '/World/GroundPlane')
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath('/World/GroundPlane'))
        for name in ('FrontWheel', 'RearWheel', 'FrontTire', 'RearTire', 'FrontSuspension', 'RearSuspension'):
            resource = UsdGeom.Scope.Define(stage, '/World/' + name).GetPrim()
            resource.CreateAttribute('test:output', Sdf.ValueTypeNames.Float).Set(.35)
            if 'Tire' in name:
                resource.CreateRelationship('physxVehicleTire:frictionTable').SetTargets(['/World/FrictionTable'])
        table = UsdGeom.Scope.Define(stage, '/World/FrictionTable').GetPrim()
        table.CreateRelationship('test:material').SetTargets(['/World/Tarmac'])
        table.CreateAttribute('test:output', Sdf.ValueTypeNames.Float).Set(.75)
        material = UsdGeom.Scope.Define(stage, '/World/Tarmac').GetPrim()
        material.CreateAttribute('test:friction', Sdf.ValueTypeNames.Float).Set(.75)
        binding = collider.GetPrim().CreateRelationship('material:binding:physics')
        binding.SetTargets(['/World/Tarmac'])
        binding.SetMetadata('bindMaterialAs', 'strongerThanDescendants')
        groups = {}
        for group in ('VehicleChassisCollisionGroup', 'VehicleWheelCollisionGroup',
                      'VehicleGroundQueryGroup', 'GroundSurfaceCollisionGroup'):
            groups[group] = UsdPhysics.CollisionGroup.Define(stage, '/World/' + group)
        groups['VehicleChassisCollisionGroup'].GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
            ['/World/Car_0/ChassisCollision'])
        groups['GroundSurfaceCollisionGroup'].GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
            ['/World/GroundPlane'])
        filters = {'VehicleChassisCollisionGroup': ['VehicleGroundQueryGroup'],
                   'VehicleWheelCollisionGroup': ['VehicleGroundQueryGroup', 'GroundSurfaceCollisionGroup'],
                   'VehicleGroundQueryGroup': ['VehicleWheelCollisionGroup', 'VehicleChassisCollisionGroup'],
                   'GroundSurfaceCollisionGroup': ['GroundSurfaceCollisionGroup', 'VehicleWheelCollisionGroup']}
        for group, names in filters.items():
            groups[group].CreateFilteredGroupsRel().SetTargets(['/World/' + name for name in names])
        wheels, wheel_colliders = [], []
        for name in ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel'):
            path = '/World/Car_0/' + name
            wheels.append(path)
            wheel = UsdGeom.Xform.Define(stage, path).GetPrim()
            axle = 'Front' if name.startswith('Front') else 'Rear'
            for key, resource in (('wheel', axle + 'Wheel'), ('tire', axle + 'Tire'), ('suspension', axle + 'Suspension')):
                wheel.CreateRelationship('physxVehicleWheelAttachment:' + key).SetTargets(['/World/' + resource])
            wheel.CreateRelationship('physxVehicleWheelAttachment:collisionGroup').SetTargets(['/World/VehicleGroundQueryGroup'])
            wheel.CreateRelationship('test:mixed').SetTargets(['/World/' + axle + 'Wheel', '/World/FrictionTable'])
            wheel.CreateAttribute('test:connected', Sdf.ValueTypeNames.Float).SetConnections(
                ['/World/' + axle + 'Wheel.test:output'])
            cylinder = UsdGeom.Cylinder.Define(stage, path + '/Collision')
            cylinder.CreateAxisAttr('Y')
            UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
            wheel_colliders.append(path + '/Collision')
        groups['VehicleWheelCollisionGroup'].GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(wheel_colliders)
        kwargs['vehiclePathsOut'].append('/World/Car_0')
        kwargs['wheelAttachmentPathsOut'].append(wheels)

    factory = SimpleNamespace(__file__=__file__, DRIVE_NONE=0, AxesIndices=lambda *args: args,
                              create4WheeledCarsScenario=author)
    stage = Usd.Stage.CreateInMemory()
    directory = tmp_path / 'scene'
    metadata = compose_factory_vehicle(stage, factory, 120, directory)
    for wheel in PATHS.wheels:
        attr = stage.GetPrimAtPath(wheel).GetAttribute('test:connected')
        attr.SetConnections(attr.GetConnections() + [Sdf.Path('/World/Physics/Resources/FrictionTable.test:output')])
    return stage, directory, metadata


def add(scene):
    stage, directory, _ = scene
    return author_extra_vehicle(stage, PATHS.vehicle, 'rl_002', POSE, directory)


def test_two_references_full_binding_closure_and_source_file_immutability(scene):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    stage, directory, _ = scene
    before = asset_hashes(directory)
    source_text = stage.GetPrimAtPath(PATHS.chassis).GetAttribute('xformOp:translate').Get()
    result = add(scene)
    assert result['static_validation']['passed'] and not result['native_validated']
    assert result['static_validation']['wheel_count'] == 8
    assert result['original_files_unchanged']
    assert all(asset_hashes(directory)[name] == digest for name, digest in before.items())
    assert len(asset_hashes(directory)) == len(before) + 1
    assert stage.GetPrimAtPath(PATHS.chassis).GetAttribute('xformOp:translate').Get() == source_text
    for root in (PATHS.vehicle, SECOND):
        prim = stage.GetPrimAtPath(root)
        assert prim.HasAuthoredReferences() and not prim.IsInstanceable()
        for wheel in vehicle_binding_paths(root).wheels:
            prim = stage.GetPrimAtPath(wheel)
            axle = 'Front' if 'Front' in wheel else 'Rear'
            assert prim.GetRelationship('physxVehicleWheelAttachment:wheel').GetTargets() == [Sdf.Path(root + '/Physics/' + axle + 'Wheel')]
            assert prim.GetRelationship('physxVehicleWheelAttachment:collisionGroup').GetTargets() == [Sdf.Path(QUERY_GROUP)]
            assert prim.GetRelationship('test:mixed').GetTargets() == [Sdf.Path(root + '/Physics/' + axle + 'Wheel'),
                                                                     Sdf.Path('/World/Physics/Resources/FrictionTable')]
            assert prim.GetAttribute('test:connected').GetConnections() == [Sdf.Path(root + '/Physics/' + axle + 'Wheel.test:output'),
                                                                          Sdf.Path('/World/Physics/Resources/FrictionTable.test:output')]
        collider = stage.GetPrimAtPath(root + '/Chassis/Colliders/Chassis')
        assert collider.GetRelationship('material:binding:physics').GetMetadata('bindMaterialAs') == 'strongerThanDescendants'
    transform = UsdGeom.Xformable(stage.GetPrimAtPath(SECOND + '/Chassis')).ComputeLocalToWorldTransform(0)
    assert list(transform.ExtractTranslation()) == list(POSE[:3])
    assert transform.ExtractRotationQuat().GetReal() == pytest.approx(POSE[6])
    assert list(transform.ExtractRotationQuat().GetImaginary()) == pytest.approx(POSE[3:6])
    assert Sdf.Path(CHASSIS_GROUP) not in UsdPhysics.CollisionGroup(stage.GetPrimAtPath(CHASSIS_GROUP)).GetFilteredGroupsRel().GetTargets()
    contents = Path(result['assembly_layer']).read_text()
    assert '@./assets/vehicle.usda@' in contents and str(directory) not in contents
    stage.GetRootLayer().Export(str(directory / 'fleet-world.usda'))
    reopened = Usd.Stage.Open(str(directory / 'fleet-world.usda'))
    assert validate_fleet_scene(reopened, [PATHS.vehicle, SECOND])['passed']


def test_three_car_snapshot_preserves_live_bindings_and_reopens_portably(scene):
    from pxr import Usd
    from usd.highway_circles_scene import export_initial_scene
    stage, directory, _ = scene
    add(scene)
    third = '/World/Vehicles/vehicle_ppo_003'
    author_extra_vehicle(stage, PATHS.vehicle, 'ppo_003', (24., 8., 1., 0., 0., 0., 1.), directory)
    roots = [PATHS.vehicle, SECOND, third]
    assert validate_fleet_scene(stage, roots)['wheel_count'] == 12
    handles = [stage.GetPrimAtPath(path) for root in roots for path in vehicle_binding_paths(root).wheels]
    original = stage.GetRootLayer().ExportToString()
    entry = export_initial_scene(stage, directory)
    assert stage.GetRootLayer().ExportToString() == original
    assert all(handles)
    reopened = Usd.Stage.Open(str(entry))
    assert all(path.startswith('./') for path in reopened.GetRootLayer().subLayerPaths)
    assert validate_fleet_scene(reopened, roots)['wheel_count'] == 12


def test_stronger_live_root_collection_opinions_are_preserved_and_extended(scene):
    from pxr import Sdf, Usd, UsdPhysics
    stage, _, _ = scene
    group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(CHASSIS_GROUP))
    original = group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets()
    with Usd.EditContext(stage, stage.GetRootLayer()):
        group.GetCollidersCollectionAPI().GetIncludesRel().SetTargets(original)
    add(scene)
    assert group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets() == original + [
        Sdf.Path(SECOND + '/Chassis/Colliders/Chassis')]


@pytest.mark.parametrize('problem', ['dangling', 'chassis_self_filter', 'query_filters_ground', 'missing_wheel_group'])
def test_invalid_source_contract_rejected_before_attach_or_file_write(scene, problem):
    from pxr import UsdPhysics
    stage, directory, _ = scene
    if problem == 'dangling':
        stage.GetPrimAtPath(PATHS.wheels[0]).CreateRelationship('test:invalid').SetTargets(['/Missing'])
    elif problem == 'missing_wheel_group':
        UsdPhysics.CollisionGroup(stage.GetPrimAtPath(WHEEL_GROUP)).GetCollidersCollectionAPI().GetIncludesRel().SetTargets([])
    else:
        path, target = (CHASSIS_GROUP, CHASSIS_GROUP) if problem == 'chassis_self_filter' else (QUERY_GROUP, GROUND_GROUP)
        rel = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(path)).GetFilteredGroupsRel()
        rel.AddTarget(target)
    before = list(stage.GetRootLayer().subLayerPaths)
    with pytest.raises(ValueError, match='Invalid physical fleet'):
        add(scene)
    assert not stage.GetPrimAtPath(SECOND)
    assert not (directory / 'fleet-rl_002.usda').exists()
    assert stage.GetRootLayer().subLayerPaths == before


def test_cross_vehicle_resource_alias_rejected_after_composition(scene):
    stage, _, _ = scene
    add(scene)
    stage.GetPrimAtPath(SECOND + '/Chassis/FrontLeftWheel').GetRelationship(
        'physxVehicleWheelAttachment:wheel').SetTargets([PATHS.vehicle + '/Physics/FrontWheel'])
    with pytest.raises(ValueError, match='Cross-vehicle resource alias'):
        validate_fleet_scene(stage, [PATHS.vehicle, SECOND])


def test_other_shared_group_cannot_silently_disable_car_collision(scene):
    from pxr import UsdPhysics
    stage, _, _ = scene
    add(scene)
    extra = UsdPhysics.CollisionGroup.Define(stage, '/World/Physics/Resources/AccidentalFilter')
    extra.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Vehicles'])
    extra.CreateFilteredGroupsRel().SetTargets([CHASSIS_GROUP])
    with pytest.raises(ValueError, match='Cross-car collision suppressed'):
        validate_fleet_scene(stage, [PATHS.vehicle, SECOND])


def test_instance_physics_override_cannot_be_lost_when_referencing_asset(scene):
    from pxr import UsdPhysics
    stage, directory, _ = scene
    UsdPhysics.MassAPI(stage.GetPrimAtPath(PATHS.chassis)).GetMassAttr().Set(2000.)
    with pytest.raises(ValueError, match='Source instance physics override'):
        add(scene)
    assert not stage.GetPrimAtPath(SECOND) and not (directory / 'fleet-rl_002.usda').exists()


def test_authoring_refuses_live_physics_and_duplicate_identity(scene):
    stage, directory, _ = scene
    for started in (True, 1, None):
        with pytest.raises(ValueError, match='before physics'):
            author_extra_vehicle(stage, PATHS.vehicle, 'rl_002', POSE, directory, physics_started=started)
    add(scene)
    before = asset_hashes(directory)
    with pytest.raises(ValueError, match='destination must not exist'):
        add(scene)
    assert asset_hashes(directory) == before


def test_moving_wrapper_and_foreign_reference_rejected(scene):
    from pxr import Gf, UsdGeom
    stage, directory, _ = scene
    UsdGeom.Xformable(stage.GetPrimAtPath(PATHS.vehicle)).AddTranslateOp().Set(Gf.Vec3d(1, 0, 0))
    with pytest.raises(ValueError, match='identity transforms'):
        add(scene)
    UsdGeom.Xformable(stage.GetPrimAtPath(PATHS.vehicle)).ClearXformOpOrder()
    stage.GetPrimAtPath(PATHS.vehicle).GetReferences().AddReference(str(directory / 'assets/ground.usda'))
    with pytest.raises(ValueError, match='reference exactly'):
        add(scene)


class Value:
    def __init__(self, value): self.value = value
    def Get(self): return self.value
    def Set(self, value): self.value = value; return True


@pytest.fixture
def fake_native(monkeypatch):
    """Exercise the adapter with native interfaces replaced, never loaded."""
    class BodyAPI:
        def __init__(self, prim): self.prim = prim
        def GetVelocityAttr(self): return Value([0, 0, 0])
        def GetRigidBodyEnabledAttr(self): return self.prim.values['enabled']
        def GetKinematicEnabledAttr(self): return self.prim.values['kinematic']
        def GetSimulationOwnerRel(self): return SimpleNamespace(GetTargets=lambda: self.prim.values['owners'].Get())
    class VehicleAPI: pass
    class AttachmentAPI: pass
    class ControllerAPI:
        def __init__(self, prim): self.prim = prim
        def GetDriveTorqueAttr(self): return self.prim.values['drive']
        def GetBrakeTorqueAttr(self): return self.prim.values['brake']
        def GetSteerAngleAttr(self): return self.prim.values['steer']
    class Prim:
        def __init__(self, path, kind=''):
            self.path, self.kind = path, kind
            self.apis = {BodyAPI, VehicleAPI} if path.endswith('/Chassis') else {AttachmentAPI, ControllerAPI}
            self.identity = path.rsplit('vehicle_', 1)[-1]
            self.values = {name: Value(0.) for name in ('drive', 'brake', 'steer')}
            self.values.update(enabled=Value(True), kinematic=Value(False), owners=Value([]))
        def __bool__(self): return True
        def GetPath(self): return self.path
        def GetTypeName(self): return self.kind
        def IsInstance(self): return False
        def IsInstanceProxy(self): return False
        def IsInstanceable(self): return False
        def IsActive(self): return True
        def IsLoaded(self): return True
        def HasAPI(self, api): return api in self.apis
        def GetCustomDataByKey(self, name): return self.identity if name == 'vehicle_id' else 'isaac_physx'
    prims = {}
    for root in (PATHS.vehicle, SECOND):
        paths = vehicle_binding_paths(root)
        for path in (root, paths.chassis, *paths.wheels):
            prims[path] = Prim(path)
    prims[PATHS.physics] = Prim(PATHS.physics, 'PhysicsScene')
    stage = SimpleNamespace(GetPrimAtPath=lambda path: prims.get(path), Traverse=lambda: list(prims.values()))
    gravity = Value(9.81)
    schemas = ModuleType('pxr')
    schemas.PhysxSchema = SimpleNamespace(PhysxVehicleAPI=VehicleAPI,
        PhysxVehicleWheelAttachmentAPI=AttachmentAPI, PhysxVehicleWheelControllerAPI=ControllerAPI)
    schemas.UsdGeom = SimpleNamespace(GetStageMetersPerUnit=lambda stage: 1., GetStageUpAxis=lambda stage: 'Z')
    schemas.UsdPhysics = SimpleNamespace(RigidBodyAPI=BodyAPI,
        Scene=lambda prim: SimpleNamespace(GetGravityMagnitudeAttr=lambda: gravity),
        MassAPI=lambda prim: SimpleNamespace(GetMassAttr=lambda: Value(1800.),
            GetDiagonalInertiaAttr=lambda: Value([700., 2600., 2800.]), GetCenterOfMassAttr=lambda: Value([0., 0., 0.])))
    interface = SimpleNamespace()
    omni = ModuleType('omni')
    physx = ModuleType('omni.physx')
    physx.get_physx_interface = lambda: interface
    omni.physx = physx
    monkeypatch.setitem(sys.modules, 'pxr', schemas)
    monkeypatch.setitem(sys.modules, 'omni', omni)
    monkeypatch.setitem(sys.modules, 'omni.physx', physx)
    metadata = {'wheelbase_m': 3.2, 'track_m': 1.6, 'factory_sha256': 'a'*64,
                'scene_composition': {'source': ['untouched']}}
    return stage, prims, metadata, gravity


def test_two_binders_eight_independent_controllers_native_identities_and_metadata(fake_native):
    stage, prims, source, gravity = fake_native
    before = deepcopy(source)
    first = bind_existing_vehicle(stage, PATHS.vehicle, source)
    second = bind_existing_vehicle(stage, SECOND + '/Chassis', source)
    assert len(first.controllers + second.controllers) == 8
    assert len({id(c.prim) for c in first.controllers + second.controllers}) == 8
    second.apply(AppliedControl(.2, .4, 0., 'policy', 1, False), 1000., 2000.)
    assert all(prims[p].values['drive'].Get() == 0 for p in first.wheel_paths)
    assert all(prims[p].values['drive'].Get() > 0 for p in second.wheel_paths[:2])
    assert all(prims[p].values['drive'].Get() == 0 for p in second.wheel_paths[2:])
    assert prims[second.wheel_paths[0]].values['steer'].Get() != prims[second.wheel_paths[1]].values['steer'].Get()
    views = {}
    def create(path):
        view = SimpleNamespace(prim_paths=[path])
        views[path] = view
        return view
    sim = SimpleNamespace(create_rigid_body_view=create)
    first.bind_native_state(sim)
    second.bind_native_state(sim)
    assert first._native_state_view is not second._native_state_view
    assert set(views) == {PATHS.chassis, SECOND + '/Chassis'}
    with pytest.raises(RuntimeError, match='already bound'):
        second.bind_native_state(sim)
    assert source == before and gravity.Get() == 9.81
    assert not second.metadata['native_fleet_validated']
    second.metadata['scene_composition']['paths']['vehicle_id'] = 'mutation'
    assert first.metadata['scene_composition']['paths']['vehicle_id'] == 'ego'


def test_binding_rejects_missing_wheel_schema_gravity_rate_and_native_alias(fake_native):
    stage, prims, source, gravity = fake_native
    wheel = prims[SECOND + '/Chassis/FrontLeftWheel']
    saved = wheel.apis
    wheel.apis = set()
    with pytest.raises(ValueError, match='independently controllable'):
        bind_existing_vehicle(stage, SECOND, source)
    wheel.apis = saved
    gravity.Set(2.)
    with pytest.raises(ValueError, match='gravity'):
        bind_existing_vehicle(stage, SECOND, source)
    assert gravity.Get() == 2.
    gravity.Set(9.81)
    for rate in (60, 120., True):
        with pytest.raises(ValueError, match='120 Hz'):
            bind_existing_vehicle(stage, SECOND, source, rate)
    subject = bind_existing_vehicle(stage, SECOND, source)
    with pytest.raises(RuntimeError, match='identity mismatch'):
        subject.bind_native_state(SimpleNamespace(create_rigid_body_view=lambda path: SimpleNamespace(prim_paths=[PATHS.chassis])))
    assert subject._native_state_view is None


@pytest.mark.parametrize('key,value', [('enabled', False), ('kinematic', True),
    ('owners', ['/World/OtherScene']), ('owners', [PATHS.physics, '/World/OtherScene'])])
def test_binding_rejects_nondynamic_or_foreign_owned_body(fake_native, key, value):
    stage, prims, source, _ = fake_native
    prims[SECOND + '/Chassis'].values[key].Set(value)
    with pytest.raises(ValueError, match='enabled dynamic'):
        bind_existing_vehicle(stage, SECOND, source)


@pytest.mark.parametrize('mutation', ['disabled', 'kinematic', 'foreign_owner'])
def test_real_usd_fleet_rejects_nondynamic_and_foreign_owner(scene, mutation):
    from pxr import UsdPhysics
    stage, _, _ = scene
    add(scene)
    body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(SECOND + '/Chassis'))
    if mutation == 'disabled': body.CreateRigidBodyEnabledAttr(False)
    elif mutation == 'kinematic': body.CreateKinematicEnabledAttr(True)
    else: body.CreateSimulationOwnerRel().SetTargets(['/World/OtherScene'])
    with pytest.raises(ValueError, match='enabled dynamic'):
        validate_fleet_scene(stage, (PATHS.vehicle, SECOND))
