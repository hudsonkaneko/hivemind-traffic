"""Composition checks without a renderer. Actual PhysX/RTX evidence is separate."""
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

import pytest

from usd.physical_scene import PATHS, _map_path


def test_versioned_paths_preserve_stable_identity_and_wheel_order():
    assert PATHS.version == 1
    assert PATHS.vehicle_id == 'ego'
    assert PATHS.lidar.startswith(PATHS.chassis + '/Sensors/')
    assert [p.rsplit('/', 1)[1] for p in PATHS.wheels] == [
        'FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel']


@pytest.fixture
def scene(tmp_path):
    pytest.importorskip('pxr.Usd')
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdLux
    from usd.physical_scene import compose_factory_vehicle, author_scene_details
    from visualization.physics_road_view import author_physics_road
    from traffic.lane_geometry import LaneRoute, RouteSegment

    def author(stage, *args, **kwargs):
        car = UsdGeom.Xform.Define(stage, '/World/Car_0')
        car.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
        UsdPhysics.RigidBodyAPI.Apply(car.GetPrim())
        UsdPhysics.MassAPI.Apply(car.GetPrim()).CreateMassAttr(1800.)
        UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisRender')
        collider = UsdGeom.Cube.Define(stage, '/World/Car_0/ChassisCollision')
        UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
        wheel_paths = []
        for name in ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel'):
            path = '/World/Car_0/'+name
            wheel_paths.append(path)
            wheel = UsdGeom.Xform.Define(stage, path)
            wheel.GetPrim().CreateRelationship('physxVehicleWheelAttachment:wheel').SetTargets(['/World/FrontWheel'])
            wheel.GetPrim().CreateRelationship('physxVehicleWheelAttachment:collisionGroup').SetTargets(['/World/VehicleGroundQueryGroup'])
            cylinder = UsdGeom.Cylinder.Define(stage, path+'/Collision')
            cylinder.CreateAxisAttr('Y')
        resource = UsdGeom.Scope.Define(stage, '/World/FrontWheel')
        resource.GetPrim().CreateAttribute('physxVehicleWheel:radius', Sdf.ValueTypeNames.Float).Set(.35)
        group = UsdPhysics.CollisionGroup.Define(stage, '/World/VehicleGroundQueryGroup')
        group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Car_0/ChassisCollision'])
        group.CreateFilteredGroupsRel().SetTargets(['/World/VehicleGroundQueryGroup'])
        UsdPhysics.Scene.Define(stage, '/World/PhysicsScene').CreateGravityMagnitudeAttr(9.81)
        UsdGeom.Mesh.Define(stage, '/World/GroundPlane')
        UsdLux.SphereLight.Define(stage, '/World/SphereLight')
        kwargs['vehiclePathsOut'].append('/World/Car_0')
        kwargs['wheelAttachmentPathsOut'].append(wheel_paths)

    factory = SimpleNamespace(__file__=__file__, DRIVE_NONE=0, AxesIndices=lambda *a:a,
                              create4WheeledCarsScenario=author)
    stage = Usd.Stage.CreateInMemory()
    directory = tmp_path/'scene'
    metadata = compose_factory_vehicle(stage, factory, 120, directory)
    author_scene_details(stage, directory, SimpleNamespace(x_m=45, y_m=0, length_m=2, width_m=1.6))
    route = LaneRoute('road', 7.2, (RouteSegment(100),))
    view = author_physics_road(stage, route, clean_layout=True, asset_directory=directory/'assets')
    UsdGeom.Xform.Define(stage, PATHS.lidar)
    return stage, directory, view, metadata


def test_references_internal_external_bindings_and_physics_layers(scene):
    from pxr import UsdPhysics, UsdGeom
    from usd.physical_scene import validate_scene
    stage, directory, view, metadata = scene
    assert validate_scene(stage)['passed']
    assert metadata['factory_contract']['passed']
    assert stage.GetPrimAtPath(PATHS.vehicle).HasAuthoredReferences()
    assert not stage.GetPrimAtPath(PATHS.vehicle).IsInstance()
    assert UsdPhysics.MassAPI(stage.GetPrimAtPath(PATHS.chassis)).GetMassAttr().Get() == 1800
    group = stage.GetPrimAtPath('/World/Physics/Resources/VehicleGroundQueryGroup')
    assert list(map(str, group.GetRelationship('collection:colliders:includes').GetTargets())) == [PATHS.chassis+'/Colliders/Chassis']
    for path in PATHS.wheels:
        wheel = stage.GetPrimAtPath(path)
        assert list(map(str, wheel.GetRelationship('physxVehicleWheelAttachment:wheel').GetTargets())) == [PATHS.vehicle+'/Physics/FrontWheel']
        assert list(map(str, wheel.GetRelationship('physxVehicleWheelAttachment:collisionGroup').GetTargets())) == ['/World/Physics/Resources/VehicleGroundQueryGroup']
        assert UsdGeom.Cylinder(stage.GetPrimAtPath(path+'/Collision')).GetAxisAttr().Get() == 'X'
    assert 'physics:mass' not in (directory/'assets/vehicle-geometry.usda').read_text()
    assert 'physics:mass' in (directory/'assets/vehicle-physics.usda').read_text()


def test_save_reopen_relative_dependencies_and_asset_immutability(scene, tmp_path):
    from pxr import Usd
    from usd.physical_scene import save_composed_scene, asset_hashes, validate_scene
    stage, directory, view, _ = scene
    entry = save_composed_scene(stage, directory, view.dynamic_layer)
    assert validate_scene(Usd.Stage.Open(entry))['passed']
    before = asset_hashes(directory)
    view.update(dict(position_m=[10,0,1]), target_xy=(15,2))
    assert asset_hashes(directory) == before
    code = "from pxr import Usd; s=Usd.Stage.Open(__import__('sys').argv[1]); assert s and not s.GetCompositionErrors(); assert s.GetPrimAtPath('/World/Vehicles/vehicle_ego/Chassis'); print('reopened')"
    child = subprocess.run([sys.executable, '-c', code, entry], cwd=tmp_path,
                           capture_output=True, text=True, timeout=30)
    assert child.returncode == 0, child.stderr
    assert 'reopened' in child.stdout
    for file in directory.rglob('*.usda'):
        assert str(directory) not in file.read_text()


def test_missing_required_content_and_unmapped_targets_fail_closed(scene):
    from pxr import Sdf
    from usd.physical_scene import validate_scene
    stage, *_ = scene
    stage.GetPrimAtPath(PATHS.chassis).SetActive(False)
    with pytest.raises(ValueError, match='Missing/unloaded'):
        validate_scene(stage)
    with pytest.raises(ValueError, match='Unmapped'):
        _map_path(Sdf.Path('/Foreign'), {'/World':'/Vehicle'})


def test_saved_sensor_profile_is_local_without_changing_live_reference(scene, tmp_path):
    from pxr import Sdf, Usd, UsdGeom
    from usd.physical_scene import save_composed_scene
    stage, directory, view, _ = scene
    profile = Usd.Stage.CreateNew(str(tmp_path/'source-profile.usda'))
    prim = UsdGeom.Xform.Define(profile, '/Sensor').GetPrim()
    profile.SetDefaultPrim(prim)
    prim.CreateAttribute('sensor:rate', Sdf.ValueTypeNames.Int).Set(20)
    profile.GetRootLayer().Save()
    sensor = stage.GetPrimAtPath(PATHS.lidar)
    sensor.GetReferences().AddReference(profile.GetRootLayer().identifier)
    before = sensor.GetMetadata('references')
    entry = save_composed_scene(stage, directory, view.dynamic_layer)
    assert sensor.GetMetadata('references') == before
    saved = Usd.Stage.Open(entry)
    assert saved.GetPrimAtPath(PATHS.lidar).GetAttribute('sensor:rate').Get() == 20
    assert saved.GetPrimAtPath(PATHS.lidar).GetMetadata('references').GetAppliedItems()[0].assetPath == './assets/lidar-profile.usda'


@pytest.mark.parametrize('mutation', ['kind', 'identity', 'authority'])
def test_model_ancestry_and_vehicle_identity_are_contracts(scene, mutation):
    from pxr import Usd
    from usd.physical_scene import validate_scene
    stage, *_ = scene
    if mutation == 'kind':
        Usd.ModelAPI(stage.GetPrimAtPath('/World/Vehicles')).SetKind('')
    else:
        key = 'vehicle_id' if mutation == 'identity' else 'movement_authority'
        stage.GetPrimAtPath(PATHS.vehicle).SetCustomDataByKey(key, 'wrong')
    with pytest.raises(ValueError, match='kind/ancestry|identity/motion'):
        validate_scene(stage)
