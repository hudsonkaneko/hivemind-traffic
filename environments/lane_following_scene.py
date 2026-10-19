"""Composed, single-car Lab scene authoring; no simulator imports at module load.

Lab owns its one physics scene and empty /World/envs bookkeeping namespace.
Vehicle/road assets remain file-backed references. Wheel commands and temporary
camera/debug opinions belong to an anonymous runtime layer, never asset files.
"""
from pathlib import Path

from usd.physical_scene import PATHS, asset_hashes


def bind_composed_vehicle(stage, physics_hz, directory):
    """Compose on an isolated stage, then bind the existing public adapter.

    This runs in DirectRLEnv._setup_scene, before sim.reset activates physics.
    No live prim is moved, renamed, or cloned. The pre-created Lab physics prim
    stays at its original path and retains Lab's authored solver settings.
    """
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics
    from traffic.physx_vehicle import PhysxVehicle

    world = stage.GetPrimAtPath('/World')
    if world and {p.GetName() for p in world.GetChildren()} - {'Physics', 'envs'}:
        raise ValueError('Lane Lab scene requires a fresh Lab project namespace')
    physics = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'PhysicsScene']
    if physics != [PATHS.physics]:
        raise ValueError('Lab must create exactly /World/Physics/Scene')
    if any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()):
        raise ValueError('Lane Lab scene cannot contain a preexisting rigid body')
    bookkeeping = stage.GetPrimAtPath('/World/envs')
    if bookkeeping and (len(bookkeeping.GetChildren()) != 1 or
                        not stage.GetPrimAtPath('/World/envs/env_0') or
                        stage.GetPrimAtPath('/World/envs/env_0').GetChildren()):
        raise ValueError('Expected one empty Lab bookkeeping environment')

    isolated = Usd.Stage.CreateInMemory('lane-vehicle-authoring-only.usda')
    vehicle = PhysxVehicle(isolated, physics_hz, scene_directory=directory)
    root = stage.GetRootLayer()
    root.subLayerPaths = list(isolated.GetRootLayer().subLayerPaths) + list(root.subLayerPaths)
    world = UsdGeom.Xform.Define(stage, '/World').GetPrim()
    stage.SetDefaultPrim(world)
    Usd.ModelAPI(world).SetKind('assembly')
    stage.SetTimeCodesPerSecond(float(physics_hz))
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # Rebind only USD handles; native paths/interface and model metadata are the
    # same as the ordinary adapter. No method or external runtime is patched.
    vehicle.stage = stage
    vehicle.controllers = [PhysxSchema.PhysxVehicleWheelControllerAPI(stage.GetPrimAtPath(p))
                           for p in vehicle.wheel_paths]
    vehicle.body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(vehicle.path))
    vehicle.metadata['lab_scene_ownership'] = {
        'physics_scene': PATHS.physics, 'physics_step_owner': 'isaaclab.DirectRLEnv',
        'runtime_namespace_exceptions': ['/World/envs/env_0'],
        'asset_authoring': 'isolated stage before native physics initialization',
        'lab_physics_root_overrides': True,
    }
    return vehicle


def author_lane_view(stage, route, directory, *, diagnostic_contact_barrier=False):
    """Reference the existing road artwork and author named camera/light layout."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
    from visualization.physics_road_view import author_physics_road

    directory = Path(directory)
    road = author_physics_road(stage, route, clean_layout=True,
                               asset_directory=directory / 'assets')
    filename = directory / 'view-layout.usda'
    if filename.exists():
        raise FileExistsError(filename)
    view = Usd.Stage.CreateNew(str(filename))
    world = UsdGeom.Xform.Define(view, '/World').GetPrim()
    view.SetDefaultPrim(world)
    Usd.ModelAPI(world).SetKind('assembly')
    world.SetCustomDataByKey('diagnostic_contact_barrier', diagnostic_contact_barrier)
    UsdGeom.SetStageMetersPerUnit(view, 1.0)
    UsdGeom.SetStageUpAxis(view, 'Z')
    view.SetTimeCodesPerSecond(stage.GetTimeCodesPerSecond())
    UsdGeom.Scope.Define(view, '/World/Lighting')
    UsdGeom.Scope.Define(view, '/World/Cameras')
    UsdLux.DomeLight.Define(view, '/World/Lighting/Dome').CreateIntensityAttr(1000.0)
    for path in (PATHS.follow_camera, PATHS.overview_camera):
        camera = UsdGeom.Camera.Define(view, path)
        camera.CreateFocalLengthAttr(18.147562)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100000.0))
    if diagnostic_contact_barrier:
        barrier_path = directory / 'assets/diagnostic-contact-barrier.usda'
        if barrier_path.exists():
            raise FileExistsError(barrier_path)
        barrier = Usd.Stage.CreateNew(str(barrier_path))
        cube = UsdGeom.Cube.Define(barrier, '/Barrier')
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.23, 0.06)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        Usd.ModelAPI(cube.GetPrim()).SetKind('component')
        barrier.SetDefaultPrim(cube.GetPrim())
        UsdGeom.SetStageMetersPerUnit(barrier, 1.0)
        UsdGeom.SetStageUpAxis(barrier, 'Z')
        barrier.GetRootLayer().Save()
        for path in ('/World/Environment', '/World/Environment/Props'):
            Usd.ModelAPI(UsdGeom.Xform.Define(view, path).GetPrim()).SetKind('group')
        placement = view.DefinePrim(PATHS.barrier)
        placement.GetReferences().AddReference('./assets/diagnostic-contact-barrier.usda')
        point = route.evaluate(8.0)
        shape = UsdGeom.Xformable(placement)
        shape.AddTranslateOp().Set(Gf.Vec3d(*point.position_xy, 0.75))
        shape.AddRotateZOp().Set(point.yaw_rad * 180.0 / 3.141592653589793)
        shape.AddScaleOp().Set(Gf.Vec3f(1.0, 3.4, 1.5))
        placement.SetCustomDataByKey('purpose', 'diagnostic rigid-contact positive control only')
    view.GetRootLayer().Save()
    runtime = Sdf.Layer.CreateAnonymous('lane-following-runtime.usda')
    stage.GetRootLayer().subLayerPaths = [runtime.identifier, str(filename)] + list(stage.GetRootLayer().subLayerPaths)
    stage.SetEditTarget(runtime)
    return road, runtime


def update_lane_view(stage, road, runtime, state, *, mode='follow', target_xy=None):
    """Update camera/debug opinions only; return the camera path for capture."""
    from pxr import Gf, Usd, UsdGeom
    if mode not in ('follow', 'overview'):
        raise ValueError('Camera mode must be follow or overview')
    road.update(state, target_xy=target_xy)
    pose = road.camera_pose(mode, state)
    path = PATHS.follow_camera if mode == 'follow' else PATHS.overview_camera
    matrix = Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*pose.eye), Gf.Vec3d(*pose.target),
                                    Gf.Vec3d(0, 0, 1)).GetInverse()
    with Usd.EditContext(stage, runtime):
        camera = UsdGeom.Xformable(stage.GetPrimAtPath(path))
        attr = stage.GetPrimAtPath(path).GetAttribute('xformOp:transform:laneCamera')
        if not attr:
            attr = camera.AddTransformOp(opSuffix='laneCamera').GetAttr()
        attr.Set(matrix)
        camera.GetXformOpOrderAttr().Set(['!resetXformStack!', attr.GetName()])
    return path


def validate_lane_scene(stage):
    """Check live or reopened composition without requiring an obstacle/sensor."""
    from pxr import Usd, UsdGeom, UsdPhysics
    errors = [str(e) for e in stage.GetCompositionErrors()]
    required = [PATHS.chassis, PATHS.physics, PATHS.ground,
                PATHS.follow_camera, PATHS.overview_camera, *PATHS.wheels,
                '/World/Environment/Highway', '/World/Debug/Route']
    for path in required:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsActive() or not prim.IsLoaded():
            errors.append('Missing active/loaded prim: ' + path)
    if (not stage.GetDefaultPrim() or str(stage.GetDefaultPrim().GetPath()) != '/World'
            or UsdGeom.GetStageUpAxis(stage) != 'Z'
            or UsdGeom.GetStageMetersPerUnit(stage) != 1.0):
        errors.append('Invalid defaultPrim, units or up axis')
    physics = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'PhysicsScene']
    if physics != [PATHS.physics]:
        errors.append('Expected exactly one Lab physics scene')
    world = stage.GetPrimAtPath('/World')
    if world:
        extra = {p.GetName() for p in world.GetChildren()} - {
            'Environment', 'Vehicles', 'Physics', 'Lighting', 'Cameras', 'Debug', 'envs'}
        errors.extend('Unexpected project root: ' + p for p in sorted(extra))
    expected_kinds = {'/World': 'assembly', '/World/Environment': 'group',
                      '/World/Vehicles': 'group', PATHS.vehicle: 'component',
                      PATHS.ground: 'component', '/World/Environment/Highway': 'component'}
    for path, kind in expected_kinds.items():
        prim = stage.GetPrimAtPath(path)
        if not prim or Usd.ModelAPI(prim).GetKind() != kind or not prim.IsModel():
            errors.append('Invalid model ancestry: ' + path)
    for path in (PATHS.vehicle, PATHS.ground, '/World/Environment/Highway'):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.HasAuthoredReferences() or prim.IsInstance():
            errors.append('Expected editable referenced component: ' + path)
    barrier_enabled = bool(world and world.GetCustomDataByKey('diagnostic_contact_barrier'))
    barrier = stage.GetPrimAtPath(PATHS.barrier)
    if barrier_enabled:
        if (not barrier or not barrier.IsActive() or not barrier.IsLoaded()
                or not barrier.HasAuthoredReferences() or barrier.IsInstance()
                or not barrier.HasAPI(UsdPhysics.CollisionAPI)
                or Usd.ModelAPI(barrier).GetKind() != 'component' or not barrier.IsModel()):
            errors.append('Missing or invalid diagnostic contact barrier')
    elif barrier:
        errors.append('Unexpected barrier in ordinary lane task')
    vehicle = stage.GetPrimAtPath(PATHS.vehicle)
    if (not vehicle or vehicle.GetCustomDataByKey('movement_authority') != 'isaac_physx'
            or vehicle.GetCustomDataByKey('vehicle_id') != PATHS.vehicle_id):
        errors.append('Invalid vehicle identity or movement authority')
    bodies = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if bodies != [PATHS.chassis]:
        errors.append('Expected exactly one native chassis rigid body')
    bookkeeping = stage.GetPrimAtPath('/World/envs')
    if bookkeeping and (len(bookkeeping.GetChildren()) != 1 or
                        not stage.GetPrimAtPath('/World/envs/env_0') or
                        stage.GetPrimAtPath('/World/envs/env_0').GetChildren()):
        errors.append('Expected one empty Lab bookkeeping environment')
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World/'):
            continue
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if target.IsAbsolutePath() and not stage.GetObjectAtPath(target):
                    errors.append('Unresolved relationship: ' + str(rel.GetPath()))
        for attr in prim.GetAttributes():
            if any(not stage.GetObjectAtPath(p) for p in attr.GetConnections()):
                errors.append('Unresolved connection: ' + str(attr.GetPath()))
    if not UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(PATHS.chassis)):
        errors.append('Missing chassis rigid body')
    report = {'passed': not errors, 'errors': errors, 'physics_scenes': physics, 'rigid_bodies': bodies,
              'diagnostic_contact_barrier': barrier_enabled,
              'prims': [{'path': str(p.GetPath()), 'type': p.GetTypeName(),
                         'kind': Usd.ModelAPI(p).GetKind(), 'reference': p.HasAuthoredReferences()}
                        for p in stage.Traverse() if str(p.GetPath()).startswith('/World')],
              'layers': [p.identifier for p in stage.GetUsedLayers()],
              'runtime_namespace_exceptions': ['/World/envs/env_0', '/Render', '/Replicator', '/OmniverseKit_*']}
    if errors:
        raise ValueError('Invalid lane Lab scene: ' + '; '.join(errors))
    return report


def save_lane_scene(stage, directory, road, runtime):
    """Export initial composed sources, excluding empty Lab bookkeeping."""
    from pxr import Sdf, Usd, UsdGeom
    directory = Path(directory)
    names = ('initial-state.usda', 'runtime-initial.usda', 'debug.usda', 'world.usda')
    if any((directory / name).exists() for name in names):
        raise FileExistsError('Lane scene snapshot already exists')
    initial = Usd.Stage.CreateNew(str(directory / names[0]))
    Sdf.CopySpec(stage.GetRootLayer(), '/World', initial.GetRootLayer(), '/World')
    initial.RemovePrim('/World/envs')
    initial.SetDefaultPrim(initial.GetPrimAtPath('/World'))
    initial.GetRootLayer().Save()
    if not runtime.Export(str(directory / 'runtime-initial.usda')) or not road.dynamic_layer.Export(str(directory / 'debug.usda')):
        raise RuntimeError('Could not export initial runtime/debug layers')
    entry = Usd.Stage.CreateNew(str(directory / 'world.usda'))
    entry.GetRootLayer().subLayerPaths = ['./' + name for name in (
        'initial-state.usda', 'runtime-initial.usda', 'debug.usda', 'view-layout.usda',
        'physics.usda', 'road-layout.usda', 'layout.usda')]
    entry.SetDefaultPrim(entry.GetPrimAtPath('/World'))
    UsdGeom.SetStageMetersPerUnit(entry, 1.0)
    UsdGeom.SetStageUpAxis(entry, 'Z')
    entry.SetTimeCodesPerSecond(stage.GetTimeCodesPerSecond())
    entry.GetRootLayer().customLayerData = {
        'lane_lab_scene_schema': 1,
        'scope': 'Initial authored scene before seeded reset; not a trajectory replay or trained policy.'}
    entry.GetRootLayer().Save()
    validate_lane_scene(entry)
    return {'entry_path': str(directory / 'world.usda'), 'static_hashes': asset_hashes(directory)}
