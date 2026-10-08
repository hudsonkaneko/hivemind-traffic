"""Versioned composition contract for the one-car physical demonstration.

No simulator imports at module load. Factory is used only on a disposable,
unattached stage, never to rename a live physics scene. External relationship
targets are scene-owned bindings, not invalid targets across asset references.
"""
from dataclasses import dataclass, asdict
import hashlib
from pathlib import Path


@dataclass(frozen=True)
class PhysicalScenePaths:
    version: int = 1
    vehicle_id: str = 'ego'
    vehicle: str = '/World/Vehicles/vehicle_ego'
    chassis: str = '/World/Vehicles/vehicle_ego/Chassis'
    physics: str = '/World/Physics/Scene'
    ground: str = '/World/Environment/Ground'
    barrier: str = '/World/Environment/Props/Barrier_001'
    lidar: str = '/World/Vehicles/vehicle_ego/Chassis/Sensors/LidarFront'
    follow_camera: str = '/World/Cameras/Follow'
    overview_camera: str = '/World/Cameras/Overview'

    @property
    def wheels(self):
        return [self.chassis + '/' + name for name in
                ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel')]


PATHS = PhysicalScenePaths()


def _metadata(stage, root, kind=None):
    from pxr import Usd, UsdGeom
    stage.SetDefaultPrim(stage.GetPrimAtPath(root))
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    stage.SetTimeCodesPerSecond(120.)
    if kind:
        Usd.ModelAPI(stage.GetPrimAtPath(root)).SetKind(kind)


def _new_stage(path):
    from pxr import Usd
    if path.exists():
        raise FileExistsError('Refusing to overwrite scene asset: ' + str(path))
    return Usd.Stage.CreateNew(str(path))


def _map_path(path, mapping):
    from pxr import Sdf
    for source, target in sorted(mapping.items(), key=lambda pair: -len(pair[0])):
        if path.HasPrefix(Sdf.Path(source)):
            return path.ReplacePrefix(Sdf.Path(source), Sdf.Path(target))
    # CopySpec already translates targets inside the copied subtree (including
    # a collision group's self-filter). Do not remap those a second time.
    if any(path.HasPrefix(Sdf.Path(target)) for target in mapping.values()):
        return path
    raise ValueError('Unmapped Factory path: ' + str(path))


def _copy_asset(source, mappings, path, asset_root, scene_root, all_mappings):
    """Copy authored specs, closing internal targets and returning external binds."""
    from pxr import Sdf, UsdGeom, Usd
    asset = _new_stage(path)
    UsdGeom.Xform.Define(asset, asset_root)
    external = []
    for old, new in mappings.items():
        destination = new.replace(scene_root, asset_root, 1)
        parent = Sdf.Path(destination).GetParentPath()
        if parent != Sdf.Path.absoluteRootPath and not asset.GetPrimAtPath(parent):
            UsdGeom.Scope.Define(asset, parent)
        if not Sdf.CopySpec(source.GetRootLayer(), old, asset.GetRootLayer(), destination):
            raise RuntimeError('Could not copy ' + old)
    for prim in asset.Traverse():
        for prop in list(prim.GetRelationships()) + list(prim.GetAttributes()):
            relationship = isinstance(prop, Usd.Relationship)
            targets = prop.GetTargets() if relationship else prop.GetConnections()
            if not targets:
                continue
            mapped = [_map_path(p.ReplacePrefix(Sdf.Path(asset_root), Sdf.Path(scene_root))
                                if p.HasPrefix(Sdf.Path(asset_root)) else p, all_mappings) for p in targets]
            if all(p.HasPrefix(Sdf.Path(scene_root)) for p in mapped):
                local = [p.ReplacePrefix(Sdf.Path(scene_root), Sdf.Path(asset_root)) for p in mapped]
                (prop.SetTargets if relationship else prop.SetConnections)(local)
            else:
                # Mixed target lists must be authored together in the assembly.
                global_prop = prop.GetPath().ReplacePrefix(Sdf.Path(asset_root), Sdf.Path(scene_root))
                external.append((str(global_prop), relationship, [str(p) for p in mapped]))
                if relationship:
                    # Keep the relationship spec and binding-strength metadata;
                    # only its out-of-asset targets move to scene assembly.
                    prop.ClearTargets(False)
                else:
                    prop.ClearConnections()
    _metadata(asset, asset_root, 'component')
    Usd.ModelAPI(asset.GetDefaultPrim()).SetAssetName(path.stem)
    asset.GetRootLayer().Save()
    return asset, external


def _separate_vehicle_layers(asset, directory):
    """Keep the public asset interface separate from shape and physics opinions."""
    from pxr import Sdf, Usd, UsdGeom
    geometry = _new_stage(directory/'vehicle-geometry.usda')
    geometry.GetRootLayer().ImportFromString(asset.GetRootLayer().ExportToString())
    physics = _new_stage(directory/'vehicle-physics.usda')
    UsdGeom.Xform.Define(physics, '/Vehicle')
    _metadata(physics, '/Vehicle', 'component')
    for prim in list(geometry.Traverse()):
        physics_apis = [name for name in prim.GetAppliedSchemas() if name.startswith(('Physics', 'Physx'))]
        props = [p for p in prim.GetAuthoredProperties() if p.GetName().startswith(('physics:', 'physx'))]
        if not props and not physics_apis:
            continue
        target = physics.OverridePrim(prim.GetPath())
        if physics_apis:
            remaining = [name for name in prim.GetAppliedSchemas() if name not in physics_apis]
            prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(remaining))
            operation = Sdf.TokenListOp()
            operation.prependedItems = physics_apis
            target.SetMetadata('apiSchemas', operation)
        for prop in props:
            Sdf.CopySpec(geometry.GetRootLayer(), prop.GetPath(), physics.GetRootLayer(), prop.GetPath())
            prim.RemoveProperty(prop.GetName())
    geometry.GetRootLayer().Save()
    physics.GetRootLayer().Save()
    asset.GetRootLayer().Clear()
    asset.GetRootLayer().subLayerPaths = ['./vehicle-physics.usda', './vehicle-geometry.usda']
    _metadata(asset, '/Vehicle', 'component')
    Usd.ModelAPI(asset.GetDefaultPrim()).SetAssetName('NativePhysxCar')
    Usd.ModelAPI(asset.GetDefaultPrim()).SetAssetVersion('v01')
    asset.GetRootLayer().Save()


def compose_factory_vehicle(stage, factory, physics_hz, directory):
    """Build a portable referenced scene before registering or starting physics."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from traffic.wheel_geometry import author_explicit_wheel_axes
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    assets = directory / 'assets'
    assets.mkdir()
    if stage.GetPrimAtPath('/World') and stage.GetPrimAtPath('/World').GetChildren():
        raise ValueError('Clean physical scene requires an empty project namespace')
    source = Usd.Stage.CreateInMemory('factory-authoring-only.usda')
    UsdGeom.Xform.Define(source, '/World')
    source.SetDefaultPrim(source.GetPrimAtPath('/World'))
    vehicles, wheels = [], []
    factory.create4WheeledCarsScenario(source, 1., 1, driveMode=factory.DRIVE_NONE,
        axes=factory.AxesIndices(2, 0, 1), timeStepsPerSecond=physics_hz,
        createCollisionShapesForWheels=True, vehiclePathsOut=vehicles,
        wheelAttachmentPathsOut=wheels)
    author_explicit_wheel_axes(source, wheels[0])
    UsdPhysics.Scene(source.GetPrimAtPath('/World/PhysicsScene')).GetGravityMagnitudeAttr().Set(9.81)
    mapping = {vehicles[0]: PATHS.chassis,
        vehicles[0] + '/ChassisRender': PATHS.chassis + '/Visuals/Body',
        vehicles[0] + '/ChassisCollision': PATHS.chassis + '/Colliders/Chassis',
        '/World/GroundPlane': PATHS.ground,
        '/World/PhysicsScene': PATHS.physics,
        '/World/SphereLight': '/World/Lighting/Key'}
    resources = ('FrontWheel', 'RearWheel', 'FrontTire', 'RearTire',
                 'FrontSuspension', 'RearSuspension')
    for prim in source.GetDefaultPrim().GetChildren():
        old = str(prim.GetPath())
        if old not in mapping:
            mapping[old] = ((PATHS.vehicle + '/Physics/' if prim.GetName() in resources
                            else '/World/Physics/Resources/') + prim.GetName())
    # Normalize subtrees on the isolated source, never an attached simulation.
    # Copy the chassis once, then edit its two static child names with USD's
    # namespace editor so relationship targets remain valid in the source.
    for old, name in ((vehicles[0] + '/ChassisRender', 'Visuals/Body'),
                      (vehicles[0] + '/ChassisCollision', 'Colliders/Chassis')):
        new = vehicles[0] + '/' + name
        UsdGeom.Scope.Define(source, str(Sdf.Path(new).GetParentPath()))
        editor = Usd.NamespaceEditor(source)
        editor.MovePrimAtPath(old, new)
        if not editor.CanApplyEdits() or not editor.ApplyEdits():
            raise RuntimeError('Could not normalize isolated Factory geometry')
        del mapping[old]
    vehicle_mapping = {p: target for p, target in mapping.items() if target.startswith(PATHS.vehicle + '/')}
    vehicle_asset, external = _copy_asset(source, vehicle_mapping, assets/'vehicle.usda',
                                          '/Vehicle', PATHS.vehicle, mapping)
    UsdGeom.Scope.Define(vehicle_asset, '/Vehicle/Chassis/Sensors')
    for wheel in ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel'):
        Usd.ModelAPI(vehicle_asset.GetPrimAtPath('/Vehicle/Chassis/' + wheel)).SetKind('subcomponent')
    vehicle_asset.GetRootLayer().Save()
    _separate_vehicle_layers(vehicle_asset, assets)
    ground, ground_external = _copy_asset(source, {'/World/GroundPlane': PATHS.ground},
        assets/'ground.usda', '/Ground', PATHS.ground, mapping)
    external += ground_external
    layout = _new_stage(directory/'layout.usda')
    UsdGeom.Xform.Define(layout, '/World')
    _metadata(layout, '/World', 'assembly')
    for name in ('Environment', 'Environment/Props', 'Vehicles'):
        Usd.ModelAPI(UsdGeom.Xform.Define(layout, '/World/'+name).GetPrim()).SetKind('group')
    for name in ('Physics', 'Lighting', 'Cameras', 'Debug'):
        UsdGeom.Scope.Define(layout, '/World/'+name)
    for target, file in ((PATHS.vehicle, 'vehicle.usda'), (PATHS.ground, 'ground.usda')):
        prim = layout.DefinePrim(target)
        prim.GetReferences().AddReference('./assets/'+file)
    layout.GetPrimAtPath(PATHS.vehicle).SetCustomDataByKey('vehicle_id', PATHS.vehicle_id)
    layout.GetPrimAtPath(PATHS.vehicle).SetCustomDataByKey('movement_authority', 'isaac_physx')
    physics = _new_stage(directory/'physics.usda')
    UsdGeom.Xform.Define(physics, '/World')
    _metadata(physics, '/World', 'assembly')
    for old, new in mapping.items():
        if new.startswith('/World/Physics/') or new.startswith('/World/Lighting/'):
            owner = layout if new.startswith('/World/Lighting/') else physics
            UsdGeom.Scope.Define(owner, str(Sdf.Path(new).GetParentPath()))
            Sdf.CopySpec(source.GetRootLayer(), old, owner.GetRootLayer(), new)
    for prim in physics.Traverse():
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            if targets:
                rel.SetTargets([_map_path(p, mapping) for p in targets])
    for path, relationship, targets in external:
        prop_path = Sdf.Path(path)
        prim = physics.OverridePrim(prop_path.GetPrimPath())
        if not relationship:
            raise ValueError('Unexpected external Factory attribute connection: ' + path)
        prim.CreateRelationship(prop_path.name, custom=False).SetTargets(targets)
    layout.GetRootLayer().Save()
    physics.GetRootLayer().Save()
    layers = [physics.GetRootLayer().identifier, layout.GetRootLayer().identifier]
    composed = Usd.Stage.CreateInMemory('pre-runtime-contract.usda')
    composed.GetRootLayer().subLayerPaths = layers
    from usd.scene_validation import compare_factory_contract
    contract = compare_factory_contract(source, composed, lambda p: _map_path(p, mapping))
    # Declare the scene type explicitly in the live stage before composition.
    # Isaac's scene-addition callback misses a PhysicsScene introduced only by
    # a parent subtree resync. All authored physics values remain in physics.usda;
    # this declaration creates no extra scene, stepping owner or solver settings.
    UsdGeom.Scope.Define(stage, '/World/Physics')
    UsdPhysics.Scene.Define(stage, PATHS.physics)
    stage.GetRootLayer().subLayerPaths = layers
    _metadata(stage, '/World', 'assembly')
    stage.GetRootLayer().customLayerData = {'physical_scene_schema': PATHS.version}
    return dict(paths=asdict(PATHS), wheel_paths=PATHS.wheels,
                factory_contract=contract,
                factory_sha256=hashlib.sha256(Path(factory.__file__).read_bytes()).hexdigest(),
                assembly_directory=str(directory), references_not_instances=True,
                payload_policy='No heavy optional content in this small fixture; required assets use references')


def author_scene_details(stage, directory, box):
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
    asset = _new_stage(Path(directory)/'assets/barrier.usda')
    cube = UsdGeom.Cube.Define(asset, '/Barrier')
    cube.CreateSizeAttr(1.)
    cube.CreateDisplayColorAttr([Gf.Vec3f(.95, .23, .06)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    _metadata(asset, '/Barrier', 'component')
    asset.GetRootLayer().Save()
    # These are assembly opinions, not runtime edits to the referenced prop.
    from pxr import Sdf, Usd
    layout = Sdf.Layer.FindOrOpen(str(Path(directory)/'layout.usda'))
    with Usd.EditContext(stage, layout):
        prim = stage.DefinePrim(PATHS.barrier)
        prim.GetReferences().AddReference('./assets/barrier.usda')
        cube = UsdGeom.Cube(prim)
        cube.AddTranslateOp().Set(Gf.Vec3d(box.x_m, box.y_m, .75))
        cube.AddScaleOp().Set(Gf.Vec3f(box.length_m, box.width_m, 1.5))
        UsdLux.DomeLight.Define(stage, '/World/Lighting/Dome').CreateIntensityAttr(1000)
        source_camera = stage.GetPrimAtPath('/OmniverseKit_Persp')
        for path in (PATHS.follow_camera, PATHS.overview_camera):
            camera = UsdGeom.Camera.Define(stage, path)
            # Preserve the original viewport optics/exposure, but not its hidden
            # runtime ownership flags or transform. Poses are initialized below.
            camera.CreateFocalLengthAttr(18.147562)
            camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 10000000.))
            camera.CreateFocusDistanceAttr(400.)
            if source_camera:
                camera.GetPrim().SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(source_camera.GetAppliedSchemas()))
                for attr in source_camera.GetAuthoredAttributes():
                    name = attr.GetName()
                    if name in UsdGeom.Camera.GetSchemaAttributeNames(False) or name.startswith('exposure:'):
                        if attr.Get() is not None:
                            camera.GetPrim().CreateAttribute(name, attr.GetTypeName(), custom=attr.IsCustom()).Set(attr.Get())
    layout.Save()


def validate_scene(stage, *, require_sensor=True):
    """Fail closed on missing content, escaped targets, or unexpected project roots."""
    from pxr import Usd, UsdGeom, UsdPhysics
    errors = [str(error) for error in stage.GetCompositionErrors()]
    required = [PATHS.vehicle, PATHS.chassis, PATHS.physics, PATHS.ground,
                PATHS.barrier, PATHS.follow_camera, PATHS.overview_camera] + PATHS.wheels
    if require_sensor:
        required.append(PATHS.lidar)
    for path in required:
        if not stage.GetPrimAtPath(path) or not stage.GetPrimAtPath(path).IsLoaded():
            errors.append('Missing/unloaded required prim: '+path)
    if (str(stage.GetDefaultPrim().GetPath()) != '/World' or
            UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1.):
        errors.append('Invalid defaultPrim/units/axis')
    expected = {'Environment', 'Vehicles', 'Physics', 'Lighting', 'Cameras', 'Debug'}
    world = stage.GetPrimAtPath('/World')
    if world:
        unexpected = {p.GetName() for p in world.GetChildren()} - expected
        errors.extend('Unexpected project root: '+p for p in sorted(unexpected))
    for path in (PATHS.vehicle, PATHS.ground, PATHS.barrier, '/World/Environment/Highway'):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.HasAuthoredReferences() or prim.IsInstance():
            errors.append('Expected editable referenced component: '+path)
    model_kinds = {'/World': 'assembly', '/World/Environment': 'group',
        '/World/Environment/Props': 'group', '/World/Vehicles': 'group',
        PATHS.vehicle: 'component', PATHS.ground: 'component',
        PATHS.barrier: 'component', '/World/Environment/Highway': 'component'}
    for path, kind in model_kinds.items():
        prim = stage.GetPrimAtPath(path)
        if not prim or Usd.ModelAPI(prim).GetKind() != kind or not prim.IsModel():
            errors.append('Invalid model kind/ancestry: '+path)
    vehicle = stage.GetPrimAtPath(PATHS.vehicle)
    if vehicle and (vehicle.GetCustomDataByKey('vehicle_id') != PATHS.vehicle_id or
                    vehicle.GetCustomDataByKey('movement_authority') != 'isaac_physx'):
        errors.append('Vehicle identity/motion authority changed')
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World/'):
            continue  # Kit owns /Render, /Replicator and its default cameras.
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if target.IsAbsolutePath() and not stage.GetObjectAtPath(target):
                    errors.append('Unresolved relationship: '+str(rel.GetPath())+' -> '+str(target))
        for attr in prim.GetAttributes():
            for target in attr.GetConnections():
                if not stage.GetObjectAtPath(target):
                    errors.append('Unresolved connection: '+str(attr.GetPath())+' -> '+str(target))
    if not UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(PATHS.chassis)):
        errors.append('Chassis rigid body missing')
    report = dict(passed=not errors, errors=errors, paths=asdict(PATHS),
        prims=[dict(path=str(p.GetPath()), type=p.GetTypeName(),
                   kind=Usd.ModelAPI(p).GetKind(), reference=p.HasAuthoredReferences())
               for p in stage.Traverse() if str(p.GetPath()).startswith('/World')],
        layers=[layer.identifier for layer in stage.GetUsedLayers()],
        runtime_namespace_exceptions=['/Render', '/Replicator', '/OmniverseKit_*', '/ActionGraph', '/WriterOrchestrator'])
    if errors:
        raise ValueError('Invalid physical scene: ' + '; '.join(errors))
    return report


def save_composed_scene(stage, directory, debug_layer):
    """Save composition, not flattening; do not persist runtime-generated graphs.

    Only /World project specs from the live root are exported. RTX render
    products/writer graphs are re-created by the runner, never portable assets.
    """
    from pxr import Sdf, Usd, UsdGeom
    directory = Path(directory)
    debug_layer.Export(str(directory/'debug.usda'))
    runtime = _new_stage(directory/'initial-state.usda')
    root = stage.GetRootLayer()
    if root.GetPrimAtPath('/World'):
        Sdf.CopySpec(root, '/World', runtime.GetRootLayer(), '/World')
    else:
        UsdGeom.Xform.Define(runtime, '/World')
    # The live RTX API resolves its NVIDIA profile through an HTTPS resolver.
    # Preserve that single, already loaded source layer in the saved package so
    # ordinary OpenUSD can reopen it offline. This changes only the saved copy,
    # not the live sensor's configuration, reference or writer bindings.
    sensor = stage.GetPrimAtPath(PATHS.lidar)
    reference_op = sensor.GetMetadata('references') if sensor else None
    references = reference_op.GetAppliedItems() if reference_op else []
    if references:
        if len(references) != 1 or not references[0].assetPath:
            raise ValueError('Expected a single external RTX profile reference')
        reference = references[0]
        profile = Sdf.Layer.Find(reference.assetPath)
        if profile is None or profile.GetExternalReferences():
            raise ValueError('RTX profile must be loaded and have no external layer dependencies')
        profile_stage = Usd.Stage.Open(profile)
        for prim in profile_stage.Traverse():
            for attr in prim.GetAuthoredAttributes():
                if attr.GetTypeName() in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                    raise ValueError('RTX profile asset-valued attributes require dependency packaging')
        target = directory/'assets/lidar-profile.usda'
        if target.exists() or not profile.Export(str(target)):
            raise RuntimeError('Cannot export fresh RTX profile snapshot')
        runtime.GetPrimAtPath(PATHS.lidar).GetReferences().SetReferences([
            Sdf.Reference('./assets/lidar-profile.usda', reference.primPath, reference.layerOffset)])
    _metadata(runtime, '/World', 'assembly')
    runtime.GetRootLayer().Save()
    entry = _new_stage(directory/'world.usda')
    entry.GetRootLayer().subLayerPaths = ['./initial-state.usda', './debug.usda',
        './physics.usda', './road-layout.usda', './layout.usda']
    _metadata(entry, '/World', 'assembly')
    entry.GetRootLayer().customLayerData = {
        'physical_scene_schema': PATHS.version,
        'runtime_note': 'Initial authored scene, not a replay. Run the Python demo to initialize control and RTX writers.'}
    entry.GetRootLayer().Save()
    validate_scene(entry)
    return str(directory/'world.usda')


def asset_hashes(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(Path(directory).rglob('*.usda'))}
