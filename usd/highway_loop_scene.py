"""Portable V02 highway assembly for the separate scripted PhysX loop fixture.

The published V02 files are read-only inputs. Required road collision is a
normal sublayer; the large render/navigation payload is loaded during driving.
This module never advances physics or modifies vehicle poses during stepping.
"""
import hashlib
import json
import math
from pathlib import Path
import shutil

from usd.physical_scene import PATHS, _metadata, _new_stage


HIGHWAY = '/World/Environment/Highway'
ROAD_COLLIDER = HIGHWAY + '/Colliders/RoadSurface'
GROUND_GROUP = '/World/Physics/Resources/GroundSurfaceCollisionGroup'
QUERY_GROUP = '/World/Physics/Resources/VehicleGroundQueryGroup'
TARMAC = '/World/Physics/Resources/TarmacMaterial'
SCHEMA_VERSION = 1


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _source_contract(source_directory):
    """Fail before authoring on modified inputs or unsupported dependencies."""
    source_directory = Path(source_directory).resolve()
    manifest = json.loads((source_directory / 'manifest.json').read_text(encoding='utf-8'))
    required = ('highway_v02.usda', 'navigation.json', 'parameters.json')
    for name in required:
        expected = manifest.get('files', {}).get(name)
        if not expected or _digest(source_directory / name) != expected:
            raise ValueError('V02 source manifest mismatch: ' + name)
    return source_directory, {name: _digest(source_directory / name) for name in required}


def _remap_source_paths(stage):
    """Close V02 cross-subtree targets before referencing the asset elsewhere."""
    from pxr import Sdf, Usd
    replacements = {
        '/World/Physics/RoadCollider': '/Highway/Colliders/RoadSurface',
        '/World/Physics/Scene': '',
        '/World': '/Highway',
    }
    for prim in stage.Traverse():
        for prop in list(prim.GetRelationships()) + list(prim.GetAttributes()):
            is_rel = isinstance(prop, Usd.Relationship)
            targets = prop.GetTargets() if is_rel else prop.GetConnections()
            if not targets:
                continue
            mapped = []
            for path in targets:
                for old, new in replacements.items():
                    if path.HasPrefix(Sdf.Path(old)):
                        if new:
                            mapped.append(path.ReplacePrefix(Sdf.Path(old), Sdf.Path(new)))
                        break
                else:
                    # CopySpec remaps intra-subtree targets automatically.
                    if path.HasPrefix(Sdf.Path('/Highway')):
                        mapped.append(path)
                    else:
                        raise ValueError('Unsupported V02 external target: ' + str(path))
            (prop.SetTargets if is_rel else prop.SetConnections)(mapped)


def _strip_physics_from_visuals(stage):
    from pxr import Sdf
    for prim in stage.Traverse():
        apis = [name for name in prim.GetAppliedSchemas()
                if not name.startswith(('Physics', 'Physx'))]
        prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(apis))
        for prop in list(prim.GetAuthoredProperties()):
            if prop.GetName().startswith(('physics:', 'physx')) or prop.GetName() == 'material:binding:physics':
                prim.RemoveProperty(prop.GetName())


def install_highway_loop(stage, scene_directory, source_directory):
    """Install before physics start, after ``PhysxVehicle`` clean composition.

    All authored files are new per-run package files. Factory's infinite plane
    is deactivated, V02's landscape remains visual-only, and tire queries use
    Factory's existing tarmac material/table and road collision filtering.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    directory = Path(scene_directory).resolve()
    assets = directory / 'assets'
    if not (directory / 'physics.usda').is_file() or not (directory / 'layout.usda').is_file():
        raise ValueError('Compose the physical vehicle scene before installing the highway')
    for path in (PATHS.chassis, PATHS.physics, GROUND_GROUP, QUERY_GROUP, TARMAC):
        if not stage.GetPrimAtPath(path):
            raise ValueError('Missing Factory integration contract: ' + path)
    if stage.GetPrimAtPath(HIGHWAY):
        raise ValueError('Highway is already installed')
    source_directory, source_hashes = _source_contract(source_directory)
    source = Usd.Stage.Open(str(source_directory / 'highway_v02.usda'))
    if source.GetCompositionErrors() or source.GetRootLayer().GetExternalReferences():
        raise ValueError('V02 source must be a self-contained, valid published stage')
    if UsdGeom.GetStageUpAxis(source) != 'Z' or UsdGeom.GetStageMetersPerUnit(source) != 1:
        raise ValueError('V02 source coordinate contract changed')
    names = ('highway.usda', 'highway-geometry.usdc', 'highway-physics.usda',
             'highway-navigation.json', 'highway-parameters.json', 'highway-source-manifest.json')
    if any((assets / name).exists() for name in names) or (directory / 'highway-layout.usda').exists():
        raise FileExistsError('Refusing to overwrite highway package files')

    geometry = _new_stage(assets / 'highway-geometry.usdc')
    UsdGeom.Xform.Define(geometry, '/Highway')
    for name in ('RoadNetwork', 'Navigation', 'SpawnPoints', 'DivergeZones', 'MergeZones', 'Looks', 'Ground'):
        if not Sdf.CopySpec(source.GetRootLayer(), '/World/' + name,
                            geometry.GetRootLayer(), '/Highway/' + name):
            raise ValueError('Missing V02 environment subtree: ' + name)
    _remap_source_paths(geometry)
    _strip_physics_from_visuals(geometry)
    _metadata(geometry, '/Highway', 'component')
    geometry.GetRootLayer().Save()

    physics = _new_stage(assets / 'highway-physics.usda')
    UsdGeom.Xform.Define(physics, '/Highway')
    UsdGeom.Scope.Define(physics, '/Highway/Colliders')
    if not Sdf.CopySpec(source.GetRootLayer(), '/World/Physics/RoadCollider',
                        physics.GetRootLayer(), '/Highway/Colliders/RoadSurface'):
        raise ValueError('V02 road support collider is missing')
    _remap_source_paths(physics)
    collider = physics.GetPrimAtPath('/Highway/Colliders/RoadSurface')
    # These two external relationships are owned by the scene assembly, not
    # authored out-of-scope across the environment reference boundary.
    collider.GetRelationship('physics:simulationOwner').ClearTargets(False)
    collider.GetRelationship('material:binding:physics').ClearTargets(False)
    _metadata(physics, '/Highway', 'component')
    physics.GetRootLayer().Save()

    interface = _new_stage(assets / 'highway.usda')
    prim = UsdGeom.Xform.Define(interface, '/Highway').GetPrim()
    interface.GetRootLayer().subLayerPaths = ['./highway-physics.usda']
    prim.GetPayloads().AddPayload('./highway-geometry.usdc', '/Highway')
    _metadata(interface, '/Highway', 'component')
    Usd.ModelAPI(prim).SetAssetName('CircularHighwayV02')
    Usd.ModelAPI(prim).SetAssetVersion('loop-integration-v01')
    prim.SetCustomDataByKey('source_version', '_v02')
    prim.SetCustomDataByKey('source_stage_sha256', source_hashes['highway_v02.usda'])
    prim.SetCustomDataByKey('required_payload', 'Load before physics; do not unload while driving')
    interface.GetRootLayer().Save()
    for src, dst in (('navigation.json', 'highway-navigation.json'),
                     ('parameters.json', 'highway-parameters.json'),
                     ('manifest.json', 'highway-source-manifest.json')):
        shutil.copy2(source_directory / src, assets / dst)

    layout = _new_stage(directory / 'highway-layout.usda')
    UsdGeom.Xform.Define(layout, '/World')
    _metadata(layout, '/World', 'assembly')
    for name in ('Environment', 'Vehicles'):
        Usd.ModelAPI(UsdGeom.Xform.Define(layout, '/World/' + name).GetPrim()).SetKind('group')
    layout.DefinePrim(HIGHWAY).GetReferences().AddReference('./assets/highway.usda')
    layout.OverridePrim(PATHS.ground).SetActive(False)
    road = layout.OverridePrim(ROAD_COLLIDER)
    road.CreateRelationship('physics:simulationOwner', custom=False).SetTargets([PATHS.physics])
    UsdShade.MaterialBindingAPI.Apply(road).Bind(UsdShade.Material(stage.GetPrimAtPath(TARMAC)),
                                               materialPurpose='physics')
    # Replace only the infinite plane's ground membership, preserving any other
    # road entries. Do not add support to the query group (that group filters
    # chassis and wheel shapes out of suspension raycasts).
    group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(GROUND_GROUP))
    members = group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets()
    members = [p for p in members if not p.HasPrefix(Sdf.Path(PATHS.ground))]
    includes = layout.OverridePrim(GROUND_GROUP).CreateRelationship('collection:colliders:includes', custom=False)
    includes.SetTargets(members + [Sdf.Path(ROAD_COLLIDER)])
    layout.GetRootLayer().customLayerData = {'highway_loop_schema': SCHEMA_VERSION,
        'motion_authority': 'isaac_physx', 'ground_policy': 'Road collider only; landscape is visual-only'}
    layout.GetRootLayer().Save()
    stage.GetRootLayer().subLayerPaths.insert(0, layout.GetRootLayer().identifier)
    stage.Load(HIGHWAY)
    report = validate_highway_loop_scene(stage)
    if _source_contract(source_directory)[1] != source_hashes:
        raise RuntimeError('Published V02 source changed during packaging')
    return dict(schema_version=SCHEMA_VERSION, highway_path=HIGHWAY, collider_path=ROAD_COLLIDER,
                source_hashes=source_hashes, validation=report, sumo_required=False,
                road_support='V02 triangle mesh; no infinite plane or landscape collision',
                tire_material=TARMAC, collision_group=GROUND_GROUP,
                payload_policy='Geometry/navigation payload must remain loaded; collision is a required sublayer')


def spawn_vehicle_on_loop(stage, angle_rad, radius_m=500., center_xy=(0., 0.)):
    """Author an initial pose only; caller must invoke before physics starts."""
    from pxr import Gf, UsdGeom
    if len(center_xy) != 2 or not all(math.isfinite(v) for v in (angle_rad, radius_m, *center_xy)) or radius_m <= 0:
        raise ValueError('Loop spawn requires finite coordinates and positive radius')
    chassis = stage.GetPrimAtPath(PATHS.chassis)
    if not chassis:
        raise ValueError('Physical chassis is missing')
    xform = UsdGeom.Xformable(chassis)
    ops = xform.GetOrderedXformOps()
    by_type = {op.GetOpType(): op for op in ops}
    allowed = {UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeOrient}
    if any(op.GetOpType() not in allowed for op in ops) or len(by_type) != len(ops):
        raise ValueError('Unsupported chassis initial transform stack')
    translate = by_type.get(UsdGeom.XformOp.TypeTranslate)
    if translate is None:
        raise ValueError('Chassis rest height must be authored by the physical asset')
    z = float(translate.Get()[2])
    x, y = center_xy[0] + radius_m * math.cos(angle_rad), center_xy[1] + radius_m * math.sin(angle_rad)
    yaw = math.atan2(math.sin(angle_rad + math.pi / 2), math.cos(angle_rad + math.pi / 2))
    xyz_type = Gf.Vec3f if translate.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    translate.Set(xyz_type(x, y, z))
    orient = by_type.get(UsdGeom.XformOp.TypeOrient)
    if orient is None:
        orient = xform.AddOrientOp()
    quat_type = Gf.Quatf if orient.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Quatd
    vector_type = Gf.Vec3f if quat_type is Gf.Quatf else Gf.Vec3d
    orient.Set(quat_type(math.cos(yaw / 2), vector_type(0, 0, math.sin(yaw / 2))))
    return dict(position_m=[x, y, z], yaw_rad=yaw, radius_m=radius_m,
                angle_rad=angle_rad, center_xy=list(center_xy),
                phase='pre-physics initial pose; never a runtime movement command')


def validate_highway_loop_scene(stage):
    """Validate integration/composition without claiming a physical driving test."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    errors = [str(e) for e in stage.GetCompositionErrors()]
    required = [PATHS.physics, PATHS.vehicle, PATHS.chassis, HIGHWAY, ROAD_COLLIDER,
                HIGHWAY + '/RoadNetwork', HIGHWAY + '/Navigation', TARMAC, GROUND_GROUP, QUERY_GROUP] + PATHS.wheels
    for path in required:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsActive() or not prim.IsLoaded():
            errors.append('Missing/unloaded required prim: ' + path)
    if (not stage.GetDefaultPrim() or str(stage.GetDefaultPrim().GetPath()) != '/World'
            or UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1):
        errors.append('Invalid world defaultPrim/units/axis')
    scenes = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if scenes != [PATHS.physics]:
        errors.append('Expected exactly one active physical scene: ' + repr(scenes))
    ground = stage.GetPrimAtPath(PATHS.ground)
    if ground and ground.IsActive():
        errors.append('Factory infinite plane is still active')
    landscape = stage.GetPrimAtPath(HIGHWAY + '/Ground')
    if landscape and landscape.HasAPI(UsdPhysics.CollisionAPI):
        errors.append('Landscape must remain visual-only')
    kinds = {'/World': 'assembly', '/World/Environment': 'group', '/World/Vehicles': 'group',
             HIGHWAY: 'component', PATHS.vehicle: 'component'}
    for path, kind in kinds.items():
        prim = stage.GetPrimAtPath(path)
        if not prim or Usd.ModelAPI(prim).GetKind() != kind or not prim.IsModel():
            errors.append('Invalid model ancestry: ' + path)
    highway = stage.GetPrimAtPath(HIGHWAY)
    if not highway or not highway.HasAuthoredReferences() or not highway.HasPayload() or highway.IsInstance():
        errors.append('Highway must be an editable referenced asset with a required payload')
    forbidden = ('Lighting', 'Cameras', 'Physics', 'Debug')
    for name in forbidden:
        if stage.GetPrimAtPath(HIGHWAY + '/' + name):
            errors.append('Unexpected source scene-owned branch: ' + name)
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World'):
            continue
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if target.IsAbsolutePath() and not stage.GetObjectAtPath(target):
                    errors.append('Unresolved relationship: ' + str(rel.GetPath()) + ' -> ' + str(target))
        for attr in prim.GetAttributes():
            for target in attr.GetConnections():
                if not stage.GetObjectAtPath(target):
                    errors.append('Unresolved connection: ' + str(attr.GetPath()))
    road = stage.GetPrimAtPath(ROAD_COLLIDER)
    if road:
        if not road.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(road).GetCollisionEnabledAttr().Get():
            errors.append('Road support collision is missing/disabled')
        if road.GetRelationship('physics:simulationOwner').GetTargets() != [Sdf.Path(PATHS.physics)]:
            errors.append('Road support must use the vehicle physics scene')
        material, _ = UsdShade.MaterialBindingAPI(road).ComputeBoundMaterial('physics')
        if not material or str(material.GetPath()) != TARMAC:
            errors.append('Road support must bind Factory tarmac tire-table material')
    group = stage.GetPrimAtPath(GROUND_GROUP)
    if group and Sdf.Path(ROAD_COLLIDER) not in UsdPhysics.CollisionGroup(group).GetCollidersCollectionAPI().GetIncludesRel().GetTargets():
        errors.append('Road support is missing Factory ground collision-group membership')
    wheel_group = '/World/Physics/Resources/VehicleWheelCollisionGroup'
    chassis_group = '/World/Physics/Resources/VehicleChassisCollisionGroup'
    expected_filters = {GROUND_GROUP: [wheel_group], wheel_group: [GROUND_GROUP],
                        QUERY_GROUP: [wheel_group, chassis_group]}
    for path, targets in expected_filters.items():
        prim = stage.GetPrimAtPath(path)
        actual = UsdPhysics.CollisionGroup(prim).GetFilteredGroupsRel().GetTargets() if prim else []
        if any(Sdf.Path(target) not in actual for target in targets):
            errors.append('Factory tire/contact collision filters changed: ' + path)
        if path == QUERY_GROUP and Sdf.Path(GROUND_GROUP) in actual:
            errors.append('Suspension query incorrectly filters out the road group')
    tire_contracts = []
    for path in PATHS.wheels:
        wheel = stage.GetPrimAtPath(path)
        if wheel and wheel.GetRelationship('physxVehicleWheelAttachment:collisionGroup').GetTargets() != [Sdf.Path(QUERY_GROUP)]:
            errors.append('Wheel ground-query group changed: ' + path)
        if not wheel:
            continue
        tire_targets = wheel.GetRelationship('physxVehicleWheelAttachment:tire').GetTargets()
        tire = stage.GetPrimAtPath(tire_targets[0]) if len(tire_targets) == 1 else wheel
        table_targets = tire.GetRelationship('physxVehicleTire:frictionTable').GetTargets() if tire else []
        table = stage.GetPrimAtPath(table_targets[0]) if len(table_targets) == 1 else None
        materials = table.GetRelationship('groundMaterials').GetTargets() if table else []
        values = table.GetAttribute('frictionValues').Get() if table else None
        if (values is None or len(values) != len(materials) or Sdf.Path(TARMAC) not in materials
                or any(not math.isfinite(value) or value <= 0 for value in values)):
            errors.append('Missing/invalid tire friction-table tarmac mapping: ' + path)
        else:
            tire_contracts.append(dict(wheel=path, table=str(table.GetPath()),
                                       tarmac_coefficient=float(values[materials.index(Sdf.Path(TARMAC))])))
    report = dict(passed=not errors, errors=errors, schema_version=SCHEMA_VERSION,
                  physics_scenes=scenes, road_collider=ROAD_COLLIDER,
                  wheel_tire_contracts=tire_contracts,
                  layers=[layer.identifier for layer in stage.GetUsedLayers()],
                  prims=[dict(path=str(p.GetPath()), type=p.GetTypeName(),
                              kind=Usd.ModelAPI(p).GetKind(), reference=p.HasAuthoredReferences())
                         for p in stage.Traverse() if str(p.GetPath()).startswith('/World')])
    if errors:
        raise ValueError('Invalid highway loop scene: ' + '; '.join(errors))
    return report


def save_highway_loop_scene(stage, directory, dynamic_layer=None):
    """Save an initial composed preview, not a flattened replay or runtime state.

    Invoke before stepping. Only project-root authored opinions and the explicit
    preview layer are saved; Kit-owned render products/graphs stay runtime-only.
    """
    from pxr import Sdf, UsdGeom
    directory = Path(directory).resolve()
    for name in ('initial-state.usda', 'preview.usda', 'world.usda'):
        if (directory / name).exists():
            raise FileExistsError('Refusing to overwrite saved initial scene: ' + name)
    validate_highway_loop_scene(stage)
    preview = _new_stage(directory / 'preview.usda')
    if dynamic_layer is not None and dynamic_layer.GetPrimAtPath('/World'):
        Sdf.CopySpec(dynamic_layer, '/World', preview.GetRootLayer(), '/World')
    else:
        UsdGeom.Xform.Define(preview, '/World')
    _metadata(preview, '/World', 'assembly')
    preview.GetRootLayer().Save()
    initial = _new_stage(directory / 'initial-state.usda')
    Sdf.CopySpec(stage.GetRootLayer(), '/World', initial.GetRootLayer(), '/World')
    _metadata(initial, '/World', 'assembly')
    initial.GetRootLayer().Save()
    entry = _new_stage(directory / 'world.usda')
    entry.GetRootLayer().subLayerPaths = ['./initial-state.usda', './preview.usda',
        './highway-layout.usda', './physics.usda', './layout.usda']
    _metadata(entry, '/World', 'assembly')
    entry.GetRootLayer().customLayerData = {'highway_loop_schema': SCHEMA_VERSION,
        'runtime_note': 'Initial scene, not a replay. Python owns driver commands; PhysX owns vehicle motion.'}
    entry.GetRootLayer().Save()
    validate_highway_loop_scene(entry)
    return str(directory / 'world.usda')
