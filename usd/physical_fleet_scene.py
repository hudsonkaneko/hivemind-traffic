"""Pre-play extension of a composed Factory scene with a second physical car.

The source asset and all existing files remain immutable. One new assembly
layer references the same vehicle asset, remaps complete relationship/connection
target lists and extends shared collision memberships. PhysX is the only motion
authority after the owner's reset. This module never starts or steps a runtime.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import math
from numbers import Real
from pathlib import Path
import sys

from traffic.namespaced_physx_vehicle import PHYSICS_SCENE, vehicle_binding_paths
from usd.physical_scene import asset_hashes

ROOT = Path(__file__).resolve().parents[1]
RESOURCE_ROOT = '/World/Physics/Resources/'
CHASSIS_GROUP = RESOURCE_ROOT + 'VehicleChassisCollisionGroup'
WHEEL_GROUP = RESOURCE_ROOT + 'VehicleWheelCollisionGroup'
QUERY_GROUP = RESOURCE_ROOT + 'VehicleGroundQueryGroup'
GROUND_GROUP = RESOURCE_ROOT + 'GroundSurfaceCollisionGroup'


def _pose(pose):
    if not isinstance(pose, (tuple, list)) or len(pose) != 7:
        raise ValueError('Pose must be world chassis (x,y,z,qx,qy,qz,qw)')
    if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in pose):
        raise ValueError('Pose must contain seven finite real values')
    values = tuple(map(float, pose))
    if abs(sum(v*v for v in values[3:]) - 1.) > 1e-6:
        raise ValueError('Pose quaternion must have unit norm')
    return values


def _remap(path, source, destination, owner):
    from pxr import Sdf
    path = Sdf.Path(path)
    if not path.IsAbsolutePath():
        path = path.MakeAbsolutePath(owner)
    return path.ReplacePrefix(source, destination) if path.HasPrefix(source) else path


def _property_metadata(source, destination):
    for name, value in source.GetAllAuthoredMetadata().items():
        if name not in {'default', 'timeSamples', 'connectionPaths', 'targetPaths',
                        'typeName', 'custom', 'variability'}:
            destination.SetMetadata(name, value)


def _asset_reference(stage, root, expected):
    """Resolve the placement's reference in its owning layer, not the cwd."""
    from pxr import Sdf
    references = []
    for spec in root.GetPrimStack():
        for ref in spec.referenceList.GetAppliedItems():
            if ref.assetPath:
                resolved = Path(Sdf.ComputeAssetPathRelativeToLayer(spec.layer, ref.assetPath)).resolve()
                references.append((resolved, str(ref.primPath)))
    if references != [(expected, '')] and references != [(expected, '/Vehicle')]:
        raise ValueError('Source must reference exactly this immutable assets/vehicle.usda')


def validate_fleet_scene(stage, vehicle_roots):
    """Static closure/collision checks only; this is not native qualification."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    roots = tuple(vehicle_binding_paths(path) for path in vehicle_roots)
    if len(roots) < 2 or len({p.vehicle for p in roots}) != len(roots):
        raise ValueError('At least two distinct vehicle roots are required')
    errors = [str(e) for e in stage.GetCompositionErrors()]
    if (not stage.GetDefaultPrim() or str(stage.GetDefaultPrim().GetPath()) != '/World'
            or UsdGeom.GetStageMetersPerUnit(stage) != 1. or UsdGeom.GetStageUpAxis(stage) != 'Z'):
        errors.append('Invalid world defaultPrim, units or axes')
    if [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'PhysicsScene'] != [PHYSICS_SCENE]:
        errors.append('Expected one shared physics scene')
    relationships = connections = 0
    for prim in stage.Traverse():
        if not prim.GetPath().HasPrefix('/World'):
            continue
        for prop in (*prim.GetRelationships(), *prim.GetAttributes()):
            rel = isinstance(prop, Usd.Relationship)
            targets = prop.GetTargets() if rel else prop.GetConnections()
            relationships += len(targets) if rel else 0
            connections += len(targets) if not rel else 0
            for target in targets:
                absolute = target if target.IsAbsolutePath() else target.MakeAbsolutePath(prim.GetPath())
                if not stage.GetObjectAtPath(absolute):
                    errors.append('Unresolved target: ' + str(prop.GetPath()) + ' -> ' + str(target))
    groups = {}
    names = {CHASSIS_GROUP, WHEEL_GROUP, QUERY_GROUP, GROUND_GROUP}
    names.update(str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'PhysicsCollisionGroup')
    for name in sorted(names):
        prim = stage.GetPrimAtPath(name)
        if not prim or prim.GetTypeName() != 'PhysicsCollisionGroup':
            errors.append('Missing collision group: ' + name)
            continue
        group = UsdPhysics.CollisionGroup(prim)
        groups[name] = (group.GetCollidersCollectionAPI().ComputeMembershipQuery(),
                        set(map(str, group.GetFilteredGroupsRel().GetTargets())))
        if prim.GetAttribute('physics:invertFilteredGroups').Get() or prim.GetAttribute('physics:mergeGroupName').Get():
            errors.append('Inverted or merged collision groups require a separate fleet contract: ' + name)
    if CHASSIS_GROUP in groups and CHASSIS_GROUP in groups[CHASSIS_GROUP][1]:
        errors.append('Chassis group self-filter disables car-to-car collision')
    required_filters = {CHASSIS_GROUP: {QUERY_GROUP}, WHEEL_GROUP: {GROUND_GROUP, QUERY_GROUP},
                        QUERY_GROUP: {CHASSIS_GROUP, WHEEL_GROUP}, GROUND_GROUP: {WHEEL_GROUP}}
    for group, required in required_filters.items():
        if group in groups and not required <= groups[group][1]:
            errors.append('Required Factory filters changed: ' + group)
    if QUERY_GROUP in groups and GROUND_GROUP in groups[QUERY_GROUP][1]:
        errors.append('Suspension query filters out road support')
    vehicle_colliders = []
    for paths in roots:
        root = stage.GetPrimAtPath(paths.vehicle)
        body = stage.GetPrimAtPath(paths.chassis)
        if (not root or not root.HasAuthoredReferences() or root.IsInstanceable()
                or root.IsInstance() or root.IsInstanceProxy() or Usd.ModelAPI(root).GetKind() != 'component'):
            errors.append('Expected non-instanceable referenced component: ' + paths.vehicle)
        if root and (root.GetCustomDataByKey('vehicle_id') != paths.vehicle_id
                     or root.GetCustomDataByKey('movement_authority') != 'isaac_physx'):
            errors.append('Incorrect vehicle identity/authority: ' + paths.vehicle)
        if not body or not body.IsActive() or not body.IsLoaded() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            errors.append('Missing active rigid chassis: ' + paths.chassis)
            continue
        body_api = UsdPhysics.RigidBodyAPI(body)
        owners = list(map(str, body_api.GetSimulationOwnerRel().GetTargets()))
        if (body_api.GetRigidBodyEnabledAttr().Get() is not True
                or body_api.GetKinematicEnabledAttr().Get() is not False
                or owners not in ([], [PHYSICS_SCENE])):
            errors.append('Chassis must be enabled dynamic with the sole canonical physics owner: ' + paths.chassis)
        rigid = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if [str(p.GetPath()) for p in rigid] != [paths.chassis]:
            errors.append('Vehicle must contain exactly its one chassis rigid body')
        vehicle_colliders.append([p.GetPath() for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI)])
        chassis_colliders = [p for p in Usd.PrimRange(body) if p.HasAPI(UsdPhysics.CollisionAPI)
                            and not any(p.GetPath().HasPrefix(w) for w in paths.wheels)]
        if not chassis_colliders:
            errors.append('Missing chassis collider: ' + paths.chassis)
        for collider in chassis_colliders:
            if CHASSIS_GROUP in groups and not groups[CHASSIS_GROUP][0].IsPathIncluded(collider.GetPath()):
                errors.append('Chassis collider missing group membership: ' + str(collider.GetPath()))
        for wheel_path in paths.wheels:
            wheel = stage.GetPrimAtPath(wheel_path)
            if not wheel or not wheel.IsActive() or not wheel.IsLoaded():
                errors.append('Missing wheel: ' + wheel_path)
                continue
            if wheel.GetRelationship('physxVehicleWheelAttachment:collisionGroup').GetTargets() != [Sdf.Path(QUERY_GROUP)]:
                errors.append('Incorrect shared wheel query binding: ' + wheel_path)
            colliders = [p for p in Usd.PrimRange(wheel) if p.HasAPI(UsdPhysics.CollisionAPI)]
            if not colliders:
                errors.append('Missing wheel collision shape: ' + wheel_path)
            for collider in colliders:
                if WHEEL_GROUP in groups and not groups[WHEEL_GROUP][0].IsPathIncluded(collider.GetPath()):
                    errors.append('Wheel collider missing group membership: ' + str(collider.GetPath()))
        for prim in Usd.PrimRange(root):
            for prop in (*prim.GetRelationships(), *prim.GetAttributes()):
                targets = prop.GetTargets() if isinstance(prop, Usd.Relationship) else prop.GetConnections()
                for target in targets:
                    if any(target.HasPrefix(other.vehicle) for other in roots if other != paths):
                        errors.append('Cross-vehicle resource alias: ' + str(prop.GetPath()))
    memberships = {str(path): {name for name, (query, _) in groups.items() if query.IsPathIncluded(path)}
                   for colliders in vehicle_colliders for path in colliders}
    for index, colliders in enumerate(vehicle_colliders):
        for others in vehicle_colliders[index+1:]:
            for first in colliders:
                for second in others:
                    first_groups, second_groups = memberships[str(first)], memberships[str(second)]
                    if (any(groups[name][1] & second_groups for name in first_groups)
                            or any(groups[name][1] & first_groups for name in second_groups)):
                        errors.append('Cross-car collision suppressed by shared group: ' + str(first) + ' / ' + str(second))
    if errors:
        raise ValueError('Invalid physical fleet: ' + '; '.join(errors[:25]))
    return dict(passed=True, native_validated=False, vehicles=[asdict(p) for p in roots],
                wheel_count=4*len(roots), relationship_targets=relationships, connections=connections,
                shared_physics_scene=PHYSICS_SCENE, car_to_car_collision_not_filtered=True)


def author_extra_vehicle(stage, source_vehicle_path, new_id, pose, scene_directory, *, physics_started=False):
    """Add one referenced car BEFORE native attach/play; pose is world xyzw.

    The caller must truthfully report manual physics initialization with
    ``physics_started``; a stopped timeline alone cannot establish that fact.
    Call before constructing the second body's native handles. Returns the new
    assembly layer and paths; the owner exports its complete world normally.
    """
    if type(physics_started) is not bool or physics_started:
        raise ValueError('Fleet authoring is allowed only before physics initialization')
    timeline = sys.modules.get('omni.timeline')
    if timeline is not None and timeline.get_timeline_interface().is_playing():
        raise ValueError('Fleet authoring cannot run while the timeline is playing')
    source = vehicle_binding_paths(source_vehicle_path)
    destination = vehicle_binding_paths('/World/Vehicles/vehicle_' + new_id) if isinstance(new_id, str) else None
    if destination is None or destination.vehicle == source.vehicle:
        raise ValueError('A distinct stable new vehicle id is required')
    pose = _pose(pose)
    directory = Path(scene_directory).resolve()
    if not directory.is_relative_to(ROOT / 'outputs'):
        raise ValueError('Fleet scene must be inside project outputs')
    asset = directory / 'assets' / 'vehicle.usda'
    if not asset.is_file():
        raise ValueError('Missing immutable shared vehicle asset')
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    source_prim = stage.GetPrimAtPath(source.vehicle)
    if not source_prim or stage.GetPrimAtPath(destination.vehicle):
        raise ValueError('Source must exist and destination must not exist')
    if source_prim.IsInstanceable() or source_prim.IsInstance() or source_prim.IsInstanceProxy():
        raise ValueError('Source cannot be an instance')
    _asset_reference(stage, source_prim, asset)
    for ancestor in (stage.GetPrimAtPath('/World'), stage.GetPrimAtPath('/World/Vehicles'), source_prim):
        matrix = UsdGeom.Xformable(ancestor).GetLocalTransformation()
        if matrix != Gf.Matrix4d(1):
            raise ValueError('World/vehicle wrappers must have identity transforms; pose belongs to chassis')
    filename = directory / ('fleet-' + new_id + '.usda')
    if filename.exists():
        raise FileExistsError(filename)
    before = asset_hashes(directory)
    source_path, target_path = Sdf.Path(source.vehicle), Sdf.Path(destination.vehicle)
    # Prepare in memory: validation failures never partially attach the fleet.
    layer = Sdf.Layer.CreateAnonymous('fleet-preflight.usda')
    extension = Usd.Stage.Open(layer)
    UsdGeom.Xform.Define(extension, '/World')
    extension.SetDefaultPrim(extension.GetPrimAtPath('/World'))
    UsdGeom.SetStageMetersPerUnit(extension, 1.)
    UsdGeom.SetStageUpAxis(extension, 'Z')
    extension.SetTimeCodesPerSecond(stage.GetTimeCodesPerSecond())
    root = extension.DefinePrim(destination.vehicle)
    # Absolute only in the temporary layer; replaced by a portable asset path.
    root.GetReferences().AddReference(str(asset))
    root.SetInstanceable(False)
    Usd.ModelAPI(root).SetKind('component')
    root.SetCustomDataByKey('vehicle_id', destination.vehicle_id)
    root.SetCustomDataByKey('movement_authority', 'isaac_physx')
    root.SetCustomDataByKey('fleet_source_vehicle', source.vehicle)
    for prim in Usd.PrimRange(source_prim):
        for prop in (*prim.GetRelationships(), *prim.GetAttributes()):
            relationship = isinstance(prop, Usd.Relationship)
            targets = prop.GetTargets() if relationship else prop.GetConnections()
            if not targets:
                continue
            mapped = [_remap(p, source_path, target_path, prim.GetPath()) for p in targets]
            owner = extension.OverridePrim(prim.GetPath().ReplacePrefix(source_path, target_path))
            copied = (owner.CreateRelationship(prop.GetName(), custom=prop.IsCustom()) if relationship else
                      owner.CreateAttribute(prop.GetName(), prop.GetTypeName(), custom=prop.IsCustom(),
                                            variability=prop.GetVariability()))
            _property_metadata(prop, copied)
            (copied.SetTargets if relationship else copied.SetConnections)(mapped)
    memberships = {}
    for prim in stage.Traverse():
        if prim.GetTypeName() != 'PhysicsCollisionGroup':
            continue
        for name in ('collection:colliders:includes', 'collection:colliders:excludes'):
            rel = prim.GetRelationship(name)
            targets = list(rel.GetTargets()) if rel else []
            additions = [p.ReplacePrefix(source_path, target_path) for p in targets if p.HasPrefix(source_path)]
            if additions:
                updated = targets + [p for p in additions if p not in targets]
                prop = extension.OverridePrim(prim.GetPath()).CreateRelationship(name, custom=rel.IsCustom())
                _property_metadata(rel, prop)
                prop.SetTargets(updated)
                memberships[str(prop.GetPath())] = updated
    chassis = UsdGeom.Xformable(extension.GetPrimAtPath(destination.chassis))
    if not chassis:
        raise ValueError('Shared asset has no chassis')
    chassis.ClearXformOpOrder()
    chassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble, opSuffix='fleetSpawn').Set(Gf.Vec3d(*pose[:3]))
    chassis.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble, opSuffix='fleetSpawn').Set(Gf.Quatd(pose[6], *pose[3:6]))
    chassis.SetResetXformStack(True)
    session = Sdf.Layer.CreateAnonymous('fleet-validation-session.usda')
    session.subLayerPaths = [layer.identifier, stage.GetSessionLayer().identifier]
    candidate = Usd.Stage.Open(stage.GetRootLayer(), session)
    report = validate_fleet_scene(candidate, (source.vehicle, destination.vehicle))
    checked_physics_values = 0
    for prim in Usd.PrimRange(source_prim):
        copied_prim = candidate.GetPrimAtPath(prim.GetPath().ReplacePrefix(source_path, target_path))
        if not copied_prim or not copied_prim.IsDefined():
            raise ValueError('Source contains content outside the reusable asset: ' + str(prim.GetPath()))
        for attr in prim.GetAttributes():
            # Contact-report lifecycle is owned separately for each car. All
            # other physical values must be those of the shared static asset.
            if (attr.GetName().startswith(('physics:', 'physx'))
                    and not attr.GetName().startswith('physxContactReport:')):
                copied = copied_prim.GetAttribute(attr.GetName())
                if (not copied or copied.Get() != attr.Get() or copied.GetTimeSamples() != attr.GetTimeSamples()
                        or any(copied.Get(t) != attr.Get(t) for t in attr.GetTimeSamples())):
                    raise ValueError('Source instance physics override is not in shared asset: ' + str(attr.GetPath()))
                checked_physics_values += 1
    # All remapped targets, including mixed internal/external lists, must agree.
    for prim in Usd.PrimRange(source_prim):
        for prop in (*prim.GetRelationships(), *prim.GetAttributes()):
            relationship = isinstance(prop, Usd.Relationship)
            targets = prop.GetTargets() if relationship else prop.GetConnections()
            if targets:
                copied = candidate.GetObjectAtPath(prop.GetPath().ReplacePrefix(source_path, target_path))
                actual = copied.GetTargets() if relationship else copied.GetConnections()
                expected = [_remap(p, source_path, target_path, prim.GetPath()) for p in targets]
                if actual != expected:
                    raise ValueError('Binding remap mismatch: ' + str(prop.GetPath()))
    # Change the reference only after it has a file-backed anchor; switching an
    # anonymous live layer to a relative path would emit transient USD errors.
    saved = Sdf.Layer.CreateNew(str(filename))
    saved.TransferContent(layer)
    saved.GetPrimAtPath(destination.vehicle).referenceList.explicitItems = [Sdf.Reference('./assets/vehicle.usda')]
    saved.Save()
    # The live anonymous root may contain stronger Lab collection opinions.
    # Mirror only those collection targets there; immutable source files stay
    # untouched, and the same complete opinions exist in the portable layer.
    stage.GetRootLayer().subLayerPaths = [str(filename)] + list(stage.GetRootLayer().subLayerPaths)
    with Usd.EditContext(stage, stage.GetRootLayer()):
        for prop_path, targets in memberships.items():
            prop = stage.GetRelationshipAtPath(prop_path)
            if prop.GetTargets() != targets:
                stage.OverridePrim(Sdf.Path(prop_path).GetPrimPath()).CreateRelationship(
                    Sdf.Path(prop_path).name, custom=False).SetTargets(targets)
    report = validate_fleet_scene(stage, (source.vehicle, destination.vehicle))
    after = asset_hashes(directory)
    if any(after.get(name) != digest for name, digest in before.items()):
        raise RuntimeError('Existing scene/asset files changed during fleet authoring')
    return dict(schema=1, **asdict(destination), source_vehicle=source.vehicle,
                assembly_layer=str(filename), assembly_sha256=hashlib.sha256(filename.read_bytes()).hexdigest(),
                shared_asset=str(asset), shared_asset_sha256=before[str(asset.relative_to(directory))],
                previous_files_sha256=before, original_files_unchanged=True,
                initial_chassis_pose_xyzw=list(pose), static_validation=report,
                shared_physics_attributes_checked=checked_physics_values,
                native_validated=False, runtime_owner='one physics owner; per-car gates/contact monitors/native views')
