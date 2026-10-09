"""Load a user's prepared native vehicle without regenerating its physics.

The local asset is an input, not a redistributed repository dependency. Only a
fresh per-run package is copied; the original layers are never edited. Imports
needed by Kit are delayed so composition checks can run in the USD-only runtime.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import shutil

from traffic.physx_vehicle import PhysxVehicle
from usd.physical_scene import PATHS


def package_hashes(directory):
    directory = Path(directory).resolve()
    return {p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(directory.rglob('*')) if p.is_file()}


def _inspect_source(directory):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

    directory = Path(directory).resolve()
    for name in ('world.usda', 'layout.usda', 'physics.usda', 'model.json', 'assets/vehicle.usda'):
        if not (directory / name).is_file():
            raise ValueError('Prepared vehicle input is missing ' + name)
    model = json.loads((directory / 'model.json').read_text(encoding='utf-8'))
    expected = {'chassis_path': PATHS.chassis, 'physics_path': PATHS.physics,
                'wheel_paths': PATHS.wheels}
    for key, value in expected.items():
        if model.get(key) != value:
            raise ValueError('Unsupported prepared vehicle path contract: ' + key)
    for name in ('wheelbase_m', 'track_m', 'wheel_radius_m'):
        value = model.get(name)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid prepared vehicle measurement: ' + name)
    for name in ('body_collision_dimensions_m', 'body_collision_center_m'):
        values = model.get(name)
        if (not isinstance(values, list) or len(values) != 3
                or not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
                or (name.endswith('dimensions_m') and any(v <= 0 for v in values))):
            raise ValueError('Invalid prepared vehicle measurement: ' + name)
    source = Usd.Stage.Open(str(directory / 'world.usda'))
    if source is None or source.GetCompositionErrors():
        raise ValueError('Prepared vehicle composition is invalid')
    layers, assets, missing = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(directory / 'world.usda')))
    if missing:
        raise ValueError('Prepared vehicle has unresolved dependencies: ' + repr(list(missing)))
    dependencies = [Path(layer.realPath).resolve() for layer in layers if layer.realPath]
    dependencies += [Path(asset).resolve() for asset in assets]
    if any(not path.is_relative_to(directory) for path in dependencies):
        raise ValueError('Prepared vehicle dependencies must stay inside its package')
    if (str(source.GetDefaultPrim().GetPath()) != '/World'
            or UsdGeom.GetStageUpAxis(source) != 'Z'
            or UsdGeom.GetStageMetersPerUnit(source) != 1):
        raise ValueError('Prepared vehicle must be a metre-scale Z-up /World scene')
    if [str(p.GetPath()) for p in source.Traverse() if p.IsA(UsdPhysics.Scene)] != [PATHS.physics]:
        raise ValueError('Prepared vehicle must contain exactly one canonical physics scene')
    rigid = source.GetPrimAtPath(PATHS.chassis)
    if not rigid or not rigid.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Prepared vehicle is not a physical chassis')
    for path in PATHS.wheels:
        wheel = source.GetPrimAtPath(path)
        if not wheel or any(not wheel.GetAttribute('physxVehicleWheelController:' + name)
                            for name in ('driveTorque', 'brakeTorque', 'steerAngle')):
            raise ValueError('Missing native wheel control interface: ' + path)
    unresolved = [str(target) for p in source.Traverse() for rel in p.GetRelationships()
                  for target in rel.GetTargets() if not source.GetObjectAtPath(target)]
    if unresolved:
        raise ValueError('Prepared vehicle has unresolved relationships: ' + repr(unresolved))
    # Reject symlinks that would copy inputs outside the declared local package.
    if any(not p.resolve().is_relative_to(directory) for p in directory.rglob('*')):
        raise ValueError('Prepared vehicle symlink escapes package')
    return model, package_hashes(directory)


def author_prepared_vehicle_scene(stage, scene_directory, source_directory, physics_hz=120):
    """Copy/compose a metadata-described vehicle; perform no physics stepping.

    The caller owns an empty stage. ``world.usda`` is kept under a provenance
    name because the final highway scene writer will author its own entry point.
    All relative asset references remain untouched and resolve inside the copy.
    """
    from pxr import Sdf, UsdGeom, UsdPhysics

    if isinstance(physics_hz, bool) or not isinstance(physics_hz, int) or physics_hz <= 0:
        raise ValueError('physics_hz must be a positive integer')
    if stage.GetPrimAtPath('/World'):
        raise ValueError('Prepared vehicle requires an empty project stage')
    source = Path(source_directory).resolve()
    target = Path(scene_directory).resolve()
    if target == source or target.is_relative_to(source) or source.is_relative_to(target):
        raise ValueError('Source and per-run vehicle package must be separate')
    if target.exists():
        raise FileExistsError('Refusing to overwrite a prepared scene package: ' + str(target))
    model, original_hashes = _inspect_source(source)
    if 'source-vehicle-world.usda' in original_hashes:
        raise ValueError('Input package uses reserved provenance entry name')
    target.mkdir(parents=True, exist_ok=False)
    for relative in original_hashes:
        copied = target / ('source-vehicle-world.usda' if relative == 'world.usda' else relative)
        copied.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / relative, copied)
    copied_hashes = package_hashes(target)
    for relative, digest in original_hashes.items():
        copied_name = 'source-vehicle-world.usda' if relative == 'world.usda' else relative
        if copied_hashes.get(copied_name) != digest:
            raise RuntimeError('Prepared vehicle copy changed content: ' + relative)
    # A root-layer PhysicsScene spec is needed by the installed manager's
    # scene-addition callback. Define the typed leaf BEFORE a bulk CopySpec or
    # sublayer composition: those send an ancestor resync, not a new typed
    # PhysicsScene leaf notice, and defining an already-copied scene is a no-op.
    # Its actual vehicle configuration still comes from the prepared sublayer.
    UsdGeom.Xform.Define(stage, '/World')
    UsdGeom.Scope.Define(stage, '/World/Physics')
    UsdPhysics.Scene.Define(stage, PATHS.physics)
    copied_root = Sdf.Layer.FindOrOpen(str(target / 'source-vehicle-world.usda'))
    if not Sdf.CopySpec(copied_root, '/World', stage.GetRootLayer(), '/World'):
        raise RuntimeError('Could not preserve prepared world-level opinions')
    stage.GetRootLayer().subLayerPaths.append(str(target / 'source-vehicle-world.usda'))
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    stage.SetTimeCodesPerSecond(float(physics_hz))
    if package_hashes(source) != original_hashes:
        raise RuntimeError('Prepared vehicle input changed during composition')
    return dict(model=model, source_hashes=original_hashes, copied_hashes=copied_hashes,
                source_directory=str(source), scene_directory=str(target),
                source_entry='world.usda', copied_entry='source-vehicle-world.usda',
                runtime_motion_authority='isaac_physx',
                license=model.get('license', 'Unknown; no redistribution permission inferred'),
                redistribution_permitted=False,
                source_asset_modified=False, physics_hz=physics_hz)


class PreparedVehicle(PhysxVehicle):
    """Existing native actuator/state interface bound to a loaded prepared car."""

    def __init__(self, stage, composition, physics_hz=120):
        import omni.physx
        from pxr import PhysxSchema, UsdPhysics

        self.stage = stage
        self.path = PATHS.chassis
        self.physics_scene_path = PATHS.physics
        self.wheel_paths = PATHS.wheels
        self.controllers = [PhysxSchema.PhysxVehicleWheelControllerAPI(stage.GetPrimAtPath(path))
                            for path in self.wheel_paths]
        self.physx = omni.physx.get_physx_interface()
        self.body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(self.path))
        self.scene_composition = composition
        self.metadata = dict(composition['model'])
        self.wheelbase_m = self.metadata['wheelbase_m']
        self.track_m = self.metadata['track_m']
        self.metadata.update(
            model='prepared native PhysX vehicle; scene loaded unchanged',
            factory_sha256=self.metadata.get('build_factory_sha256'),
            chassis_collision_dimensions_m=self.metadata['body_collision_dimensions_m'],
            chassis_collision_center_m=self.metadata['body_collision_center_m'],
            movement_authority='isaac_physx', pose_reference='chassis origin; not SUMO front bumper',
            quaternion_order='xyzw', world_frame='metres, Z up, +X forward, +Y left',
            scene_composition=composition, sumo_required=False, lidar_enabled=False,
            isaac_lab_validated=False,
            validation_scope='Asset loaded; new highway/showcase driving requires its own evidence')
        PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(PATHS.physics)).GetTimeStepsPerSecondAttr().Set(physics_hz)


def compose_prepared_vehicle(stage, scene_directory, source_directory, physics_hz=120):
    """Compose the required local package and return the actuator/state adapter."""
    composition = author_prepared_vehicle_scene(stage, scene_directory, source_directory, physics_hz)
    return PreparedVehicle(stage, composition, physics_hz)
