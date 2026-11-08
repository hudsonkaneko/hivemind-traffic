"""One precreated collidable hazard for the versioned loop-segment Lab task.

Author before play, bind after native initialization, target each next physics
tick, then read measured state. PhysX owns ego motion. Hazard authority is an
explicit scripted kinematic trajectory; this is not reactive vehicle dynamics.
All runtime USD writes are nonphysical render-proxy opinions in an anonymous
layer. Published base/asset files remain immutable. No simulator starts here.
"""
from __future__ import annotations

from dataclasses import asdict, is_dataclass
import hashlib
import math
from numbers import Real
from pathlib import Path

import numpy as np

from usd.physical_scene import PATHS, _metadata, _new_stage, asset_hashes

ROOT = Path(__file__).resolve().parents[1]
HAZARD_ID = 'hazard_000'
HAZARD_ROOT = '/World/Environment/Props/Hazard_000'
BODY_PATH = HAZARD_ROOT + '/Body'
COLLIDER_PATH = BODY_PATH + '/Collider'
PROXY_PATH = HAZARD_ROOT + '/RenderProxy'
GROUP_PATH = '/World/Physics/Resources/LoopHazardCollisionGroup'
QUERY_GROUP = '/World/Physics/Resources/VehicleGroundQueryGroup'
PARKING = (10000., 10000., .5)
PHYSICS_HZ = 120
MAX_SPEED_M_S = 10.


def _tick(tick):
    if type(tick) is not int or tick < 0:
        raise ValueError('Tick must be a nonnegative integer')
    return tick


def _array(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.asarray(value)


def _poses(objects):
    if not isinstance(objects, (list, tuple)) or len(objects) > 1:
        raise ValueError('Hazard scene v1 permits zero or one fixed cube')
    if not objects:
        return np.asarray([[*PARKING, 0., 0., 0., 1.]], dtype=np.float32), False
    row = asdict(objects[0]) if is_dataclass(objects[0]) else objects[0]
    if not isinstance(row, dict) or row.get('object_id') != HAZARD_ID:
        raise ValueError('Unexpected hazard identity')
    vals = [row.get(k) for k in ('x_m', 'y_m', 'yaw_rad')]
    if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in vals):
        raise ValueError('Finite hazard pose required')
    if abs(vals[0]) > 2000 or abs(vals[1]) > 2000:
        raise ValueError('Active hazard outside bounded scene domain')
    for key in ('length_m', 'width_m', 'height_m'):
        if key in row and (isinstance(row[key], bool) or row[key] != 1.):
            raise ValueError('Hazard dimensions are fixed to one metre')
    x, y, yaw = vals
    return np.asarray([[x, y, .5, 0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]], dtype=np.float32), True


def _inventory(stage):
    from pxr import UsdPhysics
    return dict(rigid_bodies=sorted(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)),
                colliders=sorted(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)))


def author_hazards(stage, directory, initial_poses=()):
    """Extend an already exported ordinary lane scene before physics begins."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from environments.lane_following_scene import validate_lane_scene
    directory = Path(directory).resolve()
    if not directory.is_relative_to(ROOT / 'outputs') or not (directory / 'world.usda').is_file():
        raise ValueError('Expected an exported base scene under project outputs')
    validate_lane_scene(stage)
    if stage.GetPrimAtPath(HAZARD_ROOT) or stage.GetPrimAtPath(PATHS.barrier):
        raise ValueError('Hazard scene requires an ordinary base without existing props')
    if not stage.GetPrimAtPath(QUERY_GROUP):
        raise ValueError('Vehicle suspension query filter is required')
    initial, active = _poses(initial_poses)
    baseline = _inventory(stage)
    base_hashes = asset_hashes(directory)
    # The visual component deliberately contains no physics schemas.
    visual = _new_stage(directory / 'assets/loop-hazard-geometry.usda')
    cube = UsdGeom.Cube.Define(visual, '/Hazard')
    cube.CreateSizeAttr(1.)
    cube.CreateDisplayColorAttr([Gf.Vec3f(.95, .25, .04)])
    _metadata(visual, '/Hazard', 'component')
    visual.GetRootLayer().Save()
    physics = _new_stage(directory / 'assets/loop-hazard-physics.usda')
    UsdGeom.Xform.Define(physics, '/Hazard')
    body = UsdGeom.Xform.Define(physics, '/Hazard/Body').GetPrim()
    rigid = UsdPhysics.RigidBodyAPI.Apply(body)
    rigid.CreateRigidBodyEnabledAttr(True)
    rigid.CreateKinematicEnabledAttr(True)
    collision = UsdGeom.Cube.Define(physics, '/Hazard/Body/Collider')
    collision.CreateSizeAttr(1.)
    UsdPhysics.CollisionAPI.Apply(collision.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdGeom.Imageable(body).CreateVisibilityAttr('invisible')
    _metadata(physics, '/Hazard', 'component')
    physics.GetRootLayer().Save()
    layout = _new_stage(directory / 'hazard-layout.usda')
    for path, kind in (('/World', 'assembly'), ('/World/Environment', 'group'),
                       ('/World/Environment/Props', 'group')):
        Usd.ModelAPI(UsdGeom.Xform.Define(layout, path).GetPrim()).SetKind(kind)
    root = layout.DefinePrim(HAZARD_ROOT)
    root.GetReferences().AddReference('./assets/loop-hazard-physics.usda')
    root.SetCustomDataByKey('object_id', HAZARD_ID)
    root.SetCustomDataByKey('movement_authority', 'scripted_native_kinematic_targets')
    body = UsdGeom.Xformable(layout.GetPrimAtPath(BODY_PATH))
    pose = initial[0]
    body.AddTranslateOp().Set(Gf.Vec3d(*map(float, pose[:3])))
    body.AddOrientOp().Set(Gf.Quatf(float(pose[6]), *map(float, pose[3:6])))
    UsdPhysics.RigidBodyAPI(body.GetPrim()).CreateSimulationOwnerRel().SetTargets([PATHS.physics])
    proxy = layout.DefinePrim(PROXY_PATH)
    proxy.GetReferences().AddReference('./assets/loop-hazard-geometry.usda')
    Usd.ModelAPI(proxy).SetKind('subcomponent')
    proxy.SetCustomDataByKey('pose_source', 'measured_native_body; render_only')
    proxy.SetCustomDataByKey('native_body_path', BODY_PATH)
    proxy_xform = UsdGeom.Xformable(proxy)
    proxy_xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, pose[:3])))
    proxy_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(pose[6]), *map(float, pose[3:6])))
    proxy_xform.SetResetXformStack(True)
    UsdGeom.Imageable(proxy).CreateVisibilityAttr('inherited' if active else 'invisible')
    group = UsdPhysics.CollisionGroup.Define(layout, GROUP_PATH)
    group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([COLLIDER_PATH])
    query = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(QUERY_GROUP))
    filters = list(query.GetFilteredGroupsRel().GetTargets())
    filters.append(Sdf.Path(GROUP_PATH))
    layout.OverridePrim(QUERY_GROUP).CreateRelationship('physics:filteredGroups', custom=False).SetTargets(filters)
    _metadata(layout, '/World', 'assembly')
    layout.GetRootLayer().customLayerData = {'loop_hazard_schema': 1, 'fixed_cube_size_m': 1.}
    layout.GetRootLayer().Save()
    runtime = Sdf.Layer.CreateAnonymous('loop-hazard-runtime.usda')
    stage.GetRootLayer().subLayerPaths = [runtime.identifier, str(directory / 'hazard-layout.usda')] + list(stage.GetRootLayer().subLayerPaths)
    if Sdf.Path(GROUP_PATH) not in query.GetFilteredGroupsRel().GetTargets():
        # Initial assembly binding only, matching the existing fleet adapter.
        # Lab may author this resource in its stronger anonymous root layer.
        # The same value is already in hazard-layout for the portable export.
        with Usd.EditContext(stage, stage.GetRootLayer()):
            query.GetFilteredGroupsRel().SetTargets(filters)
    pool = HazardPool(stage, directory, runtime, baseline, base_hashes, initial, active)
    pool.validate()
    return pool


class HazardPool:
    def __init__(self, stage, directory, runtime, baseline, base_hashes, initial, active):
        self.stage, self.directory, self.runtime = stage, Path(directory), runtime
        self.baseline, self.base_hashes = baseline, base_hashes
        self._target, self.active = initial.copy(), active
        self._view = None
        self._episode_id = None
        self._target_tick = 0
        self._read_tick = 0
        self.metadata = dict(schema_version=1, count=1, object_id=HAZARD_ID,
            dimensions_m=[1., 1., 1.], body_path=BODY_PATH, render_path=PROXY_PATH,
            collider_path=COLLIDER_PATH, physics_hz=PHYSICS_HZ,
            motion_authority='scripted_native_kinematic_targets',
            observation_source='measured_native_pose_velocity_privileged_simulator_state',
            reset_policy='set_transforms only at explicit new-episode boundary; never set_velocities',
            render_policy='nonphysical proxy follows measured native pose in anonymous layer',
            limitation='nonreactive kinematic obstacle; contact is failure, not calibrated impact dynamics')

    def bind(self, simulation_view):
        if self._view is not None:
            raise RuntimeError('Hazard native view already bound')
        view = simulation_view.create_rigid_body_view(BODY_PATH)
        if list(view.prim_paths) != [BODY_PATH]:
            raise RuntimeError('Native hazard identity mismatch')
        sample = view.get_transforms()
        if hasattr(sample, 'ptr') and hasattr(sample, 'device'):
            import warp as wp
            self._float = lambda v: wp.array(v, dtype=wp.float32, device=sample.device)
            self._indices = lambda v: wp.array(v, dtype=wp.uint32, device=sample.device)
        elif hasattr(sample, 'detach'):
            import torch
            self._float = lambda v: torch.as_tensor(v, dtype=torch.float32, device=sample.device)
            self._indices = lambda v: torch.as_tensor(v, dtype=torch.int32, device=sample.device)
        else:
            self._float = self._indices = lambda v: v
        self._view = view
        return self.pose_check()

    def _native(self):
        if self._view is None:
            raise RuntimeError('Bind hazard native physics first')
        transforms = _array(self._view.get_transforms()).copy()
        velocities = _array(self._view.get_velocities()).copy()
        if (transforms.shape != (1, 7) or velocities.shape != (1, 6)
                or not np.isfinite(transforms).all() or not np.isfinite(velocities).all()
                or abs(np.linalg.norm(transforms[0, 3:]) - 1.) > .001):
            raise RuntimeError('Malformed/nonfinite native hazard state')
        return transforms, velocities

    def reset_poses(self, objects, *, episode_id):
        if not isinstance(episode_id, str) or not episode_id or episode_id == self._episode_id:
            raise ValueError('Explicit new episode identity required for hazard reset')
        if self._view is None:
            raise RuntimeError('Bind hazard native physics before reset')
        targets, active = _poses(objects)
        indices = self._indices(np.asarray([0], dtype=np.uint32))
        self._view.set_transforms(self._float(targets), indices)
        # Clear the previous episode's pending target. PhysX derives velocity
        # from subsequent targets; never invent a zero velocity readback.
        self._view.set_kinematic_targets(self._float(targets), indices)
        self._target, self.active = targets, active
        self._target_tick = self._read_tick = 0
        self._episode_id = episode_id
        return self.pose_check()

    def set_targets(self, objects, *, tick):
        _tick(tick)
        if self._view is None or self._episode_id is None:
            raise RuntimeError('Bind and explicitly reset the hazard before stepping')
        if tick != self._target_tick + 1:
            raise ValueError('Exactly one hazard target per consecutive physics tick required')
        targets, active = _poses(objects)
        if active != self.active:
            raise ValueError('Hazard membership may change only at episode reset')
        if np.linalg.norm(targets[0, :3] - self._target[0, :3]) > MAX_SPEED_M_S / PHYSICS_HZ + .002:
            raise ValueError('Hazard target jumps beyond the declared speed bound')
        # Quaternion sign changes are equivalent rotations. Bound actual angle.
        dot = min(1., abs(float(np.dot(targets[0, 3:], self._target[0, 3:]))))
        if 2 * math.acos(dot) > math.pi / PHYSICS_HZ + .002:
            raise ValueError('Hazard yaw target jumps beyond the angular-speed bound')
        self._view.set_kinematic_targets(self._float(targets), self._indices(np.asarray([0], dtype=np.uint32)))
        self._target, self._target_tick = targets, tick

    def pose_check(self):
        transforms, velocities = self._native()
        error = float(np.linalg.norm(transforms[0, :3] - self._target[0, :3]))
        dot = min(1., abs(float(np.dot(transforms[0, 3:], self._target[0, 3:]))))
        angle = 2 * math.acos(dot)
        if error > .005 or angle > .005:
            raise RuntimeError('Native hazard pose does not match completed target')
        return dict(passed=True, position_error_m=error, angular_error_rad=angle,
                    measured_velocity_m_s=velocities[0, :3].tolist(), target_tick=self._target_tick)

    def measured_frames(self, tick):
        _tick(tick)
        if tick != self._target_tick or tick < self._read_tick:
            raise ValueError('Readback must name the completed target tick')
        self.pose_check()
        poses, velocities = self._native()
        self._read_tick = tick
        if not self.active:
            return []
        x, y, z, qx, qy, qz, qw = map(float, poses[0])
        return [dict(object_id=HAZARD_ID, x_m=x, y_m=y,
            yaw_rad=math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)),
            vx_m_s=float(velocities[0, 0]), vy_m_s=float(velocities[0, 1]),
            length_m=1., width_m=1., timestamp_s=tick/PHYSICS_HZ)]

    def measured_objects(self, tick):
        from traffic.road_hazards import MeasuredObject
        return tuple(MeasuredObject(**row) for row in self.measured_frames(tick))

    def publish_visuals(self):
        from pxr import Gf, Usd, UsdGeom
        self.pose_check()
        pose = self._native()[0][0]
        with Usd.EditContext(self.stage, self.runtime):
            proxy = self.stage.GetPrimAtPath(PROXY_PATH)
            proxy.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*map(float, pose[:3])))
            proxy.GetAttribute('xformOp:orient').Set(Gf.Quatd(float(pose[6]), *map(float, pose[3:6])).GetNormalized())
            UsdGeom.Imageable(proxy).GetVisibilityAttr().Set('inherited' if self.active else 'invisible')
        return {'passed': True, 'pose_source': 'measured_native', 'active': self.active}

    def validate(self):
        return validate_hazard_scene(self.stage, self.baseline, directory=self.directory, base_hashes=self.base_hashes)

    def save_entry(self):
        path = self.directory / 'hazard-world.usda'
        snapshot = self.directory / 'hazard-runtime-initial.usda'
        if path.exists() or snapshot.exists():
            raise FileExistsError('Hazard entry already saved')
        if not self.runtime.Export(str(snapshot)):
            raise RuntimeError('Cannot export hazard runtime initial layer')
        entry = _new_stage(path)
        entry.GetRootLayer().subLayerPaths = ['./hazard-runtime-initial.usda', './hazard-layout.usda', './world.usda']
        _metadata(entry, '/World', 'assembly')
        entry.GetRootLayer().customLayerData = {'loop_hazard_schema': 1,
            'scope': 'initial composed scene; not a replay',
            'base_rigid_bodies': self.baseline['rigid_bodies'], 'base_colliders': self.baseline['colliders']}
        entry.GetRootLayer().Save()
        validate_hazard_scene(entry, self.baseline, directory=self.directory, base_hashes=self.base_hashes)
        return dict(entry_path=str(path), static_hashes=asset_hashes(self.directory), base_hashes=dict(self.base_hashes))

    def release(self):
        self._view = self.stage = self.runtime = None
        self._float = self._indices = None


def validate_hazard_scene(stage, baseline, *, directory=None, base_hashes=None):
    """Strict added-body/collider inventory without changing the legacy validator."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    errors = [str(e) for e in stage.GetCompositionErrors()]
    actual = _inventory(stage)
    expected = dict(rigid_bodies=sorted([*baseline['rigid_bodies'], BODY_PATH]),
                    colliders=sorted([*baseline['colliders'], COLLIDER_PATH]))
    if baseline['rigid_bodies'] != [PATHS.chassis] or actual != expected:
        errors.append('Rigid body or collider inventory changed')
    for path in (PATHS.chassis, PATHS.physics, PATHS.ground, *PATHS.wheels,
                 PATHS.follow_camera, PATHS.overview_camera, '/World/Environment/Highway',
                 HAZARD_ROOT, BODY_PATH, COLLIDER_PATH, PROXY_PATH):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsActive() or not prim.IsLoaded():
            errors.append('Missing active loaded prim: ' + path)
    if (not stage.GetDefaultPrim() or str(stage.GetDefaultPrim().GetPath()) != '/World' or UsdGeom.GetStageUpAxis(stage) != 'Z'
            or UsdGeom.GetStageMetersPerUnit(stage) != 1. or stage.GetTimeCodesPerSecond() != 120.):
        errors.append('Scene coordinates/defaultPrim/rate changed')
    world = stage.GetPrimAtPath('/World')
    if world and {p.GetName() for p in world.GetChildren()} - {
            'Environment', 'Vehicles', 'Physics', 'Lighting', 'Cameras', 'Debug', 'envs'}:
        errors.append('Unexpected project root branch')
    bookkeeping = stage.GetPrimAtPath('/World/envs')
    if bookkeeping and (len(bookkeeping.GetChildren()) != 1 or
            not stage.GetPrimAtPath('/World/envs/env_0') or stage.GetPrimAtPath('/World/envs/env_0').GetChildren()):
        errors.append('Lab bookkeeping must remain one empty environment')
    scenes = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'PhysicsScene']
    if scenes != [PATHS.physics]:
        errors.append('Expected sole native physics scene')
    for path, kind in (('/World', 'assembly'), ('/World/Environment', 'group'),
                       ('/World/Environment/Props', 'group'), (HAZARD_ROOT, 'component')):
        prim = stage.GetPrimAtPath(path)
        if not prim or Usd.ModelAPI(prim).GetKind() != kind or not prim.IsModel():
            errors.append('Invalid model ancestry: ' + path)
    root = stage.GetPrimAtPath(HAZARD_ROOT)
    if (not root or not root.HasAuthoredReferences() or root.IsInstance()
            or root.GetCustomDataByKey('object_id') != HAZARD_ID
            or root.GetCustomDataByKey('movement_authority') != 'scripted_native_kinematic_targets'):
        errors.append('Hazard identity/reference/authority changed')
    body = stage.GetPrimAtPath(BODY_PATH)
    if body:
        if UsdGeom.Imageable(body).GetVisibilityAttr().Get() != 'invisible':
            errors.append('Native collider rendering must remain hidden')
        operations = UsdGeom.Xformable(body).GetOrderedXformOps()
        if [op.GetOpType() for op in operations] != [UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeOrient]:
            errors.append('Hazard body transform stack changed')
        rigid = UsdPhysics.RigidBodyAPI(body)
        if (not rigid.GetRigidBodyEnabledAttr().Get() or not rigid.GetKinematicEnabledAttr().Get()
                or list(map(str, rigid.GetSimulationOwnerRel().GetTargets())) != [PATHS.physics]):
            errors.append('Hazard must be enabled kinematic body owned by declared scene')
    collider = stage.GetPrimAtPath(COLLIDER_PATH)
    if not collider or not UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get() or UsdGeom.Cube(collider).GetSizeAttr().Get() != 1.:
        errors.append('Fixed enabled one-metre hazard collider required')
    proxy = stage.GetPrimAtPath(PROXY_PATH)
    if not proxy or not proxy.HasAuthoredReferences() or proxy.IsInstance():
        errors.append('Editable referenced render proxy required')
    elif any(p.GetAppliedSchemas() for p in Usd.PrimRange(proxy)):
        errors.append('Render proxy must have no applied physics schemas')
    group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(GROUP_PATH))
    query = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(QUERY_GROUP))
    if (not group or list(map(str, group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets())) != [COLLIDER_PATH]
            or group.GetFilteredGroupsRel().GetTargets() or not query
            or Sdf.Path(GROUP_PATH) not in query.GetFilteredGroupsRel().GetTargets()):
        errors.append('Hazard collision/suspension filtering changed')
    ego = stage.GetPrimAtPath(PATHS.chassis)
    if ego and UsdPhysics.RigidBodyAPI(ego).GetKinematicEnabledAttr().Get():
        errors.append('Ego must remain dynamic')
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if target.IsAbsolutePath() and not stage.GetObjectAtPath(target):
                    errors.append('Unresolved relationship: ' + str(rel.GetPath()))
        for attr in prim.GetAttributes():
            if any(not stage.GetObjectAtPath(target) for target in attr.GetConnections()):
                errors.append('Unresolved connection: ' + str(attr.GetPath()))
    if directory is not None and base_hashes is not None:
        for name, digest in base_hashes.items():
            path = Path(directory) / name
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                errors.append('Published base scene file changed: ' + name)
    if errors:
        raise ValueError('Invalid hazard scene: ' + '; '.join(errors))
    return dict(passed=True, errors=[], **actual, physics_scenes=scenes,
                object_id=HAZARD_ID, physics_authority='PhysX ego; scripted kinematic obstacle',
                layers=[layer.identifier for layer in stage.GetUsedLayers()])
