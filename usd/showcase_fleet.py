"""Referenced block cars with explicit, deterministic kinematic motion authority.

The slow cars are moving collision objects, not physical-driver models. Native
PhysX kinematic targets move their rigid bodies; no target is ever applied to the
dynamic ego. The controller receives measured native body transforms, not the
requested targets. The per-run reusable asset and layout stay immutable.
"""
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import re

from usd.physical_scene import PATHS, _metadata, _new_stage


BASE_RADIUS_M = 500.0
LANE_WIDTH_M = 3.7
BODY_Z_M = 0.75
LENGTH_M = 4.8
WIDTH_M = 1.9  # Includes the wheel envelope, not just the painted body.
WHEEL_RADIUS_M = 0.35
BACKGROUND_GROUP = '/World/Physics/Resources/ShowcaseBackgroundCollisionGroup'
QUERY_GROUP = '/World/Physics/Resources/VehicleGroundQueryGroup'
WHEEL_NAMES = ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel')
BODY_PALETTE = ((.18, .42, .63), (.46, .55, .62), (.72, .46, .18), (.30, .50, .39))
BODY_STYLE_NAMES = ('Blue', 'Silver', 'Gold', 'Green')


@dataclass(frozen=True)
class FleetSpec:
    vehicle_id: str
    lane: int
    station_m: float
    speed_m_s: float

    def __post_init__(self):
        if not isinstance(self.vehicle_id, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', self.vehicle_id):
            raise ValueError('Background vehicle IDs must be stable legal USD identifiers')
        if isinstance(self.lane, bool) or not isinstance(self.lane, int) or not 0 <= self.lane <= 3:
            raise ValueError('Background lane must be an integer from 0 to 3')
        if (not all(math.isfinite(v) for v in (self.station_m, self.speed_m_s))
                or self.speed_m_s < 0 or self.speed_m_s > 20):
            raise ValueError('Background station/speed must be finite, with speed in [0, 20] m/s')

    @property
    def chassis_path(self):
        key=self.vehicle_id if self.vehicle_id.startswith('background_') else 'background_'+self.vehicle_id
        return '/World/Vehicles/vehicle_' + key + '/Chassis'

    @property
    def vehicle_path(self):
        return self.chassis_path.rsplit('/', 1)[0]

    @property
    def render_path(self):
        return self.vehicle_path + '/RenderProxy'


def fleet_pose(spec, sim_time_s):
    """World chassis-center pose; initial station is measured on radius 500 m.

    Each car's configured speed is its actual lane arc speed, so outer lanes
    have lower angular speed for the same speed in metres/second.
    """
    if not math.isfinite(sim_time_s) or sim_time_s < 0:
        raise ValueError('Simulation time must be finite and nonnegative')
    radius = BASE_RADIUS_M + spec.lane * LANE_WIDTH_M
    angle = spec.station_m / BASE_RADIUS_M + spec.speed_m_s * sim_time_s / radius
    yaw = math.atan2(math.sin(angle + math.pi / 2), math.cos(angle + math.pi / 2))
    return dict(position_m=(radius * math.cos(angle), radius * math.sin(angle), BODY_Z_M),
                yaw_rad=yaw, quaternion_xyzw=(0., 0., math.sin(yaw / 2), math.cos(yaw / 2)),
                velocity_m_s=(-spec.speed_m_s * math.sin(angle), spec.speed_m_s * math.cos(angle), 0.),
                wheel_spin_deg=math.degrees(spec.speed_m_s * sim_time_s / WHEEL_RADIUS_M) % 360.)


def _box(stage, path, dimensions, position, color):
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3f(*dimensions))
    cube.CreateDisplayColorAttr([color])
    return cube


def stopping_pose(spec, sim_time_s, stop_started_s=None):
    """Same circle, with a 3 m/s2 base-lane deceleration during demo rundown."""
    if stop_started_s is None or sim_time_s<=stop_started_s:
        return fleet_pose(spec,sim_time_s)
    deceleration=3.*(BASE_RADIUS_M+spec.lane*LANE_WIDTH_M)/BASE_RADIUS_M
    elapsed=min(sim_time_s-stop_started_s,spec.speed_m_s/deceleration)
    equivalent=stop_started_s+elapsed-(.5*deceleration*elapsed**2/spec.speed_m_s if spec.speed_m_s else 0.)
    pose=fleet_pose(spec,equivalent)
    speed=max(0.,spec.speed_m_s-deceleration*(sim_time_s-stop_started_s))
    yaw=pose['yaw_rad']
    pose['velocity_m_s']=(speed*math.cos(yaw),speed*math.sin(yaw),0.)
    return pose


def _create_asset(assets, separate_visuals=False):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    geometry = _new_stage(assets / 'showcase-block-car-geometry.usda')
    UsdGeom.Xform.Define(geometry, '/Vehicle')
    UsdGeom.Xform.Define(geometry, '/Vehicle/Chassis')
    UsdGeom.Scope.Define(geometry, '/Vehicle/Chassis/Visuals')
    _box(geometry, '/Vehicle/Chassis/Visuals/Body', (4.8, 1.8, 1.05), (0., 0., .075), (.19, .38, .54))
    _box(geometry, '/Vehicle/Chassis/Visuals/Cabin', (2.4, 1.55, .45), (-.2, 0., .825), (.10, .16, .21))
    _box(geometry, '/Vehicle/Chassis/Visuals/FrontLamp', (.06, 1.35, .12), (2.37, 0., .18), (1., .92, .62))
    _box(geometry, '/Vehicle/Chassis/Visuals/RearLamp', (.06, 1.35, .12), (-2.37, 0., .18), (.85, .08, .06))
    for name, x, y in zip(WHEEL_NAMES, (1.6, 1.6, -1.6, -1.6), (.80, -.80, .80, -.80)):
        path = '/Vehicle/Chassis/' + name
        wheel = UsdGeom.Xform.Define(geometry, path)
        wheel.AddTranslateOp().Set(Gf.Vec3d(x, y, -.4))
        wheel.AddRotateYOp(opSuffix='spin').Set(0.)
        tire = UsdGeom.Cylinder.Define(geometry, path + '/Tire')
        tire.CreateAxisAttr('Y')
        tire.CreateRadiusAttr(WHEEL_RADIUS_M)
        tire.CreateHeightAttr(.22)
        tire.CreateDisplayColorAttr([(.045, .05, .055)])
        # The silver cross makes wheel rotation legible without a texture.
        _box(geometry, path + '/SpokeVertical', (.08, .235, .50), (0, 0, 0), (.5, .54, .58))
        _box(geometry, path + '/SpokeHorizontal', (.50, .235, .08), (0, 0, 0), (.5, .54, .58))
    _metadata(geometry, '/Vehicle', 'component')
    geometry.GetRootLayer().Save()

    physics = _new_stage(assets / 'showcase-block-car-physics.usda')
    UsdGeom.Xform.Define(physics, '/Vehicle')
    chassis = UsdGeom.Xform.Define(physics, '/Vehicle/Chassis').GetPrim()
    body = UsdPhysics.RigidBodyAPI.Apply(chassis)
    body.CreateRigidBodyEnabledAttr(True)
    body.CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(chassis).CreateMassAttr(1800.)
    UsdGeom.Scope.Define(physics, '/Vehicle/Chassis/Colliders')
    collider = _box(physics, '/Vehicle/Chassis/Colliders/Body', (LENGTH_M, WIDTH_M, 1.6),
                    (0, 0, .25), (.3, .3, .3))
    collider.CreateVisibilityAttr('invisible')
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
    _metadata(physics, '/Vehicle', 'component')
    physics.GetRootLayer().Save()

    asset = _new_stage(assets / 'showcase-block-car.usda')
    asset.GetRootLayer().subLayerPaths = ['./showcase-block-car-physics.usda']
    if not separate_visuals:
        asset.GetRootLayer().subLayerPaths.append('./showcase-block-car-geometry.usda')
    _metadata(asset, '/Vehicle', 'component')
    root = asset.GetDefaultPrim()
    Usd.ModelAPI(root).SetAssetName('ShowcaseBlockCar')
    Usd.ModelAPI(root).SetAssetVersion('v02-render-proxy')
    root.SetCustomDataByKey('motion_authority', 'scripted_kinematic_targets')
    root.SetCustomDataByKey('position_reference', 'chassis center; metres; +X forward; Z up')
    asset.GetRootLayer().Save()


def _create_instanced_visual_assets(assets):
    """Share immutable geometry below separately editable poses and wheel spins."""
    from pxr import Gf, Usd, UsdGeom
    library = _new_stage(assets / 'showcase-block-car-visual-library.usda')
    UsdGeom.Xform.Define(library, '/VisualLibrary')
    for name, color in zip(BODY_STYLE_NAMES, BODY_PALETTE):
        path = '/VisualLibrary/Body' + name
        body = library.DefinePrim(path)
        body.GetReferences().AddReference('./showcase-block-car-geometry.usda', '/Vehicle/Chassis/Visuals')
        Usd.ModelAPI(body).SetKind('subcomponent')
        library.GetPrimAtPath(path + '/Body').GetAttribute('primvars:displayColor').Set([color])
    wheel = library.DefinePrim('/VisualLibrary/Wheel')
    wheel.GetReferences().AddReference('./showcase-block-car-geometry.usda', '/Vehicle/Chassis/FrontLeftWheel')
    # The reusable source includes a placement/spin pivot. Those operations
    # belong to each car's editable parent, not the shared geometry subtree.
    UsdGeom.Xformable(wheel).SetXformOpOrder([])
    Usd.ModelAPI(wheel).SetKind('subcomponent')
    _metadata(library, '/VisualLibrary', 'component')
    library.GetRootLayer().Save()

    geometry = _new_stage(assets / 'showcase-block-car-instanced-geometry.usda')
    UsdGeom.Xform.Define(geometry, '/Vehicle')
    UsdGeom.Xform.Define(geometry, '/Vehicle/Chassis')
    body = geometry.DefinePrim('/Vehicle/Chassis/Visuals')
    body.GetReferences().AddReference('./showcase-block-car-visual-library.usda', '/VisualLibrary/BodyBlue')
    body.SetInstanceable(True)
    for name, x, y in zip(WHEEL_NAMES, (1.6, 1.6, -1.6, -1.6), (.80, -.80, .80, -.80)):
        path = '/Vehicle/Chassis/' + name
        pivot = UsdGeom.Xform.Define(geometry, path)
        pivot.AddTranslateOp().Set(Gf.Vec3d(x, y, -.4))
        pivot.AddRotateYOp(opSuffix='spin').Set(0.)
        wheel = geometry.DefinePrim(path + '/Geometry')
        wheel.GetReferences().AddReference('./showcase-block-car-visual-library.usda', '/VisualLibrary/Wheel')
        wheel.SetInstanceable(True)
    _metadata(geometry, '/Vehicle', 'component')
    geometry.GetRootLayer().Save()


def _array(value):
    """Native tensor frontend may be numpy, Warp or Torch in this runtime."""
    if hasattr(value, 'detach'):
        value = value.detach().cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    import numpy as np
    return np.asarray(value)


class KinematicFleet:
    """Author before play, bind after play, command next tick, read after step.

    ``tracks(tick)`` returns measured native body state at the caller's current
    tick. Calling it before physics binding intentionally fails. ``pose_check``
    must follow each physics step to detect stale/misbound targets. No hidden
    stepping, timeline changes, installed-runtime edits or background threads.

    Readback caching is opt-in for a single-owned-scene lifetime: the caller
    must call update(), then step physics, before any measured-state consumer.
    No other owner may step or mutate native bodies between those consumers.
    """
    def __init__(self, stage, scene_directory, specs, *, episode_id='showcase', wheel_update_every=1,
                 batch_visual_updates=False, separate_visuals=False, cache_readback=False,
                 instance_visuals=False):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        if (isinstance(wheel_update_every, bool) or not isinstance(wheel_update_every, int)
                or wheel_update_every <= 0 or wheel_update_every > 120 or 120 % wheel_update_every):
            raise ValueError('Wheel visual update interval must be a positive integer divisor of 120')
        self.stage = stage
        if type(cache_readback) is not bool:
            raise ValueError('Readback caching must be boolean')
        self.cache_readback = cache_readback
        self.readback_count = 0
        self.track_construction_count = 0
        self._measured_cache = None
        self._track_cache = None
        self.directory = Path(scene_directory).resolve()
        self.episode_id = episode_id
        self.wheel_update_every = wheel_update_every
        if type(batch_visual_updates) is not bool:
            raise ValueError('Visual batching must be boolean')
        self.batch_visual_updates = batch_visual_updates
        if type(separate_visuals) is not bool:
            raise ValueError('Visual separation must be boolean')
        self.separate_visuals = separate_visuals
        if type(instance_visuals) is not bool:
            raise ValueError('Visual instancing must be boolean')
        self.instance_visuals = instance_visuals
        self.specs = []
        for spec in specs:
            if isinstance(spec, dict):
                spec = dict(spec)
                if 'id' in spec:
                    spec['vehicle_id'] = spec.pop('id')
                spec = FleetSpec(**spec)
            if not isinstance(spec, FleetSpec):
                raise ValueError('Fleet entries must be FleetSpec values or matching dictionaries')
            self.specs.append(spec)
        if (not self.specs or len({s.vehicle_id for s in self.specs}) != len(self.specs)
                or len({s.chassis_path for s in self.specs}) != len(self.specs)):
            raise ValueError('Fleet must contain at least one uniquely identified car')
        if not isinstance(episode_id, str) or not episode_id:
            raise ValueError('A nonempty episode identity is required')
        if not stage.GetPrimAtPath(PATHS.physics):
            raise ValueError('Create the single shared physics scene before the fleet')
        if any(stage.GetPrimAtPath(s.vehicle_path) for s in self.specs):
            raise ValueError('Fleet would overwrite an existing vehicle identity')
        assets = self.directory / 'assets'
        assets.mkdir(parents=True, exist_ok=True)
        files = [assets / ('showcase-block-car' + suffix + '.usda')
                 for suffix in ('', '-physics', '-geometry')]
        if instance_visuals:
            files.extend(assets / ('showcase-block-car' + suffix + '.usda')
                         for suffix in ('-visual-library', '-instanced-geometry'))
        files.append(self.directory / 'showcase-fleet-layout.usda')
        if any(p.exists() for p in files):
            raise FileExistsError('Refusing to overwrite a fleet asset or layout')
        _create_asset(assets,separate_visuals)
        if instance_visuals:
            _create_instanced_visual_assets(assets)
        layout = _new_stage(files[-1])
        UsdGeom.Xform.Define(layout, '/World')
        _metadata(layout, '/World', 'assembly')
        Usd.ModelAPI(UsdGeom.Xform.Define(layout, '/World/Vehicles').GetPrim()).SetKind('group')
        for index, spec in enumerate(self.specs):
            placement = layout.DefinePrim(spec.vehicle_path)
            placement.GetReferences().AddReference('./assets/showcase-block-car.usda')
            placement.SetCustomDataByKey('vehicle_id', spec.vehicle_id)
            placement.SetCustomDataByKey('driver_mode', 'constant_speed_scripted_background')
            placement.SetCustomDataByKey('lane_index', spec.lane)
            chassis = UsdGeom.Xformable(layout.GetPrimAtPath(spec.chassis_path))
            initial = fleet_pose(spec, 0.)
            chassis.AddTranslateOp().Set(Gf.Vec3d(*initial['position_m']))
            x, y, z, w = initial['quaternion_xyzw']
            chassis.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
            UsdPhysics.RigidBodyAPI(chassis.GetPrim()).CreateSimulationOwnerRel().SetTargets([PATHS.physics])
            # In the installed USD-output pipeline, native kinematic targets
            # move collision bodies without updating their authored USD poses.
            # Hide only their rendering; collision and native motion stay on.
            UsdGeom.Imageable(chassis.GetPrim()).CreateVisibilityAttr('invisible')
            proxy = layout.DefinePrim(spec.render_path)
            visual_asset = ('showcase-block-car-instanced-geometry.usda' if instance_visuals
                            else 'showcase-block-car-geometry.usda')
            proxy.GetReferences().AddReference('./assets/' + visual_asset, '/Vehicle/Chassis')
            proxy.SetCustomDataByKey('pose_source', 'measured native body transform; render only')
            proxy.SetCustomDataByKey('native_body_path', spec.chassis_path)
            proxy.SetCustomDataByKey('vehicle_id', spec.vehicle_id)
            proxy_xform = UsdGeom.Xformable(proxy)
            proxy_xform.AddTranslateOp().Set(Gf.Vec3d(*initial['position_m']))
            proxy_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(w, x, y, z).GetNormalized())
            proxy_xform.SetResetXformStack(True)
            if not separate_visuals:
                layout.GetPrimAtPath(spec.chassis_path + '/Visuals/Body').GetAttribute('primvars:displayColor').Set([BODY_PALETTE[index % len(BODY_PALETTE)]])
            if instance_visuals:
                body = layout.GetPrimAtPath(spec.render_path + '/Visuals')
                body.GetReferences().SetReferences([Sdf.Reference('./assets/showcase-block-car-visual-library.usda',
                    '/VisualLibrary/Body' + BODY_STYLE_NAMES[index % len(BODY_STYLE_NAMES)])])
            else:
                layout.GetPrimAtPath(spec.render_path + '/Visuals/Body').GetAttribute('primvars:displayColor').Set([BODY_PALETTE[index % len(BODY_PALETTE)]])
        group = UsdPhysics.CollisionGroup.Define(layout, BACKGROUND_GROUP)
        group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([
            s.chassis_path + '/Colliders/Body' for s in self.specs])
        # These cars must collide with ego, but their roofs are not road support.
        # Do NOT put them in the ground or wheel/chassis collision groups.
        existing_query = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(QUERY_GROUP))
        if existing_query:
            filters = list(existing_query.GetFilteredGroupsRel().GetTargets())
            if Sdf.Path(BACKGROUND_GROUP) not in filters:
                filters.append(Sdf.Path(BACKGROUND_GROUP))
            layout.OverridePrim(QUERY_GROUP).CreateRelationship('physics:filteredGroups', custom=False).SetTargets(filters)
        layout.GetRootLayer().customLayerData = dict(
            showcase_fleet_schema=2, motion_authority='scripted kinematic targets; ego excluded',
            render_authority='nonphysical RenderProxy follows measured native body pose; no physics feedback',
            limitation='Constant-speed nonreactive background; not a human-driving or crash-dynamics model')
        layout.GetRootLayer().Save()
        stage.GetRootLayer().subLayerPaths.insert(0, layout.GetRootLayer().identifier)
        if existing_query and Sdf.Path(BACKGROUND_GROUP) not in existing_query.GetFilteredGroupsRel().GetTargets():
            # A caller may have authored shared physics resources in the root
            # instead of a weaker physics layer. This is an initial scene-owned
            # binding (saved with initial-state), never a per-frame mutation.
            with Usd.EditContext(stage, stage.GetRootLayer()):
                existing_query.GetFilteredGroupsRel().SetTargets(filters)
        self.dynamic_layer = Sdf.Layer.CreateAnonymous('showcase-fleet-motion.usda')
        stage.GetRootLayer().subLayerPaths.insert(0, self.dynamic_layer.identifier)
        self._files = files
        self._hashes = {str(p.relative_to(self.directory)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        self._view = None
        self._ordered_specs = None
        self._target_time_s = 0.
        self._stop_time_s = None
        self._target_count = 0
        self._visual_update_count = 0
        self._last_track_tick = None
        self.metadata = dict(schema_version=2, count=len(self.specs), episode_id=episode_id,
            motion_authority='native PhysX kinematic targets; scripted nonreactive backgrounds',
            observation_source='native measured body pose and velocity; privileged simulator state, not LiDAR',
            pose_contract='metres; Z-up; +X forward; yaw CCW from +X; chassis center; quaternion XYZW',
            base_radius_m=BASE_RADIUS_M, lane_width_m=LANE_WIDTH_M,
            length_m=LENGTH_M, width_m=WIDTH_M, sumo_required=False,
            native_target_hz=120, wheel_visual_update_every_steps=wheel_update_every,
            wheel_visual_hz=120 / wheel_update_every,
            visual_publication='caller publishes measured native pose after physics, before each rendered frame',
            visual_cadence_note='wheel_visual_hz is maximum; publication also bounded by caller render cadence',
            render_proxy_policy='nonphysical geometry-only reference; original chassis invisible but collidable',
            render_orientation='double precision unit quaternion normalized from measured native quaternion',
            physics_scope='one dynamic ego plus collidable kinematic backgrounds; not all-physical traffic',
            collision_group=BACKGROUND_GROUP, road_support_policy='background excluded from suspension queries',
            specs=[asdict(s) for s in self.specs],
            identity_mapping={s.vehicle_id: s.vehicle_path for s in self.specs},
            render_identity_mapping={s.vehicle_id: s.render_path for s in self.specs},
            asset_hashes=self._hashes, layout='showcase-fleet-layout.usda')
        self.validate()
        # Retain attribute handles instead of resolving 48 paths every physics
        # tick. All writes still target the disposable dynamic layer. These
        # handles are explicitly released before the runtime closes its stage.
        self._wheel_attributes = {spec.vehicle_id: tuple(
            stage.GetPrimAtPath(spec.render_path + '/' + name).GetAttribute('xformOp:rotateY:spin')
            for name in WHEEL_NAMES) for spec in self.specs}
        self._render_attributes = {spec.vehicle_id: (
            stage.GetPrimAtPath(spec.render_path).GetAttribute('xformOp:translate'),
            stage.GetPrimAtPath(spec.render_path).GetAttribute('xformOp:orient')) for spec in self.specs}
        self._visual_specs = {}
        if batch_visual_updates:
            # Collect composed values BEFORE the Sdf block. Only Sdf writes are
            # permitted inside it: USD queries cannot see partially applied edits.
            attributes=[a for values in (*self._render_attributes.values(),*self._wheel_attributes.values()) for a in values]
            initial=[(a.GetPath(),a.GetTypeName(),a.Get()) for a in attributes]
            with Sdf.ChangeBlock():
                for path,type_name,value in initial:
                    prim=Sdf.CreatePrimInLayer(self.dynamic_layer,path.GetPrimPath())
                    attribute=Sdf.AttributeSpec(prim,path.name,type_name)
                    attribute.default=value
                    self._visual_specs[str(path)]=attribute
        self.metadata['batched_visual_updates']=batch_visual_updates
        self.metadata['separate_visuals']=separate_visuals
        self.metadata['cache_readback']=cache_readback
        if instance_visuals:
            self.metadata.update(instance_visuals=True,
                visual_instance_policy='shared immutable body palettes and wheel Geometry; editable proxy and spin pivots',
                wheel_geometry_child='Geometry', visual_library='assets/showcase-block-car-visual-library.usda')

    def wheel_geometry_path(self, spec, name):
        """Resolve visual geometry while preserving the public wheel spin pivot."""
        if name not in WHEEL_NAMES:
            raise ValueError('Unknown showcase wheel: ' + str(name))
        return spec.render_path + '/' + name + ('/Geometry' if self.instance_visuals else '')

    def validate(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        errors = [str(e) for e in self.stage.GetCompositionErrors()]
        for spec in self.specs:
            root = self.stage.GetPrimAtPath(spec.vehicle_path)
            chassis = self.stage.GetPrimAtPath(spec.chassis_path)
            if not root or not root.HasAuthoredReferences() or root.IsInstance() or Usd.ModelAPI(root).GetKind() != 'component':
                errors.append('Background must be an editable referenced component: ' + spec.vehicle_id)
            if not chassis or not UsdPhysics.RigidBodyAPI(chassis).GetKinematicEnabledAttr().Get():
                errors.append('Background requires a kinematic rigid body: ' + spec.vehicle_id)
            if chassis and UsdGeom.Imageable(chassis).GetVisibilityAttr().Get() != 'invisible':
                errors.append('Native chassis rendering must be hidden: ' + spec.vehicle_id)
            proxy = self.stage.GetPrimAtPath(spec.render_path)
            if (not proxy or not proxy.HasAuthoredReferences()
                    or UsdGeom.Imageable(proxy).ComputeVisibility() == 'invisible'):
                errors.append('Visible referenced render proxy missing: ' + spec.vehicle_id)
            if proxy:
                for prim in Usd.PrimRange(proxy, Usd.TraverseInstanceProxies()):
                    if any(name.startswith(('Physics', 'Physx')) for name in prim.GetAppliedSchemas()):
                        errors.append('Render proxy must not contain physics APIs: ' + str(prim.GetPath()))
            collider = self.stage.GetPrimAtPath(spec.chassis_path + '/Colliders/Body')
            if not collider or not UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get():
                errors.append('Background collider missing: ' + spec.vehicle_id)
            for name in WHEEL_NAMES:
                wheel = UsdGeom.Cylinder(self.stage.GetPrimAtPath(self.wheel_geometry_path(spec, name) + '/Tire'))
                if not wheel or wheel.GetAxisAttr().Get() != 'Y':
                    errors.append('Wheel axle must be local Y: ' + spec.vehicle_id + '/' + name)
            if self.instance_visuals:
                immutable_paths = [spec.render_path + '/Visuals'] + [self.wheel_geometry_path(spec, name)
                    for name in WHEEL_NAMES]
                for path in immutable_paths:
                    prim = self.stage.GetPrimAtPath(path)
                    if not prim or not prim.IsInstance():
                        errors.append('Immutable visual subtree must share a prototype: ' + path)
                for path in [spec.render_path] + [spec.render_path + '/' + name for name in WHEEL_NAMES]:
                    prim = self.stage.GetPrimAtPath(path)
                    if not prim or prim.IsInstance() or prim.IsInstanceProxy():
                        errors.append('Proxy and wheel spin pivots must remain editable: ' + path)
        if errors:
            raise ValueError('Invalid showcase fleet: ' + '; '.join(errors))
        return dict(passed=True, count=len(self.specs), errors=[])

    def bind(self, simulation_view):
        if self._view is not None:
            raise RuntimeError('Fleet is already bound to native physics')
        self._invalidate_caches()
        view = simulation_view.create_rigid_body_view('/World/Vehicles/vehicle_background_*/Chassis')
        mapping = {s.chassis_path: s for s in self.specs}
        paths = list(view.prim_paths)
        if len(paths) != len(mapping) or set(paths) != set(mapping):
            raise RuntimeError('Native fleet identity binding mismatch: ' + repr(paths))
        self._view = view
        self._ordered_specs = [mapping[path] for path in paths]
        sample = view.get_transforms()
        if hasattr(sample, 'ptr') and hasattr(sample, 'device'):
            import warp as wp
            self._float_tensor = lambda values: wp.array(values, dtype=wp.float32, device=sample.device)
            self._index_tensor = lambda values: wp.array(values, dtype=wp.uint32, device=sample.device)
        elif hasattr(sample, 'detach'):
            import torch
            self._float_tensor = lambda values: torch.as_tensor(values, dtype=torch.float32, device=sample.device)
            self._index_tensor = lambda values: torch.as_tensor(values, dtype=torch.int32, device=sample.device)
        else:
            self._float_tensor = self._index_tensor = lambda values: values
        self.metadata['native_binding_paths'] = paths
        return self.pose_check()

    bind_physics = bind

    def begin_stop(self):
        """Explicit demo shutdown, not traffic intelligence or reactive driving."""
        if self._view is None or self._stop_time_s is not None:
            raise RuntimeError('Fleet rundown requires a bound, not-yet-stopping fleet')
        self._stop_time_s=self._target_time_s
        return dict(start_time_s=self._stop_time_s,base_lane_deceleration_m_s2=3.,
                    scope='Coordinated end-of-demo rundown; not autonomous human response')

    def update(self, sim_time_s):
        if self._view is None:
            raise RuntimeError('Bind fleet after physics start and before commanding motion')
        if not math.isfinite(sim_time_s) or sim_time_s < self._target_time_s:
            raise ValueError('Fleet target time must be finite and monotonic')
        import numpy as np
        poses = [self.expected_pose(s, sim_time_s) for s in self._ordered_specs]
        targets = np.asarray([(*p['position_m'], *p['quaternion_xyzw']) for p in poses], dtype=np.float32)
        self._view.set_kinematic_targets(self._float_tensor(targets),
            self._index_tensor(np.arange(len(poses), dtype=np.uint32)))
        self._target_time_s = float(sim_time_s)
        self._target_count += 1
        self._invalidate_caches()

    def expected_pose(self, spec, time_s):return stopping_pose(spec,time_s,self._stop_time_s)

    def is_active(self, spec):return True

    def logical_identity(self, spec):return spec.vehicle_id

    def publish_visuals(self, *, verify=True, focus_position_xy=None, wheel_distance_m=None):
        """Mirror measured native state to nonphysical geometry before rendering.

        Never write a Chassis transform: that would be a second kinematic input
        and could feed a stale render pose back into the next physics step.
        Visual geometry is not used as an observation or collision body.
        Distance culling skips decorative wheel spins only; all body poses are
        published. verify=False requires the caller to schedule pose checks.
        """
        if type(verify) is not bool:
            raise ValueError('Visual verification must be boolean')
        if (focus_position_xy is None) != (wheel_distance_m is None):
            raise ValueError('Wheel distance and focus position must be supplied together')
        if wheel_distance_m is not None:
            try:
                valid = (not isinstance(wheel_distance_m, bool)
                         and math.isfinite(wheel_distance_m) and wheel_distance_m >= 0
                         and len(focus_position_xy) == 2
                         and all(not isinstance(v, bool) and math.isfinite(v) for v in focus_position_xy))
            except (TypeError, ValueError):
                valid = False
            if not valid:
                raise ValueError('Wheel distance and XY focus must be finite, with nonnegative distance')
        from pxr import Gf, Sdf, Usd
        transforms, _ = self._measured()
        pending=[]
        with Usd.EditContext(self.stage, self.dynamic_layer):
            for spec, pose in zip(self._ordered_specs, transforms):
                x, y, z, qx, qy, qz, qw = map(float, pose)
                translate, orient = self._render_attributes[spec.vehicle_id]
                pending.append((translate,Gf.Vec3d(x, y, z)))
                # PhysX returns float32 quaternion components. Their norm can
                # differ slightly from one; Gf's quaternion-to-rotation path
                # amplifies scalar-component rounding near identity. Normalize
                # in double precision before USD converts it to a matrix.
                # This changes rendering only, never the native body or tracks.
                orientation = Gf.Quatd(qw, qx, qy, qz)
                if abs(orientation.GetLength() - 1.) > .001:
                    raise RuntimeError('Native quaternion is not approximately unit length')
                pending.append((orient,orientation.GetNormalized()))
                near_focus = (wheel_distance_m is None
                              or math.dist((x, y), focus_position_xy) <= wheel_distance_m)
                if self._target_count % self.wheel_update_every == 0 and near_focus:
                    # Decorative wheel angle only; chassis visuals use measured
                    # native pose, never fleet_pose's requested target.
                    spin = self.expected_pose(spec, self._target_time_s)['wheel_spin_deg']
                    for attribute in self._wheel_attributes[spec.vehicle_id]:
                        pending.append((attribute,spin))
            if self.batch_visual_updates:
                edits=[(self._visual_specs[str(attribute.GetPath())],value) for attribute,value in pending]
                with Sdf.ChangeBlock():
                    for spec,value in edits:spec.default=value
            else:
                for attribute,value in pending:attribute.Set(value)
        self._visual_update_count += 1
        return self.visual_pose_check() if verify else dict(
            verified=False, publications=self._visual_update_count, count=len(self.specs))

    def visual_pose_check(self):
        """Check composed visible proxy poses against actual native bodies.

        Call again after rendering to catch competing USD opinions. Screenshots
        are still required to prove RTX consumes the verified composed stage.
        """
        from pxr import Gf, Usd, UsdGeom
        transforms, _ = self._measured()
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        positions, angles, visible, visibility_matches = [], [], [], []
        for spec, pose in zip(self._ordered_specs, transforms):
            prim = self.stage.GetPrimAtPath(spec.render_path)
            matrix = cache.GetLocalToWorldTransform(prim)
            position = matrix.ExtractTranslation()
            forward = matrix.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
            qx, qy, qz, qw = map(float, pose[3:])
            native_yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            rendered_yaw = math.atan2(forward[1], forward[0])
            positions.append(math.dist(position, pose[:3]))
            angles.append(abs(math.atan2(math.sin(rendered_yaw-native_yaw), math.cos(rendered_yaw-native_yaw))))
            visible.append(UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible')
            visibility_matches.append(visible[-1]==self.is_active(spec))
        finite = all(math.isfinite(v) for v in positions+angles)
        return dict(passed=finite and all(visibility_matches) and max(positions) <= .002 and max(angles) <= .0001,
                    max_position_error_m=max(positions), max_yaw_error_rad=max(angles),
                    all_proxies_visible=all(visible), visibility_matches_active=all(visibility_matches), count=len(self.specs),
                    native_target_time_s=self._target_time_s, publications=self._visual_update_count,
                    pose_source='actual native kinematic body transforms; nonphysical visible proxy')

    def _measured(self):
        if self._view is None:
            raise RuntimeError('Measured fleet state requires a bound native body view')
        if self.cache_readback and self._measured_cache is not None:
            return self._measured_cache
        transforms = _array(self._view.get_transforms()).copy()
        velocities = _array(self._view.get_velocities()).copy()
        import numpy as np
        if (transforms.shape != (len(self.specs), 7) or velocities.shape != (len(self.specs), 6)
                or not np.isfinite(transforms).all() or not np.isfinite(velocities).all()):
            raise RuntimeError('Native fleet body state is malformed or nonfinite')
        self.readback_count += 1
        if self.cache_readback:
            transforms.setflags(write=False)
            velocities.setflags(write=False)
            self._measured_cache = (transforms, velocities)
        return transforms, velocities

    def _invalidate_caches(self):
        self._measured_cache = None
        self._track_cache = None

    def tracks(self, tick):
        from traffic.loop_showcase_control import ObjectTrack
        if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
            raise ValueError('Object track tick must be nonnegative integer')
        if self._last_track_tick is not None and tick < self._last_track_tick:
            raise ValueError('Object track ticks cannot go backwards')
        if self.cache_readback and self._track_cache is not None and self._track_cache[0] == tick:
            return list(self._track_cache[1])
        transforms, velocities = self._measured()
        result = []
        for spec, pose, velocity in zip(self._ordered_specs, transforms, velocities):
            if not self.is_active(spec):continue
            x, y, z, qx, qy, qz, qw = map(float, pose)
            yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy+qz*qz))
            result.append(ObjectTrack(episode_id=self.episode_id, vehicle_id=self.logical_identity(spec),
                tick=tick, x_m=x, y_m=y, yaw_rad=yaw,
                speed_m_s=float(velocity[0]*math.cos(yaw)+velocity[1]*math.sin(yaw)),
                length_m=LENGTH_M, width_m=WIDTH_M))
        self._last_track_tick = tick
        self.track_construction_count += 1
        if self.cache_readback:
            self._track_cache = (tick, tuple(result))
        return result

    def pose_check(self, target_time_s=None):
        """After stepping, check actual native state against the commanded target."""
        if target_time_s is None:
            target_time_s = self._target_time_s
        transforms, velocities = self._measured()
        positions, angles, speed_errors = [], [], []
        for spec, pose, velocity in zip(self._ordered_specs, transforms, velocities):
            expected = self.expected_pose(spec, target_time_s)
            positions.append(math.dist(pose[:3], expected['position_m']))
            qx, qy, qz, qw = map(float, pose[3:])
            yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy+qz*qz))
            angles.append(abs(math.atan2(math.sin(yaw-expected['yaw_rad']), math.cos(yaw-expected['yaw_rad']))))
            speed_errors.append(abs(math.hypot(*velocity[:2])-math.hypot(*expected['velocity_m_s'][:2])))
        # Initial warm-up is stationary; commanded target motion must match its
        # configured speed after the first tick. Float32 poses at R500 quantize
        # finite-difference native velocities by a few millimetres/second.
        max_speed = max(speed_errors) if self._target_count else 0.
        return dict(passed=max(positions) <= .002 and max(angles) <= .0001 and max_speed <= .10,
                    max_position_error_m=max(positions), max_yaw_error_rad=max(angles),
                    max_speed_error_m_s=max_speed, target_time_s=float(target_time_s),
                    commanded_steps=self._target_count, count=len(self.specs))

    def static_assets_unchanged(self):
        actual = {str(p.relative_to(self.directory)): hashlib.sha256(p.read_bytes()).hexdigest() for p in self._files}
        return actual == self._hashes

    def release(self):
        self._invalidate_caches()
        self._view = None
        self._ordered_specs = None
        self._float_tensor = self._index_tensor = None
        self._wheel_attributes.clear()
        self._render_attributes.clear()
        self._visual_specs.clear()
        self.stage = None
        self.dynamic_layer = None
