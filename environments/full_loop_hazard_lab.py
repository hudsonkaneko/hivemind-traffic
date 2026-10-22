"""Full highway deployment of the frozen native hazard callbacks.

Only the pre-physics assembly differs: complete V02 highway plus twelve active
scripted kinematic background cars. One dynamic ego follows a bounded 60m task.
The original lane-world export remains retained; full-world.usda is the actual
full-road assembly. This class does not start an app or train a policy.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
from pathlib import Path

from isaaclab.utils import configclass

from environments.loop_hazard_lab import LoopHazardLabCfg, LoopHazardLabEnv
from environments.full_loop_hazard_contract import FullLoopHazardTask, background_settings, background_specs
from traffic.bypass_validation import OrientedBox
from traffic.road_hazards import MeasuredObject
from traffic.showcase_clearance import nearest_clearance
from usd.physical_scene import PATHS
from usd.loop_hazard_scene import BODY_PATH, COLLIDER_PATH, PROXY_PATH, GROUP_PATH, QUERY_GROUP


@configclass
class FullLoopHazardLabCfg(LoopHazardLabCfg):
    highway_directory = 'highway_usd/_v02'


class FullLoopHazardLabEnv(LoopHazardLabEnv):
    def __init__(self, cfg, **kwargs):
        self.background = None
        self.background_frames = ()
        self._background_bound = False
        self.full_scene_export = None
        super().__init__(cfg, **kwargs)
        self.background.bind(self.sim.physics_sim_view)
        self._background_bound = True

    def _setup_scene(self):
        from pxr import UsdPhysics
        from traffic.live_showcase import LiveTraffic
        from usd.live_showcase_fleet import LiveKinematicFleet
        from usd.highway_loop_scene import install_highway_loop
        super()._setup_scene()
        # Only the live stage composition changes, before physics starts.
        # Preserve the base saved road/package as provenance; no published edit.
        old = self.road_view.static_layer.identifier
        root = self.sim.stage.GetRootLayer()
        if old not in root.subLayerPaths:
            raise RuntimeError('Cannot identify the initial decorative road layer')
        root.subLayerPaths = [name for name in root.subLayerPaths if name != old]
        if self.sim.stage.GetPrimAtPath('/World/Environment/Highway'):
            raise RuntimeError('Unexpected stronger decorative road opinions')
        from environments.lane_following_lab import ROOT
        self.highway_report = install_highway_loop(self.sim.stage, self.scene_directory,
                                                   ROOT/self.cfg.highway_directory)
        self.task = FullLoopHazardTask(self.route, stage=self.cfg.hazard_stage)
        specs = background_specs()
        self.live_traffic = LiveTraffic(specs, background_settings())
        self.background = LiveKinematicFleet(self.sim.stage, self.scene_directory, specs,
            live=self.live_traffic, episode_id='full-showcase', wheel_update_every=4,
            batch_visual_updates=True, separate_visuals=True, cache_readback=True,
            instance_visuals=True)
        self.full_inventory = dict(
            bodies=sorted(str(p.GetPath()) for p in self.sim.stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)),
            colliders=sorted(str(p.GetPath()) for p in self.sim.stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)))
        self.full_scene_export = self._save_full_scene()
        self.full_static_hashes = self._package_hashes()

    def _package_hashes(self):
        return {p.relative_to(self.scene_directory).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(Path(self.scene_directory).rglob('*')) if p.is_file()}

    def _save_full_scene(self):
        from pxr import Sdf, Usd, UsdGeom
        directory = Path(self.scene_directory)
        initial_path = directory/'full-initial-state.usda'
        entry_path = directory/'full-world.usda'
        if initial_path.exists() or entry_path.exists():
            raise FileExistsError('Full scene export already exists')
        initial = Usd.Stage.CreateNew(str(initial_path))
        if not Sdf.CopySpec(self.sim.stage.GetRootLayer(), '/World', initial.GetRootLayer(), '/World'):
            raise RuntimeError('Cannot export full scene initial root opinions')
        initial.RemovePrim('/World/envs')
        initial.SetDefaultPrim(initial.GetPrimAtPath('/World'))
        initial.GetRootLayer().Save()
        layers = ['./full-initial-state.usda']
        for index, identifier in enumerate(self.sim.stage.GetRootLayer().subLayerPaths):
            layer = Sdf.Layer.Find(identifier)
            if layer is None:
                raise RuntimeError('Missing composed layer')
            if layer.anonymous:
                name = f'full-runtime-initial-{index:02d}.usda'
                path = directory/name
                if path.exists() or not layer.Export(str(path)):
                    raise RuntimeError('Cannot export unique initial runtime layer')
            else:
                path = Path(layer.realPath).resolve()
                if not path.is_relative_to(directory.resolve()):
                    raise ValueError('Full scene sublayer must be packaged locally')
                name = path.relative_to(directory).as_posix()
            layers.append('./'+name)
        entry = Usd.Stage.CreateNew(str(entry_path))
        entry.GetRootLayer().subLayerPaths = layers
        entry.SetDefaultPrim(entry.GetPrimAtPath('/World'))
        UsdGeom.SetStageMetersPerUnit(entry,1.)
        UsdGeom.SetStageUpAxis(entry,'Z')
        entry.SetTimeCodesPerSecond(120.)
        entry.GetRootLayer().customLayerData = {'full_rsl_scene_schema':1,
            'scope':'Initial full-highway assembly; not trajectory replay; one physical learned ego'}
        entry.GetRootLayer().Save()
        from usd.highway_loop_scene import validate_highway_loop_scene
        validate_highway_loop_scene(entry)
        return {'entry_path':str(entry_path),'layers':layers}

    def _refresh_objects(self):
        super()._refresh_objects()
        if not self._background_bound:
            return
        tracks = self.background.tracks(self._total_physics_ticks)
        _, velocities = self.background._measured()
        measured_velocity = {self.background.logical_identity(spec): velocity
            for spec, velocity in zip(self.background._ordered_specs, velocities)}
        self.background_frames = tuple(MeasuredObject(t.vehicle_id,t.x_m,t.y_m,t.yaw_rad,
            float(measured_velocity[t.vehicle_id][0]),float(measured_velocity[t.vehicle_id][1]),
            t.length_m,t.width_m,self.episode_tick/120) for t in tracks)
        self.task.set_background_objects(self.background_frames,now_s=self.episode_tick/120)

    def _reset_idx(self,env_ids):
        if env_ids is not None and len(env_ids)==0:
            return
        super()._reset_idx(env_ids)
        self.last_reset.update(background_frames=[asdict(o) for o in self.background_frames],
            physical_reset_count=self.reset_count,background_global_tick=self._total_physics_ticks)
        self.extras['reset']=deepcopy(self.last_reset)

    def _observe_completed_tick(self):
        fresh = self._sampled_tick != self.episode_tick
        super()._observe_completed_tick()
        if fresh and self._background_bound:
            native = self.background.pose_check(self._total_physics_ticks/120)
            if not native['passed']:
                raise RuntimeError('Native background motion contract failed')
            state = self.last_state
            box = OrientedBox(*state['position_m'][:2],state['yaw_rad'],4.8,1.8)
            frames = (*self.hazard_frames,*self.background_frames)
            clearance = nearest_clearance(box,[OrientedBox(o.x_m,o.y_m,o.yaw_rad,o.length_m,o.width_m) for o in frames])
            self._step_samples[-1].update(background_frames=[asdict(o) for o in self.background_frames],
                all_object_clearance=clearance, native_background=native,global_tick=self._total_physics_ticks)
            if clearance['overlap'] and self._failure is None:
                self._failure={'tick':self.episode_tick,'reason':'object_overlap','physics_state':deepcopy(state)}

    def _apply_action(self):
        self._observe_completed_tick()
        self.background.update((self._total_physics_ticks+1)/120)
        super()._apply_action()

    def _get_dones(self):
        result = super()._get_dones()
        record = self.extras['lane_transition']
        record.update(background_frames=[asdict(o) for o in self.background_frames],
            deployment_observation=deepcopy(self.task.last_deployment_observation),
            physical_reset_count=self.reset_count,global_physics_tick=self._total_physics_ticks,
            full_sim_scope='one learned dynamic ego; twelve nonreactive kinematic background cars')
        self.extras['final_info']=deepcopy(record)
        return result

    def render_snapshot(self, mode='follow', *, render=True):
        self.background.publish_visuals()
        return super().render_snapshot(mode,render=render)

    def scene_contract(self):
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        from usd.highway_loop_scene import validate_highway_loop_scene
        stage=self.sim.stage
        report=validate_highway_loop_scene(stage)
        self._validate_backgrounds()
        actual=dict(bodies=sorted(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)),
            colliders=sorted(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)))
        if actual!=self.full_inventory or len(actual['bodies'])!=82:
            raise RuntimeError('Full scene body/collider inventory changed')
        rigid=UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(BODY_PATH))
        collider=UsdPhysics.CollisionAPI(stage.GetPrimAtPath(COLLIDER_PATH))
        query=UsdPhysics.CollisionGroup(stage.GetPrimAtPath(QUERY_GROUP))
        if (not rigid.GetRigidBodyEnabledAttr().Get() or not rigid.GetKinematicEnabledAttr().Get()
                or list(map(str,rigid.GetSimulationOwnerRel().GetTargets()))!=[PATHS.physics]
                or not collider.GetCollisionEnabledAttr().Get()
                or UsdGeom.Cube(collider.GetPrim()).GetSizeAttr().Get()!=1.
                or Sdf.Path(GROUP_PATH) not in query.GetFilteredGroupsRel().GetTargets()):
            raise RuntimeError('Hazard physical contract changed')
        if any(p.GetAppliedSchemas() for p in Usd.PrimRange(stage.GetPrimAtPath(PROXY_PATH))):
            raise RuntimeError('Hazard render proxy contains physics')
        if UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(PATHS.chassis)).GetKinematicEnabledAttr().Get():
            raise RuntimeError('Learned ego lost dynamic ownership')
        if self._package_hashes()!=self.full_static_hashes:
            raise RuntimeError('Published full scene sources changed')
        self._contact_reporting_snapshot('full_scene_contract')
        report.update(inventory=actual,static_hashes_unchanged=True,static_hashes=self.full_static_hashes,
            entry_path=self.full_scene_export['entry_path'],contact_reporting=deepcopy(self.contact_reporting),
            background_active_count=len(self.live_traffic.active),background_pool_capacity=80,
            physical_learned_car_count=1,training=False,sensors=False)
        return report

    def _validate_backgrounds(self):
        """Respect intentional dormant invisibility without editing live state."""
        from pxr import Usd, UsdGeom, UsdPhysics
        stage=self.sim.stage
        if len(self.live_traffic.active)!=12 or len(self.background.specs)!=80:
            raise RuntimeError('Fixed background membership changed')
        for spec in self.background.specs:
            root=stage.GetPrimAtPath(spec.vehicle_path)
            body=stage.GetPrimAtPath(spec.chassis_path)
            proxy=stage.GetPrimAtPath(spec.render_path)
            collider=stage.GetPrimAtPath(spec.chassis_path+'/Colliders/Body')
            rigid=UsdPhysics.RigidBodyAPI(body)
            if (not root.HasAuthoredReferences() or root.IsInstance()
                    or Usd.ModelAPI(root).GetKind()!='component'
                    or not rigid.GetRigidBodyEnabledAttr().Get()
                    or not rigid.GetKinematicEnabledAttr().Get()
                    or not collider or not UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get()
                    or UsdGeom.Imageable(body).ComputeVisibility()!='invisible'
                    or not proxy.HasAuthoredReferences()
                    or (UsdGeom.Imageable(proxy).ComputeVisibility()!='invisible')!=self.background.is_active(spec)):
                raise RuntimeError('Background ownership/visibility changed: '+spec.vehicle_id)
            if any(any(n.startswith(('Physics','Physx')) for n in p.GetAppliedSchemas())
                   for p in Usd.PrimRange(proxy,Usd.TraverseInstanceProxies())):
                raise RuntimeError('Background render proxy contains physics')
        if not self.background.static_assets_unchanged():
            raise RuntimeError('Published background assets changed')

    def close(self):
        try:
            if self.background is not None:
                self.background.release()
        finally:
            super().close()
