"""v09 fixed PhysX pool adapter. No runtime prim creation or tensor rebinds."""
from usd.showcase_fleet import KinematicFleet


class LiveKinematicFleet(KinematicFleet):
    def __init__(self, *args, live, **kwargs):
        self.live=live
        self.maneuver=False
        super().__init__(*args,**kwargs)
        from pxr import Gf, Usd, UsdGeom
        with Usd.EditContext(self.stage,self.dynamic_layer):
            for spec in self.specs:
                pose=live.pose(spec.vehicle_id)
                # Initial-only pose setup before PhysX starts. Thereafter native
                # set_transforms is reserved for explicit remote entry/exit.
                for path in (spec.chassis_path,spec.render_path):
                    prim=self.stage.GetPrimAtPath(path)
                    prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*pose['position_m']))
                UsdGeom.Imageable(self.stage.GetPrimAtPath(spec.render_path)).CreateVisibilityAttr(
                    'inherited' if self.is_active(spec) else 'invisible')
        self.metadata.update(pool_capacity=80,initial_active_count=len(live.active),
            dormant_policy='Hidden noninteracting parking at X>=10000,Y=10000,Z=20 m; collider remains loaded',
            identity_policy='Stable USD slot; new logical vehicle ID per entry generation',
            live_speed_ramp_m_s2=.5,entry_exit_exclusion_m=live.exclusion_m)

    def expected_pose(self,spec,time_s):return self.live.pose(spec.vehicle_id)
    def is_active(self,spec):return spec.vehicle_id in self.live.active
    def logical_identity(self,spec):return self.live.identity(spec.vehicle_id)

    def reconcile(self,tick,ego_xy,*,maneuver=False):
        events=self.live.reconcile(tick,ego_xy,maneuver=maneuver)
        if not events:return
        import numpy as np
        from pxr import Usd, UsdGeom
        transforms,_=self._measured()
        transforms=transforms.copy();indices=[]
        for event in events:
            index=next(i for i,s in enumerate(self._ordered_specs) if s.vehicle_id==event['slot'])
            spec=self._ordered_specs[index];pose=self.live.pose(spec.vehicle_id)
            transforms[index]=(*pose['position_m'],*pose['quaternion_xyzw'])
            indices.append(index)
            with Usd.EditContext(self.stage,self.dynamic_layer):
                UsdGeom.Imageable(self.stage.GetPrimAtPath(spec.render_path)).GetVisibilityAttr().Set(
                    'inherited' if self.is_active(spec) else 'invisible')
        # Teleport only remote, explicitly entering/exiting pool bodies. Using
        # kinematic targets for a road-to-parking sweep could hit unrelated cars.
        selection=self._index_tensor(np.asarray(indices,dtype=np.uint32))
        self._view.set_transforms(self._float_tensor(transforms),selection)
        # Kinematic velocities are derived by PhysX from the next target. Direct
        # velocity writes are invalid for these bodies (native PhysX rejects them).
        self._invalidate_caches()

    def begin_stop(self):
        self.live.stopping=True
        return super().begin_stop()

    def update(self,sim_time_s):
        self.live.advance(sim_time_s,maneuver=self.maneuver)
        super().update(sim_time_s)
