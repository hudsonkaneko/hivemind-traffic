"""Event scene adapter. Session-only edits keep base road layers immutable."""
import math
from pxr import Gf,UsdGeom,UsdPhysics
from traffic.highway_events import development_events,active_restrictions,preview_trajectory


def right_lane_offset(route,station):
    import bisect
    s=route['station_m'];i=min(len(s)-2,max(0,bisect.bisect_right(s,station)-1))
    f=max(0.,min(1.,(station-s[i])/(s[i+1]-s[i])))
    width=route['widths_m'][i]*(1-f)+route['widths_m'][i+1]*f
    return -width/2+1.8


def route_pose(route,station,lateral=0):
    import bisect
    samples=route['center_xyz'];s=route['station_m']
    if not math.isfinite(station) or not math.isfinite(lateral):raise ValueError('Finite route pose required')
    station=max(s[0],min(s[-1],station));i=min(len(s)-2,max(0,bisect.bisect_right(s,station)-1))
    f=(station-s[i])/(s[i+1]-s[i]);a=samples[i];b=samples[i+1]
    dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
    p=[a[k]+f*(b[k]-a[k]) for k in range(3)]
    p[0]+=-dy/length*lateral;p[1]+=dx/length*lateral
    # Crossfall from the fitted ribbon, interpolated in station and across width.
    width=route['widths_m'][i]*(1-f)+route['widths_m'][i+1]*f
    if abs(lateral)>width/2:raise ValueError('Actor center outside provisional road envelope')
    col=max(0.,min(6.,(lateral/width+.5)*6));j=min(5,int(col));g=col-j
    verts=route['points']
    p[2]=sum(w*verts[row*7+c][2] for row,wrow in [(i,1-f),(i+1,f)]
             for c,w in [(j,wrow*(1-g)),(j+1,wrow*g)])
    yaw=math.degrees(math.atan2(dy,dx));pitch=-math.degrees(math.atan2(b[2]-a[2],length))
    return p,(0.,pitch,yaw)


class EventLayerController:
    def __init__(self,stage,route,probe_path=None):
        self.stage=stage;self.route=route;self.presets=development_events();self.mode='Off'
        self.probe_path=probe_path
        self.previews={mode:preview_trajectory(events) for mode,events in self.presets.items()}
        self.probe_state=None
        self.root='/World/Environment/Props/Events'
        self.actor_paths={'work_zone_demo':[self.root+f'/Cone_{i:02}' for i in range(13)],
                          'disabled_demo':['/World/Vehicles/vehicle_disabled_demo'],
                          'traffic_break_demo':['/World/Vehicles/vehicle_police_demo']}
        for paths in self.actor_paths.values():
            for path in paths:
                if not stage.GetPrimAtPath(path):raise ValueError('Missing event actor '+path)

    def update(self,time_s,mode=None):
        if mode is not None:
            if mode not in self.presets:raise ValueError('Unknown event preset')
            self.mode=mode
        restrictions=active_restrictions(self.presets[self.mode],time_s)
        active={s['event_id']:s for s in restrictions}
        # Caller owns a session edit target; never mutate source actor assets.
        if self.stage.GetEditTarget().GetLayer()!=self.stage.GetSessionLayer():
            raise RuntimeError('Events require a session-layer edit target')
        for event_id,paths in self.actor_paths.items():
            state=active.get(event_id)
            for index,path in enumerate(paths):
                prim=self.stage.GetPrimAtPath(path)
                UsdGeom.Imageable(prim).CreateVisibilityAttr().Set('inherited' if state else 'invisible')
                # Visibility alone does not disable physics.
                for descendant in self._descendants(prim):
                    if descendant.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(descendant).GetCollisionEnabledAttr().Set(bool(state))
                if state:
                    station=state['station_m']+(index*5 if event_id=='work_zone_demo' else 0)
                    right=right_lane_offset(self.route,station)
                    lateral=(right+min(index,4)*.45 if event_id=='work_zone_demo'
                             else right if event_id=='disabled_demo' else state['lateral_m'])
                    pos,angles=route_pose(self.route,station,lateral)
                    if event_id=='traffic_break_demo':
                        angles=(angles[0],angles[1],angles[2]+math.degrees(math.atan2(state['lateral_speed_m_s'],state['obstacle_speed_m_s'])))
                    xf=UsdGeom.Xformable(prim);ops=xf.GetOrderedXformOps()
                    ops[0].Set(Gf.Vec3d(*pos));ops[1].Set(Gf.Vec3f(*angles))
        if self.probe_path:
            trace=self.previews[self.mode];self.probe_state=trace[min(len(trace)-1,int(round(time_s/.1)))]
            station=self.probe_state['station_m']
            pos,angles=route_pose(self.route,station,right_lane_offset(self.route,station))
            ops=UsdGeom.Xformable(self.stage.GetPrimAtPath(self.probe_path)).GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(*pos));ops[1].Set(Gf.Vec3f(*angles))
        return restrictions

    @staticmethod
    def _descendants(prim):
        from pxr import Usd
        return Usd.PrimRange(prim)
