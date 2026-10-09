"""Showcase-only camera/path presentation; never controls a chassis."""
import math
from visualization.highway_loop_view import HighwayLoopView
from usd.physical_scene import PATHS


class LoopShowcaseView(HighwayLoopView):
    def update_showcase(self,state,station_m,mode,path_points):
        from pxr import Gf,Usd,UsdGeom
        super().update(state,station_m,'overview' if mode=='overview' else 'follow')
        with Usd.EditContext(self.stage,self.dynamic_layer):
            self.path.GetCurveVertexCountsAttr().Set([len(path_points)])
            self.path.GetPointsAttr().Set(path_points)
            self.path.GetWidthsAttr().Set([.20])
        if mode=='overview':return PATHS.overview_camera
        x,y,z=state['position_m'];yaw=state['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
        if mode=='traffic':
            target=(x+30*c,y+30*s,0.);eye=(target[0]-1,target[1]-1,90.)
        else:
            eye=(x-20*c,y-20*s,z+13);target=(x+23*c,y+23*s,z+.15)
        path=PATHS.follow_camera
        matrix=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
        for layer in (self.dynamic_layer,self.camera_layer):
            with Usd.EditContext(self.stage,layer):
                camera=self.stage.GetPrimAtPath(path)
                op=UsdGeom.XformOp(camera.GetAttribute('xformOp:transform:highwayLoopCamera'))
                op.Set(matrix);UsdGeom.Xformable(camera).SetXformOpOrder([op],resetXformStack=True)
        self._camera_requested=dict(path=path,mode=mode,eye=tuple(eye),target=tuple(target))
        return path


def make_controls(state):
    import omni.ui as ui
    window=ui.Window('Highway Sim | Live traffic showcase',width=520,height=195)
    def camera(mode):state['camera']=mode
    def pause():state['paused']=not state['paused']
    with window.frame:
        with ui.VStack(spacing=5):
            ui.Label('PHYSICS CAR / MOVING TRAFFIC / SCRIPTED PASSING',height=22)
            with ui.HStack(height=28):
                for title,mode in [('Follow','follow'),('Traffic overhead','traffic'),('Whole loop','overview')]:
                    ui.Button(title,clicked_fn=lambda mode=mode:camera(mode))
                ui.Button('Pause',clicked_fn=pause)
            label=ui.Label('Preparing...',word_wrap=True)
            ui.Label('Cyan: current planned path. Object positions from simulator state.\nNo LiDAR perception or trained policy in this showcase.',word_wrap=True)
    return window,label
