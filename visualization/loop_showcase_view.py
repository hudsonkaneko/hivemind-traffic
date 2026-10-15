"""Showcase-only camera/path presentation; never controls a chassis."""
import math
from visualization.highway_loop_view import HighwayLoopView
from usd.physical_scene import PATHS


class LoopShowcaseView(HighwayLoopView):
    def __init__(self,stage,route,*,optimized=False,responsive=False):
        super().__init__(stage,route)
        self.optimized=optimized
        self.responsive=responsive
        self.follow_distance_m=20.
        self.follow_height_m=17.
        self._path_visible=True
        if responsive:
            self._path_attributes=(self.path.GetCurveVertexCountsAttr(),
                self.path.GetPointsAttr(),self.path.GetWidthsAttr())
            self._path_count=None
            self._path_width_initialized=False
            self._camera_attributes={}
            self._overview_pose=None

    @property
    def path_visible(self):
        """Whether the caller needs to generate the next overlay points."""
        return self._path_visible

    def set_path_visible(self,visible):
        """Toggle only the disposable overlay, never a vehicle or static asset."""
        from pxr import Usd,UsdGeom
        if not isinstance(visible,bool):raise ValueError('Path visibility must be bool')
        with Usd.EditContext(self.stage,self.dynamic_layer):
            self.path.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)
        self._path_visible=visible

    def _update_responsive(self,state,station_m,mode,path_points):
        """Reuse handles and skip unchanged opinions, checking camera ownership."""
        from pxr import Gf,Sdf,Usd,UsdGeom,Vt
        if not all(math.isfinite(value) for value in (*state['position_m'],state['yaw_rad'],station_m)):
            raise ValueError('Camera state and station must be finite')
        if self.path_visible:
            counts,points,widths=self._path_attributes
            count=len(path_points)
            with Usd.EditContext(self.stage,self.dynamic_layer):
                if count!=self._path_count:
                    counts.Set([count]);self._path_count=count
                points.Set(path_points)
                if not self._path_width_initialized:
                    widths.Set([.20]);self._path_width_initialized=True
        x,y,z=state['position_m'];yaw=state['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
        if mode=='overview':
            eye=(0.,-850.,950.);target=(0.,0.,0.);path=PATHS.overview_camera
        elif mode=='traffic':
            target=(x+30*c,y+30*s,0.);eye=(target[0]-1,target[1]-1,90.);path=PATHS.follow_camera
        else:
            eye=(x-self.follow_distance_m*c,y-self.follow_distance_m*s,z+self.follow_height_m);target=(x+12*c,y+12*s,z+.15);path=PATHS.follow_camera
        if mode=='overview' and self._overview_pose is not None:
            matrix,interest=self._overview_pose
        else:
            matrix=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
            interest=matrix.GetInverse().Transform(Gf.Vec3d(*target))
            if mode=='overview':self._overview_pose=(matrix,interest)
        if path not in self._camera_attributes:
            with Usd.EditContext(self.stage,self.dynamic_layer):
                camera=self.stage.GetPrimAtPath(path)
                xform=UsdGeom.Xformable(camera)
                transform=camera.GetAttribute('xformOp:transform:highwayLoopCamera')
                if not transform:transform=xform.AddTransformOp(opSuffix='highwayLoopCamera').GetAttr()
                order=xform.GetXformOpOrderAttr()
                center=camera.CreateAttribute('omni:kit:centerOfInterest',Sdf.ValueTypeNames.Vector3d,
                    custom=True,variability=Sdf.VariabilityUniform)
                desired_order=Vt.TokenArray(['!resetXformStack!',transform.GetName()])
                self._camera_attributes[path]=(transform,order,center,desired_order)
        transform,order,center,desired_order=self._camera_attributes[path]
        for layer in (self.dynamic_layer,self.camera_layer):
            with Usd.EditContext(self.stage,layer):
                for attribute,value in ((transform,matrix),(order,desired_order),(center,interest)):
                    # Compare each owning layer, not only the composed value:
                    # a viewport can replace an unchanged pose/order/interest,
                    # and the portable preview must stay correct independently.
                    spec=layer.GetPropertyAtPath(attribute.GetPath())
                    if spec is None or spec.default!=value:attribute.Set(value)
        self._camera_requested=dict(path=path,mode=mode,eye=tuple(eye),target=tuple(target))
        self.camera_check()
        return path

    def _update_optimized(self,state,station_m,mode,path_points):
        """Author the final pose once per layer, without a nominal-path pass."""
        from pxr import Gf,Sdf,Usd,UsdGeom
        if not all(math.isfinite(value) for value in (*state['position_m'],state['yaw_rad'],station_m)):
            raise ValueError('Camera state and station must be finite')
        if self.path_visible:
            with Usd.EditContext(self.stage,self.dynamic_layer):
                self.path.GetCurveVertexCountsAttr().Set([len(path_points)])
                self.path.GetPointsAttr().Set(path_points)
                self.path.GetWidthsAttr().Set([.20])
        x,y,z=state['position_m'];yaw=state['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
        if mode=='overview':
            eye=(0.,-850.,950.);target=(0.,0.,0.);path=PATHS.overview_camera
        elif mode=='traffic':
            target=(x+30*c,y+30*s,0.);eye=(target[0]-1,target[1]-1,90.);path=PATHS.follow_camera
        else:
            eye=(x-20*c,y-20*s,z+17);target=(x+12*c,y+12*s,z+.15);path=PATHS.follow_camera
        matrix=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
        interest=matrix.GetInverse().Transform(Gf.Vec3d(*target))
        camera=self.stage.GetPrimAtPath(path)
        for layer in (self.dynamic_layer,self.camera_layer):
            with Usd.EditContext(self.stage,layer):
                xform=UsdGeom.Xformable(camera)
                attribute=camera.GetAttribute('xformOp:transform:highwayLoopCamera')
                op=(UsdGeom.XformOp(attribute) if attribute
                    else xform.AddTransformOp(opSuffix='highwayLoopCamera'))
                op.Set(matrix)
                # Reassert even an unchanged pose: viewport tools can replace
                # the strongest transform order between display updates.
                xform.SetXformOpOrder([op],resetXformStack=True)
                camera.CreateAttribute('omni:kit:centerOfInterest',Sdf.ValueTypeNames.Vector3d,
                    custom=True,variability=Sdf.VariabilityUniform).Set(interest)
        self._camera_requested=dict(path=path,mode=mode,eye=tuple(eye),target=tuple(target))
        self.camera_check()
        return path

    def update_showcase(self,state,station_m,mode,path_points):
        from pxr import Gf,Usd,UsdGeom
        if mode not in ('follow','traffic','overview'):raise ValueError('Unknown showcase camera mode: '+str(mode))
        if self.responsive:return self._update_responsive(state,station_m,mode,path_points)
        if self.optimized:return self._update_optimized(state,station_m,mode,path_points)
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
            # A steeper view keeps the complete ego body and nearby traffic in
            # frame instead of wasting the upper image on the unlit horizon.
            eye=(x-20*c,y-20*s,z+17);target=(x+12*c,y+12*s,z+.15)
        path=PATHS.follow_camera
        matrix=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
        for layer in (self.dynamic_layer,self.camera_layer):
            with Usd.EditContext(self.stage,layer):
                camera=self.stage.GetPrimAtPath(path)
                op=UsdGeom.XformOp(camera.GetAttribute('xformOp:transform:highwayLoopCamera'))
                op.Set(matrix);UsdGeom.Xformable(camera).SetXformOpOrder([op],resetXformStack=True)
        self._camera_requested=dict(path=path,mode=mode,eye=tuple(eye),target=tuple(target))
        return path


def make_controls(state,view=None):
    import omni.ui as ui
    window=ui.Window('Highway Sim | Live traffic showcase',width=520,height=225 if view else 195)
    def camera(mode):state['camera']=mode
    def pause():state['paused']=not state['paused']
    def toggle_path():
        state['show_path']=not state.get('show_path',True)
        view.set_path_visible(state['show_path'])
    if view:
        state.setdefault('show_path',True)
        view.set_path_visible(state['show_path'])
    with window.frame:
        with ui.VStack(spacing=5):
            ui.Label('PHYSICS CAR / MOVING TRAFFIC / SCRIPTED PASSING',height=22)
            with ui.HStack(height=28):
                for title,mode in [('Follow','follow'),('Traffic overhead','traffic'),('Whole loop','overview')]:
                    ui.Button(title,clicked_fn=lambda mode=mode:camera(mode))
                ui.Button('Pause',clicked_fn=pause)
            if view:ui.Button('Show / hide planned path',height=24,clicked_fn=toggle_path)
            label=ui.Label('Preparing...',word_wrap=True)
            ui.Label('Cyan: current planned path. Object positions from simulator state.\nNo LiDAR perception or trained policy in this showcase.',word_wrap=True)
    return window,label
