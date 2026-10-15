"""Disposable preview and explicit session-owned camera control.

The live camera matrix/order are strongest session-layer opinions. Initial poses
are mirrored into the preview layer so saved assemblies remain portable. Camera
commands never touch road/vehicle assets or use Kit's history-sensitive transform
commands; the composed world pose is checked independently after each update.
"""
import math

from usd.physical_scene import PATHS


class HighwayLoopView:
    def __init__(self, stage, route):
        from pxr import Sdf, Usd, UsdGeom, UsdLux
        self.stage, self.route = stage, route
        self.camera_layer = stage.GetSessionLayer()
        self._camera_requested = None
        self.dynamic_layer = Sdf.Layer.CreateAnonymous('loop-preview.usda')
        stage.GetRootLayer().subLayerPaths.insert(0,self.dynamic_layer.identifier)
        with Usd.EditContext(stage,self.dynamic_layer):
            for path in ('/World/Lighting','/World/Cameras','/World/Debug','/World/Debug/Route'):
                UsdGeom.Scope.Define(stage,path)
            sun=UsdLux.DistantLight.Define(stage,'/World/Lighting/Sun')
            sun.CreateIntensityAttr(3000)
            UsdGeom.Xformable(sun).AddRotateXYZOp().Set((-35,25,20))
            sky=UsdLux.DomeLight.Define(stage,'/World/Lighting/Sky')
            sky.CreateIntensityAttr(550)
            for path in (PATHS.follow_camera,PATHS.overview_camera):
                camera=UsdGeom.Camera.Define(stage,path)
                camera.CreateClippingRangeAttr((0.1,10000.))
                camera.CreateFocalLengthAttr(28.)
                camera.CreateHorizontalApertureAttr(36.)
                camera.CreateVerticalApertureAttr(22.5)
                camera.GetPrim().SetCustomDataByKey('camera_owner', 'highway_loop_preview')
                camera.GetPrim().SetCustomDataByKey('live_pose_layer', 'session; portable pose mirrored in preview')
            line=UsdGeom.BasisCurves.Define(stage,'/World/Debug/Route/ProjectedPath')
            line.CreateTypeAttr('linear')
            line.CreateWrapAttr('nonperiodic')
            line.CreateWidthsAttr([.12])
            line.SetWidthsInterpolation('constant')
            line.CreateDisplayColorAttr([(0.,0.85,1.)])
            line.CreateCurveVertexCountsAttr([101])
            self.path=line

    def update(self, state, station_m, mode):
        from pxr import Gf, Sdf, Usd, UsdGeom
        if mode not in ('follow', 'overview'):
            raise ValueError('Unknown highway loop camera mode: ' + str(mode))
        if not all(math.isfinite(value) for value in (*state['position_m'], state['yaw_rad'], station_m)):
            raise ValueError('Camera state and station must be finite')
        with Usd.EditContext(self.stage,self.dynamic_layer):
            points=[self.route.evaluate(station_m+i).position_xy for i in range(101)]
            self.path.GetPointsAttr().Set([(x,y,.065) for x,y in points])
            x,y,z=state['position_m']; yaw=state['yaw_rad']
            if mode=='follow':
                eye=(x-14*math.cos(yaw),y-14*math.sin(yaw),z+9)
                target=(x+3*math.cos(yaw),y+3*math.sin(yaw),z+0.25)
                path=PATHS.follow_camera
            else:
                eye=(0.,-850.,950.); target=(0.,0.,0.)
                path=PATHS.overview_camera
        matrix=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
        camera=UsdGeom.Camera(self.stage.GetPrimAtPath(path))
        # USD cameras look down local -Z with local +Y up. Resetting this camera's
        # transform stack makes the authored matrix explicitly world-space.
        # Session is stronger than GUI/root xform opinions; reassert the order
        # each frame in case viewport tools authored their own SRT op stack.
        for layer in (self.dynamic_layer,self.camera_layer):
            with Usd.EditContext(self.stage,layer):
                xform=UsdGeom.Xformable(camera.GetPrim())
                attribute=camera.GetPrim().GetAttribute('xformOp:transform:highwayLoopCamera')
                operation=(UsdGeom.XformOp(attribute) if attribute
                           else xform.AddTransformOp(opSuffix='highwayLoopCamera'))
                operation.Set(matrix)
                xform.SetXformOpOrder([operation],resetXformStack=True)
                camera.GetPrim().CreateAttribute('omni:kit:centerOfInterest',
                    Sdf.ValueTypeNames.Vector3d,custom=True,variability=Sdf.VariabilityUniform
                    ).Set(matrix.GetInverse().Transform(Gf.Vec3d(*target)))
        self._camera_requested=dict(path=path,mode=mode,eye=tuple(eye),target=tuple(target))
        self.camera_check()
        return path

    def camera_check(self):
        """Check composed USD eye/direction, including after a Kit render pump.

        This detects stale or competing USD opinions, not renderer image lag;
        actual preview screenshots remain a separate acceptance requirement.
        """
        from pxr import Gf, Usd, UsdGeom
        if self._camera_requested is None:
            raise RuntimeError('Camera has not been initialized')
        request=self._camera_requested
        matrix=UsdGeom.Xformable(self.stage.GetPrimAtPath(request['path'])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position=matrix.ExtractTranslation()
        actual_forward=matrix.TransformDir(Gf.Vec3d(0,0,-1)).GetNormalized()
        requested_forward=(Gf.Vec3d(*request['target'])-Gf.Vec3d(*request['eye'])).GetNormalized()
        position_error=(position-Gf.Vec3d(*request['eye'])).GetLength()
        direction_error=(actual_forward-requested_forward).GetLength()
        finite=all(math.isfinite(v) for v in (*position,*actual_forward,position_error,direction_error))
        return dict(camera_path=request['path'],mode=request['mode'],
            requested_position_m=list(request['eye']),actual_position_m=list(position),
            requested_forward=list(requested_forward),actual_forward=list(actual_forward),
            position_error_m=position_error,direction_error=direction_error,
            passed=finite and position_error<=1e-4 and direction_error<=1e-4,
            pose_owner='session layer; initial pose mirrored in preview layer')


def make_controls(state):
    import omni.ui as ui
    window=ui.Window('Highway Sim | Continuous physical loop',width=485,height=165)
    def camera(mode): state['camera']=mode
    def pause(): state['paused']=not state['paused']
    with window.frame:
        with ui.VStack(spacing=6):
            ui.Label('ONE PHYSX CAR | SCRIPTED LOOP FOLLOWER',height=22)
            with ui.HStack(height=30):
                ui.Button('Follow car',clicked_fn=lambda:camera('follow'))
                ui.Button('Overview',clicked_fn=lambda:camera('overview'))
                ui.Button('Pause / resume',clicked_fn=pause)
            label=ui.Label('Settling...',word_wrap=True)
            ui.Label('Cyan: known-map target path. No SUMO, LiDAR or learned policy.',word_wrap=True)
    return window,label
