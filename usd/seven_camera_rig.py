"""Seven perspective views on a reusable vehicle; provisional calibration."""
import json
import math
from pathlib import Path
import shutil

# name, chassis XYZ metres, heading degrees (+left), downward pitch, horizontal FOV
CAMERAS = (
    ('CameraFront', (.8,0,1.12), 0,15,120),
    ('CameraFrontTele', (.8,.18,1.12), 0,3,30),
    ('CameraCrossLeft', (.5,.8,1.05), 65,10,120),
    ('CameraCrossRight', (.5,-.8,1.05), -65,10,120),
    ('CameraRearLeft', (-1.,.8,1.05), 135,10,70),
    ('CameraRearRight', (-1.,-.8,1.05), -135,10,70),
    ('CameraRearTele', (-1.2,0,1.12), 180,3,30),
)
RESOLUTION = (640,360)


def camera_specs():
    return [dict(name=n,translation_m=list(p),heading_deg=y,down_pitch_deg=t,
                 horizontal_fov_deg=f,focal_length_mm=18/math.tan(math.radians(f)/2),
                 resolution=list(RESOLUTION),frequency_hz=10) for n,p,y,t,f in CAMERAS]


def author_seven_camera_rig(stage, chassis_path):
    from pxr import Gf, Sdf, UsdGeom
    root=chassis_path+'/Sensors'
    if not stage.GetPrimAtPath(root):
        raise ValueError('Requires the existing v08 sensor rig')
    # The existing front camera identity remains stable; legacy v08 is untouched.
    for spec in camera_specs():
        name=spec['name']; path=root+'/'+name
        cam=UsdGeom.Camera.Define(stage,path)
        cam.GetPrim().AddAppliedSchema('OmniSensorAPI')
        cam.GetPrim().CreateAttribute('omni:sensor:tickRate',Sdf.ValueTypeNames.Float,custom=False).Set(10.)
        yaw=math.radians(spec['heading_deg']); pitch=math.radians(spec['down_pitch_deg'])
        direction=Gf.Vec3d(math.cos(yaw)*math.cos(pitch),math.sin(yaw)*math.cos(pitch),-math.sin(pitch))
        position=Gf.Vec3d(*spec['translation_m'])
        matrix=Gf.Matrix4d().SetLookAt(position,position+direction,Gf.Vec3d(0,0,1)).GetInverse()
        cam.ClearXformOpOrder()
        # RTX's wrapper expects the standard translate/orient stack even when
        # reset_xform_op_properties=False; matrix-only mounts are discarded.
        cam.GetPrim().RemoveProperty('xformOp:transform:calibratedMount')
        cam.AddTranslateOp().Set(position)
        cam.AddOrientOp().Set(Gf.Quatf(matrix.ExtractRotationQuat()))
        cam.CreateFocalLengthAttr().Set(spec['focal_length_mm'])
        cam.CreateHorizontalApertureAttr().Set(36.)
        cam.CreateVerticalApertureAttr().Set(20.25)
        cam.CreateClippingRangeAttr().Set(Gf.Vec2f(.1,250.))
        cam.GetPrim().CreateAttribute('sensor:resolution',Sdf.ValueTypeNames.Int2,custom=True).Set(Gf.Vec2i(*RESOLUTION))
        cam.GetPrim().CreateAttribute('sensor:frequencyHz',Sdf.ValueTypeNames.Float,custom=True).Set(10.)
        cam.GetPrim().SetCustomDataByKey('cameraRole',name)
        if name != 'CameraFront':
            for suffix,z,scale in [('Housing',position[2],(.12,.12,.09)),
                                    ('Support',position[2]-.16,(.035,.035,.25))]:
                cube=UsdGeom.Cube.Define(stage,root+'/'+name+suffix)
                cube.CreateSizeAttr().Set(1.)
                cube.ClearXformOpOrder()
                # Place behind the optical centre, not across the image.
                cube.AddTranslateOp().Set(Gf.Vec3d(position[0]-.1*math.cos(yaw),position[1]-.1*math.sin(yaw),z))
                cube.AddScaleOp().Set(Gf.Vec3d(*scale))
                cube.CreateDisplayColorAttr().Set([Gf.Vec3f(.06,.08,.10)])
                cube.GetPrim().SetCustomDataByKey('visualOnlyNoMassOrCollision',True)
    stage.GetPrimAtPath(root).SetCustomDataByKey('sensorRigVersion','seven-camera-v01')


def prepare(source, target):
    from pxr import Usd
    from usd.showcase_vehicle import _inspect_source, package_hashes
    root=Path(__file__).resolve().parents[1]
    source,target=Path(source).resolve(),Path(target).resolve()
    if not source.is_relative_to(root) or not target.is_relative_to(root) or target.exists():
        raise ValueError('Use existing project source and a new project version directory')
    model,before=_inspect_source(source)
    shutil.copytree(source,target)
    rig=Usd.Stage.Open(str(target/'assets/vehicle-sensors.usda'))
    author_seven_camera_rig(rig,'/Vehicle/Chassis')
    rig.GetRootLayer().Save()
    entry=Usd.Stage.Open(str(target/'assets/vehicle.usda'))
    entry.GetDefaultPrim().SetAssetInfoByKey('version',target.name)
    entry.GetRootLayer().Save()
    model['sensor_rig'].update(version='seven-camera-v01', camera_resolution=list(RESOLUTION),
                              focal_length_mm=camera_specs()[0]['focal_length_mm'],
                              limitations='Provisional perspective rig; see multi_camera_rig for all seven views')
    model.update(version=target.name,parent_version=source.name,
                 multi_camera_rig=dict(version='seven-camera-v01',cameras=camera_specs(),
                    layer='assets/vehicle-sensors.usda',source_authored=True,
                    dynamics_modified=False,limitations='Provisional perspective rig; not calibrated Hyperion or Cosmos-ready'))
    (target/'model.json').write_text(json.dumps(model,indent=2)+'\n',encoding='utf-8')
    _inspect_source(target)
    if package_hashes(source)!=before:
        raise RuntimeError('Parent source changed')
    return dict(parent_hashes=before,source_hashes=package_hashes(target),cameras=camera_specs())
