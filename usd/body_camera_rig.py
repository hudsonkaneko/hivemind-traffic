"""Seat seven compact cameras against measured exterior body surfaces."""
import json
import math
from pathlib import Path
import shutil
import numpy as np
from usd.seven_camera_rig import camera_specs

# name, axis of outward ray, outward sign, other coordinates at body surface
ANCHORS = [('CameraFront',0,1,(0.,0.,-.05)),
           ('CameraFrontTele',0,1,(0.,.22,.20)),
           ('CameraCrossLeft',1,1,(1.2,0.,-.10)),
           ('CameraCrossRight',1,-1,(1.2,0.,-.10)),
           ('CameraRearLeft',1,1,(-1.85,0.,.25)),
           ('CameraRearRight',1,-1,(-1.85,0.,.25)),
           ('CameraRearTele',0,-1,(0.,0.,.20))]


def exterior_anchor(triangles,axis,sign,coordinates):
    """Intersect an inward ray with the actual body; nearest exterior triangle."""
    origin=np.array(coordinates,dtype=float);origin[axis]=sign*10.
    direction=np.zeros(3);direction[axis]=-sign
    e1=triangles[:,1]-triangles[:,0];e2=triangles[:,2]-triangles[:,0]
    h=np.cross(direction,e2);a=np.einsum('ij,ij->i',e1,h)
    valid=np.abs(a)>1e-10
    inv=np.divide(1.,a,out=np.zeros_like(a),where=valid)
    s=origin-triangles[:,0];u=inv*np.einsum('ij,ij->i',s,h)
    q=np.cross(s,e1);v=inv*(q@direction);t=inv*np.einsum('ij,ij->i',e2,q)
    hits=t[valid&(u>=0)&(v>=0)&(u+v<=1)&(t>=0)]
    if not len(hits):raise ValueError('No exterior body surface for '+str(coordinates))
    return origin+float(hits.min())*direction


def prepare(source,target):
    from pxr import Gf,Usd,UsdGeom
    from usd.showcase_vehicle import _inspect_source,package_hashes
    root=Path(__file__).resolve().parents[1]
    source,target=Path(source).resolve(),Path(target).resolve()
    if not source.is_relative_to(root) or not target.is_relative_to(root) or target.exists():
        raise ValueError('Use a new version inside the project')
    model,before=_inspect_source(source)
    composed=Usd.Stage.Open(str(source/'assets/vehicle.usda'))
    body=UsdGeom.Mesh(composed.GetPrimAtPath('/Vehicle/Chassis/Visuals/Body'))
    transform=body.ComputeLocalToWorldTransform(Usd.TimeCode.Default())*UsdGeom.Xformable(composed.GetPrimAtPath('/Vehicle/Chassis')).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    points=np.array([transform.Transform(Gf.Vec3d(*p)) for p in body.GetPointsAttr().Get()])
    indices=np.array(body.GetFaceVertexIndicesAttr().Get());tri=[];offset=0
    for count in body.GetFaceVertexCountsAttr().Get():
        face=indices[offset:offset+count];offset+=count
        for j in range(1,count-1):tri.append(points[[face[0],face[j],face[j+1]]])
    triangles=np.array(tri)
    specs=camera_specs()
    for spec,(name,axis,sign,coords) in zip(specs,ANCHORS):
        assert spec['name']==name
        anchor=exterior_anchor(triangles,axis,sign,coords)
        yaw=math.radians(spec['heading_deg']);direction=np.array([math.cos(yaw),math.sin(yaw),0.])
        spec.update(body_anchor_m=anchor.tolist(),translation_m=(anchor+.10*direction).tolist(),
                    housing_center_m=(anchor+.035*direction).tolist(),mount_style='compact body housing')
    shutil.copytree(source,target)
    stage=Usd.Stage.Open(str(target/'assets/vehicle-sensors.usda'));base='/Vehicle/Chassis/Sensors/'
    for p in list(stage.GetPrimAtPath(base.rstrip('/')).GetChildren()):
        if p.GetName().startswith('Camera') and p.GetName().endswith(('Housing','Support')):
            stage.RemovePrim(p.GetPath())
    for spec in specs:
        cam=UsdGeom.Camera(stage.GetPrimAtPath(base+spec['name']))
        yaw=math.radians(spec['heading_deg']);pitch=math.radians(spec['down_pitch_deg'])
        pos=Gf.Vec3d(*spec['translation_m']);direction=Gf.Vec3d(math.cos(yaw)*math.cos(pitch),math.sin(yaw)*math.cos(pitch),-math.sin(pitch))
        matrix=Gf.Matrix4d().SetLookAt(pos,pos+direction,Gf.Vec3d(0,0,1)).GetInverse()
        cam.ClearXformOpOrder();cam.AddTranslateOp().Set(pos);cam.AddOrientOp().Set(Gf.Quatf(matrix.ExtractRotationQuat()))
        housing=UsdGeom.Cube.Define(stage,base+spec['name']+'Housing')
        housing.CreateSizeAttr().Set(1.)
        housing.AddTranslateOp().Set(Gf.Vec3d(*spec['housing_center_m']))
        housing.AddRotateZOp().Set(spec['heading_deg'])
        housing.AddScaleOp().Set(Gf.Vec3d(.13,.12,.10))
        housing.CreateDisplayColorAttr().Set([Gf.Vec3f(.035,.045,.055)])
        housing.GetPrim().SetCustomDataByKey('visualOnlyNoMassOrCollision',True)
        housing.GetPrim().SetCustomDataByKey('bodySurfaceAnchor',Gf.Vec3d(*spec['body_anchor_m']))
    stage.GetRootLayer().Save()
    entry=Usd.Stage.Open(str(target/'assets/vehicle.usda'));entry.GetDefaultPrim().SetAssetInfoByKey('version',target.name);entry.GetRootLayer().Save()
    model.update(version=target.name,parent_version=source.name)
    model['multi_camera_rig'].update(version='body-camera-v01',cameras=specs,mount_style='body attached; no stalks')
    model['sensor_rig'].update(camera_translation_m=specs[0]['translation_m'],visual_supports='Seven compact body camera housings; central roof LiDAR support retained')
    (target/'model.json').write_text(json.dumps(model,indent=2)+'\n',encoding='utf-8')
    _inspect_source(target)
    if package_hashes(source)!=before:raise RuntimeError('Parent source changed')
    return dict(parent_hashes=before,source_hashes=package_hashes(target),cameras=specs)
