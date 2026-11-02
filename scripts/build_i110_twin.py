"""Compose MLK--Vernon source-fitted roads and independent incident assets.

Run in the existing CPU OpenUSD environment. No PeMS, Kit, physics or training.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdLux,UsdPhysics,UsdShade
from scripts.build_visual_twin import new_stage,material,sha,vehicle_path
from usd.highway_event_layer import route_pose,right_lane_offset


def mesh(stage,path,data,color,collision=False):
    obj=UsdGeom.Mesh.Define(stage,path)
    obj.CreatePointsAttr(data['points']);obj.CreateFaceVertexCountsAttr([3]*len(data['faces']))
    obj.CreateFaceVertexIndicesAttr(np.array(data['faces']).ravel().tolist())
    obj.CreateSubdivisionSchemeAttr('none');obj.CreateDoubleSidedAttr(True)
    obj.CreateDisplayColorAttr([color])
    mat=material(stage,path+'/Material',color,roughness=.9)
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:UsdPhysics.CollisionAPI.Apply(obj.GetPrim()).CreateCollisionEnabledAttr(True)
    return obj


def group(stage,path,kind='group'):
    prim=UsdGeom.Xform.Define(stage,path).GetPrim();Usd.ModelAPI(prim).SetKind(kind);return prim


def support_sampler(record,spacing=.5):
    """Exact triangle height for flat paint; no cylinder markings above asphalt."""
    vertices=np.array(record['points']);triangles=vertices[np.array(record['faces'])]
    cells={}
    for tri in triangles:
        key=tuple(np.floor(tri[:,:2].mean(axis=0)/spacing).astype(int))
        cells.setdefault(key,[]).append(tri)
    def height(x,y):
        key=(int(np.floor(x/spacing)),int(np.floor(y/spacing)))
        for tri in cells.get(key,[]):
            ab=tri[1,:2]-tri[0,:2];ac=tri[2,:2]-tri[0,:2]
            uv=np.linalg.solve(np.column_stack([ab,ac]),np.array([x,y])-tri[0,:2])
            if min(uv)>=-1e-7 and sum(uv)<=1+1e-7:
                return float(tri[0,2]+uv[0]*(tri[1,2]-tri[0,2])+uv[1]*(tri[2,2]-tri[0,2]))
        raise ValueError(f'Paint outside collision surface at {x},{y}')
    return height


def build(geometry,vehicle,output):
    data=json.loads(geometry.read_text());output.mkdir(parents=True,exist_ok=False)
    route=data['meshes']['Southbound'];(output/'event-route.json').write_text(json.dumps(route))
    env=new_stage(output/'environment.usda')
    group(env,'/World/Environment');group(env,'/World/Environment/Highway','component')
    roadmat=(.16,.18,.19)
    for name,record in data['meshes'].items():
        context=name in ('Terrain','BridgeDecks')
        if 'collision_mesh' in data and not context:
            continue
        path='/World/Environment/Highway/'+name
        if name=='Terrain':
            # Cut context terrain beneath the separately fitted road surfaces.
            v=np.array(record['points']);f=np.array(record['faces']);center=v[f].mean(axis=1)
            keep=np.ones(len(f),dtype=bool)
            for road_name in ('Southbound','MlkEntrance','VernonExit'):
                road=data['meshes'][road_name];p=np.array(road['center_xyz']);w=np.array(road['widths_m'])
                for k in range(0,len(center),2000):
                    delta=center[k:k+2000,None,:2]-p[None,:,:2];d=np.sum(delta*delta,axis=2)
                    nearest=np.argmin(d,axis=1)
                    keep[k:k+2000]&=np.sqrt(d[np.arange(len(nearest)),nearest])>w[nearest]/2+.3
            record=dict(record,faces=f[keep].tolist())
        obj=mesh(env,path,record,(.31,.34,.27) if name=='Terrain' else (.44,.45,.43) if context else roadmat)
        obj.GetPrim().SetCustomDataByKey('source','USGS CA_LosAngeles_1_B23 + OSM nominal road envelope')
        obj.GetPrim().SetCustomDataByKey('accuracy','source fitted; independent accuracy unqualified')
        if not context:
            # Render offset avoids z-fighting; collision uses the unshifted source fit.
            obj.AddTranslateOp().Set((0,0,.015))
    if 'collision_mesh' in data:
        obj=mesh(env,'/World/Environment/Highway/RoadSurface',data['collision_mesh'],roadmat)
        obj.AddTranslateOp().Set((0,0,.015))
        obj.GetPrim().SetCustomDataByKey('source','NAIP image-interpreted envelope + USGS lidar fit; common support mesh')
    env.GetRootLayer().Save()

    physics=new_stage(output/'physics.usda')
    group(physics,'/World/Physics')
    scene=UsdPhysics.Scene.Define(physics,'/World/Physics/Scene')
    scene.CreateGravityDirectionAttr((0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    if 'collision_mesh' in data:
        support=new_stage(output/'road-support.usda','/RoadSupport','component')
        surface=mesh(support,'/RoadSupport/Surface',data['collision_mesh'],roadmat,True).GetPrim()
        # Explicit margins: auto-scaled margins on an 815m mesh otherwise emit
        # speculative chassis contacts while it is still >0.28m above the road.
        surface.AddAppliedSchema('PhysxCollisionAPI')
        surface.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.02)
        surface.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
        support.GetRootLayer().Save()
        obj=UsdGeom.Xform.Define(physics,'/World/Physics/RoadSupport')
        obj.GetPrim().GetReferences().AddReference('./road-support.usda')
        obj.CreatePurposeAttr('guide');obj.CreateVisibilityAttr('invisible')
    else:
        for name in ('Southbound','MlkEntrance','VernonExit'):
            obj=mesh(physics,'/World/Physics/'+name+'Support',data['meshes'][name],roadmat,True)
            obj.CreatePurposeAttr('guide');obj.CreateVisibilityAttr('invisible')
    physics.GetRootLayer().Save()

    props=new_stage(output/'cone.usda','/Cone','component')
    cone=UsdGeom.Cone.Define(props,'/Cone/Body');cone.CreateRadiusAttr(.2);cone.CreateHeightAttr(.7)
    cone.AddTranslateOp().Set((0,0,.35));cone.CreateDisplayColorAttr([(1,.22,.025)])
    UsdPhysics.CollisionAPI.Apply(cone.GetPrim()).CreateCollisionEnabledAttr(False)
    base=UsdGeom.Cube.Define(props,'/Cone/Base');base.CreateSizeAttr(1)
    base.AddTranslateOp().Set((0,0,.04));base.AddScaleOp().Set((.45,.45,.08));base.CreateDisplayColorAttr([(.05,.05,.05)])
    UsdPhysics.CollisionAPI.Apply(base.GetPrim()).CreateCollisionEnabledAttr(False)
    props.GetRootLayer().Save()

    event=new_stage(output/'events.usda')
    group(event,'/World/Environment');group(event,'/World/Environment/Props');group(event,'/World/Environment/Props/Events')
    for i in range(13):
        xf=UsdGeom.Xform.Define(event,f'/World/Environment/Props/Events/Cone_{i:02}')
        xf.GetPrim().GetReferences().AddReference('./cone.usda')
        xf.AddTranslateOp().Set((0,0,0));xf.AddRotateXYZOp().Set((0,0,0));xf.CreateVisibilityAttr('invisible')
    group(event,'/World/Vehicles')
    asset=os.path.relpath(vehicle,output).replace('\\','/')
    for name in ('disabled_demo','police_demo'):
        path='/World/Vehicles/vehicle_'+name;group(event,path,'component')
        xf=UsdGeom.Xformable(event.GetPrimAtPath(path));xf.AddTranslateOp().Set((0,0,0));xf.AddRotateXYZOp().Set((0,0,0))
        UsdGeom.Imageable(xf).CreateVisibilityAttr('invisible')
        model=UsdGeom.Xform.Define(event,path+'/Model');model.GetPrim().GetReferences().AddReference(asset)
        Usd.ModelAPI(model.GetPrim()).SetKind('subcomponent')
        box=UsdGeom.Cube.Define(event,path+'/CollisionEnvelope');box.CreateSizeAttr(1)
        box.AddTranslateOp().Set((-2.5,0,.92));box.AddScaleOp().Set((5.,2.22,1.84))
        box.CreatePurposeAttr('guide');box.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(box.GetPrim()).CreateCollisionEnabledAttr(False)
        if name=='police_demo':
            for side,color in [(-1,(1.,.03,.03)),(1,(.03,.15,1.))]:
                lamp=UsdGeom.Cube.Define(event,path+('/RedBeacon' if side<0 else '/BlueBeacon'))
                lamp.CreateSizeAttr(1);lamp.AddTranslateOp().Set((-2.5,side*.35,1.9));lamp.AddScaleOp().Set((.3,.45,.12));lamp.CreateDisplayColorAttr([color])
    event.GetRootLayer().Save()

    motion=new_stage(output/'motion.usda');group(motion,'/World/Vehicles')
    identifier='inspection_probe';path=vehicle_path(identifier);group(motion,path,'component')
    pos,angles=route_pose(route,60,right_lane_offset(route,60))
    xf=UsdGeom.Xformable(motion.GetPrimAtPath(path));xf.AddTranslateOp().Set(Gf.Vec3d(*pos));xf.AddRotateXYZOp().Set(Gf.Vec3f(*angles))
    model=UsdGeom.Xform.Define(motion,path+'/Model');model.GetPrim().GetReferences().AddReference(asset);Usd.ModelAPI(model.GetPrim()).SetKind('subcomponent')
    xf.GetPrim().SetCustomDataByKey('motion_authority','scripted kinematic inspection; no traffic or physical driving claim')
    motion.GetRootLayer().Save()

    layout=new_stage(output/'layout.usda');UsdGeom.Scope.Define(layout,'/World/Lighting')
    sky=UsdLux.DomeLight.Define(layout,'/World/Lighting/Sky');sky.CreateIntensityAttr(300)
    sky.CreateColorAttr((.8,.86,1.))
    light=UsdLux.DistantLight.Define(layout,'/World/Lighting/Sun');light.CreateIntensityAttr(1000);light.AddRotateXYZOp().Set((-30,-25,0))
    UsdGeom.Scope.Define(layout,'/World/Cameras')
    for name in ('Overview','Follow'):
        camera=UsdGeom.Camera.Define(layout,'/World/Cameras/'+name);camera.CreateFocalLengthAttr(22 if name=='Overview' else 35)
        camera.CreateHorizontalApertureAttr(36);camera.CreateClippingRangeAttr((.1,3000))
    if 'lane_boundaries_m' in route:
        # Reconstructed markings are appearance/semantics, never extra colliders.
        group(layout,'/World/Environment');group(layout,'/World/Environment/Highway','component')
        UsdGeom.Scope.Define(layout,'/World/Environment/Highway/Markings')
        s=np.array(route['station_m'])
        boundaries=route['lane_boundaries_m']+[route['aux_outer_boundary_m']]
        surface_z=support_sampler(data['collision_mesh'])
        for i,offsets in enumerate(boundaries):
            points=[];faces=[]
            for a in np.arange(s[0],s[-1]-1,3):
                auxiliary=float(np.interp(a,s,route['aux_fraction']))
                if i==5 and auxiliary<=.001:continue
                dashed=0<i<4 or (i==4 and auxiliary>.001)
                if dashed and int(round(a/3))%4:continue
                b=min(a+(3 if dashed else 3.05),s[-1])
                corners=[]
                for t,edge in ((a,-.065),(b,-.065),(b,.065),(a,.065)):
                    lateral=float(np.interp(t,s,offsets))+edge;p,_=route_pose(route,float(t),lateral)
                    try:z=surface_z(p[0],p[1])+.019
                    except ValueError:break  # clipped boundary endpoint, never extrapolate paint
                    corners.append((p[0],p[1],z))
                if len(corners)!=4:continue
                first=len(points);points.extend(corners);faces.extend([[first,first+1,first+2],[first,first+2,first+3]])
            marking=mesh(layout,f'/World/Environment/Highway/Markings/Boundary_{i}',dict(points=points,faces=faces),
                         (.95,.72,.12) if i==0 else (.85,.85,.8))
            marking.GetPrim().SetCustomDataByKey('status','Flat reconstructed image-supported semantics; paint locations approximate')
    # Provisional centerline guide is explicitly separate from the road asset.
    UsdGeom.Scope.Define(layout,'/World/Debug')
    guide=UsdGeom.BasisCurves.Define(layout,'/World/Debug/ProvisionalAlignment')
    guide.CreateTypeAttr('linear');guide.CreateCurveVertexCountsAttr([len(route['center_xyz'])])
    guide.CreatePointsAttr([(x,y,z+.08) for x,y,z in route['center_xyz']]);guide.CreateWidthsAttr([.15]);guide.SetWidthsInterpolation('constant')
    guide.CreateDisplayColorAttr([(1,.6,.05)]);guide.CreateVisibilityAttr('invisible')
    layout.GetRootLayer().Save()
    world=new_stage(output/'world.usda')
    world.GetRootLayer().subLayerPaths=['./motion.usda','./events.usda','./layout.usda','./physics.usda','./environment.usda']
    world.SetStartTimeCode(0);world.SetEndTimeCode(80*60);world.GetRootLayer().Save()
    if world.GetCompositionErrors():raise RuntimeError(str(world.GetCompositionErrors()))
    frames=[dict(time=t,vehicles=[dict(id=identifier,x=pos[0],y=pos[1],speed=0.,heading=180.)]) for t in range(81)]
    (output/'recording.json').write_text(json.dumps(dict(source='inspection time base; live scripted probe overrides pose',frames=frames)))
    manifest=dict(ids=[identifier],duration_s=80,source_time_origin=0,road_bounds_m=[-65,-840,70,30],
        units='metres',up_axis='Z',origin_6340=data['origin_6340'],z_offset_m=data['z_offset_m'],
        highway_events=True,event_route='event-route.json',probe_path=path,
        viewer_title='I-110 near USC | MLK to Vernon',viewer_heading='MLK → VERNON | FIRST ROAD BUILD',
        viewer_subtitle=('Image-refined envelopes, reconstructed markings, unified road support.' if 'collision_mesh' in data else 'Lidar-fitted grade and separate bridges. Pavement widths/tapers provisional.'),
        viewer_footer='Scripted inspection car; no learned driving or calibrated traffic.\nEvents are synthetic and optional. PeMS is not required to open this scene.',
        geometry_sha256=sha(geometry),vehicle_sha256=sha(vehicle),limitations=data['limitations'],
        validated_physics_driving=False,pems_required=False,event_defaults='Off',
        supplied_vehicle_redistribution='local only; not cleared for publication')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(dict(scene=str(output/'world.usda'),composition_errors=0,ids=manifest['ids'])))
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('geometry','vehicle','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    if not args.output.resolve().is_relative_to(Path(__file__).resolve().parents[1]):p.error('Output must remain in checkout')
    build(args.geometry,args.vehicle,args.output)
