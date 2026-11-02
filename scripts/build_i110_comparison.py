"""Portable paired I-110 replay: exact trace positions, reconstructed lane attitude.

Two translated copies make the fixed baseline and preview treatment visible at
the same simulation time. No private car, live SUMO or physics solver is needed.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.build_visual_twin import new_stage, material, vehicle_path, sha
from scripts.report_i110_anticipation import read,write,verify_run


class LaneAttitude:
    def __init__(self,network):
        self.lanes={}
        for lane in ET.parse(network).findall('.//lane'):
            points=np.array([list(map(float,p.split(','))) for p in lane.get('shape').split()])
            if points.shape[1]==2:points=np.column_stack([points,np.zeros(len(points))])
            arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
            self.lanes[lane.get('id')]=(points,arc,float(lane.get('length')))

    def rotation(self,lane_id,position):
        points,arc,length=self.lanes[lane_id]
        distance=max(0.,min(length,position))*arc[-1]/length
        k=min(len(points)-2,max(0,bisect.bisect_right(arc,distance)-1))
        delta=points[k+1]-points[k]
        yaw=math.degrees(math.atan2(delta[1],delta[0]))
        pitch=-math.degrees(math.atan2(delta[2],math.hypot(delta[0],delta[1])))
        return (0.,pitch,yaw)


def shape(stage,path,center,size,mat):
    obj=UsdGeom.Cube.Define(stage,path);obj.CreateSizeAttr(1)
    obj.AddTranslateOp().Set(Gf.Vec3d(*center));obj.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)


def car(path,color):
    stage=new_stage(path,'/Vehicle','component')
    stage.GetDefaultPrim().SetCustomDataByKey('provenance','Procedural illustrative car; no private supplied asset')
    paint=material(stage,'/Vehicle/Looks/Paint',color,.4,.32)
    glass=material(stage,'/Vehicle/Looks/Glass',(.035,.055,.07),.3,.18)
    rubber=material(stage,'/Vehicle/Looks/Rubber',(.025,.025,.025),0,.9)
    lamp=material(stage,'/Vehicle/Looks/Lamp',(.9,.93,.84),0,.25)
    rear=material(stage,'/Vehicle/Looks/RearLamp',(.6,.04,.025),0,.3)
    shape(stage,'/Vehicle/Body',(-2.4,0,.67),(4.8,1.9,.72),paint)
    shape(stage,'/Vehicle/Cabin',(-2.55,0,1.2),(2.4,1.62,.65),glass)
    shape(stage,'/Vehicle/Roof',(-2.55,0,1.54),(2.45,1.64,.08),paint)
    for label,x,y in [('FrontLeft',-.85,.94),('FrontRight',-.85,-.94),
                      ('RearLeft',-3.9,.94),('RearRight',-3.9,-.94)]:
        wheel=UsdGeom.Cylinder.Define(stage,'/Vehicle/Wheels/'+label)
        wheel.CreateRadiusAttr(.32);wheel.CreateHeightAttr(.18);wheel.CreateAxisAttr('Y')
        wheel.AddTranslateOp().Set((x,y,.32))
        UsdShade.MaterialBindingAPI.Apply(wheel.GetPrim()).Bind(rubber)
    for y in (-.66,.66):
        suffix='Left' if y>0 else 'Right'
        shape(stage,'/Vehicle/Headlights/'+suffix,(-.01,y,.73),(.04,.4,.18),lamp)
        shape(stage,'/Vehicle/Taillights/'+suffix,(-4.79,y,.73),(.04,.4,.18),rear)
    stage.GetRootLayer().Save()


def build(baseline,preview,road,output):
    baseline,preview,road,output=[Path(p).resolve() for p in (baseline,preview,road,output)]
    output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(__file__,output/'build-source.py')
    for run in (baseline,preview):verify_run(run)
    if sha(baseline/'network.net.xml')!=sha(preview/'network.net.xml'):
        raise ValueError('Comparison requires the identical network')
    sources={}
    rd=output/'road';rd.mkdir()
    for name in ('environment.usda','layout.usda','physics.usda','road-support.usda'):
        sources[str(road/name)]=sha(road/name);shutil.copyfile(road/name,rd/name)
    road_stage=new_stage(rd/'world.usda')
    road_stage.GetRootLayer().subLayerPaths=['./layout.usda','./physics.usda','./environment.usda']
    road_stage.GetRootLayer().Save()
    write(rd/'provenance.json',dict(source_layer_sha256={Path(k).name:v for k,v in sources.items()},
        source_manifest=read(road/'manifest.json'),
        exclusions='No private supplied car, incident assets, raw lidar or imagery included.',
        status='Provisional source-fitted road; independent survey/current-epoch accuracy remains unqualified.'))
    assets=output/'assets';assets.mkdir()
    for name,color in [('baseline',(.08,.35,.88)),('preview',(.12,.65,.3)),
                       ('human',(.62,.65,.68)),('ramp',(.95,.5,.08))]:
        car(assets/(name+'.usda'),color)
    attitude=LaneAttitude(baseline/'network.net.xml')
    traces={label:{round(f['time'],6):f for f in map(json.loads,(run/'trace.jsonl').read_text(encoding='utf-8').splitlines())}
            for label,run in [('baseline',baseline),('preview',preview)]}
    shifts={'baseline':-110.,'preview':110.}
    trace_digests={label:sha(run/'trace.jsonl') for label,run in [('baseline',baseline),('preview',preview)]}
    times=sorted(set(traces['baseline'])|set(traces['preview']))
    frames=[];mapping={}
    for t in times:
        vehicles=[];counts={}
        for label,trace in traces.items():
            f=trace.get(t)
            counts[label]=f['totals'] if f else trace[max(trace)]['totals']
            for vid,s in (f['states'].items() if f else []):
                identifier=label+'/'+vid
                rotation=attitude.rotation(s['lane_id'],s['lane_position'])
                vehicles.append(dict(id=identifier,source_id=vid,treatment=label,
                    x=s['x']+shifts[label],y=s['y'],z=s['z'],rotation_xyz=rotation,
                    heading=90-rotation[2],speed=s['speed'],acceleration=s['acceleration'],
                    preview_active=s.get('preview',{}).get('active',False)))
                mapping[identifier]=dict(source_id=vid,treatment=label,translation_m=[shifts[label],0,0],
                                         source_trace_sha256=trace_digests[label])
        frames.append(dict(time=t,vehicles=vehicles,comparison=counts))
    write(output/'recording.json',dict(source='Paired recorded SUMO runs; duplicated map for comparison',
        horizon_s=90,frames=frames))
    motion=new_stage(output/'motion.usda');Usd.ModelAPI(UsdGeom.Xform.Define(motion,'/World/Vehicles').GetPrim()).SetKind('group')
    ops={}
    for identifier,meta in sorted(mapping.items()):
        xf=UsdGeom.Xform.Define(motion,vehicle_path(identifier));Usd.ModelAPI(xf.GetPrim()).SetKind('component')
        for k,v in meta.items():
            xf.GetPrim().SetCustomDataByKey(k,Gf.Vec3d(*v) if k=='translation_m' else v)
        xf.GetPrim().SetCustomDataByKey('motion_authority','Recorded SUMO; paired visual replay, no physics')
        ops[identifier]=(xf.AddTranslateOp(),xf.AddRotateXYZOp(),xf.CreateVisibilityAttr())
        model=UsdGeom.Xform.Define(motion,vehicle_path(identifier)+'/Model')
        role=meta['treatment'] if meta['source_id'].startswith('agent_') else 'ramp' if meta['source_id'].startswith('ramp_') else 'human'
        model.GetPrim().GetReferences().AddReference('./assets/'+role+'.usda')
        Usd.ModelAPI(model.GetPrim()).SetKind('subcomponent');model.GetPrim().SetInstanceable(True)
    for f in frames:
        tc=(f['time']-times[0])*60;present={v['id']:v for v in f['vehicles']}
        for identifier,(pos,rot,vis) in ops.items():
            v=present.get(identifier);vis.Set('inherited' if v else 'invisible',tc)
            if v:
                pos.Set((v['x'],v['y'],v['z']),tc);rot.Set(Gf.Vec3f(*v['rotation_xyz']),tc)
    motion.GetRootLayer().Save()
    static=new_stage(output/'static.usda');Usd.ModelAPI(UsdGeom.Xform.Define(static,'/World/Environment').GetPrim()).SetKind('group')
    for label,shift in shifts.items():
        xf=UsdGeom.Xform.Define(static,'/World/Environment/'+label.title())
        xf.GetPrim().GetReferences().AddReference('./road/world.usda','/World/Environment')
        xf.AddTranslateOp().Set((shift,0,0))
    UsdGeom.Scope.Define(static,'/World/Lighting');UsdGeom.Scope.Define(static,'/World/Cameras')
    sky=UsdLux.DomeLight.Define(static,'/World/Lighting/Sky');sky.CreateIntensityAttr(350)
    sun=UsdLux.DistantLight.Define(static,'/World/Lighting/Sun');sun.CreateIntensityAttr(1200);sun.AddRotateXYZOp().Set((-35,-25,0))
    for name in ('Overview','Follow'):
        camera=UsdGeom.Camera.Define(static,'/World/Cameras/'+name)
        camera.CreateFocalLengthAttr(32);camera.CreateHorizontalApertureAttr(36);camera.CreateClippingRangeAttr((.1,3000))
    static.GetRootLayer().Save()
    world=new_stage(output/'world.usda');world.GetRootLayer().subLayerPaths=['./motion.usda','./static.usda']
    world.SetStartTimeCode(0);world.SetEndTimeCode((times[-1]-times[0])*60);world.GetRootLayer().Save()
    if world.GetCompositionErrors():raise RuntimeError(str(world.GetCompositionErrors()))
    # Inspect every sampled pose, source-to-display translation and lifecycle.
    max_error=0.;checks=0
    for f in frames:
        tc=(f['time']-times[0])*60;present={v['id']:v for v in f['vehicles']}
        for identifier in mapping:
            prim=world.GetPrimAtPath(vehicle_path(identifier));v=present.get(identifier)
            assert (UsdGeom.Imageable(prim).ComputeVisibility(tc)!='invisible')==bool(v)
            if v:
                matrix=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tc)
                error=(matrix.ExtractTranslation()-Gf.Vec3d(v['x'],v['y'],v['z'])).GetLength()
                max_error=max(max_error,error);assert error<1e-6
                pitch,yaw=map(math.radians,v['rotation_xyz'][1:])
                forward=Gf.Vec3d(math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),-math.sin(pitch))
                assert (matrix.TransformDir(Gf.Vec3d(1,0,0))-forward).GetLength()<1e-6
                original=traces[v['treatment']][f['time']]['states'][v['source_id']]
                assert abs(v['x']-shifts[v['treatment']]-original['x'])<1e-9
                assert v['y']==original['y'] and v['z']==original['z']
            checks+=1
    assert all(sha(path)==digest for path,digest in sources.items())
    unresolved=[]
    for layer in world.GetUsedLayers():
        for asset in layer.GetExternalReferences():
            if not (Path(layer.realPath).parent/asset).resolve().is_file():unresolved.append(asset)
    assert not unresolved
    manifest=dict(ids=sorted(mapping),duration_s=times[-1]-times[0],source_time_origin=times[0],
        road_bounds_m=[-175,-840,180,30],comparison=True,
        comparison_pairs={i:('preview/' if i.startswith('baseline/') else 'baseline/')+m['source_id'] for i,m in mapping.items()},
        viewer_title='I-110 | Baseline vs Preview',viewer_heading='BLUE BASELINE  |  GREEN PREVIEW',
        viewer_subtitle='Same demand and clock. Left: smooth ACC. Right: 240 m preview. Orange: ramp traffic.',
        viewer_footer='Recorded SUMO replay; no Isaac vehicle dynamics.\nPreview remains experimental; inspect the measured jerk and traffic cost.',
        initial_replay_time_s=48.-times[0],initial_vehicle='preview/agent_0',initial_camera='follow',
        baseline_metrics=read(baseline/'metrics.json')['max_controlled_abs_jerk_m_s3'],
        preview_metrics=read(preview/'metrics.json')['max_controlled_abs_jerk_m_s3'],
        mapping=mapping,source_layer_sha256=sources,source_layers_unchanged=True,
        source_runs={label:dict(manifest_sha256=sha(run/'manifest.json'),trace_sha256=sha(run/'trace.jsonl'))
                     for label,run in [('baseline',baseline),('preview',preview)]},
        pose_lifecycle_checks=checks,max_position_error_m=max_error,unresolved_assets=unresolved,
        attitude='Roll zero; yaw/pitch reconstructed from network lane tangent at source lane position. Not logged body attitude or perception truth.',
        datum='Original SUMO x/y/z in the source road frame; display-only x offsets -110/+110 m. No vertical draping or physics.',
        procedural_vehicle='Reusable illustrative 4.8 m car; not the privately supplied asset',
        source_sha256=sha(__file__),status='composed and numerically checked; native render pending')
    write(output/'manifest.json',manifest)
    shutil.copyfile(baseline/'network.net.xml',output/'network.net.xml')
    print(json.dumps(dict(output=str(output),checks=checks,max_position_error_m=max_error,ids=len(mapping))))
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('baseline','preview','road','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    try:
        build(**vars(args))
    except BaseException as exc:
        if args.output.exists():
            import traceback
            write(args.output/'failure.json',dict(status='failed',error=repr(exc),traceback=traceback.format_exc()))
        raise
