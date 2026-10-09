"""Independent saved-stage, geometry, route-policy and curvature acceptance."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon,LineString,Point
from shapely.ops import unary_union
from pxr import Usd,UsdGeom,UsdPhysics,UsdValidation
from route_policy import generate_plans


def check(condition,message):
    if not condition: raise AssertionError(message)


def faces(mesh):
    points=np.array(mesh.GetPointsAttr().Get(),dtype=float)
    counts=mesh.GetFaceVertexCountsAttr().Get(); indexes=mesh.GetFaceVertexIndicesAttr().Get()
    valid,reason=UsdGeom.Mesh.ValidateTopology(indexes,counts,len(points)); check(valid,str(mesh.GetPath())+reason)
    result=[]; cursor=0
    for count in counts:
        result.append(list(indexes[cursor:cursor+count])); cursor+=count
    return points,result


def analytic_curvature(p,outbound):
    # Independent differentiation of quintic radial interpolation, densely
    # evaluated between waypoint samples to catch a hidden curvature maximum.
    t=np.linspace(0,1,100001)
    alpha=math.radians(p['connector_span_deg'])
    inner=p['inner_radius_m']+4.5*p['lane_width_m']; outer=p['collector_radius_m']
    r0,r1=(inner,outer) if outbound else (outer,inner); delta=r1-r0
    r=r0+delta*(10*t**3-15*t**4+6*t**5)
    dr=delta*(30*t**2-60*t**3+30*t**4)/alpha
    ddr=delta*(60*t-180*t**2+120*t**3)/(alpha*alpha)
    curvature=np.abs((r*r+2*dr*dr-r*ddr)/(r*r+dr*dr)**1.5)
    return dict(min_radius_m=float(1/max(curvature)),max_lateral_g_at_65mph=float(max(curvature)*p['target_speed_m_s']**2/9.81))


def validate(directory):
    n=json.loads((directory/'navigation.json').read_text()); p=n['parameters']; lanes=n['lanes']; edges=n['edges']
    stage=Usd.Stage.Open(str(directory/'highway_v02.usda'))
    check(stage is not None and not stage.GetCompositionErrors(),'USD open/composition failure')
    check(str(stage.GetDefaultPrim().GetPath())=='/World','defaultPrim')
    check(UsdGeom.GetStageMetersPerUnit(stage)==1 and UsdGeom.GetStageUpAxis(stage)=='Z','Coordinate convention')
    check(len(lanes)==14 and len(edges)==28 and len(n['nodes'])==16,'Network counts')
    for name in ['RoadNetwork/CircularHighway','RoadNetwork/AuxiliaryLane','RoadNetwork/OuterCollector','Navigation','SpawnPoints','MergeZones','DivergeZones','Debug','Physics']+['RoadNetwork/Ramps/'+s for s in ['East','North','West','South']]:
        check(bool(stage.GetPrimAtPath('/World/'+name)),'Missing hierarchy: '+name)
    relation_count=0
    for prim in stage.Traverse():
        check(not prim.HasAPI(UsdPhysics.RigidBodyAPI),'Environment must remain static')
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                check(bool(stage.GetObjectAtPath(target)),'Dangling relationship: '+str(target)); relation_count+=1
        if prim.IsA(UsdGeom.Mesh):
            points,meshfaces=faces(UsdGeom.Mesh(prim)); check(np.isfinite(points).all(),'Nonfinite mesh')
            for f in meshfaces:
                a,b,c=points[f[:3]]
                check(np.linalg.norm(np.cross(b-a,c-a))>1e-12,'Degenerate mesh face: '+str(prim.GetPath()))
    float_error=0.; max_step=0.; tangent_error=0.
    for item in [*lanes.values(),*edges.values()]:
        c=UsdGeom.BasisCurves(stage.GetPrimAtPath(item['usd_path'])); saved=np.array(c.GetPointsAttr().Get())
        pts=np.array(item['points']); check(saved.shape==pts.shape,'Waypoint count')
        float_error=max(float_error,float(np.max(np.abs(saved-pts))))
        check(c.GetTypeAttr().Get()=='linear','Curve interpolation')
        check(c.GetWrapAttr().Get()==('periodic' if item['closed'] else 'nonperiodic'),'Curve closure')
        steps=np.linalg.norm(np.diff(pts,axis=0),axis=1)
        check(steps.min()>0,'Duplicate navigation points'); max_step=max(max_step,float(steps.max()))
    check(float_error<.0001 and max_step<1.1,'Coordinate/sample resolution')
    links={}
    for key,lane in lanes.items():
        pts=np.array(lane['points'])
        if lane['closed']:
            check(np.max(np.abs(np.linalg.norm(pts[:,:2],axis=1)-lane['radius_m']))<1e-8,'Noncircular ring')
            check(np.cross(pts[0],pts[1])[2]>0,'Wrong circle direction')
        else:
            line=LineString(pts); check(line.is_simple,'Self-crossing connector'); links[key]=line
            for idx,direction in [(0,pts[1]-pts[0]),(-1,pts[-1]-pts[-2])]:
                desired=np.array([-pts[idx,1],pts[idx,0],0])
                error=math.degrees(math.acos(float(np.clip(np.dot(direction,desired)/np.linalg.norm(direction)/np.linalg.norm(desired),-1,1))))
                tangent_error=max(tangent_error,error); check(error<.06,'Connector tangent mismatch')
    for i,(key,line) in enumerate(links.items()):
        for other,otherline in list(links.items())[i+1:]:
            check(line.distance(otherline)>p['lane_width_m'],'Unintended connector crossing/overlap')
    for key,e in edges.items():
        check(bool(e['successors']),'Dead end: '+key)
        authored=[str(x) for x in stage.GetPrimAtPath(e['usd_path']).GetRelationship('navigation:successors').GetTargets()]
        check(authored==[edges[k]['usd_path'] for k in e['successors']],'USD route graph mismatch')
        for successor in e['successors']:
            check(successor in edges,'Unknown successor')
            if not e['closed']:
                check(math.dist(e['points'][-1],edges[successor]['points'][0])<1e-9,'Disconnected route join: '+key)
    routable={k for k,e in edges.items() if e['kind']!='main'}
    for start in routable:
        todo=[start]; seen=set()
        while todo:
            key=todo.pop()
            if key not in seen: seen.add(key); todo.extend(edges[key]['successors'])
        check(seen==routable,'Collector/auxiliary routes not strongly connected')
    curvature={kind:analytic_curvature(p,kind=='exit') for kind in ['exit','return']}
    sampled_radii={}
    for key in links:
        pts=np.array(lanes[key]['points'])
        ab=pts[1:-1]-pts[:-2]; bc=pts[2:]-pts[1:-1]; ac=pts[2:]-pts[:-2]
        radius=np.linalg.norm(ab,axis=1)*np.linalg.norm(bc,axis=1)*np.linalg.norm(ac,axis=1)/(2*np.maximum(np.linalg.norm(np.cross(ab,ac),axis=1),1e-20))
        sampled_radii[key]=float(radius.min())
        check(abs(sampled_radii[key]/curvature[lanes[key]['kind']]['min_radius_m']-1)<.001,'Authored connector curvature differs from analytic profile')
    check(all(v['max_lateral_g_at_65mph']<=p['maximum_design_lateral_g'] for v in curvature.values()),'Connector exceeds design lateral load bound')
    randomized_choices=set(); random_plans=0
    for seed in range(20):
        plans=generate_plans(n,seed,64,1.)
        check(plans==generate_plans(n,seed,64,1.),'Seed not repeatable')
        for plan in plans['plans']:
            kinds=[edges[k]['kind'] for k in plan['edges']]
            check(kinds.count('exit')==1 and kinds.count('return')==1 and kinds[-1]=='auxiliary','Route fails to return')
            check(sum(k=='collector' for k in kinds)<=8,'Route exceeds collector-lap budget')
            randomized_choices.add(plan['return_opportunity']); random_plans+=1
    check(randomized_choices=={1,2,3,4},'Randomizer does not expose all reentry opportunities')
    check(generate_plans(n,1,32,1.)!=generate_plans(n,2,32,1.),'Different seeds do not vary plans')
    check(all(not p['take_exit'] for p in generate_plans(n,1,32,0.)['plans']),'Zero exit probability ignored')
    examples=json.loads((directory/'route_examples.json').read_text())
    check(examples==generate_plans(n,20261006,24),'Saved route examples stale')
    collider=UsdGeom.Mesh(stage.GetPrimAtPath('/World/Physics/RoadCollider'))
    check(collider.GetPrim().HasAPI(UsdPhysics.CollisionAPI),'Missing road collision')
    check(UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get()=='none','Incorrect collision approximation')
    check(stage.GetPrimAtPath('/World/Ground').HasAPI(UsdPhysics.CollisionAPI),'Missing ground collision')
    pts,meshfaces=faces(collider); incidence=Counter(); direction=Counter(); top=[]
    for f in meshfaces:
        for a,b in zip(f,f[1:]+f[:1]): incidence[tuple(sorted((a,b)))]+=1; direction[(a,b)]+=1
        if np.all(pts[f,2]==0):
            a,b,c=pts[f[:3]]; check(np.cross(b-a,c-a)[2]>0,'Downward road top'); top.append(Polygon(pts[f,:2]))
    check(set(incidence.values())=={2},'Road collider not watertight')
    check(all(direction[(a,b)]==direction[(b,a)] for a,b in direction),'Collider winding inconsistent')
    surface=unary_union(top)
    check(surface.geom_type=='Polygon' and len(surface.interiors)==9,'Road disconnected or unexpected island topology')
    check(abs(sum(t.area for t in top)-surface.area)<.001,'Overlapping collision faces')
    for key,lane in lanes.items(): check(surface.buffer(.001).covers(LineString(lane['points'])),'Lane leaves pavement: '+key)
    check(len(n['spawn_points'])==24,'Spawn count')
    for spawn in n['spawn_points']: check(surface.contains(Point(spawn['point'][:2])),'Spawn off road')
    visual=[]
    for prim in stage.Traverse():
        if str(prim.GetPath()).endswith('/Surface') and prim.IsA(UsdGeom.Mesh):
            a,fs=faces(UsdGeom.Mesh(prim)); visual.extend(Polygon(a[f,:2]) for f in fs)
    rendered=unary_union(visual); mismatch=rendered.symmetric_difference(surface).area
    check(mismatch<.1,'Render/collision discrepancy')
    check(abs(sum(t.area for t in visual)-rendered.area)<.02,'Coplanar visual overlaps')
    validators=UsdValidation.ValidationRegistry().GetOrLoadAllValidators()
    issues=UsdValidation.ValidationContext(validators).Validate(stage)
    check(not issues,'USD validation issues: '+str([e.GetMessage() for e in issues]))
    manifest=json.loads((directory/'manifest.json').read_text())
    for name,expected in manifest['files'].items():
        source=directory/name
        if not source.exists(): source=Path(__file__).with_name(name)
        check(hashlib.sha256(source.read_bytes()).hexdigest()==expected,'Hash mismatch: '+name)
    return dict(passed=True,stage_sha256=manifest['files']['highway_v02.usda'],usd_validator_count=len(validators),prim_count=len(list(stage.Traverse())),relationship_targets=relation_count,lane_count=14,edge_count=28,spawn_count=24,strongly_connected_auxiliary_collector=True,all_routes_continue=True,connector_metrics=curvature,sampled_connector_minimum_radii_m=sampled_radii,maximum_tangent_error_deg=tangent_error,maximum_waypoint_spacing_m=max_step,maximum_usd_float_error_m=float_error,watertight_collision=True,render_collision_difference_m2=mismatch,collision_faces=len(meshfaces),seeded_return_plans_checked=random_plans,return_opportunities=sorted(randomized_choices),main_lap_seconds_at_65mph=math.tau*(p['inner_radius_m']+.5*p['lane_width_m'])/p['target_speed_m_s'],collector_lap_seconds_at_65mph=math.tau*p['collector_radius_m']/p['target_speed_m_s'],scope='Static geometry, route planning and theoretical unbanked lateral acceleration. No vehicle handling or traffic-controller acceptance.')


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--directory',type=Path,default=Path(__file__).resolve().parent)
    args=parser.parse_args(); result=validate(args.directory.resolve())
    (args.directory/'validation_report.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()
