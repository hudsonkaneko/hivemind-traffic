"""Generate v02: circular highway, outer collector, smooth links and route data."""
from __future__ import annotations
import argparse
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import shapely
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from pxr import Gf,Sdf,Usd,UsdGeom,UsdLux,UsdPhysics,UsdShade
from usd_helpers import attr,relation,material,mesh,curve,path_length

WORLD='/World'
ROAD=WORLD+'/RoadNetwork'
NAV=WORLD+'/Navigation'
SIDES={'East':0,'North':90,'West':180,'South':270}


@dataclass(frozen=True)
class Parameters:
    inner_radius_m: float = 498.15
    lane_width_m: float = 3.7
    collector_radius_m: float = 535.0
    connector_span_deg: float = 42.0
    circle_samples: int = 3600
    connector_samples: int = 640
    target_speed_m_s: float = 29.0576
    maximum_design_lateral_g: float = 0.25
    road_thickness_m: float = 0.3
    ground_margin_m: float = 40.0

    @property
    def auxiliary_radius(self):
        return self.inner_radius_m+4.5*self.lane_width_m

    @property
    def outer_radius(self):
        return self.inner_radius_m+5*self.lane_width_m

    def validate(self):
        if not (self.inner_radius_m > 100 and self.lane_width_m > 0 and self.road_thickness_m > 0):
            raise ValueError('Positive road dimensions and radius >100m required')
        if not (self.collector_radius_m-self.outer_radius > 4*self.lane_width_m):
            raise ValueError('Collector needs a separated paved band')
        if not (10 < self.connector_span_deg <=42):
            raise ValueError('Connectors must stay within non-crossing sectors')
        if self.circle_samples < 3600 or self.circle_samples%360:
            raise ValueError('Use >=3600 circle samples, multiple of360')
        if self.connector_samples < 640:
            raise ValueError('Use >=640 connector segments')


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')


def circle(radius,p):
    return [(radius*math.cos(i*math.tau/p.circle_samples),radius*math.sin(i*math.tau/p.circle_samples),0) for i in range(p.circle_samples)]


def disk(radius,p):
    return Polygon([(x,y) for x,y,_ in circle(radius,p)])


def connector(start_deg,outbound,p):
    t=np.linspace(0,1,p.connector_samples+1)
    smooth=10*t**3-15*t**4+6*t**5
    first,last=(p.auxiliary_radius,p.collector_radius_m) if outbound else (p.collector_radius_m,p.auxiliary_radius)
    r=first+(last-first)*smooth
    a=np.deg2rad(start_deg+p.connector_span_deg*t)
    return np.column_stack((r*np.cos(a),r*np.sin(a),t*0)).tolist()


def build_network(p):
    lanes,edges,nodes={},{},{}
    def lane(key,kind,points,closed,radius=None):
        lanes[key]=dict(id=key,kind=kind,points=points,closed=closed,width_m=p.lane_width_m,
            usd_path=f'{NAV}/Lanes/{key}',speed_hint_m_s=p.target_speed_m_s,radius_m=radius,
            left_neighbor=None,right_neighbor=None)
    for i in range(5):
        key=f'MainLane{i+1}' if i<4 else 'AuxiliaryLane'
        lane(key,'main' if i<4 else 'auxiliary',circle(p.inner_radius_m+(i+.5)*p.lane_width_m,p),True,p.inner_radius_m+(i+.5)*p.lane_width_m)
        lanes[key]['left_neighbor']=f'MainLane{i}' if i else None
        lanes[key]['right_neighbor']=(f'MainLane{i+2}' if i<3 else 'AuxiliaryLane') if i<4 else None
    lane('CollectorLane','collector',circle(p.collector_radius_m,p),True,p.collector_radius_m)
    events={'AuxiliaryLane':[],'CollectorLane':[]}
    def node(name,road,angle,kind):
        sample=round((angle%360)/360*p.circle_samples)%p.circle_samples
        if abs(sample/p.circle_samples*360-angle%360)>1e-8:
            raise ValueError('Junction angles must align with ring samples')
        nodes[name]=dict(id=name,lane=road,sample_index=sample,point=lanes[road]['points'][sample],kind=kind)
        events[road].append((sample,name))
    for side,angle in SIDES.items():
        node(side+'Diverge','AuxiliaryLane',angle,'diverge')
        node(side+'CollectorMerge','CollectorLane',angle+p.connector_span_deg,'merge')
        node(side+'CollectorDiverge','CollectorLane',angle+45,'diverge')
        node(side+'Merge','AuxiliaryLane',angle+45+p.connector_span_deg,'merge')
        for direction,outbound,start,start_node,end_node in [
            ('Exit',True,angle,side+'Diverge',side+'CollectorMerge'),
            ('Return',False,angle+45,side+'CollectorDiverge',side+'Merge')]:
            key=side+direction
            points=connector(start,outbound,p)
            points[0]=nodes[start_node]['point']
            points[-1]=nodes[end_node]['point']
            lane(key,'exit' if outbound else 'return',points,False)
            edges[key]=dict(id=key,lane=key,kind=lanes[key]['kind'],closed=False,points=points,start_node=start_node,end_node=end_node)
    for lane_key,evts in events.items():
        evts.sort()
        pts=lanes[lane_key]['points']
        for i,(start,name) in enumerate(evts):
            stop,end=evts[(i+1)%len(evts)]
            key=f'{lane_key}_{name}_to_{end}'
            points=[pts[(start+j)%len(pts)] for j in range((stop-start)%len(pts)+1)]
            edges[key]=dict(id=key,lane=lane_key,kind=lanes[lane_key]['kind'],closed=False,points=points,start_node=name,end_node=end)
    for i in range(1,5):
        key=f'MainLane{i}'
        edges[key]=dict(id=key,lane=key,kind='main',closed=True,points=lanes[key]['points'],start_node=None,end_node=None)
    for key,e in edges.items():
        e['usd_path']=f'{NAV}/Edges/{key}'
        e['successors']=[key] if e['closed'] else [k for k,v in edges.items() if v['start_node']==e['end_node']]
        e['length_m']=path_length(e['points'],e['closed'])
    return dict(schema_version=2,version='_v02',parameters=asdict(p),units='meters',up_axis='Z',east_axis='+X',north_axis='+Y',forward_axis='+X',traffic_direction='counterclockwise',point_reference='surface centerline; apply vehicle-specific height offset',lanes=lanes,edges=edges,nodes=nodes)


def shapes(n,p):
    main=disk(p.inner_radius_m+4*p.lane_width_m,p).difference(disk(p.inner_radius_m,p))
    aux=disk(p.outer_radius,p).difference(disk(p.inner_radius_m+4*p.lane_width_m,p))
    collector=disk(p.collector_radius_m+p.lane_width_m/2,p).difference(disk(p.collector_radius_m-p.lane_width_m/2,p))
    links={k:LineString(v['points']).buffer(p.lane_width_m/2,cap_style='flat',join_style='round') for k,v in n['lanes'].items() if v['kind'] in ['exit','return']}
    pavement=unary_union([main,aux,collector,*links.values()])
    if pavement.geom_type!='Polygon' or not pavement.is_valid:
        raise ValueError('Pavement must be a connected valid polygon')
    return main,aux,collector,links,pavement


def author(output,n,p):
    stage=Usd.Stage.CreateNew(str(output/'highway_v02.usda'))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,WORLD).GetPrim())
    UsdGeom.SetStageUpAxis(stage,'Z'); UsdGeom.SetStageMetersPerUnit(stage,1)
    UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    stage.SetTimeCodesPerSecond(60)
    stage.GetRootLayer().customLayerData=dict(version='_v02',generator='generate_highway.py',navigation='navigation.json',motionAuthority='external controller; static environment',routePolicy='route_policy.py')
    for path in [ROAD,ROAD+'/CircularHighway',ROAD+'/AuxiliaryLane',ROAD+'/OuterCollector',ROAD+'/Ramps',ROAD+'/Markings',NAV,NAV+'/Lanes',NAV+'/Edges',WORLD+'/SpawnPoints',WORLD+'/DivergeZones',WORLD+'/MergeZones',WORLD+'/Debug',WORLD+'/Physics',WORLD+'/Looks']:
        UsdGeom.Xform.Define(stage,path)
    scene=UsdPhysics.Scene.Define(stage,WORLD+'/Physics/Scene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); scene.CreateGravityMagnitudeAttr(9.81)
    asphalt=material(stage,'Asphalt',(.055,.065,.075),True)
    auxmat=material(stage,'AuxiliaryAsphalt',(.085,.105,.12))
    earth=material(stage,'Ground',(.12,.18,.11),True)
    white=material(stage,'WhitePaint',(.85,.87,.86)); yellow=material(stage,'YellowPaint',(.95,.65,.12))
    main,aux,col,links,pavement=shapes(n,p)
    for name,shape,mat,lane_names in [('CircularHighway',main,asphalt,[f'MainLane{i}' for i in range(1,5)]),('AuxiliaryLane',aux,auxmat,['AuxiliaryLane']),('OuterCollector',col,asphalt,['CollectorLane'])]:
        mesh(stage,f'{ROAD}/{name}/Surface',shape,0,mat)
        relation(stage.GetPrimAtPath(f'{ROAD}/{name}'),'road:lanes',[n['lanes'][key]['usd_path'] for key in lane_names])
    ring_area=unary_union([main,aux,col])
    for side in SIDES:
        parent=UsdGeom.Xform.Define(stage,ROAD+'/Ramps/'+side).GetPrim()
        for direction in ['Exit','Return']:
            key=side+direction
            path=ROAD+'/Ramps/'+side+'/'+direction
            prim=UsdGeom.Xform.Define(stage,path).GetPrim()
            mesh(stage,path+'/Surface',links[key].difference(ring_area),0,asphalt)
            relation(prim,'road:navigation',[n['lanes'][key]['usd_path']])
            relation(prim,'road:from',[ROAD+('/AuxiliaryLane' if direction=='Exit' else '/OuterCollector')])
            relation(prim,'road:to',[ROAD+('/OuterCollector' if direction=='Exit' else '/AuxiliaryLane')])
    collision=mesh(stage,WORLD+'/Physics/RoadCollider',pavement,0,None,p.road_thickness_m,True)
    UsdShade.MaterialBindingAPI.Apply(collision.GetPrim()).Bind(asphalt,materialPurpose='physics')
    relation(stage.GetPrimAtPath(ROAD),'road:collision',[collision.GetPath()])
    attr(stage.GetPrimAtPath(ROAD),'road:direction',Sdf.ValueTypeNames.Token,'counterclockwise')
    attr(stage.GetPrimAtPath(ROAD),'road:mainLaneCount',Sdf.ValueTypeNames.Int,4)
    attr(stage.GetPrimAtPath(ROAD),'road:designSpeed',Sdf.ValueTypeNames.Float,p.target_speed_m_s)
    ground=UsdGeom.Cube.Define(stage,WORLD+'/Ground'); ground.CreateSizeAttr(2)
    ground.AddTranslateOp().Set(Gf.Vec3d(0,0,-p.road_thickness_m-.5))
    extent=p.collector_radius_m+p.ground_margin_m
    ground.AddScaleOp().Set(Gf.Vec3f(extent,extent,.5))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim()).CreateSimulationOwnerRel().SetTargets([scene.GetPath()])
    binding=UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()); binding.Bind(earth); binding.Bind(earth,materialPurpose='physics')
    borders=[LineString(r.coords).buffer(.075) for r in [pavement.exterior,*pavement.interiors]]
    mesh(stage,ROAD+'/Markings/Edges',unary_union(borders),.01,white)
    innerring=LineString(circle(p.inner_radius_m+.15,p)+[(p.inner_radius_m+.15,0,0)])
    mesh(stage,ROAD+'/Markings/InnerYellow',innerring.buffer(.08),.012,yellow)
    dashes=[]
    for j in range(1,5):
        r=p.inner_radius_m+j*p.lane_width_m
        for theta in np.linspace(0,math.tau,round(math.tau*r/12),endpoint=False):
            dashes.append(LineString([(r*math.cos(theta+k/r),r*math.sin(theta+k/r)) for k in np.linspace(0,4,6)]).buffer(.075,cap_style='flat'))
    mesh(stage,ROAD+'/Markings/LaneDividers',unary_union(dashes),.013,white)
    arrows=[]
    for key,lane in n['lanes'].items():
        pts=lane['points']
        indexes=range(100,len(pts)-1,300) if lane['closed'] else [160,320,480]
        for i in indexes:
            a,b=np.array(pts[i]),np.array(pts[i+1]); forward=(b-a)/np.linalg.norm(b-a)
            left=np.array([-forward[1],forward[0],0])
            coords=[(-1.8,-.14),(.3,-.14),(.3,-.55),(1.8,0),(.3,.55),(.3,.14),(-1.8,.14)]
            arrows.append(Polygon([(a+u*forward+v*left)[:2] for u,v in coords]))
        prim=curve(stage,lane['usd_path'],pts,lane['closed'])
        attr(prim,'lane:kind',Sdf.ValueTypeNames.Token,lane['kind'])
        attr(prim,'lane:width',Sdf.ValueTypeNames.Float,p.lane_width_m)
        attr(prim,'lane:speedHint',Sdf.ValueTypeNames.Float,p.target_speed_m_s)
        for side in ['left','right']:
            if lane[side+'_neighbor']:
                relation(prim,'lane:'+side+'Neighbor',[n['lanes'][lane[side+'_neighbor']]['usd_path']])
        relation(prim,'lane:edges',[e['usd_path'] for e in n['edges'].values() if e['lane']==key])
    mesh(stage,ROAD+'/Markings/Arrows',unary_union(arrows),.015,white)
    for key,e in n['edges'].items():
        prim=curve(stage,e['usd_path'],e['points'],e['closed'])
        UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible')
        relation(prim,'navigation:successors',[n['edges'][k]['usd_path'] for k in e['successors']])
        relation(prim,'navigation:lane',[n['lanes'][e['lane']]['usd_path']])
    for name,node in n['nodes'].items():
        path=WORLD+('/DivergeZones/' if node['kind']=='diverge' else '/MergeZones/')+name
        prim=UsdGeom.Xform.Define(stage,path).GetPrim()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*node['point']))
        attr(prim,'zone:sampleIndex',Sdf.ValueTypeNames.Int,node['sample_index'])
        attr(prim,'zone:policy',Sdf.ValueTypeNames.String,'Controller must yield; route selection does not grant merge priority')
        relation(prim,'zone:lane',[n['lanes'][node['lane']]['usd_path']])
    n['spawn_points']=[]
    for name,lane in list(n['lanes'].items())[:6]:
        for angle in [20,110,200,290]:
            i=round(angle/360*p.circle_samples); point=lane['points'][i]
            path=f'{WORLD}/SpawnPoints/{name}_{angle}'
            prim=UsdGeom.Xform.Define(stage,path).GetPrim()
            transform=UsdGeom.Xformable(prim); transform.AddTranslateOp().Set(Gf.Vec3d(*point)); transform.AddRotateZOp().Set(angle+90)
            relation(prim,'spawn:lane',[lane['usd_path']])
            attr(prim,'spawn:heightOffsetRequired',Sdf.ValueTypeNames.Bool,True)
            n['spawn_points'].append(dict(usd_path=path,lane=name,point=point,yaw_deg=angle+90))
    UsdLux.DistantLight.Define(stage,WORLD+'/Lighting/Sun').CreateIntensityAttr(1700)
    UsdLux.DomeLight.Define(stage,WORLD+'/Lighting/Sky').CreateIntensityAttr(250)
    camera=UsdGeom.Camera.Define(stage,WORLD+'/Cameras/Top')
    camera.CreateProjectionAttr('orthographic'); camera.CreateHorizontalApertureAttr(extent*20); camera.CreateVerticalApertureAttr(extent*20)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.1,4000)); camera.AddTranslateOp().Set(Gf.Vec3d(0,0,1800))
    stage.GetRootLayer().Save()
    return pavement


def preview(output,n,pavement):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Patch
    fig,ax=plt.subplots(figsize=(11,11),facecolor='#edf2ed'); ax.set_facecolor('#edf2ed')
    ax.add_patch(Patch(pavement.exterior.coords,color='#364349'))
    for hole in pavement.interiors: ax.add_patch(Patch(hole.coords,color='#edf2ed'))
    for name,lane in n['lanes'].items():
        if lane['kind'] in ['exit','return']:
            a=np.array(lane['points']); color='#d88724' if lane['kind']=='exit' else '#228860'
            ax.plot(a[:,0],a[:,1],color=color,lw=2)
            i=len(a)//2; d=a[i+10]-a[i]; d=d/np.linalg.norm(d)*15
            ax.arrow(a[i,0],a[i,1],d[0],d[1],head_width=7,color=color,length_includes_head=True)
    ax.text(0,65,'OUTER COLLECTOR',ha='center',fontsize=19,fontweight='bold',color='#254238')
    ax.text(0,25,'Four main lanes + continuous auxiliary',ha='center',fontsize=12)
    ax.text(0,-10,'Orange: exit    Green: return',ha='center',fontsize=11)
    ax.text(0,-45,'Seeded re-entry choice: first to fourth opportunity',ha='center',fontsize=10)
    ax.text(0,-80,'65 mph target · vehicle handling unvalidated',ha='center',fontsize=10)
    ax.set_aspect('equal'); ax.autoscale(); ax.set_xlabel('East / X (m)'); ax.set_ylabel('North / Y (m)')
    ax.set_title('_v02 | Smooth collector and return network',loc='left',pad=15)
    fig.tight_layout(); fig.savefig(output/'overview.png',dpi=180); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--parameters',type=Path)
    args=parser.parse_args(); p=Parameters(**(json.loads(args.parameters.read_text()) if args.parameters else {})); p.validate()
    out=args.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    n=build_network(p); pavement=author(out,n,p); preview(out,n,pavement)
    (out/'navigation.json').write_text(json.dumps(n,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8',newline='\n')
    write_json(out/'parameters.json',asdict(p))
    from route_policy import generate_plans
    write_json(out/'route_examples.json',generate_plans(n,seed=20261006,vehicles=24))
    source=Path(__file__).resolve().parent
    names=['highway_v02.usda','navigation.json','parameters.json','route_examples.json','generate_highway.py','route_policy.py','usd_helpers.py']
    write_json(out/'manifest.json',dict(version='_v02',usd_version='.'.join(map(str,Usd.GetVersion())),shapely_version=shapely.__version__,files={name:hashlib.sha256(((out/name) if (out/name).exists() else source/name).read_bytes()).hexdigest() for name in names}))
    print(json.dumps(dict(stage=str(out/'highway_v02.usda'),lanes=len(n['lanes']),edges=len(n['edges']),area_m2=pavement.area)))


if __name__=='__main__': main()
