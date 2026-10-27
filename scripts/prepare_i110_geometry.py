"""Fit a source-aligned road surface, preserving observed support and missing spans.

Geo environment only. Nominal horizontal widths are explicit assumptions, not HD lanes.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree, Delaunay
from pyproj import Transformer

MAIN = ['173347946','609022189','399276942','399276944','399276945',
        '1119770688','1119770689','148252622']
RAMP_IN = ['148761222','399277053','399277061']
RAMP_OUT = ['148385755','399277054']


def sample_path(xy, spacing=3.):
    xy = np.asarray(xy, dtype=float)
    xy = xy[np.r_[True, np.linalg.norm(np.diff(xy, axis=0),axis=1)>.001]]
    chain = np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    s = np.linspace(0,chain[-1],int(np.ceil(chain[-1]/spacing))+1)
    p = np.column_stack([np.interp(s,chain,xy[:,i]) for i in range(2)])
    tangent = np.gradient(p,axis=0);tangent /= np.linalg.norm(tangent,axis=1)[:,None]
    normal = np.column_stack([-tangent[:,1],tangent[:,0]])
    return s,p,normal


def fill_short_gaps(s, values, max_gap=60.):
    """No unbounded terrain drape or endpoint extrapolation."""
    v = np.array(values, dtype=float); known = np.isfinite(v)
    if known.sum()<2 or not known[0] or not known[-1]:
        raise ValueError('Road endpoint lacks source support')
    for i in np.flatnonzero(~known):
        left = np.flatnonzero(known & (s<s[i]))[-1]
        right = np.flatnonzero(known & (s>s[i]))[0]
        if s[right]-s[left] > max_gap:
            raise ValueError(f'Unobserved road span {s[left]:.2f}--{s[right]:.2f} exceeds {max_gap} m')
    v[~known] = np.interp(s[~known],s[known],v[known])
    return v, ~known


def ribbon(xy, ground, widths):
    s,p,n = sample_path(xy)
    width = np.interp(s,widths[:,0],widths[:,1])
    lateral = np.linspace(-.5,.5,7)
    samples = p[:,None,:]+n[:,None,:]*width[:,None,None]*lateral[None,:,None]
    tree = cKDTree(ground[:,:2]); z = np.full(samples.shape[:2],np.nan)
    supports = np.zeros(z.shape,dtype=int)
    center = []
    for point in p:
        ids = tree.query_ball_point(point,1.2)
        center.append(float(np.median(ground[ids,2])) if len(ids)>=8 else np.nan)
    center,_ = fill_short_gaps(s,center)
    for i,row in enumerate(samples):
        for j,point in enumerate(row):
            ids = tree.query_ball_point(point,1.6)
            if len(ids)<8:continue
            cloud = ground[ids]
            # Avoid fitting a wall/adjacent raised road into the freeway pavement.
            cloud = cloud[abs(cloud[:,2]-center[i])<.75]
            if len(cloud)<8:continue
            delta = cloud[:,:2]-point
            # Robust plane; exclude non-ground objects by class before fitting.
            design = np.column_stack([delta,np.ones(len(delta))])
            coef = np.linalg.lstsq(design,cloud[:,2],rcond=None)[0]
            residual = cloud[:,2]-design@coef
            good = abs(residual-np.median(residual))<.15
            if good.sum()<8:continue
            coef = np.linalg.lstsq(design[good],cloud[good,2],rcond=None)[0]
            if np.linalg.norm(coef[:2])>.15:continue
            z[i,j] = coef[2];supports[i,j] = int(good.sum())
    filled = np.zeros(z.shape,dtype=bool)
    for j in range(z.shape[1]):
        try:z[:,j],filled[:,j] = fill_short_gaps(s,z[:,j])
        except ValueError as e:raise ValueError(f'Cross-section column {j}: {e}') from e
    vertices = np.dstack([samples,z]).reshape(-1,3)
    faces=[];cols=z.shape[1]
    for i in range(len(p)-1):
        for j in range(cols-1):
            a=i*cols+j;b=a+cols
            # Normals face up for travel-direction then left-to-right ordering.
            faces.extend([[a,b,b+1],[a,b+1,a+1]])
    faces=np.array(faces)
    tri=vertices[faces]; flip=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])[:,2]<0
    faces[flip]=faces[flip][:,[0,2,1]]
    return dict(points=vertices.tolist(),faces=faces.tolist(),
                station_m=s.tolist(),center_xyz=np.column_stack([p,z[:,3]]).tolist(),
                widths_m=width.tolist(),support_counts=supports.tolist(),
                interpolated=filled.tolist(),normal_xy=n.tolist())


def measured_context(points, cls, spacing=1.5):
    q=points[points[:,3]==cls,:3]
    # Deterministic median within each XY cell; class 17 remains separate from ground.
    cells=np.floor(q[:,:2]/spacing).astype(int)
    keys,inv=np.unique(cells,axis=0,return_inverse=True)
    order=np.argsort(inv);starts=np.r_[0,np.flatnonzero(np.diff(inv[order]))+1,len(order)]
    v=np.array([np.median(q[order[a:b]],axis=0) for a,b in zip(starts[:-1],starts[1:])])
    faces=Delaunay(v[:,:2]).simplices
    tri=v[faces];edges=np.stack([tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]],axis=1)
    good=(np.linalg.norm(edges[:,:,:2],axis=2).max(axis=1)<spacing*2.6)&(abs(edges[:,:,2]).max(axis=1)<1.0)
    faces=faces[good];tri=v[faces];flip=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])[:,2]<0
    faces[flip]=faces[flip][:,[0,2,1]]
    return dict(points=v.tolist(),faces=faces.tolist(),classification=cls,cell_m=spacing)


def prepare(source, roads, output):
    points=np.load(source/'points.npy')
    meta=json.loads((source/'manifest.json').read_text())
    road_data=json.loads(roads.read_text())['i110'];ways={w['id']:w for w in road_data}
    transform=Transformer.from_crs(4326,6340,always_xy=True)
    origin=np.array(meta['origin_6340']);ground=points[points[:,3]==2,:3]
    meshes={}
    for name,ids in [('Southbound',MAIN),('MlkEntrance',RAMP_IN),('VernonExit',RAMP_OUT)]:
        xy=[]
        for key in ids:
            w=ways[key]
            for coordinate in w['xy']:
                p=np.array(transform.transform(*coordinate))-origin
                if -890<=p[1]<=30:xy.append(p)
        s,p,n=sample_path(xy)
        if name=='Southbound':
            # Preliminary road envelope; surveyed markings/edge taper are still needed.
            widths=np.array([[0,14.4],[35,14.4],[60,18.0],[610,18.0],[785,14.4],[s[-1],14.4]])
        else:widths=np.array([[0,7.2 if name=='MlkEntrance' else 4.0],[s[-1],4.0]])
        print('Fitting',name,'length',s[-1],flush=True)
        mesh=ribbon(xy,ground,widths);mesh.update(osm_way_ids=ids,
              horizontal_status='OSM alignment + provisional nominal lane envelope, not surveyed boundaries')
        meshes[name]=mesh
    meshes['Terrain']=measured_context(points,2,2.)
    meshes['BridgeDecks']=measured_context(points,17,1.)
    # Preserve measured height spread, shift source Z by a declared offset only.
    z_origin=44.0
    for m in meshes.values():
        a=np.array(m['points']);a[:,2]-=z_origin;m['points']=a.round(5).tolist()
        if 'center_xyz' in m:
            a=np.array(m['center_xyz']);a[:,2]-=z_origin;m['center_xyz']=a.round(5).tolist()
    result=dict(schema_version=1,origin_6340=meta['origin_6340'],origin_lonlat=meta['origin_lonlat'],
                z_offset_m=z_origin,units='metres',up_axis='Z',horizontal_crs='EPSG:6340',
                axis='UTM grid east +X, grid north +Y; headings use transformed route tangents',
                source_points_sha256=meta['points_sha256'],osm_sha256=hashlib.sha256(roads.read_bytes()).hexdigest(),
                meshes=meshes,geometry_accuracy='unqualified',
                limitations=['Nominal lane widths/edges and taper require imagery/as-built validation',
                             'Short occluded pavement spans interpolated, separately flagged',
                             'Project source epoch 2023--24, present-day road changes unqualified',
                             'Vertical datum not transformed; source offset only',
                             'Context terrain/deck meshes are display-only, not collision surfaces'])
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,separators=(',',':')))
    summary={name:dict(vertices=len(m['points']),triangles=len(m['faces']),
                interpolated_vertices=int(np.sum(m.get('interpolated',[])))) for name,m in meshes.items()}
    (output.parent/'geometry-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','roads','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    if not args.output.resolve().is_relative_to(Path(__file__).resolve().parents[1]):parser.error('Output must remain in checkout')
    prepare(args.source,args.roads,args.output)
