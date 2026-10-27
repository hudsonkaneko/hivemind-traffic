"""Image-constrained development ribbons and one seam-free road support mesh.

The annotations are approximate NAIP interpretation, not survey control. Keep
occlusion, smoothing and blend diagnostics alongside the generated geometry.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import LinearNDInterpolator
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree

from scripts.prepare_i110_geometry import sample_path, fill_short_gaps


def fit_cross_sections(xy, ground, widths):
    """Fit one robust local road plane per station, not walls at shoulder edges."""
    s,p,n=sample_path(xy); w=np.interp(s,widths[:,0],widths[:,1])
    tree=cKDTree(ground[:,:2]); coefficients=[]; counts=[]
    for point, normal, width in zip(p,n,w):
        ids=tree.query_ball_point(point,min(10.,width/2))
        cloud=ground[ids]; delta=cloud[:,:2]-point
        across=delta@normal; along=delta@np.array([normal[1],-normal[0]])
        keep=(abs(along)<3)&(abs(across)<width/2-.6)
        cloud=cloud[keep];across=across[keep];along=along[keep]
        if len(cloud)<25:
            coefficients.append([np.nan,np.nan]);counts.append(0);continue
        keep=abs(cloud[:,2]-np.median(cloud[:,2]))<.75
        design=np.column_stack([across,along,np.ones(len(cloud))])
        coef=np.linalg.lstsq(design[keep],cloud[keep,2],rcond=None)[0]
        for _ in range(3):
            residual=cloud[:,2]-design@coef
            keep=abs(residual-np.median(residual))<.12
            coef=np.linalg.lstsq(design[keep],cloud[keep,2],rcond=None)[0]
        if keep.sum()<25 or np.linalg.norm(coef[:2])>.15:
            coefficients.append([np.nan,np.nan]);counts.append(0)
        else:coefficients.append([coef[2],coef[0]]);counts.append(int(keep.sum()))
    coef=np.array(coefficients);filled=[]
    for j in range(2):coef[:,j],mask=fill_short_gaps(s,coef[:,j]);filled.append(mask)
    lateral=np.linspace(-.5,.5,7)
    positions=p[:,None,:]+n[:,None,:]*w[:,None,None]*lateral[None,:,None]
    z=coef[:,0,None]+coef[:,1,None]*w[:,None]*lateral
    vertices=np.dstack([positions,z]).reshape(-1,3);faces=[]
    for i in range(len(p)-1):
        for j in range(6):
            a=i*7+j;b=a+7;faces.extend([[a,b,b+1],[a,b+1,a+1]])
    faces=np.array(faces);v=vertices[faces];flip=np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0])[:,2]<0
    faces[flip]=faces[flip][:,[0,2,1]]
    return dict(points=vertices.tolist(),faces=faces.tolist(),station_m=s.tolist(),
                center_xyz=np.column_stack([p,coef[:,0]]).tolist(),widths_m=w.tolist(),
                normal_xy=n.tolist(),support_counts=np.repeat(np.array(counts)[:,None],7,axis=1).tolist(),
                interpolated=np.repeat(np.any(filled,axis=0)[:,None],7,axis=1).tolist())


def bounds_at(trace, y):
    a = np.asarray(trace)[::-1]
    return np.interp(y, a[:, 0], a[:, 1]), np.interp(y, a[:, 0], a[:, 2])


def union_support(meshes, traces, spacing=.5):
    """Single-valued, shared-vertex support. Never fill the ramp/mainline gore."""
    x = np.arange(-42, 12 + spacing / 2, spacing)
    y = np.arange(-815, 17 + spacing / 2, spacing)
    xx, yy = np.meshgrid(x, y)
    xy = np.column_stack([xx.ravel(), yy.ravel()])
    heights, weights, masks = [], [], []
    for name, mesh in meshes.items():
        trace = traces[name]
        west, east = bounds_at(trace, xy[:, 1])
        v = np.asarray(mesh['points'])
        z = LinearNDInterpolator(v[:, :2], v[:, 2])(xy)
        mask = ((xy[:, 1] <= trace[0][0]) & (xy[:, 1] >= trace[-1][0]) &
                (xy[:, 0] >= west) & (xy[:, 0] <= east) & np.isfinite(z))
        # Equal shared contact surface in overlaps; taper weight to zero at edges.
        w = np.maximum(.01, np.minimum(xy[:, 0]-west, east-xy[:, 0])) * mask
        heights.append(z); weights.append(w); masks.append(mask)
    masks = np.asarray(masks); heights = np.asarray(heights); weights = np.asarray(weights)
    valid = masks.any(axis=0)
    overlap = masks.sum(axis=0) > 1
    differences = []
    for i in range(len(masks)):
        for j in range(i):
            use = masks[i] & masks[j]
            differences.extend(abs(heights[i, use]-heights[j, use]).tolist())
    z = np.nansum(np.where(masks, heights, 0)*weights, axis=0) / np.maximum(weights.sum(axis=0), 1e-12)
    ids = np.arange(len(xy)).reshape(xx.shape)
    a, b, c, d = ids[:-1, :-1].ravel(), ids[:-1, 1:].ravel(), ids[1:, 1:].ravel(), ids[1:, :-1].ravel()
    faces = np.concatenate([np.column_stack([a,b,c]), np.column_stack([a,c,d])])
    faces = faces[valid[faces].all(axis=1)]
    # Reject triangles spanning disconnected ribbons even if their vertices exist.
    keep = np.zeros(len(faces), bool)
    for mask in masks:
        keep |= mask[faces].all(axis=1)
    faces = faces[keep]
    used = np.unique(faces); remap = np.full(len(xy), -1); remap[used] = np.arange(len(used))
    v = np.column_stack([xy[used],z[used]])
    return dict(points=v.tolist(),faces=remap[faces].tolist()), dict(
        grid_m=spacing, overlap_grid_points=int(overlap.sum()),
        preblend_max_height_difference_m=max(differences,default=0),
        preblend_p95_height_difference_m=float(np.percentile(differences,95)) if differences else 0,
        seam_policy='Shared XY vertices, one Z and one triangle mesh; no overlapping road colliders',
        edge_policy='Conservative 0.5m grid inset, no bridging disconnected ribbons')


def refine(previous, points, annotations, output):
    if output.exists():
        raise FileExistsError(output)
    data=json.loads(previous.read_text()); trace=json.loads(annotations.read_text())
    cloud=np.load(points); ground=cloud[cloud[:,3]==2,:3]
    # Image-visible overpasses can include ground-class deck approaches. Never
    # fit those elevated returns as the obscured freeway/ramp pavement below.
    for low,high in trace['imagery_occlusion_y_ranges']:
        ground=ground[~((ground[:,1]>=low-2)&(ground[:,1]<=high+2))]
    roads={}; diagnostics={}
    for name, edges in trace['roads'].items():
        a=np.array(edges); xy=np.column_stack([a[:,1:3].mean(axis=1),a[:,0]])
        chain=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        print('Fitting',name,flush=True)
        m=fit_cross_sections(xy,ground,np.column_stack([chain,a[:,2]-a[:,1]]))
        v=np.array(m['points']).reshape(-1,7,3); original=v[:,:,2].copy()
        # Remove lidar sampling chatter, retain gradual grades and crossfall.
        v[:,:,2]=savgol_filter(v[:,:,2],9,2,axis=0)-data['z_offset_m']
        m['points']=v.reshape(-1,3).tolist();m['center_xyz']=v[:,3,:].tolist()
        m['horizontal_status']='NAIP interpreted pavement envelope; semantic lane reconstruction with explicit uncertainty'
        m['right_lane_offsets_m']=(-np.array(m['widths_m'])/2+4.75).tolist()
        if name=='Southbound':
            s=np.array(m['station_m']); fractions=np.array(trace['lane_model']['aux_lane_fraction_by_s'])
            aux=np.interp(s,fractions[:,0],fractions[:,1])
            # East-side through lanes stay continuous; disappearing auxiliary lane joins them.
            left=np.array(m['widths_m'])/2-1.6
            m['right_lane_offsets_m']=(left-3.7*(3.5+aux)).tolist()
            m['lane_boundaries_m']=[(left-3.7*k).tolist() for k in range(5)]
            m['aux_outer_boundary_m']=(left-3.7*(4+aux)).tolist()
            m['aux_fraction']=aux.tolist()
        roads[name]=m
        diagnostics[name]=dict(max_smoothing_adjustment_m=float(np.max(abs(v[:,:,2]+data['z_offset_m']-original))),
                               interpolated_vertices=int(np.sum(m['interpolated'])),length_m=m['station_m'][-1])
    support,join=union_support(roads,trace['roads'])
    data['meshes'].update(roads); data['collision_mesh']=support
    data['schema_version']=2
    data['annotation_sha256']=hashlib.sha256(annotations.read_bytes()).hexdigest()
    data['geometry_accuracy']='image interpreted; independent positional accuracy and present-day epoch unqualified'
    data['refinement_diagnostics']=dict(roads=diagnostics,joins=join)
    data['limitations']=trace['limitations']+[
        'Lane divider spacing and paint reconstruction approximate; shoulder is not a traffic lane',
        '0.5m collision-grid boundary is conservatively inset; not tire-scale surveyed pavement roughness',
        'Lidar elevations smoothed over 9 cross-sections (~24m), occlusions interpolated',
        'Vehicle dynamics validity is separate from road contact tests']
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(data,separators=(',',':')))
    (output.parent/'refinement-diagnostics.json').write_text(json.dumps(data['refinement_diagnostics'],indent=2))
    print(json.dumps(data['refinement_diagnostics'],indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('previous','points','annotations','output'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if not args.output.resolve().is_relative_to(Path(__file__).resolve().parents[1]):p.error('Output outside checkout')
    refine(**vars(args))
