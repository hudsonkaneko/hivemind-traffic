"""Offline sparse occupancy from saved measured LiDAR; no simulator or training.

Geometry inference accepts no semantic oracle, rendered depth or scene geometry.
A dominant approximately horizontal plane is estimated from timely WORLD points.
Occupancy is sparse evidence, not a dense image mask or a free-space guarantee.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from traffic.semantic_lidar_fusion import fuse_semantic_lidar, FUTURE_TOLERANCE_S

DEFAULTS = dict(max_time_delta_s=.05, occlusion_cell_px=2, occlusion_tolerance_m=.15,
    boundary_margin_px=2, ground_distance_m=.08, minimum_height_m=.15,
    maximum_height_m=3., max_range_m=60., plane_trials=128, plane_sample_limit=4096,
    plane_seed=20261031, minimum_plane_support=.30, maximum_slope_degrees=15.)


def load_saved_frame(case_dir, stem):
    """Read only XYZ/flags/times and sensor calibration, never oracle arrays."""
    root=Path(case_dir).resolve()
    if not isinstance(stem,str) or not stem.isdigit(): raise ValueError('Numeric frame stem required')
    metadata=json.loads((root/(stem+'-metadata.json')).read_text(encoding='utf-8'))
    if metadata.get('lidar_frame')!='WORLD NONCOMPENSATED': raise ValueError('WORLD frame required')
    with np.load(root/(stem+'-lidar.npz'),allow_pickle=False) as data:
        frame={k:data[k].copy() for k in ('xyz_world','flags','point_times_s')}
    frame.update({k:metadata[k] for k in ('camera_to_world','calibration','image_time_s')})
    return frame


def estimate_ground(points, settings=None):
    """Deterministic bounded RANSAC; Z-up is a sensor-frame convention, not GT."""
    cfg=dict(DEFAULTS,**(settings or {})); xyz=np.asarray(points,dtype=np.float64)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all(): raise ValueError('Finite Nx3 points required')
    if len(xyz)<30: return dict(valid=False,reason='insufficient_timely_points')
    rng=np.random.default_rng(cfg['plane_seed'])
    sample=xyz[rng.choice(len(xyz),min(len(xyz),cfg['plane_sample_limit']),replace=False)]
    lower=sample[sample[:,2]<=np.quantile(sample[:,2],.6)]
    if len(lower)<3: return dict(valid=False,reason='insufficient_ground_candidates')
    best=None; best_count=0; min_normal=np.cos(np.deg2rad(cfg['maximum_slope_degrees']))
    for _ in range(cfg['plane_trials']):
        a,b,c=lower[rng.choice(len(lower),3,replace=False)]
        normal=np.cross(b-a,c-a); norm=np.linalg.norm(normal)
        if norm<1e-8: continue
        normal/=norm
        if normal[2]<0:normal=-normal
        if normal[2]<min_normal:continue
        offset=-normal@a; inliers=np.abs(sample@normal+offset)<=cfg['ground_distance_m']
        count=int(inliers.sum())
        if count>best_count:best=(normal,offset,inliers);best_count=count
    if best is None or best_count/len(sample)<cfg['minimum_plane_support']:
        return dict(valid=False,reason='no_supported_ground_plane')
    fitting=sample[best[2]]; center=fitting.mean(axis=0)
    _,_,vh=np.linalg.svd(fitting-center,full_matrices=False); normal=vh[-1]
    if normal[2]<0:normal=-normal
    offset=-normal@center
    span=np.ptp(fitting[:,:2],axis=0)
    support=float(np.mean(np.abs(sample@normal+offset)<=cfg['ground_distance_m']))
    if normal[2]<min_normal or support<cfg['minimum_plane_support'] or min(span)<2.:
        return dict(valid=False,reason='unstable_ground_plane')
    return dict(valid=True,normal=normal.tolist(),offset_m=float(offset),
        support_fraction=support,sampled_points=len(sample),inlier_xy_span_m=span.tolist(),
        residual_median_m=float(np.median(np.abs(fitting@normal+offset))))


def compute_preview(predicted_mask, frame, settings=None):
    cfg=dict(DEFAULTS,**(settings or {}))
    if set(cfg)!=set(DEFAULTS):raise ValueError('Unknown occupancy setting')
    for key,value in cfg.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value):raise ValueError('Invalid '+key)
    if not (0<cfg['ground_distance_m']<cfg['minimum_height_m']<cfg['maximum_height_m'] and 0<cfg['minimum_plane_support']<=1 and 0<cfg['maximum_slope_degrees']<45 and 0<cfg['max_range_m']<=250):raise ValueError('Invalid geometric limits')
    for key,bound in [('plane_trials',1024),('plane_sample_limit',16384),('plane_seed',2**32-1)]:
        if type(cfg[key]) is not int or not 0<cfg[key]<=bound:raise ValueError('Invalid bounded '+key)
    xyz=np.asarray(frame['xyz_world'],dtype=np.float64);times=np.asarray(frame['point_times_s'])
    shared=dict(point_times_s=times,image_time_s=frame['image_time_s'],flags=frame['flags'],
        max_time_delta_s=cfg['max_time_delta_s'],occlusion_cell_px=cfg['occlusion_cell_px'],occlusion_tolerance_m=cfg['occlusion_tolerance_m'])
    prediction=np.asarray(predicted_mask)
    if prediction.dtype.kind not in 'iu' or np.any(prediction<0) or np.any(prediction>5):raise ValueError('Project semantic IDs 0..5 required')
    semantic=fuse_semantic_lidar(xyz,prediction,frame['camera_to_world'],frame['calibration'],boundary_margin_px=cfg['boundary_margin_px'],**shared)
    geometry=fuse_semantic_lidar(xyz,np.zeros(prediction.shape,dtype=np.uint8),frame['camera_to_world'],frame['calibration'],boundary_margin_px=0,**shared)
    # Plane uses all timely sensor returns, including outside front-camera FOV.
    origin=np.asarray(frame['camera_to_world'])[3,:3]
    ranges=np.linalg.norm(xyz-origin,axis=1); delta=times-frame['image_time_s']
    timely=((np.asarray(frame['flags'])&64)!=0)&np.isfinite(xyz).all(axis=1)&np.isfinite(times)&(times>=0)&(delta<=FUTURE_TOLERANCE_S)&(np.abs(delta)<=cfg['max_time_delta_s']+1e-9)&(ranges<=cfg['max_range_m'])
    plane=estimate_ground(xyz[timely],cfg)
    height=np.full(len(xyz),np.nan); occupied=np.zeros(len(xyz),dtype=bool)
    if plane['valid']:
        height=xyz@np.asarray(plane['normal'])+plane['offset_m']
        occupied=geometry['accepted']&timely&(height>=cfg['minimum_height_m'])&(height<=cfg['maximum_height_m'])
    # Only occupied samples receive a geometric fused label. RGB remains untouched.
    fused=np.full(len(xyz),-1,dtype=np.int16);fused[occupied]=5
    known=occupied&semantic['accepted']&np.isin(semantic['labels'],[3,4])
    fused[known]=semantic['labels'][known]
    pixels=np.floor(geometry['uv'][occupied]).astype(int)
    counts=dict(input_points=len(xyz),timely_fit_points=int(timely.sum()),
        projectable_points=int(geometry['projectable'].sum()),visible_timely_points=int(geometry['accepted'].sum()),
        occupied_points=int(occupied.sum()),occupied_pixels=len(np.unique(pixels,axis=0)),
        occupied_predicted_road_points=int(np.count_nonzero(occupied&semantic['accepted']&(semantic['labels']==1))),
        occupied_semantic_vehicle_points=int(np.count_nonzero(fused==3)),
        occupied_unclassified_points=int(np.count_nonzero(fused==5)))
    return dict(occupied=occupied,height_m=height,fused_occupied_labels=fused,
        semantic=semantic,geometry=geometry,plane=plane,metrics=dict(**counts,plane=plane,settings=cfg,
        ground_truth_used=False,rendered_depth_used=False,dense_mask_modified=False,
        geometry_rejection_counts=geometry['stats']['reason_counts'],semantic_rejection_counts=semantic['stats']['reason_counts'],
        limitations='Sparse visible measured points only; unobserved is unknown. No ego-motion or moving-object deskew. Ground fit assumes locally planar near-horizontal surface.'))


def evaluate_preview(result, oracle_class):
    """Evaluation only; binary obstacle includes cars, pedestrians, generic objects."""
    truth=np.asarray(oracle_class);occupied=result['occupied'];roi=result['geometry']['projectable'];eligible=result['geometry']['accepted']
    if truth.shape!=occupied.shape or truth.dtype.kind not in 'iu':raise ValueError('Oracle shape/type mismatch')
    known=truth>=0;positive=np.isin(truth,[3,4,5]);compared=eligible&known
    tp=int(np.count_nonzero(compared&occupied&positive));fp=int(np.count_nonzero(compared&occupied&~positive));fn=int(np.count_nonzero(compared&~occupied&positive))
    ratio=lambda a,b:float(a/b) if b else None
    per_class={}
    for name,label in [('vehicle',3),('pedestrian',4),('generic_obstacle',5)]:
        gt=truth==label; a=int(np.count_nonzero(gt&occupied));b=int(np.count_nonzero(gt&eligible));c=int(np.count_nonzero(gt&roi))
        per_class[name]=dict(occupied_points=a,visible_timely_truth_points=b,projectable_truth_points=c,visible_timely_recall=ratio(a,b),projectable_recall=ratio(a,c))
    return dict(true_positive_points=tp,false_positive_points=fp,false_negative_points=fn,
        precision=ratio(tp,tp+fp),visible_timely_recall=ratio(tp,tp+fn),per_class=per_class,
        scope='Point-level occupancy, not object detection or dense semantic obstacle segmentation')


def render_overlay(rgb,result):
    """Exact-pixel sparse overlay. Orange=unclassified occupancy, blue=RGB vehicle."""
    image=np.asarray(rgb).copy()
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:raise ValueError('uint8 RGB required')
    uv=result['geometry']['uv'];occupied=result['occupied'];labels=result['fused_occupied_labels']
    points=np.flatnonzero(occupied)
    # Far first so nearest occupies any repeated projected pixel.
    points=points[np.argsort(result['geometry']['camera_depth_m'][points])[::-1]]
    for idx in points:
        x,y=np.floor(uv[idx]).astype(int)
        image[y,x]=[60,140,250] if labels[idx]==3 else ([230,80,180] if labels[idx]==4 else [255,125,20])
    return image
