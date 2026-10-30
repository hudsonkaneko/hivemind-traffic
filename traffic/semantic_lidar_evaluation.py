"""Offline evidence for camera-mask/LiDAR fusion; no simulator or learner startup.

Scene object identities and rendered depth enter this evaluator only, never the
fusion function. Accuracy is conditional on assignment; ROI coverage is reported
before temporal, boundary and sparse-occlusion rejection to expose abstention.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from traffic.semantic_lidar_fusion import REASONS, REASON_IDS, fuse_semantic_lidar


# Native fixture taxonomy, kept independent of optional dataset-preparation work.
CLASSES = dict(background=0, drivable=1, lane=2, vehicle=3, pedestrian=4, obstacle=5)
PALETTE = np.array([[35,40,50],[70,180,90],[250,230,60],[60,140,250],
                    [230,80,180],[245,125,40]], dtype=np.uint8)
DEFAULT_GATES = dict(minimum_accuracy=.97, minimum_class_accuracy=.90,
    minimum_class_points=20, minimum_roi_coverage=.20,
    minimum_vehicle_frames=4, minimum_oracle_coverage=.98)
SETTING_NAMES = {'max_time_delta_s','occlusion_cell_px','occlusion_tolerance_m',
                 'boundary_margin_px','min_confidence'}


def _path(directory, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError('Capture file paths must be relative')
    path = (directory / relative).resolve()
    if not path.is_relative_to(directory.resolve()) or not path.is_file():
        raise ValueError('Missing or escaping capture file: ' + relative)
    return path


def _ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


def _percentiles(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(min=float(values.min()), median=float(np.median(values)),
                p95=float(np.quantile(values,.95)), max=float(values.max()))


def measure_fusion(fused, oracle_class, *, depth_evaluation=None):
    """Independent per-frame metrics; no oracle alters the projected labels."""
    labels = np.asarray(fused['labels'])
    truth = np.asarray(oracle_class)
    accepted = np.asarray(fused['accepted'])
    roi = np.asarray(fused['projectable'])
    if (truth.shape != labels.shape or truth.dtype.kind not in 'iu'
            or np.any(truth < -1) or np.any(truth >= len(CLASSES))):
        raise ValueError('Oracle classes must match returns and use -1 or known class IDs')
    known = roi & (truth >= 0)
    compared = accepted & known
    if np.any(labels[accepted] < 0) or np.any(labels[accepted] >= len(CLASSES)):
        raise ValueError('Assigned camera class outside declared class inventory')
    matrix = np.zeros((len(CLASSES),len(CLASSES)), dtype=np.int64)
    np.add.at(matrix, (truth[compared],labels[compared]), 1)
    per_class = {}
    for name, class_id in CLASSES.items():
        gt = known & (truth == class_id)
        checked = compared & (truth == class_id)
        prediction_count = int(matrix[:,class_id].sum())
        correct = int(matrix[class_id,class_id])
        per_class[name] = dict(roi_truth_points=int(gt.sum()), compared_points=int(checked.sum()),
            correct_points=correct, predicted_points=prediction_count,
            conditional_accuracy=_ratio(correct,int(checked.sum())),
            precision=_ratio(correct,prediction_count),
            assignment_coverage=_ratio(int(checked.sum()),int(gt.sum())))
    rejected = {name:int(np.count_nonzero(fused['reason']==value)) for name,value in REASON_IDS.items()}
    skew = np.asarray(fused['time_delta_s'])
    max_skew = fused['stats']['settings']['max_time_delta_s']
    future_tolerance = fused['stats']['settings'].get('future_tolerance_s',1e-5)
    invalid_assigned = int(np.count_nonzero(accepted & (~roi | ~np.isfinite(skew)
        | (np.abs(skew)>max_skew+1e-9) | (skew>future_tolerance))))
    result = dict(input_points=len(labels), roi_points=int(roi.sum()),
        oracle_known_roi_points=int(known.sum()), assigned_points=int(accepted.sum()),
        compared_points=int(compared.sum()), correct_points=int(np.trace(matrix)),
        roi_coverage=_ratio(int(accepted.sum()),int(roi.sum())),
        oracle_roi_coverage=_ratio(int(known.sum()),int(roi.sum())),
        accuracy=_ratio(int(np.trace(matrix)),int(compared.sum())),
        invalid_assigned_points=invalid_assigned, reason_counts=rejected,
        confusion_matrix=matrix.tolist(), per_class=per_class,
        absolute_roi_time_skew_s=_percentiles(np.abs(skew[roi])),
        signed_assigned_time_delta_s=_percentiles(skew[accepted]))
    if depth_evaluation is not None:
        depth_image = np.asarray(depth_evaluation)
        if depth_image.ndim != 2:
            raise ValueError('Depth evaluation image must be 2D')
        indices = np.flatnonzero(accepted)
        pixels = np.floor(fused['uv'][indices]).astype(int)
        if len(indices) and (np.any(pixels<0) or np.any(pixels[:,0]>=depth_image.shape[1])
                            or np.any(pixels[:,1]>=depth_image.shape[0])):
            raise ValueError('Depth evaluation resolution disagrees with fusion')
        visible_depth = depth_image[pixels[:,1],pixels[:,0]]
        valid = np.isfinite(visible_depth) & (visible_depth>0)
        residual = fused['camera_depth_m'][indices][valid]-visible_depth[valid]
        threshold = np.maximum(.1,.01*visible_depth[valid])
        hidden = int(np.count_nonzero(residual>threshold))
        result['depth_evaluation_only'] = dict(checked_points=int(valid.sum()),
            behind_camera_visible_surface_points=hidden,
            behind_surface_fraction=_ratio(hidden,int(valid.sum())),
            signed_depth_residual_m=_percentiles(residual),
            threshold='max(0.1m, 1% of rendered image-plane depth); diagnostic, not a fusion input')
    return result


def aggregate_metrics(frames, gates=None):
    gates = dict(DEFAULT_GATES, **(gates or {}))
    if set(gates) != set(DEFAULT_GATES):
        raise ValueError('Unknown fusion acceptance gate')
    for name, value in gates.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value):
            raise ValueError('Invalid fusion acceptance gate ' + name)
        if name.endswith('_points') or name.endswith('_frames'):
            if not isinstance(value,int) or value<1:
                raise ValueError('Count gates must be positive integers')
        elif not 0<=value<=1:
            raise ValueError('Fraction gates must be in [0,1]')
    if not frames:
        raise ValueError('Cannot evaluate an empty capture')
    sum_keys = ('input_points','roi_points','oracle_known_roi_points','assigned_points',
                'compared_points','correct_points','invalid_assigned_points')
    total = {key:sum(frame[key] for frame in frames) for key in sum_keys}
    total.update(frames=len(frames),
        accuracy=_ratio(total['correct_points'],total['compared_points']),
        roi_coverage=_ratio(total['assigned_points'],total['roi_points']),
        oracle_roi_coverage=_ratio(total['oracle_known_roi_points'],total['roi_points']),
        reason_counts={name:sum(f['reason_counts'][name] for f in frames) for name in REASONS},
        confusion_matrix=np.sum([f['confusion_matrix'] for f in frames],axis=0).tolist())
    per_class = {}
    for name in CLASSES:
        counts = {k:sum(f['per_class'][name][k] for f in frames)
                  for k in ('roi_truth_points','compared_points','correct_points','predicted_points')}
        counts.update(conditional_accuracy=_ratio(counts['correct_points'],counts['compared_points']),
            precision=_ratio(counts['correct_points'],counts['predicted_points']),
            assignment_coverage=_ratio(counts['compared_points'],counts['roi_truth_points']),
            frames_with_assigned_truth=sum(f['per_class'][name]['compared_points']>0 for f in frames))
        per_class[name]=counts
    total['per_class']=per_class
    def meets(value, bound):
        return value is not None and value>=bound
    checks = dict(overall_accuracy=meets(total['accuracy'],gates['minimum_accuracy']),
        roi_coverage=meets(total['roi_coverage'],gates['minimum_roi_coverage']),
        oracle_coverage=meets(total['oracle_roi_coverage'],gates['minimum_oracle_coverage']),
        no_invalid_or_stale_assignments=total['invalid_assigned_points']==0,
        moving_vehicle_support=per_class['vehicle']['frames_with_assigned_truth']>=gates['minimum_vehicle_frames'])
    for name in ('vehicle','obstacle'):
        value=per_class[name]
        checks[name+'_support']=value['compared_points']>=gates['minimum_class_points']
        checks[name+'_accuracy']=meets(value['conditional_accuracy'],gates['minimum_class_accuracy'])
        checks[name+'_precision']=meets(value['precision'],gates['minimum_class_accuracy'])
    total.update(gates=gates,checks=checks,passed=all(checks.values()),
        metric_scope='Accuracy/precision on assigned independently mapped returns; coverage denominator is all geometrically projectable valid returns before fusion filters')
    return total


def measure_actor_support(fused, object_id_bytes, object_map, actor_positions):
    """Evaluation-only grouping by RTX ID and declared actor path ancestry."""
    positions={path:np.asarray(value,dtype=float) for path,value in actor_positions.items()}
    if any(not path.startswith('/') or p.shape!=(3,) or not np.isfinite(p).all()
           for path,p in positions.items()):
        raise ValueError('Actor positions must be finite world XYZ keyed by prim path')
    if object_id_bytes is None:
        paths=['']*len(fused['labels'])
    else:
        raw=np.asarray(object_id_bytes)
        if raw.dtype!=np.uint8 or raw.shape!=(len(fused['labels']),16):
            raise ValueError('Object-ID bytes must be uint8 N x 16')
        # Matches installed RTX parse_object_ids; never imported into fusion.
        paths=[object_map.get(str(int.from_bytes(row.tobytes(),'little')),{}).get('path','') for row in raw]
    result={}
    for actor,position in positions.items():
        member=np.asarray([p==actor or p.startswith(actor+'/') for p in paths],dtype=bool)
        assigned=member & fused['accepted'];roi=member & fused['projectable']
        result[actor]=dict(position_world_m=position.tolist(),roi_points=int(roi.sum()),
            assigned_points=int(assigned.sum()),
            correct_vehicle_points=int(np.count_nonzero(assigned & (fused['labels']==CLASSES['vehicle']))))
    return result


def aggregate_actor_support(frames, *, moving):
    actors=set().union(*(frame.keys() for frame in frames))
    stable=all(set(frame)==actors for frame in frames)
    result={}
    for actor in sorted(actors):
        rows=[frame[actor] for frame in frames if actor in frame]
        assigned=sum(r['assigned_points'] for r in rows);correct=sum(r['correct_vehicle_points'] for r in rows)
        roi=sum(r['roi_points'] for r in rows)
        positions=np.asarray([r['position_world_m'] for r in rows])
        displacement=float(np.linalg.norm(positions[-1]-positions[0]))
        support_frames=sum(r['assigned_points']>0 for r in rows)
        accuracy=_ratio(correct,assigned)
        checks=dict(assigned_support=assigned>=20,frame_support=support_frames>=4,
            vehicle_accuracy=accuracy is not None and accuracy>=.90,
            nonzero_displacement=displacement>1e-6)
        result[actor]=dict(roi_points=roi,assigned_points=assigned,correct_vehicle_points=correct,
            vehicle_accuracy=accuracy,assignment_coverage=_ratio(assigned,roi),
            frames_with_assigned_points=support_frames,first_position_world_m=positions[0].tolist(),
            last_position_world_m=positions[-1].tolist(),displacement_m=displacement,checks=checks,
            passed=all(checks.values()))
    checks=dict(stable_actor_inventory=stable,two_traffic_actors=len(actors)==2,
        each_actor_supported=bool(result) and all(row['passed'] for row in result.values()))
    return dict(actors=result,checks=checks,required=moving,passed=not moving or all(checks.values()),
        scope='Evaluation-only RTX IDs grouped by actor prim ancestry; displacement from captured actor metadata',
        gates=dict(minimum_assigned_points=20,minimum_frames=4,minimum_vehicle_accuracy=.90,minimum_displacement_m=1e-6))


def _screenshot(path, rgb, mask, xyz, fused, camera_to_world, metrics, title):
    """Measured sensor panels, never generated imagery or predicted-model claims."""
    from PIL import Image, ImageDraw
    rgb=np.asarray(rgb,dtype=np.uint8)[:,:,:3];h,w=rgb.shape[:2]
    image=Image.new('RGB',(2*w,2*h+100),(20,24,32));draw=ImageDraw.Draw(image)
    colors=np.full((*mask.shape,3),[100,100,100],dtype=np.uint8)
    known=(mask>=0)&(mask<len(PALETTE));colors[known]=PALETTE[mask[known]]
    image.paste(Image.fromarray(rgb),(0,30));image.paste(Image.fromarray(colors),(w,30))
    overlay=Image.fromarray(rgb);marks=ImageDraw.Draw(overlay)
    accepted=np.flatnonzero(fused['accepted']);stride=max(1,len(accepted)//20000)
    for i in accepted[::stride]:
        u,v=np.floor(fused['uv'][i]).astype(int);color=tuple(int(c) for c in PALETTE[fused['labels'][i]])
        marks.ellipse((u-1,v-1,u+1,v+1),fill=color)
    image.paste(overlay,(0,h+65))
    cloud=Image.new('RGB',(w,h),(25,30,38));cd=ImageDraw.Draw(cloud)
    matrix=np.asarray(camera_to_world);delta=xyz-matrix[3,:3]
    side=delta@matrix[0,:3];forward=delta@(-matrix[2,:3])
    for forward_m in (0,10,20,30,40,50,60):
        y=h-1-int(forward_m/60*(h-1));cd.line((0,y,w,y),fill=(60,65,70));cd.text((3,max(0,y-12)),str(forward_m)+'m',fill='white')
    cd.line((w//2,0,w//2,h),fill=(65,70,80))
    visible=accepted[(forward[accepted]>=0)&(forward[accepted]<=60)&(np.abs(side[accepted])<=15)]
    stride=max(1,len(visible)//25000)
    for i in visible[::stride]:
        x=int((side[i]+15)/30*(w-1));y=h-1-int(forward[i]/60*(h-1))
        cd.point((x,y),fill=tuple(int(c) for c in PALETTE[fused['labels'][i]]))
    image.paste(cloud,(w,h+65))
    draw.text((8,8),title+' | actual RGB',fill='white')
    draw.text((w+8,8),'Native semantic mask (ground truth, not predictions)',fill='white')
    draw.text((8,h+42),'Mask-projected LiDAR points | accepted only',fill='white')
    draw.text((w+8,h+42),'Measured accepted cloud | camera-forward depth / lateral',fill='white')
    footer=f"Assigned {metrics['assigned_points']}/{metrics['roi_points']} ROI | accuracy {metrics['accuracy'] if metrics['accuracy'] is not None else 'N/A'} | blue vehicle / orange obstacle / green drivable / yellow lane"
    draw.text((8,2*h+76),footer,fill='white');image.save(path)


def evaluate_capture(directory, fusion_settings, gates, *, output_dir=None):
    directory=Path(directory).resolve();destination=Path(output_dir).resolve() if output_dir else directory/'fusion'
    if not isinstance(fusion_settings,dict) or set(fusion_settings)-SETTING_NAMES:
        raise ValueError('Unknown fusion settings')
    report=json.loads((directory/'capture-result.json').read_text(encoding='utf-8'))
    map_path=directory/'object-id-oracle-map.json'
    object_map=json.loads(map_path.read_text(encoding='utf-8')) if map_path.is_file() else {}
    frames=report.get('frames',[])
    if not frames:
        raise ValueError('Native capture has no frames to evaluate')
    destination.mkdir(parents=True,exist_ok=False)
    result=dict(passed=False,training=False,semantic_source='native simulator mask, not learned predictions',
        inference_uses_oracle=False,depth_role='independent evaluation only',frames=[],screenshots=[])
    selected_frames={0,len(frames)//2,len(frames)-1};last_image_time=-np.inf
    try:
        from PIL import Image
        for index,frame in enumerate(frames):
            meta_path=frame.get('labels_path',frame.get('metadata_path'))
            metadata=json.loads(_path(directory,meta_path).read_text(encoding='utf-8'))
            image_time=float(metadata['image_time_s'])
            if not np.isfinite(image_time) or image_time<=last_image_time:
                raise ValueError('Camera times must be finite and strictly increasing')
            last_image_time=image_time
            rgb=np.asarray(Image.open(_path(directory,frame['rgb_path'])).convert('RGB'))
            mask=np.load(_path(directory,frame['semantic_path']),allow_pickle=False)
            with np.load(_path(directory,frame['lidar_path']),allow_pickle=False) as data:
                xyz=data['xyz_world'].copy();flags=data['flags'].copy()
                times=data['point_times_s' if 'point_times_s' in data else 'timestamp_s'].copy()
                oracle=data['oracle_class'].copy()
                object_ids=data['object_id_bytes'].copy() if 'object_id_bytes' in data else None
                sensor_index=data['sensor_index'].copy() if 'sensor_index' in data else np.zeros(len(xyz),dtype=np.int32)
            fused=fuse_semantic_lidar(xyz,mask,metadata['camera_to_world'],metadata['calibration'],
                point_times_s=times,image_time_s=image_time,flags=flags,**fusion_settings)
            depth_path=frame.get('depth_path',frame.get('depth_evaluation_path'))
            depth=np.load(_path(directory,depth_path),allow_pickle=False) if depth_path else None
            metrics=measure_fusion(fused,oracle,depth_evaluation=depth)
            actor_support=measure_actor_support(fused,object_ids,object_map,metadata.get('actor_positions',{}))
            stem=f'frame-{index:04d}'
            np.savez_compressed(destination/(stem+'-fused.npz'),xyz_world=xyz,flags=flags,
                point_times_s=times,sensor_index=sensor_index,labels=fused['labels'],uv=fused['uv'],
                camera_depth_m=fused['camera_depth_m'],accepted=fused['accepted'],
                projectable=fused['projectable'],reason=fused['reason'],confidence=fused['confidence'],
                oracle_class_evaluation_only=oracle)
            result['frames'].append(dict(frame_index=index,image_time_s=image_time,metrics=metrics,actor_support=actor_support,
                fused_file=stem+'-fused.npz',source_files=dict(rgb=frame['rgb_path'],mask=frame['semantic_path'],
                lidar=frame['lidar_path'],metadata=meta_path)))
            if index in selected_frames:
                filename=stem+'-fusion.png';_screenshot(destination/filename,rgb,mask,xyz,fused,
                    metadata['camera_to_world'],metrics,report.get('scenario',directory.name))
                result['screenshots'].append(filename)
            if index==0:
                # These diagnostics rerun the SAME inputs with known bad calibration/time.
                # They cannot affect nominal labels or tune parameters to oracle outcomes.
                stale=fuse_semantic_lidar(xyz,mask,metadata['camera_to_world'],metadata['calibration'],
                    point_times_s=times,image_time_s=image_time+.25,flags=flags,**fusion_settings)
                wrong=np.asarray(metadata['camera_to_world']).copy();wrong[3,:3]+=wrong[0,:3]*3.
                shifted=fuse_semantic_lidar(xyz,mask,wrong,metadata['calibration'],
                    point_times_s=times,image_time_s=image_time,flags=flags,**fusion_settings)
                result['negative_controls']=dict(stale_camera_plus_250ms=dict(accepted_points=int(stale['accepted'].sum()),
                    passed=int(stale['accepted'].sum())==0),camera_lateral_offset_3m=measure_fusion(shifted,oracle))
        result['summary']=aggregate_metrics([f['metrics'] for f in result['frames']],gates)
        result['actor_support']=aggregate_actor_support([f['actor_support'] for f in result['frames']],moving=report.get('mode')=='moving')
        result['passed']=bool(report.get('passed',False) and result['summary']['passed']
            and result['negative_controls']['stale_camera_plus_250ms']['passed'] and result['actor_support']['passed'])
        result['native_capture_passed']=bool(report.get('passed',False))
        result['fusion_settings']=fusion_settings
        result['limitations']=['Ground-truth camera masks validate geometric fusion, not learned perception.',
            'WORLD NONCOMPENSATED LiDAR is time-gated, not deskewed for moving objects.',
            'Sparse z-buffer cannot reject occluders without a nearer LiDAR return.',
            'Object IDs and rendered depth are evaluation-only; confidence1 denotes provided native labels, not model calibration.']
    except Exception as error:
        result['error']=repr(error)
        raise
    finally:
        result['output_directory']=str(destination)
        result['source_capture_result_sha256']=hashlib.sha256((directory/'capture-result.json').read_bytes()).hexdigest()
        (destination/'evaluation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return result
