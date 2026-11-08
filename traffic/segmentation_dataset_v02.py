"""Deterministic dataset plan and fail-closed manifest checks; no simulator import."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import numpy as np

CLASSES={'background':0,'road':1,'lane':2,'vehicle':3,'pedestrian':4,'obstacle':5}
SPLITS={'train':144,'validation':18,'test':18}
HUMANS={'train':'female_adult_police_02','validation':'male_adult_construction_03','test':'male_adult_police_04'}
ROOT=Path(__file__).resolve().parents[1]

def sha256(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def scene_plan(config,smoke=False):
    if config.get('training') is not False or config.get('frames_per_scene')!=20 or config.get('split_scene_groups')!=SPLITS:raise ValueError('Unexpected dataset contract')
    scenes=[]
    for split_index,(split,total) in enumerate(SPLITS.items()):
        count=2 if smoke else total
        for i in range(count):
            seed=config['seed']+split_index*10000+i;rng=np.random.default_rng(seed)
            scene=dict(scene_id=f'{split}-{i:03d}',split=split,seed=seed,
                frames=2 if smoke else 20,empty_negative=(i%6==1),
                human_identity=HUMANS[split],road_family={'train':'constant_curvature','validation':'slow_s_bend','test':'tight_s_bend'}[split],
                obstacle_family={'train':'box_and_cone','validation':'cylinder_and_barrier','test':'pyramid_and_lowcylinder'}[split],
                curvature=float(rng.uniform(-.004,.004)),road_width=float(rng.uniform(9.5,12.)),
                road_color=[float(rng.uniform(.055,.085))]*3,ground_color=rng.uniform(.08,.22,3).tolist(),
                sunlight=float(rng.uniform(600,2500)),sun_elevation=float(rng.uniform(20,75)),
                camera_height=float(rng.uniform(.9,1.35)),camera_pitch=float(rng.uniform(6,14)),
                lead_distance=float(rng.uniform(8,28)),obstacle_distance=float(rng.uniform(6,25)),
                pedestrian_distance=float(rng.uniform(6,20)),camera_lateral=float(rng.uniform(-1,1)))
            # The same predeclared coverage layout in every split; not label-dependent retries.
            scene['coverage_fixture']=i%6==0
            if scene['coverage_fixture']:
                scene.update(lead_distance=float(rng.uniform(18,28)),pedestrian_distance=float(rng.uniform(6,9)),obstacle_distance=float(rng.uniform(6,10)))
            scenes.append(scene)
    return scenes

def canonical_mask(raw,id_to_labels):
    raw=np.asarray(raw);raw=raw[:,:,0] if raw.ndim==3 and raw.shape[2]==1 else raw
    if raw.ndim!=2:raise ValueError('Semantic image must be two dimensional')
    mask=np.zeros(raw.shape,dtype=np.uint8);unknown=[]
    if not set(map(int,np.unique(raw))).issubset(set(map(int,id_to_labels))):raise ValueError('Unmapped native semantic ID')
    for key,value in id_to_labels.items():
        if isinstance(value,dict):value=value.get('class','background')
        names=set(str(value).replace(',',' ').split());names={'vehicle' if x=='car' else 'road' if x=='drivable' else x for x in names}
        names.discard('BACKGROUND');names.discard('UNLABELLED')
        if not names:continue
        if len(names)!=1 or not names.issubset(CLASSES):unknown.append([str(key),str(value)]);continue
        mask[raw==int(key)]=CLASSES[next(iter(names))]
    if unknown:raise ValueError('Unknown semantic labels: '+repr(unknown))
    return mask

def validate_manifest(manifest,root,verify_hashes=True):
    root=Path(root).resolve();frames=manifest['frames'];errors=[];counts={s:np.zeros(6,dtype=np.int64) for s in SPLITS}
    plans={s['scene_id']:s for s in manifest.get('scenes',[])}
    groups={};pairs=set();rgb_hashes=set();duplicates=[];group_frames={};families={s:{k:set() for k in ('human_identity','road_family','obstacle_family')} for s in SPLITS}
    for row in frames:
        split=row['split'];group=row['scene_id']
        if split not in SPLITS:raise ValueError('Unknown split')
        if row.get('group_id')!=group:errors.append('group_identity_mismatch:'+group)
        if group in groups and groups[group]!=split:errors.append('group_overlap:'+group)
        groups[group]=split
        if plans:
            planned=plans.get(group)
            if not planned or any(row.get(k)!=planned.get(k) for k in ('seed','split','road_family','obstacle_family','human_identity','empty_negative')):errors.append('plan_mismatch:'+group)
        group_frames.setdefault(group,[]).append(row['frame_index'])
        for key in families[split]:families[split][key].add(row[key])
        if row['human_identity']!=HUMANS[split]:errors.append('human_split_identity:'+group)
        expected_road={'train':'constant_curvature','validation':'slow_s_bend','test':'tight_s_bend'}[split]
        expected_obstacle={'train':'box_and_cone','validation':'cylinder_and_barrier','test':'pyramid_and_lowcylinder'}[split]
        if row['road_family']!=expected_road or row['obstacle_family']!=expected_obstacle:errors.append('family_split:'+group)
        paths={}
        for key in ('rgb_path','semantic_path','instance_path','depth_path')+ (('metadata_path',) if 'metadata_path' in row else ()):
            if Path(row[key]).is_absolute():raise ValueError('Absolute annotation path')
            path=(root/row[key]).resolve()
            if not path.is_relative_to(root) or not path.is_file():raise ValueError('Missing/escaping '+key)
            paths[key]=path
            if verify_hashes and sha256(path)!=row[key.replace('_path','_sha256')]:raise ValueError('Hash mismatch '+key)
        mask=np.load(paths['semantic_path'],allow_pickle=False)
        if mask.shape!=(360,640) or mask.dtype!=np.uint8 or np.any(mask>5):raise ValueError('Invalid semantic mask')
        counts[split]+=np.bincount(mask.ravel(),minlength=6)
        from PIL import Image
        with Image.open(paths['rgb_path']) as image:
            if image.size!=(640,360):raise ValueError('Invalid RGB size')
            gray=np.asarray(image.convert('L').resize((8,8)));actual_hash=''.join('1' if v>gray.mean() else '0' for v in gray.ravel())
            if row.get('perceptual_hash')!=actual_hash:raise ValueError('Perceptual hash missing or mismatched')
        for key in ('instance_path','depth_path'):
            annotation=np.load(paths[key],allow_pickle=False)
            if annotation.shape!=(360,640):raise ValueError('Annotation dimensions disagree')
            if key=='instance_path' and annotation.dtype.kind not in 'iu':raise ValueError('Instance IDs must be integers')
            if key=='depth_path' and (annotation.dtype.kind!='f' or np.isnan(annotation).any() or (annotation<0).any()):raise ValueError('Invalid depth')
        pair=(row['rgb_sha256'],row['semantic_sha256'])
        if pair in pairs:duplicates.append(row['rgb_path'])
        pairs.add(pair)
        if row['rgb_sha256'] in rgb_hashes:errors.append('duplicate_rgb:'+row['rgb_path'])
        rgb_hashes.add(row['rgb_sha256'])
    if duplicates:errors.append('duplicate_rgb_mask_pairs')
    if any(sorted(v)!=list(range(20)) for v in group_frames.values()):errors.append('per_group_frame_inventory')
    for split,n in SPLITS.items():
        if sum(v==split for v in groups.values())!=n:errors.append('scene_count:'+split)
        if sum(r['split']==split for r in frames)!=n*20:errors.append('frame_count:'+split)
        if np.any(counts[split]==0):errors.append('missing_class:'+split)
    near=[]
    for i,a in enumerate(frames):
        for b in frames[i+1:]:
            if a['split']==b['split']:continue
            distance=(int(a.get('perceptual_hash','0'),2)^int(b.get('perceptual_hash','0'),2)).bit_count()
            if distance<=2 and len(near)<100:near.append(dict(a=a['rgb_path'],b=b['rgb_path'],hamming=distance))
    near_digest=hashlib.sha256(json.dumps(near,sort_keys=True).encode()).hexdigest()
    dataset_digest=hashlib.sha256(json.dumps(frames,sort_keys=True).encode()).hexdigest()
    review=manifest.get('near_duplicate_review',{})
    reviewed=bool(review.get('reviewed') is True and review.get('pairs_sha256')==near_digest and review.get('dataset_frames_sha256')==dataset_digest and review.get('explanation'))
    if near and not reviewed:errors.append('near_duplicate_review_pending')
    return dict(training_ready=not errors,errors=sorted(set(errors)),frame_count=len(frames),scene_count=len(groups),
                class_counts_by_split={k:v.tolist() for k,v in counts.items()},duplicate_pairs=duplicates,
                near_duplicate_diagnostic=dict(method='8x8 grayscale average hash, Hamming<=2; first100 cross-split pairs; common road silhouettes are not proof of duplicate scenes',pairs=near,pairs_sha256=near_digest,dataset_frames_sha256=dataset_digest,review_required=bool(near and not reviewed)),
                limitations=['Shared vehicle visual identity across splits; only road-family, obstacle-family and pedestrian-identity holdout',
                'Static rigged human rest poses, procedural road/props, simple lighting; not photoreal or real-road generalization',
                'No motion vectors; temporal flicker cannot be motion-compensated from this dataset alone'])
