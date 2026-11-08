"""Supervised six-class segmentation primitives; importing starts no training."""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import random
import numpy as np

CLASSES = {'background':0,'road':1,'lane':2,'vehicle':3,'pedestrian':4,'obstacle':5}
MEAN=(.485,.456,.406)
STD=(.229,.224,.225)


def class_weights_from_counts(counts):
    counts=np.asarray(counts,dtype=np.float64)
    if counts.shape!=(6,) or not np.isfinite(counts).all() or np.any(counts<=0):
        raise ValueError('Every training class must have positive pixel support')
    raw=np.maximum(counts/counts.sum(),.001)**-.5
    return np.clip(raw/raw.mean(),.25,4.).astype(np.float32)


def prepare_batch(rgb_arrays, mask_arrays, device='cpu', augment=False, rng=None):
    """Preserve 640x360 geometry and pad normalized RGB with exact zero."""
    import torch
    import torch.nn.functional as F
    if not rgb_arrays or len(rgb_arrays)!=len(mask_arrays):raise ValueError('Paired nonempty batch required')
    rng=rng if rng is not None else np.random.default_rng(0)
    images=[];targets=[]
    for rgb,mask in zip(rgb_arrays,mask_arrays):
        rgb=np.asarray(rgb);mask=np.asarray(mask)
        if rgb.dtype!=np.uint8 or rgb.shape!=(360,640,3) or mask.shape!=(360,640):raise ValueError('Expected RGB640x360 and matching mask')
        if mask.dtype.kind not in 'iu' or not np.isin(mask,[0,1,2,3,4,5,255]).all():raise ValueError('Invalid six-class target')
        image=rgb.astype(np.float32)/255.;target=mask.astype(np.int64,copy=True)
        if augment:
            image=np.clip(image*float(rng.uniform(.8,1.2)),0,1)
            image=np.clip((image-image.mean(axis=(0,1),keepdims=True))*float(rng.uniform(.8,1.2))+image.mean(axis=(0,1),keepdims=True),0,1)
            if rng.random()<.5:image=image[:,::-1].copy();target=target[:,::-1].copy()
        image=(image-np.asarray(MEAN,dtype=np.float32))/np.asarray(STD,dtype=np.float32)
        images.append(torch.from_numpy(image.transpose(2,0,1).copy()))
        targets.append(torch.from_numpy(target.copy()))
    return F.pad(torch.stack(images),(0,0,12,12),value=0).to(device),torch.stack(targets).to(device)


def restore_logits(logits,target_hw=(360,640),input_hw=(384,640)):
    import torch.nn.functional as F
    if tuple(target_hw)!=(360,640) or tuple(input_hw)!=(384,640):raise ValueError('Fixed reviewed input geometry required')
    return F.interpolate(logits.float(),size=input_hw,mode='bilinear',align_corners=False)[:,:,12:372,:]


def segmentation_loss(logits,targets,class_weights):
    """CE + .5 foreground Dice + .2 differentiable boundary Dice, FP32."""
    import torch
    import torch.nn.functional as F
    if logits.ndim!=4 or logits.shape[1]!=6 or targets.shape!=(logits.shape[0],*logits.shape[2:]):raise ValueError('Six-class logits/target shape mismatch')
    if targets.dtype!=torch.long or not torch.isin(targets,torch.tensor([0,1,2,3,4,5,255],device=targets.device)).all():raise ValueError('Invalid targets')
    valid=targets!=255
    if not valid.any():raise ValueError('No valid target pixels')
    weights=torch.as_tensor(class_weights,dtype=torch.float32,device=logits.device)
    if weights.shape!=(6,) or not torch.isfinite(weights).all() or (weights<=0).any():raise ValueError('Invalid class weights')
    z=logits.float();ce=F.cross_entropy(z,targets,weight=weights,ignore_index=255)
    probs=z.softmax(1);onehot=F.one_hot(targets.masked_fill(~valid,0),6).permute(0,3,1,2).float()
    v=valid[:,None].float();p=probs*v;y=onehot*v
    counts=y.sum((0,2,3));supported=counts[1:]>0
    dice_each=1-(2*(p*y).sum((0,2,3))+1)/(p.sum((0,2,3))+counts+1)
    zero=z.sum()*0.;dice=dice_each[1:][supported].mean() if supported.any() else zero
    # Exclude the full neighborhood of ignored pixels and image borders.
    domain=1-F.max_pool2d(F.pad(1-v,(1,1,1,1),value=1),3,stride=1)
    def gradient(a):return F.max_pool2d(a,3,1,1)+F.max_pool2d(-a,3,1,1)
    bp=gradient(probs)*domain;by=gradient(onehot)*domain
    supported_b=by.sum((0,2,3))[1:]>0
    boundary_each=1-(2*(bp*by).sum((0,2,3))+1)/(bp.sum((0,2,3))+by.sum((0,2,3))+1)
    boundary=boundary_each[1:][supported_b].mean() if supported_b.any() else zero
    total=ce+.5*dice+.2*boundary
    return total,dict(ce=ce,dice=dice,boundary=boundary,total=total)


def build_model(model_config,model_dir,seed):
    """Strict original head load, then explicitly initialize a new six-class head."""
    import torch
    from transformers import SegformerConfig,SegformerForSemanticSegmentation
    from traffic.segmentation_preview import CITYSCAPES_LABELS
    model_dir=Path(model_dir);weights=model_dir/'pytorch_model.bin'
    if hashlib.sha256(weights.read_bytes()).hexdigest()!=model_config['weights_sha256']:raise ValueError('Pretrained weights checksum mismatch')
    torch.manual_seed(seed)
    cfg=SegformerConfig.from_pretrained(model_dir,local_files_only=True)
    if cfg.num_labels!=19 or cfg.id2label!={i:n for i,n in enumerate(CITYSCAPES_LABELS)}:raise ValueError('Unexpected pretrained label contract')
    model=SegformerForSemanticSegmentation(cfg)
    model.load_state_dict(torch.load(weights,weights_only=True,map_location='cpu'),strict=True)
    old=model.decode_head.classifier
    model.decode_head.classifier=torch.nn.Conv2d(old.in_channels,6,kernel_size=1)
    model.config.num_labels=6;model.config.id2label={v:k for k,v in CLASSES.items()};model.config.label2id=dict(CLASSES)
    model.requires_grad_(True)
    return model


def build_optimizer(model,config):
    import torch
    groups={}
    for name,param in model.named_parameters():
        if not param.requires_grad:continue
        role='encoder' if name.startswith('segformer.') else 'decoder'
        decay=not(name.endswith('.bias') or param.ndim<=1)
        groups.setdefault((role,decay),[]).append(param)
    return torch.optim.AdamW([dict(params=params,lr=config['encoder_learning_rate' if role=='encoder' else 'decoder_and_classifier_learning_rate'],weight_decay=config['weight_decay'] if decay else 0.) for (role,decay),params in groups.items()],betas=tuple(config['betas']),eps=config['epsilon'])


def lr_multiplier(step,warmup=180,total=3600):
    if step<warmup:return .1+.9*step/warmup
    progress=min(1.,(step-warmup)/max(1,total-warmup))
    return .1+.9*.5*(1+math.cos(math.pi*progress))


def six_class_metrics(prediction,truth):
    """All six prediction classes count; ignore255 only. No unsupported classes."""
    from traffic.segmentation_refinement import _boundary,_dilate_disk
    p=np.asarray(prediction);g=np.asarray(truth)
    if p.shape!=g.shape or p.ndim!=2 or p.dtype.kind not in 'iu' or g.dtype.kind not in 'iu':raise ValueError('Integer 2D masks required')
    if not np.isin(p,range(6)).all() or not np.isin(g,[0,1,2,3,4,5,255]).all():raise ValueError('Invalid six-class mask')
    valid=g!=255;confusion=np.zeros((6,6),dtype=np.int64);np.add.at(confusion,(g[valid],p[valid]),1)
    boundary={}
    # A one-pixel exclusion around ignored samples prevents manufactured edges.
    edge_domain=~_dilate_disk(~valid,1)
    for name,pm,gm in [('road',np.isin(p,[1,2]),np.isin(g,[1,2])),('vehicle',p==3,g==3)]:
        pb=_boundary(pm)&edge_domain;gb=_boundary(gm)&edge_domain
        boundary[name]=dict(predicted=int(pb.sum()),truth=int(gb.sum()),matched_predicted=int((pb&_dilate_disk(gb,2)).sum()),matched_truth=int((gb&_dilate_disk(pb,2)).sum()))
    nonroad=valid&~np.isin(g,[1,2])
    return dict(confusion=confusion.tolist(),boundary=boundary,false_road=int(((p==1)&nonroad).sum()),nonroad=int(nonroad.sum()))


def aggregate_metrics(rows):
    if not rows:raise ValueError('Nonempty evaluation required')
    c=sum(np.asarray(r['confusion'],dtype=np.int64) for r in rows);per_class={};ratio=lambda a,b:float(a/b) if b else None
    for name,i in CLASSES.items():
        tp=int(c[i,i]);gt=int(c[i].sum());pred=int(c[:,i].sum())
        per_class[name]=dict(tp=tp,gt=gt,predicted=pred,iou=ratio(tp,gt+pred-tp),precision=ratio(tp,pred),recall=ratio(tp,gt))
    boundaries={}
    for name in ('road','vehicle'):
        counts={k:sum(r['boundary'][name][k] for r in rows) for k in ('predicted','truth','matched_predicted','matched_truth')}
        precision=ratio(counts['matched_predicted'],counts['predicted']);recall=ratio(counts['matched_truth'],counts['truth'])
        f1=None if precision is None or recall is None else (2*precision*recall/(precision+recall) if precision+recall else 0.)
        boundaries[name]=dict(**counts,precision=precision,recall=recall,f1=f1)
    ious=[r['iou'] for r in per_class.values() if r['iou'] is not None];bf=[r['f1'] for r in boundaries.values() if r['f1'] is not None]
    all_classes=all(r['gt']>0 for r in per_class.values())
    return dict(confusion=c.tolist(),per_class=per_class,boundary=boundaries,mean_iou=float(np.mean(ious)) if ious else None,
        false_drivable_rate=ratio(sum(r['false_road'] for r in rows),sum(r['nonroad'] for r in rows)),
        all_classes_present=all_classes,selection_score=.5*float(np.mean(ious))+.5*float(np.mean(bf)) if all_classes and len(bf)==2 else None)


def read_manifest(path,verify_hashes=True):
    """Validate split isolation and paths before any model or learner is created."""
    from PIL import Image
    path=Path(path).resolve();data=json.loads(path.read_text(encoding='utf-8'));root=path.parent
    if data.get('schema_version')!='segmentation-dataset-v02' or data.get('classes')!=CLASSES:raise ValueError('Dataset schema/classes mismatch')
    frames=data.get('frames',[])
    if not frames:raise ValueError('Dataset has no frames')
    scenes={};digests={};counts={s:np.zeros(6,dtype=np.int64) for s in ('train','validation','test')};splits={s:[] for s in counts}
    for row in frames:
        split=row['split'];scene=row['scene_id']
        if split not in splits:raise ValueError('Invalid split')
        if scene in scenes and scenes[scene]!=split:raise ValueError('Scene crosses splits')
        scenes[scene]=split
        for key in ('rgb','semantic'):
            rel=Path(row[key+'_path']);f=(root/rel).resolve()
            if rel.is_absolute() or not f.is_relative_to(root) or not f.is_file():raise ValueError('Missing/escaping dataset path')
            if verify_hashes and hashlib.sha256(f.read_bytes()).hexdigest()!=row[key+'_sha256']:raise ValueError('Dataset hash mismatch')
            if key=='rgb':
                digest=row[key+'_sha256']
                if digest in digests:raise ValueError('Duplicate RGB capture')
                digests[digest]=split
            row[key+'_absolute']=str(f)
        mask=np.load(row['semantic_absolute'],allow_pickle=False)
        if mask.shape!=(360,640) or mask.dtype.kind not in 'iu' or not np.isin(mask,[0,1,2,3,4,5,255]).all():raise ValueError('Invalid target dimensions/classes')
        with Image.open(row['rgb_absolute']) as image:
            if image.size!=(640,360):raise ValueError('Invalid RGB dimensions')
        counts[split]+=np.bincount(mask[mask!=255].ravel(),minlength=6)
        splits[split].append(row)
    for split in splits:
        if not splits[split] or np.any(counts[split]==0):raise ValueError('All classes required in every split: '+split)
    return data,splits,counts


def atomic_checkpoint(path,payload):
    import torch
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('wb') as f:torch.save(payload,f);f.flush();os.fsync(f.fileno())
    os.replace(temporary,path)


def rng_state(generator):
    import torch
    return dict(python=random.getstate(),numpy=generator.bit_generator.state,torch=torch.get_rng_state(),cuda=torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [])


def restore_rng(state,generator):
    import torch
    random.setstate(state['python']);generator.bit_generator.state=state['numpy'];torch.set_rng_state(state['torch'])
    if state['cuda']:torch.cuda.set_rng_state_all(state['cuda'])


def config_fingerprint(config):
    # Authorization changes do not alter the statistical experiment.
    content={k:v for k,v in config.items() if k not in ('status','launch_enabled','training_authorized')}
    return hashlib.sha256(json.dumps(content,sort_keys=True,separators=(',',':')).encode()).hexdigest()
