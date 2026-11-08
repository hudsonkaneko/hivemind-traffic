"""Guarded supervised segmentation trainer; --check never creates a learner."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess
import time
import traceback
from experiments.probe_support import ROOT,run_package,finish,write_json

DEFAULT_CONFIG=ROOT/'configs/perception/segmentation_training_v02.json'


def validate_config(cfg,launch=False):
    if launch and (cfg.get('launch_enabled') is not True or cfg.get('training_authorized') is not True):raise ValueError('Training is disabled pending explicit user authorization')
    if cfg['model']['output_classes']!=6 or cfg['dataset']['classes']!={'background':0,'road':1,'lane':2,'vehicle':3,'pedestrian':4,'obstacle':5}:raise ValueError('Six-class configuration required')
    b=cfg['budget']
    if b['microbatch_size'] not in (2,4) or b['microbatch_size']*b['gradient_accumulation_steps']!=16:raise ValueError('Reviewed effective batch16 required')
    if b['epochs_max']!=20 or b['optimizer_updates_max']!=3600 or b['wall_time_cap_hours_including_validation']!=2:raise ValueError('Reviewed training budget required')
    if cfg['preprocessing']['source_hw']!=[360,640] or cfg['preprocessing']['input_hw']!=[384,640]:raise ValueError('Reviewed preprocessing required')
    if cfg['preprocessing']['normalization_mean']!=[.485,.456,.406] or cfg['preprocessing']['normalization_std']!=[.229,.224,.225] or cfg['preprocessing']['label_reduction'] is not False:raise ValueError('Normalization/class IDs differ from reviewed contract')
    if cfg['loss']['formula']!='weighted_CE + 0.5 * soft_Dice + 0.2 * boundary_Dice':raise ValueError('Loss implementation differs from requested formula')
    augmentation=cfg['preprocessing']['augmentation']
    if augmentation['brightness_factor']!=[.8,1.2] or augmentation['contrast_factor']!=[.8,1.2] or augmentation['horizontal_flip_probability']!=.5 or augmentation['validation_test_augmentation'] is not False:raise ValueError('Augmentation differs from implemented reviewed contract')
    if cfg['optimization']['optimizer']!='AdamW':raise ValueError('Only reviewed AdamW is implemented')
    path=cfg['dataset'].get('manifest')
    if path is not None:
        p=(ROOT/path).resolve()
        if not p.is_relative_to(ROOT/'outputs'):raise ValueError('Dataset must remain in project outputs')
    if launch and (not path or not cfg['dataset']['ready'] or not b.get('cuda_runtime_verified')):raise ValueError('Dataset and CUDA preflight must be ready')


def gpu_exclusive_check():
    if os.name=='nt':
        command="Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'kit.exe' -or ($_.Name -eq 'python.exe' -and $_.ExecutablePath -like 'C:\\isaacsim\\*') } | Select-Object -ExpandProperty ProcessId"
        active=subprocess.check_output(['powershell','-NoProfile','-Command',command],text=True,timeout=15).strip()
        if active:raise RuntimeError('Isaac/Kit is active; GPU jobs must run serially')
    active=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name','--format=csv,noheader,nounits'],text=True,timeout=15)
    other=[];desktop=[]
    for line in active.splitlines():
        fields=line.split(',',1)
        if len(fields)!=2 or not fields[0].strip().isdigit():continue
        pid=int(fields[0].strip());name=fields[1].strip();low=name.lower()
        if pid==os.getpid():continue
        if any(word in low for word in ('python','kit.exe','isaac','comfy','blender','cuda','torch')):other.append(dict(pid=pid,name=name))
        else:desktop.append(dict(pid=pid,name=name))
    if other:raise RuntimeError('Other GPU compute processes are active: '+str(other))
    return desktop


def dataset_quality_check(path):
    from traffic.segmentation_dataset_v02 import validate_manifest
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    checked=validate_manifest(data,Path(path).parent,verify_hashes=True)
    if not checked.get('training_ready'):raise ValueError('Dataset structure/quality gates failed: '+str(checked.get('errors')))
    return checked


def resolve_dataset_manifest(cfg):
    """Bind even --check to the reviewed manifest bytes, before model creation."""
    name=cfg['dataset'].get('manifest')
    if name is None:
        if cfg['dataset'].get('ready'):raise ValueError('Ready dataset has no manifest')
        return None
    path=(ROOT/name).resolve()
    if not path.is_relative_to(ROOT/'outputs') or not path.is_file():raise ValueError('Declared dataset manifest is missing or outside project outputs')
    expected=cfg['dataset'].get('manifest_sha256')
    if cfg['dataset'].get('ready') and not expected:raise ValueError('Ready dataset requires a reviewed manifest SHA-256')
    if expected is not None:
        if not isinstance(expected,str) or len(expected)!=64 or any(c not in '0123456789abcdef' for c in expected):raise ValueError('Invalid dataset manifest SHA-256')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise ValueError('Dataset manifest differs from reviewed configuration')
    return path


def load_batch(rows):
    import numpy as np
    from PIL import Image
    rgb=[];masks=[]
    for row in rows:
        with Image.open(row['rgb_absolute']) as image:rgb.append(np.array(image.convert('RGB')))
        masks.append(np.load(row['semantic_absolute'],allow_pickle=False))
    return rgb,masks


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--resume',type=Path)
    parser.add_argument('--evaluate',type=Path,help='Evaluate a frozen checkpoint; never updates weights')
    parser.add_argument('--split',choices=('validation','test'),default='validation')
    parser.add_argument('--device',choices=('cpu','cuda'),default='cpu',help='Frozen evaluation device only; training always requires CUDA')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:raise RuntimeError('Run from canonical project root')
    cfg_path=args.config.resolve()
    if not cfg_path.is_relative_to(ROOT):raise ValueError('Config must stay in project')
    cfg=json.loads(cfg_path.read_text(encoding='utf-8'));validate_config(cfg,launch=not args.check and not args.evaluate)
    if args.resume and args.evaluate:raise ValueError('Choose resume or frozen evaluation')
    data_path=resolve_dataset_manifest(cfg)
    if args.check:
        detail=dict(valid=True,launch_enabled=cfg.get('launch_enabled',False),training_authorized=cfg.get('training_authorized',False),learner_created=False)
        if data_path is not None:
            from traffic.segmentation_training import read_manifest
            detail['dataset_quality']=dataset_quality_check(data_path)
            _,splits,counts=read_manifest(data_path);detail.update(split_counts={s:len(v) for s,v in splits.items()},class_counts={s:v.tolist() for s,v in counts.items()})
        print(json.dumps(detail,indent=2));return 0
    from traffic.segmentation_training import (read_manifest,build_model,prepare_batch,restore_logits,segmentation_loss,
        class_weights_from_counts,build_optimizer,lr_multiplier,six_class_metrics,aggregate_metrics,
        atomic_checkpoint,rng_state,restore_rng,config_fingerprint)
    if data_path is None:raise ValueError('A prepared dataset is required')
    dataset,splits,counts=read_manifest(data_path)
    dataset_quality_check(data_path)
    if not dataset.get('training_ready'):raise ValueError('Dataset quality gates have not passed')
    expected=cfg['dataset']['split_frames']
    if any(len(splits[s])!=expected[s] for s in splits):raise ValueError('Dataset split counts do not match reviewed config')
    dataset_hash=hashlib.sha256(data_path.read_bytes()).hexdigest();fingerprint=config_fingerprint(cfg)
    if args.evaluate:return evaluate_checkpoint(args,cfg,cfg_path,data_path,splits)
    desktop_gpu_processes=gpu_exclusive_check()
    import numpy as np
    import torch
    if not torch.cuda.is_available():raise RuntimeError('Verified CUDA runtime required')
    seed=cfg['budget']['initial_training_seed'];random.seed(seed);np.random.seed(seed);torch.manual_seed(seed);torch.cuda.manual_seed_all(seed)
    rng=np.random.default_rng(seed);torch.backends.cudnn.benchmark=False
    model_dir=ROOT/'runtimes/perception-preview/models'/cfg['model']['revision']
    model=build_model(cfg['model'],model_dir,seed).to('cuda')
    optimizer=build_optimizer(model,cfg['optimization'])
    scheduler=torch.optim.lr_scheduler.LambdaLR(optimizer,lr_multiplier)
    scaler=torch.amp.GradScaler('cuda',init_scale=float(cfg['budget'].get('scaler_initial_scale',128.)))
    class_weights=class_weights_from_counts(counts['train'])
    budget=cfg['budget'];micro=budget['microbatch_size'];accum=budget['gradient_accumulation_steps']
    state=dict(epoch=0,next_index=0,order=[],phase='train',updates=0,best_score=None,best_false_road=None,best_epoch=None,best_checkpoint_path=None,stale_epochs=0,elapsed_seconds=0.)
    sources=['experiments/train_segmentation.py','experiments/probe_support.py','traffic/segmentation_training.py','traffic/segmentation_refinement.py','traffic/segmentation_dataset_v02.py',cfg_path.relative_to(ROOT).as_posix()]
    output,manifest=run_package('perception-training/segmentation-refinement-v02',cfg,sources)
    import importlib.metadata
    manifest.update(dataset_manifest_sha256=dataset_hash,config_fingerprint=fingerprint,class_weights=class_weights.tolist(),device=torch.cuda.get_device_name(),torch_version=torch.__version__,desktop_gpu_processes=desktop_gpu_processes,runtime_versions={p:importlib.metadata.version(p) for p in ('torch','transformers','numpy','Pillow')},pretrained_weights_sha256=cfg['model']['weights_sha256'])
    result=dict(passed=False,training=True,held_out_test_evaluated=False,quality_accepted=False,updates=0)
    if args.resume:
        resume=args.resume.resolve()
        if not resume.is_relative_to(ROOT/'outputs'):raise ValueError('Resume only project checkpoints')
        saved=torch.load(resume,map_location='cpu',weights_only=True)
        if saved['config_fingerprint']!=fingerprint or saved['dataset_manifest_sha256']!=dataset_hash:raise ValueError('Resume config/data mismatch')
        model.load_state_dict(saved['model'],strict=True);optimizer.load_state_dict(saved['optimizer']);scheduler.load_state_dict(saved['scheduler']);scaler.load_state_dict(saved['scaler']);state=saved['state'];restore_rng(saved['rng'],rng)
        manifest['resume_source']=str(resume.relative_to(ROOT))
    prior_elapsed=state['elapsed_seconds'];start=time.perf_counter();cap=budget['wall_time_cap_hours_including_validation']*3600
    def elapsed():return prior_elapsed+time.perf_counter()-start
    def checkpoint(name):
        state['elapsed_seconds']=elapsed()
        atomic_checkpoint(output/name,dict(schema='segmentation-checkpoint-v02',selection_status='validation_selected' if name=='best.pt' else 'resume_only',classes=dict(cfg['dataset']['classes']),model=model.state_dict(),optimizer=optimizer.state_dict(),scheduler=scheduler.state_dict(),scaler=scaler.state_dict(),state=dict(state),rng=rng_state(rng),config=cfg,config_fingerprint=fingerprint,dataset_manifest_sha256=dataset_hash,class_weights=class_weights.tolist()))
    def append_log(row):
        with (output/'metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
    try:
        checkpoint('last.pt')
        stop_reason='epochs_complete'
        while state['epoch']<budget['epochs_max']:
            if elapsed()>=cap:stop_reason='wall_time_cap';break
            if state['phase']=='train':
                if not state['order']:state['order']=rng.permutation(len(splits['train'])).tolist()
                model.train()
                while state['next_index']<len(state['order']):
                    if elapsed()>=cap or state['updates']>=budget['optimizer_updates_max']:break
                    index=state['next_index'];indices=state['order'][index:index+micro*accum]
                    optimizer.zero_grad(set_to_none=True);parts=[];losses=[]
                    for offset in range(0,len(indices),micro):
                        ids=indices[offset:offset+micro];batch=[splits['train'][i] for i in ids]
                        x,y=prepare_batch(*load_batch(batch),device='cuda',augment=True,rng=rng)
                        with torch.autocast('cuda',dtype=torch.float16):logits=model(pixel_values=x).logits
                        loss,terms=segmentation_loss(restore_logits(logits),y,class_weights)
                        if not torch.isfinite(loss):raise FloatingPointError('Nonfinite training loss')
                        scaler.scale(loss*(len(ids)/len(indices))).backward();losses.append(float(loss.detach()))
                        parts.append({k:float(v.detach()) for k,v in terms.items()})
                    scaler.unscale_(optimizer);grad_norm=torch.nn.utils.clip_grad_norm_(model.parameters(),cfg['optimization']['gradient_clip_norm'],error_if_nonfinite=True)
                    scaler.step(optimizer);scaler.update();scheduler.step()
                    state['updates']+=1;state['next_index']+=len(indices)
                    append_log(dict(kind='train',epoch=state['epoch'],update=state['updates'],loss=float(np.mean(losses)),components={k:float(np.mean([p[k] for p in parts])) for k in parts[0]},grad_norm=float(grad_norm),elapsed_seconds=elapsed()))
                    if state['updates']%50==0:checkpoint('last.pt')
                if state['next_index']<len(state['order']):stop_reason='wall_time_cap' if elapsed()>=cap else 'update_cap';break
                state['phase']='validation';checkpoint('last.pt')
            model.eval();rows=[];per_scene={}
            with torch.inference_mode():
                for offset in range(0,len(splits['validation']),micro):
                    if elapsed()>=cap:break
                    batch=splits['validation'][offset:offset+micro];x,y=prepare_batch(*load_batch(batch),device='cuda')
                    with torch.autocast('cuda',dtype=torch.float16):logits=model(pixel_values=x).logits
                    predictions=restore_logits(logits).argmax(1).cpu().numpy();truth=y.cpu().numpy()
                    for item,p,g in zip(batch,predictions,truth):
                        metric=six_class_metrics(p,g);rows.append(metric);per_scene.setdefault(item['scene_id'],[]).append(metric)
            if len(rows)!=len(splits['validation']):stop_reason='wall_time_cap_during_validation';break
            scored=aggregate_metrics(rows)
            if scored['selection_score'] is None:raise RuntimeError('Validation lacks required class/boundary support')
            score=scored['selection_score'];false_road=scored['false_drivable_rate']
            write_json(output/('validation-epoch-%02d.json'%state['epoch']),dict(aggregate=scored,per_scene={s:aggregate_metrics(r) for s,r in per_scene.items()},metrics_not_implemented=['instance-object matching','distance/size bins','motion-compensated temporal flicker']))
            append_log(dict(kind='validation',epoch=state['epoch'],update=state['updates'],score=score,false_drivable_rate=false_road,elapsed_seconds=elapsed()))
            improved=state['best_score'] is None or score>state['best_score'] or (score==state['best_score'] and false_road<state['best_false_road'])
            significant=state['best_score'] is None or score>=state['best_score']+cfg['selection']['minimum_score_improvement']
            state['stale_epochs']=0 if significant else state['stale_epochs']+1
            if improved:state.update(best_score=score,best_false_road=false_road,best_epoch=state['epoch'],best_checkpoint_path=str((output/'best.pt').relative_to(ROOT)))
            state['epoch']+=1;state['next_index']=0;state['order']=[];state['phase']='train'
            if improved:checkpoint('best.pt')
            checkpoint('last.pt')
            if state['stale_epochs']>=cfg['selection']['early_stopping_patience_epochs']:stop_reason='early_stopping';break
            if state['updates']>=budget['optimizer_updates_max']:stop_reason='update_cap';break
        checkpoint('last.pt');result.update(passed=True,stop_reason=stop_reason,updates=state['updates'],epochs_completed=state['epoch'],best_score=state['best_score'],best_epoch=state['best_epoch'],best_checkpoint_path=state.get('best_checkpoint_path'),elapsed_seconds=elapsed())
    except Exception as error:
        # last.pt is always a completed optimizer-update boundary. Never overwrite
        # it with partial gradients or a half-consumed accumulation group.
        # Charge failed work to the cumulative budget while retaining the last
        # complete optimizer/RNG/cursor state (never partial gradients).
        last=output/'last.pt'
        if last.exists():
            preserved=torch.load(last,map_location='cpu',weights_only=True)
            preserved['state']['elapsed_seconds']=elapsed();atomic_checkpoint(last,preserved)
        result.update(error=repr(error),traceback=traceback.format_exc(),resume_checkpoint='last.pt',elapsed_seconds=elapsed());traceback.print_exc()
    finally:
        result['source_changed_during_run']=[p for p,h in manifest['source_hashes'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['passed']=bool(result['passed'] and not result['source_changed_during_run']);finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),**result),indent=2));return 0 if result['passed'] else 1


def evaluate_checkpoint(args,cfg,cfg_path,data_path,splits):
    """Frozen evaluation only; held-out test requires a validation-selected actor."""
    import numpy as np
    import torch
    from PIL import Image,ImageDraw
    from traffic.segmentation_preview import PALETTE,_font
    from traffic.segmentation_training import build_model,prepare_batch,restore_logits,six_class_metrics,aggregate_metrics,config_fingerprint,CLASSES
    checkpoint=args.evaluate.resolve()
    if not checkpoint.is_relative_to(ROOT/'outputs'):raise ValueError('Checkpoint must remain in project outputs')
    saved=torch.load(checkpoint,map_location='cpu',weights_only=True)
    data_hash=hashlib.sha256(data_path.read_bytes()).hexdigest()
    if saved['config_fingerprint']!=config_fingerprint(cfg) or saved['dataset_manifest_sha256']!=data_hash or saved.get('classes')!=CLASSES:raise ValueError('Evaluation checkpoint contract mismatch')
    if args.split=='test' and saved.get('selection_status')!='validation_selected':raise ValueError('Held-out test requires validation-selected best.pt')
    desktop=gpu_exclusive_check() if args.device=='cuda' else []
    model=build_model(cfg['model'],ROOT/'runtimes/perception-preview/models'/cfg['model']['revision'],cfg['budget']['initial_training_seed'])
    model.load_state_dict(saved['model'],strict=True);model=model.to(args.device).eval();model.requires_grad_(False)
    output,manifest=run_package('perception-evaluation/segmentation-v02',dict(config=cfg,split=args.split,training=False),['experiments/train_segmentation.py','traffic/segmentation_training.py','traffic/segmentation_refinement.py',cfg_path.relative_to(ROOT).as_posix()])
    manifest.update(checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),dataset_manifest_sha256=data_hash,desktop_gpu_processes=desktop)
    result=dict(passed=False,training=False,quality_accepted=False,split=args.split,frames=[]);rows=[];by_scene={};latencies=[];start=time.perf_counter()
    try:
        with torch.inference_mode():
            for i,item in enumerate(splits[args.split]):
                rgb,gt=load_batch([item]);x,y=prepare_batch(rgb,gt,device=args.device)
                if args.device=='cuda':torch.cuda.synchronize()
                tick=time.perf_counter();prediction=restore_logits(model(pixel_values=x).logits).argmax(1)[0].cpu().numpy()
                if args.device=='cuda':torch.cuda.synchronize()
                latency=time.perf_counter()-tick;latencies.append(latency)
                metric=six_class_metrics(prediction,gt[0]);rows.append(metric);by_scene.setdefault(item['scene_id'],[]).append(metric)
                result['frames'].append(dict(scene_id=item['scene_id'],rgb_path=item['rgb_path'],metric=metric,inference_seconds=latency))
                if i<3:
                    error=np.where((prediction==gt[0])[...,None],np.array([40,180,80]),np.array([230,60,60])).astype(np.uint8)
                    error[gt[0]==255]=[100,100,100]
                    canvas=Image.new('RGB',(2560,410),(20,24,32));draw=ImageDraw.Draw(canvas)
                    for j,(name,panel) in enumerate(zip(('Saved RGB','Predicted six-class mask','Ground truth','Errors: green correct / red wrong'),(rgb[0],PALETTE[prediction],PALETTE[np.minimum(gt[0],5)],error))):
                        canvas.paste(Image.fromarray(panel),(j*640,50));draw.text((j*640+10,15),name,fill='white',font=_font(20))
                    canvas.save(output/('preview-%04d.png'%i))
        result.update(passed=True,aggregate=aggregate_metrics(rows),per_scene={s:aggregate_metrics(r) for s,r in by_scene.items()},latency_seconds=dict(median=float(np.median(latencies)),p95=float(np.quantile(latencies,.95))),metrics_not_implemented=['instance-object matching','distance/size bins','motion-compensated temporal flicker'])
    except Exception as error:result.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:result['wall_seconds']=time.perf_counter()-start;finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),passed=result['passed']),indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
