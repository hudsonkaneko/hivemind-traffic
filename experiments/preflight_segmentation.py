"""Bounded segmentation CUDA check: forward/backward only, zero optimizer steps."""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import time
import traceback

from experiments.probe_support import ROOT, run_package, finish, gpu_sample

DEFAULT_CONFIG=ROOT/'configs/perception/segmentation_training_v02.json'
REGRESSION_ROOT=ROOT/'outputs/semantic-lidar-fusion/v01/20261030T223941Z-65f18896'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_config(config):
    """This executable cannot launch or authorize learning."""
    if config.get('launch_enabled') is not False or config.get('training_authorized') is not False:
        raise ValueError('Use a disabled, unauthorized training config for preflight')
    budget=config['budget']
    if budget['microbatch_size'] not in (2,4) or budget['gradient_accumulation_steps'] not in (4,8):
        raise ValueError('Bounded batch2/4 and accumulation4/8 required')
    if budget['microbatch_size']*budget['gradient_accumulation_steps']!=16:
        raise ValueError('Effective batch must remain16')
    if config['preprocessing']['input_hw']!=[384,640] or config['model']['output_classes']!=6:
        raise ValueError('Expected reviewed six-class384x640 contract')


def ensure_gpu_exclusive():
    """Refuse known concurrent Isaac or learner processes; never terminate them."""
    if os.name!='nt':raise RuntimeError('This preflight exclusivity check is Windows-specific')
    command=("Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'kit.exe' -or "
        "($_.Name -match '^python(w)?\\.exe$' -and ("
        "$_.ExecutablePath -like 'C:\\isaacsim\\*' -or $_.CommandLine -match 'C:\\\\isaacsim\\\\' -or "
        "$_.CommandLine -match '(isaaclab|train_segmentation|[\\\\/]train[^\\s]*\\.py|capture_segmentation_dataset_v02|capture_semantic_lidar_fusion|probe_semantic_lidar_fusion)')) } "
        "| Select-Object ProcessId,Name,ExecutablePath,CommandLine | ConvertTo-Json -Compress")
    text=subprocess.check_output(['powershell','-NoProfile','-Command',command],text=True,timeout=20).strip()
    rows=json.loads(text) if text else []
    if isinstance(rows,dict):rows=[rows]
    rows=[row for row in rows if row['ProcessId']!=os.getpid()]
    if rows:raise RuntimeError('Concurrent Isaac/training process; preflight must run serially: '+json.dumps(rows))
    applications=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name',
        '--format=csv,noheader,nounits'],text=True,timeout=15)
    desktop=[];compute=[]
    for line in applications.splitlines():
        fields=line.split(',',1)
        if len(fields)!=2 or not fields[0].strip().isdigit():continue
        pid=int(fields[0].strip());name=fields[1].strip()
        if pid==os.getpid():continue
        row=dict(pid=pid,name=name)
        if any(word in name.lower() for word in ('python','kit.exe','isaac','comfy','blender','cuda','torch')):compute.append(row)
        else:desktop.append(row)
    if compute:raise RuntimeError('Other GPU compute process; preflight must run serially: '+json.dumps(compute))
    return dict(checked=True,known_conflicting_processes=rows,desktop_gpu_processes=desktop,
                limitation='Process inventory excludes ordinary desktop rendering; not an OS-wide GPU lock')


def load_inputs(config,mode):
    """Return actual input paths/hashes; held-out frames never enter this check."""
    batch=config['budget']['microbatch_size']
    if mode=='dataset':
        rel=config['dataset'].get('manifest')
        if not rel:raise ValueError('Dataset manifest is required; choose saved-regression explicitly for memory-only checks')
        manifest_path=(ROOT/rel).resolve()
        if not manifest_path.is_relative_to(ROOT/'outputs'):raise ValueError('Manifest must stay inside outputs')
        manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
        if manifest.get('schema_version')!='segmentation-dataset-v02':raise ValueError('Dataset schema mismatch')
        frames=[f for f in manifest['frames'] if f['split']=='train'][:batch]
        if len(frames)!=batch:raise ValueError('Insufficient train frames')
        pairs=[];hashes={str(manifest_path.relative_to(ROOT)):digest(manifest_path)}
        for frame in frames:
            paths=[]
            for key in ('rgb','semantic'):
                path=(manifest_path.parent/frame[key+'_path']).resolve()
                if not path.is_relative_to(manifest_path.parent) or digest(path)!=frame[key+'_sha256']:
                    raise ValueError('Dataset path/hash mismatch')
                paths.append(path);hashes[str(path.relative_to(ROOT))]=digest(path)
            pairs.append(tuple(paths))
        counts=manifest['class_counts_by_split']['train']
        return pairs,hashes,counts,dict(mode=mode,training_ready=manifest.get('training_ready'),
            selection='First declared training split microbatch only; no validation/test access')
    if mode!='saved-regression':raise ValueError('Unknown input mode')
    source_manifest=json.loads((REGRESSION_ROOT/'manifest.json').read_text(encoding='utf-8'))
    pairs=[];hashes={}
    for index in range(batch):
        paths=[]
        for suffix in ('rgb.png','semantic.npy'):
            path=REGRESSION_ROOT/'static-101'/f'{index:04d}-{suffix}'
            rel=path.relative_to(REGRESSION_ROOT).as_posix();actual=digest(path)
            expected=source_manifest['output_hashes'].get(rel,source_manifest['output_hashes'].get(rel.replace('/','\\')))
            if expected!=actual:raise ValueError('Regression input hash mismatch')
            paths.append(path);hashes[str(path.relative_to(ROOT))]=actual
        pairs.append(tuple(paths))
    return pairs,hashes,None,dict(mode=mode,training_ready=False,
        selection='Four already-inspected regression images; uniform weights; memory/finite-gradient evidence only, not dataset readiness')


def state_digest(model,parameters_only=False):
    h=hashlib.sha256()
    values=model.named_parameters() if parameters_only else model.state_dict().items()
    for name,tensor in values:
        value=tensor.detach().cpu().contiguous()
        h.update(name.encode());h.update(str(value.dtype).encode());h.update(str(tuple(value.shape)).encode())
        h.update(value.numpy().tobytes())
    return h.hexdigest()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--input-mode',choices=['dataset','saved-regression'],default='dataset')
    parser.add_argument('--check',action='store_true',help='Validate inputs/config without importing torch or touching CUDA')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:raise RuntimeError('Run from canonical project root')
    path=args.config.resolve()
    if not path.is_relative_to(ROOT/'configs'):raise ValueError('Configuration must stay under project configs')
    config=json.loads(path.read_text(encoding='utf-8'));validate_config(config)
    pairs,input_hashes,counts,input_info=load_inputs(config,args.input_mode)
    if args.check:
        print(json.dumps(dict(valid=True,training=False,optimizer_steps=0,input_info=input_info)));return 0
    exclusive=ensure_gpu_exclusive()
    source_files=['experiments/preflight_segmentation.py','traffic/segmentation_training.py',
        'scripts/fetch_segmentation_preview_model.py','experiments/probe_support.py',
        'environments/requirements-perception-training.txt','environments/requirements-perception-training.lock.txt',
        path.relative_to(ROOT).as_posix()]
    output,manifest=run_package('perception-preflight/v02',config,source_files)
    manifest.update(command=list(__import__('sys').argv),input_hashes=input_hashes,input_info=input_info,
                    gpu_exclusivity=exclusive)
    result=dict(passed=False,training=False,optimizer_steps=0,new_simulator_runs=False,
        scope='Bounded train-mode forward/backward and eval forward; no learned weights or model-quality result',
        input_info=input_info)
    started=time.perf_counter()
    print('SEGMENTATION_PREFLIGHT='+str(output),flush=True)
    try:
        import numpy as np
        import torch
        from PIL import Image
        from traffic.segmentation_training import build_model,prepare_batch,restore_logits,segmentation_loss,class_weights_from_counts
        from scripts.fetch_segmentation_preview_model import fetch_model
        model_config=json.loads((ROOT/'configs/perception/pretrained_segmentation_preview_v01.json').read_text())
        model_dir,model_manifest=fetch_model(model_config)
        if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
        torch.set_num_threads(2);torch.manual_seed(config['budget']['initial_training_seed'])
        manifest['model']=model_manifest
        manifest['runtime']={p:importlib.metadata.version(p) for p in ('torch','transformers','numpy','Pillow')}
        manifest['runtime'].update(cuda=torch.version.cuda,cudnn=torch.backends.cudnn.version(),
            gpu_name=torch.cuda.get_device_name(0),gpu_capability=list(torch.cuda.get_device_capability(0)),
            torch_arches=torch.cuda.get_arch_list(),gpu_before=gpu_sample())
        rgbs=[];targets=[]
        for rgb,mask in pairs:
            with Image.open(rgb) as image:rgbs.append(np.array(image.convert('RGB')))
            targets.append(np.load(mask,allow_pickle=False))
        x,y=prepare_batch(rgbs,targets,device='cuda',augment=False)
        weights=class_weights_from_counts(counts) if counts is not None else np.ones(6,dtype=np.float32)
        weights=torch.tensor(weights,dtype=torch.float32,device='cuda')
        model=build_model(config['model'],model_dir,config['budget']['initial_training_seed']).to('cuda')
        before_parameters=state_digest(model,True);before_state=state_digest(model)
        buffers={name:buf.detach().clone() for name,buf in model.named_buffers()}
        optimizer=torch.optim.AdamW(model.parameters(),lr=3e-5)
        # Allocate the two FP32 AdamW moment tensors without taking any step.
        for parameter in model.parameters():
            optimizer.state[parameter]['step']=torch.tensor(0.)
            optimizer.state[parameter]['exp_avg']=torch.zeros_like(parameter)
            optimizer.state[parameter]['exp_avg_sq']=torch.zeros_like(parameter)
        scaler=torch.amp.GradScaler('cuda',init_scale=128.)
        model.train();torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats()
        timings=[];losses=[];microbatches=config['budget']['gradient_accumulation_steps']
        for index in range(microbatches):
            tick=time.perf_counter()
            with torch.autocast(device_type='cuda',dtype=torch.float16):
                raw=model(pixel_values=x).logits
                logits=restore_logits(raw)
                loss,parts=segmentation_loss(logits,y,weights)
            if not torch.isfinite(loss):raise RuntimeError('Nonfinite preflight loss')
            scaler.scale(loss/microbatches).backward();torch.cuda.synchronize()
            timings.append(time.perf_counter()-tick);losses.append(float(loss.detach()))
        scaler.unscale_(optimizer)
        gradients=[p.grad for p in model.parameters() if p.grad is not None]
        if not gradients or not all(torch.isfinite(g).all() for g in gradients):raise RuntimeError('Nonfinite or missing gradients')
        gradient_norm=float(torch.nn.utils.clip_grad_norm_(model.parameters(),1.))
        if not np.isfinite(gradient_norm):raise RuntimeError('Nonfinite gradient norm')
        result['train_mode_buffers_changed']=sum(not torch.equal(buf,buffers[name]) for name,buf in model.named_buffers())
        with torch.no_grad():
            for name,buf in model.named_buffers():buf.copy_(buffers[name])
        model.zero_grad(set_to_none=True);model.eval();eval_times=[]
        for _ in range(3):
            tick=time.perf_counter()
            with torch.inference_mode(),torch.autocast(device_type='cuda',dtype=torch.float16):
                evaluated=restore_logits(model(pixel_values=x).logits)
            torch.cuda.synchronize();eval_times.append(time.perf_counter()-tick)
            if not torch.isfinite(evaluated).all():raise RuntimeError('Nonfinite eval logits')
        result.update(microbatch_size=len(rgbs),accumulation_microbatches=microbatches,
            effective_batch=len(rgbs)*microbatches,input_shape=list(x.shape),target_shape=list(y.shape),
            class_weights=weights.cpu().tolist(),gradient_norm_before_clip=gradient_norm,
            all_gradients_finite=True,loss_values=losses,grad_scaler_initial_scale=128.,
            training_forward_backward_seconds=timings,eval_microbatch_seconds=eval_times,
            peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
            peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
            adam_moments_allocated_without_step=True,
            parameters_before_sha256=before_parameters,parameters_after_sha256=state_digest(model,True),
            full_state_before_sha256=before_state,full_state_after_buffer_restore_sha256=state_digest(model))
        # This estimate excludes data loading, optimizer-step time, checkpointing and other contention.
        train_micro=float(np.median(timings[1:] or timings));eval_micro=float(np.median(eval_times))
        result['projected_compute_only_epoch_seconds']=train_micro*2880/len(rgbs)+eval_micro*360/len(rgbs)
        result['timing_data_residency']='Input microbatch already loaded and GPU resident; eval forward uses the same training rows, never held-out examples'
        result['projection_limit']='Compute-only extrapolation, not measured epoch throughput or completion promise; excludes optimizer-step time, IO, augmentation, checkpointing and contention'
        result['parameters_unchanged']=before_parameters==result['parameters_after_sha256']
        result['full_state_restored']=before_state==result['full_state_after_buffer_restore_sha256']
        result['weights_file_unchanged']=digest(model_dir/'pytorch_model.bin')==model_config['model']['weights_sha256']
        result['gpu_after']=gpu_sample()
        result['passed']=result['parameters_unchanged'] and result['full_state_restored'] and result['weights_file_unchanged']
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['wall_seconds']=time.perf_counter()-started
        result['source_changed_during_run']=[p for p,h in manifest['source_hashes'].items() if digest(ROOT/p)!=h]
        result['inputs_changed_during_run']=[p for p,h in input_hashes.items() if digest(ROOT/p)!=h]
        result['passed']=bool(result['passed'] and not result['source_changed_during_run'] and not result['inputs_changed_during_run'])
        finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),passed=result['passed'],optimizer_steps=0,error=result.get('error'))))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
