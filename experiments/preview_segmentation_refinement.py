"""CPU-only frozen segmentation comparisons on immutable saved captures."""
from __future__ import annotations
import argparse
import importlib.metadata
import json
from pathlib import Path
import time
import traceback

from experiments.probe_support import ROOT, run_package, finish, write_json
from experiments.preview_pretrained_segmentation import sha256, validate_config, capture_inputs

CONFIG=ROOT/'configs/perception/segmentation_refinement_preview_v02.json'


def checked_inputs(config):
    base=json.loads((ROOT/config['base_config']).read_text(encoding='utf-8'))
    source=validate_config(base)
    items,hashes=capture_inputs(base,source)
    source_manifest=json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    for item in items:
        for suffix in ('metadata.json','lidar.npz'):
            path=source/item['case']/(item['stem']+'-'+suffix)
            rel=path.relative_to(source).as_posix()
            expected=source_manifest['output_hashes'].get(rel,source_manifest['output_hashes'].get(rel.replace('/','\\')))
            if sha256(path)!=expected:raise ValueError('Saved sensor hash mismatch: '+rel)
            hashes[rel]=expected
    return base,source,items,hashes


def validate_refinement(config):
    if config.get('training') is not False or config.get('new_simulator_runs') is not False:
        raise ValueError('This runner is inference only')
    if config.get('variants')!=['square512','letterbox512','letterbox1024']:
        raise ValueError('Frozen three-variant comparison required')
    if config.get('boundary_tolerance_px')!=2:
        raise ValueError('Declared boundary tolerance is 2 pixels')
    base=(ROOT/config['base_config']).resolve()
    if not base.is_relative_to(ROOT/'configs/perception'):raise ValueError('Invalid base config')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=CONFIG)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:raise RuntimeError('Run from project root')
    cfg_path=args.config.resolve()
    if not cfg_path.is_relative_to(ROOT):raise ValueError('Config outside project')
    cfg=json.loads(cfg_path.read_text(encoding='utf-8'));validate_refinement(cfg)
    base,source,items,hashes=checked_inputs(cfg)
    if args.check:
        print(json.dumps(dict(valid=True,frames=len(items),variants=cfg['variants'],training=False,new_simulator_runs=False)));return 0
    from scripts.fetch_segmentation_preview_model import fetch_model
    from traffic.lidar_obstacle_preview import DEFAULTS
    cfg['resolved_lidar_settings']=dict(DEFAULTS)
    model_dir,model_manifest=fetch_model(base)
    files=['experiments/preview_segmentation_refinement.py','experiments/preview_pretrained_segmentation.py',
           'experiments/probe_support.py','traffic/segmentation_refinement.py','traffic/lidar_obstacle_preview.py',
           'traffic/semantic_lidar_fusion.py','traffic/segmentation_preview.py','scripts/fetch_segmentation_preview_model.py',
           'traffic/segmentation_refinement_gallery.py',
           'environments/requirements-perception-preview.lock.txt',cfg['base_config'],cfg_path.relative_to(ROOT).as_posix()]
    output,manifest=run_package('perception-refinement/v02',cfg,files)
    manifest.update(input_hashes=hashes,source_capture_manifest_sha256=sha256(source/'manifest.json'),model=model_manifest)
    result=dict(passed=False,training=False,new_simulator_runs=False,quality_accepted=False,frames=[],
                scope='Exploratory same-capture comparisons; sparse occupancy is not a dense semantic obstacle mask')
    print('REFINEMENT_RUN='+str(output),flush=True);start=time.perf_counter()
    try:
        import numpy as np
        import torch
        from PIL import Image
        from transformers import SegformerConfig,SegformerForSemanticSegmentation,SegformerImageProcessor
        from traffic.segmentation_preview import CITYSCAPES_LABELS,map_cityscapes,metrics,aggregate_metrics,render_preview
        from traffic.segmentation_refinement import prepare_rgb,restore_logits,boundary_metrics,aggregate_boundary_metrics
        from traffic.lidar_obstacle_preview import load_saved_frame,compute_preview,evaluate_preview,render_overlay
        torch.set_num_threads(2);torch.set_num_interop_threads(1)
        torch.manual_seed(20261030);torch.use_deterministic_algorithms(True)
        model_cfg=SegformerConfig.from_pretrained(model_dir,local_files_only=True)
        if model_cfg.num_labels!=19 or model_cfg.id2label!={i:n for i,n in enumerate(CITYSCAPES_LABELS)}:raise ValueError('Unexpected pretrained labels')
        processor=SegformerImageProcessor.from_pretrained(model_dir,local_files_only=True)
        model=SegformerForSemanticSegmentation(model_cfg)
        loaded=model.load_state_dict(torch.load(model_dir/'pytorch_model.bin',weights_only=True,map_location='cpu'),strict=True)
        model.eval();model.requires_grad_(False)
        manifest['runtime']={p:importlib.metadata.version(p) for p in ('torch','transformers','Pillow','numpy')}
        manifest['runtime'].update(device='cpu',threads=2,interop_threads=1)
        manifest['processor']=processor.to_dict()
        manifest['weight_loading']=dict(strict=True,weights_only=True,missing=list(loaded.missing_keys),unexpected=list(loaded.unexpected_keys))
        result['repeat_identical']={}
        for item_index,item in enumerate(items):
            with Image.open(item['rgb_path']) as im:rgb=np.array(im.convert('RGB'))
            sensor=load_saved_frame(source/item['case'],item['stem'])
            for variant in cfg['variants']:
                def predict():
                    prepared,transform=prepare_rgb(rgb,variant)
                    inputs=processor(images=prepared,do_resize=False,return_tensors='pt')
                    with torch.inference_mode():
                        native=restore_logits(model(**inputs).logits,transform).argmax(1)[0].to(torch.uint8).numpy()
                    return native,transform
                tick=time.perf_counter();native,transform=predict();elapsed=time.perf_counter()-tick
                if item_index==0:
                    repeat,_=predict();result['repeat_identical'][variant]=bool(np.array_equal(native,repeat))
                mapped=map_cityscapes(native)
                lidar=compute_preview(mapped,sensor)
                # Ground truth and simulator IDs enter evaluation only after predictions.
                gt=np.load(item['semantic_path'],allow_pickle=False)
                with np.load(source/item['case']/(item['stem']+'-lidar.npz'),allow_pickle=False) as raw:
                    oracle=raw['oracle_class'].copy()
                scored=metrics(mapped,gt);boundaries=boundary_metrics(mapped,gt,tolerance_px=2)
                row=dict(case=item['case'],frame=item['stem'],variant=variant,transform=transform,
                         metrics=scored,boundary_metrics=boundaries,inference_seconds=elapsed,
                         lidar_metrics=lidar['metrics'],lidar_evaluation=evaluate_preview(lidar,oracle))
                if variant=='square512':
                    prior=ROOT/'outputs/perception-preview/v01/20261031T020708Z-69f78dab'/item['case']/(item['stem']+'-native19.npy')
                    row['matches_prior_baseline']=bool(np.array_equal(native,np.load(prior,allow_pickle=False))) if prior.exists() else None
                folder=output/variant/item['case'];folder.mkdir(parents=True,exist_ok=True)
                prefix=folder/item['stem']
                np.save(str(prefix)+'-native19.npy',native,allow_pickle=False)
                np.save(str(prefix)+'-project.npy',mapped,allow_pickle=False)
                arrays={k:v for k,v in lidar.items() if isinstance(v,np.ndarray)}
                for group in ('geometry','semantic'):
                    arrays.update({group+'_'+k:v for k,v in lidar[group].items() if isinstance(v,np.ndarray)})
                np.savez_compressed(str(prefix)+'-lidar.npz',**arrays)
                render_preview(Path(str(prefix)+'-preview.png'),rgb,mapped,gt,scored,
                               item['case']+' / '+item['stem']+' / '+variant,model_label='Frozen SegFormer B0 / '+variant)
                Image.fromarray(render_overlay(rgb,lidar)).save(str(prefix)+'-occupancy.png')
                write_json(Path(str(prefix)+'-metrics.json'),row);result['frames'].append(row)
            print('Completed '+item['case']+'/'+item['stem'],flush=True)
        result['variants']={}
        for variant in cfg['variants']:
            rows=[r for r in result['frames'] if r['variant']==variant]
            times=[r['inference_seconds'] for r in rows]
            result['variants'][variant]=dict(metrics=aggregate_metrics([r['metrics'] for r in rows]),
                boundary_metrics=aggregate_boundary_metrics([r['boundary_metrics'] for r in rows]),
                latency_seconds=dict(median=float(np.median(times)),p95=float(np.quantile(times,.95))))
        from traffic.segmentation_refinement_gallery import render_comparison
        result['comparison_screenshots']=[str(render_comparison(output,source,c,'0000',result['frames']).relative_to(output)) for c in base['cases']]
        result['weights_unchanged']=sha256(model_dir/'pytorch_model.bin')==base['model']['weights_sha256']
        result['baseline_comparison']=dict(compared=sum(r.get('matches_prior_baseline') is not None for r in result['frames']),
            mismatches=sum(r.get('matches_prior_baseline') is False for r in result['frames']))
        result['passed']=bool(len(result['frames'])==72 and all(result['repeat_identical'].values()) and result['weights_unchanged'] and not result['baseline_comparison']['mismatches'])
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['wall_seconds']=time.perf_counter()-start
        result['source_changed_during_run']=[p for p,h in manifest['source_hashes'].items() if sha256(ROOT/p)!=h]
        result['inputs_changed_during_run']=[p for p,h in hashes.items() if sha256(source/p)!=h]
        result['passed']=bool(result['passed'] and not result['source_changed_during_run'] and not result['inputs_changed_during_run'])
        finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),passed=result['passed'],error=result.get('error'))))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
