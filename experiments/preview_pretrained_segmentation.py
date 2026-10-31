"""Frozen RGB-only segmentation of existing captures. No simulator or training."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import re
import subprocess
import time

from experiments.probe_support import ROOT, run_package, finish

CONFIG = ROOT/'configs/perception/pretrained_segmentation_preview_v01.json'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_config(config):
    if (config.get('version') != 'pretrained-segmentation-preview-v01'
            or config.get('training') is not False or config.get('new_simulator_runs') is not False):
        raise ValueError('Explicit inference-only, no-simulator configuration required')
    inf=config['inference']
    if inf['device']!='cpu' or type(inf['threads']) is not int or not 1<=inf['threads']<=4 or inf['batch_size']!=1:
        raise ValueError('Preview requires bounded single-image CPU inference')
    if inf['processor_size']!={'height':512,'width':512} or config['model']['trained_head_replaced'] is not False:
        raise ValueError('Keep the frozen pretrained processor and trained head')
    if type(config['frames_per_case']) is not int or not 1<=config['frames_per_case']<=8:
        raise ValueError('Bounded existing-capture preview required')
    cases=config['cases']
    if not 1<=len(cases)<=3 or len(set(cases))!=len(cases) or any(not re.fullmatch('[a-z][a-z0-9-]{1,63}',c) for c in cases):
        raise ValueError('Invalid capture case inventory')
    source=(ROOT/config['source_capture_run']).resolve()
    if not source.is_relative_to(ROOT/'outputs') or not source.is_dir():
        raise ValueError('Existing project capture run required')
    return source


def capture_inputs(config,source):
    """Read only explicitly declared saved RGB and ground-truth frames."""
    manifest=json.loads((source/'manifest.json').read_text())
    if manifest.get('status')!='passed':raise ValueError('Use an accepted source capture run')
    items=[];hashes={}
    for case in config['cases']:
        report_path=source/case/'capture-result.json'
        report=json.loads(report_path.read_text())
        if not report.get('passed') or len(report['frames'])!=config['frames_per_case']:
            raise ValueError('Incomplete declared capture case')
        for frame in report['frames']:
            paths={}
            for key in ('rgb_path','semantic_path'):
                relative=Path(frame[key]);path=(source/case/relative).resolve()
                if relative.is_absolute() or not path.is_relative_to(source/case) or not path.is_file():
                    raise ValueError('Escaping or missing capture input')
                rel=path.relative_to(source).as_posix();digest=sha256(path)
                expected=manifest['output_hashes'].get(rel,manifest['output_hashes'].get(rel.replace('/','\\')))
                if digest!=expected:raise ValueError('Source capture hash mismatch: '+rel)
                hashes[rel]=digest;paths[key]=path
            items.append(dict(case=case,stem=frame['stem'],**paths))
    return items,hashes


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=CONFIG)
    parser.add_argument('--check',action='store_true',help='Validate saved inputs without downloading or inference')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:raise RuntimeError('Run from project root')
    cfg_path=args.config.resolve()
    if not cfg_path.is_relative_to(ROOT):raise ValueError('Configuration must stay inside project')
    config=json.loads(cfg_path.read_text());source=validate_config(config)
    items,input_hashes=capture_inputs(config,source)
    if args.check:
        print(json.dumps(dict(valid=True,training=False,new_simulator_runs=False,frames=len(items),config=config),indent=2));return 0
    from scripts.fetch_segmentation_preview_model import fetch_model
    model_dir,model_manifest=fetch_model(config)
    sources=['experiments/preview_pretrained_segmentation.py','scripts/fetch_segmentation_preview_model.py',
             'traffic/segmentation_preview.py','environments/requirements-perception-preview.txt',str(cfg_path.relative_to(ROOT)).replace('\\','/')]
    output,manifest=run_package('perception-preview/v01',config,sources)
    print('PREVIEW_RUN='+str(output),flush=True)
    manifest.update(input_hashes=input_hashes,source_capture_manifest_sha256=sha256(source/'manifest.json'),model=model_manifest)
    result=dict(passed=False,training=False,new_simulator_runs=False,quality_accepted=False,
                scope='Frozen pretrained RGB inference; exploratory shared-class scores, not driving acceptance',frames=[])
    started=time.perf_counter()
    try:
        import numpy as np
        import torch
        from PIL import Image
        from transformers import SegformerConfig,SegformerForSemanticSegmentation,SegformerImageProcessor
        from traffic.segmentation_preview import CITYSCAPES_LABELS,map_cityscapes,metrics,aggregate_metrics,render_preview
        torch.set_num_threads(config['inference']['threads']);torch.set_num_interop_threads(1)
        torch.manual_seed(config['inference']['seed']);torch.use_deterministic_algorithms(True)
        model_config=SegformerConfig.from_pretrained(model_dir,local_files_only=True)
        expected={i:name for i,name in enumerate(CITYSCAPES_LABELS)}
        if model_config.num_labels!=19 or model_config.id2label!=expected:raise ValueError('Unexpected pretrained class inventory')
        processor=SegformerImageProcessor.from_pretrained(model_dir,local_files_only=True)
        if processor.size!=config['inference']['processor_size']:raise ValueError('Processor defaults changed')
        model=SegformerForSemanticSegmentation(model_config)
        state=torch.load(model_dir/'pytorch_model.bin',map_location='cpu',weights_only=True)
        loaded=model.load_state_dict(state,strict=True);del state
        model.eval();model.requires_grad_(False)
        manifest['runtime']={p:importlib.metadata.version(p) for p in ('torch','transformers','Pillow','numpy')}
        manifest['runtime'].update(device='cpu',threads=torch.get_num_threads(),interop_threads=torch.get_num_interop_threads())
        manifest['processor']=processor.to_dict()
        manifest['weight_loading']=dict(weights_only=True,strict=True,missing=list(loaded.missing_keys),unexpected=list(loaded.unexpected_keys),native_classes=19)
        (output/'requirements.freeze.txt').write_text(subprocess.check_output([str(Path(__import__('sys').executable)),'-m','pip','freeze'],text=True))
        latencies=[]
        for index,item in enumerate(items):
            with Image.open(item['rgb_path']) as image:rgb=np.array(image.convert('RGB'))
            h,w=rgb.shape[:2]
            def predict():
                # The sole model input is saved RGB. No GT, depth, LiDAR or object IDs.
                inputs=processor(images=rgb,return_tensors='pt')
                with torch.inference_mode():
                    logits=model(**inputs).logits
                    resized=torch.nn.functional.interpolate(logits,size=(h,w),mode='bilinear',align_corners=False)
                    return resized.argmax(dim=1)[0].to(torch.uint8).numpy()
            tick=time.perf_counter();native=predict();elapsed=time.perf_counter()-tick;latencies.append(elapsed)
            if index==0:
                repeated=predict();result['repeat_first_prediction_identical']=bool(np.array_equal(native,repeated))
                if not result['repeat_first_prediction_identical']:raise RuntimeError('Frozen CPU prediction was not repeatable')
            mapped=map_cityscapes(native)
            # Ground truth enters only after prediction, for comparison and drawing.
            gt=np.load(item['semantic_path'],allow_pickle=False)
            measured=metrics(mapped,gt)
            frame_dir=output/item['case'];frame_dir.mkdir(exist_ok=True)
            prefix=frame_dir/item['stem']
            np.save(str(prefix)+'-native19.npy',native,allow_pickle=False)
            np.save(str(prefix)+'-project.npy',mapped,allow_pickle=False)
            picture=Path(str(prefix)+'-preview.png')
            render_preview(picture,rgb,mapped,gt,measured,item['case']+' / '+item['stem'])
            row=dict(case=item['case'],frame=item['stem'],rgb_source=str(item['rgb_path'].relative_to(ROOT)),
                     ground_truth_source=str(item['semantic_path'].relative_to(ROOT)),metrics=measured,
                     inference_seconds=elapsed,native_shape=list(native.shape),native_counts={CITYSCAPES_LABELS[int(k)]:int(v) for k,v in zip(*np.unique(native,return_counts=True))},
                     screenshot=str(picture.relative_to(output)).replace('\\','/'))
            result['frames'].append(row)
            (Path(str(prefix)+'-metrics.json')).write_text(json.dumps(row,indent=2)+'\n')
        result['aggregate']=aggregate_metrics([r['metrics'] for r in result['frames']])
        result['by_case']={c:aggregate_metrics([r['metrics'] for r in result['frames'] if r['case']==c]) for c in config['cases']}
        result['latency_seconds']=dict(first=latencies[0],median=float(np.median(latencies)),p95=float(np.quantile(latencies,.95)),scope='CPU preprocessing + forward + logits resize/argmax, excludes rendering and file IO; first includes warmup')
        result['model_eval_mode']=not model.training
        result['all_gradients_disabled']=all(not p.requires_grad for p in model.parameters())
        result['weights_unchanged']=sha256(model_dir/'pytorch_model.bin')==config['model']['weights_sha256']
        result['passed']=bool(len(result['frames'])==len(items) and result['model_eval_mode'] and result['all_gradients_disabled'] and result['weights_unchanged'])
    except Exception as error:
        import traceback
        result.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['wall_seconds']=time.perf_counter()-started
        result['source_changed_during_run']=[p for p,h in manifest['source_hashes'].items() if sha256(ROOT/p)!=h]
        result['inputs_changed_during_run']=[p for p,h in input_hashes.items() if sha256(source/p)!=h]
        result['passed']=bool(result['passed'] and not result['source_changed_during_run'] and not result['inputs_changed_during_run'])
        finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),passed=result['passed'],training=False,new_simulator_runs=False,error=result.get('error'))))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
