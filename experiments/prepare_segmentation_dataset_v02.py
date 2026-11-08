"""Prepare native labeled images without launching any learner."""
import argparse,json,subprocess,os,time
from pathlib import Path
from experiments.probe_support import ROOT,run_package,finish
from traffic.segmentation_dataset_v02 import CLASSES,scene_plan,validate_manifest
from traffic.segmentation_dataset_v02 import sha256
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime,load_config

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--smoke',action='store_true');parser.add_argument('--check',action='store_true');args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:raise RuntimeError('Use project root')
    import PIL  # Fail before opening Isaac if the chosen isolated environment lacks Pillow.
    cfg=json.loads((ROOT/'configs/perception/segmentation_dataset_v02.json').read_text());cfg['scenes']=scene_plan(cfg,args.smoke);cfg['smoke']=args.smoke
    if args.check:print(json.dumps(dict(scenes=len(cfg['scenes']),frames=sum(s['frames'] for s in cfg['scenes']),training=False)));return 0
    check=subprocess.check_output(['powershell','-NoProfile','-Command',"Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'kit.exe' -or ($_.Name -eq 'python.exe' -and $_.CommandLine -match 'IsaacLab|capture_semantic_lidar|capture_segmentation_dataset') } | Select-Object -ExpandProperty ProcessId"],text=True).strip()
    if check:raise RuntimeError('Isaac/Kit process already active: '+check)
    sources=['traffic/segmentation_dataset_v02.py','usd/segmentation_dataset_scene.py','scripts/capture_segmentation_dataset_v02.py','experiments/prepare_segmentation_dataset_v02.py','configs/perception/segmentation_dataset_v02.json','configs/perception/segmentation_dataset_assets_v02.json','scripts/fetch_segmentation_dataset_assets.py']
    output,manifest=run_package('segmentation-dataset/v02',cfg,sources);print('DATASET_RUN='+str(output),flush=True)
    asset_paths=list((ROOT/cfg['vehicle_visual']).parent.rglob('*'))+[ROOT/row['path'] for row in json.loads((ROOT/cfg['human_source_manifest']).read_text())['sources']]
    for row in json.loads((ROOT/'configs/perception/segmentation_dataset_assets_v02.json').read_text())['sources']:
        if sha256(ROOT/row['path'])!=row['sha256']:raise ValueError('Human source asset hash mismatch')
    asset_hashes={str(p.relative_to(ROOT)):sha256(p) for p in asset_paths if p.is_file()};manifest['source_asset_hashes']=asset_hashes
    result=dict(passed=False,training=False,smoke=args.smoke);log=ROOT/'logs'/('segmentation-dataset-'+output.name+'.log');start=time.perf_counter()
    try:
        command=[str(isaac_runtime(ROOT,load_config(ROOT))),str(ROOT/'scripts/capture_segmentation_dataset_v02.py'),'--output',str(output)]
        with log.open('w',encoding='utf-8') as stream:
            process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            try:code=process.wait(timeout=cfg['maximum_capture_seconds'])
            except subprocess.TimeoutExpired:stop_child(process);raise RuntimeError('Bounded capture timeout')
        captured=json.loads((output/'capture-result.json').read_text());dataset=dict(schema_version='segmentation-dataset-v02',classes=CLASSES,frames=captured['frames'],scenes=cfg['scenes'],training_ready=False)
        validation=validate_manifest(dataset,output);dataset.update(validation=validation,training_ready=validation['training_ready'] and captured['passed'],class_counts_by_split=validation['class_counts_by_split'])
        (output/'dataset-manifest.json').write_text(json.dumps(dataset,indent=2))
        result.update(passed=bool(code==0 and captured['passed'] and (args.smoke or dataset['training_ready'])),dataset_validation=validation,log=str(log.relative_to(ROOT)),error=captured.get('error'),manifest='dataset-manifest.json')
    except Exception as e:result['error']=repr(e)
    result['source_changed_during_run']=[p for p,h in manifest['source_hashes'].items() if sha256(ROOT/p)!=h]
    result['assets_changed_during_run']=[p for p,h in asset_hashes.items() if sha256(ROOT/p)!=h]
    result['passed']=bool(result['passed'] and not result['source_changed_during_run'] and not result['assets_changed_during_run'])
    result['wall_seconds']=time.perf_counter()-start;finish(output,manifest,result);print(json.dumps(dict(output=str(output),**result),indent=2));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
