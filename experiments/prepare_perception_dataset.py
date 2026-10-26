"""Bounded native labeled-data preparation. Never starts training."""
import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path
from experiments.probe_support import ROOT,run_package,finish
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime,load_config
from traffic.perception_dataset import write_manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-limit',type=int,default=6,choices=range(1,7))
    args=parser.parse_args()
    if Path.cwd().resolve()!=ROOT:raise RuntimeError('Use project root')
    if os.name=='nt' and subprocess.check_output(['powershell','-NoProfile','-Command',"Get-Process | Where-Object ProcessName -eq 'kit' | Select-Object -ExpandProperty Id"],text=True).strip():
        raise RuntimeError('Another Kit process is active')
    cfg=json.loads((ROOT/'configs/perception/dataset_smoke_v01.json').read_text())
    expected=dict(training_enabled=False,camera='CameraFront',resolution_wh=[640,360],capture_hz=10,target_frames_per_scene=10)
    if any(cfg.get(key)!=value for key,value in expected.items()) or cfg['limits']['max_total_frames']!=60:
        raise ValueError('Unsupported smoke settings: native capture requires '+str(expected)+' and max_total_frames=60')
    cfg['scenes']=cfg['scenes'][:args.scene_limit]
    sources=['scripts/capture_perception_scene.py','experiments/prepare_perception_dataset.py',
        'traffic/perception_dataset.py','traffic/native_sensor_capture.py','traffic/physical_lidar.py',
        'traffic/rendered_physics_session.py','usd/showcase_vehicle.py','usd/physical_scene.py',
        'visualization/physics_road_view.py','configs/perception/dataset_smoke_v01.json']
    output,manifest=run_package('perception-dataset/v01',cfg,sources)
    runtime=isaac_runtime(ROOT,load_config(ROOT));frames=[];runs=[];child=None
    result=dict(passed=False,training=False)
    try:
        for scene in cfg['scenes']:
            directory=output/scene['scene_id'];directory.mkdir()
            resolved=dict(scene=scene,source_vehicle='vehicles/sim_ready/americano-i7-ev/'+cfg['source_vehicle_version'],training=False)
            (directory/'resolved-config.json').write_text(json.dumps(resolved,indent=2))
            log_path=ROOT/'logs/perception-dataset'/f'{output.name}-{scene["scene_id"]}.log';log_path.parent.mkdir(parents=True,exist_ok=True)
            command=[str(runtime),str(ROOT/'scripts/capture_perception_scene.py'),'--output',str(directory)]
            with log_path.open('w',encoding='utf-8') as log:
                child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                child.wait(timeout=180)
            report=json.loads((directory/'capture-result.json').read_text())
            runs.append(dict(scene_id=scene['scene_id'],passed=report['passed'],log=str(log_path.relative_to(ROOT)),command=command))
            if child.returncode or not report['passed']:raise RuntimeError('Capture failed: '+scene['scene_id']+' '+str(report.get('error')))
            for frame in report['frames']:
                for field in ('rgb_path','semantic_path','lidar_path','labels_path'):frame[field]=scene['scene_id']+'/'+frame[field]
                frames.append(frame)
        (output/'manifest.json').rename(output/'initial-run-manifest.json')
        dataset=write_manifest(output,frames,dict(requested_classes=cfg['requested_classes'],
            purpose='Capture/label pipeline smoke sample, not training-ready',source_vehicle_version=cfg['source_vehicle_version'],
            scene_runs=runs,calibration_source='Per-scene capture-result.json',lidar_coordinate_frame='WORLD NONCOMPENSATED',
            limitations=['No pedestrian asset: crossing target is labeled obstacle','Only six short road fixtures; no full training diversity','No learned model or training started']))
        result.update(passed=True,dataset_validation=dataset['validation'],scene_runs=runs)
    except Exception as error:result['error']=repr(error)
    finally:
        if child and child.poll() is None:stop_child(child)
        changed=[p for p,h in manifest['source_hashes'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['source_changed_during_run']=changed;result['passed']=bool(result.get('passed') and not changed)
        # run_package uses manifest.json; preserve dataset manifest separately before finish.
        if (output/'manifest.json').exists() and 'frames' in json.loads((output/'manifest.json').read_text()):
            (output/'manifest.json').rename(output/'dataset-manifest.json')
        finish(output,manifest,result)
    print('DATASET_RUN='+str(output));print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
