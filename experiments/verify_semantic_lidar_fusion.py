"""Bounded static-then-moving native fusion validation. Never starts training."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

from experiments.probe_support import ROOT, run_package, finish
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config

CONFIG = ROOT / 'configs/perception/semantic_lidar_fusion_v01.json'


def validate_config(config):
    if config.get('training') is not False or config.get('version') != 'semantic-lidar-fusion-v01':
        raise ValueError('Explicit no-training fusion configuration required')
    if type(config['frames']) is not int or not 4 <= config['frames'] <= 12:
        raise ValueError('Capture must remain bounded to 4..12 frames')
    if not .5 <= config['capture_start_s'] <= 2 or not .05 <= config['frame_stride_s'] <= .2:
        raise ValueError('Unsupported capture timing')
    if config['capture_start_s'] + (config['frames'] - 1) * config['frame_stride_s'] > 4:
        raise ValueError('Capture duration exceeds bounded fixture')
    if any(abs(value * 20 - round(value * 20)) > 1e-8
           for value in (config['capture_start_s'], config['frame_stride_s'])):
        raise ValueError('Capture timing must align with the 20 Hz render clock')
    if abs(config['frame_stride_s'] * 10 - round(config['frame_stride_s'] * 10)) > 1e-8:
        raise ValueError('Capture timing must also align with the 10 Hz camera clock')
    if not 30 <= config['max_wall_seconds_per_case'] <= 300:
        raise ValueError('Native process budget must be 30..300 seconds')
    source=(ROOT/config['source_vehicle']).resolve()
    if not source.is_relative_to(ROOT/'vehicles/sim_ready') or not source.is_dir():
        raise ValueError('Prepared source must exist inside project vehicles/sim_ready')
    if not config['cases'] or config['cases'][0]['mode'] != 'static':
        raise ValueError('Static validation must precede moving traffic')
    if len({c['scene_id'] for c in config['cases']}) != len(config['cases']):
        raise ValueError('Scene identities must be unique')
    for case in config['cases']:
        if case['mode'] not in ('static','moving') or not 0 <= case['moving_speed_m_s'] <= 10:
            raise ValueError('Unsupported bounded motion fixture')
        if not re.fullmatch('[a-z][a-z0-9-]{1,63}',case['scene_id']):
            raise ValueError('Invalid scene identity')
    json.dumps(config, allow_nan=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=CONFIG)
    parser.add_argument('--phase',choices=('static','all'),default='all')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT: raise RuntimeError('Use project root')
    config=json.loads(args.config.read_text(encoding='utf-8'));validate_config(config)
    if args.check:
        print(json.dumps(config,indent=2));return 0
    if os.name=='nt':
        busy=subprocess.check_output(['powershell','-NoProfile','-Command',
            "Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'kit.exe' -or ($_.Name -eq 'python.exe' -and $_.ExecutablePath -like 'C:\\isaacsim\\*') } | Select-Object -ExpandProperty ProcessId"],text=True)
        if busy.strip(): raise RuntimeError('Another Isaac/Kit process is active; do not overlap GPU validation')
    sources=['experiments/verify_semantic_lidar_fusion.py','scripts/capture_semantic_lidar_fusion.py',
             'scripts/evaluate_semantic_lidar_capture.py',
             'traffic/semantic_lidar_capture.py','traffic/semantic_lidar_fusion.py','traffic/semantic_lidar_evaluation.py',
             'configs/perception/semantic_lidar_fusion_v01.json','traffic/native_sensor_capture.py',
             'traffic/physical_lidar.py','traffic/rendered_physics_session.py','usd/showcase_vehicle.py',
             'usd/physical_scene.py','visualization/physics_road_view.py']
    config['cases']=[c for c in config['cases'] if args.phase=='all' or c['mode']=='static']
    output,manifest=run_package('semantic-lidar-fusion/v01',config,sources)
    print('FUSION_RUN='+str(output),flush=True)
    runtime=isaac_runtime(ROOT,load_config(ROOT));child=None;cases=[];started=time.monotonic()
    result=dict(passed=False,training=False,phase=args.phase,cases=cases)
    try:
        for case in config['cases']:
            directory=output/case['scene_id'];directory.mkdir()
            resolved={k:config[k] for k in ('training','source_vehicle','frames','capture_start_s','frame_stride_s')}
            resolved.update(case)
            (directory/'resolved-config.json').write_text(json.dumps(resolved,indent=2)+'\n')
            log_path=ROOT/'logs/semantic-lidar-fusion'/f'{output.name}-{case["scene_id"]}.log'
            log_path.parent.mkdir(parents=True,exist_ok=True)
            command=[str(runtime),str(ROOT/'scripts/capture_semantic_lidar_fusion.py'),'--output',str(directory)]
            row=dict(scene_id=case['scene_id'],mode=case['mode'],moving_speed_m_s=case['moving_speed_m_s'],
                     command=command,log=str(log_path.relative_to(ROOT)),passed=False)
            cases.append(row)
            with log_path.open('w',encoding='utf-8') as log:
                child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                child.wait(timeout=config['max_wall_seconds_per_case'])
            report=json.loads((directory/'capture-result.json').read_text())
            row.update(capture_passed=report.get('passed',False),exit_code=child.returncode)
            if child.returncode or not report.get('passed'):
                raise RuntimeError('Native capture failed: '+case['scene_id']+' '+str(report.get('error')))
            gates=dict(config['gates'],minimum_accuracy=case['minimum_accuracy'])
            (directory/'evaluation-config.json').write_text(json.dumps(dict(fusion=config['fusion'],gates=gates),indent=2)+'\n')
            # Pillow stays in the external Isaac runtime; traffic dependencies
            # remain isolated. This second process imports no SimulationApp.
            evaluate_command=[str(runtime),str(ROOT/'scripts/evaluate_semantic_lidar_capture.py'),'--capture',str(directory)]
            with log_path.open('a',encoding='utf-8') as log:
                child=subprocess.Popen(evaluate_command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                child.wait(timeout=120)
            evaluation=json.loads((directory/'fusion'/'evaluation.json').read_text())
            row.update(passed=bool(evaluation['passed'] and child.returncode == 0),
                       evaluation_exit_code=child.returncode,evaluation=evaluation)
            if not row['passed']:
                raise RuntimeError('Fusion gates failed: '+case['scene_id'])
        result['passed']=bool(cases) and all(c['passed'] for c in cases)
    except Exception as error:
        result['error']=repr(error)
    finally:
        if child is not None and child.poll() is None: stop_child(child)
        result['wall_seconds']=time.monotonic()-started
        result['source_changed_during_run']=[p for p,h in manifest['source_hashes'].items()
                                            if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['passed']=bool(result['passed'] and not result['source_changed_during_run'])
        finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),passed=result['passed'],training=False,error=result.get('error'))))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
