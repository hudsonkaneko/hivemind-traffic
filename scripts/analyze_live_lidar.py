"""Offline audit of raw live lidar scans, controller decisions and trace hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.lidar_control import forward_clearance,target_speed


def evaluate(path):
    manifest=json.loads((path/'manifest.json').read_text())
    for name,wanted in manifest['hashes'].items():
        if hashlib.sha256((path/name).read_bytes()).hexdigest()!=wanted:
            raise ValueError('Changed artifact: '+name)
    rows=json.loads((path/'telemetry.json').read_text());config=manifest['config']
    for i,row in enumerate(rows):
        if row['step'] != i:
            raise ValueError('Step numbering mismatch')
        with np.load(path/f'scan_{i:04d}.npz') as scan:
            clearance=forward_clearance(scan['az'],scan['el'],scan['ranges'],scan['flags'])
            if abs(clearance.distance-row['lidar_distance'])>1e-8: raise ValueError('Clearance mismatch')
            if clearance.healthy!=row['healthy'] or clearance.points!=row['points']: raise ValueError('Health/ROI mismatch')
            if int(scan['timestamp'])*1e-9 != row['sensor_time']: raise ValueError('Scan timestamp mismatch')
            if (int(scan['timestamp'])+int(scan['offset'].min()))*1e-9<row['sensor_epoch']+.1:
                raise ValueError('Scan predates pose settling interval')
        requested,reason=target_speed(clearance,row['speed'],cruise=config['speed'],fresh=i!=config['fault_step'])
        if abs(requested-row['requested_speed'])>1e-8 or reason!=row['reason']: raise ValueError('Control mismatch')
        if i and (abs(row['traffic_time']-rows[i-1]['traffic_time']-.1)>1e-8 or row['sensor_time']<=rows[i-1]['sensor_time']):
            raise ValueError('Clock ordering failed')
        if abs(row['error_m']-abs(row['lidar_distance']-row['true_gap']))>1e-8:
            raise ValueError('Distance error mismatch')
        if i:
            previous=rows[i-1]
            if abs(row['speed']-previous['realized_speed'])>1e-8:
                raise ValueError('Speed continuity mismatch')
            if abs(row['ego_x']-previous['ego_x']-previous['realized_speed']*.1)>1e-8:
                raise ValueError('Position integration mismatch')
            if abs(row['true_gap']-previous['post_gap'])>1e-8:
                raise ValueError('Gap continuity mismatch')
    s=json.loads((path/'summary.json').read_text())
    if not rows: raise ValueError('No completed steps to verify')
    expected=dict(steps=len(rows),min_gap_m=min(r['post_gap'] for r in rows),
        progress_m=rows[-1]['ego_x']+rows[-1]['realized_speed']*.1-rows[0]['ego_x'],
        max_clearance_error_m=max(abs(r['lidar_distance']-r['true_gap']) for r in rows),
        final_speed=rows[-1]['realized_speed'],speed_interventions=sum(abs(r['realized_speed']-r['requested_speed'])>.15 for r in rows),
        sensor_fault_actions=sum(r['reason']=='sensor-failsafe' for r in rows))
    for k,v in expected.items():
        if abs(s[k]-v)>1e-8: raise ValueError('Summary mismatch: '+k)
    accepted=(s['min_gap_m']>=2 and s['max_clearance_error_m']<=.25 and s['progress_m']>=5
        and s['final_speed']<.2 and s['speed_interventions']==0 and not any(r['collisions'] for r in rows)
        and all(r['healthy'] for r in rows) and len(rows)==round(config['seconds']/.1))
    if bool(accepted)!=s['passed']: raise ValueError('Acceptance mismatch')
    return dict(run=path.name,verified=True,config=config,summary=s,
                manifest_sha256=hashlib.sha256((path/'manifest.json').read_bytes()).hexdigest())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();report={'runs':[evaluate(r) for r in a.runs]};text=json.dumps(report,indent=2)
    if a.output:
        with a.output.open('x') as f:f.write(text+'\n')
    print(text)
