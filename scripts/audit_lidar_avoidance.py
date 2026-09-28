"""Recompute avoidance commands and geometric safety from immutable run evidence."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.lidar_avoidance import AvoidancePlanner,road_points,body_bounds,rectangle_gap,summarize


def audit(path):
    manifest=json.loads((path/'manifest.json').read_text());config=manifest['config']
    if config['mode']!='avoid':raise ValueError('Expected avoidance run')
    for name,wanted in manifest['hashes'].items():
        if hashlib.sha256((path/name).read_bytes()).hexdigest()!=wanted:raise ValueError('Changed artifact: '+name)
    rows=json.loads((path/'telemetry.json').read_text())
    planner=AvoidancePlanner(config['speed'])
    for i,row in enumerate(rows):
        with np.load(path/f'scan_{i:04d}.npz') as scan:
            points,healthy=road_points(scan['az'],scan['el'],scan['ranges'],scan['flags'],row['ego'])
            d=planner.step(points,healthy,row['ego'],fresh=i!=config['fault_step'])
            if int(scan['timestamp'])*1e-9!=row['sensor_time']:raise ValueError('Timestamp mismatch')
            if (int(scan['timestamp'])+int(scan['offset'].min()))*1e-9<row['sensor_epoch']+.1:raise ValueError('Stale scan')
        if row['step']!=i or abs(d.speed-row['requested_speed'])>1e-8:raise ValueError('Command mismatch')
        if d.phase!=row['phase'] or d.reason!=row['reason'] or d.lane_request!=row['lane_request']:raise ValueError('Maneuver mismatch')
        if d.healthy!=row['healthy'] or d.adjacent_clear!=row['adjacent_clear'] or abs(d.clearance-row['lidar_distance'])>1e-8:raise ValueError('Observation mismatch')
        if i:
            if row['ego']!=rows[i-1]['ego_after']:raise ValueError('Pose continuity mismatch')
            if abs(row['traffic_time']-rows[i-1]['traffic_time']-.1)>1e-8 or row['sensor_time']<=rows[i-1]['sensor_time']:raise ValueError('Clock mismatch')
        a,b=body_bounds(row['ego']),body_bounds(row['ego_after'])
        swept=(min(a[0],b[0])-.02,max(a[1],b[1])+.02,min(a[2],b[2])-.02,max(a[3],b[3])+.02)
        gap=min(rectangle_gap(swept,o) for o in manifest['evaluation_obstacles'])
        if abs(gap-row['obstacle_gap'])>1e-8:raise ValueError('Geometric clearance mismatch')
    expected=summarize(rows,config['blocked_lane'])
    summary=json.loads((path/'summary.json').read_text())
    for key,value in expected.items():
        if summary[key]!=value:raise ValueError('Summary mismatch: '+key)
    if len(rows)!=round(config['seconds']/.1):raise ValueError('Incomplete run')
    return dict(run=path.name,verified=True,config=config,summary=summary,
                manifest_sha256=hashlib.sha256((path/'manifest.json').read_bytes()).hexdigest())


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path)
    args=p.parse_args();report=dict(runs=[audit(path) for path in args.runs]);text=json.dumps(report,indent=2)
    if args.output:
        with args.output.open('x') as f:f.write(text+'\n')
    print(text)
    raise SystemExit(0 if all(r['summary']['passed'] for r in report['runs']) else 1)
