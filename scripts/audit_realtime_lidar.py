"""Recompute real-time lidar decisions and gates from hashed raw run evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.realtime_lidar import packet_points, summarize_realtime
from traffic.lidar_avoidance import AvoidancePlanner, body_bounds, rectangle_gap


def audit(directory):
    root=Path(directory);manifest=json.loads((root/'manifest.json').read_text())
    config=manifest['config'];rows=json.loads((root/'telemetry.json').read_text())
    summary=json.loads((root/'summary.json').read_text())
    contract=json.loads((root/'timing-contract.json').read_text())
    assert config['realtime'] and len(rows)==round(config['seconds']*10),'Incomplete realtime run'
    assert manifest.get('hashes'),'Missing evidence hashes'
    for name,digest in manifest['hashes'].items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,'Hash mismatch: '+name
    planner=AvoidancePlanner(config['speed']);previous=-1;alignment_max=0.
    for i,row in enumerate(rows):
        with np.load(root/f'continuous_{i:04d}.npz') as raw:
            points,healthy,age=packet_points(raw,row['ego'],row['sensor_time'])
            stamp=int(raw['timestamp'])
        fresh=healthy and -.001<=row.get('receipt_age',0)<=.25 and i!=config['fault_step']
        assert healthy==row['healthy'] and fresh==row['fresh']
        assert abs(age-row['sensor_age'])<1e-8 and stamp==row['packet_timestamp']
        assert abs(row['traffic_time']-contract['traffic_origin']-i*.1)<1e-7
        assert abs(row['sensor_time']-contract['sensor_origin']-i*.1)<1e-7
        assert stamp>=previous,'Regressed packet time'
        decision=planner.step(points if stamp>previous else np.empty((0,2)),healthy,row['ego'],fresh=fresh)
        previous=stamp
        assert decision.phase==row['phase'] and decision.lane_request==row['lane_request']
        assert decision.reason==row['reason'] and abs(decision.speed-row['requested_speed'])<1e-8
        assert abs(decision.clearance-row['lidar_distance'])<1e-6
        if i: assert rows[i-1]['ego_after']==row['ego'],'Discontinuous vehicle state'
        a,b=body_bounds(row['ego']),body_bounds(row['ego_after'])
        swept=(min(a[0],b[0])-.02,max(a[1],b[1])+.02,min(a[2],b[2])-.02,max(a[3],b[3])+.02)
        obstacles=manifest['evaluation_obstacles']
        gap=min(rectangle_gap(swept,o) for o in obstacles)
        assert abs(gap-row['obstacle_gap'])<1e-8
        evaluate_alignment=bool(len(points)) if contract['version']==1 else bool(len(points) and fresh and row['is_new'])
        if contract['version']>=2: assert evaluate_alignment==row['alignment_evaluated']
        if evaluate_alignment:
            distances=[]
            for xmin,xmax,ymin,ymax in obstacles:
                distances.append(np.hypot(np.maximum(np.maximum(xmin-points[:,0],points[:,0]-xmax),0),
                    np.maximum(np.maximum(ymin-points[:,1],points[:,1]-ymax),0)))
            alignment=float(np.quantile(np.min(distances,axis=0),.95))
            assert abs(alignment-row['alignment_p95_m'])<1e-7
            alignment_max=max(alignment_max,alignment)
    computed=summarize_realtime(rows,config['blocked_lane'],config['dropout_step'],summary['loop_wall_seconds'],alignment_max)
    assert computed['passed']==summary['passed'],'Summary gate mismatch'
    return dict(run=root.name,integrity_passed=True,**computed)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run');args=p.parse_args()
    result=audit(args.run);print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
