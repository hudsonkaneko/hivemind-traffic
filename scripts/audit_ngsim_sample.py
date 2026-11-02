"""Audit a bounded I-80 development sample; never fit models or open audit data."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path


def audit(path: Path, row_cap=100000):
    rows=json.loads(path.read_text(encoding='utf-8'))
    if not rows or len(rows)>=row_cap:
        raise ValueError('Empty or possibly truncated query; do not claim complete window')
    tracks=defaultdict(list);keys=Counter();lanes=Counter();nonfinite=0
    fields=('global_time','local_x','local_y','global_x','global_y','v_length','v_width','v_vel','v_acc','lane_id')
    for row in rows:
        if row['location']!='i-80':raise ValueError('Unexpected location')
        values={key:float(row[key]) for key in fields}
        nonfinite+=sum(not math.isfinite(value) for value in values.values())
        tracks[row['vehicle_id']].append(row)
        keys[(row['vehicle_id'],row['frame_id'],row['global_time'])]+=1
        lanes[row['lane_id']]+=1
    if nonfinite:raise ValueError('Nonfinite source fields')
    times=[int(row['global_time']) for row in rows];start=min(times);end=max(times)
    dt=Counter();reverse=0;transitions=Counter();censored=0;merge_candidates=[]
    for vid,track in tracks.items():
        track.sort(key=lambda r:int(r['global_time']))
        if len(track)<int(track[0]['total_frames']):censored+=1
        for a,b in zip(track,track[1:]):
            delta=int(b['global_time'])-int(a['global_time']);dt[delta]+=1
            reverse+=float(b['local_y'])<float(a['local_y'])
            if a['lane_id']!=b['lane_id']:
                transitions[a['lane_id']+'->'+b['lane_id']]+=1
                if a['lane_id']=='7' and b['lane_id']=='6':
                    merge_candidates.append(dict(vehicle_id=vid,time=int(b['global_time']),
                        sample_offset_s=(int(b['global_time'])-start)/1000,consecutive_100ms=delta==100))
    return dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),rows=len(rows),vehicles=len(tracks),
        first_epoch_ms=start,last_epoch_ms=end,span_s=(end-start)/1000,
        first_utc=datetime.fromtimestamp(start/1000,timezone.utc).isoformat(),
        duplicate_vehicle_frame_time_rows=sum(n-1 for n in keys.values()),nonfinite_values=nonfinite,
        lane_row_counts=dict(lanes),within_vehicle_time_delta_ms=dict(sorted(dt.items())),
        negative_longitudinal_steps=reverse,tracks_incomplete_within_sample=censored,
        lane_transitions=dict(transitions),ramp_to_mainline_transition_candidates=merge_candidates,
        negative_speed_rows=sum(float(r['v_vel'])<0 for r in rows),
        nonpositive_dimension_rows=sum(float(r['v_length'])<=0 or float(r['v_width'])<=0 for r in rows),
        source_velocity_range_m_s=[min(float(r['v_vel']) for r in rows)*.3048,max(float(r['v_vel']) for r in rows)*.3048],
        maximum_absolute_source_acceleration_m_s2=max(abs(float(r['v_acc'])) for r in rows)*.3048,
        units='Local geometry, speed and acceleration converted from feet using 0.3048; global coordinates require the documented State Plane CRS and survey-foot treatment',
        status='Exploratory development quality audit; not model admission or gap-acceptance calibration',
        limitations=['Only an early 120-second query; partial tracks expected at sample boundaries',
            'Transitions are candidates, not independently verified accepted/rejected gap events',
            'Observed entry flow cannot establish requested demand or external queues',
            'Official metadata says original study omitted boundary-incomplete trajectories',
            'API conversion not yet crosschecked against original TXT',
            'Reserved late-period audit rows not accessed'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sample',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Preserve earlier reports; choose a fresh output')
    result=audit(args.sample)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))
