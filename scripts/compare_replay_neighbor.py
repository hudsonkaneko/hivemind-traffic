"""Offline geometry-selected adjacent-car comparison; no simulator or learning."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
MPH = 2.2369362920544


def adjacent(a, b):
    edge_a, lane_a = a['lane_id'].rsplit('_', 1)
    edge_b, lane_b = b['lane_id'].rsplit('_', 1)
    return edge_a == edge_b and abs(int(lane_a)-int(lane_b)) == 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); source=args.recording.resolve(); out=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not out.is_relative_to(ROOT/'outputs') or out.exists():
        raise ValueError('Use project root and a fresh output under outputs/')
    data=json.loads(source.read_text());frames=data['frames'];ego='agent_0'
    for frame in frames:
        main=next((v for v in frame['vehicles'] if v['id']==ego),None)
        if main and main['lane_id'].startswith('sb'):
            candidates=[v for v in frame['vehicles'] if not v['controlled'] and adjacent(main,v)
                        and v['lane_identity']['kind']=='through']
            if not candidates:raise ValueError('No native adjacent car at first highway entry')
            peer=min(candidates,key=lambda v:(math.hypot(v['x']-main['x'],v['y']-main['y']),v['id']))
            entry=frame['time']; selection=dict(time_s=entry,ego_lane=main['lane_id'],peer_lane=peer['lane_id'],
                distance_m=math.hypot(peer['x']-main['x'],peer['y']-main['y']),
                lane_station_offset_m=peer['lane_position_m']-main['lane_position_m'],
                rule='Nearest noncontrolled adjacent through-lane vehicle at first external highway entry; no performance-based selection')
            peer=peer['id'];break
    else:raise ValueError('No highway entry')
    rows=[]; previous={}; initial_adjacent_end=None; interval_open=True
    for frame in frames:
        cars={v['id']:v for v in frame['vehicles']}
        both=ego in cars and peer in cars
        side=both and adjacent(cars[ego],cars[peer])
        if frame['time']>=entry and interval_open:
            if side:initial_adjacent_end=frame['time']
            else:interval_open=False
        for identity in (ego,peer):
            if identity not in cars:continue
            car=cars[identity]; old=previous.get(identity)
            jerk=(car['acceleration']-old[1])/(frame['time']-old[0]) if old else None
            rows.append(dict(time_s=frame['time'],vehicle=identity,controlled=car['controlled'],
                speed_m_s=car['speed'],speed_mph=car['speed']*MPH,acceleration_m_s2=car['acceleration'],
                jerk_m_s3=jerk,lane_id=car['lane_id'],legal_limit_mph=car['legal_speed_m_s']*MPH,
                native_allowed_mph=car['native_allowed_speed_m_s']*MPH,adjacent_to_comparator=side))
            previous[identity]=(frame['time'],car['acceleration'])
    shared_end=min(max(r['time_s'] for r in rows if r['vehicle']==v) for v in (ego,peer))
    def metrics(identity,start,end):
        subset=[r for r in rows if r['vehicle']==identity and start-1e-8<=r['time_s']<=end+1e-8]
        speed=np.array([r['speed_mph'] for r in subset]);acc=np.array([r['acceleration_m_s2'] for r in subset])
        # Exclude the derivative that begins before this comparison window.
        jerk=np.diff(acc)/np.diff([r['time_s'] for r in subset])
        return dict(samples=len(subset),start_speed_mph=float(speed[0]),end_speed_mph=float(speed[-1]),
            mean_speed_mph=float(speed.mean()),min_speed_mph=float(speed.min()),max_speed_mph=float(speed.max()),
            mean_abs_acceleration_m_s2=float(abs(acc).mean()),mean_abs_jerk_m_s3=float(abs(jerk).mean()),
            p95_abs_jerk_m_s3=float(np.percentile(abs(jerk),95)),peak_abs_jerk_m_s3=float(abs(jerk).max()),
            first_35_mph_s=next((r['time_s'] for r in subset if r['speed_mph']>=35),None))
    routes=next(source.parent.glob('episode-*/routes.rou.xml'));tree=ET.parse(routes)
    types={v.get('id'):v.attrib for v in tree.iter('vType')}
    profiles={v.get('id'):dict(vehicle=v.attrib,driver_type=types[v.get('type')])
              for v in tree.iter('vehicle') if v.get('id') in (ego,peer)}
    result=dict(training=False,simulator_runs=0,ego=ego,peer=peer,selection=selection,
        first_adjacent_window_s=[entry,initial_adjacent_end],shared_recording_end_s=shared_end,profiles=profiles,
        windows={name:dict(start_s=start,end_s=end,vehicles={v:metrics(v,start,end) for v in (ego,peer)})
                 for name,start,end in [('adjacent_lanes',entry,initial_adjacent_end),('first_25_seconds_after_start',entry,25.),('shared_after_entry',entry,shared_end)]},
        input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),input_path=str(source.relative_to(ROOT)),
        routes_sha256=hashlib.sha256(routes.read_bytes()).hexdigest(),
        interpretation='Descriptive same-replay comparison, not a matched-condition or causal policy test. Ego merges; peer already cruises, has a different initial speed and speed factor, and later becomes its leader. Native motion may interact with policy traffic.')
    out.mkdir(parents=True);shutil.copy2(__file__,out/'analysis-source.py')
    (out/'comparison.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    with (out/'traces.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,1,figsize=(13,7.2),sharex=True,constrained_layout=True)
    fig.suptitle('Main MAPPO car vs the native car alongside it\nSame saved I-110 replay; no smoothing',fontsize=17)
    for identity,color,label in [(ego,'#2563eb','Main car - frozen MAPPO'),(peer,'#15803d','Adjacent car main_8 - native SUMO IDM')]:
        subset=[r for r in rows if r['vehicle']==identity and r['time_s']<=shared_end]
        t=[r['time_s'] for r in subset]
        axes[0].plot(t,[r['speed_mph'] for r in subset],color=color,lw=2.5,label=label)
        axes[1].plot(t,[r['acceleration_m_s2'] for r in subset],color=color,lw=2)
    axes[0].axhline(18*MPH,color='#6b7280',ls='--',lw=1,label='Highway limit: 40.3 mph')
    axes[0].axhline(35,color='#c4c4c4',ls=':',lw=1)
    axes[0].set_ylim(-1,44);axes[0].set_ylabel('Speed (mph)');axes[0].legend(loc='lower left',fontsize=10)
    axes[1].axhline(0,color='#6b7280',lw=.8);axes[1].set_ylabel('Acceleration (m/s2)');axes[1].set_xlabel('Simulation time (seconds)')
    for ax in axes:
        ax.axvspan(entry,initial_adjacent_end,color='#94a3b8',alpha=.15)
        ax.axvline(entry,color='#64748b',lw=.8,ls=':');ax.grid(alpha=.16);ax.set_xlim(.1,shared_end)
    axes[0].text(entry+.4,42,'Initially adjacent lanes',fontsize=10,color='#475569')
    fig.savefig(out/'speed-comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({k:result[k] for k in ['selection','first_adjacent_window_s','windows']},indent=2))


if __name__=='__main__':main()
