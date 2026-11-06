"""Offline native-trace comparison and exact5fps renderer inputs; no simulation.

Example (paths are explicit, never selected by outcome):
  python scripts/visualize_highway_flow.py --left RECORD_DIR --right RECORD_DIR \
      --left-label "Frozen actor" --right-label "Flow reference" --output FRESH_DIR

Full-route CSV/SVG plots use all original10Hz samples. Render inputs keep only
frames[::2], preserve original poses, and label the temporal subset. This module
deliberately does not import the training runner or change its source inventory.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import csv
import hashlib
from html import escape
import json
import math
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
COLORS=('#2166c2','#d56b12')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def checked_path(value,*,output=False):
    path=Path(value).resolve()
    if not path.is_relative_to(ROOT/'outputs' if output else ROOT):
        raise ValueError('All comparison inputs/outputs must remain in the project; outputs belong under outputs/')
    return path


def load_recording(directory):
    directory=checked_path(directory)
    data=json.loads((directory/'recording.json').read_text(encoding='utf-8'))
    evaluation=json.loads((directory/'evaluation.json').read_text(encoding='utf-8'))
    frames=data.get('frames',[])
    if len(frames)<2 or data.get('source_timestep')!=.1:
        raise ValueError('Expected an original0.1s native recording, not a previous temporal subset')
    if data.get('temporal_subset'):
        raise ValueError('Use the original recording, not a derived subset')
    previous=None
    for frame in frames:
        stamp=float(frame['time'])
        if not math.isfinite(stamp) or (previous is not None and abs(stamp-previous-.1)>1e-6):
            raise ValueError('Original frames must be contiguous exact0.1s samples')
        previous=stamp
        identities=set()
        for vehicle in frame['vehicles']:
            if vehicle['id'] in identities:
                raise ValueError('Duplicate native vehicle identity')
            identities.add(vehicle['id'])
            for key in ('speed','distance_m','acceleration','x','y','z','heading'):
                if not math.isfinite(float(vehicle[key])):
                    raise ValueError('Nonfinite native pose or motion')
    network_sha=sha(directory/'network.net.xml')
    if network_sha!=data['network_sha256']:
        raise ValueError('Recording network copy fails its checksum')
    routes=list((directory/'simulation').rglob('routes.rou.xml'))
    demand_sha=sha(routes[0]) if len(routes)==1 else None
    check_path=directory/'recording-check.json'
    check=json.loads(check_path.read_text(encoding='utf-8')) if check_path.exists() else None
    if check and check.get('recording_sha256') and sha(directory/'recording.json')!=check['recording_sha256']:
        raise ValueError('Original recording checksum disagrees with saved recorder check')
    provenance=dict(directory=directory.relative_to(ROOT).as_posix(),recording_sha256=sha(directory/'recording.json'),
        evaluation_sha256=sha(directory/'evaluation.json'),network_sha256=network_sha,demand_sha256=demand_sha,
        source_sha256=deepcopy(data.get('source_sha256',{})),controller=data.get('controller'),
        policy_sha256=data.get('policy_sha256'),seed=data.get('seed'),recording_check=check,
        primary_horizon_complete=evaluation.get('primary_horizon_complete'),
        accounting_comparison_valid=evaluation.get('accounting_comparison_valid'))
    return dict(directory=directory,data=data,evaluation=evaluation,provenance=provenance)


def trace_for(record,agent):
    return [dict(vehicle,time_s=frame['time']) for frame in record['data']['frames']
        for vehicle in frame['vehicles'] if vehicle['id']==agent]


def summarize_agent(record,agent,label):
    trace=trace_for(record,agent)
    ledger=next((r for r in record['evaluation'].get('accounting',{}).get('trips',[]) if r['vehicle_id']==agent),{})
    result=dict(label=label,controller=record['data'].get('controller'),agent=agent,
        observed=bool(trace),samples=len(trace),ledger_state=ledger.get('state'),
        arrived_at_s=ledger.get('arrived_at'),cohort_boundary_time_s=ledger.get('boundary_time_s'),
        recording_primary_horizon_complete=record['evaluation'].get('primary_horizon_complete'),
        recording_intervention_fraction=record['evaluation'].get('intervention_fraction'))
    if not trace:
        return result
    pairs=[(a,b) for a,b in zip(trace,trace[1:]) if abs(b['time_s']-a['time_s']-.1)<1e-6]
    gaps=sum(abs(b['time_s']-a['time_s']-.1)>=1e-6 for a,b in zip(trace,trace[1:]))
    longest=0.; stop_start=None; prior=None
    for row in trace:
        consecutive=prior is not None and abs(row['time_s']-prior['time_s']-.1)<1e-6
        if row['speed']<.1:
            if stop_start is None or not consecutive:
                stop_start=row['time_s']
            longest=max(longest,row['time_s']-stop_start)
        else:
            stop_start=None
        prior=row
    jerk=[abs(b['acceleration']-a['acceleration'])/.1 for a,b in pairs]
    legal=[b for _,b in pairs if b.get('legal_speed_m_s') is not None]
    result.update(first_observed_s=trace[0]['time_s'],last_observed_s=trace[-1]['time_s'],
        first_native_distance_m=trace[0]['distance_m'],last_native_distance_m=trace[-1]['distance_m'],
        observed_distance_change_m=trace[-1]['distance_m']-trace[0]['distance_m'],
        minimum_speed_m_s=min(r['speed'] for r in trace),sample_mean_speed_m_s=sum(r['speed'] for r in trace)/len(trace),
        stopped_measured_exposure_s=.1*sum(b['speed']<.1 for _,b in pairs),
        longest_observed_stop_span_s=longest,measured_intervals=len(pairs),missing_spans=gaps,
        below_half_legal_measured_exposure_s=.1*sum(r['speed']<.5*r['legal_speed_m_s'] for r in legal) if legal else None,
        mean_abs_measured_jerk_m_s3=sum(jerk)/len(jerk) if jerk else None,
        peak_abs_measured_jerk_m_s3=max(jerk) if jerk else None,
        minimum_acceleration_m_s2=min(r['acceleration'] for r in trace),maximum_acceleration_m_s2=max(r['acceleration'] for r in trace))
    return result


def _segments(trace):
    parts=[]
    for row in trace:
        if not parts or abs(row['time_s']-parts[-1][-1]['time_s']-.1)>1e-6:
            parts.append([])
        parts[-1].append(row)
    return parts


def plot_agent(records,labels,agent,path):
    traces=[trace_for(r,agent) for r in records]
    start=min(r['data']['frames'][0]['time'] for r in records)
    end=max(r['data']['frames'][-1]['time'] for r in records)
    width,height,left,right=1440,1100,105,1380
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#202936}.tick{font-size:16px}.axis{font-size:20px}</style>',
        f'<text x="{left}" y="43" font-size="28" font-weight="bold">{escape(agent)} — full recorded route</text>',
        f'<text x="{left}" y="72" font-size="17">Original 10 Hz SUMO measurements; gaps and departure/arrival boundaries are not filled</text>']
    for i,label in enumerate(labels):
        x=left+i*570
        svg.append(f'<line x1="{x}" y1="103" x2="{x+40}" y2="103" stroke="{COLORS[i]}" stroke-width="4"/>')
        absent=' (not present in recording)' if not traces[i] else ''
        svg.append(f'<text x="{x+50}" y="109" font-size="19">{escape(label+absent)}</text>')
    for panel,(key,title) in enumerate((('speed','Speed (m/s)'),('distance_m','Native traveled distance (m)'),('acceleration','Realized acceleration (m/s²)'))):
        top,bottom=155+panel*285,375+panel*285
        values=[float(row[key]) for trace in traces for row in trace]
        if key=='speed':
            values += [float(row['legal_speed_m_s']) for trace in traces for row in trace if row.get('legal_speed_m_s') is not None]
        lo=min([0.,*values]); hi=max([1.,*values]); pad=(hi-lo)*.06
        lo=lo-pad if lo<0 else 0.; hi+=pad
        x=lambda t:left+(t-start)/(end-start)*(right-left)
        y=lambda v:bottom-(v-lo)/(hi-lo)*(bottom-top)
        svg.append(f'<text x="{left}" y="{top-14}" class="axis" font-weight="bold">{title}</text>')
        for j in range(6):
            value=lo+(hi-lo)*j/5; yy=y(value)
            svg.append(f'<line x1="{left}" y1="{yy:.2f}" x2="{right}" y2="{yy:.2f}" stroke="#e0e5eb"/>')
            svg.append(f'<text x="{left-14}" y="{yy+5:.2f}" text-anchor="end" class="tick">{value:.1f}</text>')
        for j in range(9):
            value=start+(end-start)*j/8; xx=x(value)
            svg.append(f'<line x1="{xx:.2f}" y1="{top}" x2="{xx:.2f}" y2="{bottom}" stroke="#edf0f4"/>')
            svg.append(f'<text x="{xx:.2f}" y="{bottom+25}" text-anchor="middle" class="tick">{value:.1f}</text>')
        for index,trace in enumerate(traces):
            for segment in _segments(trace):
                points=' '.join(f'{x(row["time_s"]):.3f},{y(row[key]):.3f}' for row in segment)
                svg.append(f'<polyline points="{points}" fill="none" stroke="{COLORS[index]}" stroke-width="2.4"/>')
            if key=='speed':
                for segment in _segments([r for r in trace if r.get('legal_speed_m_s') is not None]):
                    points=' '.join(f'{x(r["time_s"]):.3f},{y(r["legal_speed_m_s"]):.3f}' for r in segment)
                    svg.append(f'<polyline points="{points}" fill="none" stroke="{COLORS[index]}" opacity=".45" stroke-dasharray="6 5" stroke-width="1.4"/>')
        svg.append(f'<text x="{(left+right)/2}" y="{bottom+50}" text-anchor="middle" class="tick">Simulation time (s)</text>')
    svg.extend(['<text x="105" y="1045" font-size="17">Dashed speed line: recorded local legal ceiling, when available. Curves stop at last observed pose.</text>',
        '<text x="105" y="1075" font-size="17">These kinematic measurements do not establish physical vehicle or sensor validation.</text>','</svg>'])
    Path(path).write_text('\n'.join(svg)+'\n',encoding='utf-8')


def derive_recording(record,output,start,end):
    output.mkdir(parents=True,exist_ok=False)
    source=record['data']; data=deepcopy(source)
    indices=list(range(0,len(source['frames']),2))
    data['frames']=[deepcopy(source['frames'][i]) for i in indices]
    selected=[f for f in data['frames'] if start-1e-6<=f['time']<=end+1e-6]
    if not selected or len(selected)>300:
        raise ValueError('Shared render window must retain1..300 exact5fps samples')
    derivation=dict(schema='highway_flow_temporal_subset_v01',source_recording=record['provenance']['directory'],
        source_recording_sha256=record['provenance']['recording_sha256'],source_evaluation_sha256=record['provenance']['evaluation_sha256'],
        original_frame_count=len(source['frames']),retained_frame_count=len(indices),source_frame_indices=indices,
        original_interval_s=.1,effective_interval_s=.2,render_fps=5,pose_transformation='none',interpolation=False,
        rule='Exact original frames[::2]; retain all vehicle dictionaries unchanged. Full evaluation is copied unchanged and is not rescored from this temporal subset.')
    data.update(temporal_subset=derivation,effective_frame_interval_s=.2,
        clip_start_time=selected[0]['time'],clip_end_time=selected[-1]['time'],clip_frame_count=len(selected),
        clip_label='Shared explicitly selected comparison window; exact every-other native sample',
        crop_rule='Identical time bounds for both recordings; no event/success-based reselection',crop_event=None)
    write_json(output/'recording.json',data)
    shutil.copyfile(record['directory']/'network.net.xml',output/'network.net.xml')
    shutil.copyfile(record['directory']/'evaluation.json',output/'evaluation.json')
    if (record['directory']/'manifest.json').is_file():
        shutil.copyfile(record['directory']/'manifest.json',output/'original-manifest.json')
    write_json(output/'manifest.json',dict(schema=derivation['schema'],training=False,optimizer_steps=0,
        source_provenance=record['provenance'],temporal_subset=derivation,
        source_sha256=source.get('source_sha256',{}),network_sha256=sha(output/'network.net.xml'),
        full_original_evaluation_sha256=sha(output/'evaluation.json'),recording_sha256=sha(output/'recording.json')))
    return dict(path=output.relative_to(ROOT).as_posix(),fps=5,start_time=selected[0]['time'],
        end_time=selected[-1]['time'],clip_frames=len(selected),recording_sha256=sha(output/'recording.json'),
        full_subset_frames=len(indices),network_sha256=sha(output/'network.net.xml'))


def compare(left,right,output,*,labels=('Frozen actor','Flow reference'),agents=('agent_0','agent_4'),start=None,end=None):
    records=[load_recording(left),load_recording(right)]
    output=checked_path(output,output=True)
    if output.exists() or output==ROOT/'outputs':
        raise ValueError('Choose a fresh comparison output directory')
    a,b=(r['data'] for r in records)
    if a.get('seed')!=b.get('seed') or a.get('selected_case')!=b.get('selected_case') or a['network_sha256']!=b['network_sha256']:
        raise ValueError('Comparison requires the same explicit seed/case/network')
    demand=[r['provenance']['demand_sha256'] for r in records]
    if all(demand) and demand[0]!=demand[1]:
        raise ValueError('Native demand differs between comparison recordings')
    common_start=max(r['data']['frames'][0]['time'] for r in records)
    common_end=min(r['data']['frames'][-1]['time'] for r in records)
    start=common_start if start is None else start
    end=min(common_end,start+30.) if end is None else end
    if not all(math.isfinite(v) for v in (start,end)) or not common_start-1e-6<=start<end<=common_end+1e-6:
        raise ValueError('Choose a shared bounded window inside both original recordings')
    if abs(a['frames'][0]['time']-b['frames'][0]['time'])>1e-6:
        raise ValueError('Every-other sampling requires matched source time origins')
    output.mkdir(parents=True)
    summaries=[summarize_agent(record,agent,label) for record,label in zip(records,labels) for agent in agents]
    fields=list(dict.fromkeys(key for row in summaries for key in row))
    with (output/'summary.csv').open('w',newline='',encoding='utf-8') as handle:
        writer=csv.DictWriter(handle,fieldnames=fields);writer.writeheader();writer.writerows(summaries)
    with (output/'traces.csv').open('w',newline='',encoding='utf-8') as handle:
        fields=('label','agent','time_s','speed_m_s','native_distance_m','acceleration_m_s2','legal_speed_m_s','lane_id')
        writer=csv.DictWriter(handle,fieldnames=fields);writer.writeheader()
        for record,label in zip(records,labels):
            for agent in agents:
                for row in trace_for(record,agent):
                    writer.writerow(dict(label=label,agent=agent,time_s=row['time_s'],speed_m_s=row['speed'],
                        native_distance_m=row['distance_m'],acceleration_m_s2=row['acceleration'],
                        legal_speed_m_s=row.get('legal_speed_m_s'),lane_id=row.get('lane_id')))
    for agent in agents:
        if not agent.replace('_','').isalnum():
            raise ValueError('Agent IDs must be safe filenames')
        plot_agent(records,labels,agent,output/(agent+'-full-route.svg'))
    derived={side:derive_recording(record,output/(side+'-5fps'),start,end) for side,record in zip(('left','right'),records)}
    result=dict(schema='highway_flow_offline_comparison_v01',training=False,optimizer_steps=0,simulation_run=False,
        labels=list(labels),agents=list(agents),sources=[r['provenance'] for r in records],
        visualizer_sha256=sha(__file__),shared_clip_requested=dict(start_time=start,end_time=end),derived_recordings=derived,
        summary=summaries,metrics_source='All original10Hz route observations, including native tail; no interpolation or synthetic terminal pose',
        stopped_definition='Speed<0.1m/s; exposure counts consecutive measured0.1s intervals, longest stop is first-to-last observed elapsed span',
        absent_vehicle='Empty CSV fields/absent plot trace, never fabricated arrival or zero-speed trajectory',
        demand_match_verified=all(demand),accepted=False)
    write_json(output/'comparison.json',result)
    lines=['Offline comparison only; no SUMO, GPU renderer, or optimizer was launched.',
        'Plots and CSVs use all original10Hz frames. Derived recorddirs contain exact frames[::2] at5Hz.',
        'Full original evaluation.json is copied unchanged; it is not computed from the subset.',
        'Default shared clip is the first30 seconds. Explicit bounds apply to both controllers.',
        'For each derived recorddir use scripts/render_i110_merge_replay.py with --fps5 (written as --fps 5),',
        '--focal-agent agent_0 or agent_4, --simready-main, explicit --ffmpeg PATH, and a fresh --output.',
        'Read source recording checks and comparison.json before describing a failed/incomplete source as accepted.']
    (output/'README.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left',type=Path,required=True);parser.add_argument('--right',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--left-label',default='Frozen actor');parser.add_argument('--right-label',default='Flow reference')
    parser.add_argument('--agents',nargs='+',default=['agent_0','agent_4'])
    parser.add_argument('--start-time',type=float);parser.add_argument('--end-time',type=float)
    args=parser.parse_args(argv)
    if not Path.cwd().resolve().is_relative_to(ROOT):
        raise ValueError('Run from the canonical project checkout')
    result=compare(args.left,args.right,args.output,labels=(args.left_label,args.right_label),agents=args.agents,
        start=args.start_time,end=args.end_time)
    print(json.dumps(dict(output=str(args.output.resolve()),agents=result['agents'],derived=result['derived_recordings'])))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
