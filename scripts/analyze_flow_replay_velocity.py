"""Offline speed/command attribution from recorded native motion and action logs."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
MPH = 2.2369362920544


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--agent', default='agent_0')
    args = parser.parse_args()
    folder, output = args.recording.resolve(), args.output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs') or output.exists():
        raise ValueError('Use project root and a fresh output under outputs/')
    data = json.loads((folder/'recording.json').read_text())
    cfg = json.loads((folder/'resolved-config.json').read_text())
    case = next(row for row in cfg['validation_rows'] if row['seed'] == data['seed'])
    episode_cap = case['speed_limit_m_s']
    journal = next(folder.glob('episode/*/actions.jsonl.gz'))
    commands = {}
    with gzip.open(journal, 'rt') as stream:
        for line in stream:
            record = json.loads(line)
            if args.agent in record['agents']:
                commands[round(record['time'], 6)] = record['agents'][args.agent]
    lanes = {lane.attrib['id']: float(lane.attrib['speed'])
             for lane in ET.parse(folder/'network.net.xml').iter('lane')}
    rows, previous = [], None
    for frame in data['frames']:
        car = next((v for v in frame['vehicles'] if v['id'] == args.agent), None)
        if car is None:
            continue
        t = frame['time']
        if previous and round(t, 6) in commands:
            pt, old = previous
            dt = t-pt
            a = commands[round(t, 6)]
            context = a['flow_context']; own = context['corridors']['own']
            front = own['front'] or {}; entry = context['entry']
            req, filt, realized = (a[k] for k in ('requested_acceleration', 'filtered_acceleration', 'realized_acceleration'))
            row = dict(time_s=t, command_start_s=pt, dt_s=dt,
                speed_m_s=car['speed'], speed_mph=car['speed']*MPH, previous_speed_m_s=old['speed'],
                delta_speed_m_s=car['speed']-old['speed'],
                vx_m_s=(car['x']-old['x'])/dt, vy_m_s=(car['y']-old['y'])/dt,
                x=car['x'], y=car['y'], lane=car['lane_id'], previous_lane=old['lane_id'],
                network_speed_m_s=lanes[car['lane_id']],
                legal_speed_m_s=min(lanes[car['lane_id']], episode_cap, 26.8224),
                requested_acceleration=req, filtered_acceleration=filt, realized_acceleration=realized,
                speed_derivative_m_s2=(car['speed']-old['speed'])/dt,
                policy_delta_v=req*dt, filter_delta_v=(filt-req)*dt, native_delta_v=(realized-filt)*dt,
                override=bool(a['override']), native_changed=bool(a['native_realization_changed']),
                filter_changed=bool(a['policy_filter_changed']), causes=';'.join(a['override_causes']),
                front_id=front.get('id', ''), front_gap_m=front.get('bumper_gap_m', np.nan),
                front_relative_speed=front.get('relative_speed_m_s', np.nan),
                pre_route_reachable=own['route']['reachable'], pre_route_remaining_m=own['route']['distance_m'],
                pre_native_estimate=own['native_response_estimate_m_s2'],
                pre_entry_distance_m=entry['distance_to_start_m'] if entry['entry_active'] else np.nan,
                pre_entry_active=entry['entry_active'], lane_request=a['lane_request'],
                maneuver=a['maneuver']['in_progress'], ceiling=a['feasible_acceleration_ceiling'])
            rows.append(row)
        previous = (t, car)
    output.mkdir(parents=True)
    shutil.copy2(__file__, output/'analysis-source.py')
    with (output/'velocity-commands.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    series = lambda key: np.array([r[key] for r in rows], dtype=float)
    def corr(x, y, mask=None):
        x, y = series(x), series(y)
        valid = np.isfinite(x)&np.isfinite(y)
        if mask is not None: valid &= mask
        if valid.sum() < 3 or np.std(x[valid]) < 1e-9 or np.std(y[valid]) < 1e-9: return None
        return dict(pearson_r=float(np.corrcoef(x[valid], y[valid])[0, 1]), n=int(valid.sum()))
    # A contiguous observed speed loss; retain only losses greater than 0.25 mph.
    groups, current = [], []
    for row in rows + [dict(delta_speed_m_s=0.)]:
        if row['delta_speed_m_s'] < -1e-8:
            current.append(row)
        elif current:
            if -sum(r['delta_speed_m_s'] for r in current)*MPH > .25:
                groups.append(current)
            current = []
    slowdowns = []
    for g in groups:
        slowdowns.append(dict(start_s=g[0]['command_start_s'], end_s=g[-1]['time_s'],
            start_mph=g[0]['previous_speed_m_s']*MPH, end_mph=g[-1]['speed_mph'],
            min_requested_acceleration=min(r['requested_acceleration'] for r in g),
            max_requested_acceleration=max(r['requested_acceleration'] for r in g),
            min_realized_acceleration=min(r['realized_acceleration'] for r in g),
            overrides=sum(r['override'] for r in g), native_changes=sum(r['native_changed'] for r in g),
            filter_changes=sum(r['filter_changed'] for r in g),
            policy_delta_v_m_s=sum(r['policy_delta_v'] for r in g),
            filter_delta_v_m_s=sum(r['filter_delta_v'] for r in g),
            native_delta_v_m_s=sum(r['native_delta_v'] for r in g),
            front_present_steps=sum(bool(r['front_id']) for r in g),
            front_gap_min_m=min((r['front_gap_m'] for r in g if r['front_id']), default=None),
            front_gap_max_m=max((r['front_gap_m'] for r in g if r['front_id']), default=None),
            front_relative_speed_min=min((r['front_relative_speed'] for r in g if r['front_id']), default=None),
            lane_requests=sorted(set(r['lane_request'] for r in g)),
            lanes=sorted(set(r['lane'] for r in g))))
    no_override = ~series('override').astype(bool)
    summary = dict(training=False, simulation_runs=0, samples=len(rows), agent=args.agent,
        policy_sha256=data['policy_sha256'], episode_speed_cap_m_s=episode_cap,
        inputs={str(p.relative_to(ROOT)):digest(p) for p in [folder/'recording.json',journal,folder/'network.net.xml',folder/'resolved-config.json']},
        time_alignment='Row at t contains command/context from t-0.1 and realized speed/acceleration at t.',
        velocity_definition='x/y finite differences of recorded SUMO front-bumper position; includes lateral motion and geometry/reference effects.',
        maximum_speed_derivative_acceleration_error=float(np.max(np.abs(series('speed_derivative_m_s2')-series('realized_acceleration')))),
        decomposition_error=float(np.max(np.abs(series('delta_speed_m_s')-series('policy_delta_v')-series('filter_delta_v')-series('native_delta_v')))),
        correlation_request_realized=corr('requested_acceleration','realized_acceleration'),
        correlation_request_realized_without_overrides=corr('requested_acceleration','realized_acceleration',no_override),
        context_correlations={key:corr(key,'requested_acceleration') for key in
            ['previous_speed_m_s','front_gap_m','front_relative_speed','pre_native_estimate','pre_route_remaining_m','pre_entry_distance_m']},
        slowdowns=slowdowns,
        limitations=['Single development replay; descriptive correlations are not causal explanations of network features.',
            'Journal records command stages and partial context, not all 117 actor inputs; no feature ablation inferred.',
            'Native intervention reason is unattributed in original logs; network inspection can suggest, not prove, its specific cause.'])
    (output/'analysis.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    t=series('time_s')
    for end, name in [(25.,'merge-speed-policy'),(40.,'full-route-speed-policy')]:
        fig, axes = plt.subplots(4,1,figsize=(13,11),sharex=True,constrained_layout=True)
        fig.suptitle('I-110 | final MAPPO round two | SimReady main car (agent_0)\nRecorded motion and commands; no new simulation',fontsize=16)
        axes[0].plot(t,series('speed_mph'),color='#0969da',lw=2,label='Actual speed')
        axes[0].plot(t,series('legal_speed_m_s')*MPH,color='#777777',ls='--',label='Effective limit (lane + episode cap)')
        axes[0].set_ylabel('Speed (mph)'); axes[0].legend(loc='lower right')
        for key,color,label,ls in [('requested_acceleration','#e67e22','Policy request','-'),('filtered_acceleration','#a855f7','After safety filter',':'),('realized_acceleration','#0969da','Actual acceleration','--')]:
            axes[1].plot(t,series(key),color=color,label=label,ls=ls,lw=1.8)
        axes[1].axhline(0,color='#999999',lw=.7); axes[1].set_ylabel('Acceleration (m/s2)'); axes[1].legend(ncol=3,loc='lower right')
        axes[2].plot(t-.1,series('front_gap_m'),color='#00875a',label='Same-lane front gap (no line = no leader)')
        axes[2].set_ylabel('Gap (m)'); axes[2].legend(loc='upper left')
        axes[3].plot(t,series('vx_m_s'),label='East/west velocity vx',color='#8b5cf6')
        axes[3].plot(t,series('vy_m_s'),label='North/south velocity vy',color='#0891b2')
        axes[3].set_ylabel('Velocity (m/s)'); axes[3].set_xlabel('Simulation time (s)'); axes[3].legend(loc='lower left')
        for ax in axes:
            ax.axvspan(2.4,3.5,color='#dc2626',alpha=.10)
            ax.axvline(10.6,color='#00875a',alpha=.7,ls=':')
            ax.grid(alpha=.18); ax.set_xlim(.1,end)
        axes[0].text(2.5,43,'SUMO braking',color='#b91c1c')
        axes[0].text(10.8,43,'Merge complete',color='#00875a')
        axes[0].set_ylim(10,48)
        if end>25:
            for ax in axes: ax.axvline(25,color='#555555',ls=':',alpha=.6)
            axes[0].text(25.2,43,'Video ends',color='#555555')
        fig.savefig(output/(name+'.png'),dpi=140); plt.close(fig)
    print(json.dumps({k:summary[k] for k in ['samples','maximum_speed_derivative_acceleration_error','correlation_request_realized','correlation_request_realized_without_overrides','slowdowns']},indent=2))


if __name__ == '__main__':
    main()
