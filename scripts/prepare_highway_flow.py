"""Bounded frozen/reference native recordings. This entry point cannot train."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
import torch
import traci

from experiments import run_highway_environment_v05 as historical
from scripts.record_highway_environment_v05 import (
    JunctionEntryTracker, merge_completion_event, frame as native_frame, choose_clip, EVENT_SCHEMA,
)
from traffic.highway_smooth_metrics import SmoothControlMetrics
from traffic.sumo_backend import find_sumo_binary
from traffic.sumo_warnings import native_emergency_report

STUDY = ROOT / 'outputs/highway-flow/v01'
CONTROLLERS = ('native', 'reference', 'v05_final', 'v05_initializer', 'reference_v4')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def route_metrics(frames, controlled_ids):
    """Measured intervals only; stop elapsed span differs from sampled exposure."""
    traces = {}
    previous_time = None
    for sample in frames:
        current_time = float(sample['time'])
        if not math.isfinite(current_time) or (previous_time is not None and current_time <= previous_time):
            raise ValueError('Recording times must be finite and strictly increasing')
        previous_time = current_time
        seen = set()
        for vehicle in sample['vehicles']:
            if vehicle['id'] in seen:
                raise ValueError('Duplicate vehicle identity in frame')
            seen.add(vehicle['id'])
            for key in ('speed', 'acceleration', 'distance_m'):
                if not math.isfinite(float(vehicle[key])):
                    raise ValueError('Nonfinite recorded motion')
            legal = vehicle.get('legal_speed_m_s')
            if legal is not None and (not math.isfinite(float(legal)) or legal <= 0):
                raise ValueError('Invalid recorded legal speed')
            traces.setdefault(vehicle['id'], []).append(dict(vehicle, time=sample['time']))
    per_car = {}
    for identity, trace in traces.items():
        measured, missing, missing_spans, jerk, stopped_since, longest = [], 0, 0, [], None, 0.
        excursions = []
        for i, row in enumerate(trace):
            prior = trace[i-1] if i else None
            consecutive = prior is not None and abs(row['time']-prior['time']-.1) < 1e-7
            if i and not consecutive:
                missing_spans += 1
                missing += max(1, int(round((row['time']-prior['time'])/.1))-1)
            if consecutive:
                measured.append(row)
                jerk.append((row['acceleration']-prior['acceleration'])/.1)
            if row['speed'] < .1:
                if stopped_since is None or (prior is not None and not consecutive):
                    stopped_since = row['time']
                longest = max(longest, row['time']-stopped_since)
            else:
                stopped_since = None
            if (consecutive and prior.get('route_reachable_without_lateral') is True
                    and row.get('route_reachable_without_lateral') is False
                    and prior.get('lane_identity', {}).get('kind') == 'through'
                    and row.get('lane_identity', {}).get('kind') == 'auxiliary'):
                excursions.append(dict(time_s=row['time'], from_lane=prior['lane_id'], to_lane=row['lane_id']))
        low = [r for r in measured if r.get('legal_speed_m_s') is not None
               and r['speed'] < .5*r['legal_speed_m_s']]
        per_car[identity] = dict(controlled=identity in controlled_ids,
            first_observed_s=trace[0]['time'], last_observed_s=trace[-1]['time'],
            native_samples=len(trace), measured_intervals=len(measured), missing_intervals=missing,
            missing_spans=missing_spans,
            minimum_speed_m_s=min(r['speed'] for r in trace),
            below_half_local_limit_exposure_s=.1*len(low),
            stopped_exposure_s=.1*sum(r['speed'] < .1 for r in measured),
            longest_consecutive_stop_s=longest,
            peak_abs_measured_jerk_m_s3=max(map(abs, jerk)) if jerk else None,
            p95_abs_measured_jerk_m_s3=float(np.percentile(np.abs(jerk),95)) if jerk else None,
            ending_lane_excursions=excursions,
            distance_at_fixed_times_m={str(t):next((r['distance_m'] for r in trace if abs(r['time']-t)<1e-7),None)
                                       for t in (10,20,30,40,50,60,70,80)})
    return dict(schema='highway_flow_full_route_metrics_v01',
        interval_s=.1, stopped_threshold_m_s=.1, stop_definition='Consecutive observed elapsed span; no fabricated arrival state',
        low_speed_definition='Cumulative measured interval exposure below half current local legal speed capped by episode ceiling',
        excursion_definition='Observed through/reachable to auxiliary/not-reachable transition; no hidden relabeling of old safety gates',
        per_car=per_car)


def capture_frame(env):
    frame = native_frame(env)
    from traffic.highway_flow_context import route_continuation
    for row in frame['vehicles']:
        state = env.states[row['id']]
        ceiling = 26.8224 if env.speed_limit_m_s is None else float(env.speed_limit_m_s)
        row['legal_speed_m_s'] = min(float(env.graph.speed_limits[state['lane_id']]), ceiling)
        row['native_allowed_speed_m_s'] = float(state['allowed_speed'])
        route = route_continuation(env.graph, state, state['lane_id'])
        row['route_reachable_without_lateral'] = bool(route['reachable'])
    return frame


def actions_for(kind, model, env, observations):
    from traffic.highway_flow_reference import teacher_actions
    if kind == 'native':
        env.control_mode = 'native'
        for identity in env.agents:
            env.conn.vehicle.setSpeed(identity, -1)
            env.conn.vehicle.setLaneChangeMode(identity, 1621)
    if kind in ('native', 'reference'):
        return teacher_actions({a: observations[a] for a in env.agents})
    if kind == 'reference_v4':
        from traffic.highway_flow_context import legacy_observation
        from traffic.highway_smooth_reference import teacher_actions as old_reference
        return old_reference({a: legacy_observation(observations[a]) for a in env.agents})
    agents = list(env.agents)
    values = torch.from_numpy(np.stack([observations[a] for a in agents]).astype(np.float32))
    with torch.inference_mode():
        result = model.actor(values)
    if result.shape != (len(agents),2) or not torch.isfinite(result).all():
        raise ValueError('Invalid frozen actor outputs')
    if bool((result.abs()>1.).any()) or not bool(((result[:,1]==-1)|(result[:,1]==0)|(result[:,1]==1)).all()):
        raise ValueError('Invalid action contract')
    return {a:result[i].cpu().numpy() for i,a in enumerate(agents)}


def collect(env, row, kind, model, deadline, buffers):
    observations, _ = env.reset(seed=row['seed'])
    env.control_mode = 'policy'
    controlled = list(env._controlled_agents())
    focal = next(iter(sorted(env.merge_agents)), 'agent_0')
    metrics = SmoothControlMetrics(task=env.task, merge_vehicle_ids=env.merge_agents,
        controlled_ids=controlled, window_s=env.stabilization_seconds)
    metrics.prime(env.time, env.states)
    tracker = JunctionEntryTracker(env.graph,controlled)
    tracker.observe(env.states,env.time)
    buffers['frames'].append(capture_frame(env))
    statuses, entered = {}, set()
    transitions, reward = 0, 0.
    while env.agents:
        historical.check_deadline(deadline)
        actions = actions_for(kind,model,env,observations)
        before = deepcopy(env.states)
        observations, rewards, _, _, infos = env.step(actions)
        metrics.observe(env.time,before,env.states,infos,merge_vehicle_ids=env.merge_agents,arrived=env.arrived)
        junctions = tracker.observe(env.states,env.time)
        buffers['junction_entry_events'].extend(junctions)
        transitions += len(rewards)
        reward += sum(float(v) for v in rewards.values())
        if not math.isfinite(reward):
            raise ValueError('Nonfinite reward')
        for agent, merge_tracker in env.merge_trackers.items():
            if merge_tracker.entry_time_s is not None and agent not in entered:
                entered.add(agent)
                buffers['entry_events'].append(dict(agent=agent,time=merge_tracker.entry_time_s,kind='through_lane_entry'))
        for agent,info in infos.items():
            if info.get('merge_completed'):
                buffers['merge_events'].append(merge_completion_event(env,agent,info,junctions))
            status = info.get('stable_merge_status')
            if status in ('succeeded','failed') and statuses.get(agent)!=status:
                buffers['stable_events'].append(dict(agent=agent,time=env.time,status=status,
                    failure_reasons=env.merge_trackers[agent].summary()['failure_reasons']))
            statuses[agent]=status
        buffers['frames'].append(capture_frame(env))
    advance=env._advance
    def drain():
        before=deepcopy(env.states)
        advance()
        metrics.observe(env.time,before,env.states,{},merge_vehicle_ids=env.merge_agents,arrived=env.arrived)
        buffers['junction_entry_events'].extend(tracker.observe(env.states,env.time))
        buffers['frames'].append(capture_frame(env))
    env._advance=drain
    try:
        report=historical.primary_accounting(env,partial=False,deadline=deadline)
    finally:
        env._advance=advance
    report.update(agent_transitions=transitions,reward=reward,partial_control_horizon=False,
        smooth_control=metrics.report(),full_route_metrics=route_metrics(buffers['frames'],controlled),
        sumo_runtime_version=env.conn.getVersion(),population_setting={k:row[k] for k in
            ('controlled_count','total_vehicles','communication','packet_loss')})
    return report,focal,controlled


def record(cfg,row,kind,model,output,hashes,deadline):
    from experiments import run_highway_flow as runner
    output.mkdir(parents=True,exist_ok=False)
    env=runner.env_for(cfg,output/'simulation',row)
    buffers={k:[] for k in ('frames','merge_events','stable_events','entry_events','junction_entry_events')}
    manifest=dict(schema='highway_flow_frozen_recording_v01',training=False,optimizer_steps=0,
        controller=kind,policy_mode='frozen learned actor' if kind.startswith('v05_') else kind+' scripted baseline',
        policy_sha256=cfg['initialization']['candidates'][kind]['actor_sha256'] if kind.startswith('v05_') else None,
        selected_case=row,seed=row['seed'],source_sha256=hashes,observation_size=117,max_agents=24,
        motion_owner='SUMO',position_reference='SUMO front-bumper center',source_timestep=.1,
        horizon_steps=800,horizon_seconds=80.,event_schema=EVENT_SCHEMA,
        network=row['case']['network'],network_sha256=row['case']['network_sha256'],
        safety_modes=dict(speed=31,lane=1621 if kind=='native' else 512),
        native_baseline_limitation='Automatic lane choices differ from external actor/reference control',
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip())
    historical.atomic_json(output/'manifest.json',manifest)
    shutil.copyfile(ROOT/row['case']['network'],output/'network.net.xml')
    commands=[]
    native_start=traci.start
    def start(command,*args,**kwargs):
        commands.append(list(map(str,command)))
        return native_start(command,*args,**kwargs)
    traci.start=start
    started=time.monotonic()
    error,report,focal,controlled=None,{},'agent_0',[]
    try:
        report,focal,controlled=collect(env,row,kind,model,deadline,buffers)
    except Exception:
        error=traceback.format_exc()
        try:
            report=env.report()
        except Exception:
            pass
        report.update(primary_horizon_complete=False,accounting_comparison_valid=False)
    finally:
        traci.start=native_start
        env.close()
    report.update(training=False,optimizer_steps=0,controller=kind,seed=row['seed'],case_id=row['case']['id'],
        difficulty=row.get('difficulty',0),traffic_mode=row.get('traffic_mode','initial'),
        wall_seconds=time.monotonic()-started,native_emergencies=native_emergency_report(env.directory/'sumo-errors.log'),
        **{k:v for k,v in buffers.items() if k!='frames'})
    historical.atomic_json(output/'evaluation.json',report)
    recording=dict(manifest,**buffers,focal_vehicle=focal,controlled_ids=controlled,native_sumo_commands=commands,
        **(choose_clip(buffers['frames'],buffers['merge_events']) if buffers['frames'] else {}))
    historical.atomic_json(output/'recording.json',recording)
    if error:
        historical.atomic_json(output/'failure.json',dict(traceback=error))
    result=dict(row_id=row.get('id',row['case']['id']),controller=kind,error=error,
        source_changed=[n for n,h in hashes.items() if sha(ROOT/n)!=h],
        path=output.relative_to(ROOT).as_posix(),recording_sha256=sha(output/'recording.json'),
        evaluation_sha256=sha(output/'evaluation.json'),wall_seconds=report['wall_seconds'],
        intervention_fraction=report.get('intervention_fraction'),frames=len(buffers['frames']))
    historical.atomic_json(output/'recording-check.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/training/highway-flow-v01.json')
    parser.add_argument('--rows-file',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--controllers',nargs='+',choices=CONTROLLERS,default=['native','reference','v05_final'])
    parser.add_argument('--max-wall-seconds',type=float,required=True)
    args=parser.parse_args()
    from experiments import run_highway_flow as runner
    output=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not output.is_relative_to(STUDY) or output.exists():
        raise ValueError('Canonical root and fresh flow output required')
    if not 0<args.max_wall_seconds<=1800:
        raise ValueError('Invalid preparation bound')
    cfg=runner.load_config(args.config)
    rows=json.loads(args.rows_file.read_text(encoding='utf-8'))
    if not isinstance(rows,list) or not rows:
        raise ValueError('Rows file must be a nonempty list of explicit cases')
    names=[r.get('id',r['case']['id'])+'-s'+str(r['seed']) for r in rows]
    if len(names)!=len(set(names)) or any(any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in n) for n in names):
        raise ValueError('Case output identities must be unique safe names')
    for row in rows:
        if sha(ROOT/row['case']['network'])!=row['case']['network_sha256']:
            raise ValueError('Network identity mismatch')
    hashes=runner.sources()
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    output.mkdir(parents=True)
    for name,digest in hashes.items():
        target=output/'source'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
        if sha(target)!=digest: raise RuntimeError('Unstable source snapshot')
    historical.atomic_json(output/'config.json',cfg)
    historical.atomic_json(output/'study.json',dict(rows=rows,controllers=args.controllers,source_hashes=hashes,
        training=False,optimizer_steps=0,command=[sys.executable,*sys.argv],max_wall_seconds=args.max_wall_seconds,
        python=platform.python_version(),torch=torch.__version__,traci=traci.__version__,
        sumo_version=subprocess.check_output([str(find_sumo_binary()),'--version'],text=True).splitlines()[0]))
    ledger_path=STUDY/'preparation-ledger.json'
    ledger=json.loads(ledger_path.read_text(encoding='utf-8')) if ledger_path.exists() else dict(schema='highway_flow_preparation_ledger_v01',attempts=[])
    if any(a['status'] not in ('complete','completed','failed') for a in ledger['attempts']):
        raise ValueError('Unfinished preparation reservation')
    prior=sum(a['charged_wall_seconds'] for a in ledger['attempts'])
    if prior+args.max_wall_seconds>1800:
        raise ValueError('Preparation exceeds approved1800s cap')
    entry=dict(output=output.relative_to(ROOT).as_posix(),status='running',charged_wall_seconds=args.max_wall_seconds,
        reserved_wall_seconds=args.max_wall_seconds,source_hashes=hashes)
    ledger['attempts'].append(entry)
    historical.atomic_json(ledger_path,ledger)
    started=time.monotonic()
    results,error=[],None
    try:
        models={k:runner.initialize_model(cfg,k) for k in args.controllers if k.startswith('v05_')}
        for row,name in zip(rows,names):
            for controller in args.controllers:
                historical.check_deadline(started+args.max_wall_seconds)
                result=record(cfg,row,controller,models.get(controller),output/name/controller,hashes,started+args.max_wall_seconds)
                results.append(result)
                historical.atomic_json(output/'results.json',results)
                print(json.dumps({k:result[k] for k in ('row_id','controller','error','intervention_fraction','wall_seconds')}),flush=True)
                if result['error'] or result['source_changed']:
                    raise RuntimeError('Native recording integrity failure')
    except Exception:
        error=traceback.format_exc()
    finally:
        elapsed=time.monotonic()-started
        entry.update(status='failed' if error else 'complete',charged_wall_seconds=elapsed)
        historical.atomic_json(ledger_path,ledger)
        historical.atomic_json(output/'summary.json',dict(training=False,optimizer_steps=0,error=error,runs=results,
            wall_seconds=elapsed,preparation_total_wall_seconds=prior+elapsed))
    return int(error is not None)


if __name__=='__main__':
    raise SystemExit(main())
