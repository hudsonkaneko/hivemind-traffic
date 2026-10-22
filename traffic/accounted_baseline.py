"""Fixed-horizon SUMO evaluation with a separate drain and viewer-compatible replay."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import traci
from environments.highway_parallel_env import HighwayParallelEnv
from policies.baselines import ScriptedPolicy
from traffic.sumo_backend import SumoBackend
from traffic.trip_accounting import TripLedger, read_requests

ROOT = Path(__file__).resolve().parents[1]


def write(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(config: Path, output: Path, *, horizon=80., drain=60., step=.2, seed=2001,
        controller='scripted', controlled_ids=('agent_0','agent_1')):
    """Keep the old two-agent policy while accounting for the entire traffic cohort.

    Native mode supports the open-merge fixture. Scripted mode deliberately keeps
    the existing two-agent observation/action contract; it is not merge MARL.
    """
    if not all(math.isfinite(x) for x in (horizon,drain,step)) or horizon<=0 or drain<0 or step<=0:
        raise ValueError('Use finite positive horizon/step and nonnegative drain')
    if any(abs(x/step-round(x/step))>1e-7 for x in (horizon,drain)):
        raise ValueError('Horizon and drain must be integral multiples of step')
    if controller not in {'scripted','native'}:
        raise ValueError('Unknown controller')
    config=config.resolve(); output=output.resolve()
    tree=ET.parse(config)
    network=(config.parent/tree.find('./input/net-file').attrib['value']).resolve()
    route_value=tree.find('./input/route-files').attrib['value']
    if ',' in route_value or tree.find('./input/additional-files') is not None:
        raise ValueError('This runner requires one explicit route file and no additional demand')
    routes=(config.parent/route_value).resolve()
    begin=tree.find('./time/begin')
    if begin is not None and float(begin.attrib['value'])!=0:
        raise ValueError('Runner requires time-zero demand; use ledger API for initial cohorts')
    requests=read_requests(routes,controlled_ids)
    if any(r.requested_at>=horizon for r in requests):
        raise ValueError('Fixed cohort must request departure before H; no new drain demand')
    if controller=='scripted' and (set(controlled_ids)!={'agent_0','agent_1'} or
        any(r.requested_at!=0 for r in requests if r.vehicle_id in controlled_ids)):
        raise ValueError('Existing scripted policy supports only agent_0/agent_1 present at reset')
    output.mkdir(parents=True,exist_ok=False)
    inputs={str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):sha(p) for p in (config,network,routes)}
    git=lambda *args: subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()
    source_names=['traffic/accounted_baseline.py','traffic/trip_accounting.py','traffic/sumo_backend.py',
                  'environments/highway_parallel_env.py','policies/baselines.py']
    for name in source_names:
        dest=output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,dest)
    (output/'source.diff').write_text(git('diff','HEAD'),encoding='utf-8')
    protocol=dict(horizon_s=horizon,drain_budget_s=drain,step_s=step,seed=seed,controller=controller,
        controlled_ids=list(controlled_ids),requests=[r.__dict__ for r in requests],
        termination='Always run to H; then drain the same cohort until no unfinished live requests or H+drain',
        metric='Boundary-inclusive time through H; report drain separately; removed/teleported trips never count as successful completions',
        limitation='Engineering fixture; native SUMO human proxies, no fitted human behavior or communication experiment')
    write(output/'resolved-config.json',protocol)
    manifest=dict(status='started',started_utc=datetime.now(timezone.utc).isoformat(),git_commit=git('rev-parse','HEAD'),
        git_dirty=bool(git('status','--porcelain')),command=sys.argv,python=sys.version,platform=platform.platform(),
        traci_file=traci.__file__,inputs=inputs,source_sha256={n:sha(ROOT/n) for n in source_names})
    write(output/'manifest.json',manifest)
    extra=('--tripinfo-output',str(output/'tripinfo.xml'),'--tripinfo-output.write-unfinished','true',
           '--tripinfo-output.write-undeparted','true','--end',str(horizon+drain+step))
    factory=lambda:SumoBackend(config,step_length=step,extra_args=extra)
    env=None; backend=None; frames=[]; checkpoints=[]; collisions=set(); ledger=TripLedger(requests)
    policy=ScriptedPolicy();policy.reset(seed);observations={};last_controlled_time=None
    try:
        if controller=='scripted':
            env=HighwayParallelEnv(config_path=config,horizon=round((horizon+drain)/step)+2,backend_factory=factory)
            observations,_=env.reset(seed=seed);backend=env._require_backend();result=env._result
        else:
            backend=factory();result=backend.reset(seed)
        conn=backend._require_connection()
        manifest['sumo_version']=conn.getVersion()
        manifest['sumo_binary']=str(backend.sumo_binary)
        manifest['sumo_extra_args']=list(extra)
        def record(result):
            departed={}
            for vid in result.departed:
                if vid not in result.states:
                    raise ValueError('Departure and removal within one step: reduce step size to observe departure')
                departed[vid]=conn.vehicle.getDeparture(vid)
            ledger.observe(result.simulation_time,departed=departed,arrived=result.arrived,
                active=result.states.keys(),teleport_started=conn.simulation.getStartingTeleportIDList(),
                teleport_ended=conn.simulation.getEndingTeleportIDList())
            collisions.update(result.collisions)
            state=ledger.snapshot()
            vehicles=[]
            for vid in sorted(result.states):
                vehicles.append(dict(result.states[vid],id=vid,type=conn.vehicle.getTypeID(vid),
                    heading=conn.vehicle.getAngle(vid),length=conn.vehicle.getLength(vid),width=conn.vehicle.getWidth(vid)))
            frames.append(dict(time=result.simulation_time,vehicles=vehicles,departed=list(result.departed),
                arrived=list(result.arrived),collisions=list(result.collisions),accounting=state['totals']))
            return state
        current=record(result)
        while ledger.time < horizon+drain-1e-8:
            if abs(ledger.time-horizon)<1e-8:
                checkpoints.append(current)
            if ledger.time>=horizon-1e-8 and not any(current['totals'][k] for k in ('waiting','active','teleporting')):
                break
            if env is not None and env.agents:
                observations,*_=env.step(policy.act(observations));result=env._result
                if not env.agents: last_controlled_time=result.simulation_time
            else:
                result=backend.step({})
            current=record(result)
        if not checkpoints and abs(ledger.time-horizon)<1e-8:
            checkpoints.append(current)
        if not checkpoints:
            raise RuntimeError('Primary fixed-horizon checkpoint was missed')
    except Exception as exc:
        manifest.update(status='failed',error=repr(exc));write(output/'manifest.json',manifest)
        raise
    finally:
        if env is not None:env.close()
        elif backend is not None:backend.close()
    native={t.attrib['id']:t.attrib for t in ET.parse(output/'tripinfo.xml').getroot().findall('tripinfo')}
    crosscheck=[]
    for trip in current['trips']:
        native_trip=native.get(trip['vehicle_id'])
        if native_trip is None:raise AssertionError('Native tripinfo omitted a declared request')
        if trip['departed_at'] is not None:
            assert abs(float(native_trip['depart'])-trip['departed_at'])<1e-6
        else:
            assert float(native_trip['depart'])<0
        if trip['state']=='completed':
            error=trip['arrived_at']-float(native_trip['arrival'])
            assert -1e-7<=error<=step+1e-7
            assert native_trip.get('vaporized','') in ('','false','0')
            crosscheck.append(error)
    report=dict(primary=checkpoints[0],drain=current,collision_ids=sorted(collisions),
        controlled_agents_left_at=last_controlled_time,native_tripinfo_crosscheck=True,
        arrival_observation_offset_s_max=max(crosscheck,default=0),
        comparison_valid=checkpoints[0]['comparison_valid'] and current['comparison_valid'] and not collisions,
        drain_exhausted=any(current['totals'][k] for k in ('waiting','active','teleporting')),
        evidence_kind='Synthetic accounting verification, not a traffic policy effect')
    write(output/'accounting.json',report)
    write(output/'recording.json',dict(seed=seed,policy=controller,source_timestep=step,pose_reference='SUMO front bumper',
        accounting_report='accounting.json',horizon_s=horizon,frames=frames))
    shutil.copyfile(network,output/'network.net.xml')
    assert inputs=={name:sha(Path(name) if Path(name).is_absolute() else ROOT/name) for name in inputs}
    manifest.update(status='completed',finished_utc=datetime.now(timezone.utc).isoformat(),
        outputs={str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file() and p.name!='manifest.json'})
    write(output/'manifest.json',manifest)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'scenarios/curved_two_agent/scenario.sumocfg')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--horizon',type=float,default=80)
    parser.add_argument('--drain',type=float,default=60)
    parser.add_argument('--step',type=float,default=.2)
    parser.add_argument('--seed',type=int,default=2001)
    parser.add_argument('--controller',choices=['scripted','native'],default='scripted')
    args=parser.parse_args()
    report=run(args.config,args.output,horizon=args.horizon,drain=args.drain,step=args.step,seed=args.seed,controller=args.controller)
    print(json.dumps({k:report[k] for k in ('comparison_valid','controlled_agents_left_at','native_tripinfo_crosscheck','drain_exhausted')}))
    print(json.dumps(dict(primary=report['primary']['totals'],drain=report['drain']['totals'])))


if __name__=='__main__':main()
