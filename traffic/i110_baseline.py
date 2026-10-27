"""Bounded, synthetic I-110 ACC baseline with complete boundary accounting.

Hudson's trained actor is deliberately not substituted by random weights.
This first fixture prepares the traffic side of the later matched comparison.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import traci

from scripts.build_i110_sumo import CAP, write_xml
from traffic.sumo_backend import SumoBackend
from traffic.trip_accounting import TripLedger, read_requests

ROOT = Path(__file__).resolve().parents[1]
CONTROLLED = tuple(f'agent_{i}' for i in range(4))
DT = .1


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_demand(output, seed, case='ordinary'):
    """Explicit requests, not hidden flows; identical seed defines a cohort."""
    if case not in {'ordinary', 'braking_leader'}:
        raise ValueError('Unknown case')
    rng = random.Random(seed)
    routes = ET.Element('routes')
    common = dict(length='4.8', width='1.9', minGap='2.5', accel='2', decel='4.5',
                  emergencyDecel='9', tau='1.5', speedDev='0', maxSpeed=str(CAP))
    ET.SubElement(routes, 'vType', id='acc', carFollowModel='ACC', speedFactor=str(25/CAP), **common)
    ET.SubElement(routes, 'vType', id='human', carFollowModel='IDM', speedFactor='1', **common)
    ET.SubElement(routes, 'route', id='through', edges='sb0 sb1 sb2 sb3 sb4 sb5')
    ET.SubElement(routes, 'route', id='entry', edges='mlk_on sb2 sb3 sb4 sb5')
    ET.SubElement(routes, 'route', id='exit', edges='sb0 sb1 sb2 vernon_off')
    cohort = []
    for i, vid in enumerate(CONTROLLED):
        cohort.append((0., vid, 'acc', 'through', str(i), 'mainline', 15.))
    # A reproducible non-calibrated mixture, including every boundary and exit.
    for i in range(32):
        depart = 2.+i*1.1
        route = 'exit' if i % 5 == 0 else 'through'
        cohort.append((depart, f'human_{i:03}', 'human', route, str(rng.randrange(4)), 'mainline', rng.uniform(13,21)))
    for i in range(8):
        cohort.append((1.+i*4.5, f'ramp_{i:03}', 'human', 'entry', '0', 'mlk_ramp', rng.uniform(7,11)))
    if case == 'braking_leader':
        cohort.append((0., 'brake_leader', 'human', 'through', '1', 'mainline', 15.))
    for depart, vid, typ, route, lane, origin, speed in sorted(cohort):
        pos = '45' if vid == 'brake_leader' else 'base'
        vehicle = ET.SubElement(routes, 'vehicle', id=vid, type=typ, route=route,
            depart=f'{depart:.3f}', departLane=lane, departSpeed=f'{speed:.6f}', departPos=pos)
        ET.SubElement(vehicle, 'param', key='research.origin', value=origin)
    write_xml(output/'routes.rou.xml', routes)
    return [row[1] for row in cohort]


def run(network, output, *, seed=730101, case='ordinary', horizon=60., drain=90., wall_budget=90.):
    if not all(math.isfinite(v) for v in (horizon,drain,wall_budget)) or horizon<40 or drain<0 or wall_budget<=0:
        raise ValueError('Use finite H >= 40 s, nonnegative drain and positive wall budget')
    if any(abs(v/DT-round(v/DT))>1e-7 for v in (horizon,drain)):
        raise ValueError('Horizon and drain must be multiples of 0.1 s')
    if type(seed) is not int or not 0<=seed<2**31:
        raise ValueError('Seed must be a nonnegative signed 32-bit integer')
    network = Path(network).resolve()
    if not network.is_file():
        raise FileNotFoundError(network)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(network, output/'network.net.xml')
    prepare_demand(output, seed, case)
    config = ET.Element('configuration')
    inp = ET.SubElement(config, 'input')
    ET.SubElement(inp, 'net-file', value='network.net.xml')
    ET.SubElement(inp, 'route-files', value='routes.rou.xml')
    processing = ET.SubElement(config, 'processing')
    for key,value in {'time-to-teleport':'-1','collision.action':'warn','collision.check-junctions':'true'}.items():
        ET.SubElement(processing,key,value=value)
    write_xml(output/'scenario.sumocfg', config)
    source_names = ['traffic/i110_baseline.py','traffic/trip_accounting.py','traffic/sumo_backend.py',
                    'scripts/build_i110_sumo.py','scenarios/i110/sumo-road-profiles.json']
    for name in source_names:
        dest=output/'source'/name
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,dest)
    git = lambda *args: subprocess.check_output(['git','-c',f'safe.directory={ROOT.as_posix()}',*args],cwd=ROOT,text=True)
    (output/'source.diff').write_text(git('diff','HEAD'), encoding='utf-8')
    protocol = dict(seed=seed,case=case,controller='SUMO_native_ACC',human_model='IDM',
        step_s=DT,horizon_s=horizon,drain_budget_s=drain,wall_budget_s=wall_budget,
        controlled_ids=CONTROLLED,controlled_lane_change_mode=512,speed_mode=31,
        event_layer={'kind':'scripted_leader_speed' if case=='braking_leader' else 'none',
            'schedule_s':[8,12,17,22] if case=='braking_leader' else [],
            'description':'15 to 5 m/s over 4 s; hold 5 s; recover over 5 s' if case=='braking_leader' else None},
        purpose='Engineering smoke; no real LA demand, no held-out efficacy claim',
        acceptance='No collision/teleport/removal; all requested trips complete within bounded drain; controlled vehicles retain physical through lane',
        primary_metric='Boundary-inclusive time through H; drain reported separately',
        comparison_status='Hudson arm unavailable: checkpoint missing and three-lane contract incompatible with this graph')
    write_json(output/'resolved-config.json',protocol)
    manifest=dict(status='started',started_utc=datetime.now(timezone.utc).isoformat(),
        command=[sys.executable,*sys.argv],python=sys.version,platform=platform.platform(),
        git_commit=git('rev-parse','HEAD').strip(),git_status=git('status','--short'),
        traci_file=traci.__file__,traci_version=traci.__version__,
        inputs={n:sha(output/n) for n in ('network.net.xml','routes.rou.xml','scenario.sumocfg','resolved-config.json')},
        source_sha256={n:sha(ROOT/n) for n in source_names},training=False,optimizer_steps=0)
    write_json(output/'manifest.json',manifest)
    backend=SumoBackend(output/'scenario.sumocfg',step_length=DT,extra_args=(
        '--tripinfo-output',str(output/'tripinfo.xml'),'--tripinfo-output.write-unfinished','true',
        '--tripinfo-output.write-undeparted','true','--end',str(horizon+drain+DT),
        '--duration-log.disable','true','--log',str(output/'sumo.log')))
    ledger=TripLedger(read_requests(output/'routes.rou.xml',CONTROLLED))
    collisions=set(); emergency=set(); primary=None; previous_accel={}; lane_sets={v:set() for v in CONTROLLED}
    min_gap=None; min_ttc=None; max_jerk=0.; peak_speed=0.; event_records=[]; trace_hash=hashlib.sha256()
    deadline=time.monotonic()+wall_budget
    try:
        with (output/'trace.jsonl').open('x',encoding='utf-8') as trace:
            result=backend.reset(seed)
            conn=backend._require_connection()
            manifest['sumo_version']=conn.getVersion()
            manifest['sumo_binary']=str(backend.sumo_binary)
            while True:
                if time.monotonic()>deadline:
                    raise TimeoutError('I-110 engineering run exceeded wall budget')
                active=set(result.states)
                for vid in result.departed:
                    conn.vehicle.setSpeedMode(vid,31)
                    if vid in CONTROLLED or vid=='brake_leader':
                        conn.vehicle.setLaneChangeMode(vid,512)
                ledger.observe(result.simulation_time,
                    departed={v:conn.vehicle.getDeparture(v) for v in result.departed},
                    arrived=result.arrived,active=active,
                    teleport_started=conn.simulation.getStartingTeleportIDList(),
                    teleport_ended=conn.simulation.getEndingTeleportIDList())
                current=ledger.snapshot()
                collisions.update(result.collisions)
                emergency.update(conn.simulation.getEmergencyStoppingVehiclesIDList())
                for vid,state in result.states.items():
                    state['z']=conn.vehicle.getPosition3D(vid)[2]
                    state['lane_position']=conn.vehicle.getLanePosition(vid)
                    state['leader']=conn.vehicle.getLeader(vid,120.)
                    if vid in CONTROLLED:
                        state['requested_acceleration']=None
                        state['filtered_acceleration']=None
                        state['action_source']='native_ACC_internal_controller'
                        state['internal_ACC_override_observable']=False
                        peak_speed=max(peak_speed,state['speed'])
                        if vid in previous_accel:
                            max_jerk=max(max_jerk,abs(state['acceleration']-previous_accel[vid])/DT)
                        previous_accel[vid]=state['acceleration']
                        road=state['road_id']
                        if road.startswith('sb'):
                            index=state['lane_index']-(1 if road in {'sb1','sb2','sb3'} else 0)
                            lane_sets[vid].add(index)
                        leader=state['leader']
                        if leader and leader[0] in active:
                            # TraCI leader gap excludes ego minGap; restore actual bumper spacing.
                            gap=leader[1]+conn.vehicle.getMinGap(vid)
                            min_gap=gap if min_gap is None else min(min_gap,gap)
                            closing=state['speed']-result.states[leader[0]]['speed']
                            if closing>1e-6:
                                ttc=gap/closing
                                min_ttc=ttc if min_ttc is None else min(min_ttc,ttc)
                frame=dict(time=result.simulation_time,states=result.states,departed=result.departed,
                    arrived=result.arrived,collisions=result.collisions,totals=current['totals'])
                line=json.dumps(frame,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n'
                trace.write(line);trace_hash.update(line.encode())
                if abs(result.simulation_time-horizon)<1e-7:
                    primary=current
                if result.simulation_time>=horizon-1e-7 and (not current['totals']['unfinished'] or result.simulation_time>=horizon+drain-1e-7):
                    break
                t=result.simulation_time
                if case=='braking_leader' and 'brake_leader' in active:
                    if 8<=t<12: target=15-2.5*(t-8)
                    elif 12<=t<17: target=5.
                    elif 17<=t<22: target=5+2*(t-17)
                    elif t<8: target=15.
                    else: target=-1.
                    conn.vehicle.setSpeed('brake_leader',target)
                    event_records.append(dict(time=t,vehicle='brake_leader',requested_speed_m_s=target))
                result=backend.step({})
        if primary is None:
            raise AssertionError('Fixed-horizon accounting checkpoint missed')
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc))
        write_json(output/'manifest.json',manifest)
        raise
    finally:
        backend.close()
    try:
        native={v.attrib['id']:v.attrib for v in ET.parse(output/'tripinfo.xml').getroot().findall('tripinfo')}
        assert set(native)==set(ledger.trips), 'Native output omitted requested trips'
        for row in current['trips']:
            item=native[row['vehicle_id']]
            if row['departed_at'] is None: assert float(item['depart'])<0
            else: assert abs(row['departed_at']-float(item['depart']))<1e-6
            if row['state']=='completed':
                assert -1e-7<=row['arrived_at']-float(item['arrival'])<=DT+1e-7
                assert item.get('vaporized','') in ('','false','0')
        lane_pass=all(lane_sets[v]=={i} for i,v in enumerate(CONTROLLED))
        passed=bool(current['comparison_valid'] and not collisions and not current['totals']['unfinished'] and lane_pass)
        report=dict(passed=passed,primary=primary,drain=current,collisions=sorted(collisions),
            emergency_stop_ids=sorted(emergency),controlled_physical_lanes={k:sorted(v) for k,v in lane_sets.items()},
            min_controlled_bumper_gap_m=min_gap,min_controlled_ttc_s=min_ttc,max_controlled_abs_jerk_m_s3=max_jerk,
            peak_controlled_speed_m_s=peak_speed,controlled_lane_pass=lane_pass,native_tripinfo_crosscheck=True,
            trace_sha256=trace_hash.hexdigest(),interpretation='ACC fixture smoke, not driver comfort acceptance or policy comparison',
            intervention_note='Native ACC collision-avoidance internals are not exposed as counters; null is not zero intervention.')
        write_json(output/'metrics.json',report)
        write_json(output/'event-actions.json',event_records)
        assert all(sha(output/n)==digest for n,digest in manifest['inputs'].items())
        manifest.update(status='completed' if passed else 'acceptance_failed',finished_utc=datetime.now(timezone.utc).isoformat(),
            passed=passed,outputs={p.name:sha(p) for p in output.iterdir() if p.is_file() and p.name!='manifest.json'})
        write_json(output/'manifest.json',manifest)
        return report
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));write_json(output/'manifest.json',manifest)
        raise


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,default=730101)
    p.add_argument('--case',choices=['ordinary','braking_leader'],default='ordinary')
    p.add_argument('--horizon',type=float,default=60.)
    p.add_argument('--drain',type=float,default=90.)
    args=p.parse_args()
    result=run(**vars(args))
    print(json.dumps({k:result[k] for k in ('passed','trace_sha256','peak_controlled_speed_m_s')}))
    raise SystemExit(0 if result['passed'] else 1)
