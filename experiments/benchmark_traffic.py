"""Bounded SUMO-only scaling probe; no RTX, communication or learning claims."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

import numpy as np
import traci
from traffic.live_runtime import find_sumo
from traffic.state_contract import Snapshot, StateStream, VehicleState
from traffic.study_config import StudyConfig
from experiments.probe_support import ROOT, finish, process_sample, run_package, write_json

SOURCES = ['experiments/benchmark_traffic.py', 'experiments/probe_support.py',
    'traffic/study_config.py', 'traffic/state_contract.py', 'traffic/live_runtime.py',
    'scenarios/scaling/nodes.nod.xml', 'scenarios/scaling/edges.edg.xml']


def run(config):
    output, manifest = run_package('traffic_scaling', config.resolved(), SOURCES)
    manifest['command'] = sys.argv
    binary = find_sumo()
    manifest.update(sumo_version=subprocess.check_output([str(binary), '--version'], text=True).splitlines()[0], traci_module=traci.__file__)
    c = None
    result = {'passed': False}
    rows = []
    try:
        network = output/'network.net.xml'
        netconvert = binary.with_name('netconvert'+binary.suffix)
        subprocess.run([str(netconvert), '--node-files', str(ROOT/SOURCES[-2]), '--edge-files', str(ROOT/SOURCES[-1]), '--output-file', str(network)], check=True, capture_output=True)
        manifest['network_sha256'] = hashlib.sha256(network.read_bytes()).hexdigest()
        label = 'scaling-' + uuid.uuid4().hex
        command = [str(binary), '--net-file', str(network), '--step-length', str(config.dt_s), '--seed', str(config.seed),
            '--time-to-teleport', '-1', '--collision.action', 'warn', '--no-step-log', 'true', '--duration-log.disable', 'true', '--log', str(output/'sumo.log')]
        manifest['sumo_command'] = command
        traci.start(command, label=label, stdout=subprocess.DEVNULL)
        c = traci.getConnection(label)
        c.route.add('road_route', ['road'])
        for kind, sigma in [('human_model', .5), ('av', 0.)]:
            c.vehicletype.copy('DEFAULT_VEHTYPE', kind)
            c.vehicletype.setLength(kind, 5.)
            c.vehicletype.setWidth(kind, 2.)
            c.vehicletype.setMaxSpeed(kind, 8.)
            c.vehicletype.setSpeedFactor(kind, 1.)
            c.vehicletype.setImperfection(kind, sigma)
        roles = config.roles()
        for i, (vid, role) in enumerate(roles.items()):
            c.vehicle.add(vid, 'road_route', typeID=role, departPos=str(15+25*(i//2)), departLane=str(i%2), departSpeed='8')
        c.simulationStep()
        if set(c.vehicle.getIDList()) != set(roles):
            raise RuntimeError(f'Population insertion incomplete: {len(c.vehicle.getIDList())}/{len(roles)}; do not benchmark a smaller fleet')
        tc = traci.constants
        for vid in roles:
            c.vehicle.subscribe(vid, [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ANGLE, tc.VAR_ACCELERATION])
        for _ in range(round(config.warmup_s/config.dt_s)):
            c.simulationStep()
        origin = c.simulation.getTime()
        state_stream = StateStream()
        trajectory_hash = hashlib.sha256()
        snapshots = []
        commands = collisions = 0
        av_ids = [vid for vid, role in roles.items() if role == 'av']
        stride = round(1/config.control_hz/config.dt_s)
        sumo_pid = c._process.pid
        before = {'python': process_sample(), 'sumo': process_sample(sumo_pid)}
        started = time.perf_counter()
        for step in range(1, round(config.seconds/config.dt_s)+1):
            begin = time.perf_counter()
            if begin-started > 120:
                raise TimeoutError('CPU workload wall budget exceeded')
            if config.control_mode == 'scripted_speed' and (step-1) % stride == 0:
                for vid in av_ids:
                    c.vehicle.setSpeed(vid, 8.)
                    commands += 1
            applied = time.perf_counter()
            c.simulationStep()
            stepped = time.perf_counter()
            states = c.vehicle.getAllSubscriptionResults()
            if set(states) != set(roles):
                raise RuntimeError('Fleet size changed during capacity probe')
            vehicles = tuple(VehicleState(vid, *states[vid][tc.VAR_POSITION], states[vid][tc.VAR_ANGLE], states[vid][tc.VAR_SPEED]) for vid in sorted(states))
            snapshot = Snapshot(output.name, step, c.simulation.getTime()-origin, config.dt_s, vehicles)
            state_stream.accept(snapshot)
            payload = asdict(snapshot)
            # The trajectory signature deliberately excludes the unique episode ID.
            trajectory_hash.update(json.dumps({k:v for k,v in payload.items() if k != 'episode_id'}, sort_keys=True).encode())
            if step == 1 or step == round(config.seconds/config.dt_s):
                snapshots.append(payload)
            collisions += len(c.simulation.getCollidingVehiclesIDList())
            end = time.perf_counter()
            rows.append(dict(step=step, command_ms=(applied-begin)*1000, sumo_ms=(stepped-applied)*1000,
                state_contract_ms=(end-stepped)*1000, total_ms=(end-begin)*1000,
                mean_speed_mps=float(np.mean([v.speed_mps for v in vehicles]))))
        wall = time.perf_counter()-started
        after = {'python': process_sample(), 'sumo': process_sample(sumo_pid)}
        timings = [r['total_ms'] for r in rows]
        cpu = sum(after[k]['cpu_seconds']-before[k]['cpu_seconds'] for k in before) if all(v.get('available') for v in [*before.values(), *after.values()]) else None
        result = dict(passed=collisions == 0, vehicles=config.vehicles, av_count=config.av_count, mode=config.control_mode,
            control_hz=config.control_hz, seed=config.seed, steps=len(rows), collisions=collisions,
            wall_seconds=wall, sim_seconds=config.seconds, real_time_factor=config.seconds/wall,
            p50_step_ms=float(np.percentile(timings,50)), p95_step_ms=float(np.percentile(timings,95)), max_step_ms=max(timings),
            commands=commands, cpu_seconds=cpu, cpu_equivalent_cores=None if cpu is None else cpu/wall,
            resources_before=before, resources_after=after, trajectory_sha256=trajectory_hash.hexdigest(),
            caveat='Sparse straight-road capacity probe; simulator-state control, no LiDAR or V2V; not effectiveness evidence')
        write_json(output/'snapshot-fixture.json', snapshots)
    except Exception as error:
        result['error'] = repr(error)
    finally:
        if c is not None:
            c.close()
        write_json(output/'steps.json', rows)
        finish(output, manifest, result)
    print(json.dumps({'output': str(output), **result}), flush=True)
    return output, result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'experiments/configs/scaling_probe.json')
    p.add_argument('--suite', action='store_true')
    args = p.parse_args()
    config = StudyConfig(**json.loads(args.config.read_text()))
    if not args.suite:
        return int(not run(config)[1]['passed'])
    suite, manifest = run_package('traffic_scaling_suites', {'plan': 'documentation/scaling-study-plan.md'}, SOURCES)
    cases = []
    # No seed selection after results; all failed runs remain in the suite.
    for n in (2,20,50,100,200):
        for mode, hz in [('sumo_native',2.), ('scripted_speed',2.), ('scripted_speed',10.)]:
            for seed in (42,43,44):
                path, result = run(replace(config, vehicles=n, av_fraction=.5, control_mode=mode, control_hz=hz, seed=seed))
                cases.append({'path': str(path.relative_to(ROOT)), **result})
                write_json(suite/'cases.json', cases)
    path, repeat = run(replace(config, vehicles=20, av_fraction=.5, control_mode='scripted_speed', control_hz=2., seed=42))
    expected = next(c for c in cases if c['vehicles']==20 and c['mode']=='scripted_speed' and c['control_hz']==2 and c['seed']==42)
    result = dict(passed=all(c['passed'] for c in cases) and repeat.get('trajectory_sha256') == expected.get('trajectory_sha256'),
        cases=len(cases), repeat_path=str(path.relative_to(ROOT)), repeat_matches=repeat.get('trajectory_sha256') == expected.get('trajectory_sha256'))
    finish(suite, manifest, result)
    print('TRAFFIC_SUITE='+str(suite), flush=True)
    return int(not result['passed'])


if __name__ == '__main__':
    raise SystemExit(main())
