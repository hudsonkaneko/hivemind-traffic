"""Bounded frozen-policy junction diagnostics; no training or safety bypasses.

Command-lifetime and native-longitudinal variants change only agent_0's control.
A checksum-verified replacement network affects all traffic. Detailed telemetry
focuses on agent_0 in every variant; that sampling scope is not treatment scope.
Requests are recomputed every 0.1 s. Policy, demand and safety modes remain fixed.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
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

import torch
import traci
from experiments import run_highway_environment_v05 as runner
from scripts.record_highway_environment_v05 import frozen_actions
from traffic.sumo_backend import find_sumo_binary
from traffic.sumo_warnings import native_emergency_report

RUN = ROOT / 'outputs/highway-environment/v05/train-20261106T024828Z-90b25880'
POLICY_SHA = 'fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def variant_labels(factor):
    scopes = {
        'baseline': ('none; unchanged frozen baseline', []),
        'acceleration_command_lifetime': ('focal vehicle command only', ['agent_0']),
        'native_longitudinal_handoff': ('focal vehicle longitudinal control only', ['agent_0']),
        'network_replacement': ('all traffic using the network', None),
    }
    scope, vehicle_ids = scopes[factor]
    return dict(variant_factor=factor, treatment_scope=scope,
        directly_treated_vehicle_ids=vehicle_ids, telemetry_focal_vehicle='agent_0',
        network_treatment='comparison_network' if factor == 'network_replacement' else 'original_frozen_network')


def install_acceleration_probe(api, duration, applied):
    """Override only the focal command lifetime, preserving other controls."""
    original = api.setAcceleration
    def acceleration(identity, value, original_duration):
        lifetime = duration if identity == 'agent_0' else original_duration
        if lifetime is None:
            applied[identity] = dict(acceleration=None, duration_s=None, native_longitudinal=True)
            return api.setSpeed(identity, -1)
        applied[identity] = dict(acceleration=float(value), duration_s=float(lifetime))
        return original(identity, value, lifetime)
    api.setAcceleration = acceleration


def native_snapshot(env, identity='agent_0'):
    """Read native diagnostic values; queries never become actor inputs."""
    if identity not in env.states:
        return dict(time=env.time, present=False)
    api = env.conn.vehicle
    def query(name, *args):
        try:
            return getattr(api, name)(identity, *args)
        except traci.TraCIException as exc:
            return {'query_error': str(exc)}
    state = env.states[identity]
    leader = query('getLeader', 200.)
    # SUMO 1.27.1 ACC::followSpeed ignores CalcReason::FUTURE and _v updates
    # lastUpdateTime/control mode. A nominal getter can therefore perturb the
    # simulation. Do not call getFollowSpeed for live diagnostic snapshots.
    fields = {key: state[key] for key in ('lane_id', 'lane_position', 'speed', 'acceleration', 'x', 'y')}
    links = env.conn.lane.getLinks(state['lane_id'], extended=True)
    foes = env.conn.lane.getFoes(state['lane_id'], links[0][0]) if links else ()
    return dict(time=float(env.time), present=True, **fields,
        lane_length=env.conn.lane.getLength(state['lane_id']),
        lane_links=links, right_of_way_lanes=foes,
        right_of_way_vehicles={identity: {k: other[k] for k in ('lane_id','lane_position','speed','x','y')}
            for identity, other in env.states.items() if other['lane_id'] in foes},
        leader=leader, leader_follow_speed=None,
        leader_follow_speed_status='not_queried: SUMO ACC getter can mutate controller state',
        native_speed_without_traci=query('getSpeedWithoutTraCI'),
        stop_state=query('getStopState'), stops=query('getStops'),
        next_links=query('getNextLinks'), junction_foes=query('getJunctionFoes', 100.),
        best_lanes=query('getBestLanes'), route=query('getRoute'),
        route_valid=query('isRouteValid'), speed_mode=query('getSpeedMode'),
        lane_change_mode=query('getLaneChangeMode'),
        lane_change_left=query('getLaneChangeState', 1),
        lane_change_right=query('getLaneChangeState', -1))


def run_variant(cfg, row, actor, output, duration, horizon_seconds, deadline, *, factor):
    output.mkdir()
    env = runner.env_for(cfg, output / 'simulation', row,
                         horizon=round(horizon_seconds / .1))
    started = time.monotonic()
    command = []
    original_start = traci.start
    def captured_start(args, *positional, **keywords):
        args = [*args, '--link-output', str(output / 'native-links.xml')]
        command.append(list(map(str, args)))
        return original_start(args, *positional, **keywords)
    traci.start = captured_start
    report, error, count, entered, stopped_samples = None, None, 0, None, 0
    trace = (output / 'diagnostics.jsonl').open('w', encoding='utf-8')
    try:
        obs, _ = env.reset(seed=row['seed'])
        env.control_mode = 'policy'
        applied = {}
        install_acceleration_probe(env.conn.vehicle, duration, applied)
        while env.agents:
            if time.monotonic() >= deadline:
                raise TimeoutError('Diagnostic wall-time bound reached')
            before = native_snapshot(env)
            actions = frozen_actions(actor, obs, list(env.agents)) if env.agents else {}
            mask = obs['agent_0'][30:33].tolist() if 'agent_0' in obs else None
            chosen = actions.get('agent_0')
            applied.clear()
            obs, _, _, _, infos = env.step(actions)
            after = native_snapshot(env)
            if after['present']:
                stopped_samples += int(after['speed'] < .1)
                if entered is None and after['lane_id'] != 'mlk_on_0':
                    entered = after['time']
            trace.write(json.dumps(dict(before=before, actor_mask=mask,
                actor_action=chosen.tolist() if chosen is not None else None,
                applied_command=applied.get('agent_0'), info=infos.get('agent_0'),
                after=after), allow_nan=False) + '\n')
            count += 1
        original_advance = env._advance
        def diagnostic_advance():
            nonlocal count
            before = native_snapshot(env)
            original_advance()
            trace.write(json.dumps(dict(before=before, actor_mask=None, actor_action=None,
                applied_command=None, info=None, after=native_snapshot(env),
                post_control_native_tail=True), allow_nan=False) + '\n')
            count += 1
        env._advance = diagnostic_advance
        try:
            report = runner.primary_accounting(env, partial=False, deadline=deadline)
        finally:
            env._advance = original_advance
    except Exception:
        error = traceback.format_exc()
    finally:
        traci.start = original_start
        env.close()
        trace.close()
    summary = dict(duration_s=duration, **variant_labels(factor),
        steps=count, first_ramp_exit_s=entered, stopped_samples=stopped_samples,
        wall_seconds=time.monotonic()-started, error=error, report=report,
        sumo_command=command, training=False, optimizer_steps=0,
        network=row['case']['network'], network_sha256=row['case']['network_sha256'],
        demand_sha256=digest(env.directory/'routes.rou.xml') if (env.directory/'routes.rou.xml').exists() else None,
        native_emergencies=native_emergency_report(env.directory/'sumo-errors.log'))
    runner.atomic_json(output / 'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--durations', type=float, nargs='+', default=[.1, 1., 5.])
    parser.add_argument('--horizon-seconds', type=float, default=80.)
    parser.add_argument('--include-native-longitudinal', action='store_true',
                        help='Append focal-only native longitudinal handoff, retaining all safety modes')
    parser.add_argument('--comparison-network', type=Path,
                        help='Append a network-only frozen-policy A/B after the unchanged baseline')
    parser.add_argument('--comparison-network-sha256')
    parser.add_argument('--max-wall-seconds', type=float, default=300.)
    args = parser.parse_args()
    output = args.output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT / 'outputs/i110-merge-blocking/v01'):
        raise ValueError('Use canonical project root and a fresh v01 diagnostic child')
    if output.exists() or not 0 < args.max_wall_seconds <= 300 or not 0 < args.horizon_seconds <= 80:
        raise ValueError('Fresh output and bounded horizon/wall-time required')
    if not args.durations or args.durations[0] != .1 or any(not .1 <= d <= 10 for d in args.durations):
        raise ValueError('Run baseline 0.1 first; command durations must be within 0.1..10 seconds')
    cfg = runner.load_config(RUN / 'config.json')
    row = deepcopy(cfg['validation_cases'][19])
    if row['seed'] != 6900019 or digest(RUN / 'policy.pt') != POLICY_SHA:
        raise ValueError('Frozen actor or case identity mismatch')
    output.mkdir(parents=True)
    comparison = None
    if args.comparison_network:
        network = args.comparison_network.resolve()
        if (not network.is_relative_to(ROOT/'outputs/i110-merge-blocking/v01')
                or digest(network) != args.comparison_network_sha256
                or args.durations != [.1] or args.include_native_longitudinal):
            raise ValueError('Network A/B requires verified diagnostic network and unchanged 0.1-s commands')
        comparison = deepcopy(row)
        comparison['case']['network'] = network.relative_to(ROOT).as_posix()
        comparison['case']['network_sha256'] = args.comparison_network_sha256
    variants = [(row,d,f'duration-{d:g}',
                 'baseline' if d == .1 else 'acceleration_command_lifetime') for d in args.durations]
    if args.include_native_longitudinal:
        variants.append((row,None,'native-longitudinal','native_longitudinal_handoff'))
    if comparison:
        variants.append((comparison,.1,'corrected-network','network_replacement'))
    hashes = runner.sources()
    hashes['scripts/probe_i110_merge_blocking.py'] = digest(__file__)
    hashes['scripts/record_highway_environment_v05.py'] = digest(ROOT/'scripts/record_highway_environment_v05.py')
    sumo = find_sumo_binary()
    manifest = dict(command=[sys.executable, *sys.argv], selected_case=row,
        policy_sha256=POLICY_SHA, config_sha256=digest(RUN/'config.json'),
        sources=hashes, sumo=str(sumo), sumo_sha256=digest(sumo),
        sumo_version=subprocess.check_output([str(sumo),'--version'],text=True).splitlines()[0],
        python=platform.python_version(), python_executable=sys.executable,
        python_executable_sha256=digest(sys.executable),
        traci_path=traci.__file__, traci_version=getattr(traci,'__version__',None),
        traci_runtime_sha256={name:digest(Path(traci.__file__).parent/name)
            for name in ('__init__.py','connection.py','_vehicle.py','_lane.py','constants.py')},
        torch_version=torch.__version__, git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],text=True),
        training=False, optimizer_steps=0, horizon_seconds=args.horizon_seconds,
        safety_settings='Unmodified environment speed/lane modes; no traffic removal or teleport override',
        variants=args.durations, native_longitudinal=args.include_native_longitudinal,
        comparison_case=comparison,
        variant_plan=[dict(name=name,duration_s=duration,network=selected['case']['network'],
            network_sha256=selected['case']['network_sha256'],**variant_labels(factor))
            for selected,duration,name,factor in variants],
        link_tuple_fields=['approached_lane','priority','open','foe','via_lane','state','direction','length'],
        next_links_limit='API tests opening at lane speed limit; this is not the exact internal planned arrival-speed decision')
    runner.atomic_json(output/'manifest.json',manifest)
    for name, expected in hashes.items():
        target = output/'source'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
        if digest(target) != expected:
            raise RuntimeError('Source changed while snapshotting diagnostic')
    shutil.copyfile(RUN/'config.json',output/'frozen-config.json')
    (output/'working-tree.diff').write_bytes(subprocess.check_output(
        ['git','diff','HEAD','--binary','--',*hashes],cwd=ROOT))
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(row['seed'])
    actor=torch.jit.load(str(RUN/'policy.pt'),map_location='cpu').eval()
    started=time.monotonic()
    summaries=[]
    for index,(selected,duration,name,factor) in enumerate(variants):
        summary=run_variant(cfg,selected,actor,output/f'{index:02d}-{name}',duration,
                            args.horizon_seconds,started+args.max_wall_seconds,factor=factor)
        summaries.append(summary)
        print(json.dumps({k:v for k,v in summary.items() if k not in ('report','sumo_command')}),flush=True)
        if summary['error']:
            break
    result=dict(variants=summaries, wall_seconds=time.monotonic()-started,
        policy_unchanged=digest(RUN/'policy.pt')==POLICY_SHA,
        changed_sources=[p for p,h in hashes.items() if digest(ROOT/p)!=h])
    runner.atomic_json(output/'summary.json',result)
    return 1 if any(s['error'] for s in summaries) else 0


if __name__ == '__main__':
    raise SystemExit(main())
