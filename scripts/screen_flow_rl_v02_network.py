"""Four-episode, 180-second frozen-policy visibility screen; never train or retry."""
from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
import traci

from experiments import run_highway_flow_rl as runner
from scripts.prepare_flow_rl_v02_network import (SCHEMA, ORIGINAL, POLICY, POLICY_SHA256,
    SEED, digest, save, verify_visibility_only)
from scripts.record_highway_environment_v05 import frame, JunctionEntryTracker, merge_completion_event
from traffic.sumo_backend import find_sumo_binary
from traffic.sumo_warnings import native_emergency_report
from traffic.highway_stage1_teacher import required_gap


def runtime_versions():
    binary = find_sumo_binary()
    version = subprocess.check_output([str(binary), '--version'], text=True).splitlines()[0]
    if '1.27.1' not in version:
        raise ValueError('This declared screen requires the verified SUMO1.27.1 runtime')
    return dict(python=sys.version, traci_file=traci.__file__, sumo_binary=str(binary), sumo_version=version)


def install_conflict_schedule(env, fixture):
    env.fixture = deepcopy(fixture)
    env.network_screen_events = []
    original = env._fixture_commands
    schedule = fixture['native_conflict_schedule']
    released = False
    def commands(self):
        nonlocal released
        original()
        if not getattr(self, '_measurement_phase', False):
            return
        identity = schedule['vehicle_id']
        if identity not in self.states:
            return
        elapsed = self.time - self.start_time
        if elapsed <= schedule['through_time_s']:
            self.conn.vehicle.setSpeedMode(identity, schedule['speed_mode'])
            self.conn.vehicle.setLaneChangeMode(identity, schedule['lane_change_mode'])
            self.conn.vehicle.setSpeed(identity, schedule['speed_m_s'])
        elif not released:
            self.conn.vehicle.setSpeed(identity, schedule['release_speed'])
            self.conn.vehicle.setLaneChangeMode(identity, schedule['release_lane_change_mode'])
            self.network_screen_events.append(dict(time=self.time, event='native_conflict_hold_released', vehicle_id=identity))
            released = True
    env._fixture_commands = types.MethodType(commands, env)


def native_snapshot(env, occupied):
    states = env.states
    guard = {a: dict(speed_mode=env.conn.vehicle.getSpeedMode(a),
        lane_change_mode=env.conn.vehicle.getLaneChangeMode(a)) for a in env.agents if a in states}
    if any(g != dict(speed_mode=31, lane_change_mode=512) for g in guard.values()):
        raise ValueError('Controlled native safety modes changed during screen')
    ego = states.get('agent_0')
    focal = None
    if ego:
        focal = {k: ego[k] for k in ('lane_id', 'lane_position', 'speed', 'acceleration', 'distance', 'allowed_speed')}
        focal.update(lane_remaining_m=env.graph.lanes[ego['lane_id']]['length'] - ego['lane_position'],
            leader=env.conn.vehicle.getLeader('agent_0', 200.),
            native_min_gap_m=env.conn.vehicle.getMinGap('agent_0'),
            next_links=env.conn.vehicle.getNextLinks('agent_0'),
            junction_foes=env.conn.vehicle.getJunctionFoes('agent_0', 100.),
            speed_without_traci=env.conn.vehicle.getSpeedWithoutTraCI('agent_0'))
    blocker = states.get('main_4') if occupied else None
    return dict(time=env.time, focal=focal, controlled_modes=guard,
        blocker={k: blocker[k] for k in ('lane_id', 'lane_position', 'speed', 'acceleration')} if blocker else None,
        blocker_modes=dict(speed_mode=env.conn.vehicle.getSpeedMode('main_4'),
            lane_change_mode=env.conn.vehicle.getLaneChangeMode('main_4')) if blocker else None)


def assess(frames, diagnostics, report, occupied):
    focal = [(f['time'], next((v for v in f['vehicles'] if v['id'] == 'agent_0'), None)) for f in frames]
    present = [(t, v) for t, v in focal if v]
    hard = [d for d in diagnostics if d['before']['focal'] and d['before']['focal']['lane_id']=='mlk_on_0'
        and d['info'] and d['info']['realized_acceleration'] is not None
        and d['info']['realized_acceleration'] <= -4.49]
    conflict = []
    entry_gaps = []
    for d in diagnostics:
        before, after = d['before'], d['after']
        ego, blocker = before['focal'], before['blocker']
        if occupied and ego and blocker and ego['lane_id']=='mlk_on_0' and ego['lane_remaining_m']<=30.:
            lane = blocker['lane_id']
            remaining = d['conflict_approach_length_m'] - blocker['lane_position'] if lane=='sb1_0' else None
            if (remaining is not None and 0.<=remaining<=50.) or lane==':n2_0_0' or (lane=='sb2_0' and blocker['lane_position']<=50.):
                conflict.append(dict(time=before['time'], ramp_remaining_m=ego['lane_remaining_m'],
                    blocker_lane=lane, blocker_position_m=blocker['lane_position'], blocker_speed_m_s=blocker['speed']))
        next_ego = after['focal']
        if ego and next_ego and ego['lane_id']!='sb2_0' and next_ego['lane_id']=='sb2_0':
            leader = next_ego['leader']
            if leader and leader[0]:
                vehicle = next((v for f in frames if abs(f['time']-after['time'])<1e-8
                    for v in f['vehicles'] if v['id']==leader[0]), None)
                if vehicle:
                    required = required_gap(next_ego['speed'], vehicle['speed'])
                    bumper_gap = leader[1] + next_ego['native_min_gap_m']
                    entry_gaps.append(dict(time=after['time'], leader=leader[0],
                        traci_leader_gap_excluding_min_gap_m=leader[1], bumper_gap_m=bumper_gap,
                        required_gap_m=required, margin_m=bumper_gap-required))
    return dict(focal_samples=len(present), focal_last_seen_s=present[-1][0] if present else None,
        focal_min_speed_m_s=min((v['speed'] for _,v in present),default=None),
        focal_distance_m=present[-1][1]['distance_m'] if present else None,
        focal_full_route=report.get('per_car_full_route',{}).get('agent_0') if report else None,
        ramp_hard_brake_samples=len(hard), ramp_hard_brake_exposure_s=.1*len(hard),
        ramp_native_hard_brake_samples=sum(d['info']['native_realization_changed'] for d in hard),
        observed_conflict_samples=len(conflict), observed_conflict=conflict,
        actual_conflict_coverage=(bool(conflict) if occupied else None), entry_gaps=entry_gaps,
        entry_gap_gate=all(g['margin_m']>=-1e-6 for g in entry_gaps),
        safety_gate=bool(report and report['collisions']==0 and report['teleports']==0 and report['overspeed_steps']==0),
        native_emergencies=report.get('native_emergencies') if report else None,
        limitations='Observed development trajectory; no future knowledge, surveyed visibility, held-out or general-safety claim')


def episode(cfg, row, actor, folder, deadline, *, fixture=None):
    folder.mkdir()
    env = runner.env_for(cfg, row, folder/'simulation')
    if fixture:
        install_conflict_schedule(env, fixture)
    frames, diagnostics, merges, entries = [], [], [], []
    report, error = None, None
    started = time.monotonic()
    try:
        obs, _ = env.reset(seed=SEED)
        tracker = JunctionEntryTracker(env.graph, env._controlled_agents())
        tracker.observe(env.states, env.time)
        frames.append(frame(env))
        native_version = env.conn.getVersion()
        while env.agents:
            runner.flow.old.check_deadline(deadline)
            before = native_snapshot(env, fixture is not None)
            local = np.stack([obs.get(a, np.zeros(117,np.float32)) for a in runner.AGENTS])
            with torch.inference_mode():
                actions = actor(torch.from_numpy(local)).numpy()
            chosen = {a: actions[i] for i,a in enumerate(runner.AGENTS) if a in env.agents}
            obs, _, _, _, infos = env.step(chosen)
            after = native_snapshot(env, fixture is not None)
            diagnostics.append(dict(before=before, after=after, info=infos.get('agent_0'),
                focal_action=chosen['agent_0'].tolist() if 'agent_0' in chosen else None,
                conflict_approach_length_m=env.graph.lanes['sb1_0']['length']))
            events = tracker.observe(env.states, env.time)
            entries.extend(events)
            for a, info in infos.items():
                if info.get('merge_completed'):
                    merges.append(merge_completion_event(env,a,info,events))
            frames.append(frame(env))
        original_advance = env._advance
        def advance_tail():
            runner.flow.old.check_deadline(deadline)
            original_advance()
            entries.extend(tracker.observe(env.states,env.time))
            frames.append(frame(env))
        env._advance = advance_tail
        try:
            report = runner.flow.old.primary_accounting(env, partial=False, deadline=deadline)
        finally:
            env._advance = original_advance
        report.update(training=False, optimizer_steps=0, seed=SEED, native_runtime_version=native_version,
            episode_role='occupied' if fixture else 'clear', held_out=False,
            network_screen_events=getattr(env,'network_screen_events',[]))
    except BaseException:
        error = traceback.format_exc()
    finally:
        env.close()
    if report:
        report['native_emergencies'] = native_emergency_report(env.directory/'sumo-errors.log')
    for name in ('network.net.xml','routes.rou.xml'):
        if (env.directory/name).exists():
            shutil.copy2(env.directory/name, folder/name)
    save(folder/'metrics.json', report)
    save(folder/'recording.json', dict(schema=SCHEMA,training=False,optimizer_steps=0,frames=frames,
        controlled_ids=list(env._controlled_agents()), merge_events=merges,junction_entry_events=entries,
        source_timestep_s=.1,position_reference='SUMO front bumper',motion_owner='native SUMO',
        policy_sha256=POLICY_SHA256,seed=SEED))
    with gzip.open(folder/'native-diagnostics.jsonl.gz','wt',encoding='utf-8') as stream:
        for d in diagnostics:
            stream.write(json.dumps(d,allow_nan=False)+'\n')
    result = dict(error=error,wall_seconds=time.monotonic()-started,
        network_sha256=digest(folder/'network.net.xml') if (folder/'network.net.xml').exists() else None,
        demand_sha256=digest(folder/'routes.rou.xml') if (folder/'routes.rou.xml').exists() else None,
        recording_sha256=digest(folder/'recording.json'),
        assessment=assess(frames,diagnostics,report,fixture is not None),
        report_path=(folder/'metrics.json').relative_to(ROOT).as_posix())
    save(folder/'summary.json',result)
    return result


def screen(prepared, *, continue_unattempted=False):
    prepared = Path(prepared).resolve()
    if Path.cwd().resolve()!=ROOT or not prepared.is_relative_to(ROOT/'outputs/highway-flow-rl/v02'):
        raise ValueError('Use canonical checkout and prepared v02 screen directory')
    ledger_path=prepared/'screen-ledger.json'
    if ledger_path.exists() and not continue_unattempted:
        raise ValueError('Screen already allocated; failed attempts count and cannot be retried by this script')
    manifest=json.loads((prepared/'manifest.json').read_text())
    verify_visibility_only(ORIGINAL,prepared/'candidate/network.net.xml')
    if digest(POLICY)!=POLICY_SHA256 or digest(prepared/'candidate/network.net.xml')!=manifest['candidate_network_sha256']:
        raise ValueError('Frozen input changed')
    for path, key in (('original/network.net.xml','original_network_sha256'),
                      ('frozen-config.json','config_sha256'), ('occupied-fixture.json','occupied_fixture_sha256')):
        if digest(prepared/path)!=manifest[key]:
            raise ValueError('Prepared comparison input changed: '+path)
    inherited=0.
    result=dict(training=False,optimizer_steps=0,held_out=False,comparisons={})
    if continue_unattempted:
        ledger=json.loads(ledger_path.read_text())
        if ledger['status']=='running' or ledger['max_episodes']!=4 or ledger['total_wall_seconds']!=180.:
            raise ValueError('Cannot continue an active or different screen budget')
        inherited=ledger['charged_seconds']
        if len(ledger['episodes'])>=4 or inherited>=180.:
            raise ValueError('No unattempted episodes or wall time remain')
        result=json.loads((prepared/'screen-results.json').read_text())
        shutil.copy2(prepared/'screen-results.json',prepared/'screen-results.failed-01.json')
        result.setdefault('previous_errors',[]).append(result.pop('error',None))
    else:
        ledger=dict(schema=SCHEMA,max_episodes=4,total_wall_seconds=180.,episodes=[],charged_seconds=0.)
    started=time.monotonic(); deadline=started+180.-inherited
    ledger['status']='running'
    save(ledger_path,ledger)
    cfg=json.loads((prepared/'frozen-config.json').read_text())
    base=json.loads((prepared/'case.json').read_text())
    fixture=json.loads((prepared/'occupied-fixture.json').read_text())
    try:
        result['runtime']=runtime_versions()
        torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
        actor=torch.jit.load(str(POLICY),map_location='cpu').eval()
        sources={**runner.hashes(), **{p:digest(ROOT/p) for p in ('scripts/prepare_flow_rl_v02_network.py','scripts/screen_flow_rl_v02_network.py')}}
        suffix='-continuation-01' if continue_unattempted else ''
        save(prepared/f'screen-source-hashes{suffix}.json',sources)
        for p in sources:
            dest=prepared/('source'+suffix)/p; dest.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(ROOT/p,dest)
        attempted={row['label'] for row in ledger['episodes']}
        for label in manifest['protocol']['order']:
            if label in attempted:
                continue
            if len(ledger['episodes'])>=4:
                raise ValueError('Four episode allocations exhausted')
            runner.flow.old.check_deadline(deadline)
            population,variant=label.split('-')
            row=deepcopy(base)
            path=prepared/variant/'network.net.xml'
            row['case'].update(network=path.relative_to(ROOT).as_posix(),network_sha256=digest(path))
            allocation=dict(label=label,status='allocated',wall_seconds=0.)
            ledger['episodes'].append(allocation);save(ledger_path,ledger)
            outcome=episode(cfg,row,actor,prepared/label,deadline,fixture=fixture if population=='occupied' else None)
            result['comparisons'][label]=outcome
            allocation.update(status='failed' if outcome['error'] else 'complete',wall_seconds=outcome['wall_seconds'])
            ledger['charged_seconds']=inherited+time.monotonic()-started;save(ledger_path,ledger)
            if outcome['error']:
                raise RuntimeError('Allocated episode failed; preserve evidence and stop without retry')
        comparisons=result['comparisons']
        paired={role: comparisons[role+'-original']['demand_sha256']==comparisons[role+'-candidate']['demand_sha256']
                for role in ('clear','occupied')}
        result['paired_demand_hashes_equal']=paired
        clear_old=comparisons['clear-original']['assessment'];clear_new=comparisons['clear-candidate']['assessment']
        occupied_new=comparisons['occupied-candidate']['assessment']
        result['narrow_screen_passed']=(all(not v['error'] for v in comparisons.values())
            and all(paired.values()) and all(v['assessment']['safety_gate'] for v in comparisons.values())
            and clear_new['ramp_native_hard_brake_samples']<clear_old['ramp_native_hard_brake_samples']
            and all(comparisons['occupied-'+v]['assessment']['actual_conflict_coverage'] for v in ('original','candidate'))
            and occupied_new['entry_gap_gate'])
        result['candidate_network']=manifest['candidate_network']
        result['candidate_network_sha256']=manifest['candidate_network_sha256']
        result['interpretation']=manifest['protocol']['observation_caveat']
        ledger['status']='complete_with_failed_attempts' if any(v['error'] for v in comparisons.values()) else 'complete'
    except BaseException:
        result['error']=traceback.format_exc();ledger['status']='failed'
    finally:
        ledger['charged_seconds']=inherited+time.monotonic()-started
        result['ledger']=ledger
        save(ledger_path,ledger);save(prepared/'screen-results.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--continue-unattempted',action='store_true',help='Consume only unattempted allocations after a preserved failure; never retry a case')
    args=parser.parse_args()
    outcome=screen(args.prepared,continue_unattempted=args.continue_unattempted)
    print(json.dumps({k:v for k,v in outcome.items() if k!='comparisons'},indent=2))
    if outcome.get('error'): sys.exit(1)
