"""Fixed training-split, zero-update smooth-reference recordings.

All runs, including failed revisions, are retained and charged to one preparation
ledger. This entry point cannot train or change weights. Native control is an
explicit separate baseline; it is never labeled as learned-policy improvement.
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

import numpy as np
import torch
import traci

from experiments import run_highway_environment_v05 as old
from scripts.record_highway_environment_v05 import collect_episode, choose_clip, EVENT_SCHEMA
from traffic.sumo_backend import find_sumo_binary
from traffic.sumo_warnings import native_emergency_report

PARENT = ROOT/'outputs/highway-environment/v05/train-20261106T024828Z-90b25880'
POLICY_SHA = 'fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec'
NETWORK = ROOT/'outputs/i110-merge-blocking/v01/network-v02/network.net.xml'
NETWORK_SHA = 'ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836'
STUDY = ROOT/'outputs/highway-smooth-control/v02'
FIXTURES = {
    'i110-merge': ('i110-merge', 6800019, 8, 16, 2, 18.),
    'cruise': ('train-3-lane-cruise', 6800020, 8, 16, 0, 18.),
    'following': ('train-3-lane-following', 6800021, 8, 16, 0, 18.),
    'passing': ('train-3-lane-passing', 6800022, 8, 16, 0, 18.),
    'merge': ('train-3-lane-merge', 6800023, 8, 16, 0, 18.),
    'dense-following': ('train-3-lane-following', 6800024, 8, 24, 2, 18.),
}
CONTROLLERS = ('native', 'teacher', 'smooth', 'frozen-v05', 'frozen-smooth')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(cfg):
    result = {}
    for name, (case_id, seed, avs, total, difficulty, speed) in FIXTURES.items():
        case = deepcopy(next(c for c in cfg['training_cases'] if c['id'] == case_id))
        if case['split'] != 'train' or seed in {r['seed'] for r in cfg['validation_cases']}:
            raise ValueError('Preparation must remain on a predeclared training split')
        if case_id.startswith('i110-'):
            case.update(network=NETWORK.relative_to(ROOT).as_posix(), network_sha256=NETWORK_SHA)
        result[name] = dict(case=case, seed=seed, controlled_count=avs, total_vehicles=total,
            difficulty=difficulty, speed_limit_m_s=speed, communication='local', packet_loss=0.,
            traffic_mode='sustained')
    return result


def make_env(cfg, row, output, objective):
    from environments.highway_smooth_control import HighwaySmoothControlEnv
    case = row['case']
    if sha(ROOT/case['network']) != case['network_sha256']:
        raise ValueError('Network checksum changed')
    env = HighwaySmoothControlEnv(ROOT/case['network'], output,
        objective_version=objective, controlled_count=row['controlled_count'],
        total_vehicles=row['total_vehicles'], horizon=800,
        communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        reward_weights=cfg['reward_weights'], control_target_version='v3',
        speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.)
    env.case_id = case['id']
    return env


class ReferenceAdapter:
    def __init__(self, kind, env):
        self.kind, self.env = kind, env

    def __call__(self, values):
        from environments.highway_environment_v05 import teacher_actions as old_teacher
        from traffic.highway_smooth_reference import teacher_actions as smooth_teacher
        observations = {str(i): row for i, row in enumerate(values.detach().cpu().numpy())}
        if self.kind == 'native':
            self.env.control_mode = 'native'
            for agent in self.env.agents:
                self.env.conn.vehicle.setSpeed(agent, -1)
                self.env.conn.vehicle.setLaneChangeMode(agent, 1621)
        choices = (smooth_teacher if self.kind == 'smooth' else old_teacher)(observations)
        return torch.from_numpy(np.stack([choices[str(i)] for i in range(len(observations))]).astype(np.float32))


def source_hashes():
    names = (*old.SOURCES, 'scripts/record_highway_environment_v05.py',
        'scripts/prepare_highway_smooth_control.py', 'environments/highway_smooth_control.py',
        'traffic/highway_smooth_reference.py', 'traffic/highway_smooth_metrics.py')
    return {n: sha(ROOT/n) for n in dict.fromkeys(names)}


def record(cfg, row, kind, output, hashes, deadline):
    output.mkdir(parents=True, exist_ok=False)
    objective = 'smooth_v01' if kind in ('smooth', 'frozen-smooth') else 'v05'
    env = make_env(cfg, row, output/'simulation', objective)
    learned = kind.startswith('frozen-')
    policy = torch.jit.load(str(PARENT/'policy.pt'), map_location='cpu').eval() if learned else ReferenceAdapter(kind, env)
    manifest = dict(schema='highway_smooth_preparation_recording_v01', training=False, optimizer_steps=0,
        controller=kind, objective_version=objective, selected_case=row, seed=row['seed'],
        observation_size=99, max_agents=24, policy_sha256=POLICY_SHA if learned else None,
        reference_actor_sha256=POLICY_SHA, policy_mode='frozen deterministic learned actor' if learned else kind+' scripted baseline',
        source_sha256=hashes, motion_owner='SUMO', position_reference='SUMO front-bumper center',
        source_timestep=.1, horizon_steps=800, horizon_seconds=80., event_schema=EVENT_SCHEMA,
        network=row['case']['network'], network_sha256=row['case']['network_sha256'],
        parent_config_sha256=sha(PARENT/'config.json'), safety_modes=dict(speed=31, lane=1621 if kind=='native' else 512),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short','--',*hashes],text=True),
        python=platform.python_version(), torch=torch.__version__, traci=traci.__version__,
        sumo_version=subprocess.check_output([str(find_sumo_binary()),'--version'],text=True).splitlines()[0],
        allowed_use='Predeclared training-split preparation; no demonstrations exported or optimizer updates',
        renderer_compatible='scripts/render_i110_merge_replay.py')
    old.atomic_json(output/'manifest.json', manifest)
    old.atomic_json(output/'config.json', cfg)
    shutil.copyfile(ROOT/row['case']['network'], output/'network.net.xml')
    for name, digest in hashes.items():
        target=output/'source'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
        if sha(target)!=digest:
            raise RuntimeError('Source changed while snapshotting: '+name)
    (output/'working-tree.diff').write_bytes(subprocess.check_output(['git','diff','HEAD','--binary','--',*hashes]))
    frames, merges, stable, entries, junctions = [], [], [], [], []
    error, report, focal, controlled = None, {}, 'agent_0', []
    started=time.monotonic()
    commands=[]
    original_start=traci.start
    def start(command,*args,**kwargs):
        commands.append(list(map(str,command)))
        return original_start(command,*args,**kwargs)
    traci.start=start
    try:
        report,focal,controlled=collect_episode(env,row,policy,deadline=deadline,frames=frames,
            merge_events=merges,stable_events=stable,entry_events=entries,junction_entry_events=junctions)
    except Exception:
        error=traceback.format_exc()
        try:
            report=env.report()
        except Exception:
            pass
        report.update(primary_horizon_complete=False,accounting_comparison_valid=False)
    finally:
        traci.start=original_start
        env.close()
    report.update(controller=kind,objective_version=objective,policy_sha256=manifest['policy_sha256'],
        training=False,optimizer_steps=0,wall_seconds=time.monotonic()-started,seed=row['seed'],
        native_emergencies=native_emergency_report(env.directory/'sumo-errors.log'),
        merge_events=merges,stable_events=stable,entry_events=entries,junction_entry_events=junctions)
    recording=dict(manifest,frames=frames,focal_vehicle=focal,controlled_ids=controlled,
        merge_events=merges,stable_events=stable,entry_events=entries,junction_entry_events=junctions,
        native_sumo_commands=commands,**(choose_clip(frames,merges) if frames else {}))
    old.atomic_json(output/'evaluation.json',report)
    old.atomic_json(output/'recording.json',recording)
    if error:
        old.atomic_json(output/'failure.json',dict(traceback=error))
    check=dict(controller=kind,error=error,objective_version=objective,training=False,optimizer_steps=0,
        frame_count=len(frames),recording_sha256=sha(output/'recording.json'),
        demand_sha256=sha(env.directory/'routes.rou.xml'),
        policy_unchanged=sha(PARENT/'policy.pt')==POLICY_SHA,
        changed_sources=[name for name,digest in hashes.items() if sha(ROOT/name)!=digest],
        metrics={k:report.get(k) for k in ('intervention_fraction','collisions','teleports','overspeed_steps',
            'emergency_braking_observed','native_emergencies','primary_horizon_complete','passes','stable_merge',
            'completed_trips','requested_trips','mean_fraction_of_speed_limit','smooth_control')},
        wall_seconds=report['wall_seconds'])
    old.atomic_json(output/'recording-check.json',check)
    return check


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixtures',nargs='+',choices=FIXTURES,default=list(FIXTURES))
    parser.add_argument('--controllers',nargs='+',choices=CONTROLLERS,default=['native','teacher','smooth'])
    parser.add_argument('--max-wall-seconds',type=float,default=900.)
    args=parser.parse_args()
    output=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not output.is_relative_to(STUDY) or output.exists():
        raise ValueError('Use canonical root and a fresh v02 output directory')
    if not 0<args.max_wall_seconds<=1800 or sha(PARENT/'policy.pt')!=POLICY_SHA or sha(NETWORK)!=NETWORK_SHA:
        raise ValueError('Invalid bound or frozen input')
    cfg=json.loads((PARENT/'config.json').read_text())
    cfg.update(training_authorized=False,readiness=None)
    selected=rows(cfg)
    hashes=source_hashes()
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(9507)
    output.mkdir(parents=True)
    old.atomic_json(output/'study.json',dict(fixtures=selected,selected_fixtures=args.fixtures,
        controllers=args.controllers,training=False,optimizer_steps=0,source_hashes=hashes,
        max_wall_seconds=args.max_wall_seconds,command=[sys.executable,*sys.argv]))
    ledger_path=STUDY/'preparation-ledger.json'
    ledger=json.loads(ledger_path.read_text()) if ledger_path.exists() else dict(schema='smooth_preparation_ledger_v01',attempts=[])
    if any(a['status']=='running' for a in ledger['attempts']):
        raise ValueError('Resolve interrupted preparation reservation before another run')
    prior=sum(a['charged_wall_seconds'] for a in ledger['attempts'])
    if prior+args.max_wall_seconds>2700:
        raise ValueError('Preparation reservation would consume the final 900-second evaluation reserve')
    entry=dict(output=output.relative_to(ROOT).as_posix(),status='running',charged_wall_seconds=args.max_wall_seconds,
        reserved_wall_seconds=args.max_wall_seconds,source_hashes=hashes)
    ledger['attempts'].append(entry)
    old.atomic_json(ledger_path,ledger)
    started=time.monotonic()
    results=[]
    failure=None
    try:
        for fixture in args.fixtures:
            for controller in args.controllers:
                old.check_deadline(started+args.max_wall_seconds)
                check=record(cfg,selected[fixture],controller,output/fixture/controller,hashes,started+args.max_wall_seconds)
                results.append(dict(fixture=fixture,**check))
                old.atomic_json(output/'results.json',results)
                print(json.dumps(dict(fixture=fixture,controller=controller,error=check['error'],
                    intervention_fraction=check['metrics']['intervention_fraction'],wall_seconds=check['wall_seconds'])),flush=True)
                if check['error'] or check['changed_sources'] or not check['policy_unchanged']:
                    raise RuntimeError('Recording failed integrity or execution check')
    except Exception:
        failure=traceback.format_exc()
    finally:
        elapsed=time.monotonic()-started
        entry.update(status='failed' if failure else 'complete',charged_wall_seconds=elapsed)
        old.atomic_json(ledger_path,ledger)
        summary=dict(training=False,optimizer_steps=0,error=failure,runs=results,
            wall_seconds=elapsed,preparation_total_wall_seconds=prior+elapsed,
            acceptance='Recorded measurements only; aggregate all predeclared gates before declaring readiness')
        old.atomic_json(output/'summary.json',summary)
    return int(failure is not None)


if __name__=='__main__':
    raise SystemExit(main())
