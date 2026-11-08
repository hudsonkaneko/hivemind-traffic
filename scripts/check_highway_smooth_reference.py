"""One authorized training-only actor/teacher reference pair, with no updates.

The existing v3 teacher consumes the same 99-field observations as the frozen
actor. This is a reference assessment, not demonstrations, tuning or training.
No native follow-speed queries, controller changes or safety bypasses occur.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
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
    sys.path.insert(0,str(ROOT))

import numpy as np
import torch
import traci
from environments.highway_environment_v05 import teacher_actions
from experiments import run_highway_environment_v05 as runner
from scripts.record_highway_environment_v05 import collect_episode, choose_clip, sha256
from scripts.diagnose_highway_smooth_control import analyze
from traffic.sumo_backend import find_sumo_binary
from traffic.sumo_warnings import native_emergency_report

RUN = ROOT/'outputs/highway-environment/v05/train-20261106T024828Z-90b25880'
POLICY_SHA = 'fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec'
NETWORK = ROOT/'outputs/i110-merge-blocking/v01/network-v02/network.net.xml'
NETWORK_SHA = 'ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836'


class ExistingTeacherAdapter:
    """Tensor transport only: delegate all choices to the existing v3 teacher."""
    def __call__(self, values):
        rows = values.detach().cpu().numpy()
        observations = {str(index):row for index,row in enumerate(rows)}
        actions = teacher_actions(observations)
        return torch.from_numpy(np.stack([actions[str(index)] for index in range(len(rows))]).astype(np.float32))


def run_one(cfg,row,controller,kind,output,manifest,deadline):
    output.mkdir()
    details = dict(manifest,controller=kind,
        policy_sha256=POLICY_SHA if kind=='learned' else None,
        reference_actor_sha256=POLICY_SHA,
        policy_mode='frozen TorchScript deterministic inference' if kind=='learned' else
                    'existing observation-only v3 teacher via tensor transport; no learned actor invocation',
        teacher_source_sha256={name:manifest['source_sha256'][name] for name in
            ('traffic/highway_stage1_teacher.py','environments/highway_environment_v05.py')} if kind=='teacher' else None)
    runner.atomic_json(output/'manifest.json',details)
    runner.atomic_json(output/'config.json',cfg)
    shutil.copyfile(NETWORK,output/'network.net.xml')
    env = runner.env_for(cfg,output/'simulation',row)
    frames,merges,stable,entries,junctions = [],[],[],[],[]
    error,report,focal,controlled = None,None,'agent_0',[]
    original_start,commands = traci.start,[]
    def start(command,*args,**kwargs):
        commands.append(list(map(str,command)))
        return original_start(command,*args,**kwargs)
    traci.start = start
    started = time.monotonic()
    try:
        report,focal,controlled = collect_episode(env,row,controller,deadline=deadline,
            frames=frames,merge_events=merges,stable_events=stable,entry_events=entries,
            junction_entry_events=junctions)
    except Exception:
        error = traceback.format_exc()
        try:
            report = env.report()
        except Exception:
            report = {}
        report.update(primary_horizon_complete=False,accounting_comparison_valid=False)
    finally:
        traci.start = original_start
        env.close()
    # collect_episode's legacy label is learned; replace it before any artifact
    # is written so teacher control is never presented as frozen actor behavior.
    report.update(controller=kind,policy_mode=details['policy_mode'],
        policy_sha256=details['policy_sha256'],reference_actor_sha256=POLICY_SHA,
        training=False,optimizer_steps=0,seed=row['seed'],
        wall_seconds=time.monotonic()-started,
        native_emergencies=native_emergency_report(env.directory/'sumo-errors.log'),
        merge_events=merges,stable_events=stable,entry_events=entries,junction_entry_events=junctions,
        episode_directory=str(env.directory.relative_to(ROOT)))
    recording = dict(details,frames=frames,focal_vehicle=focal,controlled_ids=controlled,
        merge_events=merges,stable_events=stable,entry_events=entries,junction_entry_events=junctions,
        native_sumo_commands=commands,
        **(choose_clip(frames,merges) if frames else {}))
    runner.atomic_json(output/'evaluation.json',report)
    runner.atomic_json(output/'recording.json',recording)
    runner.atomic_json(output/'native-command.json',commands)
    check = dict(error=error,primary_horizon_complete=report.get('primary_horizon_complete',False),
        frame_count=len(frames),first_time_s=frames[0]['time'] if frames else None,
        last_time_s=frames[-1]['time'] if frames else None,
        demand_sha256=sha256(env.directory/'routes.rou.xml'),training=False,optimizer_steps=0)
    if error:
        runner.atomic_json(output/'failure.json',dict(traceback=error))
    else:
        residuals=analyze(output)
        residuals.update(allowed_use='Training-split reference assessment only; no demonstrations exported',
            evidence_scope='One predeclared training-split seed, separate from validation/test',
            recorded_controller=kind)
        residuals['objective_review']['evidence_scope']='One predeclared training-split reference case; no learning or generalization result'
        runner.atomic_json(output/'control-residuals.json',residuals)
        check.update(absolute_realized_jerk=residuals['overall']['residuals']['absolute_realized_jerk_m_s3'],
            absolute_request_target_error=residuals['overall']['residuals']['absolute_request_target_error_m_s2'])
    check.update(controller=kind,intervention_fraction=report.get('intervention_fraction'),
        stable_merge={k:v for k,v in report.get('stable_merge',{}).items() if k!='per_agent'},
        requested_trips=report.get('requested_trips'),completed_trips=report.get('completed_trips'),
        minimum_headway_s=report.get('minimum_headway_s'),minimum_ttc_s=report.get('minimum_ttc_s'),
        collisions=report.get('collisions'),teleports=report.get('teleports'),
        overspeed_steps=report.get('overspeed_steps'),wall_seconds=report['wall_seconds'])
    runner.atomic_json(output/'recording-check.json',check)
    return check


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-wall-seconds',type=float,default=180.)
    args=parser.parse_args()
    output=args.output.resolve()
    if (Path.cwd().resolve()!=ROOT or not output.is_relative_to(ROOT/'outputs/highway-smooth-control/v01')
            or output.exists() or not 0 < args.max_wall_seconds <= 180):
        raise ValueError('Fresh v01 output, canonical root, and at most 180 seconds required')
    if sha256(RUN/'policy.pt')!=POLICY_SHA or sha256(NETWORK)!=NETWORK_SHA:
        raise ValueError('Frozen actor/network checksum mismatch')
    cfg=runner.load_config(RUN/'config.json')
    original_case=next(c for c in cfg['training_cases'] if c['id']=='i110-merge')
    case=deepcopy(original_case)
    if case['split']!='train' or any(row['seed']==6800019 for row in cfg['validation_cases']):
        raise ValueError('Reference must use the declared training split, not validation demand')
    case.update(network=str(NETWORK.relative_to(ROOT)),network_sha256=NETWORK_SHA)
    row=dict(case=case,controlled_count=8,total_vehicles=16,seed=6800019,
        speed_limit_m_s=18.,communication='local',packet_loss=0.,difficulty=2,traffic_mode='sustained')
    cfg=deepcopy(cfg)
    cfg.update(training_authorized=False,readiness=None)
    sources=runner.sources()
    for name in ('scripts/check_highway_smooth_reference.py','scripts/record_highway_environment_v05.py',
                 'scripts/diagnose_highway_smooth_control.py'):
        sources[name]=sha256(ROOT/name)
    sumo=find_sumo_binary()
    manifest=dict(schema='highway_smooth_control_frozen_reference_pair_v01',training=False,optimizer_steps=0,
        seed=row['seed'],selected_case=row,parent_training_case=original_case,
        parent_config=str((RUN/'config.json').relative_to(ROOT)),parent_config_sha256=sha256(RUN/'config.json'),
        source_sha256=sources,network_sha256=NETWORK_SHA,
        selection='Exactly one predeclared training-only seed; two controllers; no seed search or optimization',
        purpose='Assess whether the existing teacher is an adequate intervention and comfort reference before proposing training',
        command=[sys.executable,*sys.argv],python=platform.python_version(),torch=torch.__version__,
        python_executable=sys.executable,python_executable_sha256=sha256(sys.executable),
        traci_path=traci.__file__,traci_version=traci.__version__,sumo=str(sumo),sumo_sha256=sha256(sumo),
        sumo_version=subprocess.check_output([str(sumo),'--version'],text=True).splitlines()[0],
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],text=True),
        source_timestep=.1,horizon_steps=800,horizon_seconds=80.,max_wall_seconds=args.max_wall_seconds,
        safety_settings='Original speedMode31/laneChangeMode512 and all existing guards; background drivers unchanged',
        demonstration_export=False,motion_owner='SUMO',position_reference='SUMO front-bumper center')
    output.mkdir(parents=True)
    runner.atomic_json(output/'manifest.json',manifest)
    runner.atomic_json(output/'resolved-case.json',row)
    for name,expected in sources.items():
        target=output/'source'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
        if sha256(target)!=expected:
            raise RuntimeError('Source changed while snapshotting')
    shutil.copyfile(RUN/'policy.pt',output/'frozen-actor.pt')
    shutil.copyfile(RUN/'config.json',output/'parent-config.json')
    (output/'working-tree.diff').write_bytes(subprocess.check_output(['git','diff','HEAD','--binary','--',*sources],cwd=ROOT))
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(row['seed'])
    actor=torch.jit.load(str(RUN/'policy.pt'),map_location='cpu').eval()
    started=time.monotonic()
    checks=[]
    for name,controller in [('learned',actor),('teacher',ExistingTeacherAdapter())]:
        check=run_one(cfg,row,controller,name,output/name,manifest,started+args.max_wall_seconds)
        checks.append(check)
        print(json.dumps(check,allow_nan=False),flush=True)
        if check['error']:
            break
    report=dict(runs=checks,wall_seconds=time.monotonic()-started,training=False,optimizer_steps=0,
        matched_demand=len(checks)==2 and checks[0]['demand_sha256']==checks[1]['demand_sha256'],
        policy_unchanged=sha256(RUN/'policy.pt')==POLICY_SHA,
        changed_sources=[name for name,expected in sources.items() if sha256(ROOT/name)!=expected],
        success_not_assumed='This single reference check does not authorize training or establish held-out performance')
    runner.atomic_json(output/'summary.json',report)
    return int(len(checks)!=2 or any(c['error'] for c in checks))


if __name__=='__main__':
    raise SystemExit(main())
