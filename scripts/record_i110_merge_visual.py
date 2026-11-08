"""Record one predeclared frozen-policy I-110 merge, without learning."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch
import traci

from environments.highway_stage1_env import HighwayStage1Env
from experiments.run_highway_stage1 import primary_accounting
from traffic.sumo_warnings import native_emergency_report

POLICY = ROOT / 'policies/versions/highway-stage1-v03/policy.pt'
NETWORK = ROOT / 'outputs/i110-merge/v01/network/network.net.xml'
POLICY_SHA = 'a6d59d3acf58fcca9d7fc33406cb61ca3863d1a3dffad6dd316b195a66c7166f'
NETWORK_SHA = 'a216efd0a451845c19147437284fe0014d503c8015aca1d29bb4b4b8e1ac8623'
SEED = 6400019


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def frame(env):
    v = env.conn.vehicle
    vehicles = []
    for identifier, state in sorted(env.states.items()):
        vehicles.append(dict(id=identifier, x=state['x'], y=state['y'],
            heading=v.getAngle(identifier), speed=state['speed'],
            acceleration=state['acceleration'], length=state['length'],
            width=v.getWidth(identifier), lane_id=state['lane_id'],
            controlled=identifier in env._controlled_agents(),
            merge_complete=identifier in env.merged))
    return dict(time=env.time, vehicles=vehicles)


def main():
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from the canonical project root')
    if sha(POLICY) != POLICY_SHA or sha(NETWORK) != NETWORK_SHA:
        raise ValueError('Frozen policy/network checksum mismatch')
    output = ROOT / 'outputs/i110-merge-visual/v01' / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True)
    started = time.monotonic()
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(SEED)
    policy = torch.jit.load(str(POLICY), map_location='cpu').eval()
    env = HighwayStage1Env(NETWORK, output / 'simulation', horizon=800,
        through_lanes=4, task='merge', density='medium', communication='local',
        packet_loss=0., control_target_version='v2')
    source_names = ['scripts/record_i110_merge_visual.py',
        'environments/highway_stage1_env.py', 'environments/i110_merge_env.py',
        'traffic/highway_merge_context.py', 'traffic/i110_policy_interface.py',
        'traffic/sumo_warnings.py']
    provenance = dict(seed=SEED, policy=str(POLICY.relative_to(ROOT)),
        checkpoint_sha256=POLICY_SHA, network=str(NETWORK.relative_to(ROOT)),
        network_sha256=NETWORK_SHA, control_target_version='v2',
        selection='Lowest pooled merge interventions among retained compatible Stage 1 actors; not universally best or accepted',
        training=False, optimizer_steps=0, motion_owner='SUMO', focal_vehicle='agent_2',
        source_timestep=.1, horizon_steps=800, communication='local', packet_loss=0.,
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True),
        python=platform.python_version(), torch=torch.__version__, traci=traci.__version__,
        source_sha256={p: sha(ROOT / p) for p in source_names})
    write(output / 'manifest.json', provenance)
    for name in source_names:
        target = output / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    frames, events = [], []
    report = None
    try:
        obs, _ = env.reset(seed=SEED)
        frames.append(frame(env))
        while env.agents:
            if time.monotonic() - started > 180:
                raise TimeoutError('Bounded visual rollout exceeded 180 wall seconds')
            ids = list(env.agents)
            with torch.inference_mode():
                actions = policy(torch.tensor(np.stack([obs[a] for a in ids]), dtype=torch.float32))
            if actions.shape != (len(ids), 2) or not torch.isfinite(actions).all():
                raise ValueError('Invalid frozen actor output')
            obs, _, _, _, infos = env.step({a: actions[i].numpy() for i, a in enumerate(ids)})
            for agent, info in infos.items():
                if info['merge_completed']:
                    events.append(dict(agent=agent, time=env.time, lane_id=env.states[agent]['lane_id'],
                        x=env.states[agent]['x'], y=env.states[agent]['y']))
            frames.append(frame(env))
        report = primary_accounting(env, partial=False, deadline=started + 180)
    except Exception as exc:
        write(output / 'failure.json', dict(error=repr(exc), frames=len(frames), merge_events=events))
        raise
    finally:
        env.close()
    report['native_emergencies'] = native_emergency_report(env.directory / 'sumo-errors.log')
    report.update(merge_targets=['agent_2', 'agent_3'], merge_events=events,
        policy_sha256=POLICY_SHA, seed=SEED, optimizer_steps=0, training=False,
        episode_directory=str(env.directory.relative_to(ROOT)),
        wall_seconds=time.monotonic() - started, full_report_is_separate_from_video_crop=True)
    write(output / 'evaluation.json', report)
    t0, t1 = frames[0]['time'], frames[-1]['time']
    focal_event = next((e for e in events if e['agent'] == 'agent_2'), None)
    if focal_event:
        clip_start, clip_end = max(t0, focal_event['time'] - 8), min(t1, focal_event['time'] + 8)
    else:
        clip_start, clip_end = t0, min(t1, t0 + 25)
    recording = dict(provenance, frames=frames, merge_events=events,
        clip_start_time=clip_start, clip_end_time=clip_end,
        crop_rule='Focal agent_2 first merge minus/plus eight seconds; if absent show first 25 seconds without rerolling',
        rendering='3D visualization of actual SUMO poses; not physical closed-loop validation')
    write(output / 'recording.json', recording)
    shutil.copyfile(NETWORK, output / 'network.net.xml')
    if sha(POLICY) != POLICY_SHA:
        raise RuntimeError('Frozen actor changed')
    write(output / 'recording-check.json', dict(frames=len(frames),
        recording_sha256=sha(output / 'recording.json'), network_sha256=sha(output / 'network.net.xml'),
        policy_unchanged=True, monotonic_time=all(a['time'] < b['time'] for a, b in zip(frames, frames[1:])),
        source_unchanged=all(sha(ROOT / p) == h for p, h in provenance['source_sha256'].items())))
    print(json.dumps(dict(output=str(output), merge_events=events,
        interventions=report['intervention_fraction'], collisions=report['collisions'],
        teleports=report['teleports'], native_emergencies=report['native_emergencies'],
        clip_start_time=clip_start, clip_end_time=clip_end), indent=2))


if __name__ == '__main__':
    main()
