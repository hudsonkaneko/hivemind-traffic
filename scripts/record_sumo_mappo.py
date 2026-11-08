"""Record actual SUMO trajectories with a frozen MAPPO actor for 2D visualization."""
import argparse
import hashlib
import gzip
import shutil
import platform
import subprocess
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch

from environments.sumo_mappo_env import SumoMappoEnv
from experiments.sumo_mappo_rollout import EpisodeMetrics
from experiments.sumo_mappo_support import load_config, DEFAULT_CONFIG, config_digest, sources
from traffic.mappo_sumo import MappoSumoBackend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--export-3d', action='store_true')
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=60)
    parser.add_argument('--seed', type=int, default=400001)
    parser.add_argument('--native-gui', action='store_true', help='Capture actual SUMO viewport PNGs at 10 fps')
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT or not args.output.resolve().is_relative_to(ROOT / 'outputs'):
        parser.error('Run from checkout root and save under outputs/')
    if not 1 <= args.seconds <= 60:
        parser.error('Recording must be between 1 and 60 seconds')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = load_config(args.config)
    (output / 'resolved-config.json').write_text(json.dumps(config, indent=2))
    source_hashes = sources()
    source_hashes['scripts/record_sumo_mappo.py'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    started = time.perf_counter()
    inference_times = []
    replay_frames = []
    journal = gzip.open(output / 'actions.jsonl.gz', 'wt', encoding='utf-8')
    torch.set_num_threads(1)
    actor = torch.jit.load(str(args.policy.resolve()), map_location='cpu').eval()
    backend = MappoSumoBackend(config['environment'], output / 'scenario', gui=args.native_gui)
    env = SumoMappoEnv(config['environment'], backend_factory=lambda: backend)
    colors = [(40, 220, 100, 255), (70, 160, 255, 255),
              (250, 190, 40, 255), (240, 80, 180, 255)]
    metrics = EpisodeMetrics(args.seed)
    snapshots = []
    colored = set()
    frames = output / 'frames'
    if args.native_gui:
        frames.mkdir()
    try:
        obs, _ = env.reset(seed=args.seed)
        if args.export_3d:
            replay = output / '3d'
            replay.mkdir()
            shutil.copyfile(backend.episode_directory / 'network.net.xml', replay / 'network.net.xml')
        runtime_version = backend.version
        view = backend.connection.gui.getIDList()[0] if args.native_gui else None
        for step in range(args.seconds * 10):
            if not env.agents:
                break
            agents = list(env.agents)
            if args.native_gui:
                connection = backend.connection
                for vehicle in set(backend.active_vehicle_ids) - colored:
                    color = colors[int(vehicle[-1])] if vehicle.startswith('agent_') else (170,170,180,255)
                    connection.vehicle.setColor(vehicle, color)
                    colored.add(vehicle)
                positions = [env._result.states[a]['x'] for a in agents]
                connection.gui.setBoundary(view, min(positions)-45, -25, max(positions)+65, 14)
                frame = frames / f'{step:05d}.png'
                # Native size avoids needless framebuffer resizing. The pause
                # lets the GUI process screenshot work before the next TraCI step.
                connection.gui.screenshot(view, str(frame))
                time.sleep(.2)
            inference_start = time.perf_counter()
            with torch.inference_mode():
                actions = actor(torch.from_numpy(np.stack([obs[a] for a in agents]))).numpy()
            inference_times.append(time.perf_counter() - inference_start)
            obs, rewards, terminated, truncated, infos = env.step(dict(zip(agents, actions)))
            metrics.add(dict(rewards=rewards, infos=infos))
            journal.write(json.dumps(dict(step=step+1, time=(step+1)*.1,
                actions={a:actions[i].tolist() for i,a in enumerate(agents)},
                infos={a:{k:v for k,v in info.items() if k!='terminal_state'} for a,info in infos.items()}))+'\n')
            if args.export_3d:
                vehicles = [dict(id=vid, x=state['x'], y=state['y'],
                    heading=float(backend.connection.vehicle.getAngle(vid)), speed=state['speed'],
                    length=state['length'], width=float(backend.connection.vehicle.getWidth(vid)))
                    for vid,state in env._result.states.items()]
                replay_frames.append(dict(time=(step+1)*.1, vehicles=vehicles))
            snapshots.append(dict(time=(step + 1) * .1, vehicles=env._result.states,
                overrides={a: i['safety_intervention'] for a, i in infos.items()}))
            if args.native_gui:
                deadline = time.monotonic() + 5
                while not frame.is_file() or frame.stat().st_size == 0:
                    if time.monotonic() > deadline:
                        raise TimeoutError('Native SUMO screenshot did not finish')
                    time.sleep(.02)
    finally:
        env.close()
        journal.close()
    (output / 'trajectories.json').write_text(json.dumps(snapshots), encoding='utf-8')
    manifest = dict(training=False, optimizer_steps=0, seed=args.seed, fps=10,
        seconds=len(snapshots) / 10, frames=len(snapshots), renderer='2D visualization of actual SUMO positions; not native GUI capture',
        config_sha256=config_digest(config), config_path=str(args.config), source_sha256=source_hashes,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        runtime=dict(python=platform.python_version(), torch=torch.__version__, sumo=runtime_version),
        wall_seconds=time.perf_counter()-started,
        inference_ms=dict(mean=float(np.mean(inference_times)*1000),p95=float(np.percentile(inference_times,95)*1000), maximum=float(max(inference_times)*1000)),
        policy=str(args.policy), policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),
        colors={f'agent_{i}':color for i,color in enumerate(colors)},
        background_color=[170,170,180,255], metrics=metrics.result())
    if args.export_3d:
        payload = dict(seed=args.seed, checkpoint_sha256=manifest['policy_sha256'],
            network_sha256=hashlib.sha256((replay/'network.net.xml').read_bytes()).hexdigest(),
            source_timestep=.1, policy='Frozen MAPPO v02; SUMO motion owner', frames=replay_frames)
        (replay/'recording.json').write_text(json.dumps(payload))
    if args.native_gui:
        manifest['renderer'] = 'Native SUMO GUI viewport screenshots; no synthetic geometry'
        manifest['capture_pause_seconds'] = .2
        manifest['frame_files'] = [p.name for p in sorted(frames.glob('*.png'))]
    (output / 'recording.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(output=str(output), frames=len(snapshots), training=False)))


if __name__ == '__main__':
    main()
