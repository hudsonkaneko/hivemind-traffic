"""Record actual SUMO trajectories with a frozen MAPPO actor for 2D visualization."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch

from environments.sumo_mappo_env import SumoMappoEnv
from experiments.sumo_mappo_rollout import EpisodeMetrics
from experiments.sumo_mappo_support import load_config
from traffic.mappo_sumo import MappoSumoBackend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=60)
    parser.add_argument('--seed', type=int, default=400001)
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT or not args.output.resolve().is_relative_to(ROOT / 'outputs'):
        parser.error('Run from checkout root and save under outputs/')
    if not 1 <= args.seconds <= 60:
        parser.error('Recording must be between 1 and 60 seconds')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = load_config()
    torch.set_num_threads(1)
    actor = torch.jit.load(str(args.policy.resolve()), map_location='cpu').eval()
    backend = MappoSumoBackend(config['environment'], output / 'scenario')
    env = SumoMappoEnv(config['environment'], backend_factory=lambda: backend)
    colors = [(40, 220, 100, 255), (70, 160, 255, 255),
              (250, 190, 40, 255), (240, 80, 180, 255)]
    metrics = EpisodeMetrics(args.seed)
    snapshots = []
    try:
        obs, _ = env.reset(seed=args.seed)
        for step in range(args.seconds * 10):
            if not env.agents:
                break
            agents = list(env.agents)
            with torch.inference_mode():
                actions = actor(torch.from_numpy(np.stack([obs[a] for a in agents]))).numpy()
            obs, rewards, terminated, truncated, infos = env.step(dict(zip(agents, actions)))
            metrics.add(dict(rewards=rewards, infos=infos))
            snapshots.append(dict(time=(step + 1) * .1, vehicles=env._result.states,
                overrides={a: i['safety_intervention'] for a, i in infos.items()}))
    finally:
        env.close()
    (output / 'trajectories.json').write_text(json.dumps(snapshots), encoding='utf-8')
    manifest = dict(training=False, optimizer_steps=0, seed=args.seed, fps=10,
        seconds=len(snapshots) / 10, frames=len(snapshots), renderer='2D visualization of actual SUMO positions; not native GUI capture',
        policy=str(args.policy), policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),
        colors={f'agent_{i}':color for i,color in enumerate(colors)},
        background_color=[170,170,180,255], metrics=metrics.result())
    (output / 'recording.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(output=str(output), frames=len(snapshots), training=False)))


if __name__ == '__main__':
    main()
