"""Record a frozen flow117 validation case; no optimizer or training."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch

from experiments import run_highway_flow_rl as runner
from scripts.record_highway_environment_v05 import (
    frame, JunctionEntryTracker, merge_completion_event, sha256,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case-index', type=int, default=3)
    args = parser.parse_args()
    output = args.output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs') or output.exists():
        raise ValueError('Use checkout root and a fresh output below outputs/')
    cfg = json.loads(args.config.read_text())
    row = cfg['validation_rows'][args.case_index]
    actor = torch.jit.load(str(args.policy), map_location='cpu').eval()
    torch.set_num_threads(1)
    output.mkdir(parents=True)
    shutil.copy2(__file__, output/'recorder-source.py')
    shutil.copy2(args.config, output/'resolved-config.json')
    env = runner.env_for(cfg, row, output/'episode')
    frames, merges, entries = [], [], []
    started = time.monotonic()
    deadline = started + 120
    try:
        obs, _ = env.reset(seed=row['seed'])
        controlled = list(env._controlled_agents())
        junctions = JunctionEntryTracker(env.graph, controlled)
        junctions.observe(env.states, env.time)
        frames.append(frame(env))
        decisions = 0
        while env.agents:
            runner.flow.old.check_deadline(deadline)
            # Match the evaluator's 24-slot batch, including inactive zero slots.
            local = np.stack([obs.get(a, np.zeros(117, np.float32)) for a in runner.AGENTS])
            if local.shape != (24, 117) or not np.isfinite(local).all():
                raise ValueError('Expected finite 24 x 117 actor observations')
            with torch.inference_mode():
                actions = actor(torch.from_numpy(local))
            if actions.shape != (24, 2) or not torch.isfinite(actions).all():
                raise ValueError('Invalid exported actor outputs')
            actions = actions.numpy()
            chosen = {a: actions[i] for i, a in enumerate(runner.AGENTS) if a in env.agents}
            obs, rewards, _, _, infos = env.step(chosen)
            decisions += len(rewards)
            events = junctions.observe(env.states, env.time)
            entries.extend(events)
            for agent, info in infos.items():
                if info.get('merge_completed'):
                    merges.append(merge_completion_event(env, agent, info, events))
            frames.append(frame(env))
        report = runner.flow.old.primary_accounting(env, partial=False, deadline=deadline)
        report.update(case_id=row['case']['id'], seed=row['seed'], training=False,
                      optimizer_steps=0, agent_transitions=decisions)
        shutil.copy2(env.directory/'network.net.xml', output/'network.net.xml')
        shutil.copy2(env.directory/'routes.rou.xml', output/'routes.rou.xml')
    finally:
        env.close()
    reference = json.loads(args.reference.read_text())
    keys = ['collisions', 'teleports', 'agent_steps', 'overrides', 'completed_trips',
            'overspeed_steps', 'peak_abs_jerk', 'mean_abs_jerk', 'intervention_fraction']
    comparison = {key: dict(original=reference[key], replay=report[key],
                           equal=reference[key] == report[key]) for key in keys}
    comparison['stable_merge'] = dict(equal=reference['stable_merge'] == report['stable_merge'])
    recording = dict(schema='highway_flow_rl_frozen_recording_v1', controller='MAPPO flow117 round two',
        policy_mode='frozen deterministic TorchScript inference', policy_sha256=sha256(args.policy),
        config_sha256=sha256(args.config), seed=row['seed'], controlled_ids=controlled,
        position_reference='SUMO front bumper', motion_owner='native SUMO', source_timestep_s=0.1,
        frames=frames, merge_events=merges, junction_entry_events=entries,
        clip_start_time=0.1, clip_end_time=25.0, training=False, optimizer_steps=0)
    (output/'recording.json').write_text(json.dumps(recording), encoding='utf-8')
    (output/'metrics.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    manifest = dict(training=False, policy_sha256=sha256(args.policy),
        source_recording_sha256=sha256(output/'recording.json'),
        network_sha256=sha256(output/'network.net.xml'),
        reference_report=str(args.reference), reference_sha256=sha256(args.reference),
        comparison=comparison, all_compared_metrics_match=all(v['equal'] for v in comparison.values()),
        frames=len(frames), wall_time_s=time.monotonic()-started,
        limitation='New deterministic replay of the same development case; not held-out evaluation.')
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
