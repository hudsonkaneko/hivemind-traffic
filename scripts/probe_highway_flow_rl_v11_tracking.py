"""Scripted probe: does tracking the gate's own merge-speed target pass the merge-quality gate?

Not training and not a policy. For cars with a pending merge, the acceleration
request of a frozen actor is replaced by a simple tracking law on the gate's own
target, `gain * (target - speed)`, clipped to the command filters' band (measured
acceleration +/- rate bound, feasible ceiling - margin). Lane decisions stay with
the frozen actor and the v11 mask. Every other car is driven by the frozen actor
unchanged. The result shows whether the gate is reachable on a validation case by
speed tracking alone, before any attempt to learn that behavior.

Each probed case is one development validation episode, charged to the shared
ledger as a preparation episode under the command name `scripted_tracking_probe`.
"""
import argparse
import hashlib
import json
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from experiments import run_highway_flow_rl_v11 as runner  # noqa: E402
from experiments.run_highway_flow_rl_v05 import AGENTS  # noqa: E402
from traffic.highway_flow_rl_v03_rewards import map_evidence  # noqa: E402
from traffic.highway_flow_rl_v04_rewards import merge_speed_target  # noqa: E402

base = runner.base


def tracking_request(target, speed, measured, ceiling, *, gain, low=-1.5, high=2., rate_bound=.35, ceiling_margin=.05):
    """Tracking law inside the filters' band (m/s2). The ceiling always wins."""
    upper = min(measured + rate_bound, ceiling - ceiling_margin)
    lower = min(measured - rate_bound, upper)
    return max(lower, min(upper, max(low, min(high, gain * (target - speed)))))


class GuidedActor:
    """Frozen actor whose acceleration is replaced for cars with a pending merge."""

    def __init__(self, model, gain, wait_offset=0., until_merged=False):
        self.model, self.gain, self.wait_offset, self.until_merged = model, gain, wait_offset, until_merged
        self.env, self.replaced = None, 0

    def values(self, *args, **kwargs):
        return self.model.values(*args, **kwargs)

    def act(self, local_obs, deterministic=False):
        actions, log_prob, entropy, latent = self.model.act(local_obs, deterministic=deterministic)
        env = self.env
        for index, agent in enumerate(AGENTS):
            tracker = env.merge_trackers.get(agent) if agent in env.agents else None
            state = env.states.get(agent)
            if tracker is None or tracker.status != 'pending' or state is None:
                continue
            if self.until_merged and tracker.merge_complete_time_s is not None:
                continue   # the frozen actor drives again once the lateral merge is complete
            target, _ = merge_speed_target(state, env.flow_contexts[agent], map_evidence(env.graph, state))
            if not target.get('eligible'):
                continue
            observation = local_obs[index]
            # While the car still waits for its lane change, aim `wait_offset` below the target
            # (inside the gate's 2 m/s band) so a faster car ahead in the target lane pulls away.
            waiting = tracker.merge_complete_time_s is None and agent not in env.maneuvers
            goal = target['target_m_s'] - (self.wait_offset if waiting else 0.)
            request = tracking_request(goal, state['speed'], float(observation[1]) * 4.5,
                                       float(observation[43]) * 4.5, gain=self.gain)
            actions[index, 0] = (request + 4.5) / 3.25 - 1.
            self.replaced += 1
        return actions, log_prob, entropy, latent


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True, help='a declared v11 round configuration (environment settings)')
    parser.add_argument('--checkpoint', type=Path, help='actor checkpoint with a `model` state dict (default: the warm start)')
    parser.add_argument('--case-index', type=int, nargs='+', required=True)
    parser.add_argument('--gain', type=float, default=.6)
    parser.add_argument('--wait-offset', type=float, default=0., help='m/s below the target while waiting to merge')
    parser.add_argument('--until-merged', action='store_true', help='hand back to the actor after the lateral merge')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    started = time.monotonic()
    runner.install()
    config_path = base.project_path(args.config)
    cfg = json.loads(config_path.read_text())
    base.validate_config(cfg)
    output = base.project_path(args.output)
    if not output.is_relative_to(base.OUTPUT_ROOT.resolve()) or output.exists():
        raise ValueError('Use a fresh output directory under outputs/highway-flow-rl/v11')
    checkpoint = base.project_path(args.checkpoint or cfg['warm_start']['path'])
    rows = [cfg['validation_rows'][i] for i in args.case_index]
    fingerprint = hashlib.sha256(config_path.read_bytes()).hexdigest()
    with base.LedgerSession(cfg, 'scripted_tracking_probe', output, fingerprint, started=started, episodes=len(rows)) as session:
        output.mkdir(parents=True)
        torch.set_num_threads(1)
        torch.use_deterministic_algorithms(True)
        torch.manual_seed(cfg['seed'])
        np.random.seed(cfg['seed'])
        random.seed(cfg['seed'])
        model = base.FlowModel()
        model.load_state_dict(torch.load(checkpoint, map_location='cpu', weights_only=False)['model'], strict=True)
        model.eval()
        guided = GuidedActor(model, args.gain, args.wait_offset, args.until_merged)
        env_for = base.env_for

        def captured(cfg_, row, directory):
            guided.env = env_for(cfg_, row, directory)
            return guided.env
        base.env_for = captured
        try:
            result = base.evaluate(cfg, output, 'evaluation', guided, session.deadline, rows=rows)
        finally:
            base.env_for = env_for
        base.save(output / 'probe.json', dict(schema='highway_flow_rl_v11_tracking_probe_v01', scripted=True,
            training=False, gain=args.gain, wait_offset_m_s=args.wait_offset, until_merged=args.until_merged, case_indices=args.case_index, replaced_decisions=guided.replaced,
            checkpoint=checkpoint.relative_to(ROOT).as_posix(),
            checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(), config_sha256=fingerprint,
            gates=[dict(case_id=g['case_id'], passed=g['passed'], failures=g['failures'])
                   for g in result['summary']['case_gates']], elapsed_seconds=session.elapsed()))
    print(json.dumps(dict(replaced_decisions=guided.replaced,
                          gates=[(g['case_id'], g['passed'], g['failures']) for g in result['summary']['case_gates']])))


if __name__ == '__main__':
    main()
