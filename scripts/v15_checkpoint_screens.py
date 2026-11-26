"""v15 Phase 0: free screens for any checkpoint. They read saved checkpoints and recorded episodes only:
no simulation, no training, nothing charged.

Run from the repository root with the traffic environment's Python:

  python scripts/v15_checkpoint_screens.py settling STATES CASE CKPT [CKPT ...] [--agents agent_0,agent_5]
  python scripts/v15_checkpoint_screens.py sensitivity STATES CASES CKPT [CKPT ...]
  python scripts/v15_checkpoint_screens.py gap ROUND_DIR

STATES is a training round's validation directory with recorded states, for example
outputs/highway-flow-rl/v14/round-21/initial-validation (its case-XX/episode-*/actions.jsonl.gz).
CKPT is a checkpoint path: a v12 checkpoint (dict with 'model'), the pinned 117-field round-16
checkpoint (migrated with policies.highway_flow_v12.migrate_from_flow117), or a TorchScript
export; NAME=PATH sets the printed name.

1. Settling points (ported from the parent session's round-21 scripts r21_components.py and
   r21_fixed2.py). On a recorded state, set the car's measured acceleration (input 1) and its
   previous request (input 5) to a value r, and ask the actor for its request f(r). Where
   f(r) = r the request feedback loop can rest: a settling point if f's slope there is below 1,
   a tipping point otherwise. A merging car whose settling point is strongly negative brakes to a
   stop and stays there. The plan reports it matched case 9's result at all 13 saved checkpoints
   of rounds 20-24.
2. Per-input sensitivity: the gradient of each actor output (the acceleration request in m/s^2
   and the three raw lane scores) with respect to each input, over recorded states. Each input's
   saliency is mean |gradient| x the input's spread over those states. The report gives the
   share held by the self-feedback inputs (1 and 5) and by the v12 target inputs (69-72).
3. Training versus validation gap: for each validation a round recorded (initial-validation,
   periodic-validation-NNNNN), the same gates and events on the training episodes it ran since the
   previous one. Training episodes sample their actions and use the training rows, validation
   runs the deterministic actor on the validation rows, so the gap mixes exploration noise with
   generalization; it is a screen, not a gate.
"""
from __future__ import annotations

import argparse
import gzip
import json
import re
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from policies.highway_flow_v12 import FlowModelV12, migrate_from_flow117  # noqa: E402
from traffic import highway_flow_rl_v12_rewards as v12  # noqa: E402
from traffic import v15_events  # noqa: E402

ACCEL_MIN, ACCEL_SPAN, ACCEL_SCALE = -4.5, 3.25, 4.5      # request = -4.5 + (tanh(raw) + 1) * 3.25
LEGACY_FIRST_LAYER_INPUTS = 86                             # the 117-field actor's first layer
SELF_FEEDBACK = (1, 5)                                     # measured acceleration, previous request
TARGET_INPUTS = tuple(range(v12.TARGET_START, v12.TARGET_STOP))
NAMES = {0: 'speed', 1: 'acceleration', 4: 'allowed_speed', 5: 'previous_request', 36: 'line_distance',
         43: 'acceleration_ceiling', **{v12.TARGET_START + i: f for i, f in enumerate(v12.FEATURES)}}
OUTPUTS = ('request_m_s2', 'lane_minus1', 'lane_stay', 'lane_plus1')


def load_actor(path):
    """A FlowModelV12 in eval mode from a v12 checkpoint, the 117-field checkpoint, or TorchScript."""
    saved = torch.load(Path(path), map_location='cpu', weights_only=False)
    model = FlowModelV12()
    if isinstance(saved, torch.jit.ScriptModule):
        scripted, own, mapped = saved.state_dict(), model.actor.state_dict(), {}
        for key in own:
            matches = [k for k in scripted if k.endswith(key)]
            if len(matches) != 1:
                raise ValueError(f'Cannot map actor tensor {key} in {path}: {matches[:3]}')
            mapped[key] = scripted[matches[0]]
        model.actor.load_state_dict(mapped, strict=True)
    elif saved['model']['actor.mean_net.1.0.weight'].shape[1] == LEGACY_FIRST_LAYER_INPUTS:
        model, _ = migrate_from_flow117(saved['model'])
    else:
        model.load_state_dict(saved['model'], strict=True)
    return model.eval()


def requests(model, observations):
    """The deterministic acceleration request in m/s^2 for each observation row."""
    with torch.no_grad():
        raw = model.actor.mean_net(torch.as_tensor(np.asarray(observations), dtype=torch.float32))
    return (ACCEL_MIN + (raw[:, 0].tanh() + 1.) * ACCEL_SPAN).numpy()


def decision_states(trace_path, agents=None):
    """121-field actor inputs rebuilt from a v14 trace: (states, [(agent, time)])."""
    rows, meta = [], []
    with gzip.open(Path(trace_path), 'rt') as handle:
        for line in handle:
            step = json.loads(line)
            for agent, decision in step['agents'].items():
                if agents is not None and agent not in agents:
                    continue
                try:
                    observation = v12.insert(decision['v04_reward_audit']['pre_action']['observation117'],
                                             decision['v12_audit']['inputs'])
                except (KeyError, TypeError):
                    continue
                rows.append(np.asarray(observation, dtype=np.float32))
                meta.append((agent, float(step['time'])))
    if not rows:
        raise ValueError(f'No actor inputs recorded in {trace_path}')
    return np.stack(rows), meta


def case_trace(states_dir, case):
    paths = sorted(Path(states_dir).glob(f'case-{int(case):02d}/episode-*/actions.jsonl.gz'))
    if len(paths) != 1:
        raise ValueError(f'Expected one recorded episode for case {case} under {states_dir}, found {len(paths)}')
    return paths[0]


def settling_points(model, observation, grid=None):
    """Where the request feedback loop rests on one state: [{request, slope, settles}]."""
    grid = np.round(np.arange(-4.5, 2.0001, .02), 3) if grid is None else np.asarray(grid, dtype=np.float64)
    batch = np.repeat(np.asarray(observation, dtype=np.float32)[None], len(grid), 0)
    batch[:, 1] = grid / ACCEL_SCALE
    batch[:, 5] = grid / ACCEL_SCALE
    f = requests(model, batch).astype(np.float64)
    g = f - grid
    points = []
    for i in range(len(grid) - 1):
        if g[i] == 0 or g[i] * g[i + 1] < 0:
            x = grid[i] if g[i] == 0 else grid[i] - g[i] * (grid[i + 1] - grid[i]) / (g[i + 1] - g[i])
            slope = (f[i + 1] - f[i]) / (grid[i + 1] - grid[i])
            points.append(dict(request=round(float(x), 4), slope=round(float(slope), 4), settles=bool(slope < 1)))
    return dict(at_zero=round(float(f[np.argmin(np.abs(grid))]), 4), points=points)


def settling_screen(models, trace_path, agents=('agent_0', 'agent_5'), samples=4, every_s=1.0):
    """Settling points per checkpoint on the first `samples` states, `every_s` apart, of each agent."""
    states, meta = decision_states(trace_path, agents=set(agents))
    picks = {}
    for row, (agent, time) in zip(states, meta):
        chosen = picks.setdefault(agent, [])
        if len(chosen) < samples and (not chosen or time - chosen[-1][0] >= every_s - 1e-6):
            chosen.append((time, row))
    out = []
    for agent in agents:
        for time, row in picks.get(agent, ()):
            out.append(dict(agent=agent, time_s=time, speed_m_s=round(float(row[0]) * v12.CAP, 3),
                            previous_request_m_s2=round(float(row[5]) * ACCEL_SCALE, 3),
                            checkpoints={name: settling_points(model, row) for name, model in models.items()}))
    return out


def sensitivity(model, states):
    """Per-input saliency of each output over `states`, and the self-feedback and target shares."""
    x = torch.as_tensor(np.asarray(states), dtype=torch.float32).requires_grad_(True)
    raw = model.actor.mean_net(x)
    outputs = torch.cat(((ACCEL_MIN + (raw[:, :1].tanh() + 1.) * ACCEL_SPAN), raw[:, 1:4]), dim=1)
    spread = x.detach().std(dim=0, unbiased=False)
    result = {}
    for k, name in enumerate(OUTPUTS):
        (gradient,) = torch.autograd.grad(outputs[:, k].sum(), x, retain_graph=k < len(OUTPUTS) - 1)
        saliency = gradient.abs().mean(dim=0) * spread
        total = float(saliency.sum())
        share = (saliency / total).numpy() if total > 0 else np.zeros(len(saliency))
        top = np.argsort(-share)[:8]
        result[name] = dict(
            total=total, self_feedback_share=float(share[list(SELF_FEEDBACK)].sum()),
            target_share=float(share[list(TARGET_INPUTS)].sum()),
            mean_abs_gradient={NAMES.get(i, str(i)): float(gradient.abs().mean(dim=0)[i])
                               for i in (*SELF_FEEDBACK, *TARGET_INPUTS)},
            top=[dict(input=int(i), name=NAMES.get(int(i)), share=round(float(share[i]), 4)) for i in top])
    return result


def _validation_labels(round_dir, last_update):
    """(training episodes done, path) for each validation: initial, periodic-validation-N, final."""
    labels = []
    for path in sorted(Path(round_dir).glob('*validation*.json')):
        match = re.fullmatch(r'periodic-validation-(\d+)', path.stem)
        if path.stem == 'initial-validation':
            labels.append((0, path))
        elif match:
            labels.append((int(match.group(1)), path))
        elif path.stem == 'final-validation':
            labels.append((last_update, path))
    return sorted(labels, key=lambda item: (item[0], item[1].stem != 'initial-validation'))


def _side(reports, traces, gate):
    events = [v15_events.episode_events(r, t) for r, t in zip(reports, traces)]
    pooled = v15_events.pool(events)
    passed = sum(int(gate(r)['passed']) for r in reports)
    n = len(reports)
    return dict(episodes=n, passed=passed, pass_rate=passed / n if n else None, decisions=pooled['decisions'],
                events_per_episode={k: v / n for k, v in pooled['events'].items()} if n else {},
                override_episodes_per_1000_decisions=(1000 * pooled['events'].get('override_episodes', 0)
                                                      / pooled['decisions']) if pooled['decisions'] else None)


def first_actor_update(updates):
    """The training episode whose update first changed the actor; episodes up to it ran the start actor."""
    for number, record in updates:
        if not record.get('critic_only') and (record.get('optimization') or {}).get('optimizer_steps'):
            return number
    return None


def gap_screen(round_dir, gate):
    """Each validation next to the training episodes since the previous one, with `gate(report)` as in
    validation. Update record N (updates/N.json) is training episode N, traced in train/episode-(N-1).
    The initial validation is compared with the episodes the starting actor ran (up to and including
    the first actor update)."""
    round_dir = Path(round_dir)
    updates = [(int(p.stem), json.loads(p.read_text())) for p in sorted((round_dir / 'updates').glob('*.json'))]
    traces = {}
    for number, _ in updates:
        paths = sorted((round_dir / 'train' / f'episode-{number - 1:04d}').glob('episode-*/actions.jsonl.gz'))
        if len(paths) != 1:
            raise ValueError(f'Expected one trace for training episode {number} in {round_dir}')
        traces[number] = v15_events.read_trace(paths[0])
    last = max((n for n, _ in updates), default=0)
    start_actor_until = first_actor_update(updates) or last
    out, previous = [], 0
    for at, path in _validation_labels(round_dir, last):
        validation = json.loads(path.read_text())
        directory = path.with_suffix('')
        validation_traces = []
        for i in range(len(validation['reports'])):
            paths = sorted((directory / f'case-{i:02d}').glob('episode-*/actions.jsonl.gz'))
            if len(paths) != 1:
                raise ValueError(f'Expected one trace for {path.stem} case {i}')
            validation_traces.append(v15_events.read_trace(paths[0]))
        if at == 0:
            window = [(n, r) for n, r in updates if n <= start_actor_until]
        else:
            window = [(n, r) for n, r in updates if previous < n <= at]
            previous = at
        training = _side([r['report'] for _, r in window], [traces[n] for n, _ in window], gate)
        checked = _side(validation['reports'], validation_traces, gate)
        out.append(dict(validation=path.stem, after_training_episode=at, training_episodes=[n for n, _ in window],
                        training=training, validation_side=checked,
                        pass_rate_gap=(None if training['pass_rate'] is None or checked['pass_rate'] is None
                                       else training['pass_rate'] - checked['pass_rate'])))
    return out


def _models(specs):
    models = {}
    for spec in specs:
        name, _, path = spec.rpartition('=')
        models[name or Path(path).stem] = load_actor(path)
    return models


def main(argv=None):
    parser = argparse.ArgumentParser(description='v15 free checkpoint screens (no simulation)')
    sub = parser.add_subparsers(dest='command', required=True)
    settle = sub.add_parser('settling')
    settle.add_argument('states', type=Path)
    settle.add_argument('case', type=int)
    settle.add_argument('checkpoints', nargs='+')
    settle.add_argument('--agents', default='agent_0,agent_5')
    settle.add_argument('--samples', type=int, default=4)
    sens = sub.add_parser('sensitivity')
    sens.add_argument('states', type=Path)
    sens.add_argument('cases', help='comma-separated validation case indices')
    sens.add_argument('checkpoints', nargs='+')
    gap = sub.add_parser('gap')
    gap.add_argument('round_dir', type=Path)
    args = parser.parse_args(argv)
    if args.command == 'settling':
        result = settling_screen(_models(args.checkpoints), case_trace(args.states, args.case),
                                 agents=tuple(args.agents.split(',')), samples=args.samples)
    elif args.command == 'sensitivity':
        states = np.concatenate([decision_states(case_trace(args.states, c))[0] for c in args.cases.split(',')])
        result = {name: sensitivity(model, states) for name, model in _models(args.checkpoints).items()}
    else:
        from experiments import run_highway_flow_rl_v12 as runner
        from experiments import run_highway_flow_rl_v14 as runner14
        runner.install = runner14.install
        runner.install()
        result = gap_screen(args.round_dir, runner.base.ruled_gates)
    print(json.dumps(result, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
