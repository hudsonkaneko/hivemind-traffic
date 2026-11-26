"""The plan follower: a scripted driver that follows v14's own speed target (the plan's yields).

A measurement baseline for v15 Phase 1 only: it never trains, collects or labels. Ported from
round 23's scratch check (`follower2.py`, mode 'v14'; copy in
outputs/test-runs/v15-p0/scratch-copies/), which the v14 plan calls "the scripted car that follows
v14's target (the plan's yields)". The scratch version read that target from a prototype patch
(`followable.py`); here it reads the committed v14 environment's own per-car target cache, whose
`gate_v14` entry is the same quantity: v14's ramped merge target before the follow gate or the
drive-aware target replace it.

Rule, for every controlled car at every decision:
- target: that `gate_v14` target when it is eligible; otherwise the free-road target,
  min(26.8224 m/s, the car's allowed speed);
- request: 3/s x (target - speed), within [-4, 2] m/s^2, then below the acceleration ceiling the
  actor observes (input 43, the raw ceiling / 4.5; not one of v06's faded inputs) minus 0.05, then
  within 0.35 m/s^2 of the car's measured acceleration;
- lanes: it always keeps its lane (lane action 0). So it can't do cases 0 and 4 of the 14 v14
  validation cases, which need a lane change; Phase 1 scores it on the other 12 (user decision,
  November 25, 2026: "Score the plan follower on the other 60 rows").

The driver id carries a version: change the rule or SETTINGS and the id must change with it, since
results are keyed by driver id.
"""
from __future__ import annotations

import math

DRIVER = 'rule:plan-follower-v1'
SETTINGS = dict(gain_per_s=3.0, request_min_m_s2=-4.0, request_max_m_s2=2.0, ceiling_input=43,
                ceiling_scale_m_s2=4.5, ceiling_margin_m_s2=0.05, rate_m_s2=0.35, free_road_m_s=26.8224)
# Cases 0 and 4 of the 14 v14 validation cases (round 24's validation rows 0-13, traffic/v15_benchmark.cases),
# which need a lane change.
LANE_CHANGE_CASES = ('v05-validation-i110-merge', 'ramp-eval-train-2-lane-ramp120-ramp-started')
AGENTS = tuple(f'agent_{i}' for i in range(24))
ACCELERATION_MIN, ACCELERATION_SPAN = -4.5, 3.25   # environments/sumo_mappo_env.action_to_acceleration


def target_speed(state, gate):
    """The speed the follower aims for: v14's gate target if eligible, else the free-road target."""
    target = gate.get('target_m_s') if gate and gate.get('eligible') else None
    if target is None or not math.isfinite(target):
        target = min(SETTINGS['free_road_m_s'], state.get('allowed_speed', SETTINGS['free_road_m_s']))
    return target


def request(state, target, ceiling_input):
    """The follower's acceleration request in m/s^2."""
    s = SETTINGS
    want = max(s['request_min_m_s2'], min(s['request_max_m_s2'], s['gain_per_s'] * (target - state['speed'])))
    want = min(want, float(ceiling_input) * s['ceiling_scale_m_s2'] - s['ceiling_margin_m_s2'])
    measured = state.get('acceleration', 0.)
    return max(measured - s['rate_m_s2'], min(measured + s['rate_m_s2'], want))


def normalized(acceleration):
    """A request as the environment's normalized action in [-1, 1]."""
    return max(-1., min(1., (acceleration - ACCELERATION_MIN) / ACCELERATION_SPAN - 1.))


class PlanFollower:
    """Drives every controlled car of the live environment that `env_source()` returns.

    Same interface as the policies' actors in the episode loop (act, values)."""

    def __init__(self, env_source):
        self.env_source = env_source

    def act(self, local, deterministic=True):
        import torch
        if not deterministic:
            raise ValueError('The plan follower is deterministic: it validates only, it never collects')
        env = self.env_source()
        if env is None:
            raise RuntimeError('The plan follower has no live environment')
        actions = torch.zeros(len(AGENTS), 2)
        for i, agent in enumerate(AGENTS):
            if agent not in env.agents:
                continue
            state = env.states.get(agent)
            if not state:
                raise ValueError(f'{agent} is controlled but has no state')
            gate = (env._v14_cache.get(agent) or {}).get('gate_v14')
            wanted = request(state, target_speed(state, gate), local[i, SETTINGS['ceiling_input']])
            actions[i, 0] = normalized(wanted)
        zeros = torch.zeros(len(AGENTS))
        return actions, zeros, zeros.clone(), actions.clone()

    def values(self, state, indices):
        import torch
        return torch.zeros(len(indices))


def scored_rows(rows):
    """The indices of `rows` the follower is scored on: every row except those of cases 0 and 4."""
    return [i for i, row in enumerate(rows) if row['case']['id'] not in LANE_CHANGE_CASES]
