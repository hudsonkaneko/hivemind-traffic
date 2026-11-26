"""The v15 rule expert (plan v15, Stage 1): Level 1, the rule executor, with the rule Level 2.

Built for Phase 2 (user approval, November 25, 2026: "Yes, approve 5 runs"). It drives every
controlled car of the live round-24 environment through the same normalized action [acceleration,
lane] a policy uses, so the environment's limits (speed, following gap, lane end, drive grade,
comfort rate, lane mask) still apply and still count interventions. It validates only: it never
trains, collects or labels. It reads the environment's own state (privileged), as the plan follower
does: the expert is a teacher, not a deployable policy.

Level 2, the rule maneuver choice, every decision:
- **Target speed:** v14's own per-car target when it is eligible (the plan's yields and the round-23
  followable reference at yield entries, which already accounts for the required merge's front and
  rear gaps), otherwise the free-road target min(26.8224 m/s, the allowed speed) less a small margin.
  Then the curvature cap (Level 1).
- **Mandatory lane change:** when the car's own lane no longer reaches its route, toward the side
  whose lane does. It is requested only where the environment's lane mask allows it and MOBIL's
  safety test passes. Once the maneuver has started, the car is committed: no request during it.
- **Discretionary lane change (passing tasks only):** MOBIL's incentive with a politeness factor
  (Kesting, Treiber and Helbing, 2007) to overtake a slower own-lane leader, with the same safety
  test and mask, and only into a lane that still reaches the route (the 2-row code check found a
  pass to the right into an ending lane, which the detour gate fails).
- **Dilemma rule at an ending lane** (the video's yellow-light rule, 3:00): while a mandatory change
  is still pending, the car brakes for the lane end only once stopping there needs at least
  `lane_end_start_m_s2`; the brake is then the constant deceleration that stops it there.
  Recorded per decision: whether stopping would need more than `dilemma_m_s2` (commit zone).

Level 1, the executor, every decision:
- **Speed and following:** IDM's interaction term (Treiber, Hennecke and Helbing, 2000) behind the
  own-lane leader (and, during a lane change, the target lane's leader), taken as the minimum with a
  free-road term gain x (target - speed) (the IDM+ form, Schakel et al., 2010). Its time gap and
  minimum gap are larger than the environment's following bound (2.5 m + 1.5 s), so the bound
  should rarely bind.
- **Limits:** the request stays within `rate_m_s2` of the measured acceleration (jerk <= 3 m/s^3)
  and below the acceleration ceiling the actor observes (input 43) less a margin. When the ceiling is
  below the rate band, the ceiling wins (the environment passes such a request unchanged).
- **Stuck rule** (user decision 2: pull away automatically): a car stopped on a free road (no leader
  within 20 m, target >= 5 m/s; the v14 stop audit's definition) for `stuck_after_s` requests at
  least `pull_away_m_s2` until it moves, still below the ceiling. Every such stop still counts as an
  event in the metrics.
- **Curvature cap** (the video's 2.5 m/s^2 sideways bound, 2:09): from the SUMO lane shapes along
  the route ahead (the map preview carries no curvature: traffic/highway_flow_rl_v03_rewards.map_evidence
  reads speed limits only), the speed now that can still slow at `curve_decel_m_s2` to
  sqrt(lateral / curvature) at each curve point within `curve_lookahead_m`.

Every decision of every car is written to `expert.jsonl` in the episode directory (the target and
its source, each acceleration term, the request, the ceiling, the lane request and the rule state),
so failures can be diagnosed from the record without another charged run.

The driver id carries a version: change the rule or SETTINGS and the id must change with it, since
results are keyed by driver id.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

DRIVER = 'rule:expert-v1'
SETTINGS = dict(
    # Level 1: speed and following
    free_gain_per_s=2.0, free_min_m_s2=-3.0, free_max_m_s2=1.8,
    idm_max_m_s2=1.8, idm_comfort_m_s2=2.0, idm_time_gap_s=1.8, idm_min_gap_m=4.0, idm_range_m=200.,
    free_road_m_s=26.8224, speed_margin_m_s=0.3,
    ceiling_input=43, ceiling_scale_m_s2=4.5, ceiling_margin_m_s2=0.05, rate_m_s2=0.3,
    # dilemma zone at an ending lane
    lane_end_start_m_s2=2.0, lane_end_margin_m=6.0, dilemma_m_s2=3.5,
    # MOBIL's safety test and incentive; the background cars' IDM (traffic/highway_v05_demand PROFILES,
    # 'native_balanced', with SUMO's default 2.5 m minimum gap)
    mobil_safe_m_s2=2.5, mobil_politeness=0.3, mobil_threshold_m_s2=0.2,
    pass_slower_m_s=2.0, pass_range_m=100.,
    background_accel_m_s2=2.0, background_decel_m_s2=4.5, background_tau_s=1.75, background_min_gap_m=2.5,
    # stuck rule
    stuck_speed_m_s=0.1, stuck_after_s=1.0, stuck_free_m=20., stuck_target_m_s=5., pull_away_m_s2=0.6,
    moving_m_s=1.0,
    # curvature cap
    lateral_m_s2=2.5, curve_window_m=20., curve_lookahead_m=150., curve_decel_m_s2=1.0, curve_step_m=5.,
)
AGENTS = tuple(f'agent_{i}' for i in range(24))
ACCELERATION_MIN, ACCELERATION_SPAN = -4.5, 3.25   # environments/sumo_mappo_env.action_to_acceleration
DT = .1


# --------------------------------------------------------------------------- pure rule pieces

def free_term(speed, target, s=SETTINGS):
    """The free-road part: gain x (target - speed), within [free_min, free_max]."""
    return max(s['free_min_m_s2'], min(s['free_max_m_s2'], s['free_gain_per_s'] * (target - speed)))


def idm_interaction(speed, leader_speed, gap, *, a_max=None, b=None, time_gap=None, min_gap=None, s=SETTINGS):
    """IDM's interaction term a_max (1 - (s*/gap)^2), s* = s0 + max(0, v T + v dv / (2 sqrt(a b)))."""
    a_max = s['idm_max_m_s2'] if a_max is None else a_max
    b = s['idm_comfort_m_s2'] if b is None else b
    time_gap = s['idm_time_gap_s'] if time_gap is None else time_gap
    min_gap = s['idm_min_gap_m'] if min_gap is None else min_gap
    desired = min_gap + max(0., speed * time_gap + speed * (speed - leader_speed) / (2. * math.sqrt(a_max * b)))
    return a_max * (1. - (desired / max(gap, .1)) ** 2)


def background_response(speed, leader_speed, gap, free_speed, s=SETTINGS):
    """A background car's IDM acceleration (SUMO's IDM, delta 4) behind a leader at `gap`."""
    a, b = s['background_accel_m_s2'], s['background_decel_m_s2']
    desired = s['background_min_gap_m'] + max(0., speed * s['background_tau_s']
                                              + speed * (speed - leader_speed) / (2. * math.sqrt(a * b)))
    free = 1. - (speed / max(free_speed, .1)) ** 4
    return a * (free - (desired / max(gap, .1)) ** 2)


def lane_end_term(speed, distance, s=SETTINGS):
    """The dilemma rule at an ending lane: (acceleration or None, stopping deceleration needed).

    No brake until stopping `lane_end_margin_m` before the end needs `lane_end_start_m_s2`; then the
    constant deceleration that stops there."""
    room = distance - s['lane_end_margin_m']
    needed = speed * speed / (2. * room) if room > .1 else math.inf
    if needed < s['lane_end_start_m_s2']:
        return None, needed
    return -min(needed, 4.5), needed


def mobil_safe(my_speed, rear, s=SETTINGS):
    """MOBIL's safety test: the target lane's new follower, with us as its leader, brakes no harder
    than `mobil_safe_m_s2`. `rear` is the environment's neighbor record (or None)."""
    if not rear:
        return True, None
    follower = max(0., my_speed + rear['relative_speed_m_s'])
    response = background_response(follower, my_speed, rear['bumper_gap_m'], max(follower, my_speed, 1.))
    return response >= -s['mobil_safe_m_s2'], response


def mobil_incentive(my_speed, target, own_front, side_front, side_rear, s=SETTINGS):
    """MOBIL's incentive (our gain plus politeness x the new follower's change) for a lane change."""
    def mine(front):
        free = free_term(my_speed, target, s)
        if not front or front['bumper_gap_m'] > s['idm_range_m']:
            return free
        return min(free, idm_interaction(my_speed, max(0., my_speed + front['relative_speed_m_s']), front['bumper_gap_m']))
    gain = mine(side_front) - mine(own_front)
    follower_change = 0.
    if side_rear:
        follower = max(0., my_speed + side_rear['relative_speed_m_s'])
        before = (background_response(follower, max(0., my_speed + side_front['relative_speed_m_s']),
                                      side_front['bumper_gap_m'] + side_rear['bumper_gap_m'], max(follower, 1.))
                  if side_front else background_response(follower, follower, 1e6, max(follower, 1.)))
        after = background_response(follower, my_speed, side_rear['bumper_gap_m'], max(follower, 1.))
        follower_change = after - before
    return gain + s['mobil_politeness'] * follower_change


def lane_curvatures(shape, s=SETTINGS):
    """(arc position of each window centre, curvature) along one lane shape: the heading change over
    `curve_window_m` of arc (the whole lane if shorter, at least 5 m), sampled every `curve_step_m`."""
    points = [tuple(map(float, p)) for p in shape]
    if len(points) < 3:
        return []
    arc = [0.]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        arc.append(arc[-1] + math.hypot(x1 - x0, y1 - y0))
    length = arc[-1]
    if length < 5.:
        return []

    def heading(position):
        position = max(0., min(length, position))
        for i in range(len(points) - 1):
            if arc[i + 1] >= position - 1e-9 and arc[i + 1] > arc[i]:
                (x0, y0), (x1, y1) = points[i], points[i + 1]
                return math.atan2(y1 - y0, x1 - x0)
        (x0, y0), (x1, y1) = points[-2], points[-1]
        return math.atan2(y1 - y0, x1 - x0)

    window = min(s['curve_window_m'], length)
    result, centre = [], window / 2.
    while centre <= length - window / 2. + 1e-9:
        turn = heading(centre + window / 2. - 1e-6) - heading(centre - window / 2. + 1e-6)
        turn = (turn + math.pi) % (2. * math.pi) - math.pi
        result.append((centre, abs(turn) / window))
        centre += s['curve_step_m']
    return result


def curve_cap(points, s=SETTINGS):
    """The speed now that can still slow at `curve_decel_m_s2` to sqrt(lateral / curvature) at each
    (distance ahead, curvature) point; inf without a binding curve."""
    cap = math.inf
    for distance, curvature in points:
        if curvature <= 1e-6:
            continue
        local = math.sqrt(s['lateral_m_s2'] / curvature)
        cap = min(cap, math.sqrt(local * local + 2. * s['curve_decel_m_s2'] * max(0., distance)))
    return cap


def compose(speed, measured, target, ceiling, *, leaders=(), lane_end=None, pull_away=False, s=SETTINGS):
    """Level 1's request (m/s^2) and its terms: min(free, IDM behind each leader, lane end), a pull-away
    floor, then the rate band around the measured acceleration, then the ceiling less a margin."""
    terms = dict(free=free_term(speed, target, s))
    interactions = [idm_interaction(speed, leader_speed, gap) for leader_speed, gap in leaders]
    if interactions:
        terms['idm'] = min(interactions)
    if lane_end is not None:
        terms['lane_end'] = lane_end
    wanted = min(terms.values())
    if pull_away:
        wanted = max(wanted, s['pull_away_m_s2'])
    banded = max(measured - s['rate_m_s2'], min(measured + s['rate_m_s2'], wanted))
    request = min(banded, ceiling - s['ceiling_margin_m_s2'])
    return max(-4.5, min(2., request)), terms


def normalized(acceleration):
    """A request as the environment's normalized action in [-1, 1]."""
    return max(-1., min(1., (acceleration - ACCELERATION_MIN) / ACCELERATION_SPAN - 1.))


def _speed(state, neighbor):
    return max(0., state['speed'] + neighbor['relative_speed_m_s'])


# --------------------------------------------------------------------------- the driver

class RuleExpert:
    """Drives every controlled car of the live environment that `env_source()` returns.

    Same interface as the policies' actors in the episode loop (act, values)."""

    def __init__(self, env_source, *, record=True):
        self.env_source = env_source
        self.record = record
        self._episode = None
        self._stopped = {}
        self._curves = {}

    def _new_episode(self, env):
        key = str(getattr(env, 'directory', id(env)))
        if key != self._episode:
            self._episode, self._stopped = key, {}

    def _lane_curves(self, env, lane_id):
        if lane_id not in self._curves:
            self._curves[lane_id] = lane_curvatures(env.conn.lane.getShape(lane_id))
        return self._curves[lane_id]

    def _curve_points(self, env, state):
        points = []
        for lane, start in env.graph.distances(state['lane_id'], state['lane_position'], state['route']).items():
            if start > SETTINGS['curve_lookahead_m']:
                continue
            for centre, curvature in self._lane_curves(env, lane):
                ahead = start + centre
                if -SETTINGS['curve_window_m'] / 2. <= ahead <= SETTINGS['curve_lookahead_m']:
                    points.append((max(0., ahead), curvature))
        return points

    def _target(self, env, agent, state):
        cached = (env._v14_cache.get(agent) or {}).get('target')
        if cached and cached.get('eligible') and cached.get('target_m_s') is not None \
                and math.isfinite(cached['target_m_s']):
            return float(cached['target_m_s']), 'v14'
        free = min(SETTINGS['free_road_m_s'], state.get('allowed_speed', SETTINGS['free_road_m_s']))
        return max(0., free - SETTINGS['speed_margin_m_s']), 'free_road'

    def decide(self, env, agent, ceiling):
        """One car's (request m/s^2, lane delta, record) at this decision."""
        from traffic.highway_flow_context import flow_context
        s = SETTINGS
        state = env.states[agent]
        speed, measured = state['speed'], state.get('acceleration', 0.)
        context = env._context(agent)
        neighbors = context['neighbors']
        own_front = neighbors['own']['front']
        target, source = self._target(env, agent, state)
        plan_target = target
        cap = curve_cap(self._curve_points(env, state))
        target = min(target, cap)
        in_maneuver = agent in env.maneuvers
        mask = env._mask(agent, context)

        # Level 2: which lane
        flow = flow_context(env.graph, state, env.states)
        own_route = flow['corridors']['own']['route']
        mandatory = None
        if own_route.get('known') and not own_route.get('reachable', True):
            for side, delta in (('left', 1), ('right', -1)):
                route = flow['corridors'][side]['route']
                if flow['corridors'][side]['lane_id'] is not None and route.get('reachable'):
                    if mandatory is None or delta == int(flow['entry'].get('lane_delta') or 0):
                        mandatory = delta
        lane, reason, safety = 0, None, None
        if not in_maneuver:
            candidates = []
            if mandatory is not None:
                candidates.append((mandatory, 'mandatory'))
            elif env.task == 'passing' and own_front and own_front['bumper_gap_m'] <= s['pass_range_m'] \
                    and _speed(state, own_front) < plan_target - s['pass_slower_m_s']:
                for side, delta in (('left', 1), ('right', -1)):
                    # only into a lane that still reaches the route (no optional ending-lane detour)
                    if not context['lane_exists'][side] or not flow['corridors'][side]['route'].get('reachable'):
                        continue
                    gain = mobil_incentive(speed, plan_target, own_front, neighbors[side]['front'], neighbors[side]['rear'])
                    if gain > s['mobil_threshold_m_s2']:
                        candidates.append((delta, 'passing'))
            for delta, why in candidates:
                side = 'left' if delta > 0 else 'right'
                safe, response = mobil_safe(speed, neighbors[side]['rear'])
                safety = dict(delta=delta, safe=safe, follower_response=response, mask=bool(mask[delta + 1]))
                if safe and mask[delta + 1]:
                    lane, reason = delta, why
                    break

        # Level 1: leaders, lane end, stuck rule
        leaders = []
        if own_front and own_front['bumper_gap_m'] <= s['idm_range_m']:
            leaders.append((_speed(state, own_front), own_front['bumper_gap_m']))
        maneuver = env.maneuvers.get(agent) if in_maneuver else None
        if maneuver is not None:
            front = neighbors['left' if maneuver['direction'] > 0 else 'right']['front']
            if front and front['bumper_gap_m'] <= s['idm_range_m']:
                leaders.append((_speed(state, front), front['bumper_gap_m']))
        lane_end, stopping = None, None
        distance = env.merge_distance(state)
        if mandatory is not None and lane == 0 and not in_maneuver and distance is not None:
            lane_end, stopping = lane_end_term(speed, distance)
        free_road = not own_front or own_front['bumper_gap_m'] > s['stuck_free_m']
        stopped = speed < s['stuck_speed_m_s'] and free_road and plan_target >= s['stuck_target_m_s']
        since = self._stopped.get(agent)
        if stopped:
            since = self._stopped.setdefault(agent, env.time)
        elif speed >= s['moving_m_s'] or not free_road:
            self._stopped.pop(agent, None)
            since = None
        pull_away = since is not None and env.time - since >= s['stuck_after_s'] - 1e-9
        request, terms = compose(speed, measured, target, ceiling, leaders=leaders, lane_end=lane_end,
                                 pull_away=pull_away)
        record = dict(t=round(env.time, 3), agent=agent, v=round(speed, 4), a=round(measured, 4),
                      target=round(target, 4), source=source, plan=round(plan_target, 4),
                      curve=None if math.isinf(cap) else round(cap, 3), ceiling=round(ceiling, 4),
                      terms={k: round(v, 4) for k, v in terms.items()}, request=round(request, 4),
                      lane=lane, reason=reason, mandatory=mandatory, maneuver=in_maneuver,
                      mask=[int(m) for m in mask], safety=safety,
                      lane_end_stop=None if stopping is None or math.isinf(stopping) else round(stopping, 3),
                      dilemma_commit=bool(stopping is not None and stopping > s['dilemma_m_s2']),
                      pull_away=pull_away, task=env.task)
        return request, lane, record

    def act(self, local, deterministic=True):
        import torch
        if not deterministic:
            raise ValueError('The rule expert is deterministic: it validates only, it never collects')
        env = self.env_source()
        if env is None:
            raise RuntimeError('The rule expert has no live environment')
        self._new_episode(env)
        actions = torch.zeros(len(AGENTS), 2)
        records = []
        for i, agent in enumerate(AGENTS):
            if agent not in env.agents:
                continue
            if not env.states.get(agent):
                raise ValueError(f'{agent} is controlled but has no state')
            ceiling = float(local[i, SETTINGS['ceiling_input']]) * SETTINGS['ceiling_scale_m_s2']
            request, lane, record = self.decide(env, agent, ceiling)
            actions[i, 0] = normalized(request)
            actions[i, 1] = lane
            records.append(record)
        if self.record and records and getattr(env, 'directory', None) is not None:
            with open(Path(env.directory) / 'expert.jsonl', 'a', encoding='utf-8') as handle:
                for record in records:
                    handle.write(json.dumps(record, separators=(',', ':')) + '\n')
        zeros = torch.zeros(len(AGENTS))
        return actions, zeros, zeros.clone(), actions.clone()

    def values(self, state, indices):
        import torch
        return torch.zeros(len(indices))
