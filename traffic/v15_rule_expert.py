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
- **Lane-change acceptance (v2):** besides MOBIL's safety test on the new follower (now 1.5 m/s^2),
  the new leader must leave `new_front_margin_m_s` of headroom under the environment's following
  bound, and IDM behind it must need no more than `new_front_idm_m_s2`.
- **Gap selection at a yield entry (v2):** while the environment's yield entry is active (a minor
  approach joins a lane through a junction), the moving cars that reach the shared lane after us
  form a stream. SUMO's mainline cars ignore a minor-approach car until it crosses the stop line, then
  follow it by distance (the code check: a mainline car braked at 9 m/s^2 the step our ramp car
  crossed at 4.4 m/s), so each gap is judged at the moment we would cross the line: the car next
  behind must brake at most `entry_safe_m_s2` by SUMO's IDM with its own parameters (the merge gate
  allows neighbours down to -3 m/s^2), and the car ahead must have passed `yield_headway_s` earlier.
  Going now is a run-up at `run_up_m_s2`. To wait for a later gap the car takes the gentlest way
  that makes it acceptable: hold the steady speed that arrives on time, else crawl at `crawl_m_s`
  and run up; it never crosses the line early, and commits once the run-up from where it is would
  not arrive before the planned time. A started commit holds while that follower would brake at most
  `entry_abort_m_s2`; if stopping would need more than `dilemma_m_s2` it commits (the dilemma rule).
  A car that is waiting is not a projected leader for other cars; one that has committed is.

Level 1, the executor, every decision:
- **Speed and following:** IDM's interaction term (Treiber, Hennecke and Helbing, 2000) behind the
  own-lane leader (and, during a lane change, v3, the leaders of the maneuver's source and target lanes
  found by lane identity, never the lane beyond the target; and, v2, the nearest car
  from another approach that reaches a shared lane ahead before us, at its gap when it reaches that
  lane and so becomes the environment's own-lane leader), taken as the minimum with a
  free-road term gain x (target - speed) (the IDM+ form, Schakel et al., 2010).
- **Following-bound anticipation (v2):** run 1 showed that IDM alone let the environment's own
  following bound, vL + 0.5 (gap - 2.5 - 1.5 v), clamp cars closing on slower leaders, with jerk
  peaks above 5 m/s^3. Behind every leader the request is also at most the acceleration that keeps
  the speed `bound_margin_m_s` under that bound (the leader's acceleration plus the closing terms of
  its derivative, divided by its 1.75 s sensitivity).
- **Coasting (v3):** a gap correction that IDM asks at most `gentle_brake_m_s2` for, with no other term
  below `-coast_m_s2`, coasts at `-coast_m_s2` instead of braking.
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

DRIVER = 'rule:expert-v3'
# v2 (Phase 2 run 2, kept byte for byte in traffic/v15_rule_expert_v2.py) -> v3, from run 2's gate audit (plan
# v15, "Phase 2 result"): (1) during a lane change v2 took the target lane's leader from the side of the car's
# current lane, which after the lane switch mid-maneuver is the lane beyond the target: row 4 braked for 0.8 s
# for a car alongside in that lane (bumper gap 0.0 m), and row 14 dropped the slow car ahead in its ending source
# lane, which then merged into the same lane (the environment's bound clamped at -4.5 m/s^2). v3 takes the
# leaders of the maneuver's source and target lanes by lane identity, never the lane beyond. (2) Small gap
# corrections behind a leader coast instead of braking: row 43's cruise car braked at 0.2-0.3 m/s^2 to open a
# 1.8 s gap while the gate's target allowed 1.6 m/s more, which the gate counts as avoidable braking.
# v1 (Phase 2 run 1) -> v2, from run 1's failures (plan v15, "Phase 2"): the environment's following bound
# clamped cars closing on slower leaders, our ramp cars entered junctions too close ahead of or behind
# mainline cars (every cut-in and the collision), and lane changes accepted by the mask left too little
# following headroom. v2 adds the following-bound anticipation, projected leaders through junctions, gap
# selection at yield entries judged at the stop line, with the dilemma rule, and a new-leader test for lane
# changes, and tightens MOBIL's safe braking from 2.5 to 1.5 m/s^2.
SETTINGS = dict(
    # Level 1: speed and following
    free_gain_per_s=2.0, free_min_m_s2=-3.0, free_max_m_s2=1.8,
    idm_max_m_s2=1.8, idm_comfort_m_s2=2.0, idm_time_gap_s=1.8, idm_min_gap_m=4.0, idm_range_m=200.,
    free_road_m_s=26.8224, speed_margin_m_s=0.3,
    ceiling_input=43, ceiling_scale_m_s2=4.5, ceiling_margin_m_s2=0.05, rate_m_s2=0.3,
    # the environment's following bound (environments/highway_stage1_env.acceleration_constraints): keep the
    # speed this far under vL + 0.5 (gap - 2.5 - 1.5 v), closing the shortfall at this rate
    bound_margin_m_s=1.0, bound_gain_per_s=1.0, bound_floor_m_s2=-4.0,
    # v3: a gap correction that IDM asks at most gentle_brake_m_s2 for, with every other term at or above
    # -coast_m_s2, coasts at -coast_m_s2. The gate counts a request below -0.2 m/s^2 for 3 decisions as avoidable
    # braking when its target is at least 0.5 m/s above the speed (traffic/highway_flow_rl_v03_rewards.BrakingDiagnostic)
    gentle_brake_m_s2=0.6, coast_m_s2=0.19,
    # projected leaders and rears at a shared lane ahead (junctions); moving vehicles only
    projection_range_m=150., projection_radius_m=250., projection_moving_m_s=3.,
    # gap selection at a yield entry's stop line: the first gap in the approaching stream whose rear car
    # would brake at most entry_safe_m_s2 (SUMO's IDM; the merge gate allows neighbours down to -3 m/s^2)
    # when we cross the line, arriving at least yield_headway_s after the car ahead of that gap passes. While
    # waiting the car either holds the steady speed that arrives on time (a short wait) or crawls at crawl_m_s,
    # keeping room for a run-up at run_up_m_s2 that crosses the line as the gap arrives, whichever enters
    # faster; never across the line before then; commit if stopping would need more than dilemma_m_s2
    # a commit already started holds while that follower would brake at most entry_abort_m_s2 (the gate's limit)
    entry_safe_m_s2=2.0, entry_abort_m_s2=3.0, yield_headway_s=1.0, crawl_m_s=0.5, crawl_brake_m_s2=2.0, run_up_m_s2=1.5,
    stop_line_margin_m=1.0,
    # a lane change also needs following headroom to its new leader
    new_front_margin_m_s=1.0, new_front_idm_m_s2=-1.5,
    # dilemma zone at an ending lane
    lane_end_start_m_s2=2.0, lane_end_margin_m=6.0, dilemma_m_s2=3.5,
    # MOBIL's safety test and incentive; the background cars' IDM (traffic/highway_v05_demand PROFILES,
    # 'native_balanced', with SUMO's default 2.5 m minimum gap)
    mobil_safe_m_s2=1.5, mobil_politeness=0.3, mobil_threshold_m_s2=0.2,
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


def background_response(speed, leader_speed, gap, free_speed, params=None, s=SETTINGS):
    """A background car's IDM acceleration (SUMO's IDM, delta 4) behind a leader at `gap`; `params`
    (accel, decel, tau, min_gap) are that car's own, else the balanced profile's."""
    a, b, tau, min_gap = params or (s['background_accel_m_s2'], s['background_decel_m_s2'],
                                    s['background_tau_s'], s['background_min_gap_m'])
    desired = min_gap + max(0., speed * tau + speed * (speed - leader_speed) / (2. * math.sqrt(a * b)))
    free = 1. - (speed / max(free_speed, .1)) ** 4
    return a * (free - (desired / max(gap, .1)) ** 2)


def bound_speed(speed, leader_speed, gap):
    """The environment's following bound as a speed: vL + 0.5 (gap - 2.5 - 1.5 v)."""
    return max(0., leader_speed + .5 * (gap - 2.5 - 1.5 * speed))


def bound_term(speed, leader_speed, leader_acceleration, gap, s=SETTINGS):
    """The acceleration that keeps the headroom h = bound speed - speed at `bound_margin_m_s`.

    dh/dt = aL + 0.5 (vL - v) - 1.75 a; asking dh/dt = -k (h - margin) gives
    a = (aL + 0.5 (vL - v) + k (h - margin)) / 1.75: far behind (h large) it does not bind."""
    headroom = bound_speed(speed, leader_speed, gap) - speed
    wanted = (leader_acceleration + .5 * (leader_speed - speed)
              + s['bound_gain_per_s'] * (headroom - s['bound_margin_m_s'])) / 1.75
    return max(s['bound_floor_m_s2'], wanted), headroom


def run_up(speed, distance, cap, s=SETTINGS):
    """(time, speed) to cover `distance` accelerating at run_up_m_s2 from `speed` up to `cap`."""
    a, speed = s['run_up_m_s2'], min(speed, cap)
    reach = max(0., (cap * cap - speed * speed) / (2. * a))
    if distance <= reach:
        final = math.sqrt(speed * speed + 2. * a * max(0., distance))
        return (final - speed) / a, final
    return (cap - speed) / a + (distance - reach) / max(cap, .1), cap


def entry_options(stop, speed, arrival, cap, s=SETTINGS):
    """The ways to wait and arrive at the stop line at `arrival`, gentlest first, as (speed at the line,
    hold speed): hold the steady speed stop / arrival, or brake to the crawl, crawl, then run up over the
    room that is left (conservatively, crawling for the whole wait)."""
    steady = min(speed, stop / max(arrival, .1))
    crawl = min(speed, s['crawl_m_s'])
    room = stop - max(0., speed * speed - crawl * crawl) / (2. * s['crawl_brake_m_s2']) - crawl * arrival
    options = [(steady, max(steady, crawl))]
    run = run_up(crawl, max(0., room), cap)[1]
    if run > steady:
        options.append((run, crawl))
    return options


def select_gap(stop, speed, stream, length=5., cap=20., safe=None, s=SETTINGS):
    """Gap selection at a stop line `stop` ahead: the first gap in the approaching `stream`
    [(distance to our stop-line point, speed, id, IDM params or None)] we can enter.

    Going now, the run-up reaches the line at T_go. Arriving at time T with speed u, the stream car next
    behind (the first that has not passed our point by T) is V (P - T) - length behind us, P its passing
    time. The gap is accepted when that car's IDM response (its own SUMO parameters) is at least
    -entry_safe_m_s2 and every car ahead of it passed at least yield_headway_s before T. Waiting cars
    (`waited_for`) push T later; u is then the run-up speed after crawling. Returns dict(T, u, go_s,
    rear, gap_at_entry, response, waited_for)."""
    safe = s['entry_safe_m_s2'] if safe is None else safe
    go_time, go_speed = run_up(speed, stop, max(cap, .1))
    passing = sorted((max(0., d) / max(v, 1.), v, vid, params) for d, v, vid, params in stream)
    arrival, waited = go_time, []
    for when, rear_speed, vid, params in passing:
        if when < arrival:
            if when > arrival - s['yield_headway_s']:
                arrival, waited = when + s['yield_headway_s'], waited + [vid]
            continue
        behind = rear_speed * (when - arrival) - length
        options = [(go_speed, None)] if not waited else entry_options(stop, speed, arrival, cap)
        for u, hold in options:                 # the gentlest way that makes this gap acceptable
            response = background_response(rear_speed, u, max(behind, .1), max(rear_speed, 1.), params)
            if behind > 0. and response >= -safe:
                return dict(T=arrival, u=u, hold=hold, go_s=go_time, rear=vid, gap_at_entry=behind,
                            response=response, waited_for=waited)
        arrival, waited = when + s['yield_headway_s'], waited + [vid]
    u, hold = (go_speed, None) if not waited else entry_options(stop, speed, arrival, cap)[0]
    return dict(T=arrival, u=u, hold=hold, go_s=go_time, rear=None, gap_at_entry=None, response=None,
                waited_for=waited)


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


def compose(speed, measured, target, ceiling, *, leaders=(), extra=None, pull_away=False, s=SETTINGS):
    """Level 1's request (m/s^2) and its terms: min(free, IDM and the following-bound term behind each
    leader (speed, gap, acceleration), any extra terms), a pull-away floor, then the rate band around the
    measured acceleration, then the ceiling less a margin."""
    terms = dict(free=free_term(speed, target, s))
    if leaders:
        terms['idm'] = min(idm_interaction(speed, vl, gap) for vl, gap, _ in leaders)
        terms['bound'] = min(bound_term(speed, vl, al, gap, s)[0] for vl, gap, al in leaders)
    for name, value in (extra or {}).items():
        if value is not None:
            terms[name] = value
    wanted = min(terms.values())
    others = [v for k, v in terms.items() if k != 'idm']
    if ('idm' in terms and -s['gentle_brake_m_s2'] <= terms['idm'] < -s['coast_m_s2']
            and all(v >= -s['coast_m_s2'] for v in others)):
        wanted = -s['coast_m_s2']                  # v3: a small gap correction coasts
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


def lane_change_ok(speed, front, rear, s=SETTINGS):
    """MOBIL's safety test on the new follower, and following headroom to the new leader.
    Returns (ok, record)."""
    safe, response = mobil_safe(speed, rear, s)
    headroom = None
    if front:
        leader = _speed(dict(speed=speed), front)
        headroom = bound_speed(speed, leader, front['bumper_gap_m']) - speed
        safe = safe and headroom >= s['new_front_margin_m_s'] \
            and idm_interaction(speed, leader, front['bumper_gap_m']) >= s['new_front_idm_m_s2']
    return safe, dict(follower_response=response, front_headroom=headroom)


# --------------------------------------------------------------------------- the driver

class RuleExpert:
    """Drives every controlled car of the live environment that `env_source()` returns.

    Same interface as the policies' actors in the episode loop (act, values)."""

    def __init__(self, env_source, *, record=True):
        self.env_source = env_source
        self.record = record
        self._episode = None
        self._stopped, self._commit, self._dist, self._dist_time = {}, {}, {}, None
        self._curves, self._params = {}, {}

    def _new_episode(self, env):
        key = str(getattr(env, 'directory', id(env)))
        if key != self._episode:
            self._episode, self._stopped, self._commit, self._dist, self._dist_time = key, {}, {}, {}, None
            self._params = {}

    def _idm(self, env, vehicle):
        """A car's own SUMO car-following parameters (accel, decel, tau, min gap), cached per episode."""
        if vehicle not in self._params:
            v = env.conn.vehicle
            self._params[vehicle] = (v.getAccel(vehicle), v.getDecel(vehicle), v.getTau(vehicle), v.getMinGap(vehicle))
        return self._params[vehicle]

    def _lane_curves(self, env, lane_id):
        if lane_id not in self._curves:
            self._curves[lane_id] = lane_curvatures(env.conn.lane.getShape(lane_id))
        return self._curves[lane_id]

    def _distances(self, env, vehicle):
        """The lanes ahead on a vehicle's route and the distance to each lane's start (cached per step)."""
        if self._dist_time is None or abs(self._dist_time - env.time) > 1e-9:
            self._dist, self._dist_time = {}, env.time
        if vehicle not in self._dist:
            s = env.states[vehicle]
            self._dist[vehicle] = env.graph.distances(s['lane_id'], s['lane_position'], s['route'])
        return self._dist[vehicle]

    def _curve_points(self, env, agent, state):
        points = []
        for lane, start in self._distances(env, agent).items():
            if start > SETTINGS['curve_lookahead_m']:
                continue
            for centre, curvature in self._lane_curves(env, lane):
                ahead = start + centre
                if -SETTINGS['curve_window_m'] / 2. <= ahead <= SETTINGS['curve_lookahead_m']:
                    points.append((max(0., ahead), curvature))
        return points

    def projections(self, env, agent):
        """Moving vehicles from other approaches that reach a lane on our route ahead (a junction).

        Returns (leaders, rears): leaders [(gap, speed, acceleration, id, lane, their distance to it)]
        reach the shared lane before us; rears [(gap, speed, their distance to it, id, lane)] after us.
        Vehicles in our own lane, or behind us on our own path, are left out (ordinary following). Our
        own merging cars that are yielding are not leaders: they do not cross their stop line until
        their nearest rear has passed."""
        s = SETTINGS
        me = env.states[agent]
        mine = self._distances(env, agent)
        ahead = sorted((d, lane) for lane, d in mine.items() if d <= s['projection_range_m'])
        leaders, rears = [], []
        if not ahead:
            return leaders, rears
        for vehicle, other in env.states.items():
            if vehicle == agent or other['lane_id'] == me['lane_id'] or other['speed'] < s['projection_moving_m_s']:
                continue
            if math.hypot(other['x'] - me['x'], other['y'] - me['y']) > s['projection_radius_m']:
                continue
            theirs = self._distances(env, vehicle)
            if me['lane_id'] in theirs:
                continue
            shared = next((lane for _, lane in ahead if lane in theirs), None)
            if shared is None:
                continue
            projected = mine[shared] - theirs[shared]
            if projected > 0:
                decision = self._commit.get(vehicle)
                if decision is not None and not decision['committed']:
                    continue
                leaders.append((projected - other.get('length', 5.), other['speed'], other.get('acceleration', 0.),
                                vehicle, shared, theirs[shared]))
            else:
                rears.append((-projected - me.get('length', 5.), other['speed'], theirs[shared], vehicle, shared))
        return leaders, rears

    def _target(self, env, agent, state):
        cached = (env._v14_cache.get(agent) or {}).get('target')
        if cached and cached.get('eligible') and cached.get('target_m_s') is not None \
                and math.isfinite(cached['target_m_s']):
            return float(cached['target_m_s']), 'v14'
        return self._free_road(state), 'free_road'

    @staticmethod
    def _free_road(state):
        free = min(SETTINGS['free_road_m_s'], state.get('allowed_speed', SETTINGS['free_road_m_s']))
        return max(0., free - SETTINGS['speed_margin_m_s'])

    def _junction(self, env, agent, state, entry, rears, in_maneuver):
        """Gap selection at an active yield entry's stop line, with the dilemma rule.

        SUMO's mainline cars ignore a car on the minor approach until it crosses the stop line onto the
        junction, and then follow it as a leader by distance to the shared lane (the 2-row code check:
        a mainline car braked at 9 m/s^2 the step our ramp car entered the junction lane at 4.4 m/s). So
        the gap is judged at the moment we cross the line, at the speed we would then have, and the car
        waits by crawling back from the line, then runs up to cross it as the gap arrives (select_gap):
        it commits once the run-up from where it is would not reach the line before the planned time.
        A started commit holds while that follower would brake at most entry_abort_m_s2. Returns None away
        from an entry, else dict(committed, dilemma, arrive_s, go_s, entry_m_s, stop_need, rear,
        gap_at_entry, response, waited_for, stop_line)."""
        s = SETTINGS
        lane = entry.get('target_lane')
        if not (entry.get('known') and entry.get('entry_active') and lane) or in_maneuver:
            self._commit.pop(agent, None)
            return None
        shared, stop = self._distances(env, agent).get(lane), entry.get('distance_to_start_m')
        if shared is None or stop is None:
            self._commit.pop(agent, None)
            return None
        junction_lane = max(0., shared - stop)        # the junction's internal lane, stop line to shared lane
        speed = state['speed']
        previous = self._commit.get(agent) or {}
        stream = [(rear_distance - junction_lane, rear_speed, rear_id, self._idm(env, rear_id))
                  for _, rear_speed, rear_distance, rear_id, shared_lane in rears if shared_lane == lane]
        plan = select_gap(stop, speed, stream, state.get('length', 5.), self._free_road(state))
        kept = False
        if plan['waited_for'] and previous.get('committed') and not previous.get('dilemma'):
            keep = select_gap(stop, speed, stream, state.get('length', 5.), self._free_road(state),
                              safe=s['entry_abort_m_s2'])
            if not keep['waited_for']:                     # the run-up has started: it holds
                plan, kept = keep, True
        decision = dict(committed=not plan['waited_for'], dilemma=False, arrive_s=round(plan['T'], 2),
                        go_s=round(plan['go_s'], 2), entry_m_s=round(plan['u'], 2),
                        hold_m_s=None if plan['hold'] is None else round(plan['hold'], 3), stop_need=None, rear=plan['rear'],
                        gap_at_entry=None if plan['gap_at_entry'] is None else round(plan['gap_at_entry'], 2),
                        response=None if plan['response'] is None else round(max(plan['response'], -99.), 3),
                        waited_for=plan['waited_for'], kept=kept, stop_line=round(stop, 2))
        if plan['waited_for']:
            stop_need = speed * speed / (2. * max(stop - s['stop_line_margin_m'], .1))
            decision.update(stop_need=round(stop_need, 3))
            if previous.get('dilemma'):
                decision.update(committed=True, dilemma=True)      # a commit under the dilemma rule holds
            elif stop_need > s['dilemma_m_s2']:
                decision.update(committed=True, dilemma=True)
        self._commit[agent] = decision
        return decision

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
        cap = curve_cap(self._curve_points(env, agent, state))
        in_maneuver = agent in env.maneuvers
        mask = env._mask(agent, context)
        flow = flow_context(env.graph, state, env.states)
        leaders_ahead, rears = self.projections(env, agent)

        # Level 2: commit or yield at a yield entry (gap acceptance, dilemma rule)
        junction = self._junction(env, agent, state, flow['entry'], rears, in_maneuver)
        extra = {}
        if junction is not None:
            if junction['committed'] and junction['rear'] is not None:
                target = max(target, min(self._free_road(state), speed))      # committed: no hesitation
            elif not junction['committed']:
                target = min(target, junction['hold_m_s'])
                # never across the stop line while yielding: IDM behind a stopped virtual car at the line
                extra['stop_line'] = idm_interaction(
                    speed, 0., junction['stop_line'] + s['idm_min_gap_m'] - s['stop_line_margin_m'])
        target = min(target, cap)

        # Level 2: which lane
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
                ok, detail = lane_change_ok(speed, neighbors[side]['front'], neighbors[side]['rear'])
                safety = dict(delta=delta, safe=ok, mask=bool(mask[delta + 1]), **detail)
                if ok and mask[delta + 1]:
                    lane, reason = delta, why
                    break

        # Level 1: leaders (own lane, the target lane during a change, projected through junctions)
        def leader(neighbor):
            other = env.states.get(neighbor.get('id')) if neighbor.get('id') else None
            return (_speed(state, neighbor), neighbor['bumper_gap_m'], (other or {}).get('acceleration', 0.))
        leaders, named = [], []
        if own_front and own_front['bumper_gap_m'] <= s['idm_range_m']:
            leaders.append(leader(own_front))
            named.append(('own', own_front.get('id'), round(own_front['bumper_gap_m'], 2)))
        maneuver = env.maneuvers.get(agent) if in_maneuver else None
        if maneuver is not None:
            # v3: the fronts of the maneuver's source and target lanes, by lane identity (the car's own lane
            # switches mid-maneuver, so a side relative to it can be the lane beyond the target)
            lanes = {maneuver['source_identity']: 'source', maneuver['target_identity']: 'target'}
            seen = {own_front.get('id')} if own_front else set()
            for side in ('own', 'left', 'right'):
                corridor = flow['corridors'].get(side) or {}
                role = lanes.get(env._lane_identity_key(corridor['lane_id'])) if corridor.get('lane_id') else None
                front = corridor.get('front')
                if role and front and front.get('id') not in seen and front['bumper_gap_m'] <= s['idm_range_m']:
                    seen.add(front.get('id'))
                    leaders.append(leader(front))
                    named.append((role, front.get('id'), round(front['bumper_gap_m'], 2)))
        # a projected leader binds once it is on the shared lane (it is then the environment's own-lane
        # front): its gap at that moment, at constant speeds; one we would then be ahead of is not a leader
        projected = None
        for gap, vl, al, vid, _, their_distance in leaders_ahead:
            at_shared = gap + (vl - speed) * max(0., their_distance) / max(vl, 1.)
            if 0. < at_shared <= s['idm_range_m'] and (projected is None or at_shared < projected[0]):
                projected = (at_shared, vl, al, vid, gap)
        if projected is not None:
            leaders.append((projected[1], projected[0], projected[2]))
            named.append(('projected', projected[3], round(projected[0], 2)))
        stopping = None
        distance = env.merge_distance(state)
        if mandatory is not None and lane == 0 and not in_maneuver and distance is not None:
            extra['lane_end'], stopping = lane_end_term(speed, distance)
        free_road = not own_front or own_front['bumper_gap_m'] > s['stuck_free_m']
        yielding = junction is not None and not junction['committed']
        stopped = speed < s['stuck_speed_m_s'] and free_road and plan_target >= s['stuck_target_m_s'] and not yielding
        since = self._stopped.get(agent)
        if stopped:
            since = self._stopped.setdefault(agent, env.time)
        elif speed >= s['moving_m_s'] or not free_road or yielding:
            self._stopped.pop(agent, None)
            since = None
        pull_away = since is not None and env.time - since >= s['stuck_after_s'] - 1e-9
        request, terms = compose(speed, measured, target, ceiling, leaders=leaders, extra=extra, pull_away=pull_away)
        record = dict(t=round(env.time, 3), agent=agent, v=round(speed, 4), a=round(measured, 4),
                      target=round(target, 4), source=source, plan=round(plan_target, 4),
                      curve=None if math.isinf(cap) else round(cap, 3), ceiling=round(ceiling, 4),
                      terms={k: round(v, 4) for k, v in terms.items()}, request=round(request, 4),
                      lane=lane, reason=reason, mandatory=mandatory, maneuver=in_maneuver, leaders=named,
                      mask=[int(m) for m in mask], safety=safety,
                      projected=None if projected is None else dict(id=projected[3], gap=round(projected[4], 2),
                                                                     at_shared=round(projected[0], 2),
                                                                     speed=round(projected[1], 3)),
                      junction=junction,
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
