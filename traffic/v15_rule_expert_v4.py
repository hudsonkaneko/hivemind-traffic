"""The v15 rule expert v4: expert v3 with four changes to how a ramp car waits for and takes a gap (the
write-up's three, and a fourth the offline replay showed was needed).

Plan v15, "Ramp driving", option A; user decision of November 27, 2026, relayed by the main Claude session: "Yes,
as recommended" (decision 14, step 1: the code, its tests and the offline replay; nothing run in SUMO). Expert v3
(traffic/v15_rule_expert.py, 'rule:expert-v3') and the policy's Level 2 module are unchanged: v4 is a new driver id,
built on v3's code, which it imports and does not modify. Everything v4 doesn't name below is v3's.

Why (plan v15, "2. Why expert v3's Level 1 does it"): with no acceptable gap v3 slows at once to the steady speed that
arrives 1 s after the last visible car (row 11: 5.2 m/s within 3.5 s, at the -3.0 m/s^2 floor); its only other way to
wait is a 0.5 m/s crawl; and a committed car's target is max(the gate's target, its speed), which doesn't make it run up
at the 1.5 m/s^2 its gap test assumed, so on row 11 the commit was undone at the 3.0 m/s^2 abort.

The changes (the write-up's three, with the choices made to write them down marked, then the fourth):
1. **No early slowing.** While it waits, the car keeps its current speed as long as it could still keep the wait plan
   after `wait_lead_s` more at that speed; only then does it slow, at `wait_brake_m_s2`. The leader, curve and
   stop-line limits still apply (v3's terms, unchanged).
   - *Choice:* the write-up's trigger was "until holding would need 2.0 m/s^2 of braking". Braking at 2.0 m/s^2 or
     more for 3 decisions is exactly what movement check M1 flags, so v4 plans its slowing at 1.5 m/s^2, the value
     of v3's own run-up (`run_up_m_s2`), and starts it once a plan at that rate would no longer be possible after
     the lead. The 0.5 s lead covers the jerk band: v3's request moves at most 0.3 m/s^2 per 0.1 s decision.
2. **A run-up option.** A waiting car may brake to a moderate hold speed, hold it, then run up at `run_up_m_s2` to
   cross the stop line exactly when the gap arrives. The hold speed is the highest, in steps of `hold_step_m_s`, from
   which that crossing passes v3's gap test (the rear car brakes at most `entry_safe_m_s2`): the gentlest acceptable
   plan. v3's own two options (the steady speed, and the crawl then run-up) stay as fallbacks after these, so v4
   accepts every gap v3 accepts.
   - *Choice:* a plan counts only if, where its run-up starts, the car could still stop at the line braking at most
     `entry_abort_m_s2` (3.0 m/s^2, the gate's limit). Without it a fast hold reaches v3's dilemma zone (stopping
     would need 3.5 m/s^2) before the gap opens, and a dilemma commit holds whatever the stream then does.
3. **A committed car runs up as planned.** When the committed gap's test depends on the run-up (it would fail if the
   car crossed at its current speed), the car's target is the run-up's: speed + run_up_m_s2 / free_gain_per_s, so the
   free-road term asks for run_up_m_s2. It is never below v3's committed target. The 3.0 m/s^2 abort is kept.
   - *Choice:* where the gap passes at the car's current speed, the commit is v3's. It still acts on a clear ramp
     wherever v3's go-now gap counts on the run-up: in the offline replay, on 5 of the 7 clear cars.

4. **The run-up it can make** (a fourth change, not in the write-up: added after the offline replay; for the user's
   decision). v3's gap test assumes a 1.5 m/s^2 run-up up to the free-road speed. The offline replay
   (experiments/v15_expert_v4_replay.py) found that run-up out of reach where the environment's drive bound caps the
   car at about 1.3 m/s^2 (the Adams ramp) or the curve cap stops it short (row 24). With change 3 a car committed to
   such a gap then fell behind its plan, had the commit undone already inside v3's dilemma zone, and crossed ahead
   of a mainline car that would need up to 8 m/s^2 of braking (open loop; rows 24 and 37). So v4's gap test, its
   wait plans and its committed run-up use the run-up the car can make now: `run_up_m_s2`, at most the
   environment's ceiling at this decision less v3's `ceiling_margin_m_s2`, at least `run_up_floor_m_s2`, and
   up to the curve cap now and at the stop line (v3's `curve_cap` seen from the line) rather than the free-road
   speed.
   - *Choice:* the 0.3 m/s^2 floor keeps the run-up defined when a leader holds the ceiling near zero; there the
     test then rejects most gaps, so the car waits.
   - Where the ceiling is at least 1.55 m/s^2 and no curve cap binds, the test is v3's.

Where none of these acts (no active yield entry, a ceiling of at least 1.55 m/s^2 and no binding curve cap, and a
go-now commit whose gap passes at the current speed) v4's requests are v3's.
"""
from __future__ import annotations

import math

from traffic import v15_rule_expert as v3
from traffic.v15_rule_expert import (_speed, background_response, compose, curve_cap, idm_interaction,
                                     lane_change_ok, lane_end_term, mobil_incentive, run_up)

DRIVER = 'rule:expert-v4'
SETTINGS = dict(v3.SETTINGS,
                # v4: plans slow at this rate (v3's run-up rate; below movement check M1's 2.0 m/s^2)
                wait_brake_m_s2=1.5,
                # v4: keep the current speed while the wait plan would still be possible after this long at it
                wait_lead_s=0.5,
                # v4: hold speeds tried, highest first, in these steps, down to v3's crawl
                hold_step_m_s=0.25,
                # v4 (change 4): the lowest run-up rate the gap test assumes, where the ceiling is lower still
                run_up_floor_m_s2=0.3)
NEW_SETTINGS = ('wait_brake_m_s2', 'wait_lead_s', 'hold_step_m_s', 'run_up_floor_m_s2')


def achievable(ceiling=None, curve=None, cap=v3.SETTINGS['free_road_m_s'], s=SETTINGS):
    """(settings, cap) for v4's gap test (change 4): the run-up the car can make now. The rate is run_up_m_s2, at most
    the environment's ceiling less ceiling_margin_m_s2 and at least run_up_floor_m_s2; the top speed is `cap` (the
    free-road speed), at most the curve cap. With no ceiling and no curve cap given, v3's."""
    if ceiling is not None and math.isfinite(ceiling):
        rate = max(s['run_up_floor_m_s2'], min(s['run_up_m_s2'], ceiling - s['ceiling_margin_m_s2']))
        if rate != s['run_up_m_s2']:
            s = dict(s, run_up_m_s2=rate)
    if curve is not None and math.isfinite(curve):
        cap = min(cap, curve)
    return s, cap


def wait_plans(stop, speed, arrival, cap, s=SETTINGS):
    """v4's run-up options for a wait (change 2): brake now at wait_brake_m_s2 to a hold speed w, hold it, then run up at
    run_up_m_s2 to cross the stop line `stop` ahead exactly at `arrival` s from now, at speed u <= cap.

    Hold speeds go from the current speed down to v3's crawl in hold_step_m_s steps, highest (gentlest) first. Returns
    [(u, w, run-up seconds)] for those that work: the hold neither crosses early nor leaves too much to run up, and
    where the run-up starts the car could still stop at the line braking at most entry_abort_m_s2, so it never reaches
    v3's dilemma zone (3.5 m/s^2) while it still waits."""
    a, b = s['run_up_m_s2'], s['wait_brake_m_s2']
    speed = max(0., float(speed))
    floor, step = min(speed, s['crawl_m_s']), s['hold_step_m_s']
    # the current speed, then the multiples of the step below it (a fixed grid, so plans compare across decisions),
    # then the crawl
    holds = [speed] + [k * step for k in range(int(math.floor((speed - 1e-9) / step)), 0, -1) if k * step > floor + 1e-9]
    if floor < speed - 1e-9:
        holds.append(floor)
    plans = []
    for w in holds:
        braking = (speed - w) / b
        room = stop - (speed * speed - w * w) / (2. * b)
        left = arrival - braking
        if left < 0. or room < 0.:
            continue
        spare = room - w * left                  # the distance the run-up must add to the hold
        if spare < -1e-9:
            continue                             # holding w would cross the line early
        run = math.sqrt(2. * max(0., spare) / a)
        if run > left + 1e-9:
            continue                             # too slow: the run-up can't cover the rest by `arrival`
        u = w + a * run
        reach = w * run + .5 * a * run * run     # the run-up's distance: it starts this far from the line
        if w * w / (2. * max(reach - s['stop_line_margin_m'], .1)) > s['entry_abort_m_s2'] + 1e-9:
            continue                             # it couldn't still stop at the line when its run-up starts
        if u <= cap + 1e-9:
            plans.append((u, w, run))
    return plans


def v3_options(stop, speed, arrival, cap, s=SETTINGS):
    """v3's own options (traffic/v15_rule_expert.entry_options, copied) with the run-up at the rate in `s` (change 4;
    v3's function runs its crawl run-up at its own module's rate). At v3's settings they are v3's."""
    steady = min(speed, stop / max(arrival, .1))
    crawl = min(speed, s['crawl_m_s'])
    room = stop - max(0., speed * speed - crawl * crawl) / (2. * s['crawl_brake_m_s2']) - crawl * arrival
    found = [(steady, max(steady, crawl))]
    run = run_up(crawl, max(0., room), cap, s)[1]
    if run > steady:
        found.append((run, crawl))
    return found


def line_cap(points, stop, s=SETTINGS):
    """The curve cap at the stop line (change 4): v3's curve_cap of the car's curve points (distance ahead, curvature)
    seen from the line `stop` ahead, over the points from half a curve window before the line on. Points further
    back are passed before the run-up's end; the curve cap now covers them."""
    return curve_cap([(ahead - stop, k) for ahead, k in points if ahead >= stop - s['curve_window_m'] / 2.], s)


def run_up_limits(driver, env, agent, state, stop):
    """(ceiling, curve) for junction_plan (change 4): the environment's ceiling the driver's decide stored for this
    decision, and the run-up's top speed from the curves, the curve cap now and at most the cap at the line."""
    ceiling, now = driver._limits.get(agent, (None, None))
    if now is None:
        return ceiling, None
    return ceiling, min(now, line_cap(driver._curve_points(env, agent, state), stop))


def options(stop, speed, arrival, cap, s=SETTINGS):
    """The ways to wait and cross at `arrival`, as (speed at the line, hold speed, kind): v4's run-up plans, highest
    hold first, then v3's own options (the steady speed, the crawl then run-up) as fallbacks."""
    found = [(u, w, 'run-up') for u, w, _ in wait_plans(stop, speed, arrival, cap, s)]
    return found + [(u, hold, 'v3') for u, hold in v3_options(stop, speed, arrival, cap, s)]


def select_gap(stop, speed, stream, length=5., cap=20., safe=None, skip=0, s=SETTINGS):
    """v3's gap selection (traffic/v15_rule_expert.select_gap, and with `skip` traffic/v15_level2.select_gap_from) with
    v4's ways to wait. With skip, a gap is accepted only once at least `skip` stream cars have been waited for.

    Returns v3's dict(T, u, hold, go_s, rear, gap_at_entry, response, waited_for) plus kind ('go', 'run-up', 'v3' or
    'none': no visible gap passed, arriving 1 s after the last car) and needs_run_up (the accepted gap would fail its
    test if the car crossed the line at its current speed)."""
    safe = s['entry_safe_m_s2'] if safe is None else safe
    go_time, go_speed = run_up(speed, stop, max(cap, .1), s)
    passing = sorted((max(0., d) / max(v, 1.), v, vid, params) for d, v, vid, params in stream)
    arrival, waited = go_time, []
    for when, rear_speed, vid, params in passing:
        if when < arrival:
            if when > arrival - s['yield_headway_s']:
                arrival, waited = when + s['yield_headway_s'], waited + [vid]
            continue
        if len(waited) >= skip:
            behind = rear_speed * (when - arrival) - length
            tries = [(go_speed, None, 'go')] if not waited else options(stop, speed, arrival, cap, s)
            for u, hold, kind in tries:      # the gentlest way that makes this gap acceptable
                response = background_response(rear_speed, u, max(behind, .1), max(rear_speed, 1.), params, s)
                if behind > 0. and response >= -safe:
                    steady_arrival = stop / max(speed, .1)
                    steady_behind = rear_speed * (when - steady_arrival) - length
                    steady = background_response(rear_speed, speed, max(steady_behind, .1), max(rear_speed, 1.), params, s)
                    return dict(T=arrival, u=u, hold=hold, go_s=go_time, rear=vid, gap_at_entry=behind,
                                response=response, waited_for=waited, kind=kind,
                                needs_run_up=not (steady_behind > 0. and steady >= -safe))
        arrival, waited = when + s['yield_headway_s'], waited + [vid]
    if not waited:
        return dict(T=arrival, u=go_speed, hold=None, go_s=go_time, rear=None, gap_at_entry=None, response=None,
                    waited_for=waited, kind='go', needs_run_up=False)
    u, hold, kind = options(stop, speed, arrival, cap, s)[0]
    return dict(T=arrival, u=u, hold=hold, go_s=go_time, rear=None, gap_at_entry=None, response=None,
                waited_for=waited, kind='none' if kind == 'run-up' else 'v3', needs_run_up=False)


def advanced(stop, stream, speed, seconds):
    """The stop line and stream after `seconds` more at constant speeds (everyone keeps their speed)."""
    return stop - speed * seconds, [(d - v * seconds, v, vid, params) for d, v, vid, params in stream]


def junction_plan(stop, speed, stream, length, cap, previous, skip=0, s=SETTINGS, ceiling=None, curve=None):
    """v4's decision at an active yield entry (v3's _junction with v4's gap selection and changes 1, 3 and 4).

    previous: the car's decision at the last decision ({} if none); ceiling and curve: the environment's ceiling
    and the curve cap at this decision (change 4). Returns v3's decision dict plus kind, keep_speed (change 1: the car
    keeps its speed this decision), hold_w (the plan's hold speed), needs_run_up and run_up_m_s2 (the rate assumed)."""
    s, cap = achievable(ceiling, curve, cap, s)
    plan = select_gap(stop, speed, stream, length, cap, skip=skip, s=s)
    kept = False
    if plan['waited_for'] and previous.get('committed') and not previous.get('dilemma'):
        keep = select_gap(stop, speed, stream, length, cap, safe=s['entry_abort_m_s2'], skip=0, s=s)
        if not keep['waited_for']:                     # the run-up has started: it holds (v3)
            plan, kept = keep, True
    hold = plan['hold']
    keep_speed = False
    if plan['waited_for'] and plan['kind'] in ('run-up', 'none'):
        # change 1: keep the current speed while a plan for the same gap would still work after the lead
        if plan['hold'] >= speed - 1e-9:
            keep_speed = True                          # the plan needs no slowing: hold, then run up on the commit
        else:
            ahead_stop, ahead_stream = advanced(stop, stream, speed, s['wait_lead_s'])
            later = select_gap(ahead_stop, speed, ahead_stream, length, cap, skip=skip, s=s) if ahead_stop > 0. else None
            # the same gap with the same hold speed: waiting longer must not force a lower hold
            keep_speed = bool(later and later['kind'] == plan['kind'] and later['rear'] == plan['rear']
                              and later['hold'] >= plan['hold'] - 1e-9)
        hold = speed if keep_speed else max(plan['hold'], speed - s['wait_brake_m_s2'] / s['free_gain_per_s'])
    decision = dict(committed=not plan['waited_for'], dilemma=False, arrive_s=round(plan['T'], 2),
                    go_s=round(plan['go_s'], 2), entry_m_s=round(plan['u'], 2),
                    hold_m_s=None if hold is None else round(hold, 3), stop_need=None, rear=plan['rear'],
                    gap_at_entry=None if plan['gap_at_entry'] is None else round(plan['gap_at_entry'], 2),
                    response=None if plan['response'] is None else round(max(plan['response'], -99.), 3),
                    waited_for=plan['waited_for'], kept=kept, stop_line=round(stop, 2),
                    kind=plan['kind'], keep_speed=keep_speed,
                    hold_w=None if plan['hold'] is None else round(plan['hold'], 3),
                    needs_run_up=bool(plan['needs_run_up']), run_up_m_s2=round(s['run_up_m_s2'], 4))
    if plan['waited_for']:
        stop_need = speed * speed / (2. * max(stop - s['stop_line_margin_m'], .1))
        decision.update(stop_need=round(stop_need, 3))
        if previous.get('dilemma'):
            decision.update(committed=True, dilemma=True)      # a commit under the dilemma rule holds (v3)
        elif stop_need > s['dilemma_m_s2']:
            decision.update(committed=True, dilemma=True)
    return decision


def committed_target(target, junction, speed, free_road, s=SETTINGS):
    """The target of a car committed at a yield entry: v3's max(target, min(free road, speed)), and with change 3,
    where the gap's test needs the run-up, at least speed + (the rate the test assumed) / free_gain_per_s."""
    target = max(target, min(free_road, speed))
    if junction.get('needs_run_up') and not junction.get('dilemma'):
        rate = junction.get('run_up_m_s2', s['run_up_m_s2'])
        target = max(target, min(free_road, speed + rate / s['free_gain_per_s']))
    return target


class RuleExpert(v3.RuleExpert):
    """Expert v4: v3's driver with v4's junction (changes 1, 2 and 4) and committed target (change 3)."""

    def __init__(self, env_source, *, record=True):
        super().__init__(env_source, record=record)
        self._limits = {}                      # agent: (ceiling, curve cap) at this decision (change 4)

    def _junction(self, env, agent, state, entry, rears, in_maneuver):
        lane = entry.get('target_lane')
        if not (entry.get('known') and entry.get('entry_active') and lane) or in_maneuver:
            self._commit.pop(agent, None)
            return None
        shared, stop = self._distances(env, agent).get(lane), entry.get('distance_to_start_m')
        if shared is None or stop is None:
            self._commit.pop(agent, None)
            return None
        junction_lane = max(0., shared - stop)
        stream = [(rear_distance - junction_lane, rear_speed, rear_id, self._idm(env, rear_id))
                  for _, rear_speed, rear_distance, rear_id, shared_lane in rears if shared_lane == lane]
        ceiling, curve = run_up_limits(self, env, agent, state, stop)
        decision = junction_plan(stop, state['speed'], stream, state.get('length', 5.), self._free_road(state),
                                 self._commit.get(agent) or {}, ceiling=ceiling, curve=curve)
        self._commit[agent] = decision
        return decision

    def decide(self, env, agent, ceiling):
        """One car's (request m/s^2, lane delta, record): v3's decide (traffic/v15_rule_expert.RuleExpert.decide,
        copied) with v4's limits for the junction and committed target. The two changed lines are marked v4."""
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
        self._limits[agent] = (ceiling, cap)                                                # v4: change 4
        in_maneuver = agent in env.maneuvers
        mask = env._mask(agent, context)
        flow = flow_context(env.graph, state, env.states)
        leaders_ahead, rears = self.projections(env, agent)

        junction = self._junction(env, agent, state, flow['entry'], rears, in_maneuver)
        extra = {}
        if junction is not None:
            if junction['committed'] and junction['rear'] is not None:
                target = committed_target(target, junction, speed, self._free_road(state))     # v4: change 3
            elif not junction['committed']:
                target = min(target, junction['hold_m_s'])
                extra['stop_line'] = idm_interaction(
                    speed, 0., junction['stop_line'] + s['idm_min_gap_m'] - s['stop_line_margin_m'])
        target = min(target, cap)

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

        def leader(neighbor):
            other = env.states.get(neighbor.get('id')) if neighbor.get('id') else None
            return (_speed(state, neighbor), neighbor['bumper_gap_m'], (other or {}).get('acceleration', 0.))
        leaders, named = [], []
        if own_front and own_front['bumper_gap_m'] <= s['idm_range_m']:
            leaders.append(leader(own_front))
            named.append(('own', own_front.get('id'), round(own_front['bumper_gap_m'], 2)))
        maneuver = env.maneuvers.get(agent) if in_maneuver else None
        if maneuver is not None:
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
