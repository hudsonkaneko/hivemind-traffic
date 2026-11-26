"""v15 Phase 3: Level 2 as an explicit choice, the rule Level 2 split out of expert v3, and the learned Level 2.

Built for Phase 3 (user approval, November 26, 2026, relayed by the main Claude session: "Recommended
size"). Plan v15, "The controller: two levels" and Stages 2-3. Every driver here is Level 1, the rule
executor of traffic/v15_rule_expert.py (v3), plus a Level 2 that makes three choices for each car at every
decision:
- **gap** (at an active yield entry): how many cars of the approaching stream to let pass before entering,
  0-3 (0 is "commit now, ahead of the nearest"; 1-3 is "behind the 1st, 2nd or 3rd"). Level 1 enters at the
  first gap at or after that one that passes its safety test (the following car brakes at most
  `entry_safe_m_s2`, SUMO's IDM with its own parameters), so a Level 2 choice can make a car wait longer,
  never cut in. A started commit holds while the abort test passes (the dilemma rule too), whatever Level 2
  chooses next: once committed, the car re-decides only when a safety check fails;
- **lane**: the lane-change intent, -1, 0 or +1. Level 1 requests it only inside the environment's lane mask
  and when MOBIL's safety test and the new-leader headroom pass, and never during a maneuver;
- **offset**: a target-speed offset in m/s, one of OFFSETS. A positive offset never raises the target above
  the larger of the plan's target and the free-road target.

The rule Level 2 (the expert's choices, the labels) is v3's own: the gap is the number of stream cars v3's
gap selection waits for (capped at 3, which gives the same entry), the lane is v3's first candidate that
passes the safety test and mask (else its first candidate), and the offset is 0. With those choices this
executor reproduces v3's requests and lane requests exactly ('l2:rule-v3' against 'rule:expert-v3';
tests/test_v15_level2.py and the Phase 3 parity checks).

The learned Level 2 ('l2:<hash>', a published Level2Net) reads only non-privileged inputs:
- the environment's 121-field actor observation (today's local fields, map fields, target inputs and the
  delivered V2V messages) without fields 1 (the car's own acceleration) and 5 (its own previous request),
  which plan v15 leaves out because they drive the feedback loop;
- the executor's state from the previous decision (committed at the yield entry, and under the dilemma
  rule), and two map facts Level 1 already uses: whether a yield entry is active and the mandatory
  lane-change direction (the side whose lane still reaches the route).
The gap Level 1 chose at the previous decision is left out: copying it locks a waiting car into waiting
(the inertia problem of plan v15, Stage 2; Codevilla et al., 2019).

Modes (traffic/v15_level2 drivers in experiments/v15_pool.py):
- validate: the driver's own Level 2 acts (the rule, or the network's most likely choice);
- collect with the rule: the expert acts and labels (behaviour-cloning data), optionally with
  perturbations (PERTURBATION, plan v15 Stage 2: speed offsets, late commits for 0.6 + 0.8u s, and one
  slow-down or stop near the merge per car), the label staying the expert's unperturbed choice;
- collect with a network (DAgger, plan v15 Stage 3): for each car and each 0.5 s, the network acts with
  probability MIXING['learner_probability'] (0.7) and the expert otherwise, and the expert labels every
  state either way. The expert's choice is computed on the same simulated step, so labelling adds compute,
  not simulated decisions.

Samples (collect): features, labels, masks and weights, one per kept car-decision (SAMPLING): every
decision near an event (an active yield entry, a lane-change candidate or mandatory change, a perturbation
or the 3 s after it, a speed under 5 m/s, a pull-away), and every 5th decision otherwise. A stopped car
whose label keeps it waiting, after a waiting label, is down-weighted to 0.25; every "stopped, so go" label
is kept at weight 1 (plan v15, Stage 2).

The driver ids carry a version: change the split, the inputs, the choices or the settings and the id must
change with them, since results are keyed by driver id.
"""
from __future__ import annotations

import hashlib
import json
import math
import random
from pathlib import Path

from traffic import v15_rule_expert as v3
from traffic.v15_rule_expert import (AGENTS, SETTINGS, _speed, background_response, compose, curve_cap,
                                     entry_options, idm_interaction, lane_change_ok, lane_end_term,
                                     mobil_incentive, normalized, run_up)

RULE_DRIVER = 'l2:rule-v3'
PREFIX = 'l2:'
SCHEMA = 'highway_merge_v15_level2_v01'
GAPS = 4                                   # 0: ahead of the nearest; 1-3: behind the 1st, 2nd or 3rd
LANES = (-1, 0, 1)
OFFSETS = (-2., -1., 0., 1., 2.)           # m/s
OBSERVATION_SIZE = 121
EXCLUDED = (1, 5)                          # the car's own acceleration and its own previous request
KEPT = tuple(i for i in range(OBSERVATION_SIZE) if i not in EXCLUDED)
EXECUTOR_INPUTS = ('entry_active', 'committed', 'dilemma', 'mandatory_left', 'mandatory_right')
INPUT_SIZE = len(KEPT) + len(EXECUTOR_INPUTS)      # 124
HIDDEN = (256, 256)
MIXING = dict(learner_probability=0.7, segment_decisions=5)
PERTURBATION = dict(
    version='p1',
    # pulses: each decision a car without one starts one with this probability, lasting 0.6 + 0.8u s; at an
    # active yield entry half are late commits (one more gap than the expert's), the rest speed offsets
    pulse_probability=0.02, pulse_base_s=0.6, pulse_span_s=0.8, late_commit_share=0.5,
    pulse_offsets=(-2., -1., 1., 2.),
    # one slow-down per car: once the merge (the yield entry's stop line or the end of an ending lane) is
    # within a distance drawn from slow_distance_m, the target drops to one of slow_speeds until the speed has
    # been within 0.5 m/s of it for a time drawn from slow_hold_s, at most slow_max_s
    slow_distance_m=(40., 150.), slow_speeds=(0., 2., 5.), slow_hold_s=(0.5, 2.), slow_max_s=8.,
)
SAMPLING = dict(routine_every=5, event_after_perturbation_s=3., event_speed_m_s=5., stopped_m_s=.5,
                waiting_weight=.25)
_RULE_SEED, _MIX_SEED = 0x5eed1, 0x5eed2


def gap_index(waited):
    """The gap choice that enters where v3 enters: its count of waited-for stream cars, capped at GAPS - 1."""
    return min(int(waited), GAPS - 1)


def select_gap_from(stop, speed, stream, length=5., cap=20., safe=None, skip=0, s=SETTINGS):
    """v3's gap selection (traffic/v15_rule_expert.select_gap) entering no earlier than gap `skip`.

    A gap is accepted only once at least `skip` stream cars have been waited for, and only if it passes the
    same safety test. With skip 0 it is select_gap exactly, and with any skip up to the count select_gap
    waits for it returns the same plan."""
    safe = s['entry_safe_m_s2'] if safe is None else safe
    go_time, go_speed = run_up(speed, stop, max(cap, .1))
    passing = sorted((max(0., d) / max(v, 1.), v, vid, params) for d, v, vid, params in stream)
    arrival, waited = go_time, []
    for when, rear_speed, vid, params in passing:
        if when < arrival:
            if when > arrival - s['yield_headway_s']:
                arrival, waited = when + s['yield_headway_s'], waited + [vid]
            continue
        if len(waited) >= skip:
            behind = rear_speed * (when - arrival) - length
            options = [(go_speed, None)] if not waited else entry_options(stop, speed, arrival, cap)
            for u, hold in options:
                response = background_response(rear_speed, u, max(behind, .1), max(rear_speed, 1.), params)
                if behind > 0. and response >= -safe:
                    return dict(T=arrival, u=u, hold=hold, go_s=go_time, rear=vid, gap_at_entry=behind,
                                response=response, waited_for=waited)
        arrival, waited = when + s['yield_headway_s'], waited + [vid]
    u, hold = (go_speed, None) if not waited else entry_options(stop, speed, arrival, cap)[0]
    return dict(T=arrival, u=u, hold=hold, go_s=go_time, rear=None, gap_at_entry=None, response=None,
                waited_for=waited)


def apply_offset(target, offset, free_road):
    """The plan's target plus a Level 2 offset; a positive offset stops at max(target, free road)."""
    if offset == 0.:
        return target
    if offset > 0.:
        return min(target + offset, max(target, free_road))
    return max(0., target + offset)


def features(observation, executor):
    """The learned Level 2's 124 inputs: the observation without fields 1 and 5, then EXECUTOR_INPUTS."""
    row = [float(observation[i]) for i in KEPT]
    return row + [float(executor[name]) for name in EXECUTOR_INPUTS]


# --------------------------------------------------------------------------- the learned Level 2

def build_net(hidden=HIDDEN):
    """Level2Net: a two-layer MLP with one head per choice (lane 3, gap 4, offset 5)."""
    import torch

    class Level2Net(torch.nn.Module):
        def __init__(self):
            super().__init__()
            layers, width = [], INPUT_SIZE
            for size in hidden:
                layers += [torch.nn.Linear(width, size), torch.nn.ReLU()]
                width = size
            self.body = torch.nn.Sequential(*layers)
            self.lane = torch.nn.Linear(width, len(LANES))
            self.gap = torch.nn.Linear(width, GAPS)
            self.offset = torch.nn.Linear(width, len(OFFSETS))

        def forward(self, x):
            h = self.body(x)
            return self.lane(h), self.gap(h), self.offset(h)

    return Level2Net()


def net_choice(net, row):
    """The network's most likely choice for one car: dict(lane, gap, offset)."""
    import torch
    with torch.no_grad():
        lane, gap, offset = net(torch.tensor([row], dtype=torch.float32))
    return dict(lane=LANES[int(lane.argmax())], gap=int(gap.argmax()), offset=OFFSETS[int(offset.argmax())])


# --------------------------------------------------------------------------- the driver

class Level2Driver(v3.RuleExpert):
    """Level 1 (v3's executor) with an explicit Level 2: the rule's, a network's, or DAgger's mixture.

    net: None for the rule Level 2, else a Level2Net. collect: record samples (labels are always the rule's).
    perturb: None or PERTURBATION['version'] (rule only). seed: the task's seed (mixing and perturbations)."""

    def __init__(self, env_source, *, net=None, collect=False, perturb=None, seed=0, record=True):
        super().__init__(env_source, record=record)
        if perturb is not None and (perturb != PERTURBATION['version'] or net is not None or not collect):
            raise ValueError('Perturbations are for the rule Level 2\'s collected data only')
        if net is None and collect is False and perturb is not None:
            raise ValueError('Perturbations need collect')
        self.net, self.collect, self.perturb = net, bool(collect), perturb
        self.mixing = net is not None and self.collect
        self._rule_rng = random.Random(seed ^ _RULE_SEED)
        self._mix_rng = random.Random(seed ^ _MIX_SEED)
        self.samples = []
        self.stats = dict(decisions=0, acted=dict(rule=0, learner=0, perturbed=0),
                          disagree=dict(lane=0, gap=0, offset=0), compared=dict(lane=0, gap=0, offset=0),
                          pulses=0, slow_events=0)
        self._count, self._mix, self._pulse, self._slow, self._last_perturb, self._last_gap = {}, {}, {}, {}, {}, {}

    def _new_episode(self, env):
        key = str(getattr(env, 'directory', id(env)))
        if key != self._episode:
            super()._new_episode(env)
            self._count, self._mix, self._pulse, self._slow, self._last_perturb, self._last_gap = {}, {}, {}, {}, {}, {}

    # ---- Level 2 pieces shared by the rule and the executor
    def _entry_stream(self, env, agent, state, entry, rears, in_maneuver):
        """(stop line distance, stream) at an active yield entry, as v3's _junction builds them; else None."""
        lane = entry.get('target_lane')
        if not (entry.get('known') and entry.get('entry_active') and lane) or in_maneuver:
            return None
        shared, stop = self._distances(env, agent).get(lane), entry.get('distance_to_start_m')
        if shared is None or stop is None:
            return None
        junction_lane = max(0., shared - stop)
        stream = [(rear_distance - junction_lane, rear_speed, rear_id, self._idm(env, rear_id))
                  for _, rear_speed, rear_distance, rear_id, shared_lane in rears if shared_lane == lane]
        return stop, stream

    def _junction_with(self, env, agent, state, entry, rears, in_maneuver, skip):
        """v3's _junction entering no earlier than gap `skip`; a started commit holds as in v3 (skip 0)."""
        s = SETTINGS
        found = self._entry_stream(env, agent, state, entry, rears, in_maneuver)
        if found is None:
            self._commit.pop(agent, None)
            return None
        stop, stream = found
        speed = state['speed']
        previous = self._commit.get(agent) or {}
        plan = select_gap_from(stop, speed, stream, state.get('length', 5.), self._free_road(state), skip=skip)
        kept = False
        if plan['waited_for'] and previous.get('committed') and not previous.get('dilemma'):
            keep = select_gap_from(stop, speed, stream, state.get('length', 5.), self._free_road(state),
                                   safe=s['entry_abort_m_s2'])
            if not keep['waited_for']:
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
                decision.update(committed=True, dilemma=True)
            elif stop_need > s['dilemma_m_s2']:
                decision.update(committed=True, dilemma=True)
        self._commit[agent] = decision
        return decision

    @staticmethod
    def _mandatory(flow):
        own_route = flow['corridors']['own']['route']
        mandatory = None
        if own_route.get('known') and not own_route.get('reachable', True):
            for side, delta in (('left', 1), ('right', -1)):
                route = flow['corridors'][side]['route']
                if flow['corridors'][side]['lane_id'] is not None and route.get('reachable'):
                    if mandatory is None or delta == int(flow['entry'].get('lane_delta') or 0):
                        mandatory = delta
        return mandatory

    def rule_choice(self, env, agent, state, flow, rears, in_maneuver, mandatory, context, mask, plan_target):
        """The rule Level 2 (v3's choices): dict(lane, reason, gap, waited, offset). Changes no state."""
        s = SETTINGS
        speed, neighbors = state['speed'], context['neighbors']
        own_front = neighbors['own']['front']
        gap = waited = None
        found = self._entry_stream(env, agent, state, flow['entry'], rears, in_maneuver)
        if found is not None:
            stop, stream = found
            plan = select_gap_from(stop, speed, stream, state.get('length', 5.), self._free_road(state))
            waited = len(plan['waited_for'])
            gap = gap_index(waited)
        lane, reason = 0, None
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
                ok, _ = lane_change_ok(speed, neighbors[side]['front'], neighbors[side]['rear'])
                if ok and mask[delta + 1]:
                    lane, reason = delta, why
                    break
            else:
                if candidates:
                    lane, reason = candidates[0]
        return dict(lane=lane, reason=reason, gap=gap, waited=waited, offset=0.)

    # ---- perturbations (rule data only)
    def _merge_ahead(self, env, state, flow):
        distances = []
        entry = flow['entry']
        if entry.get('known') and entry.get('entry_active') and entry.get('distance_to_start_m') is not None:
            distances.append(float(entry['distance_to_start_m']))
        distance = env.merge_distance(state)
        if distance is not None:
            distances.append(float(distance))
        return min(distances) if distances else None

    def _perturbed(self, env, agent, state, flow, label):
        """Apply this car's perturbation to the expert's choice: (choice, slow target or None, active)."""
        p, rng, now = PERTURBATION, self._rule_rng, env.time
        choice = dict(label)
        slow = self._slow.get(agent)
        if slow is None:
            slow = self._slow[agent] = dict(distance=rng.uniform(*p['slow_distance_m']), speed=rng.choice(p['slow_speeds']),
                                            hold=rng.uniform(*p['slow_hold_s']), state='armed', since=None, within=None)
        cap = None
        if slow['state'] == 'armed':
            ahead = self._merge_ahead(env, state, flow)
            if ahead is not None and ahead <= slow['distance']:
                slow.update(state='active', since=now)
                self.stats['slow_events'] += 1
        if slow['state'] == 'active':
            if state['speed'] <= slow['speed'] + .5:
                slow['within'] = now if slow['within'] is None else slow['within']
            else:
                slow['within'] = None
            if (slow['within'] is not None and now - slow['within'] >= slow['hold'] - 1e-9) \
                    or now - slow['since'] >= p['slow_max_s'] - 1e-9:
                slow['state'] = 'done'
                self._last_perturb[agent] = now
            else:
                cap = slow['speed']
        pulse = self._pulse.get(agent)
        if pulse is not None and now >= pulse['end'] - 1e-9:
            self._pulse.pop(agent)
            self._last_perturb[agent] = now
            pulse = None
        if pulse is None and cap is None and rng.random() < p['pulse_probability']:
            duration = p['pulse_base_s'] + p['pulse_span_s'] * rng.random()
            if label['gap'] is not None and rng.random() < p['late_commit_share']:
                pulse = dict(kind='late_commit', end=now + duration)
            else:
                pulse = dict(kind='speed_offset', end=now + duration, offset=rng.choice(p['pulse_offsets']))
            self._pulse[agent] = pulse
            self.stats['pulses'] += 1
        if pulse is not None:
            if pulse['kind'] == 'late_commit' and label['waited'] is not None:
                choice['skip'] = label['waited'] + 1
            elif pulse['kind'] == 'speed_offset':
                choice['offset'] = pulse['offset']
        return choice, cap, bool(pulse is not None or cap is not None)

    # ---- one car's decision
    def decide(self, env, agent, ceiling, observation=None):
        """One car's (request m/s^2, lane delta, record, sample or None) at this decision."""
        from traffic.highway_flow_context import flow_context
        s = SETTINGS
        state = env.states[agent]
        speed, measured = state['speed'], state.get('acceleration', 0.)
        context = env._context(agent)
        neighbors = context['neighbors']
        own_front = neighbors['own']['front']
        base_target, source = self._target(env, agent, state)
        cap = curve_cap(self._curve_points(env, agent, state))
        in_maneuver = agent in env.maneuvers
        mask = env._mask(agent, context)
        flow = flow_context(env.graph, state, env.states)
        leaders_ahead, rears = self.projections(env, agent)
        mandatory = self._mandatory(flow)
        previous = self._commit.get(agent) or {}
        entry = flow['entry']
        executor = dict(entry_active=float(bool(entry.get('known') and entry.get('entry_active') and entry.get('target_lane'))),
                        committed=float(bool(previous.get('committed'))), dilemma=float(bool(previous.get('dilemma'))),
                        mandatory_left=float(mandatory == 1), mandatory_right=float(mandatory == -1))

        # Level 2: the rule's choice (always: it is the label), then who acts
        label = self.rule_choice(env, agent, state, flow, rears, in_maneuver, mandatory, context, mask, base_target)
        row = features(observation, executor) if (self.net is not None or self.collect) else None
        n = self._count.get(agent, 0)
        self._count[agent] = n + 1
        actor, slow_cap, perturbed = 'rule', None, False
        choice = dict(label)
        learned = None
        if self.net is not None:
            learned = net_choice(self.net, row)
            use = True
            if self.mixing:
                if n % MIXING['segment_decisions'] == 0:
                    self._mix[agent] = self._mix_rng.random() < MIXING['learner_probability']
                use = self._mix[agent]
            if use:
                actor, choice = 'learner', dict(learned, reason='level2')
        elif self.perturb is not None:
            choice, slow_cap, perturbed = self._perturbed(env, agent, state, flow, label)
            if perturbed:
                actor = 'perturbed'
        skip = choice.get('skip', choice['gap'] if choice['gap'] is not None else 0)

        # Level 1 executes the choice
        target = apply_offset(base_target, float(choice['offset']), self._free_road(state))
        if slow_cap is not None:
            target = min(target, slow_cap)
        plan_target = target
        junction = self._junction_with(env, agent, state, entry, rears, in_maneuver, skip)
        extra = {}
        if junction is not None:
            if junction['committed'] and junction['rear'] is not None:
                target = max(target, min(self._free_road(state), speed))
            elif not junction['committed']:
                target = min(target, junction['hold_m_s'])
                extra['stop_line'] = idm_interaction(
                    speed, 0., junction['stop_line'] + s['idm_min_gap_m'] - s['stop_line_margin_m'])
        target = min(target, cap)
        lane, reason, safety = 0, None, None
        intent = int(choice['lane'])
        if not in_maneuver and intent != 0:
            side = 'left' if intent > 0 else 'right'
            ok, detail = lane_change_ok(speed, neighbors[side]['front'], neighbors[side]['rear'])
            safety = dict(delta=intent, safe=ok, mask=bool(mask[intent + 1]), **detail)
            if ok and mask[intent + 1]:
                lane, reason = intent, choice.get('reason')

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

        # bookkeeping: stats, the record and the sample
        self.stats['decisions'] += 1
        self.stats['acted'][actor] += 1
        if learned is not None:
            for name, valid in (('lane', not in_maneuver), ('gap', label['gap'] is not None), ('offset', True)):
                if valid:
                    self.stats['compared'][name] += 1
                    self.stats['disagree'][name] += int(learned[name] != label[name])
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
                      pull_away=pull_away, task=env.task,
                      level2=dict(actor=actor, label=dict(lane=label['lane'], gap=label['gap'], offset=label['offset']),
                                  acted=dict(lane=int(choice['lane']), gap=choice['gap'], skip=skip,
                                             offset=float(choice['offset'])),
                                  learned=learned, slow_cap=slow_cap))
        sample = None
        if self.collect:
            since_perturb = env.time - self._last_perturb[agent] if agent in self._last_perturb else math.inf
            event = (label['gap'] is not None or label['lane'] != 0 or mandatory is not None or perturbed
                     or since_perturb <= SAMPLING['event_after_perturbation_s'] + 1e-9
                     or speed < SAMPLING['event_speed_m_s'] or pull_away)
            previous_gap = self._last_gap.get(agent)
            self._last_gap[agent] = label['gap']
            if event or n % SAMPLING['routine_every'] == 0:
                waiting = (speed < SAMPLING['stopped_m_s'] and label['gap'] is not None and label['gap'] >= 1
                           and previous_gap is not None and previous_gap >= 1)
                sample = dict(x=row, lane=LANES.index(label['lane']), gap=-1 if label['gap'] is None else label['gap'],
                              offset=OFFSETS.index(label['offset']), lane_valid=not in_maneuver,
                              gap_valid=label['gap'] is not None,
                              weight=SAMPLING['waiting_weight'] if waiting else 1.,
                              actor=('rule', 'learner', 'perturbed').index(actor), event=bool(event),
                              speed=float(speed), t=float(env.time), agent=AGENTS.index(agent))
        return request, lane, record, sample

    def act(self, local, deterministic=True):
        import torch
        if not deterministic:
            raise ValueError('Level 2 drivers act deterministically; mixing and perturbations use their own seed')
        env = self.env_source()
        if env is None:
            raise RuntimeError('The Level 2 driver has no live environment')
        self._new_episode(env)
        actions = torch.zeros(len(AGENTS), 2)
        records = []
        for i, agent in enumerate(AGENTS):
            if agent not in env.agents:
                continue
            if not env.states.get(agent):
                raise ValueError(f'{agent} is controlled but has no state')
            ceiling = float(local[i, SETTINGS['ceiling_input']]) * SETTINGS['ceiling_scale_m_s2']
            request, lane, record, sample = self.decide(env, agent, ceiling, local[i].tolist())
            actions[i, 0] = normalized(request)
            actions[i, 1] = lane
            records.append(record)
            if sample is not None:
                self.samples.append(sample)
        if self.record and records and getattr(env, 'directory', None) is not None:
            with open(Path(env.directory) / 'level2.jsonl', 'a', encoding='utf-8') as handle:
                for record in records:
                    handle.write(json.dumps(record, separators=(',', ':')) + '\n')
        zeros = torch.zeros(len(AGENTS))
        return actions, zeros, zeros.clone(), actions.clone()

    def sample_batch(self):
        """The collected samples as tensors (None when nothing was collected)."""
        import torch
        if not self.samples:
            return None
        return dict(x=torch.tensor([s['x'] for s in self.samples], dtype=torch.float32),
                    lane=torch.tensor([s['lane'] for s in self.samples], dtype=torch.long),
                    gap=torch.tensor([s['gap'] for s in self.samples], dtype=torch.long),
                    offset=torch.tensor([s['offset'] for s in self.samples], dtype=torch.long),
                    lane_valid=torch.tensor([s['lane_valid'] for s in self.samples]),
                    gap_valid=torch.tensor([s['gap_valid'] for s in self.samples]),
                    weight=torch.tensor([s['weight'] for s in self.samples], dtype=torch.float32),
                    actor=torch.tensor([s['actor'] for s in self.samples], dtype=torch.long),
                    event=torch.tensor([s['event'] for s in self.samples]),
                    speed=torch.tensor([s['speed'] for s in self.samples], dtype=torch.float32),
                    t=torch.tensor([s['t'] for s in self.samples], dtype=torch.float32),
                    agent=torch.tensor([s['agent'] for s in self.samples], dtype=torch.long))


def settings_digest():
    """A digest of everything that defines these drivers' behaviour besides the network weights."""
    value = dict(schema=SCHEMA, rule=RULE_DRIVER, expert=v3.DRIVER, expert_settings=SETTINGS, gaps=GAPS, lanes=LANES,
                 offsets=OFFSETS, excluded=EXCLUDED, executor_inputs=EXECUTOR_INPUTS, hidden=HIDDEN, mixing=MIXING,
                 perturbation=PERTURBATION, sampling=SAMPLING)
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
