"""v15 Level 2 drivers on expert v4's executor: the policy's learned Level 2, and the rule Level 2, unchanged, with
Level 1 changed to v4's (traffic/v15_rule_expert_v4.py).

Plan v15, "Ramp driving", option A: Level 1 is the executor of both expert v3 and the policy, so a Level 1 change
gives both new driver ids. User decision of November 27, 2026, relayed by the main Claude session: "Yes, as
recommended" (decision 14, step 1: code and tests; the SUMO re-check is step 2 and needs the user's go-ahead).

- 'l2v4:<hash>': a published Level 2 network (traffic/v15_level2.Level2Net, the same weights and inputs as
  'l2:<hash>') on v4's executor. The policy on v4 is 'l2v4:bc7c050f6d3da3ce'.
- 'l2v4:rule-v4': the rule Level 2 on v4's executor; with its choices this driver reproduces expert v4's requests
  and lane requests ('rule:expert-v4'), as 'l2:rule-v3' reproduces 'rule:expert-v3'.

traffic/v15_level2.py and its drivers are unchanged. This module subclasses its Level2Driver and replaces three
methods with copies that use v4's gap selection and committed target (each changed line is marked v4):
_junction_with (v4's junction_plan, entering no earlier than the chosen gap), rule_choice (the rule's gap from v4's
gap selection, with the run-up the car can make: v4's change 4) and decide (v4's limits and committed target).
These drivers validate only: no collection, perturbation or probe.
The network's gap choices were learned from expert v3's labels; whether a relabel and refit are needed is unknown
until the re-check (plan v15, option A).
"""
from __future__ import annotations

import hashlib
import json
import math

from traffic import v15_level2 as l2
from traffic import v15_rule_expert_v4 as v4
from traffic.v15_level2 import (AGENTS, LANES, MIXING, OFFSETS, SAMPLING, apply_offset, features, gap_index,
                                net_choice, probe_choice)
from traffic.v15_rule_expert import (SETTINGS, _speed, compose, curve_cap, idm_interaction, lane_change_ok,
                                     lane_end_term, mobil_incentive)

PREFIX = 'l2v4:'
RULE_DRIVER = 'l2v4:rule-v4'
SCHEMA = 'highway_merge_v15_level2_v4_v01'


def is_driver(driver):
    """A Level 2 driver on v4's executor: 'l2v4:rule-v4' or a published network 'l2v4:<16 hex>'."""
    if not isinstance(driver, str) or not driver.startswith(PREFIX):
        return False
    name = driver[len(PREFIX):]
    return driver == RULE_DRIVER or (len(name) == 16 and all(c in '0123456789abcdef' for c in name))


def network_id(driver):
    """The 'l2:<hash>' id of the published network a 'l2v4:<hash>' driver runs."""
    if not is_driver(driver) or driver == RULE_DRIVER:
        raise ValueError(f'{driver!r} is not a published network on v4\'s executor')
    return l2.PREFIX + driver[len(PREFIX):]


class Level2Driver(l2.Level2Driver):
    """Level 1 (v4's executor) with an explicit Level 2: the rule's (net None) or a network's. Validates only."""

    def __init__(self, env_source, *, net=None, record=True):
        super().__init__(env_source, net=net, collect=False, perturb=None, seed=0, record=record, probe=None)
        self._limits = {}                      # agent: (ceiling, curve cap) at this decision (v4's change 4)

    def _junction_with(self, env, agent, state, entry, rears, in_maneuver, skip):
        """v4's junction (traffic/v15_rule_expert_v4.junction_plan) entering no earlier than gap `skip`."""
        found = self._entry_stream(env, agent, state, entry, rears, in_maneuver)
        if found is None:
            self._commit.pop(agent, None)
            return None
        stop, stream = found
        ceiling, curve = v4.run_up_limits(self, env, agent, state, stop)
        decision = v4.junction_plan(stop, state['speed'], stream, state.get('length', 5.), self._free_road(state),
                                    self._commit.get(agent) or {}, skip=skip, ceiling=ceiling, curve=curve)     # v4
        self._commit[agent] = decision
        return decision

    def rule_choice(self, env, agent, state, flow, rears, in_maneuver, mandatory, context, mask, plan_target):
        """The rule Level 2 on v4's executor (v3's choices, with v4's gap selection): dict(lane, reason, gap, waited,
        offset). Changes no state. traffic/v15_level2.Level2Driver.rule_choice, copied; the changed lines are marked v4."""
        s = SETTINGS
        speed, neighbors = state['speed'], context['neighbors']
        own_front = neighbors['own']['front']
        gap = waited = None
        found = self._entry_stream(env, agent, state, flow['entry'], rears, in_maneuver)
        if found is not None:
            stop, stream = found
            limited, top = v4.achievable(*v4.run_up_limits(self, env, agent, state, stop), cap=self._free_road(state))  # v4
            plan = v4.select_gap(stop, speed, stream, state.get('length', 5.), top, s=limited)     # v4
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

    def decide(self, env, agent, ceiling, observation=None):
        """One car's (request m/s^2, lane delta, record, sample or None): traffic/v15_level2.Level2Driver.decide,
        copied, with v4's limits for the junction and committed target. The changed lines are marked v4."""
        from traffic.highway_flow_context import flow_context
        s = SETTINGS
        state = env.states[agent]
        speed, measured = state['speed'], state.get('acceleration', 0.)
        context = env._context(agent)
        neighbors = context['neighbors']
        own_front = neighbors['own']['front']
        base_target, source = self._target(env, agent, state)
        cap = curve_cap(self._curve_points(env, agent, state))
        self._limits[agent] = (ceiling, cap)                                                # v4: change 4
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
        elif self.probe is not None:
            actor, choice = 'probe', probe_choice(label, self.probe)
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
                target = v4.committed_target(target, junction, speed, self._free_road(state))     # v4: change 3
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


def settings_digest():
    """A digest of everything that defines these drivers' behaviour besides the network weights."""
    value = dict(schema=SCHEMA, base=l2.settings_digest(), rule=RULE_DRIVER, expert=v4.DRIVER, expert_settings=v4.SETTINGS)
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
