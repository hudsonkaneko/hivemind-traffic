"""V06 learning objective plus explicit native/AV lane-change admission.

Background longitudinal control remains native. Raw SUMO lane-change intentions
are admitted through the same headway/TTC test used for AVs; joint reservations
prevent two individually admissible moves into the same gap. In-progress gaps
are audited every tick; native collision avoidance is never disabled.
"""
from copy import deepcopy

from environments.highway_flow_rl_v06_env import HighwayFlowRLV06Env
from traffic.highway_flow_context import route_continuation
from traffic.highway_merge_cutin_v07 import SETTINGS, VERSION, gap_check


class HighwayMergeCutinV07Env(HighwayFlowRLV06Env):
    def __init__(self, *args, cutin_stress=False, **kwargs):
        self.cutin_stress = bool(cutin_stress)
        self._reset_admission()
        super().__init__(*args, **kwargs)

    def _reset_admission(self):
        self.reservations, self.native_moves = {}, {}
        self.admission = dict(version=VERSION, native_requests=0, native_allowed=0,
            native_blocked=0, av_blocked=0, ongoing_gap_violations=0,
            stress_requests=0, stress_started=0, stress_completed=0, events=[])

    def reset(self, seed=None, options=None):
        self._reset_admission()
        return super().reset(seed=seed, options=options)

    def _write_demand(self, path, seed):
        if not self.missed_gap:
            return super()._write_demand(path, seed)
        from traffic.highway_v05_demand import write_demand
        self.population_manifest = write_demand(path, seed, self.graph, self.task,
            self.controlled_count, self.total_vehicles, speed_limit_m_s=self.speed_limit_m_s,
            difficulty=self.difficulty, traffic_mode=self.traffic_mode,
            stabilization_seconds=self.native_warmup_seconds, horizon_seconds=self.horizon * .1,
            controlled_car_follow_model=self.controlled_car_follow_model, missed_gap=True,
            require_missed_gap_blocker=True)
        self.merge_agents = tuple(v['vehicle_id'] for v in self.population_manifest
                                 if v['controlled'] and v['route'] == 'ramp')

    def _target(self, vid, delta):
        state = self.states[vid]
        lane, known = self.graph.adjacent(state['lane_id'], delta)
        return lane if known else None

    def _evidence(self, vid, lane, *, horizon=None, stress=False):
        if lane is None or lane not in self.graph.lanes:
            return dict(permitted=False, reason='unknown_target', checks={})
        ego = self.states[vid]
        projected = ego['lane_position'] / self.graph.lanes[ego['lane_id']]['length'] * self.graph.lanes[lane]['length']
        expanded = dict(self.states)
        # Include admitted moves before there is measurable lateral motion.
        for other, reservation in self.reservations.items():
            if other == vid or other not in self.states:
                continue
            s, target = self.states[other], reservation['target']
            if target not in self.graph.lanes:
                continue
            key = other + ':v07-reservation'
            expanded[key] = dict(s, id=key, lane_id=target,
                lane_position=s['lane_position'] / self.graph.lanes[s['lane_id']]['length'] * self.graph.lanes[target]['length'],
                lateral_position=0.)
        neighbors = {side: self.graph.neighbor(ego, expanded, lane, projected, forward=side == 'front')
                     for side in ('front', 'rear')}
        evidence = gap_check(ego['speed'], neighbors,
            horizon=SETTINGS['horizon_s'] if horizon is None else horizon,
            headway=SETTINGS['stress_headway_s'] if stress else SETTINGS['headway_s'])
        evidence['target_lane'] = lane
        return evidence

    def _mask(self, agent, context):
        mask = super()._mask(agent, context)
        for index, delta in ((0, -1), (2, 1)):
            if mask[index] and not self._evidence(agent, self._target(agent, delta))['permitted']:
                mask[index] = 0.
        return mask

    def _lane_rejection_causes(self, agent, context, delta):
        causes = super()._lane_rejection_causes(agent, context, delta)
        if delta and not self._evidence(agent, self._target(agent, delta))['permitted']:
            causes.append('v07_projected_gap_or_reservation')
            self.admission['av_blocked'] += 1
        return causes

    def _begin_maneuver(self, agent, delta):
        super()._begin_maneuver(agent, delta)
        lane = self._target(agent, delta)
        self.reservations[agent] = dict(target=lane, identity=self.graph.identity(lane), started=self.time, stress=False)

    def _event(self, **event):
        self.admission['events'].append(dict(time_s=self.time, **event))

    def _advance(self):
        if getattr(self, 'states', None) and getattr(self, 'conn', None):
            self._admit_native_moves()
        super()._advance()
        self._audit_moves()

    def _admit_native_moves(self):
        controlled = set(self._controlled_agents())
        for vid in sorted(set(self.states) - controlled):
            # Read native model intent, not the TraCI-modified result. Mode 512
            # retains all native safety checks but prevents unreviewed changes.
            self.conn.vehicle.setLaneChangeMode(vid, 512)
            if vid in self.native_moves or abs(self.states[vid].get('lateral_position', 0.)) > .02:
                continue
            for delta, bit in ((1, 2), (-1, 4)):
                lane = self._target(vid, delta)
                if lane is None or not route_continuation(self.graph, self.states[vid], lane)['reachable']:
                    continue
                raw, _ = self.conn.vehicle.getLaneChangeState(vid, delta)
                evidence = self._evidence(vid, lane)
                rear = evidence.get('checks', {}).get('rear', {}).get('id', '')
                # Stress rows deliberately admit one smaller-but-native-safe
                # gap ahead of an AV. No speedMode/lane safety bits are relaxed.
                stress = (self.cutin_stress and self.admission['stress_requests'] < SETTINGS['stress_max_events']
                    and rear in controlled and not evidence['permitted']
                    and self._evidence(vid, lane, stress=True)['permitted'])
                if not (raw & bit) and not stress:
                    continue
                self.admission['native_requests'] += 1
                if not evidence['permitted'] and not stress:
                    self.admission['native_blocked'] += 1
                    continue
                self.conn.vehicle.changeLane(vid, self.graph.lanes[lane]['index'], self.lane_change_duration + .1)
                reservation = dict(target=lane, identity=self.graph.identity(lane), started=self.time, stress=stress, moved=False)
                self.reservations[vid] = reservation
                self.native_moves[vid] = reservation
                self.admission['native_allowed'] += 1
                self.admission['stress_requests'] += int(stress)
                self._event(vehicle=vid, event='admitted', stress=stress, evidence=evidence)
                break

    def _audit_moves(self):
        for vid, reservation in list(self.reservations.items()):
            state = self.states.get(vid)
            if state is None:
                self.reservations.pop(vid, None)
                self.native_moves.pop(vid, None)
                continue
            edge = self.graph.lanes[state['lane_id']]['edge']
            candidates = [lane for lane, meta in self.graph.lanes.items()
                if meta['edge'] == edge and self.graph.identity(lane) == reservation['identity']]
            if candidates:
                reservation['target'] = candidates[0]
            elapsed = self.time - reservation['started']
            moved = abs(state.get('lateral_speed', 0.)) > .01 or state['lane_id'] == reservation['target']
            if vid in self.native_moves and moved and not reservation.get('moved'):
                reservation['moved'] = True
                self.admission['stress_started'] += int(reservation['stress'])
            if not self._evidence(vid, reservation['target'], horizon=max(.1, 3.1 - elapsed),
                                  stress=reservation['stress'])['permitted']:
                self.admission['ongoing_gap_violations'] += 1
            if vid in self.native_moves:
                centered = abs(state.get('lateral_position', 0.)) < .02 and abs(state.get('lateral_speed', 0.)) < .02
                complete = reservation.get('moved') and centered
                expired = elapsed > 3.3 and not reservation.get('moved')
                if complete or expired:
                    self.admission['stress_completed'] += int(complete and reservation['stress'])
                    self._event(vehicle=vid, event='completed' if complete else 'native_refused', stress=reservation['stress'])
                    self.native_moves.pop(vid, None)
                    self.reservations.pop(vid, None)
            elif vid not in self.maneuvers:
                self.reservations.pop(vid, None)

    def report(self):
        result = super().report()
        result.update(v07_admission=deepcopy(self.admission), cutin_stress=self.cutin_stress,
                      lane_admission_version=VERSION)
        return result
