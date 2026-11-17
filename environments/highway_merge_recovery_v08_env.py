"""V07 admission plus dense merge learning and guaranteed cut-in exposure."""
from copy import deepcopy
from environments.highway_merge_cutin_v07_env import HighwayMergeCutinV07Env
from traffic.highway_flow_rl_v06_rewards import jerk_excess
from traffic.highway_merge_recovery_v08 import REWARD_VERSION, SETTINGS, recovery_terms, write_cutin_demand


class HighwayMergeRecoveryV08Env(HighwayMergeCutinV07Env):
    def _reset_admission(self):
        super()._reset_admission()
        self.cutin_spec = None
        self.cutin_proof = dict(moving_at_handoff=False, completed_into_target=False, handoff_headway_s=None)
        self.request_history = {}
        self.recovery_audit = dict(reward_terms={}, per_car={})

    def _write_demand(self, path, seed):
        if not self.cutin_stress:
            return super()._write_demand(path, seed)
        self.population_manifest, self.cutin_spec = write_cutin_demand(path, seed, self.graph, self.speed_limit_m_s)
        self.merge_agents = ()

    def _spawn_ready(self):
        ready = super()._spawn_ready()
        if not self.cutin_stress or not ready:
            return ready
        spec = self.cutin_spec
        cutter = self.states[spec['cutter']]
        ego = self.states[spec['ego']]
        gap = cutter['lane_position'] - cutter['length'] - ego['lane_position']
        headway = gap / max(ego['speed'], .1)
        moving = abs(cutter.get('lateral_speed', 0.)) > .01
        if moving and self.admission['stress_started'] and .5 <= headway < 1.5:
            self.cutin_proof.update(moving_at_handoff=True, handoff_headway_s=headway,
                handoff_time=self.time, native_speed_mode=self.conn.vehicle.getSpeedMode(spec['cutter']),
                native_lane_change_mode=self.conn.vehicle.getLaneChangeMode(spec['cutter']))
            self.conn.vehicle.setSpeed(spec['ego'], -1.)
            self.conn.vehicle.setSpeed(spec['cutter'], -1.)
            return True
        return False  # bounded native spawn loop fails rather than train an empty scenario

    def _admit_native_moves(self):
        spec = self.cutin_spec
        if self.cutin_stress and spec and not self.cutin_proof['moving_at_handoff']:
            ego, cutter = spec['ego'], spec['cutter']
            if ego in self.states and cutter in self.states:
                self.conn.vehicle.setSpeedMode(ego, 31)
                self.conn.vehicle.setSpeedMode(cutter, 31)
                self.conn.vehicle.setLaneChangeMode(ego, 512)
                self.conn.vehicle.setLaneChangeMode(cutter, 512)
                self.conn.vehicle.setSpeed(ego, spec['ego_speed'])
                self.conn.vehicle.setSpeed(cutter, spec['cutter_speed'])
                lane = spec['target_lane']
                if cutter not in self.native_moves and self._evidence(cutter, lane, stress=True)['permitted']:
                    self.conn.vehicle.changeLane(cutter, self.graph.lanes[lane]['index'], 3.1)
                    move = dict(target=lane, identity=self.graph.identity(lane), started=self.time, stress=True, moved=False)
                    self.native_moves[cutter] = move
                    self.reservations[cutter] = move
                    self.admission['stress_requests'] += 1
                    self.admission['native_requests'] += 1
                    self.admission['native_allowed'] += 1
                    self._event(vehicle=cutter, event='fixture_requested', stress=True)
        # No opportunistic stress: the one explicit cut-in is measured above.
        enabled = self.cutin_stress
        self.cutin_stress = False
        try:
            super()._admit_native_moves()
        finally:
            self.cutin_stress = enabled

    def _audit_moves(self):
        super()._audit_moves()
        if not self.cutin_spec:
            return
        state = self.states.get(self.cutin_spec['cutter'])
        if (state and self.admission['stress_started'] and self.cutin_proof['moving_at_handoff']
            and self.graph.identity(state['lane_id']) == self.graph.identity(self.cutin_spec['target_lane'])
            and abs(state.get('lateral_position', 0.)) < .02 and abs(state.get('lateral_speed', 0.)) < .02):
            self.cutin_proof['completed_into_target'] = True

    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        terms = super()._extra_rewards(agent, info, snapshot, audit, state, merge_active)
        previous = self.request_history.get(agent)
        request_slew = 0. if previous is None else (audit['requested'] - previous) / .1
        self.request_history[agent] = audit['requested']
        # Actuator lag is not raw-command jerk. Preserve the historical metric
        # and its strict gate, but learn from actual command differences.
        terms['jerk_excess'] = jerk_excess(info['realized_jerk_m_s3'], request_slew, 1.5)
        terms['merge_tracking'] = 0.  # replaced by the longer-range dense term
        extra, desired = recovery_terms(snapshot['ego_state']['speed'], state['speed'] if state else None,
            snapshot['target'], audit['requested'], info['feasible_acceleration_ceiling'],
            info['acceleration_bounds'].get('speed_ceiling'), merge_active)
        terms.update(extra)
        for key, value in terms.items():
            self.recovery_audit['reward_terms'][key] = self.recovery_audit['reward_terms'].get(key, 0.) + value
        car = self.recovery_audit['per_car'].setdefault(agent, dict(peak_command_slew=0., samples=0))
        car['peak_command_slew'] = max(car['peak_command_slew'], abs(request_slew))
        car['samples'] += int(previous is not None)
        info['v08_reward_audit'] = dict(request_slew=request_slew, desired_merge_acceleration=desired,
            pre_action_target=snapshot['target'], terms=terms, first_command=previous is None)
        return terms

    def report(self):
        result = super().report()
        result.update(reward_version=REWARD_VERSION, objective_version=REWARD_VERSION,
            v08_settings=SETTINGS, cutin_fixture=deepcopy(self.cutin_proof),
            cutin_initial_conditions=deepcopy(self.cutin_spec), v08_reward_audit=deepcopy(self.recovery_audit))
        return result
