"""Opt-in v04 maneuver guard using local neighbors and delivered V2V only."""
import math
import numpy as np

DRIVING_DEFAULTS = dict(enabled=True, cooperative_headway_s=.8, ordinary_headway_s=1.5,
    cooperative_max_age_s=.2, response_time_s=.3, follower_braking_m_s2=4.5,
    leader_braking_m_s2=9., lane_change_min_ttc_s=3., lane_change_cooldown_s=5.,
    slow_leader_speed_m_s=17., pass_speed_deficit_m_s=2., pass_clearance_m=10.)


def validate_driving(config):
    if set(config) != set(DRIVING_DEFAULTS) or config['enabled'] is not True:
        raise ValueError('Complete enabled v04 driving contract required')
    for key,value in config.items():
        if key != 'enabled' and (isinstance(value,bool) or not math.isfinite(value) or value <= 0):
            raise ValueError('Invalid driving setting '+key)
    if not .8 <= config['cooperative_headway_s'] <= config['ordinary_headway_s']:
        raise ValueError('Cooperative headway must be at least .8s and no greater than ordinary')
    if config['ordinary_headway_s'] < 1.5 or config['cooperative_max_age_s'] > .2:
        raise ValueError('Ordinary headway/freshness lower safety bounds violated')
    if config['response_time_s'] < .3 or config['follower_braking_m_s2'] > 4.5 or config['leader_braking_m_s2'] < 9:
        raise ValueError('Conservative stopping assumptions cannot be weakened')
    if config['lane_change_min_ttc_s'] < 3 or config['lane_change_cooldown_s'] < 5:
        raise ValueError('Lane-change TTC/cooldown cannot be weakened')


def required_gap(speed, leader_speed, headway, config, min_gap=2.5):
    return max(min_gap+headway*speed,
        min_gap+config['response_time_s']*speed+speed*speed/(2*config['follower_braking_m_s2'])
        -leader_speed*leader_speed/(2*config['leader_braking_m_s2']))


class DrivingMixin:
    def _messages(self, agent):
        state=self._result.states.get(agent)
        return self.communication.receive(agent,self._step_count*self.dt,state['position']) if self.communication and state else []

    def _headway(self, agent, neighbor=None):
        cfg=self.config['driving'];state=self._result.states[agent]
        neighbor=self._neighbor(agent,0,True) if neighbor is None else neighbor
        cooperative=False
        if neighbor and neighbor[2] in self.possible_agents:
            for message in self._messages(agent):
                if (message.sender_id==neighbor[2] and message.lane==state['lane_index']
                    and message.position_m>state['position']
                    and self._step_count*self.dt-message.timestamp_s<=cfg['cooperative_max_age_s']+1e-9):
                    cooperative=True;break
        return cfg['cooperative_headway_s'] if cooperative else cfg['ordinary_headway_s'],cooperative

    def _required_gap(self,agent,neighbor=None):
        neighbor=self._neighbor(agent,0,True) if neighbor is None else neighbor
        state=self._result.states[agent];headway,cooperative=self._headway(agent,neighbor)
        return required_gap(state['speed'],neighbor[1] if neighbor else state['speed'],headway,self.config['driving']),cooperative

    def _lane_change_reason(self,agent,delta):
        if delta==0:return None
        cfg=self.config['driving'];ego=self._result.states[agent]
        if not 0<=ego['lane_index']+delta<3:return 'lane_boundary'
        if self._step_count*self.dt-self._last_lane_changes.get(agent,-math.inf)<cfg['lane_change_cooldown_s']-1e-9:
            return 'lane_cooldown'
        # Both candidate-lane sides use ordinary headway. Future cooperation is not assumed.
        for ahead in (True,False):
            neighbor=self._neighbor(agent,delta,ahead)
            if neighbor:
                gap,speed,_=neighbor
                follower,leader=(ego['speed'],speed) if ahead else (speed,ego['speed'])
                if gap<required_gap(follower,leader,cfg['ordinary_headway_s'],cfg):return 'lane_front_gap' if ahead else 'lane_rear_gap'
                closing=follower-leader
                if closing>0 and gap/closing<cfg['lane_change_min_ttc_s']:return 'lane_front_ttc' if ahead else 'lane_rear_ttc'
        return None

    def lane_action_mask(self,agent):
        return np.asarray([self._lane_change_reason(agent,d) is None for d in (-1,0,1)],dtype=np.float32)

    def _filter_lanes(self,requests):
        applied={};reasons={}
        for agent,delta in requests.items():
            reason=self._lane_change_reason(agent,delta)
            applied[agent]=0 if reason else delta
            reasons[agent]=[reason] if reason else []
        # Joint batch resolution is a safety guard, never a leaked actor observation.
        accepted=[]
        for agent in sorted(applied):
            if not applied[agent]:continue
            state=self._result.states[agent];target=state['lane_index']+applied[agent]
            for prior in accepted:
                other=self._result.states[prior];other_target=other['lane_index']+applied[prior]
                overlapping=abs(state['position']-other['position'])<max(
                    required_gap(state['speed'],other['speed'],self.config['driving']['ordinary_headway_s'],self.config['driving']),
                    required_gap(other['speed'],state['speed'],self.config['driving']['ordinary_headway_s'],self.config['driving']))+max(state['length'],other['length'])
                if overlapping and (target==other_target or (target==other['lane_index'] and other_target==state['lane_index'])):
                    applied[agent]=0;reasons[agent].append('joint_lane_conflict');break
            if applied[agent]:accepted.append(agent)
        return applied,reasons

    def _driving_safety_filter(self,agent,requested,*,lane_delta=0,ordinary=False):
        ego=self._result.states[agent];speed=ego['speed'];cfg=self.config['driving']
        filtered=min(requested,(self.config['speed_limit_m_s']-speed)/self.dt)
        reasons=['speed_ceiling'] if filtered<requested-1e-8 else []
        neighbor=self._neighbor(agent,lane_delta,True)
        if neighbor:
            gap,leader_speed,_=neighbor
            headway=cfg['ordinary_headway_s'] if ordinary else self._headway(agent,neighbor)[0]
            # Require the next-step gap to cover both headway and conservative stopping distance.
            def safe(acceleration):
                next_speed=max(0.,speed+acceleration*self.dt)
                next_leader=max(0.,leader_speed-cfg['leader_braking_m_s2']*self.dt)
                next_gap=gap+(next_leader-next_speed)*self.dt
                return next_gap>=required_gap(next_speed,next_leader,headway,cfg)
            if not safe(filtered):
                lo,hi=-4.5,filtered
                for _ in range(30):
                    mid=(lo+hi)/2
                    if safe(mid):lo=mid
                    else:hi=mid
                filtered=lo;reasons.append('stopping_gap')
        if filtered<-speed/self.dt:
            filtered=-speed/self.dt;reasons.append('zero_speed')
        return float(np.clip(filtered,-4.5,2)),tuple(reasons)

    def scripted_action(self,agent,cooperative=True):
        from environments.sumo_mappo_env import physical_acceleration_to_action
        cfg=self.config['driving'];state=self._result.states[agent];neighbor=self._neighbor(agent,0,True)
        target=self.config['cruise_speed_m_s'];delta=0
        if neighbor:
            gap,speed,leader=neighbor
            headway,_=self._headway(agent,neighbor) if cooperative else (cfg['ordinary_headway_s'],False)
            desired=required_gap(state['speed'],speed,headway,cfg)
            # Reference controller keeps a 5 m buffer above the hard bound so
            # ordinary acceleration does not continually rely on the shield.
            target=min(target,max(0.,speed+(gap-desired-5.)*.3))
            if speed<self.config['cruise_speed_m_s']-cfg['pass_speed_deficit_m_s'] and gap<100:
                for candidate in (1,-1):
                    front=self._neighbor(agent,candidate,True)
                    if self._lane_change_reason(agent,candidate) is None and (front is None or front[1]>speed+1):
                        delta=candidate;break
        acceleration=float(np.clip((target-state['speed'])*.6,-4.5,2))
        return np.array([physical_acceleration_to_action(acceleration),delta],dtype=np.float32)
