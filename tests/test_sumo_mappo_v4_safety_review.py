"""Independent adversarial checks of the v04 local/joint maneuver boundary."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from environments.sumo_mappo_env import SumoMappoEnv, physical_acceleration_to_action
from traffic.mappo_driving import DRIVING_DEFAULTS, required_gap
from test_sumo_mappo_env import FakeBackend


class ManeuverBackend(FakeBackend):
    def step(self, commands, *, lane_changes=None, headways=None):
        self.headways = dict(headways or {})
        for agent, delta in (lane_changes or {}).items():
            self.states[agent]['lane_index'] += delta
        return super().step(commands)


@pytest.fixture
def driving_env():
    communication = dict(mode='local', range_m=150., max_neighbors=6, update_hz=10.,
        latency_s=[0.,0.], packet_loss=0., expiry_s=.5, outages_s=[])
    backend = ManeuverBackend()
    env = SumoMappoEnv(dict(driving=deepcopy(DRIVING_DEFAULTS), communication=communication),
                       backend_factory=lambda: backend)
    env.reset(seed=1)
    for states in (env._result.states, backend.states):
        states.pop('ordinary')
        for i, agent in enumerate(env.possible_agents):
            states[agent].update(position=1000.+500*i, speed=20., lane_index=2)
        states['agent_0'].update(position=100., lane_index=0)
    yield env
    env.close()


def arrange(env, agent, **values):
    for states in (env._result.states, env._backend.states):
        states[agent].update(values)


def actions(env, lane=0):
    return {a:np.array([physical_acceleration_to_action(0.), lane if a=='agent_0' else 0],dtype=np.float32)
            for a in env.agents}


def refresh_messages(env):
    env.communication.reset(1)
    env._broadcast()


def test_short_target_never_bypasses_worst_case_braking_distance():
    gap=required_gap(26.8224,26.8224,.8,DRIVING_DEFAULTS)
    assert gap == pytest.approx(2.5+.3*26.8224+26.8224**2/18)
    assert gap > 2.5+1.5*26.8224
    assert required_gap(26.8224,0.,.8,DRIVING_DEFAULTS) > gap


def test_only_fresh_actual_same_lane_leader_authorizes_short_target(driving_env):
    env=driving_env
    arrange(env,'agent_1',position=180.,lane_index=0)
    refresh_messages(env)
    assert env._headway('agent_0') == (.8,True)
    # Still within channel's .5 s retention, but past cooperative .2 s limit.
    env._step_count=3
    assert env._headway('agent_0') == (1.5,False)
    env._step_count=0
    refresh_messages(env)
    assert env._headway('agent_0') == (.8,True)
    msg=env.communication._inbox['agent_0']['agent_1']
    env.communication._inbox['agent_0']['agent_1']=replace(msg,lane=1)
    assert env._headway('agent_0') == (1.5,False)
    # A fresh nearby peer cannot authorize short following behind an ordinary car.
    for states in (env._result.states,env._backend.states):
        states['ordinary']=dict(states['agent_1'],position=150.)
    refresh_messages(env)
    assert env._headway('agent_0') == (1.5,False)


def test_fast_rear_vehicle_blocks_cut_in(driving_env):
    env=driving_env
    arrange(env,'agent_1',position=80.,lane_index=1,speed=26.8224)
    assert env._lane_change_reason('agent_0',1) == 'lane_rear_gap'
    assert env.lane_action_mask('agent_0').tolist() == [0.,1.,0.]


def test_simultaneous_inward_merge_reservation_is_deterministic(driving_env):
    env=driving_env
    arrange(env,'agent_1',position=100.,lane_index=2)
    assert env._lane_change_reason('agent_0',1) is None
    assert env._lane_change_reason('agent_1',-1) is None
    applied,reasons=env._filter_lanes({'agent_1':-1,'agent_0':1})
    assert applied=={'agent_1':0,'agent_0':1}
    assert reasons['agent_1']==['joint_lane_conflict']


def test_post_change_observation_immediately_exposes_cooldown(driving_env):
    env=driving_env
    obs,_,_,_,infos=env.step(actions(env,lane=1))
    assert infos['agent_0']['driving_metrics']['actual_lane_delta']==1
    assert obs['agent_0'][29] == 1.
    assert obs['agent_0'][30:33].tolist() == [0.,1.,0.]
    assert env.lane_action_mask('agent_0').tolist() == [0.,1.,0.]


def test_invalid_fractional_lane_rejects_entire_joint_batch(driving_env):
    env=driving_env
    request=actions(env,lane=1)
    request['agent_3'][1]=.4
    with pytest.raises(ValueError):env.step(request)
    assert env._backend.steps==0
    assert env._backend.states['agent_0']['lane_index']==0


def test_same_slow_vehicle_cannot_pay_repeated_pass_rewards(driving_env):
    env=driving_env
    for states in (env._result.states,env._backend.states):
        states['ordinary']=dict(states['agent_0'],position=180.,speed=10.)
    env.step(actions(env)) # Register a genuine slower leader ahead.
    arrange(env,'agent_0',position=210.,lane_index=1)
    _,_,_,_,infos=env.step(actions(env))
    assert infos['agent_0']['driving_metrics']['pass_completed']==1
    arrange(env,'agent_0',position=100.,lane_index=0)
    env.step(actions(env))
    arrange(env,'agent_0',position=210.,lane_index=1)
    _,_,_,_,infos=env.step(actions(env))
    assert infos['agent_0']['driving_metrics']['pass_completed']==0


def test_outage_revokes_cooperative_permission_immediately(driving_env):
    env=driving_env
    arrange(env,'agent_1',position=180.,lane_index=0)
    refresh_messages(env)
    assert env._headway('agent_0') == (.8,True)
    env.communication.config['outages_s']=[[.1,.5]]
    env._step_count=1
    assert env._headway('agent_0') == (1.5,False)


def test_lane_entry_refilters_acceleration_and_tau_for_destination_leader(driving_env):
    env=driving_env
    arrange(env,'agent_1',position=190.,lane_index=0,speed=20.)
    gap=required_gap(20.,10.,1.5,DRIVING_DEFAULTS)+.01
    for states in (env._result.states,env._backend.states):
        states['ordinary']=dict(states['agent_0'],position=100.+4.8+gap,speed=10.,lane_index=1)
    refresh_messages(env)
    assert env._headway('agent_0') == (.8,True)
    assert env._lane_change_reason('agent_0',1) is None
    request=actions(env,lane=1)
    request['agent_0'][0]=physical_acceleration_to_action(2.)
    _,_,_,_,infos=env.step(request)
    assert infos['agent_0']['driving_metrics']['actual_lane_delta']==1
    assert env._backend.headways['agent_0']==1.5
    assert env._backend.commands[-1]['agent_0']<0.
    assert infos['agent_0']['driving_metrics']['applied_native_headway_s']==1.5
