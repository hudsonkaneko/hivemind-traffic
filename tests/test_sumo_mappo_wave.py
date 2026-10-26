"""No-learning contracts for explicit communication and wave disturbances."""
from copy import deepcopy
import numpy as np
import pytest
from environments.sumo_mappo_env import SumoMappoEnv
from test_sumo_mappo_env import FakeBackend, actions
from traffic.mappo_sumo import write_episode

COMM = dict(mode="local", range_m=150., max_neighbors=6, update_hz=10., latency_s=[.1,.1], packet_loss=0., expiry_s=.5, outages_s=[])
WAVE = dict(enabled=True,pulse_start_s_range=[10.,15.],pulse_duration_s=4.,pulse_acceleration_m_s2=-2.,detector_position_m=1000.)

def test_delayed_received_only_observations_and_reset():
    env = SumoMappoEnv({"communication": COMM}, backend_factory=FakeBackend)
    try:
        obs,_=env.reset(seed=3)
        assert env.observation_size == 69
        assert env.state().shape == (280,)
        assert all(np.all(value[27:]==0) for value in obs.values())
        obs,*_=env.step(actions(env))
        assert obs["agent_0"][33] == 1
        # Own motion is current, sender fields originate in the previous simulation snapshot.
        assert obs["agent_0"][27] == pytest.approx((190.-env._result.states["agent_0"]["position"])/150., abs=.02)
        again,_=env.reset(seed=3)
        assert all(np.all(value[27:]==0) for value in again.values())
    finally: env.close()

def test_no_comm_same_shape_no_messages():
    env = SumoMappoEnv({"communication":dict(COMM,mode="none")}, backend_factory=FakeBackend)
    try:
        env.reset(seed=3)
        obs,*_=env.step(actions(env))
        assert all(np.all(v[27:]==0) for v in obs.values())
    finally: env.close()

def test_wave_metric_denominator_and_progress_reward():
    cfg={"wave":WAVE,"reward_weights":dict(progress=1.,headway=-.5,jerk=-.02,collision=-10.,intervention=-1.,traffic_progress=1.,wave_fluctuation=-.1)}
    env=SumoMappoEnv(cfg,backend_factory=FakeBackend)
    try:
        env.reset(seed=3)
        _,_,_,_,info=env.step(actions(env))
        metrics=info["agent_0"]["wave_metrics"]
        assert metrics["elapsed_s"] == .1
        assert metrics["downstream_speed_count"] > 0
        assert metrics["traffic_progress_m"] > 0
        assert info["agent_0"]["reward_components"]["traffic_progress"] > 0
    finally: env.close()

def test_seeded_wave_demand_fixed_lane_platoon(tmp_path):
    env=SumoMappoEnv({"wave":WAVE})
    manifest=write_episode(env.config,1,tmp_path/"episode")
    agents=[v for v in manifest["vehicles"] if v["id"].startswith("agent_")]
    assert len(agents)==4 and all(v["lane"]==0 for v in agents)
    assert sorted(v["position"] for v in agents)==[65.,120.,175.,230.]


def test_wave_pulse_applied_only_during_window():
    from traffic.mappo_sumo import MappoSumoBackend, AGENT_IDS
    class Vehicles:
        def __init__(self): self.calls=[]
        def getIDList(self): return [*AGENT_IDS,"background_initial_0_1"]
        def setAcceleration(self,*args): self.calls.append(args)
    class Simulation:
        time=0.
        def getTime(self): return self.time
    class Connection:
        vehicle=Vehicles()
        simulation=Simulation()
    backend=object.__new__(MappoSumoBackend)
    backend.config={"wave":WAVE,"dt":.1}
    backend.connection=Connection()
    backend._episode_start_time=0.
    backend._pulse_start_s=10.
    backend._advance=lambda: None
    backend.connection.simulation.time=9.9
    backend.step({})
    assert backend.connection.vehicle.calls==[]
    backend.connection.simulation.time=10.
    backend.step({})
    assert backend.connection.vehicle.calls==[("background_initial_0_1",-2.,.1)]
    backend.connection.vehicle.calls.clear()
    backend.connection.simulation.time=14.
    backend.step({})
    assert backend.connection.vehicle.calls==[]


def test_wave_rewards_and_throughput_ignore_unaffected_lanes():
    backend=FakeBackend()
    env=SumoMappoEnv({"wave":WAVE},backend_factory=lambda:backend)
    try:
        env.reset(seed=3)
        before=deepcopy(env._result)
        expected=env._wave_metrics(before)
        for state in env._result.states.values():
            if state["lane_index"]!=0:
                state.update(speed=26.8224,acceleration=-4.5,position=1001.,distance=1001.)
        actual=env._wave_metrics(before)
        assert actual==expected
        assert not actual["pulse_leader_present"]
    finally: env.close()


def test_wave_parallel_api_and_seed_contract():
    from pettingzoo.test import parallel_api_test, parallel_seed_test
    def make():
        return SumoMappoEnv({'wave':WAVE, 'communication':dict(COMM,latency_s=[0.,.3],packet_loss=.1)}, backend_factory=FakeBackend)
    env=make()
    try:
        parallel_api_test(env,num_cycles=100)
    finally:
        env.close()
    parallel_seed_test(make,num_cycles=100)
