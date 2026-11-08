"""Bounded v04 SUMO checks, with no model or optimizer."""
import json
from pathlib import Path
from uuid import uuid4
import pytest
import numpy as np
from pettingzoo.test import parallel_api_test,parallel_seed_test
from environments.sumo_mappo_env import SumoMappoEnv
from traffic.mappo_sumo import ROOT
from traffic.mappo_communication import V2VMessage


def config():
    return json.loads((ROOT/'experiments/configs/sumo_mappo_v4.json').read_text())['environment']


def test_optional_lane_intent_preserves_legacy_payload_and_validates():
    legacy=V2VMessage('a','ep',0,0.,100.,0,20.,0.,0.)
    assert 'intended_lane_delta' not in json.loads(legacy.serialize())
    maneuver=V2VMessage('a','ep',0,0.,100.,0,20.,0.,0.,1)
    assert json.loads(maneuver.serialize())['intended_lane_delta']==1
    with pytest.raises(ValueError):V2VMessage('a','ep',0,0.,100.,0,20.,0.,0.,2).serialize()


def test_native_v04_passes_and_dynamic_native_headway_falls_back():
    cfg=config();cfg['communication'].update(latency_s=[0.,0.],packet_loss=0.)
    env=SumoMappoEnv(cfg,run_directory=ROOT/'outputs/sumo_mappo_tests'/('v04-native-'+uuid4().hex[:8]))
    try:
        obs,_=env.reset(seed=1310001)
        assert obs['agent_1'][27]==1
        actions={a:env.scripted_action(a) for a in env.agents}
        for action in actions.values():action[1]=0
        env.step(actions)
        assert env._backend.connection.vehicle.getTau('agent_1')==pytest.approx(.8)
        now=env._step_count*env.dt
        env.communication.config['outages_s']=[[now,now+1.]]
        env.step({a:env.scripted_action(a) for a in env.agents})
        assert env._backend.connection.vehicle.getTau('agent_1')==pytest.approx(1.5)
        assert env._last_observations['agent_1'][27]==0
        passes=changes=collisions=0
        while env.agents:
            _,_,_,_,infos=env.step({a:env.scripted_action(a) for a in env.agents})
            for info in infos.values():
                passes+=info['driving_metrics']['pass_completed']
                changes+=info['driving_metrics']['lane_change_applied']
                collisions+=len(info['collisions'])
                assert info['speed_m_s']<=26.8224+1e-5
        assert passes>=1 and changes>=1 and collisions==0
    finally:env.close()


def test_v04_parallel_api_and_seed_contract():
    cfg=config();cfg['horizon']=12
    directory=ROOT/'outputs/sumo_mappo_tests'/('v04-api-'+uuid4().hex[:8])
    def make():return SumoMappoEnv(cfg,run_directory=directory)
    env=make()
    try:
        parallel_api_test(env,num_cycles=20)
        for _ in range(20):assert env.action_space('agent_0').sample()[1] in (-1,0,1)
    finally:env.close()
    parallel_seed_test(make,num_cycles=12)
