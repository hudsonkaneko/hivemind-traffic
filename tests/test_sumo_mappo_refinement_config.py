import copy
import pytest
from experiments.sumo_mappo_support import load_config,validate_config


def test_refinement_preserves_safety_and_contract_with_disjoint_new_seeds():
    old=load_config('experiments/configs/sumo_mappo_v4.json')
    new=load_config('experiments/configs/sumo_mappo_v4_refine.json')
    assert new['environment']==old['environment'] and new['model']==old['model']
    assert new['evaluation']['acceptance']==old['evaluation']['acceptance']
    assert new['evaluation']['driving_targets']==old['evaluation']['driving_targets']
    assert set(old['evaluation']['test_seeds']).isdisjoint(new['evaluation']['test_seeds'])
    assert new['training']['max_agent_transitions']==1000000


@pytest.mark.parametrize('field,value',[('episodes',1000),('epochs',0),('initial_log_std',1),('coefficient',-1),('lane_change_weight',100)])
def test_imitation_settings_are_bounded(field,value):
    config=load_config('experiments/configs/sumo_mappo_v4_refine.json')
    config['training']['imitation'][field]=value
    with pytest.raises(ValueError): validate_config(config)


def test_demonstrations_cannot_exceed_total_training_budget():
    config=load_config('experiments/configs/sumo_mappo_v4_refine.json')
    config['training']['max_agent_transitions']=48000
    with pytest.raises(ValueError,match='total transition budget'): validate_config(config)


def test_stability_refinement_keeps_physics_masks_and_acceptance_limits():
    before=load_config('experiments/configs/sumo_mappo_v4_refine.json')
    after=load_config('experiments/configs/sumo_mappo_v4_stable.json')
    env=copy.deepcopy(after['environment'])
    assert env['reward_weights']['lane_change']==-.5
    env['reward_weights']['lane_change']=before['environment']['reward_weights']['lane_change']
    assert env==before['environment']
    assert after['model']==before['model']
    assert after['environment']['driving']['lane_change_cooldown_s']==5
    for key in ('acceptance','driving_targets','selection'):
        assert after['evaluation'][key]==before['evaluation'][key]
    assert after['training']['max_agent_transitions']==1000000
    assert after['training']['imitation']['lane_change_weight']==10
    assert after['training']['imitation']['ppo_lane_change_weight']==2
    assert set(before['evaluation']['test_seeds']).isdisjoint(after['evaluation']['test_seeds'])


@pytest.mark.parametrize('value', [0, 21, float('nan'), float('inf'), True, '2'])
def test_ppo_lane_weight_is_finite_numeric_and_bounded(value):
    config=load_config('experiments/configs/sumo_mappo_v4_refine.json')
    config['training']['imitation']['ppo_lane_change_weight']=value
    with pytest.raises(ValueError,match='PPO lane weighting'): validate_config(config)


@pytest.mark.parametrize('value', [1, 2.5, 20])
def test_optional_ppo_lane_weight_keeps_initial_weight(value):
    config=load_config('experiments/configs/sumo_mappo_v4_refine.json')
    config['training']['imitation']['ppo_lane_change_weight']=value
    validate_config(config)
    assert config['training']['imitation']['lane_change_weight']==10
