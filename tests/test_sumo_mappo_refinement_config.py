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
