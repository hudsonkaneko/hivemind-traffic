"""Hybrid likelihood, mask and deployment tests; no simulator or training run."""
import copy
import numpy as np
import pytest
import torch

from policies.mappo import MappoActorCritic, export_actor


@pytest.fixture
def model():
    torch.set_num_threads(1)
    return MappoActorCritic(observation_size=81, state_size=328, message_encoder=True,
                           hybrid_actions=True, actor_hidden_sizes=(16,), critic_hidden_sizes=(16,))


def observations(n=20):
    obs = torch.rand(n,81)
    obs[:,30:33] = torch.tensor([1.,1.,1.])
    obs[:,40::8] = 1
    return obs


def test_sampled_hybrid_likelihood_and_mask(model):
    obs=observations()
    obs[:10,30:33]=torch.tensor([0.,1.,0.])
    action,log_prob,entropy,latent=model.act(obs)
    assert action.shape == latent.shape == (20,2)
    assert (action[:10,1]==0).all()
    assert set(action[:,1].tolist()) <= {-1.,0.,1.}
    replay, replay_entropy=model.evaluate_actions(obs,action,latent)
    torch.testing.assert_close(replay,log_prob,rtol=0,atol=1e-6)
    torch.testing.assert_close(replay_entropy,entropy)
    assert torch.isfinite(entropy).all()
    bad=action.clone();bad[0,1]=1
    bad_latent=latent.clone();bad_latent[0,1]=1
    with pytest.raises(ValueError,match='mask'):
        model.evaluate_actions(obs,bad,bad_latent)
    bad[0,1]=.5
    with pytest.raises(ValueError,match='exactly'):
        model.evaluate_actions(obs,bad,bad_latent)


def test_joint_likelihood_is_gaussian_plus_categorical(model):
    obs=observations(4)
    action,lp,entropy,latent=model.act(obs)
    from policies.mappo import _log_tanh_jacobian
    gaussian=model._distribution(obs)
    categorical=torch.distributions.Categorical(logits=model.actor.lane_logits(obs,model.actor.mean_net(obs)))
    expected=(gaussian.log_prob(latent[:,:1])-_log_tanh_jacobian(latent[:,:1])).sum(-1)
    expected+=categorical.log_prob((action[:,1]+1).long())
    torch.testing.assert_close(lp,expected)
    torch.testing.assert_close(entropy,model._entropy(gaussian)+categorical.entropy())


def test_actor_export_and_absent_message_invariance(model,tmp_path):
    obs=observations(4)
    obs[:,40::8]=0
    missing=obs.clone();missing[:,33:]=torch.randn(4,48);missing[:,40::8]=0
    torch.testing.assert_close(model.actor(obs),model.actor(missing))
    obs[:,40::8]=1
    permuted=torch.cat((obs[:,:33],obs[:,33:].reshape(4,6,8).flip(1).reshape(4,48)),-1)
    torch.testing.assert_close(model.actor(obs),model.actor(permuted))
    actor=torch.jit.load(str(export_actor(model,tmp_path/'policy.pt')))
    torch.testing.assert_close(actor(obs),model.actor(obs))
    assert actor(torch.zeros(4,81))[:,1].eq(0).all()
    for width in (27,69):
        with pytest.raises((ValueError,torch.jit.Error)):
            actor(torch.zeros(4,width))


def test_empty_mask_sample_stays_and_latent_must_match(model):
    obs=torch.zeros(4,81)
    action,lp,entropy,latent=model.act(obs)
    assert action[:,1].eq(0).all() and torch.isfinite(lp).all()
    bad=latent.clone();bad[0,1]=1
    with pytest.raises(ValueError,match='categorical'):
        model.evaluate_actions(obs,action,bad)


def test_random_baseline_respects_driving_mask():
    from experiments.run_sumo_mappo import baseline_actions
    obs=np.zeros(81,dtype=np.float32);obs[32]=1
    a=baseline_actions('random',{'agent_0':obs},np.random.default_rng(2))['agent_0']
    assert a.shape==(2,) and a[1]==1


def test_v04_configuration_and_checkpoint_contract(tmp_path,monkeypatch):
    from experiments.sumo_mappo_support import load_config,validate_config
    from experiments import run_sumo_mappo as runner
    from policies.mappo import MappoTrainer
    cfg=load_config('experiments/configs/sumo_mappo_v4.json')
    model=runner.make_model(cfg)
    assert model.action_size==2
    changed=copy.deepcopy(cfg);changed['model']['action']='tanh_gaussian_normalized_acceleration'
    with pytest.raises(ValueError,match='contract'):validate_config(changed)
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    path=tmp_path/'outputs/checkpoint.pt';path.parent.mkdir()
    trainer=MappoTrainer(model)
    runner.save_checkpoint(path,model,trainer,cfg,0,0,1000000,(),0)
    restored=runner.make_model(cfg);runner.load_checkpoint(path,restored,cfg)
    assert runner.model_hash(restored)==runner.model_hash(model)
    legacy=load_config('experiments/configs/sumo_mappo_v3.json')
    with pytest.raises(ValueError,match='configuration differs'):
        runner.load_checkpoint(path,runner.make_model(legacy),legacy)
