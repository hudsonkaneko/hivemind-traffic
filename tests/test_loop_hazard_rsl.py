"""Isolated CPU RSL checks; tiny synthetic update, never a native training run.

Normal traffic CI skips optional RSL dependencies. The dedicated dependency
interpreter exercises actual model/optimizer/export code without AppLauncher,
Kit, physics, rendering, native policies or user training checkpoints.
"""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
HAS_RSL=importlib.util.find_spec('rsl_rl') is not None


@unittest.skipUnless(HAS_RSL,'Requires isolated RSL/Torch dependency interpreter')
class HazardRslTests(unittest.TestCase):
    def setUp(self):
        import torch
        torch.set_num_threads(1)
        torch.manual_seed(983)

    def model(self,dim=42):
        import torch
        from tensordict import TensorDict
        from environments.loop_hazard_rsl import FixedRoadMLP
        obs=TensorDict({'policy':torch.zeros(1,dim)},batch_size=[1])
        model=FixedRoadMLP(obs,{'actor':['policy']},'actor',2,hidden_dims=[8,8],
                          activation='tanh',obs_normalization=True,
                          distribution_cfg={'class_name':'GaussianDistribution','init_std':.2,'std_type':'log'})
        return model,obs

    def test_fixed_feature_scaling_finite_42_to_2_and_no_empirical_updates(self):
        import torch
        model,obs=self.model()
        before={k:v.clone() for k,v in model.obs_normalizer.state_dict().items()}
        model.update_normalization(obs*10000)
        for k,v in before.items():torch.testing.assert_close(model.obs_normalizer.state_dict()[k],v)
        raw=torch.ones(1,42)
        actual=model.obs_normalizer(raw)
        self.assertAlmostEqual(float(actual[0,0]),1/60)
        self.assertEqual(float(actual[0,4]),0.)
        self.assertEqual(float(actual[0,10]),0.)
        self.assertAlmostEqual(float(actual[0,31]),5.)
        obs['policy']=raw
        output=model(obs)
        self.assertEqual(tuple(output.shape),(1,2))
        self.assertTrue(bool(torch.isfinite(output).all()))
        with self.assertRaises(ValueError):self.model(41)

    def test_jit_embeds_scales_and_matches_repeated_frozen_inference(self):
        import torch
        from tensordict import TensorDict
        model,_=self.model();model.eval()
        raw=torch.arange(84,dtype=torch.float32).reshape(2,42)/13
        with torch.no_grad():expected=model(TensorDict({'policy':raw},batch_size=[2]))
        with tempfile.TemporaryDirectory(dir=ROOT/'outputs') as folder:
            path=Path(folder)/'synthetic-policy.pt'
            torch.jit.script(model.as_jit().eval()).save(str(path))
            loaded=torch.jit.load(str(path),map_location='cpu').eval()
            with torch.no_grad():
                for _ in range(3):torch.testing.assert_close(loaded(raw),expected,rtol=0,atol=0)
            self.assertTrue(any('obs_normalizer._std'==key for key in loaded.state_dict()))

    def test_real_ppo_consumes_eight_synthetic_transitions_and_updates_finitely(self):
        import torch
        from tensordict import TensorDict
        from rsl_rl.runners import OnPolicyRunner
        cfg=json.loads((ROOT/'experiments/lane-follow-rsl-v1.json').read_text())['runner']
        cfg=deepcopy(cfg);cfg['num_steps_per_env']=8;cfg['max_iterations']=1
        for key in ('actor','critic'):
            cfg[key]['class_name']='environments.loop_hazard_rsl:FixedRoadMLP'
            cfg[key]['hidden_dims']=[8,8]
        cfg['algorithm'].update(num_learning_epochs=1,num_mini_batches=2)
        observations=TensorDict({'policy':torch.zeros(1,42)},batch_size=[1])
        env=SimpleNamespace(num_envs=1,num_actions=2,cfg={},device='cpu',get_observations=lambda:observations)
        runner=OnPolicyRunner(env,cfg,log_dir=None,device='cpu')
        alg=runner.alg
        initial=[p.detach().clone() for p in alg.actor.parameters()]
        frozen_scales=alg.actor.obs_normalizer._std.clone()
        with torch.no_grad():
            for step in range(8):
                actions=alg.act(observations)
                self.assertEqual(tuple(actions.shape),(1,2))
                terminal=TensorDict({'policy':torch.full((1,42),(step+1)*.01)},batch_size=[1])
                done=(step+1)%4==0
                observations=TensorDict({'policy':torch.zeros(1,42) if done else terminal['policy'].clone()},batch_size=[1])
                alg.process_env_step(observations,torch.tensor([1.-float(actions.square().sum())*.1]),
                                     torch.tensor([int(done)]),
                                     {'time_outs':torch.tensor([done]),'lane_terminal_observation':terminal})
            self.assertEqual(alg.storage.step,8)
            alg.compute_returns(observations)
        calls=[];original=alg.optimizer.step
        def counted(*a,**kw):calls.append(1);return original(*a,**kw)
        with patch.object(alg.optimizer,'step',side_effect=counted):losses=alg.update()
        self.assertEqual(len(calls),2)
        self.assertEqual(alg.storage.step,0)
        self.assertTrue(all(torch.isfinite(torch.tensor(v)).all() for v in losses.values()))
        self.assertTrue(all(bool(torch.isfinite(p).all()) for p in alg.actor.parameters()))
        self.assertTrue(any(not torch.equal(a,b) for a,b in zip(initial,alg.actor.parameters())))
        torch.testing.assert_close(alg.actor.obs_normalizer._std,frozen_scales,rtol=0,atol=0)

    def wrapper_fixture(self,callback=None,deadline=None):
        import torch
        # Load only the lightweight installed wrapper file. Its native type
        # validation imports are inside __init__, replaced for this fake env.
        path=Path(r'C:\Users\hudso\Documents\IsaacLab\source\isaaclab_rl\isaaclab_rl\rsl_rl\vecenv_wrapper.py')
        if not path.is_file():self.skipTest('Installed lightweight wrapper source unavailable')
        spec=importlib.util.spec_from_file_location('_hazard_rsl_wrapper_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        cls=module.RslRlVecEnvWrapper
        stub=ModuleType('isaaclab_rl.rsl_rl');stub.RslRlVecEnvWrapper=cls
        parent=ModuleType('isaaclab_rl');parent.rsl_rl=stub
        env=SimpleNamespace(device='cpu',num_envs=1,max_episode_length=2100,cfg=SimpleNamespace(is_finite_horizon=False))
        env.unwrapped=env;env.calls=0
        terminal=torch.full((1,42),7.)
        env.terminal=terminal
        def step(actions):
            env.calls+=1;env.applied=actions.clone()
            extras={'lane_transition':{'tick':2},'reset':{'tick':0},
                    'final_observation':{'policy':terminal},'final_truncated':torch.tensor([True]),
                    'final_terminated':torch.tensor([False])}
            return {'policy':torch.zeros(1,42)},torch.tensor([1.]),torch.tensor([False]),torch.tensor([True]),extras
        env.step=step
        def init(instance,task,clip_actions):
            instance.env=task;instance.clip_actions=clip_actions;instance.num_envs=1;instance.device='cpu'
        from environments.loop_hazard_rsl import wrap_hazard_rsl
        with patch.dict(sys.modules,{'isaaclab_rl':parent,'isaaclab_rl.rsl_rl':stub}),patch.object(cls,'__init__',init):
            wrapper=wrap_hazard_rsl(env,deadline=time.monotonic()+30 if deadline is None else deadline,on_transition=callback)
        return wrapper,env

    def test_wrapper_signed_mapping_terminal_copy_callback_once_and_budget_count(self):
        import torch
        records=[]
        wrapper,env=self.wrapper_fixture(lambda *args:records.append(args))
        obs,reward,done,extras=wrapper.step(torch.tensor([[2.,-.7]]))
        torch.testing.assert_close(env.applied,torch.tensor([[.5,0.,.7]]))
        self.assertEqual(wrapper.control_steps,1);self.assertEqual(len(records),1)
        self.assertTrue(bool(extras['time_outs'][0]))
        self.assertEqual(float(obs['policy'][0,0]),0.)
        self.assertEqual(float(extras['lane_terminal_observation']['policy'][0,0]),7.)
        env.terminal.zero_()
        self.assertEqual(float(extras['lane_terminal_observation']['policy'][0,0]),7.)

    def test_invalid_action_and_deadline_do_not_advance_fake_env(self):
        import torch
        wrapper,env=self.wrapper_fixture()
        for actions in (torch.tensor([[float('nan'),0.]]),torch.zeros(1,3)):
            with self.assertRaises(ValueError):wrapper.step(actions)
        self.assertEqual(env.calls,0)
        wrapper,env=self.wrapper_fixture(deadline=0.)
        with self.assertRaises(TimeoutError):wrapper.step(torch.zeros(1,2))
        self.assertEqual(env.calls,0)

    def test_failed_recorder_still_counts_completed_transition(self):
        import torch
        def fail(*args):raise ValueError('recorder failed')
        wrapper,env=self.wrapper_fixture(fail)
        with self.assertRaises(ValueError):wrapper.step(torch.zeros(1,2))
        self.assertEqual((wrapper.control_steps,env.calls),(1,1))


if __name__=='__main__':unittest.main()
