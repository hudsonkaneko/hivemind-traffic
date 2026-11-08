from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.train_lane_follow_rsl import ROOT, read_config

HAS_RSL = importlib.util.find_spec('rsl_rl') is not None


class ConfigTests(unittest.TestCase):
    def test_exact_budget_and_local_logging(self):
        cfg = read_config(ROOT / 'experiments/lane-follow-rsl-v1.json')
        self.assertEqual(cfg['total_timesteps'], 262144)
        self.assertEqual(cfg['runner']['num_steps_per_env'] * cfg['runner']['max_iterations'], 262144)
        self.assertEqual(cfg['runner']['logger'], 'tensorboard')
        self.assertEqual(cfg['rsl_rl_version'], '5.0.0')

    def test_mismatched_budget_or_wrong_bootstrap_rejected(self):
        cfg = read_config(ROOT / 'experiments/lane-follow-rsl-v1.json')
        (ROOT / 'outputs').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / 'outputs') as directory:
            path = Path(directory) / 'config.json'
            for field in ('budget', 'algorithm'):
                changed = deepcopy(cfg)
                if field == 'budget':
                    changed['total_timesteps'] += 1
                else:
                    changed['runner']['algorithm']['class_name'] = 'PPO'
                path.write_text(json.dumps(changed), encoding='utf-8')
                with self.assertRaises(ValueError):
                    read_config(path)


@unittest.skipUnless(HAS_RSL, 'Requires isolated RSL/native Python dependencies')
class RslBoundaryTests(unittest.TestCase):
    def test_signed_actions_are_exclusive(self):
        import torch
        from environments.lane_follow_rsl import signed_vehicle_actions
        actual = signed_vehicle_actions(torch.tensor([[-2., -2.], [.4, .7], [0., 0.]]))
        torch.testing.assert_close(actual, torch.tensor([[-.5, 0., 1.], [.2, .7, 0.], [0., 0., 0.]]))
        self.assertFalse(bool((actual[:, 1] * actual[:, 2]).any()))
        with self.assertRaises(ValueError):
            signed_vehicle_actions(torch.tensor([[float('nan'), 0.]]))

    def test_timeout_uses_terminal_value_once(self):
        import torch
        from tensordict import TensorDict
        from rsl_rl.algorithms import PPO
        from environments.lane_follow_rsl import TerminalObservationPPO
        algo = SimpleNamespace(device='cpu', gamma=.9, critic=lambda x: x['policy'][:, :1])
        current = TensorDict({'policy': torch.full((2, 22), 100.)}, batch_size=[2])
        terminal = TensorDict({'policy': torch.tensor([[10.] * 22, [20.] * 22])}, batch_size=[2])
        extras = {'time_outs': torch.tensor([True, False]), 'lane_terminal_observation': terminal}
        # Bind the actual method to a lightweight subclass instance, without an optimizer.
        instance = object.__new__(TerminalObservationPPO)
        instance.__dict__.update(vars(algo))
        rewards = torch.tensor([1., 2.])
        with patch.object(PPO, 'process_env_step') as parent:
            instance.process_env_step(current, rewards, torch.ones(2), extras)
        torch.testing.assert_close(parent.call_args.args[1], torch.tensor([10., 2.]))
        self.assertNotIn('time_outs', parent.call_args.args[3])
        torch.testing.assert_close(rewards, torch.tensor([1., 2.]))
        self.assertIn('time_outs', extras)

    def test_real_runner_constructs_and_exports_without_learning(self):
        import torch
        from tensordict import TensorDict
        from rsl_rl.runners import OnPolicyRunner
        cfg = read_config(ROOT / 'experiments/lane-follow-rsl-v1.json')
        observations = TensorDict({'policy': torch.zeros(1, 22)}, batch_size=[1])
        env = SimpleNamespace(num_envs=1, num_actions=2, cfg={}, device='cpu',
                              get_observations=lambda: observations)
        runner = OnPolicyRunner(env, deepcopy(cfg['runner']), log_dir=None, device='cpu')
        self.assertEqual(len(runner.alg.optimizer.state), 0)
        policy = runner.get_inference_policy(device='cpu')
        with torch.no_grad():
            expected = policy(observations)
        (ROOT / 'outputs').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / 'outputs') as directory:
            runner.export_policy_to_jit(directory, 'policy.pt')
            loaded = torch.jit.load(str(Path(directory) / 'policy.pt'))
            with torch.no_grad():
                torch.testing.assert_close(loaded(torch.zeros(1, 22)), expected)
        self.assertEqual(len(runner.alg.optimizer.state), 0)


if __name__ == '__main__':
    unittest.main()
