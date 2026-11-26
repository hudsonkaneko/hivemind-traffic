"""v15 free checkpoint screens: settling points, per-input sensitivity, training versus validation gap."""
import gzip
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from policies.highway_flow_v12 import FlowModelV12, OBSERVATION_SIZE
from traffic import highway_flow_rl_v12_rewards as v12

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('v15_checkpoint_screens', ROOT / 'scripts' / 'v15_checkpoint_screens.py')
screens = importlib.util.module_from_spec(spec)
spec.loader.exec_module(screens)


def model(seed=0):
    torch.manual_seed(seed)
    return FlowModelV12().eval()


def function_actor(f):
    """A stand-in actor whose request is f(previous request in m/s^2), which must stay inside (-4.5, 2)."""
    def mean_net(x):
        request = f(x[:, 5] * 4.5)
        assert ((request > -4.5) & (request < 2.)).all()
        raw = torch.atanh((request + 4.5) / 3.25 - 1.)
        return torch.stack((raw, torch.zeros_like(raw), torch.zeros_like(raw), torch.zeros_like(raw)), 1)
    return SimpleNamespace(actor=SimpleNamespace(mean_net=mean_net))


def test_requests_are_the_deterministic_action_in_m_s2():
    m = model()
    x = torch.rand(16, OBSERVATION_SIZE)
    with torch.no_grad():
        actions = m.act(x, deterministic=True)[0][:, 0]
    assert np.allclose(screens.requests(m, x.numpy()), (-4.5 + (actions + 1.) * 3.25).numpy(), atol=1e-6)


def test_v12_checkpoints_load_with_identical_outputs(tmp_path):
    m = model(3)
    path = tmp_path / 'checkpoint.pt'
    torch.save(dict(model=m.state_dict()), path)
    x = np.random.default_rng(0).random((8, OBSERVATION_SIZE), dtype=np.float32)
    assert np.array_equal(screens.requests(screens.load_actor(path), x), screens.requests(m, x))


def test_a_loop_with_slope_below_one_settles_and_above_one_tips():
    obs = np.zeros(OBSERVATION_SIZE, np.float32)
    settles = screens.settling_points(function_actor(lambda r: 0.5 * r - 1.0), obs)
    assert len(settles['points']) == 1
    assert settles['points'][0]['request'] == pytest.approx(-2.0, abs=1e-3)
    assert settles['points'][0]['slope'] == pytest.approx(0.5, abs=1e-3) and settles['points'][0]['settles']
    assert settles['at_zero'] == pytest.approx(-1.0, abs=1e-3)
    # The request range is bounded, so a tipping point always sits between two settling points.
    three = screens.settling_points(function_actor(lambda r: r - 0.05 * (r + 4) * (r + 2) * (r - 1)), obs)
    assert [p['settles'] for p in three['points']] == [True, False, True]
    assert [p['request'] for p in three['points']] == pytest.approx([-4.0, -2.0, 1.0], abs=2e-3)


def test_sensitivity_shares_sum_to_one_and_match_finite_differences():
    m = model(1)
    states = np.random.default_rng(1).random((32, OBSERVATION_SIZE), dtype=np.float32)
    result = screens.sensitivity(m, states)
    assert set(result) == set(screens.OUTPUTS)
    for output in result.values():
        assert 0 <= output['self_feedback_share'] <= 1 and 0 <= output['target_share'] <= 1
    x = torch.as_tensor(states, dtype=torch.float64)
    m64 = m.double()
    step = 1e-6
    plus, minus = x.clone(), x.clone()
    plus[:, 5] += step
    minus[:, 5] -= step
    request = lambda v: -4.5 + (m64.actor.mean_net(v)[:, 0].tanh() + 1.) * 3.25
    with torch.no_grad():
        numeric = ((request(plus) - request(minus)) / (2 * step)).abs().mean()
    assert result['request_m_s2']['mean_abs_gradient']['previous_request'] == pytest.approx(float(numeric), rel=1e-3)


def _decision(observation117, inputs, **extra):
    return dict(v04_reward_audit=dict(pre_action=dict(observation117=observation117)), v12_audit=dict(inputs=inputs),
                v14_stop=dict(stopped=False, free_road=False), override=False, **extra)


def _write_trace(path, steps):
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, 'wt') as handle:
        for i, agents in enumerate(steps):
            handle.write(json.dumps(dict(time=round(0.2 + 0.1 * i, 6), agents=agents, control_mode='policy')) + '\n')


def test_decision_states_insert_the_target_inputs_at_69(tmp_path):
    obs = list(np.arange(117, dtype=float))
    path = tmp_path / 'case-09' / 'episode-1-x' / 'actions.jsonl.gz'
    _write_trace(path, [dict(agent_0=_decision(obs, [1., 2., 3., 4.]), agent_1=dict(override=False))] * 3)
    states, meta = screens.decision_states(screens.case_trace(tmp_path, 9))
    assert states.shape == (3, OBSERVATION_SIZE) and meta[0] == ('agent_0', 0.2)
    assert list(states[0, v12.TARGET_START:v12.TARGET_STOP]) == [1., 2., 3., 4.]
    assert states[0, v12.TARGET_STOP] == 69. and states[0, 68] == 68.


def _report(passed):
    return dict(case_id='c', seed=1, passed=passed, collisions=0, v14_cut_in=dict(charged=0, events=[]),
                native_emergencies=dict(unique_events=0, events=[]))


def test_gap_compares_each_validation_with_the_training_episodes_before_it(tmp_path):
    obs, inputs = [0.] * 117, [0.] * 4
    steps = [dict(agent_0=_decision(obs, inputs))] * 2
    records = {1: True, 2: False, 3: False, 4: True}
    actor_update = {1: 0, 2: 5, 3: 0, 4: 5}           # optimizer steps: the actor first changes at episode 2
    for number, passed in records.items():
        (tmp_path / 'updates').mkdir(exist_ok=True)
        (tmp_path / 'updates' / f'{number:05d}.json').write_text(json.dumps(dict(
            report=_report(passed), critic_only=False, optimization=dict(optimizer_steps=actor_update[number]), row={})))
        _write_trace(tmp_path / 'train' / f'episode-{number - 1:04d}' / 'episode-x' / 'actions.jsonl.gz', steps)
    for label, passes in (('initial-validation', [True, True]), ('periodic-validation-00004', [False, True])):
        (tmp_path / f'{label}.json').write_text(json.dumps(dict(reports=[_report(p) for p in passes], summary={})))
        for i in range(len(passes)):
            _write_trace(tmp_path / label / f'case-{i:02d}' / 'episode-x' / 'actions.jsonl.gz', steps)
    result = screens.gap_screen(tmp_path, lambda report: dict(passed=report['passed']))
    assert [r['validation'] for r in result] == ['initial-validation', 'periodic-validation-00004']
    initial, periodic = result
    assert initial['training_episodes'] == [1, 2]       # the starting actor ran episodes 1-2
    assert initial['training']['pass_rate'] == 0.5 and initial['validation_side']['pass_rate'] == 1.0
    assert initial['pass_rate_gap'] == pytest.approx(-0.5)
    assert periodic['training_episodes'] == [1, 2, 3, 4]
    assert periodic['training']['passed'] == 2 and periodic['validation_side']['passed'] == 1
