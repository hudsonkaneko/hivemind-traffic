import hashlib
from pathlib import Path
import tempfile

import pytest

from traffic.driver_control import DriverControlGate
from traffic.rsl_loop_policy import RslLoopPolicy, physical_action

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('raw,expected', [([2.,-.6],(.5,0.,.6)), ([-2.,2.],(-.5,1.,0.)), ([0.,0.],(0.,0.,0.))])
def test_signed_action_preserves_exclusive_throttle_brake(raw, expected):
    physical, clipped, mask = physical_action(raw)
    assert physical == expected
    assert physical[1]*physical[2] == 0
    assert sum(mask) == sum(a!=b for a,b in zip(raw,clipped))


@pytest.mark.parametrize('value', [[0.], [0.,0.,0.], [float('nan'),0.], [True,0.]])
def test_bad_policy_values_rejected(value):
    with pytest.raises(ValueError): physical_action(value)


@pytest.fixture
def exported():
    torch = pytest.importorskip('torch')
    class Actor(torch.nn.Module):
        def forward(self, x):
            # Stand-in embedded normalization is observable in this fixture.
            return (x[:, :2] - 2.) / 2.
    with tempfile.TemporaryDirectory(prefix='policy-test-', dir=ROOT/'outputs') as folder:
        path = Path(folder)/'policy.pt'
        torch.jit.trace(Actor(),torch.zeros(1,42)).save(str(path))
        yield path, hashlib.sha256(path.read_bytes()).hexdigest()


def test_actual_jit_loaded_once_normalized_once_and_gate_keeps_ownership(exported):
    path, digest = exported
    policy = RslLoopPolicy(path,digest,episode_id='test')
    observation = [3.,4.] + [0.]*40
    command = policy.command(observation,tick=0,observation_tick=0)
    assert (command.steering_rad,command.throttle,command.brake) == (.25,1.,0.)
    gate = DriverControlGate('test','ego')
    applied = gate.step(tick=0,dt_s=1/120,command=command)
    assert applied.steering_rad == pytest.approx(.5/120)
    assert command.expires_tick == 12
    # Delete the file after load; ongoing inference uses the verified actor.
    path.unlink()
    assert policy.command(observation,tick=2,observation_tick=2).sequence == 1
    assert policy.metadata['normalizer'].startswith('embedded')


def test_sha_mismatch_precedes_deserialization(exported,monkeypatch):
    torch=pytest.importorskip('torch')
    monkeypatch.setattr(torch.jit,'load',lambda *a,**k: pytest.fail('Must not deserialize'))
    with pytest.raises(ValueError,match='SHA256 mismatch'):
        RslLoopPolicy(exported[0],'0'*64,episode_id='test')


@pytest.mark.parametrize('tick,obs_tick', [(0,2),(1,1),(-2,-2),(True,True)])
def test_command_clock_and_freshness(exported,tick,obs_tick):
    policy=RslLoopPolicy(*exported,episode_id='test')
    with pytest.raises(ValueError): policy.command([0.]*42,tick=tick,observation_tick=obs_tick)


def test_observation_size_nonfinite_and_float32_overflow_rejected(exported):
    policy=RslLoopPolicy(*exported,episode_id='test')
    for values in ([0.]*22,[float('nan')]*42,[1e100]*42):
        with pytest.raises(ValueError): policy.predict(values)


def test_new_episode_resets_sequence_and_stop_dominates_inference(exported):
    policy=RslLoopPolicy(*exported,episode_id='test')
    command=policy.command(None,tick=0,observation_tick=0,stop=True)
    assert (command.throttle,command.brake)==(0.,1.)
    with pytest.raises(ValueError): policy.command([0.]*42,tick=0,observation_tick=0)
    policy.reset('next')
    assert policy.command([0.]*42,tick=0,observation_tick=0).sequence==0
    with pytest.raises(ValueError): policy.reset('next')


def test_actor_wrong_output_shape_fails_on_load(tmp_path):
    torch=pytest.importorskip('torch')
    class Wrong(torch.nn.Module):
        def forward(self,x):return x[:,:3]
    with tempfile.TemporaryDirectory(dir=ROOT/'outputs') as directory:
        path=Path(directory)/'wrong.pt'
        torch.jit.trace(Wrong(),torch.zeros(1,42)).save(str(path))
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        with pytest.raises(ValueError,match='shape'):
            RslLoopPolicy(path,digest,episode_id='test')
