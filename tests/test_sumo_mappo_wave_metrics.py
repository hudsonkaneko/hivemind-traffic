"""Wave telemetry cannot be inflated by repeated per-agent team info."""
import numpy as np
import pytest
import torch

from experiments.sumo_mappo_rollout import EpisodeMetrics, compare_wave_episodes, RolloutCollector
from test_sumo_mappo_rollout import FakePool, initial, reply
from policies.mappo import MappoActorCritic, MessageObservationEncoder, export_actor


def test_team_wave_metrics_count_once_and_compute_space_time_variance():
    metric = EpisodeMetrics(1)
    packet = reply()
    for info in packet['infos'].values():
        info['wave_metrics'] = dict(downstream_speed_sum=30, downstream_speed_squared_sum=500,
            downstream_speed_count=2, throughput_count=1, elapsed_s=.1, traffic_progress_m=3)
    metric.add(packet)
    metric.add(packet)
    wave = metric.result()['wave']
    assert wave['downstream_speed_count'] == 4
    assert wave['downstream_speed_variance_m2_s2'] == 25
    assert wave['throughput_count'] == 2
    assert wave['traffic_progress_m'] == 6
    assert wave['elapsed_s'] == .2


def episode(seed, variance=10., throughput=100.):
    return dict(seed=seed, wave=dict(elapsed_s=60.,
        downstream_speed_variance_m2_s2=variance, throughput_vehicles_per_hour=throughput))


def test_paired_wave_gain_and_throughput_gate():
    baseline = [episode(2), episode(1)]
    candidate = [episode(1, 8, 99), episode(2, 8, 99)]
    result = compare_wave_episodes(candidate, baseline)
    assert result['wave_targets_met']
    assert result['variance_reduction_fraction'] == pytest.approx(.2)
    assert compare_wave_episodes([episode(1, 8.5, 98)], [episode(1)])['wave_targets_met']
    candidate[0]['wave']['throughput_vehicles_per_hour'] = 90
    assert not compare_wave_episodes(candidate, baseline)['wave_targets_met']


def test_paired_wave_rejects_mismatched_duplicate_or_shortened_episodes():
    with pytest.raises(ValueError, match='identical'):
        compare_wave_episodes([episode(1)], [episode(2)])
    with pytest.raises(ValueError, match='unique'):
        compare_wave_episodes([episode(1), episode(1)], [episode(1)])
    shortened = episode(1)
    shortened['wave']['elapsed_s'] = 30
    with pytest.raises(ValueError, match='duration'):
        compare_wave_episodes([shortened], [episode(1)])
    assert not compare_wave_episodes([episode(1, 0, 0)], [episode(1, 0, 0)])['valid']


def test_rollout_accepts_expanded_message_observations():
    start, end = initial(), reply()
    for packet in (start, end):
        packet['observations'] = {agent: np.zeros(87, np.float32) for agent in packet['agents']}
    class Model:
        def act(self, local):
            assert local.shape[-1] == 87
            a = torch.zeros((len(local), 1))
            return a, torch.zeros(len(local)), torch.zeros(len(local)), a
        def values(self, state, identity):
            return torch.zeros(len(identity))
    pool = FakePool([[(start, [end])]])
    config = {'training': dict(episode_seed_start=1, episode_seed_end=10, gamma=.99, gae_lambda=.95)}
    batch, _, count = RolloutCollector(pool, Model(), config).collect(1, 4)
    assert batch['local_obs'].shape == (4, 87)
    assert count == 4


def test_message_encoder_is_permutation_invariant_and_ignores_masked_values():
    torch.manual_seed(71)
    encoder = MessageObservationEncoder()
    local = torch.randn(3, 27)
    messages = torch.randn(3, 6, 7)
    messages[..., 6] = torch.tensor([1., 0., 1., 0., 1., 0.])
    observations = torch.cat((local, messages.flatten(1)), dim=1)
    original = encoder(observations)
    shuffled = messages[:, [5, 0, 3, 2, 1, 4]]
    assert torch.allclose(original, encoder(torch.cat((local, shuffled.flatten(1)), dim=1)), atol=1e-6)
    messages[:, [1, 3, 5], :6] = 999.
    assert torch.equal(original, encoder(torch.cat((local, messages.flatten(1)), dim=1)))
    messages[..., 6] = 0.
    empty = encoder(torch.cat((local, messages.flatten(1)), dim=1))
    assert torch.equal(empty[..., :27], local)
    assert torch.count_nonzero(empty[..., 27:]) == 0


def test_communication_actor_export_and_likelihood_without_global_state(tmp_path):
    torch.manual_seed(72)
    model = MappoActorCritic(observation_size=69, state_size=280, message_encoder=True)
    observations = torch.zeros(4, 69)
    observations[:, 33] = 1.  # First delivered message mask.
    action, probability, _, latent = model.act(observations)
    reevaluated, _ = model.evaluate_actions(observations, action, latent)
    assert torch.equal(probability, reevaluated)
    output = tmp_path / 'actor.pt'
    export_actor(model, output)
    loaded = torch.jit.load(str(output))
    assert torch.equal(loaded(observations), model.actor(observations))
    with pytest.raises(ValueError, match='width'):
        model.actor(torch.zeros(4, 280))
