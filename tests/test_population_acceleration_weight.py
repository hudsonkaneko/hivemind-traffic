import pytest
import torch
from policies.mappo import MappoActorCritic, MappoTrainer


def test_acceleration_weight_changes_only_acceleration_objective():
    model = MappoActorCritic(observation_size=93, state_size=752, num_agents=8,
        message_encoder=True, hybrid_actions=True, driving_local_size=45,
        population_critic=True)
    obs = torch.zeros(3, 93)
    obs[:, 31] = 1
    labels = torch.tensor([[.4, 0.], [-.2, 0.], [.8, 0.]])
    base = MappoTrainer(model, imitation_coef=.04)
    revised = MappoTrainer(model, imitation_coef=.04, imitation_acceleration_weight=50.)
    total, acceleration, lane = base._imitation_loss(obs, labels)
    new_total, new_acceleration, new_lane = revised._imitation_loss(obs, labels)
    assert torch.equal(acceleration, new_acceleration)
    assert torch.equal(lane, new_lane)
    assert torch.allclose(new_total - total, 49 * acceleration)
    for invalid in (0, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            MappoTrainer(model, imitation_acceleration_weight=invalid)
