import numpy as np
import pytest

from traffic.highway_stage1_teacher import (
    SPEED_CAP, teacher_action, teacher_actions, required_gap, acc_response_ceiling,
)


def observation(speed=20.0, limit=SPEED_CAP):
    obs = np.zeros(93, dtype=np.float32)
    obs[0] = speed / SPEED_CAP
    obs[4] = limit / SPEED_CAP
    obs[27] = 1.0
    obs[30:33] = 1.0
    obs[43] = 2.0 / 4.5
    obs[44] = limit / SPEED_CAP
    return obs


def physical(action):
    return -4.5 + (float(action[0]) + 1.0) * 3.25


def neighbor(obs, start, gap, speed):
    ego_speed = float(obs[0]) * SPEED_CAP
    obs[start:start + 3] = (gap / 120.0, (speed - ego_speed) / SPEED_CAP, 1.0)


def test_teacher_tapers_acceleration_before_limit_and_does_not_weave():
    far = teacher_action(observation(15.0))
    near = teacher_action(observation(SPEED_CAP - 0.3))
    at_cap = teacher_action(observation(SPEED_CAP))
    assert 1.9 < physical(far) <= 2.0
    assert 0.0 < physical(near) < 0.3
    assert physical(at_cap) < 0.0
    assert far[1] == near[1] == at_cap[1] == 0


def test_slow_front_causes_feasible_braking_even_when_passing():
    obs = observation()
    neighbor(obs, 6, 25.0, 10.0)
    action = teacher_action(obs)
    assert action[1] == 1.0
    assert physical(action) < 0.0


def test_no_lane_change_when_alternatives_have_no_speed_advantage():
    obs = observation(15.0)
    for start in (6, 12, 18):
        neighbor(obs, start, 25.0, 10.0)
    assert teacher_action(obs)[1] == 0


def test_passing_can_choose_right_when_left_is_masked():
    obs = observation(15.0)
    neighbor(obs, 6, 25.0, 10.0)
    obs[32] = 0.0
    assert teacher_action(obs)[1] == -1.0


@pytest.mark.parametrize("mask", [(0, 1, 0), (0, 0, 0)])
def test_masks_cannot_be_bypassed(mask):
    obs = observation(15.0)
    neighbor(obs, 6, 25.0, 10.0)
    obs[30:33] = mask
    assert teacher_action(obs)[1] == 0.0


def test_teacher_respects_observed_ceiling_without_hidden_state():
    obs = observation(12.0)
    obs[43] = -0.8 / 4.5
    assert physical(teacher_action(obs)) == pytest.approx(-0.8, abs=2e-5)


def test_required_gap_preserves_headway_and_stopping_floor():
    for follower, leader in [(0.0, 0.0), (10.0, 10.0), (26.0, 0.0), (26.0, 20.0)]:
        gap = required_gap(follower, leader)
        assert gap >= 2.5 + 1.5 * follower
        assert gap >= 2.5 + 0.1 * follower + follower**2 / 9.0 - leader**2 / 18.0


def test_current_acc_response_matches_measured_following_example():
    # Frozen diagnostic: same current leader as native SUMO; no native API used.
    speed = 15.324225895927606
    front = (48.09934393611613, speed - 0.9532559713850599)
    assert acc_response_ceiling(speed, SPEED_CAP, front) == pytest.approx(0.14191542658094747)
    obs = observation(speed)
    neighbor(obs, 6, *front)
    assert physical(teacher_action(obs)) == pytest.approx(0.14191542658094747, abs=2e-5)


def test_unknown_acc_hysteresis_uses_conservative_response():
    speed, limit = 25.0, SPEED_CAP
    front = (112.5, 25.0)
    response = acc_response_ceiling(speed, limit, front)
    assert response == pytest.approx(0.4 * (limit - speed))
    assert response <= 0.8 * (front[1] - speed) + 0.04 * (front[0] - 2.5 - 1.5 * speed)


def test_acc_prediction_is_finite_at_standstill_and_does_not_change_gap_floor():
    assert np.isfinite(acc_response_ceiling(0.0, SPEED_CAP, (2.5, 0.0)))
    assert acc_response_ceiling(26.0, SPEED_CAP, (5.0, 0.0)) == -4.5
    assert required_gap(26.0, 0.0) > 26.0 * 1.5


def test_merge_only_with_known_safe_target_and_legal_delta():
    obs = observation(15.0)
    obs[28] = 1.0
    obs[33] = 1.0
    obs[35] = 1.0
    assert teacher_action(obs)[1] == 1.0
    neighbor(obs, 40, 4.0, 22.0)
    assert teacher_action(obs)[1] == 0.0
    obs[42] = 0.0
    neighbor(obs, 37, 4.0, 10.0)
    assert teacher_action(obs)[1] == 0.0
    obs[39] = 0.0
    obs[33] = 0.0
    assert teacher_action(obs)[1] == 0.0


def test_projected_merge_target_does_not_allow_pre_junction_lane_change():
    obs = observation(12.0, 13.4112)
    obs[28] = obs[33] = 1.0
    obs[35] = 0.0
    obs[36] = 25.0 / 240.0
    obs[44] = 1.0
    action = teacher_action(obs)
    assert action[1] == 0.0
    assert physical(action) < 1.2


def test_rear_merge_alignment_does_not_override_own_lane_leader():
    obs = observation(15.0)
    obs[28] = obs[33] = 1.0
    obs[35] = 1.0
    neighbor(obs, 40, 5.0, 22.0)
    neighbor(obs, 6, 20.0, 8.0)
    action = teacher_action(obs)
    assert action[1] == 0.0
    assert physical(action) < 0.0


def test_ending_lane_brakes_before_missing_merge():
    obs = observation(15.0)
    obs[28] = 1.0
    obs[27] = 8.0 / 240.0
    assert physical(teacher_action(obs)) == pytest.approx(-4.5)


def test_joint_teacher_is_deterministic_and_does_not_mutate_inputs():
    values = {"agent_2": observation(12.0), "agent_0": observation(20.0)}
    original = {key: value.copy() for key, value in values.items()}
    first, second = teacher_actions(values), teacher_actions(values)
    assert set(first) == set(values)
    for key in values:
        assert np.array_equal(values[key], original[key])
        assert np.array_equal(first[key], second[key])
        assert first[key].dtype == np.float32
        assert first[key].shape == (2,)
        assert np.isfinite(first[key]).all()
        assert np.all(np.abs(first[key]) <= 1.0)


@pytest.mark.parametrize("bad", [np.zeros(81), np.zeros((1, 93)), np.full(93, np.nan), np.full(93, 2.0)])
def test_bad_contract_rejected(bad):
    with pytest.raises(ValueError, match="Teacher"):
        teacher_action(bad)
