"""Deterministic Stage 1 teacher operating only on the 93-field actor input.

This controller supplies candidate demonstrations, not accepted driving policy
labels. Native closed-loop validation must establish its usefulness before BC.
No simulator connection, future state, evaluator label or persistent hidden
vehicle state is used. All longitudinal quantities are SI after decoding.
"""
from __future__ import annotations

from collections.abc import Mapping
import math

import numpy as np


TEACHER_VERSION = "highway_stage1_observation_teacher_v2"
TEACHER_V3_VERSION = "highway_stage1_observation_teacher_v3"
OBSERVATION_SIZE = 93
SPEED_CAP = 26.8224
ACCELERATION_MIN = -4.5
ACCELERATION_MAX = 2.0
MIN_GAP = 2.5
HEADWAY = 1.5
DT = 0.1
FOLLOW_BUFFER = 3.0
SPEED_BUFFER = 0.10
MINOR_STOP_OFFSET_M = 0.1  # SUMO1.27.1 POSITION_EPS minimum stop-line offset


def required_gap(follower_speed: float, leader_speed: float) -> float:
    """Existing 1.5 s and stopping-distance floor, never a shorter teacher gap."""
    return MIN_GAP + max(
        HEADWAY * follower_speed,
        DT * follower_speed + follower_speed**2 / 9.0 - leader_speed**2 / 18.0,
    )


def _neighbor(observation: np.ndarray, start: int, speed: float):
    if observation[start + 2] <= 0.5:
        return None
    return (
        max(0.0, float(observation[start]) * 120.0),
        max(0.0, speed + float(observation[start + 1]) * SPEED_CAP),
    )


def _following_target(speed: float, limit: float, neighbor) -> float:
    if neighbor is None:
        return limit
    gap, leader_speed = neighbor
    margin = gap - required_gap(speed, leader_speed) - FOLLOW_BUFFER
    return min(limit, max(0.0, leader_speed + 0.35 * margin))


def acc_response_ceiling(speed: float, limit: float, neighbor) -> float:
    """Conservative ACC response prediction from measured speed and gap only.

    The controlled vehicle type uses SUMO 1.27.1 ACC. Its ordinary following
    response can be much smaller than the analytical collision/speed ceiling.
    Parameter source: eclipse-sumo/sumo v1_27_1, MSCFModel_ACC.cpp and the official
    Car-Following-Models/ACC documentation. This is an explicit simulator-model
    calibration, not a native API result or a deployability claim.

    Bumper gap includes the 2.5 m standstill margin, whereas the native model
    operates on the remaining space. Its historical mode in the 100--120 m
    transition band is unavailable in this observation; use the smaller of
    its possible responses instead of consulting hidden simulator state.
    Additional native safety/junction responses remain independently measured.
    """
    if neighbor is None:
        return ACCELERATION_MAX
    bumper_gap, leader_speed = neighbor
    gap = bumper_gap - MIN_GAP
    dynamic_margin = min(2.0, max(0.0, 75.0 / max(speed, 1e-6) - 5.0))
    spacing_error = gap - HEADWAY * speed - dynamic_margin
    relative_speed = leader_speed - speed
    speed_response = 0.4 * (limit - speed)
    if abs(spacing_error) < 0.2 and abs(speed - limit) < 0.1:
        following_response = 0.07 * relative_speed + 0.23 * spacing_error
    elif spacing_error < 0.0:
        following_response = 0.23 * relative_speed + 0.8 * spacing_error
    else:
        following_response = 0.8 * relative_speed + 0.04 * spacing_error
    response = (following_response if gap < 100.0 else
                speed_response if gap > 120.0 else min(following_response, speed_response))
    return float(np.clip(response, ACCELERATION_MIN, ACCELERATION_MAX))


def _teacher_action_v2(observation) -> np.ndarray:
    """Return normalized acceleration and right/stay/left from declared inputs.

    Indices 0:33 preserve the previous graph-local layout. The new merge fields
    are 33 known, 34 target lane index/7, 35 signed target lane delta, 36 signed
    distance to target start/240, 37:40 front gap/relative speed/presence,
    40:43 rear gap/relative speed/presence, 43 feasible acceleration ceiling/4.5,
    and 44 target speed limit/cap. The final 48 values contain delivered messages.
    Lane permission still comes exclusively from the stored masks at 30:33.
    """
    obs = np.asarray(observation, dtype=np.float64)
    if obs.shape != (OBSERVATION_SIZE,) or not np.isfinite(obs).all():
        raise ValueError("Teacher requires one finite 93-field Stage 1 observation")
    if np.any(obs < -1.000001) or np.any(obs > 1.000001):
        raise ValueError("Teacher observation is outside the normalized contract")

    speed = max(0.0, float(obs[0]) * SPEED_CAP)
    limit = max(0.0, min(SPEED_CAP, float(obs[4]) * SPEED_CAP) - SPEED_BUFFER)
    own_front = _neighbor(obs, 6, speed)
    target = _following_target(speed, limit, own_front)
    lane_delta = 0
    masks = obs[30:33] > 0.5
    requires_merge = obs[28] > 0.5

    if requires_merge and obs[33] > 0.5:
        merge_front = _neighbor(obs, 37, speed)
        merge_rear = _neighbor(obs, 40, speed)
        merge_limit = max(0.0, float(obs[44]) * SPEED_CAP - SPEED_BUFFER)
        # Respect the current lane limit until the legal limit actually changes.
        target = min(target, _following_target(speed, min(limit, merge_limit), merge_front))
        delta = int(np.rint(obs[35]))
        if delta and masks[delta + 1]:
            front_safe = merge_front is None or merge_front[0] >= required_gap(speed, merge_front[1])
            rear_safe = merge_rear is None or merge_rear[0] >= required_gap(merge_rear[1], speed)
            if front_safe and rear_safe:
                lane_delta = delta
        # When a rear car closes on the projected target slot, align its speed
        # only if front room permits. The legal lane mask still prevents entry.
        if not lane_delta and merge_rear is not None:
            rear_gap, rear_speed = merge_rear
            if rear_gap < required_gap(rear_speed, speed) + FOLLOW_BUFFER:
                front_target = _following_target(speed, limit, merge_front)
                catchup = rear_speed + 1.0
                if front_target > speed + 0.5:
                    target = min(
                        _following_target(speed, limit, own_front),
                        front_target, max(target, catchup),
                    )
    elif not requires_merge and own_front is not None:
        own_gap, leader_speed = own_front
        # A slow front vehicle is necessary; empty lanes alone are not a reason
        # to weave. Compare both candidates and prefer left on a genuine tie.
        if own_gap < 90.0 and leader_speed < limit - 2.0:
            choices = []
            for delta, start in ((1, 12), (-1, 18)):
                if not masks[delta + 1]:
                    continue
                alternative = _following_target(speed, limit, _neighbor(obs, start, speed))
                if alternative > target + 1.5:
                    choices.append((alternative, delta))
            if choices:
                _, lane_delta = max(choices)
                # The own-lane leader still constrains the command until the
                # lane change has really occurred; no anticipated free space.

    acceleration = 0.8 * (target - speed)
    acceleration = min(acceleration, acc_response_ceiling(speed, min(SPEED_CAP, float(obs[4]) * SPEED_CAP), own_front))
    # Ending-lane braking is derived from the same declared distance and floor.
    if requires_merge:
        distance = max(0.0, float(obs[27]) * 240.0)
        stop_speed = math.sqrt(9.0 * max(0.0, distance - MIN_GAP))
        acceleration = min(acceleration, (stop_speed - speed) / DT)
    ceiling = float(np.clip(obs[43] * 4.5, ACCELERATION_MIN, ACCELERATION_MAX))
    acceleration = float(np.clip(min(acceleration, ceiling - 1e-5), ACCELERATION_MIN, ACCELERATION_MAX))
    normalized = (acceleration - ACCELERATION_MIN) / 3.25 - 1.0
    return np.array([normalized, lane_delta], dtype=np.float32)


def discrete_stop_speed(distance: float, deceleration: float = 4.5) -> float:
    """Current-distance Euler stopping-speed estimate, dt/headway0.1 s.

    Port of the algebra in SUMO1.27.1 MSCFModel::maximumSafeStopSpeedEuler;
    no native API, vehicle state mutation, future trajectory, or fitted labels.
    The1mm distance subtraction is native NUMERICAL_EPS, not fitted to data.
    """
    gap = max(0., float(distance)-1e-3)
    b, s, t = deceleration*DT, DT, DT
    n = math.floor(.5-(t-.5*math.sqrt(s*s+4*(s*(2*gap/b-t)+t*t)))/s)
    h = .5*n*(n-1)*b*s+n*b*t
    return n*b+(gap-h)/(n*s+t)


def longitudinal_target_v3(observation) -> float:
    """Single actor-visible acceleration target shared by teacher and reward.

    Inputs use the opt-in v3 entry/auxiliary context, not historical v2 context.
    A zero projected lateral delta with known context describes a minor entry;
    internal-ramp context is unknown until an actual adjacent lane is available.
    """
    obs = np.asarray(observation, dtype=np.float64)
    baseline = _teacher_action_v2(obs)  # also validates the existing93-field box
    acceleration = ACCELERATION_MIN+(float(baseline[0])+1)*3.25
    speed = float(obs[0])*SPEED_CAP
    limit = min(SPEED_CAP, float(obs[4])*SPEED_CAP)
    # Missing a <=120m neighbor does not imply no native ACC leader (250m range).
    acceleration = min(acceleration, .4*(limit-SPEED_BUFFER-speed))
    if obs[28] > .5 and obs[33] > .5 and abs(obs[35]) < .5:
        distance = max(0., float(obs[36])*240.)
        front = _neighbor(obs, 37, speed)
        rear = _neighbor(obs, 40, speed)
        # All following/entry gaps keep the existing1.5s/stopping-distance floor.
        front_safe = front is None or front[0] >= required_gap(speed, front[1])
        rear_safe = rear is None or rear[0] >= required_gap(rear[1], speed)
        if distance > 4.5 or not (front_safe and rear_safe):
            acceleration = min(acceleration, (discrete_stop_speed(distance-MINOR_STOP_OFFSET_M)-speed)/DT)
    # Taper before a disappearing lane instead of demanding full speed until
    # the analytical hard bound engages. This is a target, not a looser guard.
    if obs[28] > .5:
        end_distance = max(0., float(obs[27])*240.)
        end_target = math.sqrt(2*2.*max(0., end_distance-MIN_GAP))
        acceleration = min(acceleration, .8*(end_target-speed))
    ceiling = float(obs[43])*4.5
    return float(np.clip(min(acceleration, ceiling-1e-5), ACCELERATION_MIN, ACCELERATION_MAX))


def teacher_action(observation, *, version='v2') -> np.ndarray:
    if version == 'v2':
        return _teacher_action_v2(observation)
    if version != 'v3':
        raise ValueError('Unknown teacher target version')
    action = _teacher_action_v2(observation)
    action[0] = (longitudinal_target_v3(observation)-ACCELERATION_MIN)/3.25-1.
    return action


def teacher_actions(observations: Mapping[str, np.ndarray], *, version='v2') -> dict[str, np.ndarray]:
    """Compute simultaneous actions without mutating observations or ID state."""
    return {agent: teacher_action(observation, version=version) for agent, observation in observations.items()}
