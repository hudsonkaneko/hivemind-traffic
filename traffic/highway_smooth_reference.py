"""Observation-only comfort reference for the unchanged v05 actor interface.

This implements the two-second entry preview and 2 m/s^3 recovery reference,
with version-4 separation of ordinary-response smoothing from hard constraints. It changes
labels/rewards only unless explicitly selected as a
scripted diagnostic controller. It is not a native actuator filter, a complete
SUMO feasibility model, or evidence that the native preflight gates pass.

Implementation correction to the proposal: an acceleration cannot reduce the
next speed below zero. Native response estimates and the final reference use
the physical floor max(-4.5, -speed / 0.1). This preserves stopping priority;
it prevents unreachable negative-speed labels at standstill.
"""
from __future__ import annotations

from collections.abc import Mapping
import math

import numpy as np

from traffic import highway_stage1_teacher as legacy


REFERENCE_VERSION = 'highway_smooth_reference_v4'
REFERENCE_STATUS = 'implemented_requires_native_preflight'
OBSERVATION_SIZE = 99
DT = .1
ENTRY_PREVIEW_TIME_S = 2.
REFERENCE_JERK_M_S3 = 2.
DESIRED_SPEED_GAIN_S_INVERSE = .4
ENTRY_VISIBILITY_M = 4.5
ENTRY_STOP_OFFSET_M = .1
LEGAL_SPEED_BUFFER_M_S = .1
STOPPING_DECELERATION_M_S2 = 2.
MERGE_SLOT_SPEED_SEPARATION_M_S = 1.
IMPLEMENTATION_NOTES = (
    'v1 native preflight failed: asymptotic entry tracking caused late full braking and excess recovery slowdown.',
    'v2 adds finite-distance entry braking and the spatial derivative of the comfortable ending-lane stop profile.',
    'v2 native preflight met peak comfort but paced an occupied merge slot until the ending lane forced a stop.',
    'v3 yields behind a closing projected rear car when its gap deficit exceeds current projected-front surplus; the 1 m/s relative-speed target reuses the legacy catch-up separation magnitude.',
    'v4 treats ordinary ACC response as a soft target before slew; unchanged hard and minor-entry stop bounds retain immediate priority afterward.',
    'Raw ordinary estimates and targets exceeding them remain reported; SUMO interventions are not hidden or reclassified.',
    'Matrix01 also recorded mandatory following-bound braking and native braking beyond the estimate; this ordinary-response revision does not establish that those comfort failures are resolved.',
    'The stopping profile joins constant 2 m/s^2 braking to 2 m/s^3 recovery; a discrete soft recovery floor avoids a terminal acceleration jump.',
    'Preview 2 s, speed-error gain .4/s, slew 2 m/s^3, visibility 4.5 m and all hard/native guards remain unchanged.',
    'Physical velocity floor prevents negative next-speed labels, including at standstill.',
    'Native response estimates are conservative labels, not unsafe-request classifications.',
    'Requires native safety, comfort, slowdown and travel-time preflight before learning.',
)


def _observation(value):
    obs = np.asarray(value, dtype=np.float64)
    if obs.shape != (OBSERVATION_SIZE,) or not np.isfinite(obs).all():
        raise ValueError('Smooth reference requires one finite 99-field observation')
    if np.any(obs < -1.000001) or np.any(obs > 1.000001):
        raise ValueError('Smooth reference observation is outside the normalized contract')
    return obs


def _legacy_observation(obs):
    # Preserve every existing local/message field while removing only the six
    # v05 motion fields. No actor schema or original teacher implementation changes.
    return np.concatenate((obs[:45], obs[51:]))


def _stopping_profile(distance, speed):
    """Map-distance speed profile and its time derivative at current speed.

    Constant comfortable deceleration b joins a positive-jerk recovery to zero
    acceleration. Integrating the recovery gives distance b^3/(6*j^2) and
    initial speed b^2/(2*j). No future traffic or native state is consulted.
    """
    distance = max(0., distance)
    b, jerk = STOPPING_DECELERATION_M_S2, REFERENCE_JERK_M_S3
    if distance >= b**3/(6.*jerk**2):
        target = math.sqrt(2.*b*(distance-b**3/(24.*jerk**2)))
        derivative = b/target
    elif distance > 0.:
        target = (4.5*jerk)**(1./3.)*distance**(2./3.)
        derivative = (2./3.)*target/distance
    else:
        return 0., legacy.ACCELERATION_MIN if speed > 0. else 0.
    return target, -derivative*speed


def _braking_recovery_floor(speed):
    """Soft braking floor that can release at the nominal discrete jerk.

    The speed consumed by n braking steps is dt*(n*b-j*dt*n*(n-1)/2).
    Inverting that sum avoids a negative-speed request at the last step. This
    is a comfort term: both the original hard bound and native estimate can
    demand stronger braking immediately.
    """
    increment = REFERENCE_JERK_M_S3*DT
    steps = max(1, math.ceil((math.sqrt(1.+8.*speed/(DT*increment))-1.)/2.))
    return -(speed/(steps*DT)+increment*(steps-1)/2.)


def reference_details(observation) -> dict:
    """Return acceleration and separately auditable soft/hard/estimated terms.

    Only current actor fields are read. There is no retained vehicle history,
    simulator connection, native getter, future trajectory, or evaluator input.
    The measured acceleration field is already bounded by the existing ABI.
    """
    obs = _observation(observation)
    speed = max(0., float(obs[0]) * legacy.SPEED_CAP)
    measured = float(obs[1]) * 4.5
    legal_limit = max(0., min(legacy.SPEED_CAP, float(obs[4]) * legacy.SPEED_CAP))
    buffered_limit = max(0., legal_limit - LEGAL_SPEED_BUFFER_M_S)
    hard_ceiling = float(np.clip(obs[43] * 4.5, legacy.ACCELERATION_MIN, legacy.ACCELERATION_MAX))
    physical_floor = max(legacy.ACCELERATION_MIN, -speed / DT)
    own_front = legacy._neighbor(obs, 6, speed)
    requires_merge = obs[28] > .5
    known_merge = bool(requires_merge and obs[33] > .5)
    minor_entry = bool(known_merge and abs(obs[35]) < .5)

    components = dict(legal_buffer=buffered_limit,
        own_following=legacy._following_target(speed, buffered_limit, own_front))
    native_components = dict(
        # A missing <=120 m observation does not prove that SUMO sees no leader
        # in its longer interaction range. This estimate remains deliberately
        # conservative and does not query its hidden ACC mode.
        cruise_response=DESIRED_SPEED_GAIN_S_INVERSE * (legal_limit - speed),
        own_following=legacy.acc_response_ceiling(speed, legal_limit, own_front))
    front = rear = None
    front_safe = rear_safe = None
    slot = dict(active=False, front_surplus_m=None, rear_deficit_m=None,
        rear_closing_speed_m_s=None, speed_target_m_s=None,
        speed_separation_m_s=MERGE_SLOT_SPEED_SEPARATION_M_S,
        reason='No constrained lateral-merge slot requiring yield')
    if known_merge:
        front = legacy._neighbor(obs, 37, speed)
        rear = legacy._neighbor(obs, 40, speed)
        target_limit = max(0., min(legacy.SPEED_CAP, float(obs[44]) * legacy.SPEED_CAP)
                           - LEGAL_SPEED_BUFFER_M_S)
        components['target_lane_following'] = legacy._following_target(
            speed, min(buffered_limit, target_limit), front)
        front_safe = front is None or front[0] >= legacy.required_gap(speed, front[1])
        rear_safe = rear is None or rear[0] >= legacy.required_gap(rear[1], speed)
        if abs(obs[35]) > .5 and front is not None and rear is not None:
            surplus = max(0., front[0]-legacy.required_gap(speed,front[1])-legacy.FOLLOW_BUFFER)
            deficit = max(0., legacy.required_gap(rear[1],speed)+legacy.FOLLOW_BUFFER-rear[0])
            closing = rear[1]-speed
            slot.update(front_surplus_m=surplus, rear_deficit_m=deficit,
                rear_closing_speed_m_s=closing)
            if closing > 0. and deficit > surplus:
                # Pacing the front car while another car closes alongside does
                # not create a usable gap. Hold speed (or yield gently below
                # the rear car's speed) until it passes. Re-evaluate from the
                # next observation; no retained identity, gap prediction or
                # permission to change lanes is introduced.
                slot_speed = min(speed,max(0.,rear[1]-MERGE_SLOT_SPEED_SEPARATION_M_S))
                components['merge_slot_yield'] = slot_speed
                slot.update(active=True, speed_target_m_s=slot_speed,
                    reason='Closing rear gap deficit exceeds available front surplus; yield behind')
    distance_accelerations = {}
    ending = dict(active=bool(requires_merge), remaining_distance_m=None,
        stopping_gap_m=None, speed_m_s=None, feed_forward_m_s2=None)
    if requires_merge:
        end_distance = max(0., float(obs[27]) * 240.)
        stop_gap = max(0., end_distance-legacy.MIN_GAP)
        end_speed, end_feed_forward = _stopping_profile(stop_gap, speed)
        components['ending_lane'] = end_speed
        distance_accelerations['ending_lane'] = (
            end_feed_forward+DESIRED_SPEED_GAIN_S_INVERSE*(end_speed-speed))
        ending.update(remaining_distance_m=end_distance, stopping_gap_m=stop_gap,
            speed_m_s=end_speed, feed_forward_m_s2=end_feed_forward)

    entry = dict(active=minor_entry, distance_m=None, projected_front_safe=front_safe,
        projected_rear_safe=rear_safe, preview_floor_m=None, preview_distance_m=None,
        preview_speed_m_s=None, native_stop_estimate_active=False,
        visibility_m=ENTRY_VISIBILITY_M, stop_offset_m=ENTRY_STOP_OFFSET_M,
        terminal_speed_m_s=None, terminal_distance_m=None,
        distance_braking_m_s2=None)
    if minor_entry:
        distance = max(0., float(obs[36]) * 240.)
        gaps_safe = bool(front_safe and rear_safe)
        floor = ENTRY_VISIBILITY_M if gaps_safe else ENTRY_STOP_OFFSET_M
        preview_distance = max(distance - ENTRY_PREVIEW_TIME_S * speed, floor)
        preview_speed = legacy.discrete_stop_speed(max(0., preview_distance-ENTRY_STOP_OFFSET_M))
        components['entry_preview'] = preview_speed
        terminal_speed = legacy.discrete_stop_speed(ENTRY_VISIBILITY_M-ENTRY_STOP_OFFSET_M) if gaps_safe else 0.
        remaining = distance-floor
        if remaining > 0.:
            if gaps_safe and speed > terminal_speed:
                # Constant-acceleration kinematics supplies the missing finite
                # distance term; the speed-error term alone converges only
                # asymptotically and reached the native boundary too fast.
                distance_accelerations['entry_distance'] = (terminal_speed**2-speed**2)/(2.*remaining)
            elif not gaps_safe:
                stop_speed, stop_feed_forward = _stopping_profile(remaining, speed)
                distance_accelerations['entry_distance'] = (
                    stop_feed_forward+DESIRED_SPEED_GAIN_S_INVERSE*(stop_speed-speed))
        stop_active = distance > ENTRY_VISIBILITY_M or not gaps_safe
        if stop_active:
            native_components['minor_entry_stop'] = (
                legacy.discrete_stop_speed(max(0., distance-ENTRY_STOP_OFFSET_M))-speed) / DT
        entry.update(distance_m=distance, preview_floor_m=floor,
            preview_distance_m=preview_distance, preview_speed_m_s=preview_speed,
            native_stop_estimate_active=stop_active,
            terminal_speed_m_s=terminal_speed, terminal_distance_m=floor,
            distance_braking_m_s2=distance_accelerations.get('entry_distance'))

    desired_speed = min(components.values())
    acceleration_components = dict(speed_error=DESIRED_SPEED_GAIN_S_INVERSE*(desired_speed-speed),
        **distance_accelerations)
    raw_ordinary_estimate = min(native_components['cruise_response'],native_components['own_following'])
    acceleration_components['ordinary_native_response'] = raw_ordinary_estimate
    raw_desired = min(acceleration_components.values())
    recovery_floor = _braking_recovery_floor(speed)
    desired = float(np.clip(max(recovery_floor, raw_desired),
                            legacy.ACCELERATION_MIN, legacy.ACCELERATION_MAX))
    slew_low, slew_high = measured-REFERENCE_JERK_M_S3*DT, measured+REFERENCE_JERK_M_S3*DT
    ramped = float(np.clip(desired, slew_low, slew_high))
    raw_native_estimate = min(native_components.values())
    native_estimate = float(np.clip(raw_native_estimate, physical_floor, legacy.ACCELERATION_MAX))
    ordinary_estimate = float(np.clip(raw_ordinary_estimate, physical_floor, legacy.ACCELERATION_MAX))
    mandatory_estimate = float(np.clip(native_components.get('minor_entry_stop',legacy.ACCELERATION_MAX),
        physical_floor,legacy.ACCELERATION_MAX))
    raw_ceiling = min(hard_ceiling, mandatory_estimate)
    ceiling = max(physical_floor, raw_ceiling)
    raw_reference = min(ramped, ceiling)
    reference = float(np.clip(raw_reference, physical_floor, legacy.ACCELERATION_MAX))
    return dict(version=REFERENCE_VERSION, status=REFERENCE_STATUS,
        acceleration_m_s2=reference, speed_m_s=speed, measured_acceleration_m_s2=measured,
        legal_limit_m_s=legal_limit, desired_speed_m_s=desired_speed,
        desired_acceleration_m_s2=desired, ramped_acceleration_m_s2=ramped,
        hard_ceiling_m_s2=hard_ceiling, native_response_estimate_m_s2=native_estimate,
        raw_native_response_estimate_m_s2=raw_native_estimate,
        ordinary_native_response_estimate_m_s2=ordinary_estimate,
        mandatory_native_response_estimate_m_s2=mandatory_estimate,
        reference_exceeds_ordinary_response_estimate=reference > ordinary_estimate+1e-9,
        reference_ceiling_m_s2=ceiling, achievable_acceleration_floor_m_s2=physical_floor,
        soft_braking_recovery_floor_m_s2=recovery_floor,
        soft_braking_recovery_floor_applied=raw_desired < recovery_floor-1e-9,
        desired_acceleration_components_m_s2=acceleration_components,
        soft_speed_components_m_s=components, native_response_components_m_s2=native_components,
        entry=entry, ending_lane=ending, merge_slot_alignment=slot,
        nominal_reference_jerk_m_s3=REFERENCE_JERK_M_S3,
        reference_slew_m_s3=(reference-measured)/DT,
        braking_override_soft_slew=reference < slew_low-1e-9,
        reference_ceiling_limited=raw_reference < ramped-1e-9,
        native_estimate_velocity_floor_applied=raw_native_estimate < physical_floor-1e-9,
        hard_ceiling_velocity_floor_applied=hard_ceiling < physical_floor-1e-9,
        reference_velocity_floor_applied=raw_reference < physical_floor-1e-9,
        label_role='Reference/supervision only; ordinary response estimate is not an unsafe-request label',
        native_authority='SUMO; this estimate does not guarantee feasibility or realized jerk')


def smooth_reference(observation) -> dict:
    """Alias for the diagnostic reference interface."""
    return reference_details(observation)


def smooth_acceleration_target(observation) -> float:
    return reference_details(observation)['acceleration_m_s2']


def teacher_action(observation) -> np.ndarray:
    """Use the new acceleration label with the exact existing v3 lane label."""
    obs = _observation(observation)
    action = legacy.teacher_action(_legacy_observation(obs), version='v3')
    acceleration = smooth_acceleration_target(obs)
    action[0] = np.clip((acceleration-legacy.ACCELERATION_MIN)/3.25-1., -1., 1.)
    return action


def teacher_actions(observations: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {identity:teacher_action(observation) for identity,observation in observations.items()}
