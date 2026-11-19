"""V12 merge objective: the gate's target speed as an actor input, plus a tolerance-band reward.

User request, November 18, 2026: "lets add the target input then add a reward with
decent weight to focus on matching that target speed. it should have some tolerance
around it tho so it could be between like 60-70 mph instead of only 65 etc."

Target: the same merge-speed target the rolling merge-quality gate grades against
(traffic/highway_flow_rl_v04_rewards.merge_speed_target), only while the car's merge
is pending and the target is eligible. Before v12 the actor never saw this number; it
had to infer it from gaps and speeds.

Inputs (4, inserted after the 69 local inputs, before the six message slots):
- target_active: 1 while the merge is pending and the target is eligible, else 0;
- target_speed: target / CAP (the speed input's own scale);
- target_error: (target - speed) / error_scale, clipped to [-1, 1];
- target_in_band: 1 when |speed - target| <= tolerance (and active), else 0.
All four are 0 when inactive and are faded like the v06 merge inputs, so a 0-to-1
flip takes `fade_steps` decisions.

Reward (target_band): zero inside +/- tolerance of the target; outside it a linear
cost of `weight` per tolerance width beyond the band, capped at `cap`:
    -weight * min(cap, max(0, |speed - target| - tolerance) / tolerance)
The tolerance is 1.8 m/s (4.0 mph), slightly inside the gate's own 2.0 m/s match
window, so a car resting at the band's edge still counts as matched. With weight 1
the cost is 1 per decision at 3.6 m/s off and saturates at 4 per decision (9 m/s
off); the inherited merge_tracking term saturates at 1.
"""
import math

import numpy as np

from traffic.highway_merge_context import CAP

REWARD_VERSION = 'highway_flow_rl_merge_v12_target_band'
OBSERVATION_VERSION = 'highway_flow_actor121_v1_target_inputs_faded'
TARGET_START = 69
FEATURES = ('target_active', 'target_speed', 'target_error', 'target_in_band')
TARGET_STOP = TARGET_START + len(FEATURES)
SETTINGS = dict(band_tolerance_m_s=1.8, band_weight=1.0, band_cap=4.0, error_scale_m_s=9.0, fade_steps=10)


def validate_settings(settings):
    settings = dict(settings)
    if settings != SETTINGS:
        raise ValueError('Unreviewed v12 target settings')
    return settings


def active_target(target, merge_active):
    """The gate's target speed (m/s) while it applies, else None."""
    if not merge_active or not target or not target.get('eligible'):
        return None
    value = target.get('target_m_s')
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def target_band(target, next_speed, merge_active, settings=SETTINGS):
    """Tolerance-band cost for the speed after this decision (0 inside the band)."""
    value = active_target(target, merge_active)
    if value is None or next_speed is None:
        return 0.
    tolerance = settings['band_tolerance_m_s']
    excess = max(0., abs(next_speed - value) - tolerance) / tolerance
    return -settings['band_weight'] * min(settings['band_cap'], excess)


def target_features(target, speed, merge_active, settings=SETTINGS):
    """The four raw (unfaded) target inputs for one car."""
    value = active_target(target, merge_active)
    if value is None or speed is None:
        return np.zeros(len(FEATURES), np.float32)
    error = value - speed
    return np.array([1., min(1., max(0., value / CAP)),
                     max(-1., min(1., error / settings['error_scale_m_s'])),
                     float(abs(error) <= settings['band_tolerance_m_s'])], np.float32)


def fade(previous, raw, settings=SETTINGS):
    """Move each target input toward its raw value by at most 1/fade_steps per decision."""
    raw = np.asarray(raw, np.float32)
    if previous is None:
        return raw.copy()
    rate = 1. / settings['fade_steps']
    previous = np.asarray(previous, np.float32)
    return previous + np.clip(raw - previous, -rate, rate)


def insert(observation, features):
    """117-field actor observation -> 121 fields with the target inputs at 69..72."""
    observation = np.asarray(observation, np.float32)
    if observation.shape != (117,):
        raise ValueError('V12 target inputs extend the 117-field actor observation')
    return np.concatenate((observation[:TARGET_START], np.asarray(features, np.float32),
                           observation[TARGET_START:])).astype(np.float32)
