"""V14 drive-aware merge target (round 22): the reward target starts at the car's speed and rises
no faster than the car's drive allows. The gate keeps the v14 target.

User decision, November 22, 2026: "Yes, raise it" (round 22 and a physics check, after round 21).
The standing autonomy decision of November 17 covers the round's reward settings.

Measured in round 21's final validation (outputs/highway-flow-rl/v14/round-21, action journals):
- At a yield entry v14's reachable target (traffic.highway_flow_rl_v14_rewards.reachable) starts
  at min(plan, speed + 1.5 m/s^2 x 3 s), i.e. up to 4.5 m/s above the car, and then rises at
  1.5 m/s^2 from the larger of its previous value and the car's speed.
- With the drive limit on, the car can give 1.0-1.4 m/s^2 on the southbound LA ramp and up to
  2.0 m/s^2 on the northbound one (the environment's comfort cap).
- The gate's speed match needs the car within 2 m/s of that target for 90% of the 16 s window
  around its lateral merge. A car that starts 4.5 m/s below a rising target closes the gap at
  (a - 1.5) m/s^2 at best: at the 2.0 m/s^2 cap it is outside the 2 m/s band for at least the
  first 5 s, and at 1.0-1.4 m/s^2 the gap grows until the target stops rising.
- Case 8's agent_6 (window from 0.2 s, 124 samples) and case 9's agent_5 (from 0.2 s, 142
  samples) start 4.5 m/s below their targets, and their logged targets never fall back to the
  car's speed, so they do not depend on how the car drives. A car accelerating at the 2.0 m/s^2
  cap from the first decision would match at most 63% and 79% of those windows; at the logged
  drive ceilings, about 56% and 37%. The gate needs 90% of the window for every one of these
  cases' merging cars, so cases 8 and 9 cannot pass rolling merge quality at these seeds
  whatever the policy does.
- Those cars are not only limited by the drive. In round 21's final validation they asked for
  0.1-0.7 m/s^2 on the southbound ramp against a 1.3 m/s^2 ceiling, and about 1.5 against 2.0
  on the northbound ramp.

The change (yield-entry targets only; every other merge keeps v12's target exactly):
- the reward target starts at min(plan, speed + initial_lead_m_s), with no lead: a car exactly
  at its target that accelerates as fast as its drive allows stays at its target;
- it rises at min(1.5 m/s^2, the drive ceiling at the car's current lane position and speed,
  the same traffic.road_grade bound the applied command gets), from the larger of its previous
  value and the car's speed, so a car slower than its drive allows falls behind it, as before;
- it falls at v14's 2.0 m/s^2, and the occupied-lane/map ceiling still applies at once.
It feeds everything v14's target fed except the gate: the four actor target inputs, the band,
tracking and speed-error rewards, and the stop fix's merge_braking and band tail. The gate's
speed match (rolling merge quality) and the free-road stop measurement keep the v14 target, so
gate counts stay comparable with rounds 20 and 21. The environment also reports, as a
diagnostic, each merging car's rolling merge quality graded on this target.
"""
import math

REWARD_VERSION = 'highway_flow_rl_merge_v14_drive_target'
SETTINGS = dict(
    initial_lead_m_s=0.,          # v14: 1.5 m/s^2 x 3 s = 4.5
    rise_m_s2=1.5,                # v14's target_acceleration_m_s2, now also capped by the drive ceiling
    fall_m_s2=2.,                 # v14's target_deceleration_m_s2
    rise_floor_m_s2=0.)           # a drive ceiling below zero does not pull the target down


def validate_settings(settings):
    if dict(settings) != SETTINGS:
        raise ValueError('Unreviewed v14 drive-target settings')
    return dict(settings)


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def rise_rate(drive_ceiling_m_s2, settings=SETTINGS):
    """The target's rise rate: v14's 1.5 m/s^2, capped by the drive ceiling (None: no drive limit)."""
    if drive_ceiling_m_s2 is None:
        return settings['rise_m_s2']
    if not finite(drive_ceiling_m_s2):
        raise ValueError('The drive ceiling must be finite')
    return max(settings['rise_floor_m_s2'], min(settings['rise_m_s2'], drive_ceiling_m_s2))


def reachable(desired, ceiling, speed, previous, now, rise_m_s2, settings=SETTINGS):
    """v14.reachable with the initial lead and the rise rate as above.

    previous: dict(time=..., value=...) of the last drive-aware target of this car, or None at
    the start of its ramp.
    """
    if not all(finite(v) for v in (desired, speed, now, rise_m_s2)):
        raise ValueError('Drive-aware target needs finite values')
    if previous is None:
        bounded = min(desired, speed + settings['initial_lead_m_s'])
    else:
        old, dt = previous['value'], now - previous['time']
        if not finite(old) or not finite(dt) or dt < -1e-9:
            raise ValueError('Drive-aware target needs an earlier finite target')
        bounded = max(old - settings['fall_m_s2'] * dt, min(max(old, speed) + rise_m_s2 * dt, desired))
    return min(ceiling, bounded) if ceiling is not None else bounded
