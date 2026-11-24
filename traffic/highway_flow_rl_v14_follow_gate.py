"""V14 followable merge target (round 23): a gate reference that a car can follow, graded as a band.

User decision, November 24, 2026: "yes redesign" (redesign the merge-quality gate's target so a car
can follow it, check it with a scripted follower, then train round 23 within the remaining budget).
The standing autonomy decision of November 17 covers the round's reward settings.

Measured after round 22 (outputs/test-runs/v14-follower-check-*, documentation/highway-merge-
training-plan-v14.md, "After round 22: troubleshooting"): a scripted car that follows v14's gate
target matches it in only 15-66% of the window in all four LA merge cases (0.9 needed), because at
yield entries the target
- starts at the plan's speed at once, up to 6 m/s below the car (case 11: 5.0 m/s, the car 11.05);
- turns from falling to rising in one decision while the car is still braking at up to 4 m/s^2;
- after the line, or after a one-decision gap in eligibility, returns to v12's road target, which
  jumps toward the speed limit (case 10: 12.9 to 25.5 m/s in 0.3 s).

The reference, per car, from its first yield-entry decision:
- starts at the car's speed and measured acceleration (the occupied-lane/map ceiling applies at once);
- moves toward the plan/road target with its acceleration in [-fall_m_s2, min(rise_m_s2, the drive
  ceiling)] and its acceleration changing at most jerk_m_s3 (the car's comfort rate limit is
  0.4 m/s^2 per 0.1 s decision, 4 m/s^3), approaching the target without overshoot;
- when the road target comes back to or above it after being below (a yield or a slower leader ends)
  while the car is slower, restarts from the car's speed and acceleration;
- is kept through the whole merge window, and ends once the car's merge quality has been graded and
  the reference is within end_tolerance_m_s of the road target.

The gate's speed match: the car's speed lies within tolerance_m_s of the span between the reference
and the road target (v04: within 2 m/s of the target). A car slower than the reference, or faster
than the road target, by more than 2 m/s does not match.

The reward target and the actor's target inputs: the reference, capped at once by the road target's
stopping bounds (an occupied-lane leader or an ending-lane runway), so it never rewards closing on a
leader faster than its bound allows. It always lies inside the gate's span.
Cars without a yield entry keep v12's target for both, exactly as before.
"""
import math

REWARD_VERSION = 'highway_flow_rl_merge_v14_follow_gate'
SETTINGS = dict(
    rise_m_s2=1.5,                # v14's target_acceleration_m_s2, capped by the drive ceiling as in round 22
    fall_m_s2=2.,                 # v14's target_deceleration_m_s2
    jerk_m_s3=2.5,                # below the car's 4 m/s^3 comfort rate limit, so a car can follow every turn
    tolerance_m_s=2.,             # v04's speed-match tolerance
    end_tolerance_m_s=.1,         # the reference ends within this of the road target, after grading
    reward_caps=('occupied_leader', 'ending_lane_runway'))   # safe_speed_target bounds applied at once


def validate_settings(settings):
    settings = dict(settings)
    if 'reward_caps' in settings:
        settings['reward_caps'] = tuple(settings['reward_caps'])
    if settings != SETTINGS:
        raise ValueError('Unreviewed v14 follow-gate settings')
    return settings


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def clip(value, low, high):
    return max(low, min(high, value))


def rise_rate(drive_ceiling_m_s2, settings=SETTINGS):
    """The reference's largest acceleration: rise_m_s2, capped by the drive ceiling (None: no drive limit)."""
    if drive_ceiling_m_s2 is None:
        return settings['rise_m_s2']
    if not finite(drive_ceiling_m_s2):
        raise ValueError('The drive ceiling must be finite')
    return max(0., min(settings['rise_m_s2'], drive_ceiling_m_s2))


def start(speed, acceleration, ceiling, rise, now, settings=SETTINGS):
    """The reference at a car's first yield-entry decision: its speed and acceleration."""
    if not all(finite(v) for v in (speed, acceleration, rise, now)) or speed < 0:
        raise ValueError('The follow reference needs a finite speed, acceleration, rise rate and time')
    value, slope = speed, clip(acceleration, -settings['fall_m_s2'], rise)
    if ceiling is not None and value > ceiling:
        value, slope = max(0., ceiling), min(slope, 0.)
    return dict(time=now, value=value, slope=slope, below=False)


def _move(value, slope, desired, rise, dt, settings):
    """One jerk-limited step toward `desired`: the largest slope that steps of jerk x dt can still bring to
    zero at the target (|error| = a^2 / 2j + a dt / 2)."""
    error = desired - value
    jerk = settings['jerk_m_s3']
    half = .5 * jerk * dt
    wanted = math.sqrt(half * half + 2. * jerk * abs(error)) - half
    wanted = clip(math.copysign(wanted, error), -settings['fall_m_s2'], rise)
    old = slope
    slope = old + clip(wanted - old, -jerk * dt, jerk * dt)
    moved = value + slope * dt
    if (error >= 0. and moved > desired and slope > 0.) or (error <= 0. and moved < desired and slope < 0.):
        # At the target: the value stays there while the slope runs down at the jerk limit.
        moved, slope = desired, old - clip(old, -jerk * dt, jerk * dt)
    return max(0., moved), slope


def advance(reference, desired, ceiling, rise, speed, acceleration, now, settings=SETTINGS):
    """The reference at the next decision, and whether it restarted from the car's speed.

    A restart happens when the road target is back at or above the moved reference after being below it at
    the previous decision, while the car is slower than the moved reference.
    """
    if not all(finite(v) for v in (desired, rise, speed, acceleration, now)):
        raise ValueError('The follow reference needs finite values')
    value, slope, dt = reference['value'], reference['slope'], now - reference['time']
    if not finite(value) or not finite(slope) or not finite(dt) or dt < -1e-9:
        raise ValueError('The follow reference needs an earlier finite reference')
    if dt > 1e-9:
        value, slope = _move(value, slope, desired, rise, dt, settings)
    if ceiling is not None and value > ceiling:
        value, slope = max(0., ceiling), min(slope, 0.)
    below = desired < value - 1e-9
    restarted = bool(reference.get('below') and not below and speed < value - 1e-9)
    if restarted:
        value, slope = speed, clip(acceleration, -settings['fall_m_s2'], rise)
        below = desired < value - 1e-9
    return dict(time=now, value=value, slope=slope, below=below), restarted


def band(reference_m_s, desired_m_s):
    """The span the gate accepts (before its tolerance): between the reference and the road target."""
    return min(reference_m_s, desired_m_s), max(reference_m_s, desired_m_s)


def acceptance(speed, lower, upper):
    """The point of the span nearest the car's speed: v04's |speed - target| <= tolerance on it is the band check."""
    return clip(speed, lower, upper)


def matches(speed, lower, upper, settings=SETTINGS):
    return lower - settings['tolerance_m_s'] <= speed <= upper + settings['tolerance_m_s']


def reward_cap(target, settings=SETTINGS):
    """The lowest stopping bound of a safe_speed_target audit (occupied-lane leader, ending-lane runway), or None."""
    values = [b['ceiling_m_s'] for b in (target or {}).get('bounds') or ()
              if b.get('reason') in settings['reward_caps'] and finite(b.get('ceiling_m_s'))]
    return min(values) if values else None
