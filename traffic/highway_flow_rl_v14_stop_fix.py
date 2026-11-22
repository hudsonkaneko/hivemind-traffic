"""V14 stop fix (round 21): a cost on braking against the merge target, and a band tail.

User decision, November 21, 2026: "Yes, build and train" (the design reported after "why did
interventions go up by so much" and "Yes, design a fix").

Round 20's measured cause (its saved SUMO validation traces, re-run through its saved actors):
- each acceleration request is mostly the car's previous request and acceleration (actor inputs 1
  and 5; loop slope 0.5-1.0) plus a small correction from what the car sees. A small negative shift
  in that correction becomes steady braking: on the southbound LA ramp the round-16 actor settles
  at +1.13 m/s^2 from a neutral start, round 20's only at about -4. Cars braked to a stop on open
  road while their merge target was 14-20 m/s;
- v06's avoidable-braking cost never applied there: traffic.highway_flow_rl_v03_rewards'
  BrakingDiagnostic requires a target with no obligations, and every pending merge carries
  'merge_or_yield' (acceleration lanes also 'ending_lane');
- once a car is 9 m/s below its target, v12's band cost is capped at 4 per decision, so crawling
  and standing still cost the same in that term.

Two reward terms, both off unless an environment is built with settings=SETTINGS:
1. merge_braking: v06's avoidable_braking formula and thresholds (request below -0.2 m/s^2, cost
   weight * min(1, depth / 2 m/s^2), weight 0.25; target at least 0.5 m/s above the car's speed; a
   safety ceiling of at least 0.2 m/s^2; no own-lane leader within 1.5 s) for the decisions v06
   leaves out: a pending merge whose target carries only 'merge_or_yield' and/or 'ending_lane'.
   The merge target already slows the car for each of them (merge gaps, the ending-lane runway,
   the unseen-yield stop and, at yield entries, v14's crossing check), so a target above the car's
   speed says the plan does not need it to slow down. A yield-entry plan with a crossing deficit
   is exempt. Decisions with no obligations stay v06's, so no decision is charged twice.
2. target_band_tail: past v12's cap (9 m/s off target), the band keeps a slope of a quarter of its
   in-band slope: -weight * tail * max(0, excess - cap), on the same target and post-decision speed
   as target_band. A stopped car with a 20.1 m/s target costs 4 + 1.54 per decision instead of 4.

Measurements only (every v14 environment): stopped decisions (speed before the decision below
0.1 m/s), how many of them ask to brake, and free-road stops: stopped with no own-lane leader
within 20 m and an eligible target of at least 5 m/s. Round 20 validations had 0, 0, 25, 64 and
180 free-road stopped decisions at the start, at updates 31, 56 and 91, and at the end.
"""
import math

from traffic import highway_flow_rl_v06_rewards as v06

REWARD_VERSION = 'highway_flow_rl_merge_v14_merge_braking'
SETTINGS = dict(
    weight=v06.WEIGHTS['avoidable_braking'],                        # 0.25
    braking_request_m_s2=v06.SETTINGS['braking_request_m_s2'],      # -0.2
    braking_scale_m_s2=v06.SETTINGS['braking_scale_m_s2'],          # 2.0
    free_headway_s=v06.SETTINGS['free_headway_s'],                  # 1.5
    target_margin_m_s=.5,                                           # BrakingDiagnostic's target >= speed + 0.5
    ceiling_floor_m_s2=.2,                                          # BrakingDiagnostic's safe ceiling >= 0.2
    obligations=('ending_lane', 'merge_or_yield'),
    band_tail_fraction=.25)
STOP_SETTINGS = dict(stopped_speed_m_s=.1, free_gap_m=20., min_target_m_s=5.)


def validate_settings(settings):
    settings = dict(settings)
    settings['obligations'] = tuple(settings.get('obligations', ()))
    if settings != SETTINGS:
        raise ValueError('Unreviewed v14 stop-fix settings')
    return settings


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def merge_braking_eligible(target, speed, ceiling, plan, merge_active, settings=SETTINGS):
    """A pending merge whose target needs no slowing: the decisions v06's avoidable braking leaves out."""
    if not merge_active or not target or not target.get('eligible') or not finite(speed):
        return False
    obligations = set(target.get('obligations') or ())
    if not obligations or not obligations <= set(settings['obligations']):
        return False
    value = target.get('target_m_s')
    if not finite(value) or value < speed + settings['target_margin_m_s']:
        return False
    if not finite(ceiling) or ceiling < settings['ceiling_floor_m_s2']:
        return False
    return not (plan or {}).get('crossing_deficit') or plan['crossing_deficit'] <= 1e-9


def free_road(speed, front, settings=SETTINGS):
    """v06.free_road with these settings: no own-lane leader inside the free headway."""
    if front is None:
        return True
    gap = front.get('bumper_gap_m')
    return finite(gap) and gap >= settings['free_headway_s'] * max(speed, 0.)


def merge_braking(target, speed, requested, ceiling, front, plan, merge_active, settings=SETTINGS):
    """Cost on the actor's own braking request when the merge target says not to slow down."""
    if not finite(requested) or requested >= settings['braking_request_m_s2']:
        return 0.
    if not merge_braking_eligible(target, speed, ceiling, plan, merge_active, settings):
        return 0.
    if not free_road(speed, front, settings):
        return 0.
    depth = (settings['braking_request_m_s2'] - requested) / settings['braking_scale_m_s2']
    return -settings['weight'] * min(1., depth)


def target_band_tail(target, next_speed, merge_active, v12_settings, settings=SETTINGS):
    """The band's cost beyond v12's cap, at band_tail_fraction of the in-band slope."""
    from traffic.highway_flow_rl_v12_rewards import active_target
    value = active_target(target, merge_active)
    if value is None or not finite(next_speed):
        return 0.
    tolerance = v12_settings['band_tolerance_m_s']
    excess = max(0., abs(next_speed - value) - tolerance) / tolerance
    beyond = max(0., excess - v12_settings['band_cap'])
    return -v12_settings['band_weight'] * settings['band_tail_fraction'] * beyond


def stop_sample(speed, requested, front, target, native_changed, settings=STOP_SETTINGS):
    """Measurement only: (stopped, stopped and asking to brake, its native override, free-road stop)."""
    stopped = finite(speed) and speed < settings['stopped_speed_m_s']
    if not stopped:
        return dict(stopped=0, asking_to_brake=0, asking_to_brake_overrides=0, free_road=0)
    braking = finite(requested) and requested < 0.
    gap = None if front is None else front.get('bumper_gap_m')
    clear = front is None or (finite(gap) and gap >= settings['free_gap_m'])
    value = (target or {}).get('target_m_s')
    free = bool(clear and (target or {}).get('eligible') and finite(value) and value >= settings['min_target_m_s'])
    return dict(stopped=1, asking_to_brake=int(braking), asking_to_brake_overrides=int(braking and bool(native_changed)),
                free_road=int(free))
