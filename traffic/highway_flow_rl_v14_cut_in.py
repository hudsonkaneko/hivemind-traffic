"""V14 cut-in charge (round 24): a controlled car pays for the hard braking it forces behind it.

User decision, November 25, 2026: "All three" (round 24 tests multi-seed validation, a cut-in charge
and a decision on grade clipping; documentation/highway-merge-training-plan-v14.md, "After round 23:
diagnosis"). The standing autonomy decision of November 17 covers the charge's settings.

Measured after round 23 (outputs/test-runs/cutin-probe-01, cases 8-11 on 6 seeds each): every LA
native emergency whose leader could be traced was a background car braking behind one of our merging
cars on junction lane :n1_0_0, and no reward term saw it. Measured on those traces:
- the charge below fires on 11 of the 12 runs with a native emergency (19.8-81.2 m/s^2 per episode),
  and is at most 22.4 on the 41 runs without one (a plan-follower entry that made a car brake at
  5.8-8.4 m/s^2; a real hard brake);
- the 40 m gap limit removes the spawn transients (background cars braking at 4.5 m/s^2 in the first
  0.3 s with our car 55-127 m ahead); no charge falls in the first second;
- only background followers count. A controlled follower's braking is its own policy's choice; with
  controlled followers counted, our cars braking behind each other would add charges of 19.0-31.9 on
  three runs without an emergency, and the one emergency of a controlled car (case 9, seed 6951003)
  still gets no charge, since its leader was not one of our cars within 40 m;
- the follower is not required to be closing: SUMO's emergency braking goes on after the follower has
  slowed below our car (case 11: main_9 at -9 m/s^2, 2.8-4 m behind agent_0 and 0.8 m/s slower).

Per decision, after the simulation step: every background vehicle whose SUMO leader within
max_gap_m is one of our active cars and whose acceleration is below braking_m_s2 charges that car
weight * (braking_m_s2 - acceleration). The reward term is cut_in (<= 0). The actor's inputs do not
change; the follower's speed and gap are already among them where it is a neighbor.
"""
import math

REWARD_VERSION = 'highway_flow_rl_merge_v14_cut_in'
SETTINGS = dict(
    weight=.25,              # v06's avoidable_braking and the stop fix's merge_braking weights
    braking_m_s2=-3.,        # the candidate charge of the diagnosis
    max_gap_m=40.,           # traced cut-in gaps were 0.3-30 m; spawn transients 55-127 m
    followers='background')  # controlled followers' braking is their own policy's


def validate_settings(settings):
    settings = dict(settings)
    if settings != SETTINGS:
        raise ValueError('Unreviewed v14 cut-in settings')
    return settings


def charges(conn, states, controlled, settings=SETTINGS):
    """{controlled car: (sum of excess braking in m/s^2, [events])} for the step just simulated.

    states: every vehicle's state after the step (acceleration in m/s^2); controlled: our active cars.
    Only background vehicles braking below the threshold are looked up, so a quiet step costs no
    TraCI call.
    """
    threshold, limit = settings['braking_m_s2'], settings['max_gap_m']
    controlled = set(controlled)
    result = {}
    for vehicle, state in states.items():
        acceleration = state.get('acceleration')
        if vehicle in controlled or vehicle.startswith('agent_') or acceleration is None:
            continue
        if not (math.isfinite(acceleration) and acceleration < threshold):
            continue
        leader = conn.vehicle.getLeader(vehicle, limit)
        if not leader or not leader[0] or leader[0] not in controlled or leader[1] > limit:
            continue
        excess = threshold - acceleration
        total, events = result.get(leader[0], (0., []))
        events.append(dict(follower=vehicle, gap_m=round(float(leader[1]), 3), acceleration_m_s2=round(acceleration, 3),
                           excess_m_s2=round(excess, 3)))
        result[leader[0]] = (total + excess, events)
    return result
