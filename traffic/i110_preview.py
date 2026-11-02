"""Bounded scripted preview using conditional paths and current simulator state."""
from dataclasses import asdict, dataclass
import math

from traffic.i110_convergence import ConvergenceGraph, screen


@dataclass(frozen=True)
class PreviewConfig:
    range_m: float = 240.
    path_limit_m: float = 300.
    other_model: str = 'constant_speed'
    headway_s: float = 2.
    reserve_m: float = 10.
    release_reserve_m: float = 20.
    release_dwell_s: float = 1.
    reaction_s: float = .1
    jerk_m_s3: float = 2.
    deceleration_m_s2: float = 2.5
    standstill_gap_m: float = 2.5

    def __post_init__(self):
        if self.other_model not in ('constant_speed','brake_9'):
            raise ValueError('Unknown other-motion model')
        if any(not math.isfinite(v) or v <= 0 for k,v in asdict(self).items() if k != 'other_model'):
            raise ValueError('Preview parameters must be finite and positive')
        if self.release_reserve_m <= self.reserve_m:
            raise ValueError('Release reserve must exceed braking reserve')


def release_speed_budget(acceleration, dt, jerk):
    """Speed lost by this negative command then discrete +jerk release to zero.

    SUMO's default Euler speed update uses the next acceleration. This includes
    that next step, and does not assume an instantaneous acceleration reset.
    """
    if dt <= 0 or jerk <= 0 or not all(math.isfinite(v) for v in (acceleration,dt,jerk)):
        raise ValueError('Invalid release parameters')
    if acceleration >= 0:
        return 0.
    count = math.ceil(-acceleration/(jerk*dt))
    return dt*(count*(-acceleration)-jerk*dt*count*(count-1)/2)


def release_guard(filtered, previous, speed, dt, jerk):
    if release_speed_budget(filtered,dt,jerk) <= speed+1e-10:
        return filtered, dict(active=False, existing_state_conflict=False)
    lo, hi = filtered, 0.
    for _ in range(50):
        mid = (lo+hi)/2
        if release_speed_budget(mid,dt,jerk) > speed:
            lo = mid
        else:
            hi = mid
    return hi, dict(active=True, existing_state_conflict=hi > previous+jerk*dt+1e-7)


class PreviewController:
    def __init__(self, network, config=PreviewConfig()):
        self.config = config
        self.graph = ConvergenceGraph(network,config.path_limit_m)
        self.memory = {}
        self.plan = dict(ranges_m=[config.range_m],other_models=[config.other_model],
            headways_s=[config.headway_s],standstill_gap_m=config.standstill_gap_m,
            jerk_m_s3=config.jerk_m_s3,comfortable_deceleration_m_s2=config.deceleration_m_s2,
            reaction_s=config.reaction_s)

    def update_latch(self, vid, time_s, margins, known):
        state = self.memory.setdefault(vid,dict(active=False,clear_since=None,last_time=None))
        if state['last_time'] is not None and time_s <= state['last_time']:
            raise ValueError('Preview clock must advance; construct a new controller at reset')
        state['last_time'] = time_s
        minimum = min(margins) if margins else None
        if minimum is not None and minimum < self.config.reserve_m:
            state.update(active=True,clear_since=None)
        elif state['active']:
            clear = known and (minimum is None or minimum > self.config.release_reserve_m)
            if clear:
                if state['clear_since'] is None:
                    state['clear_since'] = time_s
                if time_s-state['clear_since'] >= self.config.release_dwell_s-1e-8:
                    state.update(active=False,clear_since=None)
            else:
                state['clear_since'] = None
        return dict(state)

    def observe(self, vid, states, time_s):
        fields = ('lane_id','lane_position','length','speed','acceleration','x','y')
        current = {key:{f:s[f] for f in fields} for key,s in states.items()}
        signal = screen(vid,current,self.graph,self.plan)
        key = f'{self.config.other_model}-h{self.config.headway_s:g}'
        supported = [(c,c['envelopes'][key]) for c in signal['candidates']
                     if c['envelopes'].get(key,{}).get('known')]
        known = (signal['ego_context_known'] and signal['stopping_profile_known']
                 and all(c['projection']['status'] != 'unknown_lane_context' for c in signal['candidates']))
        margins = [e['margin_m'] for _,e in supported]
        latch = self.update_latch(vid,time_s,margins,known)
        limiting = min(supported,key=lambda pair:(pair[1]['margin_m'],pair[0]['id'])) if supported else None
        return dict(active=latch['active'],context_known=known,
            requested_ceiling_m_s2=-self.config.deceleration_m_s2 if latch['active'] else None,
            candidate_count=len(signal['candidates']),supported_count=len(supported),
            minimum_margin_m=min(margins) if margins else None,
            limiting_vehicle=limiting[0]['id'] if limiting else None,
            limiting_projection=limiting[0]['projection'] if limiting else None,
            clear_since_s=latch['clear_since'],range_m=self.config.range_m,
            information='current exact simulator state; conditional paths, unknown intent')
