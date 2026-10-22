"""Pre-reset scenario compatibility, separate from policy observations/outcomes.

Fixed nonreactive backgrounds have a known angular plan. We reject a hazard
schedule whose complete 35 s trajectory cannot retain 0.8 m box separation from
that plan. 10 Hz exact samples use a Hausdorff-speed half-step bound between
samples, plus 0.02 m for the existing native pose/angle tracking allowances.
This certifies only the declared scripted object/background trajectories;
it says nothing about the learned ego's future safety or success.
"""
from __future__ import annotations
from copy import deepcopy
import json
import math

from traffic.bypass_validation import OrientedBox
from traffic.road_hazards import HazardSchedule,make_hazard_schedule,number
from traffic.showcase_clearance import nearest_clearance

MAX_CANDIDATES=16
HORIZON_SECONDS=35.
SAMPLE_HZ=10
MIN_CLEARANCE_M=.8
NATIVE_TRACKING_PADDING_M=.02
BASE_SPEED_M_S=5*.44704
ANGULAR_SPEED_RAD_S=BASE_SPEED_M_S/500
BACKGROUND_HALF_DIAGONAL_M=math.hypot(4.8,1.9)/2


class ScenarioCompatibilityError(RuntimeError):
    def __init__(self,report):
        self.report=report
        super().__init__('No compatible declared hazard/background scenario: '+json.dumps(report,allow_nan=False,separators=(',',':')))


def background_plan(live):
    """Read the fixed live plan without stepping, changing membership or poses."""
    if (len(live.active)!=12 or len(live.specs)!=80 or live.stopping or live.ramping
            or live.events or live.pending_lifecycle
            or abs(live.speed_m_s-BASE_SPEED_M_S)>1e-12
            or live.settings['traffic_count']!=12 or live.settings['traffic_mph']!=5.):
        raise ValueError('Compatibility preflight requires the fixed12-car5mph background plan')
    entries=[]
    for key in sorted(live.active):
        spec=live.specs[key]
        if spec.lane not in (1,2,3):raise ValueError('Expected adjacent background lanes')
        entries.append(dict(object_id=live.identity(key),lane=spec.lane,station_m=live.station(key)))
    return dict(global_start_s=live.time_s,angular_speed_rad_s=ANGULAR_SPEED_RAD_S,entries=entries)


def _validated_plan(plan):
    if not isinstance(plan,dict) or set(plan)!={'global_start_s','angular_speed_rad_s','entries'}:
        raise ValueError('Incomplete background plan')
    start=number(plan['global_start_s'],'global_start_s')
    omega=number(plan['angular_speed_rad_s'],'angular_speed_rad_s')
    entries=plan['entries']
    if start<0 or omega!=ANGULAR_SPEED_RAD_S or not isinstance(entries,(tuple,list)) or len(entries)!=12:
        raise ValueError('Invalid fixed background plan')
    identities=set()
    for row in entries:
        if (not isinstance(row,dict) or set(row)!={'object_id','lane','station_m'}
                or not isinstance(row['object_id'],str) or not row['object_id']
                or row['object_id'] in identities or type(row['lane']) is not int or row['lane'] not in (1,2,3)):
            raise ValueError('Invalid background identity/lane')
        number(row['station_m'],'station_m');identities.add(row['object_id'])
    return entries


def assess_schedule(schedule,route,plan):
    entries=_validated_plan(plan)
    if not isinstance(schedule,HazardSchedule):raise ValueError('Expected declared HazardSchedule')
    if any(h.speed_m_s>1.25 for h in schedule.hazards):
        raise ValueError('Hazard speed exceeds the declared v1 schedule envelope')
    max_hazard_speed=max((h.speed_m_s for h in schedule.hazards),default=0.)
    # A rotating footprint's every point moves no faster than omega*(radius +
    # half diagonal). Every time lies within half a sample step of an endpoint.
    # Existing native checks allow background center/yaw errors .002 m/.0001
    # rad and hazard center/yaw .005 m/.005 rad; .02 m exceeds their summed
    # center plus corner Hausdorff allowance (less than .011 m).
    max_background_point_speed=ANGULAR_SPEED_RAD_S*(511.1+BACKGROUND_HALF_DIAGONAL_M)
    temporal_padding=(max_hazard_speed+max_background_point_speed)/(2*SAMPLE_HZ)
    required=MIN_CLEARANCE_M+NATIVE_TRACKING_PADDING_M+temporal_padding
    report=dict(passed=True,horizon_seconds=HORIZON_SECONDS,sample_hz=SAMPLE_HZ,
        minimum_required_clearance_m=MIN_CLEARANCE_M,tracking_padding_m=NATIVE_TRACKING_PADDING_M,
        between_sample_padding_m=temporal_padding,required_sample_clearance_m=required,
        minimum_checked_clearance_m=None,continuous_clearance_lower_bound_m=None,
        samples_checked=0,exact_pairs=0,background_count=12,first_rejection=None,
        premise='fixed angular background targets; fixed hazard schedule; existing native tracking gates',
        scope='scenario object/background compatibility only; no ego outcome or actor future input')
    if not schedule.hazards:return report
    minimum=math.inf
    for sample_index in range(round(HORIZON_SECONDS*SAMPLE_HZ)+1):
        t=sample_index/SAMPLE_HZ
        backgrounds=[]
        for row in entries:
            angle=row['station_m']/500+ANGULAR_SPEED_RAD_S*t
            radius=500+3.7*row['lane']
            backgrounds.append(OrientedBox(radius*math.cos(angle),radius*math.sin(angle),
                                           angle+math.pi/2,4.8,1.9))
        for obj in schedule.sample(route,t):
            geometry=nearest_clearance(OrientedBox(obj.x_m,obj.y_m,obj.yaw_rad,1.,1.),backgrounds)
            minimum=min(minimum,geometry['clearance_m'])
            report['exact_pairs']+=geometry['exact_pairs']
            report['samples_checked']+=1
            if geometry['overlap'] or geometry['clearance_m']<required:
                report.update(passed=False,first_rejection=dict(time_s=t,object_id=obj.object_id,
                    clearance_m=geometry['clearance_m'],overlap=geometry['overlap'],reason='scheduled_background_clearance'))
                break
        if not report['passed']:break
    report.update(minimum_checked_clearance_m=minimum,
        continuous_clearance_lower_bound_m=minimum-temporal_padding-NATIVE_TRACKING_PADDING_M)
    return report


def choose_schedule(stage,seed,reset_index,route,plan,*,side=None):
    """Same seed, bounded declared draws; never replace a failed evaluation seed."""
    if type(reset_index) is not int or reset_index<0:raise ValueError('Invalid reset index')
    _validated_plan(plan)
    report=dict(schema_version=1,seed=seed,reset_episode_index=reset_index,requested_side=side,
        candidate_rule='candidate_episode_index = reset_episode_index*16 + draw_index',
        candidate_limit=MAX_CANDIDATES,background_plan=deepcopy(plan),draws=[],selected_draw=None,
        outcome='invalid_scenario',actor_future_schedule_input=False)
    for draw in range(MAX_CANDIDATES):
        schedule=make_hazard_schedule(stage,seed,reset_index*MAX_CANDIDATES+draw,route=route,side=side)
        assessment=assess_schedule(schedule,route,plan)
        report['draws'].append(dict(draw_index=draw,schedule=schedule.as_dict(),assessment=assessment))
        if assessment['passed']:
            report.update(outcome='accepted',selected_draw=draw)
            return schedule,report
    raise ScenarioCompatibilityError(report)
