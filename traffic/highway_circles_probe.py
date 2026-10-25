"""Pure contracts for a three-car highway inference experiment."""
import math

from environments.lane_following_contract import _state
from environments.loop_hazard_contract import HazardLaneTask
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.lane_validation import footprint_lateral_bound

CEILING = 60*.44704


def validate_protocol(config):
    """Reject edits to v1 constants that the bounded runner does not parameterize."""
    fixed = dict(version='v01', speed_ceiling_mph=60., speed_ceiling_m_s=CEILING,
                 physics_hz=120, control_hz=60, circles_hz=20, settle_s=2.,
                 vehicle_ids=['ego', 'ppo_002', 'ppo_003'],
                 lane_radii_m=[500., 503.7, 507.4], start_angles_rad=[0., .04, -.04],
                 traffic_target_speed_m_s=3.,
                 no_leader_input=dict(leader_speed_m_s=CEILING, gap_m=1000.),
                 training=False, optimizer_steps=0)
    for name, expected in fixed.items():
        if config.get(name) != expected:
            raise ValueError('Unsupported v1 protocol setting: ' + name)
    if not 2. < config['duration_s'] <= 35.:
        raise ValueError('v1 duration must be greater than settling and at most 35 s')
    if not 30 < config['max_process_seconds'] <= 600:
        raise ValueError('v1 process deadline must be between 30 and 600 s')


def lane_route(identity, radius, angle, length=100.):
    return LaneRoute(identity,3.6,(RouteSegment(length,1/radius),),
                     (radius*math.cos(angle),radius*math.sin(angle)),angle+math.pi/2)


def lane_only_observation(state, applied, *, radius, start_angle, tick):
    """Explicit nonreactive traffic: same clear-road 42-field policy interface.

    References roll at 60m without pose changes. Empty object slots are the
    declared traffic task, not a claim of sensing or learned collision avoidance.
    """
    angle=math.atan2(state['position_m'][1],state['position_m'][0])
    total=radius*(angle-start_angle)
    window=max(0,int(total//60.))
    task=HazardLaneTask(lane_route('traffic',radius,start_angle+window*60/radius),stage='clear')
    task.set_objects((),now_s=tick/120.)
    native=_state(state)
    relative=task._relative(native,tick)
    vector=task._observation(native,(applied.steering_rad,applied.throttle,applied.brake),tick,relative)
    return list(vector),dict(window=window,total_progress_m=total,objects_masked_by_task=True)


def circles_inputs(states, route):
    """Nearest measured same-lane leader; explicit surrogate when lane is clear."""
    ego=states[0]
    yaw=ego['yaw_rad']; c,s=math.cos(yaw),math.sin(yaw)
    speed=max(0.,c*ego['velocity_m_s'][0]+s*ego['velocity_m_s'][1])
    here=route.project(*ego['position_m'][:2])
    candidates=[]
    for i,other in enumerate(states[1:],1):
        p=route.project(*other['position_m'][:2])
        delta=p.s_m-here.s_m
        if delta>0 and abs(p.lateral_error_m)<=1.8:
            gap=delta-4.8
            velocity=max(0.,c*other['velocity_m_s'][0]+s*other['velocity_m_s'][1])
            candidates.append((gap,i,velocity))
    if candidates:
        gap,index,velocity=min(candidates)
        if gap<=0: raise ValueError('Overlapping same-lane lead vehicle')
        return [speed,velocity,gap],dict(leader_index=index,source='native_same_lane_arc_gap')
    return [speed,CEILING,1000.],dict(leader_index=None,source='declared_no_leader_surrogate')


def guard(state, route, contacts, unsupported, tick):
    state=_state(state)
    p=route.project(*state['position_m'][:2])
    bound=footprint_lateral_bound(state,p,(4.8,1.8,1.4),1/500.)
    forward=sum(a*b for a,b in zip(state['velocity_m_s'][:2],
                                  (math.cos(p.point.yaw_rad),math.sin(p.point.yaw_rad))))
    reason=None
    if contacts: reason='rigid_contact'
    elif bound>1.8: reason='lane_departure'
    elif tick>240 and unsupported>12: reason='wheel_support_loss'
    elif state['upright_z']<math.cos(math.radians(5)): reason='excessive_tilt'
    elif not 0<=state['position_m'][2]<=2: reason='unsafe_height'
    elif state['speed_m_s']>CEILING: reason='overspeed'
    elif forward<-.2: reason='reverse_motion'
    return dict(reason=reason,lateral_error_m=p.lateral_error_m,
                footprint_lateral_bound_m=bound,progress_m=p.s_m)
