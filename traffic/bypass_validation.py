"""CPU geometry and conservative point-cloud checks for the bounded bypass.

Ground truth boxes belong ONLY to evaluation. RouteSafetyBrake consumes scan
returns and odometry-relative route poses; sparse/occluded returns cannot prove
free space. Its sampled box envelope is padded by route sample spacing.
"""
from dataclasses import dataclass, replace
import math
from traffic.lidar_braking import LidarEmergencyBrake


@dataclass(frozen=True)
class OrientedBox:
    x_m: float
    y_m: float
    yaw_rad: float
    length_m: float = 4.8
    width_m: float = 1.8

    def __post_init__(self):
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   for v in (self.x_m, self.y_m, self.yaw_rad, self.length_m, self.width_m)):
            raise ValueError('Box values must be finite')
        if min(self.length_m, self.width_m) <= 0:
            raise ValueError('Box dimensions must be positive')

    def corners(self):
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return [(self.x_m+c*x-s*y, self.y_m+s*x+c*y)
                for x, y in ((-self.length_m/2,-self.width_m/2),
                             (self.length_m/2,-self.width_m/2),
                             (self.length_m/2,self.width_m/2),
                             (-self.length_m/2,self.width_m/2))]


def _point_segment_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def box_clearance(a, b):
    """Exact planar polygon distance; touching counts as overlap (gap zero)."""
    pa, pb = a.corners(), b.corners()
    separated = False
    for polygon in (pa, pb):
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            axis = (q[1]-p[1], p[0]-q[0])
            aa = [x*axis[0]+y*axis[1] for x,y in pa]
            bb = [x*axis[0]+y*axis[1] for x,y in pb]
            if max(aa) < min(bb) or max(bb) < min(aa):
                separated = True
    if not separated:
        return dict(clearance_m=0., overlap=True)
    gap = min(_point_segment_distance(p, u, v)
              for points, polygon in ((pa,pb),(pb,pa)) for p in points
              for u,v in zip(polygon,polygon[1:]+polygon[:1]))
    return dict(clearance_m=gap, overlap=False)


def footprint_within_road(box, *, y_min_m=-1.8, y_max_m=5.4,
                          x_min_m=None, x_max_m=None):
    if not all(math.isfinite(v) for v in (y_min_m,y_max_m)) or y_min_m >= y_max_m:
        raise ValueError('Road bounds must be finite and ordered')
    return all(y_min_m <= y <= y_max_m and (x_min_m is None or x >= x_min_m)
               and (x_max_m is None or x <= x_max_m) for x,y in box.corners())


def predicted_route_poses(route, state, speed_m_s, max_scan_age_s=.2):
    """Join actual ego pose to the projected route over two metres, then sample.

    This is a geometric preview, not a bicycle-model rollout or a dynamics
    guarantee. It preserves the initial lateral/heading error and removes it
    gradually; the safety envelope must be combined with tracking-error gates.
    """
    x,y = state['position_m'][:2]
    yaw = state['yaw_rad']
    if not all(math.isfinite(v) for v in (x,y,yaw,speed_m_s,max_scan_age_s)) or not 0 <= speed_m_s <= 3 or max_scan_age_s < 0:
        raise ValueError('Invalid preview state')
    start = route.project(x,y).s_m
    projected = route.evaluate(start)
    ox,oy = x-projected.position_xy[0],y-projected.position_xy[1]
    heading_error = math.atan2(math.sin(yaw-projected.yaw_rad),math.cos(yaw-projected.yaw_rad))
    horizon = speed_m_s**2/3+speed_m_s*(.2+max_scan_age_s)+2.
    c,s = math.cos(yaw),math.sin(yaw)
    result = [(0.,0.,0.)]
    for i in range(1,math.ceil(horizon/.1)+1):
        distance = i*.1
        point = route.evaluate(start+distance)
        blend = max(0.,1-distance/2.)
        dx,dy = point.position_xy[0]+ox*blend-x,point.position_xy[1]+oy*blend-y
        angle = math.atan2(math.sin(point.yaw_rad+heading_error*blend-yaw),
                           math.cos(point.yaw_rad+heading_error*blend-yaw))
        target = (c*dx+s*dy,-s*dx+c*dy,angle)
        previous = result[-1]
        delta = math.atan2(math.sin(angle-previous[2]),math.cos(angle-previous[2]))
        count = max(1,math.ceil(math.hypot(target[0]-previous[0],target[1]-previous[1])/.2),
                    math.ceil(abs(delta)/.08))
        result.extend((previous[0]+(target[0]-previous[0])*j/count,
                       previous[1]+(target[1]-previous[1])*j/count,
                       previous[2]+delta*j/count) for j in range(1,count+1))
    return result


class RouteSafetyBrake:
    """Freshness/straight stop plus conservative sampled future body envelope.

    Route begins at (0,0,0) in the scan's current chassis reference frame.
    Arc increments <=0.25 m and yaw increments <=0.1 rad are mandatory; padding
    covers translation plus rotation between samples. Missing/short/malformed
    routes fail safe. This assumes static returns, not moving-object tracking.
    """
    def __init__(self, episode_id, vehicle_id, config=None):
        self.brake = LidarEmergencyBrake(episode_id,vehicle_id,config)
        self._route_latched = False

    def evaluate(self, scan, *, tick, speed_m_s, predicted_route,
                 requested_target_speed_m_s=3.):
        decision = self.brake.evaluate(scan,tick=tick,speed_m_s=speed_m_s,
                                      requested_target_speed_m_s=requested_target_speed_m_s)
        if decision.status != 'clear':
            return decision
        def stop(reason, status='stale_invalid'):
            return replace(decision,status=status,reason=reason,target_speed_m_s=0.,brake_override=1.,
                           stop_latched=self._route_latched or decision.stop_latched)
        reference = scan.acquisition_end_tick if scan.coordinate_reference_tick is None else scan.coordinate_reference_tick
        if reference != tick:
            return stop('scan_not_in_current_chassis_frame')
        if self._route_latched:
            return stop('latched_route_stop','obstacle')
        try:
            route = [OrientedBox(*pose) for pose in predicted_route]
            if not route or max(abs(route[0].x_m),abs(route[0].y_m),abs(route[0].yaw_rad)) > 1e-6:
                return stop('invalid_route_origin')
            traveled, boxes = 0., []
            radius = math.hypot(2.4,.9)
            for i, box in enumerate(route):
                padding = 0.
                if i:
                    previous = route[i-1]
                    distance = math.hypot(box.x_m-previous.x_m,box.y_m-previous.y_m)
                    angle = abs(math.atan2(math.sin(box.yaw_rad-previous.yaw_rad),math.cos(box.yaw_rad-previous.yaw_rad)))
                    if distance > .250001 or angle > .100001:
                        return stop('route_sampling_too_sparse')
                    traveled += distance
                    padding = distance+radius*angle
                boxes.append((box,padding))
                if traveled >= decision.stopping_distance_m:
                    break
            if traveled < decision.stopping_distance_m:
                return stop('route_shorter_than_stopping_horizon')
            cfg = self.brake.config
            for x,y,z in scan.points:
                if not cfg.min_height_m <= z <= cfg.max_height_m:
                    continue
                # Ignore only points strictly inside the current own footprint.
                if abs(x) < 2.4 and abs(y) < .9:
                    continue
                for box,padding in boxes:
                    c,s = math.cos(box.yaw_rad),math.sin(box.yaw_rad)
                    dx,dy = x-box.x_m,y-box.y_m
                    if (abs(c*dx+s*dy) <= 2.4+padding+cfg.lateral_margin_m
                            and abs(-s*dx+c*dy) <= .9+padding+cfg.lateral_margin_m):
                        self._route_latched = True
                        return stop('lidar_hit_in_predicted_body_envelope','obstacle')
        except (TypeError,ValueError,OverflowError):
            return stop('invalid_predicted_route')
        return decision


def assess_bypass(rows, *, mode='pass', obstacle=None, physics_hz=120):
    """Assess every physics row; success and fail-safe stop modes are distinct.

    Required extra fields: obstacle_contacts, route_adopted_from_lidar;
    existing state fields: position_m, yaw_rad, speed_m_s, upright_z,
    wheel_on_ground, tick, control. hold rows use phase='hold'.
    """
    if mode not in ('pass','blocked','dropout'):
        raise ValueError('Unsupported bypass mode')
    obstacle = obstacle or OrientedBox(45.,0.,0.,2.,1.6)
    gates = dict(nonempty=bool(rows))
    try:
        boxes = [OrientedBox(*r['position_m'][:2],r['yaw_rad']) for r in rows]
        finite = all(math.isfinite(r['speed_m_s']) and math.isfinite(r['upright_z']) for r in rows)
        gates['finite_telemetry'] = finite
        gates['clock'] = all(r['tick'] == rows[0]['tick']+i for i,r in enumerate(rows))
        if not rows or not finite:
            return dict(passed=False,gates=gates,metrics={})
        clearance = min(box_clearance(b,obstacle)['clearance_m'] for b in boxes)
        hold = [r for r in rows if r.get('phase') == 'hold']
        gates.update(no_contacts=all(r['obstacle_contacts'] == 0 for r in rows),
            road_footprint=all(footprint_within_road(b) for b in boxes),
            clearance=clearance >= .35,
            speed_bound=all(0 <= r['speed_m_s'] <= 3. for r in rows),
            upright=all(r['upright_z'] > .99 for r in rows),
            four_supports=all(len(r['wheel_on_ground']) == 4 and all(r['wheel_on_ground']) for r in rows),
            hold_duration=len(hold)/physics_hz >= 5,
            hold_stopped=bool(hold) and all(r['speed_m_s'] < .05 for r in hold))
        if mode == 'pass':
            adoption = [r for r in rows if r['route_adopted_from_lidar']]
            active = [r for r in rows if r['tick'] > 2*physics_hz]
            gates.update(progress=rows[-1]['position_m'][0] >= 85,
                detour=max(r['position_m'][1] for r in rows) >= 3.,
                lane_return=abs(rows[-1]['position_m'][1]) < .25,
                sensed_early_adoption=bool(adoption) and adoption[0]['position_m'][0] < 30.,
                uninterrupted_safe_pass=bool(active) and all(
                    r.get('safety_status') == 'clear' and r['lidar']['status'] == 'clear'
                    and r['lidar']['brake_override'] == 0
                    and r['planner']['status'] not in ('stop','stale_invalid','no_safe_route','blocked')
                    for r in active))
        else:
            if mode == 'blocked':
                failing = [r for r in rows if r['tick'] > 2*physics_hz and
                           r.get('safety_status') in ('blocked','no_safe_route','obstacle')]
            else:
                failing = [r for r in rows if r['tick'] >= 12*physics_hz and
                           r.get('safety_status') == 'stale_invalid']
            first = rows.index(failing[0]) if failing else len(rows)
            gates.update(fault_observed=bool(failing),
                full_brake=bool(failing) and all(r['control']['brake'] == 1 and r['control']['throttle'] == 0 for r in rows[first:]),
                stopped_before_barrier=all(max(x for x,y in b.corners()) < 44. for b in boxes),
                no_unexpected_prefault=all(r.get('safety_status') == 'clear'
                    for r in rows[:first] if r['tick'] > 2*physics_hz))
            if mode == 'dropout':
                preceding = [r for r in rows[:first] if failing and
                             failing[0]['tick']-physics_hz/2 <= r['tick'] < failing[0]['tick']]
                gates.update(moving_before_dropout=len(preceding) >= math.ceil(physics_hz/2)
                             and all(r['speed_m_s'] > 2 for r in preceding))
                # Rows are logged AFTER a physics step: control tick = row tick-1.
                # Freeze starts at control tick 1440; derive the oldest return
                # from the latest valid control evaluation at/before that tick.
                anchors = [r for r in rows if r['tick'] <= 12*physics_hz+1
                           and r.get('control_tick') is True and r['lidar']['status'] == 'clear'
                           and isinstance(r['lidar'].get('oldest_sample_age_s'),(int,float))
                           and math.isfinite(r['lidar']['oldest_sample_age_s'])
                           and 0 <= r['lidar']['oldest_sample_age_s'] <= .2]
                deadline = None
                if anchors:
                    anchor = anchors[-1]
                    oldest = anchor['tick']-1-anchor['lidar']['oldest_sample_age_s']*physics_hz
                    expires = math.floor(oldest+.2*physics_hz+1e-6)+1
                    deadline = expires+(expires%2)+1
                gates['dropout_brake_timely'] = (bool(failing) and deadline is not None
                    and failing[0]['tick'] <= deadline
                    and failing[0].get('control_tick') is True
                    and failing[0]['lidar']['status'] == 'stale_invalid'
                    and failing[0]['lidar']['brake_override'] == 1
                    and failing[0]['lidar']['target_speed_m_s'] == 0)
        return dict(passed=all(gates.values()),gates=gates,metrics=dict(minimum_clearance_m=clearance,
                    final_x_m=rows[-1]['position_m'][0],final_y_m=rows[-1]['position_m'][1],
                    maximum_y_m=max(r['position_m'][1] for r in rows),
                    road_departure_samples=sum(not footprint_within_road(b) for b in boxes)))
    except (KeyError,TypeError,ValueError,OverflowError):
        gates['valid_trace_schema'] = False
        return dict(passed=False,gates=gates,metrics={})
