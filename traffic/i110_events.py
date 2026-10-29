"""Small synthetic SUMO event adapter; permanent geometry is read-only.

One manifest, one actor. Unsupported closures/police sweeps fail closed. This
adapter requests motion; the next simulation state is the realization evidence.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from scripts.build_i110_sumo import write_xml


def physical_lane(road, index):
    if road not in {f'sb{i}' for i in range(6)}:
        return None
    value = index - (1 if road in {'sb1', 'sb2', 'sb3'} else 0)
    return value if 0 <= value <= 3 else None


def load_event(path):
    spec = json.loads(Path(path).read_text(encoding='utf-8'))
    common = {'schema_version', 'event_id', 'enabled', 'kind', 'provenance',
              'motion_owner', 'actor_id', 'route_id', 'origin', 'depart_s',
              'depart_position_m', 'initial_physical_lane', 'cruise_speed_m_s',
              'actor_deceleration_m_s2', 'onset_s', 'release_s', 'follower_id',
              'extent', 'speed_mode', 'lane_change_mode', 'limits'}
    extra = ({'duration_s', 'target_physical_lane'} if spec.get('kind') == 'cut_in'
             else {'requested_deceleration_m_s2', 'hold_s', 'recovery_acceleration_m_s2'})
    optional = {'request_lane_change'} if spec.get('kind') == 'cut_in' else set()
    if (not (common | extra) <= set(spec) or set(spec) - (common | extra | optional)
            or spec['kind'] not in {'cut_in', 'leader_brake'}):
        raise ValueError('Unsupported event schema or fields')
    if 'request_lane_change' in spec and type(spec['request_lane_change']) is not bool:
        raise ValueError('request_lane_change must be boolean')
    if (spec['schema_version'] != 1 or type(spec['enabled']) is not bool
            or spec['provenance'] != 'synthetic' or spec['motion_owner'] != 'SUMO'
            or spec['speed_mode'] != 31 or spec['lane_change_mode'] != 512
            or spec['actor_id'] != 'event_actor' or spec['follower_id'] != 'agent_1'
            or spec['route_id'] != 'through' or spec['origin'] != 'mainline'
            or spec['depart_s'] != 0
            or spec['extent'] != {'edges': [f'sb{i}' for i in range(6)]}
            or not isinstance(spec['event_id'], str) or not spec['event_id']):
        raise ValueError('Unsupported event identity, authority, extent or safety mode')
    for key, value in spec.items():
        if key.endswith(('_s', '_m', '_m_s', '_m_s2')):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError('Invalid event number: ' + key)
    for key in ('cruise_speed_m_s', 'actor_deceleration_m_s2', 'onset_s', 'release_s'):
        if spec[key] <= 0:
            raise ValueError('Expected positive event parameter: ' + key)
    if not 4.8 <= spec['depart_position_m'] <= 50 or spec['cruise_speed_m_s'] > 25 or spec['actor_deceleration_m_s2'] > 9:
        raise ValueError('Event actor outside bounded fixture envelope')
    for key in ('initial_physical_lane', 'target_physical_lane'):
        if key in spec and (type(spec[key]) is not int or not 0 <= spec[key] <= 3):
            raise ValueError('Invalid physical lane')
    if spec['kind'] == 'cut_in':
        if (spec['duration_s'] <= 0 or abs(spec['initial_physical_lane']-spec['target_physical_lane']) != 1
                or spec['release_s'] <= spec['onset_s']+spec['duration_s']):
            raise ValueError('Invalid cut-in interval or adjacency')
    else:
        decel = spec['requested_deceleration_m_s2']
        recover = spec['recovery_acceleration_m_s2']
        if not 0 < decel <= spec['actor_deceleration_m_s2'] or not 0 < recover <= 2:
            raise ValueError('Unsupported braking/recovery capability')
        end = spec['onset_s'] + spec['cruise_speed_m_s']/decel + spec['hold_s'] + spec['cruise_speed_m_s']/recover
        if spec['release_s'] < end:
            raise ValueError('Release precedes scheduled recovery')
    return spec


def add_actor(routes_path, spec):
    if not spec['enabled']:
        return
    root = ET.parse(routes_path).getroot()
    if root.find(f"./vehicle[@id='{spec['actor_id']}']") is not None:
        raise ValueError('Event actor already exists')
    typ = deepcopy(root.find("./vType[@id='human']"))
    typ.set('id', 'event_type')
    typ.set('decel', str(spec['actor_deceleration_m_s2']))
    root.insert(2, typ)
    actor = ET.Element('vehicle', id=spec['actor_id'], type='event_type', route=spec['route_id'],
        depart=str(spec['depart_s']), departPos=str(spec['depart_position_m']),
        departLane=str(spec['initial_physical_lane']), departSpeed=str(spec['cruise_speed_m_s']))
    ET.SubElement(actor, 'param', key='research.origin', value=spec['origin'])
    vehicles = list(root.findall('vehicle')) + [actor]
    for vehicle in root.findall('vehicle'):
        root.remove(vehicle)
    root.extend(sorted(vehicles, key=lambda v: (float(v.get('depart')), v.get('id'))))
    write_xml(routes_path, root)


def speed_target(spec, time):
    if time >= spec['release_s'] - 1e-9:
        return -1.0  # Return longitudinal authority to native IDM.
    cruise = spec['cruise_speed_m_s']
    if spec['kind'] == 'cut_in' or time < spec['onset_s']:
        return cruise
    elapsed = time - spec['onset_s']
    brake_time = cruise / spec['requested_deceleration_m_s2']
    if elapsed < brake_time:
        return max(0., cruise - spec['requested_deceleration_m_s2'] * elapsed)
    if elapsed < brake_time + spec['hold_s']:
        return 0.
    return min(cruise, (elapsed-brake_time-spec['hold_s']) * spec['recovery_acceleration_m_s2'])


class EventAdapter:
    def __init__(self, spec):
        self.spec = spec
        self.records = []
        self.pending = None
        self.lane_requested = False
        self.released = False
        self.follower_seen = False
        self.lane_transition_seen = False
        self.previous_lane = None
        self.min_speed = None
        self.max_deceleration = 0.
        self.missing_steps = 0
        self.braking_with_follower = False

    def observe(self, time, states):
        if not self.spec['enabled']:
            return
        spec = self.spec
        state = states.get(spec['actor_id'])
        if self.pending is not None:
            row = self.pending
            row['realized_time_s'] = time
            row['realized'] = ({k: state[k] for k in ('speed', 'acceleration', 'road_id', 'lane_index', 'lane_position')}
                               if state else None)
            row['speed_request_error_m_s'] = (state['speed']-row['requested_speed_m_s']
                if state and row['requested_speed_m_s'] >= 0 else None)
            self.pending = None
        if state:
            lane = physical_lane(state['road_id'], state['lane_index'])
            during = spec['onset_s'] <= time <= spec['release_s']
            follower = states.get(spec['follower_id'], {})
            leader = follower.get('leader')
            following = bool(leader and leader[0] == spec['actor_id'])
            if during:
                self.follower_seen |= following
                self.min_speed = state['speed'] if self.min_speed is None else min(self.min_speed, state['speed'])
                self.max_deceleration = max(self.max_deceleration, -state['acceleration'])
                if spec['kind'] == 'leader_brake':
                    self.braking_with_follower |= following and -state['acceleration'] >= .9*spec['requested_deceleration_m_s2']
                elif (time <= spec['onset_s']+spec['duration_s']+.1 and lane == spec['target_physical_lane']
                      and self.previous_lane == spec['initial_physical_lane']):
                    self.lane_transition_seen = True
            if lane is not None:
                self.previous_lane = lane

    def apply(self, conn, time, states, dt):
        spec = self.spec
        if not spec['enabled'] or self.released:
            return
        actor = states.get(spec['actor_id'])
        row = dict(event_id=spec['event_id'], requested_at_s=time,
                   effective_step_end_s=round(time+dt, 10), actor_id=spec['actor_id'],
                   requested_speed_m_s=speed_target(spec, time+dt), lane_request=None,
                   actor_present=actor is not None, speed_mode=31, lane_change_mode=512)
        self.records.append(row)
        self.pending = row
        if actor is None:
            self.missing_steps += 1
            if time >= spec['release_s']:
                self.released = True
            return
        conn.vehicle.setSpeed(spec['actor_id'], row['requested_speed_m_s'])
        if (spec['kind'] == 'cut_in' and spec.get('request_lane_change',True) and not self.lane_requested
                and spec['onset_s']-1e-9 <= time < spec['onset_s']+spec['duration_s']):
            road = actor['road_id']
            if road in spec['extent']['edges']:
                target = spec['target_physical_lane'] + (1 if road in {'sb1', 'sb2', 'sb3'} else 0)
                duration = spec['onset_s']+spec['duration_s']-time
                conn.vehicle.changeLane(spec['actor_id'], target, duration)
                row['lane_request'] = dict(road_id=road, lane_index=target,
                    physical_lane=spec['target_physical_lane'], duration_s=duration)
                self.lane_requested = True
        self.released = row['requested_speed_m_s'] == -1

    def summary(self):
        enabled = self.spec['enabled']
        realized = (self.follower_seen and (self.lane_transition_seen if self.spec['kind'] == 'cut_in'
                    else self.min_speed is not None and self.min_speed <= .1 and self.braking_with_follower))
        if self.spec['kind']=='cut_in' and not self.spec.get('request_lane_change',True):
            realized = not self.lane_transition_seen and not self.lane_requested
        return dict(event_id=self.spec['event_id'], enabled=enabled,
            request_lane_change=self.spec.get('request_lane_change',True) if self.spec['kind']=='cut_in' else None,
            event_gate_passed=bool(not enabled or (realized and not self.missing_steps and self.released)),
            follower_seen=self.follower_seen, lane_request_sent=self.lane_requested,
            lane_transition_seen=self.lane_transition_seen, min_actor_speed_m_s=self.min_speed,
            max_actor_deceleration_m_s2=self.max_deceleration, braking_with_follower=self.braking_with_follower,
            missing_actor_command_steps=self.missing_steps, released=self.released,
            interpretation='Requested event is not realization; blocked/unrealized events fail the fixture gate.')
