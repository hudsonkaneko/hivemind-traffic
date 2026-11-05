"""Frozen Stage-1 actor bridge for a bounded straight-road PhysX fixture.

Inputs are explicit simulator truth and a known map, NOT camera/LiDAR estimates.
The actor's original v2 93-field meaning is preserved for straight roads without
merges. One autonomous ego means no delivered V2V messages; the six absent slots
are meaningful absence, not substitutes for unavailable measurements. Physical
lane changes additionally mask new maneuvers until the adapter completes one.
No SUMO, Isaac or training dependency is imported by observation construction.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

CAP_M_S = 26.8224
EXPECTED_POLICY_SHA256 = '8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8'
SCHEMA = 'highway_stage1_graph_messages_v2'
SOURCE = 'privileged_physx_state_and_known_straight_map_no_v2v'
LOCAL_FEATURES = (
    'speed_over_cap', 'acceleration_over_4p5', 'lane_index_fraction',
    'remaining_route_fraction', 'allowed_speed_over_cap', 'previous_request_over_4p5',
    *(f'{lane}_{direction}_{feature}' for lane in ('own', 'left', 'right')
      for direction in ('front', 'rear')
      for feature in ('gap_over_120', 'relative_speed_over_cap', 'present')),
    'left_lane_exists', 'right_lane_exists', 'lane_count_over_7',
    'distance_to_required_merge_over_240', 'requires_merge', 'lane_cooldown_over_5',
    'mask_right', 'mask_stay', 'mask_left', 'merge_context_known',
    'merge_target_lane_index_over_7', 'merge_target_lane_delta',
    'distance_to_target_lane_start_over_240', 'target_front_gap_over_120',
    'target_front_relative_speed_over_cap', 'target_front_present',
    'target_rear_gap_over_120', 'target_rear_relative_speed_over_cap',
    'target_rear_present', 'feasible_acceleration_ceiling_over_4p5',
    'target_allowed_speed_over_cap',
)
MESSAGE_FEATURES = ('southbound_position_delta_over_150', 'sender_requires_merge',
    'relative_speed_over_cap', 'acceleration_over_4p5',
    'previous_acceleration_request_over_4p5', 'age_over_0p5',
    'previous_lane_request', 'present')


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _clip(value, lower=-1., upper=1.):
    return min(upper, max(lower, value))


class StraightRoadPolicyBridge:
    """Map measured chassis-center state into the frozen actor's straight subset.

    +X is forward, +Y left; lane 0 is rightmost. Position conversion uses the
    oriented chassis footprint's projected front/rear envelope. At zero yaw this
    is center +/- length/2, exactly the original SUMO front-bumper gap convention.
    Traffic straddling a boundary occupies both lanes for conservative gap masks.
    Fields describing merges are explicitly unknown/not-required because this
    fixture contains only continuous parallel lanes with a common finite end.
    """
    def __init__(self, *, road_length_m=2000., lane_count=3, lane_width_m=3.6,
                 speed_limit_m_s=15.6464, start_x_m=0., lane0_y_m=0.):
        if type(lane_count) is not int or not 1 <= lane_count <= 7:
            raise ValueError('lane_count must be an integer in [1,7]')
        self.lane_count = lane_count
        for name, value in dict(road_length_m=road_length_m, lane_width_m=lane_width_m,
                speed_limit_m_s=speed_limit_m_s, start_x_m=start_x_m, lane0_y_m=lane0_y_m).items():
            setattr(self, name, _finite(value, name))
        if self.road_length_m <= 0 or self.lane_width_m <= 0 or not 0 < self.speed_limit_m_s <= CAP_M_S:
            raise ValueError('Positive road dimensions and speed limit <=60 mph required')

    def _vehicle(self, data):
        if not isinstance(data, dict) or not isinstance(data.get('id'), str) or not data['id']:
            raise ValueError('Each physical vehicle needs a nonempty id')
        state = dict(id=data['id'])
        for key in ('x_m', 'y_m', 'yaw_rad', 'speed_m_s', 'acceleration_m_s2'):
            if key not in data:
                raise ValueError(f'Missing measured vehicle field: {key}')
            state[key] = _finite(data[key], key)
        state['length_m'] = _finite(data.get('length_m', 4.8), 'length_m')
        state['width_m'] = _finite(data.get('width_m', 1.8), 'width_m')
        if state['length_m'] <= 0 or state['width_m'] <= 0 or state['speed_m_s'] < 0:
            raise ValueError('Vehicle dimensions must be positive and speed nonnegative')
        if abs(state['yaw_rad']) >= math.pi / 2:
            raise ValueError('This fixture supports forward-facing straight-road traffic only')
        c, s = abs(math.cos(state['yaw_rad'])), abs(math.sin(state['yaw_rad']))
        half_x = .5 * (state['length_m'] * c + state['width_m'] * s)
        half_y = .5 * (state['length_m'] * s + state['width_m'] * c)
        state.update(front_x_m=state['x_m'] + half_x, rear_x_m=state['x_m'] - half_x,
                     projected_length_m=2 * half_x)
        state['occupied_lanes'] = [lane for lane in range(self.lane_count)
            if abs(state['y_m'] - (self.lane0_y_m + lane * self.lane_width_m))
            < self.lane_width_m / 2 + half_y]
        return state

    @staticmethod
    def _neighbor(ego, traffic, lane, direction):
        candidates = []
        for other in traffic:
            if lane not in other['occupied_lanes']:
                continue
            ahead = other['front_x_m'] >= ego['front_x_m']
            if ahead != (direction == 'front'):
                continue
            # Match the source contract's subtraction order at the 120 m edge.
            gap = (other['front_x_m'] - ego['front_x_m'] - other['projected_length_m'] if ahead
                   else ego['front_x_m'] - other['front_x_m'] - ego['projected_length_m'])
            gap = max(0., gap)
            if gap <= 120.:
                candidates.append((gap, other['id'], other['speed_m_s']))
        if not candidates:
            return None
        gap, identity, speed = min(candidates)
        return dict(id=identity, bumper_gap_m=gap,
                    relative_speed_m_s=speed - ego['speed_m_s'])

    def observe(self, ego, traffic=(), *, time_s, distance_travelled_m,
                previous_request_m_s2=0., last_lane_change_s=-5.,
                lane_change_active=False, ego_lane_index=None):
        now = _finite(time_s, 'time_s')
        distance = _finite(distance_travelled_m, 'distance_travelled_m')
        previous = _finite(previous_request_m_s2, 'previous_request_m_s2')
        last_change = _finite(last_lane_change_s, 'last_lane_change_s')
        if now < 0 or distance < 0 or last_change > now or not -4.5 <= previous <= 2.:
            raise ValueError('Invalid time, distance, previous request or lane-change timestamp')
        if type(lane_change_active) is not bool:
            raise ValueError('lane_change_active must be boolean')
        ego = self._vehicle(ego)
        others = [self._vehicle(item) for item in traffic]
        ids = [ego['id'], *(item['id'] for item in others)]
        if len(set(ids)) != len(ids):
            raise ValueError('Vehicle ids must be unique and traffic must exclude ego')
        nearest = math.floor((ego['y_m'] - self.lane0_y_m) / self.lane_width_m + .5)
        lane = nearest if ego_lane_index is None else ego_lane_index
        if type(lane) is not int or not 0 <= lane < self.lane_count:
            raise ValueError('Ego lane is outside the declared road')
        if lane not in ego['occupied_lanes']:
            raise ValueError('Declared ego lane does not intersect its physical footprint')
        station = ego['front_x_m'] - self.start_x_m
        if not 0 <= station < self.road_length_m:
            raise ValueError('Ego front bumper is outside the finite road')
        neighbors = {label: {direction: self._neighbor(ego, others, lane + delta, direction)
            if 0 <= lane + delta < self.lane_count else None for direction in ('front', 'rear')}
            for label, delta in (('own', 0), ('left', 1), ('right', -1))}
        exists = dict(left=lane + 1 < self.lane_count, right=lane > 0)
        mask = [0., 1., 0.]
        if now - last_change >= 5. and not lane_change_active:
            for label, index in (('right', 0), ('left', 2)):
                if not exists[label]:
                    continue
                safe = True
                for direction, other in neighbors[label].items():
                    if other:
                        other_speed = max(0., ego['speed_m_s'] + other['relative_speed_m_s'])
                        follower, leader = ((ego['speed_m_s'], other_speed) if direction == 'front'
                                            else (other_speed, ego['speed_m_s']))
                        required = 2.5 + max(1.5 * follower,
                            .1 * follower + follower * follower / 9. - leader * leader / 18.)
                        safe &= other['bumper_gap_m'] >= required
                if safe:
                    mask[index] = 1.
        ceiling_bounds = dict(speed_ceiling=(self.speed_limit_m_s - ego['speed_m_s']) / .1)
        front = neighbors['own']['front']
        if front:
            leader_speed = max(0., ego['speed_m_s'] + front['relative_speed_m_s'])
            safe_speed = max(0., leader_speed + .5 *
                (front['bumper_gap_m'] - 2.5 - 1.5 * ego['speed_m_s']))
            ceiling_bounds['following'] = (safe_speed - ego['speed_m_s']) / .1
        physical_bounds = {}
        # The instantaneous SUMO lane-change interface had no straddling ego.
        # Account separately for both occupied physical lanes during transition;
        # preserve the original actor's own-lane observation and ceiling meaning.
        if lane_change_active:
            for occupied_lane in ego['occupied_lanes']:
                if occupied_lane == lane:
                    continue
                occupied_front = self._neighbor(ego, others, occupied_lane, 'front')
                if occupied_front:
                    leader = max(0., ego['speed_m_s'] + occupied_front['relative_speed_m_s'])
                    safe = max(0., leader + .5 *
                        (occupied_front['bumper_gap_m'] - 2.5 - 1.5 * ego['speed_m_s']))
                    physical_bounds[f'physical_occupied_lane_{occupied_lane}_following'] = (
                        safe - ego['speed_m_s']) / .1
        ceiling = _clip(min(2., *ceiling_bounds.values()), -4.5, 2.)
        remaining = self.road_length_m - station
        local = [_clip(ego['speed_m_s'] / CAP_M_S, 0., 1.),
            _clip(ego['acceleration_m_s2'] / 4.5), lane / max(1, self.lane_count - 1),
            remaining / (distance + remaining), self.speed_limit_m_s / CAP_M_S, previous / 4.5]
        for label in ('own', 'left', 'right'):
            for direction in ('front', 'rear'):
                other = neighbors[label][direction]
                local.extend([other['bumper_gap_m'] / 120.,
                    _clip(other['relative_speed_m_s'] / CAP_M_S), 1.] if other else [0., 0., 0.])
        local.extend([float(exists['left']), float(exists['right']), self.lane_count / 7.,
                      1., 0., max(0., 1. - (now - last_change) / 5.), *mask])
        # The route is reachable without a merge: original v2 unknown context,
        # with the current-state feasible ceiling retained in its own channel.
        local.extend([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., ceiling / 4.5, 0.])
        messages = [[0., 0., 0., 0., 0., 0., 0., 0.] for _ in range(6)]
        observation = local + [value for slot in messages for value in slot]
        if len(local) != 45 or len(observation) != 93 or any(not -1. <= value <= 1. for value in observation):
            raise ValueError('Invalid complete actor observation')
        return dict(observation=observation, schema=SCHEMA, source=SOURCE,
            lane_index=lane, mask=mask, neighbors=neighbors,
            acceleration_ceiling_m_s2=ceiling, acceleration_ceiling_bounds=ceiling_bounds,
            physical_acceleration_bounds=physical_bounds,
            physical_lane_change_mask=lane_change_active, delivered_message_count=0,
            messages_reason='single autonomous ego; background cars do not transmit',
            front_bumper_station_m=station, remaining_route_m=remaining)

    @staticmethod
    def filter_action(prediction, observation_result):
        """Original analytical speed/following guards; no hidden SUMO safety."""
        requested = _finite(prediction['requested_acceleration_m_s2'], 'requested acceleration')
        lane = prediction['lane_delta']
        if not -4.5 <= requested <= 2. or type(lane) is not int or lane not in (-1, 0, 1):
            raise ValueError('Invalid decoded actor command')
        bounds = dict(observation_result['acceleration_ceiling_bounds'],
                      **observation_result.get('physical_acceleration_bounds', {}))
        filtered = _clip(min(requested, *bounds.values()), -4.5, 2.)
        filtered_lane = lane if observation_result['mask'][lane + 1] else 0
        causes = [key for key, bound in bounds.items()
                  if requested > max(-4.5, bound) + 1e-7]
        if filtered_lane != lane:
            causes.append('lane_mask')
        return dict(prediction, filtered_acceleration_m_s2=filtered,
                    filtered_lane_delta=filtered_lane, causes=causes)


class FrozenPhysicalActor:
    """Checksum-locked exported best reference; CPU inference only, no optimizer."""
    def __init__(self, policy_path, contract_path=None):
        policy_path = Path(policy_path)
        actual = hashlib.sha256(policy_path.read_bytes()).hexdigest()
        if actual != EXPECTED_POLICY_SHA256:
            raise ValueError('Frozen reference policy checksum mismatch')
        contract_path = Path(contract_path) if contract_path else policy_path.with_name('contract.json')
        contract = json.loads(contract_path.read_text(encoding='utf-8'))
        expected = dict(schema=SCHEMA, observation_size=93, action_size=2,
            normalization='fixed analytic scaling; no running statistics',
            policy_sha256=EXPECTED_POLICY_SHA256, local_features=list(LOCAL_FEATURES),
            message_features=list(MESSAGE_FEATURES), mask_slice=[30, 33], message_slice=[45, 93],
            acceleration_m_s2=[-4.5, 2.], lane_delta=[-1, 0, 1])
        if any(contract.get(key) != value for key, value in expected.items()):
            raise ValueError('Frozen reference observation/action contract mismatch')
        import torch
        self.torch = torch
        self.actor = torch.jit.load(str(policy_path), map_location='cpu').eval()
        for parameter in self.actor.parameters():
            parameter.requires_grad_(False)
        self.policy_sha256 = actual
        self.contract_sha256 = hashlib.sha256(contract_path.read_bytes()).hexdigest()

    def act(self, observation):
        torch = self.torch
        values = torch.as_tensor(observation, dtype=torch.float32, device='cpu')
        if values.shape != (93,) or not torch.isfinite(values).all() or (values.abs() > 1.).any():
            raise ValueError('Actor requires one finite 93-field bounded observation')
        if values[31] != 1. or any(value not in (0., 1.) for value in values[30:33].tolist()):
            raise ValueError('Invalid explicit lane mask')
        with torch.inference_mode():
            output = self.actor(values.unsqueeze(0))
        if output.shape != (1, 2) or not torch.isfinite(output).all():
            raise ValueError('Invalid actor output')
        normalized, lane_float = map(float, output[0].tolist())
        if not -1. <= normalized <= 1. or lane_float not in (-1., 0., 1.):
            raise ValueError('Actor action contract violation')
        lane = int(lane_float)
        if values[30 + lane + 1] != 1.:
            raise ValueError('Actor violated its stored lane mask')
        return dict(normalized_acceleration=normalized,
                    requested_acceleration_m_s2=-4.5 + (normalized + 1.) * 3.25,
                    lane_delta=lane)
