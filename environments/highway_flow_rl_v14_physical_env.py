"""V14 merge environment with physics-driven controlled cars and SUMO shadows (no training).

User decision (November 19, 2026): "Yes, policy drives it" -- the learned policy drives a
physics car on the LA road model, while SUMO keeps the background traffic. Training stays in
SUMO; this environment is for running a trained policy with a physical vehicle.

One controller per car, as in v14, now with physics in the loop. Every 0.1 s, for each
physical car:
1. the v14 environment turns the policy's action into a filtered acceleration and a lane
   decision exactly as before (speed limit, following gap, lane end, comfort rate limit,
   lane-change mask);
2. those commands go to a `driver` (the physics side), never to SUMO: the wrapped TraCI
   connection intercepts setAcceleration and changeLane for physical cars;
3. the driver advances the physics car 0.1 s along a reference path built from the SUMO
   lanes (traffic/physical_drive.py) and returns its measured front-bumper pose and speed;
4. SUMO's copy of the car (its shadow) is placed exactly there (setSpeed + moveToXY with
   keepRoute 3, the pattern scripts/evaluate_gpudrive_i110.py used), and SUMO steps the
   background traffic, which reacts to where the physics car actually is.

SUMO reports no lateral speed for a moved vehicle, so the wrapped getLateralSpeed returns the
change of the shadow's lateral position over the last step (corrected across a lane line) at
the moment the environment reads it, before its lane-change bookkeeping uses it. Observations,
rewards, gates and the drain are v14's. After the scored horizon the shadow is released to
native SUMO for the drain, as every version drains, and the physics car is removed.
"""
from __future__ import annotations

import math

from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
from traffic.physical_drive import LaneNetwork, reference_path

DT = .1
SHADOW_KEEP_ROUTE = 3          # exact position, including lateral offset, within 100 m of the route
SHADOW_MATCH_THRESHOLD_M = 100.
MAX_SHADOW_ERROR_M = 1e-3
ARRIVAL_MARGIN_M = .1          # hand the final step to the route end to SUMO within this margin


class _ShadowVehicle:
    """TraCI vehicle domain that sends physical cars' commands to the driver instead of SUMO."""

    def __init__(self, env, domain):
        self._env, self._domain = env, domain

    def __getattr__(self, name):
        return getattr(self._domain, name)

    def _physical(self, vid):
        return vid in self._env.physical_agents and not self._env.physical_released

    def setAcceleration(self, vid, acceleration, duration):
        if self._physical(vid):
            self._env.physical_commands.setdefault(vid, {})['acceleration'] = float(acceleration)
            return None
        return self._domain.setAcceleration(vid, acceleration, duration)

    def changeLane(self, vid, index, duration):
        if self._physical(vid):
            source = self._env.states[vid]['lane_id']
            edge = source.rsplit('_', 1)[0]
            self._env.physical_maneuvers[vid] = dict(source_lane=source, target_lane=f'{edge}_{int(index)}',
                started_at=self._env.time, duration_s=self._env.lane_change_duration)
            return None
        return self._domain.changeLane(vid, index, duration)

    def changeLaneRelative(self, vid, indexOffset, duration):
        if self._physical(vid):
            raise RuntimeError('Physical cars change lanes only through the environment maneuver')
        return self._domain.changeLaneRelative(vid, indexOffset, duration)

    def getLateralSpeed(self, vid):
        """SUMO reports 0 for a moved car; use the change of its lateral position over the last step."""
        if not self._physical(vid):
            return self._domain.getLateralSpeed(vid)
        env, now = self._env, self._env.time
        cached = env.physical_lateral_speed.get(vid)
        if cached is not None and cached[0] == now:
            return cached[1]
        lane, lateral = self._domain.getLaneID(vid), float(self._domain.getLateralLanePosition(vid))
        previous = env.physical_lateral_previous.get(vid)
        speed = 0.
        if previous is not None and now > previous[2]:
            old_lane, old_lateral = previous[0], previous[1]
            old, new = env.graph.lanes.get(old_lane), env.graph.lanes.get(lane)
            if old and new and old_lane != lane and old['edge'] == new['edge'] and new['index'] != old['index']:
                # crossing a lane line: express the old offset relative to the new lane's center
                old_lateral -= math.copysign(.5 * (env.lanes.width(old_lane) + env.lanes.width(lane)),
                                             new['index'] - old['index'])
            speed = (lateral - old_lateral) / (now - previous[2])
        env.physical_lateral_previous[vid] = (lane, lateral, now)
        env.physical_lateral_speed[vid] = (now, speed)
        return speed


class _ShadowConnection:
    """TraCI connection whose simulationStep first advances the physics cars and places their shadows."""

    def __init__(self, env, connection):
        self._env, self._connection = env, connection
        self.vehicle = _ShadowVehicle(env, connection.vehicle)

    def __getattr__(self, name):
        return getattr(self._connection, name)

    def simulationStep(self, *args):
        self._env._advance_physical()
        result = self._connection.simulationStep(*args)
        self._env._check_shadows()
        return result


class HighwayFlowRLV14PhysicalEnv(HighwayFlowRLV14Env):
    def __init__(self, *args, driver, physical_agents=('agent_0',), **kwargs):
        super().__init__(*args, **kwargs)
        self.driver = driver
        self.physical_agents = frozenset(physical_agents)
        self.lanes = LaneNetwork(self.network)

    def reset(self, seed=None, options=None):
        self.physical_released = False
        self.physical_commands, self.physical_maneuvers = {}, {}
        self.physical_lateral_speed, self.physical_lateral_previous = {}, {}
        self.physical_placed, self.physical_log = {}, []
        result = super().reset(seed=seed, options=options)
        missing = sorted(a for a in self.physical_agents if a not in self.states)
        if missing:
            raise ValueError(f'Physical agents absent after reset: {missing}')
        self.driver.reset(self.lanes)
        for agent in sorted(self.physical_agents):
            self.driver.spawn(agent, self._shadow_pose(agent))
        self.conn = _ShadowConnection(self, self.conn)
        return result

    def _shadow_pose(self, agent):
        state = self.states[agent]
        return dict(time=self.time, x=state['x'], y=state['y'], angle=float(self.conn.vehicle.getAngle(agent)),
                    speed=state['speed'], acceleration=state['acceleration'], lane_id=state['lane_id'],
                    route=list(state['route']))

    def _advance_physical(self):
        if self.physical_released:
            return
        commands = {}
        for agent in sorted(self.physical_agents):
            state = self.states.get(agent)
            if state is None:
                continue
            maneuver = self.physical_maneuvers.get(agent)
            if maneuver and agent not in self.maneuvers:      # the environment ended or refused it
                maneuver = self.physical_maneuvers.pop(agent)
                maneuver = None
            acceleration = self.physical_commands.get(agent, {}).get('acceleration', 0.)
            path = reference_path(self.lanes, list(state['route']), state['lane_id'], state['x'], state['y'],
                                  state['speed'], maneuver=maneuver, now=self.time)
            commands[agent] = dict(acceleration=acceleration, path=path, lane_id=state['lane_id'],
                                   maneuver=None if maneuver is None else dict(maneuver))
        if not commands:
            return
        background = [dict(id=vid, x=s['x'], y=s['y'], angle=float(s.get('angle', self.conn.vehicle.getAngle(vid))),
                           length=s['length'], width=float(s.get('width', 1.8)))
                      for vid, s in self.states.items() if vid not in commands]
        poses = self.driver.advance(commands, DT, time=self.time, background=background)
        for agent, pose in poses.items():
            speed = max(0., float(pose['speed']))
            self.conn.vehicle.setSpeed(agent, speed)
            remaining = self._route_remaining(agent)
            arriving = remaining is not None and remaining <= speed * DT + ARRIVAL_MARGIN_M
            if arriving:
                # A placed vehicle never arrives in SUMO, so its last step to the route end is
                # SUMO's own move at the physics speed; the vehicle then leaves the network.
                self.physical_placed.pop(agent, None)
            else:
                self.conn.vehicle.moveToXY(agent, '', -1, float(pose['x']), float(pose['y']),
                                           angle=float(pose['angle']), keepRoute=SHADOW_KEEP_ROUTE,
                                           matchThreshold=SHADOW_MATCH_THRESHOLD_M)
                self.physical_placed[agent] = (float(pose['x']), float(pose['y']))
            self.physical_log.append(dict(time=round(self.time + DT, 6), agent=agent, arriving=arriving, command=dict(
                acceleration=commands[agent]['acceleration'], maneuver=commands[agent]['maneuver']), pose=pose))
        for agent in set(commands) - set(poses):
            raise RuntimeError(f'Driver returned no pose for {agent}')

    def _route_remaining(self, agent):
        """Metres from the shadow's front bumper to its route end when it is on the last route edge."""
        state = self.states[agent]
        lane = self.lanes.lane(state['lane_id'])
        if lane.getEdge().getID() != state['route'][-1]:
            return None
        return max(0., lane.getLength() - float(state['lane_position']))

    def _check_shadows(self):
        if self.physical_released:
            return
        present = set(self.conn.vehicle.getIDList())
        for agent in sorted(self.physical_agents & set(self.driver_cars())):
            if agent not in present:
                self.driver.remove(agent)
                self.physical_placed.pop(agent, None)
        for agent, (x, y) in list(self.physical_placed.items()):
            sx, sy = self.conn.vehicle.getPosition(agent)
            if math.hypot(sx - x, sy - y) > MAX_SHADOW_ERROR_M:
                raise RuntimeError(f'SUMO moved the shadow of {agent} away from the physics pose')

    def driver_cars(self):
        return self.driver.active()

    def finish_accounting(self):
        self.physical_released = True
        present = set(self.conn.vehicle.getIDList())
        for agent in sorted(self.physical_agents & set(self.driver_cars())):
            self.driver.remove(agent)
            if agent in present:
                self.conn.vehicle.setSpeed(agent, -1)     # end the shadow's held speed; SUMO drives the drain
        self.physical_placed.clear()
        return super().finish_accounting()

    def close(self):
        try:
            return super().close()
        finally:
            if isinstance(getattr(self, 'conn', None), _ShadowConnection):
                self.conn = self.conn._connection

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v14_physical', physical_agents=sorted(self.physical_agents),
                      physical_driver=self.driver.describe(), physical_steps=len(self.physical_log),
                      shadow=dict(keep_route=SHADOW_KEEP_ROUTE, match_threshold_m=SHADOW_MATCH_THRESHOLD_M,
                                  max_error_m=MAX_SHADOW_ERROR_M))
        return result


class IdealDriver:
    """Executes commands exactly: speed integrates the acceleration, the car rides the path.

    A stand-in for the physics side in tests: with it the loop should reproduce SUMO's own
    execution of the same commands, apart from the lane-change shape.
    """

    def reset(self, lanes):
        self.cars = {}

    def spawn(self, agent, pose):
        self.cars[agent] = dict(pose)

    def remove(self, agent):
        self.cars.pop(agent, None)

    def active(self):
        return sorted(self.cars)

    def describe(self):
        return dict(kind='ideal', note='exact acceleration, path-following kinematics; not physics')

    def advance(self, commands, dt, time=None, background=()):
        out = {}
        for agent, command in commands.items():
            car = self.cars[agent]
            speed = max(0., car['speed'] + command['acceleration'] * dt)
            distance = .5 * (car['speed'] + speed) * dt
            path = command['path']
            x, y, heading = path[0][0], path[0][1], None
            for (ax, ay), (bx, by) in zip(path, path[1:]):
                seg = math.hypot(bx - ax, by - ay)
                if seg < 1e-9:
                    continue
                heading = math.degrees(math.atan2(bx - ax, by - ay)) % 360
                if distance <= seg:
                    x, y = ax + (bx - ax) * distance / seg, ay + (by - ay) * distance / seg
                    break
                distance -= seg
                x, y = bx, by
            car.update(x=x, y=y, speed=speed, acceleration=(speed - car['speed']) / dt,
                       angle=heading if heading is not None else car['angle'])
            out[agent] = dict(x=x, y=y, angle=car['angle'], speed=speed, acceleration=car['acceleration'])
        return out
