"""Shared 42-feature deployment on full-road background traffic, CPU only."""
from dataclasses import replace
import math

from environments.lane_following_contract import LaneFollowingTask
from environments.loop_hazard_contract import HazardLaneTask, hazard_observation, safe_target_speed
from traffic.road_hazards import MeasuredObject, number


class FullLoopHazardTask(HazardLaneTask):
    """Hazard completion evidence remains separate from background actor inputs.

    The actor still receives nearest-two slots. No special hazard priority is
    introduced, and masking a hazard by nearer traffic is explicitly recorded.
    This task does not turn the kinematic backgrounds into learned vehicles.
    """
    def __init__(self, *args, **kwargs):
        self.background_objects = ()
        self.background_time_s = 0.
        self.last_deployment_observation = {}
        super().__init__(*args, **kwargs)

    def set_background_objects(self, objects, *, now_s):
        objects = tuple(objects)
        now = number(now_s, 'background time')
        if (now < 0 or len(objects) > 80 or any(not isinstance(o, MeasuredObject) for o in objects)
                or len({o.object_id for o in objects}) != len(objects)):
            raise ValueError('Expected a complete unique measured background frame')
        self.background_objects, self.background_time_s = objects, now

    def _relative(self, state, tick):
        relative = super()._relative(state, tick)
        ego, base = self._frame(state, tick, relative)
        speed, details = safe_target_speed(ego, base, (*self._objects, *self.background_objects),
                                           now_s=tick/self.config.physics_hz)
        self.last_background_target = details
        return replace(relative, target_speed_m_s=min(relative.target_speed_m_s, speed))

    def _observation(self, state, applied, tick, relative):
        base = LaneFollowingTask._observation(self, state, applied, tick, relative)
        ego, _ = self._frame(state, tick, relative)
        now = tick/self.config.physics_hz
        frame = hazard_observation(ego, base, (*self._objects, *self.background_objects), now_s=now)
        if (not frame.healthy or abs(self._objects_time-now) > 1/120+1e-9
                or abs(self.background_time_s-now) > 1/120+1e-9):
            raise ValueError('Missing/stale measured deployment frame')
        self.last_hazard_observation = frame
        self.last_deployment_observation = dict(
            selected_ids=list(frame.selected_ids), background_count=len(self.background_objects),
            hazard_ids=[o.object_id for o in self._objects],
            masked_hazard_ids=[o.object_id for o in self._objects if o.object_id not in frame.selected_ids],
            slots=2, selection='nearest squared chassis distance then identity; no hazard priority',
            source='current measured native pose/velocity, privileged simulator state',
            limitation='background traffic adds distribution shift beyond one-hazard training')
        return frame.vector


def background_settings():
    from experiments.showcase_controls import defaults
    return {**defaults(), 'traffic_count': 12, 'traffic_mph': 5., 'ego_mph': 3./.44704}


def background_specs():
    """Eighty fixed pool slots; selected twelve remain on adjacent lanes.

    No same-lane nonreactive rear traffic can push the 3 m/s evaluated car during
    a stop. Initial active slots are fixed by unchanged LiveTraffic selection.
    """
    result = []
    for index in range(80):
        lane = 1 + index % 3
        result.append(dict(vehicle_id=f'background_{index:03d}', lane=lane,
            station_m=40. + (index//3)*(math.tau*500/27),
            speed_m_s=5.*.44704*(500+3.7*lane)/500))
    return result
