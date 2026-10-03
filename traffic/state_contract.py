"""Versioned SUMO state boundary; meters, seconds, Z-up, front-bumper poses."""
from dataclasses import asdict, dataclass
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def prim_path(episode, vehicle):
    if not all(isinstance(s, str) and s for s in (episode, vehicle)):
        raise ValueError('Nonempty episode and vehicle IDs required')
    return '/World/Episodes/e' + episode.encode().hex() + '/Vehicles/v' + vehicle.encode().hex()


def decode_prim_path(path):
    parts = path.split('/')
    if len(parts) != 6 or parts[1:3] != ['World', 'Episodes'] or parts[4] != 'Vehicles' or not parts[3].startswith('e') or not parts[5].startswith('v'):
        raise ValueError('Invalid vehicle prim path')
    result = bytes.fromhex(parts[3][1:]).decode(), bytes.fromhex(parts[5][1:]).decode()
    if prim_path(*result) != path:
        raise ValueError('Noncanonical vehicle prim path')
    return result


@dataclass(frozen=True)
class VehicleState:
    vehicle_id: str
    x_m: float
    y_m: float
    heading_deg: float
    speed_mps: float
    length_m: float = 5.
    width_m: float = 2.

    def __post_init__(self):
        if not isinstance(self.vehicle_id, str) or not self.vehicle_id:
            raise ValueError('Vehicle ID required')
        if not all(finite(v) for k, v in asdict(self).items() if k != 'vehicle_id'):
            raise ValueError('Finite numeric state required')
        if self.speed_mps < 0 or min(self.length_m, self.width_m) <= 0 or not 0 <= self.heading_deg < 360:
            raise ValueError('Invalid speed, dimensions or heading')

    def center_pose(self):
        yaw = 90 - self.heading_deg
        angle = math.radians(yaw)
        return ((self.x_m - self.length_m / 2 * math.cos(angle),
                 self.y_m - self.length_m / 2 * math.sin(angle), 0.), yaw)


@dataclass(frozen=True)
class Snapshot:
    episode_id: str
    step: int
    time_s: float
    dt_s: float
    vehicles: tuple[VehicleState, ...]
    schema_version: int = 1
    authority: str = 'SUMO'
    frame: str = 'meters:+X-east,+Y-north,+Z-up;origin=SUMO-network'
    pose_reference: str = 'front-bumper-center'

    def __post_init__(self):
        if self.schema_version != 1 or type(self.schema_version) is not int or self.authority != 'SUMO' or self.frame != 'meters:+X-east,+Y-north,+Z-up;origin=SUMO-network' or self.pose_reference != 'front-bumper-center':
            raise ValueError('Unsupported state contract')
        if not isinstance(self.episode_id, str) or not self.episode_id or type(self.step) is not int or self.step < 0:
            raise ValueError('Episode ID and nonnegative integer step required')
        if not finite(self.time_s) or not finite(self.dt_s) or self.dt_s <= 0 or abs(self.time_s-self.step*self.dt_s) > 1e-7:
            raise ValueError('Episode clock must equal step * dt')
        if not isinstance(self.vehicles, tuple) or not all(isinstance(v, VehicleState) for v in self.vehicles):
            raise ValueError('Vehicle states must be a tuple')
        if len({v.vehicle_id for v in self.vehicles}) != len(self.vehicles):
            raise ValueError('Duplicate vehicle identity')


def from_dict(record):
    fields = dict(record)
    fields['vehicles'] = tuple(VehicleState(**v) for v in fields['vehicles'])
    return Snapshot(**fields)


class StateStream:
    """Consume live or replay snapshots; explicit reset required between episodes."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.episode = None
        self.step = -1
        self.dt = None
        self.active = set()
        self.retired = set()

    def accept(self, snapshot):
        if not isinstance(snapshot, Snapshot):
            raise ValueError('Validated Snapshot required')
        if self.episode is not None and (snapshot.episode_id != self.episode or snapshot.dt_s != self.dt):
            raise ValueError('Reset stream before changing episode or timestep')
        if snapshot.step <= self.step:
            raise ValueError('Duplicate or out-of-order snapshot')
        ids = {v.vehicle_id for v in snapshot.vehicles}
        if ids & self.retired:
            raise ValueError('Vehicle ID reused within an episode')
        result = dict(spawned=sorted(ids-self.active), updated=sorted(ids & self.active),
                      removed=sorted(self.active-ids))
        self.retired |= self.active-ids
        self.active, self.step, self.episode, self.dt = ids, snapshot.step, snapshot.episode_id, snapshot.dt_s
        return result
