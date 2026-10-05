"""Vehicle rigid-contact reporting, separate from raycast tire support.

Configure before attaching the stage, then sample immediately after each
simulate/fetch_results pair. Native buffers expire after one physics tick;
only copied JSON-compatible values are retained. No global physics settings or
extra simulation steps are used here.
"""
from __future__ import annotations

import math


def _vehicle_member(path, vehicle_path):
    return path == vehicle_path or path.startswith(vehicle_path+'/')


def classify_vehicle_contacts(headers, data, *, vehicle_path, stage_id, tick,
                              path_converter, event_type_names):
    """Copy current-step FOUND/PERSIST vehicle contacts; ignore LOST/other stages.

    ``path_converter`` and ``event_type_names`` are supplied by the runtime so
    classification and buffer-lifetime handling can be tested without Isaac.
    Tire support is not inferred from these rigid-body contacts.
    """
    if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
        raise ValueError('Contact tick must be a nonnegative integer')
    events = []
    for header in headers:
        name = event_type_names.get(header.type)
        if name is None or header.stage_id != stage_id:
            continue
        paths = {key: str(path_converter(getattr(header, key)))
                 for key in ('actor0', 'actor1', 'collider0', 'collider1')}
        if not any(_vehicle_member(path, vehicle_path) for path in paths.values()):
            continue
        offset, count = header.contact_data_offset, header.num_contact_data
        if offset < 0 or count < 0 or offset+count > len(data):
            raise RuntimeError('PhysX contact report contains an invalid data range')
        points = []
        for index in range(offset, offset+count):
            contact = data[index]
            point = {key: [float(v) for v in getattr(contact, key)]
                     for key in ('position', 'normal', 'impulse')}
            point['separation_m'] = float(contact.separation)
            values = point['position']+point['normal']+point['impulse']+[point['separation_m']]
            if any(len(point[key]) != 3 for key in ('position', 'normal', 'impulse')) or not all(math.isfinite(v) for v in values):
                raise RuntimeError('PhysX contact data are malformed or non-finite')
            points.append(point)
        events.append(dict(tick=tick, stage_id=int(stage_id), event_type=name,
                           **paths, contact_count=int(count), contacts=points))
    return events


class VehicleContactMonitor:
    """Collect undesired rigid contacts involving one vehicle and its colliders.

    In the current Factory car, tires use raycast suspension support and rigid
    wheel-to-ground collisions are filtered. Therefore a reported chassis/ground
    contact is not normal support and must not be exempted. Future tire models
    require an explicit allowlist and fresh positive-control validation.
    """
    def __init__(self, stage, vehicle_path):
        if not isinstance(vehicle_path, str) or not vehicle_path.startswith('/') or vehicle_path == '/' or vehicle_path.endswith('/'):
            raise ValueError('vehicle_path must identify an absolute non-root prim')
        from pxr import PhysxSchema, PhysicsSchemaTools, UsdPhysics, UsdUtils
        from omni.physx.bindings._physx import ContactEventType

        prim = stage.GetPrimAtPath(vehicle_path)
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Contact reporting requires the vehicle rigid-body prim')
        report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        if not report:
            raise RuntimeError('Unable to enable vehicle contact reports')
        report.CreateThresholdAttr().Set(0.0)
        if report.GetThresholdAttr().Get() != 0.0:
            raise RuntimeError('Unable to set zero contact-report threshold')
        self.vehicle_path = vehicle_path
        self.stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.events = []
        self._last_tick = None
        self._path_converter = PhysicsSchemaTools.intToSdfPath
        self._event_type_names = {
            ContactEventType.CONTACT_FOUND: 'found',
            ContactEventType.CONTACT_PERSIST: 'persist',
        }
        # No prim/schema/stage object is saved on this monitor.

    def sample(self, sim, tick):
        """Read this physics tick exactly once and copy events before next step."""
        if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0 or (self._last_tick is not None and tick <= self._last_tick):
            raise ValueError('Contact ticks must strictly increase')
        headers, data = sim.get_contact_report()
        events = classify_vehicle_contacts(
            headers, data, vehicle_path=self.vehicle_path, stage_id=self.stage_id,
            tick=tick, path_converter=self._path_converter,
            event_type_names=self._event_type_names)
        self._last_tick = tick
        self.events.extend(events)
        return events
