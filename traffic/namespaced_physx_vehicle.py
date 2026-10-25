"""Bind an already composed Factory car without authoring another world.

Import is simulator-free. Binding requires Isaac's USD schemas/interface, but
does not start physics, change gravity, reset a body or write an actuator. The
owner creates one DriverControlGate/contact monitor per returned vehicle and
binds its native view after the single scene has initialized.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
import math
import re

from traffic.physx_vehicle import PhysxVehicle


WHEEL_NAMES = ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel')
PHYSICS_SCENE = '/World/Physics/Scene'


@dataclass(frozen=True)
class VehicleBindingPaths:
    vehicle_id: str
    vehicle: str
    chassis: str
    wheels: tuple[str, ...]
    physics_scene: str = PHYSICS_SCENE


def vehicle_binding_paths(path):
    """Accept a stable vehicle component root or its chassis, never a wildcard."""
    if not isinstance(path, str):
        raise ValueError('Vehicle path must be a string')
    root = path.removesuffix('/Chassis')
    match = re.fullmatch(r'/World/Vehicles/vehicle_([A-Za-z0-9][A-Za-z0-9_]*)', root)
    if not match:
        raise ValueError('Expected a stable /World/Vehicles/vehicle_<id> component')
    chassis = root + '/Chassis'
    return VehicleBindingPaths(match[1], root, chassis,
                               tuple(chassis + '/' + name for name in WHEEL_NAMES))


class NamespacedPhysxVehicle(PhysxVehicle):
    """The frozen adapter's actuation/readback methods, with explicit bindings.

    Deliberately does not call PhysxVehicle.__init__: that constructor creates
    the original single-car world. This constructor only validates and binds.
    Native support/contact/multi-car behavior still requires runtime evidence.
    """
    def __init__(self, stage, path, metadata_source, physics_hz=120):
        if type(physics_hz) is not int or physics_hz != 120:
            raise ValueError('Physical fleet v1 requires exactly 120 Hz')
        paths = vehicle_binding_paths(path)
        if not isinstance(metadata_source, dict):
            raise ValueError('metadata_source must be the source vehicle metadata dict')
        for key, expected in (('wheelbase_m', 3.2), ('track_m', 1.6)):
            value = metadata_source.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value != expected:
                raise ValueError('Unsupported Factory geometry: ' + key)
        from pxr import PhysxSchema, UsdGeom, UsdPhysics
        import omni.physx

        root = stage.GetPrimAtPath(paths.vehicle)
        body = stage.GetPrimAtPath(paths.chassis)
        if not root or root.IsInstance() or root.IsInstanceProxy() or root.IsInstanceable():
            raise ValueError('Vehicle must be a non-instanceable component')
        if root.GetCustomDataByKey('vehicle_id') != paths.vehicle_id:
            raise ValueError('Vehicle identity metadata disagrees with its path')
        if root.GetCustomDataByKey('movement_authority') != 'isaac_physx':
            raise ValueError('Vehicle motion authority must be PhysX')
        if not body or not body.IsActive() or not body.IsLoaded() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Missing active chassis rigid body')
        rigid = UsdPhysics.RigidBodyAPI(body)
        owners = list(map(str, rigid.GetSimulationOwnerRel().GetTargets()))
        if (rigid.GetRigidBodyEnabledAttr().Get() is not True
                or rigid.GetKinematicEnabledAttr().Get() is not False
                or owners not in ([], [paths.physics_scene])):
            raise ValueError('Chassis must be enabled dynamic with the sole canonical physics owner')
        if not body.HasAPI(PhysxSchema.PhysxVehicleAPI):
            raise ValueError('Chassis lacks PhysxVehicleAPI')
        scenes = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'PhysicsScene']
        if scenes != [paths.physics_scene]:
            raise ValueError('Fleet must share exactly one /World/Physics/Scene')
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.0 or UsdGeom.GetStageUpAxis(stage) != 'Z':
            raise ValueError('Fleet requires Z-up metre units')
        scene = UsdPhysics.Scene(stage.GetPrimAtPath(paths.physics_scene))
        gravity = scene.GetGravityMagnitudeAttr().Get()
        if gravity is None or not math.isfinite(gravity) or abs(gravity - 9.81) > 1e-5:
            raise ValueError('Unexpected shared gravity; binding never changes scene settings')
        controllers = []
        for wheel_path in paths.wheels:
            wheel = stage.GetPrimAtPath(wheel_path)
            if (not wheel or not wheel.IsActive() or not wheel.IsLoaded()
                    or not wheel.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI)
                    or not wheel.HasAPI(PhysxSchema.PhysxVehicleWheelControllerAPI)):
                raise ValueError('Missing independently controllable wheel: ' + wheel_path)
            controllers.append(PhysxSchema.PhysxVehicleWheelControllerAPI(wheel))
        mass = UsdPhysics.MassAPI(body)
        mass_value = mass.GetMassAttr().Get()
        inertia = mass.GetDiagonalInertiaAttr().Get()
        center = mass.GetCenterOfMassAttr().Get()
        if (mass_value is None or not math.isfinite(mass_value) or mass_value <= 0
                or inertia is None or len(inertia) != 3 or any(not math.isfinite(v) or v <= 0 for v in inertia)
                or center is None or len(center) != 3 or not all(math.isfinite(v) for v in center)):
            raise ValueError('Explicit finite positive chassis mass/inertia required')
        self.stage, self.binding_paths = stage, paths
        self.path, self.wheel_paths = paths.chassis, list(paths.wheels)
        self.physics_scene_path = paths.physics_scene
        self.controllers = controllers
        self.body = rigid
        self.physx = omni.physx.get_physx_interface()
        self.wheelbase_m, self.track_m = 3.2, 1.6
        self._native_state_view = None
        self.scene_composition = dict(binding='existing referenced vehicle; no scene authoring',
                                      paths=asdict(paths), native_fleet_validated=False)
        self.metadata = deepcopy(metadata_source)
        self.metadata.update(vehicle_id=paths.vehicle_id, mass_kg=float(mass_value),
            inertia_kg_m2=list(inertia), center_of_mass_from_root_m=list(center),
            scene_composition=deepcopy(self.scene_composition), isaac_lab_validated=False,
            native_fleet_validated=False, physics_hz=physics_hz)


def bind_existing_vehicle(stage, path, metadata_source, physics_hz=120):
    """Return an independent adapter; caller later calls bind_native_state(view)."""
    return NamespacedPhysxVehicle(stage, path, metadata_source, physics_hz)
