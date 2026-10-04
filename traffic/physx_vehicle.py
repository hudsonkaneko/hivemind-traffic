"""Experimental full-size PhysX vehicle adapter; import only inside Isaac Sim.

Uses the installed NVIDIA Factory sample without modifying or vendoring it.
This is a compatibility spike, not a calibrated highway vehicle or Lab asset.
"""
from dataclasses import asdict
import hashlib
import math
from pathlib import Path

from traffic.driver_control import ackermann_steering, wheel_torques


class PhysxVehicle:
    """One pose authority: PhysX. Runtime commands touch wheel actuators only."""

    def __init__(self, stage, physics_hz=120):
        import omni.physx
        from traffic.physics_session import vehicle_factory
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        Factory = vehicle_factory()

        self.stage = stage
        root = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(root.GetPrim())
        paths, wheels = [], []
        Factory.create4WheeledCarsScenario(
            stage, 1.0, 1, driveMode=Factory.DRIVE_NONE,
            axes=Factory.AxesIndices(2, 0, 1), timeStepsPerSecond=physics_hz,
            createCollisionShapesForWheels=True, vehiclePathsOut=paths,
            wheelAttachmentPathsOut=wheels)
        self.path = paths[0]
        self.wheel_paths = wheels[0]
        self.controllers = [PhysxSchema.PhysxVehicleWheelControllerAPI(stage.GetPrimAtPath(p))
                            for p in self.wheel_paths]
        self.physx = omni.physx.get_physx_interface()
        self.body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(self.path))
        # Factory ordering FL, FR, RL, RR: Z-up/X-forward => left is +Y.
        self.wheelbase_m, self.track_m = 3.2, 1.6
        scene = UsdPhysics.Scene(stage.GetPrimAtPath('/World/PhysicsScene'))
        scene.GetGravityMagnitudeAttr().Set(9.81)
        mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(self.path))
        self.metadata = dict(
            model='installed omni.physx.vehicle Factory DRIVE_NONE',
            factory_sha256=hashlib.sha256(Path(Factory.__file__).read_bytes()).hexdigest(),
            mass_kg=mass.GetMassAttr().Get(),
            inertia_kg_m2=list(mass.GetDiagonalInertiaAttr().Get()),
            center_of_mass_from_root_m=list(mass.GetCenterOfMassAttr().Get()),
            chassis_collision_dimensions_m=[4.8, 1.8, 1.4],
            wheelbase_m=self.wheelbase_m, track_m=self.track_m, wheel_radius_m=0.35,
            gravity_m_s2=9.81, world_frame='right-handed X forward, Y left, Z up; meters',
            pose_reference='chassis prim origin; NOT SUMO front bumper',
            quaternion_order='xyzw', movement_authority='isaac_physx',
            tire_contact='Factory raycast suspension/tire model; not a deformable tire',
            tire_tarmac_friction=0.75, rigid_static_friction=0.9, rigid_dynamic_friction=0.7,
            suspension_spring_n_m=45000, suspension_damper_n_s_m=4500, suspension_travel_m=0.2,
            ground='Infinite collision plane; sample visible ground is only 30 x 30 m',
            sumo_required=False, lidar_enabled=False, isaac_lab_validated=False)

    def apply(self, control, drive_torque_nm, brake_torque_nm):
        angles = ackermann_steering(control.steering_rad, self.wheelbase_m, self.track_m)
        drive, brake = wheel_torques(control, drive_torque_nm, brake_torque_nm)
        for i, controller in enumerate(self.controllers):
            controller.GetDriveTorqueAttr().Set(drive[i])
            controller.GetBrakeTorqueAttr().Set(brake[i])
            controller.GetSteerAngleAttr().Set(angles[i] if i < 2 else 0.0)
        return dict(**asdict(control), wheel_drive_nm=list(drive),
                    wheel_brake_nm=list(brake), front_steer_rad=list(angles))

    def state(self):
        from omni.physx.bindings._physx import VEHICLE_WHEEL_STATE_IS_ON_GROUND
        state = self.physx.get_rigidbody_transformation(self.path)
        if not state['ret_val']:
            raise RuntimeError('PhysX rigid-body state unavailable')
        position, q = list(state['position']), list(state['rotation'])
        x, y, z, w = q
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        upright = 1-2*(x*x+y*y)
        self.physx.update_transformations(False, True, True, False)
        velocity = list(self.body.GetVelocityAttr().Get())
        values = position + q + velocity
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Non-finite PhysX state')
        return dict(position_m=position, quaternion_xyzw=q, velocity_m_s=velocity,
                    speed_m_s=math.hypot(velocity[0], velocity[1]),
                    yaw_rad=yaw, upright_z=upright,
                    wheel_on_ground=[bool(self.physx.get_wheel_state(p)[VEHICLE_WHEEL_STATE_IS_ON_GROUND])
                                     for p in self.wheel_paths])
