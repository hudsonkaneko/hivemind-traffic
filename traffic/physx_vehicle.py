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

    def __init__(self, stage, physics_hz=120, *, scene_directory=None):
        import omni.physx
        from traffic.physics_session import vehicle_factory
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        Factory = vehicle_factory()

        self.stage = stage
        self.scene_composition = None
        if scene_directory is not None:
            from usd.physical_scene import PATHS, compose_factory_vehicle
            self.scene_composition = compose_factory_vehicle(stage, Factory, physics_hz, scene_directory)
            self.path, self.wheel_paths = PATHS.chassis, PATHS.wheels
            self.physics_scene_path = PATHS.physics
        else:
            # Preserved low-level fixtures and the separate Leatherback/Lab work
            # retain their original namespace and evidence contract.
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
            from traffic.wheel_geometry import author_explicit_wheel_axes
            author_explicit_wheel_axes(stage, self.wheel_paths)
            self.physics_scene_path = '/World/PhysicsScene'
        self.controllers = [PhysxSchema.PhysxVehicleWheelControllerAPI(stage.GetPrimAtPath(p))
                            for p in self.wheel_paths]
        self.physx = omni.physx.get_physx_interface()
        self.body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(self.path))
        # Factory ordering FL, FR, RL, RR: Z-up/X-forward => left is +Y.
        self.wheelbase_m, self.track_m = 3.2, 1.6
        scene = UsdPhysics.Scene(stage.GetPrimAtPath(self.physics_scene_path))
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
            wheel_geometry='X-axis cylinder with explicit local Z+90deg; lateral Y axle',
            gravity_m_s2=9.81, world_frame='right-handed X forward, Y left, Z up; meters',
            pose_reference='chassis prim origin; NOT SUMO front bumper',
            quaternion_order='xyzw', movement_authority='isaac_physx',
            tire_contact='Factory raycast suspension/tire model; not a deformable tire',
            tire_tarmac_friction=0.75, rigid_static_friction=0.9, rigid_dynamic_friction=0.7,
            suspension_spring_n_m=45000, suspension_damper_n_s_m=4500, suspension_travel_m=0.2,
            ground='Infinite collision plane; sample visible ground is only 30 x 30 m',
            sumo_required=False, lidar_enabled=False, isaac_lab_validated=False)
        if self.scene_composition:
            self.metadata['scene_composition'] = self.scene_composition

    def apply(self, control, drive_torque_nm, brake_torque_nm):
        angles = ackermann_steering(control.steering_rad, self.wheelbase_m, self.track_m)
        drive, brake = wheel_torques(control, drive_torque_nm, brake_torque_nm)
        for i, controller in enumerate(self.controllers):
            controller.GetDriveTorqueAttr().Set(drive[i])
            controller.GetBrakeTorqueAttr().Set(brake[i])
            controller.GetSteerAngleAttr().Set(angles[i] if i < 2 else 0.0)
        return dict(**asdict(control), wheel_drive_nm=list(drive),
                    wheel_brake_nm=list(brake), front_steer_rad=list(angles))

    def bind_native_state(self, simulation_view):
        """Bind a checked native velocity view for render-decoupled readback."""
        if getattr(self,'_native_state_view',None) is not None:
            raise RuntimeError('Native ego state already bound')
        view=simulation_view.create_rigid_body_view(self.path)
        if list(view.prim_paths)!=[self.path]:
            raise RuntimeError('Native ego state identity mismatch')
        self._native_state_view=view
        return dict(velocity_source='native rigid body tensor view',body_path=self.path)

    def state(self, *, refresh_usd=True):
        """Read native pose/wheels and either bound native or USD velocity.

        Without a native velocity view, skip refresh only after an owned session
        published this tick. A bound view permits render-decoupled observations.
        Defaults preserve standalone/headless probe behavior.
        """
        if type(refresh_usd) is not bool:
            raise ValueError('USD refresh flag must be boolean')
        from omni.physx.bindings._physx import VEHICLE_WHEEL_STATE_IS_ON_GROUND
        state = self.physx.get_rigidbody_transformation(self.path)
        if not state['ret_val']:
            raise RuntimeError('PhysX rigid-body state unavailable')
        position, q = list(state['position']), list(state['rotation'])
        x, y, z, w = q
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        upright = 1-2*(x*x+y*y)
        if refresh_usd:
            self.physx.update_transformations(False, True, True, False)
        view=getattr(self,'_native_state_view',None)
        if view is not None:
            values=view.get_velocities()
            if hasattr(values,'detach'):values=values.detach().cpu()
            if hasattr(values,'numpy'):values=values.numpy()
            if values.shape!=(1,6):raise RuntimeError('Malformed native ego velocity')
            velocity=list(map(float,values[0,:3]))
        else:
            velocity = list(self.body.GetVelocityAttr().Get())
        values = position + q + velocity
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Non-finite PhysX state')
        return dict(position_m=position, quaternion_xyzw=q, velocity_m_s=velocity,
                    speed_m_s=math.hypot(velocity[0], velocity[1]),
                    yaw_rad=yaw, upright_z=upright,
                    wheel_on_ground=[bool(self.physx.get_wheel_state(p)[VEHICLE_WHEEL_STATE_IS_ON_GROUND])
                                     for p in self.wheel_paths])
