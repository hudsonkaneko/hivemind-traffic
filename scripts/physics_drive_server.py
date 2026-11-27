"""Isaac Sim physics server: a PhysX Americano driven by policy commands on the LA road model.

Run only by traffic/physics_link.RemoteDriver (under Isaac Sim's python); it connects back to
the SUMO/policy process. User decision (November 19, 2026): "Yes, policy drives it".

What moves what:
- the physics car (the Americano v14 SimReady package, provisional physics) is moved only by
  PhysX: every physics tick (120 Hz) traffic/physical_controller.py turns the current policy
  command (acceleration, reference path) into drive torque, brake torque and steering on its
  wheels. Its pose and velocity are written once, at spawn (a reset, as ADR 001 allows);
- the road is la_10_110_flat.usdc, referenced unchanged. Its own physics scene is switched off in
  this stage so the car's scene is the only one; the freeway decks join the car package's
  ground-surface collision group (so wheel collision shapes leave contact to the suspension
  queries), and the car's tire friction table gets the model's PhysAsphalt with the same value
  as the package's tarmac (0.75). The package's infinite ground plane is switched off;
- background cars are SUMO's: drawn at SUMO's front-bumper poses each 0.1 s, with height and
  pitch from the model's lane surface under their front and rear. They are visual only (no
  colliders); SUMO checks collisions with the physics car's shadow.

Each 'advance' (one 0.1 s decision) first renders the frame for its start time, when the
physics car's pose equals its SUMO shadow's and the background is SUMO's for that time, then
runs 12 physics ticks and replies with the new front-bumper pose, forward speed and telemetry.

--controller picks the low-level controller version (traffic/physical_controller.CONTROLLERS; plan v15
"Physics gap proposal", decision 11). 'stage5', the default, is the controller of every earlier drive. For
the versions that take the grade from the road model ('c1', 'c2') the server gives the controller, at
every tick, road_grade: the sine of the slope between the model's lane surface under the front and the
rear axle, both taken speed * grade_preview_s ahead along the heading. The wheels' moment of inertia is
read from the package (physxVehicleWheel:moi) for the versions that use it in the feedforward.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic import physics_link  # noqa: E402
from traffic.physical_controller import (CONTROLLERS, LowLevelController, VehicleSpec, controller_gains,  # noqa: E402
                                         cornering_terms)

PHYSICS_HZ = 120
DECISION_S = .1
STEPS_PER_DECISION = 12
ROAD = '/World/Environment/RoadModel'
TRAFFIC = '/World/Traffic'
CAMERA = '/World/Cameras/Chase'
POOL = 64
ASPHALT_FRICTION = .75          # the package's own tarmac value in its WinterTireFrictionTable


def quaternion(yaw, pitch):
    """(w, x, y, z): pitch (nose up positive) about the body's y axis, then yaw about world z."""
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(-pitch / 2), math.sin(-pitch / 2)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


def rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # v' = q v q*
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


class Surface:
    """scripts/render_i110_merge_replay.surface_elevation, vectorised with numpy."""

    def __init__(self, surface):
        import numpy as np
        self.np = np
        self.lanes = []
        for lane in surface.values():
            p = np.asarray(lane['points'], dtype=float)
            self.lanes.append((p[:-1], p[1:] - p[:-1]))

    def height(self, x, y, limit=4.):
        np = self.np
        best = []
        for a, d in self.lanes:
            n = np.maximum(d[:, 0] ** 2 + d[:, 1] ** 2, 1e-12)
            u = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / n, 0., 1.)
            dist2 = (x - a[:, 0] - u * d[:, 0]) ** 2 + (y - a[:, 1] - u * d[:, 1]) ** 2
            i = int(np.argmin(dist2))
            best.append((math.sqrt(float(dist2[i])), float(a[i, 2] + u[i] * d[i, 2])))
        best.sort()
        if limit is not None and best[0][0] > limit:
            raise ValueError(f'({x:.1f}, {y:.1f}) is {best[0][0]:.2f} m from the model road')
        weights = [(1. / max(dd, .01) ** 2, z) for dd, z in best[:2]]
        return sum(w * z for w, z in weights) / sum(w for w, _ in weights)


def author_background_asset(path):
    """The replay renderer's box car (origin at the front bumper, ground at z = 0)."""
    from pxr import Gf, Usd, UsdGeom
    stage = Usd.Stage.CreateNew(str(path))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, '/Vehicle').GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    def cube(name, position, scale, color):
        item = UsdGeom.Cube.Define(stage, '/Vehicle/' + name)
        item.CreateSizeAttr(1.)
        item.AddTranslateOp().Set(Gf.Vec3d(*position))
        item.AddScaleOp().Set(Gf.Vec3f(*scale))
        item.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube('Body', (-2.3, 0., .62), (4.6, 1.8, .9), (.46, .5, .53))
    cube('Cabin', (-2.5, 0., 1.23), (2.45, 1.52, .55), (.14, .19, .23))
    for name, x, y in [('FrontLeft', -.85, .9), ('FrontRight', -.85, -.9), ('RearLeft', -3.55, .9), ('RearRight', -3.55, -.9)]:
        wheel = UsdGeom.Cylinder.Define(stage, '/Vehicle/' + name + 'Wheel')
        wheel.CreateAxisAttr('Y'); wheel.CreateRadiusAttr(.33); wheel.CreateHeightAttr(.22)
        wheel.AddTranslateOp().Set(Gf.Vec3d(x, y, .33))
        wheel.CreateDisplayColorAttr([Gf.Vec3f(.025, .03, .035)])
    for side, y in [('Left', .6), ('Right', -.6)]:
        cube(side + 'Headlight', (-.005, y, .77), (.03, .35, .15), (.95, .95, .8))
        cube(side + 'Taillight', (-4.605, y, .77), (.03, .35, .15), (.7, .025, .015))
    stage.GetRootLayer().Save()


class Server:
    def __init__(self, args):
        self.args = args
        self.output = args.output
        self.frames_dir = self.output / 'frames'
        self.log_rows, self.frames, self.events = [], [], []
        self.agent = None
        self.removed = False
        self.decisions = 0
        self.slots, self.free = {}, list(range(POOL))

    # --- scene -----------------------------------------------------------------------
    def build(self):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from usd.physical_scene import PATHS
        from usd.showcase_vehicle import compose_prepared_vehicle
        from scripts.render_i110_merge_replay import read_model_surface, surface_elevation

        args = self.args
        surface, self.surface_info = read_model_surface(args.surface_manifest, args.network)
        self.surface = Surface(surface)
        for lane in list(surface.values())[:3]:          # the vectorised height equals the renderer's
            x, y, _ = lane['points'][len(lane['points']) // 2]
            for dx, dy in ((0., 0.), (1.3, -.7)):
                expected = surface_elevation(dict(x=x + dx, y=y + dy), surface)
                if abs(self.surface.height(x + dx, y + dy) - expected) > 1e-9:
                    raise RuntimeError('Vectorised surface height differs from the renderer')
        render_hz = args.render_hz or 10
        self.session = RenderedPhysicsSession(physics_hz=PHYSICS_HZ, render_hz=render_hz,
                                              physics_scene_path=PATHS.physics)
        stage = self.stage = self.session.stage
        scene_dir = self.output / 'scene'
        self.vehicle = compose_prepared_vehicle(stage, scene_dir / 'vehicle', args.vehicle_package, PHYSICS_HZ)
        meta = self.vehicle.metadata
        # the road model, unchanged, and the stage-level physics joins described in the docstring
        road = UsdGeom.Xform.Define(stage, ROAD).GetPrim()
        road.GetReferences().AddReference(str(args.road_model))
        stage.GetPrimAtPath(ROAD + '/PhysicsScene').SetActive(False)
        stage.GetPrimAtPath(PATHS.ground).SetActive(False)
        decks = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(ROAD + '/Freeway'))
                 if p.GetName() == 'Deck' and p.HasAPI(UsdPhysics.CollisionAPI)]
        group = stage.GetPrimAtPath('/World/Physics/Resources/GroundSurfaceCollisionGroup')
        includes = group.GetRelationship('collection:colliders:includes')
        kept = [t for t in includes.GetTargets() if not t.HasPrefix(Sdf.Path(PATHS.ground))]
        includes.SetTargets(kept + decks)
        asphalt = Sdf.Path(ROAD + '/Looks/PhysAsphalt')
        if not stage.GetPrimAtPath(asphalt).HasAPI(UsdPhysics.MaterialAPI):
            raise RuntimeError('The road model has no PhysAsphalt physics material')
        table = stage.GetPrimAtPath('/World/Physics/Resources/WinterTireFrictionTable')
        materials = table.GetRelationship('groundMaterials')
        values = list(table.GetAttribute('frictionValues').Get())
        materials.SetTargets(list(materials.GetTargets()) + [asphalt])
        table.GetAttribute('frictionValues').Set(values + [ASPHALT_FRICTION])
        for light in [p for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Lighting')) if p.HasAPI(UsdLux.LightAPI)]:
            light.SetActive(False)
        UsdLux.DomeLight.Define(stage, '/World/Lighting/Sky').CreateIntensityAttr(800.)
        sun = UsdLux.DistantLight.Define(stage, '/World/Lighting/Sun')
        sun.CreateIntensityAttr(1800.); sun.AddRotateXYZOp().Set(Gf.Vec3f(-35., -20., 35.))
        camera = UsdGeom.Camera.Define(stage, CAMERA)
        camera.CreateFocalLengthAttr(24.); camera.CreateHorizontalApertureAttr(36.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 3000.)); camera.AddTransformOp()
        # background car pool
        author_background_asset(scene_dir / 'background-vehicle.usda')
        UsdGeom.Scope.Define(stage, TRAFFIC)
        self.pool = []
        for i in range(POOL):
            root = UsdGeom.Xform.Define(stage, f'{TRAFFIC}/Car_{i:03d}')
            ops = (root.AddTranslateOp(), root.AddRotateZOp(), root.AddRotateYOp())
            model = UsdGeom.Xform.Define(stage, f'{TRAFFIC}/Car_{i:03d}/Model')
            model.GetPrim().GetReferences().AddReference(str(scene_dir / 'background-vehicle.usda'))
            scale = model.AddScaleOp()
            root.CreateVisibilityAttr('invisible')
            self.pool.append(dict(root=root, ops=ops, scale=scale))
        mass_api = UsdPhysics.MassAPI(stage.GetPrimAtPath(PATHS.chassis))
        mass = float(mass_api.GetMassAttr().Get())
        front_x = max(float(c[0]) for c in meta['wheel_centers_m'].values())
        rear_x = min(float(c[0]) for c in meta['wheel_centers_m'].values())
        tires = {}
        for name in ('FrontTire', 'RearTire'):        # per-tire lateral stiffness at rest load: latStiffY / latStiffX
            graph = stage.GetPrimAtPath(PATHS.vehicle + '/Physics/' + name).GetAttribute(
                'physxVehicleTire:lateralStiffnessGraph').Get()
            tires[name] = float(graph[1]) / float(graph[0])
        self.cornering = cornering_terms(mass, front_x - rear_x, float(mass_api.GetCenterOfMassAttr().Get()[0]),
                                         front_x, rear_x, tires['FrontTire'], tires['RearTire'])
        self.cornering.update(tire_lateral_stiffness_n_rad=tires, source=(
            'asset mass, centre of mass, wheel positions and lateralStiffnessGraph; linear bicycle model'))
        moi = {}
        for name in ('FrontWheel', 'RearWheel'):
            value = stage.GetPrimAtPath(PATHS.vehicle + '/Physics/' + name).GetAttribute('physxVehicleWheel:moi').Get()
            if value is None:
                raise RuntimeError(f'The package has no physxVehicleWheel:moi on {name}')
            moi[name] = float(value)
        self.axles = dict(front_x_m=front_x, rear_x_m=rear_x)
        self.gains = controller_gains(args.controller)
        self.spec = VehicleSpec(mass_kg=mass, wheelbase_m=float(meta['wheelbase_m']),
                                front_bumper_m=float(meta['body_collision_center_m'][0] + meta['body_collision_dimensions_m'][0] / 2),
                                rear_axle_m=-rear_x, wheel_radius_m=float(meta['wheel_radius_m']),
                                max_drive_nm_per_wheel=args.max_drive_nm, max_brake_nm_per_wheel=args.max_brake_nm,
                                understeer_rad_per_m_s2=self.cornering['understeer_rad_per_m_s2'],
                                rear_slip_rad_per_m_s2=self.cornering['rear_slip_rad_per_m_s2'],
                                wheels=len(meta['wheel_centers_m']),
                                wheel_moi_kg_m2=(moi['FrontWheel'] + moi['RearWheel']) / 2)
        self.controller = LowLevelController(self.spec, self.gains)
        self.scene = dict(road_model=str(args.road_model), road_decks_in_ground_group=len(decks),
                          road_physics_scene='deactivated in this stage', package_ground_plane='deactivated',
                          asphalt_tire_friction=ASPHALT_FRICTION, tire_friction_table=dict(
                              materials=[str(t) for t in materials.GetTargets()], values=values + [ASPHALT_FRICTION]),
                          surface=self.surface_info, physics_hz=PHYSICS_HZ, background='visual only, SUMO poses',
                          cornering=self.cornering, controller=args.controller, wheel_moi_kg_m2=moi)
        self.paths = PATHS

    def spawn(self, pose):
        """Author the car's start pose and velocity on the road under SUMO's pose, then start physics."""
        from pxr import Gf, UsdGeom, UsdPhysics
        spec = self.spec
        yaw = math.radians(90. - float(pose['angle']))
        c, s = math.cos(yaw), math.sin(yaw)
        cx, cy = pose['x'] - spec.front_bumper_m * c, pose['y'] - spec.front_bumper_m * s
        half = spec.wheelbase_m / 2
        zf = self.surface.height(cx + half * c, cy + half * s)
        zr = self.surface.height(cx - half * c, cy - half * s)
        pitch = math.atan2(zf - zr, spec.wheelbase_m)
        rest = float(self.vehicle.metadata.get('chassis_origin_in_v05_m', [0, 0, 1.])[2])
        z = (zf + zr) / 2 + rest * math.cos(pitch)
        q = quaternion(yaw, pitch)
        forward = rotate(q, (1., 0., 0.))
        if abs(forward[2] - math.sin(pitch)) > 1e-9 or abs(math.atan2(forward[1], forward[0]) - math.atan2(s, c)) > 1e-9:
            raise RuntimeError('Spawn orientation check failed')
        chassis = self.stage.GetPrimAtPath(self.paths.chassis)
        ops = {op.GetOpType(): op for op in UsdGeom.Xformable(chassis).GetOrderedXformOps()}
        translate, orient = ops[UsdGeom.XformOp.TypeTranslate], ops[UsdGeom.XformOp.TypeOrient]
        translate.Set(type(translate.Get())(cx, cy, z))
        quat = Gf.Quatf if orient.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Quatd
        vec = Gf.Vec3f if quat is Gf.Quatf else Gf.Vec3d
        orient.Set(quat(q[0], vec(q[1], q[2], q[3])))
        speed = float(pose['speed'])
        velocity = tuple(speed * f for f in forward)
        UsdPhysics.RigidBodyAPI(chassis).CreateVelocityAttr().Set(Gf.Vec3f(*velocity))
        UsdPhysics.RigidBodyAPI(chassis).CreateAngularVelocityAttr().Set(Gf.Vec3f(0., 0., 0.))
        from traffic.driver_control import AppliedControl
        self.vehicle.apply(AppliedControl(0., 0., 0., 'spawn', None, False), self.spec.max_drive_nm_per_wheel,
                           self.spec.max_brake_nm_per_wheel)
        self.session.start()
        view = self.session.manager.get_physics_simulation_view()
        self.vehicle.bind_native_state(view)
        native = self.vehicle._native_state_view
        sample = native.get_velocities()
        values = [[*velocity, 0., 0., 0.]]
        if hasattr(sample, 'ptr') and hasattr(sample, 'device'):        # warp, as experiments/run_highway_circles.reset_fleet
            import warp as wp
            native.set_velocities(wp.array(values, dtype=wp.float32, device=sample.device),
                                  wp.array([0], dtype=wp.uint32, device=sample.device))
        elif hasattr(sample, 'detach'):
            import torch
            native.set_velocities(torch.tensor(values, dtype=torch.float32, device=sample.device),
                                  torch.tensor([0], dtype=torch.int32, device=sample.device))
        else:
            import numpy as np
            native.set_velocities(np.asarray(values, dtype=np.float32), np.asarray([0], dtype=np.uint32))
        for wheel in self.vehicle.wheel_paths:
            self.vehicle.physx.set_wheel_rotation_speed(wheel, speed / spec.wheel_radius_m)
        after = self.read()
        self.speed = after['speed']
        self.spawn_report = dict(requested=pose, chassis=[cx, cy, z], yaw_rad=yaw, pitch_rad=pitch,
                                 road_height_front_axle=zf, road_height_rear_axle=zr, rest_height_m=rest,
                                 after_start=after, start=self.session.startup)
        self.camera_state = None
        return self.spawn_report

    def read(self):
        from omni.physx.bindings._physx import (VEHICLE_WHEEL_STATE_GROUND_SHAPE, VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE,
                                                VEHICLE_WHEEL_STATE_TIRE_FRICTION)
        raw = self.vehicle.state(refresh_usd=False)
        x, y, z, w = raw['quaternion_xyzw']
        q = (w, x, y, z)
        forward, left = rotate(q, (1., 0., 0.)), rotate(q, (0., 1., 0.))
        horizontal = math.hypot(forward[0], forward[1])
        velocity = raw['velocity_m_s']
        px, py, pz = raw['position_m']
        bumper = (px + self.spec.front_bumper_m * forward[0], py + self.spec.front_bumper_m * forward[1])
        wheels = [self.vehicle.physx.get_wheel_state(p) for p in self.vehicle.wheel_paths]
        return dict(chassis=[px, py, pz], yaw=math.atan2(forward[1], forward[0]), grade=forward[2],
                    pitch_rad=math.asin(max(-1., min(1., forward[2]))), roll_rad=math.asin(max(-1., min(1., left[2]))),
                    speed=(velocity[0] * forward[0] + velocity[1] * forward[1]) / max(horizontal, 1e-9),
                    vertical_speed=velocity[2], bumper=bumper, x=px, y=py,
                    angle=(90. - math.degrees(math.atan2(forward[1], forward[0]))) % 360.,
                    on_ground=raw['wheel_on_ground'],
                    jounce=[None if s.get(VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE) is None
                            else float(s[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]) for s in wheels],
                    tire_friction=[None if s.get(VEHICLE_WHEEL_STATE_TIRE_FRICTION) is None
                                   else float(s[VEHICLE_WHEEL_STATE_TIRE_FRICTION]) for s in wheels],
                    ground=[str(s.get(VEHICLE_WHEEL_STATE_GROUND_SHAPE, '')) for s in wheels])

    # --- background and camera -----------------------------------------------------------
    def road_grade(self, state):
        """Sine of the model road's slope between the front and rear axle, grade_preview_s ahead."""
        ahead = max(float(state['speed']), 0.) * self.gains.grade_preview_s
        c, s = math.cos(state['yaw']), math.sin(state['yaw'])
        front, rear = self.axles['front_x_m'] + ahead, self.axles['rear_x_m'] + ahead
        zf = self.surface.height(state['x'] + front * c, state['y'] + front * s)
        zr = self.surface.height(state['x'] + rear * c, state['y'] + rear * s)
        rise, run = zf - zr, front - rear
        return rise / math.hypot(run, rise)

    def place_background(self, background):
        from pxr import Gf
        present = {b['id']: b for b in background}
        for vid in [v for v in self.slots if v not in present]:
            slot = self.slots.pop(vid)
            self.pool[slot]['root'].GetVisibilityAttr().Set('invisible')
            self.free.append(slot)
        for vid, b in sorted(present.items()):
            if vid not in self.slots:
                if not self.free:
                    raise RuntimeError('Background car pool exhausted')
                self.slots[vid] = self.free.pop(0)
            item = self.pool[self.slots[vid]]
            heading = math.radians(90. - float(b['angle']))
            length = float(b.get('length', 4.6))
            zf = self.surface.height(b['x'], b['y'])
            zr = self.surface.height(b['x'] - length * math.cos(heading), b['y'] - length * math.sin(heading))
            translate, rotate_z, rotate_y = item['ops']
            translate.Set(Gf.Vec3d(b['x'], b['y'], zf))
            rotate_z.Set(math.degrees(heading))
            rotate_y.Set(-math.degrees(math.atan2(zf - zr, length)))     # nose up is negative about +y
            item['scale'].Set(Gf.Vec3f(length / 4.6, float(b.get('width', 1.8)) / 1.8, 1.))
            item['root'].GetVisibilityAttr().Set('inherited')

    def place_camera(self, state):
        from pxr import Gf, UsdGeom
        a = self.args
        yaw = state['yaw']
        if self.camera_state is None:
            self.camera_state = dict(yaw=yaw)
        self.camera_state['yaw'] += .25 * math.atan2(math.sin(yaw - self.camera_state['yaw']),
                                                     math.cos(yaw - self.camera_state['yaw']))
        cy = self.camera_state['yaw']
        px, py, pz = state['chassis']
        target = Gf.Vec3d(px + a.camera_ahead * math.cos(cy), py + a.camera_ahead * math.sin(cy), pz)
        eye = Gf.Vec3d(px - a.camera_behind * math.cos(cy), py - a.camera_behind * math.sin(cy), pz + a.camera_height)
        matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0., 0., 1.)).GetInverse()
        UsdGeom.Xformable(self.stage.GetPrimAtPath(CAMERA)).GetOrderedXformOps()[0].Set(matrix)
        return dict(eye=list(eye), target=list(target))

    def capture(self, time_s, state):
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        if not hasattr(self, 'viewport'):
            self.viewport = get_active_viewport()
            self.viewport.camera_path = CAMERA
            self.viewport.resolution = tuple(self.args.resolution)
            self.frames_dir.mkdir(parents=True, exist_ok=True)
        camera = self.place_camera(state)
        self.session.publish_transforms()
        for _ in range(4 if self.frames else 20):      # settle the real-time renderer (the replay renderer used 3)
            self.session.render()
        target = self.frames_dir / f'{len(self.frames):05d}.png'
        capture_viewport_to_file(self.viewport, str(target))
        deadline = time.perf_counter() + 30.
        while not target.exists() and time.perf_counter() < deadline:
            self.session.render()
        if not target.is_file():
            raise RuntimeError(f'Viewport capture timed out: {target.name}')
        self.frames.append(dict(index=len(self.frames), time=time_s, file=target.name, camera=camera))

    # --- one decision ------------------------------------------------------------------
    def advance(self, message):
        from traffic.driver_control import AppliedControl
        command = message['commands'].get(self.agent)
        if command is None or self.removed:
            raise RuntimeError('No command for the physics car')
        now = float(message['time'])
        self.place_background(message.get('background', []))
        start = self.read()
        if self.args.render_hz:
            self.capture(now, start)
        self.controller.begin_decision(time=now, acceleration=float(command['acceleration']),
                                       path=command['path'], speed=start['speed'])
        dt = 1. / PHYSICS_HZ
        ticks, state = [], start
        road = self.gains.grade_source == 'road'
        for k in range(STEPS_PER_DECISION):
            inputs = dict(x=state['x'], y=state['y'], yaw=state['yaw'], speed=state['speed'], grade=state['grade'])
            if road:
                inputs['road_grade'] = self.road_grade(state)
            control = self.controller.command(inputs, now + k * dt, dt)
            applied = AppliedControl(control['steer_rad'],
                                     control['drive_nm'] / self.spec.max_drive_nm_per_wheel if self.spec.max_drive_nm_per_wheel else 0.,
                                     control['brake_nm'] / self.spec.max_brake_nm_per_wheel if self.spec.max_brake_nm_per_wheel else 0.,
                                     'policy_command', self.decisions, False)
            self.vehicle.apply(applied, self.spec.max_drive_nm_per_wheel, self.spec.max_brake_nm_per_wheel)
            self.session.step(publish_usd=False)
            state = self.read()
            ticks.append(control)
        self.decisions += 1
        measured = (state['speed'] - start['speed']) / DECISION_S
        pose = dict(x=state['bumper'][0], y=state['bumper'][1], angle=state['angle'], speed=state['speed'],
                    acceleration=measured)
        telemetry = dict(time=round(now + DECISION_S, 6), commanded_acceleration=float(command['acceleration']),
                         measured_acceleration=measured, speed=state['speed'],
                         saturated_ticks=sum(t['saturated'] for t in ticks),
                         drive_nm=sum(t['drive_nm'] for t in ticks) / len(ticks),
                         brake_nm=sum(t['brake_nm'] for t in ticks) / len(ticks),
                         steer_rad=ticks[-1]['steer_rad'], lateral_error_m=ticks[-1]['lateral_error_m'],
                         heading_error_rad=ticks[-1]['heading_error_rad'], bias_n=ticks[-1]['bias_n'],
                         grade_used=ticks[-1]['grade_used'],
                         grade_used_range=[min(t['grade_used'] for t in ticks), max(t['grade_used'] for t in ticks)],
                         maneuver=command.get('maneuver'), lane_id=command.get('lane_id'),
                         chassis=state['chassis'], pitch_rad=state['pitch_rad'], roll_rad=state['roll_rad'],
                         vertical_speed=state['vertical_speed'], on_ground=state['on_ground'], jounce=state['jounce'],
                         tire_friction=state['tire_friction'], ground=sorted(set(state['ground'])))
        self.log_rows.append(telemetry)
        if self.decisions % 10 == 0:
            print(f'decision {self.decisions} t={now:.1f} v={state["speed"]:.2f} a_cmd={command["acceleration"]:+.2f} '
                  f'a={measured:+.2f} lat={telemetry["lateral_error_m"]:+.3f} wheels={state["on_ground"]}', flush=True)
        return dict(poses={self.agent: pose}, telemetry=telemetry)

    def report(self):
        rows = self.log_rows
        def stat(key, f=abs):
            values = [f(r[key]) for r in rows]
            return max(values) if values else None
        errors = [abs(r['commanded_acceleration'] - r['measured_acceleration']) for r in rows]
        return dict(schema='physics_drive_server_v01', agent=self.agent, decisions=len(rows),
                    controller=self.args.controller,
                    bias_paused_decisions=sum(h['bias_paused'] for h in self.controller.history),
                    vehicle=dict(package=str(self.args.vehicle_package), spec=self.controller.describe(),
                                 provisional_physics=self.vehicle.metadata.get('provisional_physics')),
                    scene=self.scene, spawn=getattr(self, 'spawn_report', None), frames=len(self.frames),
                    max_abs_acceleration_error=max(errors) if errors else None,
                    mean_abs_acceleration_error=sum(errors) / len(errors) if errors else None,
                    max_abs_lateral_error_m=stat('lateral_error_m'), max_abs_pitch_rad=stat('pitch_rad'),
                    max_abs_roll_rad=stat('roll_rad'), saturated_decisions=sum(r['saturated_ticks'] > 0 for r in rows),
                    wheels_off_ground_decisions=sum(not all(r['on_ground']) for r in rows),
                    ground_shapes=sorted({g for r in rows for g in r['ground']}),
                    motion_authority='PhysX (Isaac Sim) for the physics car; SUMO poses drawn for background cars')

    # --- message loop --------------------------------------------------------------------
    def serve(self, sock):
        while True:
            message = physics_link.receive(sock)
            kind = message.get('kind')
            try:
                if kind == 'describe':
                    reply = dict(scene=self.scene, vehicle=self.controller.describe(), render_hz=self.args.render_hz,
                                 resolution=list(self.args.resolution), python=sys.version)
                elif kind == 'spawn':
                    if self.agent is not None:
                        raise RuntimeError('The server drives one car per run')
                    self.agent = message['agent']
                    reply = dict(spawn=self.spawn(message['pose']))
                elif kind == 'advance':
                    reply = self.advance(message)
                elif kind == 'remove':
                    from traffic.driver_control import AppliedControl
                    self.removed = True
                    self.vehicle.apply(AppliedControl(0., 0., 1., 'removed', None, True),
                                       self.spec.max_drive_nm_per_wheel, self.spec.max_brake_nm_per_wheel)
                    reply = dict(removed=message['agent'])
                elif kind == 'close':
                    report = self.report()
                    (self.output / 'physics-log.json').write_text(json.dumps(dict(rows=self.log_rows, frames=self.frames),
                                                                              indent=1), encoding='utf-8')
                    (self.output / 'physics-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
                    physics_link.send(sock, dict(report=report))
                    return
                else:
                    raise ValueError(f'Unknown message {kind!r}')
            except Exception:
                physics_link.send(sock, dict(error=traceback.format_exc()))
                raise
            physics_link.send(sock, reply)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--vehicle-package', type=Path, required=True)
    parser.add_argument('--road-model', type=Path, required=True)
    parser.add_argument('--surface-manifest', type=Path, required=True)
    parser.add_argument('--network', type=Path, required=True)
    parser.add_argument('--render-hz', type=int, default=10, choices=[0, 10])
    parser.add_argument('--resolution', type=int, nargs=2, default=[1280, 720])
    parser.add_argument('--camera-behind', type=float, default=16.)
    parser.add_argument('--camera-height', type=float, default=6.)
    parser.add_argument('--camera-ahead', type=float, default=10.)
    parser.add_argument('--max-drive-nm', type=float, default=700.)
    parser.add_argument('--max-brake-nm', type=float, default=1500.)
    parser.add_argument('--link-port', type=int, required=True)
    parser.add_argument('--controller', default='stage5', choices=sorted(CONTROLLERS))
    args = parser.parse_args()
    args.output = args.output.resolve()
    if not args.output.is_relative_to(ROOT / 'outputs') or args.output.exists():
        parser.error('Use a fresh output directory under outputs/')
    args.output.mkdir(parents=True)
    token = os.environ.pop(physics_link.TOKEN_ENV)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': args.resolution[0], 'height': args.resolution[1],
                         'limit_cpu_threads': 8, 'extra_args': ['--/app/useFabricSceneDelegate=false',
                                                                '--/persistent/app/viewport/displayOptions=0']})
    server = sock = None
    try:
        import carb
        carb.settings.get_settings().set('/persistent/app/viewport/displayOptions', 0)
        server = Server(args)
        started = time.perf_counter()
        server.build()
        print(f'scene built in {time.perf_counter() - started:.1f} s', flush=True)
        sock = physics_link.connect_back(args.link_port, token)
        server.serve(sock)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        if sock is not None:
            sock.close()
        if server is not None and getattr(server, 'session', None) is not None:
            server.vehicle = server.controller = None
            try:
                server.session.close()
            except Exception:
                traceback.print_exc()
        app.close()


if __name__ == '__main__':
    main()
