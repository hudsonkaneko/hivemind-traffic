"""Explicit wheel-shape authoring and read-only PhysX/USD alignment evidence."""
import math

AXIS_REFERENCE = 'native_body_times_wheel_lateral_axis'


def direction_error_deg(a, b):
    """Unsigned angle between axle lines; reversing an axle is equivalent."""
    if len(a) != 3 or len(b) != 3 or not all(math.isfinite(v) for v in (*a, *b)):
        raise ValueError('Expected finite three-component directions')
    norm = math.sqrt(sum(v*v for v in a) * sum(v*v for v in b))
    if norm == 0:
        raise ValueError('Direction cannot have zero length')
    return math.degrees(math.acos(min(1.0, abs(sum(x*y for x,y in zip(a,b))) / norm)))


def native_axle_world(body_quaternion_xyzw, wheel_quaternion_xyzw):
    """Native wheel-local Y axle rotated through the native body world pose."""
    from pxr import Gf
    quaternions = []
    for values in (body_quaternion_xyzw, wheel_quaternion_xyzw):
        quaternion_error_deg(values, values)  # Validate before constructing USD objects.
        quaternions.append(Gf.Quatd(values[3], Gf.Vec3d(*values[:3])).GetNormalized())
    return tuple(Gf.Rotation(quaternions[0] * quaternions[1]).TransformDir(Gf.Vec3d(0, 1, 0)))


def author_explicit_wheel_axes(stage, wheel_paths):
    """Keep the same Y axle using an X cylinder plus an explicit child rotation.

    The installed PhysX vehicle runtime leaks a Y-cylinder's implicit shape
    basis conversion into the attachment pose. With an X cylinder and explicit
    child rotation, the composed shape pose is correct (the runtime may fold
    that rotation into the attachment). Author once before simulation; never
    override poses.
    """
    from pxr import UsdGeom

    cylinders = []
    for path in wheel_paths:
        cylinder = UsdGeom.Cylinder(stage.GetPrimAtPath(path + '/Collision'))
        if not cylinder or cylinder.GetAxisAttr().Get() != 'Y':
            raise ValueError('Expected a fresh Factory Y-axis wheel cylinder')
        if cylinder.GetOrderedXformOps():
            raise ValueError('Expected an untransformed wheel cylinder')
        cylinders.append(cylinder)
    for cylinder in cylinders:
        cylinder.GetAxisAttr().Set('X')
        cylinder.AddRotateZOp().Set(90.0)
        cylinder.GetExtentAttr().Set(UsdGeom.Cylinder.ComputeExtentFromPlugins(cylinder, 0))


def sample_wheel_geometry(vehicle, tick):
    from pxr import Gf, Usd, UsdGeom
    from omni.physx.bindings import _physx as native

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body_state = vehicle.physx.get_rigidbody_transformation(vehicle.path)
    if not body_state['ret_val']:
        raise RuntimeError('Native body pose unavailable for wheel evidence')
    body_xyzw = list(body_state['rotation'])
    body_q = Gf.Quatd(body_xyzw[3], Gf.Vec3d(*body_xyzw[:3])).GetNormalized()
    rows = []
    for path in vehicle.wheel_paths:
        prim = vehicle.stage.GetPrimAtPath(path)
        collision = vehicle.stage.GetPrimAtPath(path + '/Collision')
        wheel = vehicle.physx.get_wheel_state(path)
        lateral = list(wheel[native.VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION])
        transform = cache.GetLocalToWorldTransform(collision)
        axis = UsdGeom.Cylinder(collision).GetAxisAttr().Get()
        unit_axis = {'X': (1,0,0), 'Y': (0,1,0), 'Z': (0,0,1)}[axis]
        axle = transform.TransformDir(Gf.Vec3d(*unit_axis)).GetNormalized()
        local = cache.GetLocalTransformation(prim)[0]
        usd_q = transform.ExtractRotationQuat()
        native_q = list(wheel[native.VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION])
        usd_xyzw = [*usd_q.GetImaginary(), usd_q.GetReal()]
        # Native wheel pose is vehicle-local; the USD attachment may carry a
        # reset transform stack. Compare the COMPOSED shape in WORLD coordinates,
        # including the explicit geometry basis, not raw attachment quaternions.
        wheel_q = Gf.Quatd(native_q[3], Gf.Vec3d(*native_q[:3]))
        expected_axle = native_axle_world(body_xyzw, native_q)
        basis_q = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90 if axis == 'X' else 0).GetQuat()
        expected_q = body_q * wheel_q * basis_q
        expected_xyzw = [*expected_q.GetImaginary(), expected_q.GetReal()]
        rows.append(dict(tick=tick, path=path, cylinder_axis=UsdGeom.Cylinder(collision).GetAxisAttr().Get(),
            axle_world=list(axle), tire_lateral_world=lateral,
            expected_axle_world=list(expected_axle), axis_reference=AXIS_REFERENCE,
            axis_error_deg=direction_error_deg(axle, expected_axle),
            tire_contact_direction_difference_deg=direction_error_deg(axle, lateral) if any(lateral) else None,
            attachment_matrix=[list(r) for r in local],
            collision_matrix=[list(r) for r in cache.GetLocalTransformation(collision)[0]],
            native_pose_xyzw=native_q,
            native_body_world_xyzw=body_xyzw,
            expected_geometry_world_xyzw=expected_xyzw,
            geometry_world_xyzw=usd_xyzw,
            native_pose_error_deg=quaternion_error_deg(usd_xyzw, expected_xyzw),
            rotation_rad=wheel[native.VEHICLE_WHEEL_STATE_ROTATION_ANGLE],
            steer_rad=wheel[native.VEHICLE_WHEEL_STATE_STEER_ANGLE]))
    return rows


def quaternion_error_deg(a, b):
    """Shortest angular distance; q and -q encode the same orientation."""
    if len(a) != 4 or len(b) != 4 or not all(math.isfinite(v) for v in (*a, *b)):
        raise ValueError('Expected finite xyzw quaternions')
    norm = math.sqrt(sum(v*v for v in a) * sum(v*v for v in b))
    if norm == 0:
        raise ValueError('Quaternion cannot have zero length')
    dot = abs(sum(x*y for x,y in zip(a,b))) / norm
    return math.degrees(2 * math.acos(min(1.0, max(0.0, dot))))


def assess_wheel_geometry(rows, wheel_paths, physics_steps):
    """Require all four wheels at each 30Hz render of this 120Hz demo."""
    expected = {(tick, path) for tick in range(4, physics_steps + 1, 4) for path in wheel_paths}
    actual = {(r['tick'], r['path']) for r in rows}
    errors = [r[key] for r in rows for key in ('axis_error_deg', 'native_pose_error_deg')]
    coverage = bool(expected) and len(wheel_paths) == 4 and actual == expected and len(rows) == len(expected)
    aligned = bool(errors) and all(math.isfinite(e) and 0 <= e <= .1 for e in errors)
    reference_valid = bool(rows) and all(r.get('axis_reference') == AXIS_REFERENCE for r in rows)
    return dict(passed=coverage and aligned and reference_valid, complete=coverage, aligned=aligned,
                axis_reference=AXIS_REFERENCE, reference_valid=reference_valid,
                samples=len(rows), tolerance_deg=.1,
                max_axis_error_deg=max((r['axis_error_deg'] for r in rows), default=None),
                max_native_pose_error_deg=max((r['native_pose_error_deg'] for r in rows), default=None))
