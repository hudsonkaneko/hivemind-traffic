"""Wheel authoring regression: identical starting shape, explicit lateral axle."""
import pytest


def make_wheel():
    pytest.importorskip('pxr.Usd')
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    wheel = UsdGeom.Xform.Define(stage, '/World/Car/Wheel')
    wheel.AddTranslateOp().Set((1.6, .8, -.65))
    cylinder = UsdGeom.Cylinder.Define(stage, '/World/Car/Wheel/Collision')
    cylinder.CreateAxisAttr('Y')
    cylinder.CreateRadiusAttr(.35)
    cylinder.CreateHeightAttr(.15)
    cylinder.CreateExtentAttr(UsdGeom.Cylinder.ComputeExtentFromPlugins(cylinder, 0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    return stage, wheel, cylinder


def test_explicit_axis_preserves_shape_center_dimensions_and_collision():
    stage, wheel, cylinder = make_wheel()
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from traffic.wheel_geometry import author_explicit_wheel_axes
    old_bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default']).ComputeWorldBound(cylinder.GetPrim()).ComputeAlignedBox()
    wheel_before = wheel.GetLocalTransformation()
    author_explicit_wheel_axes(stage, ['/World/Car/Wheel'])
    new_bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default']).ComputeWorldBound(cylinder.GetPrim()).ComputeAlignedBox()
    assert tuple(new_bounds.GetMin()) == pytest.approx(tuple(old_bounds.GetMin()), abs=1e-7)
    assert tuple(new_bounds.GetMax()) == pytest.approx(tuple(old_bounds.GetMax()), abs=1e-7)
    assert cylinder.GetAxisAttr().Get() == 'X'
    axle = cylinder.GetLocalTransformation().TransformDir(Gf.Vec3d(1, 0, 0))
    assert tuple(axle) == pytest.approx((0, 1, 0), abs=1e-12)
    assert wheel.GetLocalTransformation() == wheel_before
    assert cylinder.GetRadiusAttr().Get() == .35
    assert cylinder.GetHeightAttr().Get() == .15
    assert cylinder.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
    assert tuple(cylinder.GetExtentAttr().Get()[1]) == pytest.approx((.075, .35, .35))


def test_repeated_or_unexpected_authoring_fails_closed():
    stage, wheel, cylinder = make_wheel()
    from traffic.wheel_geometry import author_explicit_wheel_axes
    cylinder.AddRotateZOp().Set(10)
    with pytest.raises(ValueError, match='untransformed'):
        author_explicit_wheel_axes(stage, ['/World/Car/Wheel'])
    cylinder.ClearXformOpOrder()
    cylinder.GetAxisAttr().Set('Z')
    with pytest.raises(ValueError, match='Y-axis'):
        author_explicit_wheel_axes(stage, ['/World/Car/Wheel'])


def test_quaternion_comparison_detects_basis_bug_and_ignores_sign_and_scale():
    from traffic.wheel_geometry import quaternion_error_deg
    assert quaternion_error_deg((0, 0, 0, 1), (0, 0, 1, 1)) == pytest.approx(90)
    assert quaternion_error_deg((0, 1, 0, 1), (0, -2, 0, -2)) == pytest.approx(0)
    for bad in [(0, 0, 0, 0), (0, float('nan'), 0, 1), (1, 2)]:
        with pytest.raises(ValueError): quaternion_error_deg(bad, (0, 0, 0, 1))


def test_wheel_gate_requires_complete_unique_finite_aligned_evidence():
    from traffic.wheel_geometry import AXIS_REFERENCE, assess_wheel_geometry
    paths = ['FL', 'FR', 'RL', 'RR']
    rows = [dict(tick=tick, path=path, axis_error_deg=0, native_pose_error_deg=0, axis_reference=AXIS_REFERENCE)
            for tick in (4, 8) for path in paths]
    assert assess_wheel_geometry(rows, paths, 8)['passed']
    for bad in [[], rows[:-1], rows + rows[:1],
                rows[:-1] + [dict(rows[-1], axis_error_deg=90)],
                rows[:-1] + [dict(rows[-1], axis_reference='tire_contact_direction')],
                [{k:v for k,v in r.items() if k != 'axis_reference'} for r in rows],
                rows[:-1] + [dict(rows[-1], native_pose_error_deg=float('nan'))]]:
        assert not assess_wheel_geometry(bad, paths, 8)['passed']


def test_tilted_steered_spinning_axle_matches_native_not_ground_contact():
    pytest.importorskip('pxr.Gf')
    from pxr import Gf
    from traffic.wheel_geometry import direction_error_deg, native_axle_world
    def xyzw(q): return [*q.GetImaginary(), q.GetReal()]
    body = Gf.Rotation(Gf.Vec3d(0,0,1), 31).GetQuat() * Gf.Rotation(Gf.Vec3d(1,0,0), 2).GetQuat()
    wheel = Gf.Rotation(Gf.Vec3d(0,0,1), 12).GetQuat() * Gf.Rotation(Gf.Vec3d(0,1,0), 83).GetQuat()
    basis = Gf.Rotation(Gf.Vec3d(0,0,1), 90).GetQuat()
    rendered = Gf.Rotation(body * wheel * basis).TransformDir(Gf.Vec3d(1,0,0))
    expected = native_axle_world(xyzw(body), xyzw(wheel))
    assert direction_error_deg(rendered, expected) < 1e-5
    contact = (expected[0], expected[1], 0.)
    assert direction_error_deg(rendered, contact) > .1
    # The old implicit Y cylinder under a folded Z90 shape basis points sideways.
    pizza = Gf.Rotation(body * wheel * basis).TransformDir(Gf.Vec3d(0,1,0))
    assert direction_error_deg(pizza, expected) == pytest.approx(90, abs=1e-5)


@pytest.mark.parametrize('bad', [(0,0,0), (0,float('nan'),1), (1,2)])
def test_invalid_axle_directions_fail_closed(bad):
    from traffic.wheel_geometry import direction_error_deg
    with pytest.raises(ValueError): direction_error_deg(bad, (0,1,0))
