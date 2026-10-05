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
    from traffic.wheel_geometry import assess_wheel_geometry
    paths = ['FL', 'FR', 'RL', 'RR']
    rows = [dict(tick=tick, path=path, axis_error_deg=0, native_pose_error_deg=0)
            for tick in (4, 8) for path in paths]
    assert assess_wheel_geometry(rows, paths, 8)['passed']
    for bad in [[], rows[:-1], rows + rows[:1],
                rows[:-1] + [dict(rows[-1], axis_error_deg=90)],
                rows[:-1] + [dict(rows[-1], native_pose_error_deg=float('nan'))]]:
        assert not assess_wheel_geometry(bad, paths, 8)['passed']

