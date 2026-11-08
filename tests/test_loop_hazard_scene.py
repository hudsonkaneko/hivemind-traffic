"""CPU native-view mocks plus optional plain OpenUSD composition checks."""
import hashlib
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from usd.loop_hazard_scene import (
    BODY_PATH, COLLIDER_PATH, HAZARD_ID, PROXY_PATH, HazardPool, _poses,
    author_hazards,
)

ROOT = Path(__file__).resolve().parents[1]


def obj(x=5., y=5., yaw=0.):
    return {'object_id': HAZARD_ID, 'x_m': x, 'y_m': y, 'yaw_rad': yaw,
            'length_m': 1., 'width_m': 1.}


class Bodies:
    def __init__(self, objects):
        self.prim_paths = [BODY_PATH]
        self.transforms = _poses(objects)[0]
        self.velocities = np.zeros((1, 6), dtype=np.float32)
        self.targets = self.transforms.copy()
        self.teleports = 0
    def get_transforms(self): return self.transforms
    def get_velocities(self): return self.velocities
    def set_transforms(self, values, indices):
        self.transforms[indices] = values[indices]
        self.teleports += 1
    def set_kinematic_targets(self, values, indices): self.targets[indices] = values[indices]
    def set_velocities(self, *args): raise AssertionError('Kinematic velocity writes forbidden')
    def step(self):
        self.velocities[:, :3] = (self.targets[:, :3] - self.transforms[:, :3]) * 120
        self.transforms = self.targets.copy()


def pool(objects=None):
    objects = [obj()] if objects is None else objects
    initial, active = _poses(objects)
    result = HazardPool(None, ROOT / 'outputs', None, {}, {}, initial, active)
    bodies = Bodies(objects)
    result.bind(SimpleNamespace(create_rigid_body_view=lambda path: bodies))
    result.reset_poses(objects, episode_id='episode-1')
    return result, bodies


def test_only_native_target_moves_body_and_actual_velocity_is_observed():
    subject, bodies = pool()
    subject.set_targets([obj(x=5.01)], tick=1)
    np.testing.assert_array_equal(bodies.transforms[:, :3], [[5., 5., .5]])
    with pytest.raises(RuntimeError, match='completed target'):
        subject.measured_frames(1)
    bodies.step()
    frame = subject.measured_frames(1)[0]
    assert frame['x_m'] == pytest.approx(5.01, abs=1e-6)
    assert frame['vx_m_s'] == pytest.approx(1.2, abs=1e-4)
    assert frame['timestamp_s'] == 1/120
    assert bodies.teleports == 1  # reset only
    # Target's invented velocity is never copied into observations.
    target = dict(obj(x=5.02), vx_m_s=999., vy_m_s=999.)
    subject.set_targets([target], tick=2); bodies.step()
    assert subject.measured_frames(2)[0]['vx_m_s'] < 2.


def test_reset_clears_pending_target_but_does_not_fabricate_zero_velocity():
    subject, bodies = pool()
    subject.set_targets([obj(x=5.01)], tick=1); bodies.step()
    subject.reset_poses([obj(x=20)], episode_id='episode-2')
    np.testing.assert_array_equal(bodies.targets, bodies.transforms)
    assert subject.measured_frames(0)[0]['vx_m_s'] > 1.
    subject.set_targets([obj(x=20)], tick=1); bodies.step()
    assert subject.measured_frames(1)[0]['vx_m_s'] == 0.
    assert bodies.teleports == 2
    with pytest.raises(ValueError, match='new episode'):
        subject.reset_poses([obj()], episode_id='episode-2')


def test_active_membership_never_changes_mid_episode():
    subject, bodies = pool([])
    subject.set_targets([], tick=1); bodies.step()
    assert subject.measured_frames(1) == []
    with pytest.raises(ValueError, match='membership'):
        subject.set_targets([obj()], tick=2)
    subject.reset_poses([obj()], episode_id='next')
    assert len(subject.measured_frames(0)) == 1
    with pytest.raises(ValueError, match='membership'):
        subject.set_targets([], tick=1)


@pytest.mark.parametrize('tick', [-1, True, 0, 2])
def test_skipped_duplicate_and_bad_ticks_rejected(tick):
    subject, _ = pool()
    with pytest.raises(ValueError): subject.set_targets([obj()], tick=tick)


@pytest.mark.parametrize('objects', [[dict(obj(), object_id='other')], [dict(obj(), x_m=float('nan'))],
    [dict(obj(), width_m=2.)], [dict(obj(), x_m=5000.)], [obj(), obj()]])
def test_invalid_pose_and_dimension_changes_rejected(objects):
    with pytest.raises(ValueError): _poses(objects)


def test_large_step_or_turn_rejected_without_changing_target():
    subject, bodies = pool()
    before = bodies.targets.copy()
    for target in ([obj(x=8.)], [obj(yaw=1.)]):
        with pytest.raises(ValueError): subject.set_targets(target, tick=1)
        np.testing.assert_array_equal(bodies.targets, before)


def test_quaternion_sign_wrap_is_continuous():
    subject, bodies = pool([obj(yaw=math.pi-.001)])
    subject.set_targets([obj(yaw=-math.pi+.001)], tick=1); bodies.step()
    assert subject.pose_check()['passed']


def test_wrong_native_binding_and_nonfinite_readback_rejected():
    subject, bodies = pool()
    bodies.transforms[0, 0] = np.nan
    with pytest.raises(RuntimeError): subject.measured_frames(0)
    other = HazardPool(None, ROOT/'outputs', None, {}, {}, _poses([obj()])[0], True)
    bodies.prim_paths = ['/World/Unexpected']
    with pytest.raises(RuntimeError, match='identity'):
        other.bind(SimpleNamespace(create_rigid_body_view=lambda _: bodies))


def test_plain_usd_composition_and_immutable_base(tmp_path):
    pytest.importorskip('pxr.Usd')
    from pxr import Usd, UsdGeom, UsdPhysics
    import importlib.util
    spec = importlib.util.spec_from_file_location('hazard_test_base_scene', ROOT/'tests/test_lane_following_scene.py')
    fixture_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture_module)
    LaneSceneTests = fixture_module.LaneSceneTests
    # Reuse the established artificial Factory fixture, not a native app.
    fixture = LaneSceneTests('test_lab_solver_and_vehicle_handles_keep_composed_identity')
    fixture.setUp()
    try:
        from environments.lane_following_scene import save_lane_scene
        _, road, runtime = fixture.compose()
        save_lane_scene(fixture.stage, fixture.directory, road, runtime)
        prior = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in fixture.directory.rglob('*') if p.is_file()}
        subject = author_hazards(fixture.stage, fixture.directory, [obj()])
        assert subject.validate()['passed']
        exported = subject.save_entry()
        assert Path(exported['entry_path']).is_file()
        assert all(hashlib.sha256(p.read_bytes()).hexdigest() == h for p, h in prior.items())
        assert UsdPhysics.RigidBodyAPI(fixture.stage.GetPrimAtPath(BODY_PATH)).GetKinematicEnabledAttr().Get()
        assert UsdPhysics.CollisionAPI(fixture.stage.GetPrimAtPath(COLLIDER_PATH)).GetCollisionEnabledAttr().Get()
        assert not fixture.stage.GetPrimAtPath(PROXY_PATH).HasAPI(UsdPhysics.RigidBodyAPI)
        from pxr import Sdf
        proxy = fixture.stage.GetPrimAtPath(PROXY_PATH)
        with Usd.EditContext(fixture.stage, subject.runtime):
            proxy.CreateRelationship('invalid:test').SetTargets(['/World/Missing'])
        with pytest.raises(ValueError, match='Unresolved relationship'): subject.validate()
        with Usd.EditContext(fixture.stage, subject.runtime):
            proxy.RemoveProperty('invalid:test')
        assert subject.validate()['passed']
        extra = UsdGeom.Cube.Define(fixture.stage, '/World/Environment/Props/Rogue')
        UsdPhysics.CollisionAPI.Apply(extra.GetPrim())
        with pytest.raises(ValueError, match='inventory'): subject.validate()
    finally:
        fixture.doCleanups()
