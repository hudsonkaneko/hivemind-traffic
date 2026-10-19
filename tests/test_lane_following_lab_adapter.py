"""Execute the real adapter hooks with fake Lab/native APIs, never start Kit."""
from copy import deepcopy
from dataclasses import asdict
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from environments.lane_following_contract import LaneFollowingTask, LaneTaskConfig
from traffic.driver_control import AppliedControl, DriverControlGate
from traffic.lane_geometry import load_routes

ROOT = Path(__file__).resolve().parents[1]


class Tensor(np.ndarray):
    def __new__(cls, values, dtype=None, **kwargs):
        return np.asarray(values, dtype=dtype).view(cls)

    def detach(self):
        return self

    def cpu(self):
        return self

    def clone(self):
        return self.copy().view(Tensor)

    def zero_(self):
        self.fill(0)


class Config:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Settings:
    def __init__(self, disabled=True):
        self.disabled = disabled
        self.writes = []

    def get_setting(self, name):
        assert name == '/physics/disableContactProcessing'
        return self.disabled

    def set_setting(self, name, value):
        assert name == '/physics/disableContactProcessing'
        self.writes.append(value)
        self.disabled = value


class LabBase:
    def close(self):
        self.base_closed = True

    def _reset_idx(self, env_ids):
        pass

    def reset(self, seed=None, options=None):
        self._reset_idx([0])
        return self._get_observations(), self.extras


def test_close_releases_named_camera_before_lab_stage_teardown(env, monkeypatch):
    viewport = SimpleNamespace(camera_path='/World/Cameras/Follow')
    rendered = []
    env.sim = SimpleNamespace(render=lambda: rendered.append(viewport.camera_path))
    monkeypatch.setitem(sys.modules, 'omni.kit.viewport.utility',
        SimpleNamespace(get_active_viewport=lambda: viewport))
    env.close()
    assert rendered == ['/OmniverseKit_Persp']
    assert env.base_closed and env.viewport_cleanup == {'attempted': True, 'passed': True}
    assert env.vehicle is None


def test_close_records_viewport_failure_but_always_tears_down(env, monkeypatch):
    def fail():
        raise RuntimeError('camera cleanup failed')
    monkeypatch.setitem(sys.modules, 'omni.kit.viewport.utility',
        SimpleNamespace(get_active_viewport=fail))
    env.close()
    assert env.base_closed and not env.viewport_cleanup['passed']
    assert 'camera cleanup failed' in env.viewport_cleanup['error']


def test_reset_seed_range_rejected_before_any_physics_write(env):
    writes = len(env.pose_writes)
    for seed in (-1, 2**32, True, 1.5):
        with pytest.raises(ValueError, match='unsigned 32-bit'):
            env.reset(seed=seed)
    assert len(env.pose_writes) == writes


@pytest.mark.parametrize('previous', [True, False])
def test_contact_processing_enables_and_restores_previous_after_close(env, monkeypatch, previous):
    env._restore_contact_processing()
    settings = Settings(previous)
    original_set = settings.set_setting
    def tracked_set(name, value):
        if value is previous and settings.writes:
            assert env.base_closed  # Restore follows Lab's physics teardown.
        original_set(name, value)
    settings.set_setting = tracked_set
    env.sim = settings
    env._enable_contact_processing()
    assert settings.disabled is False
    env._require_contact_processing()
    monkeypatch.setitem(sys.modules, 'omni.kit.viewport.utility',
        SimpleNamespace(get_active_viewport=lambda: None))
    env.close()
    assert settings.writes == [False, previous]
    assert env.contact_reporting['cleanup'] == {'restored_value': previous, 'passed': True}
    assert env.contact_reporting['readback_checks'] == 2


def test_contact_setting_change_fails_before_next_control_physics(env):
    before = (env.episode_tick, len(env.actuator_records))
    env.sim.disabled = True
    with pytest.raises(RuntimeError, match='became disabled'):
        step_hooks(env)
    assert (env.episode_tick, len(env.actuator_records)) == before


def test_contact_enable_requires_successful_boolean_readback(env):
    env._restore_contact_processing()
    env.sim = Settings(None)
    with pytest.raises(RuntimeError, match='boolean native readback'):
        env._enable_contact_processing()
    assert env.sim.writes == []
    env.sim = Settings(True)
    env.sim.set_setting = lambda name, value: None
    with pytest.raises(RuntimeError, match='became disabled'):
        env._enable_contact_processing()
    env._restore_contact_processing()
    assert env.contact_reporting['cleanup']['passed']


def test_constructor_failure_restores_contact_setting(module, monkeypatch):
    settings = Settings(True)
    def failed_base_init(self, cfg, **kwargs):
        self.sim = settings
        self._enable_contact_processing()
        raise RuntimeError('later setup failure')
    monkeypatch.setattr(LabBase, '__init__', failed_base_init)
    cfg = module.LaneFollowingLabCfg()
    cfg.events = cfg.action_noise_model = cfg.observation_noise_model = None
    from uuid import uuid4
    cfg.scene_directory = str(ROOT / 'outputs' / ('unused-contact-constructor-' + uuid4().hex))
    with pytest.raises(RuntimeError, match='later setup failure'):
        module.LaneFollowingLabEnv(cfg)
    assert settings.disabled is True and settings.writes == [False, True]


def test_contact_stage_and_schema_provenance_reject_stale_monitor(env, monkeypatch):
    env.sim.stage = SimpleNamespace(GetPrimAtPath=lambda path: SimpleNamespace(
        HasAPI=lambda schema: True, GetAttribute=lambda name: SimpleNamespace(Get=lambda: 0.0)))
    env.contacts.stage_id, env.contacts.vehicle_path = 14, env.vehicle.path
    cache = SimpleNamespace(GetId=lambda stage: SimpleNamespace(ToLongInt=lambda: 14))
    monkeypatch.setitem(sys.modules, 'pxr', SimpleNamespace(
        PhysxSchema=SimpleNamespace(PhysxContactReportAPI=object()),
        UsdUtils=SimpleNamespace(StageCache=SimpleNamespace(Get=lambda: cache))))
    snapshot = env._contact_reporting_snapshot('test_native_ready')
    assert snapshot['stage_cache_id'] == snapshot['monitor_stage_id'] == 14
    assert snapshot['threshold'] == 0 and snapshot['contact_report_api']
    env.contacts.stage_id = 13
    with pytest.raises(RuntimeError, match='stage/schema identity mismatch'):
        env._contact_reporting_snapshot('stale')


def load_adapter(monkeypatch, decorator=None):
    torch = ModuleType('torch')
    torch.tensor = Tensor
    torch.zeros = lambda shape, dtype=None, **kwargs: Tensor(np.zeros(shape), dtype=dtype)
    torch.float32, torch.bool = np.float32, np.bool_
    gym = ModuleType('gymnasium')
    gym.spaces = SimpleNamespace(Box=lambda *a, **k: Config())
    entries = {
        'torch': torch, 'gymnasium': gym,
        'isaaclab.assets': {'RigidObject': Config, 'RigidObjectCfg': Config},
        'isaaclab.envs': {'DirectRLEnv': LabBase, 'DirectRLEnvCfg': Config},
        'isaaclab.scene': {'InteractiveSceneCfg': Config},
        'isaaclab.sim': {'SimulationCfg': Config},
        'isaaclab.utils': {'configclass': decorator or (lambda cls: cls)},
        'isaaclab_physx.physics': {'PhysxCfg': Config},
        'omni.physx.bindings._physx': {'VEHICLE_WHEEL_STATE_ROTATION_SPEED': 0},
    }
    for name, value in entries.items():
        obj = value if isinstance(value, ModuleType) else ModuleType(name)
        if isinstance(value, dict):
            obj.__dict__.update(value)
        monkeypatch.setitem(sys.modules, name, obj)
    spec = importlib.util.spec_from_file_location('_lane_lab_hook_test', ROOT / 'environments/lane_following_lab.py')
    mod = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, mod)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def module(monkeypatch):
    return load_adapter(monkeypatch)


def test_installed_configclass_can_construct_and_copy_serialized_task(monkeypatch):
    """Run the installed traversal that caused the native pre-episode failure.

    Only unrelated dict/string utility imports are stubs. The configclass
    decorator, post-init recursion and deepcopy/replace are the installed code.
    No Isaac package import, AppLauncher or Kit startup occurs.
    """
    source = ROOT.parent / 'IsaacLab/source/isaaclab/isaaclab/utils/configclass.py'
    if not source.is_file():
        pytest.skip('Installed Isaac Lab configclass source is optional')
    package = '_lane_installed_utils'
    monkeypatch.setitem(sys.modules, package, ModuleType(package))
    for suffix, values in {
        'dict': {'class_to_dict': lambda obj: obj.__dict__.copy(),
                 'update_class_from_dict': lambda *a, **k: None},
        'string': {'ResolvableString': type('ResolvableString', (str,), {})},
    }.items():
        dependency = ModuleType(package + '.' + suffix)
        dependency.__dict__.update(values)
        monkeypatch.setitem(sys.modules, dependency.__name__, dependency)
    spec = importlib.util.spec_from_file_location(package + '.configclass', source)
    installed = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, installed)
    spec.loader.exec_module(installed)
    from dataclasses import FrozenInstanceError
    @installed.configclass
    class ReproducedFailure:
        task = LaneTaskConfig()
    with pytest.raises(FrozenInstanceError, match='physics_hz'):
        ReproducedFailure()
    adapter = load_adapter(monkeypatch, installed.configclass)
    first, second = adapter.LaneFollowingLabCfg(), adapter.LaneFollowingLabCfg()
    copied = first.copy()
    assert first.task == second.task == copied.task == asdict(LaneTaskConfig())
    assert first.task is not second.task and first.task is not copied.task
    first.task['stop_s_m'] = 59.0
    assert second.task['stop_s_m'] == copied.task['stop_s_m'] == 60.0
    with pytest.raises(ValueError, match='frozen limits'):
        LaneTaskConfig(**first.task)
    immutable = LaneTaskConfig(**second.task)
    with pytest.raises(FrozenInstanceError):
        immutable.stop_s_m = 59.0


def state(x=0.0, y=0.0, yaw=0.0):
    return dict(position_m=[x, y, 1.0], quaternion_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                velocity_m_s=[0., 0., 0.], speed_m_s=0., yaw_rad=yaw,
                upright_z=1., wheel_on_ground=[True]*4)


@pytest.fixture
def env(module):
    e = object.__new__(module.LaneFollowingLabEnv)
    e.cfg = SimpleNamespace(task=LaneTaskConfig(), reset_perturb=False,
        command_ttl_ticks=12, drive_torque_per_front_wheel_nm=700., brake_torque_per_wheel_nm=1500.)
    e.route = load_routes(ROOT / 'scenarios/physics-road/routes.json')['straight100']
    e.task = LaneFollowingTask(e.route)
    e.device, e.physics_dt = 'cpu', 1/120
    e.episode_serial = e.episode_tick = e.sequence = e.reset_count = 0
    e._total_physics_ticks = e._sampled_tick = e._unsupported_ticks = 0
    e._reset_seed, e._next_reset_index = 101, 0
    e._reset_options, e.extras = {}, {}
    e._failure = e._transition = e.pending_command = None
    e._step_samples, e._step_contacts = [], []
    e._initialized_episode = False
    e.sim = Settings(True)
    e._contact_settings_owner = None
    e.contact_reporting = None
    e._enable_contact_processing()
    e.initial_height_m = 1.
    e.gate = DriverControlGate('lane-lab-0', 'ego')
    e.applied = AppliedControl(0., 0., 1., 'initial', None, True)
    e.actions = Tensor([[0., 0., 0.]], dtype=np.float32)
    e.data = state()
    e.actuator_records = []
    e.pose_writes = []
    e.contact_ticks = []
    e.injected_contacts = {}

    def pose_write(root_pose, env_ids):
        e.pose_writes.append(root_pose.tolist())
        e.data['position_m'] = root_pose[0, :3].astype(float).tolist()
        e.data['quaternion_xyzw'] = root_pose[0, 3:].astype(float).tolist()
        x, y, z, w = e.data['quaternion_xyzw']
        e.data['yaw_rad'] = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def velocity_write(root_velocity, env_ids):
        e.data['velocity_m_s'] = root_velocity[0, :3].astype(float).tolist()
        e.data['speed_m_s'] = 0.

    def apply(control, drive, brake):
        result = asdict(control)
        e.actuator_records.append(deepcopy(result))
        return result

    def contacts_sample(sim, tick):
        assert not e.contact_ticks or tick > e.contact_ticks[-1]
        e.contact_ticks.append(tick)
        return deepcopy(e.injected_contacts.get(tick, []))

    e.vehicle = SimpleNamespace(path='/World/Vehicles/vehicle_ego/Chassis',
        wheel_paths=['fl', 'fr', 'rl', 'rr'], state=lambda **k: deepcopy(e.data), apply=apply,
        physx=SimpleNamespace(set_vehicle_to_rest_state=lambda path: None,
                             set_wheel_rotation_angle=lambda path, value: None,
                             get_wheel_state=lambda path: [0.]))
    e.chassis = SimpleNamespace(write_root_pose_to_sim_index=pose_write,
                                write_root_velocity_to_sim_index=velocity_write)
    e.contacts = SimpleNamespace(sample=contacts_sample)
    e._physx_sim = object()
    e.reset(seed=101)
    return e


def step_hooks(e, *, substep=None, action=(0., 0.4, 0.)):
    e._pre_physics_step(Tensor([action], dtype=np.float32))
    for index in range(2):
        e._apply_action()
        if substep:
            substep(index, e)
    terminated, truncated = e._get_dones()
    reward = e._get_rewards()
    if bool(terminated[0] or truncated[0]):
        e._reset_idx([0])
    return e._get_observations(), reward, terminated, truncated, e.extras


def test_all_step_record_contains_both_physics_samples_and_no_teleport(env):
    before = len(env.pose_writes)
    _, reward, term, trunc, extras = step_hooks(env)
    assert [s['tick'] for s in extras['lane_transition']['physics_samples']] == [1, 2]
    assert env.contact_ticks == [1, 2]
    assert len(env.pose_writes) == before
    assert reward.tolist() == [0.0] and not term.any() and not trunc.any()
    assert extras['_final_observation'].tolist() == [False]
    assert all(r['brake'] == 1. and r['throttle'] == 0. for r in env.actuator_records)


def test_one_tick_contact_latches_and_terminal_state_survives_autoreset(env):
    env.injected_contacts[1] = [{'tick': 1, 'event_type': 'found'}]
    obs, reward, term, trunc, extras = step_hooks(env)
    record = extras['lane_transition']
    assert record['reason'] == 'rigid_contact'
    assert record['episode_tick'] == 2 and env.episode_tick == 0
    assert term.tolist() == [True] and trunc.tolist() == [False]
    assert extras['_final_observation'].tolist() == [True]
    assert extras['final_terminated'].tolist() == [True]
    assert extras['final_truncated'].tolist() == [False]
    assert extras['final_reward'].tolist() == reward.tolist()
    assert record['physics_samples'][0]['contacts'] == env.injected_contacts[1]
    assert record['physics_samples'][1]['contacts'] == []
    assert record['first_guard_failure']['tick'] == 1
    assert extras['final_observation']['policy'][0, 19] > obs['policy'][0, 19]
    saved = deepcopy(record)
    env.data['position_m'][0] = 42.
    obs['policy'].fill(99.)
    assert record == saved
    assert not np.any(extras['final_observation']['policy'] == 99.)


def test_terminal_extras_do_not_leak_into_next_step(env):
    env.injected_contacts[1] = [{'tick': 1}]
    *_, first = step_hooks(env)
    old_final = first['final_observation']['policy'].copy()
    *_, second = step_hooks(env)
    assert first is not second
    assert first['_final_observation'].tolist() == [True]
    assert second['_final_observation'].tolist() == [False]
    assert np.array_equal(first['final_observation']['policy'], old_final)
    assert [s['tick'] for s in second['lane_transition']['physics_samples']] == [1, 2]
    assert env.contact_ticks == [1, 2, 3, 4]


@pytest.mark.parametrize('action', [(float('nan'), 0., 0.), (0., float('inf'), 0.)])
def test_bad_action_rejected_before_actuation(env, action):
    before = len(env.actuator_records)
    with pytest.raises(ValueError):
        env._pre_physics_step(Tensor([action], dtype=np.float32))
    assert len(env.actuator_records) == before and env.episode_tick == 0


@pytest.mark.parametrize('yaw', [-.035, .035])
def test_bounded_corner_reset_uses_xyzw_and_measured_signed_yaw(env, yaw):
    obs, extras = env.reset(seed=303, options={'lateral_offset_m': .15, 'yaw_error_rad': yaw})
    reset = extras['reset']
    q = reset['physics_state']['quaternion_xyzw']
    assert q[:2] == [0., 0.]
    assert q[2] == pytest.approx(math.sin(yaw/2), abs=1e-8)
    assert q[3] == pytest.approx(math.cos(yaw/2), abs=1e-7)
    assert reset['physics_state']['yaw_rad'] == pytest.approx(yaw, abs=1e-7)
    assert obs['policy'][0, 2] == pytest.approx(.15)
    assert reset['wheel_rotation_speed_rad_s'] == [0.]*4


def test_seeded_reset_sequence_repeats_and_invalid_corner_does_not_move(env):
    env.cfg.reset_perturb = True
    env.reset(seed=42)
    first = deepcopy(env.last_reset['reset_spec'])
    env.reset()
    second = deepcopy(env.last_reset['reset_spec'])
    assert first != second
    env.reset(seed=42)
    assert env.last_reset['reset_spec'] == first
    env.reset()
    assert env.last_reset['reset_spec'] == second
    before = len(env.pose_writes)
    with pytest.raises(ValueError):
        env.reset(options={'lateral_offset_m': .16})
    assert len(env.pose_writes) == before


def test_middle_substep_departure_not_erased_by_final_safe_state(env):
    def move(index, e):
        e.data['position_m'][1] = 2. if index == 0 else 0.
    *_, term, trunc, extras = step_hooks(env, substep=move)
    assert term.tolist() == [True] and trunc.tolist() == [False]
    assert extras['lane_transition']['reason'] == 'lane_departure'
    assert extras['lane_transition']['physics_state']['position_m'][1] == 0.


def test_reward_is_computed_before_reset_not_from_next_episode(env):
    env.injected_contacts[2] = [{'tick': 2}]
    _, reward, _, _, extras = step_hooks(env)
    assert reward[0] < 0
    assert extras['lane_transition']['reward'] == pytest.approx(float(reward[0]))
    assert extras['reset']['episode_serial'] > extras['lane_transition']['episode_serial']


def test_horizon_is_truncated_and_preserves_final_observation(env):
    # Advance the pure accounting consistently; native fake state stays stopped.
    for tick in range(2, 4200, 2):
        env.task.advance(env.data, AppliedControl(0., 0., 1., 'accepted', 1, False), tick=tick)
    env.episode_tick = env._sampled_tick = env._total_physics_ticks = 4198
    env.gate.reset(episode_id=f'lane-lab-{env.episode_serial}', vehicle_id='ego')
    _, _, term, trunc, extras = step_hooks(env)
    assert not term.any() and trunc.tolist() == [True]
    assert extras['lane_transition']['reason'] == 'time_limit'
    assert extras['final_observation']['policy'][0, 19] == 1.
    assert env.episode_tick == 0


def test_support_counter_counts_once_per_tick_and_clears_on_support(env):
    for tick in range(2, 242, 2):
        step_hooks(env)
    env.data['wheel_on_ground'][0] = False
    for _ in range(6):
        *_, extras = step_hooks(env)
    assert extras['lane_transition']['physics_samples'][-1]['unsupported_ticks'] == 12
    env.data['wheel_on_ground'][0] = True
    step_hooks(env)
    assert env._unsupported_ticks == 0
    env.data['wheel_on_ground'][0] = False
    for _ in range(7):
        *_, extras = step_hooks(env)
    assert extras['lane_transition']['reason'] == 'wheel_support_loss'
    assert env._unsupported_ticks == 0  # reset after failure
