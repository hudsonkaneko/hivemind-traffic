"""Native render profiling is bounded and restores state without loading Kit."""
import json
from types import SimpleNamespace

import pytest

from visualization import showcase_render_profile as profile_module
from visualization.showcase_render_profile import RenderProfile


class Settings:
    def __init__(self, values):
        self.values = dict(values)
        self.fail_once = None

    def get(self, name):
        return self.values.get(name)

    def set(self, name, value):
        self.values[name] = value
        if self.fail_once == (name, value):
            self.fail_once = None
            raise RuntimeError('setter failed after changing value')

    set_bool = set

    def destroy_item(self, name):
        self.values.pop(name, None)


@pytest.fixture
def native(monkeypatch):
    state = SimpleNamespace(update=100, mask=7, mask_writes=[],
                            cpu_nodes=[dict(name='frame', duration=8., indent=0),
                                       dict(name='tiny', duration=.05, indent=1),
                                       dict(name='Hydra', duration=7., indent=1)],
                            gpu_nodes=[[dict(name='GPU', duration=4., indent=0),
                                        dict(name=None, query=8, duration=.1, indent=1)]],
                            capture_error=None, restore_mask_error=False)
    settings = Settings({profile_module._GPU_ENABLED: False, profile_module._CPU_SAVE: True})

    def set_mask(value):
        state.mask_writes.append(value)
        if state.restore_mask_error and value == 7:
            raise RuntimeError('mask restore failed')
        state.mask = value

    def snapshot():
        if state.capture_error:
            raise state.capture_error
        return SimpleNamespace(get_main_thread_id=lambda: 12,
                               get_profile_events=lambda thread: state.cpu_nodes)

    runtime = dict(app=SimpleNamespace(get_update_number=lambda: state.update),
                   settings=settings,
                   cpu=SimpleNamespace(get_capture_mask=lambda: state.mask, set_capture_mask=set_mask),
                   monitor=SimpleNamespace(get_last_profile_events=snapshot),
                   gpu=SimpleNamespace(get_gpu_profiler_result=lambda: state.gpu_nodes))
    monkeypatch.setattr(profile_module, '_load_runtime', lambda viewport: runtime)
    return state, settings, runtime


def test_native_window_auto_closes_and_restores_exact_state(native):
    state, settings, _ = native
    profile = RenderProfile(object())
    assert state.mask == 0xFFFFFFFFFFFFFFFF
    assert not settings.values[profile_module._CPU_SAVE]
    assert settings.values[profile_module._GPU_ENABLED]
    for update in range(101, 161):
        state.update = update
        assert profile.after_render()
    assert state.mask == 7
    assert settings.values == {profile_module._GPU_ENABLED: False, profile_module._CPU_SAVE: True}
    state.update = 161
    assert not profile.after_render()
    profile.close()
    report = profile.summary()
    assert report['sampled_updates'] == 60
    assert report['diagnostic_status'] == 'captured'
    assert report['settings_restored']
    assert not report['performance_qualification_eligible']
    json.dumps(report, allow_nan=False)


def test_threshold_preserves_original_hierarchy_and_inclusive_root_total(native):
    state, _, _ = native
    with RenderProfile(object(), max_updates=1) as profile:
        state.update += 1
        profile.after_render()
    row = profile.summary()['samples'][0]
    assert row['cpu']['root_total_ms'] == 8.  # Do not add the nested 7 ms again.
    assert row['cpu']['raw_node_count'] == 3
    assert row['cpu']['nodes'] == [dict(name='frame', duration_ms=8., indent=0, raw_index=0),
                                   dict(name='Hydra', duration_ms=7., indent=1, raw_index=2)]
    assert row['gpu'][0]['nodes'][1]['name'] == 'query-8'
    assert row['gpu'][0]['nodes'][1]['duration_ms'] == .1


def test_no_extra_samples_for_duplicate_updates_and_no_extended_capture(native):
    state, _, _ = native
    profile = RenderProfile(object(), max_updates=2)
    assert not profile.after_render()
    state.update = 101
    assert profile.after_render()
    assert not profile.after_render()
    state.update = 103
    assert not profile.after_render()
    assert profile.summary()['sampled_updates'] == 1
    assert profile.summary()['settings_restored']
    assert profile.summary()['window_exhausted']


def test_missing_native_nodes_are_reported_as_missing_evidence(native):
    state, _, _ = native
    state.cpu_nodes = []
    state.gpu_nodes = []
    profile = RenderProfile(object(), max_updates=1)
    state.update += 1
    profile.after_render()
    report = profile.summary()
    assert report['diagnostic_status'] == 'missing_native_data'
    assert not report['cpu_data_available']
    assert not report['gpu_data_available']
    assert report['settings_restored']


def test_early_close_restores_preexisting_and_absent_settings(native):
    state, settings, _ = native
    settings.values = {profile_module._GPU_ENABLED: True}
    profile = RenderProfile(object())
    profile.close()
    profile.close()
    assert settings.values == {profile_module._GPU_ENABLED: True}
    assert state.mask == 7
    assert profile.summary()['diagnostic_status'] == 'no_samples'


def test_setup_failure_after_partial_write_restores_state(native):
    state, settings, _ = native
    settings.fail_once = (profile_module._GPU_ENABLED, True)
    with pytest.raises(RuntimeError, match='setup failed.*setter failed'):
        RenderProfile(object())
    assert settings.values == {profile_module._GPU_ENABLED: False, profile_module._CPU_SAVE: True}
    assert state.mask == 7


def test_unavailable_runtime_is_an_explicit_setup_error(monkeypatch):
    def unavailable(viewport):
        raise ImportError('omni.hydra.engine.stats unavailable')
    monkeypatch.setattr(profile_module, '_load_runtime', unavailable)
    with pytest.raises(RuntimeError, match='setup failed.*omni.hydra.engine.stats'):
        RenderProfile(object())


@pytest.mark.parametrize('failure', ['native_exception', 'nan', 'counter_reset'])
def test_capture_failures_restore_every_setting_and_remain_failed(native, failure):
    state, settings, _ = native
    profile = RenderProfile(object())
    state.update += 1
    if failure == 'native_exception':
        state.capture_error = RuntimeError('native snapshot failed')
    elif failure == 'nan':
        state.cpu_nodes[0]['duration'] = float('nan')
    else:
        state.update = 99
    with pytest.raises(RuntimeError, match='capture failed'):
        profile.after_render()
    assert state.mask == 7
    assert settings.values == {profile_module._GPU_ENABLED: False, profile_module._CPU_SAVE: True}
    assert profile.summary()['diagnostic_status'] == 'failed'
    assert profile.summary()['settings_restored']
    with pytest.raises(RuntimeError, match='previously failed'):
        profile.after_render()


def test_restore_failure_still_attempts_other_settings_and_can_retry(native):
    state, settings, _ = native
    profile = RenderProfile(object())
    state.restore_mask_error = True
    with pytest.raises(RuntimeError, match='restoration failed.*mask restore failed'):
        profile.close()
    assert settings.values == {profile_module._GPU_ENABLED: False, profile_module._CPU_SAVE: True}
    assert not profile.summary()['settings_restored']
    state.restore_mask_error = False
    profile.close()
    assert state.mask == 7
    assert profile.summary()['settings_restored']
    assert profile.summary()['diagnostic_status'] == 'failed'


@pytest.mark.parametrize('options', [dict(max_updates=0), dict(max_updates=61),
                                    dict(max_updates=True), dict(minimum_ms=0),
                                    dict(minimum_ms=float('nan'))])
def test_invalid_bounds_fail_before_runtime_loading(monkeypatch, options):
    def forbidden(viewport):
        raise AssertionError('Runtime must not load')
    monkeypatch.setattr(profile_module, '_load_runtime', forbidden)
    with pytest.raises(ValueError):
        RenderProfile(object(), **options)
