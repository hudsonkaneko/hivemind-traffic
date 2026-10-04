"""CPU-only repeatability/lifecycle acceptance checks; no Isaac import."""
from copy import deepcopy
import json
from pathlib import Path

from scripts.probe_vehicle_resets import assess_resets, validate_config
from traffic.physics_session import PhysicsSession

ROOT = Path(__file__).resolve().parents[1]


def fixture():
    config = json.loads((ROOT/'experiments/physics-resets.json').read_text())
    episode = dict(passed=True, final_state={'position_m': [3., 0., 1.], 'speed_m_s': 0.},
                   process_after_discard={'available': True, 'rss_bytes': 5*1024**3},
                   closed_lifecycle={'cache_count': 0, 'old_stage_cached_after_erase': False,
                                     'context_has_stage': False})
    return config, [deepcopy(episode) for _ in range(config['repeats'])]


def test_ten_identical_reset_summaries_pass():
    config, episodes = fixture()
    result = assess_resets(episodes, config)
    assert result['passed']
    assert result['maximum_repeat_position_difference_m'] == 0


def test_last_episode_difference_is_measured():
    config, episodes = fixture()
    episodes[-1]['final_state']['position_m'][0] += .11
    assert not assess_resets(episodes, config)['gates']['repeat_position']


def test_cache_growth_fails():
    config, episodes = fixture()
    episodes[5]['closed_lifecycle']['cache_count'] = 1
    assert not assess_resets(episodes, config)['gates']['stage_cache_bounded']


def test_postwarm_memory_peak_not_just_final_is_checked():
    config, episodes = fixture()
    episodes[4]['process_after_discard']['rss_bytes'] += 257*2**20
    result = assess_resets(episodes, config)
    assert not result['gates']['provisional_rss_bound']
    assert result['post_warmup_rss_growth_mib'] == 257


def test_unavailable_memory_does_not_silently_pass():
    config, episodes = fixture()
    episodes[-1]['process_after_discard'] = {'available': False}
    assert not assess_resets(episodes, config)['passed']


def test_incomplete_episode_count_fails():
    config, episodes = fixture()
    assert not assess_resets(episodes[:-1], config)['gates']['complete']


def test_invalid_rate_rejected_before_isaac_import():
    import pytest
    for value in [0, -1, True, 120.5]:
        with pytest.raises(ValueError):
            PhysicsSession(value)


def test_checked_in_configs_validate():
    for name in ('physics-resets.json', 'physics-reset-control.json'):
        config = json.loads((ROOT/'experiments'/name).read_text())
        assert validate_config(config) is config


def test_unbounded_or_nonfinite_configs_fail_before_isaac():
    import pytest
    config, _ = fixture()
    for key, value in [('repeats', 11), ('physics_hz', True), ('control_hz', 120),
                       ('target_speed_m_s', float('nan')), ('max_process_seconds', 181),
                       ('command_ttl_ticks', 13), ('drive_torque_per_front_wheel_nm', 701),
                       ('brake_torque_per_wheel_nm', float('inf')), ('disable_authoring_tools', 1)]:
        changed = deepcopy(config)
        changed[key] = value
        with pytest.raises(ValueError):
            validate_config(changed)


def test_phase_order_and_total_budget_are_bounded():
    import pytest
    config, _ = fixture()
    for phases in [[['straight', 3], ['settle', 1], ['brake', 2]],
                   [['settle', 1], ['straight', 4], ['brake', 2]],
                   [['settle', 1], ['straight', 3.00001], ['brake', 2]]]:
        changed = deepcopy(config)
        changed['phases'] = phases
        with pytest.raises(ValueError):
            validate_config(changed)


def test_memory_gate_cannot_be_disabled_by_no_comparable_samples():
    import pytest
    config, _ = fixture()
    config['acceptance']['memory_warmup_episodes'] = config['repeats']
    with pytest.raises(ValueError):
        validate_config(config)
