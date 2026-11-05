"""CPU-only configuration, scheduling and before-guard evidence checks."""
from copy import deepcopy
import math
from types import SimpleNamespace

import numpy as np
import pytest

from experiments import near_limit_support as support
from experiments import run_highway_population as runner


def config(version='v4'):
    return runner.load_config(runner.ROOT / f'experiments/configs/highway_population_{version}.json')


def test_v04_is_disabled_bounded_and_retains_all_original_validation_rows():
    cfg, legacy = config(), config('v3')
    assert cfg['training_authorized'] is False
    assert cfg['preflight']['learning'] is False
    assert cfg['training']['device'] == 'cpu'
    assert cfg['training']['max_agent_transitions'] == 100000
    assert cfg['training']['max_wall_seconds'] == 1800
    assert cfg['speed_curriculum']['ceilings_m_s'] == [12., 18., 22.352, 26.8224]
    assert cfg['validation_cases'][:16] == legacy['validation_cases']
    assert len(cfg['validation_cases']) == 24
    assert {(r['speed_limit_m_s'], r['case']['task'], r['controlled_count'])
            for r in cfg['validation_cases'][16:]} == {
        (speed, task, population) for speed in (12., 18.)
        for task in ('cruise', 'passing') for population in (1, 5)}
    assert cfg['initial_checkpoint'] == legacy['initial_checkpoint']
    assert cfg['output_directory'] == 'outputs/highway-population/v04'


def test_each_four_training_episodes_cover_all_ceilings_without_partition_leakage():
    cfg = config()
    rows = [runner.training_case(cfg, i) for i in range(180)]
    assert rows == [runner.training_case(cfg, i) for i in range(180)]
    for start in range(0, len(rows), 4):
        assert sorted(r['speed_limit_m_s'] for r in rows[start:start + 4]) == support.SPEEDS
    assert {r['case']['task'] for r in rows} == {'cruise', 'following', 'passing', 'merge'}
    assert {r['controlled_count'] for r in rows} == {1, 2, 4, 6, 8}
    assert all(r['case']['split'] == 'train' for r in rows)
    assert not {r['seed'] for r in rows} & {r['seed'] for r in cfg['validation_cases']}
    assert all(r['controlled_count'] < r['total_vehicles'] for r in rows
               if r['case']['task'] in ('passing', 'following'))
    # Adding the independently seeded speed schedule does not reorder the same
    # population/task/road schedule when the experiment seed is held fixed.
    no_curriculum = deepcopy(cfg)
    no_curriculum['revision'] = 'population-v03-aligned-control'
    assert [{k: v for k, v in r.items() if k != 'speed_limit_m_s'} for r in rows] == [
        runner.training_case(no_curriculum, i) for i in range(180)]


@pytest.mark.parametrize('version', ['v1', 'v2', 'v3'])
def test_legacy_schedules_do_not_opt_into_speed_curriculum(version):
    cfg = config(version)
    assert support.enabled(cfg) is False
    assert support.speed_for_episode(cfg, 0) is None
    assert len(cfg['validation_cases']) == 16
    assert all('speed_limit_m_s' not in runner.training_case(cfg, i) for i in range(12))


@pytest.mark.parametrize('index', [-1, 1.5, True, '0'])
def test_speed_schedule_rejects_invalid_episode_index(index):
    with pytest.raises(ValueError, match='nonnegative integer'):
        support.speed_for_episode(config(), index)


@pytest.mark.parametrize('version,ceiling', [('v3', None), ('v4', 12.), ('v4', 26.8224)])
def test_env_receives_explicit_ceiling_only_when_requested(tmp_path, monkeypatch, version, ceiling):
    from environments import highway_population_env
    cfg = config(version)
    row = runner.training_case(cfg, 0)
    if ceiling is not None:
        row['speed_limit_m_s'] = ceiling
    calls = []

    def constructor(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace()

    monkeypatch.setattr(highway_population_env, 'HighwayPopulationEnv', constructor)
    monkeypatch.setattr(runner, 'digest', lambda path: row['case']['network_sha256'])
    env = runner.env_for(cfg, tmp_path, row)
    args, kwargs = calls.pop()
    assert args == (runner.ROOT / row['case']['network'], tmp_path)
    assert kwargs['max_agents'] == 8 and kwargs['horizon'] == 800
    assert kwargs['control_target_version'] == 'v3'
    assert env.case_id == row['case']['id']
    assert env.near_limit_metrics_enabled is (version == 'v4')
    if ceiling is None:
        assert 'speed_limit_m_s' not in kwargs
    else:
        assert kwargs['speed_limit_m_s'] == ceiling


def test_changed_road_is_rejected_before_constructing_environment(tmp_path, monkeypatch):
    from environments import highway_population_env
    cfg = config()
    monkeypatch.setattr(runner, 'digest', lambda path: 'changed')
    monkeypatch.setattr(highway_population_env, 'HighwayPopulationEnv',
                        lambda *a, **k: pytest.fail('Constructed environment before source verification'))
    with pytest.raises(ValueError, match='Frozen road checksum changed'):
        runner.env_for(cfg, tmp_path, runner.training_case(cfg, 0))


@pytest.mark.parametrize('edit', [
    lambda c: c['optimizer'].update(near_limit_coef=0.),
    lambda c: c['optimizer'].update(near_limit_band_m_s=2.),
    lambda c: c.update(objective_version='unversioned'),
    lambda c: c.update(output_directory='outputs/highway-population/v03'),
    lambda c: c['speed_curriculum'].update(ceilings_m_s=[12., 18., 22.352, 27.]),
    lambda c: c['speed_curriculum'].update(ceilings_m_s=[12., 18., 22.352, float('nan')]),
    lambda c: c['validation_cases'].pop(),
    lambda c: c['validation_cases'][0].update(speed_limit_m_s=12.),
    lambda c: c['validation_cases'][16].update(speed_limit_m_s=22.352),
    lambda c: c['validation_cases'][16].update(seed=c['training']['seed_start']),
    lambda c: c['environment'].update(control_target_version='v2'),
])
def test_v04_contract_failures_are_rejected(edit):
    cfg = config()
    edit(cfg)
    with pytest.raises(ValueError):
        runner.validate_config(cfg)


def test_v04_cannot_bypass_objective_validation_using_legacy_control_and_output():
    cfg = config()
    cfg['environment']['control_target_version'] = 'v2'
    cfg['output_directory'] = 'outputs/highway-population/v01'
    cfg['optimizer']['near_limit_coef'] = 0.
    with pytest.raises(ValueError):
        runner.validate_config(cfg)


def test_speed_curriculum_and_objective_are_in_preflight_fingerprint():
    cfg = config()
    before = runner.preparation_digest(cfg)
    for mutate in (
        lambda c: c['speed_curriculum']['ceilings_m_s'].__setitem__(0, 13.),
        lambda c: c['optimizer'].update(near_limit_coef=.2),
        lambda c: c['validation_cases'][16].update(speed_limit_m_s=18.),
    ):
        changed = deepcopy(cfg)
        mutate(changed)
        assert runner.preparation_digest(changed) != before
    assert 'environments/highway_population_env.py' in runner.SOURCES
    assert 'experiments/near_limit_support.py' in runner.SOURCES


def observation(speed, limit=12.):
    value = np.zeros(93, dtype=np.float64)
    value[0], value[4] = speed / support.SPEED_CAP, limit / support.SPEED_CAP
    return value


def action(acceleration):
    return np.array([(acceleration + 4.5) / 3.25 - 1., 0.])


def test_raw_metrics_known_si_values_do_not_change_inputs_or_guard_accounting():
    metrics = support.RawNearLimitMetrics()
    examples = [(10., 2., 1.), (11., 1., .5), (11.9, 2., 0.),
                (12., .04, -.1), (12., .06, 0.), (12.1, 0., -1.)]
    for speed, requested, target in examples:
        obs, chosen, teacher = observation(speed), action(requested), action(target)
        original = [value.copy() for value in (obs, chosen, teacher)]
        metrics.add(obs, chosen, teacher)
        assert all(np.array_equal(a, b) for a, b in zip((obs, chosen, teacher), original))
    report = metrics.report()
    assert report['all_decisions'] == 6
    assert report['near_limit_decisions'] == 5
    assert report['raw_teacher_rmse_m_s2'] == pytest.approx(math.sqrt(5.2732 / 5))
    assert report['mean_raw_request_m_s2'] == pytest.approx(.62)
    assert report['mean_teacher_target_m_s2'] == pytest.approx(-.12)
    assert report['raw_speed_ceiling_excess_count'] == 3
    assert report['raw_speed_ceiling_excess_fraction'] == pytest.approx(.6)
    assert report['maximum_raw_ceiling_excess_m_s2'] == pytest.approx(1.)
    assert report['at_limit_decisions'] == 2
    assert report['positive_at_limit_count'] == 1
    assert report['positive_at_limit_fraction'] == .5
    assert report['diagnostic_tolerance_m_s2'] == .05
    assert 'do not redefine safety accounting' in report['scope']


def test_raw_metrics_empty_bin_remains_missing_instead_of_perfect():
    metrics = support.RawNearLimitMetrics()
    metrics.add(observation(10.), action(2.), action(1.))
    report = metrics.report()
    assert report['all_decisions'] == 1 and report['near_limit_decisions'] == 0
    for field in ('raw_teacher_rmse_m_s2', 'mean_raw_request_m_s2',
                  'raw_speed_ceiling_excess_fraction', 'maximum_raw_ceiling_excess_m_s2',
                  'positive_at_limit_fraction'):
        assert report[field] is None


@pytest.mark.parametrize('bad_input', ['speed', 'limit', 'action', 'teacher'])
def test_raw_metrics_reject_nonfinite_values_without_partial_count(bad_input):
    metrics = support.RawNearLimitMetrics()
    obs, chosen, teacher = observation(12.), action(0.), action(0.)
    if bad_input == 'speed':
        obs[0] = float('nan')
    elif bad_input == 'limit':
        obs[4] = float('inf')
    elif bad_input == 'action':
        chosen[0] = float('inf')
    else:
        teacher[0] = float('nan')
    with pytest.raises(ValueError, match='Nonfinite'):
        metrics.add(obs, chosen, teacher)
    assert metrics.report()['all_decisions'] == 0
