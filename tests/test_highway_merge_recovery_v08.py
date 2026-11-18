import json
from pathlib import Path
from copy import deepcopy
import pytest
from traffic.highway_merge_recovery_v08 import recovery_terms, write_cutin_demand
from traffic.highway_merge_recovery_v08_contract import validate_config

ROOT = Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT / 'configs/training/highway-merge-recovery-2026-11-17.json').read_text())


def require_generated_networks(rows):
    """Skip in a clean clone: these ramp networks are generated locally under ignored outputs/."""
    missing = [row['case']['network'] for row in rows if not (ROOT / row['case']['network']).is_file()]
    if missing:
        pytest.skip('Needs locally generated ramp network not stored in Git: ' + missing[0])


def test_dense_tracking_still_improves_when_error_exceeds_two():
    target = dict(eligible=True, target_m_s=25.)
    a, _ = recovery_terms(19., 19., target, 1., 2., 2., True)
    b, _ = recovery_terms(19., 20., target, 1., 2., 2., True)
    assert b['merge_speed_tracking_v08'] > a['merge_speed_tracking_v08']


def test_merge_alignment_prefers_acceleration_and_respects_braking():
    target = dict(eligible=True, target_m_s=25.)
    slow, _ = recovery_terms(15., 15., target, 0., 2., 2., True)
    brisk, desired = recovery_terms(15., 15., target, 2., 2., 2., True)
    assert desired == 2. and brisk['merge_acceleration_alignment'] > slow['merge_acceleration_alignment']
    safe, desired = recovery_terms(15., 15., target, -3., -3., 2., True)
    assert desired == -3. and safe['merge_acceleration_alignment'] == 0.


def test_speed_limit_riding_has_direct_cost_and_no_braking_penalty():
    target = dict(eligible=False)
    a, _ = recovery_terms(20., 20., target, .05, 0., 0., False)
    b, _ = recovery_terms(20., 20., target, -.5, 0., 0., False)
    assert a['positive_limit_request'] == pytest.approx(-.125)
    assert b['positive_limit_request'] == 0.


def test_contract_preserves_budget_and_requires_actual_cutins():
    cfg = config()
    validate_config(cfg, True)
    for edit in ('initial_policy_std', 'v08_settings'):
        bad = deepcopy(cfg)
        bad[edit] = None
        with pytest.raises(ValueError):
            validate_config(bad, True)
    bad = deepcopy(cfg)
    next(r for r in bad['training_rows'] if r.get('cutin_stress'))['cutin_coverage_required'] = False
    with pytest.raises(ValueError):
        validate_config(bad, True)


def test_seeded_cutins_are_real_adjacent_spawn_pairs(tmp_path):
    from experiments.run_highway_merge_recovery_v08 import env_for
    cfg = config()
    rows = [r for r in cfg['training_rows'] if r.get('cutin_stress')]
    require_generated_networks(rows)
    for i, row in enumerate(rows):
        env = env_for(cfg, row, tmp_path / str(i))
        for seed in (100, 811, 9919, row['seed'] + 70000):
            path = tmp_path / f'{i}-{seed}.xml'
            manifest, spec = write_cutin_demand(path, seed, env.graph, row['speed_limit_m_s'])
            ego = next(r for r in manifest if r['vehicle_id'] == spec['ego'])
            cutter = next(r for r in manifest if r['vehicle_id'] == spec['cutter'])
            assert len(manifest) == 8 and ego['lane'] + 1 == cutter['lane']
            assert .5 < (cutter['position'] - 4.8 - ego['position']) / ego['speed'] < 1.5
            assert cutter['speed'] > ego['speed']
            original = path.read_bytes()
            write_cutin_demand(path, seed, env.graph, row['speed_limit_m_s'])
            assert path.read_bytes() == original


def test_empty_stress_episode_is_rejected(monkeypatch, tmp_path):
    from experiments import run_highway_merge_recovery_v08 as runner
    cfg = config()
    row = next(r for r in cfg['training_rows'] if r.get('cutin_stress'))
    monkeypatch.setattr(runner, '_episode', lambda *a, **k: ({'cutin_fixture': {}}, None))
    with pytest.raises(RuntimeError, match='exposure missing'):
        runner.episode(cfg, row, tmp_path, None)
