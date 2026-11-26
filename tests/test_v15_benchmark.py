"""v15 benchmark rows: 14 cases x 5 development seeds, 3 held-out seeds per case reserved (no SUMO)."""
import importlib

import pytest

from traffic import highway_flow_rl_v14_contract as contract
from traffic import v15_benchmark as bench


@pytest.fixture(scope='module')
def round24():
    return contract.build_config(24)


def _training_seeds(cfg):
    """Every training seed a v14 round of up to 400 episodes can draw (as the round-24 contract test)."""
    rows, seeds = cfg['training_rows'], set()
    for round_index in range(1, 25):
        for critic in (False, True):
            for cursor in range(400):
                seed = rows[cursor % len(rows)]['seed'] + 1000 * (cursor // len(rows))
                seeds.add(seed + 10000 * (round_index - 1) + (50000 if critic else 0))
    return seeds


def test_seventy_development_rows_are_the_fourteen_cases_at_five_seeds(round24):
    rows = bench.development_rows(round24)
    cases = round24['validation_rows'][:14]
    assert len(rows) == 70 and len({r['case']['id'] for r in rows}) == 14
    for k, seed in enumerate(bench.DEVELOPMENT_SEEDS):
        for c, case in enumerate(cases):
            row = rows[14 * k + c]
            assert row['seed'] == seed and dict(row, seed=case['seed']) == case


def test_round24_added_seed_rows_are_development_rows_unchanged(round24):
    """28 of the 70 rows are round 24's validation rows 14-41, so their round-24 records carry over."""
    assert bench.development_rows(round24)[:28] == round24['validation_rows'][14:42]


def test_seeds_are_disjoint_from_every_other_use(round24):
    development, held_out = set(bench.DEVELOPMENT_SEEDS), set(bench.HELD_OUT_SEEDS)
    assert len(development) == 5 and len(held_out) == 3 and not development & held_out
    own = {row['seed'] for row in round24['validation_rows'][:14]}
    used = (_training_seeds(round24) | {row['seed'] for row in round24['held_out_rows']}
            | set(bench.V14_DIAGNOSIS) | own)
    assert not (development | held_out) & used
    assert not held_out & {row['seed'] for row in round24['validation_rows']}
    assert min(_training_seeds(round24)) >= bench.TRAINING_SEEDS_FROM
    assert {row['seed'] for row in round24['held_out_rows']} <= set(bench.V02_HELD_OUT)


def test_held_out_rows_need_explicit_authorization(round24):
    with pytest.raises(PermissionError):
        bench.held_out_rows(round24)
    with pytest.raises(PermissionError):
        bench.held_out_rows(round24, authorized='yes')
    rows = bench.held_out_rows(round24, authorized=True)
    assert len(rows) == 42 and {r['seed'] for r in rows} == set(bench.HELD_OUT_SEEDS)


def test_manifest_records_the_reserve_without_building_held_out_rows(round24):
    record = bench.manifest(round24)
    assert record['development_rows'] == 70 and record['held_out_rows'] == 42
    assert record['held_out_seeds'] == [6953001, 6953002, 6953003]
    assert 'never run' in record['held_out_status']


def test_the_parallel_runner_offers_no_held_out_row_set(round24):
    runner = importlib.import_module('experiments.v15_parallel')
    assert runner.ROW_SETS == ('v14-validation', 'v15-development')
    assert len(runner.row_set(round24, 'v15-development')) == 70
    with pytest.raises(ValueError):
        runner.row_set(round24, 'v15-held-out')
