"""The v15 benchmark rows: the 14 v14 validation cases at 5 development seeds (70 rows), plus 3
held-out seeds per case that are reserved and never run.

v15 plan, "The v15 benchmark": development rows choose; held-out seeds are never used to choose
anything and run once, for the final candidate only, with the user's authorization (Stage 5).

Seeds (chosen in Phase 0, November 25, 2026; checked unused in every outputs/ episode directory):
- Development: 6952001 and 6952002 (round 24's added validation seeds, so 28 of the 70 rows
  already have round-24 records) and three new ones, 6952003-6952005. The cases' own v14
  validation seeds (rows 0-13 of round 24's validation) are left out: rounds 20-23 were judged
  on them, and the round-16 policy passes 8 of 14 cases there against about 4 on other seeds.
- Held-out: 6953001-6953003, new and never run.
All are disjoint from the v14 training seeds (9,100,000 and up), the v02 held-out rows
(8,500,000-8,500,005) and the v14 diagnosis seeds (6,951,001-6,951,011).

Rows are the round-24 validation cases (traffic/highway_flow_rl_v14_contract.build_config(24),
rows 0-13: case, network and its checksum, traffic, horizon) with only the seed changed, ordered
seed by seed: row 14 * k + c is case c at the k-th seed.
"""
from __future__ import annotations

from copy import deepcopy

SCHEMA = 'highway_merge_v15_benchmark_v01'
CASES = 14
DEVELOPMENT_SEEDS = (6952001, 6952002, 6952003, 6952004, 6952005)
HELD_OUT_SEEDS = (6953001, 6953002, 6953003)
SOURCE_ROUND = 24
# Seeds already used for something else; the benchmark must stay disjoint from all of them.
TRAINING_SEEDS_FROM = 9_100_000
V02_HELD_OUT = range(8_500_000, 8_500_006)
V14_DIAGNOSIS = range(6_951_001, 6_951_012)


def cases(cfg):
    """The 14 v14 validation cases (round 24's rows 0-13, each at its own seed)."""
    rows = cfg['validation_rows'][:CASES]
    if len(rows) != CASES or len({row['case']['id'] for row in rows}) != CASES:
        raise ValueError('Expected the 14 distinct v14 validation cases')
    return rows


def _at(rows, seeds):
    out = []
    for seed in seeds:
        for row in rows:
            moved = deepcopy(row)
            moved['seed'] = seed
            out.append(moved)
    return out


def development_rows(cfg):
    """The 70 development rows."""
    return _at(cases(cfg), DEVELOPMENT_SEEDS)


def held_out_rows(cfg, *, authorized=False):
    """The 42 reserved held-out rows. Only the final candidate, only with the user's authorization."""
    if authorized is not True:
        raise PermissionError('The v15 held-out rows are reserved: they run once, for the final candidate, '
                              'only with the user\'s authorization')
    return _at(cases(cfg), HELD_OUT_SEEDS)


def manifest(cfg):
    """A record of the benchmark: case ids, seeds and what is reserved (no held-out row is built)."""
    return dict(schema=SCHEMA, source=f'v14 round {SOURCE_ROUND} validation rows 0-{CASES - 1}',
                cases=[row['case']['id'] for row in cases(cfg)],
                development_seeds=list(DEVELOPMENT_SEEDS), development_rows=CASES * len(DEVELOPMENT_SEEDS),
                held_out_seeds=list(HELD_OUT_SEEDS), held_out_rows=CASES * len(HELD_OUT_SEEDS),
                held_out_status='reserved; never run; final candidate only, with the user\'s authorization',
                row_order='row 14 * k + c is case c at development seed k')
