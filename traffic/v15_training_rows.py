"""v15 Phase 3 training rows: round 24's 20 training rows at new seeds, for behaviour cloning and DAgger.

Plan v15, "Phase 3 proposal", approved November 26, 2026 ("Recommended size", relayed by the main Claude
session). The rows are round 24's `training_rows` (configs/training/highway-flow-rl-v14-round24.json;
traffic/highway_flow_rl_v14_contract.build_config(24)) with only the seed changed. Seed set k gives row i the
seed SEED_BASE + 1000 k + i, so row 20 k + i of the 'v15-training' row set is training row i at seed set k.

Seeds (checked November 26, 2026): 9,700,000-9,739,019 are in the training range (9,100,000 and up) and no
seed field in any outputs/ or configs/ JSON, JSONL or SUMO file uses 9,700,000-9,739,999. Every seed a v14
round could draw (row seed + 1000 x cursor + 10000 x (round - 1) + 50000 for the critic, rounds 1-24,
cursors under 400) is at most 9,599,011, and the v15 development (6,952,001-6,952,005), v15 held-out
(6,953,001-6,953,003), v02 held-out (8,500,000-8,500,005) and v14 diagnosis (6,951,001-6,951,011) seeds are
all outside it. tests/test_v15_level2.py checks the disjointness.

Never the development or held-out rows: those are traffic/v15_benchmark's, and selection uses only them
(development) or nothing (held-out).
"""
from __future__ import annotations

from copy import deepcopy

SCHEMA = 'highway_merge_v15_training_rows_v01'
ROWS = 20
SEED_BASE = 9_700_000
SETS = 40                                  # seed sets 0-39 (data, DAgger iterations); 800 rows


def seed(k, i):
    if not (0 <= k < SETS and 0 <= i < ROWS):
        raise ValueError(f'No v15 training seed for set {k}, row {i}')
    return SEED_BASE + 1000 * k + i


def base_rows(cfg):
    rows = cfg['training_rows']
    if len(rows) != ROWS:
        raise ValueError(f'Expected round 24\'s {ROWS} training rows')
    return rows


def rows(cfg):
    """All 800 rows: row 20 k + i is training row i at seed set k."""
    out = []
    for k in range(SETS):
        for i, row in enumerate(base_rows(cfg)):
            moved = deepcopy(row)
            moved['seed'] = seed(k, i)
            out.append(moved)
    return out


def index(k, i):
    seed(k, i)
    return ROWS * k + i


def manifest(cfg):
    return dict(schema=SCHEMA, source='v14 round 24 training_rows', cases=[r['case']['id'] for r in base_rows(cfg)],
                seed_rule=f'row 20 k + i: training row i at seed {SEED_BASE} + 1000 k + i', seed_sets=SETS)
