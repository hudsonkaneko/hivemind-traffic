"""v15 Phase 0: run v14 validation rows serially or across worker processes, over TraCI or libsumo.

Engineering checks only (user, November 25, 2026: "tell the other session to start phase 0"):
no training, no optimizer, nothing charged to the shared ledger. Each worker process runs one
SUMO simulation at a time (libsumo allows one per process) and drives it with the deterministic
actor, exactly as the v14 runner's validation does (`base.episode` with the round's config).

usage:
  python -m experiments.v15_parallel run OUT --rows 0-13 [--workers N] [--backend traci|libsumo]
         [--round 24] [--actor warm|CHECKPOINT] [--row-set v14-validation|v15-development]
  python -m experiments.v15_parallel compare RUN_A RUN_B

`run` writes OUT/summary.json and one directory per row with its report.json and the episode's
actions.jsonl.gz trace. The summary records what the user's ledger decision needs (November 25,
2026: "use clock time"): the run's clock time, the time summed across workers, the simulated
time, the worker count and the decisions across all workers.
`compare` checks that two runs produced identical reports and per-decision traces for the same
rows, after removing the one field that names the run's own episode directory (VOLATILE).
"""
from __future__ import annotations

import argparse
import gzip
import os
import json
import multiprocessing as mp
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

OUTPUT_ROOT = ROOT / 'outputs'
# The only run-specific field in a v14 report: the episode directory this run wrote (checked on
# round 24's rows: traces carry simulation time only, reports carry no wall-clock field).
VOLATILE = ('episode_directory',)
_STATE = {}
ROW_SETS = ('v14-validation', 'v15-development')   # never the held-out rows (traffic/v15_benchmark.py)


def row_set(cfg, name):
    if name == 'v14-validation':
        return cfg['validation_rows']
    if name == 'v15-development':
        from traffic import v15_benchmark
        return v15_benchmark.development_rows(cfg)
    raise ValueError(f'Unknown row set {name!r}; expected one of {ROW_SETS}')


def setup(backend, round_index, actor, rows='v14-validation'):
    """Configure this process: SUMO coupling, the v14 runner, the round's config and the actor."""
    import numpy as np
    import torch
    from traffic import sumo_coupling
    sumo_coupling.use(backend)
    from experiments import run_highway_flow_rl_v12 as runner
    from experiments import run_highway_flow_rl_v14 as runner14
    from traffic import highway_flow_rl_v14_contract as contract
    runner.install = runner14.install
    runner.install()
    cfg = contract.build_config(round_index)
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(cfg['seed']); np.random.seed(cfg['seed']); random.seed(cfg['seed'])
    model, _ = runner.load_warm_start(cfg)
    if actor != 'warm':
        model.load_state_dict(torch.load(ROOT / actor, map_location='cpu', weights_only=False)['model'])
    model.eval()
    _STATE.update(base=runner.base, cfg=cfg, model=model, backend=backend, rows=row_set(cfg, rows))
    return _STATE


def _run_row(task):
    import torch
    index, out = task
    base, cfg = _STATE['base'], _STATE['cfg']
    directory = Path(out) / f'row-{index:02d}'
    started = time.perf_counter()
    with torch.no_grad():
        report, _ = base.episode(cfg, _STATE['rows'][index], directory / 'episode', _STATE['model'])
    seconds = time.perf_counter() - started
    report = json.loads(json.dumps(report, default=str))
    (directory / 'report.json').write_text(json.dumps(report, sort_keys=True))
    return dict(index=index, seconds=seconds, decisions=int(report.get('agent_transitions') or 0),
                simulated_seconds=float(report.get('simulation_time') or 0.), pid=os.getpid())


def run_rows(indices, out, *, workers=1, backend='traci', round_index=24, actor='warm', rows='v14-validation'):
    """Run validation rows `indices`; workers <= 1 runs them in this process, in order."""
    out = Path(out).resolve()
    if not out.is_relative_to(OUTPUT_ROOT):
        raise ValueError('Outputs must stay inside project outputs')
    out.mkdir(parents=True, exist_ok=False)
    from traffic import highway_flow_rl_v14_contract as contract
    validation_rows = row_set(contract.build_config(round_index), rows)
    if any(not 0 <= int(i) < len(validation_rows) for i in indices):
        raise IndexError(f'Row set {rows} has rows 0-{len(validation_rows) - 1}')
    # Longest rows first so the last worker to finish is not holding a 2000-step row alone.
    order = list(dict.fromkeys(int(i) for i in indices))
    if workers > 1:
        order.sort(key=lambda i: (-validation_rows[i]["horizon_steps"], i))
    tasks = [(i, str(out)) for i in order]
    started = time.perf_counter()
    if workers <= 1:
        setup(backend, round_index, actor, rows)
        results = [_run_row(task) for task in tasks]
    else:
        context = mp.get_context('spawn')
        with context.Pool(workers, initializer=setup, initargs=(backend, round_index, actor, rows)) as pool:
            results = list(pool.imap_unordered(_run_row, tasks, chunksize=1))
    wall = time.perf_counter() - started
    results.sort(key=lambda row: row['index'])
    decisions = sum(row['decisions'] for row in results)
    summary = dict(schema='highway_merge_v15_phase0_run_v01', backend=backend, workers=max(1, workers),
                   round_index=round_index, actor=actor, row_set=rows, rows=[row['index'] for row in results],
                   wall_seconds=wall, clock_seconds=wall, worker_seconds=sum(row['seconds'] for row in results),
                   simulated_seconds=sum(row['simulated_seconds'] for row in results),
                   decisions=decisions, decisions_per_second=decisions / wall if wall else None,
                   per_row=results, charged=False, training=False)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    return summary


def _strip(value):
    if isinstance(value, dict):
        return {k: _strip(v) for k, v in value.items() if k not in VOLATILE}
    if isinstance(value, list):
        return [_strip(v) for v in value]
    return value


def _trace(directory, pattern='episode/*/actions.jsonl.gz'):
    paths = sorted(Path(directory).glob(pattern))
    if len(paths) != 1:
        raise ValueError(f'Expected one episode trace under {directory}, found {len(paths)}')
    with gzip.open(paths[0], 'rt') as handle:
        return [_strip(json.loads(line)) for line in handle]


class _Run:
    """Rows of a `run` output directory."""

    def __init__(self, path):
        self.path = Path(path)
        self.rows = json.loads((self.path / 'summary.json').read_text())['rows']

    def report(self, index):
        return _strip(json.loads((self.path / f'row-{index:02d}' / 'report.json').read_text()))

    def trace(self, index):
        return _trace(self.path / f'row-{index:02d}')


class _TrainerValidation:
    """Rows of a training round's own validation record (LABEL.json plus LABEL/case-XX/episode-*)."""

    def __init__(self, path):
        self.path = Path(path)
        self.reports = json.loads(self.path.with_suffix('.json').read_text())['reports']
        self.rows = list(range(len(self.reports)))

    def report(self, index):
        return _strip(json.loads(json.dumps(self.reports[index], default=str)))

    def trace(self, index):
        return _trace(self.path / f'case-{index:02d}', 'episode-*/actions.jsonl.gz')


def _source(path):
    path = Path(path)
    return _Run(path) if (path / 'summary.json').is_file() else _TrainerValidation(path)


def compare(run_a, run_b):
    """Identical reports and per-decision traces for every row both sources share.

    A source is a `run` output directory or a training round's validation directory
    (for example outputs/highway-flow-rl/v14/round-24/initial-validation)."""
    a, b = _source(run_a), _source(run_b)
    shared = sorted(set(a.rows) & set(b.rows))
    result = dict(run_a=str(run_a), run_b=str(run_b), rows=shared, identical=bool(shared), differences=[],
                  decisions=0)
    for index in shared:
        report_a, report_b = a.report(index), b.report(index)
        trace_a, trace_b = a.trace(index), b.trace(index)
        result['decisions'] += sum(len(line.get('agents', {})) for line in trace_a)
        if report_a != report_b:
            keys = sorted(k for k in set(report_a) | set(report_b) if report_a.get(k) != report_b.get(k))
            result['differences'].append(dict(row=index, where='report', keys=keys[:20]))
        if trace_a != trace_b:
            first = next((i for i, (x, y) in enumerate(zip(trace_a, trace_b)) if x != y),
                         min(len(trace_a), len(trace_b)))
            result['differences'].append(dict(row=index, where='trace', first_step=first,
                                              steps=(len(trace_a), len(trace_b))))
    result['identical'] = bool(shared) and not result['differences']
    return result


def _rows(text):
    out = []
    for part in text.split(','):
        lo, _, hi = part.partition('-')
        out.extend(range(int(lo), int(hi or lo) + 1))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run')
    run.add_argument('out', type=Path)
    run.add_argument('--rows', required=True, type=_rows)
    run.add_argument('--workers', type=int, default=1)
    run.add_argument('--backend', choices=('traci', 'libsumo'), default='traci')
    run.add_argument('--round', type=int, default=24)
    run.add_argument('--actor', default='warm')
    run.add_argument('--row-set', choices=ROW_SETS, default='v14-validation')
    cmp_ = sub.add_parser('compare')
    cmp_.add_argument('run_a', type=Path)
    cmp_.add_argument('run_b', type=Path)
    args = parser.parse_args(argv)
    if args.command == 'run':
        summary = run_rows(args.rows, args.out, workers=args.workers, backend=args.backend,
                           round_index=args.round, actor=args.actor, rows=args.row_set)
        print(json.dumps({k: v for k, v in summary.items() if k != 'per_row'}))
    else:
        result = compare(args.run_a, args.run_b)
        print(json.dumps(result))
        return 0 if result['identical'] else 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
