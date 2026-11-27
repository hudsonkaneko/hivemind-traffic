"""Shared parts for new v15 runners: configuration pinning, the lock and ledger, pool dispatch and report writing.

Repository cleanup, user decision of November 27, 2026, relayed by the main Claude session: "Yes, as recommended"
(plan v15, "Repository cleanup"; coordination master, "repository cleanup approved as recommended"). Going forward
only: the runners of recorded phases (experiments/v15_phase1.py ... v15_physics_gap.py) keep their own frozen copies of
these pieces, and no recorded configuration, hash, ledger entry, result or evidence file is rewritten or moved. A new
runner uses this module, and its tests in tests/test_v15_runner.py cover each piece. Where a piece is a copy of a
recorded runner's code, a test checks that the two agree.

Slim configurations. A new runner commits only its decided settings and the canonical hash of its full resolved
configuration (`write_slim`). The generated sections (`GENERATED`: batch lists, row lists, the shared benchmark manifest,
the parity row, the baseline and the training rows) are rebuilt by the runner's resolve() at run time. Before dispatch,
`pin` refuses a run whose resolved configuration differs from the committed settings or hash, and `write_resolved`
keeps the full resolved configuration in the run's outputs/ folder. The hash is the one the recorded runners pass to
the ledger: sha256 of json.dumps(config, sort_keys=True), so it doesn't depend on whitespace or key order.

Saved results. Where results go into Git, `write_results` writes the full results as a deterministic .json.gz and a
short summary with their hashes; `read_results` checks both hashes.

Limits. `check_limits` keeps the user's CPU limit: at most 16 SUMO workers, at below-normal priority.

usage, in a new runner:
    resolved = json.loads(json.dumps(resolve(step)))
    sha = R.pin(config_path(step), resolved)                 # ConfigMismatch: re-resolve and record it first
    R.check_limits(resolved); R.check_fit(resolved['charge'])
    R.write_resolved(out, resolved)
    with R.session(resolved, sha, out, phase=PHASE, decision_allowance=ALLOWANCE) as session, P.WorkerPool(...) as pool:
        result, pool_tasks = R.dispatch(session, pool, batch['tasks'], index=0, run=f'{name}-{step}',
                                        background=BACKGROUND, projected=R.projected(batch['expected_decisions']))
    R.write_charge(out, session)
"""
from __future__ import annotations

import gzip
import hashlib
import io
import json
from pathlib import Path

SLIM_SCHEMA = 'highway_merge_v15_slim_config_v01'
RESULTS_SCHEMA = 'highway_merge_v15_results_summary_v01'
# the generated sections of a resolved configuration (79% of the 18 recorded v15 configurations' lines)
GENERATED = ('batches', 'rows', 'benchmark', 'training_rows', 'baseline', 'parity')
MAX_WORKERS, PRIORITY = 16, 'below_normal'       # the user's CPU limit (November 25, 2026)
MARGIN = 0.05                                    # experiments/v15_phase3.MARGIN: the projected charge's margin
TRAIN = 'v15-training'                           # experiments/v15_phase3.TRAIN: training-row episodes


class ConfigMismatch(ValueError):
    """The resolved configuration is not the committed one: re-resolve and record it first."""


class ResultsMismatch(ValueError):
    """Saved results don't match their summary's hashes."""


# --------------------------------------------------------------------------- configurations

def canonical_sha(value):
    """sha256 of json.dumps(value, sort_keys=True): the configuration hash the v15 runners give the ledger."""
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def dumps(value):
    """The runners' file format: indent 1, sorted keys, one trailing newline."""
    return json.dumps(value, indent=1, sort_keys=True) + '\n'


def _write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(text)
    return path


def split(config, generated=GENERATED):
    """(decided settings, the generated keys present): the settings are every top-level key not generated."""
    if not isinstance(config, dict):
        raise TypeError('A configuration is a JSON object')
    present = sorted(k for k in config if k in set(generated))
    return {k: v for k, v in config.items() if k not in set(generated)}, present


def slim(config, generated=GENERATED):
    """The committed form of a resolved configuration: its decided settings and its canonical hash."""
    config = json.loads(json.dumps(config))
    settings, present = split(config, generated)
    return dict(schema=SLIM_SCHEMA, resolved_sha256=canonical_sha(config), generated=present, settings=settings,
                note='Generated sections are rebuilt by the runner at run time; the full resolved configuration is '
                     'written to the run\'s outputs/ folder (experiments/v15_runner.py).')


def write_slim(path, config, generated=GENERATED):
    """Write the slim form of `config` to `path` (LF line endings); returns it."""
    form = slim(config, generated)
    _write(path, dumps(form))
    return form


def read_slim(path):
    form = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(form, dict) or form.get('schema') != SLIM_SCHEMA:
        raise ConfigMismatch(f'{path} is not a slim configuration ({SLIM_SCHEMA})')
    for key, kind in (('resolved_sha256', str), ('generated', list), ('settings', dict)):
        if not isinstance(form.get(key), kind):
            raise ConfigMismatch(f'{path} has no valid {key!r}')
    return form


def pin(path, resolved):
    """Check a freshly resolved configuration against the committed slim one; returns its hash.

    Refuses (ConfigMismatch) if a decided setting differs, a generated section is added or dropped, or the full
    resolved configuration's canonical hash differs from the committed one (a generated section changed)."""
    form = read_slim(path)
    resolved = json.loads(json.dumps(resolved))
    settings, present = split(resolved, form['generated'])
    unknown = sorted(set(present) ^ set(form['generated']))
    if unknown:
        raise ConfigMismatch(f'Generated sections differ from {path}: {unknown}')
    changed = sorted(k for k in set(settings) | set(form['settings']) if settings.get(k) != form['settings'].get(k))
    if changed:
        raise ConfigMismatch(f'The resolved settings differ from {path} in {changed}; re-resolve and record it first')
    sha = canonical_sha(resolved)
    if sha != form['resolved_sha256']:
        raise ConfigMismatch(f'The resolved configuration hashes to {sha}, not {path}\'s {form["resolved_sha256"]}: '
                             f'a generated section ({form["generated"]}) changed; re-resolve and record it first')
    return sha


def write_resolved(out, resolved, name='resolved-config.json'):
    """Keep the full resolved configuration in the run's outputs/ folder; returns (path, hash)."""
    path = _write(Path(out) / name, dumps(resolved))
    sha = canonical_sha(json.loads(path.read_text(encoding='utf-8')))
    if sha != canonical_sha(resolved):
        raise ConfigMismatch(f'{path} does not read back as the resolved configuration')
    return path, sha


# --------------------------------------------------------------------------- limits and charge

def check_limits(config):
    """The user's CPU limit: at most 16 workers, at below-normal priority."""
    if int(config['workers']) > MAX_WORKERS:
        raise ValueError(f'At most {MAX_WORKERS} workers (the user\'s CPU limit), not {config["workers"]}')
    if config.get('priority') != PRIORITY:
        raise ValueError(f'Workers and the learner run at {PRIORITY} priority (the user\'s CPU limit), not '
                         f'{config.get("priority")!r}')


def projected(expected_decisions, measured_ratio=1., margin=MARGIN):
    """A batch's projected charge (experiments/v15_phase4._projected): expected decisions at the measured rate (never
    below 1) plus the margin."""
    return expected_decisions * max(1., float(measured_ratio)) * (1 + margin)


def check_fit(charge):
    """Stop before dispatch when the projected charge or the expected clock time doesn't fit what is left
    (experiments/v15_phase4._check_fit)."""
    from experiments import v15_ledger as L
    planned = charge.get('projected_decisions', projected(charge['expected_decisions']))
    if planned > charge['decisions_left'] or charge['expected_clock_seconds'] > charge['seconds_left']:
        raise L.BudgetExceeded(f'Projected {planned:,.0f} decisions and {charge["expected_clock_seconds"]} s; '
                               f'{charge["decisions_left"]:,} decisions and {charge["seconds_left"]} s are left. Stop and '
                               'ask: the batch is not trimmed to fit')


# --------------------------------------------------------------------------- the lock and ledger

def session(config, sha, out, *, phase, decision_allowance, study=None, clock=None):
    """The ledger session (experiments/v15_ledger.EvaluationSession) for a resolved configuration: it takes the shared
    lock on entry and charges clock time and every decision. Not entered; use `with`."""
    from experiments import v15_ledger as L
    check_limits(config)
    charge = config['charge']
    extra = {} if clock is None else dict(clock=clock)
    return L.EvaluationSession(phase=phase, output=out, config_sha256=sha, expected_decisions=charge['expected_decisions'],
                               decision_allowance=decision_allowance,
                               expected_clock_seconds=charge['expected_clock_seconds'],
                               reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                               intended_episodes=charge['expected_episodes'], workers=config['workers'],
                               priority=config['priority'], authorization=config['authorization'],
                               round_index=config['round_index'], study=study, **extra)


def charge(session, result, pool_tasks, training_row_set=TRAIN):
    """Charge a batch (experiments/v15_phase3._add): its training-row episodes are counted as training episodes."""
    from experiments.v15_ledger import _save
    session.add_batch(result)
    simulated = [r for r in result.records if not r.get('reused')]
    training = sum(1 for r in simulated if r['fields']['row_set'] == training_row_set)
    canary_training = int(result.canary is not None and any(r['key'] == result.canary['key']
                                                            and r['fields']['row_set'] == training_row_set
                                                            for r in simulated))
    session.segment['evaluation_episodes'] -= training + canary_training
    session.segment['training_episodes'] = session.segment.get('training_episodes', 0) + training + canary_training
    _save(session.path, session.ledger)


# --------------------------------------------------------------------------- pool dispatch

def make_task(P, t, run, background):
    """A pool task from a configuration's task entry (experiments/v15_phase3._task)."""
    return P.Task(t['row_set'], t['row'], t['driver'], background=background, mode=t.get('mode', 'validate'),
                  sample_seed=t.get('sample_seed', 0), label_with=t.get('label_with'), perturb=t.get('perturb'), run=run)


def dispatch(session, pool, tasks, *, index, run, background, projected, P=None):
    """Run one batch on the pool and charge it; returns (result, pool tasks).

    Before dispatch the session refuses a batch that would pass the allowance or the time reservation. A canary
    mismatch charges the batch it reports (or the projection if there is none); any other failure before the batch
    reports charges the projection. Either way the error is raised again."""
    if P is None:
        from experiments import v15_pool as P
    session.check_next(projected)
    pool_tasks = [make_task(P, t, run, background) for t in tasks]
    try:
        result = pool.run(pool_tasks)
    except P.CanaryMismatch as error:
        if error.result is not None:
            charge(session, error.result, pool_tasks)
        else:
            session.charge_estimate(projected, f'batch {index}: {error}')
        raise
    except BaseException as error:
        session.charge_estimate(projected, f'batch {index} failed before reporting: {type(error).__name__}')
        raise
    charge(session, result, pool_tasks)
    return result, pool_tasks


def check_canary(result, index, P=None):
    """After a charged batch is logged: stop (CanaryMismatch) if it simulated anything without a matched canary
    (experiments/v15_phase3 and v15_phase4)."""
    if P is None:
        from experiments import v15_pool as P
    if result.simulated_tasks and (result.canary is None or not result.canary['matched']):
        raise P.CanaryMismatch(f'Batch {index}: canary {result.canary}')


# --------------------------------------------------------------------------- reports and results

class Log:
    """A run's JSON-lines log: each entry is appended and printed."""

    def __init__(self, path, echo=True):
        self.path, self.echo = Path(path), echo

    def __call__(self, entry):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, 'a', encoding='utf-8', newline='\n') as handle:
            handle.write(json.dumps(entry) + '\n')
        if self.echo:
            print(json.dumps(entry), flush=True)


CHARGE_FIELDS = ('status', 'charged_seconds', 'clock_seconds', 'decisions', 'canary_decisions', 'evaluation_episodes',
                 'worker_seconds', 'simulated_seconds', 'workers', 'batches', 'decision_allowance', 'expected_decisions',
                 'started_at_unix', 'config_sha256', 'failure')


def write_charge(out, session):
    """charge.json: the session's ledger segment, summarized (experiments/v15_stage5._write_charge)."""
    segment = session.segment
    summary = {k: segment.get(k) for k in CHARGE_FIELDS}
    summary['ledger'] = str(session.path)
    _write(Path(out) / 'charge.json', json.dumps(summary, indent=1))
    return summary


def write_report(out, result, markdown=None):
    """report.json, and report.md when a Markdown renderer is given."""
    _write(Path(out) / 'report.json', json.dumps(result, indent=1))
    if markdown is not None:
        _write(Path(out) / 'report.md', markdown(result))


def _gzip_bytes(value):
    """Deterministic gzip of the canonical JSON (no file name, mtime 0), so the same results give the same bytes."""
    buffer = io.BytesIO()
    with gzip.GzipFile(filename='', mode='wb', fileobj=buffer, mtime=0, compresslevel=9) as handle:
        handle.write(dumps(value).encode('utf-8'))
    return buffer.getvalue()


def write_results(directory, name, results, summary=None):
    """Full results as <name>.json.gz plus <name>.summary.json with their hashes; returns the summary.

    `summary` is the short, human-readable part (counts, verdicts); the hashes are of the compressed file's bytes and of
    the results' canonical JSON."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    data = _gzip_bytes(results)
    gz = directory / f'{name}.json.gz'
    gz.write_bytes(data)
    entries = len(results) if isinstance(results, (list, dict)) else None
    head = dict(schema=RESULTS_SCHEMA, results=gz.name, gzip_sha256=hashlib.sha256(data).hexdigest(),
                content_sha256=canonical_sha(results), entries=entries, summary=summary or {})
    _write(directory / f'{name}.summary.json', dumps(head))
    return head


def read_results(summary_path):
    """The full results named by a summary written by `write_results`, after checking both hashes."""
    summary_path = Path(summary_path)
    head = json.loads(summary_path.read_text(encoding='utf-8'))
    if head.get('schema') != RESULTS_SCHEMA:
        raise ResultsMismatch(f'{summary_path} is not a results summary ({RESULTS_SCHEMA})')
    data = (summary_path.parent / head['results']).read_bytes()
    if hashlib.sha256(data).hexdigest() != head['gzip_sha256']:
        raise ResultsMismatch(f'{head["results"]} does not match its summary\'s gzip hash')
    results = json.loads(gzip.decompress(data).decode('utf-8'))
    if canonical_sha(results) != head['content_sha256']:
        raise ResultsMismatch(f'{head["results"]} does not match its summary\'s content hash')
    return results


def line_counts(paths, generated=GENERATED):
    """{path: (lines as committed, lines of its slim form)} for recorded configurations; nothing is written."""
    out = {}
    for path in paths:
        text = Path(path).read_text(encoding='utf-8')
        out[str(path)] = (len(text.splitlines()), len(dumps(slim(json.loads(text), generated)).splitlines()))
    return out
