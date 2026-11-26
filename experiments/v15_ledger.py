"""v15 charged evaluation runs on the shared study ledger.

User decisions (November 25, 2026, relayed by the main Claude session and recorded in the
coordination master):
- plan v15 decision 5, "use clock time": a run charges its elapsed clock time, however many SUMO
  workers it uses; the decision budget counts every decision across all workers; every run records
  its clock time, the summed worker time, the summed simulated time and the worker count;
- "Yes, all four", for Phase 1: a decision allowance for Phase 1 only, sized to its resolved expected
  charge and at most 400,000 decisions; the study's time cap unchanged.

How a v15 evaluation run is written to the ledger:
- **The ledger** is the study ledger every v05-v14 run uses, `outputs/highway-flow-rl/v02/cost-ledger.json`
  beside Git's common directory (experiments/run_highway_flow_rl_v09.shared_study_directory), and its
  `active.lock`. The lock is taken with exclusive create: if another run holds it, the session refuses
  to start and never overrides it.
- **The segment** is category 'preparation', command 'v15_evaluation', evaluation_only. Those are the
  categories the ledger's own totals check accepts (run_highway_flow_rl_v05.ledger_totals), so every
  existing runner still reads the ledger. Its clock time counts in the study's total wall time, as
  every evaluation segment's did before it.
- **Episodes** are recorded as evaluation_episodes, not in `episodes`, so they do not count against
  the diagnostic-episode budget (174); the user approved Phase 1 as its own evaluation budget.
- **Decisions** count against the phase's own allowance (a v15 total, `totals()['v15']`). The v05
  totals count training decisions only, so the training decision budget is unchanged.
- **While running**, the segment is charged its whole time reservation, as v05's sessions are; at the
  end, its elapsed clock time. A segment left 'running' (a killed process) stops every runner until it
  is reconciled by hand, as for any other run.

Phase 3 (imitation; user decision of November 26, 2026, "Recommended size": at most 2,700,000 decisions and
3,000 s of clock time as Phase 3's own allowance) uses the same segments with v15_phase '3', plus:
- **v15_work**: what the segment did, any of 'collect' (training-row episodes that record Level 2 samples),
  'fit' (training a Level 2 network in the learner process) and 'evaluation' (development rows);
- **training_episodes** for the training-row episodes (not evaluation_episodes, not diagnostic episodes),
  and **fits** and **fit_seconds**. A fit runs inside the session, so its clock time is part of the charge;
- category 'preparation' and evaluation_only false, so the ledger's own totals check still accepts every
  segment and the v05 training-decision total (counted from 'training' segments only) is untouched.
Phase 3's time cap is applied by its runner (experiments/v15_phase3.py): each run's reservation is at most
what is left of the 3,000 s.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMMAND = 'v15_evaluation'
CATEGORY = 'preparation'
CHARGE_RULE = ('clock time (plan v15 decision 5); every decision across workers, the canaries\' re-runs included; '
               'episodes as evaluation_episodes, not diagnostic episodes')


class LockHeld(RuntimeError):
    """Another run holds the shared ledger's lock. Wait for it; never override it."""


class BudgetExceeded(RuntimeError):
    """The run would exceed (or has reached) its decision allowance or the study's time cap."""


def study_directory():
    from experiments.run_highway_flow_rl_v09 import shared_study_directory
    return shared_study_directory()


def _save(path, value):
    from experiments.run_highway_flow_rl_v05 import save
    save(Path(path), value)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError(f'Invalid {name}')
    return value


def totals(ledger):
    """The ledger's own totals (run_highway_flow_rl_v05.ledger_totals, which also checks every segment is
    closed and of a known category), plus the v15 evaluation sums by phase."""
    from experiments.run_highway_flow_rl_v05 import ledger_totals
    result = ledger_totals(ledger, -1)
    v15 = {}
    for segment in ledger.get('segments', []):
        if segment.get('command') != COMMAND:
            continue
        if segment.get('category') != CATEGORY or segment.get('episodes', 0) != 0:
            raise ValueError('A v15 evaluation segment must be preparation with its episodes in evaluation_episodes')
        decisions = segment.get('decisions', 0)
        if type(decisions) is not int or decisions < 0:
            raise ValueError('Invalid v15 evaluation decisions')
        phase = v15.setdefault(segment.get('v15_phase'), dict(segments=0, decisions=0, seconds=0., evaluation_episodes=0))
        phase['segments'] += 1
        phase['decisions'] += decisions
        phase['seconds'] += float(segment['charged_seconds'])
        phase['evaluation_episodes'] += int(segment.get('evaluation_episodes', 0))
        for name in ('training_episodes', 'fits'):          # Phase 3 segments only
            if name in segment:
                value = segment[name]
                if type(value) is not int or value < 0:
                    raise ValueError(f'Invalid v15 {name}')
                phase[name] = phase.get(name, 0) + value
        if 'fit_seconds' in segment:
            phase['fit_seconds'] = phase.get('fit_seconds', 0.) + float(_number(segment['fit_seconds'], 'fit seconds'))
    result['v15'] = v15
    return result


class EvaluationSession:
    """One charged v15 evaluation segment, holding the shared lock for its whole run.

    with EvaluationSession(...) as session:   # takes the lock, checks the budget, writes the segment
        session.check_next(expected)          # before each batch: refuses one that would pass the allowance
        session.add_batch(batch)              # after each batch: charges it (clock time, decisions, ...)
    """

    def __init__(self, *, phase, output, config_sha256, expected_decisions, decision_allowance,
                 expected_clock_seconds, reserve_seconds, total_wall_seconds, intended_episodes, workers,
                 authorization, round_index, priority='normal', study=None, clock=time.monotonic):
        self.phase, self.output, self.config_sha256 = str(phase), Path(output), str(config_sha256)
        self.expected_decisions = int(_number(expected_decisions, 'expected decisions'))
        self.decision_allowance = int(_number(decision_allowance, 'decision allowance'))
        self.expected_clock_seconds = float(_number(expected_clock_seconds, 'expected clock time'))
        self.reserve_seconds = float(_number(reserve_seconds, 'time reservation'))
        self.total_wall_seconds = float(_number(total_wall_seconds, 'study time cap'))
        self.intended_episodes, self.workers, self.priority = int(intended_episodes), int(workers), str(priority)
        self.authorization, self.round_index = str(authorization), int(round_index)
        self.study = Path(study) if study is not None else study_directory()
        self.path, self.lock = self.study / 'cost-ledger.json', self.study / 'active.lock'
        self.clock = clock
        self.segment = None

    def __enter__(self):
        if not self.path.is_file():
            raise ValueError(f'No study ledger at {self.path}; never start a new one')
        try:
            with self.lock.open('x', encoding='utf-8') as stream:
                stream.write(str(self.output))
        except FileExistsError:
            holder = self.lock.read_text(encoding='utf-8', errors='replace').strip()
            raise LockHeld(f'The shared ledger lock is held by {holder!r}; wait for that run to end') from None
        try:
            self.started = self.clock()
            self.ledger = json.loads(self.path.read_text())
            self.before = totals(self.ledger)
            used = self.before['v15'].get(self.phase, {}).get('decisions', 0)
            self.allowance_left = self.decision_allowance - used
            if self.expected_decisions > self.allowance_left:
                raise BudgetExceeded(f'Expected {self.expected_decisions:,} decisions; {self.allowance_left:,} of the '
                                     f'phase\'s {self.decision_allowance:,} remain')
            seconds_left = self.total_wall_seconds - self.before['total']
            allowed = min(seconds_left, self.reserve_seconds)
            if self.expected_clock_seconds > allowed:
                raise BudgetExceeded(f'Expected {self.expected_clock_seconds:.0f} s of clock time; {seconds_left:.1f} s '
                                     f'remain in the study and the run reserves {self.reserve_seconds:.0f} s')
            self.allowed_seconds = allowed
            try:
                output = self.output.resolve().relative_to(ROOT).as_posix()
            except ValueError:
                output = self.output.as_posix()
            self.segment = dict(command=COMMAND, category=CATEGORY, v15_phase=self.phase, round_index=self.round_index,
                                output=output, status='running', charged_seconds=allowed, reserved_seconds=allowed,
                                decisions=0, episodes=0, evaluation_only=True, evaluation_episodes=0,
                                intended_episodes=self.intended_episodes, expected_decisions=self.expected_decisions,
                                expected_clock_seconds=self.expected_clock_seconds,
                                decision_allowance=self.decision_allowance, workers=self.workers,
                                process_priority=self.priority, worker_seconds=0.,
                                simulated_seconds=0., batches=0, canary_decisions=0, charge_rule=CHARGE_RULE,
                                config_sha256=self.config_sha256, authorization=self.authorization,
                                started_at_unix=time.time())
            self.ledger['segments'].append(self.segment)
            _save(self.path, self.ledger)
            return self
        except BaseException:
            self.lock.unlink(missing_ok=True)
            raise

    def elapsed(self):
        return max(0., self.clock() - self.started)

    @property
    def decisions_left(self):
        return self.allowance_left - self.segment['decisions']

    def check_next(self, expected_decisions):
        """Refuse a batch whose expected decisions would pass the phase's allowance, or a run out of time."""
        if expected_decisions > self.decisions_left:
            raise BudgetExceeded(f'The next batch expects {expected_decisions:,.0f} decisions; {self.decisions_left:,} '
                                 'remain in the allowance')
        if self.elapsed() >= self.allowed_seconds:
            raise BudgetExceeded(f'The run has used its {self.allowed_seconds:.0f} s reservation')

    def add_batch(self, batch):
        """Charge one pool batch (experiments.v15_pool.BatchResult). Raises once the allowance is passed."""
        segment = self.segment
        segment['decisions'] += int(batch.charged_decisions)
        segment['canary_decisions'] += int(batch.canary_decisions)
        segment['evaluation_episodes'] += int(batch.simulated_tasks) + int(batch.canary is not None)
        segment['worker_seconds'] += float(batch.worker_seconds)
        segment['simulated_seconds'] += float(batch.simulated_seconds)
        segment['batches'] += 1
        segment['clock_seconds_so_far'] = self.elapsed()   # the charge stays the full reservation until the end
        _save(self.path, self.ledger)
        if segment['decisions'] > self.allowance_left:
            raise BudgetExceeded(f'Charged {segment["decisions"]:,} decisions, past the {self.allowance_left:,} left')

    def charge_estimate(self, decisions, note):
        """Charge a batch that failed before reporting its count (a crashed task): its expected decisions."""
        self.segment['decisions'] += int(math.ceil(decisions))
        self.segment.setdefault('estimated_charges', []).append(dict(decisions=int(math.ceil(decisions)), note=str(note)))
        _save(self.path, self.ledger)

    def __exit__(self, kind, error, tb):
        try:
            if self.segment is not None:
                self.segment.pop('clock_seconds_so_far', None)
                self.segment.update(status='failed' if error is not None else 'complete', charged_seconds=self.elapsed(),
                                    clock_seconds=self.elapsed())
                if error is not None:
                    self.segment['failure'] = f'{type(error).__name__}: {error}'[:500]
                _save(self.path, self.ledger)
        finally:
            self.lock.unlink(missing_ok=True)
