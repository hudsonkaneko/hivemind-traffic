"""v15 worker pool: many SUMO workers feeding one learner, with cross-checks built in.

User direction, relayed November 25, 2026: "ok we should run multiple sims and have them work
together to improve learning speed and cross references". Phase 0 builds and measures the pool
only: nothing here trains or updates a policy, and nothing is charged.

- **Workers.** Each worker is its own process with one SUMO simulation at a time (libsumo allows
  one per process), set up once for a v14 round's configuration. The learner (the process that
  owns the WorkerPool) holds the policy.
- **Synchronous learning loop** (PPO, Stage 4): each iteration the learner publishes its weights
  (`publish`, content-addressed by hash), the workers collect episodes in parallel with exactly
  those weights, the samples come back, and only then may the learner update. Every sample in an
  update comes from one policy version, so PPO stays on-policy and runs are reproducible.
  `learner_round_trips` measures the loop's overhead with no update.
- **Shared dataset** (cloning and DAgger, Stages 2-3): a task with `label_with` also records the
  labelling driver's deterministic action on every visited state; the pool appends one line per
  episode to `dataset/index.jsonl`, the workers write the sample files.
- **Results are keyed by what was run, never by who ran it:** config round, row set, row, case,
  seed, driver (policy hash), background driver model, mode, sample seed and labeller. Sampled
  actions use a torch seed derived from that key, so the same task gives the same result on any
  worker. The run name and the worker are recorded but are not part of the key.
- **Canary:** every batch re-runs one task on a different worker; its report, trace and samples
  must match exactly, or the batch fails.
- **Paired validation:** `paired_tasks` runs every driver on the same rows and seeds under each
  background model, into one results table (`results.jsonl`); `paired_table` lines them up.
- **Interleaved runs:** `interleave` merges several runs' tasks row by row, so the 3 training runs
  of a setup share one pool and are compared on identical rows.
- **Drivers.** A published policy ('policy:<hash>'), or a rule driver, which validates only: the
  plan follower (traffic/v15_plan_follower.py, 'rule:plan-follower-v1') or the Phase 2 rule expert
  (traffic/v15_rule_expert.py, 'rule:expert-v3'; run 1's 'rule:expert-v1' and run 2's 'rule:expert-v2' are kept
  byte for byte in traffic/v15_rule_expert_v1.py and traffic/v15_rule_expert_v2.py so those runs can be repeated).
  Phase 3 adds the Level 2 drivers (traffic/v15_level2.py): 'l2:rule-v3', v3 split into its executor and the rule
  Level 2, and 'l2:<hash>', a published Level 2 network (publish_level2) with the same executor. They validate,
  or collect Level 2 samples labelled by the rule ('l2:rule-v3' as label_with): the rule's own data, with
  optional perturbations (Task.perturb), or DAgger rollouts of a network. Their samples are written to
  level2.pt in the episode directory and hashed like any other samples.
- **Background drivers.** The v14 demand writer gives every background car one of three IDM
  profiles (traffic/highway_v05_demand.py, PROFILES; checked in the routes files of round 24's
  validation). 'idm' is that recorded traffic; 'krauss' rewrites those three vTypes to SUMO's
  Krauss model with the same parameters, as the robustness variant. Our own cars' native model
  stays as configured (Krauss in round 24).

Budget (user decision of November 25, "use clock time"): a run charges its elapsed clock time,
however many workers it uses, and every decision across workers, the canaries' re-runs included
(BatchResult.charged_decisions). Each batch records its clock time, the summed worker time, the summed
simulated time and the worker count. The pool itself charges nothing: experiments/v15_ledger.py does.
"""
from __future__ import annotations

import copy
import hashlib
import io
import json
import multiprocessing as mp
import os
import random
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

OUTPUT_ROOT = ROOT / 'outputs'
SCHEMA = 'highway_merge_v15_pool_v01'
ROW_SETS = ('v14-validation', 'v14-training', 'v15-development', 'v15-training')   # never the held-out rows
BACKGROUNDS = ('idm', 'krauss')
MODES = ('validate', 'collect')
BACKGROUND_PROFILES = ('native_cautious', 'native_balanced', 'native_attentive')
VOLATILE = ('episode_directory',)
RULE_DRIVERS = ('rule:plan-follower-v1', 'rule:expert-v1', 'rule:expert-v2', 'rule:expert-v3')   # follower, expert v1-v3
# Process priority of the workers (and, set by the caller, the learner). User direction, November 25, 2026,
# relayed by the main Claude session: "don't use all cpu cores i need some to do other tasks"; so at most
# 16 workers and below-normal priority. Priority and worker count do not change results.
PRIORITIES = ('normal', 'below_normal')
_WINDOWS_PRIORITY = {'normal': 0x20, 'below_normal': 0x4000}   # NORMAL_ / BELOW_NORMAL_PRIORITY_CLASS


class CanaryMismatch(RuntimeError):
    """A task re-run on another worker gave a different result. `result` is the batch, for charging."""

    def __init__(self, message, result=None):
        super().__init__(message)
        self.result = result


@dataclass(frozen=True)
class Task:
    row_set: str
    row: int
    driver: str                      # 'policy:<hash>' from WorkerPool.publish, or one of RULE_DRIVERS
    background: str = 'idm'
    mode: str = 'validate'           # 'validate': deterministic actor; 'collect': sampled actions + samples
    sample_seed: int = 0             # collect: which draw (for example the iteration)
    label_with: str | None = None    # a driver whose deterministic actions label the visited states
    run: str = 'default'             # who asked; recorded, not part of the key
    perturb: str | None = None       # Level 2 rule data only: the perturbation version (traffic/v15_level2)

    def __post_init__(self):
        if self.row_set not in ROW_SETS:
            raise ValueError(f'Unknown row set {self.row_set!r}; expected one of {ROW_SETS}')
        if self.background not in BACKGROUNDS or self.mode not in MODES:
            raise ValueError(f'Unknown background {self.background!r} or mode {self.mode!r}')
        if is_level2(self.driver):
            from traffic import v15_level2 as level2
            if self.mode == 'collect' and self.label_with != level2.RULE_DRIVER:
                raise ValueError(f'Level 2 samples are labelled by {level2.RULE_DRIVER}')
            if self.mode == 'validate' and (self.label_with is not None or self.perturb is not None):
                raise ValueError('A Level 2 validation has no labels and no perturbations')
            if self.perturb is not None and (self.driver != level2.RULE_DRIVER
                                             or self.perturb != level2.PERTURBATION['version']):
                raise ValueError(f'Perturbations are for {level2.RULE_DRIVER} only, version '
                                 f'{level2.PERTURBATION["version"]}')
            return
        if self.perturb is not None:
            raise ValueError('Only Level 2 rule data takes perturbations')
        if self.driver in RULE_DRIVERS:
            if self.mode != 'validate':
                raise ValueError(f'{self.driver} validates only: it has no sampled actions to collect')
        elif not self.driver.startswith('policy:'):
            raise ValueError(f'Unknown driver {self.driver!r}: a published policy (policy:<hash>) or one of {RULE_DRIVERS}')
        if self.label_with and not self.label_with.startswith('policy:'):
            raise ValueError('Labels come from published policies; the rule expert labels in Phase 2')


def is_level2(driver):
    """A Level 2 driver: 'l2:rule-v3' or a published Level 2 network 'l2:<16 hex>'."""
    if not isinstance(driver, str) or not driver.startswith('l2:'):
        return False
    from traffic import v15_level2
    name = driver.split(':', 1)[1]
    return driver == v15_level2.RULE_DRIVER or (len(name) == 16 and all(c in '0123456789abcdef' for c in name))


def set_priority(priority):
    """Set this process's scheduling priority ('normal' or 'below_normal'); returns the priority read back.

    On Windows, processes this one starts afterwards inherit a below-normal class."""
    if priority not in PRIORITIES:
        raise ValueError(f'Unknown priority {priority!r}; expected one of {PRIORITIES}')
    if os.name == 'nt':
        import ctypes
        from ctypes import wintypes
        kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        kernel32.SetPriorityClass.argtypes = (wintypes.HANDLE, wintypes.DWORD)
        kernel32.GetPriorityClass.argtypes = (wintypes.HANDLE,)
        kernel32.GetPriorityClass.restype = wintypes.DWORD
        handle = kernel32.GetCurrentProcess()
        if not kernel32.SetPriorityClass(handle, _WINDOWS_PRIORITY[priority]):
            raise OSError(ctypes.get_last_error(), 'SetPriorityClass failed')
        read = kernel32.GetPriorityClass(handle)
        return next((name for name, value in _WINDOWS_PRIORITY.items() if value == read), hex(read))
    if priority == 'below_normal' and os.nice(0) < 10:
        os.nice(10 - os.nice(0))
    return 'below_normal' if os.nice(0) >= 10 else 'normal'


def key_fields(task, round_index, row):
    """Everything that decides a result. Not the run name, not the worker."""
    fields = dict(round=round_index, row_set=task.row_set, row=task.row, case=row['case']['id'], seed=row['seed'],
                  driver=task.driver, background=task.background, mode=task.mode,
                  sample_seed=task.sample_seed if task.mode == 'collect' else 0, label_with=task.label_with)
    if task.perturb is not None:          # only when set, so every earlier key is unchanged
        fields['perturb'] = task.perturb
    return fields


def task_key(fields):
    return hashlib.sha256(json.dumps(fields, sort_keys=True).encode()).hexdigest()


def plan_batch(keys, known, *, reuse=True):
    """Which of a batch's tasks to simulate.

    Each key runs at most once per batch; its later tasks share that result. With `reuse`, a key that
    already has a result in `known` is not simulated again. Returns (indices to run, {key: first index})."""
    first = {}
    for index, key in enumerate(keys):
        first.setdefault(key, index)
    run = [index for index, key in enumerate(keys) if first[key] == index and not (reuse and key in known)]
    return run, first


def free_attempt(episode_dir, stem):
    """An attempt directory name under `episode_dir` that does not exist yet: stem, stem-2, stem-3, ...
    (an earlier batch, or an episode that crashed before recording, may hold the plain name)."""
    episode_dir = Path(episode_dir)
    if not (episode_dir / stem).exists():
        return stem
    n = 2
    while (episode_dir / f'{stem}-{n}').exists():
        n += 1
    return f'{stem}-{n}'


def torch_seed(key):
    return int(key[:8], 16)


def state_hash(state_dict):
    """Content hash of a policy's weights (names, shapes, dtypes and values)."""
    import torch
    digest = hashlib.sha256()
    for name in sorted(state_dict):
        tensor = state_dict[name].detach().cpu().contiguous()
        digest.update(name.encode())
        digest.update(str((tuple(tensor.shape), str(tensor.dtype))).encode())
        digest.update(tensor.numpy().tobytes() if tensor.dtype != torch.bfloat16 else tensor.float().numpy().tobytes())
    return digest.hexdigest()[:16]


def set_background(path, model):
    """Rewrite the background vTypes of a routes file to `model` ('idm' leaves the file as written)."""
    if model == 'idm':
        return
    if model != 'krauss':
        raise ValueError(f'Unknown background model {model!r}')
    tree = ET.parse(path)
    found = {}
    for vtype in tree.getroot().iter('vType'):
        if vtype.get('id') in BACKGROUND_PROFILES:
            if vtype.get('carFollowModel') != 'IDM':
                raise ValueError(f'Expected IDM background vType {vtype.get("id")}, found {vtype.get("carFollowModel")}')
            vtype.set('carFollowModel', 'Krauss')
            found[vtype.get('id')] = True
    if set(found) != set(BACKGROUND_PROFILES):
        raise ValueError(f'Background vTypes missing from {path}: {sorted(set(BACKGROUND_PROFILES) - set(found))}')
    tree.write(path, encoding='UTF-8', xml_declaration=True)


def _strip(value):
    if isinstance(value, dict):
        return {k: _strip(v) for k, v in value.items() if k not in VOLATILE}
    if isinstance(value, list):
        return [_strip(v) for v in value]
    return value


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, default=str).encode()).hexdigest()


def _tensor_sha(batch):
    digest = hashlib.sha256()
    for name in sorted(batch):
        digest.update(name.encode())
        digest.update(batch[name].contiguous().numpy().tobytes())
    return digest.hexdigest()


def _differing(a, b):
    """The result hashes on which two records of the same key disagree."""
    return [name for name in ('report_sha', 'trace_sha', 'samples_sha') if a.get(name) != b.get(name)]


# ----------------------------------------------------------------------------------------- worker

_W = {}


def _row_set(cfg, name):
    if name == 'v14-validation':
        return cfg['validation_rows']
    if name == 'v14-training':
        return cfg['training_rows']
    if name == 'v15-development':
        from traffic import v15_benchmark
        return v15_benchmark.development_rows(cfg)
    if name == 'v15-training':
        from traffic import v15_training_rows
        return v15_training_rows.rows(cfg)
    raise ValueError(name)


def _worker_setup(backend, round_index, policy_dir):
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
    np.random.seed(cfg['seed'])
    random.seed(cfg['seed'])
    _W.update(runner=runner, base=runner.base, cfg=cfg, policies={}, policy_dir=Path(policy_dir),
              rows={name: _row_set(cfg, name) for name in ROW_SETS}, round_index=round_index)


def _driver(driver, task=None, key=None):
    """The actor for a driver id: a published policy, a rule driver reading the live environment, or a
    Level 2 driver (built per task: its mixing and perturbations are seeded by the task key)."""
    if is_level2(driver):
        from traffic import v15_level2
        net = None if driver == v15_level2.RULE_DRIVER else _level2_net(driver)
        collect = task is not None and task.mode == 'collect'
        return v15_level2.Level2Driver(lambda: _W.get('env'), net=net, collect=collect,
                                       perturb=task.perturb if task is not None else None,
                                       seed=torch_seed(key) if key else 0)
    if driver in RULE_DRIVERS:
        from traffic import v15_plan_follower, v15_rule_expert, v15_rule_expert_v1, v15_rule_expert_v2
        if driver == v15_plan_follower.DRIVER:
            return v15_plan_follower.PlanFollower(lambda: _W.get('env'))
        for module in (v15_rule_expert, v15_rule_expert_v1, v15_rule_expert_v2):
            if driver == module.DRIVER:
                return module.RuleExpert(lambda: _W.get('env'))
        raise ValueError(f'No implementation for {driver}')
    return _policy(driver)


def _policy(driver):
    import torch
    cached = _W['policies'].get(driver)
    if cached is None:
        if 'skeleton' not in _W:
            _W['skeleton'], _ = _W['runner'].load_warm_start(_W['cfg'])
        model = copy.deepcopy(_W['skeleton'])
        saved = torch.load(_W['policy_dir'] / f'{driver.split(":", 1)[1]}.pt', map_location='cpu', weights_only=False)
        model.load_state_dict(saved)
        model.eval()
        if state_hash(model.state_dict()) != driver.split(':', 1)[1]:
            raise ValueError(f'Published weights do not match {driver}')
        cached = _W['policies'][driver] = model
    return cached


def _level2_net(driver):
    import torch
    cached = _W['policies'].get(driver)
    if cached is None:
        from traffic import v15_level2
        net = v15_level2.build_net()
        saved = torch.load(_W['policy_dir'] / f'{driver.split(":", 1)[1]}.pt', map_location='cpu', weights_only=False)
        net.load_state_dict(saved)
        net.eval()
        if state_hash(net.state_dict()) != driver.split(':', 1)[1]:
            raise ValueError(f'Published weights do not match {driver}')
        cached = _W['policies'][driver] = net
    return cached


def _labels(driver, local_obs):
    """The labelling driver's deterministic action on each visited state ([steps, agents, 2])."""
    import torch
    model = _policy(driver)
    steps, agents = local_obs.shape[:2]
    with torch.no_grad():
        actions = model.act(local_obs.reshape(steps * agents, -1), deterministic=True)[0]
    return actions.reshape(steps, agents, -1)


def _compact(batch):
    """Drop the 24 identical copies of the centralized state per step (the learner re-expands them)."""
    import torch
    states = batch['global_states']
    if not torch.equal(states, states[:, :1].expand_as(states)):
        raise ValueError('Expected one centralized state per step, repeated for every agent')
    compact = {k: v for k, v in batch.items() if k != 'global_states'}
    compact['global_state_per_step'] = states[:, 0].clone()
    return compact


def expand(batch):
    """A compact sample batch back in the trainer's layout."""
    out = {k: v for k, v in batch.items() if k not in ('global_state_per_step', 'labels')}
    per_step = batch['global_state_per_step']
    out['global_states'] = per_step[:, None].expand(per_step.shape[0], batch['local_obs'].shape[1], per_step.shape[1])
    return out


def _execute(task, attempt, out):
    import numpy as np
    import torch
    from traffic import v15_events
    base, cfg = _W['base'], _W['cfg']
    row = _W['rows'][task.row_set][task.row]
    fields = key_fields(task, _W['round_index'], row)
    key = task_key(fields)
    torch.manual_seed(torch_seed(key))
    np.random.seed(torch_seed(key))
    random.seed(torch_seed(key))
    directory = Path(out) / 'episodes' / key[:16] / attempt
    if directory.exists():
        raise FileExistsError(f'{directory} already holds an attempt; the learner picks a free attempt name')
    original_env_for = base.env_for

    def env_for(*args, **kwargs):
        env = _W['env'] = original_env_for(*args, **kwargs)
        if task.background != 'idm':
            write = env._write_demand

            def write_with_background(path, seed):
                result = write(path, seed)
                set_background(path, task.background)
                return result
            env._write_demand = write_with_background
        return env

    started = time.perf_counter()
    base.env_for = env_for
    try:
        actor = _driver(task.driver, task, key)
        if is_level2(task.driver):          # deterministic to the episode loop; samples come from the actor
            with torch.no_grad():
                report, batch = base.episode(cfg, row, directory, actor)
        elif task.mode == 'collect':
            report, batch = base.episode(cfg, row, directory, actor, collect=True)
        else:
            with torch.no_grad():
                report, batch = base.episode(cfg, row, directory, actor)
    finally:
        base.env_for = original_env_for
        _W.pop('env', None)
    seconds = time.perf_counter() - started
    report = json.loads(json.dumps(report, default=str))
    (directory / 'report.json').write_text(json.dumps(report, sort_keys=True))
    traces = sorted(directory.glob('episode-*/actions.jsonl.gz'))
    if len(traces) != 1:
        raise ValueError(f'Expected one trace in {directory}')
    trace = v15_events.read_trace(traces[0])
    events = v15_events.episode_events(report, trace)
    gate = base.ruled_gates(report)
    record = dict(key=key, fields=fields, run=task.run, attempt=attempt, worker=_W.get('worker'), pid=os.getpid(),
                  seconds=seconds, decisions=int(report.get('agent_transitions') or 0),
                  simulated_seconds=float(report.get('simulation_time') or 0.),
                  report_sha=_sha(_strip(report)), trace_sha=_sha(_strip(trace)),
                  passed=bool(gate['passed']), failures=list(gate['failures']), events=events['events'],
                  intervention_fraction=report.get('intervention_fraction'), directory=str(directory))
    if is_level2(task.driver):
        record['level2'] = actor.stats
        level2 = actor.sample_batch() if task.mode == 'collect' else None
        if level2 is not None:
            record['samples_sha'] = _tensor_sha(level2)
            record['samples'] = str(directory / 'level2.pt')
            record['sample_bytes'] = _save(level2, directory / 'level2.pt')
            record['level2_samples'] = int(level2['x'].shape[0])
    elif batch is not None:
        compact = _compact(batch)
        if task.label_with:
            compact['labels'] = _labels(task.label_with, compact['local_obs'])
        record['samples_sha'] = _tensor_sha(compact)
        record['samples'] = str(directory / 'samples.pt')
        record['sample_bytes'] = _save(compact, directory / 'samples.pt')
    return record


def _save(batch, path):
    import torch
    buffer = io.BytesIO()
    torch.save(batch, buffer)
    data = buffer.getvalue()
    Path(path).write_bytes(data)
    return len(data)


def _worker_main(worker, inbox, outbox, backend, round_index, policy_dir, priority='normal'):
    try:
        read = set_priority(priority)
        _worker_setup(backend, round_index, policy_dir)
        _W['worker'] = worker
        outbox.put(('ready', worker, os.getpid(), read))
    except BaseException:
        outbox.put(('failed', worker, traceback.format_exc()))
        return
    while True:
        message = inbox.get()
        if message[0] == 'stop':
            return
        _, ticket, task_dict, attempt, out = message
        try:
            outbox.put(('done', worker, ticket, _execute(Task(**task_dict), attempt, out)))
        except BaseException:
            outbox.put(('error', worker, ticket, traceback.format_exc()))


# ----------------------------------------------------------------------------------------- learner side

@dataclass
class BatchResult:
    records: list
    canary: dict | None
    clock_seconds: float
    worker_seconds: float
    simulated_seconds: float
    decisions: int
    workers: int
    timings: dict = field(default_factory=dict)
    simulated_tasks: int = 0          # tasks simulated in this batch (the canary's re-run not counted)
    reused_tasks: int = 0             # tasks given an existing result with the same key
    repeats: dict | None = None       # reuse=False: re-runs of known keys, which must match
    canary_decisions: int = 0         # the canary's re-run (simulated driving, so charged)

    @property
    def charged_decisions(self):
        """Every decision this batch simulated, across workers: its tasks and the canary's re-run."""
        return self.decisions + self.canary_decisions


class WorkerPool:
    """N worker processes, each owning one SUMO simulation at a time, and the learner's side of the loop."""

    def __init__(self, workers, out, *, backend='libsumo', round_index=24, start_timeout_s=600, priority='normal'):
        self.out = Path(out).resolve()
        if not self.out.is_relative_to(OUTPUT_ROOT):
            raise ValueError('Pool outputs must stay inside project outputs')
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / 'policies').mkdir(exist_ok=True)
        self.backend, self.round_index, self.size = backend, round_index, int(workers)
        if priority not in PRIORITIES:
            raise ValueError(f'Unknown priority {priority!r}; expected one of {PRIORITIES}')
        self.priority, self.worker_priorities = priority, {}
        from traffic import highway_flow_rl_v14_contract as contract
        self.cfg = contract.build_config(round_index)
        self.rows = {name: _row_set(self.cfg, name) for name in ROW_SETS}
        context = mp.get_context('spawn')
        self.outbox = context.Queue()
        self.inboxes, self.processes = [], []
        started = time.perf_counter()
        for worker in range(self.size):
            inbox = context.Queue()
            process = context.Process(target=_worker_main, daemon=True,
                                      args=(worker, inbox, self.outbox, backend, round_index, str(self.out / 'policies'), priority))
            process.start()
            self.inboxes.append(inbox)
            self.processes.append(process)
        ready = 0
        while ready < self.size:
            message = self.outbox.get(timeout=start_timeout_s)
            if message[0] == 'failed':
                self.close()
                raise RuntimeError(f'Worker {message[1]} failed to start:\n{message[2]}')
            self.worker_priorities[message[1]] = message[3]
            if message[3] != priority:
                self.close()
                raise RuntimeError(f'Worker {message[1]} runs at {message[3]} priority, not {priority}')
            ready += 1
        self.start_seconds = time.perf_counter() - started
        self._ticket = 0
        # Set by a charged run (experiments/v15_ledger.py) to its ledger segment; recorded on every batch.
        self.charged_by = None
        # Results this pool directory already holds, by key (the first recorded attempt of each).
        self._known = {}
        if (self.out / 'results.jsonl').is_file():
            for record in read_results(self.out / 'results.jsonl'):
                record.setdefault('directory', str(self.out / 'episodes' / record['key'][:16] / record['attempt']))
                self._known.setdefault(record['key'], record)

    def key(self, task):
        return task_key(key_fields(task, self.round_index, self.rows[task.row_set][task.row]))

    def publish_level2(self, state_dict):
        """Make Level 2 network weights available to every worker; returns the driver id 'l2:<hash>'."""
        return 'l2:' + self.publish(state_dict).split(':', 1)[1]

    def publish(self, state_dict):
        """Make weights available to every worker; returns the driver id 'policy:<hash>'."""
        import torch
        digest = state_hash(state_dict)
        path = self.out / 'policies' / f'{digest}.pt'
        if not path.is_file():
            temporary = path.with_suffix('.tmp')
            torch.save({k: v.detach().cpu().clone() for k, v in state_dict.items()}, temporary)
            temporary.replace(path)
        return f'policy:{digest}'

    def run(self, tasks, *, canary=True, reuse=True, results_table=True, dataset=True):
        """Run tasks on idle workers, longest rows first. With `canary`, the shortest simulated task runs
        first and is then re-run on a different worker; the two must match exactly.

        Results are keyed, so a key is simulated at most once per batch, and its other tasks share the
        result. A key this pool already has a result for (an earlier batch, or this directory's
        results.jsonl) is reused with `reuse`. Without it, the task is re-run into a new attempt
        directory and must match the stored result exactly: a cross-check like the canary.
        Reused tasks cost nothing: the batch's time and decisions count simulated episodes only."""
        tasks = list(tasks)
        started = time.perf_counter()
        keys = [self.key(t) for t in tasks]
        known_before = dict(self._known)
        to_run, first_of = plan_batch(keys, known_before, reuse=reuse)
        horizon = {i: self.rows[tasks[i].row_set][tasks[i].row]['horizon_steps'] for i in to_run}
        order = sorted(to_run, key=lambda i: (-horizon[i], i))
        canary_task = min(to_run, key=lambda i: (horizon[i], i)) if canary and to_run else None
        if canary_task is not None:
            order.remove(canary_task)
            order.insert(0, canary_task)
        episodes = self.out / 'episodes'
        pending = [(i, free_attempt(episodes / keys[i][:16], 'repeat' if keys[i] in known_before else 'first'), False)
                   for i in order]
        idle = list(range(self.size))
        busy, records, canary_record, errors, canary_avoid = {}, [None] * len(tasks), None, [], None
        while pending or busy:
            dispatched = True
            while dispatched and pending and idle:
                dispatched = False
                for slot, (index, attempt, is_canary) in enumerate(pending):
                    allowed = [w for w in idle if not is_canary or w != canary_avoid or self.size == 1]
                    if allowed:
                        worker = allowed[0]
                        pending.pop(slot)
                        idle.remove(worker)
                        self._ticket += 1
                        busy[self._ticket] = (index, attempt, is_canary)
                        self.inboxes[worker].put(('task', self._ticket, asdict(tasks[index]), attempt, str(self.out)))
                        dispatched = True
                        break
            message = self.outbox.get()
            kind, worker, ticket, payload = message
            index, attempt, is_canary = busy.pop(ticket)
            idle.append(worker)
            if kind == 'error':
                errors.append(dict(task=asdict(tasks[index]), attempt=attempt, worker=worker, traceback=payload))
            elif is_canary:
                canary_record = payload
            else:
                records[index] = dict(payload, reused=False)
                if index == canary_task:
                    canary_avoid = worker
                    pending.insert(0, (index, free_attempt(episodes / keys[index][:16], 'canary'), True))
        clock = time.perf_counter() - started
        if errors:
            raise RuntimeError(f'{len(errors)} task(s) failed; first:\n{errors[0]["traceback"]}')
        simulated = [records[i] for i in to_run]
        for index, key in enumerate(keys):
            if records[index] is None:
                source = records[first_of[key]] if records[first_of[key]] is not None else known_before[key]
                records[index] = dict(source, run=tasks[index].run, reused=True)
        check = None
        if canary_record is not None:
            first = records[canary_task]
            same = _differing(first, canary_record)
            check = dict(key=first['key'], first_worker=first['worker'], canary_worker=canary_record['worker'],
                         different_worker=first['worker'] != canary_record['worker'], matched=not same,
                         mismatched=same, seconds=canary_record['seconds'])
        repeats = None
        repeated = [records[i] for i in to_run if keys[i] in known_before]
        if repeated:
            failed = {r['key']: _differing(known_before[r['key']], r) for r in repeated}
            repeats = dict(tasks=len(repeated), matched=not any(failed.values()),
                           mismatched={k[:16]: v for k, v in failed.items() if v})
        for record in simulated:
            self._known.setdefault(record['key'], record)
        result = BatchResult(records=records, canary=check, clock_seconds=clock,
                             worker_seconds=sum(r['seconds'] for r in simulated) + (canary_record or {}).get('seconds', 0.),
                             simulated_seconds=sum(r['simulated_seconds'] for r in simulated),
                             decisions=sum(r['decisions'] for r in simulated), workers=self.size,
                             simulated_tasks=len(simulated), reused_tasks=len(tasks) - len(simulated), repeats=repeats,
                             canary_decisions=int((canary_record or {}).get('decisions', 0)))
        self._record(result, simulated, known_before, results_table, dataset)
        if check is not None and not check['matched']:
            raise CanaryMismatch(f'Canary {check["key"][:16]} differs on worker {check["canary_worker"]}: '
                                 f'{check["mismatched"]}', result)
        if repeats is not None and not repeats['matched']:
            raise CanaryMismatch(f'Re-runs differ from their stored results: {repeats["mismatched"]}', result)
        return result

    def _record(self, result, simulated, known_before, results_table, dataset):
        if results_table:
            with open(self.out / 'results.jsonl', 'a', encoding='utf-8') as handle:
                for record in simulated:
                    handle.write(json.dumps({k: v for k, v in record.items() if k != 'directory'}, sort_keys=True) + '\n')
            with open(self.out / 'batches.jsonl', 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(dict(schema=SCHEMA, workers=result.workers, priority=self.priority,
                    tasks=len(result.records),
                    simulated_tasks=result.simulated_tasks, reused_tasks=result.reused_tasks,
                    clock_seconds=result.clock_seconds, worker_seconds=result.worker_seconds,
                    simulated_seconds=result.simulated_seconds, decisions=result.decisions,
                    canary_decisions=result.canary_decisions, canary=result.canary,
                    repeats=result.repeats, charged=self.charged_by is not None, charged_by=self.charged_by,
                    training=False)) + '\n')
        if dataset:
            labelled = [r for r in simulated if r['key'] not in known_before
                        and r['fields']['label_with'] and r.get('samples')]
            if labelled:
                (self.out / 'dataset').mkdir(exist_ok=True)
                with open(self.out / 'dataset' / 'index.jsonl', 'a', encoding='utf-8') as handle:
                    for record in labelled:
                        handle.write(json.dumps(dict(key=record['key'], fields=record['fields'], samples=record['samples'],
                                                     samples_sha=record['samples_sha'], decisions=record['decisions'])) + '\n')

    def close(self):
        for inbox, process in zip(self.inboxes, self.processes):
            if process.is_alive():
                inbox.put(('stop',))
        for process in self.processes:
            process.join(timeout=30)
            if process.is_alive():
                process.terminate()
        self.processes = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def load_samples(record):
    import torch
    return torch.load(record['samples'], map_location='cpu', weights_only=False)


def learner_round_trips(pool, model, rows, iterations, *, row_set='v14-training', update=None, run='learner'):
    """The synchronous loop: publish weights, collect `rows` in parallel, load the samples, then `update`.

    Phase 0 passes update=None: nothing is trained. Overhead per iteration is the iteration's clock
    time minus its slowest episode (with no more episodes than workers, that is the time the loop
    adds beyond the slowest simulation)."""
    out = []
    for iteration in range(iterations):
        started = time.perf_counter()
        driver = pool.publish(model.state_dict())
        published = time.perf_counter()
        batch = pool.run([Task(row_set, r, driver, mode='collect', sample_seed=iteration, run=run) for r in rows],
                         canary=False)
        collected = time.perf_counter()
        samples = [expand(load_samples(record)) for record in batch.records]
        loaded = time.perf_counter()
        if update is not None:
            update(model, samples)
        done = time.perf_counter()
        slowest = max(r['seconds'] for r in batch.records)
        out.append(dict(iteration=iteration, driver=driver, episodes=len(rows), decisions=batch.decisions,
                        clock_seconds=done - started, publish_seconds=published - started,
                        collect_seconds=collected - published, load_seconds=loaded - collected,
                        update_seconds=done - loaded, slowest_episode_seconds=slowest,
                        overhead_seconds=(done - started) - slowest - (done - loaded),
                        sample_bytes=sum(r.get('sample_bytes', 0) for r in batch.records)))
    return out


def interleave(*runs):
    """Merge several runs' task lists row by row: A0, B0, C0, A1, B1, C1, ..."""
    out, longest = [], max((len(r) for r in runs), default=0)
    for i in range(longest):
        out.extend(run[i] for run in runs if i < len(run))
    return out


def paired_tasks(row_set, rows, drivers, backgrounds=BACKGROUNDS, run='validation'):
    """Every driver on the same rows and seeds under every background model."""
    return [Task(row_set, r, d, background=b, run=run) for b in backgrounds for r in rows for d in drivers]


def paired_table(records, a, b):
    """Rows where drivers a and b both ran with the same row, seed and background: their pass/fail side by side."""
    def slot(record):
        f = record['fields']
        return (f['round'], f['row_set'], f['row'], f['seed'], f['background'], f['mode'], f['sample_seed'])
    by = {}
    for record in records:
        by.setdefault(slot(record), {})[record['fields']['driver']] = record
    out = []
    for key, drivers in sorted(by.items()):
        if a in drivers and b in drivers:
            out.append(dict(row=key[2], seed=key[3], background=key[4], a_passed=drivers[a]['passed'],
                            b_passed=drivers[b]['passed'], a_events=drivers[a]['events'], b_events=drivers[b]['events']))
    return out


def read_results(path):
    with open(path, encoding='utf-8') as handle:
        return [json.loads(line) for line in handle if line.strip()]
