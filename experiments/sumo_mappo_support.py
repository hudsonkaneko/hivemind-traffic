"""Configuration, isolated SUMO workers and evidence for the MAPPO study."""
from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'experiments/configs/sumo_mappo_v1.json'


def jsonable(value):
    if hasattr(value, 'tolist'):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(item) for item in value]
    return value


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(jsonable(value), indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def config_digest(config):
    encoded = json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def load_config(path=DEFAULT_CONFIG):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    validate_config(config)
    return config


def validate_config(config):
    """Bound this first study and reject unsupported feature claims."""
    if config['schema_version'] != 1:
        raise ValueError('Unsupported configuration schema')
    env, model, train = (config[key] for key in ('environment', 'model', 'training'))
    from environments.sumo_mappo_env import SumoMappoEnv
    SumoMappoEnv(config=env)  # Validate the exact environment contract without starting SUMO.
    fixed = {'observation_size': 27, 'state_size': 112, 'num_agents': 4,
             'normalization': 'fixed_environment_feature_scaling', 'communication': 'none',
             'action': 'tanh_gaussian_normalized_acceleration',
             'critic_input': 'joint_observations_alive_masks_and_agent_one_hot'}
    if any(model[key] != value for key, value in fixed.items()):
        raise ValueError('Unsupported actor/critic contract')
    if env['speed_limit_m_s'] != 60 * .44704 or env['dt'] != .1:
        raise ValueError('This study requires a 60 mph ceiling and 0.1 s steps')
    if env['acceleration_min_m_s2'] != -4.5 or env['acceleration_max_m_s2'] != 2.:
        raise ValueError('Unsupported acceleration interface')
    if not isinstance(env['horizon'], int) or not 1 <= env['horizon'] <= 600:
        raise ValueError('Episode horizon must be 1..600 steps')
    if train['algorithm'] != 'MAPPO' or train['device'] != 'cpu':
        raise ValueError('This runner implements CPU MAPPO only')
    bounds = {'num_workers': (1, 4), 'rollout_steps': (1, 256),
              'max_agent_transitions': (4, 1000000), 'max_wall_seconds': (1, 7200),
              'torch_threads': (1, 4), 'epochs': (1, 10), 'minibatch_size': (1, 4096)}
    for key, (low, high) in bounds.items():
        value = train[key]
        if type(value) is not int or not low <= value <= high:
            raise ValueError('Out-of-bounds training setting: ' + key)
    if not 0 < train['gamma'] <= 1 or not 0 <= train['gae_lambda'] <= 1:
        raise ValueError('Invalid return discount')
    for key in ('actor_lr', 'critic_lr', 'clip', 'value_clip', 'max_grad_norm', 'target_kl'):
        if not math.isfinite(train[key]) or train[key] <= 0:
            raise ValueError('Expected positive finite setting: ' + key)
    for key in ('entropy_coef', 'value_coef'):
        if not math.isfinite(train[key]) or train[key] < 0:
            raise ValueError('Expected nonnegative finite setting: ' + key)
    validation = config['evaluation']['validation_seeds']
    tests = config['evaluation']['test_seeds']
    preflight = config['preflight']['seeds']
    seeds = validation + tests + preflight
    if not validation or not tests or not preflight or len(seeds) != len(set(seeds)):
        raise ValueError('Evaluation/preflight seeds must be nonempty and disjoint')
    if any(type(seed) is not int or not 0 <= seed < 2**31 for seed in seeds):
        raise ValueError('Seeds must be nonnegative signed-32-bit integers')
    if any(type(train[key]) is not int or not 0 <= train[key] < 2**31
           for key in ('seed', 'episode_seed_start', 'episode_seed_end')):
        raise ValueError('Training seeds must be signed-32-bit nonnegative integers')
    if not 0 <= train['episode_seed_start'] <= train['episode_seed_end'] < 2**31:
        raise ValueError('Invalid training seed interval')
    if any(train['episode_seed_start'] <= seed <= train['episode_seed_end'] for seed in seeds):
        raise ValueError('Training seeds overlap evaluation/preflight')
    if not 1 <= config['evaluation']['every_updates'] <= 100:
        raise ValueError('Invalid validation interval')
    if config['evaluation']['selection'] != 'accepted_then_fewer_collisions_then_fewer_overspeed_steps_then_nominal_episodes_then_reward_then_fewer_interventions':
        raise ValueError('Unsupported checkpoint selection rule')
    execution = config['execution']
    if (execution['rendering'] is not False or execution['log_mode'] != 'files_only'
            or execution['requires_explicit_train_mode'] is not True):
        raise ValueError('Headless, quiet, explicit-launch execution is required')
    if not 1 <= execution['worker_timeout_seconds'] <= 60:
        raise ValueError('Worker timeout must be 1..60 seconds')
    if not 1 <= config['preflight']['rollout_steps'] <= 64 or not 1 <= config['preflight']['max_wall_seconds'] <= 300:
        raise ValueError('Preflight must remain bounded')
    # Reject NaN/infinity anywhere, including documentary settings.
    json.dumps(config, allow_nan=False)


def sources():
    names = ['experiments/run_sumo_mappo.py', 'experiments/sumo_mappo_support.py',
             'experiments/sumo_mappo_rollout.py', 'experiments/configs/sumo_mappo_v1.json',
             'policies/mappo.py', 'environments/sumo_mappo_env.py', 'traffic/mappo_sumo.py',
             'traffic/sumo_backend.py', 'requirements.txt']
    names += [str(path.relative_to(ROOT)).replace('\\', '/')
              for path in sorted((ROOT / 'scenarios/mappo_highway').glob('*')) if path.is_file()]
    names += [str(path.relative_to(ROOT)).replace('\\', '/')
              for path in sorted((ROOT / 'experiments/configs').glob('sumo_mappo_v[2-9]*.json'))]
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in names}


def create_run(config, mode):
    run_id = mode + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
    output = ROOT / 'outputs/sumo_mappo/v01' / run_id
    logs = ROOT / 'logs/sumo_mappo' / run_id
    output.mkdir(parents=True, exist_ok=False)
    logs.mkdir(parents=True, exist_ok=False)
    atomic_json(output / 'resolved-config.json', config)
    return output, logs


def manifest(config, mode):
    import gymnasium
    import numpy
    import pettingzoo
    import torch
    import traci
    from traffic.sumo_backend import find_sumo_binary
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=ROOT, text=True)
    return dict(schema_version=1, mode=mode, training=mode == 'train', optimizer_steps=0,
                started_utc=datetime.now(timezone.utc).isoformat(), config=config,
                config_sha256=config_digest(config), source_sha256=sources(),
                git_commit=git('rev-parse', 'HEAD').strip(), git_status=git('status', '--short'),
                git_diff_sha256=hashlib.sha256(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT)).hexdigest(),
                command=[sys.executable, *sys.argv], platform=platform.platform(),
                cpu=platform.processor(), python=platform.python_version(),
                versions=dict(torch=torch.__version__, numpy=numpy.__version__,
                              gymnasium=gymnasium.__version__, pettingzoo=pettingzoo.__version__,
                              traci=getattr(traci, '__version__', 'unknown'),
                              sumo=subprocess.check_output([str(find_sumo_binary()), '--version'], text=True).splitlines()[0]),
                rendering=False, communication='none')


def _worker_main(connection, config, run_directory, log_file):
    """Each spawned process owns exactly one TraCI connection/SUMO child."""
    if os.name != 'nt':
        os.setsid()  # Isolate this owned worker and its SUMO descendants for bounded cleanup.
    environment = None
    with open(log_file, 'x', encoding='utf-8', buffering=1) as log, redirect_stdout(log), redirect_stderr(log):
        try:
            from environments.sumo_mappo_env import SumoMappoEnv
            environment = SumoMappoEnv(config=config, run_directory=run_directory)
            while True:
                command, value = connection.recv()
                if command == 'close':
                    environment.close()
                    connection.send({'closed': True})
                    break
                if command == 'reset':
                    if isinstance(value, dict):
                        environment.control_mode = value.get('control_mode', 'policy')
                        seed = value['seed']
                    else:
                        environment.control_mode = 'policy'
                        seed = value
                    observations, infos = environment.reset(seed=seed)
                    response = dict(observations=observations, infos=infos)
                elif command == 'step':
                    observations, rewards, terminated, truncated, infos = environment.step(value)
                    response = dict(observations=observations, rewards=rewards,
                                    terminated=terminated, truncated=truncated, infos=infos)
                else:
                    raise ValueError('Unknown worker command')
                response.update(state=environment.state(), agents=list(environment.agents))
                connection.send(response)
        except EOFError:
            pass
        except BaseException:
            error = traceback.format_exc()
            print(error, file=log)
            try:
                connection.send({'error': error})
            except (BrokenPipeError, EOFError, OSError):
                pass
        finally:
            if environment is not None:
                environment.close()
            connection.close()


class SumoWorkerPool:
    """Explicit processes with bounded replies and cleanup of owned process trees."""
    def __init__(self, config, count, output, logs, timeout=30):
        self.timeout = timeout
        self.processes, self.connections = [], []
        self.closed = False
        self.forced_cleanup = []
        context = mp.get_context('spawn')
        try:
            for index in range(count):
                parent, child = context.Pipe()
                directory = Path(output) / ('worker-' + str(index))
                directory.mkdir(parents=True, exist_ok=False)
                process = context.Process(target=_worker_main,
                    args=(child, config, directory, Path(logs) / ('worker-' + str(index) + '.log')))
                process.start()
                child.close()
                self.processes.append(process)
                self.connections.append(parent)
        except BaseException:
            self.close()
            raise

    def request(self, commands):
        started = time.monotonic()
        for index, command in commands.items():
            self.connections[index].send(command)
        result = {}
        for index in commands:
            remaining = max(0, self.timeout - (time.monotonic() - started))
            if not self.connections[index].poll(remaining):
                raise TimeoutError('SUMO worker did not respond: ' + str(index))
            reply = self.connections[index].recv()
            if 'error' in reply:
                raise RuntimeError(reply['error'])
            result[index] = reply
        return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        for connection, process in zip(self.connections, self.processes):
            if process.is_alive():
                try:
                    connection.send(('close', None))
                except (BrokenPipeError, EOFError, OSError):
                    pass
        for connection, process in zip(self.connections, self.processes):
            process.join(timeout=5)
            if process.is_alive():
                self.forced_cleanup.append(process.pid)
                if os.name == 'nt':
                    subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
                else:
                    try:
                        if os.getpgid(process.pid) == process.pid:
                            os.killpg(process.pid, signal.SIGTERM)
                            process.join(timeout=2)
                            # SUMO may survive even if its worker has already exited.
                            os.killpg(process.pid, signal.SIGKILL)
                        else:
                            process.terminate()
                    except ProcessLookupError:
                        pass
                process.join(timeout=5)
            connection.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
