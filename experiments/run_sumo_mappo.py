"""Prepare, validate or explicitly train the headless SUMO MAPPO pilot.

Default invocation only prints the reviewed configuration. Preflight collects
real SUMO transitions but never constructs an optimizer. Training is a separate
explicit mode, pinned to the SHA-256 of the configuration approved by the user.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import gzip
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.sumo_mappo_support import (
    DEFAULT_CONFIG, SumoWorkerPool, atomic_json, config_digest, create_run,
    jsonable, load_config, manifest, sources,
)


def make_model(config):
    from policies.mappo import MappoActorCritic
    keys = ('observation_size', 'state_size', 'num_agents', 'actor_hidden_sizes', 'critic_hidden_sizes')
    return MappoActorCritic(**{key: config['model'][key] for key in keys})


def seed_runtime(config):
    import numpy as np
    import torch
    seed = config['training']['seed']
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(config['training']['torch_threads'])
    torch.use_deterministic_algorithms(True)


def model_hash(model):
    digest = hashlib.sha256()
    for name, tensor in sorted(model.state_dict().items()):
        digest.update(name.encode())
        digest.update(str((tuple(tensor.shape), tensor.dtype)).encode())
        digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    return digest.hexdigest()


def baseline_actions(kind, observations, rng, model=None):
    import numpy as np
    import torch
    from environments.sumo_mappo_env import physical_acceleration_to_action
    agents = sorted(observations)
    if kind in ('learned', 'untrained'):
        with torch.no_grad():
            actions = model.actor(torch.from_numpy(np.stack([observations[a] for a in agents]))).numpy()
        return {agent: actions[i] for i, agent in enumerate(agents)}
    actions = {}
    for agent in agents:
        if kind == 'random':
            actions[agent] = np.asarray([rng.uniform(-1., 1.)], dtype=np.float32)
            continue
        acceleration = 0.
        if kind == 'scripted':
            obs = observations[agent]
            speed = obs[0] * 26.8224
            target = 26.8224
            if obs[8] > .5:
                gap = obs[6] * 120.
                leader_speed = speed + obs[7] * 26.8224
                target = min(target, leader_speed + .5 * (gap - 2.5 - 1.5 * speed))
            acceleration = float(np.clip(.8 * (target - speed), -4.5, 2.))
        elif kind != 'native':
            raise ValueError('Unknown controller: ' + kind)
        actions[agent] = np.asarray([physical_acceleration_to_action(acceleration)], dtype=np.float32)
    return actions


def evaluate(config, kind, seeds, output, logs, tag, *, model=None, deadline=None, journal=None):
    import numpy as np
    from experiments.sumo_mappo_rollout import EpisodeMetrics, aggregate, journal_packet
    run = output / tag
    log = logs / tag
    run.mkdir(parents=True, exist_ok=False)
    log.mkdir(parents=True, exist_ok=False)
    episodes = []
    with SumoWorkerPool(config['environment'], 1, run, log, config['execution']['worker_timeout_seconds']) as pool:
        for seed in seeds:
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError('Evaluation wall-time budget reached')
            packet = pool.request({0: ('reset', dict(seed=seed, control_mode='native' if kind == 'native' else 'policy'))})[0]
            rng = np.random.default_rng(seed)
            metrics = EpisodeMetrics(seed)
            while packet['agents']:
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError('Evaluation wall-time budget reached')
                observations = {agent: packet['observations'][agent] for agent in packet['agents']}
                actions = baseline_actions(kind, observations, rng, model)
                packet = pool.request({0: ('step', actions)})[0]
                metrics.add(packet)
                journal_packet(journal, kind, seed, actions, packet)
            episodes.append(metrics.result())
        if pool.forced_cleanup:
            raise RuntimeError('Evaluation workers required forced cleanup')
    if pool.forced_cleanup:
        raise RuntimeError('Evaluation workers required forced cleanup')
    result = dict(controller=kind, episodes=episodes,
                  summary=aggregate(episodes, config['evaluation']['acceptance']))
    atomic_json(run / 'evaluation.json', result)
    return result


def save_checkpoint(path, model, trainer, config, transitions, update, next_seed, best_score, optimizer_steps=0):
    import numpy as np
    import torch
    numpy_state = np.random.get_state()
    payload = dict(schema_version=1, model_state=model.state_dict(), optimizer_state=trainer.optimizer.state_dict(),
                   config=config, config_sha256=config_digest(config), agent_transitions=transitions,
                   update=update, next_episode_seed=next_seed, best_score=list(best_score),
                   optimizer_steps_in_run=optimizer_steps,
                   torch_rng_state=torch.get_rng_state(), python_rng_state=random.getstate(),
                   numpy_rng_state=[numpy_state[0], numpy_state[1].tolist(), *numpy_state[2:]],
                   resume_boundary='fresh episodes; in-flight SUMO state is not checkpointed')
    temporary = Path(path).with_suffix('.tmp')
    torch.save(payload, temporary)
    os.replace(temporary, path)


def load_checkpoint(path, model, config, trainer=None):
    import numpy as np
    import torch
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT / 'outputs'):
        raise ValueError('Checkpoints must be explicitly selected inside project outputs')
    checkpoint = torch.load(path, map_location='cpu', weights_only=True)
    if checkpoint['schema_version'] != 1 or checkpoint['config_sha256'] != config_digest(config):
        raise ValueError('Checkpoint configuration differs from the approved experiment')
    model.load_state_dict(checkpoint['model_state'])
    if trainer is not None:
        trainer.optimizer.load_state_dict(checkpoint['optimizer_state'])
        torch.set_rng_state(checkpoint['torch_rng_state'])
        random.setstate(checkpoint['python_rng_state'])
        state = checkpoint['numpy_rng_state']
        np.random.set_state((state[0], np.asarray(state[1], dtype=np.uint32), *state[2:]))
    return checkpoint


def preflight(config, output, logs, deadline):
    import torch
    from experiments.sumo_mappo_rollout import RolloutCollector
    from policies.mappo import export_actor
    model = make_model(config)
    before = model_hash(model)
    result = dict(training=False, optimizer_steps=0, evaluations={})
    with gzip.open(output / 'preflight-actions.jsonl.gz', 'wt', encoding='utf-8') as journal:
        for kind in ('native', 'scripted', 'random', 'untrained'):
            result['evaluations'][kind] = evaluate(config, kind, config['preflight']['seeds'], output, logs,
                                                   kind, model=model, deadline=deadline, journal=journal)
        repeat = evaluate(config, 'scripted', config['preflight']['seeds'][:1], output, logs,
                          'scripted-repeat', deadline=deadline, journal=journal)
        result['scripted_repeat_equal'] = repeat['episodes'][0] == result['evaluations']['scripted']['episodes'][0]
        rollout_dir, rollout_logs = output / 'rollout', logs / 'rollout'
        rollout_dir.mkdir(); rollout_logs.mkdir()
        with SumoWorkerPool(config['environment'], config['training']['num_workers'], rollout_dir,
                            rollout_logs, config['execution']['worker_timeout_seconds']) as pool:
            collector = RolloutCollector(pool, model, config, journal=journal, deadline=deadline,
                                         seed_start=510000, seed_end=519999)
            batch, _, count = collector.collect(config['preflight']['rollout_steps'],
                                                 config['preflight']['rollout_steps'] * config['training']['num_workers'] * 4)
            if not count or not all(torch.isfinite(value).all() for value in batch.values()):
                raise RuntimeError('Invalid real-SUMO rollout tensors')
            mask = batch['active']
            with torch.no_grad():
                log_probs, _ = model.evaluate_actions(batch['local_obs'][mask], batch['actions'][mask],
                                                       batch['pre_tanh_actions'][mask])
            error = float((log_probs - batch['old_log_probs'][mask]).abs().max())
            if error > 1e-5:
                raise RuntimeError('Unchanged actor changed stored rollout likelihood')
            result['rollout'] = dict(agent_transitions=count, environment_steps=collector.total_environment_steps,
                                      likelihood_max_error=error, finite=True, optimizer_steps=0)
            torch.save(batch, output / 'untrained-rollout-interface-test.pt')
        result['forced_worker_cleanup'] = pool.forced_cleanup
    export_actor(model, output / 'untrained-actor-interface-test.pt')
    actor = torch.jit.load(str(output / 'untrained-actor-interface-test.pt'), map_location='cpu')
    with torch.no_grad():
        test_obs = batch['local_obs'][batch['active']][:8]
        result['actor_export_equal'] = bool(torch.allclose(actor(test_obs), model.actor(test_obs)))
    result['parameters_unchanged'] = before == model_hash(model)
    result['passed'] = bool(result['scripted_repeat_equal'] and result['actor_export_equal']
                            and result['parameters_unchanged'] and not result['forced_worker_cleanup'])
    return result


def train(config, output, logs, deadline, resume=None):
    import torch
    from experiments.sumo_mappo_rollout import RolloutCollector
    from policies.mappo import MappoTrainer, export_actor
    settings = config['training']
    model = make_model(config)
    keys = ('actor_lr', 'critic_lr', 'clip', 'value_clip', 'entropy_coef', 'value_coef',
            'max_grad_norm', 'epochs', 'minibatch_size', 'target_kl')
    trainer = MappoTrainer(model, **{key: settings[key] for key in keys})
    count = update = optimizer_steps = 0
    best_score = ()
    next_seed = settings['episode_seed_start']
    checkpoints = output / 'checkpoints'
    checkpoints.mkdir()
    if resume:
        restored = load_checkpoint(resume, model, config, trainer)
        count, update, next_seed = restored['agent_transitions'], restored['update'], restored['next_episode_seed']
        # A resumed run selects its own best snapshot on fresh validation; the parent remains immutable.
        if settings['max_agent_transitions'] - count < config['model']['num_agents']:
            raise ValueError('Checkpoint leaves insufficient budget for a fresh four-agent step')
    with gzip.open(output / 'training-actions.jsonl.gz', 'wt', encoding='utf-8') as journal, \
            (output / 'updates.jsonl').open('x', encoding='utf-8', buffering=1) as updates, \
            (output / 'episodes.jsonl').open('x', encoding='utf-8', buffering=1) as episodes:
        with SumoWorkerPool(config['environment'], settings['num_workers'], output, logs,
                            config['execution']['worker_timeout_seconds']) as pool:
            collector = RolloutCollector(pool, model, config, journal=journal, deadline=deadline, seed_start=next_seed)
            while count < settings['max_agent_transitions']:
                batch, completed, actual = collector.collect(settings['rollout_steps'], settings['max_agent_transitions'] - count)
                if not actual:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('Training wall-time budget reached before optimizer update')
                losses = trainer.update(batch)
                optimizer_steps += losses['optimizer_steps']
                count += actual
                update += 1
                # Persist learned work before validation can fail or reach its deadline.
                save_checkpoint(checkpoints / 'latest.pt', model, trainer, config, count, update,
                                collector.next_seed, best_score, optimizer_steps)
                atomic_json(output / 'status.json', dict(status='training', agent_transitions=count, update=update,
                    optimizer_steps=optimizer_steps, environment_agent_transitions=collector.total_agent_transitions,
                    rendering=False))
                for row in completed:
                    episodes.write(json.dumps(row, allow_nan=False) + '\n')
                row = dict(update=update, agent_transitions=count, **losses)
                final = settings['max_agent_transitions'] - count < 4
                if update % config['evaluation']['every_updates'] == 0 or final or not (checkpoints / 'best.pt').exists():
                    validation = evaluate(config, 'learned', config['evaluation']['validation_seeds'], output, logs,
                                          'validation-' + str(update), model=model, deadline=deadline)
                    score = tuple(validation['summary']['selection_score'])
                    row['validation'] = validation['summary']
                    if score > best_score:
                        best_score = score
                        save_checkpoint(checkpoints / 'best.pt', model, trainer, config, count, update,
                                        collector.next_seed, best_score, optimizer_steps)
                        export_actor(model, output / 'policy.pt')
                save_checkpoint(checkpoints / 'latest.pt', model, trainer, config, count, update,
                                collector.next_seed, best_score, optimizer_steps)
                updates.write(json.dumps(jsonable(row), allow_nan=False) + '\n')
                atomic_json(output / 'status.json', dict(status='training', agent_transitions=count, update=update,
                                                         optimizer_steps=optimizer_steps, rendering=False))
        if pool.forced_cleanup:
            raise RuntimeError('Training workers required forced cleanup')
    selected = make_model(config)
    load_checkpoint(checkpoints / 'best.pt', selected, config)
    held_out = {}
    with gzip.open(output / 'held-out-actions.jsonl.gz', 'wt', encoding='utf-8') as journal:
        for kind in ('learned', 'scripted', 'native'):
            held_out[kind] = evaluate(config, kind, config['evaluation']['test_seeds'], output, logs,
                                      'held-out-' + kind, model=selected, deadline=deadline, journal=journal)
    return dict(training=True, optimizer_steps=optimizer_steps, agent_transitions=count,
                unused_transition_budget=settings['max_agent_transitions'] - count,
                updates=update, held_out=held_out, best_validation_score=best_score,
                policy='policy.pt', checkpoint='checkpoints/best.pt', latest_checkpoint='checkpoints/latest.pt',
                passed=True, learned_acceptance=held_out['learned']['summary']['accepted'])


def execute(config, mode, resume=None):
    output, logs = create_run(config, mode)
    record = manifest(config, mode)
    atomic_json(output / 'manifest.json', record)
    for relative in record['source_sha256']:
        destination = output / 'source' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, destination)
    started = time.monotonic()
    seconds = config['preflight']['max_wall_seconds'] if mode == 'preflight' else config['training']['max_wall_seconds']
    result = dict(status='running', training=mode == 'train', optimizer_steps=0)
    atomic_json(output / 'status.json', result)
    print('RUN_DIRECTORY=' + str(output), flush=True)
    with (logs / 'runtime.log').open('x', encoding='utf-8', buffering=1) as log, redirect_stdout(log), redirect_stderr(log):
        try:
            seed_runtime(config)
            if mode == 'preflight':
                result = preflight(config, output, logs, started + seconds)
            else:
                result = train(config, output, logs, started + seconds, resume=resume)
            result['status'] = 'completed' if result['passed'] else 'failed'
        except BaseException as error:
            previous = json.loads((output / 'status.json').read_text())
            result.update(status='failed', error=repr(error), traceback=traceback.format_exc(),
                          last_saved_status=previous, optimizer_steps=previous.get('optimizer_steps', 0))
            traceback.print_exc()
        finally:
            result['wall_seconds'] = time.monotonic() - started
            result['source_unchanged'] = sources() == record['source_sha256']
            if not result['source_unchanged']:
                result.update(status='failed', error='Source changed while experiment was running')
            atomic_json(output / 'result.json', result)
            atomic_json(output / 'status.json', {key: value for key, value in result.items()
                if key not in ('evaluations', 'held_out', 'traceback')})
            hashes = {str(path.relative_to(output)).replace('\\', '/'): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in output.rglob('*') if path.is_file() and 'source' not in path.relative_to(output).parts}
            atomic_json(output / 'artifact-hashes.json', hashes)
    print(json.dumps(dict(output=str(output), status=result['status'], training=result['training'],
                          optimizer_steps=result.get('optimizer_steps', 0))), flush=True)
    return 0 if result['status'] == 'completed' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--mode', choices=('check', 'preflight', 'train'), default='check')
    parser.add_argument('--approved-config-sha256', help='Exact configuration digest displayed and approved before training')
    parser.add_argument('--resume', type=Path, help='Resume a matching local checkpoint in a fresh run package')
    args = parser.parse_args(argv)
    if Path.cwd().resolve() != ROOT:
        parser.error('Run from this checkout root')
    config = load_config(args.config)
    if args.mode == 'check':
        print(json.dumps(dict(config_sha256=config_digest(config), training_started=False, config=config), indent=2))
        return 0
    if args.resume is not None and args.mode != 'train':
        parser.error('--resume is only valid in explicit train mode')
    if args.mode == 'train' and args.approved_config_sha256 != config_digest(config):
        parser.error('Training requires --approved-config-sha256 matching the configuration shown for approval')
    return execute(config, args.mode, args.resume)


if __name__ == '__main__':
    raise SystemExit(main())
