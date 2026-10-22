"""Native-only RSL PPO curriculum with an explicit demonstration warm start.

This module starts no process on import. All training is invoked by the bounded
run_loop_hazard_rsl supervisor after native scene and baseline checks.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import random
import time

ROOT = Path(__file__).resolve().parents[1]


def validate_training_config(cfg):
    if (cfg['schema_version'] != 1 or cfg['rsl_rl_version'] != '5.0.0'
            or cfg['observation_count'] != 42
            or cfg['action_mapping'] != 'normalized_steering_signed_longitudinal'
            or cfg['normalization'] != 'fixed_physical_scales_embedded_in_export'):
        raise ValueError('Unsupported hazard training contract')
    stages = cfg['curriculum']
    if [s['stage'] for s in stages] != ['clear', 'static', 'crossing']:
        raise ValueError('Curriculum order must be clear/static/crossing')
    count = sum(s['max_updates'] for s in stages)
    runner = cfg['runner']
    if count * runner['num_steps_per_env'] != cfg['maximum_ppo_transitions']:
        raise ValueError('Transition budget must exactly equal maximum curriculum rollouts')
    if (cfg.get('budget_prior_transitions', cfg.get('prior_ppo_transitions', 0)) + cfg['maximum_ppo_transitions'] > cfg.get('lifetime_ppo_limit', 1048576)
            or cfg.get('lifetime_ppo_limit', 1048576) > 1048576):
        raise ValueError('This protocol cannot exceed the original lifetime PPO budget')
    if cfg.get('resume_expected_updates', 0) * runner['num_steps_per_env'] != cfg.get('prior_ppo_transitions', 0):
        raise ValueError('Prior transition count does not match the resumed update boundary')
    interval = cfg['probe_every_updates']
    if interval <= 0 or any(s['max_updates'] <= 0 or s['max_updates'] % interval for s in stages):
        raise ValueError('Stage budgets must contain whole probe blocks')
    if runner['algorithm']['class_name'] != 'environments.lane_follow_rsl:TerminalObservationPPO':
        raise ValueError('Pre-reset timeout bootstrap required')
    if runner['logger'] != 'tensorboard' or not runner['check_for_nan']:
        raise ValueError('Local logging and finite rollout checks required')
    if any(runner[name]['class_name'] != 'environments.loop_hazard_rsl:FixedRoadMLP' for name in ('actor', 'critic')):
        raise ValueError('Shared fixed physical feature scales required')
    json.dumps(cfg, allow_nan=False)
    return cfg


def train_native(args, output, env):
    import numpy as np
    import torch
    from tensordict import TensorDict
    from rsl_rl.runners import OnPolicyRunner
    from environments.loop_hazard_rsl import wrap_hazard_rsl
    from environments.lane_follow_rsl import signed_vehicle_actions
    from experiments.run_loop_hazard_rsl import atomic_json, make_accumulator

    cfg = validate_training_config(json.loads(args.config.read_text(encoding='utf-8')))
    if cfg.get('environment', 'isolated') != args.environment:
        raise ValueError('Training configuration environment differs from launched native scene')
    atomic_json(output / 'training-config.json', cfg)
    if args.demonstrations is None:
        raise ValueError('Version1 requires qualified native demonstrations')
    if args.demonstration_audit is None:
        raise ValueError('Version1 requires independent demonstration qualification')
    audit = json.loads(args.demonstration_audit.read_text(encoding='utf-8'))
    if (not audit.get('passed') or not audit.get('training_warmstart_data_qualified')
            or audit['demonstrations']['path'] != args.demonstrations.relative_to(ROOT).as_posix()
            or audit['demonstrations']['sha256'] != hashlib.sha256(args.demonstrations.read_bytes()).hexdigest()):
        raise ValueError('Independent audit does not bind these exact demonstration bytes')
    for name, recorded in audit['artifacts'].items():
        if hashlib.sha256((args.demonstrations.parent / name).read_bytes()).hexdigest() != recorded['sha256']:
            raise ValueError('Audited collector artifact changed: ' + name)
    qualification = json.loads((args.demonstrations.parent / 'result.json').read_text(encoding='utf-8'))
    if qualification.get('environment', 'isolated') != args.environment:
        raise ValueError('Demonstrations must use the same native scene and observation context')
    collector_exit = json.loads((args.demonstrations.parent / 'exit.json').read_text(encoding='utf-8'))
    if (not qualification.get('completed') or not qualification.get('demonstrations_qualified')
            or collector_exit['returncode'] != 0 or collector_exit['timed_out']
            or not collector_exit['sources_unchanged'] or collector_exit['kit_after']['count'] != 0
            or not qualification['contact_cleanup']['passed']
            or not all(e['success'] and e['contacts'] == 0 for e in qualification['episodes'])):
        raise ValueError('Demonstrations were not qualified by complete successful native episodes')
    data = np.load(args.demonstrations, allow_pickle=False)
    observations = torch.tensor(data['observations'])
    actions = torch.tensor(data['actions'])
    if (observations.ndim != 2 or observations.shape[1] != 42 or actions.shape != (len(observations), 2)
            or not torch.isfinite(observations).all() or not torch.isfinite(actions).all()
            or bool((actions.abs() > 1).any())):
        raise ValueError('Invalid demonstration tensor contract')
    random.seed(cfg['seed']); np.random.seed(cfg['seed']); torch.manual_seed(cfg['seed'])
    deadline = time.monotonic() + args.max_seconds - 60
    checkpoints = output / 'checkpoints'
    checkpoints.mkdir()
    episodes = []
    raw_stream = (output / 'training-samples.jsonl').open('x', encoding='utf-8')
    episode_return = 0.0
    control_steps = 0
    partial_episodes = []
    optimizer_counts = {'behavior_cloning': 0, 'ppo': 0}
    accumulator = None

    def transition(record, raw_action, reset):
        nonlocal episode_return, control_steps, accumulator
        control_steps += 1
        episode_return += record['reward']
        accumulator.update(record)
        done = record['terminated'] or record['truncated']
        if control_steps % 6 == 0 or done or record['contacts']:
            raw_stream.write(json.dumps({'control_step': control_steps,
                                         'raw_action': raw_action.tolist(),
                                         'transition': record}, allow_nan=False) + '\n')
        if done:
            outcome = accumulator.finish(complete=True)
            if not outcome['evidence_valid']:
                raise RuntimeError('Completed training episode evidence failed validation')
            episodes.append({'control_step': control_steps, 'stage': record['hazard_stage'],
                             'episode_serial': record['episode_serial'], 'reason': record['reason'],
                             'success': record['success'], 'return': episode_return,
                             'independent_assessment': outcome,
                             'progress_m': record['route_state']['progress_m'],
                             'final_transition': record})
            episode_return = 0.0
            accumulator = make_accumulator(args, reset, route=env.route, stage=record['hazard_stage'])
            raw_stream.flush()
            atomic_json(output / 'training-episodes.json', episodes)

    env.cfg.hazard_stage = env.task.stage = 'clear'
    env.cfg.hazard_side = None
    wrapped = wrap_hazard_rsl(env, deadline=deadline, on_transition=transition)
    accumulator = make_accumulator(args, env.last_reset, route=env.route, stage='clear')
    log_dir = ROOT / 'logs/rsl_integration' / output.name / 'rsl'

    class HazardRunner(OnPolicyRunner):
        def save(self, path, infos=None):
            destination = checkpoints / Path(path).name
            temporary = destination.with_suffix('.tmp')
            state = self.alg.save()
            state.update(iter=self.current_learning_iteration, infos=infos)
            torch.save(state, temporary)
            os.replace(temporary, destination)

    runner = HazardRunner(wrapped, deepcopy(cfg['runner']), log_dir=str(log_dir), device='cpu')
    def count_ppo(*unused):
        optimizer_counts['ppo'] += 1
        atomic_json(output / 'optimizer-counts.json', optimizer_counts)
    runner.alg.optimizer.register_step_post_hook(count_ppo)
    actor = runner.alg.actor
    warm = cfg['behavior_cloning']
    resumed_updates = 0
    if args.resume is None:
        optimizer = torch.optim.Adam(actor.mlp.parameters(), lr=warm['learning_rate'])
        losses = []
        actor.train()
        for index in range(warm['gradient_steps']):
            if time.monotonic() > deadline:
                raise TimeoutError('Warm-start deadline reached')
            batch = torch.randint(len(observations), (warm['batch_size'],))
            predicted = actor(TensorDict({'policy': observations[batch]}, batch_size=[len(batch)]))
            errors = (predicted - actions[batch]).square()
            loss = (warm['steering_loss_weight'] * errors[:, 0] + errors[:, 1]).mean()
            if not torch.isfinite(loss):
                raise RuntimeError('Nonfinite behavior cloning loss before optimization')
            optimizer.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.mlp.parameters(), 1.0, error_if_nonfinite=True)
            optimizer.step()
            optimizer_counts['behavior_cloning'] += 1
            atomic_json(output / 'optimizer-counts.json', optimizer_counts)
            if any(not torch.isfinite(parameter).all() for parameter in actor.mlp.parameters()):
                raise RuntimeError('Nonfinite behavior cloning parameter')
            if index % 100 == 0 or index == warm['gradient_steps'] - 1:
                losses.append({'gradient_step': index + 1, 'loss': float(loss.detach())})
                print(f'WARM_START step={index + 1} loss={losses[-1]["loss"]:.7f}', flush=True)
        atomic_json(output / 'warm-start.json', {'gradient_steps': warm['gradient_steps'],
                    'demonstration_steps': len(observations), 'loss_samples': losses,
                    'demonstrations_sha256': hashlib.sha256(args.demonstrations.read_bytes()).hexdigest(),
                    'collector_manifest_sha256': hashlib.sha256((args.demonstrations.parent / 'manifest.json').read_bytes()).hexdigest(),
                    'collector_exit_sha256': hashlib.sha256((args.demonstrations.parent / 'exit.json').read_bytes()).hexdigest(),
                    'independent_audit_sha256': hashlib.sha256(args.demonstration_audit.read_bytes()).hexdigest(),
                    'role': 'scripted imitation initialization; subsequent updates use RSL-RL PPO'})
        runner.save('warm-start.pt', {'ppo_updates': 0})
        runner.get_inference_policy('cpu')
        runner.export_policy_to_jit(str(output), filename='warm-start-policy.pt')
    else:
        expected = cfg.get('resume_expected_updates')
        if type(expected) is not int or expected <= 0:
            raise ValueError('Resume requires an explicit prior update count')
        info = runner.load(str(args.resume), map_location='cpu')
        if info.get('ppo_updates') != expected or info.get('stage') != 'clear':
            raise ValueError('Resume checkpoint does not match declared curriculum boundary')
        resumed_updates = expected
        atomic_json(output / 'resume.json', {
            'checkpoint': args.resume.relative_to(ROOT).as_posix(),
            'sha256': hashlib.sha256(args.resume.read_bytes()).hexdigest(),
            'prior_completed_updates': resumed_updates,
            'prior_ppo_transitions': resumed_updates * cfg['runner']['num_steps_per_env'],
            'new_transition_cap': cfg['maximum_ppo_transitions'],
            'optimizer_state_restored': True, 'new_imitation_steps': 0})
        runner.get_inference_policy('cpu')
        runner.export_policy_to_jit(str(output), filename='resumed-initial-policy.pt')

    probes = []
    completed_updates = resumed_updates
    curriculum_complete = True

    def reset_training_accounting():
        nonlocal episode_return, accumulator
        if episode_return or env.episode_tick:
            partial_episodes.append({'control_steps': control_steps, 'return': episode_return,
                                     'native_episode_tick': env.episode_tick,
                                     'independent_assessment': (accumulator.finish(complete=False)
                                         if accumulator is not None and accumulator.steps else None),
                                     'reason': 'manual_curriculum_or_probe_boundary'})
            atomic_json(output / 'partial-training-exposures.json', partial_episodes)
        episode_return = 0.0
        accumulator = None
        runner.logger.cur_reward_sum.zero_()
        runner.logger.cur_episode_length.zero_()

    def probe(stage, label):
        actor.eval()
        cases = []
        with (output / f'probe-{label}.jsonl').open('x', encoding='utf-8') as stream:
            for index, seed in enumerate(cfg['development_seeds']):
                env.cfg.hazard_stage = env.task.stage = stage
                env.cfg.hazard_side = ('left' if index % 2 == 0 else 'right') if stage == 'crossing' else None
                obs, _ = env.reset(seed=seed)
                contacts = 0
                assessment = make_accumulator(args, env.last_reset, route=env.route, stage=stage)
                for step in range(2101):
                    with torch.no_grad():
                        raw = actor(TensorDict(obs, batch_size=[1]))
                    obs, _, terminated, truncated, extras = env.step(signed_vehicle_actions(raw))
                    tr = extras['lane_transition']
                    assessment.update(tr)
                    contacts += len(tr['contacts'])
                    stream.write(json.dumps({'seed': seed, 'transition': tr}, allow_nan=False) + '\n')
                    if bool(terminated[0] or truncated[0]):
                        outcome = assessment.finish(complete=True)
                        cases.append({'seed': seed, 'success': outcome['driving_success'], 'reason': tr['reason'],
                                      'progress_m': tr['route_state']['progress_m'], 'contacts': contacts,
                                      'independent_assessment': outcome})
                        break
                else:
                    raise RuntimeError('Probe did not terminate at the task horizon')
                if time.monotonic() > deadline:
                    raise TimeoutError('Curriculum evaluation deadline reached')
        success = sum(case['success'] for case in cases)
        passed = (success >= cfg['promotion']['minimum_successes']
                  and sum(case['contacts'] for case in cases) == 0
                  and all(case['reason'] != 'lane_departure' for case in cases)
                  and all(case['independent_assessment']['evidence_valid'] for case in cases))
        report = {'stage': stage, 'label': label, 'ppo_updates': completed_updates,
                  'successes': success, 'passed': passed, 'episodes': cases}
        probes.append(report)
        atomic_json(output / 'development-probes.json', probes)
        print(f'PROBE stage={stage} updates={completed_updates} successes={success}/10 passed={passed}', flush=True)
        return passed

    try:
        initial_passed = probe('clear', 'resumed-clear' if args.resume else 'warm-start-clear')
        if not initial_passed and args.resume is None:
            runner.export_policy_to_jit(str(output), filename='policy.pt')
            return {'curriculum_complete': False, 'reason': 'warm_start_development_gate_failed',
                    'ppo_updates': 0, 'ppo_control_steps': 0, 'optimizer_steps': 0,
                    'optimizer_counts': optimizer_counts, 'development_probes': probes,
                    'policy': str((output / 'policy.pt').relative_to(ROOT))}
        for stage_cfg in cfg['curriculum']:
            stage = stage_cfg['stage']
            if args.resume is not None and stage == 'clear' and initial_passed:
                continue  # Prior16 PPO updates now passed their postponed probe.
            promoted = False
            for stage_updates in range(0, stage_cfg['max_updates'], cfg['probe_every_updates']):
                env.cfg.hazard_stage = env.task.stage = stage
                env.cfg.hazard_side = None
                reset_training_accounting()
                env.reset(seed=cfg['seed'] + completed_updates)
                accumulator = make_accumulator(args, env.last_reset, route=env.route, stage=stage)
                runner.current_learning_iteration = completed_updates
                writer = getattr(runner.logger, 'writer', None)
                if writer is not None:
                    writer.flush(); writer.close()
                runner.learn(cfg['probe_every_updates'], init_at_random_ep_len=False)
                completed_updates += cfg['probe_every_updates']
                label = f'{stage}-{completed_updates:04d}'
                runner.save(f'{label}.pt', {'ppo_updates': completed_updates, 'stage': stage})
                runner.get_inference_policy('cpu')
                runner.export_policy_to_jit(str(output), filename=f'policy-{label}.pt')
                reset_training_accounting()
                if probe(stage, label):
                    promoted = True
                    break
            if not promoted:
                curriculum_complete = False
                break
        runner.get_inference_policy('cpu')
        runner.export_policy_to_jit(str(output), filename='policy.pt')
        runner.save('model-final.pt', {'ppo_updates': completed_updates,
                                       'curriculum_complete': curriculum_complete})
        return {'curriculum_complete': curriculum_complete, 'ppo_updates': completed_updates,
                'ppo_control_steps': wrapped.control_steps,
                'prior_ppo_updates': resumed_updates,
                'lifetime_ppo_transitions': (resumed_updates * cfg['runner']['num_steps_per_env'] + wrapped.control_steps),
                'experiment_budget_consumed': cfg.get('budget_prior_transitions', cfg.get('prior_ppo_transitions', 0)) + wrapped.control_steps,
                'maximum_ppo_transitions': cfg['maximum_ppo_transitions'],
                'behavior_cloning_gradient_steps': warm['gradient_steps'],
                'development_probes': probes,
                'policy_sha256': hashlib.sha256((output / 'policy.pt').read_bytes()).hexdigest(),
                'policy': str((output / 'policy.pt').relative_to(ROOT)),
                'optimizer_steps': optimizer_counts['ppo'], 'optimizer_counts': optimizer_counts}
    finally:
        atomic_json(output / 'optimizer-counts.json', optimizer_counts)
        raw_stream.close()
        writer = getattr(runner.logger, 'writer', None)
        if writer is not None:
            writer.flush(); writer.close()
