"""Stage 1 preparation and explicitly authorized BC/MAPPO; defaults to config only."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import shutil
import subprocess
import time
import traceback
import uuid

import numpy as np
import torch

from experiments.run_i110_merge import rank, save_checkpoint, PilotBudgetExhausted
from experiments.sumo_mappo_support import atomic_json, config_digest
from policies.mappo import MappoActorCritic, MappoTrainer, compute_gae, export_actor

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'experiments/configs/highway_stage1_v2.json'
SCHEMA = 'highway_stage1_graph_messages_v2'
GEOMETRY_VERSION = 'highway_right_entry_v2'
AGENTS = tuple(f'agent_{i}' for i in range(4))
OBSERVATION_SIZE, STATE_SIZE = 93, 376
RANKING_VERSION = 4
SOURCES = ('experiments/run_highway_stage1.py', 'environments/highway_stage1_env.py',
           'environments/i110_merge_env.py', 'traffic/highway_merge_context.py',
           'traffic/highway_stage1_teacher.py', 'traffic/i110_policy_interface.py',
           'traffic/trip_accounting.py', 'policies/mappo.py', 'environments/sumo_mappo_env.py',
           'traffic/highway_selection.py',
           'traffic/sumo_backend.py', 'experiments/run_i110_merge.py', 'experiments/sumo_mappo_support.py',
           'scripts/build_highway_curriculum_v2.py', 'scripts/build_i110_sumo.py')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_config(path):
    from environments.highway_stage1_env import REWARD_VERSION
    cfg = json.loads(Path(path).read_text())
    version = cfg.get('experiment_version', 'v02')
    if version not in ('v02', 'v03'):
        raise ValueError('Unsupported Stage 1 experiment version')
    if version == 'v03' and cfg.get('ranking_version') != 5:
        raise ValueError('Stage 1 v03 requires ranking version 5')
    if cfg['observation_schema'] != SCHEMA or cfg['initial_checkpoint'] is not None:
        raise ValueError('Stage 1 requires a fresh explicitly versioned actor')
    expected = dict(observation_size=93, state_size=376, num_agents=4,
                    driving_local_size=45, driving_message_size=8, driving_mask_start=30)
    if cfg['model'] != expected or cfg['environment']['horizon'] != 800:
        raise ValueError('Unsupported Stage 1 actor/environment contract')
    if cfg['reward_version'] != REWARD_VERSION or cfg['preflight']['merge_density_coverage'] != ['light', 'dense']:
        raise ValueError('Reward version or preflight density contract mismatch')
    if cfg.get('geometry', {}).get('version') != GEOMETRY_VERSION:
        raise ValueError('Stage 1 requires explicit right-entry v2 geometry provenance')
    train, bc = cfg['training'], cfg['behavior_cloning']
    if not 4 <= train['max_agent_transitions'] <= 500000 or not 1 <= train['max_wall_seconds'] <= 7200:
        raise ValueError('Attempt exceeds proposed transition/wall budget')
    if not 4 <= bc['max_agent_transitions'] <= 50000 or not 1 <= bc['epochs'] <= (12 if version == 'v03' else 4):
        raise ValueError('Demonstration/BC allocation exceeds proposed bounds')
    weight = bc.get('acceleration_loss_weight', 1.)
    floor = train.get('imitation_floor', 0.)
    initial = cfg['optimizer']['imitation_coef']
    if not math.isfinite(weight) or not 1. <= weight <= (4. if version == 'v03' else 1.):
        raise ValueError('Invalid BC acceleration loss weight')
    if (not all(math.isfinite(x) for x in (floor, initial, train['imitation_decay_transitions']))
            or not 0 <= floor <= initial or train['imitation_decay_transitions'] <= 0
            or (version == 'v02' and floor != 0)):
        raise ValueError('Invalid imitation retention schedule')
    if bc['max_agent_transitions'] + sum(p['agent_transitions'] for p in cfg['phases']) != train['max_agent_transitions']:
        raise ValueError('Demo and phase allocations must equal the total budget')
    if any(p['agent_transitions'] < 4 or not p['task_cycle'] or
           set(p['task_cycle']) - {'cruise', 'following', 'passing', 'merge'} for p in cfg['phases']):
        raise ValueError('Invalid curriculum phase')
    if len({p['name'] for p in cfg['phases']}) != len(cfg['phases']):
        raise ValueError('Curriculum phase names must be unique')
    if train['device'] != 'cpu' or train['num_workers'] != 1:
        raise ValueError('This reviewed runner supports one CPU worker')
    intervals = [(train['seed_start'], train['seed_end']), (bc['seed_start'], bc['seed_end'])]
    if any(a > b for a, b in intervals) or max(x[0] for x in intervals) <= min(x[1] for x in intervals):
        raise ValueError('Training/demo seed ranges overlap or are empty')
    sets = [set(cfg['evaluation'][k]) for k in ('validation_seeds', 'test_seeds', 'preflight_seeds')]
    if any(a & b for i, a in enumerate(sets) for b in sets[i + 1:]):
        raise ValueError('Evaluation seed partitions overlap')
    if any(lo <= s <= hi for lo, hi in intervals for subset in sets for s in subset):
        raise ValueError('Training and evaluation seed partitions overlap')
    if not set(cfg['evaluation']['sentinel_seeds']) <= sets[0]:
        raise ValueError('Sentinel seeds must belong to validation')
    if len(cfg['evaluation']['validation_seeds']) != 12 or len(cfg['evaluation']['sentinel_seeds']) != 4:
        raise ValueError('Expected fixed twelve-case validation and four-case sentinel')
    if cfg['evaluation']['every_agent_transitions'] <= 0:
        raise ValueError('Invalid evaluation interval')
    if type(cfg['training_authorized']) is not bool:
        raise ValueError('Training authorization must be boolean')
    return cfg


def model_for(cfg):
    return MappoActorCritic(**cfg['model'], message_encoder=True, hybrid_actions=True)


def phase_index(cfg, spent):
    for i, phase in enumerate(cfg['phases']):
        if phase['agent_transitions'] - spent[i] >= 4:
            return i
    return None


def case_for(cfg, seed, split='train', phase=None, episode_index=0):
    bank = [c for c in cfg['curriculum'] if c['split'] == split]
    if not bank:
        raise ValueError('Empty scenario partition')
    if phase is not None:
        if split != 'train':
            raise ValueError('Training curriculum cannot select evaluation data')
        task = phase['task_cycle'][episode_index % len(phase['task_cycle'])]
        bank = [c for c in bank if c['task'] == task]
    return bank[seed % len(bank)]


def env_for(cfg, output, seed, split='train', phase=None, episode_index=0):
    from environments.highway_stage1_env import HighwayStage1Env
    case = case_for(cfg, seed, split, phase, episode_index)
    network = ROOT / case['network']
    if digest(network) != case['network_sha256']:
        raise ValueError('Frozen network hash mismatch')
    density = case['density']
    if phase is not None and case['task'] == 'merge':
        density = phase.get('merge_density', density)
    env = HighwayStage1Env(network, output, **cfg['environment'],
        reward_weights=cfg['reward_weights'], through_lanes=case['through_lanes'],
        task=case['task'], density=density)
    env.case_id = case['id']
    return env


def check_deadline(deadline):
    if deadline is not None and time.monotonic() >= deadline:
        raise PilotBudgetExhausted('Aggregate wall-time budget exhausted')


def primary_accounting(env, *, partial, deadline=None):
    """Account for the declared cohort through H without the old native drain."""
    control = env.report()
    primary = env.start_time + env.horizon * .1
    control_end = env.time
    if not partial:
        while env.time < primary - 1e-8:
            check_deadline(deadline)
            env._advance()
    result = env.report()
    result.update(control_end_time=control_end, primary_horizon_time=primary,
        primary_horizon_complete=env.time >= primary - 1e-8,
        accounting_comparison_valid=(not partial and env.time >= primary - 1e-8
                                     and result.get('accounting', {}).get('comparison_valid', True)),
        post_control_native_steps=max(0, round((env.time - control_end) / .1)),
        control_end_accounting=control.get('accounting'),
        post_control_events={key: result[key] - control[key] for key in ('collisions', 'teleports')})
    return result


def episode(env, seed, *, kind='teacher', model=None, collect=False,
            demonstrations=False, limit=None, deadline=None):
    from traffic.highway_stage1_teacher import teacher_actions
    if collect and (model is None or kind != 'learned'):
        raise ValueError('PPO collection requires a current learned policy')
    if collect and demonstrations:
        raise ValueError('Demonstrations are separate from on-policy PPO samples')
    rows, demo_obs, demo_actions = [], [], []
    reward_sum = 0.0
    transitions = 0
    rng = np.random.default_rng(seed + 70000000)
    try:
        check_deadline(deadline)
        obs, _ = env.reset(seed=seed)
        env.control_mode = 'native' if kind == 'native' else 'policy'
        while env.agents:
            check_deadline(deadline)
            if limit is not None and transitions + len(env.agents) > limit:
                break
            active = torch.tensor([a in env.agents for a in AGENTS])
            indices = torch.arange(4)
            local = torch.tensor(np.stack([obs.get(a, np.zeros(93, np.float32)) for a in AGENTS]))
            state = torch.as_tensor(env.state()).repeat(4, 1)
            labels = teacher_actions({a: obs[a] for a in env.agents})
            if kind == 'learned':
                with torch.no_grad():
                    actions, logp, _, latent = model.act(local, deterministic=not collect)
                    values = model.values(state, indices)
                chosen = {a: actions[i].numpy() for i, a in enumerate(AGENTS) if a in env.agents}
            elif kind in ('teacher', 'native'):
                if kind == 'native':
                    for a in env.agents:
                        env.conn.vehicle.setSpeed(a, -1)
                        env.conn.vehicle.setLaneChangeMode(a, 1621)
                chosen = labels
            elif kind == 'random':
                chosen = {a: np.array([rng.uniform(-1, 1), rng.choice(np.flatnonzero(obs[a][30:33] > .5)) - 1], np.float32)
                          for a in env.agents}
            else:
                raise ValueError('Unknown controller')
            if demonstrations:
                demo_obs.extend([obs[a].copy() for a in env.agents])
                demo_actions.extend([labels[a].copy() for a in env.agents])
            next_obs, rewards, term, trunc, info = env.step(chosen)
            if not all(math.isfinite(float(v)) for v in rewards.values()):
                raise ValueError('Nonfinite episode reward')
            transitions += len(rewards)
            reward_sum += sum(rewards.values())
            if collect:
                # Truncation bootstraps the actual final state, never masked reset state.
                final = np.concatenate([np.r_[next_obs.get(a, np.zeros(93)), float(not term.get(a, True))]
                                        for a in AGENTS]).astype(np.float32)
                with torch.no_grad():
                    next_values = model.values(torch.tensor(final).repeat(4, 1), indices)
                rows.append(dict(local_obs=local, global_states=state, agent_indices=indices,
                    actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
                    next_values=next_values, rewards=torch.tensor([rewards.get(a, 0.) for a in AGENTS]),
                    terminated=torch.tensor([term.get(a, False) for a in AGENTS]),
                    truncated=torch.tensor([trunc.get(a, False) for a in AGENTS]), active=active,
                    teacher_actions=torch.tensor(np.stack([labels.get(a, np.zeros(2, np.float32)) for a in AGENTS]))))
            obs = {a: next_obs[a] for a in env.agents}
        # Finish only the fixed primary accounting horizon with a deadline;
        # partial collector windows retain an explicitly incomplete ledger.
        report = primary_accounting(env, partial=bool(env.agents), deadline=deadline)
        report.update(seed=seed, controller=kind, case_id=env.case_id, reward=reward_sum,
                      agent_transitions=transitions, episode_directory=str(env.directory.relative_to(ROOT)),
                      native_drain_run=False, partial_control_horizon=bool(env.agents))
        batch = None
        if rows:
            batch = {k: torch.stack([r[k] for r in rows]) for k in rows[0]}
            batch['advantages'], batch['returns'] = compute_gae(batch['rewards'], batch['old_values'],
                batch['next_values'], batch['terminated'], batch['truncated'], batch['active'])
        demos = None if not demonstrations else dict(observations=torch.tensor(np.asarray(demo_obs), dtype=torch.float32).reshape(-1, 93),
                                                    actions=torch.tensor(np.asarray(demo_actions), dtype=torch.float32).reshape(-1, 2))
        return report, batch, demos
    except Exception:
        if hasattr(env, 'ledger'):
            interrupted = env.report()
            interrupted.update(primary_horizon_complete=False, accounting_comparison_valid=False,
                               incomplete_episode=True, actual_agent_transitions=transitions)
            atomic_json(env.directory / 'interrupted-accounting.json', interrupted)
        raise
    finally:
        env.close()


def quality(reports, cfg):
    failures = []
    target = cfg['evaluation']['provisional_targets']
    for r in reports:
        causes = []
        if any(r[k] != 0 for k in ('collisions', 'teleports', 'overspeed_steps')): causes.append('safety_event')
        if r['intervention_fraction'] > target['maximum_intervention_fraction']: causes.append('overrides')
        if r['task'] == 'cruise' and r['mean_fraction_of_speed_limit'] < target['cruise_mean_speed_limit_fraction_min']: causes.append('cruise')
        if r['task'] == 'passing' and r['passes'] < target['passes_min_per_passing_episode']: causes.append('passing')
        if r['task'] == 'merge' and r['successful_merges'] / 2 < target['merge_success_fraction_min']: causes.append('merging')
        if causes: failures.append(dict(case_id=r['case_id'], seed=r['seed'], causes=causes))
    return dict(passed=bool(reports) and not failures, failures=failures, cases=len(reports))


def selection_rank(reports, cfg):
    """Prefer full nominal acceptance; retain competence before low overrides."""
    if cfg.get('experiment_version') == 'v03':
        from traffic.highway_selection import selection_rank as intervention_rank
        return intervention_rank(reports, cfg)
    previous = rank(reports)
    return (previous[0], int(quality(reports, cfg)['passed']), *previous[1:])


def ranking_version(cfg):
    return 5 if cfg.get('experiment_version') == 'v03' else RANKING_VERSION


def imitation_weight(cfg, ppo_transitions):
    """Keep the original zero-floor schedule for v02 and explicit retention for v03."""
    floor = cfg['training'].get('imitation_floor', 0.)
    initial = cfg['optimizer']['imitation_coef']
    fraction = max(0., 1. - ppo_transitions / cfg['training']['imitation_decay_transitions'])
    return floor + (initial - floor) * fraction


def bc_objective(losses, cfg):
    return cfg['behavior_cloning'].get('acceleration_loss_weight', 1.) * losses[1] + losses[2]


def evaluate(cfg, run, model, seeds, *, split='validation', deadline=None, prefix='validation'):
    reports = []
    for seed in seeds:
        report, _, _ = episode(env_for(cfg, run / prefix, seed, split), seed,
                              model=model, kind='learned', deadline=deadline)
        reports.append(report)
        atomic_json(run / f'{prefix}-{seed}.json', report)
    atomic_json(run / f'{prefix}.json', reports)
    return reports


def source_hashes():
    return {p: digest(ROOT / p) for p in SOURCES}


def geometry_check(cfg):
    """Verify bank identity and topology without executing held-out traffic."""
    from scripts.build_highway_curriculum_v2 import GEOMETRY_VERSION as BUILDER_VERSION, validate_geometry
    geometry = cfg['geometry']
    if geometry['version'] != GEOMETRY_VERSION or BUILDER_VERSION != GEOMETRY_VERSION:
        raise ValueError('Geometry version mismatch')
    manifest = (ROOT / geometry['bank_manifest']).resolve()
    if not manifest.is_relative_to(ROOT / 'outputs') or digest(manifest) != geometry['bank_manifest_sha256']:
        raise ValueError('Geometry bank manifest path/hash mismatch')
    bank = json.loads(manifest.read_text())
    if bank['geometry_version'] != GEOMETRY_VERSION or len(bank['layouts']) != 10:
        raise ValueError('Expected the complete versioned ten-layout geometry bank')
    by_network = {row['network']: row for row in bank['layouts']}
    if len(by_network) != 10: raise ValueError('Geometry bank repeats a network')
    preserved = geometry['preserved_i110']
    reports = []
    for network, row in by_network.items():
        path = (ROOT / network).resolve()
        if not path.is_relative_to(ROOT / 'outputs') or digest(path) != row['network_sha256']:
            raise ValueError('Geometry bank network path/hash mismatch')
        checked = validate_geometry(path, through_lanes=row['through_lanes'])
        if not checked['passed']:
            raise ValueError(f'Geometry structural checks failed for {row["id"]}: {checked}')
        reports.append(dict(id=row['id'], split=row['split'], network=network, checks=checked))
    referenced = set()
    for case in cfg['curriculum']:
        network = case['network']
        if network in by_network:
            row = by_network[network]
            if any(case[k] != row[k] for k in ('network_sha256', 'through_lanes', 'split', 'speed_limit_mph')):
                raise ValueError('Curriculum geometry does not match its bank record')
            referenced.add(network)
        elif not (case['id'].startswith('i110-') and case['split'] == 'train'
                  and network == preserved['network'] and case['network_sha256'] == preserved['network_sha256']):
            raise ValueError('Curriculum network is neither v2 geometry nor the preserved I-110 source')
    if referenced != set(by_network): raise ValueError('Curriculum omits generated bank geometry')
    if digest(ROOT / preserved['network']) != preserved['network_sha256']:
        raise ValueError('Preserved I-110 geometry checksum mismatch')
    return dict(passed=True, version=GEOMETRY_VERSION, manifest_sha256=geometry['bank_manifest_sha256'],
        layouts=reports, preserved_i110=preserved,
        held_out_scope='static network topology inspection only; no held-out traffic or policy evaluation')


def preparation_digest(cfg):
    payload = {k: cfg[k] for k in ('observation_schema', 'model', 'environment',
        'reward_version', 'reward_weights', 'geometry', 'curriculum', 'phases', 'preflight')} | {
            'targets': cfg['evaluation']['provisional_targets'],
            'preflight_seeds': cfg['evaluation']['preflight_seeds']}
    if cfg.get('experiment_version') == 'v03':
        payload['revision'] = dict(experiment_version='v03', ranking_version=ranking_version(cfg),
            behavior_cloning=cfg['behavior_cloning'], imitation_floor=cfg['training']['imitation_floor'],
            imitation_decay_transitions=cfg['training']['imitation_decay_transitions'],
            imitation_coef=cfg['optimizer']['imitation_coef'])
    return config_digest(payload)


def readiness_check(cfg):
    ready = cfg['readiness']
    if not ready.get('preflight_passed') or not ready.get('teacher_ready'):
        raise ValueError('Training requires verified preflight and feasible teacher evidence')
    path = ROOT / ready['summary_path']
    if not path.resolve().is_relative_to(ROOT / 'outputs') or digest(path) != ready['summary_sha256']:
        raise ValueError('Readiness evidence path/hash mismatch')
    evidence = json.loads(path.read_text())
    if not evidence.get('passed') or not evidence.get('teacher_ready') or evidence.get('optimizer_steps') != 0:
        raise ValueError('Readiness evidence does not verify zero-update preparation')
    checks = ('api_passed', 'seed_test_passed', 'teacher_repeat_exact', 'collector_finite', 'actor_export_passed')
    if not all(evidence.get(key) is True for key in checks) or evidence.get('bc_optimizer_steps') != 0:
        raise ValueError('Readiness requires the complete no-learning preflight, not teacher-only preparation')
    if not evidence.get('geometry', {}).get('passed') or evidence['geometry'].get('version') != GEOMETRY_VERSION:
        raise ValueError('Readiness requires verified right-entry v2 structural geometry checks')
    if evidence.get('source_hashes') != source_hashes():
        raise ValueError('Readiness evidence predates current runtime source')
    if evidence.get('preparation_sha256') != preparation_digest(cfg):
        raise ValueError('Readiness evidence predates current environment/configuration')


def new_state(cfg):
    return dict(demo_spent=0, ppo_spent=0, demo_actual=0, ppo_actual=0,
        phase_spent=[0] * len(cfg['phases']), phase_episodes=[0] * len(cfg['phases']),
        next_seed=cfg['training']['seed_start'], next_demo_seed=cfg['behavior_cloning']['seed_start'],
        updates=0, optimizer_steps=0, bc_optimizer_steps=0, bc_complete=False,
        after_bc_evaluated=False,
        elapsed_seconds=0., best_rank=None, next_sentinel=cfg['evaluation']['every_agent_transitions'],
        evaluated_boundaries=[], pending=None, bc_epoch=0, bc_cursor=0,
        bc_order=None, bc_optimizer=None, run_closed=False, teacher_admission_failed=False,
        selected_generation=None)


def reserve_episode(state, cfg, phase=None):
    """Reserve before launching SUMO; failed/incomplete episodes remain charged."""
    if phase is None:
        remaining = cfg['behavior_cloning']['max_agent_transitions'] - state['demo_spent']
        key, seed_key, end = 'demo_spent', 'next_demo_seed', cfg['behavior_cloning']['seed_end']
    else:
        remaining = cfg['phases'][phase]['agent_transitions'] - state['phase_spent'][phase]
        key, seed_key, end = 'ppo_spent', 'next_seed', cfg['training']['seed_end']
    if remaining < 4 or state[seed_key] > end:
        return None
    reserved = min(remaining, cfg['environment']['horizon'] * 4)
    seed = state[seed_key]
    state[seed_key] += 1
    state[key] += reserved
    if phase is not None:
        state['phase_spent'][phase] += reserved
        state['phase_episodes'][phase] += 1
    state['pending'] = dict(kind='episode', phase=phase, seed=seed, reserved=reserved)
    return seed, reserved


def settle_episode(state, actual):
    pending = state['pending']
    if pending is None or pending['kind'] != 'episode' or not 0 <= actual <= pending['reserved']:
        raise ValueError('Invalid episode budget settlement')
    unused = pending['reserved'] - actual
    if pending['phase'] is None:
        state['demo_spent'] -= unused
        state['demo_actual'] += actual
    else:
        state['ppo_spent'] -= unused
        state['ppo_actual'] += actual
        state['phase_spent'][pending['phase']] -= unused
    state['pending'] = None


def contract(cfg):
    from environments.highway_stage1_env import LOCAL_FEATURES, MESSAGE_FEATURES
    return dict(schema=SCHEMA, reward_version=cfg['reward_version'], geometry=cfg['geometry'],
        observation_size=93, state_size=376, action_size=2,
        model=cfg['model'], local_features=LOCAL_FEATURES, message_features=MESSAGE_FEATURES,
        mask_slice=[30, 33], message_slice=[45, 93], normalization='fixed analytic scaling; no running statistics',
        acceleration_m_s2=[-4.5, 2.], lane_delta=[-1, 0, 1], legacy_checkpoint_compatible=False,
        information_source='current exact SUMO state/map plus delivered delayed messages; not camera/LiDAR')


def selected_contract(run, state, fingerprint):
    generation = state.get('selected_generation')
    if generation is None:
        if state['best_rank'] is not None: raise ValueError('Selected score has no artifact generation')
        return None
    if Path(generation).name != generation or not generation.startswith('selected-'):
        raise ValueError('Invalid selected artifact generation')
    folder = run / generation
    sidecar = json.loads((folder / 'policy-contract.json').read_text())
    if sidecar['schema'] != SCHEMA or sidecar['config_sha256'] != fingerprint:
        raise ValueError('Selected actor contract mismatch')
    for name, key in (('policy.pt', 'policy_sha256'), ('best.pt', 'checkpoint_sha256'), ('validation.json', 'validation_sha256')):
        if digest(folder / name) != sidecar[key]: raise ValueError('Selected artifact checksum mismatch')
    if tuple(sidecar['validation_rank']) != tuple(state['best_rank']):
        raise ValueError('Selected validation rank mismatch')
    return sidecar


def train(cfg, run, fingerprint, hashes, started, resume=None):
    model = model_for(cfg)
    trainer = MappoTrainer(model, **cfg['optimizer'])
    state = new_state(cfg)
    demo_data = dict(observations=torch.empty((0, 93)), actions=torch.empty((0, 2)))
    demo_file = None
    prior = 0.
    if resume is not None:
        if resume.name != 'last.pt': raise ValueError('Resume requires last.pt, not a selected evaluation checkpoint')
        saved = torch.load(resume, map_location='cpu', weights_only=False)
        if saved.get('schema') != SCHEMA or saved.get('config_sha256') != fingerprint or saved.get('source_hashes') != hashes:
            raise ValueError('Resume schema/config/source mismatch')
        state = saved['state']
        if state.get('teacher_admission_failed'):
            raise ValueError('Prior teacher admission failed; diagnose and prepare a new version before learning')
        prior = float(state['elapsed_seconds'])
        if not state.get('run_closed'):
            # An unclean kill has no final stopwatch record: charge the entire
            # unobserved interval, including any downtime, rather than reset it.
            prior += max(0., time.time() - saved['saved_at_unix'])
        model.load_state_dict(saved['model'])
        trainer.optimizer.load_state_dict(saved['optimizer'])
        torch.set_rng_state(saved['torch_rng']); np.random.set_state(saved['numpy_rng']); random.setstate(saved['python_rng'])
        if saved.get('demonstrations_sha256'):
            demo_file = saved['demonstrations_file']
            if Path(demo_file).name != demo_file: raise ValueError('Invalid demonstration manifest path')
            source = resume.parent / demo_file
            if digest(source) != saved['demonstrations_sha256']: raise ValueError('Resume demonstration checksum mismatch')
            shutil.copyfile(source, run / demo_file)
            demo_data = torch.load(run / demo_file, map_location='cpu', weights_only=False)
        if state['best_rank'] is not None:
            selected_contract(resume.parent, state, fingerprint)
            generation = state['selected_generation']
            shutil.copytree(resume.parent / generation, run / generation)
            atomic_json(run / 'selection.json', dict(generation=generation))
        # Episode reservations are intentionally not refunded on a failed run.
        if state['pending'] and state['pending']['kind'] == 'episode': state['pending'] = None
    deadline = started + max(0., cfg['training']['max_wall_seconds'] - prior)
    state['run_closed'] = False
    result = dict(passed=False, held_out_test_run=False)

    def checkpoint(name='last.pt'):
        state['elapsed_seconds'] = prior + time.monotonic() - started
        payload = dict(schema=SCHEMA, config_sha256=fingerprint, config=cfg, source_hashes=hashes,
            model=model.state_dict(), optimizer=trainer.optimizer.state_dict(), state=deepcopy(state),
            selection_status='validation_selected' if Path(name).name == 'best.pt' else 'resume_only',
            ranking_version=ranking_version(cfg), saved_at_unix=time.time(), torch_rng=torch.get_rng_state(),
            numpy_rng=np.random.get_state(), python_rng=random.getstate(),
            demonstrations_file=demo_file,
            demonstrations_sha256=digest(run / demo_file) if demo_file is not None else None)
        save_checkpoint(run / name, payload)

    def validation(full, label):
        seeds = cfg['evaluation']['validation_seeds' if full else 'sentinel_seeds']
        reports = evaluate(cfg, run, model, seeds, deadline=deadline, prefix=label)
        if full:
            score = selection_rank(reports, cfg)
            if state['best_rank'] is None or score > tuple(state['best_rank']):
                state['best_rank'] = score
                generation = f'selected-{state["updates"]:05d}-{uuid.uuid4().hex[:8]}'
                folder = run / generation; folder.mkdir()
                state['selected_generation'] = generation
                checkpoint(generation + '/best.pt'); export_actor(model, folder / 'policy.pt')
                atomic_json(folder / 'validation.json', reports)
                atomic_json(folder / 'policy-contract.json', dict(**contract(cfg), config_sha256=fingerprint,
                    validation_rank=score, ranking_version=ranking_version(cfg),
                    quality_accepted=quality(reports, cfg)['passed'], validation_report=label + '.json',
                    policy_sha256=digest(folder / 'policy.pt'), checkpoint_sha256=digest(folder / 'best.pt'),
                    validation_sha256=digest(folder / 'validation.json')))
                # Publish a complete immutable generation; old selected files
                # are never overwritten while three artifacts are being written.
                atomic_json(run / 'selection.json', dict(generation=generation))
        checkpoint()

    def update_pending():
        pending = state['pending']
        if not pending or pending['kind'] != 'ppo_update': return
        trainer.imitation_coef = imitation_weight(cfg, state['ppo_actual'])
        stats = trainer.update(pending['batch'], deadline=deadline)
        state['optimizer_steps'] += stats['optimizer_steps']; state['updates'] += 1
        atomic_json(run / f'train-{state["updates"]:05d}.json', dict(episode=pending['report'], optimization=stats,
            phase=cfg['phases'][pending['phase']]['name'], imitation_coef=trainer.imitation_coef))
        state['pending'] = None
        checkpoint()

    checkpoint()
    try:
        # Collect only training scenarios; demonstrations never become PPO samples.
        while state['demo_spent'] < cfg['behavior_cloning']['max_agent_transitions']:
            check_deadline(deadline)
            reserved = reserve_episode(state, cfg)
            if reserved is None: break
            seed, limit = reserved
            checkpoint()
            report, _, data = episode(env_for(cfg, run / 'demonstrations', seed), seed,
                                     demonstrations=True, limit=limit, deadline=deadline)
            settle_episode(state, report['agent_transitions'])
            # Preserve and charge rejected teacher rollouts; do not silently fit
            # labels from an unsafe or shield-dependent episode on new seeds.
            admission = quality([report], cfg)
            if report['partial_control_horizon']:
                admission['failures'] = [dict(item, causes=[c for c in item['causes'] if c in ('safety_event', 'overrides')])
                                         for item in admission['failures']]
                admission['failures'] = [item for item in admission['failures'] if item['causes']]
                admission['passed'] = not admission['failures']
            atomic_json(run / f'demonstration-{seed}.json', dict(report, demonstration_admission=admission))
            state['teacher_admission_failed'] = not admission['passed']
            checkpoint()
            if not admission['passed']: raise ValueError('Fresh teacher episode failed demonstration admission; retain evidence and revise teacher')
            demo_data = {k: torch.cat((demo_data[k], data[k])) for k in demo_data}
            # Immutable versions keep the previous checkpoint recoverable if
            # the process dies between writing the dataset and its checkpoint.
            demo_file = f'demonstrations-{seed}.pt'
            save_checkpoint(run / demo_file, demo_data)
            checkpoint()
        if not len(demo_data['observations']): raise ValueError('No training demonstrations collected')
        bc = cfg['behavior_cloning']
        bc_optimizer = torch.optim.Adam(model.actor.parameters(), lr=bc['learning_rate'], eps=1e-5)
        if state['bc_optimizer'] is not None: bc_optimizer.load_state_dict(state['bc_optimizer'])
        while not state['bc_complete'] and state['bc_epoch'] < bc['epochs']:
            if state['bc_order'] is None:
                generator = torch.Generator().manual_seed(bc['seed'] + state['bc_epoch'])
                state['bc_order'] = torch.randperm(len(demo_data['observations']), generator=generator)
            order = state['bc_order']
            while state['bc_cursor'] < len(order):
                check_deadline(deadline)
                index = order[state['bc_cursor']:state['bc_cursor'] + bc['minibatch_size']]
                losses = trainer._imitation_loss(demo_data['observations'][index], demo_data['actions'][index])
                loss = bc_objective(losses, cfg)
                if not all(torch.isfinite(v) for v in (*losses, loss)): raise ValueError('Nonfinite BC loss')
                bc_optimizer.zero_grad(set_to_none=True); loss.backward()
                torch.nn.utils.clip_grad_norm_(model.actor.parameters(), cfg['optimizer']['max_grad_norm'], error_if_nonfinite=True)
                bc_optimizer.step()
                state['bc_cursor'] += len(index); state['bc_optimizer_steps'] += 1
                state['bc_optimizer'] = bc_optimizer.state_dict()
                with (run / 'bc-metrics.jsonl').open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps(dict(epoch=state['bc_epoch'], cursor=state['bc_cursor'],
                        optimizer_steps=state['bc_optimizer_steps'], loss=float(loss.detach()),
                        acceleration_loss=float(losses[1].detach()), lane_loss=float(losses[2].detach()),
                        acceleration_loss_weight=bc.get('acceleration_loss_weight', 1.),
                        acceleration_rmse_m_s2=3.25 * float(losses[1].detach().sqrt()))) + '\n')
                checkpoint()
            state['bc_epoch'] += 1; state['bc_cursor'] = 0; state['bc_order'] = None
            checkpoint()
        if not state['bc_complete']:
            with torch.no_grad(): model.log_std.fill_(bc['initial_log_std'])
            state['bc_complete'] = True; checkpoint()
        if not state['after_bc_evaluated']:
            validation(True, 'validation-after-bc')
            state['after_bc_evaluated'] = True
            while state['next_sentinel'] <= state['demo_spent'] + state['ppo_spent']:
                state['next_sentinel'] += cfg['evaluation']['every_agent_transitions']
            checkpoint()
        update_pending()
        for previous, phase in enumerate(cfg['phases']):
            if phase['agent_transitions'] - state['phase_spent'][previous] < 4 and previous not in state['evaluated_boundaries']:
                validation(True, f'validation-phase-{previous}')
                state['evaluated_boundaries'].append(previous); checkpoint()
        while phase_index(cfg, state['phase_spent']) is not None:
            check_deadline(deadline)
            p = phase_index(cfg, state['phase_spent'])
            reserved = reserve_episode(state, cfg, p)
            if reserved is None: break
            seed, limit = reserved
            checkpoint()
            report, batch, _ = episode(env_for(cfg, run / 'train', seed, phase=cfg['phases'][p],
                episode_index=state['phase_episodes'][p] - 1), seed, kind='learned', model=model,
                collect=True, limit=limit, deadline=deadline)
            settle_episode(state, report['agent_transitions'])
            if batch is None: break
            # The PPO sample was freshly generated by this current actor. Teacher
            # labels enter a separate auxiliary loss, never its likelihood ratio.
            state['pending'] = dict(kind='ppo_update', batch={k: v.detach() for k, v in batch.items()}, report=report, phase=p)
            checkpoint()
            update_pending()
            boundary = cfg['phases'][p]['agent_transitions'] - state['phase_spent'][p] < 4
            total = state['demo_spent'] + state['ppo_spent']
            due = total >= state['next_sentinel']
            if boundary:
                validation(True, f'validation-phase-{p}')
                state['evaluated_boundaries'].append(p)
            elif due:
                validation(False, f'sentinel-{state["updates"]:05d}')
            if due:
                while state['next_sentinel'] <= total: state['next_sentinel'] += cfg['evaluation']['every_agent_transitions']
            checkpoint()
        result.update(passed=True, stop_reason='transition_budget')
    except PilotBudgetExhausted:
        result.update(passed=True, stop_reason='wall_time_budget', incomplete_episode_or_validation_discarded=True)
    except Exception as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
        # Roll back an interrupted optimizer operation to the last atomic model,
        # optimizer, RNG and pending fresh batch. Episode reservations stay charged.
        stable = torch.load(run / 'last.pt', map_location='cpu', weights_only=False)
        state = stable['state']; model.load_state_dict(stable['model'])
        trainer.optimizer.load_state_dict(stable['optimizer'])
        torch.set_rng_state(stable['torch_rng']); np.random.set_state(stable['numpy_rng']); random.setstate(stable['python_rng'])
    finally:
        state['run_closed'] = True
        checkpoint()
        selected = selected_contract(run, state, fingerprint)
        result.update(optimizer_steps=state['optimizer_steps'], bc_optimizer_steps=state['bc_optimizer_steps'],
            actual_agent_transitions=state['demo_actual'] + state['ppo_actual'],
            charged_agent_transitions=state['demo_spent'] + state['ppo_spent'],
            demo_actual=state['demo_actual'], ppo_actual=state['ppo_actual'], updates=state['updates'],
            elapsed_seconds=state['elapsed_seconds'], best_rank=state['best_rank'],
            selected_generation=state['selected_generation'],
            quality_accepted=selected.get('quality_accepted', False) if selected else False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT)
    parser.add_argument('--mode', choices=('config', 'preflight', 'prepare-teacher', 'train', 'evaluate'), default='config')
    parser.add_argument('--approved-config-sha256')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--split', choices=('validation', 'test'), default='validation')
    args = parser.parse_args(argv)
    if Path.cwd().resolve() != ROOT: raise ValueError('Run from the project root')
    cfg = load_config(args.config); fingerprint = config_digest(cfg)
    if args.mode == 'config':
        print(json.dumps(dict(config=cfg, sha256=fingerprint), indent=2)); return 0
    if args.mode == 'train':
        if not cfg['training_authorized'] or args.approved_config_sha256 != fingerprint:
            raise ValueError('Training awaits explicit authorization and matching reviewed config hash')
        readiness_check(cfg)
    if args.resume and (args.mode != 'train' or not args.resume.resolve().is_relative_to(ROOT / 'outputs')):
        raise ValueError('Resume requires a project-local training checkpoint')
    if args.checkpoint and not args.checkpoint.resolve().is_relative_to(ROOT / 'outputs'):
        raise ValueError('Checkpoint must remain under project outputs')
    for case in cfg['curriculum']:
        if digest(ROOT / case['network']) != case['network_sha256']: raise ValueError('Frozen network hash mismatch')
    torch.set_num_threads(1); torch.manual_seed(cfg['training']['seed'])
    np.random.seed(cfg['training']['seed']); random.seed(cfg['training']['seed'])
    torch.use_deterministic_algorithms(True)
    run = ROOT / 'outputs/highway-stage1' / cfg.get('experiment_version', 'v02') / (args.mode + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
    run.mkdir(parents=True); atomic_json(run / 'config.json', cfg); atomic_json(run / 'contract.json', contract(cfg))
    hashes = source_hashes(); started = time.monotonic()
    import traci
    atomic_json(run / 'provenance.json', dict(config_sha256=fingerprint, source_hashes=hashes,
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True),
        python=platform.python_version(), torch=torch.__version__, traci=traci.__version__,
        command=list(argv) if argv is not None else __import__('sys').argv[1:], resume=str(args.resume) if args.resume else None,
        network_hashes={c['id']: c['network_sha256'] for c in cfg['curriculum']}))
    diff = subprocess.check_output(['git', 'diff', 'HEAD', '--binary'], cwd=ROOT)
    (run / 'working-tree.diff').write_bytes(diff)
    atomic_json(run / 'source-snapshot.json', dict(diff_sha256=hashlib.sha256(diff).hexdigest(), source_hashes=hashes))
    for path in hashes:
        destination = run / 'source' / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / path, destination)
        if digest(destination) != hashes[path]: raise ValueError('Runtime source changed while snapshotting')
    result = dict(passed=False, optimizer_steps=0, bc_optimizer_steps=0, training=args.mode == 'train',
                  source_hashes=hashes, preparation_sha256=preparation_digest(cfg), held_out_test_run=False)
    try:
        result['geometry'] = geometry_check(cfg)
        if args.mode in ('preflight', 'prepare-teacher'):
            deadline = started + cfg['preflight']['max_wall_seconds']
            reports = []
            for seed in cfg['evaluation']['preflight_seeds']:
                report, _, data = episode(env_for(cfg, run / 'teacher', seed), seed, deadline=deadline,
                    demonstrations=args.mode == 'prepare-teacher')
                reports.append(report); atomic_json(run / f'teacher-{seed}.json', report)
                if data is not None: save_checkpoint(run / f'demonstration-{seed}.pt', data)
            # Paired preflight seeds span every training layout at each planned
            # merge density. These are training scenarios, never held-out tests.
            for density in cfg['preflight']['merge_density_coverage']:
                phase = dict(task_cycle=['merge'], merge_density=density)
                for seed in cfg['evaluation']['preflight_seeds'][:5]:
                    label = f'teacher-merge-{density}-{seed}'
                    report, _, _ = episode(env_for(cfg, run / label, seed, phase=phase), seed, deadline=deadline)
                    reports.append(report); atomic_json(run / f'{label}.json', report)
            teacher_quality = quality(reports, cfg)
            if args.mode == 'preflight':
                from pettingzoo.test import parallel_api_test, parallel_seed_test
                def factory():
                    check_deadline(deadline)
                    env = env_for(cfg, run / 'api', cfg['evaluation']['preflight_seeds'][0])
                    step = env.step
                    def bounded_step(actions):
                        check_deadline(deadline)
                        return step(actions)
                    env.step = bounded_step
                    return env
                api = factory()
                try: parallel_api_test(api, num_cycles=100)
                finally: api.close()
                parallel_seed_test(factory, num_cycles=100)
                for kind in ('native', 'random'):
                    for seed in cfg['evaluation']['preflight_seeds'][:2]:
                        report, _, _ = episode(env_for(cfg, run / kind, seed), seed, kind=kind, deadline=deadline)
                        atomic_json(run / f'{kind}-{seed}.json', report)
                seed = cfg['evaluation']['preflight_seeds'][0]
                repeat, _, _ = episode(env_for(cfg, run / 'repeat', seed), seed, deadline=deadline)
                comparable = lambda r: {k: v for k, v in r.items() if k != 'episode_directory'}
                if comparable(reports[0]) != comparable(repeat): raise ValueError('Teacher repeat changed')
                model = model_for(cfg)
                _, batch, _ = episode(env_for(cfg, run / 'collector', seed), seed,
                    kind='learned', model=model, collect=True, limit=128, deadline=deadline)
                if batch is None or not all(torch.isfinite(v).all() for v in batch.values()): raise ValueError('Invalid collector batch')
                export_actor(model, run / 'untrained-contract-probe.pt')
                probe = torch.jit.load(str(run / 'untrained-contract-probe.pt'))
                if not torch.equal(probe(batch['local_obs'][0]), model.actor(batch['local_obs'][0])): raise ValueError('Actor export mismatch')
                result.update(api_passed=True, seed_test_passed=True, teacher_repeat_exact=True, collector_finite=True, actor_export_passed=True)
            plumbing = all(r['collisions'] == r['teleports'] == r['overspeed_steps'] == 0 for r in reports)
            result.update(passed=plumbing, plumbing_passed=plumbing,
                          reports=reports, teacher_ready=teacher_quality['passed'], teacher_quality=teacher_quality,
                          demonstration_use='diagnostic preparation only; not imported into learning budget')
        elif args.mode == 'evaluate':
            if args.checkpoint is None: raise ValueError('Evaluation requires an explicit frozen checkpoint')
            saved = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
            if saved['schema'] != SCHEMA or saved['config_sha256'] != fingerprint: raise ValueError('Checkpoint contract mismatch')
            if args.split == 'test' and saved.get('selection_status') != 'validation_selected': raise ValueError('Test requires validation-selected checkpoint')
            if args.split == 'test':
                accepted = json.loads((args.checkpoint.parent / 'policy-contract.json').read_text())
                if not accepted.get('quality_accepted') or accepted.get('config_sha256') != fingerprint:
                    raise ValueError('Held-out test requires nominal quality acceptance before transfer testing')
            model = model_for(cfg); model.load_state_dict(saved['model']); model.eval()
            reports = evaluate(cfg, run, model, cfg['evaluation'][args.split + '_seeds'], split=args.split,
                               deadline=started + cfg['evaluation']['max_wall_seconds'])
            result.update(passed=True, reports=reports, quality=quality(reports, cfg), held_out_test_run=args.split == 'test')
        else:
            result.update(train(cfg, run, fingerprint, hashes, started, args.resume))
    except Exception as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        result['sources_changed'] = [p for p, h in hashes.items() if digest(ROOT / p) != h]
        if result['sources_changed']: result['passed'] = False
        result['wall_seconds'] = time.monotonic() - started
        atomic_json(run / 'summary.json', result)
    print(json.dumps(dict(run=str(run), **result), indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
