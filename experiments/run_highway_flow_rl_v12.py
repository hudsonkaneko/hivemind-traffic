"""V12 merge runner: target-speed inputs, the target_band reward and the LA I-10/I-110 corridors.

Reuses experiments/run_highway_flow_rl_v11.py (and through it the v10, v09, v06 and
v05 loops: ledger, preflight, complete-episode rollouts, buffered PPO updates,
checkpoints, periodic validation, collision and intervention stop rules and the
canonical ledger through Git's common directory) and replaces only:
- the environment (HighwayFlowRLV12Env: 121-field actor observation, 2928-field
  critic state, the target_band term) and the episode loop's width checks;
- the model (policies/highway_flow_v12.py): the pinned 117-field warm start migrated
  with zero weights on the four new inputs, so it starts exactly where it was;
- the trainer: buffered MAPPO plus the v11 shield-consistency loss plus an anchor
  that holds the actor to the warm start on every decision without an active merge
  target (AnchoredShieldTrainer). PPO is free only where the new reward applies;
- the merge-speed collapse guard, relative to this round's own initial validation
  (the declared rows include new LA corridors whose starting score is unknown);
- the config contract (traffic/highway_flow_rl_v12_contract.py) and outputs under
  this worktree's outputs/highway-flow-rl/v12 with a v12 checkpoint schema.
No training happens on import.
"""
from __future__ import annotations

import json
import sys
import time
import traceback
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from experiments import run_highway_flow_rl_v06 as v06runner  # noqa: E402
from experiments import run_highway_flow_rl_v11 as previous  # noqa: E402
from experiments.run_highway_flow_rl_v06 import BufferedMappoTrainer, StopRuleTriggered  # noqa: E402
from policies import highway_flow_v12 as flow12  # noqa: E402
from policies.mappo import compute_gae, export_actor  # noqa: E402
from traffic import highway_flow_rl_v12_contract as contract  # noqa: E402
from traffic import highway_flow_rl_v12_rewards as v12  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402
from traffic.highway_merge_context import CAP  # noqa: E402

base = previous.base
SOURCES = ('experiments/run_highway_flow_rl_v12.py', 'environments/highway_flow_rl_v12_env.py',
           'traffic/highway_flow_rl_v12_rewards.py', 'traffic/highway_flow_rl_v12_contract.py',
           'policies/highway_flow_v12.py', 'scripts/prepare_highway_flow_rl_v12.py',
           'scripts/build_la_10_110_corridors.py', 'scripts/build_la_10_110_corridors_exact.py')
OBSERVATION_SIZE, STATE_SIZE = flow12.OBSERVATION_SIZE, flow12.STATE_SIZE
CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v12'


def env_for(cfg, row, directory):
    from environments.highway_flow_rl_v12_env import HighwayFlowRLV12Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    env = HighwayFlowRLV12Env(network, directory,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], rate_limit=cfg['rate_limit'],
        v06_weights=cfg['v06_reward_weights'], v10_settings=cfg['v10_reward_settings'],
        v11_settings=cfg['v11_settings'], v12_settings=cfg['v12_settings'],
        missed_gap=row.get('missed_gap', False), controlled_car_follow_model=cfg['controlled_car_follow_model'],
        raw_request_scope=cfg.get('raw_request_scope', 'all'),
        road_end_leader_exit_rule=cfg['road_end_leader_exit_rule'])
    env.case_id = case['id']
    return env


def episode(cfg, row, directory, model, *, collect=False, limit=None, deadline=None, record=False):
    """experiments/run_highway_flow_rl_v05.episode with the v12 widths (121 / 2928)."""
    env = base.env_for(cfg, row, directory)
    samples, frames, decisions, steps, report = [], [], 0, 0, None
    modes = {}
    agents = base.AGENTS
    try:
        base.check_deadline(deadline)
        obs, _ = env.reset(seed=row['seed'])
        controlled = list(env._controlled_agents())
        if record:
            from scripts.prepare_highway_flow import capture_frame
            frames.append(capture_frame(env))
            advance = env._advance

            def recorded_advance():
                result = advance()
                frames.append(capture_frame(env))
                return result
            env._advance = recorded_advance
        while env.agents:
            base.check_deadline(deadline)
            if limit is not None and decisions + len(env.agents) > limit:
                break
            if row.get('limit_steps') is not None and steps >= row['limit_steps']:
                break
            for agent in env.agents:
                value = np.asarray(obs[agent])
                if value.shape != (OBSERVATION_SIZE,) or not np.isfinite(value).all():
                    raise ValueError('Invalid local observation')
                modes[agent] = dict(speed=env.conn.vehicle.getSpeedMode(agent), lane_change=env.conn.vehicle.getLaneChangeMode(agent))
                if modes[agent] != dict(speed=31, lane_change=512):
                    raise ValueError('Native safety mode changed')
            active, indices = torch.tensor([agent in env.agents for agent in agents]), torch.arange(24)
            local = torch.tensor(np.stack([obs.get(agent, np.zeros(OBSERVATION_SIZE, np.float32)) for agent in agents]))
            global_state = np.asarray(env.state())
            if global_state.shape != (STATE_SIZE,) or not np.isfinite(global_state).all():
                raise ValueError('Invalid centralized critic state')
            state = torch.as_tensor(global_state).repeat(24, 1)
            with torch.no_grad():
                actions, logp, _, latent = model.act(local, deterministic=not collect)
                values = model.values(state, indices)
            if not torch.isfinite(actions).all() or (actions[:, 0].abs() > 1).any() or not ((actions[:, 1] == -1) | (actions[:, 1] == 0) | (actions[:, 1] == 1)).all():
                raise ValueError('Invalid normalized acceleration/lane action')
            chosen = {agent: actions[i].numpy() for i, agent in enumerate(agents) if agent in env.agents}
            nxt, rewards, term, trunc, infos = env.step(chosen)
            if not all(np.isfinite(value) for value in rewards.values()) or any(info.get('teacher_reference_used') is not False for info in infos.values()):
                raise ValueError('Nonfinite or teacher-derived reward')
            decisions += len(rewards)
            steps += 1
            if collect:
                successor = np.concatenate([np.r_[nxt.get(agent, np.zeros(OBSERVATION_SIZE)), float(not term.get(agent, True))] for agent in agents]).astype(np.float32)
                with torch.no_grad():
                    next_values = model.values(torch.tensor(successor).repeat(24, 1), indices)
                samples.append(dict(local_obs=local, global_states=state, agent_indices=indices,
                    actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
                    next_values=next_values, rewards=torch.tensor([rewards.get(agent, 0.) for agent in agents]),
                    terminated=torch.tensor([term.get(agent, False) for agent in agents]),
                    truncated=torch.tensor([trunc.get(agent, False) for agent in agents]), active=active))
            obs = {agent: nxt[agent] for agent in env.agents}
        partial = bool(env.agents)
        if samples and partial:
            samples[-1]['truncated'] |= samples[-1]['active'] & ~samples[-1]['terminated']
        report = base.primary_accounting(env, partial=partial, deadline=deadline)
        report.update(case_id=env.case_id, seed=row['seed'], task=row['case']['task'], agent_transitions=decisions,
            partial_control_horizon=partial, population_setting=deepcopy(row), native_control_modes=modes,
            episode_directory=directory.relative_to(ROOT).as_posix(), teacher_reference_used=False)
        if record:
            from scripts.prepare_highway_flow import route_metrics
            report['full_route_metrics'] = route_metrics(frames, controlled)
            base.save(directory / 'recording.json', dict(schema='highway_flow_rl_recording_v05',
                frames=frames, controlled_ids=controlled, case_id=env.case_id, seed=row['seed'], dt_s=.1))
        batch = None
        if samples:
            batch = {key: torch.stack([sample[key] for sample in samples]) for key in samples[0]}
            batch['advantages'], batch['returns'] = compute_gae(batch['rewards'], batch['old_values'],
                batch['next_values'], batch['terminated'], batch['truncated'], batch['active'],
                gamma=cfg['gae']['gamma'], gae_lambda=cfg['gae']['lambda'])
        return report, batch
    except BaseException:
        base.save(directory / 'interrupted-accounting.json', dict(actual_agent_transitions=decisions,
            incomplete_episode=True, traceback=traceback.format_exc()))
        if record and frames:
            base.save(directory / 'recording-partial.json', dict(frames=frames, incomplete_episode=True))
        raise
    finally:
        env.close()
        if report is not None:
            from traffic.sumo_warnings import native_emergency_report
            report['native_emergencies'] = native_emergency_report(env.directory / 'sumo-errors.log')


def finish_preflight(cfg, output, model, trainer, report, batch, fingerprint, sources, deadline):
    """experiments/run_highway_flow_rl_v05.finish_preflight with the v12 width and the v12 reward check."""
    if batch is None:
        raise ValueError('Preflight yielded no active on-policy batch')
    flow = base.flow
    before = deepcopy(dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(), rng=flow.capture_rng()))
    diagnostics = trainer.diagnose_batch(batch, phase='v12_zero_update_preflight')
    json.dumps(diagnostics, allow_nan=False)
    if not flow.tree_equal(before, dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(), rng=flow.capture_rng())):
        raise ValueError('Zero-update diagnostics changed weights, optimizer or RNG')
    if any(report.get(key) != 0 for key in ('collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed')):
        raise ValueError('Preflight observed native safety events')
    native = report.get('native_emergencies', {})
    if native.get('available') is not True or native.get('unique_events') != 0 or not report['native_control_modes']:
        raise ValueError('Preflight lacks clean native warning/control-mode evidence')
    if not report.get('v04_reward_summary') or not report.get('v04_mask_audit'):
        raise ValueError('Preflight lacks measured reward/mask audit')
    if report.get('reward_version') != cfg['reward_version'] or report['v04_mask_audit'].get('version') != cfg['mask_version']:
        raise ValueError('Measured reward or mask version differs from the resolved configuration')
    if report.get('observation_size') != OBSERVATION_SIZE or report.get('v12_settings') != cfg['v12_settings']:
        raise ValueError('Measured v12 observation or target settings differ from the configuration')
    actor_path = output / 'actor-preflight.pt'
    export_actor(model, actor_path)
    actor = torch.jit.load(str(actor_path), map_location='cpu')
    probe = batch['local_obs'].reshape(-1, OBSERVATION_SIZE)[batch['active'].reshape(-1)]
    with torch.no_grad():
        if not torch.equal(model.act(probe, deterministic=True)[0], actor(probe)):
            raise ValueError('Export changed deterministic actor actions')
    base.check_deadline(deadline)
    if base.hashes() != sources:
        raise ValueError('Sources changed during preflight')
    proof = dict(schema='highway_flow_rl_preflight_v05', passed=True, training=False, optimizer_steps=0,
        config_sha256=fingerprint, source_hashes=sources,
        checks=dict(api=True, shapes=True, native_guards=True, finite_gradient=True, export=True, reward_version=True, mask_version=True),
        report=report, gradient_diagnostics=diagnostics,
        scope='Zero-update runtime evidence for the v12 widths; no teacher or performance qualification')
    base.validate_preflight(proof, fingerprint, sources)
    base.save(output / 'preflight.json', proof)
    return dict(completed=True, training=False, preflight=str(output / 'preflight.json'))


def mean_request(raw):
    return -4.5 + (raw[..., 0].tanh() + 1.) * 3.25      # environments.sumo_mappo_env.action_to_acceleration


class AnchoredShieldTrainer(BufferedMappoTrainer):
    """Buffered MAPPO + the v11 shield-consistency loss + an anchor to the warm-start actor.

    anchor: on decisions whose observation shows no active merge target
    (target_active input is 0) the mean request and the lane outputs are held to the
    frozen warm-start actor:
        anchor_coef * (request - warm request)^2 + anchor_lane_coef * mean (lane - warm lane)^2.
    Where a merge target is active (any faded level above 0) the anchor is off, so
    the PPO update, driven by the new target_band reward and every inherited term,
    decides the merge behavior. Round 15 showed unanchored PPO at this scale drifts
    everywhere (5 of 8 gates to 2 of 8).
    shield: as experiments/run_highway_flow_rl_v11.ShieldConsistentTrainer, with the
    optional speed-limit-aware upper bound (comfort_speed_ceiling) of round 17.
    """

    def __init__(self, model, *, anchor=None, anchor_coef=0., anchor_lane_coef=0., shield_coef=0.,
                 ceiling_margin_m_s2=.05, rate_bound_m_s2=.35, comfort_speed_ceiling=False, **kwargs):
        numbers = (anchor_coef, anchor_lane_coef, shield_coef, ceiling_margin_m_s2, rate_bound_m_s2)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and v >= 0 for v in numbers):
            raise ValueError('Invalid anchor or shield settings')
        if (anchor_coef or anchor_lane_coef) and anchor is None:
            raise ValueError('An anchored trainer needs the frozen warm-start model')
        if type(comfort_speed_ceiling) is not bool:
            raise ValueError('comfort_speed_ceiling must be a boolean')
        super().__init__(model, **kwargs)
        self.anchor = anchor
        self.anchor_coef, self.anchor_lane_coef, self.shield_coef = anchor_coef, anchor_lane_coef, shield_coef
        self.ceiling_margin, self.rate_bound = ceiling_margin_m_s2, rate_bound_m_s2
        self.comfort_speed_ceiling = comfort_speed_ceiling

    def _loss_terms(self, part, ratio, entropy):
        loss, weighted, stats = super()._loss_terms(part, ratio, entropy)
        observations = part['local_obs']
        raw = self.model.actor.mean_net(observations)
        request = mean_request(raw)
        if self.shield_coef:
            margin = ((observations[..., previous.LIMIT_INDEX] - observations[..., previous.SPEED_INDEX]) * CAP
                      if self.comfort_speed_ceiling else None)
            distance = previous.shield_consistency(request,
                observations[..., previous.ACCELERATION_INDEX] * previous.OBSERVATION_SCALE_M_S2,
                observations[..., previous.CEILING_INDEX] * previous.OBSERVATION_SCALE_M_S2,
                ceiling_margin=self.ceiling_margin, rate_bound=self.rate_bound, speed_margin=margin)
            weighted['shield'] = self.shield_coef * distance.mean()
            loss = loss + weighted['shield']
            stats.update(shield_loss_m2_s4=float(distance.mean().detach()),
                         shield_outside_fraction=float((distance > 0).float().mean().detach()))
        if self.anchor_coef or self.anchor_lane_coef:
            held = observations[..., v12.TARGET_START] <= 0.
            with torch.no_grad():
                reference = self.anchor.actor.mean_net(observations)
            zero = raw.sum() * 0.
            if held.any():
                acceleration = (request - mean_request(reference))[held].square().mean()
                lane = (raw[..., 1:] - reference[..., 1:])[held].square().mean()
            else:
                acceleration = lane = zero
            weighted['anchor'] = self.anchor_coef * acceleration
            weighted['anchor_lane'] = self.anchor_lane_coef * lane
            loss = loss + weighted['anchor'] + weighted['anchor_lane']
            stats.update(anchor_request_mse_m2_s4=float(acceleration.detach()),
                         anchor_lane_mse=float(lane.detach()), anchor_held_fraction=float(held.float().mean()),
                         target_active_samples=int((~held).sum()))
        if not torch.isfinite(loss):
            raise ValueError('Nonfinite v12 loss')
        return loss, weighted, stats


def load_warm_start(cfg):
    """The pinned 117-field checkpoint, migrated to v12 (identical outputs, zero weights on new inputs)."""
    path = base.project_path(cfg['warm_start']['path'])
    if file_digest(path) != cfg['warm_start']['sha256']:
        raise ValueError('Pinned warm-start checksum changed')
    saved = torch.load(path, map_location='cpu', weights_only=False)
    if saved.get('schema') != contract.WARM_START_SCHEMA or 'model' not in saved:
        raise ValueError('V12 warm start must be the pinned v11 projection checkpoint')
    return flow12.migrate_from_flow117(saved['model'])


def trainer_for(cfg, anchor):
    """Trainer factory bound to the round's v12 training settings and the frozen anchor."""
    extra = dict(cfg['v12_training'])

    def build(model, **optimizer):
        return AnchoredShieldTrainer(model, anchor=anchor, **optimizer, **extra)
    return build


_v05_build_model = base.build_model


def build_model(cfg, *, checkpoint=None, resume=False, fingerprint=None, sources=None, evaluation=False):
    """Warm start: migrate the pinned 117 checkpoint. Resume / evaluation of v12 checkpoints: the v05 loader."""
    anchor_model, migration = load_warm_start(cfg)
    base.MappoTrainer = trainer_for(cfg, flow12.frozen_copy(anchor_model))  # looked up at call time
    path = base.project_path(checkpoint or cfg['warm_start']['path'])
    if resume or path != base.project_path(cfg['warm_start']['path']):
        return _v05_build_model(cfg, checkpoint=checkpoint, resume=resume, fingerprint=fingerprint,
                                sources=sources, evaluation=evaluation)
    model, _ = load_warm_start(cfg)
    return model, base.MappoTrainer(model, **cfg['optimizer']), dict(schema=contract.WARM_START_SCHEMA,
        migration=migration, state=None)


def merge_floor(cfg, output, label, result):
    """Stop when a periodic validation's merge speed match falls `drop` below this round's initial validation."""
    rule = cfg['stop_rules']['relative_merge_floor']
    initial_path = output / 'initial-validation.json'
    score = result['summary'].get('merge_speed_match')
    if not initial_path.is_file():
        return
    initial = v06runner.merge_speed_match(json.loads(initial_path.read_text())['reports'])
    floor = None if initial is None else max(rule['minimum'], initial - rule['drop'])
    result['summary'].update(relative_merge_floor=floor, initial_merge_speed_match=initial)
    if label.startswith('periodic-validation') and floor is not None and (score is None or score < floor):
        raise StopRuleTriggered(f'{label}: merge speed match {score} fell below {floor} '
                                f'(initial {initial} minus {rule["drop"]})')


_v06_evaluate = v06runner.evaluate


def evaluate(cfg, output, label, model, deadline, **kwargs):
    result = _v06_evaluate(cfg, output, label, model, deadline, **kwargs)
    merge_floor(cfg, output, label, result)
    reports = result['reports']
    result['summary']['v12_target_band'] = dict(
        in_band_fraction=_ratio(sum((r.get('v12_metrics') or {}).get('in_band_decisions', 0) for r in reports),
                                sum((r.get('v12_metrics') or {}).get('band_decisions', 0) for r in reports)),
        per_case={r['case_id']: (r.get('v12_metrics') or {}).get('in_band_fraction') for r in reports})
    base.save(output / f'{label}.json', result)
    return result


def _ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def install():
    """Point the shared loop at the v12 pieces (module globals are looked up at call time)."""
    previous.install()
    base.OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v12'
    base.CHECKPOINT_SCHEMA = CHECKPOINT_SCHEMA
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.build_model, base.evaluate = env_for, build_model, evaluate
    base.FlowModel = flow12.FlowModelV12
    base.critic_update = flow12.critic_update
    base.finish_preflight = finish_preflight
    v06runner._episode = episode          # base.episode is v06's collision-stop wrapper around this


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
