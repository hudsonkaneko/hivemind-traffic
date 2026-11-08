"""Explicit contracts and actor migration for the v05 environment pilot."""
from __future__ import annotations

import math
import torch

SCHEMA = 'highway_environment_v05'
OBSERVATION_SIZE = 99
MAX_AGENTS = 24
STATE_SIZE = 2400


def validate_config(cfg):
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('V05 schema/authorization mismatch')
    if cfg['model'] != dict(observation_size=99, state_size=2400, num_agents=24,
            driving_local_size=51, driving_message_size=8, driving_mask_start=30, population_critic=True):
        raise ValueError('Explicit v05 actor and critic contract required')
    t, env = cfg['training'], cfg['environment']
    if (t['device'] != 'cpu' or t['num_workers'] != 1 or t['torch_threads'] != 1
            or not 1 <= t['max_agent_transitions'] <= 250000
            or not 1 <= t['max_wall_seconds'] <= 3600
            or not 0 <= t['critic_warmup_transitions'] <= 4096
            or not 0 < t['final_evaluation_reserve_seconds'] < t['max_wall_seconds']):
        raise ValueError('V05 bounded pilot budget exceeded')
    if (env['horizon'] != 800 or env['control_target_version'] != 'v3'
            or env['lane_change_duration'] != 3. or env['stabilization_seconds'] != 8.):
        raise ValueError('Reviewed continuous maneuver contract required')
    if cfg['output_directory'] != 'outputs/highway-environment/v05':
        raise ValueError('V05 outputs must be isolated')
    if cfg['population']['total_vehicles'] != [8, 16, 24]:
        raise ValueError('Declared population domain changed')
    if cfg['population']['training_fractions'] != [.125, .25, .5, .75, 1.]:
        raise ValueError('AV participation coverage changed')
    if cfg['speed_curriculum']['ceilings_m_s'] != [12., 18., 22.352, 26.8224]:
        raise ValueError('Four speed ceilings required')
    if cfg['optimizer'].get('near_limit_coef') != .1 or cfg['optimizer'].get('gradient_diagnostics') is not True:
        raise ValueError('Preserve near-limit objective and measured optimization diagnostics')
    if set(t['tasks']) != {'cruise', 'following', 'passing', 'merge'}:
        raise ValueError('Preserve all driving tasks')
    if not cfg['training_cases'] or any(c['split'] != 'train' for c in cfg['training_cases']):
        raise ValueError('Training partition only')
    rows = cfg['validation_cases']
    if len(rows) != 24 or len({r['seed'] for r in rows}) != 24:
        raise ValueError('Twenty-four fixed validation cases required')
    for row in rows:
        n, total = row['controlled_count'], row['total_vehicles']
        if (row['case']['split'] != 'validation' or not 1 <= n <= total <= 24
                or t['seed_start'] <= row['seed'] <= t['seed_end']
                or row['speed_limit_m_s'] not in cfg['speed_curriculum']['ceilings_m_s']):
            raise ValueError('Invalid validation split, population or speed')
        if n == total and (row['traffic_mode'] != 'initial' or row['case']['task'] in ('following', 'passing')):
            raise ValueError('Full-AV cases must not hide native inflow or a scripted leader')
    if not any(r['controlled_count'] == r['total_vehicles'] == 24 for r in rows):
        raise ValueError('Dense full-AV validation required')
    if cfg['preflight']['learning'] is not False or cfg['preflight']['max_wall_seconds'] > 900:
        raise ValueError('Preflight must be bounded and cannot learn')


def migrate_actor(model, saved):
    """Copy 45 old local columns and 17 pooled features; zero six new columns.

    This preserves old predictions on identical old features. New v3/motion
    semantics still require training/evaluation; width compatibility is not
    claimed. Optimizer and centralized critic are deliberately not transferred.
    """
    actor = {k[len('actor.'):]: v for k, v in saved['model'].items() if k.startswith('actor.')}
    name = 'mean_net.1.0.weight'
    previous = actor[name]
    target = model.actor.state_dict()[name]
    if previous.shape != (128, 62) or target.shape != (128, 68):
        raise ValueError('Unexpected actor migration layout')
    weight = torch.zeros_like(target)
    weight[:, :45] = previous[:, :45]
    weight[:, 51:] = previous[:, 45:]
    actor[name] = weight
    model.actor.load_state_dict(actor, strict=True)
    with torch.no_grad():
        model.log_std.copy_(saved['model']['log_std'])
    return dict(old_observation_size=93, new_observation_size=99,
                inserted_local_indices=list(range(45, 51)), initialization='zero input weights',
                critic='fresh masked 24-slot critic', optimizer='fresh')


def acceptance(reports):
    """Same nominal safety gates, plus completed continuous/stable merges."""
    if not reports:
        raise ValueError('Cannot accept an empty evaluation')
    failures = []
    for r in reports:
        integer_fields = ('agent_steps', 'overrides', 'collisions', 'teleports', 'overspeed_steps',
                          'emergency_braking_observed', 'passes', 'successful_merges', 'controlled_count')
        for key in integer_fields:
            value = r.get(key)
            if type(value) is not int or value < 0:
                raise ValueError('Missing/nonfinite integer evidence: ' + key)
        for key in ('intervention_fraction', 'mean_fraction_of_speed_limit'):
            value = r.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError('Missing/nonfinite driving evidence: ' + key)
        if r['overrides'] > r['agent_steps'] or r['intervention_fraction'] > 1.:
            raise ValueError('Invalid intervention denominator')
        if not math.isclose(r['intervention_fraction'], r['overrides']/max(1,r['agent_steps']), abs_tol=1e-10):
            raise ValueError('Intervention fraction does not match raw counts')
        if r['controlled_count'] and not r['agent_steps']:
            raise ValueError('Controlled evaluation has no decisions')
        reasons = []
        if any(r[k] for k in ('collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed')):
            reasons.append('safety_event')
        native = r.get('native_emergencies', {})
        if native.get('available') is not True or type(native.get('unique_events')) is not int or native['unique_events'] != 0:
            reasons.append('native_emergency_or_missing_accounting')
        if r.get('primary_horizon_complete') is not True or r.get('accounting_comparison_valid') is not True or r.get('partial_control_horizon') is not False:
            reasons.append('incomplete_episode')
        if not math.isfinite(r['intervention_fraction']) or r['intervention_fraction'] > .05:
            reasons.append('interventions')
        if r['task'] == 'cruise' and r['mean_fraction_of_speed_limit'] < .9:
            reasons.append('cruise_speed')
        if r['task'] == 'passing' and r['passes'] < 1:
            reasons.append('passing')
        if r['task'] == 'merge' and r['controlled_count']:
            targets = r.get('merge_target_count')
            result = r.get('stable_merge', {})
            stable = result.get('stable')
            if type(targets) is not int or targets <= 0 or type(stable) is not int or not 0 <= stable <= targets:
                raise ValueError('Missing/invalid controlled merge stability evidence')
            if stable / targets < .9:
                reasons.append('stable_merge')
        if reasons:
            failures.append(dict(case_id=r['case_id'], seed=r['seed'], reasons=reasons))
    decisions = sum(r['agent_steps'] for r in reports)
    return dict(cases=len(reports), failures=failures, all_gates_passed=not failures,
        pooled_intervention_fraction=sum(r['overrides'] for r in reports) / max(1, decisions),
        worst_intervention_fraction=max((r['intervention_fraction'] for r in reports), default=0.),
        merges=sum(r['successful_merges'] for r in reports),
        stable_merges=sum(r.get('stable_merge', {}).get('stable', 0) for r in reports),
        passes=sum(r['passes'] for r in reports), held_out_test_run=False,
        acceptance_status='development validation only; no held-out promotion')
