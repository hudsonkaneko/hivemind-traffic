"""Frozen CPU reduction for the approved first one-car PPO pilot.

Evidence validity is not driving success. The unchanged scripted-validation
assessor supplies its schema, native-guard, reward and transition replay. For
arbitrary failures we invoke its lane-departure template without changing any
raw record, replace only template-specific ending checks, and report physical
quality separately. A failed policy episode is retained, never counted as a
successful drive merely because its failure was accurately recorded.

Checkpoint file hashes must be computed by the package owner from frozen model
and normalization files. This module checks their exact identity throughout the
matrix; it cannot authenticate a caller that fabricates both bytes and hashes.
No runtime, neural-network library, optimizer or simulator is imported.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
import re

from environments.lane_following_contract import make_reset_spec
from experiments import lane_following_lab_validation as audit

UPDATES = (8, 16, 24, 32)
SELECTION_SEEDS = (3101, 3102, 3103)
HELD_OUT_SEEDS = (4101, 4102, 4103, 4104, 4105)
ROUTE_IDS = ('straight100', 'left_r60', 'right_r60')
FAILURE_REASONS = frozenset((
    'rigid_contact', 'command_fallback', 'wheel_support_loss', 'excessive_tilt',
    'unsafe_height', 'lane_departure', 'heading_outside_envelope', 'overspeed',
    'reverse_motion', 'before_route_start', 'endpoint_overshoot', 'state_discontinuity',
))
QUALITY_CHECKS = frozenset((
    'upright', 'native_heights', 'support_fraction', 'unsupported_gap',
    'no_rigid_contacts', 'footprint_containment', 'other_motion_envelopes',
    'no_physical_failure', 'forward_progress', 'lateral_rms', 'lateral_max', 'stopped_hold',
))
TEMPLATE_CHECKS = frozenset(('terminal_reason', 'expected_physical_mechanism'))
CASE_KEYS = frozenset(('label', 'policy', 'checkpoint', 'route_id', 'seed', 'records', 'reset_record'))
CHECKPOINT_KEYS = frozenset(('update', 'control_steps', 'model_sha256', 'normalization_sha256'))


def case_label(stage, policy, route_id, seed, update=None):
    """Canonical inventory identity; callers must not replace failed seeds."""
    if stage not in ('selection', 'heldout') or policy not in ('ppo', 'scripted'):
        raise ValueError('Unknown evaluation stage or policy')
    if route_id not in ROUTE_IDS or type(seed) is not int:
        raise ValueError('Unknown route or malformed seed')
    if policy == 'ppo':
        if type(update) is not int or update not in UPDATES:
            raise ValueError('Unknown checkpoint update')
        return f'{stage}/ppo/u{update:02d}/{route_id}/seed-{seed}'
    if update is not None or stage != 'heldout':
        raise ValueError('Scripted policy belongs only to the held-out comparison')
    return f'{stage}/scripted/{route_id}/seed-{seed}'


def _checkpoint(value):
    return (isinstance(value, dict) and set(value) == CHECKPOINT_KEYS
        and type(value['update']) is int and value['update'] in UPDATES
        and type(value['control_steps']) is int and value['control_steps'] == value['update']*2048
        and all(isinstance(value[key], str) and re.fullmatch('[0-9a-f]{64}', value[key])
                for key in ('model_sha256', 'normalization_sha256')))


def _stats(values):
    if not values:
        return dict(count=0, mean=None, std_population=None, minimum=None, maximum=None, rms=None)
    mean = math.fsum(values)/len(values)
    return dict(count=len(values), mean=mean,
        std_population=math.sqrt(math.fsum((v-mean)**2 for v in values)/len(values)),
        minimum=min(values), maximum=max(values), rms=math.sqrt(math.fsum(v*v for v in values)/len(values)))


def _action_metrics(records):
    """Count requested Box clipping and brake priority, including ignored settle requests."""
    counts = dict(samples=0, clipped_actions=0, clipped_components=0,
                  brake_throttle_overrides=0, steering_slew_limited=0, guard_brake_intervals=0)
    driving = dict(counts)
    previous_steering = 0.
    for row in records:
        raw, supplied, requested = (row.get(key) for key in ('policy_action_raw', 'env_action', 'requested_action'))
        if not all(audit._vector(values, 3) for values in (raw, supplied, requested)):
            return False, {}
        clipped = [max(low, min(high, value)) for value, low, high in zip(raw, (-.5, 0., 0.), (.5, 1., 1.))]
        bounded = [clipped[0], 0. if clipped[2] > 0 else clipped[1], clipped[2]]
        if not audit._same_vector(supplied, clipped, 3) or not audit._same_vector(requested, bounded, 3):
            return False, {}
        active = row['tick'] > audit.SETTLE_TICKS
        # Replay both native command intervals. An odd-tick guard replaces the
        # second command by accepted emergency brake (not a watchdog fallback).
        target = list(bounded) if active and not row['control']['is_fallback'] else [0., 0., 1.]
        steering = previous_steering
        for sample in row['physics_samples']:
            steering += max(-.5/120, min(.5/120, target[0]-steering))
            applied = [steering, target[1], target[2]]
            if sample['guard_reason']:
                target = [0., 0., 1.]
        if not audit._same_vector([row['control'][name] for name in ('steering_rad', 'throttle', 'brake')], applied, 3):
            return False, {}
        previous_steering = row['control']['steering_rad']
        guard_brake = row['physics_samples'][0]['guard_reason'] is not None
        components = sum(value < low or value > high for value, low, high in zip(raw, (-.5, 0., 0.), (.5, 1., 1.)))
        for bucket in (counts, driving) if active else (counts,):
            bucket['samples'] += 1
            bucket['clipped_actions'] += bool(components)
            bucket['clipped_components'] += components
            bucket['brake_throttle_overrides'] += clipped[1] > 0 and clipped[2] > 0
            bucket['guard_brake_intervals'] += guard_brake
            bucket['steering_slew_limited'] += active and not guard_brake and not row['control']['is_fallback'] and abs(row['control']['steering_rad']-bounded[0]) > 1e-6
    return True, dict(all_requested=counts, post_settle_requested=driving,
        settling_requests_ignored=counts['samples']-driving['samples'])


def reduce_episode(records, reset_record, route, acceptance):
    """Validate and reduce one complete arbitrary learned-policy ending.

    Finite malformed or incomplete evidence returns evidence_valid=False.
    Metrics on failed evidence are diagnostic only and cannot enter selection.
    Acceleration/jerk use scalar planar speed at120Hz, never vector acceleration
    or comfort certification. Reset contributes tick0 only; no auto-reset jump
    enters derivatives. Post-settle jerk requires both differences after tick240.
    """
    checks, quality, metrics = {}, {}, {}
    reason = records[-1].get('reason') if isinstance(records, list) and records and isinstance(records[-1], dict) else None
    result = dict(evidence_valid=False, driving_success=False, task_success=False,
                  physical_failure=False, terminal_reason=reason, checks=checks,
                  quality_checks=quality, metrics=metrics)
    checks['supported_terminal_reason'] = isinstance(reason, str) and (reason in FAILURE_REASONS or reason in ('success', 'time_limit'))
    if not checks['supported_terminal_reason']:
        return result
    template = reason if reason in ('success', 'time_limit') else 'lane_departure'
    try:
        base = audit.assess_episode(records, reset_record, route, acceptance, expected_reason=template)
        checks.update({key: value for key, value in base['checks'].items()
                       if key not in QUALITY_CHECKS and key not in TEMPLATE_CHECKS})
        quality.update({key: value for key, value in base['checks'].items() if key in QUALITY_CHECKS})
        metrics.update(base['metrics'])
        # A complete replay must reach these checks; early schema rejection must
        # never accidentally become valid after filtering expected quality faults.
        checks['complete_independent_replay'] = all(key in base['checks'] for key in (
            'terminal_decisions_recomputed', 'reward_components_recomputed', 'sb3_terminal_observation',
            'terminal_return_is_reset', 'other_motion_envelopes'))
        if not all(checks.values()):
            return result
        samples = [sample for row in records for sample in row['physics_samples']]
        first = metrics['first_physical_failure']
        # Independently identify control-rate jumps; native guard wins a same
        # control interval, exactly as the frozen task's declared precedence.
        previous = reset_record['physics_state']['position_m'][:2]
        earliest = None
        for row in records:
            native = first if first is not None and first[0] <= row['tick'] else None
            jump = math.dist(previous, row['physics_state']['position_m'][:2]) > 4/60+.005
            if native or jump:
                earliest = (row['tick'], native[1] if native else 'state_discontinuity')
                break
            previous = row['physics_state']['position_m'][:2]
        checks['actual_ending_mechanism'] = (
            earliest == (records[-1]['tick'], reason) if reason in FAILURE_REASONS
            else earliest is None and records[-1]['reason'] == reason)
        actions_ok, actions = _action_metrics(records)
        checks['raw_and_supplied_actions'] = actions_ok
        speeds = [reset_record['physics_state']['speed_m_s']] + [s['physics_state']['speed_m_s'] for s in samples]
        acceleration = [(b-a)*120 for a, b in zip(speeds, speeds[1:])]
        jerk = [(b-a)*120 for a, b in zip(acceleration, acceleration[1:])]
        metrics.update(actions=actions, scalar_speed_derivatives=dict(
            sampling_hz=120, finite_difference='backward differences of scalar planar speed; unsmoothed',
            reset_boundary_excluded=True, all_acceleration_m_s2=_stats(acceleration),
            all_jerk_m_s3=_stats(jerk), post_settle_acceleration_m_s2=_stats(acceleration[240:]),
            post_settle_jerk_m_s3=_stats(jerk[240:])))
        # The generic departure template exempts footprint; restore the actual
        # quality observation without turning a legitimate failure into bad data.
        quality['footprint_containment'] = metrics['footprint_lateral_bound_max_m'] <= route.width_m/2
        quality['no_rigid_contacts'] = metrics['rigid_contact_events'] == 0
        quality['no_physical_failure'] = earliest is None
        result['evidence_valid'] = all(checks.values())
        result['task_success'] = result['evidence_valid'] and reason == 'success'
        result['driving_success'] = result['task_success'] and all(quality.values())
        result['physical_failure'] = result['evidence_valid'] and reason in FAILURE_REASONS
    except (KeyError, TypeError, ValueError, IndexError, OverflowError, ZeroDivisionError):
        checks['malformed_evidence'] = False
    return result


def _case(case, stage, routes, acceptance, checkpoint):
    checks = dict(schema=isinstance(case, dict) and set(case) == CASE_KEYS)
    outcome = None
    if not checks['schema']:
        return dict(valid=False, checks=checks, outcome=None)
    route_id, seed, policy = case['route_id'], case['seed'], case['policy']
    checks['identity'] = (route_id in routes and route_id in ROUTE_IDS and type(seed) is int
                          and policy in ('ppo', 'scripted'))
    checks['checkpoint_identity'] = case['checkpoint'] == checkpoint and (checkpoint is None or _checkpoint(checkpoint))
    if not all(checks.values()):
        return dict(valid=False, checks=checks, outcome=None)
    update = checkpoint['update'] if checkpoint else None
    checks['label'] = case['label'] == case_label(stage, policy, route_id, seed, update)
    expected = asdict(make_reset_spec(routes[route_id], seed=seed, episode_index=0, perturb=True))
    checks['exact_seeded_reset'] = isinstance(case['reset_record'], dict) and case['reset_record'].get('reset_spec') == expected
    outcome = reduce_episode(case['records'], case['reset_record'], routes[route_id], acceptance)
    checks['valid_episode_evidence'] = outcome['evidence_valid']
    if outcome['evidence_valid']:
        expected_auto = asdict(make_reset_spec(routes[route_id], seed=seed, episode_index=1, perturb=True))
        checks['exact_seeded_auto_reset'] = case['records'][-1]['auto_reset']['reset_spec'] == expected_auto
    return dict(valid=all(checks.values()), checks=checks, outcome=outcome)


def _aggregate(outcomes):
    rms = [row['metrics']['driving_lateral_rms_m'] for row in outcomes]
    reasons = {}
    for row in outcomes:
        reasons[row['terminal_reason']] = reasons.get(row['terminal_reason'], 0)+1
    return dict(episodes=len(outcomes), driving_successes=sum(row['driving_success'] for row in outcomes),
        task_successes=sum(row['task_success'] for row in outcomes),
        physical_failures=sum(row['physical_failure'] for row in outcomes), terminal_reasons=reasons,
        mean_progress_m=math.fsum(row['metrics']['progress_m'] for row in outcomes)/len(outcomes),
        mean_driving_lateral_rms_m=None if any(v is None for v in rms) else math.fsum(rms)/len(rms),
        progress_m=_stats([row['metrics']['progress_m'] for row in outcomes]),
        driving_lateral_rms_m=_stats([v for v in rms if v is not None]),
        missing_driving_rms_episodes=sum(v is None for v in rms))


def _rank(row):
    metrics = row['aggregate']
    return (-metrics['driving_successes'], metrics['physical_failures'], -metrics['mean_progress_m'],
            math.inf if metrics['mean_driving_lateral_rms_m'] is None else metrics['mean_driving_lateral_rms_m'],
            row['checkpoint']['update'])


def _fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def select_checkpoint(cases, checkpoints, routes, acceptance):
    """Select from all12predeclared validation episodes, never held-out data.

    A missing driving RMS (failure during settling) is worst at the RMS tie
    break, not silently omitted. No checkpoint is selected from invalid evidence.
    """
    checks = dict(checkpoints=isinstance(checkpoints, list) and len(checkpoints) == 4
                  and all(_checkpoint(c) for c in checkpoints))
    result = dict(valid=False, checks=checks, selected_checkpoint=None, rankings=[], cases={},
        rule=['most_driving_successes', 'fewest_physical_failures', 'highest_mean_progress_m',
              'lowest_mean_driving_lateral_rms_m', 'earliest_update'], selection_seeds=list(SELECTION_SEEDS))
    if not checks['checkpoints']:
        return result
    by_update = {c['update']: c for c in checkpoints}
    checks['exact_checkpoint_inventory'] = set(by_update) == set(UPDATES)
    checks['cases_schema'] = isinstance(cases, list) and all(isinstance(c, dict) and set(c) == CASE_KEYS for c in cases)
    if not all(checks.values()):
        return result
    expected = {case_label('selection', 'ppo', 'left_r60', seed, update) for update in UPDATES for seed in SELECTION_SEEDS}
    labels = [case['label'] for case in cases]
    checks['exact_case_inventory'] = len(labels) == len(expected) and all(isinstance(v, str) for v in labels) and set(labels) == expected
    if not checks['exact_case_inventory']:
        return result
    buckets = {update: [] for update in UPDATES}
    for case in sorted(cases, key=lambda row: row['label']):
        update = case['checkpoint'].get('update') if isinstance(case['checkpoint'], dict) else None
        metadata = case['policy'] == 'ppo' and case['route_id'] == 'left_r60' and case['seed'] in SELECTION_SEEDS and update in by_update
        detail = _case(case, 'selection', routes, acceptance, by_update[update]) if metadata else dict(valid=False, checks=dict(metadata=False), outcome=None)
        result['cases'][case['label']] = detail
        if detail['valid']:
            buckets[update].append(detail['outcome'])
    checks['all_case_evidence_valid'] = all(row['valid'] for row in result['cases'].values())
    if not all(checks.values()):
        return result
    result['rankings'] = sorted([dict(checkpoint=by_update[update], aggregate=_aggregate(buckets[update])) for update in UPDATES], key=_rank)
    result['selected_checkpoint'] = dict(result['rankings'][0]['checkpoint'])
    result['valid'] = True
    result['selection_evidence_sha256'] = _fingerprint(result)
    return result


def assess_held_out(cases, selected_checkpoint, routes, acceptance):
    """Compare frozen selected PPO with scripted on30exact matched episodes.

    This function never selects or changes a checkpoint. A valid result can
    report zero policy successes. Invalid evidence or incomplete inventory
    blocks the aggregate; no seed is replaced or dropped.
    """
    checks = dict(selected_checkpoint=_checkpoint(selected_checkpoint),
                  cases_schema=isinstance(cases, list) and all(isinstance(c, dict) and set(c) == CASE_KEYS for c in cases))
    result = dict(valid=False, checks=checks, selected_checkpoint=selected_checkpoint,
                  cases={}, aggregates={}, paired_differences=[], held_out_used_for_selection=False)
    if not all(checks.values()):
        return result
    update = selected_checkpoint['update']
    expected = {case_label('heldout', policy, route, seed, update if policy == 'ppo' else None)
                for policy in ('ppo', 'scripted') for route in ROUTE_IDS for seed in HELD_OUT_SEEDS}
    labels = [case['label'] for case in cases]
    checks['exact_case_inventory'] = len(labels) == len(expected) and all(isinstance(v, str) for v in labels) and set(labels) == expected
    if not checks['exact_case_inventory']:
        return result
    indexed = {}
    for case in sorted(cases, key=lambda row: row['label']):
        metadata = case['route_id'] in ROUTE_IDS and case['seed'] in HELD_OUT_SEEDS and case['policy'] in ('ppo', 'scripted')
        detail = _case(case, 'heldout', routes, acceptance, selected_checkpoint if case['policy'] == 'ppo' else None) if metadata else dict(valid=False, checks=dict(metadata=False), outcome=None)
        result['cases'][case['label']] = detail
        if detail['valid']:
            indexed[(case['policy'], case['route_id'], case['seed'])] = (case, detail['outcome'])
    checks['all_case_evidence_valid'] = all(row['valid'] for row in result['cases'].values())
    if not all(checks.values()):
        return result
    checks['matched_initial_native_states'] = all(
        audit._raw_states_match(indexed[('ppo', route, seed)][0]['reset_record']['physics_state'],
                                indexed[('scripted', route, seed)][0]['reset_record']['physics_state'])
        for route in ROUTE_IDS for seed in HELD_OUT_SEEDS)
    if not all(checks.values()):
        return result
    for policy in ('ppo', 'scripted'):
        result['aggregates'][policy] = dict(all_routes=_aggregate([indexed[(policy, route, seed)][1] for route in ROUTE_IDS for seed in HELD_OUT_SEEDS]),
            by_route={route: _aggregate([indexed[(policy, route, seed)][1] for seed in HELD_OUT_SEEDS]) for route in ROUTE_IDS})
    for route in ROUTE_IDS:
        for seed in HELD_OUT_SEEDS:
            ppo, script = indexed[('ppo', route, seed)][1], indexed[('scripted', route, seed)][1]
            p, s = ppo['metrics'], script['metrics']
            result['paired_differences'].append(dict(route_id=route, seed=seed,
                driving_success=int(ppo['driving_success'])-int(script['driving_success']),
                physical_failure=int(ppo['physical_failure'])-int(script['physical_failure']),
                progress_m=p['progress_m']-s['progress_m'],
                driving_lateral_rms_m=None if p['driving_lateral_rms_m'] is None or s['driving_lateral_rms_m'] is None
                    else p['driving_lateral_rms_m']-s['driving_lateral_rms_m']))
    result['valid'] = True
    result['comparison_evidence_sha256'] = _fingerprint(result)
    return result
