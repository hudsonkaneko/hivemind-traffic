"""V14 runner: the v12 loop with the v14 environment, policy-only SUMO modes and rounds 20-21's rules.

install() points the shared v12 loop at the v14 environment, requires SUMO speed mode 96 and
lane change mode 0 on every controlled car at every decision, validates configurations against
traffic/highway_flow_rl_v14_contract.py and writes under outputs/highway-flow-rl/v14. A v12
configuration is still accepted for evaluation and recordings on the v14 environment, never
for training. main() runs the shared loop (preflight / evaluate / train).

Round 20's stop rules replace "any collision" with a collision regression check at every
periodic validation, against this round's own initial validation (collision_regression).
Every validation summary also reports the drive limit's use (v14_drive_limit) and the stopped
decisions (v14_stops). Round 21 adds a free-road stop regression check at every periodic
validation (free_road_stop_regression) and turns the stop fix on (v14_stop_fix).
No training happens on import.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import run_highway_flow_rl_v12 as v12runner  # noqa: E402
from experiments.run_highway_flow_rl_v06 import StopRuleTriggered  # noqa: E402
from traffic import highway_flow_rl_v14_contract as contract  # noqa: E402
from traffic import highway_flow_rl_v14_rewards as v14  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

base = v12runner.base
_v12_install = v12runner.install   # kept, so callers may point v12runner.install at install() below
_v12_evaluate = v12runner.evaluate
OUTPUT_ROOT = ROOT / contract.OUTPUT_ROOT
NATIVE_MODES = dict(speed=v14.CONTROL['speed_mode'], lane_change=v14.CONTROL['lane_change_mode'])
SOURCES = ('experiments/run_highway_flow_rl_v14.py', 'environments/highway_flow_rl_v14_env.py',
           'traffic/highway_flow_rl_v14_rewards.py', 'traffic/highway_flow_rl_v14_contract.py',
           'traffic/road_grade.py', 'scripts/prepare_highway_flow_rl_v14.py',
           'traffic/highway_flow_rl_v14_stop_fix.py')


def road_model_manifest(case, network):
    """The corridor manifest that ties a case's network to its road model, or None for a network without one."""
    if not case.get('road_model'):
        return None
    manifest = network.parent / 'corridor-manifest.json'
    if not manifest.is_file():
        raise ValueError(f"Case {case['id']} declares a road model but has no corridor manifest")
    return manifest


def env_for(cfg, row, directory, env_class=None, **extra):
    """The v14 environment of a row; env_class/extra select a subclass such as the physical one.

    A configuration with v14_drive_limit (traffic.road_grade.DRIVE_LIMIT) turns the drive limit on,
    with the case's road model for the grade; extra drive_limit=... does the same for one run.
    A configuration with v14_stop_fix (traffic.highway_flow_rl_v14_stop_fix.SETTINGS) turns the stop
    fix on; extra stop_fix=... does the same for one run.
    """
    from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
    case = row['case']
    network = base.project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    drive_limit = extra.pop('drive_limit', cfg.get('v14_drive_limit'))
    if drive_limit is not None:
        extra.update(drive_limit=drive_limit, road_model_manifest=road_model_manifest(case, network))
    stop_fix = extra.pop('stop_fix', cfg.get('v14_stop_fix'))
    if stop_fix is not None:
        extra['stop_fix'] = stop_fix
    env = (env_class or HighwayFlowRLV14Env)(network, directory, **extra,
        v14_settings=cfg.get('v14_settings'),
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


def _collisions(reports):
    return {f"{index:02d} {report['case_id']}": int(report.get('collisions', 0)) for index, report in enumerate(reports)}


def collision_regression(cfg, output, label, result):
    """Stop a periodic validation that collides where this round's initial validation did not, or more often."""
    if 'collision_regression' not in cfg['stop_rules']:
        return
    initial_path = output / 'initial-validation.json'
    if not initial_path.is_file():
        return
    initial = _collisions(json.loads(initial_path.read_text())['reports'])
    now = _collisions(result['reports'])
    new_cases = sorted(case for case, count in now.items() if count and not initial.get(case))
    regressed = bool(new_cases) or sum(now.values()) > sum(initial.values())
    result['summary']['collision_regression'] = dict(initial_collisions=sum(initial.values()),
        collisions=sum(now.values()), colliding_cases=sorted(case for case, count in now.items() if count),
        new_colliding_cases=new_cases, regressed=regressed)
    if label.startswith('periodic-validation') and regressed:
        raise StopRuleTriggered(f'{label}: collision regression against the initial validation '
                                f'({sum(now.values())} collisions vs {sum(initial.values())}; new cases {new_cases})')


def drive_limit_summary(reports):
    per_case = {f"{index:02d} {r['case_id']}": r['v14_drive_limit'] for index, r in enumerate(reports)
                if r.get('v14_drive_limit')}
    if not per_case:
        return None
    decisions = sum(v['decisions'] for v in per_case.values())
    limited = sum(v['limited'] for v in per_case.values())
    return dict(decisions=decisions, limited=limited, limited_fraction=limited / decisions if decisions else None,
                per_case={case: dict(decisions=v['decisions'], limited=v['limited'], max_grade=v['max_grade'],
                                     min_ceiling_m_s2=v['min_ceiling_m_s2']) for case, v in per_case.items()})


STOP_KEYS = ('decisions', 'stopped', 'asking_to_brake', 'asking_to_brake_overrides', 'free_road')


def stops_summary(reports):
    """Pooled v14_stops of a validation: stopped decisions, those asking to brake, free-road stops."""
    per_case = {f"{index:02d} {r['case_id']}": r['v14_stops'] for index, r in enumerate(reports) if r.get('v14_stops')}
    if not per_case:
        return None
    result = {key: sum(v[key] for v in per_case.values()) for key in STOP_KEYS}
    result['per_case'] = {case: {key: v[key] for key in STOP_KEYS} for case, v in per_case.items()
                          if v['stopped'] or v['free_road']}
    return result


def free_road_stop_regression(cfg, output, label, result):
    """Stop a periodic validation whose free-road stops exceed both the rule's minimum and its factor
    times this round's initial validation."""
    rule = cfg['stop_rules'].get('free_road_stop_regression')
    stops = result['summary'].get('v14_stops')
    if rule is None or stops is None:
        return
    initial_path = output / 'initial-validation.json'
    if not initial_path.is_file():
        return
    initial = (json.loads(initial_path.read_text())['summary'].get('v14_stops') or {}).get('free_road')
    if initial is None:
        return
    limit = max(rule['minimum'], rule['factor'] * initial)
    regressed = stops['free_road'] > limit
    result['summary']['free_road_stop_regression'] = dict(initial_free_road=initial, free_road=stops['free_road'],
                                                          limit=limit, regressed=regressed)
    if label.startswith('periodic-validation') and regressed:
        raise StopRuleTriggered(f"{label}: {stops['free_road']} free-road stopped decisions, above {limit:g} "
                                f'(initial validation {initial})')


def evaluate(cfg, output, label, model, deadline, **kwargs):
    """v12's evaluation (intervention and merge-floor rules) plus the collision and free-road stop rules."""
    result = _v12_evaluate(cfg, output, label, model, deadline, **kwargs)
    result['summary']['v14_drive_limit'] = drive_limit_summary(result['reports'])
    result['summary']['v14_stops'] = stops_summary(result['reports'])
    try:
        stopped = None
        try:
            free_road_stop_regression(cfg, output, label, result)
        except StopRuleTriggered as error:
            stopped = error
        collision_regression(cfg, output, label, result)
        if stopped is not None:
            raise stopped
    finally:
        base.save(output / f'{label}.json', result)
    return result


def install():
    """The v12 loop with the v14 environment, modes, contract and output root (globals read at call time)."""
    _v12_install()
    v12runner.NATIVE_MODES = dict(NATIVE_MODES)
    base.OUTPUT_ROOT = OUTPUT_ROOT
    base.NEW_SOURCES = tuple(dict.fromkeys((*base.NEW_SOURCES, *SOURCES)))
    base.validate_config, base.validate_preflight = contract.validate_config, contract.validate_preflight
    base.env_for, base.evaluate = env_for, evaluate


def main(argv=None):
    install()
    return base.main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
