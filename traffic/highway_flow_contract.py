"""Fail-closed flow pilot configuration, selection and shared cost accounting."""
from copy import deepcopy
import json
import math
from pathlib import Path

from traffic.highway_smooth_contract import (file_digest, number, project_path,
    new_arm_state as _legacy_arm_state, reserve_episode, complete_episode, elapsed_on_resume, recover_pending)
from traffic.highway_flow_schedule import build_schedule, digest

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'highway_flow_training_v01'
ARMS = ('acceleration_imitation_1', 'acceleration_imitation_5')
CHECKS = ('contract','safety','reference','comfort','invariance','gradient','coverage','initializer_selection')
PARENT_CONFIG = 'policies/versions/highway-environment-v05/training-config.json'
PARENT_CONFIG_SHA = '17a28932f90c0ae64d875beab733c4d275de506487f7cad7b1a11909423121f9'
INITIALIZERS = {
    'v05_final': dict(path='outputs/highway-environment/v05/train-20261106T024828Z-90b25880/last.pt',
        sha256='9279b6103d13d2be3fd18fe05e9bae72462e581a16d881fc9e4ff87b0b2ac02d', observation_size=99,
        actor_path='policies/versions/highway-environment-v05/policy.pt',
        actor_sha256='fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec'),
    'v05_initializer': dict(path='outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761/selected-00036-d97376b9/best.pt',
        sha256='b77f55ae7262178ce0fa15729fe844e18693cd168905eea8b1bbd579c6913ab1', observation_size=93,
        actor_sha256='8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8')}
NETWORK = dict(path='outputs/i110-merge-blocking/v01/network-v02/network.net.xml',
    sha256='ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836')


def preparation_digest(cfg):
    return digest({k:v for k,v in cfg.items() if k not in ('training_authorized','readiness')})


def new_arm_state():
    return dict(_legacy_arm_state(),critic_schedule_index=0)


def frozen_base(network_overrides=None):
    if file_digest(ROOT/PARENT_CONFIG) != PARENT_CONFIG_SHA:
        raise ValueError('Frozen parent configuration changed')
    parent = json.loads((ROOT/PARENT_CONFIG).read_text(encoding='utf-8'))
    parent['training_authorized'], parent['readiness'] = False, None
    parent['model'].update(observation_size=117, state_size=2832, driving_local_size=69)
    parent['training'].update(seed_start=7100000, seed_end=7100999, max_agent_transitions=30000,
        max_wall_seconds=5400, critic_warmup_transitions=2048, final_evaluation_reserve_seconds=900,
        tasks=['cruise','following'], counting='Per-arm 30k including critic and failed reservations; outer paired 5400s is authoritative',
        schedule='Immutable matched longitudinal cases; planned case shares and actual decision exposure reported separately')
    for case in parent['training_cases'] + [r['case'] for r in parent['validation_cases']]:
        original = case['network_sha256']
        if 'i110' in case['id'].lower():
            case.update(network=NETWORK['path'], network_sha256=NETWORK['sha256'])
        replacement = (network_overrides or {}).get(original)
        if replacement:
            case.update(network=replacement['path'], network_sha256=replacement['sha256'])
    return parent


def reference_parameters():
    from traffic import highway_flow_reference as ref
    names = ('DT','PLANNING_HORIZON_S','LANE_CHANGE_DURATION_S','REFERENCE_JERK_M_S3',
        'SPEED_GAIN','COMFORT_DECELERATION_M_S2','PASS_ADVANTAGE_M_S','GAP_BUFFER_M')
    return {key:getattr(ref,key) for key in names}


def comparison_settings():
    return dict(foundation_tasks=['cruise','following'], minimum_cruise_speed_ratio=.9,
        maximum_intervention_fraction=.05, paired_mean_boundary_time_regression_max=.02,
        paired_speed_ratio_drop_max=.02, retained_case_outcomes=['passes','stable_merges','completed_trips'],
        selection_order=['collision_teleport_events','paired_completed_trip_deficit','foundation_failure_cases',
            'mean_cohort_boundary_time_s','pooled_override_fraction','prefer_v05_final'],
        selection_direction='Ascending lexicographic; tie prefers v05_final; no held-out or accepted claim',
        boundary_definition='Per-case sum(request-to-successful-arrival-or-80s) divided by fixed requested cohort, including waiting/unfinished; equal case mean',
        stage_scope='All24 measured; foundation advancement plus non-foundation retention is separate from global acceptance')


def validate_config(cfg):
    from traffic.highway_flow_reference import REFERENCE_VERSION
    if cfg.get('schema') != SCHEMA or type(cfg.get('training_authorized')) is not bool:
        raise ValueError('Flow schema/authorization mismatch')
    if cfg.get('initialization',{}).get('candidates') != INITIALIZERS or cfg['initialization'].get('seed') != 9507:
        raise ValueError('Pinned initializer identities/seed changed')
    if cfg['initialization'].get('selected_id') not in (None,*INITIALIZERS):
        raise ValueError('Unknown selected initializer')
    bank = cfg.get('network_bank')
    overrides = {}
    if bank is not None:
        path = project_path(bank['manifest_path'], outputs=True)
        if file_digest(path) != bank['manifest_sha256']:
            raise ValueError('Frozen flow network manifest changed')
        manifest = json.loads(path.read_text(encoding='utf-8'))
        if bank.get('network_overrides',{}) or manifest.get('network_overrides',{}):
            raise ValueError('Counterfactual roads cannot replace the fixed24-row network bank')
    if cfg['base_config'] != frozen_base(overrides):
        raise ValueError('Pinned full case bank/base configuration changed')
    if cfg.get('optimizer') != cfg['base_config']['optimizer'] or cfg.get('gae') != dict(gamma=.99,gae_lambda=.95):
        raise ValueError('Reviewed optimizer/GAE changed')
    if cfg.get('arms') != [dict(id=ARMS[0],imitation_acceleration_weight=1.),dict(id=ARMS[1],imitation_acceleration_weight=5.)]:
        raise ValueError('Only acceleration imitation 1 versus 5 may differ')
    if cfg.get('reference_version') != REFERENCE_VERSION or cfg.get('reference_parameters') != reference_parameters():
        raise ValueError('Resolved reference differs from implementation')
    if cfg.get('reward_version') != 'highway_flow_legacy_weights_new_reference_v1':
        raise ValueError('Flow keeps legacy reward formulas under the common reference')
    if cfg.get('comparison') != comparison_settings():
        raise ValueError('Predeclared comparison changed')
    fixed = dict(per_arm_decisions=30000,total_decisions=60000,wall_seconds=5400,
        critic_warmup_per_arm=2048,preparation_wall_seconds=1800,final_evaluation_reserve_seconds=900)
    if cfg.get('budget') != fixed:
        raise ValueError('Reviewed shared budget changed')
    if cfg['schedule'] != dict(seed=9507,seed_start=7100000,seed_end=7100999,
            tasks=['cruise','following'],planned_case_shares=dict(cruise=.5,following=.5),
            first_ppo_block_controlled_agents=1,first_ppo_block_cases=18,warmup_cursor='independent; PPO restarts the full coverage block',
            definition='First18 PPO cases use1AV each: at most14400 active decisions plus2048 warmup. Later blocks vary participation. Every longitudinal geometry once per block; difficulty cycles0/1/2; independent total population/speed/channel cycles. Case shares are not decision quotas.'):
        raise ValueError('Reviewed schedule changed')
    if cfg['schedule_sha256'] != digest(build_schedule(cfg)):
        raise ValueError('Immutable matched schedule changed')
    if cfg.get('evaluation') != dict(shared_initial_cases=24,final_cases_per_arm=24,
            initializer_cases_per_candidate=24,held_out=False):
        raise ValueError('Complete 24-row evaluation required')
    if cfg.get('provenance') != dict(parent_config=PARENT_CONFIG,parent_config_sha256=PARENT_CONFIG_SHA,
            plan='documentation/highway-training-rethink-v01.md'):
        raise ValueError('Parent provenance changed')
    if cfg.get('output_directory') != 'outputs/highway-flow/v01' or cfg.get('preparation_ledger') != 'outputs/highway-flow/v01/preparation-ledger.json':
        raise ValueError('Flow outputs/ledger must remain separate')
    return cfg


def validated_reports(cfg,reports):
    from experiments.highway_v05_support import acceptance
    rows = cfg['base_config']['validation_cases']
    if len(reports)!=24 or len(rows)!=24:
        raise ValueError('All24 complete reports required')
    acceptance(reports)  # Strict finite raw counters; actual gate failures remain data.
    result=[]
    for row,report in zip(rows,reports):
        if any(report.get(k)!=v for k,v in (('seed',row['seed']),('case_id',row['case']['id']),('task',row['case']['task']))):
            raise ValueError('Case/seed/order mismatch')
        if report.get('primary_horizon_complete') is not True or report.get('accounting_comparison_valid') is not True or report.get('partial_control_horizon') is not False:
            raise ValueError('Both comparison arms require complete primary horizons/accounting')
        account=report['accounting']; totals=account['totals']
        if account.get('comparison_valid') is not True or abs(number(account['time'],'accounting time')-80)>1e-6:
            raise ValueError('Complete80s cohort accounting required')
        requested=number(totals['requested'],'requested trips',low=1,integer=True)
        completed=number(totals['completed'],'completed trips',integer=True,high=requested)
        if report['requested_trips']!=requested or report['completed_trips']!=completed or totals['unfinished']!=requested-completed:
            raise ValueError('Trip cohort counters disagree')
        if sum(number(totals[k],k,integer=True) for k in ('waiting','active','teleporting','completed','removed'))!=requested:
            raise ValueError('Requested trip cohort is not conserved')
        native=report['native_emergencies']
        if native.get('available') is not True:
            raise ValueError('Missing native warning accounting')
        hazards=sum(number(report[k],k,integer=True) for k in ('collisions','teleports','overspeed_steps','emergency_braking_observed'))+number(native['unique_events'],'native emergencies',integer=True)
        result.append(dict(report=report,mean_boundary_time_s=number(totals['boundary_time_s'],'boundary time')/requested,
            requested=requested,completed=completed,hard_safety_events=hazards))
    return result


def rank_selection(cfg,reports_by_initializer):
    from experiments.highway_v05_support import acceptance
    if set(reports_by_initializer)!=set(INITIALIZERS):
        raise ValueError('Both frozen initializers must be measured on all24')
    candidates={}
    validated={name:validated_reports(cfg,reports) for name,reports in reports_by_initializer.items()}
    for paired in zip(*validated.values()):
        if len({r['requested'] for r in paired})!=1:
            raise ValueError('Initializer requested cohorts differ')
    best_completed=[max(r['completed'] for r in pair) for pair in zip(*validated.values())]
    for name,reports in reports_by_initializer.items():
        rows=validated[name]
        foundations=[r for r in reports if r['task'] in ('cruise','following')]
        summary,foundation=acceptance(reports),acceptance(foundations)
        rank=[sum(r['collisions']+r['teleports'] for r in reports),
            sum(best-r['completed'] for best,r in zip(best_completed,rows)),len(foundation['failures']),
            sum(r['mean_boundary_time_s'] for r in rows)/24,summary['pooled_intervention_fraction'],
            0 if name=='v05_final' else 1]
        candidates[name]=dict(rank=rank,summary=summary,foundation=foundation)
    selected=min(candidates,key=lambda name:tuple(candidates[name]['rank']))
    return dict(selected_id=selected,candidates=candidates,accepted=False,held_out_test_run=False,
        rule=deepcopy(cfg['comparison']['selection_order']))


def compare_endpoints(cfg,initial,control,treatment):
    from experiments.highway_v05_support import acceptance
    baseline=validated_reports(cfg,initial)
    output={}
    for name,reports in zip(ARMS,(control,treatment)):
        values=validated_reports(cfg,reports)
        retained=[]
        for index,(before,after) in enumerate(zip(baseline,values)):
            b,a=before['report'],after['report']
            same_cohort=before['requested']==after['requested']
            outcomes=(a['passes']>=b['passes'] and a.get('stable_merge',{}).get('stable',0)>=b.get('stable_merge',{}).get('stable',0) and after['completed']>=before['completed'])
            retained.append(dict(index=index,seed=a['seed'],task=a['task'],same_cohort=same_cohort,
                safety_not_worse=after['hard_safety_events']<=before['hard_safety_events'],
                outcomes_not_worse=outcomes,
                boundary_time_passed=after['mean_boundary_time_s']<=before['mean_boundary_time_s']*1.02+1e-8,
                speed_passed=a['mean_fraction_of_speed_limit']>=b['mean_fraction_of_speed_limit']-.02-1e-8))
        foundation=acceptance([r for r in reports if r['task'] in ('cruise','following')])
        retention=all(all(r[k] for k in ('same_cohort','safety_not_worse','outcomes_not_worse','boundary_time_passed','speed_passed')) for r in retained)
        output[name]=dict(foundation=foundation,retention_passed=retention,retention=retained,
            foundation_stage_gates_passed=foundation['all_gates_passed'] and retention,
            full_task_gates=acceptance(reports))
    return dict(arms=output,accepted=False,held_out_test_run=False,
        limitation='Foundation readiness is not global passing/merging acceptance; training coverage gate is reported separately')


def readiness_check(cfg,source_hashes):
    ready=cfg.get('readiness')
    if not isinstance(ready,dict) or not ready:
        raise ValueError('Passed zero-update readiness required')
    path=project_path(ready.get('summary_path',''),outputs=True)
    if not path.is_file() or file_digest(path)!=ready.get('summary_sha256'):
        raise ValueError('Readiness path/hash mismatch')
    proof=json.loads(path.read_text(encoding='utf-8'))
    if (proof.get('schema')!='highway_flow_preflight_v01' or proof.get('passed') is not True
            or proof.get('training') is not False or type(proof.get('optimizer_steps')) is not int or proof['optimizer_steps']!=0
            or proof.get('source_hashes')!=source_hashes or proof.get('preparation_sha256')!=preparation_digest(cfg)
            or any(proof.get('checks',{}).get(k) is not True for k in CHECKS)
            or cfg['initialization']['selected_id'] is None or proof.get('selected_initializer_id')!=cfg['initialization']['selected_id']):
        raise ValueError('Readiness is incomplete, learned, failed or stale')
    ledger_path=project_path(cfg['preparation_ledger'],outputs=True)
    if file_digest(ledger_path)!=proof.get('preparation_ledger_sha256'):
        raise ValueError('Preparation ledger changed')
    ledger=json.loads(ledger_path.read_text(encoding='utf-8'))
    if ledger.get('schema')!='highway_flow_preparation_ledger_v01' or not ledger.get('attempts'):
        raise ValueError('Invalid preparation ledger')
    total=0.
    for attempt in ledger['attempts']:
        if attempt.get('status') not in ('complete','completed','failed'):
            raise ValueError('Unfinished preparation attempt')
        total+=number(attempt.get('charged_wall_seconds'),'preparation cost')
    if not math.isclose(total,number(proof.get('preparation_wall_seconds'),'proof cost'),abs_tol=1e-6):
        raise ValueError('Preparation cost disagrees with append-only ledger')
    if total>cfg['budget']['preparation_wall_seconds'] or total>=cfg['budget']['wall_seconds']-cfg['budget']['final_evaluation_reserve_seconds']:
        raise ValueError('Preparation budget exhausted; no automatic extension')
    return dict(proof,charged_preparation_wall_seconds=total)


def require_training(cfg,approved_config_sha256,source_hashes):
    validate_config(cfg)
    if cfg['training_authorized'] is not True or approved_config_sha256!=digest(cfg):
        raise ValueError('Training requires explicit authorization and exact reviewed configuration checksum')
    return readiness_check(cfg,source_hashes)
